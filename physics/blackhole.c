/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * blackhole.c — Gargantua: 3D Schwarzschild null-geodesic ray tracer
 *
 * Computes exact light paths near a black hole.  Each terminal cell fires
 * a backwards null geodesic; RK4 integrates the Schwarzschild equation
 * until the photon hits the accretion disk, falls into the horizon, or
 * escapes to the background sky.  A lensing table is built once at startup
 * (~0.3–0.8 s); each animation frame is a fast table-lookup + Doppler colour.
 *
 * Physics (geometric units  c = G = 1,  r_s = 2M = 1):
 *   - Event horizon  :  r = 0.5 r_s  …  but in Schwarzschild coords r = r_s.
 *     We set r_s = 1, so horizon at r = 1, photon sphere at r = 1.5, ISCO r = 3.
 *   - Geodesic eq.   :  d²pos/dλ² = −(3/2) h² pos / r^5
 *     where h = pos × vel  (specific angular momentum vector).
 *     This is exact for null geodesics in Schwarzschild spacetime.
 *   - Doppler beaming:  D = [(1+β)/(1−β)]^(3/2),  β = v_orb · n̂_obs
 *     Keplerian orbit  v_orb = √(M/r) = √(1/(2r))
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/blackhole.c \
 *       -o blackhole -lncurses -lm
 *
 * Keys:
 *   q / ESC   quit          p   pause / resume
 *   r         reset spin    t   cycle theme (11 palettes:
 *                                Matrix  Fire    Oceanic  Neon    Mono
 *                                Ice     Nova    Forest   Desert  Eclipse
 *                                Blackbody — physically-correct
 *                                temperature ramp, blue=hot red=cool)
 *   + / =     closer cam    -   farther cam
 *   a / A     more / less inclination (tilt above disk plane; 0=edge-on,
 *                                       85=nearly face-on; rebuilds table)
 *   k         toggle lensed background star field (default OFF;
 *             distracting at the sides, dramatic near the photon ring)
 *
 * Scene composition: orbiting disk with Keplerian hot-spot clumps,
 * plus a lensed background star field (rays that escape are matched
 * against a hash-based star catalog, so stars directly behind the BH
 * get magnified into an Einstein-ring smear just outside the shadow).
 *
 * HUD: canonical CLAUDE.md two-bar — row 0 right shows live status
 * (dist, tilt, theme, paused/running, fps); row rows-1 lists the action keys.
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Backwards ray tracing + lookup table.
 *                  Each pixel shoots a null geodesic BACKWARDS from the
 *                  camera; if it hits the disk → disk colour; horizon → black.
 *                  Computing 900 RK4 steps per ray × all pixels live would be
 *                  too slow for animation.  Instead a lensing table maps
 *                  (camera_theta, camera_phi) → (impact_type, disk_radius)
 *                  computed once at startup, then looked up each frame.
 *
 * Physics        : General relativity — Schwarzschild null geodesics.
 *                  In geometric units c=G=1, r_s=2M=1:
 *                    Event horizon: r = 1 (r_s)
 *                    Photon sphere: r = 1.5 (r_s)  — unstable circular photon orbit
 *                    ISCO:          r = 3 (r_s)    — innermost stable circular orbit
 *                    (accretion disk starts at ISCO, so DISK_IN = 3)
 *                  Geodesic equation in 3D: d²pos/dλ² = −(3/2)h²·pos/r⁵
 *                  where h = pos × vel (specific angular momentum, conserved).
 *
 * Astrophysics   : Doppler beaming (relativistic).
 *                  Disk material orbits at Keplerian speed v = √(M/(2r)).
 *                  Approaching side blueshifts (brighter); receding side
 *                  redshifts (dimmer).  Beaming factor D = [(1+β)/(1−β)]^(3/2).
 *
 * ASPECT=0.47    : Measured empirically for this terminal font.
 *                  (≠ 0.50 from CELL_W/CELL_H ratio — physical pixels differ.)
 *                  Ensures the event horizon appears circular, not oval.
 *
 * References (cite inline as [n]):
 *
 *   [1] Misner, C. W.; Thorne, K. S.; Wheeler, J. A. — *Gravitation*
 *       (W. H. Freeman, 1973). §25 covers null geodesics of the
 *       Schwarzschild metric; §31 covers Schwarzschild black holes.
 *       The canonical derivation of the Binet equation we cast into 3D
 *       Cartesian form in §5 (geo_deriv).
 *
 *   [2] Thorne, K. S. — *The Science of Interstellar* (W. W. Norton,
 *       2014), Ch. 8–9. Visual reference for the Gargantua image,
 *       Doppler beaming of the accretion disk, and the asymmetric
 *       secondary-image arc that loops above/below the shadow.
 *
 *   [3] James, O.; von Tunzelmann, E.; Franklin, P.; Thorne, K. S.
 *       (2015) — "Gravitational lensing by spinning black holes in
 *       astrophysics, and in the movie *Interstellar*", *Class.
 *       Quantum Grav.* 32 065001. The paper behind the Interstellar
 *       VFX; describes the same backward ray-tracing pipeline used
 *       here, scaled up to a real film.
 *
 *   [4] Hamilton, A. — *Inside Black Holes*, JILA / U. Colorado
 *       (jila.colorado.edu/~ajsh/insidebh/). Free interactive
 *       simulator and pedagogy on Schwarzschild and Kerr geodesics;
 *       cross-check for the photon-ring brightness profile.
 *
 *   [5] Ware, C. — *Information Visualization: Perception for Design*,
 *       4th ed., Morgan Kaufmann (2020). Perceptually-ordered colour
 *       and luminance ramps (Ch. 4) back the ring → dim disk gradient,
 *       the 9-tier character-density encoding " .:+oO08#@" for
 *       brightness, and the 10 brightness-safe theme palettes in §3.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A black hole is a region where spacetime is curved so steeply that
 * light's "straight lines" (null geodesics) bend visibly. This demo
 * fires one ray BACKWARD from each terminal cell through that curved
 * spacetime; depending on what it hits, the cell paints as one of
 * three classes:
 *
 *   R_HORIZON — fell through r ≈ 1 r_s. Pure black (the SHADOW).
 *   R_DISK    — pierced the equatorial accretion disk at DISK_IN ≤ r
 *               ≤ DISK_OUT. Doppler-beamed and redshifted, with
 *               orbital hot-spot clumps adding flowing brightness.
 *   R_ESCAPED — reached the far field. If it grazed the photon
 *               sphere it paints the photon RING; otherwise the
 *               lensed background STAR FIELD.
 *
 * Tracing 900 RK4 steps per ray is too slow for live animation, so we
 * cache the outcome of every ray in a 2-D lensing TABLE at startup.
 * Each frame is then just: lookup → apply current disk rotation →
 * compute Doppler + redshift + texture + clump bumps → glyph + colour.
 *
 * A handful of CLUMPS — bright knots of disk material — orbit at each
 * radius's Keplerian rate Ω(r) = √(M/r³) (no rigid co-rotation: inner
 * laps outer), and their lensing comes for free from the same table.
 *
 * The background STAR FIELD is hash-sampled on the celestial sphere
 * using the cell's escape angle (esc_th, esc_ph), so rays bent hard
 * by the BH carry the magnified images of stars from directly behind
 * it into the Einstein-ring band just outside the shadow.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. STARTUP — build camera basis from tilt_deg (default TILT_DEG_DEF
 *     = 5°, adjustable live via a / A keys; rebuilds the lensing
 *     table) and FOV_DEG (72°).  Camera sits at distance cam_dist r_s,
 *     looking at the origin, tilted above the equatorial plane so
 *     the disk reads as an annulus rather than a line.
 *
 *  2. STARTUP — for each (row, col):
 *        • map screen UV → ray direction in camera space
 *        • integrate `geo_step()` (RK4 of d²pos/dλ² = −(3/2)h²·pos/r⁵)
 *          backward from the camera with adaptive ds ∝ r
 *        • terminate when r < 0.92 r_s (FELL into horizon),
 *          equatorial-plane crossing within [DISK_IN, DISK_OUT] (HIT disk),
 *          or r > ESCAPE_R = 130 r_s (ESCAPED to far field)
 *        • record outcome + (disk_r, disk_phi) on disk hit, or min_r
 *          for photon-ring proximity on escape
 *
 *  3. STARTUP — store everything in `g_scene.table[MAX_ROWS][MAX_COLS]`.
 *     Total cost: rows·cols rays × ~900 RK4 steps each (~0.3–0.8 s).
 *
 *  4. PER FRAME — disk_angle += spin (rigid spiral-texture rotation);
 *     each clump advances its phi by Ω(r) · CLUMP_RATE_SCALE
 *     (Keplerian — differential rotation, inner faster than outer).
 *
 *  5. PER FRAME, per visible cell — read `g_scene.table[row][col]`
 *     and dispatch on its RayKind:
 *
 *     R_HORIZON  → leave black (erase() already cleared the cell).
 *
 *     R_DISK     → phi_now    = disk_phi + disk_angle
 *                  v_orb      = √(M/r) = √(1/(2r))                 // Keplerian
 *                  β          = −v_orb · cos(phi_now) · cos(tilt)  // Doppler β
 *                  D          = [(1+β)/(1−β)]^(3/2)                 // beaming
 *                  g          = √(1 − r_s/r)                        // gravitational redshift
 *                  rad        = (1 − 0.86·r_n)^2.2 + 0.65·exp(−Δr²) // ISCO spike
 *                  tex        = 1 + 0.18·sin(5φ − 4·disk_angle)     // spiral texture
 *                  bump       = Σₖ Iₖ · exp(-Δφ²/σ_φ² - Δr²/σ_r²)   // hot-spot clumps
 *                  brightness = clamp(D · g · rad · tex + bump)
 *                  pick disk_char + disk_pair, mvaddch.
 *
 *     R_ESCAPED  → photon-ring BLOOM first (sum of three terms):
 *                    main = exp(-(mr - 1.5) · 1.4)        // broad halo
 *                    rim  = 0.55 · exp(-(mr - 1) · 10)    // near-horizon spike
 *                    echo = 0.40·G(σ_1) + 0.30·G(σ_2)     // n=1,2 echoes
 *                    rb   = clamp(main + rim + echo, 0, 1)
 *                  4-tier glyph (#, *, +, .) by rb.  Otherwise (if rb too
 *                  dim and stars enabled via 'k'), hash (esc_th, esc_ph)
 *                  to the star-field grid and paint *  + . if a star
 *                  lives at that direction.
 *
 *  6. PER FRAME — canonical two-bar HUD: row 0 right shows live status
 *     (dist, tilt, theme, paused/running, fps); rows-1 lists the
 *     action keys.
 *
 *  7. RESIZE / camera-distance / tilt change → rebuild lensing table.
 *
 * KEY FORMULAS
 * ────────────
 *  Schwarzschild radii (in units r_s = 1, M = 0.5):
 *    Event horizon  : r = r_s   = 1
 *    Photon sphere  : r = 1.5 r_s   (unstable circular photon orbit)
 *    ISCO           : r = 3 r_s     (innermost stable circular orbit;
 *                                    accretion disk inner edge)
 *
 *  Null geodesic in 3D Cartesian (derivation in CONCEPTS):
 *    h        = pos × vel                       (specific angular mom.)
 *    d(pos)/dλ = vel
 *    d(vel)/dλ = −(3/2) · |h|² · pos / r⁵
 *
 *  Keplerian orbital speed at disk radius r:
 *    v_orb = √(M / r) = √(1 / (2r))
 *
 *  Relativistic Doppler factor (radial component β):
 *    D = [(1 + β) / (1 − β)]^(3/2)
 *    (3/2 instead of 3 or 4 — visually muted to keep dynamic range
 *     within the 7-tier disk character ramp)
 *
 *  Gravitational redshift seen by far observer:
 *    g = √(1 − r_s/r)
 *    (drops to 0 at the horizon → faint blue limb just outside r=1)
 *
 *  Adaptive RK4 step:
 *    ds = clamp(0.05 · r,  0.003,  0.10)
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 199309L
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1  config ─────────────────────────────────────────────────────────── */

#define SIM_FPS      20
#define RENDER_FPS   60
#define ASPECT       0.47f      /* terminal cell h / w                    */

/* Schwarzschild geometry in units where r_s = 1 (Schwarzschild radius).
 * All lengths are dimensionless multiples of r_s = 2GM/c².               */
#define BH_R         1.0f   /* event horizon radius: r = r_s (= 2GM/c²)   */
#define PHOTON_R     1.5f   /* photon sphere: r = 3/2 · r_s.  Unstable
                              * circular orbit — photons here spiral in or out;
                              * gives the bright ring visible around the shadow. */
#define DISK_IN      3.0f   /* ISCO (innermost stable circular orbit): r = 3·r_s.
                              * Timelike (massive particle) orbits below r=3 are
                              * unstable → material plunges into horizon.
                              * The accretion disk begins here, not at r=1.       */
#define DISK_OUT     12.0f  /* outer disk edge (arbitrary, visually tuned).      */

/* camera — both tilt and distance are runtime-adjustable                  */
#define TILT_DEG_DEF  5.0f      /* default inclination above equatorial plane
                                  * (a / A keys cycle; rebuilds lensing table)*/
#define TILT_DEG_MIN  0.0f      /* edge-on: camera in disk plane — max
                                  * Doppler asymmetry, thin disk silhouette  */
#define TILT_DEG_MAX 85.0f      /* nearly face-on; capped < 90° to avoid the
                                  * camera-basis degeneracy at fwd ∥ world_up*/
#define TILT_DEG_STEP 5.0f      /* per keypress                              */
#define FOV_DEG      72.0f      /* horizontal field-of-view               */
#define CAM_DIST_DEF 24.0f      /* default camera distance (r_s units) —
                                  * comfortable framing: full disk + shadow
                                  * visible with a clear lensing arc above
                                  * the horizon, not so close that the disk
                                  * overruns the FOV.                      */
#define CAM_DIST_MIN  4.0f      /* closest  → largest Gargantua on screen.
                                  * Stays just outside ISCO (r = 3) so the
                                  * camera isn't inside the disk plane.    */
#define CAM_DIST_MAX 72.0f      /* farthest → smallest, far-field flat-ish*/
#define CAM_DIST_STEP 1.5f      /* step per keypress (finer near MIN)     */

/* disk rotation                                                            */
#define SPIN_DEF     0.04f      /* rad / sim-tick                         */

/* ray integration                                                          */
#define MAX_STEPS    900
#define ESCAPE_R     130.0f     /* fixed far-field boundary (r_s units)   */
#define DS_BASE      0.10f      /* affine-parameter step size             */

/* table dimensions                                                         */
#define MAX_COLS     512
#define MAX_ROWS     256

/* orbital hot-spot clumps (bright knots of disk material flowing past
 * the camera at the local Keplerian rate Ω(r) = √(M/r³)).  CLUMP_RATE_SCALE
 * is tuned so that at r_ref = 6 the clump rate ≈ SPIN_DEF — clumps near
 * the disk midline drift at roughly the same speed as the texture spiral. */
#define N_CLUMPS           10
#define CLUMP_RATE_SCALE   0.85f

/* ── §2  clock ──────────────────────────────────────────────────────────── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3  color / themes ─────────────────────────────────────────────────── */

enum {
    CP_RING  = 1,   /* photon ring / white-hot inner disk    — per theme */
    CP_HOT,         /* inner disk                            — per theme */
    CP_WARM,        /* mid-inner                             — per theme */
    CP_MID,         /* mid                                   — per theme */
    CP_COOL,        /* outer                                 — per theme */
    CP_DIM,         /* far outer                             — per theme */
    CP_STAR,        /* background stars      — fixed light grey          */
    CP_HUD,         /* HUD top status        — bright yellow + bold      */
    CP_HINT,        /* HUD bottom hint bar   — bright cyan   + bold      */
    CP_COUNT
};

static int g_256;

/*
 * Theme — six theme-driven 256-colour cube indices, ring (brightest,
 * inner-disk peak) → dim (outer-disk falloff).  Pairs CP_STAR, CP_HUD,
 * CP_HINT are fixed across all themes so the HUD chrome stays legible
 * regardless of which palette is active.
 *
 * Brightness safety (CLAUDE.md): every entry sits at index ≥ 30, or
 * 24-29 / 240-243 only as the lowest ramp tier.  16-23 / 232-239 are
 * forbidden — they vanish on default-background terminals.
 */
typedef struct {
    const char *name;
    short ring, hot, warm, mid, cool, dim;
} Theme;

static const Theme g_themes[] = {
    /*  name        ring hot  warm mid  cool dim                       */
    { "Matrix",     46,  118, 82,  40,  34,  28  }, /* cyber green       */
    { "Fire",       231, 226, 220, 208, 166, 130 }, /* white-hot → ember */
    { "Oceanic",    51,  87,  45,  44,  37,  31  }, /* bioluminescent    */
    { "Neon",       231, 213, 177, 165, 129, 93  }, /* pink / magenta    */
    { "Mono",       255, 252, 248, 244, 242, 240 }, /* grayscale         */
    { "Ice",        231, 195, 159, 123, 117, 111 }, /* polar white-blue  */
    { "Nova",       231, 195, 153, 111, 105, 99  }, /* stellar violet    */
    { "Forest",     156, 142, 100, 94,  64,  58  }, /* leaves to bark    */
    { "Desert",     230, 220, 214, 180, 136, 94  }, /* sand / gold       */
    { "Eclipse",    196, 160, 124, 88,  52,  240 }, /* red corona / dark */
    { "Blackbody",  153, 195, 231, 226, 214, 196 }, /* physical T(r):
                                                       inner=blue-white
                                                       hot, outer=red
                                                       cool (Wien's law) */
};
#define THEME_N    ((int)(sizeof g_themes / sizeof g_themes[0]))
#define THEME_DEF  10   /* Blackbody — physically-correct T(r) ramp */

/* Fixed chrome — same on every theme. */
#define CHROME_STAR_256  253      /* light grey         */
#define CHROME_HUD_256   226      /* bright yellow      */
#define CHROME_HINT_256   51      /* bright cyan        */

static void theme_apply(int idx)
{
    const Theme *t = &g_themes[idx % THEME_N];
    if (g_256) {
        init_pair(CP_RING, t->ring, -1);
        init_pair(CP_HOT,  t->hot,  -1);
        init_pair(CP_WARM, t->warm, -1);
        init_pair(CP_MID,  t->mid,  -1);
        init_pair(CP_COOL, t->cool, -1);
        init_pair(CP_DIM,  t->dim,  -1);
        init_pair(CP_STAR, CHROME_STAR_256, -1);
        init_pair(CP_HUD,  CHROME_HUD_256,  -1);
        init_pair(CP_HINT, CHROME_HINT_256, -1);
    } else {
        /* 8-colour fallback: theme-independent warm ramp + standard chrome. */
        init_pair(CP_RING, COLOR_WHITE,  -1);
        init_pair(CP_HOT,  COLOR_YELLOW, -1);
        init_pair(CP_WARM, COLOR_YELLOW, -1);
        init_pair(CP_MID,  COLOR_RED,    -1);
        init_pair(CP_COOL, COLOR_RED,    -1);
        init_pair(CP_DIM,  COLOR_RED,    -1);
        init_pair(CP_STAR, COLOR_WHITE,  -1);
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
}

/* ── §4  V3 math ─────────────────────────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline float   v3dot  (V3 a, V3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float   v3len2 (V3 a)      { return v3dot(a,a); }
static inline float   v3len  (V3 a)      { return sqrtf(v3len2(a)); }
static inline V3      v3add  (V3 a, V3 b){ return (V3){a.x+b.x,a.y+b.y,a.z+b.z}; }
static inline V3      v3sub  (V3 a, V3 b){ return (V3){a.x-b.x,a.y-b.y,a.z-b.z}; }
static inline V3      v3scale(float s,V3 a){ return (V3){s*a.x,s*a.y,s*a.z}; }
static inline V3      v3cross(V3 a, V3 b){
    return (V3){ a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x };
}
static inline V3 v3norm(V3 a){
    float l = v3len(a);
    return l > 1e-12f ? v3scale(1.0f/l, a) : (V3){0,1,0};
}

/* ── §5  Schwarzschild null geodesic integrator ──────────────────────────── */

/*
 * Derivative of the geodesic state (pos, vel) with respect to affine λ.
 *
 *   d(pos)/dλ = vel
 *   d(vel)/dλ = −(3/2) · |pos × vel|² · pos / |pos|^5
 *
 * Exact for Schwarzschild null geodesics in isotropic-like Cartesian
 * embedding (r_s = 1).  Derivation: Binet equation d²u/dφ² + u = 3Mu²
 * with M = 0.5 cast into 3D Cartesian form using h = pos × vel.
 */
static void geo_deriv(V3 pos, V3 vel, V3 *dpos, V3 *dvel)
{
    *dpos = vel;
    V3    h    = v3cross(pos, vel);
    float h2   = v3len2(h);
    float r2   = v3len2(pos);
    float r    = sqrtf(r2);
    float coef = -1.5f * h2 / (r2 * r2 * r);   /* −(3h²/2r^5) */
    *dvel = v3scale(coef, pos);
}

/* One RK4 step of size ds */
static void geo_step(V3 *pos, V3 *vel, float ds)
{
    V3 dp1,dv1, dp2,dv2, dp3,dv3, dp4,dv4;

    geo_deriv(*pos, *vel, &dp1, &dv1);

    V3 pa = v3add(*pos, v3scale(0.5f*ds, dp1));
    V3 va = v3add(*vel, v3scale(0.5f*ds, dv1));
    geo_deriv(pa, va, &dp2, &dv2);

    V3 pb = v3add(*pos, v3scale(0.5f*ds, dp2));
    V3 vb = v3add(*vel, v3scale(0.5f*ds, dv2));
    geo_deriv(pb, vb, &dp3, &dv3);

    V3 pc = v3add(*pos, v3scale(ds, dp3));
    V3 vc = v3add(*vel, v3scale(ds, dv3));
    geo_deriv(pc, vc, &dp4, &dv4);

    float k = ds / 6.0f;
    *pos = v3add(*pos, v3scale(k,
        v3add(dp1, v3add(v3scale(2,dp2), v3add(v3scale(2,dp3), dp4)))));
    *vel = v3add(*vel, v3scale(k,
        v3add(dv1, v3add(v3scale(2,dv2), v3add(v3scale(2,dv3), dv4)))));
}

/* ── §6  ray table / scene ───────────────────────────────────────────────── */

/*
 * RayKind — outcome class for one backward null geodesic.
 *
 *   R_HORIZON — ray fell through r ≈ BH_R. Paint pure black (the SHADOW).
 *   R_DISK    — ray pierced the equatorial accretion disk inside
 *               [DISK_IN, DISK_OUT]. disk_r / disk_phi pin the hit point
 *               in the disk's inertial frame.
 *   R_ESCAPED — ray reached the far field (r > ESCAPE_R) without hitting
 *               the disk or horizon. esc_th / esc_ph encode the celestial-
 *               sphere direction the photon CAME FROM; min_r drives the
 *               photon-ring bloom strength.
 */
typedef enum { R_HORIZON = 0, R_DISK, R_ESCAPED } RayKind;

/*
 * Cell — one entry in the precomputed lensing table.  One Cell per
 * screen pixel, ~3 MB for a 256×512 grid.
 *
 * Why precompute (lookup table vs per-frame raytrace):
 *   Each ray takes up to MAX_STEPS = 900 RK4 steps.  At ~150 000
 *   visible cells × 60 fps that would be ~8 billion steps per second
 *   — impossible.  But the lensing GEOMETRY only changes when the
 *   camera moves (cam_dist or tilt), so we cache the per-pixel ray
 *   outcome once at startup and read back every frame.  Per-frame
 *   work then collapses to one lookup + a handful of Doppler / bloom
 *   arithmetic per cell.
 *
 * Field usage by RayKind (union-like, all packed into one struct):
 *
 *     R_HORIZON  uses none of the data fields.
 *     R_DISK     uses disk_r, disk_phi.  Renderer applies Doppler,
 *                redshift, texture, clump bumps at draw time.
 *     R_ESCAPED  uses esc_th, esc_ph, min_r.  min_r drives the
 *                4-tier photon-ring bloom (main + rim + echoes);
 *                (esc_th, esc_ph) seed the lensed background star
 *                field when stars are enabled.
 *
 * Why a plain struct, not a tagged union:
 *   Saves 12 bytes per cell but costs verbose access syntax.  At
 *   the current table size we're comfortably inside L2/L3; no
 *   real motivation to shave bytes.
 *
 * Algorithm refs (header REFERENCES):
 *   Schwarzschild null-geodesic integration   — MTW §25 [1]
 *   Backward ray-tracing of curved spacetime  — James et al. [3]
 *   Photon-ring closest-approach magnification — Hamilton [4]
 */
typedef struct {
    RayKind kind;
    float   disk_r;    /* (R_DISK)    cylindrical radius of hit, r_s units */
    float   disk_phi;  /* (R_DISK)    inertial-frame azimuth, [-π, π]     */
    float   esc_th;    /* (R_ESCAPED) polar    angle of escape, [0, π]    */
    float   esc_ph;    /* (R_ESCAPED) azimuth   angle of escape, [-π, π]  */
    float   min_r;     /* closest approach of the geodesic to the BH —
                        * R_ESCAPED uses this to drive the bloom         */
} Cell;

/*
 * Clump — one orbiting hot spot embedded in the disk.
 *
 * Why a struct (not parallel float arrays):
 *   Each clump bundles three coupled state values (r, phi, intensity).
 *   Grouping makes per-tick advance read as "for each clump, advance
 *   its phi at Ω(r)" and per-pixel sampling read as "for each clump,
 *   add a Gaussian bump at (r, phi)" — no parallel-array indexing
 *   gymnastics.
 *
 * What it represents:
 *   A concentrated knot of hotter / denser disk gas at orbital radius r.
 *   Each sim tick, phi advances at the local Keplerian rate
 *   Ω(r) = √(M/r³) — so inner clumps lap outer clumps and the disk
 *   visibly churns with differential rotation, not the rigid rotation
 *   the spiral texture has on its own.
 *
 * Lensing is automatic:
 *   The brightness-bump check uses the lensing table's (disk_r,
 *   disk_phi) at the pixel.  These coordinates are already lensed —
 *   the secondary-image arc above the shadow carries its own
 *   (disk_r, disk_phi) pair.  So a clump on the FAR side of the BH
 *   lights up the secondary image arc too, exactly like real
 *   Gargantua-class imagery.
 *
 * Algorithm refs (header REFERENCES):
 *   Keplerian orbital rate  Ω = √(M/r³)       — MTW §25 [1]
 *   Doppler beaming of orbital hot spots      — Thorne [2] Ch. 8
 *   Hot-spot disk variability                 — James et al. [3]
 */
typedef struct {
    float r;          /* orbital radius, r_s units; constant per clump.
                       * Sets the Keplerian rate Ω(r) = √(M/r³).          */
    float phi;        /* current inertial-frame azimuth, [0, 2π);
                       * advanced by Ω(r) · CLUMP_RATE_SCALE each tick.   */
    float intensity;  /* peak brightness boost at clump centre, ~[0,1].
                       * Randomised at init so clumps read as distinct.   */
} Clump;

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — state that spans precompute(), render(), clumps_tick(), and
 * the main loop.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters  — consumed by precompute (lensing-table
 *     rebuild) and the per-tick clump / disk-angle advance.  Anything
 *     that affects the PHYSICS (ray outcomes, clump positions, disk
 *     rotation) lives here.  Mutated by physics-affecting keys:
 *     + / - (zoom, rebuilds table), a / A (tilt, rebuilds table),
 *     p (pause), r (reset).
 *
 *   Rendering parameters   — consumed by render and screen_hud only.
 *     Toggling any of these while paused must leave the lensing table
 *     and clump positions byte-identical — only colours / overlays
 *     may differ.  Mutated by purely cosmetic keys: t (theme), k (stars).
 *
 * Locality rationale (this contract matters, not the bytes):
 *   The split exists for the READER, not the CPU.  A new flag landing
 *   in the rendering group when it actually triggers a table rebuild
 *   would silently couple display to physics — exactly the bug the
 *   separation prevents.  When adding a field, ask: does this change
 *   what precompute / ray_trace / clumps_tick produces?  If yes,
 *   simulation; if no, rendering.
 *
 * Single instance (file-scope `g_scene`):
 *   The struct embeds the ~3 MB lensing table, so it lives in BSS
 *   as a file-static rather than being passed by pointer.  All
 *   scene state is accessed as `g_scene.<field>` from the few
 *   helpers and the main loop that need it.
 *
 * What stays OUTSIDE this struct (intentionally):
 *   g_run / g_resize   sig_atomic_t flags read by signal handlers;
 *                      must stay at file scope for async-signal safety.
 *   g_256              one-shot color-capability flag set at startup;
 *                      never mutated, no benefit in scene membership.
 *   cols / rows        screen geometry tracked by the main loop;
 *                      Scene stays geometry-agnostic so resize handling
 *                      is the main loop's concern, not the renderer's.
 *   fps / frame_time / sim_accum / ...  main-loop timing bookkeeping;
 *                                       neither physics nor display.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ── Simulation parameters ─────────────────────────────────────── */
    float cam_dist;       /* camera distance from BH, r_s units (+/-).
                           * Changing invalidates the lensing table —
                           * caller must set need_rebuild = 1.            */
    float tilt_deg;       /* inclination above equator, degrees (a/A).
                           * Same rebuild trigger as cam_dist.            */
    float spin;           /* per-tick rigid disk-texture rotation rate
                           * (rad/tick).                                  */
    float disk_ang;       /* cumulative disk-texture rotation, [0, 2π);
                           * advanced by spin each tick when !paused.    */
    int   paused;         /* 1 → freeze the sim; HUD shows PAUSED.        */

    /* Sim buffers — populated by precompute / clumps_init, read each
     * frame by render. */
    Cell  table[MAX_ROWS][MAX_COLS];   /* precomputed ray outcomes        */
    Clump clumps[N_CLUMPS];            /* orbital hot spots               */

    /* ── Rendering parameters ──────────────────────────────────────── */
    int   theme;          /* index into g_themes[] (§3); t key cycles.   */
    int   show_stars;     /* 1 → render lensed background star field;
                           * k key toggles, off by default.               */
} Scene;

static Scene g_scene = {
    .cam_dist   = CAM_DIST_DEF,
    .tilt_deg   = TILT_DEG_DEF,
    .spin       = SPIN_DEF,
    .disk_ang   = 0.0f,
    .paused     = 0,
    .theme      = THEME_DEF,
    .show_stars = 0,
    /* .table[] and .clumps[] are BSS-zeroed; populated before first read. */
};

/*
 * Geodesic-loop helpers — each one isolates a single step of the
 * backward ray-tracing integration so ray_trace itself reads as
 * "while integrating: check horizon, check escape, step, check disk".
 *
 * Algorithm refs (header REFERENCES):
 *   Schwarzschild null-geodesic integration   — MTW §25 [1]
 *   Backward ray-tracing of curved spacetime  — James et al. [3]
 *   Adaptive-stepsize / photon-ring practice  — Hamilton [4]
 */

/*
 * adaptive_geodesic_step_size — finer integration near the BH where
 * spacetime curvature is high; coarser in the nearly-flat far field.
 *
 *     ds = clamp(0.05 · r, 0.003, DS_BASE)
 *
 * At r = 1.5 r_s (photon sphere): ds ≈ 0.075 — captures the rapid
 * direction changes of grazing geodesics.  At r = 100 r_s: ds saturates
 * at DS_BASE = 0.10 — wasteful to integrate finer through near-flat
 * space.  Min 0.003 catches the rare ultra-close approach.
 */
static float adaptive_geodesic_step_size(float r)
{
    return fmaxf(0.003f, fminf(DS_BASE, r * 0.05f));
}

/*
 * fell_through_horizon — is the photon captured?
 *
 * Tests r < BH_R · 0.92.  The 0.92 fudge factor catches rays that are
 * on the inevitable plunge but haven't quite reached r = r_s yet —
 * the geodesic would integrate them through the singularity if we
 * waited for r < r_s exactly.  This is the "captured by BH" criterion.
 */
static int fell_through_horizon(float r)
{
    return r < BH_R * 0.92f;
}

/*
 * escaped_to_far_field — has the photon left the gravitational well?
 *
 * Outside r = ESCAPE_R (130 r_s) the spacetime is nearly Minkowski;
 * the photon's direction is essentially frozen, so we stop the
 * integration and record the asymptotic direction.
 */
static int escaped_to_far_field(float r)
{
    return r > ESCAPE_R;
}

/*
 * crossed_equatorial_plane — did the photon's y component sign-flip?
 *
 * Detects a crossing of the equatorial plane y = 0 between two
 * consecutive geodesic samples.  Used to find intersections with the
 * (flat, thin) accretion disk.  Catches BOTH the primary image (near-
 * side disk crossed from above by a camera ray) AND the secondary
 * image (rays that loop around the photon sphere and cross the FAR
 * side of the disk on their way back).
 */
static int crossed_equatorial_plane(V3 prev, V3 pos)
{
    return prev.y * pos.y < 0.0f;
}

/*
 * interpolate_disk_plane_hit — linearly interpolate the (x, z) position
 * of the y = 0 crossing between two geodesic samples that bracket it.
 *
 *   t   = |y_prev| / (|y_prev| + |y_pos|)         ∈ [0, 1]
 *   hit = prev + t · (pos − prev)
 *
 * Equivalent to solving prev.y + t·(pos.y − prev.y) = 0 in [0, 1].
 * The interpolated (x, z) gives the disk-frame radius and azimuth
 * of the hit.
 */
static V3 interpolate_disk_plane_hit(V3 prev, V3 pos)
{
    float t = fabsf(prev.y) / (fabsf(prev.y) + fabsf(pos.y));
    return v3add(v3scale(1.f - t, prev), v3scale(t, pos));
}

/*
 * record_far_field_direction — convert a final velocity vector to
 * (theta, phi) celestial-sphere coordinates and pack into an
 * R_ESCAPED Cell.
 *
 *   theta = acos(d.y)              ∈ [0, π]
 *   phi   = atan2(d.z, d.x)        ∈ (-π, π]
 *
 * Used both for normal escape (r > ESCAPE_R) AND the max-steps
 * fallback (rays that spiraled near the photon sphere without
 * terminating — they still get an asymptotic direction).
 */
static Cell record_far_field_direction(V3 vel, float min_r)
{
    V3    d  = v3norm(vel);
    float th = acosf(fmaxf(-1.f, fminf(1.f, d.y)));
    float ph = atan2f(d.z, d.x);
    return (Cell){ R_ESCAPED, 0, 0, th, ph, min_r };
}

/*
 * ray_trace — one backward null geodesic from the camera, until it
 * is captured by the horizon, escapes to the far field, or pierces
 * the accretion disk.
 *
 * Pseudocode:
 *   pos, vel ← (camera_origin, ray_direction)
 *   min_r   ← |pos|
 *   for up to MAX_STEPS:
 *       r ← |pos|;  min_r ← min(min_r, r)
 *       if fell_through_horizon(r):       return R_HORIZON cell
 *       if escaped_to_far_field(r):       return record_far_field_direction
 *       ds ← adaptive_geodesic_step_size(r)
 *       prev ← pos;  geo_step(pos, vel, ds)
 *       if crossed_equatorial_plane(prev, pos):
 *           hit ← interpolate_disk_plane_hit(prev, pos)
 *           if  DISK_IN ≤ |hit_xz| ≤ DISK_OUT:
 *               return R_DISK cell with (hit_r, hit_phi)
 *   // spiral near photon sphere without escape — record asymptote anyway
 *   return record_far_field_direction
 */
static Cell ray_trace(V3 origin, V3 dir)
{
    V3    pos   = origin;
    V3    vel   = dir;           /* affine-parameter velocity, |vel| ≈ 1 */
    V3    prev  = pos;
    float min_r = v3len(origin); /* tracks closest approach for the bloom */

    for (int step = 0; step < MAX_STEPS; step++) {
        float r = v3len(pos);
        if (r < min_r) min_r = r;

        if (fell_through_horizon(r))
            return (Cell){ R_HORIZON, 0, 0, 0, 0, min_r };
        if (escaped_to_far_field(r))
            return record_far_field_direction(vel, min_r);

        float ds = adaptive_geodesic_step_size(r);
        prev = pos;
        geo_step(&pos, &vel, ds);

        if (crossed_equatorial_plane(prev, pos)) {
            V3    hit = interpolate_disk_plane_hit(prev, pos);
            float cr  = sqrtf(hit.x*hit.x + hit.z*hit.z);
            if (cr >= DISK_IN && cr <= DISK_OUT) {
                float ph = atan2f(hit.z, hit.x);
                return (Cell){ R_DISK, cr, ph, 0, 0, min_r };
            }
        }
    }

    return record_far_field_direction(vel, min_r);
}

/* ── §7  precompute lensing table ────────────────────────────────────────── */

/*
 * Precompute helpers — set up the pinhole camera frame, map screen
 * pixels to ray directions, and show progress feedback during the
 * 0.3–0.8 s table build.
 */

/*
 * build_camera_basis — orthonormal frame for the pinhole camera looking
 * at the origin from distance cam_dist, tilted tilt_rad above the
 * equatorial plane.
 *
 *   cam = (0,  cam_dist · sin t,  − cam_dist · cos t)    [position]
 *   fwd = normalize(−cam)                                [toward origin]
 *   rgt = normalize(fwd × world_up)                      [screen-right]
 *   up  = rgt × fwd                                      [screen-up]
 *
 * world_up is fixed at (0, 1, 0).  At tilt = ±90° fwd would be parallel
 * to world_up and rgt would degenerate — the §1 TILT_DEG_MAX = 85°
 * cap keeps us safely away from that singularity.
 */
static void build_camera_basis(float cam_dist, float tilt_rad,
                               V3 *cam, V3 *fwd, V3 *rgt, V3 *up)
{
    *cam = (V3){ 0.f, cam_dist * sinf(tilt_rad), -cam_dist * cosf(tilt_rad) };
    *fwd = v3norm(v3scale(-1.f, *cam));
    V3 world_up = { 0.f, 1.f, 0.f };
    *rgt = v3norm(v3cross(*fwd, world_up));
    *up  = v3cross(*rgt, *fwd);
}

/*
 * pinhole_pixel_to_ray — map a screen pixel (col, row) to a 3D ray
 * direction in world coordinates.
 *
 *   u =  (col − cx) / cx · hw         [right + ]
 *   v = −(row − cy) / cx · hw / ASPECT [up + ; minus flips row↓ to scene↑]
 *   dir = normalize(fwd + u · rgt + v · up)
 *
 * The (col − cx) factor uses cols-half as reference for BOTH axes so
 * pixels are square in isotropic camera space; ASPECT then corrects
 * for the terminal's row/col cell-shape ratio (cells are ~2× taller
 * than wide).
 *
 * hw = tan(FOV/2) scales the unit screen NDC to actual ray-direction
 * tangents.  This is a standard pinhole-camera projection.
 */
static V3 pinhole_pixel_to_ray(int col, int row, float cx, float cy,
                               float hw, V3 fwd, V3 rgt, V3 up)
{
    float u =  (col - cx) / cx * hw;
    float v = -(row - cy) / cx * hw / ASPECT;
    return v3norm(v3add(fwd, v3add(v3scale(u, rgt), v3scale(v, up))));
}

/*
 * draw_progress_header — central "Building lensing table…" message
 * shown once before the precompute loop.  Visual feedback so the
 * user knows the freeze is intentional (a full rebuild takes
 * 0.3–0.8 seconds on commodity hardware).
 */
static void draw_progress_header(int cols, int rows)
{
    attron(A_BOLD);
    mvprintw(rows/2,   cols/2-18, "  Building lensing table …          ");
    mvprintw(rows/2+1, cols/2-18, "  (exact Schwarzschild geodesics)   ");
    attroff(A_BOLD);
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * draw_progress_percent — periodic "[NN%]" update during the rebuild
 * loop.  Called every few rows so the user sees progress incrementing
 * rather than a frozen header.
 */
static void draw_progress_percent(int rows_done, int rows_total,
                                  int cols, int rows)
{
    int pct = rows_done * 100 / rows_total;
    mvprintw(rows/2+2, cols/2-10, "  [%3d%%] ", pct);
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * precompute — populate the lensing table by shooting one backward
 * ray from the camera through every visible pixel.
 *
 * Pseudocode:
 *   build_camera_basis(cam_dist, tilt) → (cam, fwd, rgt, up)
 *   draw_progress_header
 *   for each (row, col) in [rows_lim] × [cols_lim]:
 *       dir ← pinhole_pixel_to_ray(col, row, cx, cy, hw, fwd, rgt, up)
 *       g_scene.table[row][col] ← ray_trace(cam, dir)
 *   every 6th row, draw_progress_percent
 */
static void precompute(int cols, int rows, float cam_dist, float tilt_deg)
{
    float tilt_rad = tilt_deg * (float)M_PI / 180.0f;
    float hw       = tanf((FOV_DEG * (float)M_PI / 180.0f) * 0.5f);
    float cx       = cols * 0.5f;
    float cy       = rows * 0.5f;

    V3 cam, fwd, rgt, up;
    build_camera_basis(cam_dist, tilt_rad, &cam, &fwd, &rgt, &up);

    draw_progress_header(cols, rows);

    int rows_lim = rows < MAX_ROWS ? rows : MAX_ROWS;
    int cols_lim = cols < MAX_COLS ? cols : MAX_COLS;

    for (int row = 0; row < rows_lim; row++) {
        for (int col = 0; col < cols_lim; col++) {
            V3 dir = pinhole_pixel_to_ray(col, row, cx, cy, hw, fwd, rgt, up);
            g_scene.table[row][col] = ray_trace(cam, dir);
        }
        if (row % 6 == 0)
            draw_progress_percent(row, rows_lim, cols, rows);
    }
}

/* ── §8  frame render ────────────────────────────────────────────────────── */

/* Clump struct + g_scene.clumps storage are defined in §6 next to Cell
 * and Scene so the data-structure layer is together.  The helpers
 * below (clumps_init / clumps_tick) operate on g_scene.clumps. */

static float clumps_frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/*
 * clumps_init — seed N_CLUMPS knots uniformly across the disk in radius
 * and angle, with mildly randomised intensities so they don't all read
 * as identical.  Called at startup and on the 'r' (reset) key.
 */
static void clumps_init(void)
{
    for (int i = 0; i < N_CLUMPS; i++) {
        g_scene.clumps[i].r         = DISK_IN + (DISK_OUT - DISK_IN) * clumps_frand();
        g_scene.clumps[i].phi       = clumps_frand() * 2.0f * (float)M_PI;
        g_scene.clumps[i].intensity = 0.22f + 0.20f * clumps_frand();  /* 0.22–0.42 */
    }
}

/*
 * clumps_tick — advance each clump's azimuth by one sim step at its own
 * Keplerian rate Ω(r) = √(M/r³).  With M = 0.5 (r_s = 1):  Ω = √(0.5/r³).
 *
 *   r =  3 (ISCO):   Ω ≈ 0.136 rad / (affine unit)  — fastest
 *   r =  6 (mid):    Ω ≈ 0.048
 *   r = 12 (outer):  Ω ≈ 0.017                       — slowest
 *
 * Ratio 8× between inner and outer — inner clumps visibly lap outer ones.
 */
static void clumps_tick(void)
{
    for (int i = 0; i < N_CLUMPS; i++) {
        float r     = g_scene.clumps[i].r;
        float omega = sqrtf(0.5f / (r * r * r));
        g_scene.clumps[i].phi += omega * CLUMP_RATE_SCALE;
        if (g_scene.clumps[i].phi >= (float)(2.0 * M_PI))
            g_scene.clumps[i].phi -= (float)(2.0 * M_PI);
    }
}

static float fclamp(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/*
 * star_lookup — deterministic background-star sampling on the celestial sphere.
 *
 * Each R_ESCAPED cell records the direction (esc_th, esc_ph) the photon
 * escaped TO.  In a backward-traced raytrace that's the direction the
 * photon came FROM at infinity — i.e. the celestial-sphere coordinate
 * for the FAR-FIELD background that the cell is looking at.
 *
 * We quantise (th, ph) onto a ~5800-cell sphere grid (30 cells per
 * radian → ~3° per cell) and hash each grid index with a SplitMix-style
 * mix.  The hash determines:
 *   • density gate  (low 8 bits): ~2% of cells contain a star
 *   • brightness    (next 8 bits): split into *, +, . classes
 *
 * Lensing is automatic: rays passing near the photon sphere fan their
 * escape directions across a large angular swath, so a small patch of
 * celestial sphere — including stars directly BEHIND the BH — gets
 * magnified into the Einstein-ring zone just outside the shadow.
 *
 * Returns 1 if there is a star at (th, ph); fills *glyph and *attr.
 */
static int star_lookup(float th, float ph, char *glyph, attr_t *attr)
{
    unsigned int q_th = (unsigned int)(th * 30.0f);
    unsigned int q_ph = (unsigned int)((ph + (float)M_PI) * 30.0f);
    unsigned int h    = q_th * 0x9E3779B9u + q_ph * 0xCC9E2D51u;
    h ^= h >> 16;
    h *= 0x85EBCA6Bu;
    h ^= h >> 13;
    h *= 0xC2B2AE35u;
    h ^= h >> 16;

    if ((h & 0xFFu) > 4u) return 0;             /* ~2% density */

    unsigned int b = (h >> 8) & 0xFFu;
    if      (b > 220u) { *glyph = '*'; *attr = A_BOLD;   }
    else if (b > 100u) { *glyph = '+'; *attr = A_NORMAL; }
    else               { *glyph = '.'; *attr = A_NORMAL; }
    return 1;
}

static char disk_char(float b)
{
    if (b > 0.92f) return '@';
    if (b > 0.82f) return '#';
    if (b > 0.70f) return '8';
    if (b > 0.57f) return '0';
    if (b > 0.45f) return 'O';
    if (b > 0.33f) return 'o';
    if (b > 0.21f) return '+';
    if (b > 0.12f) return ':';
    return '.';
}

static void disk_pair(float b, float r_norm, int *cp, attr_t *a)
{
    if      (b > 0.85f){ *cp=CP_RING; *a=A_BOLD;                         }
    else if (b > 0.67f){ *cp=CP_HOT;  *a=(r_norm<0.25f)?A_BOLD:A_NORMAL; }
    else if (b > 0.50f){ *cp=CP_WARM; *a=A_NORMAL;                        }
    else if (b > 0.33f){ *cp=CP_MID;  *a=A_NORMAL;                        }
    else if (b > 0.17f){ *cp=CP_COOL; *a=A_NORMAL;                        }
    else               { *cp=CP_DIM;  *a=A_NORMAL; /* no A_DIM darkening */ }
}

/*
 * Render helpers — each one isolates one physics / visual term that
 * goes into the per-pixel brightness, plus the per-pixel drawing.
 *
 * Algorithm refs (header REFERENCES):
 *   Doppler beaming + gravitational redshift  — MTW §25 [1], Thorne [2]
 *   Accretion-disk thermal profile             — Thorne [2] Ch. 8
 *   Photon-ring multi-tier bloom               — Hamilton [4]
 *   Hot-spot brightness modulation             — James et al. [3]
 *   Perceptual brightness ramps                — Ware [5]
 */

/*
 * keplerian_doppler_factor — relativistic beaming/dimming for orbiting
 * disk material seen by an observer at the camera tilt.
 *
 *   v_orb = √(M / r) = √(1 / (2r))           [r_s = 1, M = 0.5]
 *   β     = −v_orb · cos(phi_rotating) · cos(tilt)
 *   D     = [(1 + β) / (1 − β)]^(3/2)
 *
 * β > 0 → material APPROACHING observer → brighter (D > 1).
 * β < 0 → receding → dimmer.  β clamped to ±0.95 to avoid the
 * v → c pole.
 *
 * The 3/2 exponent (vs the full-SR 3 or 4) is intentionally muted to
 * keep the dynamic range inside the 9-tier disk character ramp — a
 * 3-exponent would saturate everything to '@' on the approaching side.
 */
static float keplerian_doppler_factor(float disk_r, float phi_rotating,
                                      float cos_tilt)
{
    float v_orb = sqrtf(0.5f / disk_r);
    float beta  = -v_orb * cosf(phi_rotating) * cos_tilt;
    beta = fclamp(beta, -0.95f, 0.95f);
    return powf((1.f + beta) / (1.f - beta), 1.5f);
}

/*
 * gravitational_redshift — Schwarzschild redshift factor seen by a
 * distant observer of light emitted at radius r.
 *
 *   g = √(1 − r_s/r)
 *
 * At r → r_s        : g → 0  (infinite redshift; light vanishes)
 * At r = 1.5 r_s    : g ≈ 0.577  (photon sphere)
 * At r = 3 r_s ISCO : g ≈ 0.816  (innermost disk material)
 * At r → ∞          : g → 1
 *
 * The fmaxf(0.01, ...) floor is defensive against any r < r_s sneaking
 * through the disk range check.
 */
static float gravitational_redshift(float disk_r)
{
    return sqrtf(fmaxf(0.01f, 1.f - 1.f / disk_r));
}

/*
 * disk_radial_temperature — thermal brightness profile vs radius.
 *
 *   power-law falloff   = (1 − 0.86 · r_norm)^2.2
 *   ISCO spike          = 0.65 · exp(−(r − DISK_IN)² · 0.65)
 *
 *     r_norm = (disk_r − DISK_IN) / (DISK_OUT − DISK_IN)  ∈ [0, 1]
 *
 * The power-law gives the standard cooler-with-radius outer-disk fade.
 * The Gaussian spike models the brightening at the inner edge where
 * infalling matter piles up at the ISCO before plunging through the
 * horizon — visually the brightest part of the disk.
 */
static float disk_radial_temperature(float disk_r)
{
    float r_n  = fclamp((disk_r - DISK_IN) / (DISK_OUT - DISK_IN), 0.f, 1.f);
    float dr   = disk_r - DISK_IN;
    float isco = expf(-dr * dr * 0.65f);
    return powf(1.f - 0.86f * r_n, 2.2f) + 0.65f * isco;
}

/*
 * spiral_density_texture — rotating 5-arm spiral modulation.
 *
 *   tex = 1 + 0.18 · sin(5 · disk_phi − 4 · disk_angle)
 *
 * Subtle ±18 % brightness modulation laid over the disk in fixed
 * inertial-frame coordinates (disk_phi is the inertial-frame azimuth
 * stored in the lensing table; the angle just precesses with the
 * rigid disk_angle).  Reads as a "spiral wake" pattern.
 */
static float spiral_density_texture(float disk_phi, float disk_angle)
{
    return 1.f + 0.18f * sinf(disk_phi * 5.f - disk_angle * 4.f);
}

/*
 * accumulate_clump_bumps — Σ over all clumps of a 2-D Gaussian
 * brightness bump centred on each clump's (r, φ).
 *
 *   for each clump k:
 *     Δφ = wrap_pi(disk_phi − clump_phi_k)
 *     Δr = disk_r − clump_r_k
 *     a  = Δφ²·30 + Δr²·1.6                  // 1/σ_φ² ≈ 30, 1/σ_r² ≈ 1.6
 *     if a < 6:  bump += I_k · exp(−a)
 *
 * σ_φ ≈ 0.18 rad (~10°), σ_r ≈ 0.79 r_s (~9 % of disk width).  The
 * a < 6 early-bail means most cells skip the expf entirely — the
 * per-pixel cost is therefore close to zero away from clumps.
 */
static float accumulate_clump_bumps(float disk_r, float disk_phi)
{
    float bump = 0.0f;
    for (int k = 0; k < N_CLUMPS; k++) {
        float dphi = disk_phi - g_scene.clumps[k].phi;
        if      (dphi >  (float)M_PI)  dphi -= (float)(2.0 * M_PI);
        else if (dphi < -(float)M_PI)  dphi += (float)(2.0 * M_PI);
        float dr = disk_r - g_scene.clumps[k].r;
        float a  = dphi*dphi * 30.0f + dr*dr * 1.6f;
        if (a < 6.0f)
            bump += g_scene.clumps[k].intensity * expf(-a);
    }
    return bump;
}

/*
 * paint_disk_pixel — given a final disk brightness and a normalised
 * radius, pick glyph + colour pair + attribute via disk_char / disk_pair
 * and draw at (row, col).
 */
static void paint_disk_pixel(int row, int col, float brightness, float r_norm)
{
    int    cp;
    attr_t a;
    disk_pair(brightness, r_norm, &cp, &a);
    attron(COLOR_PAIR(cp) | a);
    mvaddch(row, col, (chtype)(unsigned char)disk_char(brightness));
    attroff(COLOR_PAIR(cp) | a);
}

/*
 * photon_ring_bloom — sum of the three-term bloom around the photon
 * sphere, as a function of the ray's closest-approach radius mr.
 *
 *   main = exp(−(mr − 1.5) · 1.4)           // broad halo
 *   rim  = 0.55 · exp(−(mr − 1) · 10)        // near-horizon spike (mr < 1.2)
 *   echo = 0.40 · G(σ_1) + 0.30 · G(σ_2)     // n = 1, 2 sub-ring echoes
 *
 *   G(σ_k): Gaussian centred at PHOTON_R + 0.06, +0.02 respectively.
 *
 * Result is clamped to [0, 1].  Caller buckets into 4 brightness
 * tiers for glyph + colour selection.
 */
static float photon_ring_bloom(float min_r)
{
    float main = expf(-(min_r - PHOTON_R) * 1.4f);

    float rim = (min_r < 1.20f)
              ? 0.55f * expf(-(min_r - 1.0f) * 10.0f)
              : 0.0f;

    float e1   = min_r - (PHOTON_R + 0.06f);
    float e2   = min_r - (PHOTON_R + 0.02f);
    float echo = 0.40f * expf(-e1*e1 * 250.0f)
               + 0.30f * expf(-e2*e2 * 700.0f);

    return fclamp(main + rim + echo, 0.0f, 1.0f);
}

/*
 * paint_photon_ring_pixel — 4-tier brightness selection for the
 * photon-ring bloom.
 *
 *   rb > 0.85 : '#'  CP_RING + A_BOLD   (peak)
 *   rb > 0.55 : '*'  CP_RING + A_BOLD   (bright halo)
 *   rb > 0.30 : '+'  CP_HOT             (mid halo)
 *   else      : '.'  CP_WARM            (outer fade)
 */
static void paint_photon_ring_pixel(int row, int col, float brightness)
{
    int    cp;
    attr_t a;
    char   ch;
    if      (brightness > 0.85f) { cp = CP_RING; a = A_BOLD;   ch = '#'; }
    else if (brightness > 0.55f) { cp = CP_RING; a = A_BOLD;   ch = '*'; }
    else if (brightness > 0.30f) { cp = CP_HOT;  a = A_NORMAL; ch = '+'; }
    else                         { cp = CP_WARM; a = A_NORMAL; ch = '.'; }
    attron(COLOR_PAIR(cp) | a);
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(cp) | a);
}

/*
 * paint_lensed_star — look up a star at the cell's escape direction
 * and draw it if present.  Only called for R_ESCAPED cells where the
 * photon-ring bloom didn't paint (so glyphs never collide).
 */
static void paint_lensed_star(int row, int col, float esc_th, float esc_ph)
{
    char   glyph;
    attr_t a;
    if (star_lookup(esc_th, esc_ph, &glyph, &a)) {
        attron(COLOR_PAIR(CP_STAR) | a);
        mvaddch(row, col, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(CP_STAR) | a);
    }
}

/*
 * render_disk_cell — full per-pixel pipeline for one R_DISK cell.
 *
 * Pseudocode:
 *   phi_rot ← c->disk_phi + disk_angle
 *   D       ← keplerian_doppler_factor(c->disk_r, phi_rot, cos_tilt)
 *   g       ← gravitational_redshift(c->disk_r)
 *   rad     ← disk_radial_temperature(c->disk_r)
 *   tex     ← spiral_density_texture(c->disk_phi, disk_angle)
 *   bump    ← accumulate_clump_bumps(c->disk_r, c->disk_phi)
 *   bright  ← clamp(D · g · rad · tex + bump,  0, 1)
 *   if bright > 0.07:
 *       r_norm ← (c->disk_r − DISK_IN) / (DISK_OUT − DISK_IN)
 *       paint_disk_pixel(row, col, bright, r_norm)
 */
static void render_disk_cell(const Cell *c, float disk_angle, float cos_tilt,
                             int row, int col)
{
    float phi_rot = c->disk_phi + disk_angle;
    float D       = keplerian_doppler_factor(c->disk_r, phi_rot, cos_tilt);
    float g       = gravitational_redshift(c->disk_r);
    float rad     = disk_radial_temperature(c->disk_r);
    float tex     = spiral_density_texture(c->disk_phi, disk_angle);
    float bright  = fclamp(D * g * rad * tex, 0.f, 1.f);

    bright = fclamp(bright + accumulate_clump_bumps(c->disk_r, c->disk_phi),
                    0.f, 1.f);
    if (bright < 0.07f) return;   /* trim dim outer-disk scatter */

    float r_norm = fclamp((c->disk_r - DISK_IN) / (DISK_OUT - DISK_IN),
                          0.f, 1.f);
    paint_disk_pixel(row, col, bright, r_norm);
}

/*
 * render_escaped_cell — paint one R_ESCAPED cell as photon-ring bloom
 * first, falling through to a lensed-star paint if the bloom is too
 * dim and stars are enabled.
 *
 * Pseudocode:
 *   if c->min_r < 4:
 *       rb ← photon_ring_bloom(c->min_r)
 *       if rb > 0.06:  paint_photon_ring_pixel; return
 *   if show_stars:    paint_lensed_star(c->esc_th, c->esc_ph)
 */
static void render_escaped_cell(const Cell *c, int row, int col, int show_stars)
{
    if (c->min_r < 4.0f) {
        float rb = photon_ring_bloom(c->min_r);
        if (rb > 0.06f) {
            paint_photon_ring_pixel(row, col, rb);
            return;
        }
    }
    if (show_stars)
        paint_lensed_star(row, col, c->esc_th, c->esc_ph);
}

/*
 * render — one animation frame.
 *
 * Pseudocode:
 *   cos_tilt ← cos(tilt_deg · π/180)
 *   compute screen-space clip radius from disk angular extent
 *   for each (row, col) inside the clip:
 *       c ← g_scene.table[row][col]
 *       dispatch on c->kind:
 *           R_HORIZON → leave black (erase() pre-cleared the cell)
 *           R_DISK    → render_disk_cell(c, disk_angle, cos_tilt, row, col)
 *           R_ESCAPED → render_escaped_cell(c, row, col, show_stars)
 */
static void render(float disk_angle, int cols, int rows, float cam_dist,
                   float tilt_deg, int show_stars)
{
    float cos_tilt = cosf(tilt_deg * (float)M_PI / 180.0f);

    int rows_lim = rows < MAX_ROWS ? rows : MAX_ROWS;
    int cols_lim = cols < MAX_COLS ? cols : MAX_COLS;

    /*
     * Screen-space clip radius (isotropic cell units from centre).
     * Scales with cam_dist so closer views don't clip the disk edge.
     * Derived from the disk outer edge angle: tan(atan(DISK_OUT/cam_dist))
     * divided by tan(half-FOV), giving the NDC fraction the disk occupies.
     * Factor 1.24 matches the clip to the actual rendered disk boundary
     * (calibrated at default distance); cap at 0.96 to stay on screen.
     */
    float cx = (float)cols * 0.5f;
    float cy = (float)rows * 0.5f;
    float fov_h_tan = tanf(FOV_DEG * (float)M_PI / 360.0f);
    float clip_frac = fminf((DISK_OUT / cam_dist) / fov_h_tan * 1.24f, 0.96f);
    float clip_r2   = (cx * clip_frac) * (cx * clip_frac);

    for (int row = 0; row < rows_lim - 1; row++) {
        for (int col = 0; col < cols_lim; col++) {
            float sdx = (float)col - cx;
            float sdy = ((float)row - cy) / ASPECT;
            if (sdx*sdx + sdy*sdy > clip_r2) continue;

            const Cell *c = &g_scene.table[row][col];
            switch (c->kind) {
            case R_HORIZON:
                /* shadow — black, erase() already cleared the cell */
                break;
            case R_DISK:
                render_disk_cell(c, disk_angle, cos_tilt, row, col);
                break;
            case R_ESCAPED:
                render_escaped_cell(c, row, col, show_stars);
                break;
            }
        }
    }
}

/* ── §9  screen / HUD ───────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run    = 1;
static volatile sig_atomic_t g_resize = 0;

static void on_sigint  (int s){ (void)s; g_run    = 0; }
static void on_sigwinch(int s){ (void)s; g_resize = 1; }
static void cleanup    (void) { endwin(); }

static void screen_init(int *cols, int *rows)
{
    initscr();
    cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    getmaxyx(stdscr, *rows, *cols);
}

/*
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0  right (CP_HUD  bright yellow + bold)  — live status
 *   row -1 left  (CP_HINT bright cyan   + bold)  — actions / keys
 */
static void screen_hud(int cols, int rows, float fps, float cam_dist,
                       float tilt_deg, int theme, int paused)
{
    /* Row 0 — right-aligned live status. */
    char top[180];
    snprintf(top, sizeof top,
             " dist:%.0f  tilt:%.0f  theme:%s  %s  %.0f fps ",
             (double)cam_dist, (double)tilt_deg,
             g_themes[theme % THEME_N].name,
             paused ? "PAUSED " : "running",
             (double)fps);
    int top_len = (int)strlen(top);
    int top_col = cols - top_len;
    if (top_col < 0) top_col = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, top_col, top, cols);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Row rows-1 — action keys. */
    const char *hint =
        " q:quit  p:pause  r:reset  t:theme  +/-:dist  a/A:tilt  k:stars ";
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvaddnstr(rows - 1, 0, hint, cols);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §10  main ───────────────────────────────────────────────────────────── */

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT,   on_sigint);
    signal(SIGWINCH, on_sigwinch);

    int cols, rows;
    screen_init(&cols, &rows);
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);

    /* All scene state (cam_dist, tilt_deg, spin, disk_ang, paused, theme,
     * show_stars, table, clumps) now lives in the file-scope g_scene
     * struct defined in §6.  The Scene docstring explains the sim/
     * rendering split and which keys mutate each group.
     *
     * Main-loop bookkeeping (timing, FPS, the need_rebuild flag) stays
     * here as local state — it's not physics, not display, just loop
     * pacing. */
    theme_apply(g_scene.theme);
    clumps_init();
    erase();
    precompute(cols, rows, g_scene.cam_dist, g_scene.tilt_deg);

    long long tick_ns    = 1000000000LL / SIM_FPS;
    long long frame_ns   = 1000000000LL / RENDER_FPS;
    long long sim_accum  = 0;
    long long frame_time = clock_ns();
    long long fps_acc    = 0;
    int       fps_cnt    = 0;
    float     fps        = 0.f;
    int       need_rebuild = 0;

    while (g_run) {

        /* ── resize ── */
        if (g_resize || need_rebuild) {
            g_resize     = 0;
            need_rebuild = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            erase();
            precompute(cols, rows, g_scene.cam_dist, g_scene.tilt_deg);
            sim_accum  = 0;
            frame_time = clock_ns();
        }

        /* ── dt ── */
        long long now = clock_ns();
        long long dt  = now - frame_time;
        if (dt > 100000000LL) dt = 100000000LL;
        frame_time = now;

        /* ── physics ── */
        if (!g_scene.paused) {
            sim_accum += dt;
            while (sim_accum >= tick_ns) {
                g_scene.disk_ang += g_scene.spin;
                if (g_scene.disk_ang >= (float)(2.0*M_PI))
                    g_scene.disk_ang -= (float)(2.0*M_PI);
                clumps_tick();         /* Keplerian flow of hot spots */
                sim_accum -= tick_ns;
            }
        }

        /* ── FPS ── */
        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        /* ── draw ── */
        long long t0 = clock_ns();
        erase();
        render(g_scene.disk_ang, cols, rows,
               g_scene.cam_dist, g_scene.tilt_deg, g_scene.show_stars);
        screen_hud(cols, rows, fps,
                   g_scene.cam_dist, g_scene.tilt_deg,
                   g_scene.theme, g_scene.paused);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── input ── */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_run = 0;                       break;
        case 'p': case 'P': g_scene.paused = !g_scene.paused;         break;
        case 'r': case 'R':
            g_scene.disk_ang = 0.f;
            clumps_init();    /* reseed flowing knots */
            break;
        case 't': case 'T':
            g_scene.theme = (g_scene.theme + 1) % THEME_N;
            theme_apply(g_scene.theme);
            break;
        case '+': case '=':
            /* closer camera → bigger Gargantua on screen */
            g_scene.cam_dist = fclamp(g_scene.cam_dist - CAM_DIST_STEP,
                                      CAM_DIST_MIN, CAM_DIST_MAX);
            need_rebuild = 1;
            break;
        case '-':
            /* farther camera → smaller Gargantua on screen */
            g_scene.cam_dist = fclamp(g_scene.cam_dist + CAM_DIST_STEP,
                                      CAM_DIST_MIN, CAM_DIST_MAX);
            need_rebuild = 1;
            break;
        case 'a':
            /* more tilt → more face-on */
            g_scene.tilt_deg = fclamp(g_scene.tilt_deg + TILT_DEG_STEP,
                                      TILT_DEG_MIN, TILT_DEG_MAX);
            need_rebuild = 1;
            break;
        case 'A':
            /* less tilt → more edge-on */
            g_scene.tilt_deg = fclamp(g_scene.tilt_deg - TILT_DEG_STEP,
                                      TILT_DEG_MIN, TILT_DEG_MAX);
            need_rebuild = 1;
            break;
        case 'k': case 'K':
            g_scene.show_stars = !g_scene.show_stars;
            break;
        }

        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }

    endwin();
    return 0;
}
