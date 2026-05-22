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
 *     We set r_s = 1, so horizon at r = 1, photon sphere at r = 1.5, ISCO r
 * = 3.
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
 *                    Photon sphere: r = 1.5 (r_s)  — unstable circular photon
 * orbit ISCO:          r = 3 (r_s)    — innermost stable circular orbit
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
 * Data-structure: A single Scene struct owns the whole program state,
 *                 composed of concept-sized sub-structs so each field's
 *                 role is obvious at the access site:
 *
 *                   Camera        — cam_dist, tilt_deg  (changing either
 *                                     triggers a lensing-table rebuild)
 *                   DiskSim       — spin, angle, paused (the per-tick
 *                                     accretion-disk animation state)
 *                   RenderConfig  — theme, show_stars   (purely cosmetic;
 *                                     never triggers a rebuild)
 *                   Screen        — cols, rows          (ncurses dims)
 *                   FrameTimer    — sim-tick / render-frame budgets,
 *                                     accumulator, rolling-fps state
 *                   table[][]     — precomputed Cell lensing table
 *                   clumps[]      — N_CLUMPS orbital hot spots
 *                   color256, running, need_resize, need_rebuild
 *
 *                 Single file-static g_scene instance. The Camera and
 *                 DiskSim split makes the sim/render contract explicit:
 *                 mutating Camera invalidates the table; mutating
 *                 RenderConfig must not.
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
 *                  g          = √(1 − r_s/r)                        //
 * gravitational redshift rad        = (1 − 0.86·r_n)^2.2 + 0.65·exp(−Δr²) //
 * ISCO spike tex        = 1 + 0.18·sin(5φ − 4·disk_angle)     // spiral texture
 *                  bump       = Σₖ Iₖ · exp(-Δφ²/σ_φ² - Δr²/σ_r²)   // hot-spot
 * clumps brightness = clamp(D · g · rad · tex + bump) pick disk_char +
 * disk_pair, mvaddch.
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
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1  config ─────────────────────────────────────────────────────────── */

#define SIM_FPS 20
#define RENDER_FPS 60
#define ASPECT 0.47f /* terminal cell h / w                    */

/* Schwarzschild geometry in units where r_s = 1 (Schwarzschild radius).
 * All lengths are dimensionless multiples of r_s = 2GM/c².               */
#define BH_R 1.0f /* event horizon radius: r = r_s (= 2GM/c²)   */
#define PHOTON_R                                                               \
  1.5f /* photon sphere: r = 3/2 · r_s.  Unstable                             \
        * circular orbit — photons here spiral in or out;                    \
        * gives the bright ring visible around the shadow. */
#define DISK_IN                                                                \
  3.0f                 /* ISCO (innermost stable circular orbit): r = 3·r_s.  \
                        * Timelike (massive particle) orbits below r=3 are     \
                        * unstable → material plunges into horizon.          \
                        * The accretion disk begins here, not at r=1.       */
#define DISK_OUT 12.0f /* outer disk edge (arbitrary, visually tuned).      */

/* camera — both tilt and distance are runtime-adjustable                  */
#define TILT_DEG_DEF                                                           \
  5.0f /* default inclination above equatorial plane                           \
        * (a / A keys cycle; rebuilds lensing table)*/
#define TILT_DEG_MIN                                                           \
  0.0f /* edge-on: camera in disk plane — max                                \
        * Doppler asymmetry, thin disk silhouette  */
#define TILT_DEG_MAX                                                           \
  85.0f                    /* nearly face-on; capped < 90° to avoid the       \
                            * camera-basis degeneracy at fwd ∥ world_up*/
#define TILT_DEG_STEP 5.0f /* per keypress                              */
#define FOV_DEG 72.0f      /* horizontal field-of-view               */
#define CAM_DIST_DEF                                                           \
  24.0f /* default camera distance (r_s units) —                             \
         * comfortable framing: full disk + shadow                             \
         * visible with a clear lensing arc above                              \
         * the horizon, not so close that the disk                             \
         * overruns the FOV.                      */
#define CAM_DIST_MIN                                                           \
  4.0f                     /* closest  → largest Gargantua on screen.        \
                            * Stays just outside ISCO (r = 3) so the           \
                            * camera isn't inside the disk plane.    */
#define CAM_DIST_MAX 72.0f /* farthest → smallest, far-field flat-ish*/
#define CAM_DIST_STEP 1.5f /* step per keypress (finer near MIN)     */

/* disk rotation                                                            */
#define SPIN_DEF 0.04f /* rad / sim-tick                         */

/* ray integration                                                          */
#define MAX_STEPS 900
#define ESCAPE_R 130.0f /* fixed far-field boundary (r_s units)   */
#define DS_BASE 0.10f   /* affine-parameter step size             */

/* table dimensions                                                         */
#define MAX_COLS 512
#define MAX_ROWS 256

/* orbital hot-spot clumps (bright knots of disk material flowing past
 * the camera at the local Keplerian rate Ω(r) = √(M/r³)).  CLUMP_RATE_SCALE
 * is tuned so that at r_ref = 6 the clump rate ≈ SPIN_DEF — clumps near
 * the disk midline drift at roughly the same speed as the texture spiral. */
#define N_CLUMPS 10
#define CLUMP_RATE_SCALE 0.85f

/* ── §1.1 physics constants (Schwarzschild, geometric units) ────── */

/* Mass parameter in geometric units c = G = 1, r_s = 2M = 1. Used by
 * Keplerian orbital speed v = √(M/r) and similar. */
#define SCHWARZSCHILD_M           0.5f

/* "Captured by the horizon" threshold. r < BH_R · HORIZON_FUDGE means
 * the photon is on the inevitable plunge — we stop integrating before
 * the singularity would force a divide-by-zero. */
#define HORIZON_FUDGE             0.92f

/* Doppler beaming factor [(1+β)/(1−β)]^DOPPLER_EXPONENT. The full SR
 * value is 3 or 4; we use 3/2 to keep the dynamic range inside the
 * 9-tier disk character ramp (3 would saturate everything to '@' on
 * the approaching side). */
#define DOPPLER_EXPONENT          1.5f

/* β clamp — avoid the v → c pole in (1±β)^k. */
#define DOPPLER_BETA_CLAMP        0.95f

/* ── §1.2 render cutoffs ─────────────────────────────────────────── */

/* Below this brightness the disk pixel paints nothing (skip dim
 * outer-disk scatter). */
#define DISK_MIN_VISIBLE          0.07f

/* Photon-ring bloom is only checked for rays whose closest approach
 * was within this radius — outside the bloom is negligible anyway. */
#define BLOOM_NEAR_RANGE          4.0f

/* Below this bloom intensity, fall through to lensed-star rendering
 * (so glyphs never collide). */
#define BLOOM_MIN_VISIBLE         0.06f

/* ── §1.3 star field ─────────────────────────────────────────────── */

/* Celestial-sphere quantisation for star_lookup: cells per radian.
 * 30 cells/rad → ~3° per cell, ~5800 cells over the full sphere. */
#define STAR_GRID_PER_RAD         30.0f

/* Density gate: stars in cells where (hash & 0xFF) < this threshold.
 * 4 out of 256 ≈ ~1.6%. */
#define STAR_DENSITY_BUCKET       4u

/* Brightness classification thresholds on the second hash byte. */
#define STAR_BRIGHT_THRESH      220u
#define STAR_MID_THRESH         100u

/* ── §1.4 main-loop timing ───────────────────────────────────────── */

/* Cap dt at 100 ms so a process suspension doesn't cause a huge
 * "catch-up" burst of sim ticks (spiral-of-death prevention). */
#define DT_CAP_NS         100000000LL

/* Rolling-fps window. fps_display refreshed when fps_acc crosses this. */
#define FPS_WINDOW_NS     500000000LL

/* ── §2  clock ──────────────────────────────────────────────────────────── */

static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ── §3  color / themes ─────────────────────────────────────────────────── */

enum {
  CP_RING = 1, /* photon ring / white-hot inner disk    — per theme */
  CP_HOT,      /* inner disk                            — per theme */
  CP_WARM,     /* mid-inner                             — per theme */
  CP_MID,      /* mid                                   — per theme */
  CP_COOL,     /* outer                                 — per theme */
  CP_DIM,      /* far outer                             — per theme */
  CP_STAR,     /* background stars      — fixed light grey          */
  CP_HUD,      /* HUD top status        — bright yellow + bold      */
  CP_HINT,     /* HUD bottom hint bar   — bright cyan   + bold      */
  CP_COUNT
};

/* Scene is defined in §6 below; theme_apply() reads g_scene.color256.
 * Forward-declared here so §3 doesn't depend on §6's typedef order. */
struct Scene_;
typedef struct Scene_ Scene;
extern Scene g_scene;
static inline int scene_is_256_color(void);   /* defined in §6 */

/*
 * Theme — one entry in the g_themes[] palette table.
 *
 * INTENT
 *   A theme is a six-tier 256-colour ramp from ring (brightest, used
 *   for the photon ring + white-hot inner disk) down to dim (used
 *   for outer-disk falloff). The 't' key cycles g_scene.render.theme
 *   through g_themes[]; theme_apply() then rebinds CP_RING .. CP_DIM
 *   from the selected row. CP_STAR / CP_HUD / CP_HINT use fixed chrome
 *   constants regardless of theme so HUD/star colours stay legible.
 *
 * WHY SIX TIERS (not 4, not 8)
 *   disk_pair() in §8 selects one of six brightness buckets per pixel.
 *   Fewer tiers and the inner-to-outer falloff would visibly step;
 *   more and the per-bucket palette entries become indistinguishable
 *   on most terminals. Six is the legibility sweet spot for ASCII
 *   gradients.
 *
 * BRIGHTNESS SAFETY (CLAUDE.md theme rule)
 *   Every entry sits at index ≥ 30, OR 24-29 / 240-243 only as the
 *   lowest ramp tier. 16-23 / 232-239 are forbidden — they vanish
 *   against the default terminal background.
 *
 * ALGORITHM REFERENCE
 *   Ware (2020) ch. 4 — perceptually-ordered colour / luminance ramps.
 *   The six-tier shape mirrors a standard brightness ramp from
 *   "perception for design" (white-hot → ember). Theme "Blackbody"
 *   additionally honours Wien's law: inner disk = hot blue, outer = cool
 *   red (the physically-correct temperature profile for a thin disk).
 */
typedef struct {
  const char *name; /* short label shown in the HUD top row.            */
  short ring;       /* tier 0 — peak white-hot. Photon-ring '#' / '*'   *
                     * AND the inner-disk peak buckets in disk_pair.    */
  short hot;        /* tier 1 — inner-disk warm core.                   */
  short warm;       /* tier 2 — mid-inner band.                         */
  short mid;        /* tier 3 — disk mid.                               */
  short cool;       /* tier 4 — outer band.                             */
  short dim;        /* tier 5 — far-outer disk fade.                    */
} Theme;

static const Theme g_themes[] = {
    /*  name        ring hot  warm mid  cool dim                       */
    {"Matrix", 46, 118, 82, 40, 34, 28},         /* cyber green       */
    {"Fire", 231, 226, 220, 208, 166, 130},      /* white-hot → ember */
    {"Oceanic", 51, 87, 45, 44, 37, 31},         /* bioluminescent    */
    {"Neon", 231, 213, 177, 165, 129, 93},       /* pink / magenta    */
    {"Mono", 255, 252, 248, 244, 242, 240},      /* grayscale         */
    {"Ice", 231, 195, 159, 123, 117, 111},       /* polar white-blue  */
    {"Nova", 231, 195, 153, 111, 105, 99},       /* stellar violet    */
    {"Forest", 156, 142, 100, 94, 64, 58},       /* leaves to bark    */
    {"Desert", 230, 220, 214, 180, 136, 94},     /* sand / gold       */
    {"Eclipse", 196, 160, 124, 88, 52, 240},     /* red corona / dark */
    {"Blackbody", 153, 195, 231, 226, 214, 196}, /* physical T(r):
                                                    inner=blue-white
                                                    hot, outer=red
                                                    cool (Wien's law) */
};
#define THEME_N ((int)(sizeof g_themes / sizeof g_themes[0]))
#define THEME_DEF 10 /* Blackbody — physically-correct T(r) ramp */

/* Fixed chrome — same on every theme. */
#define CHROME_STAR_256 253 /* light grey         */
#define CHROME_HUD_256 226  /* bright yellow      */
#define CHROME_HINT_256 51  /* bright cyan        */

/*
 * 8-colour fallback palette — theme-independent warm ramp + standard
 * chrome. Used when COLORS < 256 (basic xterm, tmux without truecolor,
 * etc). The 6 disk tiers collapse to a warm white→yellow→red gradient.
 */
static const struct { short pair; short fg; } FALLBACK_PALETTE[] = {
    {CP_RING,  COLOR_WHITE },
    {CP_HOT,   COLOR_YELLOW},
    {CP_WARM,  COLOR_YELLOW},
    {CP_MID,   COLOR_RED   },
    {CP_COOL,  COLOR_RED   },
    {CP_DIM,   COLOR_RED   },
    {CP_STAR,  COLOR_WHITE },
    {CP_HUD,   COLOR_YELLOW},
    {CP_HINT,  COLOR_CYAN  },
};
#define FALLBACK_PALETTE_LEN \
    (int)(sizeof FALLBACK_PALETTE / sizeof FALLBACK_PALETTE[0])

/* Bind the 6 disk tiers from a 256-mode Theme, then the 3 chrome
 * pairs that stay constant across all themes. */
static void theme_apply_256(const Theme *t) {
  init_pair(CP_RING, t->ring, -1);
  init_pair(CP_HOT,  t->hot,  -1);
  init_pair(CP_WARM, t->warm, -1);
  init_pair(CP_MID,  t->mid,  -1);
  init_pair(CP_COOL, t->cool, -1);
  init_pair(CP_DIM,  t->dim,  -1);
  init_pair(CP_STAR, CHROME_STAR_256, -1);
  init_pair(CP_HUD,  CHROME_HUD_256,  -1);
  init_pair(CP_HINT, CHROME_HINT_256, -1);
}

/* Walk the 8-colour fallback table — same pairs, theme-independent. */
static void theme_apply_8(void) {
  for (int i = 0; i < FALLBACK_PALETTE_LEN; i++)
    init_pair(FALLBACK_PALETTE[i].pair, FALLBACK_PALETTE[i].fg, -1);
}

static void theme_apply(int idx) {
  const Theme *t = &g_themes[idx % THEME_N];
  if (scene_is_256_color()) theme_apply_256(t);
  else                      theme_apply_8();
}

/* ── §4  V3 math ─────────────────────────────────────────────────────────── */

/*
 * V3 — 3-D vector in geometric units (c = G = 1, r_s = 2M = 1).
 *
 * INTENT
 *   Shared currency of the geodesic integrator and the camera frame.
 *   Photon position, photon velocity, camera origin, camera basis
 *   vectors — all V3s in the same Cartesian frame centred on the
 *   black hole at the origin.
 *
 * COORDINATE CONVENTION
 *   y is the polar axis (perpendicular to the disk plane). The
 *   accretion disk lies in the y = 0 plane with cylindrical radius
 *   √(x² + z²) ∈ [DISK_IN, DISK_OUT]. At tilt = 0 the camera sits
 *   at (0, 0, −cam_dist); positive tilt lifts it toward +y above
 *   the equator.
 *
 *   A photon's progress is tracked by its squared norm
 *      r²  = x² + y² + z²
 *   which appears in the geodesic equation's r^−5 force term.
 *
 * UNITS (geometric, c = G = 1, M = 0.5 ⇒ r_s = 1)
 *   All distances are dimensionless multiples of r_s. Velocity is
 *   in units where c = 1 (affine-parameter velocity for null
 *   geodesics, with |vel| ≈ 1 throughout the trace).
 *
 * ALGORITHM REFERENCE
 *   Cartesian embedding of Schwarzschild null geodesics — MTW §25 [1].
 *   The 3-D Cartesian form d²pos/dλ² = −(3/2)|h|²·pos/r⁵ is derived by
 *   casting the Binet equation into vector form via h = pos × vel.
 */
typedef struct {
  float x; /* horizontal (camera-right at tilt = 0), r_s units      */
  float y; /* polar axis perpendicular to disk plane, r_s units     */
  float z; /* horizontal (camera-back at tilt = 0), r_s units       */
} V3;

static inline float v3dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len2(V3 a) { return v3dot(a, a); }
static inline float v3len(V3 a) { return sqrtf(v3len2(a)); }
static inline V3 v3add(V3 a, V3 b) {
  return (V3){a.x + b.x, a.y + b.y, a.z + b.z};
}
static inline V3 v3sub(V3 a, V3 b) {
  return (V3){a.x - b.x, a.y - b.y, a.z - b.z};
}
static inline V3 v3scale(float s, V3 a) {
  return (V3){s * a.x, s * a.y, s * a.z};
}
static inline V3 v3cross(V3 a, V3 b) {
  return (V3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x};
}
static inline V3 v3norm(V3 a) {
  float l = v3len(a);
  return l > 1e-12f ? v3scale(1.0f / l, a) : (V3){0, 1, 0};
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
static void geo_deriv(V3 pos, V3 vel, V3 *dpos, V3 *dvel) {
  *dpos = vel;
  V3 h = v3cross(pos, vel);
  float h2 = v3len2(h);
  float r2 = v3len2(pos);
  float r = sqrtf(r2);
  float coef = -1.5f * h2 / (r2 * r2 * r); /* −(3h²/2r^5) */
  *dvel = v3scale(coef, pos);
}

/* RK4 weighted average:  (k1 + 2·k2 + 2·k3 + k4) / 6.
 * The classical 4th-order Runge-Kutta increment. Pulled out so the
 * step body reads as "average the four slopes" rather than a nested
 * v3add chain. */
static V3 rk4_weighted_avg(V3 k1, V3 k2, V3 k3, V3 k4) {
  V3 sum2 = v3add(v3scale(2.0f, k2), v3scale(2.0f, k3));
  V3 total = v3add(v3add(k1, k4), sum2);
  return v3scale(1.0f / 6.0f, total);
}

/*
 * geo_step — one RK4 step of size ds along the null geodesic.
 *
 *   k1 = f(y)                    slope at current point
 *   k2 = f(y + ds/2 · k1)        slope at midpoint, using k1
 *   k3 = f(y + ds/2 · k2)        slope at midpoint, using k2
 *   k4 = f(y + ds   · k3)        slope at endpoint, using k3
 *   y_next = y + ds · (k1 + 2·k2 + 2·k3 + k4) / 6
 *
 * Reads as four slope samples + a weighted average per state variable.
 */
static void geo_step(V3 *pos, V3 *vel, float ds) {
  V3 dp1, dv1, dp2, dv2, dp3, dv3, dp4, dv4;

  /* (1) slope at the current point */
  geo_deriv(*pos, *vel, &dp1, &dv1);

  /* (2) slope at the midpoint, predictor using k1 */
  V3 pa = v3add(*pos, v3scale(0.5f * ds, dp1));
  V3 va = v3add(*vel, v3scale(0.5f * ds, dv1));
  geo_deriv(pa, va, &dp2, &dv2);

  /* (3) slope at the midpoint, corrector using k2 */
  V3 pb = v3add(*pos, v3scale(0.5f * ds, dp2));
  V3 vb = v3add(*vel, v3scale(0.5f * ds, dv2));
  geo_deriv(pb, vb, &dp3, &dv3);

  /* (4) slope at the endpoint, predictor using k3 */
  V3 pc = v3add(*pos, v3scale(ds, dp3));
  V3 vc = v3add(*vel, v3scale(ds, dv3));
  geo_deriv(pc, vc, &dp4, &dv4);

  /* (5) advance state by ds · weighted-average-slope */
  *pos = v3add(*pos, v3scale(ds, rk4_weighted_avg(dp1, dp2, dp3, dp4)));
  *vel = v3add(*vel, v3scale(ds, rk4_weighted_avg(dv1, dv2, dv3, dv4)));
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
  float disk_r;   /* (R_DISK)    cylindrical radius of hit, r_s units */
  float disk_phi; /* (R_DISK)    inertial-frame azimuth, [-π, π]     */
  float esc_th;   /* (R_ESCAPED) polar    angle of escape, [0, π]    */
  float esc_ph;   /* (R_ESCAPED) azimuth   angle of escape, [-π, π]  */
  float min_r;    /* closest approach of the geodesic to the BH —
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
  float r;         /* orbital radius, r_s units; constant per clump.
                    * Sets the Keplerian rate Ω(r) = √(M/r³).          */
  float phi;       /* current inertial-frame azimuth, [0, 2π);
                    * advanced by Ω(r) · CLUMP_RATE_SCALE each tick.   */
  float intensity; /* peak brightness boost at clump centre, ~[0,1].
                    * Randomised at init so clumps read as distinct.   */
} Clump;

/* ─────────────────────────────────────────────────────────────────────── *
 *
 * §6.1 Scalar sub-structs of Scene
 *
 * Scene is composed from five concept-sized sub-structs plus the two
 * large data buffers (table, clumps) and a handful of process flags.
 * Each sub-struct groups fields that share a mutation pattern, so
 * "where does this field live?" answers "what kind of state is it?".
 *
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * Camera — pinhole-camera parameters that determine which rays get
 * shot through the precompute loop.
 *
 * INTENT
 *   Together these two scalars fully define the camera's position and
 *   orientation. The camera sits at distance `cam_dist` from the BH
 *   on the cone of inclination `tilt_deg` above the equatorial plane,
 *   always looking at the origin:
 *
 *      cam = (0,  cam_dist · sin t,  − cam_dist · cos t)     [t = tilt_rad]
 *      fwd = normalize(−cam)                                 [toward BH]
 *      rgt = normalize(fwd × ŷ)                              [screen-right]
 *      up  = rgt × fwd                                       [screen-up]
 *
 *   build_camera_basis() in §7 computes (cam, fwd, rgt, up) from these
 *   two scalars; every ray direction is then `pinhole_pixel_to_ray()`
 *   on that basis.
 *
 * INVARIANT
 *   Mutating ANY field here invalidates the entire lensing table —
 *   different camera position ⇒ different rays ⇒ different outcomes
 *   at every pixel. The caller MUST set g_scene.need_rebuild = 1
 *   after writing here; main()'s next loop iteration runs precompute().
 *
 * WHY tilt_deg HAS A < 90° CAP
 *   At tilt = 90° the camera would sit on +ŷ and `fwd` would be
 *   anti-parallel to world_up (0, 1, 0); the cross product in
 *   build_camera_basis() collapses and `rgt` becomes undefined.
 *   TILT_DEG_MAX = 85° keeps us safely away from that singularity.
 *
 * WHY cam_dist HAS A MIN (NOT 0)
 *   CAM_DIST_MIN = 4 keeps the camera outside the ISCO (r = 3) so the
 *   camera isn't INSIDE the disk plane. Closer than ISCO and the
 *   disk would visually clip / wrap unpredictably.
 *
 * ALGORITHM REFERENCE
 *   Standard pinhole camera projection (Foley/van Dam/Feiner/Hughes,
 *   "Computer Graphics: Principles and Practice", §6.4).
 *   Schwarzschild ray-tracing application — James et al. [3].
 */
typedef struct {
  float cam_dist; /* camera distance from BH, r_s units. Clamped to     *
                   * [CAM_DIST_MIN, CAM_DIST_MAX] = [4, 72]. +/- keys   *
                   * nudge by CAM_DIST_STEP = 1.5. Smaller → closer →   *
                   * bigger Gargantua on screen.                        */
  float tilt_deg; /* inclination above equatorial plane, degrees.       *
                   * Clamped to [TILT_DEG_MIN, TILT_DEG_MAX] = [0, 85]. *
                   * a/A keys nudge by TILT_DEG_STEP = 5°.              *
                   * 0  = edge-on (disk as a thin line, max Doppler     *
                   *      asymmetry)                                    *
                   * 85 = nearly face-on (disk as an annulus around the *
                   *      shadow; the iconic Gargantua look)            */
} Camera;

/*
 * DiskSim — accretion-disk animation state, advanced once per sim tick.
 *
 * INTENT
 *   The disk has TWO motions, layered:
 *     (1) RIGID-ROTATION TEXTURE — `angle` advances by `spin` each
 *         tick. Drives the 5-arm spiral_density_texture() pattern,
 *         which rotates as a single rigid sheet.
 *     (2) KEPLERIAN HOT-SPOT FLOW — each Clump in Scene.clumps
 *         advances its own φ at Ω(r) = √(M/r³) (in clumps_tick()),
 *         so inner clumps visibly lap outer clumps.
 *
 *   The rigid texture creates a consistent "spinning record" pattern;
 *   the clumps create differential motion that breaks the rigid look.
 *   Together they read as a real accretion disk with structure and
 *   flow.
 *
 * WHY spin IS A USER PARAMETER (not derived)
 *   Real accretion disks rotate at SIM_FPS-independent rates — the
 *   visual gait depends on the chosen `spin`. SPIN_DEF = 0.04 rad/tick
 *   at SIM_FPS = 20 gives ~0.8 rad/sec, a comfortable visual tempo.
 *   Tied to SIM_FPS rather than wall-clock: physics is fixed-timestep
 *   in the FrameTimer accumulator pattern, so the rate is per-tick,
 *   not per-second.
 *
 * MUTATION SIDE EFFECTS
 *   `angle` and `paused` are CHEAP to mutate (no rebuild). `spin` is
 *   also cheap, but is currently never written at runtime (no
 *   keyboard binding) — the field is present so adding a "faster/
 *   slower disk" key is a one-line change.
 *
 * ALGORITHM REFERENCES
 *   Keplerian orbital rate            — MTW §25 [1]
 *   Hot-spot disk variability         — James et al. [3]
 *   Spiral density wave pattern       — standard accretion-disk
 *                                       astrophysics (Pringle 1981)
 */
typedef struct {
  float spin;   /* per-tick rigid disk-texture rotation rate (rad/tick).*
                 * Default SPIN_DEF = 0.04. Used as both the rigid      *
                 * texture rate AND as the CLUMP_RATE_SCALE reference   *
                 * (clump rate at r = 6 ≈ spin, so the two motions      *
                 * roughly match at the disk midline).                   */
  float angle;  /* cumulative rigid rotation, [0, 2π). Advanced by spin*
                 * each sim tick when !paused; wrapped on overflow;     *
                 * reset to 0 by the 'r' key (re-syncs clumps too).     */
  int   paused; /* 1 → freeze the simulation entirely (disk angle      *
                 * frozen AND clumps_tick skipped). HUD shows PAUSED.   *
                 * spacebar / 'p' toggles.                              */
} DiskSim;

/*
 * RenderConfig — purely cosmetic flags.
 *
 * INTENT
 *   The "skin" of the renderer. Mutating these MUST leave the lensing
 *   table and clump positions byte-identical — only colours / overlays
 *   may differ. The contract is enforced by SEPARATION from Camera
 *   and DiskSim: a flag here physically can't trigger a rebuild
 *   because the rebuild path doesn't read RenderConfig.
 *
 * INVARIANT (READER GUIDE FOR FUTURE EXTENSIONS)
 *   When adding a new flag, ask: does this change what precompute() /
 *   ray_trace() / clumps_tick() produces? If YES, it belongs in
 *   Camera or DiskSim. If NO (purely changes colour, glyph, opacity,
 *   or overlay), it belongs here.
 *
 * WHY show_stars DEFAULTS OFF
 *   Stars are visually busy at the wide-angle sides of the FOV; they
 *   distract from the photon-ring detail. The user opts in via 'k'
 *   when they want to see the Einstein-ring magnification effect.
 */
typedef struct {
  int theme;      /* index into g_themes[] (§3). 't' key cycles mod    *
                   * THEME_N. theme_apply() rebinds CP_* on change.    *
                   * Default THEME_DEF = 10 (Blackbody).               */
  int show_stars; /* 1 → render lensed background star field via      *
                   * star_lookup(). 0 → no stars (default). 'k' key.  */
} RenderConfig;

/*
 * Screen — current ncurses window dimensions.
 *
 * INTENT
 *   Single point of truth for "how big is the terminal RIGHT NOW".
 *   precompute() and render() both consult these to bound their loops
 *   at  min(rows, MAX_ROWS)  and  min(cols, MAX_COLS) , so the lensing
 *   table is always indexed safely even when the terminal exceeds the
 *   compiled-in maximum.
 *
 * MUTATION RULE
 *   Modified only inside screen_init() (at startup) and the main
 *   loop's resize branch (which calls getmaxyx after SIGWINCH). Never
 *   changes mid-frame — physics and renderer treat them as immutable
 *   for one tick.
 *
 * RESIZE TRIGGERS A FULL TABLE REBUILD
 *   Different dimensions ⇒ different pixel-to-ray mapping ⇒ different
 *   ray outcomes per cell. The resize branch in main() calls
 *   precompute() to rebuild the table from scratch.
 */
typedef struct {
  int cols; /* terminal width in character cells.  Clamped at indexing *
             * time by min(cols, MAX_COLS) inside precompute / render. */
  int rows; /* terminal height in character cells. Same clamp via      *
             * MAX_ROWS.                                                */
} Screen;

/*
 * FrameTimer — main-loop timing bookkeeping.
 *
 * INTENT
 *   Pulls seven loose locals out of main() into one named struct so
 *   the loop body reads as "measure dt → tick physics → draw → sleep"
 *   without scattered accumulators.
 *
 * TWO TIMING SCALES (this struct holds the state for both)
 *   (1) RENDER scale — frame_ns = 1e9/RENDER_FPS. Each loop iteration
 *       paints one frame and sleeps the remainder of frame_ns.
 *   (2) SIM scale — tick_ns = 1e9/SIM_FPS, FIXED. The dt between
 *       frames is accumulated in sim_accum; whenever it crosses
 *       tick_ns, one physics tick is consumed. The sim is therefore
 *       decoupled from the render rate — slow frame → catch-up
 *       ticks; fast frame → ticks straddle frames cleanly.
 *
 * WHY DECOUPLED (renderer != physics)
 *   Disk rotation pace must be CONSISTENT regardless of how fast the
 *   renderer is running. A rigid 1-tick-per-frame would make the disk
 *   spin slower on slow machines, breaking the gait. Fiedler's
 *   "Fix Your Timestep!" is the canonical write-up of this pattern.
 *
 * WHY A 0.5-SEC ROLLING FPS WINDOW
 *   Per-frame fps fluctuates wildly. A 0.5-sec window gives a HUD
 *   reading the human eye can actually read. Shorter = jumpier,
 *   longer = stale.
 *
 * ALGORITHM REFERENCE
 *   Glenn Fiedler, "Fix Your Timestep!" — accumulator pattern.
 *   https://gafferongames.com/post/fix_your_timestep/
 */
typedef struct {
  long long tick_ns;    /* one sim-tick budget in nanoseconds.         *
                         * Computed at boot as 1e9/SIM_FPS = 50 ms.    *
                         * Read-only after init.                       */
  long long frame_ns;   /* one render-frame budget in nanoseconds.     *
                         * Computed at boot as 1e9/RENDER_FPS ≈ 16.7ms.*
                         * Read-only after init.                       */
  long long sim_accum;  /* dt accumulator for fixed-timestep physics.  *
                         * Each frame:  sim_accum += dt; while         *
                         * sim_accum ≥ tick_ns: tick(); sim_accum -=   *
                         * tick_ns. Reset on resize / rebuild.         */
  long long frame_time; /* clock_ns() at the start of the previous     *
                         * frame. dt_this_frame = now - frame_time.    *
                         * Capped at 100 ms to prevent spiral-of-death *
                         * when the process is suspended.              */
  long long fps_acc;    /* rolling-fps window accumulator (ns).        *
                         * Resets when it crosses 500 ms.              */
  int       fps_cnt;    /* frames included in fps_acc.                 *
                         *   fps = fps_cnt · 1e9 / fps_acc             */
  float     fps;        /* most recent rolling-window fps value.       *
                         * Updated ≈ 2× per second; displayed in HUD.  */
} FrameTimer;

/* ─────────────────────────────────────────────────────────────────────── *
 *
 * §6.2 Scene — top-level container
 *
 * INTENT
 *   Single file-static instance (g_scene) owns every long-lived piece
 *   of program state — camera, disk, renderer, screen, timer, the
 *   ~3 MB lensing table, the clumps, and the few process-level flags.
 *   Helpers and main() touch state as `g_scene.<sub>.<field>`; no
 *   parameter threads more than two levels deep.
 *
 * WHY A SINGLE GLOBAL (not passed by pointer)
 *   The lensing table alone is ~3 MB. Passing &g_scene through every
 *   helper costs nothing at the call site but the file-static gives
 *   signal handlers a path to running / need_resize without API
 *   ceremony. The fields outside table[]/clumps[] are all small
 *   scalars, so the compiler folds the dereference chain efficiently.
 *
 * SUB-STRUCT LAYERING (read these in order to understand the program)
 *
 *   1. Camera        what the lensing table was BUILT FROM
 *   2. DiskSim       what advances each tick (and the pause gate)
 *   3. RenderConfig  cosmetic skin (theme, stars)
 *   4. Screen        terminal dimensions
 *   5. FrameTimer    loop pacing (render + sim timebases)
 *   6. table[][]     the precomputed lensing cache
 *   7. clumps[]      orbital hot spots
 *   8. flags         color256, need_rebuild, running, need_resize
 *
 * MUTATION CONTRACT (which keys touch which sub-struct)
 *
 *   Camera         + / -    cam_dist           → need_rebuild = 1
 *                  a / A    tilt_deg           → need_rebuild = 1
 *   DiskSim        p        paused
 *                  r        angle = 0          (clumps reseeded too)
 *                  (per tick) angle += spin
 *   RenderConfig   t        theme              + theme_apply
 *                  k        show_stars
 *   clumps[]       (per tick) Keplerian phi advance
 *                  r        clumps_init        (full reseed)
 *   running        q / Q / ESC, SIGINT, SIGTERM
 *   need_resize    SIGWINCH
 *
 *   Camera and DiskSim writes trigger physics-relevant changes.
 *   RenderConfig writes must NOT (contract enforced by separation).
 *
 * INITIALIZATION ORDER (see main())
 *   1. screen_init   — bring ncurses up, learn rows/cols
 *   2. set color256  — read COLORS, store the capability flag
 *   3. theme_apply   — bind CP_* from the default theme
 *   4. clumps_init   — randomise clump positions/intensities
 *   5. precompute    — fill the lensing table from camera params
 *   6. timer.tick_ns, frame_ns, frame_time — seed FrameTimer
 *
 * ─────────────────────────────────────────────────────────────────────── */
struct Scene_ {
  /* — Concept-sized sub-structs (small scalars, grouped by mutation) — */
  Camera       camera;       /* what was the lensing table BUILT FROM.  *
                              * Mutating triggers need_rebuild = 1.      *
                              * See §6.1 Camera doc.                     */
  DiskSim      disk;         /* per-tick disk animation state +         *
                              * pause gate. See §6.1 DiskSim doc.        */
  RenderConfig render;       /* cosmetic flags — never trigger rebuild. *
                              * See §6.1 RenderConfig doc.               */
  Screen       screen;       /* current ncurses dimensions. See §6.1    *
                              * Screen doc.                              */
  FrameTimer   timer;        /* render + sim timebases + rolling fps.   *
                              * See §6.1 FrameTimer doc.                 */

  /* — Large data buffers (top-level, no wrapper) ———————————————————— *
   * Kept at top level so per-pixel indexing reads g_scene.table[r][c]
   * without an extra .cells / .items layer of noise.                    */
  Cell  table[MAX_ROWS][MAX_COLS];   /* precomputed ray outcomes —      *
                                      * populated by precompute(),       *
                                      * read each frame by render().     *
                                      * ~3 MB at MAX_ROWS×MAX_COLS.      */
  Clump clumps[N_CLUMPS];            /* orbital hot spots — populated   *
                                      * by clumps_init(), advanced each *
                                      * sim tick by clumps_tick().       */

  /* — Process / control flags ————————————————————————————————————— */
  int  color256;     /* 1 iff COLORS >= 256 at startup. Read by         *
                      * theme_apply() to pick the truecolour or 8-mode  *
                      * branch. Written ONCE in main(); read-only after.*/
  int  need_rebuild; /* deferred work — caller sets to 1 when Camera   *
                      * is mutated; main()'s next iteration runs       *
                      * precompute() to rebuild the lensing table.     */
  volatile sig_atomic_t running;     /* main-loop continue flag.       *
                                      * Cleared by 'q'/Q/ESC, SIGINT,  *
                                      * or SIGTERM (on_sigint).        *
                                      * volatile + sig_atomic_t for    *
                                      * async-signal safety.           */
  volatile sig_atomic_t need_resize; /* set by SIGWINCH handler        *
                                      * (on_sigwinch). Main loop reads *
                                      * + clears it at the top of each *
                                      * iteration, re-syncs Screen,    *
                                      * and rebuilds the table.        */
};

Scene g_scene = {
    .camera  = { .cam_dist = CAM_DIST_DEF, .tilt_deg = TILT_DEG_DEF },
    .disk    = { .spin     = SPIN_DEF,     .angle    = 0.0f,
                 .paused   = 0 },
    .render  = { .theme    = THEME_DEF,    .show_stars = 0 },
    .running = 1,
    /* screen, timer, table, clumps, color256, need_rebuild,
     * need_resize are BSS-zeroed; populated before first read.        */
};

static inline int scene_is_256_color(void) { return g_scene.color256; }

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
static float adaptive_geodesic_step_size(float r) {
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
static int fell_through_horizon(float r) { return r < BH_R * HORIZON_FUDGE; }

/*
 * escaped_to_far_field — has the photon left the gravitational well?
 *
 * Outside r = ESCAPE_R (130 r_s) the spacetime is nearly Minkowski;
 * the photon's direction is essentially frozen, so we stop the
 * integration and record the asymptotic direction.
 */
static int escaped_to_far_field(float r) { return r > ESCAPE_R; }

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
static int crossed_equatorial_plane(V3 prev, V3 pos) {
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
static V3 interpolate_disk_plane_hit(V3 prev, V3 pos) {
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
static Cell record_far_field_direction(V3 vel, float min_r) {
  V3 d = v3norm(vel);
  float th = acosf(fmaxf(-1.f, fminf(1.f, d.y)));
  float ph = atan2f(d.z, d.x);
  return (Cell){R_ESCAPED, 0, 0, th, ph, min_r};
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
static Cell ray_trace(V3 origin, V3 dir) {
  V3 pos = origin;
  V3 vel = dir; /* affine-parameter velocity, |vel| ≈ 1 */
  V3 prev = pos;
  float min_r = v3len(origin); /* tracks closest approach for the bloom */

  for (int step = 0; step < MAX_STEPS; step++) {
    float r = v3len(pos);
    if (r < min_r)
      min_r = r;

    if (fell_through_horizon(r))
      return (Cell){R_HORIZON, 0, 0, 0, 0, min_r};
    if (escaped_to_far_field(r))
      return record_far_field_direction(vel, min_r);

    float ds = adaptive_geodesic_step_size(r);
    prev = pos;
    geo_step(&pos, &vel, ds);

    if (crossed_equatorial_plane(prev, pos)) {
      V3 hit = interpolate_disk_plane_hit(prev, pos);
      float cr = sqrtf(hit.x * hit.x + hit.z * hit.z);
      if (cr >= DISK_IN && cr <= DISK_OUT) {
        float ph = atan2f(hit.z, hit.x);
        return (Cell){R_DISK, cr, ph, 0, 0, min_r};
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
static void build_camera_basis(float cam_dist, float tilt_rad, V3 *cam, V3 *fwd,
                               V3 *rgt, V3 *up) {
  *cam = (V3){0.f, cam_dist * sinf(tilt_rad), -cam_dist * cosf(tilt_rad)};
  *fwd = v3norm(v3scale(-1.f, *cam));
  V3 world_up = {0.f, 1.f, 0.f};
  *rgt = v3norm(v3cross(*fwd, world_up));
  *up = v3cross(*rgt, *fwd);
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
static V3 pinhole_pixel_to_ray(int col, int row, float cx, float cy, float hw,
                               V3 fwd, V3 rgt, V3 up) {
  float u = (col - cx) / cx * hw;
  float v = -(row - cy) / cx * hw / ASPECT;
  return v3norm(v3add(fwd, v3add(v3scale(u, rgt), v3scale(v, up))));
}

/*
 * draw_progress_header — central "Building lensing table…" message
 * shown once before the precompute loop.  Visual feedback so the
 * user knows the freeze is intentional (a full rebuild takes
 * 0.3–0.8 seconds on commodity hardware).
 */
static void draw_progress_header(int cols, int rows) {
  attron(A_BOLD);
  mvprintw(rows / 2, cols / 2 - 18, "  Building lensing table …          ");
  mvprintw(rows / 2 + 1, cols / 2 - 18, "  (exact Schwarzschild geodesics)   ");
  attroff(A_BOLD);
  wnoutrefresh(stdscr);
  doupdate();
}

/*
 * draw_progress_percent — periodic "[NN%]" update during the rebuild
 * loop.  Called every few rows so the user sees progress incrementing
 * rather than a frozen header.
 */
static void draw_progress_percent(int rows_done, int rows_total, int cols,
                                  int rows) {
  int pct = rows_done * 100 / rows_total;
  mvprintw(rows / 2 + 2, cols / 2 - 10, "  [%3d%%] ", pct);
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
static void precompute(int cols, int rows, float cam_dist, float tilt_deg) {
  float tilt_rad = tilt_deg * (float)M_PI / 180.0f;
  float hw = tanf((FOV_DEG * (float)M_PI / 180.0f) * 0.5f);
  float cx = cols * 0.5f;
  float cy = rows * 0.5f;

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

static float clumps_frand(void) { return (float)rand() / (float)RAND_MAX; }

/*
 * clumps_init — seed N_CLUMPS knots uniformly across the disk in radius
 * and angle, with mildly randomised intensities so they don't all read
 * as identical.  Called at startup and on the 'r' (reset) key.
 */
static void clumps_init(void) {
  for (int i = 0; i < N_CLUMPS; i++) {
    g_scene.clumps[i].r = DISK_IN + (DISK_OUT - DISK_IN) * clumps_frand();
    g_scene.clumps[i].phi = clumps_frand() * 2.0f * (float)M_PI;
    g_scene.clumps[i].intensity =
        0.22f + 0.20f * clumps_frand(); /* 0.22–0.42 */
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
static void clumps_tick(void) {
  for (int i = 0; i < N_CLUMPS; i++) {
    float r = g_scene.clumps[i].r;
    float omega = sqrtf(SCHWARZSCHILD_M / (r * r * r));  /* Keplerian Ω(r) */
    g_scene.clumps[i].phi += omega * CLUMP_RATE_SCALE;
    if (g_scene.clumps[i].phi >= (float)(2.0 * M_PI))
      g_scene.clumps[i].phi -= (float)(2.0 * M_PI);
  }
}

static float fclamp(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

/*
 * Star-field helpers — deterministic background-star sampling on the
 * celestial sphere, shared by sky_cell_hash / sky_quantise_direction /
 * classify_star_brightness / star_lookup below.
 *
 * Each R_ESCAPED cell records the direction (esc_th, esc_ph) the photon
 * escaped TO. In a backward-traced raytrace that's the direction the
 * photon came FROM at infinity — i.e. the celestial-sphere coordinate
 * for the FAR-FIELD background that the cell is looking at.
 *
 * We quantise (th, ph) onto a ~5800-cell sphere grid (STAR_GRID_PER_RAD
 * cells per radian → ~3° per cell) and hash each grid index with a
 * SplitMix-style mix. The hash determines:
 *   • density gate  (low 8 bits): ~1.6% of cells contain a star
 *   • brightness    (next 8 bits): split into *, +, . classes
 *
 * LENSING IS AUTOMATIC: rays passing near the photon sphere fan their
 * escape directions across a large angular swath, so a small patch of
 * celestial sphere — including stars directly BEHIND the BH — gets
 * magnified into the Einstein-ring zone just outside the shadow.
 */

/*
 * sky_cell_hash — SplitMix-style mix of the celestial-sphere grid
 * cell (q_th, q_ph). Stable hash → stars stay put in the same grid
 * cells across frames, so a star "lives" at a fixed direction on
 * the celestial sphere.
 */
static unsigned int sky_cell_hash(unsigned int q_th, unsigned int q_ph) {
  unsigned int h = q_th * 0x9E3779B9u + q_ph * 0xCC9E2D51u;
  h ^= h >> 16;
  h *= 0x85EBCA6Bu;
  h ^= h >> 13;
  h *= 0xC2B2AE35u;
  h ^= h >> 16;
  return h;
}

/* Quantise (θ, φ) onto the celestial-sphere grid (STAR_GRID_PER_RAD
 * cells per radian). Used as the hash input AND as the cell identity
 * a star is anchored to. */
static void sky_quantise_direction(float th, float ph,
                                   unsigned int *q_th, unsigned int *q_ph) {
  *q_th = (unsigned int)(th * STAR_GRID_PER_RAD);
  *q_ph = (unsigned int)((ph + (float)M_PI) * STAR_GRID_PER_RAD);
}

/* Brightness class from the hash's second byte. Stars come in three
 * visual sizes; the threshold split shapes the rarity ratio
 * (bright > mid > faint at roughly 14% : 47% : 39%). */
static void classify_star_brightness(unsigned int hash,
                                     char *glyph, attr_t *attr) {
  unsigned int b = (hash >> 8) & 0xFFu;
  if (b > STAR_BRIGHT_THRESH) { *glyph = '*'; *attr = A_BOLD;   }
  else if (b > STAR_MID_THRESH) { *glyph = '+'; *attr = A_NORMAL; }
  else                          { *glyph = '.'; *attr = A_NORMAL; }
}

static int star_lookup(float th, float ph, char *glyph, attr_t *attr) {
  /* (1) quantise (θ, φ) onto the celestial-sphere grid */
  unsigned int q_th, q_ph;
  sky_quantise_direction(th, ph, &q_th, &q_ph);

  /* (2) hash the grid cell to a stable random value */
  unsigned int h = sky_cell_hash(q_th, q_ph);

  /* (3) density gate — most cells return "empty" (no star here) */
  if ((h & 0xFFu) > STAR_DENSITY_BUCKET) return 0;

  /* (4) classify the star's brightness from the next hash byte */
  classify_star_brightness(h, glyph, attr);
  return 1;
}

/*
 * disk_char — 9-tier brightness → glyph ramp. Picks the densest
 * character whose threshold is below the input brightness, so the
 * disk reads as a continuous gradient even at low colour depth.
 *
 * Ramp order: '@' (densest) → '.' (sparsest), Ware (2020) perceptual
 * brightness ordering.
 */
static const struct { float thresh; char glyph; } DISK_GLYPH_RAMP[] = {
    {0.92f, '@'}, {0.82f, '#'}, {0.70f, '8'},
    {0.57f, '0'}, {0.45f, 'O'}, {0.33f, 'o'},
    {0.21f, '+'}, {0.12f, ':'},
};
#define DISK_GLYPH_TIERS  (int)(sizeof DISK_GLYPH_RAMP / sizeof DISK_GLYPH_RAMP[0])

static char disk_char(float b) {
  for (int i = 0; i < DISK_GLYPH_TIERS; i++)
    if (b > DISK_GLYPH_RAMP[i].thresh) return DISK_GLYPH_RAMP[i].glyph;
  return '.';
}

/*
 * disk_pair — 6-tier brightness → colour-pair + attribute. Mirrors the
 * Theme.{ring, hot, warm, mid, cool, dim} ramp; A_BOLD applied at the
 * top of each tier where the perceptual jump benefits from it.
 *
 * The ring/hot split uses an inner-disk A_BOLD boost (r_norm < 0.25
 * = inner quarter of the disk) so the innermost ring of hot material
 * reads brighter than the outer hot band.
 */
#define INNER_DISK_BOLD_FRAC  0.25f

static const struct { float thresh; int cp; } DISK_PAIR_RAMP[] = {
    {0.85f, CP_RING}, {0.67f, CP_HOT},  {0.50f, CP_WARM},
    {0.33f, CP_MID},  {0.17f, CP_COOL},
};
#define DISK_PAIR_TIERS  (int)(sizeof DISK_PAIR_RAMP / sizeof DISK_PAIR_RAMP[0])

static void disk_pair(float b, float r_norm, int *cp, attr_t *a) {
  for (int i = 0; i < DISK_PAIR_TIERS; i++) {
    if (b > DISK_PAIR_RAMP[i].thresh) {
      *cp = DISK_PAIR_RAMP[i].cp;
      *a  = (i == 0) ? A_BOLD
                     : (i == 1 && r_norm < INNER_DISK_BOLD_FRAC) ? A_BOLD
                                                                  : A_NORMAL;
      return;
    }
  }
  *cp = CP_DIM;
  *a  = A_NORMAL;  /* no A_DIM — would vanish on default background */
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
                                      float cos_tilt) {
  float v_orb = sqrtf(SCHWARZSCHILD_M / disk_r);
  float beta  = -v_orb * cosf(phi_rotating) * cos_tilt;
  beta = fclamp(beta, -DOPPLER_BETA_CLAMP, DOPPLER_BETA_CLAMP);
  return powf((1.f + beta) / (1.f - beta), DOPPLER_EXPONENT);
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
static float gravitational_redshift(float disk_r) {
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
static float disk_radial_temperature(float disk_r) {
  float r_n = fclamp((disk_r - DISK_IN) / (DISK_OUT - DISK_IN), 0.f, 1.f);
  float dr = disk_r - DISK_IN;
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
static float spiral_density_texture(float disk_phi, float disk_angle) {
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
static float accumulate_clump_bumps(float disk_r, float disk_phi) {
  float bump = 0.0f;
  for (int k = 0; k < N_CLUMPS; k++) {
    float dphi = disk_phi - g_scene.clumps[k].phi;
    if (dphi > (float)M_PI)
      dphi -= (float)(2.0 * M_PI);
    else if (dphi < -(float)M_PI)
      dphi += (float)(2.0 * M_PI);
    float dr = disk_r - g_scene.clumps[k].r;
    float a = dphi * dphi * 30.0f + dr * dr * 1.6f;
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
static void paint_disk_pixel(int row, int col, float brightness, float r_norm) {
  int cp;
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
static float photon_ring_bloom(float min_r) {
  float main = expf(-(min_r - PHOTON_R) * 1.4f);

  float rim = (min_r < 1.20f) ? 0.55f * expf(-(min_r - 1.0f) * 10.0f) : 0.0f;

  float e1 = min_r - (PHOTON_R + 0.06f);
  float e2 = min_r - (PHOTON_R + 0.02f);
  float echo =
      0.40f * expf(-e1 * e1 * 250.0f) + 0.30f * expf(-e2 * e2 * 700.0f);

  return fclamp(main + rim + echo, 0.0f, 1.0f);
}

/*
 * Photon-ring brightness → glyph + colour table. Each row gives the
 * minimum bloom intensity required to use that tier's appearance.
 *
 *   peak       '#' CP_RING A_BOLD   (mr right at photon sphere)
 *   bright     '*' CP_RING A_BOLD   (broad halo)
 *   mid        '+' CP_HOT  A_NORMAL (mid halo)
 *   fade       '.' CP_WARM A_NORMAL (outer fade — fallthrough)
 */
static const struct { float thresh; char ch; int cp; attr_t a; }
    RING_RAMP[] = {
        {0.85f, '#', CP_RING, A_BOLD  },
        {0.55f, '*', CP_RING, A_BOLD  },
        {0.30f, '+', CP_HOT,  A_NORMAL},
    };
#define RING_RAMP_TIERS  (int)(sizeof RING_RAMP / sizeof RING_RAMP[0])

static void paint_photon_ring_pixel(int row, int col, float brightness) {
  char   ch = '.';
  int    cp = CP_WARM;
  attr_t a  = A_NORMAL;
  for (int i = 0; i < RING_RAMP_TIERS; i++) {
    if (brightness > RING_RAMP[i].thresh) {
      ch = RING_RAMP[i].ch; cp = RING_RAMP[i].cp; a = RING_RAMP[i].a;
      break;
    }
  }
  attron (COLOR_PAIR(cp) | a);
  mvaddch(row, col, (chtype)(unsigned char)ch);
  attroff(COLOR_PAIR(cp) | a);
}

/*
 * paint_lensed_star — look up a star at the cell's escape direction
 * and draw it if present.  Only called for R_ESCAPED cells where the
 * photon-ring bloom didn't paint (so glyphs never collide).
 */
static void paint_lensed_star(int row, int col, float esc_th, float esc_ph) {
  char glyph;
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
                             int row, int col) {
  float phi_rot = c->disk_phi + disk_angle;
  float D = keplerian_doppler_factor(c->disk_r, phi_rot, cos_tilt);
  float g = gravitational_redshift(c->disk_r);
  float rad = disk_radial_temperature(c->disk_r);
  float tex = spiral_density_texture(c->disk_phi, disk_angle);
  float bright = fclamp(D * g * rad * tex, 0.f, 1.f);

  bright =
      fclamp(bright + accumulate_clump_bumps(c->disk_r, c->disk_phi), 0.f, 1.f);
  if (bright < DISK_MIN_VISIBLE)
    return; /* trim dim outer-disk scatter */

  float r_norm = fclamp((c->disk_r - DISK_IN) / (DISK_OUT - DISK_IN), 0.f, 1.f);
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
static void render_escaped_cell(const Cell *c, int row, int col,
                                int show_stars) {
  if (c->min_r < BLOOM_NEAR_RANGE) {
    float rb = photon_ring_bloom(c->min_r);
    if (rb > BLOOM_MIN_VISIBLE) {
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
                   float tilt_deg, int show_stars) {
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
  float clip_r2 = (cx * clip_frac) * (cx * clip_frac);

  for (int row = 0; row < rows_lim - 1; row++) {
    for (int col = 0; col < cols_lim; col++) {
      float sdx = (float)col - cx;
      float sdy = ((float)row - cy) / ASPECT;
      if (sdx * sdx + sdy * sdy > clip_r2)
        continue;

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

/* Signal handlers flip the volatile sig_atomic_t flags inside g_scene.
 * Async-signal-safe because the only access is to a sig_atomic_t scalar.*/
static void on_sigint(int s) {
  (void)s;
  g_scene.running = 0;
}
static void on_sigwinch(int s) {
  (void)s;
  g_scene.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void screen_init(Screen *s) {
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1);
  getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0  right (CP_HUD  bright yellow + bold)  — live status
 *   row -1 left  (CP_HINT bright cyan   + bold)  — actions / keys
 */
static void screen_hud(int cols, int rows, float fps, float cam_dist,
                       float tilt_deg, int theme, int paused) {
  /* Row 0 — right-aligned live status. */
  char top[180];
  snprintf(top, sizeof top, " dist:%.0f  tilt:%.0f  theme:%s  %s  %.0f fps ",
           (double)cam_dist, (double)tilt_deg, g_themes[theme % THEME_N].name,
           paused ? "PAUSED " : "running", (double)fps);
  int top_len = (int)strlen(top);
  int top_col = cols - top_len;
  if (top_col < 0)
    top_col = 0;
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

/* ── §10.1 setup + per-step main-loop helpers ───────────────────────────── */

/* One-shot setup before the loop: signals, ncurses, palette, clumps,
 * the first lensing table, and the FrameTimer seed. */
static void scene_setup(void) {
  srand((unsigned)time(NULL));
  atexit(cleanup);
  signal(SIGINT,   on_sigint);
  signal(SIGWINCH, on_sigwinch);

  /* Dependency order: screen → color caps → theme → clumps → lensing. */
  screen_init(&g_scene.screen);
  start_color();
  use_default_colors();
  g_scene.color256 = (COLORS >= 256);

  theme_apply(g_scene.render.theme);
  clumps_init();
  erase();
  precompute(g_scene.screen.cols, g_scene.screen.rows,
             g_scene.camera.cam_dist, g_scene.camera.tilt_deg);

  /* FrameTimer seed (other fields are BSS-zeroed). */
  g_scene.timer.tick_ns    = 1000000000LL / SIM_FPS;
  g_scene.timer.frame_ns   = 1000000000LL / RENDER_FPS;
  g_scene.timer.frame_time = clock_ns();
}

/* Resize OR camera-mutation rebuild — both go through the same code
 * path: re-read terminal size and rebuild the lensing table from
 * scratch (cheap, ~0.3-0.8 s, only triggered by user action). */
static void scene_handle_resize_or_rebuild(void) {
  g_scene.need_resize  = 0;
  g_scene.need_rebuild = 0;
  endwin();
  refresh();
  getmaxyx(stdscr, g_scene.screen.rows, g_scene.screen.cols);
  erase();
  precompute(g_scene.screen.cols, g_scene.screen.rows,
             g_scene.camera.cam_dist, g_scene.camera.tilt_deg);
  g_scene.timer.sim_accum  = 0;
  g_scene.timer.frame_time = clock_ns();
}

/* Wall-clock dt since the previous frame, capped to DT_CAP_NS so a
 * stalled process can't dump a giant catch-up burst into the sim. */
static long long frame_measure_dt(void) {
  long long now = clock_ns();
  long long dt  = now - g_scene.timer.frame_time;
  if (dt > DT_CAP_NS) dt = DT_CAP_NS;
  g_scene.timer.frame_time = now;
  return dt;
}

/* Fixed-timestep sim advance — accumulate dt and consume one full
 * sim tick whenever the accumulator crosses tick_ns. Decouples disk
 * pace from render fps. */
static void scene_advance_physics(long long dt) {
  if (g_scene.disk.paused) return;
  FrameTimer *tm = &g_scene.timer;
  tm->sim_accum += dt;
  while (tm->sim_accum >= tm->tick_ns) {
    g_scene.disk.angle += g_scene.disk.spin;
    if (g_scene.disk.angle >= (float)(2.0 * M_PI))
      g_scene.disk.angle -= (float)(2.0 * M_PI);
    clumps_tick();          /* Keplerian flow of hot spots */
    tm->sim_accum -= tm->tick_ns;
  }
}

/* Rolling-window fps update — refreshes fps_display ~2× per second. */
static void frame_tick_fps(long long dt) {
  FrameTimer *tm = &g_scene.timer;
  tm->fps_acc += dt;
  tm->fps_cnt++;
  if (tm->fps_acc >= FPS_WINDOW_NS) {
    tm->fps     = (float)tm->fps_cnt * 1e9f / (float)tm->fps_acc;
    tm->fps_acc = 0;
    tm->fps_cnt = 0;
  }
}

/* erase → render → HUD → present. Reads scene state, writes ncurses. */
static void scene_draw_one_frame(void) {
  erase();
  render(g_scene.disk.angle,
         g_scene.screen.cols,  g_scene.screen.rows,
         g_scene.camera.cam_dist, g_scene.camera.tilt_deg,
         g_scene.render.show_stars);
  screen_hud(g_scene.screen.cols, g_scene.screen.rows, g_scene.timer.fps,
             g_scene.camera.cam_dist, g_scene.camera.tilt_deg,
             g_scene.render.theme,    g_scene.disk.paused);
  wnoutrefresh(stdscr);
  doupdate();
}

/* Sleep the remainder of frame_ns so we hit RENDER_FPS regardless of
 * how fast the rest of the frame ran. frame_start_ns marks the start
 * of the work we want to budget. */
static void frame_cap_to_target_fps(long long frame_start_ns) {
  clock_sleep_ns(g_scene.timer.frame_ns - (clock_ns() - frame_start_ns));
}

/* ── §10.2 keyboard action handlers ─────────────────────────────────────── */

/* Spacebar / 'p' — pause/resume the simulation (renderer keeps running). */
static void key_pause_toggle(void) {
  g_scene.disk.paused = !g_scene.disk.paused;
}

/* 'r' — reset disk angle and reseed clump positions. */
static void key_reset_disk(void) {
  g_scene.disk.angle = 0.0f;
  clumps_init();
}

/* 't' — cycle theme and rebind CP_RING..CP_DIM. */
static void key_cycle_theme(void) {
  g_scene.render.theme = (g_scene.render.theme + 1) % THEME_N;
  theme_apply(g_scene.render.theme);
}

/* '+' / '-' — nudge camera distance (smaller dist = closer = bigger
 * Gargantua). Either change triggers a full lensing-table rebuild. */
static void key_camera_zoom(float delta) {
  g_scene.camera.cam_dist = fclamp(g_scene.camera.cam_dist + delta,
                                   CAM_DIST_MIN, CAM_DIST_MAX);
  g_scene.need_rebuild = 1;
}

/* 'a' / 'A' — nudge camera tilt (more = more face-on annular view).
 * Rebuilds lensing table. */
static void key_camera_tilt(float delta) {
  g_scene.camera.tilt_deg = fclamp(g_scene.camera.tilt_deg + delta,
                                   TILT_DEG_MIN, TILT_DEG_MAX);
  g_scene.need_rebuild = 1;
}

/* 'k' — toggle lensed background star field. */
static void key_toggle_stars(void) {
  g_scene.render.show_stars = !g_scene.render.show_stars;
}

/* Read one queued keystroke (non-blocking) and dispatch to its named
 * action. Each case is one helper call; the switch reads as the keymap. */
static void scene_handle_one_keystroke(void) {
  switch (getch()) {
  case 'q': case 'Q': case 27 /* ESC */: g_scene.running = 0;       break;
  case 'p': case 'P':                    key_pause_toggle();        break;
  case 'r': case 'R':                    key_reset_disk();          break;
  case 't': case 'T':                    key_cycle_theme();         break;
  case '+': case '=':                    key_camera_zoom(-CAM_DIST_STEP); break;
  case '-':                              key_camera_zoom(+CAM_DIST_STEP); break;
  case 'a':                              key_camera_tilt(+TILT_DEG_STEP); break;
  case 'A':                              key_camera_tilt(-TILT_DEG_STEP); break;
  case 'k': case 'K':                    key_toggle_stars();        break;
  default:                                                          break;
  }
}

/* ── §10.3 main ─────────────────────────────────────────────────────────── *
 *
 * Reads as the program's lifecycle:
 *
 *   SETUP:
 *     scene_setup() — signals, ncurses, palette, clumps, lensing table,
 *                     FrameTimer seed.
 *
 *   LOOP (each frame):
 *     1. handle pending resize or camera-rebuild
 *     2. measure wall-clock dt (capped)
 *     3. advance physics (fixed-timestep sim catches up via accumulator)
 *     4. tick rolling-fps window
 *     5. draw the scene
 *     6. dispatch one keystroke
 *     7. sleep to hit RENDER_FPS
 *
 * ─────────────────────────────────────────────────────────────────────── */
int main(void) {
  scene_setup();

  while (g_scene.running) {
    /* (1) deferred work — resize or rebuild lensing table */
    if (g_scene.need_resize || g_scene.need_rebuild)
      scene_handle_resize_or_rebuild();

    /* (2) frame timing */
    long long dt = frame_measure_dt();

    /* (3) advance physics (no-op if paused) */
    scene_advance_physics(dt);

    /* (4) rolling fps for HUD */
    frame_tick_fps(dt);

    /* (5) draw — render() + HUD + present */
    long long frame_start = clock_ns();
    scene_draw_one_frame();

    /* (6) handle one queued keystroke */
    scene_handle_one_keystroke();

    /* (7) sleep to target frame rate */
    frame_cap_to_target_fps(frame_start);
  }

  endwin();
  return 0;
}
