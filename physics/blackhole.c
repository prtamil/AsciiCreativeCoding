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
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/blackhole.c \
 *       -o blackhole -lncurses -lm
 *
 * Keys:
 *   q / ESC   quit          p   pause / resume
 *   r         reset spin    t   cycle theme (10 palettes)
 *   + / =     faster spin   -   slower spin
 *   i / k     tilt up/down  (rebuild lensing table)
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

/* camera — tilt is fixed; size is runtime-adjustable                      */
#define TILT_DEG      5.0f      /* fixed inclination above equatorial plane */
#define FOV_DEG      72.0f      /* horizontal field-of-view               */
#define CAM_DIST_DEF 38.0f      /* default camera distance (r_s units)    */
#define CAM_DIST_MIN 14.0f      /* closest  → largest Gargantua on screen */
#define CAM_DIST_MAX 72.0f      /* farthest → smallest                    */
#define CAM_DIST_STEP 3.0f      /* step per keypress                      */

/* disk rotation                                                            */
#define SPIN_DEF     0.04f      /* rad / sim-tick                         */

/* ray integration                                                          */
#define MAX_STEPS    900
#define ESCAPE_R     130.0f     /* fixed far-field boundary (r_s units)   */
#define DS_BASE      0.10f      /* affine-parameter step size             */

/* table dimensions                                                         */
#define MAX_COLS     512
#define MAX_ROWS     256

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

/* ── §3  color ──────────────────────────────────────────────────────────── */

enum {
    CP_RING  = 1,   /* photon ring / white-hot inner disk       */
    CP_HOT,         /* inner disk: yellow-white                 */
    CP_WARM,        /* mid-inner: orange-yellow                 */
    CP_MID,         /* mid: orange                              */
    CP_COOL,        /* outer: red                               */
    CP_DIM,         /* far outer: dark red                      */
    CP_STAR,        /* background stars                         */
    CP_HUD,         /* status line                              */
    CP_COUNT
};

static int g_256;

typedef struct { short fg[CP_COUNT]; } Theme;

static const Theme g_themes[] = {
    /* 0  interstellar — Kip Thorne warm gold / DNGR palette              */
    /*    ring→white   hot→lt-yel  warm→gold  mid→orng  cool→red-org  dim→tan  */
    { { 0, 255, 229, 214, 208, 172, 130, 253, 15 } },
    /* 1  matrix       — cyber green: lime core fading to moss            */
    /*    ring→lt-grn  hot→lime    warm→grn   mid→med-g  cool→drk-g   dim→olive */
    { { 0,  46, 120,  82,  40,  34,  64, 253, 15 } },
    /* 2  nova         — stellar detonation: white→ice-blue→violet        */
    /*    ring→white   hot→lt-blu  warm→blue  mid→b-vio  cool→violet  dim→purp  */
    { { 0, 231, 195, 153, 111, 105,  99, 253, 15 } },
    /* 3  ocean        — bioluminescent deep sea cyan                     */
    /*    ring→br-cyn  hot→lt-cyn  warm→teal  mid→d-teal cool→bl-tel  dim→navy  */
    { { 0,  51,  87,  45,  44,  37,  31, 253, 15 } },
    /* 4  poison       — acid chartreuse / toxic green-yellow             */
    /*    ring→br-yel  hot→yel-grn warm→lime  mid→grn    cool→med-g   dim→olive */
    { { 0, 226, 190, 154, 118,  82,  76, 253, 15 } },
    /* 5  fire         — white-hot core fading through orange to ember    */
    /*    ring→white   hot→br-yel  warm→yel   mid→orange cool→red-org dim→brown */
    { { 0, 231, 226, 220, 208, 166, 130, 253, 15 } },
    /* 6  plasma       — neon magenta / hot pink                          */
    /*    ring→white   hot→lt-pink warm→pink  mid→magnt  cool→purp    dim→d-purp */
    { { 0, 255, 219, 213, 177, 141, 105, 253, 15 } },
    /* 7  gold         — molten gold through amber to antique bronze      */
    /*    ring→br-yel  hot→yellow  warm→gold  mid→orange cool→d-gold  dim→bronze */
    { { 0, 226, 220, 214, 208, 178, 136, 253, 15 } },
    /* 8  arctic       — polar ice white into steel blue                  */
    /*    ring→white   hot→ice     warm→lt-bl mid→blue   cool→st-bl   dim→deep  */
    { { 0, 231, 195, 159, 123, 117, 111, 253, 15 } },
    /* 9  lava         — volcanic red bleeding into dark magenta-purple   */
    /*    ring→br-red  hot→red-org warm→red   mid→d-mag  cool→purple  dim→d-purp */
    { { 0, 196, 202, 160, 161, 125,  89, 253, 15 } },
    /* 10 mono         — white-hot centre through clean grey ramp         */
    /*    ring→white   hot→lt-gry  warm→gry   mid→med-g  cool→d-grey  dim→drk   */
    { { 0, 255, 252, 248, 244, 241, 238, 253, 15 } },
};
#define THEME_N  ((int)(sizeof g_themes / sizeof g_themes[0]))

static const short g_8fg[CP_COUNT] = {
    0,
    COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW,
    COLOR_RED,   COLOR_RED,    COLOR_RED,
    COLOR_WHITE, COLOR_WHITE
};

static void theme_apply(int idx)
{
    const Theme *t = &g_themes[idx % THEME_N];
    for (int i = 1; i < CP_COUNT; i++) {
        short fg = g_256 ? t->fg[i] : g_8fg[i];
        init_pair((short)i, fg, -1);
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

/* ── §6  ray table ───────────────────────────────────────────────────────── */

typedef enum { R_HORIZON = 0, R_DISK, R_ESCAPED } RayKind;

typedef struct {
    RayKind kind;
    float   disk_r;    /* disk radius at hit (R_DISK)                    */
    float   disk_phi;  /* disk azimuth at hit (R_DISK)                   */
    float   esc_th;    /* escape polar   angle θ ∈ [0,π]  (R_ESCAPED)   */
    float   esc_ph;    /* escape azimuth angle φ ∈ (-π,π] (R_ESCAPED)   */
    float   min_r;     /* closest approach to BH during integration      */
} Cell;

static Cell g_table[MAX_ROWS][MAX_COLS];

/*
 * Trace one backward null geodesic.
 *
 * The ray is shot BACKWARD from the camera (reversed time).  It intersects
 * the accretion disk (y = 0 plane, DISK_IN ≤ r ≤ DISK_OUT) if it crosses
 * that plane with the right radius.  Sign-change detection catches both the
 * primary image (near side of disk) and the secondary image (far side, the
 * thin arc above/below the shadow).
 */
static Cell ray_trace(V3 origin, V3 dir)
{
    V3    pos  = origin;
    V3    vel  = dir;           /* affine-parameter velocity, |vel| ≈ 1   */
    V3    prev = pos;
    float mr   = v3len(origin); /* track closest approach for photon ring */

    for (int step = 0; step < MAX_STEPS; step++) {
        float r = v3len(pos);
        if (r < mr) mr = r;

        /* ── fell into horizon ── */
        if (r < BH_R * 0.92f) {
            return (Cell){ R_HORIZON, 0,0,0,0, mr };
        }

        /* ── escaped to far field ── */
        if (r > ESCAPE_R) {
            V3 d = v3norm(vel);
            float th = acosf(fmaxf(-1.f, fminf(1.f, d.y)));
            float ph = atan2f(d.z, d.x);
            return (Cell){ R_ESCAPED, 0,0, th, ph, mr };
        }

        /* ── adaptive step: finer near BH, coarser far away ── */
        float ds = fmaxf(0.003f, fminf(DS_BASE, r * 0.05f));

        prev = pos;
        geo_step(&pos, &vel, ds);

        /* ── equatorial-plane crossing detection ──
         *
         * Detects sign change in y (equatorial plane y = 0).
         * Works for BOTH primary image (near-disk ray crossing from above)
         * AND secondary image (ray that loops around and hits back disk).
         */
        if (prev.y * pos.y < 0.0f) {
            /* linear interpolate exact crossing position */
            float t   = fabsf(prev.y) / (fabsf(prev.y) + fabsf(pos.y));
            V3    hit = v3add(v3scale(1.f-t, prev), v3scale(t, pos));
            float cr  = sqrtf(hit.x*hit.x + hit.z*hit.z);
            if (cr >= DISK_IN && cr <= DISK_OUT) {
                float ph = atan2f(hit.z, hit.x);
                return (Cell){ R_DISK, cr, ph, 0,0, mr };
            }
        }
    }

    /* max steps: treat as escaped */
    V3 d = v3norm(vel);
    return (Cell){ R_ESCAPED, 0,0,
        acosf(fmaxf(-1.f, fminf(1.f, d.y))),
        atan2f(d.z, d.x), mr };
}

/* ── §7  precompute lensing table ────────────────────────────────────────── */

static void precompute(int cols, int rows, float cam_dist)
{
    float tilt  = TILT_DEG * (float)M_PI / 180.0f;
    float fov_h = (FOV_DEG * (float)M_PI / 180.0f) * 0.5f;

    /* camera basis */
    V3 cam = { 0.f, cam_dist * sinf(tilt), -cam_dist * cosf(tilt) };
    V3 fwd = v3norm(v3scale(-1.f, cam));   /* forward  = toward origin     */
    V3 wup = { 0.f, 1.f, 0.f };
    V3 rgt = v3norm(v3cross(fwd, wup));    /* right                        */
    V3 up  = v3cross(rgt, fwd);            /* up  (camera-space)           */

    /* half-tangent of FOV, using cols as reference for both axes so pixels
     * are square in isotropic space; ASPECT corrects the cell shape.      */
    float hw = tanf(fov_h);
    float cx = cols * 0.5f;
    float cy = rows * 0.5f;

    /* progress message */
    int rows_lim = rows < MAX_ROWS ? rows : MAX_ROWS;
    int cols_lim = cols < MAX_COLS ? cols : MAX_COLS;

    attron(A_BOLD);
    mvprintw(rows/2,   cols/2-18, "  Building lensing table …          ");
    mvprintw(rows/2+1, cols/2-18, "  (exact Schwarzschild geodesics)   ");
    attroff(A_BOLD);
    wnoutrefresh(stdscr);
    doupdate();

    for (int row = 0; row < rows_lim; row++) {
        for (int col = 0; col < cols_lim; col++) {
            /* screen UV in [-1,1] × [-ar,ar] */
            float u =  (col - cx) / cx * hw;
            float v = -(row - cy) / cx * hw / ASPECT;  /* -: row↓ = scene↑ */

            V3 dir = v3norm(
                v3add(fwd, v3add(v3scale(u, rgt), v3scale(v, up))));

            g_table[row][col] = ray_trace(cam, dir);
        }

        /* simple text progress bar */
        if (row % 6 == 0) {
            int pct = row * 100 / rows_lim;
            mvprintw(rows/2+2, cols/2-10, "  [%3d%%] ", pct);
            wnoutrefresh(stdscr);
            doupdate();
        }
    }
}

/* ── §8  frame render ────────────────────────────────────────────────────── */

static float fclamp(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
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
 * Render one animation frame.
 *
 * disk_angle: cumulative rotation of the accretion disk (rad).
 * Tilt is fixed at TILT_DEG = 5°.
 */
static void render(float disk_angle, int cols, int rows, float cam_dist)
{
    /* Keplerian orbital speed at disk_r (r_s=1, M=0.5):
     * v_orb = √(M/r) = √(1/(2r))
     * Observer at cam direction ≈ (0, sin_tilt, -cos_tilt).
     * Disk velocity at phi: v*(-sin φ, 0, cos φ).
     * Radial velocity toward observer:
     *   β = v_orb · dot( (-sinφ,0,cosφ), (0, sin_tilt, -cos_tilt) )
     *     = v_orb · (-cos φ · cos_tilt)
     * Positive β → approaching → brighter.
     */
    float cos_tilt = cosf(TILT_DEG * (float)M_PI / 180.0f);

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
    float clip_r    = cx * clip_frac;
    float clip_r2   = clip_r * clip_r;

    for (int row = 0; row < rows_lim - 1; row++) {
        for (int col = 0; col < cols_lim; col++) {
            /* isotropic distance from screen centre — same space the camera
             * uses, so the clip matches the rendered circle exactly.       */
            float sdx = (float)col - cx;
            float sdy = ((float)row - cy) / ASPECT;
            if (sdx*sdx + sdy*sdy > clip_r2) continue;   /* outside clip  */

            const Cell *c = &g_table[row][col];

            switch (c->kind) {

            case R_HORIZON:
                /* absolute black – leave from erase()                   */
                break;

            case R_DISK: {
                /* phi in the disk frame, advanced by disk rotation       */
                float phi = c->disk_phi + disk_angle;
                float r_n = fclamp((c->disk_r - DISK_IN)/(DISK_OUT-DISK_IN),
                                   0.f, 1.f);

                /* Keplerian speed and radial Doppler component           */
                float v_orb = sqrtf(0.5f / c->disk_r);
                float beta  = -v_orb * cosf(phi) * cos_tilt;
                beta = fclamp(beta, -0.95f, 0.95f);

                /* relativistic beaming  D = [(1+β)/(1−β)]^(3/2)        */
                float D = powf((1.f+beta)/(1.f-beta), 1.5f);

                /* gravitational redshift:  g = √(1 − r_s/r) = √(1 − 1/r) */
                float g = sqrtf(fmaxf(0.01f, 1.f - 1.f/c->disk_r));

                /* radial temperature: ISCO spike + power-law falloff    */
                float dr    = c->disk_r - DISK_IN;
                float isco  = expf(-dr * dr * 0.65f);
                float rad   = powf(1.f - 0.86f*r_n, 2.2f) + 0.65f*isco;

                /* spiral density texture (rotates with disk)            */
                float tex = 1.f + 0.18f*sinf(c->disk_phi*5.f - disk_angle*4.f);

                float bright = fclamp(D * g * rad * tex, 0.f, 1.f);
                if (bright < 0.07f) break;   /* trim dim outer-disk scatter */

                int cp; attr_t a;
                disk_pair(bright, r_n, &cp, &a);
                attron(COLOR_PAIR(cp) | a);
                mvaddch(row, col, disk_char(bright));
                attroff(COLOR_PAIR(cp) | a);
                break;
            }

            case R_ESCAPED: {
                /*
                 * Photon ring: escaped rays that grazed the photon sphere
                 * (r ≈ 1.5 r_s) are bent hard and cluster into a bright arc
                 * just outside the shadow — the characteristic Gargantua ring.
                 * Brightness falls off exponentially with closest approach.
                 */
                float mr = c->min_r;
                if (mr < 3.2f) {
                    float rb = expf(-(mr - PHOTON_R) * 2.4f);
                    if (rb > 0.09f) {
                        rb = fclamp(rb, 0.f, 1.f);
                        int   cp = rb > 0.55f ? CP_RING : CP_HOT;
                        attr_t a = rb > 0.55f ? A_BOLD  : A_NORMAL;
                        char   ch = rb > 0.65f ? '*' : rb > 0.35f ? '+' : '.';
                        attron(COLOR_PAIR(cp) | a);
                        mvaddch(row, col, ch);
                        attroff(COLOR_PAIR(cp) | a);
                    }
                }
                break;
            }

            }   /* switch */
        }
    }
}

/* ── §9  screen / HUD ───────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run    = 1;
static volatile sig_atomic_t g_resize = 0;

static void on_sigint  (int s){ (void)s; g_run    = 0; }
static void on_sigwinch(int s){ (void)s; g_resize = 1; }

static void screen_init(int *cols, int *rows)
{
    initscr();
    cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    getmaxyx(stdscr, *rows, *cols);
}

static void screen_hud(int cols, int rows,
                        float fps, float cam_dist, int theme)
{
    (void)cols;
    static const char *const names[] = {
        "interstellar","matrix","nova","ocean","poison",
        "fire","plasma","gold","arctic","lava","mono"
    };
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows-1, 1,
        " fps:%.0f  dist:%.0f  theme:%s"
        "   [+]closer  [-]farther  [t]theme  [p]pause  [q]quit ",
        (double)fps, (double)cam_dist,
        names[theme % THEME_N]);
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §10  main ───────────────────────────────────────────────────────────── */

int main(void)
{
    srand((unsigned)time(NULL));
    signal(SIGINT,   on_sigint);
    signal(SIGWINCH, on_sigwinch);

    int cols, rows;
    screen_init(&cols, &rows);
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);

    int   theme    = 0;
    float cam_dist = CAM_DIST_DEF;
    float spin     = SPIN_DEF;
    float disk_ang = 0.f;

    theme_apply(theme);
    erase();
    precompute(cols, rows, cam_dist);

    long long tick_ns    = 1000000000LL / SIM_FPS;
    long long frame_ns   = 1000000000LL / RENDER_FPS;
    long long sim_accum  = 0;
    long long frame_time = clock_ns();
    long long fps_acc    = 0;
    int       fps_cnt    = 0;
    float     fps        = 0.f;
    int       paused     = 0;
    int       need_rebuild = 0;

    while (g_run) {

        /* ── resize ── */
        if (g_resize || need_rebuild) {
            g_resize     = 0;
            need_rebuild = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            erase();
            precompute(cols, rows, cam_dist);
            sim_accum  = 0;
            frame_time = clock_ns();
        }

        /* ── dt ── */
        long long now = clock_ns();
        long long dt  = now - frame_time;
        if (dt > 100000000LL) dt = 100000000LL;
        frame_time = now;

        /* ── physics ── */
        if (!paused) {
            sim_accum += dt;
            while (sim_accum >= tick_ns) {
                disk_ang += spin;
                if (disk_ang >= (float)(2.0*M_PI))
                    disk_ang -= (float)(2.0*M_PI);
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
        render(disk_ang, cols, rows, cam_dist);
        screen_hud(cols, rows, fps, cam_dist, theme);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── input ── */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_run = 0;             break;
        case 'p': case 'P': paused = !paused;               break;
        case 'r': case 'R': disk_ang = 0.f;                 break;
        case 't': case 'T':
            theme = (theme+1) % THEME_N;
            theme_apply(theme);
            break;
        case '+': case '=':
            /* closer camera → bigger Gargantua on screen */
            cam_dist = fclamp(cam_dist - CAM_DIST_STEP, CAM_DIST_MIN, CAM_DIST_MAX);
            need_rebuild = 1;
            break;
        case '-':
            /* farther camera → smaller Gargantua on screen */
            cam_dist = fclamp(cam_dist + CAM_DIST_STEP, CAM_DIST_MIN, CAM_DIST_MAX);
            need_rebuild = 1;
            break;
        }

        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }

    endwin();
    return 0;
}
