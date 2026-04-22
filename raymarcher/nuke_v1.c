/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nuke_v1.c — High-fidelity Nuclear Detonation & Mushroom Cloud
 *
 * A photoreal-leaning mushroom cloud rendered in the terminal via volumetric
 * raymarching. The simulation runs at internal HIGH resolution (2× vertical
 * supersampling) and is downsampled to character cells using top/bottom-
 * weighted glyphs that recover sub-cell vertical detail.
 *
 * Key improvements over nuke_basic.c:
 *
 *   1. SUPERSAMPLING (SS_Y=2). Two rays per terminal row — terminal cells
 *      are ~2:1 tall, so vertical detail is the limiting factor. Top and
 *      bottom samples drive a glyph picker:
 *        both strong  → '#' '@' '%' (full)
 *        top  only    → '"' '`' '^' (top-weighted)
 *        bottom only  → '.' ',' '_' (bottom-weighted)
 *        both medium  → ':' ';' '|' (middle)
 *
 *   2. FULL MUSHROOM ANATOMY. The mushroom is a sum of Gaussian-density
 *      primitives — no SDF surfaces, all soft volumes:
 *        - fireball   : hot spheroid (rises during RISING)
 *        - stem       : tapered cylinder, wider at the top
 *        - cap        : oblate spheroid (the dome)
 *        - vortex     : torus rolling outward (the bulging cap edge)
 *        - skirt      : thin condensation collar at base of cap
 *        - pyrocumulus: 4 rotating billows on top of the cap
 *
 *   3. SEPARATE EMISSION & EXTINCTION. Density attenuates light (smoke is
 *      gray); a separate "heat" field emits orange-yellow-white. So a
 *      thick cold smoke plume reads as gray and the hot core glows through.
 *
 *   4. CHEAP SELF-SHADOWING. A Y-position diffuse term darkens the cap
 *      underside and brightens the top, simulating sun-from-above without
 *      paying for shadow rays.
 *
 *   5. HEAVY CULLING. Per-shape bounding spheres (not just per-bomb) keep
 *      the cost manageable with the richer geometry.
 *
 * Keys:   q/ESC quit  |  space pause  |  r restart  |  +/- speed
 *         n detonate again  |  t cycle theme  |  s toggle smoke  |  l toggle lighting
 * Themes: REALISTIC, MATRIX, OCEAN, NOVA, TOXIC  (cycle with 't')
 * Build:  gcc -std=c11 -O2 -Wall -Wextra raymarcher/nuke_v1.c -o nuke_v1 \
 *             -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config      — all tunable constants
 *   §2  clock       — monotonic timer + sleep
 *   §3  color       — 32-entry smoke + fire palette
 *   §4  vec3 math   — 3D vector operations
 *   §5  noise/fBm   — value noise, fBm, domain warp
 *   §6  density     — full mushroom anatomy, density + heat fields
 *   §7  particles   — debris + embers
 *   §8  simulation  — phase state machine (FLASH..PLATEAU)
 *   §9  raymarcher  — Beer-Lambert integrator with 2× vertical supersampling
 *  §10  canvas      — sub-pixel pair buffer + glyph picker
 *  §11  renderer    — bridges simulation → screen, draws HUD
 *  §12  screen      — ncurses terminal setup
 *  §13  app         — main loop
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config — tunable constants                                         */
/* ===================================================================== */

enum {
    SIM_FPS        = 24,
    HUD_COLS       = 96,
    FPS_MS         = 500,
    MAX_PARTICLES  = 240,
};

/* ── Supersampling ─────────────────────────────────────────────────────
 * SS_Y=2 fires 2 rays per terminal row (top half + bottom half of each
 * cell). The two samples then drive top-weighted, bottom-weighted, or
 * full glyphs. SS_X=1 because horizontal cell resolution already matches
 * world units after CELL_ASPECT correction.
 */
#define SS_X            1
#define SS_Y            2

/* ── Raymarcher (volumetric) ─────────────────────────────────────────── *
 * Tuned for ~20+ fps at typical terminal sizes. Key wins vs the original:
 *   - cloud_density() has an early bounding-box check that skips the
 *     expensive noise + Gaussian work entirely for empty-space samples
 *     (about 60% of step samples on a typical ray).
 *   - RM_MAX_STEPS lowered from 52 to 40 with a slightly larger RM_STEP;
 *     supersampling hides the lower step density.
 *   - TRANSMIT_BAIL relaxed from 0.04 → 0.07; once a ray is 93% opaque
 *     the remaining contribution is invisible.
 */
#define RM_MAX_STEPS   40     /* max integration steps per ray             */
#define RM_STEP        0.18f  /* fixed step size (world units)             */
#define RM_MAX_DIST    18.0f  /* ray bail-out distance                     */
#define EXTINCTION     3.6f   /* smoke extinction coefficient              */
#define DENSITY_THRESH 0.012f /* skip below this                           */
#define CULL_R_BOMB    3.6f   /* per-bomb bounding sphere — tighter        */
#define TRANSMIT_BAIL  0.07f  /* stop marching once ray is mostly opaque   */

/* ── Camera ─────────────────────────────────────────────────────────── */
#define CAM_Z          6.8f
#define FOV_HALF_TAN   0.70f
#define CELL_ASPECT    2.0f   /* terminal cell h/w; corrects oval pixels   */

/* ── Blast geometry ─────────────────────────────────────────────────── *
 * The cloud spans roughly y ∈ [BLAST_Y, BLAST_Y + STEM_HEIGHT_MAX + CAP_R_V].
 * The camera sees roughly y ∈ [-pa·CAM_Z·FOV, +pa·CAM_Z·FOV] at z=0,
 * about ±2.6 for typical terminals. We size the cloud so the cap top sits
 * comfortably below +2.0 — earlier values had the cap + pyrocumulus
 * billows poke above the visible frame, looking like a vertical "beam".
 */
#define BLAST_Y       (-2.4f)

#define FIREBALL_R_MAX  1.05f
#define STEM_HEIGHT_MAX 2.55f  /* shorter stem so cap top stays in frame   */
#define STEM_R_BASE     0.20f
#define STEM_R_TOP      0.55f  /* stem widens significantly at the top    */
#define CAP_R_H         1.60f  /* cap horizontal radius                   */
#define CAP_R_V         0.58f  /* cap vertical radius (oblate)            */
#define VORTEX_R_MAJ    1.15f  /* torus major radius — outward roll       */
#define VORTEX_R_MIN    0.40f  /* torus tube radius                       */
#define SKIRT_R_MAJ     1.02f  /* thin condensation collar major          */
#define SKIRT_R_MIN     0.10f  /* skirt tube radius                       */
#define PYRO_COUNT      4      /* pyrocumulus billows on top of cap       */
#define PYRO_OFFSET_R   0.52f  /* horizontal offset of each billow        */
#define PYRO_OFFSET_Y   0.20f  /* vertical offset above cap centre (low!) */
#define PYRO_R          0.38f  /* billow radius                           */

/* ── Surface noise ──────────────────────────────────────────────────── */
#define DISP_AMP        0.55f  /* fBm displacement amplitude              */
#define DISP_FREQ       1.40f  /* base noise frequency                    */
#define DISP_SPD        0.18f  /* surface evolution speed                 */
#define WARP_AMP        0.50f  /* domain warp strength                    */
#define WARP_FREQ       0.78f  /* warp spatial frequency                  */
#define RISE_BIAS       0.28f  /* upward noise advection (rising gas)     */

/* ── Density Gaussian widths ────────────────────────────────────────── */
#define FIRE_SPREAD    0.42f
#define STEM_SPREAD    1.10f
#define CAP_SPREAD     1.05f
#define VORTEX_SPREAD  0.85f
#define SKIRT_SPREAD   1.15f
#define PYRO_SPREAD    0.80f

/* ── Lighting ───────────────────────────────────────────────────────── *
 * Cheap single-axis self-shadow:
 *   underside of cap (high Y, but inside cap shadow) is darkened
 *   top + right are brighter (sun angle)
 */
#define LIGHT_AMBIENT     0.35f
#define LIGHT_DIFFUSE     0.65f
#define SUN_DIR_X         0.45f
#define SUN_DIR_Y         0.78f
#define SUN_DIR_Z         0.42f

/* ── Shockwave ──────────────────────────────────────────────────────── */
#define SHOCKWAVE_SPD  3.6f

/* ── Particles ──────────────────────────────────────────────────────── */
#define DEBRIS_N       50
#define EMBER_N        12
#define PART_GRAVITY  (-3.2f)
#define DEBRIS_SPD     3.2f
#define EMBER_SPD      0.55f

/* ── Continuous-time keypoints (seconds since detonation) ──────────────
 *
 * No phase-machine cliffs. Every geometry parameter is a smoothstep
 * between two keypoints; phases overlap on purpose so the fireball is
 * still rising while the cap is starting to bulge out, etc.
 *
 *    t=0.00   detonation
 *    t=0.05   fireball begins growing                ── FLASH overlaps
 *    t=0.35   flash fades out
 *    t=0.80   fireball reaches max radius           ── FIREBALL phase
 *    t=0.90   fireball begins to rise (stem forms)  ── RISING phase
 *    t=2.40   cap begins to form (overlaps RISING!) ── CAP_FORM phase
 *    t=2.80   vortex roll begins
 *    t=3.20   fireball reaches top (stem fully extended)
 *    t=5.40   cap fully formed                      ── MUSHROOM phase
 *    t=5.80   vortex fully extended
 *    t=2.00   fireball begins shrinking (cools, feeds cap)
 *    t=6.00   fireball at minimum size
 *    t=9.40+  PLATEAU — gentle breathing pulse only
 *    t=35.0   auto-deactivate
 */
#define T_FLASH_END     0.35f
#define T_FB_GROW_BEG   0.05f
#define T_FB_GROW_END   0.80f
#define T_RISE_BEG      0.90f
#define T_RISE_END      3.20f
#define T_CAP_BEG       2.40f
#define T_CAP_END       5.40f
#define T_VORTEX_BEG    2.80f
#define T_VORTEX_END    5.80f
#define T_FB_SHRINK_BEG 2.00f
#define T_FB_SHRINK_END 6.00f
#define T_MUSHROOM_BEG  5.40f
#define T_PLATEAU_BEG   9.40f
#define T_PLATEAU_END   35.0f

/* ── Timing ─────────────────────────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color — themable palette                                           */
/*                                                                        */
/* Each Theme defines:                                                    */
/*   smoke[N_SMOKE]  — 8-step gradient driven by lit-smoke brightness    */
/*   fire [N_FIRE ]  — 16-step gradient driven by emission heat          */
/*   flash/shock/debris/ember/corona — accent colors                     */
/*                                                                        */
/* Cycle with the 't' key. color_init() is re-run on theme change to     */
/* rewrite the curses color pairs in place.                              */
/* ===================================================================== */

#define N_SMOKE   8
#define N_FIRE    16
#define CP_SMOKE_BASE 1
#define CP_FIRE_BASE  (CP_SMOKE_BASE + N_SMOKE)
#define CP_FLASH      25
#define CP_SHOCK      26
#define CP_DEBRIS     27
#define CP_EMBER      28
#define CP_CORONA     29
#define CP_HUD        30

typedef struct {
    const char *name;
    short smoke[N_SMOKE];
    short fire [N_FIRE ];
    short flash, shock, debris, ember, corona;
} Theme;

#define N_THEMES 5

static const Theme g_themes[N_THEMES] = {
    /* REALISTIC — gray smoke + classic orange/yellow/white fire core */
    { .name  = "REALISTIC",
      .smoke = {234, 236, 239, 242, 245, 248, 251, 254},
      .fire  = { 52,  88, 124, 160, 196, 202, 208, 214,
                220, 221, 222, 226, 228, 229, 230, 231},
      .flash = 231, .shock = 123, .debris = 214, .ember = 208, .corona = 220 },

    /* MATRIX — dark green → bright green smoke; green-white fire */
    { .name  = "MATRIX",
      .smoke = { 22,  28,  34,  40,  46,  82, 118, 154},
      .fire  = { 22,  28,  34,  40,  46,  82, 118, 154,
                190, 191, 192, 226, 228, 229, 230, 231},
      .flash = 231, .shock =  46, .debris = 154, .ember =  82, .corona = 154 },

    /* OCEAN — deep blue smoke; cyan/white "underwater" core */
    { .name  = "OCEAN",
      .smoke = { 17,  18,  19,  24,  31,  38,  45,  51},
      .fire  = { 17,  18,  19,  24,  31,  38,  45,  51,
                 87, 117, 123, 159, 195, 219, 230, 231},
      .flash = 231, .shock =  51, .debris =  51, .ember =  39, .corona = 195 },

    /* NOVA — deep purple → magenta → pink → white (stellar explosion) */
    { .name  = "NOVA",
      .smoke = { 53,  54,  55,  89,  90, 125, 161, 197},
      .fire  = { 53,  89,  90, 125, 161, 197, 198, 199,
                200, 207, 213, 219, 225, 226, 230, 231},
      .flash = 231, .shock = 219, .debris = 213, .ember = 199, .corona = 226 },

    /* TOXIC — sickly green → mustard yellow → bright yellow (radioactive) */
    { .name  = "TOXIC",
      .smoke = { 22,  28,  64, 100, 142, 178, 184, 220},
      .fire  = { 22,  28,  64, 100, 142, 178, 184, 220,
                221, 222, 154, 191, 192, 226, 228, 231},
      .flash = 231, .shock = 154, .debris = 226, .ember = 154, .corona = 226 },
};

static int g_theme_idx = 0;

static void color_init(void)
{
    start_color();
    use_default_colors();
    const Theme *th = &g_themes[g_theme_idx];
    if (COLORS >= 256) {
        for (int i = 0; i < N_SMOKE; i++)
            init_pair((short)(CP_SMOKE_BASE + i), th->smoke[i], -1);
        for (int i = 0; i < N_FIRE; i++)
            init_pair((short)(CP_FIRE_BASE + i), th->fire[i], -1);
        init_pair(CP_FLASH,  th->flash,  -1);
        init_pair(CP_SHOCK,  th->shock,  -1);
        init_pair(CP_DEBRIS, th->debris, -1);
        init_pair(CP_EMBER,  th->ember,  -1);
        init_pair(CP_CORONA, th->corona, -1);
        init_pair(CP_HUD,    231, -1);
    } else {
        /* 8-color fallback: only realistic theme makes sense, others map
         * loosely to nearest ANSI primary. */
        short fb_smoke = COLOR_WHITE, fb_fire = COLOR_RED;
        switch (g_theme_idx) {
            case 1: fb_smoke = COLOR_GREEN;   fb_fire = COLOR_GREEN;   break;
            case 2: fb_smoke = COLOR_BLUE;    fb_fire = COLOR_CYAN;    break;
            case 3: fb_smoke = COLOR_MAGENTA; fb_fire = COLOR_MAGENTA; break;
            case 4: fb_smoke = COLOR_GREEN;   fb_fire = COLOR_YELLOW;  break;
        }
        for (int i = 0; i < N_SMOKE; i++)
            init_pair((short)(CP_SMOKE_BASE + i), fb_smoke, -1);
        for (int i = 0; i < N_FIRE;  i++)
            init_pair((short)(CP_FIRE_BASE + i), fb_fire, -1);
        init_pair(CP_FLASH,  COLOR_WHITE,  -1);
        init_pair(CP_SHOCK,  COLOR_CYAN,   -1);
        init_pair(CP_DEBRIS, COLOR_YELLOW, -1);
        init_pair(CP_EMBER,  fb_fire,      -1);
        init_pair(CP_CORONA, COLOR_YELLOW, -1);
        init_pair(CP_HUD,    COLOR_WHITE,  -1);
    }
}

static void cycle_theme(void)
{
    g_theme_idx = (g_theme_idx + 1) % N_THEMES;
    color_init();
}

/* heat ∈ [0,1] picks a fire palette entry; brightness ∈ [0,1] picks the
 * smoke palette entry. The renderer chooses which palette based on heat. */
static attr_t fire_pair(float heat)
{
    if (heat < 0.f) heat = 0.f;
    if (heat > 1.f) heat = 1.f;
    int i = (int)(heat * (float)(N_FIRE - 1));
    if (i >= N_FIRE) i = N_FIRE - 1;
    attr_t a = COLOR_PAIR(CP_FIRE_BASE + i);
    if (heat > 0.55f) a |= A_BOLD;
    return a;
}

static attr_t smoke_pair(float bright)
{
    if (bright < 0.f) bright = 0.f;
    if (bright > 1.f) bright = 1.f;
    int i = (int)(bright * (float)(N_SMOKE - 1));
    if (i >= N_SMOKE) i = N_SMOKE - 1;
    attr_t a = COLOR_PAIR(CP_SMOKE_BASE + i);
    if (bright > 0.75f) a |= A_BOLD;
    return a;
}

/* ===================================================================== */
/* §4  vec3 math                                                          */
/* ===================================================================== */

typedef struct { float x, y, z; } V3;

static inline V3    v3(float x, float y, float z)  { return (V3){x, y, z}; }
static inline V3    v3add(V3 a, V3 b)              { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3sub(V3 a, V3 b)              { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3mul(V3 a, float s)           { return v3(a.x*s, a.y*s, a.z*s); }
static inline float v3dot(V3 a, V3 b)              { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3len(V3 a)                    { return sqrtf(v3dot(a, a)); }
static inline V3    v3norm(V3 a)
{
    float l = v3len(a); return l > 1e-7f ? v3mul(a, 1.f / l) : v3(0, 0, 1);
}

static inline float clmpf(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }
static inline float lerpf(float a, float b, float t)   { return a + (b - a) * t; }

static inline float smoothstep(float e0, float e1, float x)
{
    float t = clmpf((x - e0) / (e1 - e0), 0.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

/* ===================================================================== */
/* §5  hash + value noise + fBm + domain warp                            */
/* ===================================================================== */

static inline float hash1(int ix, int iy, int iz)
{
    unsigned u = (unsigned)(ix*127 + iy*311 + iz*743 + ix*iy*17 + iy*iz*29);
    u = (u ^ (u >> 16)) * 0x45d9f3bU;
    u = (u ^ (u >> 16)) * 0x45d9f3bU;
    u = u ^ (u >> 16);
    return (float)(u & 0xFFFFFF) / (float)0x1000000;
}

static float noise3(V3 p)
{
    int ix=(int)floorf(p.x), iy=(int)floorf(p.y), iz=(int)floorf(p.z);
    float fx=p.x-floorf(p.x), fy=p.y-floorf(p.y), fz=p.z-floorf(p.z);
    float ux=fx*fx*fx*(fx*(fx*6.f-15.f)+10.f);
    float uy=fy*fy*fy*(fy*(fy*6.f-15.f)+10.f);
    float uz=fz*fz*fz*(fz*(fz*6.f-15.f)+10.f);
    float v000=hash1(ix,iy,iz),     v100=hash1(ix+1,iy,iz);
    float v010=hash1(ix,iy+1,iz),   v110=hash1(ix+1,iy+1,iz);
    float v001=hash1(ix,iy,iz+1),   v101=hash1(ix+1,iy,iz+1);
    float v011=hash1(ix,iy+1,iz+1), v111=hash1(ix+1,iy+1,iz+1);
    float x0=lerpf(v000,v100,ux), x1=lerpf(v010,v110,ux);
    float x2=lerpf(v001,v101,ux), x3=lerpf(v011,v111,ux);
    float y0=lerpf(x0,x1,uy),     y1=lerpf(x2,x3,uy);
    return lerpf(y0, y1, uz);
}

/* 4-octave fBm — primary cauliflower texture */
static float fbm4(V3 p)
{
    float v=0.f, amp=0.5f, freq=1.f;
    for (int i=0; i<4; i++) {
        v    += amp * (noise3(v3mul(p, freq)) - 0.5f);
        amp  *= 0.5f;
        freq *= 2.1f;
    }
    return v;
}

/* 2-octave fBm — cheap secondary axis */
static float fbm2(V3 p)
{
    float v = (noise3(p) - 0.5f) * 0.5f;
    v += (noise3(v3mul(p, 2.1f)) - 0.5f) * 0.25f;
    return v;
}

/* Domain-warped fBm — gives the swirling, cauliflower-like turbulence. */
static float warped_fbm(V3 p, float t)
{
    V3 pa = v3add(p, v3(t * DISP_SPD, t * DISP_SPD * 0.6f, 0.f));
    V3 warp = v3(
        noise3(v3add(v3mul(pa, WARP_FREQ), v3(1.7f, 9.2f, 0.f))),
        noise3(v3add(v3mul(pa, WARP_FREQ), v3(8.3f, 2.8f, 0.f))),
        noise3(v3add(v3mul(pa, WARP_FREQ), v3(3.1f, 5.4f, 0.f)))
    );
    V3 warped = v3add(pa, v3mul(warp, WARP_AMP));
    return fbm4(v3mul(warped, DISP_FREQ));
}

/* ===================================================================== */
/* §6  density + heat — full mushroom anatomy                             */
/*                                                                        */
/* Six summed Gaussian volumes:                                           */
/*   fireball   — hot core; rises during RISING                          */
/*   stem       — ground-to-cap, widens at top                           */
/*   cap        — oblate spheroid dome                                   */
/*   vortex     — torus rolling outward (the bulging cap edge)           */
/*   skirt      — thin condensation collar at base of cap                */
/*   pyrocumulus — 4 rotating billows on top of cap                      */
/*                                                                        */
/* Each shape is a Gaussian: smooth, semi-transparent, no hard edges.    */
/* The whole thing is then displaced by 3D noise to make it amorphous.   */
/* ===================================================================== */

static inline float gauss(float d2, float r2)
{
    return expf(-d2 / fmaxf(r2, 1e-6f));
}

/* Cheap 2D torus distance² (around Y axis, centred at (0, cy, 0)). */
static inline float torus_d2(V3 p, float cy, float major_r)
{
    float ax = sqrtf(p.x*p.x + p.z*p.z) - major_r;
    float ty = p.y - cy;
    return ax*ax + ty*ty;
}

/*
 * cloud_density() — total smoke/gas density at world point p.
 *   time           — bomb's elapsed time (animates noise)
 *   fireball_r     — current fireball radius
 *   fireball_y     — current fireball Y centre
 *   stem_h         — stem height above BLAST_Y
 *   cap_y          — cap centre Y
 *   cap_scale      — 0..1, cap grows during CAP_FORM
 *   vortex_scale   — 0..1, vortex roll grows alongside cap
 */
static float cloud_density(V3 p, float time,
                            float fireball_r, float fireball_y,
                            float stem_h,
                            float cap_y, float cap_scale, float vortex_scale)
{
    /* ── EARLY BOUNDS CHECK ────────────────────────────────────────────
     * Roughly 60% of step samples on a hit ray are still in empty space
     * around the cloud (entering, exiting, or between shapes). Returning
     * 0 here BEFORE the noise + Gaussian work is the single biggest perf
     * win. The bounds adapt to the current shape sizes.
     */
    {
        float r_horiz2 = p.x*p.x + p.z*p.z;

        /* Widest possible horizontal extent at this moment, with a small
         * margin for noise displacement. */
        float r_max = fmaxf(fireball_r, CAP_R_H * cap_scale);
        if (stem_h > 0.05f) r_max = fmaxf(r_max, STEM_R_TOP);
        r_max += DISP_AMP + 0.15f;
        if (r_horiz2 > r_max * r_max) return 0.f;

        /* Vertical extent: top is the higher of fireball_y+r and the cap
         * top; bottom is the blast site. Both padded for displacement. */
        float y_top_fb  = fireball_y + fireball_r;
        float y_top_cap = (cap_scale > 0.05f) ? (cap_y + CAP_R_V) : -1e9f;
        float y_top     = fmaxf(y_top_fb, y_top_cap) + DISP_AMP * 0.95f;
        float y_bot     = BLAST_Y - DISP_AMP * 0.4f;
        if (p.y > y_top || p.y < y_bot) return 0.f;
    }

    /* 3D displacement: independent X/Y/Z noise → genuine swirl, not puffing.
     * Sample point drifts upward (RISE_BIAS) so turbulence appears to rise. */
    V3 pa = v3(p.x, p.y - time * RISE_BIAS, p.z);

    float nx = warped_fbm(pa, time);
    float ny = fbm2(v3add(v3mul(pa, DISP_FREQ), v3(5.3f, 1.7f, 3.2f)));
    float nz = fbm2(v3add(v3mul(pa, DISP_FREQ), v3(2.9f, 8.1f, 0.7f)));

    V3 pp = v3(p.x + nx * DISP_AMP,
               p.y + ny * DISP_AMP * 0.85f,
               p.z + nz * DISP_AMP * 0.90f);

    float d = 0.f;

    /* ── 1. Fireball — hot spheroid (rises during RISING) ────────────── */
    if (fireball_r > 0.001f) {
        V3 dp = v3sub(pp, v3(0, fireball_y, 0));
        float r2 = fireball_r * fireball_r;
        d += gauss(v3dot(dp, dp), r2 * FIRE_SPREAD);
    }

    /* ── 2. Stem — Gaussian cylinder, wider at top ───────────────────── */
    if (stem_h > 0.05f) {
        float ax2    = pp.x*pp.x + pp.z*pp.z;
        float y_frac = clmpf((pp.y - BLAST_Y) / stem_h, 0.f, 1.f);
        float R      = lerpf(STEM_R_BASE, STEM_R_TOP, y_frac);
        /* taper at top so stem blends into cap, taper at base too */
        float ym     = smoothstep(0.f, 0.10f, y_frac)
                     * smoothstep(1.10f, 0.78f, y_frac);
        d += gauss(ax2, R * R * STEM_SPREAD) * ym;
    }

    /* ── 3+4+5. Cap, vortex, skirt — only after cap has begun forming ── */
    if (cap_scale > 0.01f) {
        /* 3. Oblate spheroid cap. Use anisotropic distance: stretch Y so
         *    the same Gaussian becomes flat horizontally + thin vertically. */
        V3 cp = v3sub(pp, v3(0, cap_y, 0));
        float dy_n  = cp.y / CAP_R_V;
        float dr_n  = sqrtf(cp.x*cp.x + cp.z*cp.z) / CAP_R_H;
        float cap_d2 = (dr_n*dr_n + dy_n*dy_n) * CAP_R_H * CAP_R_H;
        float cap_r  = CAP_R_H * cap_scale;
        d += gauss(cap_d2, cap_r * cap_r * CAP_SPREAD) * 0.95f;

        /* 4. Vortex roll — torus rotating outward → bulging cap edges. */
        if (vortex_scale > 0.01f) {
            float vmaj  = VORTEX_R_MAJ * vortex_scale;
            float vmin2 = VORTEX_R_MIN * VORTEX_R_MIN * vortex_scale * vortex_scale;
            float td2   = torus_d2(cp, 0.f, vmaj);
            d += gauss(td2, vmin2 * VORTEX_SPREAD) * 0.55f;
        }

        /* 5. Skirt — thin condensation collar where stem meets cap base. */
        float smaj  = SKIRT_R_MAJ * cap_scale;
        float smin2 = SKIRT_R_MIN * SKIRT_R_MIN;
        V3    sp    = v3sub(pp, v3(0, cap_y - CAP_R_V * 0.85f, 0));
        d += gauss(torus_d2(sp, 0.f, smaj), smin2 * SKIRT_SPREAD) * 0.45f;

        /* 6. Pyrocumulus — small billows scattered on top of the cap. */
        float pyro_r2 = PYRO_R * PYRO_R * cap_scale * cap_scale;
        for (int i = 0; i < PYRO_COUNT; i++) {
            float ang = (float)i * (float)(2.0 * M_PI / PYRO_COUNT)
                      + time * 0.12f;
            V3 bp = v3(cosf(ang) * PYRO_OFFSET_R * cap_scale,
                       PYRO_OFFSET_Y * cap_scale,
                       sinf(ang) * PYRO_OFFSET_R * cap_scale);
            V3 dd = v3sub(cp, bp);
            d += gauss(v3dot(dd, dd), pyro_r2 * PYRO_SPREAD) * 0.40f;
        }
    }

    return clmpf(d, 0.f, 2.5f);
}

/*
 * cloud_heat() — temperature at world point p, ∈ [0,1].
 * Hot fireball core dominates; stem has warm interior; cap is mostly cold
 * smoke with a thin hot rim where the fireball pokes through.
 */
static float cloud_heat(V3 p,
                         float fireball_r, float fireball_y,
                         float stem_h, float cap_y, float cap_scale)
{
    float heat = 0.f;

    if (fireball_r > 0.001f) {
        V3 dp = v3sub(p, v3(0, fireball_y, 0));
        float r2 = fireball_r * fireball_r;
        heat = fmaxf(heat, 0.95f * gauss(v3dot(dp, dp), r2 * 0.30f));
    }

    if (stem_h > 0.05f) {
        float y_frac = clmpf((p.y - BLAST_Y) / stem_h, 0.f, 1.f);
        float ax2    = p.x*p.x + p.z*p.z;
        float fall   = gauss(ax2, STEM_R_TOP * STEM_R_TOP * 1.6f);
        heat = fmaxf(heat, lerpf(0.55f, 0.10f, y_frac) * fall);
    }

    /* Cap retains a dim warm glow at its core early on (still incandescent). */
    if (cap_scale > 0.01f) {
        V3 cp = v3sub(p, v3(0, cap_y, 0));
        float r2 = (CAP_R_H * 0.5f) * (CAP_R_H * 0.5f);
        heat = fmaxf(heat, 0.30f * gauss(v3dot(cp, cp), r2)
                          * (1.f - cap_scale * 0.6f));
    }

    return clmpf(heat, 0.f, 1.f);
}

/* ===================================================================== */
/* §7  particles — debris + embers                                        */
/* ===================================================================== */

typedef enum { PART_DEBRIS = 0, PART_EMBER = 1 } PartType;

typedef struct {
    float    x, y, z;
    float    vx, vy, vz;
    float    life, max_life;
    PartType type;
    bool     alive;
} Particle;

static Particle g_parts[MAX_PARTICLES];
static unsigned g_pseed = 12345u;

static float prand(void)
{
    g_pseed = g_pseed * 1664525u + 1013904223u;
    return (float)((g_pseed >> 8) & 0xFFFF) / 65535.f;
}

static Particle *part_alloc(void)
{
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (!g_parts[i].alive) return &g_parts[i];
    return NULL;
}

static void particles_spawn_debris(void)
{
    for (int i = 0; i < DEBRIS_N; i++) {
        Particle *p = part_alloc();
        if (!p) break;
        float theta = prand() * (float)(2.0 * M_PI);
        float phi   = acosf(1.f - 2.f * prand());
        V3 dir = v3(sinf(phi)*cosf(theta), sinf(phi)*sinf(theta), cosf(phi));
        dir.y = fabsf(dir.y) * (0.3f + prand() * 0.7f);
        dir   = v3norm(dir);
        float spd = DEBRIS_SPD * (0.4f + prand() * 0.8f);
        p->x = 0; p->y = BLAST_Y; p->z = 0;
        p->vx = dir.x * spd;
        p->vy = dir.y * spd;
        p->vz = dir.z * spd;
        p->max_life = p->life = 0.8f + prand() * 1.4f;
        p->type  = PART_DEBRIS;
        p->alive = true;
    }
}

static void particles_spawn_ember(float stem_h)
{
    Particle *p = part_alloc();
    if (!p) return;
    float theta = prand() * (float)(2.0 * M_PI);
    float y_off = prand() * stem_h;
    float r = STEM_R_BASE * (0.4f + prand() * 0.8f);
    p->x = cosf(theta) * r;
    p->y = BLAST_Y + y_off;
    p->z = sinf(theta) * r;
    p->vx = (prand() - 0.5f) * 0.3f;
    p->vy = EMBER_SPD * (0.5f + prand() * 0.8f);
    p->vz = (prand() - 0.5f) * 0.3f;
    p->max_life = p->life = 1.5f + prand() * 2.0f;
    p->type  = PART_EMBER;
    p->alive = true;
}

static void particles_tick(float dt)
{
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &g_parts[i];
        if (!p->alive) continue;
        p->vy += PART_GRAVITY * dt;
        p->x  += p->vx * dt;
        p->y  += p->vy * dt;
        p->z  += p->vz * dt;
        p->life -= dt;
        if (p->life <= 0.f || p->y < BLAST_Y - 2.f) p->alive = false;
    }
}

static void particles_draw(int cols, int rows, int canv_ox, int canv_oy,
                            int cw, int ch)
{
    float pa = ((float)ch * CELL_ASPECT) / (float)cw;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &g_parts[i];
        if (!p->alive) continue;
        float cam_dz = CAM_Z - p->z;
        if (cam_dz < 0.1f) continue;
        float u =  p->x / (cam_dz * FOV_HALF_TAN);
        float v = -p->y / (cam_dz * FOV_HALF_TAN * pa);
        int cx = (int)((u + 1.f) * 0.5f * (float)cw);
        int cy = (int)((v + 1.f) * 0.5f * (float)ch);
        int tx = canv_ox + cx, ty = canv_oy + cy;
        if (tx < 0 || tx >= cols || ty < 0 || ty >= rows) continue;
        float age     = 1.f - p->life / p->max_life;
        float bright  = smoothstep(0.f, 0.05f, 1.f - age) * smoothstep(1.f, 0.75f, age);
        if (bright < 0.05f) continue;
        char ch_p; attr_t a;
        if (p->type == PART_DEBRIS) {
            ch_p = (bright > 0.6f) ? '*' : '.';
            a = COLOR_PAIR(CP_DEBRIS) | (bright > 0.5f ? A_BOLD : 0);
        } else {
            ch_p = (bright > 0.5f) ? '+' : '.';
            a = COLOR_PAIR(CP_EMBER);
        }
        attron(a); mvaddch(ty, tx, (chtype)(unsigned char)ch_p); attroff(a);
    }
}

/* ===================================================================== */
/* §8  simulation — continuous-time geometry                              */
/*                                                                        */
/* The previous version used a hard switch(phase) state machine: each    */
/* phase set the parameters of "its" shapes and zeroed the others. That  */
/* made transitions feel like sprite frames — stem appeared suddenly     */
/* when FIREBALL → RISING, cap snapped in at RISING → CAP_FORM.          */
/*                                                                        */
/* Now every parameter is a smoothstep over absolute simulated time, and */
/* the keypoints overlap. The fireball is already rising while the cap   */
/* is starting to bulge; nothing snaps from 0 to non-zero. The phase     */
/* enum survives only as a label for the HUD.                            */
/* ===================================================================== */

typedef enum {
    NK_FLASH, NK_FIREBALL, NK_RISING, NK_CAP_FORM, NK_MUSHROOM, NK_PLATEAU
} NukePhase;

static const char *k_phase_names[] = {
    "FLASH", "FIREBALL", "RISING", "CAP FORM", "MUSHROOM", "PLATEAU"
};

typedef struct {
    NukePhase phase;        /* derived from time — for HUD only           */
    float     time;         /* seconds of simulated time since detonation */
    float     time_scale;
    bool      paused;
    bool      done;

    /* derived geometry */
    float     fireball_r;
    float     fireball_y;
    float     stem_h;
    float     cap_y;
    float     cap_scale;
    float     vortex_scale;
    float     flash_intensity;
    float     shockwave_r;

    /* particle bookkeeping */
    float     ember_accum;
    bool      debris_spawned;
} NukeState;

static NukeState g_nuke;
static float     g_global_ts    = 1.0f;
static bool      g_global_pause = false;
static bool      g_show_smoke   = true;
static bool      g_lighting     = true;

static void sim_init(NukeState *s, float ts)
{
    memset(s, 0, sizeof *s);
    s->phase      = NK_FLASH;
    s->time_scale = ts;
}

/*
 * Derive a phase label from absolute time. Used only for the HUD —
 * geometry no longer branches on phase.
 */
static NukePhase phase_for_time(float t)
{
    if (t < T_FLASH_END)     return NK_FLASH;
    if (t < T_RISE_BEG)      return NK_FIREBALL;
    if (t < T_CAP_BEG)       return NK_RISING;
    if (t < T_MUSHROOM_BEG)  return NK_CAP_FORM;
    if (t < T_PLATEAU_BEG)   return NK_MUSHROOM;
    return NK_PLATEAU;
}

/*
 * Continuous-time geometry. Each parameter blends between keypoints with
 * smoothstep(); keypoints overlap deliberately so phases bleed into each
 * other instead of swapping like sprite frames.
 *
 * Reading guide:
 *   smoothstep(a, b, t) = 0 for t<a, 1 for t>b, smooth S-curve in between.
 *   So `r = R_MAX * smoothstep(t0, t1, t)` means r grows from 0→R_MAX
 *   over the time window [t0, t1] and stays at R_MAX afterwards.
 */
static void sim_update_geometry(NukeState *s)
{
    float t = s->time;

    /* Flash decays over [0, T_FLASH_END]. */
    s->flash_intensity = 1.f - smoothstep(0.f, T_FLASH_END, t);

    /* Fireball radius: grows in, then partly shrinks as it cools and
     * feeds the cap. Two overlapping smoothsteps multiplied together. */
    float fb_grow   = smoothstep(T_FB_GROW_BEG, T_FB_GROW_END, t);
    float fb_shrink = 1.f - smoothstep(T_FB_SHRINK_BEG, T_FB_SHRINK_END, t)
                            * 0.42f;
    s->fireball_r = FIREBALL_R_MAX * fb_grow * fb_shrink;

    /* Fireball Y: rests at BLAST_Y until T_RISE_BEG, then climbs to
     * BLAST_Y + STEM_HEIGHT_MAX over [T_RISE_BEG, T_RISE_END]. */
    s->fireball_y = BLAST_Y + STEM_HEIGHT_MAX
                   * smoothstep(T_RISE_BEG, T_RISE_END, t);

    /* Stem height tracks how far the fireball has risen — perfectly
     * continuous because it's a direct function of fireball_y. */
    s->stem_h = fmaxf(0.f, s->fireball_y - BLAST_Y);

    /* Cap & vortex grow on overlapping windows. Cap starts BEFORE the
     * fireball reaches the top, which is what kills the sprite-feel:
     * during 2.4s..3.2s the cap is forming around a still-rising fireball.
     */
    float cap_growth = smoothstep(T_CAP_BEG,    T_CAP_END,    t);
    float vortex     = smoothstep(T_VORTEX_BEG, T_VORTEX_END, t);

    /* Gentle breathing pulse, mostly visible once the cap is mature.
     * Faded in via smoothstep so it doesn't appear suddenly. */
    float breathe_amount = smoothstep(T_CAP_END, T_CAP_END + 1.0f, t);
    float pulse = 1.f + 0.025f * sinf(t * 1.4f) * breathe_amount;

    s->cap_scale    = cap_growth * pulse;
    s->vortex_scale = vortex;

    /* Cap rides slightly above the fireball — works for every t because
     * fireball_y is itself continuous from BLAST_Y to the apex. */
    s->cap_y = s->fireball_y + 0.20f;

    /* Shockwave keeps expanding (kept for reference; not currently drawn). */
    s->shockwave_r = (t > T_FB_GROW_BEG)
        ? SHOCKWAVE_SPD * (t - T_FB_GROW_BEG) : 0.f;

    /* Phase label for HUD only. */
    s->phase = phase_for_time(t);
}

static void sim_tick(NukeState *s, float dt)
{
    if (s->paused || s->done) return;

    float adt = dt * s->time_scale;
    s->time  += adt;

    if (s->time > T_PLATEAU_END) {
        s->done = true;
        return;
    }

    sim_update_geometry(s);

    /* Spawn debris once, when the fireball first appears. */
    if (!s->debris_spawned && s->time > T_FB_GROW_BEG + 0.05f) {
        particles_spawn_debris();
        s->debris_spawned = true;
    }

    /* Embers drift up the stem from the moment it starts forming until
     * the cap is mature — no phase test needed. */
    if (s->time > T_RISE_BEG && s->time < T_PLATEAU_BEG && s->stem_h > 0.1f) {
        s->ember_accum += adt;
        float interval = 1.f / (float)EMBER_N;
        while (s->ember_accum >= interval) {
            particles_spawn_ember(s->stem_h);
            s->ember_accum -= interval;
        }
    }
}

static void nuke_reset(void)
{
    memset(g_parts, 0, sizeof g_parts);
    sim_init(&g_nuke, g_global_ts);
}

/* ===================================================================== */
/* §9  raymarcher — Beer-Lambert with 2× vertical supersampling           */
/*                                                                        */
/* The render loop in §10 calls rm_cast_one() twice per terminal cell    */
/* (top half + bottom half) and combines the two samples into a glyph.   */
/* ===================================================================== */

typedef struct {
    float heat;       /* emission accumulated along ray, ∈ [0,1]         */
    float opacity;    /* 1 - transmittance; ∈ [0,1]                       */
    float bright;     /* lit-smoke brightness (extinction × diffuse)      */
} Sample;

/*
 * Cast one ray through the volume.
 *
 *   sub_row ∈ [0..SS_Y-1] — which vertical sub-row within the cell
 *
 * Returns emission (heat), opacity (smoke density), and lit brightness.
 */
static Sample rm_cast_one(int px_col, int py_row, int sub_row,
                           int cw, int ch_cells, const NukeState *s)
{
    /* Sub-pixel vertical jitter: split each cell into SS_Y horizontal
     * stripes; the ray for sub_row passes through the centre of stripe i. */
    float v_offset = ((float)sub_row + 0.5f) / (float)SS_Y;
    float hires_h  = (float)(ch_cells * SS_Y);

    float u  =  ((float)px_col + 0.5f) / (float)cw * 2.f - 1.f;
    float v  = -((float)py_row + v_offset) / (float)ch_cells * 2.f + 1.f;

    /* Aspect compensation: account for SS_Y so vertical world-units stay
     * proportional to horizontal world-units when supersampling. */
    float pa = ((float)ch_cells * CELL_ASPECT) / (float)cw;
    (void)hires_h;

    V3 ro = v3(0.f, 0.f, CAM_Z);
    V3 rd = v3norm(v3(u * FOV_HALF_TAN, v * FOV_HALF_TAN * pa, -1.f));

    Sample out = {0};

    /* Cloud is centered around (0, cloud_cy, 0). Bound the cull sphere
     * on that center — earlier code used the origin which made the test
     * loose for the elongated mushroom shape. */
    float cloud_cy = (BLAST_Y + s->cap_y + CAP_R_V) * 0.5f;
    V3    cloud_c  = v3(0.f, cloud_cy, 0.f);

    /* Ray-sphere bounds test — bail if the ray misses the cloud entirely. */
    {
        V3 oc = v3sub(ro, cloud_c);
        float b = v3dot(oc, rd);
        float c = v3dot(oc, oc) - CULL_R_BOMB * CULL_R_BOMB;
        if (b*b - c < 0.f) return out;
    }

    float transmit = 1.f;
    float heat_acc = 0.f;
    float t        = 0.2f;
    const float cull_r2 = CULL_R_BOMB * CULL_R_BOMB;

    for (int i = 0; i < RM_MAX_STEPS; i++) {
        V3 p = v3add(ro, v3mul(rd, t));

        /* Per-step cull against the cloud bounding sphere. Take a larger
         * step when clearly outside — gets us through empty space faster. */
        float dx = p.x, dy = p.y - cloud_cy, dz = p.z;
        if (dx*dx + dy*dy + dz*dz > cull_r2) {
            t += RM_STEP * 1.6f;
            if (t > RM_MAX_DIST) break;
            continue;
        }

        float d = cloud_density(p, s->time,
                                s->fireball_r, s->fireball_y, s->stem_h,
                                s->cap_y, s->cap_scale, s->vortex_scale);

        if (d > DENSITY_THRESH) {
            float h = cloud_heat(p, s->fireball_r, s->fireball_y,
                                 s->stem_h, s->cap_y, s->cap_scale);

            float sigma = d * RM_STEP * EXTINCTION;
            float dT    = expf(-sigma);

            /* Cheap self-shadow: smoke high in the column is brighter
             * because less stuff is between it and the sun. */
            float shadow = smoothstep(BLAST_Y - 0.5f,
                                      BLAST_Y + STEM_HEIGHT_MAX + 1.0f, p.y);
            /* Sun angle from the right: brighten right side of cloud. */
            float side   = smoothstep(-1.5f, 1.5f, p.x);
            float light  = LIGHT_AMBIENT
                         + LIGHT_DIFFUSE * lerpf(shadow, side, 0.4f);

            heat_acc      += transmit * (1.f - dT) * h;
            out.bright    += transmit * (1.f - dT) * light;
            transmit      *= dT;

            if (transmit < TRANSMIT_BAIL) break;
        }

        t += RM_STEP;
        if (t > RM_MAX_DIST) break;
    }

    out.opacity = 1.f - transmit;
    out.heat    = clmpf(heat_acc, 0.f, 1.f);
    return out;
}

/* ===================================================================== */
/* §10  canvas — sub-pixel pair buffer + glyph picker                     */
/*                                                                        */
/* For each terminal cell we keep (top, bot) Samples. The glyph picker   */
/* uses both to choose top-weighted, bottom-weighted, full, or empty     */
/* characters — recovering vertical detail beyond the cell grid.         */
/* ===================================================================== */

typedef struct { Sample top, bot; } CellPair;

typedef struct { int w, h; CellPair *cells; } Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows)
{
    c->w = cols; c->h = rows;
    c->cells = calloc((size_t)(cols * rows), sizeof(CellPair));
}

static void canvas_free(Canvas *c)
{
    free(c->cells); c->cells = NULL; c->w = c->h = 0;
}

static void canvas_fill(Canvas *c, const NukeState *s)
{
    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            CellPair *cp = &c->cells[py*c->w + px];
            cp->top = rm_cast_one(px, py, 0, c->w, c->h, s);
            cp->bot = rm_cast_one(px, py, 1, c->w, c->h, s);
        }
    }
}

/*
 * pick_glyph — choose ASCII char from (top opacity, bottom opacity).
 *
 * The glyph encodes which half of the cell is denser:
 *   both strong  → full-body chars     '#' '@' '%' '8'
 *   top  only    → ascender chars      '"' '`' '^' '~'
 *   bottom only  → descender chars     '.' ',' '_' 'u'
 *   both medium  → middle chars        ':' ';' '|' '|'
 *   sparse       → '.' or ' '
 */
static char pick_glyph(float top_op, float bot_op)
{
    /* Quantise opacities into [0..3]: 0 empty, 1 light, 2 mid, 3 dense. */
    int qt = (int)(clmpf(top_op, 0.f, 1.f) * 3.999f);
    int qb = (int)(clmpf(bot_op, 0.f, 1.f) * 3.999f);
    if (qt > 3) qt = 3;
    if (qb > 3) qb = 3;

    static const char full[4]    = { ' ', '+', '%', '#' };  /* by max(qt,qb) */
    static const char top_w[4]   = { ' ', '`', '"', '^' };  /* top-only */
    static const char bot_w[4]   = { ' ', '.', ',', 'u' };  /* bottom-only */
    static const char middle[4]  = { ' ', ':', ';', '|' };  /* both medium */

    if (qt == 0 && qb == 0) return ' ';

    /* Asymmetric: one half clearly heavier than the other → weighted glyph. */
    if (qt > qb + 1) return top_w[qt];
    if (qb > qt + 1) return bot_w[qb];

    /* Roughly symmetric. Pick full body if anything is dense, else middle. */
    int q = (qt > qb) ? qt : qb;
    if (q >= 3) return full[q];
    return middle[q];
}

/*
 * canvas_draw() — map cell pairs to ncurses, then overlay flash + particles.
 *
 * Per cell: pick glyph from the (top, bot) opacity pair, then pick color
 * from heat (fire) or brightness (smoke). When heat is high we use the fire
 * palette and bias towards bold; otherwise we use the smoke palette so cold
 * smoke renders as gray instead of being culled.
 */
static void canvas_draw(const Canvas *c, int tcols, int trows,
                         const NukeState *s)
{
    int ox = (tcols - c->w) / 2;
    int oy = (trows - c->h) / 2;

    for (int y = 0; y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            int tx = ox + x, ty = oy + y;
            if (tx < 0 || tx >= tcols || ty < 0 || ty >= trows) continue;
            const CellPair *cp = &c->cells[y*c->w + x];

            float top_op = cp->top.opacity;
            float bot_op = cp->bot.opacity;
            float avg_op = (top_op + bot_op) * 0.5f;
            if (avg_op < 0.04f) continue;

            char g = pick_glyph(top_op, bot_op);
            if (g == ' ') continue;

            float avg_heat = (cp->top.heat + cp->bot.heat) * 0.5f;
            float avg_lit  = (cp->top.bright + cp->bot.bright) * 0.5f;

            attr_t a;
            if (avg_heat > 0.18f) {
                /* Hot region — use fire palette driven by heat. */
                a = fire_pair(avg_heat);
            } else if (g_show_smoke) {
                /* Cold smoke — use smoke palette driven by lit brightness. */
                float b = g_lighting
                        ? (avg_lit / fmaxf(avg_op, 0.1f))
                        : avg_op;
                /* Slight per-cell random dither to break up banding. */
                b += (prand() - 0.5f) * 0.05f;
                a = smoke_pair(b);
            } else {
                continue;
            }
            attron(a);
            mvaddch(ty, tx, (chtype)(unsigned char)g);
            attroff(a);
        }
    }

    /* Particles drawn on top. */
    particles_draw(tcols, trows, ox, oy, c->w, c->h);

    /* Initial-flash overlay — sparse white sparkles across the screen. */
    if (s->flash_intensity > 0.01f) {
        static const char flash_chars[] = "@#$%+*";
        int    ncov = (int)(s->flash_intensity * 5.9f);
        char   fch  = flash_chars[ncov > 5 ? 5 : ncov];
        attr_t fa   = COLOR_PAIR(CP_FLASH) | A_BOLD;
        attron(fa);
        for (int y = 0; y < trows - 2; y++)
            for (int x = 0; x < tcols; x++)
                if (prand() < s->flash_intensity)
                    mvaddch(y, x, (chtype)(unsigned char)fch);
        attroff(fa);
    }
}

/* ===================================================================== */
/* §11  renderer                                                          */
/* ===================================================================== */

typedef struct { Canvas canvas; } Renderer;

static void renderer_init(Renderer *r, int cols, int rows)
{
    canvas_alloc(&r->canvas, cols, rows);
}

static void renderer_free(Renderer *r) { canvas_free(&r->canvas); }

static void renderer_resize(Renderer *r, int cols, int rows)
{
    canvas_free(&r->canvas);
    canvas_alloc(&r->canvas, cols, rows);
}

static void renderer_render(Renderer *r) { canvas_fill(&r->canvas, &g_nuke); }

static void renderer_draw(const Renderer *r, int cols, int rows, double fps)
{
    canvas_draw(&r->canvas, cols, rows, &g_nuke);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%.1ffps  %s  t=%.1f  spd:%.1fx  theme:%s  smoke:%s  light:%s  %s",
             fps, k_phase_names[g_nuke.phase], g_nuke.time,
             g_global_ts,
             g_themes[g_theme_idx].name,
             g_show_smoke ? "on" : "off",
             g_lighting   ? "on" : "off",
             g_global_pause ? "PAUSED" : "");
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, "%.*s", cols - 1, buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  +/-:speed  n:redo  t:theme  s:smoke  l:light");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §12  screen — ncurses                                                  */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §13  app — main loop                                                   */
/* ===================================================================== */

typedef struct {
    Renderer  rend;
    Screen    screen;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit  (int sig) { (void)sig; g_app.running = 0; }
static void on_resize(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup  (void)   { endwin(); }

static void app_do_resize(App *a)
{
    screen_resize(&a->screen);
    renderer_resize(&a->rend, a->screen.cols, a->screen.rows);
    a->need_resize = 0;
}

static bool app_handle_key(App *a, int ch)
{
    (void)a;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':
        g_global_pause = !g_global_pause;
        g_nuke.paused  = g_global_pause;
        break;
    case 'r': case 'R':
    case 'n': case 'N':
        nuke_reset();
        break;
    case '+': case '=':
        g_global_ts = fminf(g_global_ts + 0.5f, 16.0f);
        g_nuke.time_scale = g_global_ts;
        break;
    case '-':
        g_global_ts = fmaxf(g_global_ts - 0.5f, 0.25f);
        g_nuke.time_scale = g_global_ts;
        break;
    case 's': case 'S': g_show_smoke = !g_show_smoke; break;
    case 'l': case 'L': g_lighting   = !g_lighting;   break;
    case 't': case 'T': cycle_theme(); break;
    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app  = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    nuke_reset();
    renderer_init(&app->rend, app->screen.cols, app->screen.rows);

    int64_t frame_t = clock_ns();
    int64_t sim_acc = 0;
    int64_t fps_acc = 0;
    int     fps_cnt = 0;
    double  fpsd    = 0.0;

    while (app->running) {
        if (app->need_resize) { app_do_resize(app); frame_t = clock_ns(); sim_acc = 0; }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_t;
        frame_t     = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick   = TICK_NS(SIM_FPS);
        float   tick_s = (float)tick / (float)NS_PER_SEC;
        sim_acc += dt;
        while (sim_acc >= tick) {
            sim_tick(&g_nuke, tick_s);
            particles_tick(tick_s);
            sim_acc -= tick;
        }

        renderer_render(&app->rend);

        erase();
        renderer_draw(&app->rend, app->screen.cols, app->screen.rows, fpsd);
        screen_present();

        fps_cnt++;
        fps_acc += dt;
        if (fps_acc >= FPS_MS * NS_PER_MS) {
            fpsd    = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
            fps_cnt = 0;
            fps_acc = 0;
        }

        int64_t elapsed = clock_ns() - frame_t + dt;
        clock_sleep_ns(NS_PER_SEC / 24 - elapsed);

        int key = getch();
        if (key != ERR && !app_handle_key(app, key)) app->running = 0;
    }

    renderer_free(&app->rend);
    screen_free(&app->screen);
    return 0;
}
