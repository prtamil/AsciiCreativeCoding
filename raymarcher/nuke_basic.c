/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nuke_basic.c — 3D Atomic Bomb Mushroom Cloud Simulation  (learner-annotated)
 *
 * Full detonation sequence rendered via volumetric density raymarching:
 *   - FLASH     → white blast pulse
 *   - FIREBALL  → sphere expands + shockwave ring races outward
 *   - RISING    → fireball rises, trailing a gaseous stem
 *   - CAP_FORM  → mushroom cap blooms, condensation skirt forms
 *   - MUSHROOM  → full cloud with boiling fBm surface + ember drift
 *   - PLATEAU   → cloud holds, then fades out after DUR_PLATEAU seconds
 *
 * Keys:  q/ESC quit  |  space pause  |  r restart all  |  +/- speed
 *        n = detonate another bomb (up to MAX_NUKES at once)
 * Build: gcc -std=c11 -O2 -Wall -Wextra raymarcher/nuke_basic.c -o nuke_basic -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config      — all tunable constants
 *   §2  clock       — monotonic timer + sleep
 *   §3  color       — nuclear heat + smoke palette
 *   §4  vec3 math   — 3D vector operations
 *   §5  noise/fBm   — value noise, fBm, domain warping
 *   §6  density     — Gaussian cloud_density + cloud_heat
 *   §7  particles   — 3D debris, embers, shockwave ring
 *   §8  simulation  — NukeState phase state machine + NukeInstance array
 *   §9  raymarcher  — volumetric Beer-Lambert integrator
 *  §10  canvas      — pixel buffer + flash/particle overlay
 *  §11  renderer    — bridges simulation → screen
 *  §12  screen      — ncurses terminal setup
 *  §13  app         — main loop
 */

/* ── HOW THIS WORKS ──────────────────────────────────────────────────────── *
 *
 *  PIPELINE (one frame):
 *
 *    sim_tick() × N_NUKES   advance each bomb's phase state machine
 *         ↓
 *    canvas_fill()   for every terminal cell, cast one ray:
 *         ↓             • pixel (col,row) → 3D ray direction
 *    rm_cast()          • for each bomb: sample density + heat at every step
 *         ↓             • Beer-Lambert: accumulate opacity + emission
 *    Pixel buffer      • result: heat ∈ [0,1] + transmittance
 *         ↓
 *    canvas_draw()   raymarched pixels + flash + shockwaves + particles
 *
 *  WHY VOLUMETRIC DENSITY (not SDF)?
 *
 *  SDF sphere-tracing gives HARD SURFACES — wrong for gas/smoke.
 *  A density field gives soft, semi-transparent, gaseous shapes:
 *   • Thin cloud regions are translucent — you see the hot core glowing through
 *   • No hard surface boundary — edges naturally fade to nothing
 *   • Multiple overlapping density sources simply add together
 *  Each shape is a Gaussian blob (exp(-d²/r²)) — 1 at centre, smooth → 0.
 *
 *  WHY 3D DISPLACEMENT INSTEAD OF SCALAR?
 *
 *  A scalar noise displaces the sample point symmetrically → puffy balls.
 *  Three independent noise samples for X/Y/Z displacement create genuine
 *  swirling, curl-like patterns — each axis churns independently.
 *  This is the visual difference between "puffy clouds" and "swirling gas".
 *
 *  MULTIPLE BOMBS:
 *
 *  Each NukeInstance has its own NukeState (phase machine) + world offset (ox,oz).
 *  rm_cast() loops over all active instances at each ray step, summing their
 *  density contributions. Heat is density-weighted across overlapping clouds.
 *  Bombs detonate independently — n key spawns a new one.
 *
 *  References:
 *    Volume rendering:  https://www.scratchapixel.com/lessons/3d-basic-rendering/
 *                       volume-rendering-for-developers
 *    fBm + warp:        https://iquilezles.org/articles/warp/
 *    Fixed timestep:    https://gafferongames.com/post/fix_your_timestep/
 *
 * ─────────────────────────────────────────────────────────────────────────*/

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
/* §1  config — all tunable constants                                     */
/* ===================================================================== */

enum {
    SIM_FPS        = 24,   /* physics ticks per second                        */
    HUD_COLS       = 80,   /* status line buffer width (chars)                */
    FPS_MS         = 500,  /* fps counter refresh interval (ms)               */
    MAX_PARTICLES  = 480,  /* total particle slots (scales with MAX_NUKES)    */
    MAX_NUKES      = 4,    /* simultaneous active bomb slots                  */
};

/* ── Raymarcher (volumetric) ─────────────────────────────────────────── */
#define RM_MAX_STEPS   64     /* max integration steps per ray                 */
#define RM_STEP        0.14f  /* fixed step size (world units)                 */
#define RM_MAX_DIST    22.0f  /* ray bail-out distance                         */
#define EXTINCTION     4.2f   /* smoke extinction coefficient                  */
#define DENSITY_THRESH 0.015f /* skip density evaluation below this            */
#define BOMB_CULL_R    3.8f   /* bounding sphere radius per bomb (world units) */

/* ── Camera ─────────────────────────────────────────────────────────── */
/*
 * CAM_Z = 9.0 and FOV = 0.78 gives a wide view: ±7 world units wide at z=0.
 * This lets 3 bombs at x = -3.0, 0, +3.0 sit comfortably in frame.
 */
#define CAM_Z          9.0f   /* camera Z distance                             */
#define FOV_HALF_TAN   0.78f  /* tan(FOV/2); 0.78 ≈ 76° horizontal FOV        */
#define CELL_ASPECT    2.0f   /* terminal cell h/w ratio; corrects oval pixels */

/* ── Blast geometry ─────────────────────────────────────────────────── */
#define BLAST_Y       (-2.0f) /* detonation Y coordinate; near bottom of view */

#define FIREBALL_R_MAX  1.30f /* max fireball radius (world units)             */
#define STEM_HEIGHT_MAX 3.20f /* how high the stem rises above BLAST_Y         */
#define STEM_R_BASE     0.28f /* stem radius at base                           */
#define STEM_R_TOP      0.36f /* stem radius at top (slightly wider)           */
#define CAP_R_H         1.50f /* mushroom cap horizontal radius                */
#define CAP_R_V         0.50f /* mushroom cap vertical radius                  */
#define SKIRT_MAJOR     0.68f /* condensation skirt torus major radius         */
#define SKIRT_MINOR     0.19f /* condensation skirt torus tube radius          */

/* ── Surface noise ──────────────────────────────────────────────────── */
/*
 * CLOUD_DISP_AMP 0.68: high displacement = very amorphous gaseous look.
 * 3D displacement (separate noise per X/Y/Z axis) creates genuine curl/swirl.
 * CLOUD_RISE_BIAS: noise sample point drifts upward over time → rising gas feel.
 */
#define CLOUD_DISP_AMP  0.68f /* fBm displacement amplitude                    */
#define CLOUD_DISP_FREQ 1.55f /* noise frequency                               */
#define CLOUD_DISP_SPD  0.20f /* surface animation speed                       */
#define CLOUD_WARP_AMP  0.42f /* domain warp offset strength                   */
#define CLOUD_WARP_FREQ 0.85f /* domain warp spatial frequency                 */
#define CLOUD_RISE_BIAS 0.30f /* upward noise advection speed (rising gas)     */

/* ── Volumetric density shape parameters ────────────────────────────── */
#define FIREBALL_SPREAD  0.40f /* Gaussian width as fraction of fireball r²    */
#define STEM_SPREAD      1.20f /* stem Gaussian looser than tight cylinder      */
#define CAP_FALLOFF      1.60f /* cap edge sharpness                            */
#define SKIRT_SPREAD     1.40f /* skirt torus tube looseness                    */

/* ── Corona / glow ──────────────────────────────────────────────────── */
#define CORONA_BRIGHT  0.50f  /* thin-smoke fringe glow brightness              */

/* ── Shockwave ──────────────────────────────────────────────────────── */
#define SHOCKWAVE_SPD  3.8f   /* ring expansion speed (world units / sec)      */

/* ── Particles ──────────────────────────────────────────────────────── */
#define DEBRIS_N       60     /* debris particles per detonation               */
#define EMBER_N        10     /* embers spawned per second during RISING       */
#define PART_GRAVITY  (-3.2f) /* gravitational acceleration (world units/s²)  */
#define DEBRIS_SPD     3.5f   /* debris initial speed (world units/sec)        */
#define EMBER_SPD      0.55f  /* ember upward drift speed                      */

/* ── Phase durations (seconds) ──────────────────────────────────────── */
#define DUR_FLASH     0.30f
#define DUR_FIREBALL  1.80f
#define DUR_RISING    2.80f
#define DUR_CAP_FORM  2.20f
#define DUR_MUSHROOM  4.00f
#define DUR_PLATEAU   16.0f   /* bomb auto-deactivates after this              */

/* ── Timing ─────────────────────────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock — monotonic timer + sleep                                    */
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
/* §3  color — nuclear heat + smoke palette                               */
/*                                                                        */
/*   1..24  heat gradient: dark smoke → red → orange → yellow → white   */
/*   25     flash (bright white)                                         */
/*   26     shockwave ring (bright cyan-white)                           */
/*   27     debris spark (orange)                                         */
/*   28     ember (dim orange)                                            */
/*   29     corona glow (yellow-orange)                                  */
/*   30     HUD — always bright white                                    */
/* ===================================================================== */

#define N_HEAT_PAIRS  24
#define CP_FLASH      25
#define CP_SHOCK      26
#define CP_DEBRIS     27
#define CP_EMBER      28
#define CP_CORONA     29
#define CP_HUD        30

static const short k_heat_pal[N_HEAT_PAIRS] = {
    236, 238, 240, 242, 244, 247,   /* smoke  */
    248, 130,  88, 124, 160, 166,   /* embers */
    196, 202, 208, 214, 220, 226,   /* fire   */
    227, 228, 229, 230, 231,  15,   /* plasma */
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        for (int i = 0; i < N_HEAT_PAIRS; i++)
            init_pair((short)(i + 1), k_heat_pal[i], -1);
        init_pair(CP_FLASH,  231, -1);
        init_pair(CP_SHOCK,  123, -1);
        init_pair(CP_DEBRIS, 214, -1);
        init_pair(CP_EMBER,  208, -1);
        init_pair(CP_CORONA, 220, -1);
        init_pair(CP_HUD,    231, -1);
    } else {
        for (int i = 0; i < 6;  i++) init_pair((short)(i+1), COLOR_WHITE,   -1);
        for (int i = 6; i < 12; i++) init_pair((short)(i+1), COLOR_RED,     -1);
        for (int i = 12;i < 18; i++) init_pair((short)(i+1), COLOR_YELLOW,  -1);
        for (int i = 18;i < 24; i++) init_pair((short)(i+1), COLOR_WHITE,   -1);
        init_pair(CP_FLASH,  COLOR_WHITE,  -1);
        init_pair(CP_SHOCK,  COLOR_CYAN,   -1);
        init_pair(CP_DEBRIS, COLOR_YELLOW, -1);
        init_pair(CP_EMBER,  COLOR_RED,    -1);
        init_pair(CP_CORONA, COLOR_YELLOW, -1);
        init_pair(CP_HUD,    COLOR_WHITE,  -1);
    }
}

static attr_t heat_attr(float heat, char *ch_out)
{
    if (heat < 0.f) heat = 0.f;
    if (heat > 1.f) heat = 1.f;
    static const char ramp[] = ".,;:+ox#$@";
    int ri = (int)(heat * 9.f); if (ri > 9) ri = 9;
    *ch_out = ramp[ri];
    int pi = (int)(heat * (float)(N_HEAT_PAIRS - 1));
    if (pi >= N_HEAT_PAIRS) pi = N_HEAT_PAIRS - 1;
    attr_t a = COLOR_PAIR(pi + 1);
    if (heat > 0.60f) a |= A_BOLD;
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
static inline V3 v3cross(V3 a, V3 b)
{
    return v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

static inline float clmpf(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }
static inline float lerpf(float a, float b, float t)   { return a + (b - a) * t; }

static inline float smoothstep(float e0, float e1, float x)
{
    float t = clmpf((x - e0) / (e1 - e0), 0.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

static inline V3 v3rotate(V3 v, V3 k, float theta)
{
    float c = cosf(theta), s = sinf(theta);
    return v3add(v3add(v3mul(v, c), v3mul(v3cross(k, v), s)),
                 v3mul(k, v3dot(k, v) * (1.f - c)));
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

static float fbm3(V3 p)
{
    float v=0.f, amp=0.5f, freq=1.f;
    for (int i=0; i<5; i++) {
        v    += amp * (noise3(v3mul(p, freq)) - 0.5f);
        amp  *= 0.5f;
        freq *= 2.1f;
    }
    return v;
}

/* fbm3_fast — 3-octave fBm for secondary displacement axes (Y, Z).
 * Visually close to fbm3 at terminal resolution; ~40% cheaper.        */
static float fbm3_fast(V3 p)
{
    float v=0.f, amp=0.5f, freq=1.f;
    for (int i=0; i<3; i++) {
        v    += amp * (noise3(v3mul(p, freq)) - 0.5f);
        amp  *= 0.5f;
        freq *= 2.1f;
    }
    return v;
}

static float warped_fbm(V3 p, float time)
{
    V3 pa = v3add(p, v3(time*CLOUD_DISP_SPD, time*CLOUD_DISP_SPD*0.65f, 0.f));
    V3 warp = v3(
        noise3(v3add(v3mul(pa, CLOUD_WARP_FREQ), v3(1.7f, 9.2f, 0.f))),
        noise3(v3add(v3mul(pa, CLOUD_WARP_FREQ), v3(8.3f, 2.8f, 0.f))),
        noise3(v3add(v3mul(pa, CLOUD_WARP_FREQ), v3(3.1f, 5.4f, 0.f)))
    );
    V3 warped = v3add(pa, v3mul(warp, CLOUD_WARP_AMP));
    return fbm3(v3mul(warped, CLOUD_DISP_FREQ));
}

/* ===================================================================== */
/* §6  cloud density + heat fields                                        */
/*                                                                        */
/* WHY DENSITY FIELD?                                                     */
/* SDF gives hard surfaces. A density field is volumetric — semi-        */
/* transparent, soft-edged, gaseous. Each shape is a Gaussian blob.      */
/*                                                                        */
/* WHY 3D DISPLACEMENT?                                                   */
/* One scalar noise → symmetric puffing. Three independent noise samples */
/* for X/Y/Z create genuine swirling curl-like patterns. Each axis       */
/* churns independently → visually indistinguishable from real turbulence.*/
/*                                                                        */
/* ox, oz — world-space offset for this bomb instance.                   */
/* ===================================================================== */

static inline float gauss(float d2, float r2)
{
    return expf(-d2 / fmaxf(r2, 1e-6f));
}

/*
 * cloud_density() — total smoke/gas density at world point p for one bomb.
 *
 *   ox, oz     — world offset for this bomb (its blast is at (ox, BLAST_Y, oz))
 *   time       — this bomb's elapsed time (drives noise animation)
 *   fireball_r — current fireball radius
 *   fireball_y — current fireball centre Y (rises during RISING phase)
 *   stem_h     — stem height = fireball_y - BLAST_Y
 *   cap_scale  — 0..1, cap grows during CAP_FORM
 *   skirt_scale— 0..1, skirt grows during CAP_FORM
 */
static float cloud_density(V3 p, float ox, float oz, float time,
                            float fireball_r, float fireball_y,
                            float stem_h, float cap_scale, float skirt_scale)
{
    /* Transform to bomb-local space */
    V3 lp = v3(p.x - ox, p.y, p.z - oz);

    /*
     * 3D displacement: three independent noise samples for X, Y, Z.
     * Noise is sampled at a point that drifts upward (CLOUD_RISE_BIAS)
     * → turbulence appears to rise, giving the rising-gas visual feel.
     */
    V3 pa = v3(lp.x, lp.y - time * CLOUD_RISE_BIAS, lp.z);

    /* X: full 2-pass domain warp + 5-octave fBm (primary shape driver)   */
    float nx = warped_fbm(pa, time);
    /* Y/Z: cheaper 3-octave fBm — secondary axes need less detail        */
    float ny = fbm3_fast(v3add(v3mul(pa, CLOUD_DISP_FREQ), v3(5.3f, 1.7f, 3.2f)));
    float nz = fbm3_fast(v3add(v3mul(pa, CLOUD_DISP_FREQ), v3(2.9f, 8.1f, 0.7f)));

    V3 pp = v3(lp.x + nx * CLOUD_DISP_AMP,
               lp.y + ny * CLOUD_DISP_AMP * 0.80f,
               lp.z + nz * CLOUD_DISP_AMP * 0.90f);

    float d = 0.f;

    /* ── 1. Fireball — hot sphere that rises during RISING phase ─────── */
    if (fireball_r > 0.001f) {
        float dx = pp.x, dy = pp.y - fireball_y, dz = pp.z;
        float r2 = fireball_r * fireball_r;
        d += gauss(dx*dx + dy*dy + dz*dz, r2 * FIREBALL_SPREAD);
    }

    /* ── 2. Stem — Gaussian cylinder, ground → fireball ─────────────── */
    if (stem_h > 0.05f) {
        float ax2    = pp.x*pp.x + pp.z*pp.z;
        float y_frac = clmpf((pp.y - BLAST_Y) / stem_h, 0.f, 1.f);
        float R      = lerpf(STEM_R_BASE, STEM_R_TOP, y_frac);
        float ym     = smoothstep(0.f, 0.12f, y_frac) * smoothstep(1.08f, 0.82f, y_frac);
        d += gauss(ax2, R * R * STEM_SPREAD) * ym;
    }

    /* Cap and skirt removed — fire-only mode: only fireball + stem rendered */
    (void)cap_scale; (void)skirt_scale;

    return clmpf(d, 0.f, 2.f);
}

/*
 * cloud_heat() — temperature at point p for one bomb, ∈ [0, 1].
 * Independent of density: dense+hot = glowing core, dense+cold = dark smoke.
 */
static float cloud_heat(V3 p, float ox, float oz,
                         float fireball_r, float fireball_y, float stem_h)
{
    V3 lp = v3(p.x - ox, p.y, p.z - oz);
    float heat = 0.f;

    if (fireball_r > 0.001f) {
        float dx = lp.x, dy = lp.y - fireball_y, dz = lp.z;
        float r2 = fireball_r * fireball_r;
        heat = fmaxf(heat, 0.72f * gauss(dx*dx + dy*dy + dz*dz, r2 * 0.35f));
    }

    if (stem_h > 0.05f) {
        float y_frac    = clmpf((lp.y - BLAST_Y) / stem_h, 0.f, 1.f);
        float axis_r    = sqrtf(lp.x*lp.x + lp.z*lp.z);
        float axis_fall = gauss(axis_r*axis_r, STEM_R_TOP * STEM_R_TOP * 2.0f);
        heat = fmaxf(heat, lerpf(0.45f, 0.14f, y_frac) * axis_fall);
    }

    return clmpf(heat, 0.f, 1.f);
}

/* ===================================================================== */
/* §7  particles — 3D debris, embers, shockwave ring                     */
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

/* ox/oz: world offset of the bomb that's detonating */
static void particles_spawn_debris(float ox, float oz)
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
        p->x = ox; p->y = BLAST_Y; p->z = oz;
        p->vx = dir.x * spd;
        p->vy = dir.y * spd;
        p->vz = dir.z * spd;
        p->max_life = p->life = 0.8f + prand() * 1.2f;
        p->type  = PART_DEBRIS;
        p->alive = true;
    }
}

static void particles_spawn_ember(float stem_h, float ox, float oz)
{
    Particle *p = part_alloc();
    if (!p) return;
    float theta = prand() * (float)(2.0 * M_PI);
    float y_off = prand() * stem_h;
    p->x = ox + cosf(theta) * STEM_R_BASE * (0.5f + prand() * 0.5f);
    p->y = BLAST_Y + y_off;
    p->z = oz + sinf(theta) * STEM_R_BASE * (0.5f + prand() * 0.5f);
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
/* §8  simulation — NukeState + NukeInstance + multi-bomb management     */
/*                                                                        */
/* NukeState: pure physics for one bomb — phase, derived geometry.       */
/* NukeInstance: NukeState + world offset (ox, oz) + active flag.        */
/* nuke_spawn(): activates a slot with a random X position.              */
/* ===================================================================== */

typedef enum {
    NK_FLASH, NK_FIREBALL, NK_RISING, NK_CAP_FORM, NK_MUSHROOM, NK_PLATEAU
} NukePhase;

static const char *k_phase_names[] = {
    "FLASH", "FIREBALL", "RISING", "CAP FORM", "MUSHROOM", "PLATEAU"
};

static const float k_phase_dur[] = {
    DUR_FLASH, DUR_FIREBALL, DUR_RISING, DUR_CAP_FORM, DUR_MUSHROOM, DUR_PLATEAU
};

typedef struct {
    NukePhase phase;
    float     phase_t;
    float     time;
    float     time_scale;
    bool      paused;
    bool      done;          /* true when PLATEAU phase completes — deactivate */

    float     fireball_r;
    float     fireball_y;
    float     stem_h;
    float     cap_scale;
    float     skirt_scale;
    float     flash_intensity;
    float     shockwave_r;

    float     ember_accum;
    bool      debris_spawned;
} NukeState;

typedef struct {
    NukeState state;
    float     ox, oz;    /* world X, Z offset */
    bool      active;
} NukeInstance;

static NukeInstance g_nukes[MAX_NUKES];
static float        g_global_ts  = 1.0f;   /* shared time scale (+/- keys) */
static bool         g_global_pause = false;

static void sim_init(NukeState *s, float ts)
{
    memset(s, 0, sizeof *s);
    s->phase      = NK_FLASH;
    s->time_scale = ts;
}

static void sim_update_geometry(NukeState *s)
{
    float t = s->phase_t;
    switch (s->phase) {

    case NK_FLASH:
        s->flash_intensity = 1.f - smoothstep(0.f, 1.f, t);
        s->fireball_r = 0.f; s->fireball_y = BLAST_Y;
        s->stem_h = 0.f; s->cap_scale = 0.f; s->skirt_scale = 0.f;
        s->shockwave_r = 0.f;
        break;

    case NK_FIREBALL:
        s->flash_intensity = 0.f;
        s->fireball_r  = FIREBALL_R_MAX * smoothstep(0.f, 0.50f, t);
        s->fireball_y  = BLAST_Y;
        s->stem_h = 0.f; s->cap_scale = 0.f; s->skirt_scale = 0.f;
        s->shockwave_r = SHOCKWAVE_SPD * t * DUR_FIREBALL;
        break;

    case NK_RISING:
        s->flash_intensity = 0.f;
        s->fireball_r  = FIREBALL_R_MAX;
        s->fireball_y  = BLAST_Y + STEM_HEIGHT_MAX * smoothstep(0.f, 0.85f, t);
        s->stem_h      = s->fireball_y - BLAST_Y;
        s->cap_scale = 0.f; s->skirt_scale = 0.f;
        s->shockwave_r = SHOCKWAVE_SPD * (DUR_FIREBALL + t * DUR_RISING);
        break;

    case NK_CAP_FORM:
        s->flash_intensity = 0.f;
        s->fireball_r  = FIREBALL_R_MAX * lerpf(1.f, 0.65f, t);
        s->fireball_y  = BLAST_Y + STEM_HEIGHT_MAX;
        s->stem_h      = STEM_HEIGHT_MAX;
        s->cap_scale   = smoothstep(0.f, 0.85f, t);
        s->skirt_scale = smoothstep(0.3f, 1.0f,  t);
        break;

    case NK_MUSHROOM:
    case NK_PLATEAU: {
        s->flash_intensity = 0.f;
        float pulse    = 1.f + 0.03f * sinf(s->time * 2.2f);
        s->fireball_r  = FIREBALL_R_MAX * 0.62f * pulse;
        s->fireball_y  = BLAST_Y + STEM_HEIGHT_MAX * pulse;
        s->stem_h      = STEM_HEIGHT_MAX * pulse;
        s->cap_scale   = 1.f * pulse;
        s->skirt_scale = 1.f;
        break;
    }
    }
}

static void sim_tick(NukeState *s, float dt, float ox, float oz)
{
    if (s->paused || s->done) return;

    float adt = dt * s->time_scale;
    s->time   += adt;
    s->phase_t += adt / k_phase_dur[s->phase];

    if (s->phase_t >= 1.f) {
        if (s->phase < NK_PLATEAU) {
            s->phase = (NukePhase)(s->phase + 1);
            s->phase_t = 0.f;
        } else {
            /* PLATEAU finished — mark done so the instance deactivates */
            s->done = true;
            return;
        }
    }

    sim_update_geometry(s);

    if (s->phase == NK_FIREBALL && !s->debris_spawned && s->phase_t > 0.02f) {
        particles_spawn_debris(ox, oz);
        s->debris_spawned = true;
    }

    if ((s->phase == NK_RISING || s->phase == NK_MUSHROOM) && s->stem_h > 0.1f) {
        s->ember_accum += adt;
        float interval = 1.f / (float)EMBER_N;
        while (s->ember_accum >= interval) {
            particles_spawn_ember(s->stem_h, ox, oz);
            s->ember_accum -= interval;
        }
    }
}

/*
 * nuke_spawn() — activate the next available slot with a random world position.
 * If all slots are full, the oldest (furthest in time) is reused.
 */
static void nuke_spawn(void)
{
    int slot = -1;
    for (int i = 0; i < MAX_NUKES; i++)
        if (!g_nukes[i].active) { slot = i; break; }

    /* All slots full: recycle the one that's been running longest */
    if (slot < 0) {
        float oldest = -1.f;
        for (int i = 0; i < MAX_NUKES; i++) {
            if (g_nukes[i].state.time > oldest) {
                oldest = g_nukes[i].state.time;
                slot = i;
            }
        }
    }

    /* Spread bombs evenly within ±3.0 on X, slight Z variation for depth */
    static const float k_x_pos[] = { 0.f, -3.0f, 3.0f, -1.5f };
    static int spawn_idx = 0;
    float ox = k_x_pos[spawn_idx % 4];
    float oz = ((float)(spawn_idx & 3) - 1.5f) * 0.3f;
    spawn_idx++;

    sim_init(&g_nukes[slot].state, g_global_ts);
    g_nukes[slot].ox     = ox;
    g_nukes[slot].oz     = oz;
    g_nukes[slot].active = true;
}

static void nuke_reset_all(void)
{
    memset(g_parts, 0, sizeof g_parts);
    for (int i = 0; i < MAX_NUKES; i++) g_nukes[i].active = false;
    nuke_spawn();  /* always start with one */
}

/* ===================================================================== */
/* §9  raymarcher — volumetric Beer-Lambert integrator                    */
/*                                                                        */
/* rm_cast() integrates density along a ray.                              */
/* At each step, density is summed across ALL active bomb instances.      */
/* Heat is density-weighted average — overlapping clouds blend naturally. */
/*                                                                        */
/* Beer-Lambert:                                                          */
/*   sigma = total_density × step × EXTINCTION                           */
/*   dT    = exp(-sigma)          (attenuation through this slab)        */
/*   emit  = transmit × (1-dT) × heat  (emission reaching camera)       */
/*   transmit *= dT               (remaining light after this slab)      */
/* ===================================================================== */

typedef struct { float heat; float corona; bool hit; } Pixel;

static Pixel rm_cast(int px_col, int py_row, int cw, int ch,
                     const NukeInstance *nukes, int n_nukes)
{
    float u  =  ((float)px_col + 0.5f) / (float)cw * 2.f - 1.f;
    float v  = -((float)py_row + 0.5f) / (float)ch * 2.f + 1.f;
    float pa = ((float)ch * CELL_ASPECT) / (float)cw;

    V3 ro = v3(0.f, 0.f, CAM_Z);
    V3 rd = v3norm(v3(u * FOV_HALF_TAN, v * FOV_HALF_TAN * pa, -1.f));

    Pixel pix      = {0};

    /* Bounding-sphere early-exit: if the ray misses every active bomb's
     * sphere, it can't hit any cloud — skip the march entirely.          */
    {
        bool any = false;
        for (int ni = 0; ni < n_nukes && !any; ni++) {
            if (!nukes[ni].active) continue;
            V3    oc = v3sub(ro, v3(nukes[ni].ox, 0.f, nukes[ni].oz));
            float b  = v3dot(oc, rd);
            float c  = v3dot(oc, oc) - BOMB_CULL_R * BOMB_CULL_R;
            if (b*b - c >= 0.f) any = true;
        }
        if (!any) return pix;
    }

    float transmit = 1.f;
    float heat_acc = 0.f;
    float t        = 0.2f;
    const float cull_r2 = BOMB_CULL_R * BOMB_CULL_R;

    for (int i = 0; i < RM_MAX_STEPS; i++) {
        V3 p = v3add(ro, v3mul(rd, t));

        /* Sum density and heat contributions from all active bombs */
        float total_dens = 0.f;
        float w_heat     = 0.f;

        for (int ni = 0; ni < n_nukes; ni++) {
            if (!nukes[ni].active) continue;
            const NukeState *s = &nukes[ni].state;

            /* Per-bomb bounding sphere cull — skip expensive density eval
             * if this step point is clearly outside this bomb's volume.   */
            float bx = p.x - nukes[ni].ox, bz = p.z - nukes[ni].oz;
            if (bx*bx + p.y*p.y + bz*bz > cull_r2) continue;

            float d = cloud_density(p, nukes[ni].ox, nukes[ni].oz, s->time,
                                    s->fireball_r, s->fireball_y, s->stem_h,
                                    s->cap_scale, s->skirt_scale);
            if (d > DENSITY_THRESH) {
                float h = cloud_heat(p, nukes[ni].ox, nukes[ni].oz,
                                     s->fireball_r, s->fireball_y, s->stem_h);
                total_dens += d;
                w_heat     += d * h;
            }
        }

        if (total_dens > DENSITY_THRESH) {
            float avg_h = w_heat / total_dens;
            float sigma = total_dens * RM_STEP * EXTINCTION;
            float dT    = expf(-sigma);
            /* No smoke floor — only glowing fire regions contribute emission */
            heat_acc += transmit * (1.f - dT) * avg_h;
            transmit *= dT;
            if (transmit < 0.02f) break;
        }

        t += RM_STEP;
        if (t > RM_MAX_DIST) break;
    }

    float opacity = 1.f - transmit;
    /* Only render pixels with meaningful fire emission — no gray smoke */
    pix.hit    = heat_acc > 0.08f;
    /*
     * Remap into [0.28, 0.95]: maps entirely to fire/ember palette entries
     * (pairs 7–22: brown → red → orange → yellow → white).
     * Gray smoke pairs 1–6 (236–247) are never used.
     */
    pix.heat   = clmpf(0.28f + heat_acc * 0.67f, 0.f, 1.f);
    pix.corona = pix.hit ? 0.f : clmpf(opacity * CORONA_BRIGHT, 0.f, 1.f);
    return pix;
}

/* ===================================================================== */
/* §10  canvas — pixel buffer + flash/particle overlay                    */
/* ===================================================================== */

typedef struct { int w, h; Pixel *pixels; } Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows)
{
    c->w = cols; c->h = rows;
    c->pixels = calloc((size_t)(cols * rows), sizeof(Pixel));
}

static void canvas_free(Canvas *c)
{
    free(c->pixels); c->pixels = NULL; c->w = c->h = 0;
}

static void canvas_fill(Canvas *c, const NukeInstance *nukes, int n_nukes)
{
    for (int py = 0; py < c->h; py++)
        for (int px = 0; px < c->w; px++)
            c->pixels[py*c->w + px] = rm_cast(px, py, c->w, c->h, nukes, n_nukes);
}


/*
 * canvas_draw() — map pixels to ncurses, then overlay effects.
 * Draw order: raymarched cloud → shockwaves (per bomb) → particles → flash.
 */
static void canvas_draw(const Canvas *c, int tcols, int trows,
                         const NukeInstance *nukes, int n_nukes)
{
    int ox = (tcols - c->w) / 2;
    int oy = (trows - c->h) / 2;

    /* 1. Raymarched cloud */
    for (int y = 0; y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            int tx = ox + x, ty = oy + y;
            if (tx < 0 || tx >= tcols || ty < 0 || ty >= trows) continue;
            const Pixel *px = &c->pixels[y*c->w + x];
            char ch_c; attr_t a;
            if (px->hit) {
                a = heat_attr(px->heat, &ch_c);
                attron(a); mvaddch(ty, tx, (chtype)(unsigned char)ch_c); attroff(a);
            } else if (px->corona > 0.06f) {
                float ct = (px->corona - 0.06f) / (1.f - 0.06f);
                static const char cglow[] = ",;:o";
                int ci = (int)(ct * 3.9f); if (ci > 3) ci = 3;
                attr_t ca = COLOR_PAIR(CP_CORONA);
                if (ct > 0.5f) ca |= A_BOLD;
                attron(ca); mvaddch(ty, tx, (chtype)(unsigned char)cglow[ci]); attroff(ca);
            }
        }
    }

    /* 2. Flash per bomb (shockwave ring removed) */
    float max_flash = 0.f;
    for (int ni = 0; ni < n_nukes; ni++) {
        if (!nukes[ni].active) continue;
        if (nukes[ni].state.flash_intensity > max_flash)
            max_flash = nukes[ni].state.flash_intensity;
    }

    /* 3. Particles */
    particles_draw(tcols, trows, ox, oy, c->w, c->h);

    /* 4. Flash overlay — use brightest active bomb's flash */
    if (max_flash > 0.01f) {
        static const char flash_chars[] = "@#$%+*";
        int    ncov = (int)(max_flash * 5.9f);
        char   fch  = flash_chars[ncov > 5 ? 5 : ncov];
        attr_t fa   = COLOR_PAIR(CP_FLASH) | A_BOLD;
        attron(fa);
        for (int y = 0; y < trows - 2; y++)
            for (int x = 0; x < tcols; x++)
                if (prand() / 65535.f < max_flash)
                    mvaddch(y, x, (chtype)(unsigned char)fch);
        attroff(fa);
    }
}

/* ===================================================================== */
/* §11  renderer                                                          */
/* ===================================================================== */

typedef struct { Canvas canvas; } Renderer;

static void renderer_init(Renderer *r, int cols, int rows) { canvas_alloc(&r->canvas, cols, rows); }
static void renderer_free(Renderer *r)                     { canvas_free(&r->canvas); }
static void renderer_resize(Renderer *r, int cols, int rows)
{
    canvas_free(&r->canvas); canvas_alloc(&r->canvas, cols, rows);
}

static void renderer_render(Renderer *r)
{
    canvas_fill(&r->canvas, g_nukes, MAX_NUKES);
}

static void renderer_draw(const Renderer *r, int cols, int rows, double fps)
{
    canvas_draw(&r->canvas, cols, rows, g_nukes, MAX_NUKES);

    /* Count active bombs + find representative phase for HUD */
    int   n_active = 0;
    const NukeState *rep = NULL;
    for (int i = 0; i < MAX_NUKES; i++) {
        if (g_nukes[i].active) {
            n_active++;
            if (!rep) rep = &g_nukes[i].state;
        }
    }

    char buf[HUD_COLS + 1];
    if (rep) {
        snprintf(buf, sizeof buf,
                 "%.1ffps  %s  t=%.1f  x%d bombs  spd:%.1fx  %s  n:new bomb",
                 fps, k_phase_names[rep->phase], rep->time, n_active,
                 g_global_ts, g_global_pause ? "PAUSED" : "      ");
    } else {
        snprintf(buf, sizeof buf, "%.1ffps  (no active bombs)  n:detonate", fps);
    }
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, "%.*s", cols - 1, buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows - 1, 0, " q:quit  spc:pause  r:reset  +:faster  -:slower  n:detonate");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §12  screen — ncurses terminal setup / teardown                        */
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
        for (int i = 0; i < MAX_NUKES; i++)
            g_nukes[i].state.paused = g_global_pause;
        break;
    case 'r':
        nuke_reset_all();
        break;
    case 'n': case 'N':
        nuke_spawn();
        break;
    case '+': case '=':
        g_global_ts = fminf(g_global_ts + 0.5f, 16.0f);
        for (int i = 0; i < MAX_NUKES; i++) g_nukes[i].state.time_scale = g_global_ts;
        break;
    case '-':
        g_global_ts = fmaxf(g_global_ts - 0.5f, 0.25f);
        for (int i = 0; i < MAX_NUKES; i++) g_nukes[i].state.time_scale = g_global_ts;
        break;
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
    nuke_reset_all();                /* spawns first bomb at x=0 */
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

        /* Fixed-timestep accumulator */
        int64_t tick   = TICK_NS(SIM_FPS);
        float   tick_s = (float)tick / (float)NS_PER_SEC;
        sim_acc += dt;
        while (sim_acc >= tick) {
            /* Tick all active bombs; deactivate those that are done */
            for (int i = 0; i < MAX_NUKES; i++) {
                if (!g_nukes[i].active) continue;
                sim_tick(&g_nukes[i].state, tick_s, g_nukes[i].ox, g_nukes[i].oz);
                if (g_nukes[i].state.done) g_nukes[i].active = false;
            }
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
