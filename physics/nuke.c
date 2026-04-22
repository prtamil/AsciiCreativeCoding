/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/nuke.c — 2D Shockwave Propagation Demo
 *
 * Simulates an expanding circular pressure wave from a point detonation:
 *   • Scalar wave equation — radial pressure ring, reflects off walls
 *   • Cylindrical energy decay — ring dims as 1/r with distance
 *   • Terrain height-map — undulating ground, ripples on shockwave impact
 *   • Debris particle burst — radial ejecta with gravity
 *   • Ground dust cloud — spawned when ring sweeps the terrain
 *   • Screen shake — decaying sinusoidal jitter on detonation
 *   • Initial flash — full-screen white, fades in ~0.2 s
 *
 * Visual flow:  flash → bright ring → ring fades + terrain ripples →
 *               debris arc → dust drifts → quiet
 *
 * Keys: q/ESC quit  |  space/b  detonate  |  p pause  |  r reset
 * Build: gcc -std=c11 -O2 -Wall -Wextra physics/nuke.c \
 *             -o nuke -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config      — all tunable constants
 *   §2  clock       — monotonic timer + sleep
 *   §3  color       — wave/terrain/particle/flash palettes
 *   §4  wave        — 2D scalar wave field (finite-difference step)
 *   §5  terrain     — height-map generation + ripple interaction
 *   §6  particles   — debris burst + ground dust pool
 *   §7  simulation  — SimState, blast trigger, full tick
 *   §8  render      — wave + terrain + particles + effects → ncurses
 *   §9  screen      — ncurses setup / teardown / resize
 *  §10  app         — main loop
 */

/* ── HOW THIS WORKS ──────────────────────────────────────────────────────── *
 *
 *  WAVE EQUATION (§4):
 *
 *    ∂²u/∂t² = c² ∇²u  −  γ ∂u/∂t
 *
 *    u(x,y,t) — pressure at terminal cell (x,y) at time t
 *    c         — wave speed (cells/second); sets ring expansion rate
 *    ∇²u       — 5-point finite-difference Laplacian
 *    γ         — damping; converts wave energy to heat (ring fades)
 *
 *  The initial condition is a tight Gaussian pulse centred just above the
 *  terrain.  As it evolves, the positive pressure peak travels outward as
 *  a ring; a rarefaction trough (negative pressure) follows immediately
 *  behind.  This is the classic N-wave shape of a real shock.
 *
 *  CFL STABILITY:
 *    c · dt · √2 ≤ 1  →  c ≤ 42 cells/s at 60 Hz.
 *    We use 2 substeps per frame (effective dt = 1/120 s) so c can reach
 *    84 cells/s.  Default c = 28 → CFL = 28/120 · √2 ≈ 0.33  ✓
 *
 *  ENERGY DECAY:
 *    Two independent mechanisms:
 *      1. Geometric (cylindrical spreading): amplitude ∝ 1/√r.
 *         The total energy in the ring is conserved; as circumference
 *         grows, energy per unit length drops — ring dims naturally.
 *      2. Physical (damping γ): energy lost per second = γ × energy.
 *         Exponential envelope exp(−γ t) wraps the oscillation.
 *    Combined, a ring 30 cells out has ~40% of its initial peak.
 *
 *  TERRAIN INTERACTION (§5):
 *    The wave propagates over the terrain rows.  We read u at each
 *    column's terrain-surface row and push it into a per-column ripple
 *    value (terrain_d[]).  terrain_d decays with a spring restoring force
 *    (exponential decay per frame) — giving a heave-and-settle motion.
 *
 *  PARTICLES (§6):
 *    Two types spawned on each blast:
 *      DEBRIS — fast radial burst from blast centre, gravity pulls down.
 *      DUST   — slow particles spawned at terrain level as the ring
 *               sweeps past; drift laterally and upward, fade slowly.
 *
 *  SCREEN SHAKE (§7):
 *    Integer cell offset (shake_r, shake_c) applied to every mvaddch call.
 *    Magnitude = SHAKE_AMP × exp(−SHAKE_DECAY × t_since_blast).
 *    Oscillates at SHAKE_FREQ Hz so it alternates direction each half-cycle.
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
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS       = 60,
    N_SUBSTEPS    =  2,    /* wave sub-steps per frame (improves CFL margin) */
    HUD_COLS      = 120,
    FPS_MS        = 500,
    MAX_PARTS     = 240,   /* total particle pool                            */
    DEBRIS_BURST  =  80,   /* debris particles per blast                     */
    DUST_PER_COL  =   2,   /* max dust particles spawned per terrain column  */
    TERRAIN_ROWS  =   6,   /* rows reserved for terrain at the screen bottom */
};

/* ── Wave physics ────────────────────────────────────────────────────── */
/*
 * WAVE_C = 28 cells/s.  With N_SUBSTEPS=2 (dt_sub = 1/120 s):
 *   CFL = 28 / 120 × √2 ≈ 0.33  — well within stability limit of 1.
 * WAVE_DAMP = 1.2 / s gives a half-life of ln(2)/1.2 ≈ 0.58 s so the
 * ring is clearly visible for ~1.5 s then fades before reflecting back
 * from walls for the second time.
 */
#define WAVE_C          28.0f   /* propagation speed (cells/sec)              */
#define WAVE_DAMP        1.2f   /* energy damping coefficient (1/sec)         */
#define BLAST_AMP        9.0f   /* peak initial pressure (wave units)         */
#define BLAST_SIGMA      3.2f   /* Gaussian half-width of the initial pulse   */
#define NORM_SCALE      (BLAST_AMP * 0.42f) /* pressure → [0,1] denominator  */

/* ── Terrain ─────────────────────────────────────────────────────────── */
#define TERRAIN_H_MIN    1.5f   /* min terrain height (rows above floor)      */
#define TERRAIN_H_MAX    4.5f   /* max terrain height                         */
#define RIPPLE_SCALE     0.80f  /* wave-pressure → terrain displacement ratio */
#define RIPPLE_DECAY     0.88f  /* per-frame decay factor for terrain ripple  */
#define DUST_THRESH      0.60f  /* |pressure| at terrain → spawn dust         */

/* ── Particles ───────────────────────────────────────────────────────── */
#define CELL_ASPECT      2.0f   /* terminal cell h:w pixel ratio              */
#define GRAV            18.0f   /* gravity (cells/sec²)                       */
#define DEBRIS_SPD_MIN  10.0f   /* debris min initial speed (cells/sec)       */
#define DEBRIS_SPD_MAX  30.0f   /* debris max initial speed                   */
#define DUST_SPD_X       3.5f   /* dust lateral drift speed                   */
#define DUST_SPD_Y       2.0f   /* dust upward drift speed                    */

/* ── Screen shake ────────────────────────────────────────────────────── */
#define SHAKE_AMP        2.2f   /* initial shake magnitude (cells)            */
#define SHAKE_DECAY      3.8f   /* exponential decay rate (1/sec)             */
#define SHAKE_FREQ      13.0f   /* oscillation frequency (rad/sec)            */

/* ── Flash ───────────────────────────────────────────────────────────── */
#define FLASH_DECAY      8.0f   /* flash fade rate (1/sec)                    */

/* ── Mushroom cloud overlay ──────────────────────────────────────────── */
#define CP_SMOKE        17     /* cloud smoke/gray color pair                */
#define DUR_CL_FIREBALL  1.5f  /* fireball expansion phase (sec)             */
#define DUR_CL_RISING    3.0f  /* fireball-rise + stem growth phase (sec)    */
#define DUR_CL_CAPFORM   2.0f  /* mushroom cap formation phase (sec)         */
#define DUR_CL_MUSHROOM  3.5f  /* full-mushroom hold phase (sec)             */
#define DUR_CL_FADE      5.0f  /* fade-out phase (sec)                       */
#define CL_FB_R_MAX      6.0f  /* max fireball radius (cols)                 */
#define CL_FB_RISE      12.0f  /* rows the fireball rises upward             */
#define CL_STEM_W        2.2f  /* stem half-width (cols)                     */
#define CL_CAP_RH        9.0f  /* cap horizontal radius (cols)               */
#define CL_CAP_RV        4.0f  /* cap vertical radius (rows)                 */

/* ── Timing ─────────────────────────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define FRAME_NS    (NS_PER_SEC / SIM_FPS)

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
/* §3  color — 6 themes, cycled with 't'                                 */
/*                                                                        */
/* Color pair layout (same slots for every theme):                       */
/*   1–8   pressure ramp — 8-step gradient, dim → bright                */
/*   9     rarefaction trough (trailing negative wave)                   */
/*   10    terrain body                                                   */
/*   11    terrain surface (undisturbed)                                 */
/*   12    terrain displaced (energy-impact highlight)                   */
/*   13    debris sparks                                                  */
/*   14    dust cloud                                                     */
/*   15    flash burst                                                    */
/*   16    HUD text                                                       */
/*   17    mushroom cloud smoke                                           */
/* ===================================================================== */

#define N_PRESS   8
#define CP_RAREFY 9
#define CP_TBASE  10
#define CP_TSURF  11
#define CP_TDISP  12
#define CP_DEBRIS 13
#define CP_DUST   14
#define CP_FLASH  15
#define CP_HUD    16

typedef struct {
    const char *name;
    short       press[N_PRESS]; /* pressure gradient, dim → bright     */
    short       rarefy;         /* rarefaction (negative pressure)      */
    short       tbase;          /* terrain body fill                    */
    short       tsurf;          /* terrain surface line                 */
    short       tdisp;          /* terrain ripple highlight             */
    short       debris;         /* debris sparks                        */
    short       dust;           /* dust cloud                           */
    short       flash;          /* initial flash                        */
    short       hud;            /* HUD / status line                    */
    short       smoke;          /* mushroom cloud smoke/gray            */
    /* fallback basic-color indices for terminals without 256 colors    */
    short       fb_press_lo;    /* low-end pressure (basic color)       */
    short       fb_press_hi;    /* high-end pressure (basic color)      */
    short       fb_rarefy;
    short       fb_smoke;       /* fallback smoke (basic-color term)    */
} Theme;

static const Theme k_themes[] = {

    /* ── 0  NUKE — original red-orange-white nuclear fire ─────────── */
    { "NUKE",
      { 88, 124, 160, 196, 202, 208, 220, 231 },
      18, 22, 100, 214, 226, 180, 231, 231, 245,
      COLOR_RED, COLOR_YELLOW, COLOR_BLUE, COLOR_WHITE
    },

    /* ── 1  BRIGHT — electric cyan through white, hot-magenta trough ─ */
    { "BRIGHT",
      { 39, 45, 51, 87, 123, 159, 195, 231 },
      201, 24, 31, 51, 231, 159, 231, 51, 252,
      COLOR_CYAN, COLOR_WHITE, COLOR_MAGENTA, COLOR_WHITE
    },

    /* ── 2  MATRIX — terminal-green code rain, dead-black trough ───── */
    { "MATRIX",
      { 22, 28, 34, 40, 46, 82, 118, 154 },
      17, 22, 34, 118, 118, 46, 154, 46, 22,
      COLOR_GREEN, COLOR_GREEN, COLOR_BLACK, COLOR_GREEN
    },

    /* ── 3  NOVA — deep-space blue-violet bursting to white ──────────  */
    { "NOVA",
      { 17, 19, 57, 93, 129, 165, 207, 231 },
      196, 17, 54, 141, 225, 141, 231, 225, 60,
      COLOR_BLUE, COLOR_MAGENTA, COLOR_RED, COLOR_BLUE
    },

    /* ── 4  FIRE — dark smoldering ember to blinding flame ──────────  */
    { "FIRE",
      { 52, 88, 124, 160, 196, 208, 214, 220 },
      17, 52, 88, 220, 220, 130, 220, 214, 238,
      COLOR_RED, COLOR_YELLOW, COLOR_BLACK, COLOR_WHITE
    },

    /* ── 5  TOXIC — bio-hazard acid lime, blood-red rarefaction ─────  */
    { "TOXIC",
      { 22, 58, 64, 70, 76, 82, 118, 154 },
      88, 22, 58, 82, 154, 112, 154, 82, 64,
      COLOR_GREEN, COLOR_GREEN, COLOR_RED, COLOR_GREEN
    },
};

#define N_THEMES ((int)(sizeof k_themes / sizeof *k_themes))

static int g_theme = 0;   /* current theme index; cycled by 't' key */

/*
 * color_apply() — (re)initialise all 16 color pairs for theme t.
 * Safe to call at any time; ncurses picks up changes on the next refresh.
 */
static void color_apply(int t)
{
    const Theme *th = &k_themes[t];
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        for (int i = 0; i < N_PRESS; i++)
            init_pair((short)(i + 1), th->press[i], -1);
        init_pair(CP_RAREFY, th->rarefy, -1);
        init_pair(CP_TBASE,  th->tbase,  -1);
        init_pair(CP_TSURF,  th->tsurf,  -1);
        init_pair(CP_TDISP,  th->tdisp,  -1);
        init_pair(CP_DEBRIS, th->debris, -1);
        init_pair(CP_DUST,   th->dust,   -1);
        init_pair(CP_FLASH,  th->flash,  -1);
        init_pair(CP_HUD,    th->hud,    -1);
        init_pair(CP_SMOKE,  th->smoke,  -1);
    } else {
        for (int i = 0; i < N_PRESS / 2; i++)
            init_pair((short)(i + 1), th->fb_press_lo, -1);
        for (int i = N_PRESS / 2; i < N_PRESS; i++)
            init_pair((short)(i + 1), th->fb_press_hi, -1);
        init_pair(CP_RAREFY, th->fb_rarefy,      -1);
        init_pair(CP_TBASE,  COLOR_GREEN,         -1);
        init_pair(CP_TSURF,  COLOR_GREEN,         -1);
        init_pair(CP_TDISP,  th->fb_press_hi,     -1);
        init_pair(CP_DEBRIS, th->fb_press_hi,     -1);
        init_pair(CP_DUST,   COLOR_WHITE,         -1);
        init_pair(CP_FLASH,  COLOR_WHITE,         -1);
        init_pair(CP_HUD,    COLOR_WHITE,         -1);
        init_pair(CP_SMOKE,  th->fb_smoke,        -1);
    }
}

/* Map normalised pressure [0,1] → color pair + ASCII char */
static void press_glyph(float norm, attr_t *attr_out, char *ch_out)
{
    static const char ramp[] = ".,;:+*#@";   /* 8 levels, index 0=dim */
    int idx = (int)(norm * (N_PRESS - 1));
    if (idx < 0) idx = 0;
    if (idx >= N_PRESS) idx = N_PRESS - 1;
    *ch_out   = ramp[idx];
    *attr_out = COLOR_PAIR(idx + 1);
    if (idx >= 5) *attr_out |= A_BOLD;
}

/* ===================================================================== */
/* §4  wave — 2D scalar wave field                                        */
/*                                                                        */
/* Uses the velocity-field (symplectic Euler) integrator:                */
/*   v += ( c² · ∇²u  −  γ · v ) · dt                                  */
/*   u += v · dt                                                          */
/*                                                                        */
/* Interior cells only.  Border cells stay at u=0 (Dirichlet BC):       */
/* waves reflect with phase inversion — you see the ring bounce back.    */
/* ===================================================================== */

static void wave_step(float *u, float *v, int rows, int cols, float dt)
{
    float c2   = WAVE_C * WAVE_C;
    float damp = WAVE_DAMP;

    /* Update velocity from Laplacian */
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            int   i   = r * cols + c;
            float lap = u[(r-1)*cols + c] + u[(r+1)*cols + c]
                      + u[r*cols + (c-1)] + u[r*cols + (c+1)]
                      - 4.f * u[i];
            v[i] += (c2 * lap - damp * v[i]) * dt;
        }
    }
    /* Apply displacement */
    for (int r = 1; r < rows - 1; r++)
        for (int c = 1; c < cols - 1; c++)
            u[r*cols + c] += v[r*cols + c] * dt;
}

/* Inject a Gaussian pressure pulse centred at (cx, cy) */
static void wave_blast(float *u, int rows, int cols, int cx, int cy)
{
    int radius = (int)(BLAST_SIGMA * 4.f);
    float sig2 = BLAST_SIGMA * BLAST_SIGMA;

    for (int r = cy - radius; r <= cy + radius; r++) {
        if (r < 0 || r >= rows) continue;
        for (int c = cx - radius; c <= cx + radius; c++) {
            if (c < 0 || c >= cols) continue;
            /* Correct for cell aspect ratio so the ring is circular  */
            float dy = (r - cy) / CELL_ASPECT;
            float dx = (float)(c - cx);
            float d2 = dx*dx + dy*dy;
            u[r*cols + c] += BLAST_AMP * expf(-d2 / (2.f * sig2));
        }
    }
}

/* ===================================================================== */
/* §5  terrain — undulating ground + ripple on wave impact               */
/*                                                                        */
/* terrain_h[c]  — static height (rows tall) of ground at column c      */
/* terrain_d[c]  — dynamic ripple displacement (positive = heave up)    */
/*                                                                        */
/* Each frame: read wave field at the terrain surface row; scale and     */
/* add to terrain_d[].  terrain_d decays toward 0 (restoring force).    */
/* ===================================================================== */

static void terrain_init(float *h, int cols)
{
    for (int c = 0; c < cols; c++) {
        h[c] = 2.8f
             + 1.2f * sinf(c * 0.09f)
             + 0.7f * sinf(c * 0.19f + 1.4f)
             + 0.3f * sinf(c * 0.41f + 0.7f);
        if (h[c] < TERRAIN_H_MIN) h[c] = TERRAIN_H_MIN;
        if (h[c] > TERRAIN_H_MAX) h[c] = TERRAIN_H_MAX;
    }
}

/*
 * Returns the screen row of the terrain surface at column c.
 * rows-1 is the bottom row; terrain rises from there.
 */
static inline int terrain_surf_row(int rows, float h, float d)
{
    int top = rows - 1 - (int)(h + d + 0.5f);
    if (top < 0) top = 0;
    return top;
}

/* ===================================================================== */
/* §6  particles — radial debris burst + terrain dust cloud              */
/* ===================================================================== */

typedef enum { PT_DEBRIS, PT_DUST } PartType;

typedef struct {
    float    x, y;
    float    vx, vy;
    float    life, max_life;
    PartType type;
    bool     alive;
} Particle;

static unsigned g_pseed = 0x3A7C1F9Du;

static float prand(void)
{
    g_pseed = g_pseed * 1664525u + 1013904223u;
    return (float)((g_pseed >> 8) & 0xFFFFFF) / (float)0x1000000;
}

static Particle *part_alloc(Particle *pool, int n)
{
    for (int i = 0; i < n; i++)
        if (!pool[i].alive) return &pool[i];
    return NULL;
}

static void spawn_debris(Particle *pool, int n, float cx, float cy)
{
    for (int i = 0; i < DEBRIS_BURST; i++) {
        Particle *p = part_alloc(pool, n);
        if (!p) break;
        float angle = prand() * 2.f * (float)M_PI;
        float spd   = DEBRIS_SPD_MIN + prand() * (DEBRIS_SPD_MAX - DEBRIS_SPD_MIN);
        p->x        = cx;
        p->y        = cy;
        p->vx       = cosf(angle) * spd;
        p->vy       = sinf(angle) * spd * (1.f / CELL_ASPECT);  /* aspect-correct */
        p->vy      -= spd * 0.3f;          /* bias upward                */
        p->max_life = p->life = 1.2f + prand() * 1.5f;
        p->type     = PT_DEBRIS;
        p->alive    = true;
    }
}

static void spawn_dust(Particle *pool, int n, float cx, float cy)
{
    Particle *p = part_alloc(pool, n);
    if (!p) return;
    float dir   = (prand() > 0.5f) ? 1.f : -1.f;
    p->x        = cx + (prand() - 0.5f) * 3.f;
    p->y        = cy;
    p->vx       = dir * DUST_SPD_X * (0.4f + prand() * 0.8f);
    p->vy       = -DUST_SPD_Y * (0.5f + prand() * 0.8f);   /* upward */
    p->max_life = p->life = 1.5f + prand() * 2.0f;
    p->type     = PT_DUST;
    p->alive    = true;
}

static void particles_tick(Particle *pool, int n, float dt)
{
    for (int i = 0; i < n; i++) {
        Particle *p = &pool[i];
        if (!p->alive) continue;
        if (p->type == PT_DEBRIS)
            p->vy += GRAV * dt / CELL_ASPECT;   /* gravity, aspect-corrected */
        else
            p->vy -= 0.5f * dt;                 /* dust: slow upward float   */
        p->x    += p->vx * dt;
        p->y    += p->vy * dt;
        p->life -= dt;
        if (p->life <= 0.f) p->alive = false;
    }
}

/* ===================================================================== */
/* §6b cloud — mushroom cloud phase machine + cell renderer              */
/*                                                                        */
/* Visual anatomy:                                                        */
/*   Phase FIREBALL : bright sphere swells from blast origin             */
/*   Phase RISING   : sphere rises; narrow stem forms beneath it        */
/*   Phase CAPFORM  : oblate ellipsoid (cap) grows from the top          */
/*   Phase MUSHROOM : iconic silhouette holds, turbulence churns it     */
/*   Phase FADE     : opacity ramps to 0 and cloud dissolves             */
/*                                                                        */
/* Each cell inside the cloud boundary gets a heat value [0,1]:         */
/*   heat = max(fireball_heat, stem_heat, cap_heat)                      */
/* mapped to a fire palette color pair (pairs 1–8) or CP_SMOKE.         */
/* Sin-wave turbulence + wave-field pressure warp the boundary.          */
/* ===================================================================== */

typedef enum {
    CL_FIREBALL, CL_RISING, CL_CAPFORM, CL_MUSHROOM, CL_FADE, CL_DONE
} CloudPhase;

typedef struct {
    CloudPhase phase;
    float      phase_t;      /* progress [0,1] within current phase        */
    float      total_time;   /* total elapsed since cloud_init             */
    int        cx;           /* blast column                               */
    int        cy_base;      /* blast row (ground level)                   */
    float      fireball_r;   /* current fireball radius (cols)             */
    float      fireball_y;   /* current fireball centre row (decreases=up) */
    float      stem_h;       /* current stem height (rows)                 */
    float      cap_rh;       /* current cap horizontal radius (cols)       */
    float      cap_rv;       /* current cap vertical radius (rows)         */
    float      opacity;      /* overall fade multiplier [0,1]              */
    bool       active;       /* false until first cloud_init               */
} CloudState;

/* Hermite smooth-step on [edge0, edge1] */
static float cl_smoothstep(float edge0, float edge1, float x)
{
    float t = (x - edge0) / (edge1 - edge0);
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    return t * t * (3.f - 2.f * t);
}

static void cloud_init(CloudState *cl, int cx, int cy_base)
{
    memset(cl, 0, sizeof *cl);
    cl->phase      = CL_FIREBALL;
    cl->cx         = cx;
    cl->cy_base    = cy_base;
    cl->fireball_y = (float)cy_base;
    cl->opacity    = 1.f;
    cl->active     = true;
}

static void cloud_tick(CloudState *cl, float dt)
{
    if (!cl->active || cl->phase == CL_DONE) return;
    cl->total_time += dt;

    switch (cl->phase) {
        case CL_FIREBALL:
            cl->phase_t += dt / DUR_CL_FIREBALL;
            cl->fireball_r = cl_smoothstep(0.f, 1.f, cl->phase_t) * CL_FB_R_MAX;
            if (cl->phase_t >= 1.f) { cl->phase = CL_RISING; cl->phase_t = 0.f; }
            break;

        case CL_RISING: {
            cl->phase_t += dt / DUR_CL_RISING;
            float t = cl_smoothstep(0.f, 1.f, cl->phase_t);
            cl->fireball_y = (float)cl->cy_base - t * CL_FB_RISE;
            cl->stem_h     = t * CL_FB_RISE;
            if (cl->phase_t >= 1.f) { cl->phase = CL_CAPFORM; cl->phase_t = 0.f; }
            break;
        }

        case CL_CAPFORM: {
            cl->phase_t += dt / DUR_CL_CAPFORM;
            float t = cl_smoothstep(0.f, 1.f, cl->phase_t);
            cl->cap_rh = t * CL_CAP_RH;
            cl->cap_rv = t * CL_CAP_RV;
            if (cl->phase_t >= 1.f) { cl->phase = CL_MUSHROOM; cl->phase_t = 0.f; }
            break;
        }

        case CL_MUSHROOM:
            cl->phase_t += dt / DUR_CL_MUSHROOM;
            if (cl->phase_t >= 1.f) { cl->phase = CL_FADE; cl->phase_t = 0.f; }
            break;

        case CL_FADE:
            cl->phase_t += dt / DUR_CL_FADE;
            cl->opacity = 1.f - cl_smoothstep(0.f, 1.f, cl->phase_t);
            if (cl->phase_t >= 1.f) { cl->phase = CL_DONE; cl->opacity = 0.f; }
            break;

        case CL_DONE: break;
    }
}

/*
 * render_cloud — draw the cloud layer cell by cell.
 * Must be called after terrain (so cloud overlays ground) and before
 * particles (so debris flies in front of the cloud).
 */
static void render_cloud(const CloudState *cl, const float *wave_u,
                          int sim_rows, int sim_cols,
                          int trows, int tcols,
                          int shake_r, int shake_c)
{
    if (!cl->active || cl->opacity < 0.01f) return;

    float opa  = cl->opacity;
    float time = cl->total_time;

    /* Compute bounding box (cols may go negative; clamp below) */
    int r_top   = (int)(cl->fireball_y) - (int)(CL_FB_R_MAX + CL_CAP_RV + 3);
    int r_bot   = cl->cy_base;
    int c_left  = cl->cx - (int)(cl->cap_rh + CL_FB_R_MAX + 3);
    int c_right = cl->cx + (int)(cl->cap_rh + CL_FB_R_MAX + 3);

    if (r_top  < 0)          r_top  = 0;
    if (r_bot  >= sim_rows)  r_bot  = sim_rows - 1;
    if (c_left < 0)          c_left = 0;
    if (c_right >= sim_cols) c_right = sim_cols - 1;

    for (int r = r_top; r <= r_bot; r++) {
        for (int c = c_left; c <= c_right; c++) {
            float dx    = (float)(c - cl->cx);
            float dy_fb = (float)r - cl->fireball_y;   /* +ve = below fb centre */

            /* Turbulence: layered sin waves + wave-field pressure warp */
            float wave_warp = wave_u[r * sim_cols + c] * 0.10f;
            float turb = 0.22f * sinf(r * 0.72f + time * 2.3f)
                       + 0.15f * sinf(c * 1.15f + time * 1.8f)
                       + 0.10f * sinf((r + c) * 0.55f + time * 3.1f)
                       + wave_warp;

            /* ── Fireball component: expanding sphere ── */
            float d_fb   = sqrtf(dx*dx + (dy_fb * CELL_ASPECT) * (dy_fb * CELL_ASPECT));
            float fb_edge = cl->fireball_r + turb;
            float heat_fb = (fb_edge > 0.1f && d_fb < fb_edge)
                          ? (1.f - d_fb / (fb_edge + 0.01f)) : 0.f;

            /* ── Stem component: narrow column below the fireball ── */
            float heat_stem = 0.f;
            if (cl->stem_h > 0.5f && (float)r > cl->fireball_y) {
                float stem_top = cl->fireball_y;
                float stem_bot = (float)cl->cy_base;
                float sw = CL_STEM_W + fabsf(turb) * 0.35f;
                if (fabsf(dx) < sw) {
                    float fill = (1.f - fabsf(dx) / (sw + 0.01f)) * 0.75f;
                    /* Taper near top (blends into fireball) and base */
                    float tp = ((float)r - stem_top) / (stem_bot - stem_top + 0.01f);
                    float taper = cl_smoothstep(0.f, 0.12f, tp)
                                * (1.f - cl_smoothstep(0.88f, 1.f, tp));
                    heat_stem = fill * taper;
                }
            }

            /* ── Cap component: oblate ellipsoid at the top ── */
            float heat_cap = 0.f;
            if (cl->cap_rh > 0.3f) {
                float dy_cap = (float)r - cl->fireball_y;
                float rh = cl->cap_rh + turb * 0.55f;
                float rv = cl->cap_rv + 0.1f;
                float ed = (dx / (rh + 0.01f)) * (dx / (rh + 0.01f))
                         + (dy_cap * CELL_ASPECT / rv) * (dy_cap * CELL_ASPECT / rv);
                if (ed < 1.0f)
                    heat_cap = (1.f - ed) * 0.65f;
            }

            /* Max across all components, then apply opacity */
            float heat = heat_fb;
            if (heat_stem > heat) heat = heat_stem;
            if (heat_cap  > heat) heat = heat_cap;
            if (heat < 0.04f) continue;
            heat *= opa;

            /* Map heat → color pair + character */
            attr_t at;
            char   ch;
            if (heat > 0.78f) {
                ch = '@'; at = COLOR_PAIR(N_PRESS) | A_BOLD;
            } else if (heat > 0.58f) {
                ch = '#'; at = COLOR_PAIR(7) | A_BOLD;
            } else if (heat > 0.38f) {
                ch = '*'; at = COLOR_PAIR(5);
            } else if (heat > 0.20f) {
                ch = 'o'; at = COLOR_PAIR(3);
            } else if (heat > 0.10f) {
                ch = ':'; at = COLOR_PAIR(2);
            } else {
                ch = '.'; at = COLOR_PAIR(CP_SMOKE);
            }

            int tr = r + shake_r, tc = c + shake_c;
            if (tr < 0 || tr >= trows - 1 || tc < 0 || tc >= tcols) continue;
            attron(at);
            mvaddch(tr, tc, (chtype)(unsigned char)ch);
            attroff(at);
        }
    }
}

/* ===================================================================== */
/* §7  simulation — full state, blast trigger, per-frame tick            */
/* ===================================================================== */

typedef struct {
    float    *u;            /* wave displacement field [rows × cols]     */
    float    *v;            /* wave velocity field     [rows × cols]     */
    float    *terrain_h;    /* static terrain height per column          */
    float    *terrain_d;    /* dynamic ripple displacement per column    */
    Particle  parts[MAX_PARTS];
    int        rows, cols;
    float      time;
    float      shake_t;      /* time elapsed since last blast            */
    float      flash;        /* flash intensity [0,1]                    */
    int        blast_cx, blast_cy;
    bool       paused;
    CloudState cloud;        /* mushroom cloud phase machine             */
} SimState;

static SimState *sim_alloc(int rows, int cols)
{
    SimState *s = calloc(1, sizeof *s);
    s->rows      = rows;
    s->cols      = cols;
    s->u         = calloc((size_t)(rows * cols), sizeof(float));
    s->v         = calloc((size_t)(rows * cols), sizeof(float));
    s->terrain_h = calloc((size_t)cols, sizeof(float));
    s->terrain_d = calloc((size_t)cols, sizeof(float));
    s->shake_t   = 1e9f;   /* no shake initially */
    terrain_init(s->terrain_h, cols);
    return s;
}

static void sim_free(SimState *s)
{
    if (!s) return;
    free(s->u); free(s->v);
    free(s->terrain_h); free(s->terrain_d);
    free(s);
}

static SimState *sim_resize(SimState *s, int rows, int cols)
{
    sim_free(s);
    return sim_alloc(rows, cols);
}

static void sim_blast(SimState *s)
{
    int cx = s->cols / 2;
    int cy = s->rows - TERRAIN_ROWS - 3;  /* just above terrain */
    if (cy < 2) cy = 2;
    wave_blast(s->u, s->rows, s->cols, cx, cy);
    spawn_debris(s->parts, MAX_PARTS, (float)cx, (float)cy);
    cloud_init(&s->cloud, cx, cy);
    s->flash    = 1.0f;
    s->shake_t  = 0.f;
    s->blast_cx = cx;
    s->blast_cy = cy;
}

static void sim_reset(SimState *s)
{
    memset(s->u, 0, (size_t)(s->rows * s->cols) * sizeof(float));
    memset(s->v, 0, (size_t)(s->rows * s->cols) * sizeof(float));
    memset(s->terrain_d, 0, (size_t)s->cols * sizeof(float));
    memset(s->parts, 0, sizeof s->parts);
    s->time    = 0.f;
    s->flash   = 0.f;
    s->shake_t = 1e9f;
    sim_blast(s);
}

static void sim_tick(SimState *s, float dt)
{
    if (s->paused) return;

    /* ── Wave substeps ────────────────────────────────────────────────── */
    float sub_dt = dt / (float)N_SUBSTEPS;
    for (int n = 0; n < N_SUBSTEPS; n++)
        wave_step(s->u, s->v, s->rows, s->cols, sub_dt);

    /* ── Terrain ripple from wave ─────────────────────────────────────── */
    for (int c = 1; c < s->cols - 1; c++) {
        int sr = terrain_surf_row(s->rows, s->terrain_h[c], 0.f);
        if (sr >= s->rows) sr = s->rows - 1;
        float wave_at_surf = s->u[sr * s->cols + c];

        /* Integrate wave pressure into ripple displacement */
        s->terrain_d[c] += wave_at_surf * RIPPLE_SCALE * dt;
        /* Spring decay toward zero */
        s->terrain_d[c] *= powf(RIPPLE_DECAY, dt * SIM_FPS);
        /* Clamp to keep terrain visible */
        if (s->terrain_d[c] >  2.5f) s->terrain_d[c] =  2.5f;
        if (s->terrain_d[c] < -1.5f) s->terrain_d[c] = -1.5f;

        /* Spawn dust where the wavefront sweeps the surface */
        if (fabsf(wave_at_surf) > DUST_THRESH && (g_pseed & 0x1F) == 0)
            spawn_dust(s->parts, MAX_PARTS, (float)c,
                       (float)terrain_surf_row(s->rows, s->terrain_h[c],
                                               s->terrain_d[c]));
    }

    particles_tick(s->parts, MAX_PARTS, dt);
    cloud_tick(&s->cloud, dt);

    s->shake_t += dt;
    s->flash    = fmaxf(0.f, s->flash - FLASH_DECAY * dt);
    s->time    += dt;
}

/* ===================================================================== */
/* §8  render                                                             */
/* ===================================================================== */

/*
 * Compute integer screen shake offset.
 * Magnitude decays exponentially; direction oscillates at SHAKE_FREQ.
 */
static void get_shake(const SimState *s, int *sr, int *sc)
{
    float mag = SHAKE_AMP * expf(-SHAKE_DECAY * s->shake_t);
    *sr = (int)(mag * sinf(s->shake_t * SHAKE_FREQ) * 0.5f);
    *sc = (int)(mag * cosf(s->shake_t * SHAKE_FREQ));
}

static void render_frame(const SimState *s, int trows, int tcols,
                          float fps, bool paused)
{
    int shake_r, shake_c;
    get_shake(s, &shake_r, &shake_c);

    int draw_rows = s->rows;   /* rows available for simulation          */

    /* ── 1. Wave field ────────────────────────────────────────────────── */
    for (int r = 0; r < draw_rows - TERRAIN_ROWS; r++) {
        for (int c = 0; c < s->cols; c++) {
            float pres = s->u[r * s->cols + c];
            float norm = pres / NORM_SCALE;

            char  ch   = ' ';
            attr_t at  = 0;

            if (norm > 0.06f) {
                float t = fminf(norm, 1.f);
                press_glyph(t, &at, &ch);
            } else if (norm < -0.06f) {
                /* Rarefaction trough: cold, dim dot */
                ch = '.';
                at = COLOR_PAIR(CP_RAREFY);
            } else {
                continue;   /* near-zero — leave as background space */
            }

            int tr = r + shake_r, tc = c + shake_c;
            if (tr < 0 || tr >= trows - 1 || tc < 0 || tc >= tcols) continue;
            attron(at);
            mvaddch(tr, tc, (chtype)(unsigned char)ch);
            attroff(at);
        }
    }

    /* ── 2. Terrain ───────────────────────────────────────────────────── */
    for (int c = 0; c < s->cols; c++) {
        float d       = s->terrain_d[c];
        float h       = s->terrain_h[c];
        int   surf    = terrain_surf_row(s->rows, h, d);
        int   surf_nd = terrain_surf_row(s->rows, h, 0.f);
        bool  ripple  = (fabsf(d) > 0.3f);

        for (int r = surf; r < s->rows; r++) {
            int tr = r + shake_r, tc = c + shake_c;
            if (tr < 0 || tr >= trows - 1 || tc < 0 || tc >= tcols) continue;

            attr_t at;
            char   ch;
            if (r == surf) {
                /* Surface row: use displaced color if rippling */
                at = COLOR_PAIR(ripple ? CP_TDISP : CP_TSURF);
                ch = ripple ? '~' : '^';
                if (ripple) at |= A_BOLD;
            } else if (r == surf_nd && r != surf) {
                /* Original surface, now buried under ripple heave */
                at = COLOR_PAIR(CP_TBASE);
                ch = '#';
            } else {
                at = COLOR_PAIR(CP_TBASE);
                ch = (r % 2 == 0) ? '#' : '=';
            }
            attron(at);
            mvaddch(tr, tc, (chtype)(unsigned char)ch);
            attroff(at);
        }
    }

    /* ── 2b. Mushroom cloud ──────────────────────────────────────────── */
    render_cloud(&s->cloud, s->u, s->rows, s->cols,
                 trows, tcols, shake_r, shake_c);

    /* ── 3. Particles ─────────────────────────────────────────────────── */
    for (int i = 0; i < MAX_PARTS; i++) {
        const Particle *p = &s->parts[i];
        if (!p->alive) continue;

        int pr = (int)(p->y + 0.5f) + shake_r;
        int pc = (int)(p->x + 0.5f) + shake_c;
        if (pr < 0 || pr >= trows - 1 || pc < 0 || pc >= tcols) continue;

        float age    = 1.f - p->life / p->max_life;
        float bright = 1.f - age;

        char   ch;
        attr_t at;
        if (p->type == PT_DEBRIS) {
            ch = (bright > 0.6f) ? '*' : (bright > 0.3f) ? '+' : '.';
            at = COLOR_PAIR(CP_DEBRIS);
            if (bright > 0.5f) at |= A_BOLD;
        } else {
            ch = (bright > 0.5f) ? ':' : '.';
            at = COLOR_PAIR(CP_DUST);
        }
        attron(at);
        mvaddch(pr, pc, (chtype)(unsigned char)ch);
        attroff(at);
    }

    /* ── 4. Flash overlay ─────────────────────────────────────────────── */
    if (s->flash > 0.05f) {
        float f  = s->flash;
        char  ch = (f > 0.7f) ? '@' : (f > 0.4f) ? '#' : '+';
        attr_t at = COLOR_PAIR(CP_FLASH) | A_BOLD;
        /* Flood the inner region (near blast centre) with the flash */
        int radius = (int)(f * 14.f);
        for (int dr = -radius; dr <= radius; dr++) {
            for (int dc = -radius * 2; dc <= radius * 2; dc++) {
                float dist = sqrtf((float)(dr*dr) + (float)(dc*dc / 4));
                if (dist > radius) continue;
                int tr = s->blast_cy + dr + shake_r;
                int tc = s->blast_cx + dc + shake_c;
                if (tr < 0 || tr >= trows - 1 || tc < 0 || tc >= tcols) continue;
                attron(at);
                mvaddch(tr, tc, (chtype)(unsigned char)ch);
                attroff(at);
            }
        }
    }

    /* ── 5. HUD ───────────────────────────────────────────────────────── */
    {
        float cfl = WAVE_C / ((float)SIM_FPS * (float)N_SUBSTEPS)
                    * 1.41421356f;
        char buf[HUD_COLS];
        snprintf(buf, sizeof buf,
                 " nuke [%s] | t=%.1fs | %.0ffps | CFL=%.2f | "
                 "space:blast  r:reset  p:pause%s  t:theme  q:quit ",
                 k_themes[g_theme].name,
                 (double)s->time, (double)fps, (double)cfl,
                 paused ? "(ON)" : "");
        attron(COLOR_PAIR(CP_HUD) | A_BOLD);
        mvaddnstr(trows - 1, 0, buf, tcols - 1);
        attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
    }
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGWINCH) g_resize = 1;
    else               g_quit   = 1;
}

static void cleanup(void) { endwin(); }

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_apply(g_theme);

    int trows, tcols;
    getmaxyx(stdscr, trows, tcols);

    /* Simulation rows = terminal rows minus HUD row */
    SimState *s = sim_alloc(trows - 1, tcols);
    sim_blast(s);

    /* FPS counter */
    int64_t fps_timer = clock_ns();
    int     fps_frames = 0;
    float   fps_val    = 0.f;

    int64_t last = clock_ns();

    while (!g_quit) {
        /* ── Resize ──────────────────────────────────────────────────── */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, trows, tcols);
            s = sim_resize(s, trows - 1, tcols);
            sim_blast(s);
        }

        /* ── Input ───────────────────────────────────────────────────── */
        int key = getch();
        switch (key) {
            case 'q': case 27: g_quit = 1;                                  break;
            case ' ': case 'b': sim_blast(s);                               break;
            case 'r':           sim_reset(s);                               break;
            case 'p':           s->paused = !s->paused;                     break;
            case 't':
                g_theme = (g_theme + 1) % N_THEMES;
                color_apply(g_theme);
                break;
        }

        /* ── Tick ────────────────────────────────────────────────────── */
        int64_t now   = clock_ns();
        float   dt    = (float)(now - last) * 1e-9f;
        last = now;
        if (dt > 0.1f) dt = 0.1f;   /* cap at 100 ms (resize/lag guard) */

        sim_tick(s, dt);

        /* ── FPS counter ─────────────────────────────────────────────── */
        fps_frames++;
        if (now - fps_timer >= FPS_MS * NS_PER_MS) {
            float elapsed = (float)(now - fps_timer) * 1e-9f;
            fps_val    = (float)fps_frames / elapsed;
            fps_frames = 0;
            fps_timer  = now;
        }

        /* ── Draw ────────────────────────────────────────────────────── */
        erase();
        render_frame(s, trows, tcols, fps_val, s->paused);
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }

    sim_free(s);
    return 0;
}
