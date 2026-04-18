/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * smoke.c  —  ncurses ASCII smoke, three physics algorithms
 *
 * ALGORITHMS (cycle with 'a'):
 *
 *   0  CA Diffusion     — Cellular automaton: density propagates upward from a
 *                         bottom source row with wide lateral jitter (±2 cols).
 *                         Like the Doom fire CA but tuned for soft, billowing
 *                         smoke spread instead of sharp upward spires.
 *
 *   1  Particle Puffs   — Pool of MAX_PARTS particles born at the source zone
 *                         with upward velocity and a random lifetime.
 *                         Density = life² (quadratic fade).  Particles are
 *                         bilinear-splatted onto the float density grid before
 *                         the shared rendering pipeline runs.
 *
 *   2  Vortex Advection — N_VORTS slowly orbiting point vortices generate a
 *                         2D velocity field via the Biot-Savart law.  Density
 *                         is advected semi-Lagrangianly each tick (bilinear
 *                         back-trace) with source injection at the bottom row.
 *                         Produces organic swirling and curling plumes.
 *
 * All three algorithms write into the same [rows × cols] float density grid.
 * The shared rendering path runs Floyd-Steinberg dithering then maps density
 * to one of 9 ASCII chars via a perceptual LUT (identical to fire.c).
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   a         next algorithm (0->1->2->0)
 *   t         next theme
 *   g  G      source intensity up / down
 *   w  W      wind right / left
 *   0         calm (no wind)
 *   ]  [      sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra smoke.c -o smoke -lncurses -lm
 *
 * Sections
 * --------
 *   §1  presets       — all tunable constants, grouped by sub-system
 *   §2  clock
 *   §3  theme         — 6 palettes, Floyd-Steinberg + LUT pipeline
 *   §4  shared helpers— warmup_scale, advance_wind, arch_envelope,
 *                       seed_source_row
 *   §5  algo 0        — CA diffusion
 *   §6  algo 1        — particle puffs
 *   §7  algo 2        — vortex advection
 *   §8  scene         — owns all state, dispatches tick / draw
 *   §9  screen        — ncurses layer + HUD
 *   §10 app           — main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm 0 — CA Diffusion:
 *   Each cell (x,y) pulls from a randomly jittered column at row y+1.
 *   The horizontal offset is ±CA_JITTER_RANGE — wider than fire's ±1 —
 *   giving smoke's characteristic lateral drift and billow.  Decay is
 *   computed from screen height so the smoke column reaches CA_REACH_FRAC
 *   of the terminal, using the same adaptive formula as fire.c.
 *
 * Algorithm 1 — Particle System:
 *   Particles carry (x, y, vx, vy, life).  Each tick: turbulence random
 *   walk on vx, upward vy drift, life decreases.  density = life² gives a
 *   quadratic fade so particles are bright at birth and fade out smoothly.
 *   Bilinear splat distributes density across 4 surrounding grid cells
 *   (1-pixel tent filter) for soft-edged smoke puffs.
 *
 * Algorithm 2 — Vortex Advection:
 *   N_VORTS vortices orbit the screen centre at different radii and speeds.
 *   Each contributes a velocity at any point via 2D Biot-Savart:
 *     vx += strength × (−dy) / (r² + VORT_EPS)
 *     vy += strength × ( dx) / (r² + VORT_EPS)
 *   Semi-Lagrangian advection each tick:
 *     new_density[p] = bilinear(old_density, p − v(p)×ADV_DT) × (1−decay)
 *                    + source(p)
 *   ADV_DT = 0.8 cells/tick keeps the advection numerically stable.
 *   Wind is accumulated once in scene_tick() and passed to the algo — not
 *   accumulated again inside vortex_tick().
 *
 * Rendering (shared):
 *   Same Floyd-Steinberg + gamma-corrected LUT pipeline as fire.c.
 *   Ramp: " .,:coO0#"  (soft round chars for a smoky feel).
 *   Only cells that were non-zero last frame and are now zero get an
 *   explicit erase — same borderless diff trick as fire.c.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  presets                                                            */
/* ===================================================================== */

/* ── loop / display ─────────────────────────────────────────────────── */
enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    HUD_COLS        = 64,
    FPS_UPDATE_MS   = 500,

    N_ALGOS         =  3,
    MAX_PARTS       = 400,   /* particle pool size                          */
    N_VORTS         =  3,    /* number of orbiting vortices                 */
};

#define WIND_MAX    3        /* max wind offset in cells/tick               */

/* ── source zone (shared by all three algos) ─────────────────────────
 * The smoke source is an arch shape along the bottom row.
 *   ARCH_MARGIN_FRAC  : fraction of cols kept empty at each side edge
 *   SRC_JITTER_BASE   : minimum random multiplier on source (prevents flat line)
 *   SRC_JITTER_RANGE  : random range added on top of BASE (0 → RANGE)
 *   WARMUP_TICKS      : linear ramp 0→1 at startup so smoke builds gradually
 *   WARMUP_CAP        : warmup counter is clamped here (prevents int overflow
 *                       and keeps warmup_scale() at exactly 1.0 after startup)
 * ─────────────────────────────────────────────────────────────────────*/
#define ARCH_MARGIN_FRAC  0.06f   /* 6% of cols kept empty at each side    */
#define SRC_JITTER_BASE   0.80f   /* min random multiplier on source       */
#define SRC_JITTER_RANGE  0.20f   /* extra random range 0→0.20            */
#define WARMUP_TICKS      80      /* ramp 0→1 over first 80 ticks         */
#define WARMUP_CAP        200     /* counter capped here; scale stays 1.0  */

/* ── algo 0: CA diffusion ───────────────────────────────────────────────
 *   CA_REACH_FRAC      : smoke column targets this fraction of rows
 *   CA_DECAY_BASE_FRAC : base decay = avg_decay × this
 *   CA_DECAY_RAND_FRAC : random component = avg_decay × this
 *   CA_JITTER_RANGE    : lateral jitter in ±CA_JITTER_RANGE columns
 *                        (wider than fire's ±1 → broader billow spread)
 *   _MIN values        : floors so tiny terminals still have visible decay
 * ─────────────────────────────────────────────────────────────────────*/
#define CA_REACH_FRAC      0.60f   /* smoke reaches 60% of rows           */
#define CA_DECAY_BASE_FRAC 0.50f
#define CA_DECAY_RAND_FRAC 0.85f
#define CA_DECAY_BASE_MIN  0.008f
#define CA_DECAY_RAND_MIN  0.012f
#define CA_JITTER_RANGE    2       /* random lateral offset in ±2 columns  */

/* ── algo 1: particle puffs ─────────────────────────────────────────────
 *   PART_LIFE_MIN/RANGE  : lifetime = MIN + rand×RANGE ticks
 *   PART_VY_BASE/RANGE   : upward speed = BASE + rand×RANGE (vy negative)
 *   PART_VX_SPREAD       : birth lateral kick ±SPREAD/2
 *   PART_TURB_STEP       : per-tick turbulence on vx (random ±TURB/2)
 *   PART_VX_DAMP         : vx damping per tick
 *   SPAWN_PER_TICK       : new particles each tick
 * ─────────────────────────────────────────────────────────────────────*/
#define PART_LIFE_MIN      35.f
#define PART_LIFE_RANGE    35.f
#define PART_VY_BASE       0.25f
#define PART_VY_RANGE      0.30f
#define PART_VX_SPREAD     0.4f
#define PART_TURB_STEP     0.12f
#define PART_VX_DAMP       0.97f
#define SPAWN_PER_TICK     5

/* ── algo 2: vortex advection ───────────────────────────────────────────
 *   ADV_DT             : semi-Lagrangian time step (cells/tick); ≤1 for stability
 *   ADV_VEL_CAP        : clamp vortex velocity so back-trace ≤ 2 cells away
 *   VORT_EPS           : Biot-Savart softening (avoids singularity at centre)
 *   VORT_REACH_FRAC    : target smoke height as fraction of rows (for decay)
 *   VORT_DECAY_SCALE   : decay = (1/target) × this
 *   VORT_DECAY_MIN     : floor on decay so tiny terminals still dissipate
 *
 * Vortex orbital presets (indices match N_VORTS=3):
 *   VORT_ORB_FRACS[]   : orbit radius as fraction of cols
 *   VORT_ORB_SPDS[]    : orbital angular speed (rad/tick); negative = clockwise
 *   VORT_STRENGTHS[]   : Biot-Savart strength; positive = CCW, negative = CW
 *   VORT_INIT_ANGLES[] : starting orbital angle (radians)
 * ─────────────────────────────────────────────────────────────────────*/
#define ADV_DT            0.8f
#define ADV_VEL_CAP       2.0f
#define VORT_EPS          6.0f
#define VORT_REACH_FRAC   0.55f
#define VORT_DECAY_SCALE  0.9f
#define VORT_DECAY_MIN    0.010f

static const float VORT_ORB_FRACS[N_VORTS]   = { 0.20f,  0.30f,  0.18f };
static const float VORT_ORB_SPDS[N_VORTS]    = { 0.018f,-0.011f, 0.025f };
static const float VORT_STRENGTHS[N_VORTS]   = { 2.5f,  -1.8f,   1.4f  };
static const float VORT_INIT_ANGLES[N_VORTS] = { 0.0f,   2.1f,   4.3f  };

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
/* §3  theme + rendering pipeline                                         */
/* ===================================================================== */

/*
 * Ramp — ASCII chars ordered light-to-dense, chosen for soft smoky shapes.
 *
 *   ' '  empty / transparent
 *   '.'  wispy trail
 *   ','  thin curl
 *   ':'  light body
 *   'c'  billow edge  (round, small)
 *   'o'  billow mid   (round)
 *   'O'  dense billow
 *   '0'  opaque core
 *   '#'  thick / black smoke
 */
static const char k_ramp[] = " .,:coO0#";
#define RAMP_N (int)(sizeof k_ramp - 1)   /* 9 */

#define CP_BASE 1   /* color pair IDs: CP_BASE .. CP_BASE+RAMP_N-1 */

/*
 * LUT break points — gamma-corrected density thresholds per ramp level.
 * Bunched in the 0.2–0.7 range so mid-density billows use the most chars
 * (most visible part of a smoke column).
 */
static const float k_lut_breaks[RAMP_N] = {
    0.000f,  /* ' '  empty        */
    0.060f,  /* '.'  wisp         */
    0.150f,  /* ','  thin         */
    0.260f,  /* ':'  light        */
    0.370f,  /* 'c'  billow edge  */
    0.480f,  /* 'o'  billow mid   */
    0.600f,  /* 'O'  dense        */
    0.740f,  /* '0'  opaque       */
    0.880f,  /* '#'  thick        */
};

static int lut_index(float v)
{
    int idx = 0;
    for (int i = RAMP_N - 1; i >= 0; i--)
        if (v >= k_lut_breaks[i]) { idx = i; break; }
    return idx;
}

static float lut_midpoint(int idx)
{
    if (idx <= 0)        return 0.f;
    if (idx >= RAMP_N-1) return 1.f;
    return (k_lut_breaks[idx] + k_lut_breaks[idx+1]) * 0.5f;
}

/*
 * Themes — 6 smoke palettes.
 * All start at clearly visible mid-tones (no near-black 232–236) so even
 * the faintest wisps show up against a dark terminal background.
 *
 *   0  Gray   — medium gray → white          (classic chimney smoke)
 *   1  Soot   — cool gray gradient           (industrial, slightly darker)
 *   2  Steam  — sky blue → cyan → white      (hot steam vent)
 *   3  Toxic  — mid green → bright lime      (poisonous cloud)
 *   4  Ember  — orange → yellow → white      (fire-lit smoke)
 *   5  Arcane — violet → pink → white        (magical smoke)
 */
typedef struct {
    const char *name;
    int         fg256[RAMP_N];
    int         fg8[RAMP_N];
    attr_t      attr8[RAMP_N];
} SmokeTheme;

static const SmokeTheme k_themes[] = {
    {   /* 0  Gray */
        "gray",
        { 242, 244, 245, 247, 248, 250, 251, 253, 255 },
        { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
          COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        { A_DIM, A_DIM, A_NORMAL, A_NORMAL, A_NORMAL,
          A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 1  Soot */
        "soot",
        { 238, 240, 241, 243, 245, 247, 249, 251, 253 },
        { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
          COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        { A_DIM, A_DIM, A_NORMAL, A_NORMAL, A_NORMAL,
          A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 2  Steam */
        "steam",
        { 25, 27, 33, 39, 45, 51, 87, 159, 231 },
        { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN,
          COLOR_CYAN, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        { A_NORMAL, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD,
          A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 3  Toxic */
        "toxic",
        { 28, 34, 40, 76, 82, 118, 154, 190, 228 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
          COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW },
        { A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD,
          A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 4  Ember */
        "ember",
        { 130, 166, 172, 202, 208, 214, 220, 226, 231 },
        { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
          COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE },
        { A_NORMAL, A_BOLD, A_DIM, A_NORMAL, A_NORMAL,
          A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 5  Arcane */
        "arcane",
        { 55, 93, 99, 135, 141, 171, 183, 213, 231 },
        { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
          COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        { A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD,
          A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static void theme_apply(int t)
{
    const SmokeTheme *th = &k_themes[t];
    for (int i = 0; i < RAMP_N; i++) {
        if (COLORS >= 256)
            init_pair(CP_BASE + i, th->fg256[i], COLOR_BLACK);
        else
            init_pair(CP_BASE + i, th->fg8[i],   COLOR_BLACK);
    }
}

static void color_init(int theme) { start_color(); theme_apply(theme); }

static attr_t ramp_attr(int i, int theme)
{
    attr_t a = COLOR_PAIR(CP_BASE + i);
    if (COLORS >= 256) {
        if (i >= RAMP_N - 2) a |= A_BOLD;
    } else {
        a |= k_themes[theme].attr8[i];
    }
    return a;
}

/* ===================================================================== */
/* §4  shared helpers                                                     */
/* ===================================================================== */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/*
 * warmup_scale() — linear ramp 0→1 over the first WARMUP_TICKS ticks.
 *
 * *warmup is the counter stored in the Scene; it is incremented here and
 * clamped at WARMUP_CAP so it never wraps and the scale stays at 1.0
 * after the warmup period ends.  Call once per tick per algo function.
 */
static float warmup_scale(int *warmup)
{
    float s = (*warmup < WARMUP_TICKS) ? (float)*warmup / (float)WARMUP_TICKS : 1.f;
    (*warmup)++;
    if (*warmup > WARMUP_CAP) *warmup = WARMUP_CAP;
    return s;
}

/*
 * arch_envelope() — squared arch weight at column x.
 *
 * Returns [0, 1]: 0 at margins and edges, 1.0 at screen centre.
 * wind_acc shifts the arch so wind leans the smoke column horizontally.
 *
 *   margin = cols × ARCH_MARGIN_FRAC
 *   t = (x − margin − wind_acc) / (cols − 2×margin)   [0,1] across span
 *   edge = min(t, 1−t) × 2                             [0,1] at centre
 *   arch = edge²                                        squared → sharper base
 */
static float arch_envelope(int x, int cols, int wind_acc)
{
    float margin = (float)cols * ARCH_MARGIN_FRAC;
    float span   = (float)cols - 2.f * margin;
    float sx     = (float)x - margin - (float)wind_acc;
    float t      = (span > 0.f) ? sx / span : 0.f;
    if (t < 0.f || t > 1.f) return 0.f;
    float edge = (t < 0.5f) ? t : 1.f - t;
    float arch = edge * 2.f;
    return arch * arch;
}

/*
 * seed_source_row() — write arch-shaped source density into the bottom row.
 *
 * Used by CA and vortex algos.  Each cell gets:
 *   density[bottom][x] = intensity × arch(x) × jitter × wscale
 * Jitter in [SRC_JITTER_BASE, SRC_JITTER_BASE + SRC_JITTER_RANGE] makes
 * the source flicker naturally each tick.
 */
static void seed_source_row(float *density, int cols, int rows,
                            float intensity, int wind_acc, float wscale)
{
    int fy = rows - 1;
    for (int x = 0; x < cols; x++) {
        float arch = arch_envelope(x, cols, wind_acc);
        if (arch <= 0.f) { density[fy * cols + x] = 0.f; continue; }
        float jitter = SRC_JITTER_BASE
                     + SRC_JITTER_RANGE * ((float)rand() / RAND_MAX);
        density[fy * cols + x] = intensity * arch * jitter * wscale;
    }
}

/* ===================================================================== */
/* §5  algo 0 — CA diffusion                                              */
/* ===================================================================== */

/*
 * ca_tick() — one CA diffusion step.
 *
 * 1. Warmup scale (increments scene warmup counter).
 * 2. Seed bottom row via seed_source_row().
 * 3. Propagate upward with ±CA_JITTER_RANGE lateral offset and adaptive decay.
 *
 * Adaptive decay:
 *   avg_decay = 1.0 / (rows × CA_REACH_FRAC)
 *   A cell at height CA_REACH_FRAC×rows has, on average, had enough decay
 *   applied to cool to zero — so the expected smoke top lands there.
 */
static void ca_tick(float *density, int cols, int rows,
                    float intensity, int wind_acc, int *warmup)
{
    float wscale = warmup_scale(warmup);
    seed_source_row(density, cols, rows, intensity, wind_acc, wscale);

    float target = (float)rows * CA_REACH_FRAC;
    float avg_d  = (target > 1.f) ? (1.f / target) : 1.f;
    float d_base = avg_d * CA_DECAY_BASE_FRAC;
    float d_rand = avg_d * CA_DECAY_RAND_FRAC;
    if (d_base < CA_DECAY_BASE_MIN) d_base = CA_DECAY_BASE_MIN;
    if (d_rand < CA_DECAY_RAND_MIN) d_rand = CA_DECAY_RAND_MIN;

    int jitter_span = CA_JITTER_RANGE * 2 + 1;   /* number of jitter choices */
    for (int y = 0; y < rows - 1; y++) {
        for (int x = 0; x < cols; x++) {
            int rx = x + (rand() % jitter_span) - CA_JITTER_RANGE;
            if (rx < 0)     rx = 0;
            if (rx >= cols) rx = cols - 1;

            float src   = density[(y + 1) * cols + rx];
            float decay = d_base + ((float)rand() / RAND_MAX) * d_rand;
            float v     = src - decay;
            density[y * cols + x] = (v < 0.f) ? 0.f : v;
        }
    }
}

/* ===================================================================== */
/* §6  algo 1 — particle puffs                                            */
/* ===================================================================== */

/*
 * Particle — one smoke puff.
 *
 * life starts at 1.0 and decreases by decay each tick.
 * Rendered density = life² so the puff fades quadratically (fast initial
 * opacity, long gentle tail) rather than a harsh linear cutoff.
 */
typedef struct {
    float x, y;     /* position in grid cells                      */
    float vx, vy;   /* velocity in cells/tick; vy negative = upward*/
    float life;     /* [1→0]; density drawn = life²                */
    float decay;    /* life lost per tick = 1/lifetime             */
    bool  active;
} Particle;

/*
 * particle_spawn() — birth one particle at the arch source zone.
 *
 * Spawn column is rejection-sampled weighted by arch_envelope so most
 * particles emerge from the centre of the smoke base.
 */
static void particle_spawn(Particle *p, int cols, int rows,
                            float intensity, int wind_acc, int warmup)
{
    float wscale = (warmup < WARMUP_TICKS) ? (float)warmup / (float)WARMUP_TICKS : 1.f;
    float margin = (float)cols * ARCH_MARGIN_FRAC;
    float span   = (float)cols - 2.f * margin;

    float bx = (float)cols * 0.5f;
    for (int attempt = 0; attempt < 8; attempt++) {
        float t  = (float)rand() / RAND_MAX;
        float cx = margin + t * span + (float)wind_acc;
        float edge   = (t < 0.5f) ? t : 1.f - t;
        float arch   = (edge * 2.f) * (edge * 2.f);
        float accept = arch * intensity * wscale;
        if (((float)rand() / RAND_MAX) < accept) { bx = cx; break; }
    }

    p->x      = bx;
    p->y      = (float)(rows - 1) - 0.5f;
    p->vx     = ((float)rand() / RAND_MAX - 0.5f) * PART_VX_SPREAD;
    p->vy     = -(PART_VY_BASE + ((float)rand() / RAND_MAX) * PART_VY_RANGE);
    float life_ticks = PART_LIFE_MIN + ((float)rand() / RAND_MAX) * PART_LIFE_RANGE;
    p->life   = 1.0f;
    p->decay  = 1.0f / life_ticks;
    p->active = true;
}

/*
 * particle_tick() — advance all particles one step, spawn new ones,
 *                   then bilinear-splat onto the density grid.
 *
 * Physics per particle each tick:
 *   vx += rand turbulence ±PART_TURB_STEP/2
 *   vx *= PART_VX_DAMP
 *   x += vx,  y += vy
 *   life -= decay
 *
 * Bilinear splat: distributes life² across 4 surrounding cells weighted
 * by fractional offsets (tent filter), giving soft-edged puffs.
 */
static void particle_tick(Particle *parts, int *next_idx,
                          float *density, int cols, int rows,
                          float intensity, int wind_acc, int *warmup)
{
    for (int i = 0; i < MAX_PARTS; i++) {
        Particle *p = &parts[i];
        if (!p->active) continue;

        p->vx += ((float)rand() / RAND_MAX - 0.5f) * PART_TURB_STEP;
        p->vx *= PART_VX_DAMP;
        p->x  += p->vx;
        p->y  += p->vy;
        p->life -= p->decay;

        if (p->life <= 0.f || p->x < 0.f || p->x >= (float)cols
                           || p->y < 0.f || p->y >= (float)rows)
            p->active = false;
    }

    for (int s = 0; s < SPAWN_PER_TICK; s++) {
        for (int tries = 0; tries < MAX_PARTS; tries++) {
            *next_idx = (*next_idx + 1) % MAX_PARTS;
            if (!parts[*next_idx].active) {
                particle_spawn(&parts[*next_idx], cols, rows,
                               intensity, wind_acc, *warmup);
                break;
            }
        }
    }
    warmup_scale(warmup);   /* advance counter; we don't need the return value here */

    memset(density, 0, (size_t)(cols * rows) * sizeof(float));

    for (int i = 0; i < MAX_PARTS; i++) {
        const Particle *p = &parts[i];
        if (!p->active) continue;

        float pd = p->life * p->life;   /* quadratic density fade */

        int   x0 = (int)p->x, y0 = (int)p->y;
        int   x1 = x0 + 1,    y1 = y0 + 1;
        float tx = p->x - (float)x0;
        float ty = p->y - (float)y0;

        if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows)
            density[y0*cols+x0] += pd * (1.f-tx) * (1.f-ty);
        if (x1 >= 0 && x1 < cols && y0 >= 0 && y0 < rows)
            density[y0*cols+x1] += pd * tx       * (1.f-ty);
        if (x0 >= 0 && x0 < cols && y1 >= 0 && y1 < rows)
            density[y1*cols+x0] += pd * (1.f-tx) * ty;
        if (x1 >= 0 && x1 < cols && y1 >= 0 && y1 < rows)
            density[y1*cols+x1] += pd * tx       * ty;
    }

    for (int i = 0; i < cols * rows; i++)
        if (density[i] > 1.f) density[i] = 1.f;
}

/* ===================================================================== */
/* §7  algo 2 — vortex advection                                          */
/* ===================================================================== */

/*
 * Vortex — a 2D point vortex orbiting the screen centre.
 *
 * Velocity contribution at point (px, py):
 *   dx = px − cx,  dy = py − cy
 *   vx += strength × (−dy) / (dx² + dy² + VORT_EPS)
 *   vy += strength × ( dx) / (dx² + dy² + VORT_EPS)
 *
 * VORT_EPS prevents a singularity when the sample point coincides with
 * the vortex centre.
 */
typedef struct {
    float cx, cy;     /* current centre in grid cells                 */
    float strength;   /* Biot-Savart strength (positive = CCW)        */
    float orb_r;      /* orbital radius in grid cells                 */
    float orb_a;      /* current orbital angle (radians)              */
    float orb_spd;    /* angular speed (radians/tick)                 */
} Vortex;

/*
 * vortex_init() — set up N_VORTS vortices using the preset arrays from §1.
 * Radii from VORT_ORB_FRACS[] are stored as absolute grid cells.
 */
static void vortex_init(Vortex vorts[N_VORTS], int cols, int rows)
{
    float cx = (float)cols * 0.5f;
    float cy = (float)rows * 0.5f;

    for (int i = 0; i < N_VORTS; i++) {
        vorts[i].orb_r   = VORT_ORB_FRACS[i] * (float)cols;
        vorts[i].orb_spd = VORT_ORB_SPDS[i];
        vorts[i].strength= VORT_STRENGTHS[i];
        vorts[i].orb_a   = VORT_INIT_ANGLES[i];
        vorts[i].cx = cx + vorts[i].orb_r * cosf(vorts[i].orb_a);
        vorts[i].cy = cy + vorts[i].orb_r * sinf(vorts[i].orb_a);
    }
}

static void vortex_advance_orbits(Vortex vorts[N_VORTS], int cols, int rows)
{
    float cx = (float)cols * 0.5f;
    float cy = (float)rows * 0.5f;

    for (int i = 0; i < N_VORTS; i++) {
        vorts[i].orb_a += vorts[i].orb_spd;
        vorts[i].cx = cx + vorts[i].orb_r * cosf(vorts[i].orb_a);
        vorts[i].cy = cy + vorts[i].orb_r * sinf(vorts[i].orb_a);
    }
}

/*
 * bilinear_sample() — sample float grid at non-integer position (sx, sy).
 * Clamps at boundaries (Neumann: zero gradient at edges).
 */
static float bilinear_sample(const float *grid, float sx, float sy,
                              int cols, int rows)
{
    int x0 = (int)sx, y0 = (int)sy;
    int x1 = x0 + 1,  y1 = y0 + 1;

    if (x0 < 0)     x0 = 0;
    if (x0 >= cols) x0 = cols - 1;
    if (x1 < 0)     x1 = 0;
    if (x1 >= cols) x1 = cols - 1;
    if (y0 < 0)     y0 = 0;
    if (y0 >= rows) y0 = rows - 1;
    if (y1 < 0)     y1 = 0;
    if (y1 >= rows) y1 = rows - 1;

    float tx = sx - (float)(int)sx;
    float ty = sy - (float)(int)sy;
    if (tx < 0.f) tx = 0.f;
    if (tx > 1.f) tx = 1.f;
    if (ty < 0.f) ty = 0.f;
    if (ty > 1.f) ty = 1.f;

    float v00 = grid[y0*cols+x0], v10 = grid[y0*cols+x1];
    float v01 = grid[y1*cols+x0], v11 = grid[y1*cols+x1];

    return (1.f-tx)*(1.f-ty)*v00 + tx*(1.f-ty)*v10
         + (1.f-tx)*ty      *v01 + tx*ty      *v11;
}

/*
 * vortex_tick() — one vortex advection step.
 *
 * 1. Advance vortex orbital positions.
 * 2. For each cell compute Biot-Savart velocity from all vortices.
 * 3. Semi-Lagrangian advection into work buffer:
 *      work[y*cols+x] = bilinear(density, x − vx×ADV_DT, y − vy×ADV_DT)
 *                     × (1 − decay) + source(x,y)
 * 4. Copy work → density.
 *
 * NOTE: wind_acc is already advanced by scene_tick() before this is called.
 * Do NOT accumulate wind again here.
 */
static void vortex_tick(float *density, float *work,
                        Vortex vorts[N_VORTS],
                        int cols, int rows,
                        float intensity, int wind_acc, int *warmup)
{
    float wscale = warmup_scale(warmup);
    vortex_advance_orbits(vorts, cols, rows);

    float target = (float)rows * VORT_REACH_FRAC;
    float decay  = (target > 1.f) ? (1.f / target) * VORT_DECAY_SCALE : VORT_DECAY_MIN;
    if (decay < VORT_DECAY_MIN) decay = VORT_DECAY_MIN;

    float margin = (float)cols * ARCH_MARGIN_FRAC;
    float span   = (float)cols - 2.f * margin;
    int   fy     = rows - 1;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            /* Biot-Savart: sum velocity from all vortices */
            float vx_field = 0.f, vy_field = 0.f;
            for (int vi = 0; vi < N_VORTS; vi++) {
                float dx = (float)x - vorts[vi].cx;
                float dy = (float)y - vorts[vi].cy;
                float r2 = dx*dx + dy*dy + VORT_EPS;
                vx_field += vorts[vi].strength * (-dy) / r2;
                vy_field += vorts[vi].strength * ( dx) / r2;
            }

            /* Clamp: back-trace no more than ADV_VEL_CAP cells away */
            if (vx_field >  ADV_VEL_CAP) vx_field =  ADV_VEL_CAP;
            if (vx_field < -ADV_VEL_CAP) vx_field = -ADV_VEL_CAP;
            if (vy_field >  ADV_VEL_CAP) vy_field =  ADV_VEL_CAP;
            if (vy_field < -ADV_VEL_CAP) vy_field = -ADV_VEL_CAP;

            float sx = (float)x - vx_field * ADV_DT;
            float sy = (float)y - vy_field * ADV_DT;

            float adv = bilinear_sample(density, sx, sy, cols, rows);

            /* Source injection: only on the bottom row */
            float src = 0.f;
            if (y == fy) {
                float t = ((float)x - margin - (float)wind_acc)
                        / (span > 0.f ? span : 1.f);
                if (t >= 0.f && t <= 1.f) {
                    float edge = (t < 0.5f) ? t : 1.f - t;
                    float arch = (edge * 2.f) * (edge * 2.f);
                    float jit  = SRC_JITTER_BASE
                               + SRC_JITTER_RANGE * ((float)rand() / RAND_MAX);
                    src = intensity * arch * jit * wscale;
                }
            }

            float v = adv * (1.f - decay) + src;
            if (v < 0.f) v = 0.f;
            if (v > 1.f) v = 1.f;
            work[y * cols + x] = v;
        }
    }

    memcpy(density, work, (size_t)(cols * rows) * sizeof(float));
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

typedef struct {
    float    *density;
    float    *prev_density;
    float    *work;          /* scratch: vortex advection + dither buffer  */
    int       cols, rows;

    int       algo;
    int       theme;
    int       warmup;        /* shared counter for all algos               */

    float     source;        /* source intensity [0.1, 1.0]                */
    int       wind;          /* cells/tick, positive = rightward           */
    int       wind_acc;      /* accumulated wind offset; advanced once/tick*/

    bool      paused;
    bool      needs_clear;

    Particle  parts[MAX_PARTS];
    int       part_idx;

    Vortex    vorts[N_VORTS];
} Scene;

static void scene_alloc(Scene *sc)
{
    sc->density      = calloc((size_t)(sc->cols * sc->rows), sizeof(float));
    sc->prev_density = calloc((size_t)(sc->cols * sc->rows), sizeof(float));
    sc->work         = calloc((size_t)(sc->cols * sc->rows), sizeof(float));
}

static void scene_free_bufs(Scene *sc)
{
    free(sc->density);      sc->density      = NULL;
    free(sc->prev_density); sc->prev_density = NULL;
    free(sc->work);         sc->work         = NULL;
}

static void scene_init(Scene *sc, int cols, int rows, int algo, int theme)
{
    memset(sc, 0, sizeof *sc);
    sc->cols        = cols;
    sc->rows        = rows;
    sc->algo        = algo;
    sc->theme       = theme;
    sc->source      = 0.85f;
    sc->wind        = 0;
    sc->wind_acc    = 0;
    sc->warmup      = 0;
    sc->part_idx    = 0;
    scene_alloc(sc);
    vortex_init(sc->vorts, cols, rows);
}

static void scene_resize(Scene *sc, int cols, int rows)
{
    int   algo  = sc->algo;
    int   theme = sc->theme;
    float src   = sc->source;
    int   wind  = sc->wind;
    scene_free_bufs(sc);
    sc->cols        = cols;
    sc->rows        = rows;
    sc->algo        = algo;
    sc->theme       = theme;
    sc->source      = src;
    sc->wind        = wind;
    sc->wind_acc    = 0;
    sc->warmup      = 0;
    sc->needs_clear = true;
    scene_alloc(sc);
    vortex_init(sc->vorts, cols, rows);
    memset(sc->parts, 0, sizeof sc->parts);
    sc->part_idx = 0;
}

/*
 * scene_tick() — advance wind once then dispatch to the active algo.
 *
 * Wind is accumulated here, exactly once per tick, before calling the
 * algo.  No algo function should advance wind_acc independently.
 */
static void scene_tick(Scene *sc)
{
    if (sc->paused) return;

    sc->wind_acc += sc->wind;
    if (sc->wind_acc >= sc->cols || sc->wind_acc <= -sc->cols)
        sc->wind_acc = 0;

    switch (sc->algo) {
    case 0:
        ca_tick(sc->density, sc->cols, sc->rows,
                sc->source, sc->wind_acc, &sc->warmup);
        break;
    case 1:
        particle_tick(sc->parts, &sc->part_idx,
                      sc->density, sc->cols, sc->rows,
                      sc->source, sc->wind_acc, &sc->warmup);
        break;
    case 2:
        vortex_tick(sc->density, sc->work, sc->vorts,
                    sc->cols, sc->rows,
                    sc->source, sc->wind_acc, &sc->warmup);
        break;
    }
}

/*
 * scene_draw() — Floyd-Steinberg dithered render (same as fire.c).
 *
 * Gamma-corrects density [0,1] → pow(v, 1/2.2) before dithering so that
 * mid-density billows use the most character variety.
 * Only cells drawn last frame that are now zero emit an explicit ' ' —
 * same borderless diff trick as fire.c.
 */
static void scene_draw(Scene *sc, int tcols, int trows)
{
    int    cols = sc->cols, rows = sc->rows;
    float *d    = sc->work;       /* dither scratch (safe: vortex already wrote back) */
    float *h    = sc->density;
    float *ph   = sc->prev_density;

    for (int i = 0; i < cols * rows; i++) {
        float v = h[i];
        if (v <= 0.f) { d[i] = -1.f; continue; }
        d[i] = powf(fminf(1.f, v), 1.f / 2.2f);
    }

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            int   i = y * cols + x;
            float v = d[i];
            if (x >= tcols || y >= trows) continue;

            if (v < 0.f) {
                if (ph[i] > 0.f) mvaddch(y, x, ' ');
                continue;
            }

            int   idx = lut_index(v);
            float qv  = lut_midpoint(idx);
            float err = v - qv;

            if (x+1 < cols && d[i+1] >= 0.f)
                d[i+1]       += err * (7.f/16.f);
            if (y+1 < rows) {
                if (x-1 >= 0 && d[i+cols-1] >= 0.f)
                    d[i+cols-1] += err * (3.f/16.f);
                if (d[i+cols] >= 0.f)
                    d[i+cols]   += err * (5.f/16.f);
                if (x+1 < cols && d[i+cols+1] >= 0.f)
                    d[i+cols+1] += err * (1.f/16.f);
            }

            attr_t attr = ramp_attr(idx, sc->theme);
            attron(attr);
            mvaddch(y, x, (chtype)(unsigned char)k_ramp[idx]);
            attroff(attr);
        }
    }

    float *tmp       = sc->prev_density;
    sc->prev_density = sc->density;
    sc->density      = tmp;
    memcpy(sc->density, sc->prev_density, (size_t)(cols * rows) * sizeof(float));
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int theme)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols); }

static const char *algo_name(int a)
{
    switch (a) {
    case 0: return "CA-diffusion";
    case 1: return "particles";
    case 2: return "vortex";
    default: return "?";
    }
}

static void screen_draw(Screen *s, Scene *sc, double fps, int sfps)
{
    if (sc->needs_clear) { erase(); sc->needs_clear = false; }
    scene_draw(sc, s->cols, s->rows);

    char buf[HUD_COLS + 1];
    const char *wstr = sc->wind > 0 ? ">>>" : sc->wind < 0 ? "<<<" : "---";
    snprintf(buf, sizeof buf,
             " %.1ffps [%s] algo:%s src:%.2f wind:%s %dHz ",
             fps, k_themes[sc->theme].name,
             algo_name(sc->algo), sc->source, wstr, sfps);

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_BASE + RAMP_N - 1) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_BASE + RAMP_N - 1) | A_BOLD);

    attron(COLOR_PAIR(CP_BASE + 2));
    mvprintw(1, hx, " q:quit spc:pause a:algo t:theme g/G:src w/W:wind ");
    attroff(COLOR_PAIR(CP_BASE + 2));
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

typedef struct {
    Scene  scene;
    Screen screen;
    int    sim_fps;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit  (int s) { (void)s; g_app.running    = 0; }
static void on_resize(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup  (void)  { endwin(); }

static bool app_handle_key(App *a, int ch)
{
    Scene *sc = &a->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ': sc->paused = !sc->paused; break;

    case 'a': case 'A':
        sc->algo        = (sc->algo + 1) % N_ALGOS;
        sc->warmup      = 0;
        sc->wind_acc    = 0;
        sc->needs_clear = true;
        memset(sc->parts, 0, sizeof sc->parts);
        sc->part_idx = 0;
        break;

    case 't': case 'T':
        sc->theme = (sc->theme + 1) % THEME_COUNT;
        theme_apply(sc->theme);
        sc->needs_clear = true;
        break;

    case 'g': sc->source += 0.05f; if (sc->source > 1.0f) sc->source = 1.0f; break;
    case 'G': sc->source -= 0.05f; if (sc->source < 0.1f) sc->source = 0.1f; break;

    case 'w': sc->wind++; if (sc->wind >  WIND_MAX) sc->wind =  WIND_MAX; break;
    case 'W': sc->wind--; if (sc->wind < -WIND_MAX) sc->wind = -WIND_MAX; break;
    case '0': sc->wind = 0; sc->wind_acc = 0; break;

    case ']': a->sim_fps += SIM_FPS_STEP; if (a->sim_fps > SIM_FPS_MAX) a->sim_fps = SIM_FPS_MAX; break;
    case '[': a->sim_fps -= SIM_FPS_STEP; if (a->sim_fps < SIM_FPS_MIN) a->sim_fps = SIM_FPS_MIN; break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app  = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, 0, 0);

    int64_t ft = clock_ns(), sa = 0, fa = 0;
    int     fc = 0;
    double  fpsd = 0.0;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            ft = clock_ns(); sa = 0;
        }

        int64_t now = clock_ns(), dt = now - ft;
        ft = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick = TICK_NS(app->sim_fps);
        sa += dt;
        while (sa >= tick) { scene_tick(&app->scene); sa -= tick; }

        fc++; fa += dt;
        if (fa >= FPS_UPDATE_MS * NS_PER_MS) {
            fpsd = (double)fc / ((double)fa / (double)NS_PER_SEC);
            fc = 0; fa = 0;
        }

        int64_t el = clock_ns() - ft + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - el);

        screen_draw(&app->screen, &app->scene, fpsd, app->sim_fps);
        screen_present();

        int ch;
        while ((ch = getch()) != ERR)
            if (!app_handle_key(app, ch)) { app->running = 0; break; }
    }

    scene_free_bufs(&app->scene);
    screen_free(&app->screen);
    return 0;
}
