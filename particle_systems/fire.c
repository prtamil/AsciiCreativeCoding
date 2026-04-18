/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fire.c  —  ncurses ASCII fire, three physics algorithms
 *
 * ALGORITHMS (cycle with 'a'):
 *
 *   0  CA (Doom-style)  — cellular automaton; bottom row held at MAX_HEAT,
 *                         heat propagates upward with lateral ±1 jitter and
 *                         random per-cell decay.  Produces natural spires.
 *
 *   1  Particles        — pool of MAX_FIRE_PARTS short-lived embers splatted
 *                         onto the heat grid via a 3×3 Gaussian kernel.
 *
 *   2  Plasma           — procedural tongues from three layered sine waves;
 *                         no persistent grid state between frames.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   a         next algorithm (0->1->2->0)
 *   t         next theme
 *   g  G      fuel intensity up / down
 *   w  W      wind right / left
 *   0         calm (no wind)
 *   ]  [      sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fire.c -o fire -lncurses -lm
 *
 * Sections
 * --------
 *   §1  presets       — all tunable constants, grouped by sub-system
 *   §2  clock
 *   §3  theme         — 6 palettes, Floyd-Steinberg + LUT pipeline
 *   §4  grid          — heat grid data, alloc / free / resize
 *   §5  shared helpers— warmup_scale, advance_wind, arch_envelope,
 *                       seed_fuel_row, splat3x3
 *   §6  algo 0        — Doom CA fire
 *   §7  algo 1        — particle fire
 *   §8  algo 2        — plasma fire
 *   §9  scene         — owns grid + theme state
 *   §10 screen        — ncurses layer + HUD
 *   §11 app           — main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm 0 — Doom CA (Fabien Sanglard, 2013):
 *   Bottom row = arch-shaped fuel source (warmup-scaled on startup).
 *   Each cell (x,y) updated from bottom to top:
 *     rx = x + rand(-1, 0, +1)        lateral jitter → sideways flicker
 *     src = heat[y+1][rx]             pull from one row below
 *     heat[y][x] = max(0, src − decay)
 *   decay = base + rand×range, computed from screen height so the average
 *   flame peak reaches CA_REACH_FRAC of the terminal height.
 *
 * Algorithm 1 — Particle fire:
 *   Each particle: born at the arch source, upward vy, random lifetime.
 *   Per tick: turbulence random-walk on vx, apply velocity, fade heat.
 *   3×3 Gaussian splat deposits heat on the 9 surrounding grid cells.
 *   Kernel weights (sum=1): centre=0.25, edge-midpoints=0.125, corners=0.0625.
 *
 * Algorithm 2 — Plasma tongues:
 *   For each column x, tongue height h(x,t) = sum of 3 sine harmonics.
 *   Heat(x,y) = gradient from 1.0 at base (y=rows-1) to 0 at tongue tip.
 *   ny = y/rows so ny=0 at top, ny=1 at bottom — same direction as CA.
 *   Entire grid is recomputed from scratch each tick; no persistent state.
 *
 * Rendering (shared by all algos):
 *   heat[] → gamma correction pow(v, 1/2.2) → Floyd-Steinberg dither
 *   → perceptual LUT → ncurses color pair + ASCII character.
 *   Borderless: only cells that were hot last frame but cold now get an
 *   explicit ' ' to erase the stale character.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

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

    N_ALGOS         =  3,    /* CA, Particle, Plasma                       */
    MAX_FIRE_PARTS  = 800,   /* particle pool size                         */
};

#define MAX_HEAT    1.0f   /* physics heat ceiling; grid lives in [0, MAX_HEAT] */
#define WIND_MAX    3      /* max wind offset in cells/tick                */

/* ── source zone (shared by all three algos) ─────────────────────────
 * The fuel source is an arch shape along the bottom row.
 *   ARCH_MARGIN_FRAC : fraction of cols kept cold at each side edge
 *   FUEL_JITTER_BASE : minimum random multiplier on fuel (prevents flat line)
 *   FUEL_JITTER_RANGE: random range added on top of BASE (0 → RANGE)
 *   WARMUP_TICKS     : linear ramp 0→1 at startup so fire builds gradually
 * ─────────────────────────────────────────────────────────────────────*/
#define ARCH_MARGIN_FRAC   0.04f   /* 4% of cols kept cold at each side   */
#define FUEL_JITTER_BASE   0.82f   /* min random multiplier on fuel        */
#define FUEL_JITTER_RANGE  0.18f   /* extra random range 0→0.18           */
#define WARMUP_TICKS       80      /* ramp 0→1 over first 80 ticks        */

/* ── algo 0: CA decay ───────────────────────────────────────────────────
 * Decay is computed per-tick from screen height so the average flame peak
 * sits at CA_REACH_FRAC of terminal height.
 *   CA_DECAY_BASE_FRAC : base decay = avg_decay × this
 *   CA_DECAY_RAND_FRAC : random component = avg_decay × this
 *   _MIN values        : floors so tiny terminals still have visible decay
 * ─────────────────────────────────────────────────────────────────────*/
#define CA_REACH_FRAC      0.75f   /* flame peak targets 75% of rows      */
#define CA_DECAY_BASE_FRAC 0.55f
#define CA_DECAY_RAND_FRAC 0.90f
#define CA_DECAY_BASE_MIN  0.010f
#define CA_DECAY_RAND_MIN  0.015f

/* ── algo 1: particle fire ──────────────────────────────────────────────
 *   PART_LIFE_MIN/RANGE  : particle lifetime = MIN + rand×RANGE (ticks)
 *   PART_VY_BASE/RANGE   : initial upward speed; vy = -(BASE + rand×RANGE)
 *   PART_VX_SPREAD       : birth lateral kick in ±VX_SPREAD/2
 *   PART_TURB_STEP       : per-tick turbulence on vx (random ±TURB/2)
 *   PART_VX_DAMP         : vx damping factor per tick
 *   SPAWN_PER_TICK       : new particles each tick at full warmup
 * ─────────────────────────────────────────────────────────────────────*/
#define PART_LIFE_MIN      15.f
#define PART_LIFE_RANGE    20.f
#define PART_VY_BASE       0.5f
#define PART_VY_RANGE      0.8f
#define PART_VX_SPREAD     0.5f
#define PART_TURB_STEP     0.15f
#define PART_VX_DAMP       0.96f
#define SPAWN_PER_TICK     20

/* ── algo 2: plasma tongues ─────────────────────────────────────────────
 * tongue(x,t) = PLASMA_BASE
 *             + H1_AMP * sin(x*H1_XFREQ + t*H1_TSPD)
 *             + H2_AMP * sin(x*H2_XFREQ − t*H2_TSPD)
 *             + H3_AMP * sin(x*H3_XFREQ + t*H3_TSPD)
 *   PLASMA_TIME_STEP : how much t advances each tick (animation speed)
 *   PLASMA_BASE      : DC offset so tongue height is always > 0
 *   Hn_AMP           : amplitude of harmonic n
 *   Hn_XFREQ         : spatial frequency of harmonic n (cycles per screen width)
 *   Hn_TSPD          : time drift speed of harmonic n
 * ─────────────────────────────────────────────────────────────────────*/
#define PLASMA_TIME_STEP  0.07f
#define PLASMA_BASE       0.50f
#define PLASMA_H1_AMP     0.28f
#define PLASMA_H1_XFREQ   5.0f
#define PLASMA_H1_TSPD    2.2f
#define PLASMA_H2_AMP     0.18f
#define PLASMA_H2_XFREQ  11.0f
#define PLASMA_H2_TSPD    1.6f
#define PLASMA_H3_AMP     0.10f
#define PLASMA_H3_XFREQ   3.0f
#define PLASMA_H3_TSPD    0.7f

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
 * Ramp — ASCII chars ordered by visual ink density (sparse → dense).
 * These are the 9 display levels the dithering quantises to.
 *
 *   ' '  0%   cold / empty
 *   '.'  5%   faint ember glow
 *   ':'  14%  low flame
 *   '+'  28%  mid flame
 *   'x'  45%  hot mid
 *   '*'  52%  bright flame body
 *   'X'  60%  intense
 *   '#'  68%  near-peak heat
 *   '@'  80%  peak heat / core
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1)   /* 9 */

/*
 * LUT break points — gamma-corrected intensity thresholds for each ramp level.
 * Clustered in the 0.3–0.75 range where flame curvature is most visible.
 * Gamma correction: raw heat → pow(heat / MAX_HEAT, 1/2.2) before lookup
 * maps linear physics values to human-perceived brightness.
 */
static const float k_lut_breaks[RAMP_N] = {
    0.000f,  /* ' '  cold       */
    0.080f,  /* '.'  ember      */
    0.180f,  /* ':'  low flame  */
    0.290f,  /* '+'  mid-low    */
    0.390f,  /* 'x'  mid        */
    0.500f,  /* '*'  mid-high   */
    0.620f,  /* 'X'  hot        */
    0.750f,  /* '#'  very hot   */
    0.900f,  /* '@'  core       */
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
 * Themes — 6 fire palettes.
 *
 * Design principle: cold levels (0–2) are dim but still visible so the wisps
 * at the flame edge are readable.  Hot levels (7–8) always use bright/white
 * colors so the core burns clearly.  No near-black (232–236) entries.
 *
 * 256-color mode uses xterm indices for precise gradients.
 * 8-color mode falls back to COLOR_* + A_DIM/A_BOLD approximations.
 */
typedef struct {
    const char *name;
    int         fg256[RAMP_N];
    int         fg8[RAMP_N];
    attr_t      attr8[RAMP_N];
} FireTheme;

#define CP_BASE 1   /* color pair IDs: CP_BASE .. CP_BASE+RAMP_N-1 */

static const FireTheme k_themes[] = {
    {   /* 0  fire — classic red / orange / yellow */
        "fire",
        {  88, 124, 160, 196, 202, 208, 214, 220, 231 },
        {  COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
           COLOR_WHITE  },
        {  A_NORMAL, A_NORMAL, A_BOLD, A_BOLD,
           A_DIM,    A_NORMAL, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 1  ice — sky blue / cyan / white */
        "ice",
        {  25, 27, 33, 39, 45, 51, 87, 159, 231 },
        {  COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
           COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_WHITE,
           COLOR_WHITE  },
        {  A_NORMAL, A_BOLD, A_NORMAL, A_NORMAL,
           A_BOLD,   A_BOLD, A_BOLD,  A_BOLD, A_BOLD }
    },
    {   /* 2  plasma — violet / magenta / white */
        "plasma",
        {  55, 91, 93, 129, 165, 201, 207, 213, 231 },
        {  COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
           COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE,
           COLOR_WHITE    },
        {  A_NORMAL, A_NORMAL, A_BOLD, A_BOLD,
           A_BOLD,   A_BOLD,   A_DIM,  A_NORMAL, A_BOLD }
    },
    {   /* 3  nova — green / lime / white */
        "nova",
        {  28, 34, 40, 46, 82, 118, 154, 190, 231 },
        {  COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
           COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE,
           COLOR_WHITE  },
        {  A_NORMAL, A_BOLD, A_BOLD, A_BOLD,
           A_BOLD,   A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 4  poison — olive / yellow-green / white */
        "poison",
        {  58, 64, 70, 76, 118, 154, 184, 220, 231 },
        {  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_YELLOW,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
           COLOR_WHITE   },
        {  A_NORMAL, A_NORMAL, A_BOLD, A_BOLD,
           A_BOLD,   A_BOLD,   A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 5  gold — amber / orange / yellow */
        "gold",
        {  130, 136, 172, 178, 208, 214, 220, 226, 231 },
        {  COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
           COLOR_WHITE   },
        {  A_NORMAL, A_BOLD, A_NORMAL, A_BOLD,
           A_BOLD,   A_BOLD,  A_BOLD,  A_BOLD, A_BOLD }
    },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static void theme_apply(int t)
{
    const FireTheme *th = &k_themes[t];
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
/* §4  grid — heat grid data, alloc / free / resize                       */
/* ===================================================================== */

/*
 * FirePart — one particle in algo 1 (particle fire).
 *
 * Born at the arch source zone with upward vy.  heat fades by decay each
 * tick so the particle colour travels hot→cool through the ramp as it rises.
 */
typedef struct {
    float x, y;    /* grid-cell position                       */
    float vx, vy;  /* velocity (cells/tick); vy negative = up */
    float heat;    /* [1→0] current heat                       */
    float decay;   /* heat lost per tick = 1/lifetime          */
    bool  active;
} FirePart;

/*
 * Grid — the heat simulation.
 *
 * heat[y * cols + x]  float [0, MAX_HEAT]
 * y=0 is top of terminal, y=rows-1 is bottom (fuel row).
 */
typedef struct {
    float    *heat;       /* [rows × cols] current heat                    */
    float    *prev_heat;  /* [rows × cols] heat drawn in previous frame    */
    float    *dither;     /* [rows × cols] Floyd-Steinberg working buffer  */
    int       cols, rows;

    float     fuel;       /* fuel intensity [0.1, 1.0]                     */
    int       wind;       /* wind offset in cells/tick (−WIND_MAX..WIND_MAX)*/
    int       wind_acc;   /* accumulated wind offset for arch shifting      */
    int       theme;
    int       warmup;     /* counts up 0→WARMUP_TICKS; flame builds slowly */
    int       algo;       /* 0=CA  1=Particle  2=Plasma                    */
    float     plasma_t;   /* plasma: animation time counter                */
    FirePart  parts[MAX_FIRE_PARTS];
    int       part_idx;   /* round-robin spawn cursor                       */
} Grid;

static void grid_alloc(Grid *g, int cols, int rows)
{
    g->cols      = cols;
    g->rows      = rows;
    g->heat      = calloc((size_t)(cols * rows), sizeof(float));
    g->prev_heat = calloc((size_t)(cols * rows), sizeof(float));
    g->dither    = calloc((size_t)(cols * rows), sizeof(float));
}

static void grid_free(Grid *g)
{
    free(g->heat);
    free(g->prev_heat);
    free(g->dither);
    memset(g, 0, sizeof *g);
}

static void grid_resize(Grid *g, int cols, int rows)
{
    grid_free(g);
    grid_alloc(g, cols, rows);
}

static void grid_init(Grid *g, int cols, int rows, int theme)
{
    grid_alloc(g, cols, rows);
    g->fuel     = 1.0f;
    g->wind     = 0;
    g->wind_acc = 0;
    g->theme    = theme;
    g->warmup   = 0;
    g->algo     = 0;
    g->plasma_t = 0.f;
    g->part_idx = 0;
}

/* ===================================================================== */
/* §5  shared helpers                                                     */
/* ===================================================================== */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/*
 * warmup_scale() — linear ramp 0→1 over the first WARMUP_TICKS ticks.
 * Increments g->warmup on every call; clamps at WARMUP_TICKS so the
 * counter never wraps and the scale stays at 1.0 after warmup.
 * Call once per tick at the start of each algo function.
 */
static float warmup_scale(Grid *g)
{
    float s = (g->warmup < WARMUP_TICKS) ? (float)g->warmup / (float)WARMUP_TICKS : 1.f;
    if (g->warmup < WARMUP_TICKS) g->warmup++;
    return s;
}

/*
 * advance_wind() — shift the accumulated wind offset by one tick.
 * Wraps back to 0 when the offset exceeds the full screen width so the
 * arch doesn't disappear off one side permanently.
 */
static void advance_wind(Grid *g)
{
    g->wind_acc += g->wind;
    if (g->wind_acc >= g->cols || g->wind_acc <= -g->cols)
        g->wind_acc = 0;
}

/*
 * arch_envelope() — squared arch weight at column x.
 *
 * Returns a value in [0, 1]: 0 at the margins and edges, peaking at 1.0
 * at the screen centre.  wind_acc shifts the arch horizontally so wind
 * leans the entire flame.
 *
 *   margin = cols × ARCH_MARGIN_FRAC
 *   t = (x − margin − wind_acc) / (cols − 2×margin)   [0, 1] across span
 *   edge = min(t, 1−t) × 2                             [0, 1] at centre
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
 * seed_fuel_row() — write arch-shaped fuel values into the bottom row.
 *
 * Every cell gets arch(x) × fuel × jitter × wscale.
 * Jitter in [FUEL_JITTER_BASE, FUEL_JITTER_BASE+FUEL_JITTER_RANGE] makes
 * the fuel line flicker so no two ticks look identical at the base.
 */
static void seed_fuel_row(Grid *g, float wscale)
{
    int    cols = g->cols;
    int    fy   = g->rows - 1;
    float *h    = g->heat;

    for (int x = 0; x < cols; x++) {
        float arch = arch_envelope(x, cols, g->wind_acc);
        if (arch <= 0.f) { h[fy * cols + x] = 0.f; continue; }
        float jitter = FUEL_JITTER_BASE
                     + FUEL_JITTER_RANGE * ((float)rand() / RAND_MAX);
        h[fy * cols + x] = MAX_HEAT * g->fuel * arch * jitter * wscale;
    }
}

/*
 * splat3x3() — 3×3 Gaussian splat of value v onto the heat grid at (cx, cy).
 *
 * Kernel weights (normalised, sum=1):
 *   corners = 0.0625,  edge-midpoints = 0.125,  centre = 0.25
 *
 * Spreads each particle's heat across 9 cells, giving a much denser and
 * more filled-in flame body than a point deposit or 2×2 bilinear splat.
 */
static void splat3x3(float *heat, int cols, int rows, int cx, int cy, float v)
{
    static const float kern[3][3] = {
        { 0.0625f, 0.125f, 0.0625f },
        { 0.125f,  0.25f,  0.125f  },
        { 0.0625f, 0.125f, 0.0625f },
    };
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int nx = cx + dx, ny = cy + dy;
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows)
                heat[ny * cols + nx] += v * kern[dy+1][dx+1];
        }
    }
}

/* ===================================================================== */
/* §6  algo 0 — Doom CA fire                                              */
/* ===================================================================== */

/*
 * ca_fire_tick() — one Doom-style cellular automaton step.
 *
 * 1. Warmup scale and wind advance (shared helpers).
 * 2. Seed bottom row with arch-shaped fuel (shared helper).
 * 3. Propagate heat upward with ±1 lateral jitter and adaptive decay.
 *
 * Adaptive decay:
 *   avg_decay = MAX_HEAT / (rows × CA_REACH_FRAC)
 *   This means a cell at height CA_REACH_FRAC×rows has, on average,
 *   had just enough decay applied to cool completely — so the expected
 *   flame top lands at that fraction of the terminal height.
 */
static void ca_fire_tick(Grid *g)
{
    int    cols = g->cols;
    int    rows = g->rows;
    float *h    = g->heat;

    float wscale = warmup_scale(g);
    advance_wind(g);
    seed_fuel_row(g, wscale);

    float target = (float)rows * CA_REACH_FRAC;
    float avg_d  = (target > 1.f) ? (MAX_HEAT / target) : MAX_HEAT;
    float d_base = avg_d * CA_DECAY_BASE_FRAC;
    float d_rand = avg_d * CA_DECAY_RAND_FRAC;
    if (d_base < CA_DECAY_BASE_MIN) d_base = CA_DECAY_BASE_MIN;
    if (d_rand < CA_DECAY_RAND_MIN) d_rand = CA_DECAY_RAND_MIN;

    for (int y = 0; y < rows - 1; y++) {
        for (int x = 0; x < cols; x++) {
            int rx = x + (rand() % 3) - 1;
            if (rx < 0)     rx = 0;
            if (rx >= cols) rx = cols - 1;

            float src   = h[(y + 1) * cols + rx];
            float decay = d_base + ((float)rand() / RAND_MAX) * d_rand;
            float v     = src - decay;
            h[y * cols + x] = (v < 0.f) ? 0.f : v;
        }
    }
}

/* ===================================================================== */
/* §7  algo 1 — particle fire                                             */
/* ===================================================================== */

/*
 * fire_part_spawn() — birth one particle at the arch source zone.
 *
 * Spawn column is rejection-sampled weighted by arch_envelope so most
 * particles come from the centre of the flame base.  wscale scales down
 * acceptance during warmup so particles build up gradually.
 */
static void fire_part_spawn(FirePart *p, Grid *g, float wscale)
{
    int   cols   = g->cols;
    int   rows   = g->rows;
    float margin = (float)cols * ARCH_MARGIN_FRAC;
    float span   = (float)cols - 2.f * margin;

    float bx = (float)cols * 0.5f;
    for (int tries = 0; tries < 8; tries++) {
        float t  = (float)rand() / RAND_MAX;
        float cx = margin + t * span + (float)g->wind_acc;
        float edge = (t < 0.5f) ? t : 1.f - t;
        float arch = (edge * 2.f) * (edge * 2.f);
        if (((float)rand() / RAND_MAX) < arch * g->fuel * wscale)
            { bx = cx; break; }
    }

    p->x      = bx;
    p->y      = (float)(rows - 1);
    p->vx     = ((float)rand() / RAND_MAX - 0.5f) * PART_VX_SPREAD;
    p->vy     = -(PART_VY_BASE + ((float)rand() / RAND_MAX) * PART_VY_RANGE);
    float life = PART_LIFE_MIN + ((float)rand() / RAND_MAX) * PART_LIFE_RANGE;
    p->heat   = 1.0f;
    p->decay  = 1.0f / life;
    p->active = true;
}

/*
 * particle_fire_tick() — advance all particles, spawn new ones, splat onto grid.
 *
 * 1. Warmup scale and wind advance.
 * 2. Advance each active particle (turbulence, velocity, decay).
 * 3. Spawn SPAWN_PER_TICK × wscale new particles.
 * 4. Clear heat grid; 3×3 Gaussian splat all active particles.
 */
static void particle_fire_tick(Grid *g)
{
    int cols = g->cols, rows = g->rows;

    float wscale = warmup_scale(g);
    advance_wind(g);

    for (int i = 0; i < MAX_FIRE_PARTS; i++) {
        FirePart *p = &g->parts[i];
        if (!p->active) continue;
        p->vx   += ((float)rand() / RAND_MAX - 0.5f) * PART_TURB_STEP;
        p->vx   *= PART_VX_DAMP;
        p->x    += p->vx;
        p->y    += p->vy;
        p->heat -= p->decay;
        if (p->heat <= 0.f || p->y < 0.f
                           || p->x < 0.f || p->x >= (float)cols)
            p->active = false;
    }

    int n = (int)((float)SPAWN_PER_TICK * wscale) + 1;
    for (int s = 0; s < n; s++) {
        for (int tries = 0; tries < MAX_FIRE_PARTS; tries++) {
            g->part_idx = (g->part_idx + 1) % MAX_FIRE_PARTS;
            if (!g->parts[g->part_idx].active) {
                fire_part_spawn(&g->parts[g->part_idx], g, wscale);
                break;
            }
        }
    }

    memset(g->heat, 0, (size_t)(cols * rows) * sizeof(float));
    for (int i = 0; i < MAX_FIRE_PARTS; i++) {
        const FirePart *p = &g->parts[i];
        if (!p->active) continue;
        splat3x3(g->heat, cols, rows,
                 (int)(p->x + 0.5f), (int)(p->y + 0.5f), p->heat);
    }
    for (int i = 0; i < cols * rows; i++)
        if (g->heat[i] > 1.f) g->heat[i] = 1.f;
}

/* ===================================================================== */
/* §8  algo 2 — plasma fire                                               */
/* ===================================================================== */

/*
 * plasma_fire_tick() — procedural fire from layered sine waves.
 *
 * For each column x, a "tongue height" is computed from three overlapping
 * sine harmonics whose phase drifts in time.  Cell (x,y) heat then ramps
 * from 1.0 at the base (ny=1) to 0 at the tongue tip.
 *
 *   nx = x/cols                   normalised column position [0,1]
 *   wx = nx + wind_acc/cols       wind-shifted x for horizontal lean
 *   tongue = PLASMA_BASE
 *          + H1_AMP × sin(wx × H1_XFREQ + t × H1_TSPD)
 *          + H2_AMP × sin(wx × H2_XFREQ − t × H2_TSPD)
 *          + H3_AMP × sin(wx × H3_XFREQ + t × H3_TSPD)
 *   tongue ×= fuel × wscale × sqrt(arch)   (scaled by source envelope)
 *
 *   ny = y/rows  (0 at top, 1 at bottom — same direction as CA)
 *   heat(x,y) = clamp((ny − (1 − tongue)) / tongue, 0, 1)
 *
 * The entire grid is recomputed from scratch each tick; no carry-over state.
 */
static void plasma_fire_tick(Grid *g)
{
    int cols = g->cols, rows = g->rows;

    float wscale = warmup_scale(g);
    advance_wind(g);

    float t      = g->plasma_t;
    g->plasma_t += PLASMA_TIME_STEP;

    for (int x = 0; x < cols; x++) {
        float nx   = (float)x / (float)cols;
        float wx   = nx + (float)g->wind_acc / (float)cols;
        float arch = arch_envelope(x, cols, g->wind_acc);

        if (arch <= 0.f) {
            for (int y = 0; y < rows; y++) g->heat[y * cols + x] = 0.f;
            continue;
        }

        float tongue = PLASMA_BASE
            + PLASMA_H1_AMP * sinf(wx * PLASMA_H1_XFREQ + t * PLASMA_H1_TSPD)
            + PLASMA_H2_AMP * sinf(wx * PLASMA_H2_XFREQ - t * PLASMA_H2_TSPD)
            + PLASMA_H3_AMP * sinf(wx * PLASMA_H3_XFREQ + t * PLASMA_H3_TSPD);
        tongue = clampf(tongue, 0.f, 1.f) * g->fuel * wscale * sqrtf(arch);

        float inv_tongue = (tongue > 0.01f) ? (1.f / tongue) : 100.f;

        for (int y = 0; y < rows; y++) {
            float ny         = (float)y / (float)rows;
            float above_base = ny - (1.f - tongue);
            float heat       = clampf(above_base * inv_tongue, 0.f, 1.f);
            g->heat[y * cols + x] = heat;
        }
    }
}

/* ── dispatcher ─────────────────────────────────────────────────────── */

static void grid_tick(Grid *g)
{
    switch (g->algo) {
    case 0: ca_fire_tick(g);       break;
    case 1: particle_fire_tick(g); break;
    case 2: plasma_fire_tick(g);   break;
    }
}

/*
 * grid_draw() — borderless dithered render.
 *
 * We never call erase() and never draw ' ' for every cold cell.  Instead:
 *   - Only cells that WERE hot last frame and ARE cold now get an explicit
 *     ' ' to erase the stale character.
 *   - All other cold cells are skipped — ncurses' doupdate() leaves them
 *     unchanged, which is the correct behaviour (clear terminal background).
 *   - Hot cells go through gamma correction → Floyd-Steinberg dither → LUT.
 * After drawing, heat ↔ prev_heat are swapped so the next frame knows
 * which cells were visible.
 */
static void grid_draw(Grid *g, int tcols, int trows)
{
    int    cols = g->cols, rows = g->rows;
    float *d    = g->dither;
    float *h    = g->heat;
    float *ph   = g->prev_heat;

    for (int i = 0; i < cols * rows; i++) {
        float v = h[i];
        if (v <= 0.f) { d[i] = -1.f; continue; }
        v    = fminf(1.f, v / MAX_HEAT);
        d[i] = powf(v, 1.f / 2.2f);
    }

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            int   i  = y * cols + x;
            float v  = d[i];
            if (x >= tcols || y >= trows) continue;

            if (v < 0.f) {
                if (ph[i] > 0.f) mvaddch(y, x, ' ');
                continue;
            }

            int   idx = lut_index(v);
            float qv  = lut_midpoint(idx);
            float err = v - qv;

            if (x+1 < cols && d[i+1] >= 0.f)
                d[i+1]      += err * (7.f/16.f);
            if (y+1 < rows) {
                if (x-1 >= 0 && d[i+cols-1] >= 0.f)
                    d[i+cols-1] += err * (3.f/16.f);
                if (d[i+cols] >= 0.f)
                    d[i+cols]   += err * (5.f/16.f);
                if (x+1 < cols && d[i+cols+1] >= 0.f)
                    d[i+cols+1] += err * (1.f/16.f);
            }

            attr_t attr = ramp_attr(idx, g->theme);
            attron(attr);
            mvaddch(y, x, (chtype)(unsigned char)k_ramp[idx]);
            attroff(attr);
        }
    }

    float *tmp   = g->prev_heat;
    g->prev_heat = g->heat;
    g->heat      = tmp;
    memcpy(g->heat, g->prev_heat, (size_t)(cols * rows) * sizeof(float));
}

/* ===================================================================== */
/* §9  scene                                                              */
/* ===================================================================== */

typedef struct {
    Grid  grid;
    bool  paused;
    bool  needs_clear;
} Scene;

static void scene_init(Scene *s, int cols, int rows, int theme)
{
    memset(s, 0, sizeof *s);
    grid_init(&s->grid, cols, rows, theme);
}

static void scene_free(Scene *s) { grid_free(&s->grid); }

static void scene_resize(Scene *s, int cols, int rows)
{
    int   t    = s->grid.theme;
    float fuel = s->grid.fuel;
    int   wind = s->grid.wind;
    int   algo = s->grid.algo;
    grid_resize(&s->grid, cols, rows);
    s->grid.fuel        = fuel;
    s->grid.wind        = wind;
    s->grid.theme       = t;
    s->grid.algo        = algo;
    s->grid.warmup      = 0;
    s->needs_clear      = true;
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    grid_tick(&s->grid);
}

static void scene_draw(Scene *s, int cols, int rows)
{
    grid_draw(&s->grid, cols, rows);
}

/* ===================================================================== */
/* §10 screen                                                             */
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
    case 0: return "CA";
    case 1: return "particles";
    case 2: return "plasma";
    default: return "?";
    }
}

static void screen_draw(Screen *s, Scene *sc, double fps, int sfps)
{
    if (sc->needs_clear) { erase(); sc->needs_clear = false; }
    scene_draw(sc, s->cols, s->rows);

    const Grid *g = &sc->grid;
    char buf[HUD_COLS + 1];
    const char *wstr = g->wind > 0 ? ">>>" : g->wind < 0 ? "<<<" : "---";
    snprintf(buf, sizeof buf,
             " %.1ffps [%s] algo:%s fuel:%.2f wind:%s %dHz ",
             fps, k_themes[g->theme].name, algo_name(g->algo),
             g->fuel, wstr, sfps);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_BASE + RAMP_N - 1) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_BASE + RAMP_N - 1) | A_BOLD);

    attron(COLOR_PAIR(CP_BASE + 2));
    mvprintw(1, hx, " q:quit spc:pause a:algo t:theme g/G:fuel w/W:wind ");
    attroff(COLOR_PAIR(CP_BASE + 2));
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §11 app                                                                */
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
    Grid *g = &a->scene.grid;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': a->scene.paused = !a->scene.paused; break;

    case 'a': case 'A':
        g->algo     = (g->algo + 1) % N_ALGOS;
        g->warmup   = 0;
        g->wind_acc = 0;
        memset(g->parts, 0, sizeof g->parts);
        g->part_idx          = 0;
        a->scene.needs_clear = true;
        break;

    case 't': case 'T':
        g->theme = (g->theme + 1) % THEME_COUNT;
        theme_apply(g->theme);
        a->scene.needs_clear = true;
        break;

    case 'g': g->fuel += 0.05f; if (g->fuel > 1.0f) g->fuel = 1.0f;  break;
    case 'G': g->fuel -= 0.05f; if (g->fuel < 0.1f) g->fuel = 0.1f;  break;

    case 'w': g->wind++; if (g->wind >  WIND_MAX) g->wind =  WIND_MAX; break;
    case 'W': g->wind--; if (g->wind < -WIND_MAX) g->wind = -WIND_MAX; break;
    case '0': g->wind = 0; break;

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
    signal(SIGINT, on_exit); signal(SIGTERM, on_exit); signal(SIGWINCH, on_resize);

    App *app  = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, 0);

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

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
