/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snow.c — drifting snowflakes with sin-sway + sandpile accumulation
 *
 * DEMO: Snowflakes drift down the screen with subtle horizontal SWAY
 *       (each flake has its own amplitude and frequency, so no two
 *       follow the same path). Wind adds a steady horizontal drift
 *       per pattern. When a flake reaches the top of the snow pile
 *       at its column, it DEPOSITS — `pile[col]++` — and dies. The
 *       deposit checks its two neighbours: if either is lower, the
 *       flake "rolls" there instead, so valleys fill before peaks
 *       and the pile naturally smooths into drifts.
 *
 *       Patterns:
 *         FLURRY    sparse, slow, big sway — light first snow
 *         SNOWFALL  medium density and speed, classic snow scene
 *         BLIZZARD  dense, fast, strong wind, sway suppressed
 *
 * Study alongside:
 *   rain.c                 — same particle-pool + pattern framework.
 *   grids/cell_grids/sandpile.c — the sandpile-stacking idea this
 *                            file's pile-deposit mechanism mirrors.
 *
 * Section map:
 *   §1 config    — constants, themes, per-pattern parameters
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair theme ramp + pile + sky pairs
 *   §4 flake     — Flake struct, sway position formula
 *   §5 pile      — 1-D accumulation array + neighbour-aware deposit
 *   §6 scene     — pool, tick, draw, prewarm, reseed
 *   §7 screen    — ncurses init / draw / resize
 *   §8 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear pile + flakes, re-prewarm)
 *   c          clear pile only (keep flakes)
 *   n / N      next pattern    (FLURRY → SNOWFALL → BLIZZARD)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster (speed multiplier ×2)
 *   -          slower (÷2)
 *   ] / [      raise / lower tick Hz
 *   w / W      wind right / left (override pattern wind by ±3 c/s)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/snow.c \
 *       -o snow -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system (one species: flakes)
 *                  + a 1-D INT array for the snow pile, one entry per
 *                  screen column storing the current pile height in
 *                  cells. Each tick:
 *
 *                    1. Top up the flake pool to `pattern.target_flakes`
 *                       (capped per-tick proportional to dt to avoid
 *                       spawn spikes).
 *                    2. Integrate each flake: vertical fall by `vy·dt`,
 *                       wind drift `drift_vx·dt`, sway position via
 *                       `sin(sway_freq·age + phase)·sway_amp`.
 *                    3. Detect pile contact: if flake.y crosses the top
 *                       of the pile at flake's column, DEPOSIT.
 *                    4. Deposit checks the two neighbouring columns: if
 *                       either is lower than the contact column, deposit
 *                       there instead. This lets falling snow fill
 *                       valleys before peaks — mimicking the angle-of-
 *                       repose behaviour of real snow.
 *                    5. Optionally MELT: every MELT_INTERVAL_S, every
 *                       column with pile > 0 decreases by 1 (slow).
 *                       Without melt, BLIZZARD fills the screen in
 *                       ~30 sec; with melt the pile reaches a steady
 *                       state height that depends on the pattern.
 *
 *                  Flake glyph picked at spawn from a small variety
 *                  set so the falling field reads as different snow
 *                  flake sizes (large `*`, medium `+`, tiny `.` `,`).
 *
 * Data-structure : Flake[MAX_FLAKES] with active flag — same pattern
 *                  as rain.c. pile[MAX_COLS] of int — height per col.
 *                  No malloc at runtime.
 *
 * Rendering      : ASCII only. Flakes use `*`, `+`, `.`, `'`, `,` —
 *                  selected at spawn for variety. Accumulated pile uses
 *                  `*` at the top and `#` below for slight depth feel.
 *                  All cells coloured from the active theme's 8-step
 *                  ramp (head bright, base dimmer for the pile).
 *
 * Performance    : O(MAX_FLAKES) per tick + occasional pile melt scan.
 *                  At BLIZZARD 600 flakes × constant work, ~24k mvaddch
 *                  per frame at 60 fps — well inside ncurses budget.
 *
 * References     :
 *   • Reeves, W. T. (1983) — "Particle Systems: A Technique for
 *     Modelling a Class of Fuzzy Objects", *ACM TOG* 2(2):91–108.
 *   • Bak, P., Tang, C. & Wiesenfeld, K. (1987) — "Self-organized
 *     criticality: An explanation of the 1/ƒ noise", *Phys. Rev. Lett.*
 *     59:381. The grain-redistribution rule that the pile-deposit
 *     mechanism here is a one-step approximation of.
 *   • Wikipedia — [Snow](https://en.wikipedia.org/wiki/Snow). Real flake
 *     terminal velocity is ~1 m/s (wet), ~3 m/s (dry).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each flake is an INDEPENDENT particle with its own fall speed,
 * sway amplitude, and sway frequency. The horizontal position is a
 * BASE x (which drifts with wind) plus a sin oscillation. When a
 * flake reaches the pile's top at its column, it lands — but if a
 * neighbour column is lower, it rolls there instead. The pile
 * grows column by column as drifts.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the screen is a glass case full of slow ping-pong balls.
 * Each ball drifts down on its own gentle curve (the sway), with
 * a constant wind blowing them sideways. They hit the bottom of
 * the case and stack — but they ROLL into low spots. The stack
 * grows uneven; some columns get tall while others stay short,
 * and the snow line undulates like real snow drifts.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SPAWN. Each tick, count active flakes; if below target, scan
 *     the pool for an inactive slot and spawn at random x with a
 *     random spawn-y in [-6, -1] (just above the visible top). On
 *     init / reseed / pattern-change, prewarm with random y across
 *     the WHOLE visible range (no "starting line" effect).
 *
 *  2. INTEGRATE per flake:
 *       flake.center_x += flake.drift_vx · dt
 *       flake.y        += flake.vy · dt
 *       flake.age      += dt
 *       flake.x        =  flake.center_x
 *                       + flake.sway_amp · sin(flake.sway_freq · flake.age
 *                                              + flake.sway_phase)
 *
 *  3. PILE CONTACT. Compute floor_y(col) = (rows − 2) − pile[col].
 *     If flake.y >= floor_y(col): DEPOSIT then deactivate.
 *
 *  4. DEPOSIT (with valley-fill):
 *       cl = col − 1, cr = col + 1
 *       if cl in range and pile[cl] < pile[col]: target = cl
 *       elif cr in range and pile[cr] < pile[col]: target = cr
 *       else target = col
 *       pile[target]++   (capped at PILE_MAX_HEIGHT)
 *
 *  5. MELT (every MELT_INTERVAL_S seconds):
 *       for c in 0..cols: if pile[c] > 0: pile[c]--
 *     Keeps the pile from filling the entire screen on long runs.
 *
 *  6. RENDER:
 *       For each active flake: render its glyph at (x, y) in a
 *       theme ramp colour based on the flake's "size" (big = bright).
 *       For each column c with pile[c] > 0: draw the pile from
 *       (rows-2) up to (rows-2 - pile[c]+1). Top cell = `*`, deeper
 *       cells = `#`.
 *
 *  7. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Sway oscillation:
 *    x(t) = base_x + drift_vx·(t − t_spawn)
 *               + sway_amp · sin(sway_freq·(t − t_spawn) + sway_phase)
 *
 *  Pile contact (per flake's column c):
 *    floor_y = (rows − 2) − pile[c]
 *    contact ⟺ flake.y ≥ floor_y
 *
 *  Valley-fill deposit:
 *    target = col,
 *      if pile[col-1] < pile[col]: target = col-1
 *      elif pile[col+1] < pile[col]: target = col+1
 *    pile[target] = min(pile[target] + 1, PILE_MAX)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • PILE_MAX_HEIGHT cap. Without it, BLIZZARD's high spawn rate
 *    would fill every column to the top of the screen. Cap at
 *    `(rows - 2) · PILE_MAX_FRAC` so even after long runs there's
 *    visible sky for falling flakes to be seen against.
 *
 *  • COLUMN OUT OF RANGE. flake.x can be negative (wind pushed it
 *    off the left edge before it hit pile) or >= cols. We treat
 *    out-of-bounds as "drifted off-screen" — flake quietly dies, no
 *    pile deposit. The active-count gap is filled by the next spawn.
 *
 *  • RESIZE. The pile array is fixed at MAX_COLS but drawn only up
 *    to current cols. On resize, we keep the existing pile data
 *    (truncated if cols shrinks) — no need to clear.
 *
 *  • SWAY AMPLITUDE BIGGER THAN SPACING. With sway_amp = 4 cells, a
 *    flake at cell 100 sways to cell 96 ↔ 104. That's normal — the
 *    flake just appears to glide horizontally. No clipping or wrap.
 *
 *  • DEPOSIT DURING FAST FALL. With BLIZZARD speed ~80 c/s and 60
 *    fps, a flake moves 1.3 cells per frame. The contact check
 *    `y >= floor_y` triggers within one frame so we don't need
 *    sub-frame interpolation; the pile just grows by 1 cell where
 *    the flake landed.
 *
 *  • PAUSE. Skip both flake integration and pile melt when paused.
 *    Drawing continues unchanged.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Flakes freeze mid-sway. Resume: motion continues.
 *
 *  • FLURRY. Sparse, slow, BIG sway — flakes visibly swing left-and-
 *    right as they fall. Pile grows slowly into gentle undulations.
 *
 *  • SNOWFALL. Medium density. Sway visible but smaller. Pile
 *    accumulates noticeably during a 10-sec watch.
 *
 *  • BLIZZARD. Dense, fast, strong wind. Sway barely visible (wind
 *    dominates). Pile builds quickly into clear drifts that lean in
 *    the wind direction.
 *
 *  • Wind override (`w`/`W`). Push flakes left/right interactively;
 *    pile drifts shift accordingly over time.
 *
 *  • Pile clear (`c`). Removes pile but keeps flakes — useful for
 *    watching just the falling motion.
 *
 *  • Theme cycle (`t`/`T`). Each theme produces a distinctively
 *    coloured snowfall + pile.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    SPEED_MIN        =   1,
    SPEED_DEF        =   8,
    SPEED_MAX        =  64,

    MAX_FLAKES       =  900,
    MAX_COLS         =  800,    /* fixed-size pile array              */

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_FLAKE_BASE  =   3,    /* +0..+7 = 8 flake tints (small→big) */
    PAIR_PILE_BASE   =  11,    /* +0..+7 = 8 pile tints (deep→top)   */
    PAIR_SKY         =  19,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Per-flake physics jitter — same idea as rain.c. */
#define FLAKE_SPEED_VARIANCE  0.50f      /* ±25% per-flake fall speed */
#define FLAKE_WIND_JITTER     1.0f       /* ±0.5 c/s per flake        */

/* Pile mechanics. */
#define PILE_MAX_FRAC     0.55f          /* pile capped at 55% screen */
#define MELT_INTERVAL_S   1.50f          /* every column melts 1 cell */

/* Wind override step. */
#define WIND_STEP         3.0f

/* Pattern enum. */
typedef enum {
    PATTERN_FLURRY   = 0,
    PATTERN_SNOWFALL = 1,
    PATTERN_BLIZZARD = 2,
    N_PATTERNS       = 3,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_FLURRY:   return "FLURRY  ";
    case PATTERN_SNOWFALL: return "SNOWFALL";
    case PATTERN_BLIZZARD: return "BLIZZARD";
    default:               return "?       ";
    }
}

/*
 * PatternParams — physics knobs per pattern.
 *
 *   target_flakes  : steady-state active flake count
 *   fall_speed     : nominal vy in cells/sec
 *   wind_x         : default horizontal drift (cells/sec)
 *   sway_amp_min/max  : sway amplitude range (cells)
 *   sway_freq_min/max : sway frequency range (rad/sec)
 *   pile_growth_mul   : multiplier on pile deposit (BLIZZARD = bigger
 *                       drifts; FLURRY = barely-grown pile)
 */
typedef struct {
    int   target_flakes;
    float fall_speed;
    float wind_x;
    float sway_amp_min, sway_amp_max;
    float sway_freq_min, sway_freq_max;
    float pile_growth_mul;     /* probability of pile deposit on hit */
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /* FLURRY    */ { 120,  10.0f,  3.0f, 1.5f, 4.0f, 0.40f, 1.30f, 0.60f },
    /* SNOWFALL  */ { 320,  18.0f,  6.0f, 0.8f, 2.5f, 0.30f, 1.10f, 0.85f },
    /* BLIZZARD  */ { 700,  45.0f, 22.0f, 0.3f, 1.0f, 0.20f, 0.80f, 1.00f },
};

/*
 * Themes — flake[8] is a SMALL→BIG ramp (smaller flakes fade dimmer,
 * bigger ones brighter). pile[8] is a DEEP→TOP ramp (older snow deeper
 * darker, fresh top brighter).
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       flake[8];   /* small → big   */
    short       pile [8];   /* deep  → top   */
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        flake[0..7]                                       pile[0..7]                                        sky */

    { "DEFAULT",  { 153, 159, 195, 195, 230, 231, 255, 255 },        { 244, 246, 248, 250, 252, 253, 254, 255 },       234 },
    { "ARCTIC",   { 110, 117, 153, 159, 195, 195, 231, 255 },        { 117, 153, 159, 195, 230, 231, 254, 255 },       235 },
    { "DUSK",     { 138, 175, 211, 217, 218, 224, 230, 231 },        { 138, 174, 175, 211, 218, 224, 254, 255 },       234 },
    { "AURORA",   {  43,  44,  79,  85, 121, 157, 195, 230 },        {  79, 115, 121, 157, 158, 194, 230, 231 },       234 },
    { "WARM",     { 215, 216, 222, 223, 224, 229, 230, 231 },        { 138, 180, 187, 223, 230, 231, 254, 255 },       234 },
    { "NEON",     { 165, 171, 207, 213, 219, 219, 225, 231 },        { 171, 177, 207, 213, 219, 225, 230, 231 },       234 },
    { "FOREST",   {  79, 115, 121, 157, 158, 194, 230, 255 },        {  43,  79, 115, 121, 157, 194, 230, 255 },       234 },
    { "MIDNIGHT", {  60,  61,  98, 104, 146, 153, 195, 231 },        {  60,  98, 104, 146, 153, 195, 231, 255 },       232 },
    { "MONO",     { 244, 246, 248, 250, 252, 253, 254, 255 },        { 244, 246, 248, 250, 252, 253, 254, 255 },       232 },
    { "VIOLET",   { 134, 135, 176, 177, 213, 219, 225, 231 },        { 134, 135, 176, 213, 219, 225, 230, 231 },       234 },
};

/* Variety of flake glyphs, indexed by SIZE class (small → big). */
static const char FLAKE_GLYPHS[8] = { '`', '.', '\'', ',', ':', '+', '*', '*' };

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
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++) {
            init_pair((short)(PAIR_FLAKE_BASE + i), t->flake[i], -1);
            init_pair((short)(PAIR_PILE_BASE  + i), t->pile [i], -1);
        }
        init_pair(PAIR_SKY, t->sky, -1);
    } else {
        for (int i = 0; i < 8; i++) {
            init_pair((short)(PAIR_FLAKE_BASE + i), COLOR_WHITE, -1);
            init_pair((short)(PAIR_PILE_BASE  + i), COLOR_WHITE, -1);
        }
        init_pair(PAIR_SKY, COLOR_BLACK, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §4  flake                                                              */
/* ===================================================================== */

typedef struct {
    float center_x;     /* x WITHOUT sway — drifts with wind             */
    float y;
    float vy;           /* fall speed (cells/sec) — variance applied     */
    float drift_vx;     /* horizontal wind component (cells/sec)         */
    float sway_amp;     /* sway amplitude (cells)                        */
    float sway_freq;    /* sway angular frequency (rad/sec)              */
    float sway_phase;   /* per-flake phase offset                        */
    float age;          /* seconds since spawn                           */
    int   size_idx;     /* 0..7 — picks glyph + colour ramp slot         */
    bool  active;
} Flake;

/* Cheap LCG — per-scene state, no global aliasing */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}
static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);    /* [0, 1) */
}

/* Compute a flake's actual screen x at this moment. */
static inline float flake_x(const Flake *f)
{
    return f->center_x
         + f->sway_amp * sinf(f->sway_freq * f->age + f->sway_phase);
}

/* ===================================================================== */
/* §5  pile — accumulation array + valley-fill deposit                   */
/* ===================================================================== */

/*
 * pile_deposit() — add one cell of snow at column `col` with neighbour
 * preference: if col-1 or col+1 is lower, deposit there instead.
 *
 * This is the angle-of-repose trick — falling snow naturally fills
 * valleys before raising peaks. Without it the pile grows as straight
 * vertical columns (one per flake hit), which looks artificial.
 *
 * Deposit fails silently (no-op) if all three columns are at PILE_MAX.
 */
static void pile_deposit(int *pile, int cols, int col, int max_h)
{
    if (col < 0 || col >= cols) return;

    int target = col;
    if (col > 0       && pile[col - 1] < pile[target]) target = col - 1;
    if (col < cols-1  && pile[col + 1] < pile[target]) target = col + 1;

    if (pile[target] < max_h) pile[target] += 1;
}

/* Slow background melt — every column with pile > 0 loses 1 cell.
 * Without this BLIZZARD fills the screen in ~30 seconds; with it the
 * pile reaches a steady state where deposit rate = melt rate. */
static void pile_melt_pass(int *pile, int cols)
{
    for (int c = 0; c < cols; c++)
        if (pile[c] > 0) pile[c] -= 1;
}

/* ===================================================================== */
/* §6  scene — pool, tick, draw                                          */
/* ===================================================================== */

typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    float     wind_override;
    uint32_t  rng;
    int       rows, cols;

    float     time_accum;       /* seconds since start (for sway phase) */
    float     melt_accum;       /* seconds since last melt pass         */

    Flake     flakes[MAX_FLAKES];
    int       pile  [MAX_COLS];
} Scene;

static int flake_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_FLAKES; i++)
        if (!s->flakes[i].active) return i;
    return -1;
}

/*
 * scene_spawn_flake — activate one flake.
 *
 *   y_min, y_max : range from which to draw initial y. Normal use
 *                  passes (-6, -1); prewarm passes (-6, rows-2).
 *
 * All physics parameters are drawn from the active pattern with
 * per-flake jitter so no two flakes follow the same path.
 */
static void scene_spawn_flake(Scene *s, float y_min, float y_max)
{
    int idx = flake_pool_find_inactive(s);
    if (idx < 0) return;
    Flake *f = &s->flakes[idx];

    const PatternParams *pp = &pattern_params[s->current_pattern];

    float wind = pp->wind_x + s->wind_override;
    float over = fabsf(wind) * 0.5f;
    float rngx = lcg_unit(&s->rng);
    float spawn_x;
    if (wind > 0.5f) {
        spawn_x = rngx * ((float)s->cols + over) - over;
    } else if (wind < -0.5f) {
        spawn_x = rngx * ((float)s->cols + over);
    } else {
        spawn_x = rngx * (float)s->cols;
    }
    float spawn_y = y_min + lcg_unit(&s->rng) * (y_max - y_min);

    /* Per-flake physics jitter — speed variance is the key ingredient
     * that makes the falling field look like independent particles
     * rather than a synchronised sheet. */
    float speed_jitter = (1.0f - FLAKE_SPEED_VARIANCE * 0.5f)
                       + lcg_unit(&s->rng) * FLAKE_SPEED_VARIANCE;
    float wind_jitter  = (lcg_unit(&s->rng) - 0.5f) * 2.0f * FLAKE_WIND_JITTER;

    float sway_amp  = pp->sway_amp_min
                    + lcg_unit(&s->rng) * (pp->sway_amp_max - pp->sway_amp_min);
    float sway_freq = pp->sway_freq_min
                    + lcg_unit(&s->rng) * (pp->sway_freq_max - pp->sway_freq_min);
    float sway_phs  = lcg_unit(&s->rng) * 2.0f * (float)M_PI;

    /* Size classes: weight toward smaller flakes. r in [0,1] →
     * size 0..7 with bias to lower indices. */
    float r = lcg_unit(&s->rng);
    int size_idx = (int)(r * r * 7.999f);   /* squared for skew toward small */
    if (size_idx < 0) size_idx = 0;
    if (size_idx > 7) size_idx = 7;

    f->center_x   = spawn_x;
    f->y          = spawn_y;
    f->vy         = pp->fall_speed * speed_jitter;
    f->drift_vx   = wind + wind_jitter;
    f->sway_amp   = sway_amp;
    f->sway_freq  = sway_freq;
    f->sway_phase = sway_phs;
    f->age        = 0.0f;
    f->size_idx   = size_idx;
    f->active     = true;
}

static void scene_clear_flakes(Scene *s)
{
    for (int i = 0; i < MAX_FLAKES; i++) s->flakes[i].active = false;
}

static void scene_clear_pile(Scene *s)
{
    memset(s->pile, 0, sizeof s->pile);
}

/*
 * scene_prewarm — fill the flake pool to target with flakes scattered
 * uniformly across the visible y range so the screen looks like
 * steady-state snow from frame 1, not a marching-band wave at the top.
 */
static void scene_prewarm(Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int target = pp->target_flakes;
    if (target > MAX_FLAKES) target = MAX_FLAKES;

    int active = 0;
    for (int i = 0; i < MAX_FLAKES; i++)
        if (s->flakes[i].active) active++;

    float y_max = (float)(s->rows - 2);
    for (int k = active; k < target; k++)
        scene_spawn_flake(s, -6.0f, y_max);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_SNOWFALL;
    s->wind_override   = 0.0f;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    s->time_accum      = 0.0f;
    s->melt_accum      = 0.0f;
    scene_clear_flakes(s);
    scene_clear_pile(s);
    scene_prewarm(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    /* Pile data preserved (truncated if cols shrinks). */
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    s->wind_override = 0.0f;
    scene_clear_flakes(s);
    scene_clear_pile(s);
    scene_prewarm(s);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    s->time_accum += dt;
    s->melt_accum += dt;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* 1. Top up flakes to target. */
    int active = 0;
    for (int i = 0; i < MAX_FLAKES; i++)
        if (s->flakes[i].active) active++;

    int target = pp->target_flakes;
    if (target > MAX_FLAKES) target = MAX_FLAKES;
    int spawn_cap = (int)((float)pp->target_flakes * dt * 4.0f) + 4;
    int to_spawn  = target - active;
    if (to_spawn > spawn_cap) to_spawn = spawn_cap;
    for (int k = 0; k < to_spawn; k++) scene_spawn_flake(s, -6.0f, -1.0f);

    /* 2. Integrate flakes; check pile contact. */
    int max_pile_h = (int)((float)(s->rows - 2) * PILE_MAX_FRAC);
    if (max_pile_h < 1) max_pile_h = 1;

    for (int i = 0; i < MAX_FLAKES; i++) {
        Flake *f = &s->flakes[i];
        if (!f->active) continue;

        f->center_x += f->drift_vx * dt;
        f->y        += f->vy * dt;
        f->age      += dt;

        /* Off-screen sideways → die quietly. */
        if (f->center_x < -8.0f || f->center_x > (float)(s->cols + 8)) {
            f->active = false;
            continue;
        }

        /* Pile contact at flake's CURRENT column (with sway). */
        float fx     = flake_x(f);
        int   col    = (int)(fx + 0.5f);
        if (col < 0 || col >= s->cols) {
            /* Sway pushed it off-screen but center still on-screen.
             * Keep alive; it'll sway back next tick. */
            continue;
        }
        int   pile_h = s->pile[col];
        float floor_y = (float)(s->rows - 2 - pile_h);

        if (f->y >= floor_y) {
            /* Deposit (with valley-fill) — but skip with probability
             * (1 - pattern.pile_growth_mul) so FLURRY doesn't pile
             * up as fast as BLIZZARD even at the same flake count. */
            float grow_roll = lcg_unit(&s->rng);
            if (grow_roll < pp->pile_growth_mul)
                pile_deposit(s->pile, s->cols, col, max_pile_h);
            f->active = false;
        }
    }

    /* 3. Periodic melt. */
    while (s->melt_accum >= MELT_INTERVAL_S) {
        pile_melt_pass(s->pile, s->cols);
        s->melt_accum -= MELT_INTERVAL_S;
    }
}

static void scene_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;     /* leave bottom for HUD */

    /* ── 1. Pile (drawn first so flakes overlay it) ───────────────── */
    for (int c = 0; c < s->cols && c < MAX_COLS; c++) {
        int h = s->pile[c];
        if (h <= 0) continue;
        for (int k = 0; k < h; k++) {
            int y = (rows_eff - 1) - k;
            if (y < 0) break;

            /* Top cell brightest; deeper cells use lower ramp slots. */
            int ramp_slot = 7 - k;
            if (ramp_slot < 0) ramp_slot = 0;
            char glyph = (k == 0) ? '*' : (k < 3 ? '#' : '+');
            int  attr  = (ramp_slot >= 6) ? A_BOLD
                       : (ramp_slot <= 1) ? A_DIM
                       :                    A_NORMAL;
            int  pair  = PAIR_PILE_BASE + ramp_slot;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(y, c, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* ── 2. Flakes ─────────────────────────────────────────────────── */
    for (int i = 0; i < MAX_FLAKES; i++) {
        const Flake *f = &s->flakes[i];
        if (!f->active) continue;
        float fx = flake_x(f);
        int   ix = (int)(fx + 0.5f);
        int   iy = (int)(f->y + 0.5f);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < 0 || iy >= rows_eff) continue;

        char glyph = FLAKE_GLYPHS[f->size_idx];
        int  pair  = PAIR_FLAKE_BASE + f->size_idx;
        int  attr  = (f->size_idx >= 6) ? A_BOLD
                   : (f->size_idx <= 1) ? A_DIM
                   :                      A_NORMAL;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize_curses(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Active flake count + max pile height for HUD. */
static void scene_counts(const Scene *s, int *out_flakes, int *out_max_pile)
{
    int n = 0;
    for (int i = 0; i < MAX_FLAKES; i++) if (s->flakes[i].active) n++;
    *out_flakes = n;

    int mp = 0;
    int cap = s->cols < MAX_COLS ? s->cols : MAX_COLS;
    for (int c = 0; c < cap; c++) if (s->pile[c] > mp) mp = s->pile[c];
    *out_max_pile = mp;
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int flakes, max_pile;
    scene_counts(s, &flakes, &max_pile);
    const PatternParams *pp = &pattern_params[s->current_pattern];
    float wind = pp->wind_x + s->wind_override;

    const char *state_str = s->paused ? "PAUSED " : pattern_name(s->current_pattern);

    char buf[220];
    snprintf(buf, sizeof buf,
             " SNOW   %s   theme:%-8s   flakes:%4d  pile_h:%2d  "
             "wind:%+5.1f c/s   %5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  w/W:wind  c:clear  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name,
             flakes, max_pile, (double)wind, fps, sim_fps, s->speed);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                     break;
    case 'r': case 'R': scene_reseed(s);                            break;
    case 'c':           scene_clear_pile(s);                        break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        scene_prewarm(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_prewarm(s);
        break;

    case 'w':
        s->wind_override += WIND_STEP;
        break;
    case 'W':
        s->wind_override -= WIND_STEP;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
