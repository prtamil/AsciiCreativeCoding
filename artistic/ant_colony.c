/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ant_colony.c — Ant Colony Optimisation: stigmergic shortest-path
 *                pheromone trails between a nest and food sources.
 *
 * DEMO: A colony of ~80 ants wanders a terminal grid.  Searching ants
 *       move with a slight bias toward the nearest food source and
 *       deposit a faint pheromone trail.  When an ant reaches food,
 *       it switches to RETURNING state and bee-lines back to the
 *       nest, depositing a HEAVY pheromone trail.  Future ants
 *       follow strong trails preferentially → positive feedback
 *       (autocatalysis) → the colony converges on the shortest
 *       routes between nest and food without any global knowledge.
 *
 *       Pheromone evaporates each tick, so disused paths fade and
 *       the colony adapts when food is moved (press `r`).  Classical
 *       Deneubourg double-bridge experiment, run live in your terminal.
 *
 *       Patterns (cycle with n / N) — different food-source layouts:
 *         DOUBLE     2 sources, opposite sides (canonical Deneubourg)
 *         SINGLE     1 source — simplest trail pattern
 *         QUAD       4 cardinal sources around nest
 *         LINE       3 collinear sources on one edge
 *         HEXAGON    6 sources at hex-arrangement angles
 *         CROSS      4 sources, cardinal directions, mid-distance
 *         TRIANGLE   3 sources at equilateral-ish triangle vertices
 *         CIRCLE     8 sources on a ring around the nest
 *         DIAGONAL   4 sources along the main diagonal
 *         CLUSTER    5 sources tightly clumped on one side
 *         DISTANT    2 sources at extreme opposite corners
 *         PERIMETER  6 sources spread along screen edges
 *         GRID       9 sources in a 3×3 grid
 *         RANDOM     7 sources at fixed-seed random positions
 *
 *       Themes (cycle with t / T):
 *         CLASSIC   blue trails, yellow searchers, magenta returners
 *         INFRARED  warm red/orange "heat" trails
 *         FOREST    green trails, brown ants
 *         NEON      vivid magenta/cyan pop
 *         TWILIGHT  violet/purple, dusk vibe
 *         SUMI_E    white-paper inverted (Japanese-ink aesthetic)
 *
 * Section map:
 *   §1 config    — themes, patterns, simulation tunables
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 6 themes (with inverted-bg support)
 *   §4 grid      — pheromone field
 *   §5 ants      — pool, state machine, tick
 *   §6 scene     — composition, food + nest, render
 *   §7 screen + app
 *
 * Keys:
 *   q / ESC      quit
 *   space / p    pause / resume simulation
 *   r            reset (clear pheromones, respawn ants)
 *   n / N        next / previous food pattern
 *   t / T        next / previous theme
 *   + / =        sim speed up (more ticks per render frame)
 *   -            sim speed down
 *   ] / [        sim Hz up / down (advanced)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/ant_colony.c \
 *       -o ant_colony -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Ant Colony Optimisation (ACO) — stigmergic path
 *                  finding via pheromone field reinforcement.
 *
 *                  ANT STATE MACHINE:
 *                     SEARCHING:
 *                       — pick next cell from a forward-arc of three
 *                         8-directional candidates, weighted by
 *                         (pheromone_at(cell) + base) · food_bias
 *                       — deposit DEPOSIT_SEARCH on current cell
 *                       — on entering food's detection radius:
 *                         flip to RETURNING; head turns toward nest
 *                     RETURNING:
 *                       — bee-line toward nest with ±1-step random
 *                         deviation (avoids over-straight paths)
 *                       — deposit DEPOSIT_RETURN (~10× SEARCH)
 *                       — on entering nest's detection radius:
 *                         increment food_collected; flip to SEARCHING
 *
 *                  PHEROMONE FIELD:
 *                     Per-cell scalar; each tick:
 *                       τ_new = max(0, τ × (1 − EVAP_RATE) + Δ)
 *                     where Δ is the sum of deposits this tick.
 *                     Evaporation makes disused paths fade →
 *                     the colony adapts to changes in environment.
 *
 *                  WHY SHORTEST PATHS EMERGE (autocatalysis):
 *                     Two paths from nest to food, lengths L1 < L2.
 *                     An ant on L1 returns sooner → reinforces L1
 *                     while its peer on L2 is still in transit.  Next
 *                     ant chooses biased by current trail strength →
 *                     more likely to take L1.  Positive feedback
 *                     compounds; within ~30 sec the colony converges
 *                     on L1 even though no individual ant knows it's
 *                     shorter.  Real ants do this in nature
 *                     (Deneubourg et al. 1990 double-bridge experiment).
 *
 * Data-structure : `g_ph[H][W]` float pheromone field.  `g_ants[N]`
 *                  ant pool (position + direction + state).  All in
 *                  BSS, no malloc.
 *
 * Rendering      : ASCII only.  Pheromone shown as 4-tier glyph ramp
 *                  (`.`/`:`/`+`/`#`) coloured by theme palette.  Ants
 *                  rendered as `o` in state-coloured pairs; nest as
 *                  `@`, food as `*` with bracket frame.
 *
 * Performance    : O(N_ants × ants_per_tick + grid_cells × evap) per
 *                  tick.  At N_ants = 80 and 200×60 grid: ~12 K cells
 *                  × evap + 80 ants × small constant work ≈ 14 K ops.
 *                  Trivial — holds well past 60 fps with multi-tick
 *                  speed multipliers.
 *
 * References     :
 *   • Deneubourg, J.-L., Aron, S., Goss, S. & Pasteels, J. M. (1990) —
 *     "The self-organizing exploratory pattern of the Argentine ant",
 *     *Journal of Insect Behavior* 3(2):159-168.  The original
 *     double-bridge experiment that this demo replicates.
 *   • Dorigo, M. & Stützle, T. (2004) — *Ant Colony Optimization*.
 *     MIT Press.  The canonical text on the metaheuristic.
 *   • Bonabeau, E., Dorigo, M., Theraulaz, G. (1999) — *Swarm
 *     Intelligence: From Natural to Artificial Systems*.  Stigmergy
 *     and self-organisation primer.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Ants don't communicate.  They drop and follow chemical trails.
 * That's the whole algorithm.  An ant returning from food drops a
 * heavy trail; future searching ants prefer cells with stronger
 * trails; over time the strongest trails are the shortest paths,
 * because shorter trips reinforce faster.  The intelligence lives
 * in the FIELD, not in any ant.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the floor of your kitchen with breadcrumbs scattered around
 * a sandwich.  Drop ten kids in the room.  They wander randomly until
 * they find a crumb; once they do, they walk back to the door
 * dropping coloured chalk dust as they go.  The next kid leaving the
 * door notices the chalk and prefers to walk along it; if it leads
 * to a crumb, they follow the same path back, depositing more chalk.
 * After 5 minutes, every kid is shuttling back and forth between
 * door and crumbs along a single bright chalk path.  Nobody planned
 * the route; it self-organised.  That's ACO.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Initialise ants at the nest, all SEARCHING, random directions.
 *
 *  2. Per tick, per ant:
 *
 *       SEARCHING:
 *         deposit DEPOSIT_SEARCH at current cell
 *         pick three forward-arc 8-dir candidates (dir−1, dir, dir+1)
 *         weight each by (pheromone(cell) + base_noise) · food_bias
 *         pick proportionally to weight
 *         move; bounce if would exit grid
 *         if ant within FOOD_RADIUS of any food: flip to RETURNING
 *
 *       RETURNING:
 *         deposit DEPOSIT_RETURN at current cell
 *         pick direction = nearest 8-dir to nest ± random offset (-1, 0, +1)
 *         move; bounce if would exit grid
 *         if ant within FOOD_RADIUS of nest: increment count, flip to SEARCHING
 *
 *  3. Per tick, evaporate the entire pheromone field:
 *
 *       τ_new = τ_old · (1 − EVAP_RATE)
 *       (clip to 0 below epsilon)
 *
 *  4. Render each frame: pheromone field as 4-tier glyph ramp; ants
 *     as state-coloured `o`; nest as `@`; food sources as `*`.
 *
 *  5. Cycle pattern (`n`) → reset and re-place food sources.
 *     Cycle theme (`t`) → re-init colour pairs only.
 *
 * KEY FORMULAS
 * ────────────
 *  Pheromone update:
 *    τ_{t+1} = max(0, τ_t · (1 − EVAP_RATE) + Σ_deposits)
 *    EVAP_RATE = 0.003 → e-fold decay every ~333 ticks.
 *
 *  Direction-biased weight (search step):
 *    w(d) = (τ(neighbour_d) + ε_base) · max(0, FOOD_BIAS · (n̂_d · n̂_food + 1))
 *    P(d) = w(d) / Σ_d' w(d')
 *
 *  8-directional unit vectors:
 *    DDX[8] = { 0, 1, 1, 1, 0,-1,-1,-1 }
 *    DDY[8] = {-1,-1, 0, 1, 1, 1, 0,-1 }
 *    (N, NE, E, SE, S, SW, W, NW)
 *
 *  Best-direction-toward (returning ants):
 *    ideal_dir = argmax_d (DDX[d] · Δx + DDY[d] · Δy)
 *    where (Δx, Δy) = (nest − ant) (or (food − ant) for searchers)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • PHEROMONE OVERFLOW.  Clipped at PH_MAX = 1.0 to prevent runaway
 *    along well-used paths.  Without the cap, a heavily-used route
 *    saturates so much that ants would never explore alternatives.
 *
 *  • DEAD ANTS (stuck at boundary).  If an ant tries to move out of
 *    the grid, we reverse its direction.  If reversing also fails
 *    (corner case — narrow grid + bad dir), the ant stays put for
 *    one tick.  Eventually picks a new direction.
 *
 *  • CONVERGENCE TIME.  Default ~30 sec for the colony to settle
 *    onto its primary trails.  Reset (`r`) restarts; pattern change
 *    (`n`) also restarts, since food positions move.
 *
 *  • TWO PATTERNS, SAME GRID.  All patterns share the nest at
 *    centre; only food positions differ.  The pheromone field is
 *    cleared on pattern change so old trails don't bias the new
 *    layout.
 *
 *  • INPUT RESPONSIVENESS.  The earlier version checked input ONCE
 *    per render frame, AFTER running `speed`-many sim ticks.  At
 *    high speeds the input felt laggy.  This version reads input at
 *    60 fps regardless of sim speed; sim ticks accumulate via a
 *    fixed-step loop, so speed scaling doesn't impact key-poll rate.
 *
 *  • INVERTED THEME (SUMI_E).  Pre-fill white "paper" before drawing,
 *    use dark fg pairs, disable A_BOLD on ants/food/nest.  Standard
 *    repo recipe.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Watch the first 30 seconds.  Initially the pheromone trails
 *    are diffuse, ants wander broadly.  Within ~10 sec, two faint
 *    trails (nest→food[0] and nest→food[1]) emerge; within ~30 sec,
 *    they crystallise into bright `+`/`#` paths.
 *
 *  • Press `n` to switch pattern.  All trails clear; new food
 *    sources appear; the colony re-explores and finds new paths.
 *
 *  • Press `t` to cycle theme.  Trails and ants recolour instantly.
 *    Geometry unchanged.
 *
 *  • Press `+` repeatedly.  Speed multiplier increases (multiple sim
 *    ticks per render frame).  Animation visibly speeds up; key
 *    presses still register immediately (responsive at any speed).
 *
 *  • Press `r`.  Reset clears trails and respawns ants at the nest.
 *    Convergence restarts.
 *
 *  • At 60×20 the demo should still show clear nest, food, ants,
 *    and at least one visible trail.  Smaller terminals push food
 *    closer to nest but layout remains sensible.
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
    SIM_FPS_MIN      =  20,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 240,
    SIM_FPS_STEP     =  20,

    SPEED_MIN        =   1,
    SPEED_DEFAULT    =   2,
    SPEED_MAX        =  16,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_PH_BASE      =  3,    /* +0..+3 — pheromone gradient (faint→strong) */
    PAIR_ANT_S        =  7,    /* searching ant                         */
    PAIR_ANT_R        =  8,    /* returning ant                         */
    PAIR_FOOD         =  9,
    PAIR_NEST         = 10,
    PAIR_PAPER        = 11,    /* inverted-theme white bg               */

    /* Static-buffer caps. */
    GRID_W_MAX        = 320,
    GRID_H_MAX        = 100,

    /* Ants. */
    N_ANTS            =  80,
    HUD_ROWS          =   1,    /* bottom row reserved for HUD          */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Pheromone tunables. */
#define EVAP_RATE       0.003f   /* fraction lost per tick            */
#define DEPOSIT_RETURN  0.40f
#define DEPOSIT_SEARCH  0.04f
#define PH_MAX          1.0f
#define PH_BASE_NOISE   0.05f    /* base attraction even on empty cells */

#define FOOD_BIAS       3.0f     /* food-direction pull strength       */
#define FOOD_RADIUS     2        /* Manhattan radius for food/nest hit */

/* 8-connected direction vectors (N NE E SE S SW W NW). */
static const int DDX[8] = { 0, 1, 1, 1, 0,-1,-1,-1 };
static const int DDY[8] = {-1,-1, 0, 1, 1, 1, 0,-1 };

/* Pattern enum + per-pattern food-source layout.
 * Each pattern arranges food sources differently around the nest, so
 * the learner can watch the colony adapt its trail topology to the
 * geometry of the foraging problem.  All patterns share the same nest
 * (centre of grid) — only food layout changes. */
typedef enum {
    PAT_DOUBLE  = 0,    /* 2 sources, opposite sides — Deneubourg classic */
    PAT_SINGLE,         /* 1 source                                       */
    PAT_QUAD,           /* 4 cardinal sources                             */
    PAT_LINE,           /* 3 collinear sources on right edge              */
    PAT_HEXAGON,        /* 6 sources around nest at hex angles            */
    PAT_CROSS,          /* 4 sources at near-distance cardinals           */
    PAT_TRIANGLE,       /* 3 sources at equilateral-ish triangle          */
    PAT_CIRCLE,         /* 8 sources on a ring                            */
    PAT_DIAGONAL,       /* 4 sources along the main diagonal              */
    PAT_CLUSTER,        /* 5 sources tightly clumped on one side          */
    PAT_DISTANT,        /* 2 sources at extreme corners                   */
    PAT_PERIMETER,      /* 6 sources spread along screen edges            */
    PAT_GRID,           /* 9 sources in a 3×3 grid                        */
    PAT_RANDOM,         /* 7 sources at fixed-seed random positions       */
    N_PATTERNS,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PAT_DOUBLE:    return "DOUBLE   ";
    case PAT_SINGLE:    return "SINGLE   ";
    case PAT_QUAD:      return "QUAD     ";
    case PAT_LINE:      return "LINE     ";
    case PAT_HEXAGON:   return "HEXAGON  ";
    case PAT_CROSS:     return "CROSS    ";
    case PAT_TRIANGLE:  return "TRIANGLE ";
    case PAT_CIRCLE:    return "CIRCLE   ";
    case PAT_DIAGONAL:  return "DIAGONAL ";
    case PAT_CLUSTER:   return "CLUSTER  ";
    case PAT_DISTANT:   return "DISTANT  ";
    case PAT_PERIMETER: return "PERIMETER";
    case PAT_GRID:      return "GRID     ";
    case PAT_RANDOM:    return "RANDOM   ";
    default:            return "?        ";
    }
}

/*
 * Theme — every entry sits in the bright half of the 256-cube per
 * the CLAUDE.md "Theme Palette Brightness" rule, so even slot 0 of
 * the pheromone gradient stays visible against a black terminal.
 * NEGATIVE-flavoured "SUMI_E" theme follows the standard inverted
 * recipe (white bg + dark fg, A_BOLD disabled).
 */
typedef struct {
    const char *name;
    short       ph[4];       /* pheromone gradient: faint → strong   */
    short       ant_s;       /* searching ant                         */
    short       ant_r;       /* returning ant                         */
    short       food;
    short       nest;
    bool        inverted;
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* CLASSIC: blue/cyan trails, warm ants.                          */
    { "CLASSIC ",
      {  39,  45,  51, 195 },    /* sky-blue → cyan → pale cyan       */
      226,                       /* searcher: yellow                  */
      201,                       /* returner: bright magenta          */
       46,                       /* food: lime                        */
      214,                       /* nest: orange                      */
      false },

    /* INFRARED: warm red→orange→yellow heat-trails.                  */
    { "INFRARED",
      { 124, 166, 208, 220 },
      226, 196,  46, 201, false },

    /* FOREST: green trails, brown ants.                              */
    { "FOREST  ",
      {  34,  70, 112, 154 },
      136, 130, 220, 196, false },

    /* NEON: vivid magenta + cyan pop.                                */
    { "NEON    ",
      { 165, 207, 213,  51 },
      226,  51,  46, 201, false },

    /* TWILIGHT: violet/lavender dusk.                                */
    { "TWILIGHT",
      {  99, 105, 147, 195 },
      226, 213,  46, 207, false },

    /* SUMI_E: white-paper bg, dark fg, ink-painting aesthetic.       */
    { "SUMI_E  ",
      { 240, 237, 234,  16 },
       60, 124,  64, 130, true  },
};

/* Glyphs: 4-tier pheromone ramp + special markers. */
static const char PH_GLYPHS[4] = { '.', ':', '+', '#' };

#define ANT_GLYPH      'o'
#define FOOD_GLYPH     '*'
#define NEST_GLYPH     '@'
#define FOOD_FRAME_L   '['
#define FOOD_FRAME_R   ']'
#define NEST_FRAME_L   '('
#define NEST_FRAME_R   ')'

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
    const Theme *t = &themes[idx];
    short bg = t->inverted ? 231 : -1;

    if (COLORS >= 256) {
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_PH_BASE + i), t->ph[i], bg);
        init_pair(PAIR_ANT_S, t->ant_s, bg);
        init_pair(PAIR_ANT_R, t->ant_r, bg);
        init_pair(PAIR_FOOD,  t->food,  bg);
        init_pair(PAIR_NEST,  t->nest,  bg);
        init_pair(PAIR_PAPER, 16, t->inverted ? 231 : -1);
    } else {
        static const short fb_ph[4] = {
            COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN,
        };
        short bg8 = t->inverted ? COLOR_WHITE : -1;
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_PH_BASE + i),
                      t->inverted ? COLOR_BLACK : fb_ph[i], bg8);
        init_pair(PAIR_ANT_S, t->inverted ? COLOR_BLACK : COLOR_YELLOW,  bg8);
        init_pair(PAIR_ANT_R, t->inverted ? COLOR_BLACK : COLOR_MAGENTA, bg8);
        init_pair(PAIR_FOOD,  t->inverted ? COLOR_BLACK : COLOR_GREEN,   bg8);
        init_pair(PAIR_NEST,  t->inverted ? COLOR_BLACK : COLOR_RED,     bg8);
        init_pair(PAIR_PAPER, COLOR_BLACK, COLOR_WHITE);
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
/* §4  pheromone grid                                                     */
/* ===================================================================== */

static float g_ph[GRID_H_MAX][GRID_W_MAX];
static int   g_grid_h, g_grid_w;

static void ph_clear(void)
{
    for (int r = 0; r < g_grid_h; r++)
        for (int c = 0; c < g_grid_w; c++)
            g_ph[r][c] = 0.0f;
}

static void ph_evaporate(void)
{
    float decay = 1.0f - EVAP_RATE;
    for (int r = 0; r < g_grid_h; r++) {
        for (int c = 0; c < g_grid_w; c++) {
            g_ph[r][c] *= decay;
            if (g_ph[r][c] < 0.001f) g_ph[r][c] = 0.0f;
        }
    }
}

static inline void ph_deposit(int row, int col, float amount)
{
    if (row < 0 || row >= g_grid_h || col < 0 || col >= g_grid_w) return;
    g_ph[row][col] += amount;
    if (g_ph[row][col] > PH_MAX) g_ph[row][col] = PH_MAX;
}

static inline float ph_at(int row, int col)
{
    if (row < 0 || row >= g_grid_h || col < 0 || col >= g_grid_w) return 0.0f;
    return g_ph[row][col];
}

/* ===================================================================== */
/* §5  ants                                                               */
/* ===================================================================== */

typedef enum { SEARCHING, RETURNING } AntState;

typedef struct {
    int       row, col;
    int       dir;
    AntState  state;
} Ant;

static Ant g_ants[N_ANTS];
static int g_nest_row, g_nest_col;
static int g_food_row[16], g_food_col[16], g_food_n;
static int g_food_collected = 0;

static bool in_range(int r, int c, int tr, int tc, int radius)
{
    int dr = r - tr; if (dr < 0) dr = -dr;
    int dc = c - tc; if (dc < 0) dc = -dc;
    return (dr + dc) <= radius;
}

/* Best 8-direction toward (tr, tc) from (sr, sc). */
static int best_dir_toward(int sr, int sc, int tr, int tc)
{
    float dx = (float)(tc - sc), dy = (float)(tr - sr);
    float best_dot = -1e9f;
    int   best = 0;
    for (int d = 0; d < 8; d++) {
        float dot = (float)DDX[d] * dx + (float)DDY[d] * dy;
        if (dot > best_dot) { best_dot = dot; best = d; }
    }
    return best;
}

static void ant_init(int i)
{
    g_ants[i].row   = g_nest_row + (rand() % 3 - 1);
    g_ants[i].col   = g_nest_col + (rand() % 3 - 1);
    g_ants[i].dir   = rand() % 8;
    g_ants[i].state = SEARCHING;
}

/* Move ant one step.  Bounce off grid edges by reversing direction. */
static void ant_tick(Ant *a)
{
    int r = a->row, c = a->col;
    int top_clip = 0;     /* HUD is on the bottom row, so top is fine */

    if (a->state == SEARCHING) {
        ph_deposit(r, c, DEPOSIT_SEARCH);

        /* Forward arc (dir-1, dir, dir+1). */
        int cands[3] = {
            (a->dir + 7) % 8,
            a->dir,
            (a->dir + 1) % 8,
        };

        /* Find nearest food source for directional bias. */
        float best_fd = 1e9f;
        int   fi = 0;
        for (int k = 0; k < g_food_n; k++) {
            float fdx = (float)(g_food_col[k] - c);
            float fdy = (float)(g_food_row[k] - r);
            float d   = sqrtf(fdx * fdx + fdy * fdy);
            if (d < best_fd) { best_fd = d; fi = k; }
        }

        /* Weight each candidate by (pheromone + base) · food-bias. */
        float weights[3];
        float total = 0.0f;
        for (int k = 0; k < 3; k++) {
            int d  = cands[k];
            int nr = r + DDY[d];
            int nc = c + DDX[d];
            float ph = ph_at(nr, nc) + PH_BASE_NOISE;

            float fdx = (float)(g_food_col[fi] - c);
            float fdy = (float)(g_food_row[fi] - r);
            float fn  = sqrtf(fdx * fdx + fdy * fdy);
            if (fn > 0.0f) { fdx /= fn; fdy /= fn; }
            float bias = FOOD_BIAS
                       * ((float)DDX[d] * fdx + (float)DDY[d] * fdy + 1.0f);
            if (bias < 0.0f) bias = 0.0f;
            weights[k] = ph * bias;
            total += weights[k];
        }

        /* Probabilistic selection. */
        int chosen = 1;
        if (total > 0.0f) {
            float rnd = (float)rand() / (float)RAND_MAX * total;
            float acc = 0.0f;
            for (int k = 0; k < 3; k++) {
                acc += weights[k];
                if (rnd <= acc) { chosen = k; break; }
            }
        }
        int nd = cands[chosen];
        int nr = r + DDY[nd], nc = c + DDX[nd];

        /* Bounce off walls. */
        if (nr < top_clip || nr >= g_grid_h
            || nc < 0 || nc >= g_grid_w) {
            nd = (nd + 4) % 8;
            nr = r + DDY[nd]; nc = c + DDX[nd];
            if (nr < top_clip || nr >= g_grid_h
                || nc < 0 || nc >= g_grid_w) {
                nr = r; nc = c;
            }
        }
        a->dir = nd;
        a->row = nr;
        a->col = nc;

        /* Detect food. */
        for (int k = 0; k < g_food_n; k++) {
            if (in_range(a->row, a->col,
                         g_food_row[k], g_food_col[k], FOOD_RADIUS)) {
                a->state = RETURNING;
                a->dir   = best_dir_toward(a->row, a->col,
                                            g_nest_row, g_nest_col);
                return;
            }
        }
    } else {
        /* RETURNING — heavier deposit, beeline to nest with deviation. */
        ph_deposit(r, c, DEPOSIT_RETURN);

        int ideal  = best_dir_toward(r, c, g_nest_row, g_nest_col);
        int offset = (rand() % 3) - 1;       /* -1, 0, +1                */
        int nd     = (ideal + offset + 8) % 8;
        int nr     = r + DDY[nd], nc = c + DDX[nd];

        if (nr < top_clip || nr >= g_grid_h
            || nc < 0 || nc >= g_grid_w) {
            nd = ideal;
            nr = r + DDY[nd];
            nc = c + DDX[nd];
            if (nr < top_clip || nr >= g_grid_h
                || nc < 0 || nc >= g_grid_w) {
                nr = r; nc = c;
            }
        }
        a->dir = nd;
        a->row = nr;
        a->col = nc;

        /* Detect nest. */
        if (in_range(a->row, a->col, g_nest_row, g_nest_col, FOOD_RADIUS)) {
            g_food_collected++;
            a->state = SEARCHING;
            a->dir   = rand() % 8;
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool       paused;
    int        cols, rows;
    int        speed;
    int        current_pattern;
    int        current_theme;
} Scene;

/*
 * pattern_place_food — set up `g_food_row[]` / `g_food_col[]` /
 * `g_food_n` according to the current pattern.  Always uses the
 * current grid extents and nest position.
 */
static void pattern_place_food(Pattern p, int cols, int rows)
{
    int margin_x = cols / 5;
    if (margin_x < 4) margin_x = 4;
    int margin_y = rows / 5;
    if (margin_y < 3) margin_y = 3;

    int cx = cols / 2;       /* nest centre x       */
    int cy = rows / 2;       /* nest centre y       */
    int rx = cols / 2 - margin_x / 2;  /* ring x-radius */
    int ry = rows / 2 - margin_y / 2;  /* ring y-radius */
    if (rx < 4) rx = 4;
    if (ry < 3) ry = 3;

    switch (p) {
    case PAT_SINGLE:
        g_food_n      = 1;
        g_food_row[0] = rows / 4;
        g_food_col[0] = cols - margin_x;
        break;
    case PAT_QUAD:
        g_food_n      = 4;
        g_food_row[0] = margin_y;             g_food_col[0] = cx;
        g_food_row[1] = rows - margin_y;      g_food_col[1] = cx;
        g_food_row[2] = cy;                   g_food_col[2] = margin_x;
        g_food_row[3] = cy;                   g_food_col[3] = cols - margin_x;
        break;
    case PAT_LINE:
        g_food_n      = 3;
        g_food_row[0] = rows / 4;             g_food_col[0] = cols - margin_x;
        g_food_row[1] = cy;                   g_food_col[1] = cols - margin_x;
        g_food_row[2] = rows * 3 / 4;         g_food_col[2] = cols - margin_x;
        break;
    case PAT_HEXAGON: {
        /* 6 sources at 60-degree increments around the nest. */
        g_food_n = 6;
        for (int k = 0; k < 6; k++) {
            float a = (float)k * (float)(M_PI / 3.0);
            g_food_col[k] = cx + (int)(cosf(a) * (float)rx);
            g_food_row[k] = cy + (int)(sinf(a) * (float)ry);
        }
        break;
    }
    case PAT_CROSS:
        g_food_n      = 4;
        g_food_row[0] = cy - ry / 2;          g_food_col[0] = cx;
        g_food_row[1] = cy + ry / 2;          g_food_col[1] = cx;
        g_food_row[2] = cy;                   g_food_col[2] = cx - rx / 2;
        g_food_row[3] = cy;                   g_food_col[3] = cx + rx / 2;
        break;
    case PAT_TRIANGLE: {
        /* 3 sources at 90°, 210°, 330° (equilateral triangle, apex up). */
        g_food_n = 3;
        const float ang[3] = {
            (float)(-M_PI / 2.0),
            (float)(-M_PI / 2.0 + 2.0 * M_PI / 3.0),
            (float)(-M_PI / 2.0 + 4.0 * M_PI / 3.0),
        };
        for (int k = 0; k < 3; k++) {
            g_food_col[k] = cx + (int)(cosf(ang[k]) * (float)rx);
            g_food_row[k] = cy + (int)(sinf(ang[k]) * (float)ry);
        }
        break;
    }
    case PAT_CIRCLE: {
        /* 8 sources on a ring at 45° increments. */
        g_food_n = 8;
        for (int k = 0; k < 8; k++) {
            float a = (float)k * (float)(M_PI / 4.0);
            g_food_col[k] = cx + (int)(cosf(a) * (float)rx);
            g_food_row[k] = cy + (int)(sinf(a) * (float)ry);
        }
        break;
    }
    case PAT_DIAGONAL:
        g_food_n      = 4;
        g_food_row[0] = margin_y;             g_food_col[0] = margin_x;
        g_food_row[1] = cy - ry / 3;          g_food_col[1] = cx - rx / 3;
        g_food_row[2] = cy + ry / 3;          g_food_col[2] = cx + rx / 3;
        g_food_row[3] = rows - margin_y;      g_food_col[3] = cols - margin_x;
        break;
    case PAT_CLUSTER:
        /* 5 sources tightly grouped on the right side. */
        g_food_n      = 5;
        g_food_row[0] = cy - 2;               g_food_col[0] = cols - margin_x;
        g_food_row[1] = cy + 2;               g_food_col[1] = cols - margin_x;
        g_food_row[2] = cy;                   g_food_col[2] = cols - margin_x - 3;
        g_food_row[3] = cy - 1;               g_food_col[3] = cols - margin_x + 2;
        g_food_row[4] = cy + 1;               g_food_col[4] = cols - margin_x + 2;
        break;
    case PAT_DISTANT:
        g_food_n      = 2;
        g_food_row[0] = margin_y;             g_food_col[0] = margin_x;
        g_food_row[1] = rows - margin_y;      g_food_col[1] = cols - margin_x;
        break;
    case PAT_PERIMETER:
        g_food_n      = 6;
        g_food_row[0] = margin_y;             g_food_col[0] = cols / 4;
        g_food_row[1] = margin_y;             g_food_col[1] = cols * 3 / 4;
        g_food_row[2] = cy;                   g_food_col[2] = margin_x;
        g_food_row[3] = cy;                   g_food_col[3] = cols - margin_x;
        g_food_row[4] = rows - margin_y;      g_food_col[4] = cols / 4;
        g_food_row[5] = rows - margin_y;      g_food_col[5] = cols * 3 / 4;
        break;
    case PAT_GRID: {
        /* 3×3 grid of sources, skipping the centre (the nest). */
        g_food_n = 0;
        int gx[3] = { margin_x, cx, cols - margin_x };
        int gy[3] = { margin_y, cy, rows - margin_y };
        for (int j = 0; j < 3; j++) {
            for (int i = 0; i < 3; i++) {
                if (i == 1 && j == 1) continue;     /* skip nest cell */
                g_food_col[g_food_n] = gx[i];
                g_food_row[g_food_n] = gy[j];
                g_food_n++;
            }
        }
        break;
    }
    case PAT_RANDOM: {
        /* 7 sources at fixed-seed random positions inside the play area
         * — same layout every reset, but the geometry looks irregular. */
        g_food_n = 7;
        unsigned r = 0xC0FFEEu;
        for (int k = 0; k < 7; k++) {
            r = r * 1664525u + 1013904223u;
            int rx2 = (int)(r >> 16) % (cols - 2 * margin_x);
            r = r * 1664525u + 1013904223u;
            int ry2 = (int)(r >> 16) % (rows - 2 * margin_y);
            g_food_col[k] = margin_x + rx2;
            g_food_row[k] = margin_y + ry2;
        }
        break;
    }
    case PAT_DOUBLE:
    default:
        g_food_n      = 2;
        g_food_row[0] = rows / 3;             g_food_col[0] = margin_x;
        g_food_row[1] = rows * 2 / 3;         g_food_col[1] = cols - margin_x;
        break;
    }
}

static void scene_layout(Scene *s)
{
    int rows_eff = s->rows - HUD_ROWS;
    if (rows_eff < 8) rows_eff = 8;

    g_grid_h   = rows_eff < GRID_H_MAX ? rows_eff : GRID_H_MAX;
    g_grid_w   = s->cols   < GRID_W_MAX ? s->cols : GRID_W_MAX;
    g_nest_row = g_grid_h / 2;
    g_nest_col = g_grid_w / 2;

    pattern_place_food((Pattern)s->current_pattern, g_grid_w, g_grid_h);
}

static void scene_reset(Scene *s)
{
    scene_layout(s);
    g_food_collected = 0;
    ph_clear();
    for (int i = 0; i < N_ANTS; i++) ant_init(i);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cols            = cols;
    s->rows            = rows;
    s->speed           = SPEED_DEFAULT;
    s->current_pattern = PAT_DOUBLE;
    s->current_theme   = 0;
    scene_reset(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    scene_reset(s);
}

static void scene_cycle_pattern(Scene *s, int dir)
{
    int idx = s->current_pattern + dir;
    while (idx < 0) idx += N_PATTERNS;
    s->current_pattern = idx % N_PATTERNS;
    scene_reset(s);
}

static void scene_cycle_theme(Scene *s, int dir)
{
    int idx = s->current_theme + dir;
    while (idx < 0) idx += N_THEMES;
    s->current_theme = idx % N_THEMES;
    theme_apply(s->current_theme);
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    for (int i = 0; i < N_ANTS; i++) ant_tick(&g_ants[i]);
    ph_evaporate();
}

static void scene_render(const Scene *s)
{
    bool inverted = themes[s->current_theme].inverted;

    /* Inverted theme: pre-fill white "paper" before drawing. */
    if (inverted) {
        attron(COLOR_PAIR(PAIR_PAPER));
        for (int r = 0; r < g_grid_h; r++)
            for (int c = 0; c < g_grid_w; c++)
                mvaddch(r, c, ' ');
        attroff(COLOR_PAIR(PAIR_PAPER));
    }

    /* Pheromone field — 4-tier glyph ramp by intensity. */
    int    last_pair = -1;
    attr_t last_attr = 0;
    for (int r = 0; r < g_grid_h; r++) {
        for (int c = 0; c < g_grid_w; c++) {
            float p = g_ph[r][c];
            if (p < 0.04f) continue;
            int slot = (p < 0.15f) ? 0
                     : (p < 0.40f) ? 1
                     : (p < 0.70f) ? 2 : 3;
            char glyph = PH_GLYPHS[slot];
            int  pair  = PAIR_PH_BASE + slot;
            attr_t attr;
            if (inverted) {
                attr = A_NORMAL;
            } else {
                attr = (slot == 3) ? A_BOLD
                     : (slot == 0) ? A_DIM : A_NORMAL;
            }
            if (pair != last_pair || attr != last_attr) {
                if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
                attron(COLOR_PAIR(pair) | attr);
                last_pair = pair;
                last_attr = attr;
            }
            mvaddch(r, c, (chtype)(unsigned char)glyph);
        }
    }
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);

    /* Food sources — `[*]` framed marker. */
    attr_t food_attr = inverted ? A_NORMAL : A_BOLD;
    attron(COLOR_PAIR(PAIR_FOOD) | food_attr);
    for (int k = 0; k < g_food_n; k++) {
        int r = g_food_row[k], c = g_food_col[k];
        if (r < 0 || r >= g_grid_h) continue;
        if (c - 1 >= 0 && c - 1 < g_grid_w)
            mvaddch(r, c - 1, (chtype)(unsigned char)FOOD_FRAME_L);
        if (c >= 0 && c < g_grid_w)
            mvaddch(r, c,     (chtype)(unsigned char)FOOD_GLYPH);
        if (c + 1 >= 0 && c + 1 < g_grid_w)
            mvaddch(r, c + 1, (chtype)(unsigned char)FOOD_FRAME_R);
    }
    attroff(COLOR_PAIR(PAIR_FOOD) | food_attr);

    /* Nest — `(@)` framed marker. */
    attr_t nest_attr = inverted ? A_NORMAL : A_BOLD;
    attron(COLOR_PAIR(PAIR_NEST) | nest_attr);
    {
        int r = g_nest_row, c = g_nest_col;
        if (c - 1 >= 0)  mvaddch(r, c - 1, (chtype)(unsigned char)NEST_FRAME_L);
        if (c >= 0)      mvaddch(r, c,     (chtype)(unsigned char)NEST_GLYPH);
        if (c + 1 < g_grid_w)
                         mvaddch(r, c + 1, (chtype)(unsigned char)NEST_FRAME_R);
    }
    attroff(COLOR_PAIR(PAIR_NEST) | nest_attr);

    /* Ants — `o`, coloured by state. */
    attr_t ant_attr = inverted ? A_NORMAL : A_BOLD;
    for (int i = 0; i < N_ANTS; i++) {
        int r = g_ants[i].row, c = g_ants[i].col;
        if (r < 0 || r >= g_grid_h) continue;
        if (c < 0 || c >= g_grid_w) continue;
        int pair = (g_ants[i].state == RETURNING) ? PAIR_ANT_R : PAIR_ANT_S;
        attron(COLOR_PAIR(pair) | ant_attr);
        mvaddch(r, c, (chtype)(unsigned char)ANT_GLYPH);
        attroff(COLOR_PAIR(pair) | ant_attr);
    }
}

static int scene_count_returning(void)
{
    int n = 0;
    for (int i = 0; i < N_ANTS; i++)
        if (g_ants[i].state == RETURNING) n++;
    return n;
}

/* ===================================================================== */
/* §7  screen + app                                                       */
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

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_render(s);

    int returning = scene_count_returning();

    char buf[260];
    snprintf(buf, sizeof buf,
             " ANT_COLONY   %s   pattern:%s   theme:%s   "
             "ants:%d  ret:%2d  food:%4d   speed:%2dx   "
             "%5.1f fps  %3d Hz   "
             "n/N:pat  t/T:theme  +/-:speed  spc:pause  r:reset  q:quit ",
             s->paused ? "PAUSED " : "FORAGE ",
             pattern_name((Pattern)s->current_pattern),
             themes[s->current_theme].name,
             N_ANTS, returning, g_food_collected, s->speed,
             fps, sim_fps);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

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
    case ' ':
    case 'p': case 'P': s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reset(s);                               break;

    case 'n':           scene_cycle_pattern(s, +1);                   break;
    case 'N':           scene_cycle_pattern(s, -1);                   break;

    case 't':           scene_cycle_theme(s, +1);                     break;
    case 'T':           scene_cycle_theme(s, -1);                     break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed++;
        break;
    case '-':
        if (s->speed > SPEED_MIN) s->speed--;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
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
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    /*
     * Modern fixed-step main loop.  Input is checked every iteration
     * (60 fps default), so key responsiveness is independent of the
     * sim speed multiplier.  The `speed` field multiplies how many
     * sim ticks run per render frame — high speed makes the ant
     * convergence visibly faster but doesn't lag input.
     */
    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        /* Input first, every frame. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* Sim ticks: speed multiplier × per-frame steps. */
        for (int i = 0; i < app->scene.speed; i++)
            scene_tick(&app->scene);

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* Render-cap to sim_fps for steady frame pacing. */
        int64_t target_ns = TICK_NS(app->sim_fps);
        int64_t elapsed   = clock_ns() - frame_time + dt;
        clock_sleep_ns(target_ns - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();
    }

    screen_free(&app->screen);
    return 0;
}
