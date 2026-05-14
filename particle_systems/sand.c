/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sand.c  —  ncurses falling sand simulation
 *
 * Sand pours from a source at the top, falls under gravity,
 * and piles into natural slopes.
 *
 * Enhancements:
 *
 *   Wind — horizontal force that drifts falling grains sideways.
 *     w / W   wind right / left
 *     0       calm
 *     Wind deflects the emitter spray so the stream leans.
 *
 *   Per-grain age — each grain accumulates age each tick it is
 *     stationary.  Age drives both the GLYPH and the COLOUR RAMP:
 *       newly falling / spawned   '`'  ramp[0]  (airborne / brightest)
 *       just landed               '.'  ramp[1]  (fresh)
 *       settling                  'o'  ramp[2]  (light pack)
 *       settled surface           'O'  ramp[3]  (mid pack)
 *       compressed mid            '0'  ramp[4]  (packed)
 *       deep compressed base      '#'  ramp[5]  (dense / darkest)
 *     New grains are bright and individual.  Buried grains darken
 *     to dense packed material.  The actual ramp colours come from
 *     the active theme (t / T cycle through 10 themes).
 *
 * Keys:
 *   q / ESC     quit
 *   space       toggle emitter on / off
 *   p           pause / resume
 *   r           clear all sand
 *   Left/Right  move the emitter
 *   =  -        wider / narrower emitter
 *   ]  [        simulation faster / slower
 *   w           wind right (press multiple times to strengthen)
 *   W           wind left
 *   0           calm — no wind
 *   t  T        next / previous colour theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra sand.c -o sand -lncurses
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  grid   — CA + per-grain age + wind drift
 *   §5  source
 *   §6  scene
 *   §7  screen — single stdscr
 *   §8  app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Cellular Automaton (CA) with probabilistic update order.
 *                  Rules applied bottom-to-top (row y = rows-1 → 0) so grains
 *                  that fell this tick don't cascade infinitely in one step.
 *                  Column order is shuffled each row to prevent directional bias
 *                  (without shuffling, sand would always prefer the left diagonal).
 *
 * Physics        : Models angle of repose — the maximum slope stable sand can
 *                  maintain before avalanching.  The two-diagonal CA rule
 *                  approximates this: a grain slides diagonally if the cell
 *                  directly below is blocked, creating ~45° pile slopes.
 *                  Real dry sand angle of repose ≈ 30–35°; CA gives ~45° because
 *                  only one diagonal is checked at a time (no multi-step sliding).
 *
 * Wind           : Probabilistic horizontal drift for unsettled grains
 *                  (age < AGE_SMALL).  P(drift) = |wind| / WIND_PROB_DEN.
 *                  Only young grains drift — settled sand is too heavy to move,
 *                  matching the physical model of light airborne particles.
 *
 * Performance    : O(W×H) per tick. The `moved[]` flag array prevents double-
 *                  processing; without it a grain could move multiple cells per
 *                  tick. Per-tick scratch arrays (nxt, nxt_age) avoid in-place
 *                  mutation during the scan (double-buffering pattern).
 *
 * Visual model   : Per-grain age (stationary ticks) encodes visual compaction.
 *                  Age resets to 0 on any movement. This gives a natural visual
 *                  where freshly fallen grains glow bright and gradually darken
 *                  as they become buried — analogous to porosity decreasing under
 *                  compressive weight in real granular materials.
 *
 * References
 * ──────────
 *   PAPERS
 *     Bak, P., Tang, C. & Wiesenfeld, K. (1987)
 *       "Self-Organized Criticality: An Explanation of 1/f Noise"
 *       Physical Review Letters 59(4): 381-384.
 *       The original sandpile cellular automaton.  Exactly the
 *       topology grid_tick uses: a 2-D cell grid, a gravity-biased
 *       rule applied tick-by-tick, avalanches of arbitrary size
 *       emerging from local rules.  Reading this paper is the
 *       cleanest path to seeing why the ~45° pile angle emerges
 *       from a two-diagonal slide rule.
 *
 *     Wolfram, S. (1984)
 *       "Cellular Automata as Models of Complexity"
 *       Nature 311: 419-424.
 *       Foundational CA paper. Frames the probabilistic update
 *       order + shuffled column scan we use as a "stochastic CA",
 *       a refinement of classical CAs that breaks directional
 *       bias without changing the rule set.
 *
 *     Jaeger, H. M., Nagel, S. R. & Behringer, R. P. (1996)
 *       "Granular solids, liquids, and gases"
 *       Reviews of Modern Physics 68(4): 1259-1273.
 *       Comprehensive review of granular-media physics.  §III
 *       covers the angle of repose (real dry sand 30-35°; why
 *       static and dynamic angles differ) — useful context for
 *       understanding why our CA's ~45° is higher than the real
 *       physical angle.
 *
 *   BOOKS
 *     Wolfram, S. — "A New Kind of Science"
 *       (Wolfram Media, 2002).  Chapter 6 covers CAs as models
 *       for physical phenomena including granular flow; the
 *       per-row column-shuffle trick used in grid_tick to remove
 *       leftward drift bias is one of the refinements Wolfram
 *       discusses.
 *
 *     Duran, J. — "Sands, Powders, and Grains: An Introduction
 *       to the Physics of Granular Materials"
 *       (Springer, 2000).  Definitive textbook on granular
 *       physics.  Chapters 1-2 cover the angle of repose and
 *       compaction under weight — direct support for the per-
 *       grain age → ramp-tint visual encoding (older grains live
 *       under more compaction → darker tint).
 *
 *     Bagnold, R. A. — "The Physics of Blown Sand and Desert Dunes"
 *       (Methuen, 1941; reprinted Dover, 2005).
 *       The classic on aeolian (wind-driven) sand transport.
 *       §1-3 motivate why only LIGHT (young, age < AGE_SMALL)
 *       grains drift in wind — heavier embedded grains resist
 *       lift-off.  The same threshold our wind rule uses.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      = 10,
    SIM_FPS_DEFAULT  = 30,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,

    HUD_COLS         = 54,
    FPS_UPDATE_MS    = 500,

    SOURCE_ROW       =  1,
    SOURCE_W_DEFAULT =  3,
    SOURCE_W_MIN     =  1,
    SOURCE_W_MAX     = 30,

    WIND_MAX         =  3,   /* max wind strength (1=gentle drift, 3=strong gale) */

    /* Age thresholds — stationary ticks before advancing to next visual level.
     * At 30 fps: AGE_DOT=3 ticks ≈ 0.1s (airborne → landed),
     *            AGE_SMALL=12 ticks ≈ 0.4s (landed → settling),
     *            AGE_MID=30 ticks ≈ 1.0s (settling → settled surface),
     *            AGE_PACK=60 ticks ≈ 2.0s (settled → compressed mid),
     *            AGE_DENSE=120 ticks ≈ 4.0s (compressed → dense base).
     * Longer thresholds at deeper levels mimic real sand compaction:
     * surface grains re-arrange quickly; deep grains are locked in place. */
    AGE_DOT    =   3,
    AGE_SMALL  =  12,
    AGE_MID    =  30,
    AGE_PACK   =  60,
    AGE_DENSE  = 120,
    AGE_MAX    = 200,   /* cap: prevents uint8 overflow (200 < 255)          */
};

#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define TICK_NS(f)    (NS_PER_SEC/(f))

/* Wind drift probability denominator: P(drift) = abs(wind) / WIND_PROB_DEN */
#define WIND_PROB_DEN  4

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

enum {
    CP_NEW    =  1,  /* ramp[0] — airborne / freshly spawned (brightest) */
    CP_GRAIN  =  2,  /* ramp[1] — freshly landed                         */
    CP_LIGHT  =  3,  /* ramp[2] — settling                               */
    CP_MID    =  4,  /* ramp[3] — settled                                */
    CP_PACK   =  5,  /* ramp[4] — packed mid-layer                       */
    CP_DENSE  =  6,  /* ramp[5] — compressed base (darkest)              */
    CP_SOURCE =  7,  /* bright yellow    — emitter marker                */
    CP_WIND   =  8,  /* light blue       — wind indicator                */
    PAIR_HUD  =  9,  /* top status bar    — bright yellow on black       */
    PAIR_HINT = 10,  /* bottom key hints  — bright cyan   on black       */
};

/*
 * Theme — one 6-step grain-age ramp: bright (just-landed / airborne)
 * at ramp[0] → dark (compressed base) at ramp[5]. Selects the foreground
 * colour for CP_NEW..CP_DENSE; CP_SOURCE / CP_WIND / HUD / HINT pairs
 * stay theme-independent for consistent legibility.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule (≥ 24 in cube, ≥ 240 in gray;
 * 24-29 and 240-243 used only at the darkest ramp tier).
 */
typedef struct {
    const char *name;
    short       ramp[6];   /* NEW (bright) → DENSE (dark)                */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        ramp[0..5]  (NEW → DENSE) */
    { "MATRIX",  { 193, 157, 121,  84,  46,  28 } },
    { "FIRE",    { 229, 220, 208, 196, 124,  88 } },
    { "OCEANIC", { 195, 159,  87,  38,  31,  24 } },
    { "NEON",    { 225, 219, 213, 207, 165,  91 } },
    { "MONO",    { 255, 252, 248, 246, 244, 240 } },
    { "ICE",     { 231, 195, 153, 117,  67,  24 } },
    { "NOVA",    { 231, 226, 220, 208, 202, 130 } },
    { "FOREST",  { 192, 156, 112,  70,  64,  28 } },
    { "DESERT",  { 230, 229, 220, 178, 136, 130 } },
    { "ECLIPSE", { 217, 209, 173, 167,  95,  52 } },
};

/* Reload the 6 grain-age pairs from one theme. Safe to call mid-run;
 * takes effect on the next rendered frame. 8-colour fallback ignores
 * themes — the gradient is subtle and only meaningful in 256-colour. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS < 256) return;
    const Theme *t = &themes[idx];
    init_pair(CP_NEW,   t->ramp[0], COLOR_BLACK);
    init_pair(CP_GRAIN, t->ramp[1], COLOR_BLACK);
    init_pair(CP_LIGHT, t->ramp[2], COLOR_BLACK);
    init_pair(CP_MID,   t->ramp[3], COLOR_BLACK);
    init_pair(CP_PACK,  t->ramp[4], COLOR_BLACK);
    init_pair(CP_DENSE, t->ramp[5], COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        /* Grain-age ramp is set by theme_apply below. */
        init_pair(CP_SOURCE, 226, COLOR_BLACK);
        init_pair(CP_WIND,   117, COLOR_BLACK);
        init_pair(PAIR_HUD,  226, COLOR_BLACK);   /* bright yellow */
        init_pair(PAIR_HINT,  51, COLOR_BLACK);   /* bright cyan   */
    } else {
        init_pair(CP_NEW,    COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_GRAIN,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_LIGHT,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_MID,    COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_PACK,   COLOR_RED,    COLOR_BLACK);
        init_pair(CP_DENSE,  COLOR_RED,    COLOR_BLACK);
        init_pair(CP_SOURCE, COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_WIND,   COLOR_CYAN,   COLOR_BLACK);
        init_pair(PAIR_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(PAIR_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* default = MATRIX (first in cycle) */
}

/* ===================================================================== */
/* §4  grid  — CA + per-grain age + wind                                 */
/* ===================================================================== */

typedef uint8_t Cell;

/*
 * Grid — the cellular-automaton state.
 *
 * INTENT: hold the whole sand world (occupancy + per-grain compaction
 * history + wind) in a flat layout that fits in BSS (allocated once at
 * init, freed only on resize/exit; never reallocated per-tick).
 *
 * DOUBLE-BUFFER PATTERN: every tick reads from cur/age and writes the
 * result to nxt/nxt_age; the buffers swap at end-of-tick. Without
 * this, in-place writes during the scan would let a grain see itself
 * as already-moved one row later — grains would teleport to the floor
 * in a single tick. The `moved[]` flag layers a second guard on top
 * so a grain that fell from row y → y+1 is not re-scanned and fallen
 * again at y+1 within the same tick.
 *
 *   cur     : [rows*cols] CURRENT-frame cell state. 0 = empty,
 *             1 = sand. Read-only during a tick. Read by every rule
 *             check and by grid_neighbors (which informs the visual
 *             compaction count).
 *
 *   nxt     : [rows*cols] NEXT-frame cell state — the write target
 *             during the tick. Swapped into cur at end-of-tick.
 *
 *   age     : [rows*cols] stationary-tick counter per grain, byte
 *             [0, 255] saturating. Drives the colour-ramp lookup:
 *             age 0 → ramp[0] (bright fresh) up to age ≥ AGE_DENSE
 *             → ramp[5] (dark packed). RESETS to 0 the moment a
 *             grain moves — so a freshly-fallen grain glows bright,
 *             and only undisturbed pile interiors darken over time.
 *
 *   nxt_age : [rows*cols] age for next state. Same double-buffer
 *             role as nxt — written during the tick, swapped at end.
 *
 *   moved   : [rows*cols] per-tick guard flag, cleared at the top of
 *             each tick. Set on every cell whose grain has been
 *             processed this tick (either kept put or relocated to)
 *             so the scan does not revisit the same grain from its
 *             new position. Critical for keeping grains to a single-
 *             cell move per tick rather than free-fall in one step.
 *
 *   cols    : grid width in cells. Cached at alloc time; matches the
 *             terminal column count at init/resize. Used by every
 *             loop and the gidx(g,x,y) index helper.
 *
 *   rows    : grid height in cells. Same role as cols, vertical.
 *
 *   wind    : signed horizontal wind, range [-WIND_MAX, +WIND_MAX].
 *             Positive = blowing right, negative = blowing left,
 *             zero = calm. Mutated by w / W / 0 keys; persists across
 *             pause and resize. Drives TWO behaviours:
 *               1. source_emit shifts the spawn column by `wind` so
 *                  the stream visually leans INTO the wind.
 *               2. grid_tick adds horizontal drift to YOUNG grains
 *                  (age < AGE_SMALL) with probability
 *                  |wind| / WIND_PROB_DEN — heavy buried grains
 *                  resist lift-off (Bagnold), the same threshold
 *                  the wind rule uses.
 */
typedef struct {
    Cell    *cur;
    Cell    *nxt;
    uint8_t *age;
    uint8_t *nxt_age;
    bool    *moved;
    int      cols;
    int      rows;
    int      wind;
} Grid;

static void grid_alloc(Grid *g, int cols, int rows)
{
    g->cols    = cols;
    g->rows    = rows;
    g->cur     = calloc((size_t)(cols*rows), sizeof(Cell));
    g->nxt     = calloc((size_t)(cols*rows), sizeof(Cell));
    g->age     = calloc((size_t)(cols*rows), sizeof(uint8_t));
    g->nxt_age = calloc((size_t)(cols*rows), sizeof(uint8_t));
    g->moved   = calloc((size_t)(cols*rows), sizeof(bool));
    g->wind    = 0;
}
static void grid_free(Grid *g)
{
    free(g->cur); free(g->nxt);
    free(g->age); free(g->nxt_age);
    free(g->moved);
    *g = (Grid){0};
}
static void grid_clear(Grid *g)
{
    size_t n = (size_t)(g->cols * g->rows);
    memset(g->cur,     0, n*sizeof(Cell));
    memset(g->nxt,     0, n*sizeof(Cell));
    memset(g->age,     0, n*sizeof(uint8_t));
    memset(g->nxt_age, 0, n*sizeof(uint8_t));
    memset(g->moved,   0, n*sizeof(bool));
}

static inline bool gin(const Grid *g, int x, int y)
    { return x>=0 && x<g->cols && y>=0 && y<g->rows; }
static inline int  gidx(const Grid *g, int x, int y)
    { return y*g->cols+x; }
static inline Cell gget(const Grid *g, int x, int y)
    { if (!gin(g,x,y)) return 1; return g->cur[gidx(g,x,y)]; }
static inline void gset_cur(Grid *g, int x, int y, Cell v)
    { if (gin(g,x,y)) g->cur[gidx(g,x,y)]=v; }
static inline bool gmoved(const Grid *g, int x, int y)
    { return gin(g,x,y) && g->moved[gidx(g,x,y)]; }
static inline void gmark(Grid *g, int x, int y)
    { if (gin(g,x,y)) g->moved[gidx(g,x,y)]=true; }

/* Move grain (sx,sy)→(dx,dy): clear source, set dest, reset age, mark both */
static void gmove(Grid *g, int sx, int sy, int dx, int dy)
{
    g->nxt[gidx(g,sx,sy)]     = 0;
    g->nxt_age[gidx(g,sx,sy)] = 0;
    g->nxt[gidx(g,dx,dy)]     = 1;
    g->nxt_age[gidx(g,dx,dy)] = 0;
    gmark(g,sx,sy); gmark(g,dx,dy);
}

static void grid_update_cell(Grid *g, int x, int y)
{
    if (gget(g,x,y) != 1) return;
    if (gmoved(g,x,y))    return;

    int i = gidx(g,x,y);

    /* 1. Straight down */
    if (gin(g,x,y+1) && gget(g,x,y+1)==0 && !gmoved(g,x,y+1)) {
        gmove(g,x,y,x,y+1); return;
    }

    /* 2. Diagonal down — random priority */
    int a = (rand()&1) ? -1 : 1, b = -a;
    if (gin(g,x+a,y+1) && gget(g,x+a,y+1)==0 && !gmoved(g,x+a,y+1)) {
        gmove(g,x,y,x+a,y+1); return;
    }
    if (gin(g,x+b,y+1) && gget(g,x+b,y+1)==0 && !gmoved(g,x+b,y+1)) {
        gmove(g,x,y,x+b,y+1); return;
    }

    /* 3. Wind drift — only light/young grains (not yet settled) */
    if (g->wind != 0 && g->age[i] < AGE_SMALL) {
        int dir  = (g->wind > 0) ? 1 : -1;
        int wabs = abs(g->wind);
        if ((rand() % WIND_PROB_DEN) < wabs) {
            int wx = x + dir;
            if (gin(g,wx,y) && gget(g,wx,y)==0 && !gmoved(g,wx,y)) {
                gmove(g,x,y,wx,y); return;
            }
        }
    }

    /* 4. At rest — copy, increment age */
    g->nxt[i]     = 1;
    g->nxt_age[i] = (g->age[i] < AGE_MAX) ? g->age[i]+1 : AGE_MAX;
    gmark(g,x,y);
}

static void grid_tick(Grid *g)
{
    int    cols = g->cols, rows = g->rows;
    size_t n    = (size_t)(cols*rows);

    memset(g->nxt,     0, n*sizeof(Cell));
    memset(g->nxt_age, 0, n*sizeof(uint8_t));
    memset(g->moved,   0, n*sizeof(bool));

    int *order = malloc((size_t)cols * sizeof(int));
    for (int x = 0; x < cols; x++) order[x] = x;

    for (int y = rows-1; y >= 0; y--) {
        for (int i = cols-1; i > 0; i--) {
            int j = rand()%(i+1);
            int t = order[i]; order[i]=order[j]; order[j]=t;
        }
        for (int i = 0; i < cols; i++)
            grid_update_cell(g, order[i], y);
    }
    free(order);

    Cell    *tc = g->cur;     g->cur     = g->nxt;     g->nxt     = tc;
    uint8_t *ta = g->age;     g->age     = g->nxt_age; g->nxt_age = ta;
}

static int grid_neighbors(const Grid *g, int x, int y)
{
    int n = 0;
    for (int dy=-1; dy<=1; dy++)
        for (int dx=-1; dx<=1; dx++)
            if ((dx||dy) && gin(g,x+dx,y+dy))
                n += g->cur[gidx(g,x+dx,y+dy)];
    return n;
}

/*
 * grain_visual() — map (age, neighbour_count) → (char, attr).
 *
 * Age drives the primary level selection.  Neighbour count suppresses
 * the lightest chars for grains surrounded by settled sand — prevents
 * a freshly dropped grain inside a pile from glowing bright.
 */
static void grain_visual(uint8_t age, int nb, char *ch_out, attr_t *attr_out)
{
    int eff = (int)age;
    if (nb >= 5 && eff < AGE_MID)   eff = AGE_MID;
    if (nb >= 3 && eff < AGE_SMALL) eff = AGE_SMALL;

    char   ch;
    attr_t attr;
    if      (eff < AGE_DOT)   { ch='`'; attr=COLOR_PAIR(CP_NEW)  |A_BOLD; }
    else if (eff < AGE_SMALL) { ch='.'; attr=COLOR_PAIR(CP_GRAIN)|A_BOLD; }
    else if (eff < AGE_MID)   { ch='o'; attr=COLOR_PAIR(CP_LIGHT)|A_BOLD; }
    else if (eff < AGE_PACK)  { ch='O'; attr=COLOR_PAIR(CP_MID);          }
    else if (eff < AGE_DENSE) { ch='0'; attr=COLOR_PAIR(CP_PACK);         }
    else                      { ch='#'; attr=COLOR_PAIR(CP_DENSE);        }
    *ch_out   = ch;
    *attr_out = attr;
}

/* ===================================================================== */
/* §5  source                                                             */
/* ===================================================================== */

/*
 * Source — the sand spigot at the top of the grid.
 *
 * INTENT: deposit one grain at each of `w` cells centred on `col`
 * (offset by grid.wind so the stream leans into the wind) at row
 * SOURCE_ROW, once per tick when `on` is true.  The source itself
 * is not a grain — source_emit just sets cur[x, SOURCE_ROW] = 1
 * with age = 0, then the CA in grid_tick takes over.
 *
 *   col : horizontal CENTRE of the emitter, in cells. Initialised to
 *         the middle of the grid by source_init. Mutated by LEFT /
 *         RIGHT arrow keys; clamped to [1, cols-2] so the full spray
 *         always stays on screen even at maximum width.
 *
 *   w   : full width of the spray in cells (NOT half-width — internal
 *         use is w/2 as the half-extent). Adjusted by + / - keys,
 *         clamped to [SOURCE_W_MIN, SOURCE_W_MAX]. Wider = thicker
 *         column of sand; narrower = thin stream.
 *
 *   on  : emission enabled? Toggled by space. When OFF the source
 *         MARKER still draws (so the user can see where it would
 *         spray) but no new grains are deposited — useful for
 *         watching an existing pile settle without new arrivals.
 */
typedef struct { int col, w; bool on; } Source;

static void source_init(Source *s, int cols)
{
    s->col = cols/2;
    s->w   = SOURCE_W_DEFAULT;
    s->on  = true;
}

static void source_emit(const Source *s, Grid *g)
{
    if (!s->on) return;
    int half        = s->w / 2;
    int wind_offset = g->wind;   /* lean stream into wind */
    for (int dx = -half; dx <= half; dx++) {
        int x = s->col + dx + wind_offset;
        if (x < 0) x = 0;
        if (x >= g->cols) x = g->cols-1;
        if (gin(g,x,SOURCE_ROW) && gget(g,x,SOURCE_ROW)==0) {
            gset_cur(g,x,SOURCE_ROW,1);
            g->age[gidx(g,x,SOURCE_ROW)] = 0;
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — owns every piece of mutable state for the sand simulation.
 * Two clearly-separated halves:
 *
 *   SIMULATION half — what scene_tick reads + writes. Owns the CA
 *                     grid (with double-buffers, age counters, and
 *                     wind), the spawn source, and the pause flag.
 *                     Mutated by scene_tick and the key handler.
 *
 *   RENDER half     — what scene_draw / screen_draw consult to pick
 *                     colours. Purely visual selection index; never
 *                     read inside the physics tick.
 *
 * The Scene knows nothing about ncurses — physics writes to the grid,
 * the render layer reads it. That separation lets the CA be
 * exercised without a terminal (useful for headless tests or
 * snapshot dumps).
 */
typedef struct {
    /* ──────────────────────────────────────────────────────────────
     *  SIMULATION HALF — physics tick reads + writes these
     * ────────────────────────────────────────────────────────────── */

    /* CELL GRID — the CA state. Holds the cur/nxt double-buffers,
     * the per-grain age counter, the moved-this-tick guard, the
     * grid dimensions, and the wind value. See Grid documentation
     * for per-field detail.  Allocated once at scene_init and freed
     * at scene_free; on resize, freed and reallocated to the new
     * terminal dims (wind value preserved across resize). */
    Grid      grid;

    /* SAND SPIGOT — where new grains spawn. Position + width are
     * mutated by arrow keys and +/- keys; on/off toggled by space.
     * source_emit writes grains into grid.cur at row SOURCE_ROW
     * each tick (offset by grid.wind so the stream leans into the
     * wind). */
    Source    source;

    /* PAUSE FLAG — scene_tick is a no-op when set. Toggled by p / P.
     * Render keeps running so the user sees the frozen pile + any
     * mid-air grains held in place. Useful for inspecting the
     * pile shape after an avalanche. */
    bool      paused;

    /* ──────────────────────────────────────────────────────────────
     *  RENDER HALF — scene_draw reads this; physics tick ignores it
     * ────────────────────────────────────────────────────────────── */

    /* THEME — index into themes[]. Cycled by t / T. Selects the
     * 6-step grain-age ramp (NEW → DENSE) that maps to CP_NEW..
     * CP_DENSE pair indices. Pure render concern — grain physics
     * (CA rules, age accumulation, wind drift) behaves identically
     * regardless of which theme is active. theme_apply(idx) re-runs
     * init_pair for the six ramp pairs whenever this changes. */
    int       current_theme;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s,0,sizeof*s);
    grid_alloc(&s->grid, cols, rows);
    source_init(&s->source, cols);
}
static void scene_free(Scene *s)  { grid_free(&s->grid); }
static void scene_resize(Scene *s, int cols, int rows)
{
    int wind = s->grid.wind;
    grid_free(&s->grid);
    grid_alloc(&s->grid, cols, rows);
    s->grid.wind = wind;
    if (s->source.col >= cols) s->source.col = cols/2;
}
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    source_emit(&s->source, &s->grid);
    grid_tick(&s->grid);
}

/* ── Per-cell render helpers ─────────────────────────────────────── */

/* Paint one occupied cell. Looks up its compaction signal
 *   (age, neighbour-count)
 * → picks glyph + attr via grain_visual → writes to the window.
 * Caller has already confirmed cur[x,y] == 1. */
static inline void grain_paint_one(WINDOW *w, const Grid *g, int x, int y)
{
    uint8_t age = g->age[gidx(g, x, y)];
    int     nb  = grid_neighbors(g, x, y);
    char    ch;
    attr_t  attr;
    grain_visual(age, nb, &ch, &attr);

    wattron (w, attr);
    mvwaddch(w, y, x, (chtype)(unsigned char)ch);
    wattroff(w, attr);
}

/* ── Layer renderers ─────────────────────────────────────────────── */

/* GRAIN LAYER — sweep the cell grid, paint every occupied cell.
 * O(cols·rows) per frame with an early-skip on empty cells, so
 * a screen that's 10% full only pays for the occupied cells. */
static void grid_render_grains(WINDOW *w, const Grid *g)
{
    for (int y = 0; y < g->rows; y++) {
        for (int x = 0; x < g->cols; x++) {
            if (g->cur[gidx(g, x, y)] == 0) continue;
            grain_paint_one(w, g, x, y);
        }
    }
}

/* SOURCE MARKER — chevron strip ('v') above the emitter row showing
 * where new grains will spawn this tick. The strip is offset by
 * grid.wind so it visually leans INTO the wind, matching where
 * source_emit will actually deposit. */
static void source_render_marker(WINDOW *w, const Source *src, const Grid *g)
{
    int half = src->w / 2;
    int wo   = g->wind;

    wattron(w, COLOR_PAIR(CP_SOURCE) | A_BOLD);
    for (int dx = -half; dx <= half; dx++) {
        int mx = src->col + dx + wo;
        if (mx >= 0 && mx < g->cols)
            mvwaddch(w, SOURCE_ROW - 1, mx, 'v');
    }
    wattroff(w, COLOR_PAIR(CP_SOURCE) | A_BOLD);
}

/* WIND INDICATOR — arrows along the bottom row whose count = |wind|
 * and direction = sign(wind). Capped at cols/4 so a high wind never
 * overwhelms the bottom-row HUD. Layout:
 *   wind > 0  → arrows on the RIGHT edge pointing '>'
 *   wind < 0  → arrows on the LEFT edge pointing '<'
 *   wind == 0 → no indicator (caller short-circuits). */
static void wind_render_indicator(WINDOW *w, const Grid *g)
{
    char wc   = (g->wind > 0) ? '>' : '<';
    int  wabs = abs(g->wind);
    int  cap  = g->cols / 4;

    wattron(w, COLOR_PAIR(CP_WIND) | A_BOLD);
    for (int i = 0; i < wabs && i < cap; i++) {
        int wx = (g->wind > 0) ? g->cols - 1 - i : i;
        if (wx >= 0 && wx < g->cols && g->rows > 1)
            mvwaddch(w, g->rows - 1, wx, (chtype)(unsigned char)wc);
    }
    wattroff(w, COLOR_PAIR(CP_WIND) | A_BOLD);
}

/* ── Driver — render all scene layers in z-order ─────────────────── */

/* Pseudocode (back-to-front so later layers overdraw earlier ones):
 *   grid_render_grains       — sand cells via per-grain age+neighbour glyph
 *   source_render_marker     — emitter chevrons (only when on)
 *   wind_render_indicator    — bottom-row direction gauge (only when wind != 0) */
static void scene_draw(const Scene *s, WINDOW *w)
{
    grid_render_grains(w, &s->grid);

    if (s->source.on)
        source_render_marker(w, &s->source, &s->grid);

    if (s->grid.wind != 0)
        wind_render_indicator(w, &s->grid);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr,TRUE); keypad(stdscr,TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr,s->rows,s->cols); }

/*
 * screen_draw — render the grid, then paint a two-layer HUD over it:
 *
 *   Row 0          STATUS LINE.  Bright yellow PAIR_HUD + A_BOLD.
 *                  Live state: paused/run, emitter on/off, source
 *                  column + width, wind direction + magnitude, fps,
 *                  sim Hz.
 *   Row rows-1     KEY HINT LINE.  Bright cyan PAIR_HINT + A_BOLD.
 *                  Every interactive key the demo accepts.
 *
 * Both rows are pre-filled with their pair colour so the coloured
 * background spans the full width, and drawn AFTER scene_draw so
 * sand grains never bleed through the bars.
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);

    /* ── Top row: dynamic status ─────────────────────────────── */
    const char *wstr =
        sc->grid.wind > 0 ? ">>>" :
        sc->grid.wind < 0 ? "<<<" : "---";

    char status[200];
    snprintf(status, sizeof status,
             " SAND   %s   theme:%-7s   emit:%s   src col:%3d  w:%d   "
             "wind:%s (%+d)   %5.1f fps  %3d Hz ",
             sc->paused     ? "PAUSED " : "running",
             themes[sc->current_theme].name,
             sc->source.on  ? "ON " : "OFF",
             sc->source.col, sc->source.w,
             wstr, sc->grid.wind,
             fps, sim_fps);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < s->cols; x++) mvaddch(0, x, ' ');
    mvprintw(0, 0, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* ── Bottom row: every interactive key ───────────────────── */
    const char *hints =
        " q:quit  spc:emit  p:pause  r:clear  </>:move  +/-:width  "
        "w/W:wind  0:calm  t/T:theme  ]/[:Hz ";

    int hint_row = s->rows - 1;
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int x = 0; x < s->cols; x++) mvaddch(hint_row, x, ' ');
    mvprintw(hint_row, 0, "%s", hints);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene  scene;
    Screen screen;
    int    sim_fps;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit_signal(int s)   { (void)s; g_app.running=0;     }
static void on_resize_signal(int s) { (void)s; g_app.need_resize=1; }
static void cleanup(void)           { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene  *sc  = &app->scene;
    Source *src = &sc->source;
    Grid   *g   = &sc->grid;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':   src->on    = !src->on;       break;
    case 'p': case 'P': sc->paused = !sc->paused; break;
    case 'r': case 'R': grid_clear(g);       break;

    case KEY_LEFT:
        src->col--; if (src->col<1) src->col=1; break;
    case KEY_RIGHT:
        src->col++; if (src->col>=g->cols-1) src->col=g->cols-2; break;

    case '=': case '+':
        src->w++; if (src->w>SOURCE_W_MAX) src->w=SOURCE_W_MAX; break;
    case '-':
        src->w--; if (src->w<SOURCE_W_MIN) src->w=SOURCE_W_MIN; break;

    case 'w':
        g->wind++; if (g->wind> WIND_MAX) g->wind= WIND_MAX; break;
    case 'W':
        g->wind--; if (g->wind<-WIND_MAX) g->wind=-WIND_MAX; break;
    case '0':
        g->wind = 0; break;

    case 't':
        sc->current_theme = (sc->current_theme + 1) % N_THEMES;
        theme_apply(sc->current_theme);
        break;
    case 'T':
        sc->current_theme = (sc->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(sc->current_theme);
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
    srand((unsigned int)clock_ns());
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
        if (dt > 100*NS_PER_MS) dt = 100*NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
            sim_accum -= tick_ns;
        }

        /*
         * ── render interpolation alpha ────────────────────────────
         *
         * sim_accum is the leftover nanoseconds after draining all
         * complete ticks — how far we are into the next tick that
         * has not fired yet.
         *
         * alpha = sim_accum / tick_ns  ∈ [0.0, 1.0)
         *
         * Sand cells snap to integer grid positions so alpha is not
         * used in the draw call, but it is computed here for
         * structural consistency and future sub-cell interpolation.
         */
        float alpha = (float)sim_accum / (float)tick_ns;
        (void)alpha;

        /* ── FPS counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS*NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum/(double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t elapsed = clock_ns()-frame_time+dt;
        clock_sleep_ns(NS_PER_SEC/60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
