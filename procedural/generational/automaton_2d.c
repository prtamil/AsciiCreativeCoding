/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * automaton_2d.c — Larger-than-Life / Extended 2-D Cellular Automaton
 *
 * Generalised CA with configurable neighbourhood radius R (1–5),
 * birth/survival count thresholds, and up to 8 cell states.
 *
 * Extends life.c (Conway's GoL with R=1 bitmask rules) to arbitrary R.
 * Radius-2+ rules produce exotic crystal structures, spirals, and moving
 * "blobs" that are impossible with R=1.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map  (follows framework.c §1–§8 skeleton)
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  coords   (reference template; CA lives entirely in cell space)
 *   §5  entity   — CA grid, Rule struct, prefix-sum machinery, step
 *   §6  scene
 *   §7  screen
 *   §8  app
 * ─────────────────────────────────────────────────────────────────────
 *
 * KEY ALGORITHM — 2-D Prefix Sum for O(1) neighbourhood queries
 * ─────────────────────────────────────────────────────────────────────
 * Naive approach: sum all (2R+1)² cells per grid cell = O(R²) per cell.
 * At R=5 on a 200×50 terminal that is 121 additions × 10,000 cells =
 * 1.2 million additions per generation — fine in C, but gets expensive
 * fast as terminal grows and R increases.
 *
 * Prefix sum approach (2D summed area table):
 *   Build P[i][j] = sum of alive cells in rows [0..i-1], cols [0..j-1].
 *   Then any rectangle sum (r0,c0)→(r1,c1) is one O(1) query:
 *
 *     sum = P[r1+1][c1+1] - P[r0][c1+1] - P[r1+1][c0] + P[r0][c0]
 *
 *   Total per generation: O(W×H) to build P, O(W×H) to query = O(W×H).
 *   No dependence on R at all.
 *
 * TOROIDAL WRAPPING
 * ─────────────────────────────────────────────────────────────────────
 * Naive approach: branch per-cell on boundary condition.
 * This approach: pad the grid with R-wide toroidal copies on all sides
 * before building the prefix sum.  Every neighbourhood query then lies
 * entirely inside the padded grid — no branching, no modular arithmetic
 * in the hot loop.
 *
 * Padded grid layout (R=2 example):
 *
 *   ┌──────────────────────────────┐
 *   │  wrap  │   original   │ wrap │  ← R rows top (bottom of original)
 *   ├────────┼──────────────┤──────┤
 *   │  wrap  │   original   │ wrap │  ← original rows
 *   ├────────┼──────────────┤──────┤
 *   │  wrap  │   original   │ wrap │  ← R rows bottom (top of original)
 *   └──────────────────────────────┘
 *
 * Cell (r,c) in the original grid occupies padded position (r+R, c+R).
 * Its (2R+1)×(2R+1) neighbourhood spans padded rows [r .. r+2R],
 * padded cols [c .. c+2R] — always a valid non-wrapping rectangle.
 *
 * RULE NOTATION (Larger-than-Life / LtL)
 * ─────────────────────────────────────────────────────────────────────
 *   R        neighbourhood radius (Chebyshev / Moore)
 *   N        number of states (2=binary, >2=multi-state with dying trail)
 *   B_min    minimum live-neighbour count for dead cell to be born
 *   B_max    maximum live-neighbour count for dead cell to be born
 *   S_min    minimum live-neighbour count for alive cell to survive
 *   S_max    maximum live-neighbour count for alive cell to survive
 *
 * Centre cell excluded from the neighbour count.
 * Max neighbours = (2R+1)² - 1.
 *
 * Multi-state dying trail:
 *   State N-1 = fully alive (contributes to birth/survival counts)
 *   States 1..N-2 = "dying" — do not count as alive neighbours,
 *                             decay by 1 each generation
 *   State 0 = dead
 *
 * Presets:
 *   Bosco  R=5 N=2 B=34..58 S=34..58  — amoeba-like blobs
 *   Bugs   R=5 N=6 B=34..45 S=34..58  — colorful bugs with trails
 *   Gnarl  R=1 N=2 B=1..1  S=1..1   — chaotic fractal growth
 *   Amoeba R=2 N=5 B=9..16 S=5..14   — organic flowing shapes
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          re-randomise grid
 *   n / p      next / previous preset
 *   + / =      more generations per tick
 *   -          fewer generations per tick
 *   ] / [      raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra automaton_2d.c -o automaton_2d -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Extended 2-D Cellular Automaton (Larger-than-Life / LtL).
 *                  Generalises Conway's Game of Life to radius R > 1.
 *                  Neighbourhood count = number of alive cells in a (2R+1)²
 *                  square.  Birth/survive thresholds are tunable per preset.
 *
 * Math           : Key optimisation: 2-D prefix sum (summed area table).
 *                  Naïve O((2R+1)²) sum per cell becomes O(1) per cell after
 *                  O(W×H) table construction:
 *                    P[r][c] = grid[r][c] + P[r-1][c] + P[r][c-1] − P[r-1][c-1]
 *                  Rectangle sum (r0,c0)→(r1,c1):
 *                    P[r1][c1] − P[r0-1][c1] − P[r1][c0-1] + P[r0-1][c0-1]
 *                  At R=5: naïve=121 additions; prefix=1 lookup (121× faster).
 *
 * Data-structure : Double-buffered grid arrays (cur/nxt) for simultaneous
 *                  update: all next-state values computed from cur, then swap.
 *                  The prefix sum array P must be rebuilt each generation.
 *
 * Performance    : O(W×H) per generation with prefix-sum optimisation.
 *                  Without it: O(W×H×R²); at R=5, W=200, H=50 → 1.2M vs 10K ops.
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
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 20,    /* CA ticks slower than physics animations  */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    HUD_ROW         =  0,    /* HUD drawn on row 0, top of screen        */
    FPS_UPDATE_MS   = 500,

    N_PRESETS       =  4,
    R_MAX           =  5,    /* max radius; sets allocation upper bound  */
    N_MAX           =  8,    /* max states                               */

    GENS_DEF        =  1,    /* CA generations per sim tick              */
    GENS_MAX        = 16,

    SEED_DENSITY    = 30,    /* alive probability at init (percent)      */
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
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
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Color strategy for multi-state CAs:
 *
 * We define 8 pairs (indices 1..8).  Pairs 1..N_MAX cover the state
 * gradient from fully alive (pair 1) to almost dead (pair N_MAX-1).
 * Pair 8 is reserved for HUD text.
 *
 * ca_set_preset() calls color_set_palette() to redefine pairs 1..(N-1)
 * with the preset's colour scheme whenever the user switches rules.
 * Redefining pairs mid-run is safe and does not flicker on any modern
 * terminal.
 *
 * Fallback: 8-color terminals receive a single colour (green/cyan/etc.)
 * for all alive states — no gradient but still correct behaviour.
 */

/* 256-color palettes, one per preset: palettes[p][state_index] */
/* state_index 0 = fully alive (state N-1), 1 = dying-1, ... */
static const short PALETTES_256[N_PRESETS][N_MAX] = {
/*  alive   d1     d2     d3     d4     d5     d6  hud */
    { 75,   75,    75,    75,    75,    75,    75,  226 },  /* Bosco  light-blue  */
    { 226,  208,   196,   124,   88,    52,    52,  226 },  /* Bugs   fire  */
    { 51,   51,    51,    51,    51,    51,    51,  51  },  /* Gnarl  cyan  */
    { 255,  153,   117,   81,    45,    45,    45,  46  },  /* Amoeba white→blue */
};

static const short PALETTES_8[N_PRESETS] = {
    COLOR_BLUE, COLOR_YELLOW, COLOR_CYAN, COLOR_WHITE,
};

static int g_n256 = 0;   /* set in color_init */

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_n256 = (COLORS >= 256);
    /* Pairs 1..8 will be set by color_set_palette(); zero-init now */
    for (int i = 1; i <= 8; i++)
        init_pair((short)i, COLOR_WHITE, COLOR_BLACK);
}

/*
 * color_set_palette() — redefine pairs 1..(N-1) for a preset.
 * Pair 8 is always the HUD color (yellow or green depending on preset).
 */
static void color_set_palette(int preset, int N)
{
    const short *pal = PALETTES_256[preset];
    if (g_n256) {
        for (int i = 0; i < N - 1 && i < N_MAX; i++)
            init_pair((short)(i + 1), pal[i], COLOR_BLACK);
        init_pair(8, pal[7], COLOR_BLACK);   /* HUD */
    } else {
        short c = PALETTES_8[preset];
        for (int i = 1; i <= N - 1; i++)
            init_pair((short)i, c, COLOR_BLACK);
        init_pair(8, COLOR_YELLOW, COLOR_BLACK);
    }
}

/* Map cell state to ncurses attribute (pair + bold/dim) */
static chtype state_attr(int state, int N)
{
    if (state == N - 1)
        return (chtype)(COLOR_PAIR(1) | A_BOLD);       /* fully alive: bright */
    /* dying states: dim, map to pairs 2..N-1 */
    int idx = (N - 1 - state);   /* 1 = barely dying, N-2 = almost dead */
    if (idx >= N_MAX - 1) idx = N_MAX - 2;
    return (chtype)(COLOR_PAIR(idx + 1) | A_DIM);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell reference (CA works entirely in cell space)   */
/* ===================================================================== */

/*
 * This CA simulation operates directly in cell (terminal) coordinates.
 * Each grid cell corresponds to one terminal character cell.
 * No pixel↔cell conversion is needed.
 *
 * §4 is retained as the reference template entry point for any future
 * animation derived from this file that adds continuous-motion physics.
 *
 * When you add motion (e.g. a ball ejected from a CA explosion):
 *   • Store the ball's position in PIXEL SPACE (below).
 *   • Convert to cell space ONLY in scene_draw via px_to_cell_x/y.
 *   • See bounce_ball.c §4 for the full explanation.
 */
#define CELL_W   8
#define CELL_H  16

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — CA (Cellular Automaton)                                   */
/* ===================================================================== */

/* ── Rule ────────────────────────────────────────────────────────────── */

typedef struct {
    int         radius;   /* R: Moore neighbourhood radius 1..5        */
    int         states;   /* N: number of cell states 2..N_MAX         */
    int         b_min;    /* minimum alive-neighbour count for birth    */
    int         b_max;    /* maximum alive-neighbour count for birth    */
    int         s_min;    /* minimum alive-neighbour count for survival */
    int         s_max;    /* maximum alive-neighbour count for survival */
    const char *name;
} Rule;

/*
 * Preset table.  Max neighbours = (2R+1)² - 1 (centre excluded).
 *   R=1 → 8 neighbours
 *   R=2 → 24 neighbours
 *   R=5 → 120 neighbours
 *
 * Bosco  (R=5 N=2): birth/survive at 34–58 of 120 ≈ 28–48%.  Classic
 *         LtL blobs that drift, split, and merge like amoebas.
 *
 * Bugs   (R=5 N=6): same neighbourhood as Bosco but 6 states.  Alive
 *         cells leave a 4-step dying trail — colorful "bugs" chase
 *         each other, each trailing orange→red fading ghost pixels.
 *
 * Gnarl  (R=1 N=2): born/survive only when exactly 1 alive neighbour.
 *         Chaotic growth from a random seed; spiderweb / fractal texture.
 *
 * Amoeba (R=2 N=5): birth at 9–16 of 24, survive at 5–14.  Organic
 *         teardrop shapes flow and morph; the 4-step dying trail gives
 *         them soft, glowing edges.
 */
static const Rule PRESETS[N_PRESETS] = {
    { 5, 2, 34, 58, 34, 58, "Bosco  R=5 N=2  (blobs)"    },
    { 5, 6, 34, 45, 34, 58, "Bugs   R=5 N=6  (trails)"   },
    { 1, 2,  1,  1,  1,  1, "Gnarl  R=1 N=2  (chaos)"    },
    { 2, 5,  9, 16,  5, 14, "Amoeba R=2 N=5  (organic)"  },
};

/* ── CA struct ───────────────────────────────────────────────────────── */

/*
 * CA — all state for the cellular automaton.
 *
 * Memory layout:
 *   cur, nxt : flat [gh × gw] byte arrays — double-buffered grid
 *   pad      : flat [(gh + 2R) × (gw + 2R)] int32 — toroidal padding
 *   psum     : flat [(gh+2R+1) × (gw+2R+1)] int32 — 2-D prefix sum
 *
 * cur/nxt are swapped each generation (pointer swap, no memcpy).
 * pad and psum are scratch space rebuilt each generation in ca_step().
 *
 * Allocation uses R_MAX=5 for pad/psum so they never need reallocation
 * when a preset changes radius. cur/nxt are reallocated on resize.
 */
typedef struct {
    uint8_t *cur;        /* current generation                          */
    uint8_t *nxt;        /* next generation (scratch)                   */
    int32_t *pad;        /* padded alive-indicator grid                 */
    int32_t *psum;       /* 2-D prefix sum of pad                       */
    int      gw, gh;     /* grid width = cols, height = rows            */
    int      gen;        /* generation counter                          */
    int      preset;     /* active preset index                         */
    int      gens;       /* CA generations per sim tick                 */
    bool     paused;
    Rule     rule;       /* copy of active rule (may be user-tweaked)   */
} CA;

/* ── allocation ──────────────────────────────────────────────────────── */

/*
 * ca_alloc() — (re)allocate all CA buffers for a given grid size.
 *
 * pad and psum are allocated for R_MAX regardless of current rule.radius
 * so preset switches never reallocate.  cur/nxt are allocated exactly
 * gw×gh; they must be reallocated on resize.
 *
 * Called from ca_init() and ca_resize().
 */
static void ca_alloc(CA *ca, int cols, int rows)
{
    free(ca->cur);  free(ca->nxt);
    free(ca->pad);  free(ca->psum);

    ca->gw = cols;
    ca->gh = rows;

    ca->cur  = calloc((size_t)(rows * cols), 1);
    ca->nxt  = calloc((size_t)(rows * cols), 1);

    /* Pad allocated for worst-case R=R_MAX on all 4 sides */
    int pw   = cols + 2 * R_MAX;
    int ph   = rows + 2 * R_MAX;
    int psw  = pw + 1;
    int psh  = ph + 1;
    ca->pad  = calloc((size_t)(ph  * pw ), sizeof(int32_t));
    ca->psum = calloc((size_t)(psh * psw), sizeof(int32_t));
}

/* ── seeding ─────────────────────────────────────────────────────────── */

/*
 * ca_seed_fill() — fill the whole grid with uniform random density (%).
 * Used by Bosco and Amoeba which are stable from random starts.
 */
static void ca_seed_fill(CA *ca, int density)
{
    int N     = ca->rule.states;
    int total = ca->gw * ca->gh;
    for (int i = 0; i < total; i++)
        ca->cur[i] = (rand() % 100 < density) ? (uint8_t)(N - 1) : 0;
    ca->gen = 0;
}

/*
 * ca_seed_block() — seed a dense central rectangle, rest empty.
 *
 * WHY Bugs needs this instead of ca_seed_fill:
 *   Bugs (R=5, B=34..45, S=34..58, max_neighbours=120) has a narrow
 *   stable density window.  From a uniform 30% random seed:
 *     Gen 0 → 30% alive (E[count]=36, inside B range for dead cells)
 *     Gen 1 → ~63% alive (mass birth spike — dead cells flood into life)
 *     Gen 2 → ~0%  alive (all live cells see ~76 neighbours > S_max=58)
 *   The cascade collapses the grid in two steps.
 *
 *   A central block starts locally dense but globally sparse.  Cells
 *   inside the block interior die naturally (too many neighbours), while
 *   the block BOUNDARY — where live cells meet empty space — sits in the
 *   stable density window.  The boundary is exactly where Bugs patterns
 *   form: moving blobs that drift outward from the shrinking interior.
 */
static void ca_seed_block(CA *ca, int density)
{
    int N  = ca->rule.states;
    int bw = ca->gw * 3 / 5;   /* central block: 60% of screen width  */
    int bh = ca->gh * 3 / 5;   /*                60% of screen height */
    int ox = (ca->gw - bw) / 2;
    int oy = (ca->gh - bh) / 2;

    memset(ca->cur, 0, (size_t)(ca->gw * ca->gh));
    for (int r = oy; r < oy + bh; r++) {
        for (int c = ox; c < ox + bw; c++) {
            if (rand() % 100 < density)
                ca->cur[r * ca->gw + c] = (uint8_t)(N - 1);
        }
    }
    ca->gen = 0;
}

/*
 * ca_seed_random() — dispatch to the appropriate seed strategy per preset.
 *
 * Also called from app_handle_key 'r', so pressing 'r' always re-seeds
 * in the style that makes each preset look its best.
 *
 *   Bosco  → uniform 30% fill  (stable from random noise)
 *   Bugs   → 40% central block (avoids density-cascade collapse)
 *   Gnarl  → sparse 5% fill    (fractal growth from small clusters;
 *                                30% causes every cell to flip every
 *                                tick — visible flicker, no structure)
 *   Amoeba → uniform 25% fill  (within amoeba stability window)
 */
static void ca_seed_random(CA *ca)
{
    switch (ca->preset) {
    case 0:  ca_seed_fill(ca,  30); break;   /* Bosco  */
    case 1:  ca_seed_block(ca, 40); break;   /* Bugs   */
    case 2:  ca_seed_fill(ca,   5); break;   /* Gnarl  */
    case 3:  ca_seed_fill(ca,  25); break;   /* Amoeba */
    default: ca_seed_fill(ca, SEED_DENSITY); break;
    }
}

/* ── preset switching ────────────────────────────────────────────────── */

static void ca_set_preset(CA *ca, int p)
{
    ca->preset = (p + N_PRESETS) % N_PRESETS;
    ca->rule   = PRESETS[ca->preset];
    color_set_palette(ca->preset, ca->rule.states);
    ca_seed_random(ca);
}

/* ── prefix sum builder ──────────────────────────────────────────────── */

/*
 * ca_build_psum() — build the toroidal-padded prefix sum for one step.
 *
 * Step 1: Fill padded grid.
 *   Only FULLY ALIVE cells (state == N-1) contribute to the count.
 *   Dying cells (states 1..N-2) do NOT count as live neighbours — this
 *   gives multi-state rules their trailing "ghost" effect rather than
 *   making dying cells interfere with births.
 *   pad[pr][pc] = 1 if padded cell is fully alive, 0 otherwise.
 *
 *   Toroidal wrap: padded position (pr, pc) maps to original cell
 *     row = (pr - R + gh) % gh
 *     col = (pc - R + gw) % gw
 *
 * Step 2: Build 2-D prefix sum on the padded grid.
 *   P[i][j] = sum of pad[0..i-1][0..j-1]  (1-indexed, P[0][*] = 0)
 *
 *   Standard recurrence:
 *     P[i][j] = pad[i-1][j-1] + P[i-1][j] + P[i][j-1] - P[i-1][j-1]
 */
static void ca_build_psum(const CA *ca)
{
    int R   = ca->rule.radius;
    int N   = ca->rule.states;
    int gw  = ca->gw, gh = ca->gh;
    int pw  = gw + 2 * R;
    int ph  = gh + 2 * R;
    int psw = pw + 1;

    /* ── step 1: fill padded alive indicator ── */
    for (int pr = 0; pr < ph; pr++) {
        int sr = ((pr - R) % gh + gh) % gh;   /* source row, wrapped  */
        for (int pc = 0; pc < pw; pc++) {
            int sc = ((pc - R) % gw + gw) % gw;
            ca->pad[pr * pw + pc] =
                (ca->cur[sr * gw + sc] == (uint8_t)(N - 1)) ? 1 : 0;
        }
    }

    /* ── step 2: 2-D prefix sum ── */
    /* P[0][*] = 0  and  P[*][0] = 0 already from calloc / previous zero */
    /* Re-zero the boundary columns/rows that may be stale */
    for (int i = 0; i <= ph; i++) ca->psum[i * psw]      = 0;
    for (int j = 0; j <= pw; j++) ca->psum[j]             = 0;

    for (int i = 1; i <= ph; i++) {
        for (int j = 1; j <= pw; j++) {
            ca->psum[i * psw + j] =
                ca->pad[(i-1) * pw + (j-1)]
              + ca->psum[(i-1) * psw + j]
              + ca->psum[ i   * psw + (j-1)]
              - ca->psum[(i-1) * psw + (j-1)];
        }
    }
}

/*
 * psum_query() — sum of alive cells in padded rectangle [r0..r1]×[c0..c1].
 *
 * Uses the standard 4-corner inclusion-exclusion on the prefix sum P.
 * Coordinates are in PADDED space (add R before calling for original coords).
 * r0,c0 are the TOP-LEFT corner; r1,c1 are the BOTTOM-RIGHT (inclusive).
 *
 * The formula:
 *   sum(r0..r1, c0..c1)
 *     = P[r1+1][c1+1] - P[r0][c1+1] - P[r1+1][c0] + P[r0][c0]
 *
 * Because P[i][j] = sum of [0..i-1][0..j-1], to include row r0 we
 * use row index r0 (not r0+1) for the subtraction, and r1+1 for the
 * addition.  The +1/-1 offsets cancel out the 1-based indexing.
 */
static inline int psum_query(const int32_t *P, int psw,
                              int r0, int c0, int r1, int c1)
{
    return P[(r1+1) * psw + (c1+1)]
         - P[ r0   * psw + (c1+1)]
         - P[(r1+1) * psw +  c0  ]
         + P[ r0   * psw +  c0  ];
}

/* ── one CA generation ───────────────────────────────────────────────── */

/*
 * ca_step() — advance the CA by one generation.
 *
 * Algorithm:
 *   1. ca_build_psum() — pad grid, compute prefix sum  O(W×H)
 *   2. For each cell (r, c):                           O(W×H)
 *      a. Query (2R+1)×(2R+1) sum from prefix sum     O(1)
 *      b. Subtract centre cell to get neighbour count  O(1)
 *      c. Apply LtL birth / survival / decay rules     O(1)
 *      d. Write result into nxt[]
 *   3. Swap cur ↔ nxt pointers
 *
 * Total: O(W×H), independent of R.
 *
 * State machine (per cell):
 *
 *   state = 0 (dead):
 *     count in [b_min, b_max]  → born:  next = N-1
 *     otherwise                → stays dead: next = 0
 *
 *   state = N-1 (fully alive):
 *     count in [s_min, s_max]  → survives: next = N-1
 *     N=2 otherwise            → dies:     next = 0
 *     N>2 otherwise            → starts dying: next = N-2
 *
 *   state = 1..N-2 (dying):
 *     always decays:           next = state - 1
 *     (dying cells do not participate in birth/survival logic)
 */
static void ca_step(CA *ca)
{
    int R   = ca->rule.radius;
    int N   = ca->rule.states;
    int gw  = ca->gw, gh = ca->gh;
    int pw  = gw + 2 * R;
    int psw = pw + 1;

    ca_build_psum(ca);

    for (int r = 0; r < gh; r++) {
        for (int c = 0; c < gw; c++) {
            /*
             * In padded coords, this cell's neighbourhood rectangle:
             *   rows [r .. r + 2R],  cols [c .. c + 2R]
             * (padded position of this cell is (r+R, c+R), so the
             *  (2R+1)×(2R+1) window starts at padded (r, c))
             */
            int count = psum_query(ca->psum, psw,
                                   r, c, r + 2*R, c + 2*R);

            /* Subtract centre cell (if it is fully alive) */
            if (ca->cur[r * gw + c] == (uint8_t)(N - 1))
                count--;

            uint8_t state = ca->cur[r * gw + c];
            uint8_t next;

            if (state == 0) {
                /* Dead → birth? */
                next = (count >= ca->rule.b_min &&
                        count <= ca->rule.b_max)
                       ? (uint8_t)(N - 1) : 0;
            } else if (state == (uint8_t)(N - 1)) {
                /* Fully alive → survive or start dying? */
                if (count >= ca->rule.s_min && count <= ca->rule.s_max) {
                    next = (uint8_t)(N - 1);        /* survives */
                } else {
                    next = (N > 2) ? (uint8_t)(N - 2) : 0;  /* decay */
                }
            } else {
                /* Dying cell → always decay by 1 */
                next = state - 1;
            }

            ca->nxt[r * gw + c] = next;
        }
    }

    /* Swap buffers — O(1) pointer swap, no memcpy */
    uint8_t *tmp = ca->cur;
    ca->cur = ca->nxt;
    ca->nxt = tmp;

    ca->gen++;
}

/* ── init / free / resize ────────────────────────────────────────────── */

static void ca_init(CA *ca, int cols, int rows)
{
    memset(ca, 0, sizeof *ca);
    ca->gens   = GENS_DEF;
    ca->paused = false;
    ca_alloc(ca, cols, rows);
    ca_set_preset(ca, 0);
}

static void ca_free(CA *ca)
{
    free(ca->cur);  free(ca->nxt);
    free(ca->pad);  free(ca->psum);
    memset(ca, 0, sizeof *ca);
}

static void ca_resize(CA *ca, int cols, int rows)
{
    int preset = ca->preset;
    int gens   = ca->gens;
    ca_free(ca);
    ca->gens = gens;
    ca_alloc(ca, cols, rows);
    ca_set_preset(ca, preset);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    CA ca;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    ca_init(&s->ca, cols, rows);
}

static void scene_free(Scene *s)
{
    ca_free(&s->ca);
}

/*
 * scene_tick() — advance the CA by ca->gens generations per sim tick.
 *
 * ca->gens is user-controlled (+/- keys) for speed without changing Hz.
 * cols/rows unused (CA grid matches terminal size; no pixel physics).
 * alpha/dt_sec are omitted from tick because the CA is not interpolated
 * — it lives entirely in discrete cell/generation space.
 */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)dt; (void)cols; (void)rows;
    if (s->ca.paused) return;
    for (int i = 0; i < s->ca.gens; i++)
        ca_step(&s->ca);
}

/*
 * scene_draw() — render the CA grid to WINDOW *w.
 *
 * alpha is unused: the CA is a discrete automaton; there is no
 * sub-tick position to interpolate.  The parameter is accepted to
 * preserve the standard scene_draw signature.
 *
 * Drawing:
 *   For each cell with state > 0, write '#' with the state-appropriate
 *   color pair (bright for alive, dimming pairs for dying states).
 *   Cells with state 0 are left blank (erase() already cleared them).
 *
 * This is the ONLY function that calls mvwaddch.  All CA logic stays
 * in §5; all ncurses calls stay in §6 scene_draw and §7 screen_draw.
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;

    const CA *ca = &s->ca;
    int N = ca->rule.states;
    int gw = ca->gw < cols ? ca->gw : cols;
    int gh = ca->gh < rows ? ca->gh : rows;

    for (int r = 0; r < gh; r++) {
        for (int c = 0; c < gw; c++) {
            uint8_t state = ca->cur[r * ca->gw + c];
            if (state == 0) continue;
            chtype attr = state_attr(state, N);
            wattron(w, attr);
            mvwaddch(w, r, c, '#');
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.  ONE window, ONE flush per frame.
 *
 * Frame sequence (same in every animation):
 *   erase()              — clear newscr
 *   scene_draw(…)        — write CA content
 *   HUD mvprintw(…)      — written last so always on top
 *   wnoutrefresh(stdscr) — mark newscr ready
 *   doupdate()           — one diff write to terminal
 *
 * See framework.c §7 for full explanation of ncurses double-buffer model.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();

    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD — top row, always on top */
    const CA *ca = &sc->ca;
    char buf[128];
    snprintf(buf, sizeof buf,
             " %-28s  R=%d N=%d  gen:%-6d  %dx/tick  %dHz  %4.1ffps  %s ",
             ca->rule.name,
             ca->rule.radius, ca->rule.states,
             ca->gen,
             ca->gens,
             sim_fps,
             fps,
             ca->paused ? "PAUSED" : "      ");

    attron(COLOR_PAIR(8) | A_BOLD);
    mvprintw(HUD_ROW, 0, "%.*s", s->cols, buf);
    attroff(COLOR_PAIR(8) | A_BOLD);

    /* hint bar — bottom row */
    attron(A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  n/p:preset  +/-:speed  [/]:Hz ");
    attroff(A_DIM);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

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

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    ca_resize(&app->scene.ca, app->screen.cols, app->screen.rows - 2);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    CA *ca = &app->scene.ca;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        ca->paused = !ca->paused;
        break;

    case 'r': case 'R':
        ca_seed_random(ca);
        break;

    case 'n':
        ca_set_preset(ca, ca->preset + 1);
        break;

    case 'p':
        ca_set_preset(ca, ca->preset - 1);
        break;

    case '=': case '+':
        if (ca->gens < GENS_MAX) ca->gens++;
        break;

    case '-':
        if (ca->gens > 1) ca->gens--;
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

/*
 * main() — the game loop (identical structure to framework.c §8).
 *
 * See framework.c main() for full annotated walk-through.
 * Summary of per-frame steps:
 *
 *   ① RESIZE      — handle SIGWINCH, re-read terminal dims, reallocate CA
 *   ② dt          — measure wall-clock elapsed; cap at 100 ms
 *   ③ ACCUMULATOR — drain fixed-step ticks (scene_tick per tick)
 *   ④ ALPHA       — sub-tick interpolation factor (unused by CA, kept
 *                    in signature for framework consistency)
 *   ⑤ FPS counter — 500 ms sliding window
 *   ⑥ FRAME CAP   — sleep BEFORE render to cap at 60 fps output
 *   ⑦ DRAW        — erase → scene_draw → HUD → wnoutrefresh → doupdate
 *   ⑧ INPUT       — non-blocking getch
 */
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
    /* Reserve top row for HUD, bottom row for hints */
    int grid_rows = app->screen.rows - 2;
    if (grid_rows < 1) grid_rows = 1;
    scene_init(&app->scene, app->screen.cols, grid_rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── ③ sim accumulator ───────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows - 2);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha (sub-tick offset — unused for discrete CA) ──── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ FPS counter ───────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep BEFORE render ───────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        /* ── ⑧ input ─────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
