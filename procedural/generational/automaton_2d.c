/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * automaton_2d.c — Larger-than-Life / Extended 2-D Cellular Automaton
 *
 * Generalised CA with configurable neighbourhood radius R (1–10),
 * birth/survival count thresholds, and up to 8 cell states.
 *
 * Extends life.c (Conway's GoL with R=1 bitmask rules) to arbitrary R.
 * Radius-2+ rules produce exotic crystal structures, spirals, and moving
 * "blobs" that are impossible with R=1.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config    — tunables, presets, themes (named constants)
 *   §2  clock     — monotonic time + sleep (the simulation/render DELAYS)
 *   §3  color     — theme → ncurses pairs (a terminal effect)
 *   §4  model     — STATE: Grid (cells), Rule, Settings; pure reads
 *   §5  areatable — PERFORMANCE: summed-area table for O(1) neighbour sums
 *   §6  sim       — the CA rule logic (pure) + seeding/step (effects)
 *   §7  render    — PURE: const Scene → screen
 *   §8  app       — orchestration: input → effects → delay → render
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
 * Presets (15, cycle with n/p; full table in §1):
 *   BOSCO BUGS GNARL AMOEBA WAFFLE GLOBE BUGSMOVIE BRAIN STARWARS
 *   FROGS STICKS CORAL CRYSTAL MAJORITY NEBULA
 *   — drifting blobs, lattices, banded globes, Brian's-Brain gliders,
 *     coral reefs, crystal growth, coarsening domains, multi-hue nebulae.
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          re-randomise grid
 *   n / p      next / previous preset (rule)
 *   t / T      next / previous theme (colour)
 *   + / =      more generations per tick
 *   -          fewer generations per tick
 *   ] / [      raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra automaton_2d.c -o automaton_2d -lncurses
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
 *
 * References     :
 *
 *   CONCEPTS — cellular automata, Larger-than-Life & Generations
 *   • Gardner, M. (1970) — "Mathematical Games: the fantastic combinations
 *     of John Conway's new solitaire game 'life'", *Scientific American*
 *     223(4):120-123.  The origin of Conway's Life — the R=1 case this
 *     file generalises.
 *   • Evans, K. M. (2001) — "Larger than Life: threshold-range scaling of
 *     Life's coherent structures", *Physica D* 159:54-67.  The canonical
 *     LtL paper: how birth/survival windows scale with radius R, and why
 *     "bugs" and blobs emerge (the BOSCO/BUGS/GLOBE family here).
 *   • Wojtowicz, M. (2001) — "Mirek's Cellebration (MCell)" rule-family
 *     reference, mirekw.com/ca/.  Documents the Generations multi-state
 *     trail rules used by BRAIN (Brian's Brain), STARWARS, FROGS, STICKS.
 *   • Griffeath, D. & Moore, C., eds. (2003) — *New Constructions in
 *     Cellular Automata*, Santa Fe Institute / Oxford.  Survey including
 *     LtL self-organisation and pattern formation.
 *
 *   MATH — the summed-area table (the O(1) neighbourhood trick)
 *   • Crow, F. C. (1984) — "Summed-area tables for texture mapping",
 *     *SIGGRAPH '84*, Computer Graphics 18(3):207-212.  Origin of the
 *     2-D prefix sum used to turn an O((2R+1)²) neighbourhood count into
 *     one 4-corner O(1) query.
 *   • Viola, P. & Jones, M. (2001) — "Rapid object detection using a
 *     boosted cascade of simple features", *CVPR 2001*.  Popularised the
 *     same table as the "integral image" for constant-time rectangle sums.
 *
 *   RENDERING — ASCII intensity ramps & terminal output
 *   • Bourke, P. (1997) — "Character representation of grey-scale images",
 *     paulbourke.net/dataformats/asciiart/.  The luminance→glyph idea
 *     behind mapping cell state to a bright/dim character.
 *   • Padala, P. (2005) — "NCURSES Programming HOWTO", TLDP
 *     (tldp.org/HOWTO/NCURSES-Programming-HOWTO/).  The colour-pair,
 *     attribute, and double-buffered (wnoutrefresh/doupdate) model used
 *     in §3/§7.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — SIMULATION / PERFORMANCE / EFFECTS / DELAYS / RENDER ─ *
 *
 * One rule keeps the layers honest: a function's SIGNATURE says whether it
 * can change the world.
 *   • non-const pointer (Grid*, AreaTable*, Scene*, App*) → an EFFECT.
 *   • const pointer / by-value                            → a pure READ.
 *
 * SIMULATION — the cellular automaton itself (§6):
 *   cell_next   pure rule logic: birth / survival / decay for one cell.
 *   sim_step    one generation: count neighbours, write nxt, swap (effect).
 *   scene_tick  one sim tick = `gens` generations (effect).
 *
 * PERFORMANCE — the optimisation that makes large R cheap (§5):
 *   AreaTable + area_build  build a toroidal-padded summed-area table once
 *                           per generation — O(W×H), independent of R.
 *   area_count              O(1) (2R+1)² neighbour sum via 4 corner lookups.
 *   Without this the step would be O(W×H×R²).  It is the ONLY reason R can
 *   reach 10; it changes performance, never the result.
 *
 * EFFECTS — everything that mutates state, all behind non-const pointers:
 *   the cur/nxt buffer swap + generation counter (sim_step), seeding,
 *   preset/theme switches, and the alloc/resize lifecycle.
 *
 * DELAYS — the only time-bending, both inside main() (§8):
 *   sim accumulator  advances the CA at sim_fps Hz, decoupled from render.
 *   frame cap        sleeps so the screen refreshes at most RENDER_FPS.
 *
 * RENDERING (§7) is PURE: render_grid / render_hud read a const Scene and
 * write only the terminal.  Read main() top to bottom to see exactly when
 * each effect and delay fires — and that the draw at the end touches none.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 20,    /* CA ticks slower than physics animations  */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,
    RENDER_FPS      = 60,    /* screen refresh cap (independent of sim)  */

    FPS_UPDATE_MS   = 500,
    DT_CAP_MS       = 100,   /* clamp a long frame gap (spiral guard)    */

    HUD_TOP_ROWS    =  1,    /* top row reserved for the data bar        */
    HUD_BOT_ROWS    =  1,    /* bottom row reserved for the hint bar     */
    HUD_ROWS        = HUD_TOP_ROWS + HUD_BOT_ROWS,

    PAIR_HUD        =  8,    /* top data bar (theme-tinted)              */
    PAIR_HINT       =  9,    /* bottom action bar (fixed bright cyan)    */

    N_PRESETS       = 15,
    N_THEMES        =  6,
    R_MAX           = 10,    /* max radius; sets allocation upper bound  */
    N_MAX           =  8,    /* max states                               */

    GENS_DEF        =  1,    /* CA generations per sim tick              */
    GENS_MAX        = 16,

    SEED_BLOCK_NUM  =  3,    /* central seed block spans 3/5 of each axis */
    SEED_BLOCK_DEN  =  5,    /* (locally dense, globally sparse)          */
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CELL_GLYPH  '#'

/*
 * Interval — a closed integer interval [lo, hi].  The entire Larger-than-
 * Life rule reduces to two membership tests: "is the live-neighbour count
 * inside the birth interval / inside the survival interval?"  An EMPTY
 * interval (lo > hi) means "never" — that is how BRAIN gets its no-survival
 * Brian's-Brain behaviour.
 */
typedef struct {
    int lo;   /* inclusive lower bound — smallest count that passes    */
    int hi;   /* inclusive upper bound — hi < lo encodes the empty set */
} Interval;
static bool interval_has(Interval iv, int n) { return n >= iv.lo && n <= iv.hi; }

/* ── Rule: the Larger-than-Life / Generations parameters ─────────────── */

/*
 * Rule — one Larger-than-Life / Generations rule: the neighbourhood size,
 * the state count, and the two count-intervals that decide birth and
 * survival.  This struct IS the algorithm's parameter vector — the same
 * numbers as Evans's LtL "Rstring" notation Rr,Cn,M0,Sₛ,Bᵦ (Evans 2001);
 * multi-state (N>2) rules add a Generations-style dying trail (Wojtowicz,
 * MCell).  It is pure DATA — the dynamics live in cell_next (§6) — so the
 * whole preset catalogue is a static table and switching rules is a single
 * assignment with no reallocation.
 */
typedef struct {
    int         radius;     /* R: Chebyshev/Moore radius 1..R_MAX.  The
                             * neighbourhood is the (2R+1)² square, so the
                             * max live-neighbour count is (2R+1)²−1.        */
    int         states;     /* N: 2 = binary; N>2 adds an (N−2)-step dying
                             * trail (states N−2…1) for the comet effect.    */
    Interval    birth;      /* a DEAD cell is born iff its live-neighbour
                             * count ∈ birth.                                */
    Interval    survive;    /* an ALIVE cell persists iff its count ∈
                             * survive; empty ⇒ never (Brian's Brain).       */
    int         seed_pct;   /* 0..100 alive probability at (re)seed, tuned
                             * per rule to land in its stable density band.  */
    bool        seed_block; /* true → dense central block (narrow-window
                             * rules); false → uniform fill (chaotic ones).  */
    const char *name;       /* 9-char fixed-width HUD label.                 */
} Rule;

/*
 * Preset table — 15 rules spanning the Larger-than-Life / Generations
 * families.  Max neighbours = (2R+1)² − 1 (centre excluded), so the count
 * windows scale with R:  R1→8, R2→24, R3→48, R4→80, R5→120, R6→168,
 * R7→224, R8→288, R10→440.  Each rule carries its own seed recipe because
 * narrow-window LtL rules collapse from a uniform random fill (a dense
 * central BLOCK keeps the boundary inside the stable density band — see
 * grid_seed_block); chaotic and Generations rules prefer a sparse FILL.
 *
 * An empty survive interval (lo > hi) means "never survive" — that is how
 * BRAIN reproduces Brian's Brain (every live cell dies after one step).
 *
 * (Thresholds for the large-R rules were tuned against a headless
 * population/activity test so they sustain and stay active rather than
 * collapsing to empty or freezing into a still-life.)
 */
static const Rule PRESETS[N_PRESETS] = {
    /*   R   N    birth       survive    seed% block  name        */
    {    5,  2, { 34,  58}, { 34,  58},   30, false, "BOSCO    " }, /* classic drifting blobs   */
    {    5,  6, { 34,  45}, { 34,  58},   40, true,  "BUGS     " }, /* colourful chasing bugs   */
    {    1,  4, {  1,   1}, {  1,   1},    5, false, "GNARL    " }, /* fractal growth, 3-step fade */
    {    2,  5, {  9,  16}, {  5,  14},   25, false, "AMOEBA   " }, /* soft organic teardrops   */
    {    7,  2, { 75, 170}, {100, 200},   45, true,  "WAFFLE   " }, /* rigid waffle lattice     */
    {    8,  4, { 68, 116}, { 68, 139},   40, true,  "GLOBE    " }, /* huge smooth drifting orbs */
    {   10,  2, {104, 177}, {104, 213},   40, true,  "BUGSMOVIE" }, /* giant drifting organism  */
    {    1,  3, {  2,   2}, {  1,   0},   25, false, "BRAIN    " }, /* survive {1,0}=never → Brian's Brain */
    {    1,  4, {  2,   2}, {  3,   5},   28, false, "STARWARS " }, /* spaceships & oscillators */
    {    1,  3, {  3,   4}, {  1,   2},   30, false, "FROGS    " }, /* hopping speckle texture  */
    {    1,  6, {  2,   2}, {  3,   6},   22, false, "STICKS   " }, /* growing trailing sticks  */
    {    3,  2, { 13,  23}, { 13,  23},   28, false, "CORAL    " }, /* static coral / reef      */
    {    4,  4, { 22,  38}, { 22,  40},   30, true,  "CRYSTAL  " }, /* faceted crystal growth   */
    {    4,  2, { 41,  81}, { 40,  80},   50, false, "MAJORITY " }, /* coarsening vote domains  */
    {    6,  7, { 47,  90}, { 47, 110},   35, true,  "NEBULA   " }, /* huge multi-hue nebula    */
};

/* ── Theme: the colour gradient (decoupled from the rule) ────────────── */

/*
 * A THEME supplies the colour gradient; the PRESET supplies the rule.  The
 * two cycle independently — n/p change the rule, t/T recolour it.  Pairs
 * 1..N-1 hold the per-state gradient (pair 1 = fully alive, bold; pairs
 * 2..N-1 = the dying trail, dimmed).  Every entry sits in the bright half
 * of the 256-cube per the "Theme Palette Brightness" rule, so the A_DIM
 * dying states stay legible.  8-colour terminals collapse to one colour.
 */
typedef struct {
    const char *name;        /* HUD label                                  */
    short       grad[N_MAX]; /* [0..N_MAX-2] state gradient (alive→dying),  */
                             /* [N_MAX-1] = HUD tint                        */
    short       fb8;         /* single colour for the 8-colour fallback    */
} Theme;

static const Theme THEMES[N_THEMES] = {
    { "OCEAN  ", {  51,  45,  39,  44,  74, 117, 159, 226 }, COLOR_CYAN    },
    { "FIRE   ", { 226, 220, 208, 202, 196, 166, 130, 226 }, COLOR_YELLOW  },
    { "MATRIX ", {  46,  47,  41,  40,  35,  34,  71, 226 }, COLOR_GREEN   },
    { "VIOLET ", { 207, 201, 171, 141, 135,  99, 105, 226 }, COLOR_MAGENTA },
    { "ICE    ", { 231, 195, 159, 123,  87,  81,  75,  51 }, COLOR_WHITE   },
    { "AMBER  ", { 229, 223, 221, 215, 214, 208, 202, 226 }, COLOR_YELLOW  },
};

/* wrap an index into [0, n) the long way (handles negative steps). */
static int wrap_idx(int i, int n) { i %= n; if (i < 0) i += n; return i; }

/* fold an index onto a torus of circumference n (handles negatives) — the
 * wrap-around that gives the grid its periodic (toroidal) boundary. */
static int wrap_torus(int i, int n) { return ((i % n) + n) % n; }

/* clamp v to [lo, hi]; smaller of two ints — keep range-limits named. */
static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int imin(int a, int b) { return a < b ? a : b; }

/* ===================================================================== */
/* §2  clock — the program's two timebases live behind these             */
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
/* §3  color  (a terminal effect: rewrites ncurses colour pairs)          */
/* ===================================================================== */

static int g_n256 = 0;   /* set in color_init */

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_n256 = (COLORS >= 256);
    /* state pairs (1..N-1) and HUD are set by palette_apply(); seed white */
    for (int i = 1; i <= PAIR_HINT; i++)
        init_pair((short)i, COLOR_WHITE, COLOR_BLACK);
    /* the hint bar is theme-independent: bright cyan on default bg */
    init_pair(PAIR_HINT, g_n256 ? 51 : COLOR_CYAN, -1);
}

/*
 * palette_apply() — bind the theme's gradient to the state pairs (1..N-1)
 * and the HUD tint to PAIR_HUD.  Called on every preset OR theme change;
 * redefining pairs mid-run is flicker-free on modern terminals.
 */
/* bind the full 256-colour gradient: one pair per live/dying state tier. */
static void palette_bind_256(const Theme *t, int N)
{
    for (int i = 0; i < N - 1 && i < N_MAX - 1; i++)
        init_pair((short)(i + 1), t->grad[i], COLOR_BLACK);
    init_pair(PAIR_HUD, t->grad[N_MAX - 1], -1);
}

/* 8-colour fallback: collapse every state to the theme's single colour. */
static void palette_bind_8(const Theme *t, int N)
{
    for (int i = 1; i <= N - 1; i++)
        init_pair((short)i, t->fb8, COLOR_BLACK);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
}

static void palette_apply(int theme, int N)
{
    const Theme *t = &THEMES[theme];
    if (g_n256) palette_bind_256(t, N);
    else        palette_bind_8(t, N);
}

/* ===================================================================== */
/* §4  model — STATE: Grid (cells) + Rule + Settings                      */
/*                                                                        */
/* Plain data and the functions that only LOOK at it.  The grid is the    */
/* simulated world; everything that mutates it lives in §6.               */
/* ===================================================================== */

/*
 * Cell-state vocabulary.  A cell is one byte:
 *   CELL_DEAD (0)        empty
 *   cell_alive(N) = N-1  fully alive — the ONLY state counted as a neighbour
 *   1 .. N-2             the "dying trail": fading ghosts that decay one tier
 *                        per generation and do NOT count as neighbours
 * Naming these keeps the rule logic (§6) and the renderer (§7) in domain
 * terms instead of bare "N-1" arithmetic.
 */
enum { CELL_DEAD = 0 };
static uint8_t cell_alive(int states) { return (uint8_t)(states - 1); }

/*
 * Grid — the cellular state, DOUBLE-BUFFERED.  A CA updates every cell
 * "simultaneously": each next-state is computed from `cur` and written to
 * `nxt`, then the two are swapped (grid_swap).  Without two buffers an
 * in-place update would let a freshly-changed cell pollute its neighbours'
 * counts — the classic CA implementation pitfall (Toffoli & Margolus,
 * "Cellular Automata Machines", 1987).
 *
 * Heap-allocated (not BSS) because the grid tracks the resizable terminal —
 * a documented exception to the no-malloc rule; it allocates only at
 * init/resize, never per frame.
 */
typedef struct {
    uint8_t *cur;        /* current generation, row-major h×w; each byte a
                          * state in [CELL_DEAD … cell_alive(N)].          */
    uint8_t *nxt;        /* next-generation scratch; undefined between
                          * steps — fully overwritten each sim_step.       */
    int      w, h;       /* grid extent in cells = cols × (rows − HUD_ROWS).*/
    long     generation; /* generations since the last seed (HUD "gen:").
                          * long: climbs for minutes at 16×/tick.          */
} Grid;

/* swap the double buffer: the freshly-written nxt becomes cur (the O(1)
 * "ping-pong" that gives every cell a simultaneous, atomic update). */
static void grid_swap(Grid *g) { uint8_t *t = g->cur; g->cur = g->nxt; g->nxt = t; }

/*
 * Settings — the user-facing knobs, kept apart from the simulated Grid so
 * input mutates THESE while the CA mutates the cells.  preset/theme are
 * stored as indices (not the live values) so n/p and t/T wrap cleanly and
 * the HUD can show "current/total"; the live Rule is copied into Scene.rule
 * on a preset switch.
 */
typedef struct {
    int  preset;     /* index into PRESETS, 0..N_PRESETS-1 (the rule).     */
    int  theme;      /* index into THEMES, 0..N_THEMES-1 (colour only).    */
    int  gens;       /* generations advanced per sim tick, 1..GENS_MAX — a
                      * speed multiplier independent of the sim Hz.        */
    bool paused;     /* freeze stepping; render + input keep running.      */
} Settings;

/* ===================================================================== */
/* §5  areatable — PERFORMANCE: the summed-area table                     */
/*                                                                        */
/* This whole section exists only to make a (2R+1)² neighbourhood count   */
/* O(1) instead of O(R²).  It holds NO simulation state — `pad`/`sat` are  */
/* scratch rebuilt from the Grid every generation.  Delete it and the CA   */
/* would still be correct, just O(W·H·R²) slower.                          */
/* ===================================================================== */

/*
 * AreaTable — toroidal-padded summed-area table (a.k.a. integral image,
 * Crow 1984).  `pad` is the alive-indicator grid padded by R on every side
 * with wrap-around copies (so a query never crosses an edge).  `sat` is its
 * 2-D prefix sum.  Both are sized for R_MAX so a radius change never
 * reallocates; `pw`/`psw` record the strides for the ACTIVE radius.
 */
typedef struct {
    int32_t *pad;   /* alive-indicator (0/1) grid, padded by R on every side
                     * with wrap-around copies — folds the torus topology
                     * into a flat rectangle so a query never crosses an edge.*/
    int32_t *sat;   /* 2-D prefix sum of pad: sat[i][j] = Σ pad[0..i-1][0..j-1],
                     * 1-indexed with a zero first row/column (sat[0][*]=
                     * sat[*][0]=0) so the 4-corner query needs no edge cases.*/
    int      pw;    /* active padded width  = w + 2R   (set by area_build).  */
    int      psw;   /* active sat row stride = pw + 1.                        */
} AreaTable;

static void area_alloc(AreaTable *a, int w, int h)
{
    free(a->pad);  free(a->sat);
    int pw = w + 2 * R_MAX, ph = h + 2 * R_MAX;     /* worst-case padding  */
    a->pad = calloc((size_t)pw * ph, sizeof(int32_t));
    a->sat = calloc((size_t)(pw + 1) * (ph + 1), sizeof(int32_t));
    /* pw/psw strides are set per-radius by area_build, not here. */
}

static void area_free(AreaTable *a)
{
    free(a->pad);  free(a->sat);
    a->pad = NULL; a->sat = NULL;
}

/*
 * area_stamp_indicator — pass 1: stamp pad[pr][pc]=1 where the toroidally-
 * wrapped source cell is fully alive, else 0.  The wrap copies make every
 * later neighbourhood query land inside the padded rectangle, so the hot
 * loop needs no boundary branching or modular arithmetic.
 */
static void area_stamp_indicator(AreaTable *a, const Grid *g, int R,
                                 uint8_t alive, int pw, int ph)
{
    int w = g->w, h = g->h;
    for (int pr = 0; pr < ph; pr++) {
        int sr = wrap_torus(pr - R, h);               /* toroidal source row */
        for (int pc = 0; pc < pw; pc++) {
            int sc = wrap_torus(pc - R, w);
            a->pad[pr * pw + pc] = (g->cur[sr * w + sc] == alive) ? 1 : 0;
        }
    }
}

/*
 * area_prefix_sum — pass 2: accumulate pad into the summed-area table by
 * the 2-D prefix-sum recurrence.  Each entry = its own value plus the sums
 * ABOVE and to the LEFT, minus the doubly-counted CORNER (inclusion-
 * exclusion).  Row 0 / column 0 stay zero so the 4-corner query never has
 * to special-case an edge.
 */
static void area_prefix_sum(AreaTable *a, int pw, int ph)
{
    int psw = a->psw;
    for (int i = 0; i <= ph; i++) a->sat[i * psw] = 0;  /* zero left column */
    for (int j = 0; j <= pw; j++) a->sat[j]       = 0;  /* zero top row     */

    for (int i = 1; i <= ph; i++) {
        for (int j = 1; j <= pw; j++) {
            int here   = a->pad[(i - 1) * pw  + (j - 1)];
            int above  = a->sat[(i - 1) * psw +  j     ];
            int left   = a->sat[ i      * psw + (j - 1)];
            int corner = a->sat[(i - 1) * psw + (j - 1)];
            a->sat[i * psw + j] = here + above + left - corner;
        }
    }
}

/* rebuild the table for one generation (a perf-scratch EFFECT; reads the
 * Grid const).  Two passes — alive indicator, then its prefix sum. */
static void area_build(AreaTable *a, const Grid *g, int R, uint8_t alive)
{
    int pw = g->w + 2 * R, ph = g->h + 2 * R;
    a->pw = pw; a->psw = pw + 1;
    area_stamp_indicator(a, g, R, alive, pw, ph);
    area_prefix_sum(a, pw, ph);
}

/*
 * area_count() — alive cells in the (2R+1)×(2R+1) window centred on grid
 * cell (r,c), via one 4-corner inclusion-exclusion (pure read).  The cell
 * sits at padded (r+R, c+R), so its window spans padded rows/cols [r..r+2R]
 * — passed here as the rectangle (r,c)→(r+2R,c+2R) in padded space.
 */
static int area_count(const AreaTable *a, int r, int c, int R)
{
    const int32_t *P = a->sat;
    int psw = a->psw;
    int r1 = r + 2 * R, c1 = c + 2 * R;       /* window (r,c)..(r+2R,c+2R) */
    int br = P[(r1 + 1) * psw + (c1 + 1)];    /* bottom-right corner */
    int tr = P[ r       * psw + (c1 + 1)];    /* top-right    corner */
    int bl = P[(r1 + 1) * psw +  c      ];    /* bottom-left  corner */
    int tl = P[ r       * psw +  c      ];    /* top-left     corner */
    return br - tr - bl + tl;                 /* inclusion-exclusion */
}

/* ===================================================================== */
/* §6  sim — the CA rule logic (pure) + the effects that advance it       */
/* ===================================================================== */

/*
 * Scene — the whole automaton in one value: the cellular state, the perf
 * scratch that accelerates it, the active rule, and the UI knobs.  This is
 * the single composition that effects mutate (Scene*) and the renderer
 * reads (const Scene*).  Member order mirrors a tick's data flow: read
 * `grid` under `rule`, rebuild `area`, write `grid`; `cfg` steers it all.
 */
typedef struct {
    Grid      grid;     /* §4 — the cellular state (double-buffered)      */
    AreaTable area;     /* §5 — perf scratch, rebuilt every generation    */
    Rule      rule;     /* active rule, a copy of PRESETS[cfg.preset]      */
    Settings  cfg;      /* §4 — pattern/theme/speed/pause knobs           */
} Scene;

/*
 * cell_next — the entire Larger-than-Life / Generations rule for ONE cell,
 * as a pure function of its state and live-neighbour count:
 *
 *   dead (CELL_DEAD)  born if count ∈ birth interval     → alive, else dead
 *   alive (N-1)       survive if count ∈ survive interval → alive
 *                     else N=2 → dead, N>2 → N-2 (start the dying trail)
 *   dying (1..N-2)    always decay one tier               → state-1
 *
 * Dying cells do NOT count as neighbours (that is what makes the trail a
 * fading ghost rather than interfering with births).
 */
static uint8_t cell_next(uint8_t state, int neighbours, const Rule *r)
{
    uint8_t alive = cell_alive(r->states);

    if (state == CELL_DEAD)                                  /* birth?   */
        return interval_has(r->birth, neighbours) ? alive : CELL_DEAD;

    if (state == alive)                                     /* survive? */
        return interval_has(r->survive, neighbours) ? alive
             : (r->states > 2 ? (uint8_t)(alive - 1) : CELL_DEAD);

    return (uint8_t)(state - 1);                             /* dying → decay */
}

/*
 * live_neighbours — fully-alive cells around (r,c), CENTRE EXCLUDED.  This
 * is the value the birth/survival intervals are tested against: area_count
 * sums the whole (2R+1)² window in O(1), then the centre is subtracted if
 * it is itself alive (dying-trail centres were never counted to begin with).
 */
static int live_neighbours(const Scene *s, int r, int c)
{
    int     R     = s->rule.radius;
    uint8_t alive = cell_alive(s->rule.states);
    int     count = area_count(&s->area, r, c, R);
    if (s->grid.cur[r * s->grid.w + c] == alive) count--;   /* drop centre */
    return count;
}

/* sim_step — advance the grid one generation (EFFECT), three named steps. */
static void sim_step(Scene *s)
{
    Grid *g = &s->grid;

    /* 1. PERF: rebuild the summed-area table so each count below is O(1). */
    area_build(&s->area, g, s->rule.radius, cell_alive(s->rule.states));

    /* 2. write every cell's next state from its live-neighbour count. */
    for (int r = 0; r < g->h; r++)
        for (int c = 0; c < g->w; c++)
            g->nxt[r * g->w + c] = cell_next(g->cur[r * g->w + c],
                                             live_neighbours(s, r, c), &s->rule);

    /* 3. publish the new generation (double-buffer swap). */
    grid_swap(g);
    g->generation++;
}

/* one sim TICK = `gens` generations (EFFECT).  No-op while paused. */
static void scene_tick(Scene *s)
{
    if (s->cfg.paused) return;
    for (int i = 0; i < s->cfg.gens; i++) sim_step(s);
}

/* ── seeding (effects) ───────────────────────────────────────────────── */

/* a Bernoulli trial: true with probability pct% (the seeding coin-flip). */
static bool chance_pct(int pct) { return rand() % 100 < pct; }

/* fill the whole grid with uniform random density (%). */
static void grid_seed_fill(Grid *g, uint8_t alive, int pct)
{
    int total = g->w * g->h;
    for (int i = 0; i < total; i++)
        g->cur[i] = chance_pct(pct) ? alive : CELL_DEAD;
    g->generation = 0;
}

/*
 * grid_seed_block — seed a dense central rectangle, rest empty.  Narrow-
 * window LtL rules collapse from a uniform fill (the global density sweeps
 * out of the stable band in one or two steps); a block starts locally dense
 * but globally sparse, so its BOUNDARY sits in the stable band and that is
 * where the moving structures nucleate.
 */
static void grid_seed_block(Grid *g, uint8_t alive, int pct)
{
    int bw = g->w * SEED_BLOCK_NUM / SEED_BLOCK_DEN;   /* central block size */
    int bh = g->h * SEED_BLOCK_NUM / SEED_BLOCK_DEN;
    int ox = (g->w - bw) / 2, oy = (g->h - bh) / 2;    /* centred origin     */
    memset(g->cur, CELL_DEAD, (size_t)g->w * g->h);
    for (int r = oy; r < oy + bh; r++)
        for (int c = ox; c < ox + bw; c++)
            if (chance_pct(pct)) g->cur[r * g->w + c] = alive;
    g->generation = 0;
}

/* (re)seed using the active rule's own recipe (also bound to the 'r' key). */
static void sim_seed(Scene *s)
{
    uint8_t alive = cell_alive(s->rule.states);
    if (s->rule.seed_block) grid_seed_block(&s->grid, alive, s->rule.seed_pct);
    else                    grid_seed_fill (&s->grid, alive, s->rule.seed_pct);
}

/* ── preset / theme switches (effects) ───────────────────────────────── */

static void sim_set_preset(Scene *s, int p)
{
    s->cfg.preset = wrap_idx(p, N_PRESETS);
    s->rule       = PRESETS[s->cfg.preset];
    palette_apply(s->cfg.theme, s->rule.states);
    sim_seed(s);
}

/* switch colour theme only — no reseed, rule/geometry unchanged. */
static void sim_set_theme(Scene *s, int t)
{
    s->cfg.theme = wrap_idx(t, N_THEMES);
    palette_apply(s->cfg.theme, s->rule.states);
}

/* ── allocation lifecycle (effects) ──────────────────────────────────── */

static void scene_alloc(Scene *s, int w, int h)
{
    /* grid buffers are exact-sized; area buffers carry R_MAX padding */
    free(s->grid.cur);  free(s->grid.nxt);
    s->grid.w = w;  s->grid.h = h;  s->grid.generation = 0;
    s->grid.cur = calloc((size_t)w * h, 1);
    s->grid.nxt = calloc((size_t)w * h, 1);
    area_alloc(&s->area, w, h);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->cfg.gens   = GENS_DEF;
    s->cfg.paused = false;
    s->cfg.preset = 0;
    s->cfg.theme  = 0;
    scene_alloc(s, w, h);
    sim_set_preset(s, 0);    /* loads rule, palette, and seeds the grid */
}

static void scene_free(Scene *s)
{
    free(s->grid.cur);  free(s->grid.nxt);
    s->grid.cur = s->grid.nxt = NULL;
    area_free(&s->area);
}

/* terminal resize: reallocate at the new size, then re-seed the preset. */
static void scene_resize(Scene *s, int w, int h)
{
    scene_alloc(s, w, h);
    sim_set_preset(s, s->cfg.preset);
}

/* ===================================================================== */
/* §7  render — PURE: const Scene → screen                                */
/*                                                                        */
/* Reads state and paints it; takes only const pointers and touches only  */
/* the terminal.  Grid row r maps to screen row r + HUD_TOP_ROWS, leaving  */
/* row 0 for the data bar and the last row for the hint bar.              */
/* ===================================================================== */

/*
 * cell_attr — map a cell's state to its ncurses attribute (the on-screen
 * "intensity ramp"): fully alive = bright pair 1 + A_BOLD; each dying tier
 * steps down a colour pair and uses A_DIM, so the trail fades to black.
 */
static chtype cell_attr(int state, int N)
{
    if (state == cell_alive(N))
        return (chtype)(COLOR_PAIR(1) | A_BOLD);     /* fully alive */
    int idx = (N - 1 - state);                       /* 1 = barely dying … */
    if (idx >= N_MAX - 1) idx = N_MAX - 2;
    return (chtype)(COLOR_PAIR(idx + 1) | A_DIM);    /* dying trail */
}

static void render_grid(const Scene *s, int rows)
{
    const Grid *g   = &s->grid;
    int         N   = s->rule.states;
    int         top = HUD_TOP_ROWS;            /* grid row r → screen r+top */
    int         gh  = imin(g->h, rows - HUD_ROWS);   /* clip to play area  */

    for (int r = 0; r < gh; r++) {
        for (int c = 0; c < g->w; c++) {
            uint8_t state = g->cur[r * g->w + c];
            if (state == CELL_DEAD) continue;
            chtype attr = cell_attr(state, N);
            attron(attr);
            mvaddch(r + top, c, (chtype)(unsigned char)CELL_GLYPH);
            attroff(attr);
        }
    }
}

/* draw one full-width HUD bar: fill the row, then write the text clipped
 * to `cols` so a long string never wraps onto the grid. */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* format the top-bar readout: preset/theme (each with current/total), the
 * rule's R/N, the generation count, the speed multiplier, and timing. */
static void hud_status_line(char *buf, size_t n, const Scene *s,
                            int sim_fps, double fps)
{
    const Rule     *rule = &s->rule;
    const Settings *cfg  = &s->cfg;
    snprintf(buf, n,
             " AUTOMATON_2D  preset:%s (%d/%d)  R%d N%d  theme:%s (%d/%d)  "
             "gen:%-6ld  %dx/tick  %dHz  %4.1ffps  %s ",
             rule->name, cfg->preset + 1, N_PRESETS,
             rule->radius, rule->states,
             THEMES[cfg->theme].name, cfg->theme + 1, N_THEMES,
             s->grid.generation, cfg->gens, sim_fps, fps,
             cfg->paused ? "PAUSED" : "running");
}

static void render_hud(const Scene *s, int cols, int rows,
                       int sim_fps, double fps)
{
    char status[160];
    hud_status_line(status, sizeof status, s, sim_fps, fps);

    const char *keys =
        " q:quit  spc:pause  r:reset  n/p:preset  t/T:theme  "
        "+/-:speed  [/]:Hz ";

    hud_bar(0,        cols, PAIR_HUD,  status);  /* top: live data  */
    hud_bar(rows - 1, cols, PAIR_HINT, keys);    /* bottom: actions */
}

/* ===================================================================== */
/* §8  app — orchestration: input → effects → delay → render              */
/* ===================================================================== */

/*
 * FpsMeter — a rolling frame-rate average over FPS_UPDATE_MS, so the HUD
 * number is readable rather than flickering every frame.  Banks elapsed
 * time + frame count, divides once per window, then resets.
 */
typedef struct {
    int64_t accum_ns;   /* nanoseconds banked since the last readout    */
    int     frames;     /* frames banked since the last readout         */
    double  value;      /* last computed fps, shown in the HUD          */
} FpsMeter;

static void fps_meter_tick(FpsMeter *m, int64_t dt)
{
    m->frames++;
    m->accum_ns += dt;
    if (m->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {
        m->value    = (double)m->frames / ((double)m->accum_ns / (double)NS_PER_SEC);
        m->frames   = 0;
        m->accum_ns = 0;
    }
}

/*
 * App — the running PROCESS around the Scene: the simulated world plus the
 * loop's own bookkeeping (terminal size, sim pacing, the timing
 * accumulator, the fps meter, and the signal flags).  Split from Scene
 * because these describe the program, not the simulation.  One file-scope
 * instance (g_app) so the async signal handlers can reach the flags.
 */
typedef struct {
    Scene                 scene;       /* the simulated world (§4-§7)         */
    int                   cols, rows;  /* terminal size in cells              */
    int                   sim_fps;     /* CA tick rate (Hz); the render rate
                                        * is capped separately at RENDER_FPS.  */
    int64_t               sim_accum;   /* fixed-step accumulator: banks real
                                        * dt, spent in whole sim ticks.        */
    FpsMeter              fps;          /* rolling render-rate readout.         */
    volatile sig_atomic_t running;     /* main-loop flag, 0 = quit.  volatile
                                        * sig_atomic_t: written by SIGINT/TERM. */
    volatile sig_atomic_t need_resize; /* set by SIGWINCH, drained atop loop.  */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void screen_init(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
}

/* terminal resize: re-read size, rebuild the scene at the new grid size. */
static void app_resize(App *app)
{
    endwin();
    refresh();
    getmaxyx(stdscr, app->rows, app->cols);
    int gh = app->rows - HUD_ROWS;
    if (gh < 1) gh = 1;
    scene_resize(&app->scene, app->cols, gh);
    app->sim_accum   = 0;
    app->need_resize = 0;
}

/* step the speed (generations/tick) and the sim Hz one notch, each clamped. */
static void settings_nudge_gens(Settings *c, int d)
{
    c->gens = clampi(c->gens + d, 1, GENS_MAX);
}
static void app_nudge_fps(App *app, int d)
{
    app->sim_fps = clampi(app->sim_fps + d, SIM_FPS_MIN, SIM_FPS_MAX);
}

/* map a keypress to an intent; some intents fire §6 effects.
 * returns false only on quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene    *s   = &app->scene;
    Settings *cfg = &s->cfg;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':           cfg->paused = !cfg->paused;          break;
    case 'r': case 'R': sim_seed(s);                         break;

    case 'n':           sim_set_preset(s, cfg->preset + 1);  break;
    case 'p':           sim_set_preset(s, cfg->preset - 1);  break;

    case 't':           sim_set_theme(s, cfg->theme + 1);    break;
    case 'T':           sim_set_theme(s, cfg->theme - 1);    break;

    case '=': case '+': settings_nudge_gens(cfg, +1);        break;
    case '-':           settings_nudge_gens(cfg, -1);        break;

    case ']':           app_nudge_fps(app, +SIM_FPS_STEP);   break;
    case '[':           app_nudge_fps(app, -SIM_FPS_STEP);   break;

    default: break;
    }
    return true;
}

/*
 * EFFECTS step: drain the fixed-step accumulator.  The CA advances one
 * tick (= gens generations) every 1/sim_fps seconds of real time, so the
 * simulation rate is decoupled from the render rate.
 */
static void app_advance(App *app, int64_t dt)
{
    int64_t tick = TICK_NS(app->sim_fps);
    app->sim_accum += dt;
    while (app->sim_accum >= tick) {
        scene_tick(&app->scene);
        app->sim_accum -= tick;
    }
}

/* RENDER step: pure paint of the current state. */
static void app_draw(const App *app)
{
    erase();
    render_grid(&app->scene, app->rows);
    render_hud(&app->scene, app->cols, app->rows, app->sim_fps, app->fps.value);
    wnoutrefresh(stdscr);
    doupdate();
}

/* TIME step: dt since the last frame, clamped so one stall can't
 * fast-forward the simulation (spiral-of-death guard). */
static int64_t frame_delta(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    return dt > DT_CAP_MS * NS_PER_MS ? DT_CAP_MS * NS_PER_MS : dt;
}

/* DELAY step: sleep so the screen refreshes at most RENDER_FPS. */
static void app_pace_frame(int64_t frame_start, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(TICK_NS(RENDER_FPS) - elapsed);
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

    screen_init();
    getmaxyx(stdscr, app->rows, app->cols);
    int gh = app->rows - HUD_ROWS;
    if (gh < 1) gh = 1;
    scene_init(&app->scene, app->cols, gh);

    int64_t frame_time = clock_ns();

    /*
     * Fixed-step main loop — each iteration reads as five named steps:
     *   INPUT   poll a key (may fire §6 effects: reset, preset, theme)
     *   TIME    dt since last frame, clamped (spiral-of-death guard)
     *   EFFECTS app_advance() — drain the sim accumulator at sim_fps Hz
     *   DELAY   sleep so the screen refreshes at most RENDER_FPS
     *   RENDER  app_draw() — pure paint, mutates nothing
     */
    while (app->running) {
        if (app->need_resize) {
            app_resize(app);
            frame_time = clock_ns();
        }

        int ch = getch();                                  /* INPUT   */
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;

        int64_t dt = frame_delta(&frame_time);             /* TIME    */

        app_advance(app, dt);                              /* EFFECTS */
        fps_meter_tick(&app->fps, dt);
        app_pace_frame(frame_time, dt);                    /* DELAY   */
        app_draw(app);                                     /* RENDER  */
    }

    scene_free(&app->scene);
    endwin();
    return 0;
}
