/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lightning.c  —  fractal branching lightning in a dark terminal sky
 *
 * THREE LAYERS, deliberately separated so the simulation is trustworthy and
 * the cosmetics are accountable:
 *
 *   1. CHANNEL  (§4)  — the bolt's GEOMETRY: which cells are part of the bolt.
 *                       Pure simulation state.  Just occupancy, nothing else.
 *   2. EFFECTS  (§6)  — glow halo + glyph shimmer, DERIVED from the channel.
 *                       Cosmetic only.  INVARIANT: §6 never writes the channel.
 *                       Delete §6 entirely and the bolt grows identically;
 *                       only its appearance changes.
 *   3. RENDER   (§8)  — reads channel + effects, paints glyphs.  Never mutates.
 *
 * The growth logic (§5 tip, §7 scene) touches ONLY the channel.  It cannot be
 * corrupted by a visual tweak, and every cosmetic decision lives in §6/§8 where
 * you can see all of it in one place.
 *
 * Growth algorithm — recursive tip branching (not DLA walkers):
 *   A seed cell sits at the top.  Three tips start there.  Each tick every
 *   active tip advances one cell downward, leaning left/right by its lean bias.
 *   After MIN_FORK_STEPS a tip may fork into two children (lean ±1 from parent)
 *   and retire.  This yields a fractal binary tree: single-cell-wide paths that
 *   spread apart as they descend.
 *
 * Color (a RENDER decision, by row depth): the bolt's 3-stop depth ramp and
 *   2-stop glow both come from the active Theme (§3), cycled with t / T.
 *   Default "Storm": light-blue → teal → white bolt over a teal/navy glow.
 *
 * Glow (an EFFECT): Manhattan-radius-2 halo around every bolt cell
 *   dist 1 → '|' inner-dim (inner corona)   dist 2 → '.' outer-dim (outer halo)
 *
 * Shimmer (an EFFECT): each bolt cell flickers through SPARK_CHARS so the bolt
 *   looks alive.  A fraction re-rolls per tick (a crawling crackle, not a strobe).
 *
 * Life cycle:
 *   ST_GROWING → tips advance; channel + glyphs are bold; live tips drawn as '!'
 *   ST_DONE    → all tips finished; the bolt is held ~1 s (still shimmering, not
 *                bold), then a fresh bolt auto-starts.  'r' restarts immediately.
 *
 * Keys:
 *   q / ESC   quit        r       new bolt
 *   t / T     cycle colour theme (Storm/Fire/Toxic/Plasma/Mono)
 *   + =       more forks   -      fewer forks
 *   ] [       faster / slower     p / spc  pause
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra lightning.c -o lightning -lncurses
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock     — wall-clock + FrameClock (fixed-timestep pacing)
 *   §3  color     — themeable palette (bolt ramp + glow), t/T cycling
 *   §4  channel   — bolt geometry (pure simulation state); CellGrid buffer
 *   §5  tip       — Tip + TipPool: motion law + branching rule
 *   §6  effects   — glow + shimmer, derived from the channel (cosmetic only)
 *   §7  scene     — growth state machine; orchestrates sim then effects
 *   §8  render    — channel + effects → glyphs via cell_put (read-only)
 *   §9  screen    — ncurses lifecycle + HUD
 *   §10 app       — main loop
 */

/* ── REFERENCES — for the concepts and the rendering ──────────────────── *
 *
 *   Fractal branching & growth  (the algorithm — §5 tip, §7 scene)
 *   ── Niemeyer, L., Pietronero, L. & Wiesmann, H. J. (1984). "Fractal
 *      Dimension of Dielectric Breakdown." Phys. Rev. Lett. 52(12), 1033.
 *      The Dielectric Breakdown Model — the physics from which fractal
 *      lightning's branching, single-channel-wide structure arises.
 *   ── Witten, T. A. & Sander, L. M. (1981). "Diffusion-Limited Aggregation,
 *      a Kinetic Critical Phenomenon." Phys. Rev. Lett. 47(19), 1400.
 *      DLA — the sibling growth model this demo deliberately is NOT; shows
 *      why tip-branching yields clean paths instead of blobby aggregates.
 *   ── Prusinkiewicz, P. & Lindenmayer, A. (1990). "The Algorithmic Beauty
 *      of Plants." Springer.  Recursive branching where one growth tip
 *      becomes two children — exactly tippool_fork() in §5.
 *   ── Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *      Self-similarity and fractal dimension — the O(d) spread and D ≈ 1.5
 *      path behaviour of the branching tree.
 *   ── Reed, T. & Wyvill, B. (1994). "Visual Simulation of Lightning."
 *      SIGGRAPH '94, pp. 359-364.  Recursive branching bolt with a dominant
 *      channel — the closest classic analogue to scene_grow_step().
 *
 *   Rendering  (§6 effects, §8 render)
 *   ── Kim, T. & Lin, M. C. (2004). "Physically Based Animation and Rendering
 *      of Lightning." Proc. Pacific Graphics 2004.  DBM growth plus an
 *      additive glow/corona — the model behind the halo in fx_build_glow().
 *   ── Rosenfeld, A. & Pfaltz, J. L. (1966). "Sequential Operations in Digital
 *      Picture Processing." J. ACM 13(4), 471.  City-block (Manhattan)
 *      distance transforms — the glow is a radius-2 Manhattan dilation.
 *   ── Gookin, D. (2007). "Programmer's Guide to NCURSES." Wiley.  The cell-
 *      drawing API behind cell_put(): init_pair, mvaddch, refresh, resize.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

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
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    GRID_ROWS_MAX   =  80,
    GRID_COLS_MAX   = 300,

    MAX_TIPS        =  64,   /* max simultaneous growing branch tips    */

    MIN_FORK_STEPS  =   2,   /* a tip must grow this many cells first   */
    LEAN_MAX        =   3,   /* lean bias clamped to ±LEAN_MAX          */

    HOLD_AFTER_DONE_MS = 1000, /* finished bolt lingers, then auto-restarts    */
    FRAME_DT_CAP_MS    =  100,  /* clamp one frame's dt — stops spiral-of-death */
    RENDER_FPS_CAP     =   60,  /* render-rate ceiling the loop sleeps down to  */
    MS_PER_SEC         = 1000,  /* ms<->tick conversion base (ms_to_ticks)      */
    PERCENT            =  100,  /* denominator for percent-probability rolls    */

    GLOW_RADIUS     =   2,   /* Manhattan radius of ambient halo        */

    N_THEMES        =   5,   /* colour themes, cycled with t / T              */
    N_PALETTE       =   5,   /* themeable colour slots: 3 bolt depth + 2 glow */

    HUD_COLS        =  80,
    FPS_UPDATE_MS   = 500,

    SPARK_UNSEEDED  = 0xFF,  /* spark slot not yet assigned a glyph     */
};

/*
 * LEAN_PCT  — percent chance a tip follows its lean each step (more = wider).
 * FORK_PCT  — percent chance a tip forks each step after MIN_FORK_STEPS.
 *             Runtime-tunable with + / - between FORK_PCT_MIN..MAX.
 */
#define LEAN_PCT            60
#define FORK_PCT_DEFAULT    30
#define FORK_PCT_MIN         8
#define FORK_PCT_MAX        50
#define FORK_PCT_STEP        2   /* +/- nudge to fork probability per keypress */

/* Shimmer glyph alphabet + the fraction of cells re-rolled per tick. */
#define SPARK_CHARS  "#%xX*+|/\\^"
#define SHIMMER_PCT  28

/* Single-glyph constants — named so the renderers read by intent, not by char. */
#define GLOW_INNER_GLYPH '|'   /* inner corona, 1 cell from the bolt */
#define GLOW_OUTER_GLYPH '.'   /* outer halo,   2 cells out          */
#define TIP_GLYPH        '!'   /* a live growth front                */

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/*
 * Small shared mechanics used across sections:
 *   clamp_int    — pin a value into [lo,hi] (keeps tunables in range).
 *   roll_percent — a percentage dice roll: true with probability pct/PERCENT.
 */
static int  clamp_int(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static bool roll_percent(int pct)            { return rand() % PERCENT < pct; }

/* ===================================================================== */
/* §2  clock — wall-clock primitives + fixed-timestep frame pacing         */
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

/*
 * FrameClock — the simulation's heartbeat.  It owns every timing variable the
 * main loop needs, so the loop body reads as intent rather than bookkeeping.
 *
 * WHY it exists: the render rate (how often the terminal repaints) must be
 * decoupled from the sim rate (how often the bolt advances).  The canonical fix
 * is the fixed-timestep accumulator — bank each frame's elapsed real time as a
 * "debt", then repay it in equal TICK_NS slices, one scene_tick() per slice.  So
 * the bolt grows at a constant rate on any machine, and a slow frame runs a few
 * catch-up ticks instead of one big lurch.
 *
 * INVARIANTS: every *_ns field is nanoseconds on the CLOCK_MONOTONIC scale;
 * after the main loop's draining `while`, sim_debt is always < one TICK_NS.
 */
typedef struct {
    int64_t prev_ns;     /* CLOCK_MONOTONIC stamp at this frame's start          */
    int64_t sim_debt;    /* unspent ns owed to the fixed step; left in [0,TICK)  */
    int64_t fps_window;  /* ns elapsed since the last fps sample was emitted     */
    int     fps_frames;  /* frames counted since the last fps sample             */
    double  fps;         /* smoothed frames/sec, refreshed every FPS_UPDATE_MS   */
} FrameClock;

static void frameclock_init(FrameClock *fc)
{
    fc->prev_ns    = clock_ns();
    fc->sim_debt   = 0;
    fc->fps_window = 0;
    fc->fps_frames = 0;
    fc->fps        = 0.0;
}

/*
 * frameclock_sample_fps() — fold one frame into the rolling fps sample; once the
 * window fills (FPS_UPDATE_MS), publish a fresh reading: frames / elapsed seconds.
 */
static void frameclock_sample_fps(FrameClock *fc, int64_t dt)
{
    fc->fps_frames++;
    fc->fps_window += dt;
    if (fc->fps_window >= FPS_UPDATE_MS * NS_PER_MS) {
        fc->fps        = (double)fc->fps_frames
                       / ((double)fc->fps_window / (double)NS_PER_SEC);
        fc->fps_frames = 0;
        fc->fps_window = 0;
    }
}

/*
 * frameclock_begin_frame() — open a new frame as named steps: measure the real
 * time since the last frame, clamp a long stall (anti spiral-of-death), then bank
 * it as sim debt and feed the fps sample.
 */
static void frameclock_begin_frame(FrameClock *fc)
{
    int64_t now = clock_ns();
    int64_t dt  = now - fc->prev_ns;                   /* time this frame represents */
    fc->prev_ns = now;

    int64_t stall_cap = FRAME_DT_CAP_MS * NS_PER_MS;
    if (dt > stall_cap) dt = stall_cap;                /* anti spiral-of-death */

    fc->sim_debt += dt;                                /* owe this much to the fixed step */
    frameclock_sample_fps(fc, dt);
}

/* frameclock_step_due() — true (and pays down the debt) while a fixed sim step
 * is owed.  `while (frameclock_step_due(...)) tick();` is the accumulator loop. */
static bool frameclock_step_due(FrameClock *fc, int64_t tick_ns)
{
    if (fc->sim_debt < tick_ns) return false;
    fc->sim_debt -= tick_ns;
    return true;
}

/*
 * frameclock_throttle() — sleep off whatever remains of this frame's budget so
 * the render loop caps near RENDER_FPS_CAP instead of busy-spinning.  `elapsed`
 * is measured from this frame's start (prev_ns), so a frame that already overran
 * its budget simply doesn't sleep.
 */
static void frameclock_throttle(const FrameClock *fc)
{
    int64_t budget  = NS_PER_SEC / RENDER_FPS_CAP;     /* time one frame may take */
    int64_t elapsed = clock_ns() - fc->prev_ns;        /* spent so far this frame  */
    clock_sleep_ns(budget - elapsed);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * ColorID — the ncurses colour-PAIR slots this program uses.  Values are pair
 * indices (1..7; pair 0 is the reserved terminal default); theme_apply() and
 * color_init() bind each slot to a real foreground colour over a black ground.
 * Code names a colour by its concept (COL_BOLT2), never by a raw palette index.
 *
 * The bolt has exactly two themeable colour concerns, and a Theme supplies both:
 *   - COL_BOLT0..2 — a 3-stop DEPTH RAMP painted down the bolt (top→bottom),
 *     chosen per cell by render_color_for_row() in §8.  The channel never stores
 *     colour; depth is a pure function of row, so it is a render decision.
 *   - COL_GLOW_I / COL_GLOW_O — the 2-stop glow pair for the halo effect (§6/§8).
 * COL_HUD / COL_HINT are theme-independent so the text stays legible on any palette.
 */
typedef enum {
    COL_BOLT0  = 1,   /* depth: top third (near cloud) */
    COL_BOLT1  = 2,   /* depth: middle third           */
    COL_BOLT2  = 3,   /* depth: bottom third (hottest) */
    COL_GLOW_I = 4,   /* halo: inner corona            */
    COL_GLOW_O = 5,   /* halo: outer halo              */
    COL_HUD    = 6,   /* HUD data — bright yellow      */
    COL_HINT   = 7,   /* HUD hint — bright cyan        */
} ColorID;

/*
 * Theme — a named palette held in one array: fg[0..2] are the bolt depth stops
 * (top→bottom), fg[3] is the glow inner stop, fg[4] the glow outer.  Cycled with
 * t / T.  Adding a theme is a single table row; the engine never changes.
 *
 * Every entry sits in the bright half of the 256-cube so even the A_DIM glow
 * stops stay visible on black (dim cube/gray indices vanish under A_DIM).
 */
typedef struct {
    const char *name;        /* HUD label, e.g. "Storm"                       */
    int fg256[N_PALETTE];    /* xterm-256 indices; bright half only (legible) */
    int fg8  [N_PALETTE];    /* ANSI 0-7 fallback when COLORS < 256           */
} Theme;

static const Theme g_themes[N_THEMES] = {
    /*            name      bolt:top mid  bot   glow:in out      8-colour fallback (same order)                               */
    { "Storm",  {  45,  51, 231,  30,  26 }, { COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE, COLOR_CYAN,    COLOR_BLUE  } },
    { "Fire",   { 196, 208, 231,  94,  88 }, { COLOR_RED,     COLOR_YELLOW,  COLOR_WHITE, COLOR_RED,     COLOR_RED   } },
    { "Toxic",  {  46, 118, 231,  34,  28 }, { COLOR_GREEN,   COLOR_GREEN,   COLOR_WHITE, COLOR_GREEN,   COLOR_GREEN } },
    { "Plasma", {  99, 141, 231,  60,  54 }, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_MAGENTA, COLOR_BLUE  } },
    { "Mono",   { 245, 251, 231, 245, 240 }, { COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE, COLOR_WHITE,   COLOR_WHITE } },
};

/* theme_apply() — bind the five themeable slots (COL_BOLT0..COL_GLOW_O). */
static void theme_apply(int theme)
{
    const Theme *t = &g_themes[theme];
    bool truecolor = COLORS >= 256;
    for (int i = 0; i < N_PALETTE; i++)
        init_pair((short)(COL_BOLT0 + i),
                  (short)(truecolor ? t->fg256[i] : t->fg8[i]), COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    /* HUD/HINT are theme-independent — yellow data, cyan actions on any palette */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* Storm; Scene.theme tracks the active one thereafter */
}

/* ===================================================================== */
/* §4  channel — bolt geometry (pure simulation state)                    */
/* ===================================================================== */

/*
 * CellGrid — one byte per terminal cell; the shape every per-cell buffer in this
 * program shares (channel occupancy, glow level, glyph index).  Naming the buffer
 * once keeps those three arrays visibly the same thing.
 *
 * Indexed row-major as grid[y][x] (row first), matching how ncurses addresses
 * cells.  It is fixed at the largest screen we serve (GRID_ROWS_MAX x COLS_MAX)
 * and embedded by value in its owner, never malloc'd: the project bans heap
 * allocation on the hot path, so the worst case is reserved up front and only the
 * owner's active rows/cols are ever touched.
 */
typedef uint8_t CellGrid[GRID_ROWS_MAX][GRID_COLS_MAX];

/*
 * Channel — the single source of truth about the bolt: its GEOMETRY and nothing
 * else.  hit[y][x] is 1 if the cell belongs to the discharge channel, 0 if it is
 * open sky.  No colour, no glyph, no glow — those are all DERIVED later (§6/§8).
 *
 * WHY so thin: in the dielectric-breakdown picture of lightning (Niemeyer,
 * Pietronero & Wiesmann 1984) the bolt simply IS the set of broken-down cells,
 * and everything visual is a reading of that set.  Storing pure occupancy means
 * the growth code in §5/§7 can only ever change WHERE the bolt is, never how it
 * looks — so a cosmetic tweak can never corrupt the simulation.
 */
typedef struct {
    CellGrid hit;       /* per cell: 1 = bolt channel, 0 = sky                  */
    int      count;     /* number of 1-cells; equals lit cells, shown in HUD    */
    int      rows;      /* active height, <= GRID_ROWS_MAX (this bolt's sky)    */
    int      cols;      /* active width,  <= GRID_COLS_MAX                      */
} Channel;

static void channel_init(Channel *c, int cols, int rows, int seed_x)
{
    c->cols = clamp_int(cols, 1, GRID_COLS_MAX);
    c->rows = clamp_int(rows, 1, GRID_ROWS_MAX);
    memset(c->hit, 0, sizeof c->hit);

    seed_x = clamp_int(seed_x, 0, c->cols - 1);   /* the bolt is born at the top edge */
    c->hit[0][seed_x] = 1;
    c->count          = 1;
}

/* cell_in_bounds() — is (x,y) inside the active grid? */
static bool cell_in_bounds(const Channel *c, int x, int y)
{
    return x >= 0 && x < c->cols && y >= 0 && y < c->rows;
}

/* channel_mark() — add cell (x,y) to the bolt.  Caller guarantees bounds. */
static void channel_mark(Channel *c, int x, int y)
{
    c->hit[y][x] = 1;
    c->count++;
}

/* channel_occupied() — bounds-checked: is this cell part of the bolt? */
static bool channel_occupied(const Channel *c, int x, int y)
{
    return cell_in_bounds(c, x, y) && c->hit[y][x] != 0;
}

/* ===================================================================== */
/* §5  tip — one branch tip, and the pool of tips that form the bolt       */
/* ===================================================================== */

/*
 * Tip — one actively growing branch of the bolt: a single growth front that
 * advances one cell per sim step.  This is the recursive-branching view of
 * lightning (Reed & Wyvill 1994) — a bolt is a tip that walks downward and
 * occasionally splits, rather than a potential field solved everywhere at once.
 *
 * Fields:
 *   x, y    current cell, always inside [0,cols) x [0,rows).
 *   lean    persistent sideways bias in [-LEAN_MAX,+LEAN_MAX].  0 = straight
 *           down; the SIGN picks the direction; the MAGNITUDE is the accumulated
 *           fork depth (children inherit lean +/-1), so deeper branches lean
 *           harder and siblings fan apart — the source of the fractal tree shape.
 *   steps   cells grown since spawn or last fork.  Forking is gated on
 *           steps >= MIN_FORK_STEPS so no segment can start as a stub.
 *   active  true while growing; cleared the step it reaches the ground, a side
 *           edge, or a cell already part of the bolt.
 */
typedef struct {
    int  x, y;
    int  lean;
    int  steps;
    bool active;
} Tip;

/*
 * TipPool — the bolt's entire advancing front: every Tip ever spawned, both
 * still-growing and retired, in one fixed array.  Modelling growth as a pool of
 * forking fronts is the L-system / branching view (Prusinkiewicz & Lindenmayer
 * 1990): the binary-tree shape emerges from a single rule — one tip becomes two.
 *
 * Slots are APPEND-ONLY (tippool_fork appends two children and retires the parent
 * in place) and never compacted or reordered.  WHY that matters: a Tip* taken
 * mid-step therefore stays valid for the rest of the bolt, which is exactly what
 * lets tippool_fork keep using its `parent` pointer while it appends children.
 */
typedef struct {
    Tip tips[MAX_TIPS];   /* slots [0,count); a retired tip stays put, inactive  */
    int count;            /* slots in use (active + retired); 0..MAX_TIPS        */
} TipPool;

static void tippool_clear(TipPool *p)
{
    p->count = 0;
    memset(p->tips, 0, sizeof p->tips);
}

/* tippool_add() — spawn a tip at (x,y) with the given lean, if a slot is free. */
static void tippool_add(TipPool *p, int x, int y, int lean)
{
    if (p->count >= MAX_TIPS) return;
    p->tips[p->count++] = (Tip){ .x = x, .y = y, .lean = lean,
                                 .steps = 0, .active = true };
}

static bool tippool_any_active(const TipPool *p)
{
    for (int i = 0; i < p->count; i++)
        if (p->tips[i].active) return true;
    return false;
}

/*
 * tip_next_cell() — the motion law for one tip: always descend one row, and with
 * LEAN_PCT chance also step one column toward its lean.  The column is clamped to
 * the grid; the row may fall past the bottom (caller reads that as "grounded").
 */
static void tip_next_cell(const Tip *t, const Channel *c, int *nx, int *ny)
{
    int dx = 0;
    if (t->lean != 0 && roll_percent(LEAN_PCT))
        dx = (t->lean > 0) ? 1 : -1;                /* step toward the lean */

    *nx = clamp_int(t->x + dx, 0, c->cols - 1);     /* stay on screen */
    *ny = t->y + 1;                                 /* always descend one row */
}

/*
 * tippool_fork() — the branching rule: replace one tip with two children whose
 * lean diverges by ±1 (clamped to ±LEAN_MAX), so siblings spread apart as they
 * descend and the bolt becomes a fractal binary tree.  The parent retires; only
 * the children carry on.  No-op when the pool has no room for two.
 */
static void tippool_fork(TipPool *p, Tip *parent)
{
    if (p->count > MAX_TIPS - 2) return;              /* need room for two children */
    int left  = clamp_int(parent->lean - 1, -LEAN_MAX, LEAN_MAX);
    int right = clamp_int(parent->lean + 1, -LEAN_MAX, LEAN_MAX);
    tippool_add(p, parent->x, parent->y, left);
    tippool_add(p, parent->x, parent->y, right);
    parent->active = false;                           /* parent retires; children grow */
}

/*
 * tip_may_fork() — branching is allowed only after MIN_FORK_STEPS of straight
 * run, then fires with probability fork_pct/PERCENT; this keeps branches from
 * splitting into stubs the instant they are born.
 */
static bool tip_may_fork(const Tip *t, int fork_pct)
{
    return t->steps >= MIN_FORK_STEPS && roll_percent(fork_pct);
}

/*
 * growth_blocked() — a tip stops when its next cell falls past the ground or is
 * already part of the bolt (a discharge channel never retraces itself).
 */
static bool growth_blocked(const Channel *c, int x, int y)
{
    return y >= c->rows || channel_occupied(c, x, y);
}

/* ===================================================================== */
/* §6  effects — glow + shimmer, derived from the channel (cosmetic only) */
/* ===================================================================== */

/*
 * Effects — the two cosmetic layers, both DERIVED from the channel and feeding
 * back into nothing.  INVARIANT: every §6 function reads the Channel and writes
 * only Effects, so deleting this struct changes how the bolt LOOKS, never how it
 * grows.  This is where all the "make it look electric" decisions live.
 *
 * Fields (both are per-cell CellGrids, indexed [y][x]):
 *   glow   an ambient corona around the bolt — the visible discharge halo of
 *          Kim & Lin (2004).  Built as a small city-block distance transform of
 *          the channel (Rosenfeld & Pfaltz 1966): a cell's level is GLOW_RADIUS+1
 *          minus its Manhattan distance to the nearest bolt cell, giving
 *          0 = none, 1 = outer (distance 2), 2 = inner (distance 1).  A pure
 *          function of the channel, rebuilt whenever the channel changes.
 *   spark  the shimmer: which SPARK_CHARS glyph the cell shows right now, as an
 *          index in 0..strlen(SPARK_CHARS)-1, or SPARK_UNSEEDED (0xFF) before a
 *          cell's first glyph is assigned.  Re-rolled over time so the bolt
 *          crackles instead of standing still.
 */
typedef struct {
    CellGrid glow;     /* halo level per cell: 0 none / 1 outer / 2 inner       */
    CellGrid spark;    /* glyph index per cell, or SPARK_UNSEEDED               */
} Effects;

static void fx_clear(Effects *fx)
{
    memset(fx->glow,  0,              sizeof fx->glow);
    memset(fx->spark, SPARK_UNSEEDED, sizeof fx->spark);
}

/*
 * manhattan() — city-block distance |dx| + |dy| (Rosenfeld & Pfaltz 1966).  This
 * is the metric the corona grows in: equal-distance rings form diamonds.
 */
static int manhattan(int dx, int dy)
{
    return (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
}

/*
 * glow_level_for_distance() — map a Manhattan distance to halo brightness: closer
 * to the bolt is brighter.  distance 1 -> 2 (inner), 2 -> 1 (outer), 0 or beyond
 * GLOW_RADIUS -> 0 (no halo).
 */
static uint8_t glow_level_for_distance(int dist)
{
    if (dist <= 0 || dist > GLOW_RADIUS) return 0;
    return (uint8_t)(GLOW_RADIUS + 1 - dist);
}

/*
 * fx_stamp_halo() — lay one bolt cell's corona onto its Manhattan neighbourhood,
 * keeping the brighter value where halos overlap and never writing over a bolt
 * cell (the channel always wins over its own glow).
 */
static void fx_stamp_halo(Effects *fx, const Channel *c, int cx, int cy)
{
    for (int dy = -GLOW_RADIUS; dy <= GLOW_RADIUS; dy++) {
        for (int dx = -GLOW_RADIUS; dx <= GLOW_RADIUS; dx++) {
            int gx = cx + dx, gy = cy + dy;
            if (!cell_in_bounds(c, gx, gy) || c->hit[gy][gx]) continue;
            uint8_t level = glow_level_for_distance(manhattan(dx, dy));
            if (level > fx->glow[gy][gx]) fx->glow[gy][gx] = level;
        }
    }
}

/*
 * fx_build_glow() — recompute the whole halo from scratch: clear it, then stamp a
 * corona around every bolt cell.  A pure function of the channel, so rebuilding
 * each growth tick can never drift out of sync.
 */
static void fx_build_glow(Effects *fx, const Channel *c)
{
    memset(fx->glow, 0, sizeof fx->glow);
    for (int y = 0; y < c->rows; y++)
        for (int x = 0; x < c->cols; x++)
            if (c->hit[y][x])
                fx_stamp_halo(fx, c, x, y);
}

/* random_glyph_index() — pick a random slot in SPARK_CHARS (one shimmer frame). */
static uint8_t random_glyph_index(void)
{
    return (uint8_t)(rand() % (int)(sizeof SPARK_CHARS - 1));
}

/*
 * fx_animate() — drive the shimmer.  In one pass over the bolt cells:
 *   - a cell that just appeared (spark == SPARK_UNSEEDED) gets a glyph now, so it
 *     never renders garbage;
 *   - an already-lit cell re-rolls its glyph with probability pct/PERCENT, a
 *     staggered crackle rather than a full-bolt strobe.
 */
static void fx_animate(Effects *fx, const Channel *c, int pct)
{
    for (int y = 0; y < c->rows; y++)
        for (int x = 0; x < c->cols; x++) {
            if (!c->hit[y][x]) continue;
            if (fx->spark[y][x] == SPARK_UNSEEDED || roll_percent(pct))
                fx->spark[y][x] = random_glyph_index();
        }
}

/*
 * fx_prime() — initialise the cosmetic layer for a brand-new bolt so frame 0
 * renders cleanly: clear it, build the seed cell's halo, and assign a glyph.
 */
static void fx_prime(Effects *fx, const Channel *c)
{
    fx_clear(fx);
    fx_build_glow(fx, c);
    fx_animate(fx, c, SHIMMER_PCT);
}

/* Read accessors — the render layer asks for an effect at a cell through these,
 * so the [y][x] convention and the SPARK_CHARS lookup stay owned by §6. */
static uint8_t fx_glow_level(const Effects *fx, int x, int y) { return fx->glow[y][x]; }
static char    fx_glyph     (const Effects *fx, int x, int y) { return SPARK_CHARS[fx->spark[y][x]]; }

/* ===================================================================== */
/* §7  scene — growth state machine; orchestrates sim then effects        */
/* ===================================================================== */

/*
 * SceneState — the bolt's life cycle.  Two phases are all it needs: the bolt is
 * either drawing itself or finished and lingering before the next strike.
 *   ST_GROWING → tips are still advancing; the animation is live and bold, with
 *                active tips drawn as '!'.
 *   ST_DONE    → every tip has retired; the finished bolt is held (still
 *                shimmering, no longer bold) for ~HOLD_AFTER_DONE_MS, then a
 *                fresh bolt auto-starts (the linger logic lives in scene_tick).
 */
typedef enum { ST_GROWING, ST_DONE } SceneState;

/*
 * Scene — the whole simulation in one object, grouped by what each part is for.
 * A reader can answer "what is the bolt / what makes it look alive / what grows
 * it / where is it in its life / what can I tune" just from the field groups.  It
 * composes the structs above: Channel (truth) + Effects (looks) + TipPool
 * (growth) + a tiny state machine + the viewer's knobs.
 */
typedef struct {
    /* what the bolt IS — the single source of truth (geometry only) */
    Channel    channel;
    /* what makes it look alive — derived from the channel, never feeds back */
    Effects    fx;
    /* what grows it — the advancing front of branch tips */
    TipPool    pool;

    /* where it is in its life cycle */
    SceneState state;          /* ST_GROWING or ST_DONE                          */
    int        hold_ticks;     /* ticks elapsed in ST_DONE; auto-restarts once   */
                               /* it reaches ~HOLD_AFTER_DONE_MS worth (else 0)  */

    /* user-tunable controls — set once and PERSIST across bolts, so each new    */
    /* strike keeps the look and feel the viewer dialled in                      */
    bool       paused;         /* space/p: freezes the sim AND the linger clock  */
    int        fork_pct;       /* +/-: fork chance per step, FORK_PCT_MIN..MAX   */
    int        theme;          /* t/T: index into g_themes, 0..N_THEMES-1        */
} Scene;

/*
 * seed_column() — where the bolt is born: a random column in the middle third of
 * the sky, so successive strikes vary but never start hard against an edge.
 */
static int seed_column(int cols)
{
    int third = cols / 3;
    return third + rand() % (third + 1);
}

/*
 * scene_seed_roots() — three initial tips at the seed (leaning left, straight,
 * right) so the bolt has spread from its very first step instead of one stem.
 */
static void scene_seed_roots(Scene *s, int seed_x)
{
    tippool_clear(&s->pool);
    tippool_add(&s->pool, seed_x, 0, -1);
    tippool_add(&s->pool, seed_x, 0,  0);
    tippool_add(&s->pool, seed_x, 0, +1);
}

/* scene_start_bolt() — birth a fresh bolt: choose a seed, reset geometry, plant
 * the root tips, and prime the cosmetic layer.  Reads as those four steps. */
static void scene_start_bolt(Scene *s, int cols, int rows)
{
    int seed_x = seed_column(cols);
    channel_init(&s->channel, cols, rows, seed_x);
    scene_seed_roots(s, seed_x);
    s->state = ST_GROWING;
    fx_prime(&s->fx, &s->channel);
}

static void scene_init(Scene *s, int cols, int rows)
{
    s->paused   = false;
    s->fork_pct = FORK_PCT_DEFAULT;
    s->theme    = 0;   /* color_init() already applied theme 0 (Storm) */
    scene_start_bolt(s, cols, rows);
}

/* scene_cycle_theme() — step the active theme (dir = +1 / -1) and rebind colours. */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

/* scene_finish() — the bolt is complete: hold it and start the linger clock. */
static void scene_finish(Scene *s)
{
    s->state      = ST_DONE;
    s->hold_ticks = 0;
}

/*
 * scene_grow_step() — PURE SIMULATION: advance every active tip one cell.
 * Reads/writes the channel and the tip pool only — no glow, no glyphs.  Each
 * step reads like the growth rule itself:
 *
 *   for each active tip:
 *       aim one cell ahead         (tip_next_cell: descend + lean)
 *       grounded or blocked        → retire it
 *       otherwise                  → mark the channel, then maybe fork
 *   no tip left active             → the bolt is finished
 */
static void scene_grow_step(Scene *s)
{
    Channel *c = &s->channel;

    for (int i = 0; i < s->pool.count; i++) {
        Tip *t = &s->pool.tips[i];
        if (!t->active) continue;

        int nx, ny;
        tip_next_cell(t, c, &nx, &ny);

        if (growth_blocked(c, nx, ny)) {
            t->active = false;            /* reached ground or hit the bolt */
            continue;
        }

        channel_mark(c, nx, ny);
        t->x = nx;
        t->y = ny;
        t->steps++;

        if (tip_may_fork(t, s->fork_pct))
            tippool_fork(&s->pool, t);
    }

    if (!tippool_any_active(&s->pool))
        scene_finish(s);
}

/*
 * ms_to_ticks() — convert a wall-clock duration to a count of sim ticks at the
 * current rate (>= 1), so a "1 second" hold stays one second even when the speed
 * keys change sim_fps.
 */
static int ms_to_ticks(int ms, int sim_fps)
{
    int ticks = sim_fps * ms / MS_PER_SEC;
    return ticks < 1 ? 1 : ticks;
}

/*
 * scene_advance_linger() — count down the post-strike hold; when it elapses the
 * next bolt fires automatically (keeping the tuned theme + fork rate).
 */
static void scene_advance_linger(Scene *s, int sim_fps)
{
    if (++s->hold_ticks >= ms_to_ticks(HOLD_AFTER_DONE_MS, sim_fps))
        scene_start_bolt(s, s->channel.cols, s->channel.rows);
}

/*
 * scene_tick() — one sim step in strict order: SIMULATION, then EFFECTS, then the
 * life-cycle clock.  Glow is rebuilt only when the channel changed (growth); the
 * shimmer animates in both states so a held bolt keeps crackling; a finished bolt
 * lingers via scene_advance_linger.  Paused ticks return early, so neither the
 * sim nor the linger advances while paused.
 */
static void scene_tick(Scene *s, int sim_fps)
{
    if (s->paused) return;

    if (s->state == ST_GROWING) {
        scene_grow_step(s);
        fx_build_glow(&s->fx, &s->channel);
    }
    fx_animate(&s->fx, &s->channel, SHIMMER_PCT);

    if (s->state == ST_DONE)
        scene_advance_linger(s, sim_fps);
}

/*
 * cell_put() — the single draw primitive: stamp one glyph at (y,x) in a colour
 * pair + attributes.  Every renderer below is built from this, so it is the only
 * place that talks to ncurses cells and the only place that casts the char.
 */
static void cell_put(WINDOW *w, int y, int x, char ch, ColorID col, attr_t attr)
{
    wattron(w, COLOR_PAIR(col) | attr);
    mvwaddch(w, y, x, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(col) | attr);
}

/*
 * render_color_for_row() — depth colour as a pure function of row: top / middle /
 * bottom third map to the active theme's three bolt stops.  This is the one place
 * the depth gradient is decided; the channel never stored colour at all.
 */
static ColorID render_color_for_row(int y, int rows)
{
    if (y < rows / 3)     return COL_BOLT0;
    if (y < 2 * rows / 3) return COL_BOLT1;
    return COL_BOLT2;
}

/* render_glow() — the ambient halo, drawn first so the bolt sits on top of it. */
static void render_glow(const Channel *c, const Effects *fx, WINDOW *w)
{
    for (int y = 0; y < c->rows; y++) {
        for (int x = 0; x < c->cols; x++) {
            if (channel_occupied(c, x, y)) continue;   /* the bolt draws over its glow */
            uint8_t lv = fx_glow_level(fx, x, y);
            if (lv == 0) continue;
            bool inner = (lv == 2);                     /* level 2 = nearest ring */
            cell_put(w, y, x,
                     inner ? GLOW_INNER_GLYPH : GLOW_OUTER_GLYPH,
                     inner ? COL_GLOW_I       : COL_GLOW_O, A_DIM);
        }
    }
}

/*
 * render_channel() — the bolt itself: depth colour (by row) + shimmer glyph.
 * Bold while growing; once `finished` the bold is dropped for a calmer held
 * bolt, but the shimmer continues.
 */
static void render_channel(const Channel *c, const Effects *fx, WINDOW *w, bool finished)
{
    attr_t attr = finished ? A_NORMAL : A_BOLD;
    for (int y = 0; y < c->rows; y++)
        for (int x = 0; x < c->cols; x++)
            if (channel_occupied(c, x, y))
                cell_put(w, y, x, fx_glyph(fx, x, y), render_color_for_row(y, c->rows), attr);
}

/* render_tips() — live growth fronts as bright '!' in the hottest depth stop. */
static void render_tips(const TipPool *p, const Channel *c, WINDOW *w)
{
    for (int i = 0; i < p->count; i++) {
        const Tip *t = &p->tips[i];
        if (t->active && cell_in_bounds(c, t->x, t->y))
            cell_put(w, t->y, t->x, TIP_GLYPH, COL_BOLT2, A_BOLD);
    }
}

static void render_scene(const Scene *s, WINDOW *w)
{
    render_glow(&s->channel, &s->fx, w);
    render_channel(&s->channel, &s->fx, w, s->state == ST_DONE);
    if (s->state == ST_GROWING)
        render_tips(&s->pool, &s->channel, w);
}

/* ===================================================================== */
/* §9  screen — ncurses lifecycle + HUD                                   */
/* ===================================================================== */

/*
 * Screen — the terminal as a drawing surface.  It holds only the current size,
 * cached from ncurses (getmaxyx) at startup and after each resize, so the hot
 * draw path never re-queries ncurses for dimensions.  The ncurses lifecycle this
 * wraps — initscr, colour setup, resize, teardown — follows Gookin (2007).
 */
typedef struct {
    int cols;   /* terminal width  in character columns; valid x are [0,cols)  */
    int rows;   /* terminal height in character rows;    valid y are [0,rows)  */
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

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* hud_line() — one HUD line, clamped to terminal width so it never wraps. */
static void hud_line(int row, int x, int pair, attr_t bold, int cols, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | bold);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | bold);
}

/* hud_status() — row 0, right-aligned: title, fps, and the run/phase word. */
static void hud_status(const Screen *s, const Scene *sc, double fps)
{
    const char *phase = sc->paused              ? "PAUSED "
                      : sc->state == ST_GROWING ? "growing"
                      :                           "done   ";
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " Lightning  %5.1f fps  %s ", fps, phase);
    hud_line(0, s->cols - (int)strlen(buf), COL_HUD, A_BOLD, s->cols, buf);
}

/* hud_params() — row 1, left: the live, tunable parameters. */
static void hud_params(const Screen *s, const Scene *sc, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " theme:%s  cells:%d  fork:%d%%  spd:%d Hz ",
             g_themes[sc->theme].name, sc->channel.count, sc->fork_pct, sim_fps);
    hud_line(1, 0, COL_HUD, A_NORMAL, s->cols, buf);
}

/* hud_keys() — bottom row: every interactive key. */
static void hud_keys(const Screen *s)
{
    hud_line(s->rows - 1, 0, COL_HINT, A_BOLD, s->cols,
             " q:quit  r:new bolt  t:theme  +/-:fork  [ / ]:speed  spc:pause ");
}

/*
 * screen_draw() — one full frame as named steps: clear, paint the scene, then the
 * three HUD rows (status / params / keys).
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    render_scene(sc, stdscr);
    hud_status(s, sc, fps);
    hud_params(s, sc, sim_fps);
    hud_keys(s);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §10  app — main loop                                                   */
/* ===================================================================== */

/*
 * App — the top-level program state: the simulation, the surface it draws to, the
 * one tunable that lives outside the Scene (tick rate), and the two signal flags.
 */
typedef struct {
    Scene                 scene;         /* the simulation (geometry+fx+tips+state) */
    Screen                screen;        /* cached terminal size                    */
    int                   sim_fps;       /* tick rate in Hz, SIM_FPS_MIN..MAX ([/]) */
    volatile sig_atomic_t running;       /* main-loop flag; cleared by SIGINT/TERM  */
    volatile sig_atomic_t need_resize;   /* set by SIGWINCH, serviced next frame    */
} App;

/*
 * The one global.  Signal handlers receive only an int, so the state they must
 * touch (running / need_resize) has to be reachable without a parameter.  Those
 * two flags are volatile sig_atomic_t — the only kind of object a handler may
 * portably write and the loop may portably read.  Everything else is passed by
 * pointer; nothing else is global.
 */
static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    /* regrow for the new size, keeping the tuned fork rate + paused state */
    scene_start_bolt(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'r': case 'R':
        app->scene.paused = false;
        scene_start_bolt(&app->scene, app->screen.cols, app->screen.rows);
        break;

    case 'p': case 'P': case ' ':
        app->scene.paused = !app->scene.paused;
        break;

    case 't': scene_cycle_theme(&app->scene, +1); break;
    case 'T': scene_cycle_theme(&app->scene, -1); break;

    case '=': case '+':
        app->scene.fork_pct = clamp_int(app->scene.fork_pct + FORK_PCT_STEP,
                                        FORK_PCT_MIN, FORK_PCT_MAX);
        break;
    case '-':
        app->scene.fork_pct = clamp_int(app->scene.fork_pct - FORK_PCT_STEP,
                                        FORK_PCT_MIN, FORK_PCT_MAX);
        break;

    case ']':
        app->sim_fps = clamp_int(app->sim_fps + SIM_FPS_STEP, SIM_FPS_MIN, SIM_FPS_MAX);
        break;
    case '[':
        app->sim_fps = clamp_int(app->sim_fps - SIM_FPS_STEP, SIM_FPS_MIN, SIM_FPS_MAX);
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

    FrameClock clock;
    frameclock_init(&clock);

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frameclock_init(&clock);     /* start timing fresh after the resize */
        }

        frameclock_begin_frame(&clock);

        /* fixed-timestep: advance the sim in equal slices, however fast we draw */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        while (frameclock_step_due(&clock, tick_ns))
            scene_tick(&app->scene, app->sim_fps);

        frameclock_throttle(&clock);

        screen_draw(&app->screen, &app->scene, clock.fps, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
