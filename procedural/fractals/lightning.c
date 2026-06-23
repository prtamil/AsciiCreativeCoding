/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lightning.c — animated branching lightning in the terminal.
 *
 * A bolt grows downward from the top of the screen as a jagged path that
 * randomly splits into dimmer side branches, ending up like a lightning-shaped
 * tree. We keep three things strictly apart: WHERE the bolt is (the channel),
 * how it LOOKS (the glow and flicker), and how it's PAINTED (the renderer). The
 * growth code only ever changes where the bolt is, so a visual tweak can never
 * break the simulation.
 *
 * The branching idea comes from the dielectric-breakdown model of real
 * lightning (Niemeyer, Pietronero & Wiesmann 1984) and the recursive
 * "one tip becomes two" growth of L-systems (Prusinkiewicz & Lindenmayer 1990).
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra lightning.c -o lightning -lncurses
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config ── */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    GRID_ROWS_MAX   =  80,
    GRID_COLS_MAX   = 300,

    MAX_TIPS        =  64,   /* most branch tips that can grow at once          */

    MIN_FORK_STEPS  =   2,   /* a tip must grow at least this far before splitting */
    LEAN_MAX        =   3,   /* how far sideways a branch is allowed to lean    */

    HOLD_AFTER_DONE_MS = 1000, /* how long a finished bolt lingers before the next */
    FRAME_DT_CAP_MS    =  100,  /* ignore stalls longer than this so we don't lurch */
    RENDER_FPS_CAP     =   60,  /* don't repaint the screen faster than this       */
    MS_PER_SEC         = 1000,
    PERCENT            =  100,  /* used for "X percent chance" dice rolls          */

    GLOW_RADIUS     =   2,   /* how many cells out the glow reaches             */

    N_THEMES        =   5,   /* number of colour themes (cycled with t / T)     */
    N_PALETTE       =   5,   /* colours a theme sets: 3 bolt shades + 2 glow    */

    HUD_COLS        =  80,
    FPS_UPDATE_MS   = 500,

    SPARK_UNSEEDED  = 0xFF,  /* "this cell hasn't picked a flicker glyph yet"   */
};

/*
 * LEAN_PCT — chance (percent) a tip leans sideways each step; higher = wider bolt.
 * FORK_PCT — chance (percent) a tip splits each step once it's old enough to.
 *            Adjustable at runtime with + / -, kept between MIN and MAX.
 */
#define LEAN_PCT            60
#define FORK_PCT_DEFAULT    30
#define FORK_PCT_MIN         8
#define FORK_PCT_MAX        50
#define FORK_PCT_STEP        2

/* The glyphs the bolt flickers through, and how many cells re-flicker per tick. */
#define SPARK_CHARS  "#%xX*+|/\\^"
#define SHIMMER_PCT  28

/* Named so the drawing code reads by meaning instead of by raw character. */
#define GLOW_INNER_GLYPH '|'   /* glow right next to the bolt */
#define GLOW_OUTER_GLYPH '.'   /* glow one cell further out   */
#define TIP_GLYPH        '!'   /* a tip that's still growing  */

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/*
 * clamp_int    — keep a value from straying outside [lo,hi].
 * roll_percent — flip a weighted coin: true pct out of 100 times.
 */
static int  clamp_int(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static bool roll_percent(int pct)            { return rand() % PERCENT < pct; }

/* ── §2  clock — read the wall clock, and pace the animation ── */

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
 * FrameClock — keeps the bolt growing at a steady pace no matter how fast or
 * slow the terminal can repaint.
 *
 * The trick: we want the bolt to advance a fixed number of times per second,
 * but screens repaint at all sorts of rates. So each frame we measure how much
 * real time passed and treat it as a "debt" of growth we owe. We pay that debt
 * off in equal-sized chunks, one growth step per chunk. A slow frame just runs
 * a few catch-up steps instead of one giant jump, so the bolt looks the same on
 * any machine.
 *
 * All *_ns fields are nanoseconds. After the main loop drains its catch-up
 * loop, sim_debt is always less than one step's worth.
 */
typedef struct {
    int64_t prev_ns;     /* when this frame started                              */
    int64_t sim_debt;    /* growth time owed but not yet spent                   */
    int64_t fps_window;  /* time gathered for the next fps reading               */
    int     fps_frames;  /* frames gathered for the next fps reading             */
    double  fps;         /* the fps number shown in the HUD, refreshed twice/sec */
} FrameClock;

static void frameclock_init(FrameClock *fc)
{
    fc->prev_ns    = clock_ns();
    fc->sim_debt   = 0;
    fc->fps_window = 0;
    fc->fps_frames = 0;
    fc->fps        = 0.0;
}

/* Count this frame toward the fps display, and publish a fresh number every
 * half second (counting frames more often than that would make it jittery). */
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

/* Start a new frame: see how much real time went by, ignore any huge stall (so
 * one big pause doesn't trigger a flood of catch-up steps), and add the rest to
 * the growth debt. */
static void frameclock_begin_frame(FrameClock *fc)
{
    int64_t now = clock_ns();
    int64_t dt  = now - fc->prev_ns;
    fc->prev_ns = now;

    int64_t stall_cap = FRAME_DT_CAP_MS * NS_PER_MS;
    if (dt > stall_cap) dt = stall_cap;

    fc->sim_debt += dt;
    frameclock_sample_fps(fc, dt);
}

/* True while we still owe at least one growth step (and subtracts it). Call in a
 * `while` loop to run all the steps this frame earned. */
static bool frameclock_step_due(FrameClock *fc, int64_t tick_ns)
{
    if (fc->sim_debt < tick_ns) return false;
    fc->sim_debt -= tick_ns;
    return true;
}

/* If this frame finished early, nap for the leftover time so we don't burn the
 * CPU repainting faster than RENDER_FPS_CAP. A frame that ran long won't sleep. */
static void frameclock_throttle(const FrameClock *fc)
{
    int64_t budget  = NS_PER_SEC / RENDER_FPS_CAP;
    int64_t elapsed = clock_ns() - fc->prev_ns;
    clock_sleep_ns(budget - elapsed);
}

/* ── §3  color ── */

/*
 * ColorID — the named colours this program draws with. ncurses works in numbered
 * "pairs" (a foreground over a background); these are those numbers, but we refer
 * to colours by meaning (COL_BOLT2) instead of by number everywhere else.
 *
 * The bolt is shaded three ways from top to bottom (COL_BOLT0..2) so it looks
 * like it brightens as it descends, and the glow has a near shade and a far
 * shade (COL_GLOW_I / COL_GLOW_O). All five change when you switch themes. The
 * HUD colours stay fixed so the text reads clearly against any theme.
 */
typedef enum {
    COL_BOLT0  = 1,   /* bolt, top third    */
    COL_BOLT1  = 2,   /* bolt, middle third */
    COL_BOLT2  = 3,   /* bolt, bottom third */
    COL_GLOW_I = 4,   /* glow, near the bolt */
    COL_GLOW_O = 5,   /* glow, further out   */
    COL_HUD    = 6,   /* HUD text — bright yellow */
    COL_HINT   = 7,   /* key hints  — bright cyan */
} ColorID;

/*
 * Theme — one named colour scheme. fg256[0..2] are the three bolt shades
 * (top→bottom), fg256[3] is the near glow, fg256[4] the far glow. fg8 is the
 * same set in plain 8-colour terms, used on terminals that can't do 256 colours.
 * Adding a theme is just one more row in the table below.
 *
 * Every colour is picked from the bright half of the palette on purpose: the
 * glow is drawn dim, and dim + a dark colour would vanish against the black sky.
 */
typedef struct {
    const char *name;        /* shown in the HUD, e.g. "Storm"          */
    int fg256[N_PALETTE];    /* the colours on a 256-colour terminal    */
    int fg8  [N_PALETTE];    /* the same idea, for an 8-colour terminal  */
} Theme;

static const Theme g_themes[N_THEMES] = {
    /*            name      bolt:top mid  bot   glow:near far     8-colour fallback (same order)                               */
    { "Storm",  {  45,  51, 231,  30,  26 }, { COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE, COLOR_CYAN,    COLOR_BLUE  } },
    { "Fire",   { 196, 208, 231,  94,  88 }, { COLOR_RED,     COLOR_YELLOW,  COLOR_WHITE, COLOR_RED,     COLOR_RED   } },
    { "Toxic",  {  46, 118, 231,  34,  28 }, { COLOR_GREEN,   COLOR_GREEN,   COLOR_WHITE, COLOR_GREEN,   COLOR_GREEN } },
    { "Plasma", {  99, 141, 231,  60,  54 }, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_MAGENTA, COLOR_BLUE  } },
    { "Mono",   { 245, 251, 231, 245, 240 }, { COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE, COLOR_WHITE,   COLOR_WHITE } },
};

/* Point the five themeable colours at the chosen theme. */
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
    /* HUD colours never change with the theme, so the text always stays readable */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* start on Storm */
}

/* ── §4  channel — where the bolt is ── */

/*
 * CellGrid — one byte for every cell on the screen, addressed grid[y][x] (row
 * first, like ncurses). Three different things in this program have this exact
 * shape — which cells are bolt, how bright the glow is, and which flicker glyph a
 * cell shows — so naming the layout once makes that family obvious.
 *
 * It's sized for the biggest screen we'll ever handle and lives inside its owner
 * by value (no malloc). This project never allocates memory while running, so we
 * reserve the worst case up front and only ever touch the cells in use.
 */
typedef uint8_t CellGrid[GRID_ROWS_MAX][GRID_COLS_MAX];

/*
 * Channel — the only record of WHERE the bolt is, and nothing about how it looks.
 * hit[y][x] is 1 when that cell is part of the bolt and 0 when it's empty sky.
 * No colour, no glyph, no glow live here — those are all worked out later from
 * this map. Keeping it this bare means the growth code can only change where the
 * bolt is, so a visual change can never accidentally break the bolt's shape.
 */
typedef struct {
    CellGrid hit;       /* 1 = part of the bolt, 0 = empty sky          */
    int      count;     /* how many cells are lit (shown in the HUD)    */
    int      rows;      /* screen height in use, up to GRID_ROWS_MAX    */
    int      cols;      /* screen width  in use, up to GRID_COLS_MAX    */
} Channel;

static void channel_init(Channel *c, int cols, int rows, int seed_x)
{
    c->cols = clamp_int(cols, 1, GRID_COLS_MAX);
    c->rows = clamp_int(rows, 1, GRID_ROWS_MAX);
    memset(c->hit, 0, sizeof c->hit);

    seed_x = clamp_int(seed_x, 0, c->cols - 1);   /* the bolt starts at the top row */
    c->hit[0][seed_x] = 1;
    c->count          = 1;
}

static bool cell_in_bounds(const Channel *c, int x, int y)
{
    return x >= 0 && x < c->cols && y >= 0 && y < c->rows;
}

/* Light up a cell. The caller must have already checked it's on screen. */
static void channel_mark(Channel *c, int x, int y)
{
    c->hit[y][x] = 1;
    c->count++;
}

static bool channel_occupied(const Channel *c, int x, int y)
{
    return cell_in_bounds(c, x, y) && c->hit[y][x] != 0;
}

/* ── §5  tip — the growing ends of the bolt ── */

/*
 * Tip — one growing end of the bolt: a point that steps down one cell at a time,
 * occasionally splitting in two. Many tips together trace out the whole jagged,
 * branching shape. (This is the "grow it tip by tip" view of lightning rather
 * than solving the whole field at once; Reed & Wyvill 1994.)
 *
 *   x, y    where the tip is right now, always on screen.
 *   lean    how it drifts sideways. 0 means straight down; the sign says left or
 *           right; the size says how hard it leans. Children inherit their
 *           parent's lean nudged by one, so the deeper a branch is, the more it
 *           leans — that's what makes siblings fan apart into a tree.
 *   steps   how many cells this tip has grown since it was born or last split.
 *           A tip can't split until this passes MIN_FORK_STEPS, so no branch is
 *           born as a tiny stub.
 *   active  true while it's still growing; turned off the moment it hits the
 *           ground, a side wall, or a cell that's already bolt.
 */
typedef struct {
    int  x, y;
    int  lean;
    int  steps;
    bool active;
} Tip;

/*
 * TipPool — every tip the current bolt has ever spawned, growing or finished, in
 * one fixed array. The whole branching tree comes from a single rule applied to
 * this pool: one tip becomes two (the L-system idea, Prusinkiewicz &
 * Lindenmayer 1990).
 *
 * Tips are only ever added, never removed or shuffled: when a tip splits we
 * append its two children and just mark the parent finished in place. That means
 * a pointer to a tip stays valid for the rest of the bolt, which is what lets the
 * split code keep using its parent pointer while it appends the children.
 */
typedef struct {
    Tip tips[MAX_TIPS];   /* finished tips stay in place, just switched off  */
    int count;            /* how many slots are used (growing + finished)    */
} TipPool;

static void tippool_clear(TipPool *p)
{
    p->count = 0;
    memset(p->tips, 0, sizeof p->tips);
}

/* Start a new tip at (x,y) leaning a given way, unless the pool is full. */
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
 * Work out where a tip wants to go next: always one row down, and some of the
 * time (LEAN_PCT) also one column in its leaning direction. The column is kept on
 * screen; the row is allowed to fall off the bottom, which the caller treats as
 * "hit the ground".
 */
static void tip_next_cell(const Tip *t, const Channel *c, int *nx, int *ny)
{
    int dx = 0;
    if (t->lean != 0 && roll_percent(LEAN_PCT))
        dx = (t->lean > 0) ? 1 : -1;

    *nx = clamp_int(t->x + dx, 0, c->cols - 1);
    *ny = t->y + 1;
}

/*
 * Split one tip into two. The children start where the parent is, one leaning a
 * touch more left and one a touch more right, so they spread apart as they fall
 * and build up the branching tree. The parent stops; the children take over.
 * Does nothing if there isn't room for both.
 */
static void tippool_fork(TipPool *p, Tip *parent)
{
    if (p->count > MAX_TIPS - 2) return;
    int left  = clamp_int(parent->lean - 1, -LEAN_MAX, LEAN_MAX);
    int right = clamp_int(parent->lean + 1, -LEAN_MAX, LEAN_MAX);
    tippool_add(p, parent->x, parent->y, left);
    tippool_add(p, parent->x, parent->y, right);
    parent->active = false;
}

/* A tip may split only after it's grown a little (so no stub branches), and then
 * only on a fork_pct-out-of-100 roll. */
static bool tip_may_fork(const Tip *t, int fork_pct)
{
    return t->steps >= MIN_FORK_STEPS && roll_percent(fork_pct);
}

/* A tip is done if its next cell is below the ground or already part of the bolt
 * (a real bolt never doubles back over itself). */
static bool growth_blocked(const Channel *c, int x, int y)
{
    return y >= c->rows || channel_occupied(c, x, y);
}

/* ── §6  effects — the glow and the flicker (looks only) ── */

/*
 * Effects — the two purely cosmetic layers. Both are worked out FROM the channel
 * and feed back into nothing: every function here reads the bolt and writes only
 * here, so you could delete this whole struct and the bolt would grow exactly the
 * same — it'd just look plainer. This is where all the "make it look electric"
 * choices live.
 *
 * Both fields are one value per screen cell, addressed [y][x]:
 *   glow   a faint halo around the bolt (the discharge corona of Kim & Lin 2004).
 *          A cell's value is how close it is to the nearest bolt cell, measured
 *          in steps up/down/left/right: 2 = right next to the bolt (brightest),
 *          1 = one further out, 0 = no glow. Rebuilt whenever the bolt changes.
 *   spark  which flicker glyph each bolt cell is currently showing (an index into
 *          SPARK_CHARS), or SPARK_UNSEEDED if it hasn't been given one yet. We
 *          re-roll some of these over time so the bolt crackles instead of
 *          sitting frozen.
 */
typedef struct {
    CellGrid glow;     /* per cell: 0 = none, 1 = outer glow, 2 = inner glow */
    CellGrid spark;    /* per cell: which flicker glyph, or SPARK_UNSEEDED   */
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
