/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diamond_square_heightmap_showcase.c — fractal terrain that grows on screen.
 *
 * Start from four random corner heights, then keep filling in the points
 * between them: each new point is the average of its neighbours plus a
 * shrinking dab of randomness. Big landforms get shaped first, finer detail
 * later, so the result looks like real terrain — water, beach, grass, hills,
 * mountains, snow — instead of plain static. This is the classic
 * Diamond-Square algorithm (Fournier, Fussell & Carpenter 1982).
 *
 * Sister file: ./bsp_dungeon_showcase.c — also builds by recursively
 * halving space, but carves discrete rooms instead of a smooth height field.
 */

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

/* ── §1  CONFIG & DATA — constants and data types ── */

enum {
    /* Biggest grid we allow: a 129x129 map (N=128). That fits a wide
     * terminal and keeps the saved state small. The grid is always
     * (N+1)x(N+1) with N a power of two — the algorithm needs that. */
    GRID_N_MAX        = 128,
    GRID_N_MIN        =   8,        /* smallest worth drawing: a 9x9 */

    /* How many terminal columns wide we draw each map cell. Terminal
     * characters are about twice as tall as they are wide, so drawing a
     * cell a few columns wide keeps the map looking square instead of
     * tall and skinny. If the terminal is too narrow to fit, the
     * auto-sizer just uses a smaller grid. */
    CELL_COLS         =   3,

    GRID_W_MAX        = GRID_N_MAX + 1,
    CELLS_MAX         = GRID_W_MAX * GRID_W_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  =   8,        /* how many cells we fill in per tick */
    OPS_PER_TICK_MAX  = 512,

    FPS_UPDATE_MS     = 500,

    /* Rows kept clear for the heads-up display; the map sits between them. */
    HUD_TOP_ROWS      =   2,        /* top two rows: the info lines  */
    HUD_BOTTOM_ROWS   =   1,        /* bottom row: the key hint      */

    /* Colour-pair slots. HUD/HINT are reserved by the project's HUD rule. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WATER        =   3,
    PAIR_BEACH        =   4,
    PAIR_GRASS        =   5,
    PAIR_HILL         =   6,
    PAIR_MOUNTAIN     =   7,
    PAIR_SNOW         =   8,
};

/* Each corner starts somewhere in the middle of the height range, not at the
 * very top or bottom, so later steps have room to push up or down. */
#define CORNER_MIN       0.2f
#define CORNER_RANGE     0.6f

/* Where one terrain type ends and the next begins, measured on the 0..1
 * height scale. We rescale every frame to that range (see normalize01) so
 * all six terrain types always show up, wherever the raw numbers landed. */
#define BAND_WATER       0.30f
#define BAND_BEACH       0.40f
#define BAND_GRASS       0.55f
#define BAND_HILL        0.72f
#define BAND_MOUNTAIN    0.88f

/* If the computed heights are all nearly equal, treat the spread as 1 so we
 * don't divide by (almost) zero when rescaling — happens early on, when only
 * the corners are set. */
#define HSPAN_EPSILON    1e-6f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* How fast we repaint the screen. This is separate from how fast the
 * terrain fills in (that's controls.sim_fps). */
#define RENDER_HZ        60
#define RENDER_PERIOD_NS (NS_PER_SEC / RENDER_HZ)

/* If a single frame somehow takes longer than this (a debugger pause, the
 * terminal getting suspended, a slow resize), pretend it was only this long.
 * Otherwise the loop tries to "make up" all the lost time at once and freezes
 * in a burst of catch-up ticks. 100 ms is at most ~6 ticks at 60 Hz. */
#define MAX_FRAME_DT_NS  (100 * NS_PER_MS)

/*
 * The algorithm alternates between two kinds of step, named for the shape
 * the SOURCE points make around the point being filled in (not the point
 * itself — that's the usual mix-up):
 *   OP_DIAMOND : fill a square's centre from its 4 corners (a square around it)
 *   OP_SQUARE  : fill a side's midpoint from its up-to-4 neighbours (a "+")
 * Stored in Op.kind and Subdivision.phase to say which step is meant.
 */
enum { OP_DIAMOND = 0, OP_SQUARE = 1 };

/*
 * Preset — one row that carries both a colour scheme and a set of terrain
 * knobs. The two halves are chosen separately: t/T pick the COLOURS (recolour
 * what's on screen, no regeneration), n/p pick the terrain SHAPE and rebuild.
 * So you can put any colour scheme on any landscape shape — mix freely.
 *
 *   name        : label shown in the HUD
 *   colors[6]   : one colour for each terrain type, in order WATER, BEACH,
 *                 GRASS, HILL, MOUNTAIN, SNOW. All chosen from the bright
 *                 half of the 256-colour range so nothing renders near-black.
 *   roughness   : how tall the FIRST, biggest landforms are — how dramatic
 *                 the overall relief is. Presets run ~0.35 (gentle) to 0.75.
 *   persistence : how quickly the randomness shrinks at each finer level.
 *                 Higher keeps small bumps alive -> jagged, busy coastlines;
 *                 lower smooths them away fast -> rounded hills. 0.5 is the
 *                 textbook value.
 *   bias        : a render-only tilt of how much land falls into low vs high
 *                 terrain types. Above 1 sinks everything -> more water,
 *                 island-y; below 1 raises everything -> more peaks; 1 leaves
 *                 it even. It only changes which type a height shows as, not
 *                 the height itself.
 */
typedef struct {
    const char *name;
    short       colors[6];
    float       roughness;
    float       persistence;
    float       bias;
} Preset;

#define N_PRESETS 15

static const Preset presets[N_PRESETS] = {
    /* name        WATER BEACH GRASS HILL  MTN   SNOW    rough  persist bias  */
    { "CLASSIC",  {  33, 221,  34,  28, 244, 231 }, 0.55f, 0.50f, 1.00f },  /* balanced default        */
    { "ATOLL",    {  24,  25,  31,  38,  45,  51 }, 0.45f, 0.45f, 1.55f },  /* smooth, water-heavy     */
    { "ALPINE",   {  25,  31,  39,  51, 189, 231 }, 0.70f, 0.60f, 0.62f },  /* jagged, peak-heavy ice  */
    { "CANYON",   {  52, 124, 160, 196, 208, 226 }, 0.72f, 0.60f, 0.95f },  /* sharp red mesas         */
    { "DUNES",    {  24, 222, 178, 137,  94, 230 }, 0.35f, 0.42f, 1.05f },  /* very smooth sand        */
    { "MATRIX",   {  28,  34,  40,  46,  82, 118 }, 0.55f, 0.55f, 1.00f },  /* digital greens          */
    { "MONO",     { 240, 244, 247, 250, 253, 255 }, 0.60f, 0.55f, 1.00f },  /* greyscale relief        */
    { "NOVA",     {  53,  92, 129, 165, 201, 219 }, 0.58f, 0.52f, 0.90f },  /* purple → pink           */
    { "EARTH",    {  58, 180,  64,  94, 137, 187 }, 0.50f, 0.50f, 1.00f },  /* brown → cream           */
    { "FOREST",   {  28,  64, 100, 136, 172, 187 }, 0.52f, 0.48f, 1.10f },  /* green → olive → tan     */
    { "VOLCANIC", {  52,  88, 124, 166, 202, 220 }, 0.68f, 0.62f, 0.80f },  /* jagged peak-heavy fire  */
    { "GLACIER",  {  31,  38,  45,  51, 159, 231 }, 0.40f, 0.45f, 1.30f },  /* smooth ice, water-heavy */
    { "SUNSET",   {  54,  96, 168, 204, 215, 229 }, 0.50f, 0.50f, 1.00f },  /* warm dusk gradient      */
    { "TROPIC",   {  30,  37,  42,  76, 148, 229 }, 0.48f, 0.50f, 1.40f },  /* lagoon, water-heavy     */
    { "INFERNO",  {  52, 124, 166, 202, 208, 231 }, 0.75f, 0.62f, 0.70f },  /* extreme jagged peaks    */
};

/* Which single terrain type to show, picked by the w/b/g/h/m/s keys.
 * FILTER_ALL means show everything. */
enum {
    FILTER_ALL      = 0,
    FILTER_WATER    = 1,
    FILTER_BEACH    = 2,
    FILTER_GRASS    = 3,
    FILTER_HILLS    = 4,
    FILTER_MOUNTAIN = 5,
    FILTER_SNOW     = 6,
};

/*
 * Tile — how one terrain type looks on screen: its colour, any extra
 * attribute, and the character to draw. Bundling all three lets the
 * classifier (band_for_height, §3) hand back a complete look in one value.
 * The characters ~ . , ; # @ run from low ground to high, lightest to
 * heaviest (Bourke's ASCII-shading idea — see file header).
 *
 *   pair  : which colour to use (one of PAIR_WATER..PAIR_SNOW)
 *   attr  : A_NORMAL, or A_BOLD to brighten the snow so peaks always show
 *   glyph : the ASCII character drawn for this terrain type
 */
typedef struct {
    int  pair;
    int  attr;
    char glyph;
} Tile;

/*
 * HeightRange — the lowest height drawn so far and how far it spreads up to
 * the highest. We rescale every cell against this window (not a fixed range)
 * so all six terrain types keep showing as the map fills and its highs and
 * lows drift.
 *   min  : lowest computed height this frame
 *   span : highest minus lowest, but never below HSPAN_EPSILON, so we never
 *          divide by zero when everything is nearly flat
 */
typedef struct {
    float min, span;
} HeightRange;

/*
 * Op — one cell waiting to be filled in. We don't fill the whole map at once;
 * we line the cells up and do a few each frame, so you can watch the terrain
 * appear.
 *
 * Each Op carries its own copy of step and rough, taken at the moment it was
 * lined up — NOT read fresh when it's finally computed. That matters because
 * the work-in-progress cursor (Subdivision) may have already moved on to a
 * finer level by then; a stale read would change the neighbour spacing and
 * randomness under the op's feet and corrupt the map.
 *
 *   x, y  : which cell to fill (each 0..N)
 *   kind  : OP_DIAMOND (a square's centre) or OP_SQUARE (a side's midpoint)
 *   step  : the spacing of this op's level; it averages neighbours at ±step/2
 *   rough : how big a random nudge to add at this level
 */
typedef struct {
    int     x, y;
    uint8_t kind;
    int     step;
    float   rough;
} Op;

/*
 * HeightField — the grid of heights the algorithm builds; this is the output.
 * It's stored as one flat array, row by row: cell (x,y) lives at y*w + x.
 *
 * The side length is always a power of two plus one (so 9, 17, 33, ... up to
 * 129). That's not a style choice — the algorithm keeps halving the spacing,
 * and only this size guarantees every halfway point lands on an exact cell.
 * Any other size eventually asks for a fractional cell and breaks.
 *
 * We keep a separate computed[] flag per cell instead of using a special
 * "empty" height, because every number — even 0 or a negative — is a valid
 * height here, so there's no spare value to mean "not set yet". The flag lets
 * the renderer leave not-yet-filled cells blank during the reveal.
 *
 * The simulation (§4) writes this; the renderer (§6) only reads it.
 *
 *   w, h, N    : the grid is (N+1)x(N+1), so w == h == N+1
 *   height[]   : each cell's height, raw and unclamped (clamping while
 *                building flattens the terrain; the renderer rescales on
 *                read instead). Sized for the largest grid.
 *   computed[] : has this cell been filled in yet? (see above)
 */
typedef struct {
    int   w, h, N;
    float height  [CELLS_MAX];
    bool  computed[CELLS_MAX];
} HeightField;

/*
 * Subdivision — the algorithm's scratch space while it's still building the
 * map: where it is in the halving process, plus the queue of cells the
 * current level still owes. (The HeightField is the finished result; this is
 * the work-in-progress.)
 *
 * It works through ever-finer spacings (step = N, N/2, N/4, ... down to 2),
 * and at each spacing does a DIAMOND pass then a SQUARE pass — in that order,
 * because the square pass averages the points the diamond pass just placed.
 * Each pass lines up its cells in the queue; a few get filled per tick, which
 * is what makes the terrain appear gradually.
 *
 * The queue is append-only: we never delete finished ops, just move the
 * "next to do" cursor forward. Keeping the old ops lets us glance back at the
 * last one done to tell whether we just finished a diamond or a square pass,
 * so we know what comes next — no extra bookkeeping needed. It's sized for
 * the whole grid, which is always enough for one level's worth of cells.
 *
 *   queue[]      : cells waiting to be filled, in order
 *   qhead, qtail : next-to-do cursor / next-free-slot. Equal means this
 *                  pass is finished.
 *   level_step   : current spacing — how big the squares we're splitting are.
 *                  Halves each level.
 *   level_rough  : current size of the random nudge; shrinks by persistence
 *                  each level. Big at first (continents), tiny later
 *                  (ripples) — this shrinking is exactly what makes it look
 *                  like terrain rather than static.
 *   persistence  : how fast the nudge shrinks (copied from the chosen
 *                  preset, so changing presets only takes effect next reset)
 *   phase        : OP_DIAMOND or OP_SQUARE — which pass we're on
 *   done         : true once we've gone as fine as we can. We stop at step 2
 *                  (half = 1): at step 1 the half would be 0, which would
 *                  divide by zero and loop forever on `y += half`.
 */
typedef struct {
    Op    queue[CELLS_MAX];
    int   qhead, qtail;
    int   level_step;
    float level_rough;
    float persistence;
    int   phase;
    bool  done;
} Subdivision;

/*
 * Controls — everything the viewer can change at runtime, kept apart from the
 * algorithm's own state. The terrain depends only on these settings plus the
 * random seed, so this struct is a tidy list of exactly what you can
 * influence. Two settings pace the work (ops_per_tick, sim_fps), two pick the
 * look (preset, theme), and the rest pause or filter.
 *
 *   paused         : freeze the build (the screen still redraws)
 *   ops_per_tick   : cells filled per tick — how fast the reveal goes.
 *                    +/- double or halve it, within [1..512].
 *   sim_fps        : how many ticks per second. ]/[ adjust it within
 *                    [10..240]. Reveal speed is ops_per_tick x sim_fps.
 *   filter_band    : show all terrain, or just one type (w/b/g/h/m/s);
 *                    'a' goes back to showing all
 *   current_preset : which preset's terrain SHAPE to use (n/p)
 *   current_theme  : which preset's COLOURS to use (t/T) — independent of
 *                    the shape, so any colours can dress any landscape
 */
typedef struct {
    bool paused;
    int  ops_per_tick;
    int  sim_fps;
    int  filter_band;
    int  current_preset;
    int  current_theme;
} Controls;

/*
 * Where the program is in its lifecycle:
 *   COMPUTING — still building the terrain, a bit more each tick
 *   HOLD      — finished; the map just stays on screen. It never restarts on
 *               its own — only when you ask (r / n / p) — so you can look at a
 *               finished map as long as you like.
 */
typedef enum {
    SCENE_COMPUTING = 0,
    SCENE_HOLD      = 1,
} SceneState;

/*
 * Scene — the whole running demo gathered in one place, ordered as the three
 * questions you'd ask about it: WHAT is being built (grid + subdiv), HOW the
 * user steers it (controls), and WHERE it is in its lifecycle (state).
 *
 * Even though the state lives together here, each function still takes only
 * the small piece it needs — the renderer gets just the read-only grid, never
 * the whole Scene — so grouping the data doesn't let the layers reach into
 * each other.
 */
typedef struct {
    /* WHAT is being built */
    HeightField grid;            /* the terrain heights                   */
    Subdivision subdiv;          /* the work-in-progress + its queue      */

    /* HOW the user steers it */
    Controls    controls;

    /* WHERE in the lifecycle */
    SceneState  state;
    int         grid_N;          /* grid side to use for this run, picked from
                                  * the terminal size. Kept here (not just in
                                  * the field) so a resize can record the new
                                  * size and the next reset rebuilds to match. */
} Scene;

/*
 * Screen — just the terminal's width and height in characters. That's all we
 * track, because ncurses handles the actual drawing buffer itself; we only
 * need the size to centre the map and place the HUD. Re-read at startup and
 * after every resize.
 */
typedef struct { int cols, rows; } Screen;

/* ── §2  PERFORMANCE — clock and sleep helpers (the main loop is §7) ── */

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

/* How long since the last frame, but never reported as longer than our cap,
 * so one big stall can't snowball. Also updates *prev to "now". */
static int64_t frame_delta_ns(int64_t *prev)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *prev;
    *prev = now;
    if (dt > MAX_FRAME_DT_NS) dt = MAX_FRAME_DT_NS;
    return dt;
}

/* Sleep just long enough that the whole frame lasts one render period, so we
 * hold a steady frame rate. We sleep before drawing, so draw time doesn't
 * throw the pacing off. */
static void pace_frame(int64_t frame_start, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(RENDER_PERIOD_NS - elapsed);
}

/* ── §3  LOGIC — pure helpers: data in, answer out, nothing changed ── */

static inline int grid_idx(const HeightField *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds(const HeightField *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/* Which terrain type a 0..1 height falls into, as a filter number. Uses the
 * same cut-offs as band_for_height, just so the filter and the drawing agree. */
static inline int band_index_for_norm(float v)
{
    if (v < BAND_WATER)    return FILTER_WATER;
    if (v < BAND_BEACH)    return FILTER_BEACH;
    if (v < BAND_GRASS)    return FILTER_GRASS;
    if (v < BAND_HILL)     return FILTER_HILLS;
    if (v < BAND_MOUNTAIN) return FILTER_MOUNTAIN;
    return FILTER_SNOW;
}

static const char *band_name(int filter)
{
    switch (filter) {
    case FILTER_WATER:    return "WATER";
    case FILTER_BEACH:    return "BEACH";
    case FILTER_GRASS:    return "GRASS";
    case FILTER_HILLS:    return "HILLS";
    case FILTER_MOUNTAIN: return "MOUNTAIN";
    case FILTER_SNOW:     return "SNOW";
    default:              return "ALL";
    }
}

static int band_color_pair(int filter)
{
    switch (filter) {
    case FILTER_WATER:    return PAIR_WATER;
    case FILTER_BEACH:    return PAIR_BEACH;
    case FILTER_GRASS:    return PAIR_GRASS;
    case FILTER_HILLS:    return PAIR_HILL;
    case FILTER_MOUNTAIN: return PAIR_MOUNTAIN;
    case FILTER_SNOW:     return PAIR_SNOW;
    default:              return PAIR_HUD;
    }
}

/* Turn a 0..1 height into the full look to draw it with — colour, attribute,
 * and character. Terrain types run low to high. */
static Tile band_for_height(float h)
{
    if (h < BAND_WATER)    return (Tile){ PAIR_WATER,    A_NORMAL, '~' };
    if (h < BAND_BEACH)    return (Tile){ PAIR_BEACH,    A_NORMAL, '.' };
    if (h < BAND_GRASS)    return (Tile){ PAIR_GRASS,    A_NORMAL, ',' };
    if (h < BAND_HILL)     return (Tile){ PAIR_HILL,     A_NORMAL, ';' };
    if (h < BAND_MOUNTAIN) return (Tile){ PAIR_MOUNTAIN, A_NORMAL, '#' };
    return (Tile){ PAIR_SNOW, A_BOLD, '@' };
}

/* Rescale a raw height to 0..1 — where does it sit between the lowest and
 * highest heights drawn so far? — and keep it inside that range. */
static inline float normalize01(float value, float min, float span)
{
    float v = (value - min) / span;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

/* Find the lowest and highest heights drawn so far. The renderer rescales
 * against this each frame, so all six terrain types keep showing up as the
 * extremes shift. */
static HeightRange computed_height_range(const HeightField *g)
{
    float min = 1.0f, max = 0.0f;
    bool any = false;
    int n = g->w * g->h;
    for (int i = 0; i < n; i++) {
        if (!g->computed[i]) continue;
        float v = g->height[i];
        if (!any) { min = v; max = v; any = true; }
        else { if (v < min) min = v; if (v > max) max = v; }
    }
    float span = (max - min > HSPAN_EPSILON) ? (max - min) : 1.0f;
    return (HeightRange){ min, span };
}

/* Average of the up/down/left/right neighbours that actually exist, `half`
 * cells away (used by the square step). Returns false if there are none.
 * Only real neighbours count, so a cell on the edge averages over 3, not a
 * fake 4 — pretending the missing one is 0 would drag the whole border down
 * into deep water. */
static bool cardinal_neighbour_mean(const HeightField *g, int x, int y,
                                    int half, float *mean)
{
    float sum = 0.0f;
    int count = 0;
    if (grid_in_bounds(g, x - half, y)) { sum += g->height[grid_idx(g, x - half, y)]; count++; }
    if (grid_in_bounds(g, x + half, y)) { sum += g->height[grid_idx(g, x + half, y)]; count++; }
    if (grid_in_bounds(g, x, y - half)) { sum += g->height[grid_idx(g, x, y - half)]; count++; }
    if (grid_in_bounds(g, x, y + half)) { sum += g->height[grid_idx(g, x, y + half)]; count++; }
    if (count == 0) return false;
    *mean = sum / (float)count;
    return true;
}

/* Which halving level we're on, and how many there are in all — just for the
 * HUD's "level 3/7" readout. */
static void subdivision_progress(const HeightField *g, const Subdivision *sd,
                                 int *current, int *total)
{
    int tot = 0;
    for (int v = g->N; v > 1; v >>= 1) tot++;
    int cur = 1;
    for (int v = g->N; v > sd->level_step && v > 1; v >>= 1) cur++;
    if (cur > tot) cur = tot;
    *current = cur;
    *total   = tot;
}

/* Pick the biggest grid that still fits the terminal (after reserving the HUD
 * rows), falling back to the minimum if the terminal is tiny. */
static int pick_grid_N(int cols, int rows)
{
    int avail_w = cols / CELL_COLS;                        /* each cell is CELL_COLS wide */
    int avail_h = rows - (HUD_TOP_ROWS + HUD_BOTTOM_ROWS); /* rows left for the map      */
    int min_dim = (avail_w < avail_h) ? avail_w : avail_h;
    int N = 1;
    while ((N * 2) + 1 <= min_dim && (N * 2) <= GRID_N_MAX) {
        N *= 2;
    }
    if (N < GRID_N_MIN) N = GRID_N_MIN;
    if (N > GRID_N_MAX) N = GRID_N_MAX;
    return N;
}

/* ── §4  SIMULATION — the only code that changes the terrain ── */

/* The randomness behind the whole thing. rand_unit gives 0..1, rand_signed
 * gives -1..1 (the per-cell nudge, scaled by the level's roughness). */
static inline float rand_unit(void)
{
    return (float)rand() / (float)RAND_MAX;
}
static inline float rand_signed(void)
{
    return rand_unit() * 2.0f - 1.0f;
}

/* The one place a cell's height gets written: store it and mark it filled. */
static void grid_set(HeightField *g, int idx, float value)
{
    g->height[idx]   = value;
    g->computed[idx] = true;
}

/* Line up every diamond cell of this level: the centre of each square at the
 * current spacing. They get filled later, a few per tick. */
static void subdiv_schedule_diamond(Subdivision *sd, const HeightField *g)
{
    sd->phase = OP_DIAMOND;
    int S = sd->level_step;
    int half = S / 2;
    for (int y = half; y < g->N; y += S) {
        for (int x = half; x < g->N; x += S) {
            sd->queue[sd->qtail++] = (Op){
                .x = x, .y = y, .kind = OP_DIAMOND,
                .step = S, .rough = sd->level_rough
            };
        }
    }
}

/* Line up every square cell of this level: the midpoints of the sides — the
 * points that sit between the diamond centres we just placed. */
static void subdiv_schedule_square(Subdivision *sd, const HeightField *g)
{
    sd->phase = OP_SQUARE;
    int S = sd->level_step;
    int half = S / 2;
    for (int y = 0; y <= g->N; y += half) {
        /* Alternate the starting column row by row so we hit the midpoints
         * and skip the diamond centres. */
        int x_start = ((y / half) & 1) ? 0 : half;
        for (int x = x_start; x <= g->N; x += S) {
            sd->queue[sd->qtail++] = (Op){
                .x = x, .y = y, .kind = OP_SQUARE,
                .step = S, .rough = sd->level_rough
            };
        }
    }
}

/* The diamond step: fill a square's centre with the average of its four
 * corners, plus a small random nudge. */
static void grid_compute_diamond(HeightField *g, const Op *op)
{
    int half = op->step / 2;
    int x = op->x, y = op->y;

    float corner_mean = 0.25f * (
        g->height[grid_idx(g, x - half, y - half)] +
        g->height[grid_idx(g, x + half, y - half)] +
        g->height[grid_idx(g, x - half, y + half)] +
        g->height[grid_idx(g, x + half, y + half)]);
    float displacement = rand_signed() * op->rough;

    grid_set(g, grid_idx(g, x, y), corner_mean + displacement);
}

/* The square step: fill a side's midpoint with the average of its real
 * neighbours, plus a small random nudge. Edge cells have fewer neighbours. */
static void grid_compute_square(HeightField *g, const Op *op)
{
    int half = op->step / 2;
    int x = op->x, y = op->y;

    float neighbour_mean;
    if (!cardinal_neighbour_mean(g, x, y, half, &neighbour_mean))
        return;                                        /* no neighbours — just skip */
    float displacement = rand_signed() * op->rough;

    grid_set(g, grid_idx(g, x, y), neighbour_mean + displacement);
}

/* Drop to the next finer level: halve the spacing, shrink the randomness, and
 * line up its diamond cells. We stop before the spacing would hit 1, since the
 * half would then be 0 — a divide-by-zero and an endless loop. */
static void subdiv_advance_level(Subdivision *sd, const HeightField *g)
{
    int next = sd->level_step / 2;
    if (next < 2) {
        sd->done = true;
        return;
    }
    sd->level_step  = next;
    sd->level_rough *= sd->persistence;
    subdiv_schedule_diamond(sd, g);
}

/* Fill in one cell, doing whichever step it asked for. */
static void grid_compute_op(HeightField *g, const Op *op)
{
    if (op->kind == OP_DIAMOND) grid_compute_diamond(g, op);
    else                        grid_compute_square (g, op);
}

/* This pass is finished — start the next one. A diamond pass is followed by a
 * square pass at the same spacing; a square pass drops to a finer level. We
 * tell which we just did by glancing at the last queued cell. Returns false
 * when there's nothing left to do. */
static bool subdiv_open_next(Subdivision *sd, const HeightField *g)
{
    if (sd->qhead == 0) return false;   /* nothing was ever scheduled */

    uint8_t last_phase = sd->queue[sd->qhead - 1].kind;
    if (last_phase == OP_DIAMOND) subdiv_schedule_square(sd, g);
    else                          subdiv_advance_level(sd, g);

    if (sd->done) return false;
    return sd->qhead < sd->qtail;       /* true if there's now work queued */
}

/* Fill in one more cell. Returns false once the whole map is done. */
static bool subdiv_step(Subdivision *sd, HeightField *g)
{
    if (sd->done) return false;
    if (sd->qhead >= sd->qtail && !subdiv_open_next(sd, g)) return false;

    Op op = sd->queue[sd->qhead++];
    grid_compute_op(g, &op);
    return true;
}

/* Start a blank map of side N+1: nothing filled in yet. */
static void grid_clear(HeightField *g, int N)
{
    g->N = N;
    g->w = N + 1;
    g->h = N + 1;

    int n = g->w * g->h;
    for (int i = 0; i < n; i++) {
        g->height[i]   = 0.0f;
        g->computed[i] = false;
    }
}

/* Set the four corners to random heights — the fixed points everything else
 * is built between. */
static void grid_seed_corners(HeightField *g)
{
    int N = g->N;
    int corners[4][2] = {{0, 0}, {N, 0}, {0, N}, {N, N}};
    for (int i = 0; i < 4; i++) {
        int idx = grid_idx(g, corners[i][0], corners[i][1]);
        grid_set(g, idx, CORNER_MIN + rand_unit() * CORNER_RANGE);
    }
}

/* Blank map with fresh random corners, ready to grow. */
static void grid_seed(HeightField *g, int N)
{
    grid_clear(g, N);
    grid_seed_corners(g);
}

/* Begin building over the freshly-seeded grid: start at the coarsest spacing
 * with the preset's settings, and line up the first diamond pass. */
static void subdiv_begin(Subdivision *sd, const HeightField *g,
                         float roughness, float persistence)
{
    sd->qhead = 0;
    sd->qtail = 0;
    sd->level_step  = g->N;
    sd->level_rough = roughness;
    sd->persistence = persistence;
    sd->done = false;
    subdiv_schedule_diamond(sd, g);
}

static void scene_reset(Scene *s)
{
    const Preset *p = &presets[s->controls.current_preset];
    grid_seed(&s->grid, s->grid_N);
    subdiv_begin(&s->subdiv, &s->grid, p->roughness, p->persistence);
    s->state = SCENE_COMPUTING;
    /* The settings and grid size carry over, so 'r' (or a filter key) just
     * rebuilds a fresh map with the same knobs. */
}

static void scene_init(Scene *s, int grid_N)
{
    memset(s, 0, sizeof *s);
    s->controls.paused         = false;
    s->controls.ops_per_tick   = OPS_PER_TICK_DEF;
    s->controls.sim_fps        = SIM_FPS_DEFAULT;
    s->controls.filter_band    = FILTER_ALL;
    s->controls.current_preset = 0;
    s->controls.current_theme  = 0;
    s->grid_N = grid_N;
    scene_reset(s);
}

/*
 * The one place the terrain advances each tick: skip if paused, otherwise
 * fill in a batch of cells (how many is ops_per_tick), and switch to HOLD
 * once the map is finished. It takes the whole Scene because it ties the
 * pieces together; the functions it calls each take just their own piece.
 */
static void scene_tick(Scene *s)
{
    if (s->controls.paused) return;

    switch (s->state) {

    case SCENE_COMPUTING:
        for (int i = 0; i < s->controls.ops_per_tick; i++) {
            if (!subdiv_step(&s->subdiv, &s->grid)) {
                s->state = SCENE_HOLD;   /* done — hold the finished map */
                break;
            }
        }
        break;

    case SCENE_HOLD:
        break;   /* finished terrain stays put until you ask for a new one */
    }
}

/* ── §5  EFFECTS — purely cosmetic state ── */
/*
 * Empty for now. There used to be glow flashes here. If a cosmetic-only layer
 * comes back, it belongs here: advanced inside scene_tick after the terrain
 * step, and only ever read by the renderer — never fed back into the maths.
 */

/* ── §6  RENDER — turn the state into something on screen ── */

/*
 * Load one preset's six terrain colours as the live colour scheme. This is
 * what t/T cycle; changing presets with n/p leaves the colours alone. It's
 * safe to call any time — ncurses updates the colours in place, so what's
 * already on screen just repaints in the new scheme.
 *
 * On a plain 8-colour terminal every scheme collapses to the same basic
 * fallback, since the rich 256-colour schemes can't be shown with 8 colours.
 */
static void palette_apply(int idx)
{
    if (idx < 0 || idx >= N_PRESETS) idx = 0;
    if (COLORS >= 256) {
        const Preset *t = &presets[idx];
        init_pair(PAIR_WATER,    t->colors[0], -1);
        init_pair(PAIR_BEACH,    t->colors[1], -1);
        init_pair(PAIR_GRASS,    t->colors[2], -1);
        init_pair(PAIR_HILL,     t->colors[3], -1);
        init_pair(PAIR_MOUNTAIN, t->colors[4], -1);
        init_pair(PAIR_SNOW,     t->colors[5], -1);
    } else {
        init_pair(PAIR_WATER,    COLOR_BLUE,    -1);
        init_pair(PAIR_BEACH,    COLOR_YELLOW,  -1);
        init_pair(PAIR_GRASS,    COLOR_GREEN,   -1);
        init_pair(PAIR_HILL,     COLOR_GREEN,   -1);
        init_pair(PAIR_MOUNTAIN, COLOR_WHITE,   -1);
        init_pair(PAIR_SNOW,     COLOR_WHITE,   -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,        51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
    }
    palette_apply(0);   /* start on the CLASSIC scheme */
}

/* Draw one map cell as a few side-by-side copies of its character (terminal
 * characters are tall and narrow, so widening keeps the map looking square).
 * Anything past the screen edge is skipped. */
static void draw_cell(int sy, int sx, int cols, Tile t)
{
    attron(COLOR_PAIR(t.pair) | t.attr);
    for (int c = 0; c < CELL_COLS; c++) {
        int xx = sx + c;
        if (xx >= 0 && xx < cols)
            mvaddch(sy, xx, (chtype)(unsigned char)t.glyph);
    }
    attroff(COLOR_PAIR(t.pair) | t.attr);
}

/*
 * Draw the whole terrain. For each filled-in cell: rescale its height to
 * 0..1, tilt it by the preset's bias, and (if a filter is on) skip it unless
 * it's the chosen terrain type, then draw it.
 */
static void grid_draw(const HeightField *g, int filter_band, float bias,
                      int cols, int rows)
{
    /* Centre the map block in the rows between the top and bottom HUD. */
    int map_rows = rows - (HUD_TOP_ROWS + HUD_BOTTOM_ROWS);
    int gx0 = (cols - g->w * CELL_COLS) / 2;
    int gy0 = (map_rows - g->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;

    HeightRange range = computed_height_range(g);

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x * CELL_COLS;
            if (sx >= cols) break;             /* rest of row is off-screen */

            int idx = grid_idx(g, x, y);
            if (!g->computed[idx]) continue;   /* not filled in yet — leave blank */

            float v = normalize01(g->height[idx], range.min, range.span);
            if (bias != 1.0f) v = powf(v, bias);   /* tilt toward low or high ground */

            /* with a filter on, draw only the chosen terrain type */
            if (filter_band != FILTER_ALL && band_index_for_norm(v) != filter_band)
                continue;

            draw_cell(sy, sx, cols, band_for_height(v));
        }
    }
}

/*
 * Print one piece of HUD text, trimmed so it can't run off the right edge —
 * if it did, ncurses would wrap it onto the next line and mangle the row
 * below. Returns where it stopped, so pieces can be chained left to right.
 */
static int hud_print(int row, int x, int max_x, int pair, int attr,
                     const char *str)
{
    if (x >= max_x) return x;
    int avail = max_x - x;
    int len   = (int)strlen(str);
    int n     = (len < avail) ? len : avail;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", n, str);
    attroff(COLOR_PAIR(pair) | attr);
    return x + n;
}

/* Top HUD line (bold): title, what it's doing, the current colour scheme and
 * terrain preset, and the frame rate. */
static void hud_draw_status(const Screen *sc, const Scene *s, double fps)
{
    const Controls *c = &s->controls;
    const char *state_str =
        c->paused                     ? "PAUSED"    :
        (s->state == SCENE_COMPUTING) ? "COMPUTING" :
                                        "HOLD";
    char seg[128];
    snprintf(seg, sizeof seg,
             " DIAMOND-SQUARE  %-9s  theme:%-8s  preset:%-8s  %.1f fps ",
             state_str, presets[c->current_theme].name,
             presets[c->current_preset].name, fps);
    hud_print(0, 0, sc->cols, PAIR_HUD, A_BOLD, seg);
}

/* Second HUD line (not bold): the build's progress (level, spacing, which
 * pass, current randomness), the speed settings, and the active filter shown
 * in its own terrain colour. */
static void hud_draw_detail(const Screen *sc, const Scene *s)
{
    const Subdivision *sd = &s->subdiv;
    const Controls    *c  = &s->controls;
    const char *phase_str =
        c->paused                 ? "(paused)" :
        (s->state == SCENE_HOLD)  ? "(done)"   :
        (sd->phase == OP_DIAMOND) ? "DIAMOND"  :
                                    "SQUARE";
    int level, total_levels;
    subdivision_progress(&s->grid, sd, &level, &total_levels);

    char row1[128];
    snprintf(row1, sizeof row1,
             " level %d/%d  step:%-3d  %-8s  rough:%.3f  ops:%-3d  hz:%-3d  show:",
             level, total_levels, sd->level_step, phase_str,
             sd->level_rough, c->ops_per_tick, c->sim_fps);
    int rx = hud_print(1, 0, sc->cols, PAIR_HUD, A_NORMAL, row1);
    hud_print(1, rx, sc->cols, band_color_pair(c->filter_band), A_NORMAL,
              band_name(c->filter_band));
}

/* Bottom HUD line: the list of every key you can press. */
static void hud_draw_actions(const Screen *sc)
{
    hud_print(sc->rows - 1, 0, sc->cols, PAIR_HINT, A_BOLD,
              " w/b/g/h/m/s a:all  t/T:theme  n/p:preset  +/-:ops  [/]:rate  spc:pause  r:reset  q:quit ");
}

/* Draw one whole frame: clear, paint the map, then the three HUD lines. */
static void screen_draw(const Screen *sc, const Scene *s, double fps)
{
    const Controls *c = &s->controls;

    erase();
    grid_draw(&s->grid, c->filter_band, presets[c->current_preset].bias,
              sc->cols, sc->rows);
    hud_draw_status (sc, s, fps);
    hud_draw_detail (sc, s);
    hud_draw_actions(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* Start up, tear down, and resize the terminal. */
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

/* ── §7  APP — input, signals, and the main loop ── */

/*
 * App — the whole program in one box: the demo, the terminal, and two flags
 * the loop watches. There's exactly one of these (g_app), as a global,
 * because signal handlers get no argument of their own and can only reach
 * these flags through a global.
 *
 *   scene       : the whole running demo
 *   screen      : the terminal's size
 *   running     : the loop runs while this is set; a Ctrl-C clears it
 *   need_resize : the resize signal sets this; the loop handles it next time
 *
 * The two flags are volatile sig_atomic_t because a signal handler writes
 * them while the main loop reads them — that's the one type C promises is
 * safe to share that way, and volatile stops the compiler from caching a
 * stale copy. The handlers only set a flag; the real work happens back in
 * the main loop where it's safe.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->scene.grid_N = pick_grid_N(app->screen.cols, app->screen.rows);
    scene_reset(&app->scene);
    app->need_resize = 0;
}

/* Turn one keypress into a settings change or a reset. Returns false only on
 * quit. It never builds terrain itself — that stays in scene_tick. */
static bool app_handle_key(App *app, int ch)
{
    Scene    *s = &app->scene;
    Controls *c = &s->controls;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     c->paused = !c->paused; break;
    case 'r': case 'R':
        scene_reset(s);
        break;

    /* Show just one terrain type, and rebuild so you watch only that type
     * appear. 'a' goes back to showing everything. */
    case 'w': case 'W': c->filter_band = FILTER_WATER;    scene_reset(s); break;
    case 'b': case 'B': c->filter_band = FILTER_BEACH;    scene_reset(s); break;
    case 'g': case 'G': c->filter_band = FILTER_GRASS;    scene_reset(s); break;
    case 'h': case 'H': c->filter_band = FILTER_HILLS;    scene_reset(s); break;
    case 'm': case 'M': c->filter_band = FILTER_MOUNTAIN; scene_reset(s); break;
    case 's': case 'S': c->filter_band = FILTER_SNOW;     scene_reset(s); break;
    case 'a': case 'A': c->filter_band = FILTER_ALL;      scene_reset(s); break;

    /* Recolour: t next scheme, T previous. Only the colours change — the
     * terrain stays, so you recolour the land you're already looking at. */
    case 't':
        c->current_theme = (c->current_theme + 1) % N_PRESETS;
        palette_apply(c->current_theme);
        break;
    case 'T':
        c->current_theme = (c->current_theme + N_PRESETS - 1) % N_PRESETS;
        palette_apply(c->current_theme);
        break;

    /* New terrain shape: n next preset, p previous. Only the landscape
     * changes and rebuilds; the colours stay (recolour with t/T). */
    case 'n':
        c->current_preset = (c->current_preset + 1) % N_PRESETS;
        scene_reset(s);
        break;
    case 'p':
        c->current_preset = (c->current_preset + N_PRESETS - 1) % N_PRESETS;
        scene_reset(s);
        break;

    case '=': case '+':
        if (c->ops_per_tick < OPS_PER_TICK_MAX) c->ops_per_tick *= 2;
        if (c->ops_per_tick > OPS_PER_TICK_MAX) c->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        c->ops_per_tick /= 2;
        if (c->ops_per_tick < OPS_PER_TICK_MIN) c->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case ']':
        c->sim_fps += SIM_FPS_STEP;
        if (c->sim_fps > SIM_FPS_MAX) c->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        c->sim_fps -= SIM_FPS_STEP;
        if (c->sim_fps < SIM_FPS_MIN) c->sim_fps = SIM_FPS_MIN;
        break;
    default: break;
    }
    return true;
}

/* Run however many ticks this frame is owed. Banking the leftover time keeps
 * the terrain building at a steady pace no matter the frame rate. */
static void run_fixed_ticks(Scene *s, int64_t *accum, int64_t tick_ns, int64_t dt)
{
    *accum += dt;
    while (*accum >= tick_ns) {
        scene_tick(s);
        *accum -= tick_ns;
    }
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

    screen_init(&app->screen);
    scene_init(&app->scene, pick_grid_N(app->screen.cols, app->screen.rows));

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

        int64_t dt = frame_delta_ns(&frame_time);

        /* build a bit more terrain */
        run_fixed_ticks(&app->scene, &sim_accum,
                        TICK_NS(app->scene.controls.sim_fps), dt);

        /* work out the fps shown in the HUD, averaged over a short window */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        pace_frame(frame_time, dt);      /* wait so we hold a steady frame rate */

        /* draw, then read a keypress */
        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
