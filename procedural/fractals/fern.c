/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fern.c  —  Fern IFS fractal, animated point-by-point, four variants
 *
 * Four affine transforms (an Iterated Function System) are applied
 * repeatedly to a single point.  The orbit of the point traces out a
 * fern attractor.  Points are plotted a few hundred per tick so you can
 * watch the fern emerge gradually from what looks like random scattered
 * dots — first the stem appears, then branches, then fine frond detail.
 *
 * Four variants cycle with n / p — each is just a different table of four
 * transforms (a different fern species):
 *   Barnsley          classic upright fern
 *   Thelypteris       narrow, delicate
 *   Leptosporangiate  fuller, bushier
 *   Windswept         leaning hard to one side
 * The view bounds are auto-fitted to each variant's attractor (no per-fern
 * tuning), so any coefficient table fills the screen.
 *
 * After TOTAL_ITERS iterations the completed fern is held briefly,
 * then the screen clears and it grows again.
 *
 * Color is assigned by height (y value), via a cycle-able theme (t / T):
 *   roots (low y)   →  theme's first colour
 *   tips  (high y)  →  theme's last colour (bright, bold)
 *
 * Keys:
 *   q / ESC   quit
 *   n / p     next / previous fern variant
 *   t / T     cycle colour theme (Forest/Aurora/Sunset/Galaxy/Tropical/Neon/Fire/Ice)
 *   r         reset fern immediately
 *   ] [       faster / slower simulation
 *   spc       pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fern.c -o fern -lncurses -lm
 *
 * Reading order — built bottom-up; each section uses only earlier ones.
 *   §1  config   — constants, palette slots
 *   §2  clock    — monotonic time
 *   §3  plane    — Vec2 (a point in the fern's plane) + Bounds (its extent)
 *   §4  ifs      — Map + FernVariant table + chaos-game step + auto-fit
 *   §5  canvas   — Cell + Grid (cell buffer, plane→cell mapping, colour bands)
 *   §6  color    — Theme table + apply
 *   §7  scene    — Scene: orbit + growth + variant/theme = the live picture
 *   §8  screen   — ncurses lifecycle + HUD
 *   §9  app      — signals, input, main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : IFS "chaos game" — single-point iteration.  Repeatedly pick
 *                  one of four affine maps (weighted by probability) and apply
 *                  it to a moving point; the points it visits fill in the fern.
 *                  Plotting each point as it is generated lets the attractor
 *                  emerge from noise instead of appearing all at once.
 *
 * Math           : Combined contractivity ≤ 0.85.  Collage theorem: the IFS
 *                  attractor is the unique compact K = T₁(K)∪T₂(K)∪T₃(K)∪T₄(K).
 *                  Probabilities p_i ∝ |det(A_i)| so point density matches the
 *                  area each map covers (the near-flat stem map gets ~1-2%).
 *
 * Data model     : A few small structs, one idea each —
 *                    Vec2          a point in the plane (the orbit, the maps' I/O)
 *                    Map / FernVariant  one affine transform, and a named set of 4
 *                    Bounds        the attractor's extent, auto-fitted per variant
 *                    Grid          the cell buffer + plane→cell mapping
 *                    Growth        the grow→hold→regrow animation state
 *                    Scene         all of the above = the live picture
 *
 * Rendering      : Colour by height maps to botanical structure (dark roots,
 *                  bright tips).  The fitted view is scaled and centred to fill
 *                  the terminal, with x stretched by ASPECT_R for cell shape.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 * Concepts & maths
 *   [1] Barnsley, M. F. (1993). "Fractals Everywhere" (2nd ed.). Academic
 *       Press.  — the source of the chaos game, the collage theorem, and this
 *       very fern (incl. the coefficient table in §4).  Start here.
 *   [2] Hutchinson, J. E. (1981). "Fractals and Self-Similarity." Indiana Univ.
 *       Math. J. 30(5):713-747.  — the maths underneath: an IFS has a unique
 *       compact attractor (the Hutchinson operator's fixed set).
 *   [3] Barnsley, M. F. & Demko, S. (1985). "Iterated Function Systems and the
 *       Global Construction of Fractals." Proc. R. Soc. Lond. A 399:243-275.
 *       — the founding IFS paper.
 *   [4] Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *       — self-similarity in nature; why an IFS draws a plant.
 *
 * Rendering & the chaos game
 *   [5] Demko, S., Hodges, L. & Naylor, B. (1985). "Construction of Fractal
 *       Objects with Iterated Function Systems." SIGGRAPH '85, Computer
 *       Graphics 19(3):271-278.  — drawing an IFS by the chaos game, and the
 *       probability ∝ |det| weighting used by Map.prob_cum in §4.
 *   [6] Peitgen, Jürgens & Saupe (1992). "Chaos and Fractals: New Frontiers of
 *       Science." Springer.  — the most readable IFS / chaos-game chapter, with
 *       the fern worked end to end.
 *
 * In-repo study companions
 *   [7] barnsley.c — the SAME fern grown into a DENSITY GRID instead of
 *       plotted point-by-point; compare the two reveal styles.
 *   [8] buddhabrot.c — the palette-table pattern §6's themes borrow.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  30,
    SIM_FPS_MAX      =  60,
    SIM_FPS_STEP     =   5,

    N_PER_TICK       = 400,     /* IFS iterations computed per tick       */
    TOTAL_ITERS      = 80000,   /* iterations before reset                */
    DONE_PAUSE_TICKS =  90,     /* ticks to hold completed fern (~3 s)    */
    N_FERN_COLORS    =   5,     /* color bands from stem to tip           */
    N_VARIANTS       =   4,     /* fern species to cycle with n/p         */
    N_THEMES         =   8,     /* colour themes to cycle with t/T        */

    FIT_WARMUP       =   50,    /* orbit steps to settle onto the attractor */
    FIT_SAMPLES      = 20000,   /* orbit steps sampled to size the view     */

    GRID_ROWS_MAX    =  80,
    GRID_COLS_MAX    = 300,

    RENDER_FPS       =  60,     /* display refresh cap (≠ reveal speed)   */
    MAX_FRAME_MS     = 100,     /* clamp on one frame's dt (anti spiral)  */
    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,     /* recompute the fps readout this often   */

    MARGIN_TOP       =   2,     /* top rows reserved for HUD data         */
    MARGIN_BOTTOM    =   2,     /* bottom rows: base row + action bar     */
    MARGIN_SIDE      =   1,     /* left/right padding                     */
};

/*
 * ASPECT_R — terminal cell height / width ≈ 2.
 * Used to keep the fern at its natural proportions in the terminal.
 */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * ColorID — the colour-pair "slots" used throughout the program.
 *
 * WHY a numbered enum: each value does double duty.  It is (a) the id ncurses
 * draws with (init_pair / COLOR_PAIR), and (b) the single byte each Grid cell
 * stores to remember its colour.  So one small number answers "what colour is
 * this cell?" with no extra lookup table.
 *
 * WHY this order: the fern is coloured by HEIGHT (ref [1]) — band 1 is the
 * roots, band 5 the tips.  Climbing the slot number climbs the plant; a Theme
 * (§6) decides the actual colour each band shows.
 *
 * Value 0 is reserved: a Grid cell holding 0 means "empty / nothing plotted".
 */
typedef enum {
    COL_FERN_1 = 1,   /* band 1 — roots / stem               */
    COL_FERN_2 = 2,   /* band 2 — lower branches             */
    COL_FERN_3 = 3,   /* band 3 — mid fronds                 */
    COL_FERN_4 = 4,   /* band 4 — upper fronds               */
    COL_FERN_5 = 5,   /* band 5 — growing tips (drawn bold)  */
    COL_HUD    = 6,   /* HUD top data lines    — yellow      */
    COL_HINT   = 7,   /* HUD bottom action bar — cyan        */
} ColorID;

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

/*
 * clamp_frame_dt — cap a single frame's elapsed time.  If the program stalls
 * (debugger, laptop sleep), a huge dt would make the fixed-timestep loop run a
 * burst of catch-up ticks at once — the "spiral of death".  Capping trades one
 * slow frame for stability.
 */
static int64_t clamp_frame_dt(int64_t dt)
{
    int64_t cap = MAX_FRAME_MS * NS_PER_MS;
    return dt > cap ? cap : dt;
}

/*
 * frame_pace — sleep so the whole frame lasts about 1 / target_fps.
 * `frame_start` is when the frame began; `work_done` is time already spent, so
 * we sleep only the leftover budget instead of busy-waiting.
 */
static void frame_pace(int target_fps, int64_t frame_start, int64_t work_done)
{
    int64_t budget = TICK_NS(target_fps);
    int64_t spent  = clock_ns() - frame_start + work_done;
    clock_sleep_ns(budget - spent);
}

/*
 * FpsCounter — a small running readout of frames-per-second for the HUD.  Sums
 * frames and elapsed time, then every FPS_UPDATE_MS divides the two for a smooth
 * value (raw per-frame fps jitters too much to read).
 */
typedef struct {
    int64_t accum_ns;   /* real time since the last readout update */
    int     frames;     /* frames counted since then               */
    double  value;      /* last computed fps — what the HUD shows  */
} FpsCounter;

static void fps_count_frame(FpsCounter *f, int64_t dt)
{
    f->frames   += 1;
    f->accum_ns += dt;
    if (f->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {       /* time to refresh? */
        f->value    = (double)f->frames / ((double)f->accum_ns / (double)NS_PER_SEC);
        f->frames   = 0;
        f->accum_ns = 0;
    }
}

/* ===================================================================== */
/* §3  plane — points and extents in the fern's coordinate space          */
/* ===================================================================== */

/*
 * Vec2 — a single point (x, y) in the fern's continuous coordinate plane.
 *
 * WHY it matters: the whole "chaos game" (ref [1]) happens in this plane.  The
 * orbit is a Vec2 that hops around; each affine map takes a Vec2 and returns a
 * new one; and the fern is just the cloud of Vec2 the orbit eventually fills in.
 *
 * WHY its own type (not loose x/y floats): naming it lets the maths read
 * plainly — `orbit = chaos_step(map, orbit)` — and lets one value flow through
 * map_apply / grid_plot instead of juggling two floats everywhere.
 *
 * WHY float: coordinates are small (≈ -3..3 wide, 0..10 tall), so single
 * precision is plenty; double would only cost speed in the millions-of-steps
 * loop.
 */
typedef struct {
    float x;   /* horizontal position — grows rightward */
    float y;   /* height              — grows upward    */
} Vec2;

/*
 * Bounds — an axis-aligned rectangle in the plane, given by its low (bottom-
 * left) and high (top-right) corners.  Plainly: the smallest box that holds the
 * whole fern.
 *
 * WHY we need it: each variant is a different size and sits in a different part
 * of the plane, but the terminal is fixed.  Measuring the box lets the canvas
 * scale the fern to fill the screen and the colour map span its full height —
 * automatically, for any variant (see variant_fit).
 *
 * Built by sampling: start the box at one point (bounds_at), then stretch it to
 * swallow each new point (bounds_cover) — the standard running min/max.
 */
typedef struct {
    Vec2 lo;   /* smallest x and y the orbit reached (bottom-left) */
    Vec2 hi;   /* largest  x and y the orbit reached (top-right)   */
} Bounds;

static Bounds bounds_at(Vec2 p) { return (Bounds){ p, p }; }

/* Grow the box to include p (the running min/max used while sampling). */
static void bounds_cover(Bounds *b, Vec2 p)
{
    if (p.x < b->lo.x) b->lo.x = p.x;
    if (p.x > b->hi.x) b->hi.x = p.x;
    if (p.y < b->lo.y) b->lo.y = p.y;
    if (p.y > b->hi.y) b->hi.y = p.y;
}

static float bounds_width (const Bounds *b) { return b->hi.x - b->lo.x; }
static float bounds_height(const Bounds *b) { return b->hi.y - b->lo.y; }

/* ===================================================================== */
/* §4  ifs — the maps, the variants, and the chaos game                   */
/* ===================================================================== */

/*
 * Map — one affine transform of the IFS (refs [1][3]):
 *     p → ( a·x + b·y + e ,  c·x + d·y + f )
 *
 * WHAT it is, plainly: a recipe that scales, rotates, and shifts a point.
 * (a,b,c,d) is the 2×2 matrix that scales/rotates; (e,f) is the shift applied
 * after.  The fern's four maps each draw one part — the stem, the big spiral of
 * leaflets, and the two side fronds.
 *
 * prob_cum — the cumulative probability × 100, so a single rand()%100 picks a
 * map (take the first whose threshold the roll falls under).  Probabilities are
 * set ∝ |det| (ref [5]) so point density matches each map's area; the near-flat
 * stem map covers almost no area, so it gets only ~1-2%.
 */
typedef struct {
    float a, b, c, d;   /* 2×2 scale / rotate matrix          */
    float e, f;         /* translation added afterwards       */
    int   prob_cum;     /* cumulative pick threshold (0..100) */
} Map;

/* Apply one affine map to a point. */
static Vec2 map_apply(const Map *m, Vec2 p)
{
    return (Vec2){ m->a * p.x + m->b * p.y + m->e,
                   m->c * p.x + m->d * p.y + m->f };
}

/*
 * FernVariant — a named fern = four affine Maps.  Iterating them as a chaos
 * game (ref [1]) converges on that species' attractor.
 *
 * WHY this is the whole "species" system: swapping this 4-row table is the ONLY
 * thing that differs between ferns.  The chaos-game engine, the auto-fit, and
 * the renderer are all shared — so adding a fern is just a new table entry.
 * (Coefficients verified to render distinct ferns; bounds are auto-fitted.)
 */
typedef struct {
    const char *name;     /* shown in the HUD, e.g. "Barnsley" */
    Map         map[4];   /* the four transforms that build it */
} FernVariant;

static const FernVariant k_variants[N_VARIANTS] = {
    /* classic Barnsley — upright fern */
    { "Barnsley", {
        {  0.00f,  0.00f,  0.00f,  0.16f,  0.00f, 0.00f,   1 }, /* stem      1% */
        {  0.85f,  0.04f, -0.04f,  0.85f,  0.00f, 1.60f,  86 }, /* leaflets 85% */
        {  0.20f, -0.26f,  0.23f,  0.22f,  0.00f, 1.60f,  93 }, /* left      7% */
        { -0.15f,  0.28f,  0.26f,  0.24f,  0.00f, 0.44f, 100 }, /* right     7% */
    }},
    /* Thelypteridaceae — narrow, delicate */
    { "Thelypteris", {
        {  0.000f,  0.000f,  0.000f,  0.25f,  0.000f, -0.40f,   2 },
        {  0.950f,  0.005f, -0.005f,  0.93f, -0.002f,  0.50f,  86 },
        {  0.035f, -0.200f,  0.160f,  0.04f, -0.090f,  0.02f,  93 },
        { -0.040f,  0.200f,  0.160f,  0.04f,  0.083f,  0.12f, 100 },
    }},
    /* Leptosporangiate — fuller, bushier */
    { "Leptosporangiate", {
        {  0.00f,  0.00f,  0.00f,  0.25f,  0.00f, -0.14f,   2 },
        {  0.85f,  0.02f, -0.02f,  0.83f,  0.00f,  1.00f,  86 },
        {  0.09f, -0.28f,  0.30f,  0.11f,  0.00f,  0.60f,  93 },
        { -0.09f,  0.28f,  0.30f,  0.09f,  0.00f,  0.70f, 100 },
    }},
    /* Windswept — classic with a heavier rotation in the leaflet map */
    { "Windswept", {
        {  0.00f,  0.00f,  0.00f,  0.16f,  0.00f, 0.00f,   1 },
        {  0.85f,  0.12f, -0.12f,  0.84f,  0.00f, 1.60f,  86 },
        {  0.20f, -0.26f,  0.23f,  0.22f,  0.00f, 1.60f,  93 },
        { -0.15f,  0.28f,  0.26f,  0.24f,  0.00f, 0.44f, 100 },
    }},
};

/*
 * pick_map_index — roulette selection: given a roll in [0,100), return the
 * index of the first map whose cumulative threshold the roll falls under.  This
 * is what makes some maps fire more often (probability ∝ |det|).
 */
static int pick_map_index(const Map m[4], int roll)
{
    return (roll < m[0].prob_cum) ? 0
         : (roll < m[1].prob_cum) ? 1
         : (roll < m[2].prob_cum) ? 2 : 3;
}

/*
 * chaos_step — one move of the chaos game: pick a map at random (weighted by
 * prob_cum) and apply it.  Iterating this from any start point converges on the
 * variant's attractor — the fern.
 */
static Vec2 chaos_step(const Map m[4], Vec2 p)
{
    int which = pick_map_index(m, rand() % 100);
    return map_apply(&m[which], p);
}

/*
 * variant_fit — size a variant's view box by sweeping its attractor: discard a
 * warm-up so the orbit settles, then cover a sample of points.  Replaces
 * hand-tuned bounds — any coefficient table just works.
 */
static Bounds variant_fit(const Map m[4])
{
    Vec2 p = { 0.0f, 0.0f };
    for (int i = 0; i < FIT_WARMUP; i++)
        p = chaos_step(m, p);

    Bounds b = bounds_at(p);
    for (int i = 0; i < FIT_SAMPLES; i++) {
        p = chaos_step(m, p);
        bounds_cover(&b, p);
    }
    return b;
}

/* ===================================================================== */
/* §5  canvas — the cell buffer and the plane → cell mapping              */
/* ===================================================================== */

/*
 * Cell — an integer terminal cell (column, row): the discrete screen position a
 * continuous plane point maps to.  It's the bridge between the float world
 * (Vec2) and the character grid — plane_to_cell produces one, grid_plot uses it.
 */
typedef struct {
    int col;   /* terminal column (x), 0 = left */
    int row;   /* terminal row    (y), 0 = top  */
} Cell;

/*
 * Grid — the picture in memory: one ColorID per terminal cell, plus the
 * auto-fitted lens that maps plane points onto those cells.
 *
 * WHY a buffer: it separates "decide a point's colour" (the fractal maths) from
 * "draw the screen" (ncurses).  The chaos game only ever writes a band number
 * into cells[][]; grid_draw is the one place that turns those into glyphs.
 *
 * WHY fixed-size arrays: this project never calls malloc in the running loop —
 * the buffer is sized to the biggest terminal we support (GRID_*_MAX); rows/cols
 * say how much of it is actually in use.
 *
 * The lens: scale_x = scale_y × ASPECT_R undoes the tall-cell squash so the
 * fern keeps true proportions.
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];  /* [row][col] → ColorID (0 = empty) */
    int     rows, cols;     /* portion of the buffer in use                       */
    Bounds  view;           /* plane box currently shown (fitted per variant)     */
    float   scale_y;        /* plane y-unit → terminal rows                       */
    float   scale_x;        /* plane x-unit → terminal cols (= scale_y × ASPECT_R)*/
} Grid;

/*
 * height_band — map a point's height onto a colour band 1..N_FERN_COLORS over
 * the fitted y-range, so roots take the theme's first colour and tips its last.
 */
static uint8_t height_band(const Grid *g, float y)
{
    /* how far up the fern this point is: 0 at the roots, 1 at the tips */
    float frac = (y - g->view.lo.y) / (g->view.hi.y - g->view.lo.y);
    int band = 1 + (int)(frac * (float)N_FERN_COLORS);
    if (band < 1) band = 1;
    if (band > N_FERN_COLORS) band = N_FERN_COLORS;
    return (uint8_t)band;
}

/*
 * plane_to_cell — where does plane point p land on the terminal?
 * x is centred on the view's midpoint; y is flipped so the fitted bottom sits
 * on the second-from-bottom row (leaving the last row for the action bar) and
 * the top climbs upward.
 */
static Cell plane_to_cell(const Grid *g, Vec2 p)
{
    float x_center  = (g->view.lo.x + g->view.hi.x) * 0.5f;  /* fern's midline */
    int   base_row  = g->rows - MARGIN_BOTTOM;               /* roots sit here */
    return (Cell){
        .col = g->cols / 2 + (int)roundf((p.x - x_center)     * g->scale_x),
        .row = base_row    - (int)roundf((p.y - g->view.lo.y) * g->scale_y),
    };
}

/*
 * grid_init — point the grid at a fitted view and pick the largest uniform
 * scale that fits the drawable area (rows 0-1 = data, last row = actions),
 * with x stretched by ASPECT_R so the shape isn't squashed by tall cells.
 */
static void grid_init(Grid *g, int cols, int rows, Bounds view)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    memset(g->cells, 0, sizeof g->cells);
    g->rows = rows;
    g->cols = cols;
    g->view = view;

    /* 1. drawable area = the screen minus the HUD margins */
    int draw_rows = rows - (MARGIN_TOP + MARGIN_BOTTOM); if (draw_rows < 1) draw_rows = 1;
    int draw_cols = cols - 2 * MARGIN_SIDE;              if (draw_cols < 1) draw_cols = 1;

    float w = bounds_width(&view);  if (w < 1e-3f) w = 1e-3f;
    float h = bounds_height(&view); if (h < 1e-3f) h = 1e-3f;

    /* 2. largest scale that fits both axes; x is stretched by ASPECT_R, so the
     *    width limit is tested against the stretched scale, keeping whichever
     *    axis runs out of room first */
    float fit_by_rows = (float)draw_rows / h;
    float fit_by_cols = (float)draw_cols / w;
    float unit = fit_by_rows;
    if (unit * ASPECT_R > fit_by_cols) unit = fit_by_cols / ASPECT_R;

    /* 3. store the lens (x doubled to undo tall-cell squash) */
    g->scale_y = unit;
    g->scale_x = unit * ASPECT_R;
}

static void grid_plot(Grid *g, Vec2 p)
{
    Cell c = plane_to_cell(g, p);
    if (c.col < 0 || c.col >= g->cols || c.row < 0 || c.row >= g->rows) return;
    g->cells[c.row][c.col] = height_band(g, p.y);
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    for (int row = 0; row < g->rows; row++) {
        for (int col = 0; col < g->cols; col++) {
            uint8_t c = g->cells[row][col];
            if (c == 0) continue;
            attr_t attr = COLOR_PAIR((int)c);
            if (c == N_FERN_COLORS) attr |= A_BOLD;   /* tips shine */
            wattron(w, attr);
            mvwaddch(w, row, col, (chtype)'*');
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §6  color — themes bound to ncurses pairs                              */
/* ===================================================================== */

/*
 * Theme — a named height→colour gradient, band 1 (roots) to band 5 (tips).
 *
 * WHY it exists: the maths only ever produces a band number per cell; a Theme
 * turns those abstract bands into real colours, so the whole fern restyles by
 * swapping one table row — no recompute.  Same pattern as buddhabrot.c's
 * palette table (ref [8]).  fg256[] are xterm-256 codes (modern terminals);
 * fg8[] is the 8-colour ANSI fallback for old ones.
 *
 * WHY every band is bright: a 256-index from the dark bottom of the cube is
 * invisible on black.  All bands sit in the bright half (no value below ~39),
 * so even the roots show; the gradient is carried by HUE, not by fading to
 * dark, and the tip band (drawn bold) shines.
 */
typedef struct {
    const char *name;            /* shown in the HUD, e.g. "Aurora"      */
    int fg256[N_FERN_COLORS];    /* band 1..5 (roots → tips), 256-colour */
    int fg8[N_FERN_COLORS];      /* same five bands, 8-colour fallback   */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*            name        roots ─────────────────── tips     8-colour fallback (same order) */
    { "Forest",   {  40,  76, 118, 154, 190 }, { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW } },
    { "Aurora",   {  51,  50,  47, 226, 231 }, { COLOR_CYAN,    COLOR_CYAN,    COLOR_GREEN,  COLOR_YELLOW, COLOR_WHITE  } },
    { "Sunset",   { 201, 198, 203, 214, 226 }, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW } },
    { "Galaxy",   { 165, 201,  51, 213, 231 }, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN,   COLOR_MAGENTA,COLOR_WHITE  } },
    { "Tropical", {  46,  82, 226, 208, 196 }, { COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW, COLOR_YELLOW, COLOR_RED    } },
    { "Neon",     { 201, 165,  51,  87, 231 }, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE  } },
    { "Fire",     { 196, 202, 208, 214, 226 }, { COLOR_RED,     COLOR_RED,     COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW } },
    { "Ice",      {  39,  45,  51, 117, 231 }, { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE  } },
};

/* Bind the five fern bands to the active theme (HUD/HINT stay as set below). */
static void theme_apply(int theme)
{
    const Theme *t = &k_themes[theme];
    bool truecolor = COLORS >= 256;
    for (int band = 0; band < N_FERN_COLORS; band++)
        init_pair(COL_FERN_1 + band,
                  truecolor ? t->fg256[band] : t->fg8[band], COLOR_BLACK);
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
    theme_apply(0);   /* Forest; Scene.theme tracks the active one thereafter */
}

/* ===================================================================== */
/* §7  scene — orbit + growth + variant/theme = the live picture          */
/* ===================================================================== */

/*
 * Growth — the grow → hold → regrow animation state, kept SEPARATE from the
 * fractal data so pausing, resizing, or restarting never touches the geometry.
 *
 * It's a tiny three-state cycle: while `points` < TOTAL_ITERS the fern is
 * GROWING (more points each tick); when it reaches TOTAL_ITERS it HOLDS,
 * counting `hold_ticks` up to DONE_PAUSE_TICKS; then it regrows from scratch.
 */
typedef struct {
    int  points;       /* orbit points plotted so far (0..TOTAL_ITERS) */
    int  hold_ticks;   /* ticks elapsed since the fern completed       */
    bool paused;       /* true = frozen (user pressed space)           */
} Growth;

/*
 * Scene — the whole "what is on screen" in one place.  Everything the program
 * does reduces to: update a Scene (scene_tick), then draw it (scene_draw).
 *
 * WHY bundle these: the orbit needs the variant's maps to step, the canvas
 * needs the orbit's point to plot, and the growth says when to stop — useless
 * apart.  One pointer carries the whole picture between functions.
 *
 * WHY ids, not pointers: variant / theme are plain indices into the k_variants
 * / k_themes tables — trivial to cycle ( (id+1) % N ), read straight into the
 * HUD, and never dangle.
 */
typedef struct {
    Grid   grid;       /* the painted cells + plane→cell mapping        */
    int    variant;    /* index into k_variants — which fern            */
    int    theme;      /* index into k_themes   — which colour gradient */
    Vec2   orbit;      /* the current chaos-game point                  */
    Growth growth;     /* how far the grow animation has gotten         */
} Scene;

/*
 * scene_init — (re)start the current variant: auto-fit its view, clear the
 * grid, then warm up the orbit before plotting begins.  Reads s->variant and
 * s->theme (both 0 on first call, since the App lives in zero-initialised BSS),
 * so reset and resize keep whatever fern/theme is selected.
 */
static void scene_init(Scene *s, int cols, int rows)
{
    const Map *m = k_variants[s->variant].map;

    grid_init(&s->grid, cols, rows, variant_fit(m));

    s->orbit            = (Vec2){ 0.0f, 0.0f };
    s->growth.points     = 0;
    s->growth.hold_ticks = 0;
    s->growth.paused     = false;

    for (int i = 0; i < FIT_WARMUP; i++)   /* settle onto the attractor */
        s->orbit = chaos_step(m, s->orbit);
}

/* Switch to another variant (wraps around) and regrow it from scratch. */
static void scene_set_variant(Scene *s, int variant, int cols, int rows)
{
    s->variant = ((variant % N_VARIANTS) + N_VARIANTS) % N_VARIANTS;
    scene_init(s, cols, rows);
}

/*
 * scene_plot_batch — take one tick's worth of chaos-game steps (N_PER_TICK) and
 * plot each point.  This is the actual "grow the fern" work.
 */
static void scene_plot_batch(Scene *s)
{
    const Map *m = k_variants[s->variant].map;
    for (int i = 0; i < N_PER_TICK; i++) {
        s->orbit = chaos_step(m, s->orbit);   /* hop to the next attractor point */
        grid_plot(&s->grid, s->orbit);        /* paint where it landed           */
    }
    s->growth.points += N_PER_TICK;
}

/*
 * scene_tick — advance the picture by one tick: grow → hold → regrow.
 * While growing, plot the next batch.  Once complete, count down the hold, then
 * restart from scratch (the fern does not stay forever).
 */
static void scene_tick(Scene *s)
{
    if (s->growth.paused) return;

    if (s->growth.points >= TOTAL_ITERS) {                /* finished — hold, then regrow */
        if (++s->growth.hold_ticks >= DONE_PAUSE_TICKS)
            scene_init(s, s->grid.cols, s->grid.rows);
        return;
    }
    scene_plot_batch(s);                                  /* still growing */
}

static void scene_draw(const Scene *s, WINDOW *w) { grid_draw(&s->grid, w); }

/* ===================================================================== */
/* §8  screen — ncurses lifecycle + HUD                                   */
/* ===================================================================== */

/*
 * Screen — the live terminal size in character cells.  Measured at startup and
 * after every resize (SIGWINCH); the canvas reads it to scale the fern and the
 * HUD reads it to place its lines.
 */
typedef struct {
    int cols;   /* terminal width  in characters */
    int rows;   /* terminal height in characters */
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

static void screen_free(Screen *s)  { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* grow_percent — how complete the current fern is, 0..100. */
static int grow_percent(const Scene *sc)
{
    int pct = sc->growth.points * 100 / TOTAL_ITERS;
    return pct > 100 ? 100 : pct;
}

/* hud_data_line — row 0, right-aligned bold: variant name + fps + run state. */
static void hud_data_line(const Screen *s, const Scene *sc, double fps)
{
    const char *state = sc->growth.paused ? "PAUSED "
                      : (sc->growth.points >= TOTAL_ITERS ? "complete" : "growing ");

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " Fern: %s  %5.1f fps  %s ",
             k_variants[sc->variant].name, fps, state);
    int right_x = s->cols - (int)strlen(buf);
    if (right_x < 0) right_x = 0;
    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, right_x, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

/* hud_params_line — row 1, plain: variant index, theme, % complete, speed. */
static void hud_params_line(const Scene *sc, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " variant %d/%d  theme:%s  %d%%  spd:%d Hz ",
             sc->variant + 1, N_VARIANTS, k_themes[sc->theme].name,
             grow_percent(sc), sim_fps);
    attron(COLOR_PAIR(COL_HUD));
    mvprintw(1, 0, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD));
}

/* hud_action_line — last row, bright cyan: every interactive key. */
static void hud_action_line(const Screen *s)
{
    attron(COLOR_PAIR(COL_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  n/p:variant  t:theme  r:reset  spc:pause  [/]:speed ");
    attroff(COLOR_PAIR(COL_HINT) | A_BOLD);
}

/*
 * screen_draw — compose one frame: the fern, then the HUD.
 * HUD layout is fixed — top is data, bottom is actions:
 *   row 0      variant + fps + state    (hud_data_line)
 *   row 1      variant/theme/%/speed     (hud_params_line)
 *   row rows-1 key actions               (hud_action_line)
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);       /* the fern itself      */
    hud_data_line(s, sc, fps);    /* row 0   — live data  */
    hud_params_line(sc, sim_fps); /* row 1   — parameters */
    hud_action_line(s);           /* last row — actions   */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app — signals, input, main loop                                    */
/* ===================================================================== */

/*
 * App — the top-level container main() owns: the picture (scene), the terminal
 * it lives in (screen), and the one tunable speed (sim_fps).  One struct so the
 * loop passes everything with a single &app, and it sits in BSS — it holds the
 * large cell buffer, which doesn't belong on the stack.
 */
typedef struct {
    Scene  scene;     /* the fern picture + all its sub-state           */
    Screen screen;    /* current terminal dimensions                    */
    int    sim_fps;   /* reveal speed, ticks/sec (SIM_FPS_MIN..MAX); ]/[ */
} App;

/* The only globals: POSIX signal handlers take no arguments, so they must
 * write a flag the main loop polls. */
static volatile sig_atomic_t g_should_quit   = 0;
static volatile sig_atomic_t g_should_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_should_quit   = 1;
    if (s == SIGWINCH)               g_should_resize = 1;
}

static void cleanup(void) { endwin(); }

static void app_resize(App *a)
{
    screen_resize(&a->screen);
    scene_init(&a->scene, a->screen.cols, a->screen.rows);
    g_should_resize = 0;
}

/* app_change_speed — nudge the reveal rate by delta, clamped to its range. */
static void app_change_speed(App *a, int delta)
{
    int fps = a->sim_fps + delta;
    if (fps < SIM_FPS_MIN) fps = SIM_FPS_MIN;
    if (fps > SIM_FPS_MAX) fps = SIM_FPS_MAX;
    a->sim_fps = fps;
}

/* app_handle_key — map one keypress to an action.  Returns false to quit. */
static bool app_handle_key(App *a, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:  return false;                          /* quit    */
    case 'n': case 'N':  scene_set_variant(&a->scene, a->scene.variant + 1,
                                           a->screen.cols, a->screen.rows); break; /* next fern */
    case 'p': case 'P':  scene_set_variant(&a->scene, a->scene.variant - 1,
                                           a->screen.cols, a->screen.rows); break; /* prev fern */
    case 't': case 'T':  a->scene.theme = (a->scene.theme + 1) % N_THEMES;
                         theme_apply(a->scene.theme);                    break; /* palette */
    case 'r': case 'R':  scene_init(&a->scene,
                                    a->screen.cols, a->screen.rows);     break; /* regrow  */
    case ' ':            a->scene.growth.paused = !a->scene.growth.paused; break; /* pause */
    case ']':            app_change_speed(a, +SIM_FPS_STEP);             break; /* faster  */
    case '[':            app_change_speed(a, -SIM_FPS_STEP);             break; /* slower  */
    default: break;
    }
    return true;
}

/*
 * app_run_due_ticks — fixed-timestep catch-up.  Add the elapsed time to an
 * accumulator and spend one scene_tick per whole tick's worth, so the reveal
 * advances at sim_fps regardless of frame rate.
 */
static void app_run_due_ticks(App *a, int64_t *sim_accum, int64_t dt)
{
    int64_t tick_ns = TICK_NS(a->sim_fps);
    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&a->scene);
        *sim_accum -= tick_ns;
    }
}

/* app_poll_input — handle at most one pending key this frame. */
static void app_poll_input(App *a)
{
    int ch = getch();
    if (ch != ERR && !app_handle_key(a, ch))
        g_should_quit = 1;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    /* static (BSS) — Scene holds large arrays; keep them off the stack */
    static App app = { .sim_fps = SIM_FPS_DEFAULT };

    screen_init(&app.screen);
    scene_init(&app.scene, app.screen.cols, app.screen.rows);

    int64_t    frame_time = clock_ns();
    int64_t    sim_accum  = 0;        /* elapsed time not yet spent on ticks */
    FpsCounter fps        = {0};

    /* Main loop — one pass per frame:
     *   1. apply a pending resize
     *   2. measure elapsed time, clamped against stalls
     *   3. run the simulation ticks that time owes
     *   4. update the fps readout
     *   5. pace the frame, then draw it
     *   6. handle one keypress
     */
    while (!g_should_quit) {

        if (g_should_resize) {
            app_resize(&app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = clamp_frame_dt(now - frame_time);
        frame_time  = now;

        app_run_due_ticks(&app, &sim_accum, dt);            /* advance the reveal     */
        fps_count_frame(&fps, dt);                          /* update the fps readout */

        frame_pace(RENDER_FPS, frame_time, dt);             /* cap the display rate   */
        screen_draw(&app.screen, &app.scene, fps.value, app.sim_fps);
        screen_present();

        app_poll_input(&app);                               /* one key → action/quit  */
    }

    screen_free(&app.screen);
    return 0;
}
