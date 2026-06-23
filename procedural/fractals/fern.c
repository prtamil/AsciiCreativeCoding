/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fern.c — draws a fern, one dot at a time, by playing the "chaos game".
 *
 * The trick: keep four little move recipes, and a single point. Each step,
 * pick one recipe at random and use it to nudge the point somewhere new;
 * plot it; repeat. Out of what looks like random scatter, a fern slowly
 * appears — stem first, then branches, then fine detail.
 *
 * Reference: M. Barnsley, "Fractals Everywhere" (1993) — origin of this fern,
 * the chaos game, and the four-recipe numbers in §4.
 * Sister file: barnsley.c grows the SAME fern a different way (counting how
 * often each cell is hit), so compare the two if you're curious.
 */

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

/* Terminal characters are about twice as tall as they are wide. We stretch
 * the fern's width by this much so it doesn't look squashed. */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * ColorID — the numbered colour "slots" the program uses.
 *
 * Each number does two jobs: it's the id ncurses paints with, AND it's the one
 * byte we store per grid cell to remember that cell's colour. So a single small
 * number both answers "what colour is this cell?" and draws it.
 *
 * The fern is coloured by how high up a dot is, so the first five slots run
 * bottom-to-top: 1 = roots, 5 = bright tips. A Theme (§6) decides what actual
 * colour each of those five means.
 *
 * Slot 0 is left out on purpose: a cell holding 0 means "empty, nothing here".
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
 * If the program freezes for a moment (laptop sleep, debugger), the next frame
 * thinks a huge amount of time passed and tries to catch up all at once, which
 * locks up. We cap how much "elapsed time" one frame can report to avoid that.
 */
static int64_t clamp_frame_dt(int64_t dt)
{
    int64_t cap = MAX_FRAME_MS * NS_PER_MS;
    return dt > cap ? cap : dt;
}

/*
 * Sleep just long enough that each frame takes the same amount of time, so the
 * animation runs at a steady speed. We subtract the work already done this frame
 * and sleep only what's left over.
 */
static void frame_pace(int target_fps, int64_t frame_start, int64_t work_done)
{
    int64_t budget = TICK_NS(target_fps);
    int64_t spent  = clock_ns() - frame_start + work_done;
    clock_sleep_ns(budget - spent);
}

/*
 * FpsCounter — works out the frames-per-second number shown on screen.
 *
 * Measuring one frame at a time gives a jumpy number that's hard to read, so we
 * tally up frames over a short window and only divide every FPS_UPDATE_MS to get
 * a steady value.
 */
typedef struct {
    int64_t accum_ns;   /* time piled up since we last recomputed the number */
    int     frames;     /* frames counted in that window                     */
    double  value;      /* the latest fps number — what's shown on screen    */
} FpsCounter;

static void fps_count_frame(FpsCounter *f, int64_t dt)
{
    f->frames   += 1;
    f->accum_ns += dt;
    if (f->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {       /* window full — recompute */
        f->value    = (double)f->frames / ((double)f->accum_ns / (double)NS_PER_SEC);
        f->frames   = 0;
        f->accum_ns = 0;
    }
}

/* ===================================================================== */
/* §3  plane — points and extents in the fern's coordinate space          */
/* ===================================================================== */

/*
 * Vec2 — one point (x, y) in the fern's own coordinate space (not the screen).
 *
 * This is the star of the chaos game: the single dot that hops around is a Vec2,
 * each move recipe takes a Vec2 and hands back a new one, and the finished fern
 * is just the cloud of points the dot has visited. Bundling x and y into one
 * named value keeps the code readable and lets a point flow through the moves
 * and the plotter as a single thing.
 *
 * Coordinates stay small (roughly -3..3 wide, 0..10 tall), so plain float is
 * plenty of precision — and faster, which matters over millions of steps.
 */
typedef struct {
    float x;   /* left-right position, bigger = further right */
    float y;   /* height,             bigger = further up     */
} Vec2;

/*
 * Bounds — the smallest box that holds the whole fern, kept as its bottom-left
 * and top-right corners.
 *
 * Every fern species is a slightly different size and sits in a different spot,
 * but the terminal is one fixed size. Knowing the fern's box lets us shrink or
 * grow it to fill the screen, and lets the colours span exactly its height — all
 * worked out automatically (see variant_fit).
 *
 * We build it by watching the dot wander: start the box at one point
 * (bounds_at), then widen it to swallow each new point (bounds_cover).
 */
typedef struct {
    Vec2 lo;   /* the lowest, leftmost point the dot reached  */
    Vec2 hi;   /* the highest, rightmost point the dot reached */
} Bounds;

static Bounds bounds_at(Vec2 p) { return (Bounds){ p, p }; }

/* Widen the box just enough to include p. */
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
 * Map — one "move recipe": take a point, and shrink/rotate/shift it to a new
 * spot. The fern is built from four of these, each responsible for one part:
 * the stem, the big swirl of leaflets, and the two side fronds.
 *
 * The six numbers a,b,c,d,e,f are the recipe. a,b,c,d shrink and rotate the
 * point; e,f then slide it sideways and up. (The formula is
 * x' = a·x + b·y + e, y' = c·x + d·y + f, the standard affine transform.)
 *
 * prob_cum sets how often this recipe gets picked, written as a running total
 * out of 100. We roll a number 0..99 and take the first recipe whose total the
 * roll lands under. The recipes are weighted by how much area they fill, so the
 * stem — which is almost a flat line — is picked only about 1 in 100 steps.
 */
typedef struct {
    float a, b, c, d;   /* shrink-and-rotate part of the recipe        */
    float e, f;         /* slide-sideways-and-up part                  */
    int   prob_cum;     /* running pick threshold out of 100 (see above) */
} Map;

/* Run one move recipe on a point and return where it ends up. */
static Vec2 map_apply(const Map *m, Vec2 p)
{
    return (Vec2){ m->a * p.x + m->b * p.y + m->e,
                   m->c * p.x + m->d * p.y + m->f };
}

/*
 * FernVariant — a named fern species: just a name plus its four move recipes.
 *
 * This is the whole "different ferns" feature. Every fern shares the same chaos
 * game, the same auto-fitting, and the same drawing code — the ONLY thing that
 * changes between species is these four recipes. So adding a new fern is nothing
 * more than adding a row to the table below.
 */
typedef struct {
    const char *name;     /* shown on screen, e.g. "Barnsley"      */
    Map         map[4];   /* the four recipes that grow this fern  */
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
 * Given a dice roll of 0..99, pick which of the four recipes to use. We take the
 * first one whose running total the roll falls under, which is what makes the
 * common recipes get chosen more often than the rare ones.
 */
static int pick_map_index(const Map m[4], int roll)
{
    return (roll < m[0].prob_cum) ? 0
         : (roll < m[1].prob_cum) ? 1
         : (roll < m[2].prob_cum) ? 2 : 3;
}

/*
 * One turn of the chaos game: roll the dice, pick a recipe, run it. Do this over
 * and over from any starting point and the dots settle into the fern shape.
 */
static Vec2 chaos_step(const Map m[4], Vec2 p)
{
    int which = pick_map_index(m, rand() % 100);
    return map_apply(&m[which], p);
}

/*
 * Measure how big a fern is before drawing it: play the chaos game for a while
 * and note the box the dots stay within. Doing this means we never have to hand-
 * tune sizes — any set of recipes just fits the screen on its own.
 */
static Bounds variant_fit(const Map m[4])
{
    Vec2 p = { 0.0f, 0.0f };
    for (int i = 0; i < FIT_WARMUP; i++)   /* let the dot drift onto the fern first */
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
 * Cell — a whole-number spot on the terminal grid (which column, which row).
 *
 * A fern point lives at smooth floating coordinates; a character on screen lives
 * at a whole row and column. Cell is the rounded-off screen spot a point lands
 * on — the bridge between the two. plane_to_cell makes one; grid_plot uses it.
 */
typedef struct {
    int col;   /* column across the screen, 0 = far left */
    int row;   /* row down the screen,      0 = top      */
} Cell;

/*
 * Grid — the picture held in memory: a colour number for every screen cell, plus
 * the settings that say where each fern point lands on screen.
 *
 * Keeping the picture in an array first, then drawing it, separates the fern math
 * from the screen drawing. The chaos game only ever writes a colour number into
 * cells[][]; grid_draw is the single place that turns those numbers into
 * characters on screen.
 *
 * The array is a fixed maximum size (we never allocate memory mid-run); rows and
 * cols say how much of it the current terminal actually uses. view, scale_y and
 * scale_x together are the "lens" that shrinks the fern to fit — scale_x is wider
 * than scale_y to undo the fact that characters are tall and narrow.
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];  /* colour number per cell, 0 = empty */
    int     rows, cols;     /* how much of the array the screen uses now            */
    Bounds  view;           /* the fern's measured box (set per species)            */
    float   scale_y;        /* how many screen rows per unit of fern height         */
    float   scale_x;        /* how many screen cols per unit of fern width          */
} Grid;

/*
 * Decide which colour a point gets from how high up the fern it sits: low points
 * get the first colour, high points the last.
 */
static uint8_t height_band(const Grid *g, float y)
{
    /* 0 at the bottom of the fern, 1 at the very top */
    float frac = (y - g->view.lo.y) / (g->view.hi.y - g->view.lo.y);
    int band = 1 + (int)(frac * (float)N_FERN_COLORS);
    if (band < 1) band = 1;
    if (band > N_FERN_COLORS) band = N_FERN_COLORS;
    return (uint8_t)band;
}

/*
 * Work out which screen cell a fern point falls on. We line the fern's middle up
 * with the middle of the screen, and flip top-for-bottom because screen rows are
 * numbered from the top down but the fern grows upward. The fern's base sits just
 * above the bottom edge, leaving the last row free for the key list.
 */
static Cell plane_to_cell(const Grid *g, Vec2 p)
{
    float x_center  = (g->view.lo.x + g->view.hi.x) * 0.5f;  /* fern's centre line */
    int   base_row  = g->rows - MARGIN_BOTTOM;               /* where the roots sit */
    return (Cell){
        .col = g->cols / 2 + (int)roundf((p.x - x_center)     * g->scale_x),
        .row = base_row    - (int)roundf((p.y - g->view.lo.y) * g->scale_y),
    };
}

/*
 * Set the grid up for a freshly measured fern: clear it, then pick the biggest
 * size that still fits on screen once the top rows (info) and bottom row (keys)
 * are reserved, stretching the width so the fern doesn't look squashed.
 */
static void grid_init(Grid *g, int cols, int rows, Bounds view)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    memset(g->cells, 0, sizeof g->cells);
    g->rows = rows;
    g->cols = cols;
    g->view = view;

    /* room left for the fern once the info and key rows are set aside */
    int draw_rows = rows - (MARGIN_TOP + MARGIN_BOTTOM); if (draw_rows < 1) draw_rows = 1;
    int draw_cols = cols - 2 * MARGIN_SIDE;              if (draw_cols < 1) draw_cols = 1;

    float w = bounds_width(&view);  if (w < 1e-3f) w = 1e-3f;
    float h = bounds_height(&view); if (h < 1e-3f) h = 1e-3f;

    /* find the biggest size that still fits, picking whichever runs out of room
     * first — height or the stretched-out width */
    float fit_by_rows = (float)draw_rows / h;
    float fit_by_cols = (float)draw_cols / w;
    float unit = fit_by_rows;
    if (unit * ASPECT_R > fit_by_cols) unit = fit_by_cols / ASPECT_R;

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
 * Theme — a named set of five colours, running from the roots up to the tips.
 *
 * The fern math only ever produces a number 1..5 per cell. A Theme is what turns
 * those numbers into actual colours, so switching the whole look is just picking
 * a different row of this table — nothing is recalculated.
 *
 * Two colour lists per theme: fg256 holds the rich colours that modern terminals
 * support; fg8 is a fallback of basic colours for old terminals. Every colour is
 * deliberately on the bright side (dark colours vanish against the black
 * background), so the look comes from the shift in hue, not from fading to dark.
 */
typedef struct {
    const char *name;            /* shown on screen, e.g. "Aurora"           */
    int fg256[N_FERN_COLORS];    /* roots → tips, rich colours (modern terms) */
    int fg8[N_FERN_COLORS];      /* same five, basic colours (old terminals)  */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*            name        roots ─────────────────── tips     basic-colour fallback (same order) */
    { "Forest",   {  40,  76, 118, 154, 190 }, { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW } },
    { "Aurora",   {  51,  50,  47, 226, 231 }, { COLOR_CYAN,    COLOR_CYAN,    COLOR_GREEN,  COLOR_YELLOW, COLOR_WHITE  } },
    { "Sunset",   { 201, 198, 203, 214, 226 }, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW } },
    { "Galaxy",   { 165, 201,  51, 213, 231 }, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN,   COLOR_MAGENTA,COLOR_WHITE  } },
    { "Tropical", {  46,  82, 226, 208, 196 }, { COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW, COLOR_YELLOW, COLOR_RED    } },
    { "Neon",     { 201, 165,  51,  87, 231 }, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE  } },
    { "Fire",     { 196, 202, 208, 214, 226 }, { COLOR_RED,     COLOR_RED,     COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW } },
    { "Ice",      {  39,  45,  51, 117, 231 }, { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE  } },
};

/* Point the five fern colours at the chosen theme (the info-bar colours don't
 * change). */
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
    /* The info bar (yellow) and key list (cyan) keep these colours no matter
     * which theme is on. */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* start on Forest; the scene remembers any later choice */
}

/* ===================================================================== */
/* §7  scene — orbit + growth + variant/theme = the live picture          */
/* ===================================================================== */

/*
 * Growth — tracks the grow → pause → start-over loop the animation runs on.
 *
 * It's a simple cycle: while points is still climbing toward TOTAL_ITERS, the
 * fern is filling in. Once it's full, hold_ticks counts up while the finished
 * fern is held on screen for a moment, then everything clears and it starts
 * over. Kept apart from the fern data so pausing or resizing never disturbs the
 * shape itself.
 */
typedef struct {
    int  points;       /* how many dots plotted so far (0..TOTAL_ITERS) */
    int  hold_ticks;   /* how long the finished fern has been held       */
    bool paused;       /* true while frozen (user pressed space)         */
} Growth;

/*
 * Scene — everything that's on screen, gathered in one place. The whole program
 * boils down to: update the scene (scene_tick), then draw it (scene_draw).
 *
 * These pieces all need each other — the wandering dot needs the recipes, the
 * picture needs the dot, the growth counter says when to stop — so they travel
 * together as one. variant and theme are just row numbers into the species and
 * theme tables, which makes cycling them as easy as adding one and wrapping.
 */
typedef struct {
    Grid   grid;       /* the picture so far, plus its on-screen placement */
    int    variant;    /* which fern species (row in k_variants)           */
    int    theme;      /* which colour theme (row in k_themes)             */
    Vec2   orbit;      /* the dot currently wandering through the fern      */
    Growth growth;     /* how far along the grow animation is              */
} Scene;

/*
 * Start (or restart) the current fern from scratch: measure it, clear the
 * picture, and let the dot wander a bit so it's on the fern before we plot. Keeps
 * whatever species and theme are already chosen, so reset and resize don't lose
 * the user's selection.
 */
static void scene_init(Scene *s, int cols, int rows)
{
    const Map *m = k_variants[s->variant].map;

    grid_init(&s->grid, cols, rows, variant_fit(m));

    s->orbit            = (Vec2){ 0.0f, 0.0f };
    s->growth.points     = 0;
    s->growth.hold_ticks = 0;
    s->growth.paused     = false;

    for (int i = 0; i < FIT_WARMUP; i++)   /* let the dot drift onto the fern */
        s->orbit = chaos_step(m, s->orbit);
}

/* Switch to another fern species (wrapping past the ends) and grow it fresh. */
static void scene_set_variant(Scene *s, int variant, int cols, int rows)
{
    s->variant = ((variant % N_VARIANTS) + N_VARIANTS) % N_VARIANTS;
    scene_init(s, cols, rows);
}

/* Add one tick's worth of dots to the fern — this is the actual growing. */
static void scene_plot_batch(Scene *s)
{
    const Map *m = k_variants[s->variant].map;
    for (int i = 0; i < N_PER_TICK; i++) {
        s->orbit = chaos_step(m, s->orbit);   /* move the dot */
        grid_plot(&s->grid, s->orbit);        /* mark where it landed */
    }
    s->growth.points += N_PER_TICK;
}

/*
 * Move the animation forward by one tick. If the fern isn't done, add more dots.
 * If it is, hold it on screen for a beat, then wipe it and start over.
 */
static void scene_tick(Scene *s)
{
    if (s->growth.paused) return;

    if (s->growth.points >= TOTAL_ITERS) {                /* done — hold, then restart */
        if (++s->growth.hold_ticks >= DONE_PAUSE_TICKS)
            scene_init(s, s->grid.cols, s->grid.rows);
        return;
    }
    scene_plot_batch(s);                                  /* still filling in */
}

static void scene_draw(const Scene *s, WINDOW *w) { grid_draw(&s->grid, w); }

/* ===================================================================== */
/* §8  screen — ncurses lifecycle + HUD                                   */
/* ===================================================================== */

/*
 * Screen — the terminal's current size in characters. Measured at startup and
 * again whenever the window is resized; the fern uses it to scale itself and the
 * info lines use it to know where to print.
 */
typedef struct {
    int cols;   /* width  in characters */
    int rows;   /* height in characters */
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
    /* The endwin/refresh shuffle is how ncurses notices the new window size. */
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* How far along the current fern is, as a percentage. */
static int grow_percent(const Scene *sc)
{
    int pct = sc->growth.points * 100 / TOTAL_ITERS;
    return pct > 100 ? 100 : pct;
}

/* Top-right line: fern name, frame rate, and whether it's growing or paused. */
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
