/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ant_colony.c — ants find the shortest path to food by leaving and
 *                following scent trails, drawn live in your terminal.
 *
 * Each ant wanders out looking for food, dropping a faint scent as it
 * goes.  Once it finds food it heads straight home, laying down a much
 * stronger scent.  The next ants prefer stronger trails, so shorter
 * round-trips get reinforced faster and the colony settles on the best
 * routes — without any ant knowing the map.  No individual is smart; the
 * cleverness lives in the trails.  This is the classic Deneubourg
 * double-bridge experiment (Deneubourg et al. 1990).
 *
 * Press n/N to swap the food layout, t/T to swap the colour theme.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/generational/ant_colony.c \
 *       -o ant_colony -lncurses -lm
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ── */

enum {
    SIM_FPS_MIN      =  20,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 240,
    SIM_FPS_STEP     =  20,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_PH_BASE      =  3,    /* +0..+3: the four trail-strength colours  */
    PAIR_ANT_S        =  7,    /* an ant out searching                     */
    PAIR_ANT_R        =  8,    /* an ant heading home with food            */
    PAIR_FOOD         =  9,
    PAIR_NEST         = 10,
    PAIR_PAPER        = 11,    /* white background for the light theme     */

    /* Biggest grid we ever store, so the buffers can be fixed size.  */
    GRID_W_MAX        = 320,
    GRID_H_MAX        = 100,

    N_ANTS            =  80,
    MAX_FOOD          =  16,    /* most food sources any layout uses        */

    /* HUD: one info row on top, one key-hint row on the bottom.      */
    HUD_TOP_ROWS      =   1,
    HUD_BOT_ROWS      =   1,
    HUD_ROWS          = HUD_TOP_ROWS + HUD_BOT_ROWS,

    SPEED_DEFAULT_IDX =   1,    /* starts at 0.5x — see SPEED_LADDER        */

    SEARCH_ARC        =   3,    /* a searching ant looks at 3 ways ahead    */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))
#define DT_CAP_MS            100      /* ignore frame gaps longer than this  */

/* How the scent trails build up and fade. */
#define EVAP_RATE       0.003f   /* fraction of every trail lost each tick   */
#define DEPOSIT_RETURN  0.40f    /* heavy scent an ant drops carrying food   */
#define DEPOSIT_SEARCH  0.04f    /* faint scent an ant drops while searching */
#define PH_MAX          1.0f     /* cap on a cell's scent (see PheromoneField)*/
#define PH_BASE_NOISE   0.05f    /* a little pull even toward scent-free cells*/
#define PH_EPSILON      0.001f   /* once a trail dips below this, call it 0   */
#define PH_DRAW_MIN     0.04f    /* trails fainter than this aren't drawn     */

#define FOOD_BIAS       3.0f     /* how strongly searchers steer toward food */
#define FOOD_RADIUS     2        /* how close counts as reaching food/nest   */
#define STEER_OFFSET    1.0f     /* keeps even a sideways step in the running
                                  * so an ant never has zero choices         */
#define SPAWN_RADIUS    1        /* ants appear within 1 cell of the nest    */
#define MIN_PLAY_ROWS   8        /* never lay out a grid shorter than this   */

/* Where food gets placed relative to the screen edges. */
#define EDGE_MARGIN_DIV   5      /* food sits about 1/5 in from the edge      */
#define EDGE_MARGIN_MIN_X 4      /* but never closer than this many cells     */
#define EDGE_MARGIN_MIN_Y 3
#define RING_RADIUS_MIN_X 4      /* smallest ring radius for circular layouts */
#define RING_RADIUS_MIN_Y 3
#define QUARTER_TURN  ((float)(M_PI / 2.0))  /* puts a ring point straight up */

/* The light "paper" theme uses near-white for the page and black for ink. */
#define XTERM_WHITE  231
#define XTERM_BLACK   16

/* Speed choices the +/- keys step through.  Below 1.0 is slow motion: at
 * 0.25x the sim advances one tick every four frames (see app_advance). */
static const float SPEED_LADDER[] = {
    0.25f, 0.5f, 1.0f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f, 12.0f, 16.0f,
};
#define N_SPEEDS ((int)(sizeof SPEED_LADDER / sizeof SPEED_LADDER[0]))

/* Trails are drawn in four strength tiers: the glyph for each tier, and
 * the scent level at which a cell jumps up to the next one. */
static const char  PH_GLYPHS[4]  = { '.', ':', '+', '#' };
static const float PH_RAMP_T[3]  = { 0.15f, 0.40f, 0.70f };

#define ANT_GLYPH      'o'
#define FOOD_GLYPH     '*'
#define NEST_GLYPH     '@'
#define FOOD_FRAME_L   '['
#define FOOD_FRAME_R   ']'
#define NEST_FRAME_L   '('
#define NEST_FRAME_R   ')'

/* The food layouts you can cycle through with n/N.  Every layout keeps
 * the nest at the grid centre and just moves the food around, so you can
 * watch the colony re-route its trails to whatever shape it's given. */
typedef enum {
    PAT_DOUBLE  = 0,    /* 2 sources on opposite sides (the classic one)  */
    PAT_SINGLE,         /* 1 source                                       */
    PAT_QUAD,           /* 4 sources, up/down/left/right                  */
    PAT_LINE,           /* 3 in a row on the right edge                   */
    PAT_HEXAGON,        /* 6 evenly around the nest                       */
    PAT_CROSS,          /* 4 close-in, up/down/left/right                 */
    PAT_TRIANGLE,       /* 3 at the corners of a triangle                 */
    PAT_CIRCLE,         /* 8 evenly around the nest                       */
    PAT_DIAGONAL,       /* 4 along the main diagonal                      */
    PAT_CLUSTER,        /* 5 bunched together on one side                 */
    PAT_DISTANT,        /* 2 in far opposite corners                      */
    PAT_PERIMETER,      /* 6 spread along the screen edges                */
    PAT_GRID,           /* 9 in a 3x3 grid                               */
    PAT_RANDOM,         /* 7 scattered (same scatter every time)          */
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
 * Theme — one colour scheme for the whole scene.  Every colour is picked
 * from the bright half of the 256-colour palette so even the faintest
 * trail tier still shows up on a black terminal (a house rule from
 * CLAUDE.md).  One theme (SUMI_E) is "inverted": a white page with dark
 * ink instead of bright glyphs on black.
 */
typedef struct {
    const char *name;        /* short label shown in the HUD               */
    short       ph[4];       /* one colour per trail-strength tier, faint to
                              * strong, so you read trail strength by colour
                              * as well as by glyph                          */
    short       ant_s;       /* colour of a searching ant                  */
    short       ant_r;       /* colour of a homebound ant — kept clearly
                              * different so the two streams stand apart    */
    short       food;        /* colour of the food markers                 */
    short       nest;        /* colour of the nest marker                  */
    bool        inverted;    /* true for the light-page look: white bg,
                              * dark glyphs, no bold                         */
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

/* Wrap an index into the range 0..n-1, working for negative steps too. */
static int wrap_idx(int i, int n)
{
    i %= n;
    if (i < 0) i += n;
    return i;
}

/* ── §2 clock ── */

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

/* ── §3 color ── */

/* Wire the theme's 256-colour palette into the trail/ant/marker pairs.
 * `bg` is the shared background — white paper, or the terminal default. */
static void theme_bind_truecolor(const Theme *t, short bg)
{
    for (int i = 0; i < 4; i++)
        init_pair((short)(PAIR_PH_BASE + i), t->ph[i], bg);
    init_pair(PAIR_ANT_S, t->ant_s, bg);
    init_pair(PAIR_ANT_R, t->ant_r, bg);
    init_pair(PAIR_FOOD,  t->food,  bg);
    init_pair(PAIR_NEST,  t->nest,  bg);
    init_pair(PAIR_PAPER, XTERM_BLACK, bg);
}

/* Fallback for terminals with only 8 colours: snap to the nearest basic
 * colour, or plain black-on-white for the inverted theme. */
static void theme_bind_8color(const Theme *t)
{
    static const short fb_ph[4] = { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN };
    short bg = t->inverted ? COLOR_WHITE : -1;
    for (int i = 0; i < 4; i++)
        init_pair((short)(PAIR_PH_BASE + i), t->inverted ? COLOR_BLACK : fb_ph[i], bg);
    init_pair(PAIR_ANT_S, t->inverted ? COLOR_BLACK : COLOR_YELLOW,  bg);
    init_pair(PAIR_ANT_R, t->inverted ? COLOR_BLACK : COLOR_MAGENTA, bg);
    init_pair(PAIR_FOOD,  t->inverted ? COLOR_BLACK : COLOR_GREEN,   bg);
    init_pair(PAIR_NEST,  t->inverted ? COLOR_BLACK : COLOR_RED,     bg);
    init_pair(PAIR_PAPER, COLOR_BLACK, COLOR_WHITE);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &themes[idx];
    if (COLORS >= 256) theme_bind_truecolor(t, t->inverted ? XTERM_WHITE : -1);
    else               theme_bind_8color(t);
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

/* ── §4 geometry ── */

/*
 * Cell — one square of the grid, which is also one character on screen.
 * Stored as (row, col) to match both ncurses' y,x order and the trail
 * buffer's indexing, so a Cell drops into either without juggling.  No
 * fractions: an ant is simply on a cell or not, never between two.
 */
typedef struct {
    int row;   /* 0 at the top, growing downward (screen y)  */
    int col;   /* 0 at the left, growing rightward (screen x) */
} Cell;

/*
 * Step — how to move from a cell to one of its 8 touching neighbours.
 * Each component is -1, 0, or +1, and they're never both 0 (that would
 * be "don't move").  We keep all 8 in the fixed DIR8 table below so a
 * direction number 0..7 turns straight into a move.
 */
typedef struct {
    int drow;  /* -1 up,   +1 down,  0 same row  */
    int dcol;  /* -1 left, +1 right, 0 same col  */
} Step;

/* The 8 directions, numbered 0..7 going clockwise from North. */
static const Step DIR8[8] = {
    {-1,  0},  /* 0 N  */  {-1,  1},  /* 1 NE */
    { 0,  1},  /* 2 E  */  { 1,  1},  /* 3 SE */
    { 1,  0},  /* 4 S  */  { 1, -1},  /* 5 SW */
    { 0, -1},  /* 6 W  */  {-1, -1},  /* 7 NW */
};

static Cell cell_step(Cell c, int heading)
{
    return (Cell){ c.row + DIR8[heading].drow, c.col + DIR8[heading].dcol };
}

/* the opposite direction — used to bounce back off a wall. */
static int heading_reverse(int heading) { return (heading + 4) % 8; }

/* which of the 8 directions points most directly at `to`. */
static int heading_toward(Cell from, Cell to)
{
    int   drow = to.row - from.row, dcol = to.col - from.col;
    float best_dot = -1e9f;
    int   best = 0;
    for (int h = 0; h < 8; h++) {
        float dot = (float)DIR8[h].dcol * dcol + (float)DIR8[h].drow * drow;
        if (dot > best_dot) { best_dot = dot; best = h; }
    }
    return best;
}

/* true when `a` and `b` are within `radius` cells of each other. */
static bool cell_in_range(Cell a, Cell b, int radius)
{
    int dr = a.row - b.row; if (dr < 0) dr = -dr;
    int dc = a.col - b.col; if (dc < 0) dc = -dc;
    return (dr + dc) <= radius;
}

/* the directions just clockwise and counter-clockwise of `h` — the two
 * sideways choices a searching ant considers alongside straight ahead. */
static int heading_cw (int h) { return (h + 1) % 8; }
static int heading_ccw(int h) { return (h + 7) % 8; }

/* A direction as smooth x/y rather than one of the 8 fixed steps — needed
 * when we want a fractional "point that way" that DIR8 can't express, like
 * the toward-food direction a searcher leans into. */
typedef struct { float dx, dy; } Vec2f;

/* direction from `from` toward `to`, scaled to length 1 (zero if same cell). */
static Vec2f unit_toward(Cell from, Cell to)
{
    float dx = (float)(to.col - from.col), dy = (float)(to.row - from.row);
    float len = sqrtf(dx * dx + dy * dy);
    if (len > 0.0f) { dx /= len; dy /= len; }
    return (Vec2f){ dx, dy };
}

static int imin(int a, int b) { return a < b ? a : b; }
static int imax(int a, int b) { return a > b ? a : b; }

/* Pick one of n choices at random, but with each choice's odds set by its
 * weight — bigger weight, more likely.  Like a spinning wheel where the
 * slices have different sizes.  If every weight is 0, just pick the middle. */
static int choose_weighted(const float *weight, int n)
{
    float total = 0.0f;
    for (int i = 0; i < n; i++) total += weight[i];
    if (total <= 0.0f) return n / 2;

    float r   = (float)rand() / (float)RAND_MAX * total;
    float acc = 0.0f;
    for (int i = 0; i < n; i++) {
        acc += weight[i];
        if (r <= acc) return i;
    }
    return n - 1;
}

/* ── §5 field ── */

/*
 * PheromoneField — the scent map: one number per cell saying how strong
 * the trail is there.  Ants add to it and read from it, and that's the
 * whole memory of the colony — no single ant remembers a route, the map
 * does.  This idea of coordinating through marks left in the shared
 * environment is called stigmergy (Grassé 1959).
 *
 *   It's a plain 2D array, not a sparse structure, because every cell
 *   gets touched each tick (ants deposit, then the whole grid fades), the
 *   grid is small, and a flat array needs no allocation — it just lives in
 *   static storage.  w/h are the part of the array actually in use for the
 *   current screen size.
 *
 *   How a cell's scent changes each tick (Dorigo & Stützle 2004):
 *     - an ant on it adds some scent (a little while searching, a lot
 *       while carrying food home)
 *     - the total is capped, so one path can't get so strong that ants
 *       stop trying anything else
 *     - then everything fades a bit, so unused trails slowly disappear
 *   The cap and the fading together are what let the colony re-route when
 *   the food moves; without them an old favourite path could never lose.
 */
typedef struct {
    int   w, h;   /* part of the array in use for the current grid size */
    float tau[GRID_H_MAX][GRID_W_MAX];
                  /* scent per cell, 0 (nothing) up to PH_MAX (a busy
                   * highway); same row,col order as Cell so a Cell indexes
                   * it directly                                            */
} PheromoneField;

static void field_resize(PheromoneField *f, int w, int h)
{
    f->w = w < GRID_W_MAX ? w : GRID_W_MAX;
    f->h = h < GRID_H_MAX ? h : GRID_H_MAX;
}

static bool field_in_bounds(const PheromoneField *f, Cell c)
{
    return c.row >= 0 && c.row < f->h && c.col >= 0 && c.col < f->w;
}

static float field_sample(const PheromoneField *f, Cell c)
{
    return field_in_bounds(f, c) ? f->tau[c.row][c.col] : 0.0f;
}

static void field_clear(PheromoneField *f)
{
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++)
            f->tau[r][c] = 0.0f;
}

/* drop some scent on a cell, never letting it exceed the PH_MAX cap. */
static void field_deposit(PheromoneField *f, Cell c, float amount)
{
    if (!field_in_bounds(f, c)) return;
    f->tau[c.row][c.col] += amount;
    if (f->tau[c.row][c.col] > PH_MAX) f->tau[c.row][c.col] = PH_MAX;
}

/* fade every trail a little; trails that get tiny enough are zeroed out. */
static void field_evaporate(PheromoneField *f)
{
    float decay = 1.0f - EVAP_RATE;
    for (int r = 0; r < f->h; r++) {
        for (int c = 0; c < f->w; c++) {
            f->tau[r][c] *= decay;
            if (f->tau[r][c] < PH_EPSILON) f->tau[r][c] = 0.0f;
        }
    }
}

/* which of the 4 strength tiers (0..3) a scent level falls into. */
static int ramp_slot(float tau)
{
    int slot = 0;
    for (int i = 0; i < 3; i++)
        if (tau >= PH_RAMP_T[i]) slot = i + 1;
    return slot;
}

/* ── §6 agents ── */

/*
 * AntPhase — which leg of its round trip an ant is on.  This one flag
 * decides everything: whether the ant wanders or beelines home, and
 * whether it drops a faint or a heavy trail.  The heavy trails dropped on
 * the way home are what guide the next wave of searchers, which is the
 * whole feedback loop that makes good paths win.
 */
typedef enum {
    ANT_SEARCHING,   /* out looking for food          */
    ANT_RETURNING,   /* heading home with food         */
} AntPhase;

/*
 * Ant — one ant.  On purpose it's almost nothing: where it is, which way
 * it's facing, and which leg of the trip it's on.  It carries no map and
 * no memory of how it got anywhere — all the routing knowledge lives in
 * the shared scent map.  That's the point: smart group behaviour out of
 * dumb individuals (Bonabeau, Dorigo & Theraulaz 1999).
 */
typedef struct {
    Cell     at;       /* where it is right now                          */
    int      heading;  /* which way it faces, 0..7 (a DIR8 index).  Having
                        * a facing lets a searcher keep roughly going the
                        * same way instead of twitching in place.         */
    AntPhase phase;    /* searching or returning                          */
} Ant;

/*
 * Colony — all the ants, in one fixed-size array.  The count never
 * changes at runtime; ants are only respawned and flipped between legs.
 * More ants means stronger trails and faster convergence.
 */
typedef struct {
    Ant ant[N_ANTS];   /* order doesn't matter — just a bag of ants      */
} Colony;

static int colony_returning(const Colony *col)
{
    int n = 0;
    for (int i = 0; i < N_ANTS; i++)
        if (col->ant[i].phase == ANT_RETURNING) n++;
    return n;
}

/* ── §7 scene ── */

/*
 * Environment — the puzzle the colony is solving: where home is and where
 * the food is.  No trail data lives here — that's deliberate, so switching
 * food layouts can move the food and wipe the trails separately.
 */
typedef struct {
    Cell nest;             /* home, always the grid centre — both where ants
                            * start and where they bring food back to       */
    Cell food[MAX_FOOD];   /* the food sources for the current layout; an ant
                            * within FOOD_RADIUS of one has reached it       */
    int  food_n;           /* how many of food[] are actually in use         */
    long delivered;        /* total food brought home so far — the "food:"
                            * HUD counter, and a rough sign of how well the
                            * colony has converged (it climbs faster once the
                            * trails settle).  long because it can run for
                            * minutes at high speed.                         */
} Environment;

/*
 * Settings — everything the keys change, kept separate from the world
 * itself so input touches only this and the simulation touches only the
 * world.  The choices that cycle are stored as a position in a list (an
 * index) rather than the value itself, so wrapping around is easy and the
 * HUD can show "3 of 14".
 */
typedef struct {
    bool paused;     /* sim frozen, but drawing and keys still work        */
    int  speed_idx;  /* which rung of SPEED_LADDER we're on (set by +/-)   */
    int  pattern;    /* which food layout (set by n/N)                     */
    int  theme;      /* which colour theme (set by t/T) — looks only       */
    int  sim_fps;    /* how often we redraw, in Hz — this is smoothness,
                      * NOT sim speed (that's speed_idx); the two are kept
                      * separate on purpose                                 */
} Settings;

/*
 * Scene — the entire running world bundled into one thing: the scent map,
 * the ants, the puzzle, the settings, and the screen size.  There's one of
 * these for the whole program, so "what's the state right now?" has a
 * single answer.  Functions that change the world take a Scene*; the
 * drawing code takes a const Scene* and can't change anything.
 */
typedef struct {
    PheromoneField field;   /* the scent map                               */
    Colony         colony;  /* the ants                                    */
    Environment    env;     /* nest + food + score                         */
    Settings       cfg;     /* the user's current choices                  */
    int            cols;    /* terminal width in cells                     */
    int            rows;    /* terminal height in cells                    */
} Scene;

static float scene_speed(const Scene *s) { return SPEED_LADDER[s->cfg.speed_idx]; }

/* index of the food source closest to `from`. */
static int nearest_food(const Environment *e, Cell from)
{
    float best = 1e18f;
    int   fi   = 0;
    for (int k = 0; k < e->food_n; k++) {
        float dr = (float)(e->food[k].row - from.row);
        float dc = (float)(e->food[k].col - from.col);
        float d2 = dr * dr + dc * dc;
        if (d2 < best) { best = d2; fi = k; }
    }
    return fi;
}

static Cell nearest_food_cell(const Environment *e, Cell from)
{
    return e->food[nearest_food(e, from)];
}

/*
 * search_weight — how appealing one step-ahead direction is to a searching
 * ant.  It's the trail strength there times how well that step lines up
 * with the direction of the nearest food.  Feeding these weights to
 * choose_weighted is what makes well-worn, foodward routes win over time
 * (Dorigo, Maniezzo & Colorni 1996).  The little base noise keeps blank
 * cells worth trying; the offset and clamp keep a sideways step possible
 * while ruling out walking backward.
 */
static float search_weight(const PheromoneField *f, Cell at, int h, Vec2f food_dir)
{
    float trail = field_sample(f, cell_step(at, h)) + PH_BASE_NOISE;
    float steer = (float)DIR8[h].dcol * food_dir.dx
                + (float)DIR8[h].drow * food_dir.dy;
    float bias  = FOOD_BIAS * (steer + STEER_OFFSET);
    if (bias < 0.0f) bias = 0.0f;
    return trail * bias;
}

/*
 * step_with_fallback — try to step in the `primary` direction; if that
 * would walk off the grid, try `fallback` instead; if that fails too, stay
 * put.  Hands back the direction it actually used through *used.  Both ant
 * legs share this for bumping into walls.
 */
static Cell step_with_fallback(const PheromoneField *f, Cell at,
                               int primary, int fallback, int *used)
{
    Cell next = cell_step(at, primary);
    if (field_in_bounds(f, next)) { *used = primary; return next; }
    *used = fallback;
    next = cell_step(at, fallback);
    return field_in_bounds(f, next) ? next : at;
}

/* shove a direction left or right by a random step, so homebound trails
 * spread into a path with some width instead of one dead-straight line. */
static int heading_wobble(int h) { return (h + (rand() % 3 - 1) + 8) % 8; }

/* which food source the ant is touching (within FOOD_RADIUS), or -1. */
static int food_underfoot(const Environment *e, Cell at)
{
    for (int k = 0; k < e->food_n; k++)
        if (cell_in_range(at, e->food[k], FOOD_RADIUS)) return k;
    return -1;
}

/* has the ant made it home — close enough to the nest? */
static bool at_nest(const Environment *e, Cell at)
{
    return cell_in_range(at, e->nest, FOOD_RADIUS);
}

/* One tick for a searching ant: drop a faint trail, lean toward food, step,
 * and if it bumps into food, switch to carrying it home. */
static void ant_search_step(Scene *s, Ant *a)
{
    PheromoneField *f = &s->field;

    field_deposit(f, a->at, DEPOSIT_SEARCH);

    /* consider straight ahead plus a little left and right, scoring each
     * by trail strength and how foodward it is, then pick by those scores. */
    int   arc[SEARCH_ARC] = { heading_ccw(a->heading), a->heading,
                              heading_cw(a->heading) };
    Vec2f food_dir = unit_toward(a->at, nearest_food_cell(&s->env, a->at));
    float weight[SEARCH_ARC];
    for (int k = 0; k < SEARCH_ARC; k++)
        weight[k] = search_weight(f, a->at, arc[k], food_dir);

    int heading = arc[choose_weighted(weight, SEARCH_ARC)];
    a->at = step_with_fallback(f, a->at, heading, heading_reverse(heading), &heading);
    a->heading = heading;

    if (food_underfoot(&s->env, a->at) >= 0) {
        a->phase   = ANT_RETURNING;
        a->heading = heading_toward(a->at, s->env.nest);
    }
}

/* One tick for a homebound ant: drop a heavy trail (the strong scent that
 * pulls in the next searchers), head for the nest with a little wobble,
 * and deliver when it arrives. */
static void ant_return_step(Scene *s, Ant *a)
{
    PheromoneField *f = &s->field;

    field_deposit(f, a->at, DEPOSIT_RETURN);

    int aim = heading_toward(a->at, s->env.nest);
    int heading;
    a->at = step_with_fallback(f, a->at, heading_wobble(aim), aim, &heading);
    a->heading = heading;

    if (at_nest(&s->env, a->at)) {
        s->env.delivered++;
        a->phase   = ANT_SEARCHING;
        a->heading = rand() % 8;
    }
}

static void ant_step(Scene *s, Ant *a)
{
    if (a->phase == ANT_SEARCHING) ant_search_step(s, a);
    else                           ant_return_step(s, a);
}

/* a random offset between -radius and +radius, for scattering ants. */
static int jitter(int radius) { return rand() % (2 * radius + 1) - radius; }

/* put one ant back near the nest, searching, facing a random way. */
static void ant_spawn(Scene *s, Ant *a)
{
    a->at.row  = s->env.nest.row + jitter(SPAWN_RADIUS);
    a->at.col  = s->env.nest.col + jitter(SPAWN_RADIUS);
    a->heading = rand() % 8;
    a->phase   = ANT_SEARCHING;
}

/* One step of the whole world: move every ant, then fade all the trails. */
static void scene_step(Scene *s)
{
    for (int i = 0; i < N_ANTS; i++) ant_step(s, &s->colony.ant[i]);
    field_evaporate(&s->field);
}

/*
 * FoodLayout — the handy numbers for placing food, all worked out once
 * from the grid size: the centre, the ring radii for circular layouts, and
 * how far in from the edges to keep things.  Clamped so even a tiny
 * terminal still gets a sane layout.
 */
typedef struct { int cx, cy, rx, ry, margin_x, margin_y; } FoodLayout;

static FoodLayout food_layout(int cols, int rows)
{
    FoodLayout L;
    L.margin_x = imax(cols / EDGE_MARGIN_DIV, EDGE_MARGIN_MIN_X);
    L.margin_y = imax(rows / EDGE_MARGIN_DIV, EDGE_MARGIN_MIN_Y);
    L.cx = cols / 2;
    L.cy = rows / 2;
    L.rx = imax(cols / 2 - L.margin_x / 2, RING_RADIUS_MIN_X);
    L.ry = imax(rows / 2 - L.margin_y / 2, RING_RADIUS_MIN_Y);
    return L;
}

/* the point at `angle` around the nest's ring — used by the layouts that
 * arrange food in a circle (hexagon, circle, triangle). */
static Cell ring_point(FoodLayout L, float angle)
{
    return (Cell){ L.cy + (int)(sinf(angle) * (float)L.ry),
                   L.cx + (int)(cosf(angle) * (float)L.rx) };
}

/* angle of point k when n points are spread evenly around the ring. */
static float ring_angle(int k, int n) { return (float)k * (float)(2.0 * M_PI / n); }

/* place the food sources for the current layout. */
static void scene_place_food(Scene *s)
{
    Environment *e = &s->env;
    int cols = s->field.w, rows = s->field.h;
    FoodLayout L = food_layout(cols, rows);
    int cx = L.cx, cy = L.cy, rx = L.rx, ry = L.ry;
    int margin_x = L.margin_x, margin_y = L.margin_y;

    switch (s->cfg.pattern) {
    case PAT_SINGLE:
        e->food_n  = 1;
        e->food[0] = (Cell){ rows / 4, cols - margin_x };
        break;
    case PAT_QUAD:
        e->food_n  = 4;
        e->food[0] = (Cell){ margin_y,        cx };
        e->food[1] = (Cell){ rows - margin_y, cx };
        e->food[2] = (Cell){ cy, margin_x };
        e->food[3] = (Cell){ cy, cols - margin_x };
        break;
    case PAT_LINE:
        e->food_n  = 3;
        e->food[0] = (Cell){ rows / 4,     cols - margin_x };
        e->food[1] = (Cell){ cy,           cols - margin_x };
        e->food[2] = (Cell){ rows * 3 / 4, cols - margin_x };
        break;
    case PAT_HEXAGON:
        /* 6 evenly around the ring */
        e->food_n = 6;
        for (int k = 0; k < 6; k++)
            e->food[k] = ring_point(L, ring_angle(k, 6));
        break;
    case PAT_CROSS:
        e->food_n  = 4;
        e->food[0] = (Cell){ cy - ry / 2, cx };
        e->food[1] = (Cell){ cy + ry / 2, cx };
        e->food[2] = (Cell){ cy, cx - rx / 2 };
        e->food[3] = (Cell){ cy, cx + rx / 2 };
        break;
    case PAT_TRIANGLE:
        /* a triangle with one point straight up */
        e->food_n = 3;
        for (int k = 0; k < 3; k++)
            e->food[k] = ring_point(L, -QUARTER_TURN + ring_angle(k, 3));
        break;
    case PAT_CIRCLE:
        /* 8 evenly around the ring */
        e->food_n = 8;
        for (int k = 0; k < 8; k++)
            e->food[k] = ring_point(L, ring_angle(k, 8));
        break;
    case PAT_DIAGONAL:
        e->food_n  = 4;
        e->food[0] = (Cell){ margin_y,        margin_x };
        e->food[1] = (Cell){ cy - ry / 3,     cx - rx / 3 };
        e->food[2] = (Cell){ cy + ry / 3,     cx + rx / 3 };
        e->food[3] = (Cell){ rows - margin_y, cols - margin_x };
        break;
    case PAT_CLUSTER:
        /* 5 bunched together on the right */
        e->food_n  = 5;
        e->food[0] = (Cell){ cy - 2, cols - margin_x };
        e->food[1] = (Cell){ cy + 2, cols - margin_x };
        e->food[2] = (Cell){ cy,     cols - margin_x - 3 };
        e->food[3] = (Cell){ cy - 1, cols - margin_x + 2 };
        e->food[4] = (Cell){ cy + 1, cols - margin_x + 2 };
        break;
    case PAT_DISTANT:
        e->food_n  = 2;
        e->food[0] = (Cell){ margin_y,        margin_x };
        e->food[1] = (Cell){ rows - margin_y, cols - margin_x };
        break;
    case PAT_PERIMETER:
        e->food_n  = 6;
        e->food[0] = (Cell){ margin_y,        cols / 4 };
        e->food[1] = (Cell){ margin_y,        cols * 3 / 4 };
        e->food[2] = (Cell){ cy,              margin_x };
        e->food[3] = (Cell){ cy,              cols - margin_x };
        e->food[4] = (Cell){ rows - margin_y, cols / 4 };
        e->food[5] = (Cell){ rows - margin_y, cols * 3 / 4 };
        break;
    case PAT_GRID: {
        /* a 3x3 grid, leaving the middle empty for the nest */
        e->food_n = 0;
        int gx[3] = { margin_x, cx, cols - margin_x };
        int gy[3] = { margin_y, cy, rows - margin_y };
        for (int j = 0; j < 3; j++)
            for (int i = 0; i < 3; i++) {
                if (i == 1 && j == 1) continue;     /* the nest's spot */
                e->food[e->food_n++] = (Cell){ gy[j], gx[i] };
            }
        break;
    }
    case PAT_RANDOM: {
        /* 7 scattered spots — the same scatter every time, so it looks
         * irregular but is reproducible (fixed starting seed). */
        e->food_n = 7;
        unsigned rng = 0xC0FFEEu;
        for (int k = 0; k < 7; k++) {
            rng = rng * 1664525u + 1013904223u;
            int rc = (int)(rng >> 16) % (cols - 2 * margin_x);
            rng = rng * 1664525u + 1013904223u;
            int rr = (int)(rng >> 16) % (rows - 2 * margin_y);
            e->food[k] = (Cell){ margin_y + rr, margin_x + rc };
        }
        break;
    }
    case PAT_DOUBLE:
    default:
        e->food_n  = 2;
        e->food[0] = (Cell){ rows / 3,     margin_x };
        e->food[1] = (Cell){ rows * 2 / 3, cols - margin_x };
        break;
    }
}

/* fit the grid and nest to the screen, then lay out the food. */
static void scene_layout(Scene *s)
{
    int play_rows = imax(s->rows - HUD_ROWS, MIN_PLAY_ROWS);

    field_resize(&s->field, s->cols, play_rows);
    s->env.nest = (Cell){ s->field.h / 2, s->field.w / 2 };
    scene_place_food(s);
}

/* start over: re-lay-out, wipe the trails and score, respawn every ant. */
static void scene_reset(Scene *s)
{
    scene_layout(s);
    s->env.delivered = 0;
    field_clear(&s->field);
    for (int i = 0; i < N_ANTS; i++) ant_spawn(s, &s->colony.ant[i]);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cols          = cols;
    s->rows          = rows;
    s->cfg.paused    = false;
    s->cfg.speed_idx = SPEED_DEFAULT_IDX;
    s->cfg.pattern   = PAT_DOUBLE;
    s->cfg.theme     = 0;
    s->cfg.sim_fps   = SIM_FPS_DEFAULT;
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
    s->cfg.pattern = wrap_idx(s->cfg.pattern + dir, N_PATTERNS);
    scene_reset(s);
}

static void scene_cycle_theme(Scene *s, int dir)
{
    s->cfg.theme = wrap_idx(s->cfg.theme + dir, N_THEMES);
    theme_apply(s->cfg.theme);
}

/* ── §8 render ── *
 * Everything here only reads the scene and paints it — it never changes the
 * world.  Grid row r is drawn one row lower than it sounds, leaving the very
 * top row free for the HUD. */

/* draw a bracketed marker like [*] or (@) at a cell, clipped to the grid. */
static void draw_marker(Cell c, int top, int gw, int gh,
                        char left, char mid, char right)
{
    if (c.row < 0 || c.row >= gh) return;
    int y = c.row + top;
    if (c.col - 1 >= 0 && c.col - 1 < gw) mvaddch(y, c.col - 1, (chtype)(unsigned char)left);
    if (c.col     >= 0 && c.col     < gw) mvaddch(y, c.col,     (chtype)(unsigned char)mid);
    if (c.col + 1 >= 0 && c.col + 1 < gw) mvaddch(y, c.col + 1, (chtype)(unsigned char)right);
}

/* how to emphasise a trail tier: strongest is bold, faintest is dim, the
 * rest normal.  The light-page theme stays plain so dark ink reads clean. */
static attr_t trail_attr(int slot, bool inverted)
{
    if (inverted)  return A_NORMAL;
    if (slot == 3) return A_BOLD;
    if (slot == 0) return A_DIM;
    return A_NORMAL;
}

/* paint the whole play area white first, for the light-page theme. */
static void render_paper(const PheromoneField *f, int top)
{
    attron(COLOR_PAIR(PAIR_PAPER));
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++)
            mvaddch(r + top, c, ' ');
    attroff(COLOR_PAIR(PAIR_PAPER));
}

/* draw the scent map as the visible trails, glyph and colour per strength. */
static void render_trails(const PheromoneField *f, bool inverted, int top)
{
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++) {
            float tau = f->tau[r][c];
            if (tau < PH_DRAW_MIN) continue;          /* too faint to draw */
            int    slot = ramp_slot(tau);
            attr_t attr = trail_attr(slot, inverted);
            attron(COLOR_PAIR(PAIR_PH_BASE + slot) | attr);
            mvaddch(r + top, c, (chtype)(unsigned char)PH_GLYPHS[slot]);
            attroff(COLOR_PAIR(PAIR_PH_BASE + slot) | attr);
        }
}

static void render_food(const Environment *e, const PheromoneField *f,
                        attr_t attr, int top)
{
    attron(COLOR_PAIR(PAIR_FOOD) | attr);
    for (int k = 0; k < e->food_n; k++)
        draw_marker(e->food[k], top, f->w, f->h,
                    FOOD_FRAME_L, FOOD_GLYPH, FOOD_FRAME_R);
    attroff(COLOR_PAIR(PAIR_FOOD) | attr);
}

static void render_nest(const Environment *e, const PheromoneField *f,
                        attr_t attr, int top)
{
    attron(COLOR_PAIR(PAIR_NEST) | attr);
    draw_marker(e->nest, top, f->w, f->h, NEST_FRAME_L, NEST_GLYPH, NEST_FRAME_R);
    attroff(COLOR_PAIR(PAIR_NEST) | attr);
}

/* draw each ant, coloured by whether it's searching or carrying food home. */
static void render_colony(const Colony *col, const PheromoneField *f,
                          attr_t attr, int top)
{
    for (int i = 0; i < N_ANTS; i++) {
        Cell at = col->ant[i].at;
        if (at.row < 0 || at.row >= f->h || at.col < 0 || at.col >= f->w) continue;
        int pair = (col->ant[i].phase == ANT_RETURNING) ? PAIR_ANT_R : PAIR_ANT_S;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(at.row + top, at.col, (chtype)(unsigned char)ANT_GLYPH);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* paint one frame, back to front: paper, trails, food, nest, ants. */
static void render_world(const Scene *s)
{
    bool   inverted = themes[s->cfg.theme].inverted;
    int    top      = HUD_TOP_ROWS;             /* shift down past the HUD */
    attr_t marker   = inverted ? A_NORMAL : A_BOLD;

    if (inverted) render_paper(&s->field, top);
    render_trails(&s->field,   inverted, top);
    render_food  (&s->env,     &s->field, marker, top);
    render_nest  (&s->env,     &s->field, marker, top);
    render_colony(&s->colony,  &s->field, marker, top);
}

/* draw one full-width HUD bar; the text is cut to `cols` so it can't spill
 * onto the next line and over the scene. */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* build the top status line: state, layout, theme, how many ants are
 * heading home, food delivered, speed, and timing. */
static void hud_status_line(char *buf, size_t n, const Scene *s, double fps)
{
    const Settings *cfg = &s->cfg;
    snprintf(buf, n,
             " ANT_COLONY  %s  pattern:%s %2d/%d  theme:%s %d/%d  "
             "ret:%2d  food:%4ld  speed:%4gx  %5.1f fps  %3d Hz ",
             cfg->paused ? "PAUSED" : "FORAGE",
             pattern_name((Pattern)cfg->pattern), cfg->pattern + 1, N_PATTERNS,
             themes[cfg->theme].name, cfg->theme + 1, N_THEMES,
             colony_returning(&s->colony), s->env.delivered,
             scene_speed(s), fps, cfg->sim_fps);
}

static void render_hud(const Scene *s, double fps)
{
    char status[200];
    hud_status_line(status, sizeof status, s, fps);

    const char *keys =
        " n/N:pattern  t/T:theme  +/-:speed  [/]:Hz  "
        "spc:pause  r:reset  q:quit ";

    hud_bar(0,           s->cols, PAIR_HUD,  status);  /* top: live info   */
    hud_bar(s->rows - 1, s->cols, PAIR_HINT, keys);    /* bottom: key hints */
}

/* ── §9 app ── */

/*
 * App — the running program wrapped around the world: the Scene plus the
 * loop's own bookkeeping.  These extra fields are about the program, not
 * the simulation, which is why they live here and not in Scene.  There's
 * one global App so the signal handlers can flip its flags.
 */
typedef struct {
    Scene scene;             /* the simulated world                         */
    float tick_accum;        /* leftover speed carried frame to frame, so
                              * slow speeds work: it builds up until it's
                              * worth a whole step.  At 0.25x that takes four
                              * frames per step.                             */
    volatile sig_atomic_t running;     /* set to 0 to quit; written by the
                                        * Ctrl-C / terminate signal handlers,
                                        * so it needs this safe type          */
    volatile sig_atomic_t need_resize; /* the resize signal sets this; we act
                                        * on it at the top of the loop, never
                                        * inside the handler itself            */
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

/* on a terminal resize: re-read the new size and re-lay-out the scene. */
static void app_resize(App *app)
{
    int cols, rows;
    endwin();
    refresh();
    getmaxyx(stdscr, rows, cols);
    scene_resize(&app->scene, cols, rows);
    app->need_resize = 0;
}

/* move the speed / redraw-rate one notch, staying inside the allowed range. */
static void settings_nudge_speed(Settings *c, int delta)
{
    c->speed_idx = imax(0, imin(N_SPEEDS - 1, c->speed_idx + delta));
}
static void settings_nudge_fps(Settings *c, int delta)
{
    c->sim_fps = imax(SIM_FPS_MIN, imin(SIM_FPS_MAX, c->sim_fps + delta));
}

/* act on a key press; returns false only when the user wants to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene    *s   = &app->scene;
    Settings *cfg = &s->cfg;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
    case 'p': case 'P': cfg->paused = !cfg->paused;     break;
    case 'r': case 'R': scene_reset(s);                 break;

    case 'n':           scene_cycle_pattern(s, +1);     break;
    case 'N':           scene_cycle_pattern(s, -1);     break;

    case 't':           scene_cycle_theme(s, +1);       break;
    case 'T':           scene_cycle_theme(s, -1);       break;

    case '=': case '+': settings_nudge_speed(cfg, +1);          break;
    case '-':           settings_nudge_speed(cfg, -1);          break;

    case ']':           settings_nudge_fps(cfg, +SIM_FPS_STEP); break;
    case '[':           settings_nudge_fps(cfg, -SIM_FPS_STEP); break;

    default: break;
    }
    return true;
}

/* run the world forward by this frame's worth of speed.  The leftover
 * carried in tick_accum is what lets slow speeds work — at 0.25x a step
 * happens only every fourth frame. */
static void app_advance(App *app)
{
    if (app->scene.cfg.paused) return;
    app->tick_accum += scene_speed(&app->scene);
    while (app->tick_accum >= 1.0f) {
        scene_step(&app->scene);
        app->tick_accum -= 1.0f;
    }
}

static void app_draw(const App *app, double fps)
{
    erase();
    render_world(&app->scene);
    render_hud(&app->scene, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * FpsMeter — averages the frame rate over a short window so the number in
 * the HUD holds steady instead of flickering every frame.  It tallies time
 * and frames, then works out a rate once each window.
 */
typedef struct {
    int64_t accum_ns;   /* time tallied since the last update   */
    int     frames;     /* frames tallied since the last update */
    double  value;      /* the rate currently shown in the HUD  */
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

/* sleep just enough that frames come out at the chosen rate. */
static void app_pace_frame(const App *app, int64_t frame_start, int64_t dt)
{
    int64_t target  = TICK_NS(app->scene.cfg.sim_fps);
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(target - elapsed);
}

/* time since the last frame, capped so one long pause (laptop sleep, a
 * stall) can't make the sim lurch forward all at once. */
static int64_t frame_delta(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    return dt > DT_CAP_MS * NS_PER_MS ? DT_CAP_MS * NS_PER_MS : dt;
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

    screen_init();
    int cols, rows;
    getmaxyx(stdscr, rows, cols);
    scene_init(&app->scene, cols, rows);

    int64_t  frame_time = clock_ns();
    FpsMeter fps        = { 0, 0, 0.0 };

    /*
     * The main loop, one pass per frame: read a key, measure the time gap,
     * step the world, wait to hit the target frame rate, then draw.  A key
     * is checked every single frame, so the controls stay snappy no matter
     * how fast or slow the simulation is running.
     */
    while (app->running) {
        if (app->need_resize) {
            app_resize(app);
            frame_time = clock_ns();
        }

        int ch = getch();                                       /* INPUT   */
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;

        int64_t dt = frame_delta(&frame_time);                  /* TIME    */

        app_advance(app);                                       /* EFFECTS */
        fps_meter_tick(&fps, dt);
        app_pace_frame(app, frame_time, dt);                    /* DELAY   */
        app_draw(app, fps.value);                               /* RENDER  */
    }

    endwin();
    return 0;
}
