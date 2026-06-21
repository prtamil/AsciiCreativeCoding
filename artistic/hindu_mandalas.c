/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hindu_mandalas.c — 30 parametric Hindu mandalas drawn in the terminal.
 *
 * Every mandala is just a list of concentric rings (petals, dotted circles,
 * star polygons, rays, ...) around a shared centre. "Thirty mandalas" is one
 * draw function fed thirty different ring recipes.
 *
 * Yantra/mandala forms: Khanna, "Yantra" (1979); Tucci, "The Theory and
 * Practice of the Mandala" (1961); Kulaichev, "Sriyantra..." (1984) for the
 * Sri Yantra triangles. Star polygons {n/d}: Coxeter, "Regular Polytopes".
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — preset table, Ring/Preset types, constants, themes ── */

enum {
    TARGET_FPS    = 60,
    FPS_UPDATE_MS = 500,

    MAX_RINGS = 8,    /* enough for complex presets (Maha Yantra uses 7) */
    N_PRESETS = 30,
    N_THEMES  = 4,

    /* Color pair IDs */
    PAIR_BINDU   = 1,
    PAIR_CIRCLE  = 2,
    PAIR_DOTS    = 3,
    PAIR_PETALS  = 4,
    PAIR_POLYGON = 5,
    PAIR_STAR    = 6,
    PAIR_RAYS    = 7,
    PAIR_HUD     = 8,
    PAIR_HINT    = 9,
};

#define ASPECT        0.5f       /* terminal cells ~2:1 tall:wide */
#define ROT_RATE      0.20f      /* rad/sec when rotation enabled */
#define SCALE_MIN     0.40f
#define SCALE_MAX     1.00f
#define SCALE_DEFAULT 0.85f
#define SCALE_STEP    0.05f
#define NS_PER_SEC    1000000000LL

/* Build-in animation timing: the centre dot appears at BINDU_AT seconds, then
 * each ring fills in over RING_BUILD_DUR seconds, one after another. */
#define BINDU_AT       0.10f
#define RING_BUILD_DUR 0.55f

/* Primitive geometry */
#define CIRCLE_OVERSAMPLE  1.4f   /* '.' samples per cell of circle circumference */
#define CIRCLE_SAMPLES_MIN 24     /* clamp so small circles still read as a ring  */
#define CIRCLE_SAMPLES_MAX 360    /* and large ones don't oversample pointlessly  */
#define PETAL_FLANK_LO     0.85f  /* inner flank dot of a petal cluster (× ring r) */
#define PETAL_FLANK_HI     1.15f  /* outer flank dot of a petal cluster (× ring r) */
#define RAY_INNER_FRAC     0.10f  /* rays start this far out → a centre gap        */

/* Layout */
#define SCREEN_MARGIN      4      /* cells kept clear around the mandala (HUD rows) */
#define MANDALA_MIN_R      2.0f   /* never shrink the mandala below this radius     */

/* ── §1.1 Ring + Preset types ── */

/* The six shapes a mandala ring can be. A mandala is a stack of these laid out
 * around a shared centre. RING_NONE is a marker: it's 0, so an array of rings
 * that wasn't fully filled in ({0}-padded) ends at the first RING_NONE.
 *   RING_CIRCLE  : a continuous dotted ring ('.')
 *   RING_DOTS    : N evenly-spaced 'o' beads
 *   RING_PETALS  : N petal clusters (a lotus / padma)
 *   RING_POLYGON : N vertices joined edge-to-edge (a regular n-gon)
 *   RING_STAR    : N vertices joined with a skip stride (a star polygon)
 *   RING_RAYS    : N spokes from an inner gap out to the rim (a chakra) */
typedef enum {
    RING_NONE = 0, RING_CIRCLE, RING_DOTS, RING_PETALS,
    RING_POLYGON, RING_STAR, RING_RAYS,
} RingType;

/* One concentric layer of a mandala: a shape repeated N times around the centre
 * at some radius. "density" is the star-polygon skip: when joining N points
 * around a circle, you connect each one to the point d steps ahead instead of
 * the next one. d=1 gives a plain polygon; d>=2 gives a star (e.g. 6 points
 * with d=2 is a hexagram — two overlaid triangles). Notation {n/d} from Coxeter.
 *   type    : which shape (a RingType, stored as int).
 *   n       : how many — beads / petals / vertices / rays (0 for a circle,
 *             whose dot count is worked out from its size at draw time).
 *   density : the star skip; <2 means polygon, >=2 means star.
 *   radius  : size as a fraction of the mandala's base radius, in (0, 1]. */
typedef struct {
    int   type;     /* RingType */
    int   n;        /* feature count (or 0 for RING_CIRCLE) */
    int   density;  /* star polygon stride (>=2 = star, else polygon) */
    float radius;   /* fraction of base radius, in (0, 1] */
} Ring;

/* One named mandala as a recipe: a centre-dot flag plus an ordered list of
 * rings. Named forms (Sri Yantra, Padma, ...) are canonical yantras; each is
 * just a particular choice of rings at particular radii. Ring order matters: it
 * runs innermost to outermost and also drives the build-in animation order.
 *   name   : label shown in the HUD.
 *   rings  : up to MAX_RINGS layers; the list ends at the first RING_NONE, so
 *            short presets leave the rest {0}-padded.
 *   bindu  : draw the central '@' dot (the bindu, the mandala's seed-point),
 *            painted last so it always wins the centre cell. */
typedef struct {
    const char *name;
    Ring rings[MAX_RINGS];
    bool bindu;
} MandalaPreset;

/* Shorthand to write one ring on one line. Parameters are CC/DD/RR rather than
 * n/d/r so a literal `n` doesn't collide with the `.n` field name when the
 * preprocessor expands the macro. */
#define R(t,CC,DD,RR) {.type = RING_##t, .n = (CC), .density = (DD), .radius = (RR)}

static const MandalaPreset PRESETS[N_PRESETS] = {
/*  20 simple mandalas — 1 to 4 rings each */
    {"Bindu",       {{0}}, true},
    {"Padma 8",     {R(CIRCLE,  0,0, 0.18f), R(PETALS,  8,0, 0.65f)},                                                    true},
    {"Padma 12",    {R(CIRCLE,  0,0, 0.18f), R(PETALS, 12,0, 0.65f)},                                                    true},
    {"Padma 16",    {R(PETALS,  8,0, 0.30f), R(PETALS, 16,0, 0.75f)},                                                    true},
    {"Padma 32",    {R(PETALS, 16,0, 0.45f), R(PETALS, 32,0, 0.85f)},                                                    true},
    {"Trikon",      {R(POLYGON, 3,0, 0.65f), R(CIRCLE,  0,0, 0.88f)},                                                    true},
    {"Shatkona",    {R(STAR,    6,2, 0.65f), R(CIRCLE,  0,0, 0.88f)},                                                    true},
    {"Ashtakona",   {R(STAR,    8,3, 0.65f), R(CIRCLE,  0,0, 0.88f)},                                                    true},
    {"Dwadashara",  {R(STAR,   12,5, 0.65f), R(CIRCLE,  0,0, 0.88f)},                                                    true},
    {"Chakra 8",    {R(RAYS,    8,0, 0.85f), R(CIRCLE,  0,0, 0.88f), R(CIRCLE, 0,0, 0.30f)},                             true},
    {"Chakra 16",   {R(RAYS,   16,0, 0.85f), R(CIRCLE,  0,0, 0.88f), R(CIRCLE, 0,0, 0.40f)},                             true},
    {"Bhupura",     {R(PETALS,  8,0, 0.55f), R(POLYGON, 4,0, 0.90f)},                                                    true},
    {"Sri Yantra",  {R(STAR,    9,4, 0.40f), R(STAR,    9,2, 0.55f), R(PETALS, 8,0, 0.72f), R(POLYGON, 4,0, 0.92f)},     true},
    {"Kali",        {R(STAR,    5,2, 0.45f), R(PETALS,  8,0, 0.70f), R(POLYGON, 4,0, 0.92f)},                            true},
    {"Lakshmi",     {R(STAR,    8,3, 0.45f), R(PETALS, 16,0, 0.75f), R(POLYGON, 4,0, 0.92f)},                            true},
    {"Saraswati",   {R(CIRCLE,  0,0, 0.20f), R(PETALS, 16,0, 0.65f), R(CIRCLE, 0,0, 0.90f)},                             true},
    {"Ganesh",      {R(PETALS,  8,0, 0.50f), R(DOTS,   24,0, 0.72f), R(POLYGON, 4,0, 0.92f)},                            true},
    {"Surya",       {R(RAYS,   12,0, 0.85f), R(CIRCLE,  0,0, 0.65f), R(PETALS, 12,0, 0.30f)},                            true},
    {"Anahata",     {R(STAR,    6,2, 0.40f), R(PETALS, 12,0, 0.75f), R(CIRCLE, 0,0, 0.90f)},                             true},
    {"Rudra",       {R(STAR,   11,4, 0.55f), R(RAYS,   11,0, 0.85f), R(CIRCLE, 0,0, 0.90f)},                             true},

/*  10 complex mandalas — 5 to 7 rings each */
    {"Sahasrara",       {R(PETALS,  8,0, 0.20f), R(PETALS, 16,0, 0.36f), R(PETALS, 24,0, 0.52f), R(PETALS, 32,0, 0.68f), R(PETALS, 48,0, 0.83f), R(CIRCLE, 0,0, 0.92f)}, true},
    {"Maha Yantra",     {R(STAR,    9,4, 0.30f), R(STAR,    9,2, 0.42f), R(STAR,    8,3, 0.52f), R(STAR,    6,2, 0.62f), R(PETALS,  8,0, 0.74f), R(PETALS, 16,0, 0.84f), R(POLYGON, 4,0, 0.94f)}, true},
    {"Kalachakra",      {R(CIRCLE,  0,0, 0.18f), R(RAYS,   12,0, 0.50f), R(PETALS, 12,0, 0.55f), R(RAYS,   24,0, 0.78f), R(PETALS, 24,0, 0.83f), R(POLYGON, 4,0, 0.92f), R(CIRCLE,  0,0, 0.95f)}, true},
    {"Mahakali",        {R(STAR,    5,2, 0.30f), R(STAR,    6,2, 0.46f), R(PETALS,  8,0, 0.60f), R(PETALS, 12,0, 0.74f), R(RAYS,   16,0, 0.86f), R(CIRCLE,  0,0, 0.92f)}, true},
    {"Bhairava",        {R(STAR,    8,3, 0.30f), R(STAR,   16,7, 0.48f), R(RAYS,    8,0, 0.65f), R(PETALS, 24,0, 0.78f), R(CIRCLE,  0,0, 0.85f), R(POLYGON, 4,0, 0.92f)}, true},
    {"Sudarshana",      {R(RAYS,    8,0, 0.34f), R(PETALS,  8,0, 0.40f), R(RAYS,   16,0, 0.55f), R(PETALS, 16,0, 0.60f), R(RAYS,   24,0, 0.78f), R(CIRCLE,  0,0, 0.85f), R(CIRCLE,  0,0, 0.92f)}, true},
    {"Mahamrityunjaya", {R(STAR,    5,2, 0.28f), R(STAR,    6,2, 0.42f), R(PETALS,  8,0, 0.56f), R(PETALS, 12,0, 0.70f), R(PETALS, 16,0, 0.82f), R(POLYGON, 4,0, 0.92f)}, true},
    {"Mahalakshmi",     {R(STAR,    9,2, 0.26f), R(STAR,    8,3, 0.40f), R(PETALS,  8,0, 0.54f), R(PETALS, 16,0, 0.66f), R(PETALS, 24,0, 0.80f), R(POLYGON, 4,0, 0.90f), R(CIRCLE,  0,0, 0.94f)}, true},
    {"Vajra",           {R(RAYS,    4,0, 0.30f), R(RAYS,    8,0, 0.50f), R(RAYS,   16,0, 0.80f), R(PETALS, 12,0, 0.42f), R(PETALS, 24,0, 0.65f), R(POLYGON, 4,0, 0.92f), R(CIRCLE,  0,0, 0.95f)}, true},
    {"Mahaganesha",     {R(STAR,    6,2, 0.28f), R(PETALS,  8,0, 0.42f), R(PETALS, 16,0, 0.58f), R(PETALS, 32,0, 0.74f), R(RAYS,   24,0, 0.86f), R(POLYGON, 4,0, 0.92f), R(CIRCLE,  0,0, 0.95f)}, true},
};

/* ── §1.2 Themes ── */
/* Six ring-type colours per theme plus a bindu colour, as 256-colour indices.
 * All kept above 24 so even the dimmest stays readable under A_DIM. */
static const int THEME_PALETTE[N_THEMES][6] = {
    /*           CIRCLE DOTS  PETALS POLY  STAR  RAYS                  */
    /* SAFFRON */ { 220,  214,  208,  202,  226,  220 }, /* warm gold/red */
    /* OCEAN   */ {  39,   45,   51,   33,  117,   75 }, /* cyans + blues */
    /* FOREST  */ {  34,   40,   46,  118,  154,   28 }, /* greens */
    /* COSMIC  */ { 165,  171,  201,  207,  213,  219 }, /* magenta + violet */
};
static const int   THEME_BINDU[N_THEMES] = { 230, 195, 195, 230 };
static const char *THEME_NAME [N_THEMES] = { "Saffron", "Ocean", "Forest", "Cosmic" };

/* ── §2 performance — monotonic clock + sleep ── */

static int64_t clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* ── §3 logic — pure maps & queries (no state changes, no screen) ── */

/* Turn an angle + radius into a screen cell. Terminal cells are about twice as
 * tall as wide, so the vertical part is squashed by ASPECT to keep circles
 * round instead of stretched. */
static inline void polar_to_cell(int cx, int cy, float r, float theta,
                                 int *col, int *row) {
    *col = cx + (int)roundf(r * cosf(theta));
    *row = cy + (int)roundf(r * sinf(theta) * ASPECT);
}

/* Angle of the i-th of n things evenly spaced around the circle, offset by rot. */
static inline float feature_angle(int i, int n, float rot) {
    return rot + (float)i / (float)n * 2.0f * (float)M_PI;
}

/* Clamp the star skip to something drawable: 1 (plain polygon) unless it's a
 * real star skip (2 up to n-1). */
static int star_stride(int density, int n) {
    int d = (density < 2) ? 1 : density;
    if (d >= n) d = 1;
    return d;
}

/* Pick the ASCII character that best matches a line's direction: - | / or \. */
static char line_glyph(int dx, int dy) {
    /* Undo the cell aspect first, so the steepness test reflects how the line
     * actually looks on screen, not its raw cell counts. */
    float adx = (float)abs(dx);
    float ady = (float)abs(dy) / ASPECT;
    if (adx < 0.5f) return '|';
    if (ady < 0.5f) return '-';
    float r = ady / adx;
    if (r < 0.5f) return '-';
    if (r > 2.0f) return '|';
    /* On screen, y grows downward, so a line going down-and-right is '\'. */
    return ((dx > 0) == (dy > 0)) ? '\\' : '/';
}

/* How many of a ring's n features to show at this point in its build-in (0..n).
 * Once a ring starts appearing it always shows at least one, so it announces
 * itself right away. The 0.999 cutoff (instead of 1.0) is a float-rounding
 * guard: the progress math can land at 0.99998 when a ring is really done, and
 * without the slack the last feature got dropped — most visibly leaving a
 * 4-sided polygon open on one edge. */
static int progress_to_count(int n, float progress) {
    if (progress >= 0.999f) return n;
    if (progress <= 0.0f)   return 0;
    int k = (int)(progress * (float)n);
    if (k < 1) k = 1;
    if (k > n) k = n;
    return k;
}

/* How far along ring i's build-in is, 0 to 1; returns <= 0 before its turn
 * comes. Ring i waits for the bindu plus i full ring-build slots, then fills
 * over one slot. */
static float ring_build_progress(int i, float build_time) {
    float ring_start = BINDU_AT + (float)i * RING_BUILD_DUR;
    float prog = (build_time - ring_start) / RING_BUILD_DUR;
    if (prog > 1.0f) prog = 1.0f;
    return prog;
}

/* How many rings a preset actually uses (stops at the first RING_NONE). */
static int preset_ring_count(const MandalaPreset *p) {
    int n = 0;
    for (int i = 0; i < MAX_RINGS; i++) {
        if (p->rings[i].type == RING_NONE) break;
        n++;
    }
    return n;
}

static float preset_build_duration(const MandalaPreset *p) {
    return BINDU_AT + (float)preset_ring_count(p) * RING_BUILD_DUR;
}

/* Biggest mandala radius that fits the screen (after leaving room for the HUD),
 * times the user's size setting. The aspect fudging keeps it round, not oval. */
static float mandala_base_radius(int rows, int cols, float scale) {
    float max_r_x = (float)(cols - SCREEN_MARGIN) * 0.5f;
    float max_r_y = (float)(rows - SCREEN_MARGIN) * 0.5f / ASPECT;
    float base_r  = fminf(max_r_x, max_r_y) * scale;
    if (base_r < MANDALA_MIN_R) base_r = MANDALA_MIN_R;
    return base_r;
}

/* ── §4 data — Scene runtime state ── */

/* All the live state of the program in one struct. The fields fall into a few
 * groups: which mandala is showing, the user's size/spin settings, the build-in
 * animation progress, the chosen colour theme, and an fps meter for the HUD. */
typedef struct {
    /* the mandala on screen */
    int     preset_idx;       /* index into PRESETS[], 0..N_PRESETS-1          */
    /* user-tunable knobs */
    float   scale;            /* size multiplier, SCALE_MIN..SCALE_MAX         */
    bool    rotation_on;      /* slow rotation enabled?                        */
    /* build-in animation + run state */
    float   build_time;       /* seconds into the ring-by-ring reveal          */
    bool    build_complete;   /* true once the whole mandala is drawn           */
    float   rot;              /* current rotation angle, radians               */
    bool    paused;           /* freezes the build + rotation                  */
    /* palette selection */
    int     theme_idx;        /* index into THEME_*, 0..N_THEMES-1             */
    /* fps meter (HUD only) — counts frames over a short window */
    float   fps;              /* last computed frames/sec                      */
    int64_t fps_window_start; /* clock_ns when the current window began         */
    int     frames_in_window; /* frames counted so far this window              */
} Scene;

static Scene g_scene;

/* ── §5 simulation — advance the build animation + rotation ── */

static void scene_tick(Scene *s, float dt) {
    if (s->paused) return;

    /* Run the build-in timer forward until the mandala is fully drawn. */
    if (!s->build_complete) {
        const MandalaPreset *p = &PRESETS[s->preset_idx];
        float dur = preset_build_duration(p);
        s->build_time += dt;
        if (s->build_time >= dur) {
            s->build_time     = dur;
            s->build_complete = true;
        }
    }

    if (s->rotation_on) {
        s->rot += ROT_RATE * dt;
        const float TWO_PI = 2.0f * (float)M_PI;
        while (s->rot >  TWO_PI) s->rot -= TWO_PI;
        while (s->rot < -TWO_PI) s->rot += TWO_PI;
    }
}

/* ── §6 render — colours, ring primitives, draw_mandala, scene_draw ── */

static void color_init(void) {
    use_default_colors();
    init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
    init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
}

static void theme_apply(int idx) {
    init_pair(PAIR_BINDU,   THEME_BINDU  [idx],     -1);
    init_pair(PAIR_CIRCLE,  THEME_PALETTE[idx][0],  -1);
    init_pair(PAIR_DOTS,    THEME_PALETTE[idx][1],  -1);
    init_pair(PAIR_PETALS,  THEME_PALETTE[idx][2],  -1);
    init_pair(PAIR_POLYGON, THEME_PALETTE[idx][3],  -1);
    init_pair(PAIR_STAR,    THEME_PALETTE[idx][4],  -1);
    init_pair(PAIR_RAYS,    THEME_PALETTE[idx][5],  -1);
}

/* Draw one character, but only if it's on screen and not on an HUD row.
 * Rows 0 and the bottom row are left for the HUD bars. */
static void paint_cell(int col, int row, char ch, int pair, int attr) {
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (col < 0 || col >= cols)        return;
    if (row < 1 || row >= rows - 1)    return;
    attron (COLOR_PAIR(pair) | attr);
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(pair) | attr);
}

/* A continuous ring made of many '.' dots spaced around the circle. */
static void draw_circle(int cx, int cy, float r, float progress) {
    if (progress <= 0.0f || r < 0.5f) return;
    int n = (int)(2.0f * (float)M_PI * r * CIRCLE_OVERSAMPLE);
    if (n < CIRCLE_SAMPLES_MIN) n = CIRCLE_SAMPLES_MIN;
    if (n > CIRCLE_SAMPLES_MAX) n = CIRCLE_SAMPLES_MAX;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = feature_angle(i, n, 0.0f);
        int col, row;
        polar_to_cell(cx, cy, r, t, &col, &row);
        paint_cell(col, row, '.', PAIR_CIRCLE, A_NORMAL);
    }
}

/* N evenly-spaced 'o' beads around the ring. */
static void draw_dots(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 1 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = feature_angle(i, n, rot);
        int col, row;
        polar_to_cell(cx, cy, r, t, &col, &row);
        paint_cell(col, row, 'o', PAIR_DOTS, A_BOLD);
    }
}

/* N petals: each is a bold '*' with a dim '.' just inside and just outside,
 * which together read as a little petal shape. */
static void draw_petals(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 1 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = feature_angle(i, n, rot);
        int col, row;
        polar_to_cell(cx, cy, r * PETAL_FLANK_LO, t, &col, &row);
        paint_cell(col, row, '.', PAIR_PETALS, A_DIM);
        polar_to_cell(cx, cy, r * PETAL_FLANK_HI, t, &col, &row);
        paint_cell(col, row, '.', PAIR_PETALS, A_DIM);
        /* centre last so the '*' sits on top of its flanks */
        polar_to_cell(cx, cy, r, t, &col, &row);
        paint_cell(col, row, '*', PAIR_PETALS, A_BOLD);
    }
}

/* Paint a straight line of characters from one cell to another. */
static void draw_line(int x1, int y1, int x2, int y2, int pair, int attr) {
    int dx = x2 - x1, dy = y2 - y1;
    int adx = abs(dx), ady = abs(dy);
    int n_steps = (adx > ady ? adx : ady);
    if (n_steps < 1) n_steps = 1;
    char ch = line_glyph(dx, dy);
    for (int i = 0; i <= n_steps; i++) {
        float t  = (float)i / (float)n_steps;
        int   col = x1 + (int)roundf(t * (float)dx);
        int   row = y1 + (int)roundf(t * (float)dy);
        paint_cell(col, row, ch, pair, attr);
    }
}

/* Draw a polygon or star: put n points around the circle and join each one to
 * the point d steps ahead. d=1 is a plain polygon; bigger d makes a star. When
 * the skip evenly divides n you get several separate shapes (e.g. 6 points with
 * d=2 is two triangles, a hexagram) — which is the intended look. */
static void draw_star_polygon(int cx, int cy, float r, int n, int density,
                              float rot, int pair, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int d = star_stride(density, n);
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t1 = feature_angle(i,           n, rot);
        float t2 = feature_angle((i + d) % n, n, rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, pair, A_BOLD);
    }
}

/* N spokes from a small inner gap out to the rim. */
static void draw_rays(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 1 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = feature_angle(i, n, rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r * RAY_INNER_FRAC, t, &x1, &y1);
        polar_to_cell(cx, cy, r,                  t, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_RAYS, A_BOLD);
    }
}

/* Draw a whole mandala: walk its rings, draw each one as far as its build-in
 * has progressed, then put the centre dot on top. */
static void draw_mandala(const MandalaPreset *p,
                         int cx, int cy, float base_r, float rot,
                         float build_time) {
    for (int i = 0; i < MAX_RINGS; i++) {
        const Ring *ring = &p->rings[i];
        if (ring->type == RING_NONE) break;          /* end of the ring list  */
        float prog = ring_build_progress(i, build_time);
        if (prog <= 0.0f) continue;                  /* this ring's turn hasn't come yet */
        float r = ring->radius * base_r;
        if (r < 0.5f) continue;                      /* too small to draw a cell */
        switch (ring->type) {
            case RING_CIRCLE:  draw_circle      (cx, cy, r,                         prog); break;
            case RING_DOTS:    draw_dots        (cx, cy, r, ring->n,           rot, prog); break;
            case RING_PETALS:  draw_petals      (cx, cy, r, ring->n,           rot, prog); break;
            case RING_POLYGON: draw_star_polygon(cx, cy, r, ring->n, 1,        rot, PAIR_POLYGON, prog); break;
            case RING_STAR:    draw_star_polygon(cx, cy, r, ring->n, ring->density, rot, PAIR_STAR,    prog); break;
            case RING_RAYS:    draw_rays        (cx, cy, r, ring->n,           rot, prog); break;
            default: break;
        }
    }
    if (p->bindu && build_time >= BINDU_AT) {
        paint_cell(cx, cy, '@', PAIR_BINDU, A_BOLD);
    }
}

/* The two HUD bars: a yellow status line top-right and a cyan key legend along
 * the bottom. */
static void draw_hud(const Scene *s, int rows, int cols) {
    const MandalaPreset *p = &PRESETS[s->preset_idx];

    /* What to show for build state: a percent while assembling, else complete
     * or paused. */
    char build_str[24];
    if (s->build_complete) {
        snprintf(build_str, sizeof build_str, "complete");
    } else if (s->paused) {
        snprintf(build_str, sizeof build_str, "PAUSED  ");
    } else {
        float dur = preset_build_duration(p);
        int   pct = (int)(100.0f * s->build_time / dur);
        if (pct < 0)   pct = 0;
        if (pct > 99)  pct = 99;
        snprintf(build_str, sizeof build_str, "build %2d%%", pct);
    }

    /* HUD top-right (yellow A_BOLD) */
    char buf[160];
    snprintf(buf, sizeof buf,
             " %5.1f fps  preset %2d/%d: %-12s  theme: %s  size: %.2f  %s  rot: %s ",
             (double)s->fps,
             s->preset_idx + 1, N_PRESETS,
             PRESETS[s->preset_idx].name,
             THEME_NAME[s->theme_idx],
             (double)s->scale,
             build_str,
             s->rotation_on ? "ON " : "OFF");
    int hud_len = (int)strlen(buf);
    int hud_x   = cols - hud_len;
    if (hud_x < 0) hud_x = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* HUD bottom (cyan A_BOLD) */
    const char *hint =
        " q:quit  n/p:cycle  t:theme  +/-:size  r:rotate  space:pause  b:replay  0:reset ";
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0, "%s", hint);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const Scene *s) {
    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    int cx = cols / 2, cy = rows / 2;

    float base_r = mandala_base_radius(rows, cols, s->scale);
    draw_mandala(&PRESETS[s->preset_idx], cx, cy, base_r, s->rot, s->build_time);
    draw_hud(s, rows, cols);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 init/reset — scene defaults & build restart ── */

/* Start the build-in animation over for the current preset. */
static void scene_restart_build(Scene *s) {
    s->build_time     = 0.0f;
    s->build_complete = false;
}

static void scene_reset(Scene *s) {
    s->preset_idx       = 0;
    s->theme_idx        = 0;
    s->scale            = SCALE_DEFAULT;
    s->rotation_on      = false;
    s->paused           = false;
    s->rot              = 0.0f;
    scene_restart_build(s);
    theme_apply(s->theme_idx);
}

static void scene_init(Scene *s) {
    s->fps               = 0.0f;
    s->fps_window_start  = clock_ns();
    s->frames_in_window  = 0;
    scene_reset(s);
}

/* ── §8 events — keys, signals, screen setup ── */

static void scene_input(Scene *s, int ch) {
    switch (ch) {
        case 'n': case KEY_RIGHT:
            s->preset_idx = (s->preset_idx + 1) % N_PRESETS;
            scene_restart_build(s);
            break;
        case 'p': case KEY_LEFT:
            s->preset_idx = (s->preset_idx + N_PRESETS - 1) % N_PRESETS;
            scene_restart_build(s);
            break;
        case 't': case 'T':
            s->theme_idx = (s->theme_idx + 1) % N_THEMES;
            theme_apply(s->theme_idx);
            break;
        case '+': case '=':
            s->scale = fminf(SCALE_MAX, s->scale + SCALE_STEP); break;
        case '-':
            s->scale = fmaxf(SCALE_MIN, s->scale - SCALE_STEP); break;
        case 'r': case 'R':
            s->rotation_on = !s->rotation_on; break;
        case ' ':
            s->paused = !s->paused; break;
        case 'b': case 'B':
            scene_restart_build(s); break;     /* replay build animation */
        case '0':
            scene_reset(s); break;
        default: break;
    }
}

static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_resize  = 0;

static void on_signal(int sig) {
    if (sig == SIGWINCH) g_resize = 1;
    else                 g_running = 0;
}

static void screen_init(void) {
    initscr();
    noecho();
    cbreak();
    keypad(stdscr, true);
    nodelay(stdscr, true);
    curs_set(0);
    typeahead(-1);
    if (has_colors()) {
        start_color();
        color_init();
    }
}

static void screen_cleanup(void) {
    endwin();
}

/* ── §9 app — the frame loop ── */

int main(void) {
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    screen_init();
    atexit(screen_cleanup);

    scene_init(&g_scene);

    int64_t prev_ns = clock_ns();
    const int64_t frame_ns = NS_PER_SEC / TARGET_FPS;

    while (g_running) {
        int64_t frame_start = clock_ns();
        float dt = (float)(frame_start - prev_ns) / 1e9f;
        if (dt > 0.1f) dt = 0.1f;
        prev_ns = frame_start;

        if (g_resize) {
            g_resize = 0;
            endwin();
            refresh();
        }

        int ch = getch();
        while (ch != ERR) {
            if (ch == 'q' || ch == 'Q' || ch == 27) {
                g_running = 0;
                break;
            }
            scene_input(&g_scene, ch);
            ch = getch();
        }

        scene_tick(&g_scene, dt);
        scene_draw(&g_scene);

        /* fps update ~every FPS_UPDATE_MS */
        g_scene.frames_in_window++;
        int64_t since = frame_start - g_scene.fps_window_start;
        if (since > (int64_t)FPS_UPDATE_MS * 1000000LL) {
            g_scene.fps = (float)g_scene.frames_in_window * 1e9f / (float)since;
            g_scene.fps_window_start = frame_start;
            g_scene.frames_in_window = 0;
        }

        int64_t spent = clock_ns() - frame_start;
        if (spent < frame_ns) clock_sleep_ns(frame_ns - spent);
    }

    return 0;
}
