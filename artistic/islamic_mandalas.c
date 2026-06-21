/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * islamic_mandalas.c — 30 parametric Islamic geometric patterns drawn in the
 * terminal. Each pattern is a stack of concentric "rings", and each ring is one
 * of six simple shapes (circle, polygon, star, outlined star, interlocking
 * polygons, rays). Refs: Critchlow, "Islamic Patterns" (1976); Bourgoin,
 * "Arabic Geometrical Pattern and Design" (1973); Lu & Steinhardt, Science 315
 * (2007) for the near-quasicrystal star patterns.
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

/* ── §1 config — presets, ring/preset types, constants, themes ── */

enum {
    TARGET_FPS    = 60,
    FPS_UPDATE_MS = 500,

    MAX_RINGS = 8,    /* enough for complex presets (some use 7) */
    N_PRESETS = 30,
    N_THEMES  = 4,

    /* Color pair IDs */
    PAIR_CENTRE     = 1,
    PAIR_CIRCLE     = 2,
    PAIR_POLYGON    = 3,
    PAIR_STAR_POLY  = 4,
    PAIR_STAR_SHAPE = 5,
    PAIR_INTERLOCK  = 6,
    PAIR_RAYS       = 7,
    PAIR_HUD        = 8,
    PAIR_HINT       = 9,
};

#define ASPECT        0.5f       /* terminal cells ~2:1 tall:wide */
#define ROT_RATE      0.18f      /* rad/sec when rotation enabled */
#define SCALE_MIN     0.40f
#define SCALE_MAX     1.00f
#define SCALE_DEFAULT 0.85f
#define SCALE_STEP    0.05f
#define NS_PER_SEC    1000000000LL

#define BINDU_AT       0.10f      /* seconds before the centre dot appears */
#define RING_BUILD_DUR 0.55f      /* seconds each ring takes to reveal      */

/* Primitive geometry */
#define CIRCLE_OVERSAMPLE  1.4f   /* '.' samples per cell of circle circumference */
#define CIRCLE_SAMPLES_MIN 24     /* clamp so small circles still read as a ring  */
#define CIRCLE_SAMPLES_MAX 360    /* and large ones don't oversample pointlessly  */
#define STAR_INNER_MIN     0.10f  /* outlined-star inner radius ratio: floor       */
#define STAR_INNER_MAX     0.95f  /*   (keeps points from collapsing / flattening) */
#define RAY_INNER_FRAC     0.10f  /* rays start this far out → a centre gap        */

/* Layout */
#define SCREEN_MARGIN      4      /* cells kept clear around the pattern (HUD rows) */
#define MANDALA_MIN_R      2.0f   /* never shrink the pattern below this radius     */

/* ── §1.1 Ring + Preset types ── */

/* The six shapes a ring can be. A pattern is just a stack of these laid out
 * around one centre; mixing a few at different radii reproduces the classic
 * tile motifs. RING_NONE is the "stop here" marker that ends a ring list (its
 * value is 0, so a {0}-padded array naturally ends a short preset).
 *   RING_CIRCLE     : a dotted ring drawn with '.'
 *   RING_POLYGON    : a regular n-sided shape (corners joined edge to edge)
 *   RING_STAR_POLY  : a "line star" — vertices joined by skipping d of them
 *                     each step (octagram, dodecagram); see Coxeter
 *   RING_STAR_SHAPE : a real outlined star with sharp points — alternates
 *                     outer vertices and shorter inner vertices
 *   RING_INTERLOCK  : two polygons, the second rotated half a step, giving a
 *                     2n-pointed star (4+4 = Khatim 8-star, 3+3 = hexagram)
 *   RING_RAYS       : N spokes from an inner gap out to the rim (sunburst) */
typedef enum {
    RING_NONE = 0, RING_CIRCLE, RING_POLYGON, RING_STAR_POLY,
    RING_STAR_SHAPE, RING_INTERLOCK, RING_RAYS,
} RingType;

/* One ring of a pattern: which shape, how many of it, one extra knob, and how
 * far out. Just four fields, because that fully describes a ring. The `density`
 * field means different things per shape — a deliberate trick so every preset
 * fits on one line:
 *   STAR_POLY  : the skip stride d — join each vertex to the one d ahead
 *                (d=1 is a plain polygon, d>=2 makes a star).
 *   STAR_SHAPE : inner radius as a percent of the outer (0..100), so 50 puts
 *                the inner points halfway in (smaller percent = sharper points).
 *   others     : unused, left 0.
 *   type    : one of the RingType values above.
 *   n       : how many vertices / points / rays (0 for a circle, which picks
 *             its own dot count from its radius at draw time).
 *   density : the per-shape knob described above.
 *   radius  : a fraction of the pattern's base radius, in (0, 1]. */
typedef struct {
    int   type;
    int   n;
    int   density;
    float radius;
} Ring;

/* One named pattern, written as a recipe: a centre flag plus a list of rings.
 * This is the whole idea of the file — "thirty different patterns" is really one
 * draw routine fed thirty different ring lists. Ring order runs inner to outer,
 * and that same order drives the build-in animation (ring 0 appears first).
 *   name       : label shown in the HUD.
 *   rings      : up to MAX_RINGS of them; the list ends at the first RING_NONE,
 *                so short presets just leave the rest zeroed.
 *   centre_dot : draw a '+' in the middle (painted last so it always shows);
 *                off for star-only patterns. */
typedef struct {
    const char *name;
    Ring rings[MAX_RINGS];
    bool centre_dot;
} MandalaPreset;

/* Shorthand so each preset fits on one line. The params are CC/DD/RR rather
 * than n/d/r because the preprocessor would otherwise rewrite the field names
 * inside the .n/.density/.radius designators below. */
#define R(t,CC,DD,RR) {.type = RING_##t, .n = (CC), .density = (DD), .radius = (RR)}

static const MandalaPreset PRESETS[N_PRESETS] = {
/* 20 simple named patterns */
    {"Khatim",          {R(INTERLOCK,    4, 0,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Hexagram",        {R(INTERLOCK,    3, 0,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Octagram",        {R(STAR_POLY,    8, 3,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Decagram",        {R(STAR_POLY,   10, 3,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Dodecagram",      {R(STAR_POLY,   12, 5,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Rub-el-hizb",     {R(INTERLOCK,    4, 0,  0.55f), R(POLYGON,       8,  0, 0.78f), R(CIRCLE,        0,  0, 0.92f)},                                                  true},
    {"Outlined 5-Star", {R(STAR_SHAPE,   5, 38, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  false},
    {"Outlined 8-Star", {R(STAR_SHAPE,   8, 50, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  false},
    {"Outlined 12-Star",{R(STAR_SHAPE,  12, 55, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  false},
    {"Persian 8-Fold",  {R(INTERLOCK,    4, 0,  0.45f), R(STAR_SHAPE,    8, 50, 0.70f), R(POLYGON,       8,  0, 0.85f), R(CIRCLE,        0,  0, 0.92f)},                  true},
    {"Iznik Medallion", {R(STAR_POLY,    8, 3,  0.45f), R(STAR_SHAPE,   16, 60, 0.72f), R(POLYGON,      16,  0, 0.88f)},                                                  false},
    {"Andalusian",      {R(INTERLOCK,    4, 0,  0.45f), R(STAR_POLY,     8,  3, 0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},
    {"Selcuk",          {R(STAR_SHAPE,  10, 40, 0.50f), R(STAR_SHAPE,   10, 60, 0.78f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},
    {"Mamluk Burst",    {R(RAYS,        16, 0,  0.70f), R(STAR_POLY,    16,  5, 0.55f), R(CIRCLE,        0,  0, 0.85f)},                                                  false},
    {"Tabriz Compass",  {R(RAYS,         8, 0,  0.85f), R(RAYS,         16,  0, 0.85f), R(STAR_SHAPE,    8, 45, 0.55f), R(CIRCLE,        0,  0, 0.92f)},                  true},
    {"Samarkand Sun",   {R(RAYS,        24, 0,  0.85f), R(STAR_POLY,    12,  5, 0.55f), R(CIRCLE,        0,  0, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                  false},
    {"Cairo Tile",      {R(STAR_POLY,   12, 5,  0.55f), R(INTERLOCK,     6,  0, 0.30f), R(CIRCLE,        0,  0, 0.85f)},                                                  false},
    {"Maghrebi Knot",   {R(STAR_POLY,    5, 2,  0.40f), R(STAR_SHAPE,   10, 50, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},
    {"Alhambra Star",   {R(STAR_SHAPE,  16, 55, 0.70f), R(POLYGON,      16,  0, 0.85f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},
    {"Mosque Window",   {R(POLYGON,      6, 0,  0.85f), R(STAR_POLY,     6,  2, 0.55f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},

/* 10 complex patterns, 5-7 rings each */
    {"Topkapi Scroll",        {R(INTERLOCK,   5,  0, 0.30f), R(STAR_SHAPE,  10, 50, 0.50f), R(STAR_POLY,   10,  3, 0.65f), R(POLYGON,     10,  0, 0.85f), R(CIRCLE,        0,  0, 0.92f)},                                                                          true},
    {"Konya Rosette",         {R(STAR_POLY,  12,  5, 0.30f), R(STAR_SHAPE,  12, 55, 0.50f), R(INTERLOCK,    6,  0, 0.65f), R(POLYGON,     12,  0, 0.82f), R(RAYS,        24,  0, 0.92f), R(CIRCLE,        0,  0, 0.95f)},                                            false},
    {"Damascus Dome",         {R(STAR_POLY,  16,  7, 0.30f), R(STAR_SHAPE,  16, 60, 0.50f), R(INTERLOCK,    8,  0, 0.65f), R(POLYGON,     16,  0, 0.82f), R(CIRCLE,        0,  0, 0.88f), R(CIRCLE,        0,  0, 0.94f)},                                          false},
    {"Cordoba Mihrab",        {R(STAR_SHAPE,  8, 45, 0.30f), R(STAR_POLY,    8,  3, 0.50f), R(INTERLOCK,    4,  0, 0.65f), R(POLYGON,      8,  0, 0.80f), R(RAYS,        16,  0, 0.92f), R(CIRCLE,        0,  0, 0.95f)},                                            true},
    {"Isfahan Garden",        {R(STAR_POLY,  12,  5, 0.28f), R(STAR_SHAPE,  12, 55, 0.46f), R(INTERLOCK,    6,  0, 0.60f), R(POLYGON,     12,  0, 0.74f), R(STAR_SHAPE,  24, 65, 0.85f), R(RAYS,        24,  0, 0.92f), R(CIRCLE,        0,  0, 0.96f)},              false},
    {"Cairo Stellate",        {R(INTERLOCK,   6,  0, 0.30f), R(STAR_POLY,   12,  5, 0.50f), R(STAR_SHAPE,  12, 55, 0.65f), R(RAYS,        12,  0, 0.85f), R(POLYGON,     12,  0, 0.85f), R(CIRCLE,        0,  0, 0.92f)},                                            false},
    {"Granada Constellation", {R(STAR_SHAPE, 16, 60, 0.30f), R(STAR_POLY,   16,  7, 0.50f), R(INTERLOCK,    8,  0, 0.65f), R(POLYGON,     16,  0, 0.78f), R(RAYS,        32,  0, 0.88f), R(CIRCLE,        0,  0, 0.94f)},                                            false},
    {"Bursa Mosque",          {R(STAR_POLY,  10,  3, 0.30f), R(STAR_SHAPE,  10, 50, 0.50f), R(INTERLOCK,    5,  0, 0.65f), R(POLYGON,     10,  0, 0.78f), R(RAYS,        20,  0, 0.88f), R(CIRCLE,        0,  0, 0.94f)},                                            false},
    {"Marrakesh Tile",        {R(STAR_POLY,   8,  3, 0.25f), R(STAR_POLY,   16,  7, 0.42f), R(STAR_SHAPE,   8, 45, 0.55f), R(STAR_SHAPE, 16, 60, 0.72f), R(INTERLOCK,    4,  0, 0.85f), R(POLYGON,     16,  0, 0.92f), R(CIRCLE,        0,  0, 0.95f)},              false},
    {"Quasi-Crystal 10",      {R(STAR_POLY,  10,  3, 0.25f), R(STAR_POLY,   10,  4, 0.40f), R(STAR_SHAPE,  10, 45, 0.55f), R(STAR_SHAPE, 20, 65, 0.72f), R(INTERLOCK,    5,  0, 0.85f), R(RAYS,        20,  0, 0.92f), R(CIRCLE,        0,  0, 0.95f)},               false},
};

/* ── §1.2 Themes — one colour per ring shape, plus the centre marker ── */
static const int THEME_PALETTE[N_THEMES][6] = {
    /*           CIRCLE POLY  S_POLY S_SHAPE INTER  RAYS                  */
    /* IZNIK    */ {  39,   45,   51,   75,  117,   45 }, /* turquoise + cobalt */
    /* PERSIAN  */ {  27,   33,   75,  117,  159,   33 }, /* deep blue + ivory */
    /* ANDALUS  */ { 118,  154,   46,  220,  214,  154 }, /* green + gold */
    /* MAMLUK   */ { 220,  214,  208,  202,  166,  220 }, /* gold + red */
};
static const int   THEME_CENTRE[N_THEMES] = { 195, 230, 230, 230 };
static const char *THEME_NAME  [N_THEMES] = { "Iznik", "Persian", "Andalusian", "Mamluk" };

/* ── §2 performance — monotonic clock and sleep ── */

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

/* ── §3 logic — pure helpers: polar-to-cell, glyphs, build timing ── */

/* Turn an angle and radius into a screen cell. Cells are about twice as tall as
 * wide, so the vertical part is squashed by ASPECT to keep circles looking
 * round instead of stretched. */
static inline void polar_to_cell(int cx, int cy, float r, float theta,
                                 int *col, int *row) {
    *col = cx + (int)roundf(r * cosf(theta));
    *row = cy + (int)roundf(r * sinf(theta) * ASPECT);
}

/* Angle of feature i when n of them are spread evenly around the circle, offset
 * by the current rotation. */
static inline float feature_angle(int i, int n, float rot) {
    return rot + (float)i / (float)n * 2.0f * (float)M_PI;
}

/* The skip stride for a line star: 1 (plain polygon) unless 2 <= d < n. */
static int star_stride(int density, int n) {
    int d = (density < 2) ? 1 : density;
    if (d >= n) d = 1;
    return d;
}

/* Inner radius of an outlined star as a fraction of the outer, kept in a sane
 * range so the points don't collapse to the centre or flatten into the rim. */
static float star_inner_ratio(int inner_pct) {
    float ratio = (float)inner_pct / 100.0f;
    if (ratio < STAR_INNER_MIN) ratio = STAR_INNER_MIN;
    if (ratio > STAR_INNER_MAX) ratio = STAR_INNER_MAX;
    return ratio;
}

/* Pick the ASCII character that best matches a line's direction: | for steep,
 * - for shallow, / or \ for diagonal. */
static char line_glyph(int dx, int dy) {
    float adx = (float)abs(dx);
    float ady = (float)abs(dy) / ASPECT;
    if (adx < 0.5f) return '|';
    if (ady < 0.5f) return '-';
    float r = ady / adx;
    if (r < 0.5f) return '-';
    if (r > 2.0f) return '|';
    return ((dx > 0) == (dy > 0)) ? '\\' : '/';
}

/* How many of n features to draw given build progress 0..1. The near-1 and
 * near-0 checks dodge float rounding so a finished build shows every feature.
 * (See hindu_mandalas.c for the same fix.) */
static int progress_to_count(int n, float progress) {
    if (progress >= 0.999f) return n;
    if (progress <= 0.0f)   return 0;
    int k = (int)(progress * (float)n);
    if (k < 1) k = 1;
    if (k > n) k = n;
    return k;
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

/* How far along ring i's reveal is, from 0 to 1. Each ring gets its own slice
 * of the build timeline; this returns <= 0 before ring i's slice starts. */
static float ring_build_progress(int i, float build_time) {
    float ring_start = BINDU_AT + (float)i * RING_BUILD_DUR;
    float prog = (build_time - ring_start) / RING_BUILD_DUR;
    if (prog > 1.0f) prog = 1.0f;
    return prog;
}

/* Largest radius that still fits on screen (after leaving room for the HUD),
 * times the user's size knob. The /ASPECT keeps the pattern round, not oval. */
static float mandala_base_radius(int rows, int cols, float scale) {
    float max_r_x = (float)(cols - SCREEN_MARGIN) * 0.5f;
    float max_r_y = (float)(rows - SCREEN_MARGIN) * 0.5f / ASPECT;
    float base_r  = fminf(max_r_x, max_r_y) * scale;
    if (base_r < MANDALA_MIN_R) base_r = MANDALA_MIN_R;
    return base_r;
}

/* ── §4 data — the whole runtime state in one struct ── */

/* Everything the program needs to know at runtime, grouped by what it is for:
 * which pattern, the user's knobs, the running animation, the colour theme, and
 * a frame-rate counter for the HUD. */
typedef struct {
    /* which pattern is showing */
    int     preset_idx;       /* index into PRESETS[], 0..N_PRESETS-1          */
    /* user knobs */
    float   scale;            /* size, SCALE_MIN..SCALE_MAX                    */
    bool    rotation_on;      /* slow spin on?                                */
    /* the build-in animation and run state */
    float   build_time;       /* seconds into the ring-by-ring reveal          */
    bool    build_complete;   /* true once the whole pattern has revealed      */
    float   rot;              /* current spin angle, radians                   */
    bool    paused;           /* freezes both the build and the spin           */
    /* colour */
    int     theme_idx;        /* index into THEME_*, 0..N_THEMES-1             */
    /* frame-rate meter, shown in the HUD only */
    float   fps;              /* last measured frames per second               */
    int64_t fps_window_start; /* clock reading when this measuring window began */
    int     frames_in_window; /* frames drawn since that point                 */
} Scene;

static Scene g_scene;

/* ── §5 simulation — advance the build-in animation and the spin ── */

static void scene_tick(Scene *s, float dt) {
    if (s->paused) return;

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

/* ── §6 render — colours, shape primitives, draw_mandala, scene_draw ── */

static void color_init(void) {
    use_default_colors();
    init_pair(PAIR_HUD,  226, -1);
    init_pair(PAIR_HINT,  51, -1);
}

static void theme_apply(int idx) {
    init_pair(PAIR_CENTRE,     THEME_CENTRE [idx],     -1);
    init_pair(PAIR_CIRCLE,     THEME_PALETTE[idx][0],  -1);
    init_pair(PAIR_POLYGON,    THEME_PALETTE[idx][1],  -1);
    init_pair(PAIR_STAR_POLY,  THEME_PALETTE[idx][2],  -1);
    init_pair(PAIR_STAR_SHAPE, THEME_PALETTE[idx][3],  -1);
    init_pair(PAIR_INTERLOCK,  THEME_PALETTE[idx][4],  -1);
    init_pair(PAIR_RAYS,       THEME_PALETTE[idx][5],  -1);
}

/* Draw one character, skipping anything off-screen. The top and bottom rows are
 * left alone because the HUD lives there. */
static void paint_cell(int col, int row, char ch, int pair, int attr) {
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (col < 0 || col >= cols)        return;
    if (row < 1 || row >= rows - 1)    return;
    attron (COLOR_PAIR(pair) | attr);
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(pair) | attr);
}

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

/* A dotted ring: scatter many '.' evenly around the circle. */
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

/* A regular n-sided shape: join each corner to the next. */
static void draw_polygon(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t1 = feature_angle(i,           n, rot);
        float t2 = feature_angle((i + 1) % n, n, rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_POLYGON, A_BOLD);
    }
}

/* A "line star": join each vertex to the one d steps ahead, not the next-door
 * one. That skipping is what makes the star shape (octagram, dodecagram). */
static void draw_star_poly(int cx, int cy, float r, int n, int density,
                           float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int d = star_stride(density, n);
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t1 = feature_angle(i,           n, rot);
        float t2 = feature_angle((i + d) % n, n, rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_STAR_POLY, A_BOLD);
    }
}

/* A real outlined star with sharp points: walk around alternating between far-
 * out points and pulled-in dips. inner_pct says how far in the dips sit (0..100
 * percent of the outer radius). */
static void draw_star_shape(int cx, int cy, float r, int n, int inner_pct,
                            float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    float r_in = r * star_inner_ratio(inner_pct);
    int total_edges = 2 * n;
    int n_done = progress_to_count(total_edges, progress);

    for (int e = 0; e < n_done; e++) {
        int v1 = e;
        int v2 = (e + 1) % total_edges;
        bool v1_outer = (v1 % 2 == 0);
        bool v2_outer = (v2 % 2 == 0);
        float t1 = feature_angle(v1, total_edges, rot);
        float t2 = feature_angle(v2, total_edges, rot);
        float radius1 = v1_outer ? r : r_in;
        float radius2 = v2_outer ? r : r_in;
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, radius1, t1, &x1, &y1);
        polar_to_cell(cx, cy, radius2, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_STAR_SHAPE, A_BOLD);
    }
}

/* Two copies of the same polygon, the second turned half a step, so together
 * they make a 2n-pointed star. Two squares give the Khatim 8-star, two triangles
 * give the hexagram. The first polygon reveals before the second. */
static void draw_interlock(int cx, int cy, float r, int n,
                           float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int total_edges = 2 * n;
    int n_done = progress_to_count(total_edges, progress);

    for (int e = 0; e < n_done; e++) {
        bool first = (e < n);
        int  edge  = first ? e : (e - n);
        float poly_rot = rot + (first ? 0.0f : ((float)M_PI / (float)n));
        float t1 = feature_angle(edge,           n, poly_rot);
        float t2 = feature_angle((edge + 1) % n, n, poly_rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_INTERLOCK, A_BOLD);
    }
}

/* A sunburst: N spokes from a small inner gap out to the rim. */
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

/* Draw a whole pattern: walk its rings, drawing each one as far as the build
 * timer has revealed, then stamp the centre marker last. */
static void draw_mandala(const MandalaPreset *p,
                         int cx, int cy, float base_r, float rot,
                         float build_time) {
    for (int i = 0; i < MAX_RINGS; i++) {
        const Ring *ring = &p->rings[i];
        if (ring->type == RING_NONE) break;          /* end of the ring list */
        float prog = ring_build_progress(i, build_time);
        if (prog <= 0.0f) continue;                  /* this ring hasn't started revealing yet */
        float r = ring->radius * base_r;
        if (r < 0.5f) continue;                      /* too small to see */
        switch (ring->type) {
            case RING_CIRCLE:     draw_circle    (cx, cy, r,                       prog); break;
            case RING_POLYGON:    draw_polygon   (cx, cy, r, ring->n,         rot, prog); break;
            case RING_STAR_POLY:  draw_star_poly (cx, cy, r, ring->n, ring->density, rot, prog); break;
            case RING_STAR_SHAPE: draw_star_shape(cx, cy, r, ring->n, ring->density, rot, prog); break;
            case RING_INTERLOCK:  draw_interlock (cx, cy, r, ring->n,         rot, prog); break;
            case RING_RAYS:       draw_rays      (cx, cy, r, ring->n,         rot, prog); break;
            default: break;
        }
    }
    if (p->centre_dot && build_time >= BINDU_AT) {
        paint_cell(cx, cy, '+', PAIR_CENTRE, A_BOLD);
    }
}

/* The on-screen readout: a data line at the top and the key legend at the bottom. */
static void draw_hud(const Scene *s, int rows, int cols) {
    const MandalaPreset *p = &PRESETS[s->preset_idx];

    /* what to show for build state: a percent while assembling, else done/paused */
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

    char buf[160];
    snprintf(buf, sizeof buf,
             " %5.1f fps  preset %2d/%d: %-16s  theme: %-10s  size: %.2f  %s  rot: %s ",
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

/* ── §7 init/reset — scene defaults and restarting the build animation ── */

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

/* ── §8 events — key handling, signals, screen setup ── */

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
            scene_restart_build(s); break;
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

/* ── §9 app — the main frame loop ── */

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
        if (dt > 0.1f) dt = 0.1f;   /* cap so a stalled frame can't make the animation jump */
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
