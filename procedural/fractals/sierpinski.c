/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sierpinski.c  —  Sierpinski triangle by recursive subdivision
 *
 * Start with one triangle.  Connect its three edge midpoints: that cuts it into
 * four half-size triangles.  Keep the three at the corners, drop the middle one
 * — the dropped middle IS the hole.  Recurse on each corner.  After N levels
 * there are 3^N filled triangles forming the classic self-similar gasket.
 *
 * DEPTH is the knob this demo is built around — it sets how many triangles:
 *   depth 1 →   3      depth 4 →  81      depth 7 → 2187
 *   depth 2 →   9      depth 5 → 243
 *   depth 3 →  27      depth 6 → 729
 * Small depth shows a few big triangles; large depth shows a fine gasket.
 *
 * FOUR LAYERS, separated so the geometry is trustworthy and it is clear what runs
 * apart from drawing:
 *
 *   §4 CORE   — pure geometry: a triangle, and the rule that splits it into three
 *               corner sub-triangles (the dropped central one is the hole).
 *   §5 LAYOUT — where the base triangle sits on screen (the math shape → cells).
 *   §6 RASTER — the cached image: the recursive build paints the gasket into a
 *               cell buffer.  This is the ONLY heavy work and a PURE FUNCTION of
 *               (depth, size) — so it is gated by Scene.dirty and runs only on a
 *               depth or resize change, never per frame.
 *   §8 RENDER — reads the cell buffer and blits glyphs.  Cheap; every frame.
 *   §7 SCENE  — orchestration: depth + theme + canvas + the dirty flag.
 *
 * EFFECTS / DELAYS: there are none.  This is a static explorer — no animation, no
 * reveal, no timer.  Changing depth rebuilds the canvas instantly; changing the
 * theme is a pure recolor (it re-binds palette pairs; the canvas is NOT rebuilt).
 *
 * Color is by which top-level corner a triangle descends from (bottom-left → V1,
 * bottom-right → V2, top → V3), so the three main sub-triangles read as distinct
 * hues and the tricolor repeats self-similarly at every scale.
 *
 * Keys:
 *   q / ESC   quit
 *   + / -     increase / decrease depth (1..7)
 *   r         reset to default depth
 *   t / T     cycle color theme forward / back (10 themes)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra sierpinski.c -o sierpinski -lncurses -lm
 *
 * Sections
 * --------
 *   §1 config   — constants, base-triangle shape, theme table
 *   §2 clock    — monotonic ns clock + sleep
 *   §3 color    — themeable corner palette (t/T) + HUD pairs
 *   §4 geometry — Pt, Tri, midpoint, the subdivision rule (PURE)
 *   §5 layout   — the base triangle placed in screen space (DOMAIN)
 *   §6 raster   — Canvas cell buffer + scan-fill + the recursive gasket build
 *   §7 scene    — orchestration: depth + theme + canvas + dirty flag
 *   §8 render   — canvas → glyphs + HUD (READ-ONLY)
 *   §9 app      — main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm : Recursive subdivision.  tri_corners() splits a triangle into its
 *             three corner sub-triangles (omitting the central hole); recurse to
 *             level 0, then fill.  3^depth leaves.
 *
 * Math      : Self-similar set, Hausdorff dimension D = log 3 / log 2 ≈ 1.585.
 *             Each level halves linear size and triples the count, so filled area
 *             scales as (3/4)^level → 0: zero 2D area, non-zero fractal measure.
 *
 * Rendering : Each leaf triangle is scan-filled by an edge-function inside-test
 *             over its bounding box; a centroid mark keeps sub-cell leaves visible.
 *
 * Alternative view : cell (n,k) of Pascal's triangle is filled iff C(n,k) is odd
 *             — the same gasket "mod 2".
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES — to understand the concepts and the rendering ───────────── *
 *
 *   The gasket & how it is built  (§4 geometry, §6 raster)
 *   ── Sierpiński, W. (1915). "Sur une courbe dont tout point est un point de
 *      ramification." C. R. Acad. Sci. Paris 160, 302–305.  The original
 *      construction of the triangle this whole demo draws.
 *   ── Peitgen, H.-O., Jürgens, H. & Saupe, D. (2004). "Chaos and Fractals"
 *      (2nd ed.). Springer.  The most approachable account of the recursive
 *      construction and the Pascal's-triangle-mod-2 view — best starting point.
 *   ── Barnsley, M. F. (1993). "Fractals Everywhere" (2nd ed.). Academic Press.
 *      The gasket as the attractor of three contraction maps (the IFS / chaos-game
 *      view) — why subdivision and the random game reach the same set.
 *
 *   Why it is a fractal — self-similarity & dimension  (CONCEPTS)
 *   ── Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *      Self-similarity and the gasket as a canonical fractal.
 *   ── Falconer, K. (2003). "Fractal Geometry: Mathematical Foundations and
 *      Applications" (2nd ed.). Wiley.  Rigorous Hausdorff dimension; D = log3/log2.
 *
 *   Rendering  (§6 canvas_fill_tri, §8)
 *   ── Pineda, J. (1988). "A Parallel Algorithm for Polygon Rasterization."
 *      Computer Graphics (SIGGRAPH '88) 22(4), 17–20.  The edge-function /
 *      barycentric triangle fill behind canvas_fill_tri().
 *   ── Gookin, D. (2007). "Programmer's Guide to NCURSES." Wiley.  The cell-
 *      drawing / colour-pair API behind §8.
 * ─────────────────────────────────────────────────────────────────────────── */

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
    DEPTH_MIN     =   1,
    DEPTH_MAX     =   7,
    DEPTH_DEFAULT =   4,

    GRID_ROWS_MAX =  80,
    GRID_COLS_MAX = 300,

    RENDER_FPS    =  30,     /* idle redraw / input-poll rate */
    N_THEMES      =  10,
};

/*
 * Base triangle in math space: V1=bottom-left, V2=bottom-right, V3=top-center.
 * x ∈ [0,1], y ∈ [0, √3/2 ≈ 0.866] — an equilateral triangle.
 */
#define V1X  0.0f
#define V1Y  0.0f
#define V2X  1.0f
#define V2Y  0.0f
#define V3X  0.5f
#define V3Y  0.8660254f   /* √3/2 */

/* ASPECT_R — terminal cell height / width ≈ 2.  Width is stretched by this so the
 * triangle reads as equilateral despite the tall character cells. */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL

/* Margins (in cells) left around the triangle so the HUD has room top and bottom. */
#define MARGIN_ROWS 3
#define MARGIN_COLS 4

/*
 * Theme — one bright color per top-level corner (the HUD has its own
 * theme-independent colors, so it is not part of the trio).
 *   c[0] = COL_V1 (bottom-left sub-triangle)
 *   c[1] = COL_V2 (bottom-right sub-triangle)
 *   c[2] = COL_V3 (top sub-triangle)
 * Each trio is visually distinct so the three sub-triangles read clearly at any
 * scale of the fractal's self-similarity.
 */
typedef struct {
    const char *name;
    int c[3];    /* 256-color   */
    int c8[3];   /* 8-color fallback */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Electric", {  87, 226, 207 }, { COLOR_CYAN,    COLOR_YELLOW,  COLOR_MAGENTA } },
    { "Matrix",   {  46, 118, 231 }, { COLOR_GREEN,   COLOR_GREEN,   COLOR_WHITE   } },
    { "Nova",     {  51,  39, 231 }, { COLOR_CYAN,    COLOR_BLUE,    COLOR_WHITE   } },
    { "Poison",   {  82, 190, 154 }, { COLOR_GREEN,   COLOR_YELLOW,  COLOR_GREEN   } },
    { "Ocean",    {  45,  33,  38 }, { COLOR_CYAN,    COLOR_BLUE,    COLOR_CYAN    } },
    { "Fire",     { 196, 208, 226 }, { COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW  } },
    { "Gold",     { 214, 220, 231 }, { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE   } },
    { "Ice",      { 159, 123, 231 }, { COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE   } },
    { "Nebula",   {  93, 201,  87 }, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN    } },
    { "Lava",     { 196, 214, 208 }, { COLOR_RED,     COLOR_YELLOW,  COLOR_RED     } },
};

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

/* Colour-pair slots.  COL_V1..V3 are the three corner tints, re-bound per theme.
 * COL_HUD / COL_HINT are theme-independent. */
typedef enum {
    COL_V1   = 1,   /* bottom-left  corner tint                          */
    COL_V2   = 2,   /* bottom-right corner tint                          */
    COL_V3   = 3,   /* top          corner tint                          */
    COL_HUD  = 4,   /* HUD data     — bright yellow (theme-independent)  */
    COL_HINT = 5,   /* HUD actions  — bright cyan   (theme-independent)  */
} ColorID;

/* theme_apply — bind the three corner tints to theme t. */
static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    bool truecolor = COLORS >= 256;
    init_pair(COL_V1, truecolor ? th->c[0] : th->c8[0], COLOR_BLACK);
    init_pair(COL_V2, truecolor ? th->c[1] : th->c8[1], COLOR_BLACK);
    init_pair(COL_V3, truecolor ? th->c[2] : th->c8[2], COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    /* HUD pairs stay constant across themes: bright yellow data, bright cyan actions */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* Classic at boot; Scene.theme tracks the active one after */
}

/* ===================================================================== */
/* §4  geometry — the subdivision rule (PURE)                             */
/* ===================================================================== */

/* Pt — a point in screen coordinates (col, row), kept as floats so the midpoints
 * stay exact all the way down the recursion. */
typedef struct { float x, y; } Pt;

/* Tri — the geometric primitive the whole algorithm operates on: a triangle. */
typedef struct { Pt a, b, c; } Tri;

static Pt midpoint(Pt p, Pt q)
{
    return (Pt){ (p.x + q.x) * 0.5f, (p.y + q.y) * 0.5f };
}

static Pt tri_centroid(Tri t)
{
    return (Pt){ (t.a.x + t.b.x + t.c.x) / 3.0f,
                 (t.a.y + t.b.y + t.c.y) / 3.0f };
}

/*
 * tri_corners — THE Sierpinski rule.  Connecting a triangle's edge midpoints cuts
 * it into four congruent half-size triangles; we return the three at the original
 * corners and never form the central one — that omission IS the hole.  Both the
 * top-level split and every deeper level go through this single function, so the
 * fractal's defining step lives in exactly one place.
 */
static void tri_corners(Tri t, Tri out[3])
{
    Pt ab = midpoint(t.a, t.b);
    Pt bc = midpoint(t.b, t.c);
    Pt ca = midpoint(t.c, t.a);
    out[0] = (Tri){ t.a, ab,  ca  };   /* corner at a */
    out[1] = (Tri){ ab,  t.b, bc  };   /* corner at b */
    out[2] = (Tri){ ca,  bc,  t.c };   /* corner at c */
}

/*
 * edge_side — the cross product of edge a→b with a→p, i.e. twice the signed area
 * of triangle (a, b, p).  Its SIGN says which side of the directed edge a→b the
 * point p lies on.  This is Pineda's "edge function" (see REFERENCES), the building
 * block of triangle rasterization.
 */
static float edge_side(Pt a, Pt b, Pt p)
{
    return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

/*
 * point_in_tri — true when p lies inside (or on the border of) triangle t.  p is
 * inside exactly when it is on the same side of all three directed edges — i.e. the
 * three edge functions share a sign.  Testing "not both a positive and a negative"
 * makes it work for either winding (clockwise or counter-clockwise).
 */
static bool point_in_tri(Tri t, Pt p)
{
    float e0 = edge_side(t.a, t.b, p);
    float e1 = edge_side(t.b, t.c, p);
    float e2 = edge_side(t.c, t.a, p);
    bool any_neg = (e0 < 0.0f) || (e1 < 0.0f) || (e2 < 0.0f);
    bool any_pos = (e0 > 0.0f) || (e1 > 0.0f) || (e2 > 0.0f);
    return !(any_neg && any_pos);
}

/* ===================================================================== */
/* §5  layout — where the base triangle sits on screen (DOMAIN)           */
/* ===================================================================== */

/*
 * base_triangle — the largest equilateral triangle that fits the terminal, in
 * screen coordinates.  Picks the scale that fits both vertically (y ∈ [0,V3Y])
 * and horizontally (x ∈ [0,1] after the ASPECT_R width stretch), then centres it
 * left-to-right and seats it near the bottom rows.  The recursion runs entirely
 * in this screen space, so no per-point math→cell conversion is needed later.
 */
static Tri base_triangle(int cols, int rows)
{
    float fit_rows = (float)(rows - MARGIN_ROWS) / V3Y;
    float fit_cols = (float)(cols - MARGIN_COLS) / ASPECT_R;
    float scale_y  = fminf(fit_rows, fit_cols);
    float scale_x  = scale_y * ASPECT_R;

    float left     = (cols - scale_x) * 0.5f;   /* so the triangle is centered */
    float baseline = (float)(rows - 2);         /* screen row of fy = 0        */

    Tri t;
    t.a = (Pt){ left + V1X * scale_x, baseline - V1Y * scale_y };
    t.b = (Pt){ left + V2X * scale_x, baseline - V2Y * scale_y };
    t.c = (Pt){ left + V3X * scale_x, baseline - V3Y * scale_y };
    return t;
}

/* ===================================================================== */
/* §6  raster — cell buffer + scan-fill + the recursive build             */
/* ===================================================================== */

/*
 * Canvas — the cached image: one colour id per cell (0 = empty, COL_V1..V3 = a
 * filled triangle).  It is a pure function of (depth, size): gasket_build() writes
 * it, the renderer (§8) only reads it.  Rebuilding it is the one heavy operation,
 * so the Scene rebuilds only when it must (Scene.dirty), never per frame.
 */
typedef struct {
    uint8_t cell[GRID_ROWS_MAX][GRID_COLS_MAX];
    int     rows, cols;
} Canvas;

static void canvas_resize(Canvas *cv, int cols, int rows)
{
    cv->cols = cols < 1 ? 1 : (cols > GRID_COLS_MAX ? GRID_COLS_MAX : cols);
    cv->rows = rows < 1 ? 1 : (rows > GRID_ROWS_MAX ? GRID_ROWS_MAX : rows);
}

static void canvas_clear(Canvas *cv)
{
    memset(cv->cell, 0, sizeof cv->cell);
}

/* CellBox — the rectangle of cell indices a triangle can touch, clamped to the
 * canvas.  Lets the rasterizer scan a tight region instead of the whole grid. */
typedef struct { int r0, r1, c0, c1; } CellBox;

/* tri_cell_box — triangle t's bounding box in cell indices, clamped to bounds. */
static CellBox tri_cell_box(const Canvas *cv, Tri t)
{
    CellBox b;
    b.c0 = (int)floorf(fminf(t.a.x, fminf(t.b.x, t.c.x)));
    b.c1 = (int)ceilf (fmaxf(t.a.x, fmaxf(t.b.x, t.c.x)));
    b.r0 = (int)floorf(fminf(t.a.y, fminf(t.b.y, t.c.y)));
    b.r1 = (int)ceilf (fmaxf(t.a.y, fmaxf(t.b.y, t.c.y)));
    if (b.c0 < 0) b.c0 = 0;
    if (b.r0 < 0) b.r0 = 0;
    if (b.c1 >= cv->cols) b.c1 = cv->cols - 1;
    if (b.r1 >= cv->rows) b.r1 = cv->rows - 1;
    return b;
}

/* canvas_mark — set one cell if it is in bounds (the rasterizer's only guarded write). */
static void canvas_mark(Canvas *cv, int row, int col, uint8_t color)
{
    if (row >= 0 && row < cv->rows && col >= 0 && col < cv->cols)
        cv->cell[row][col] = color;
}

/*
 * canvas_fill_tri — rasterize triangle t into the buffer in `color`:
 *   1. mark its centroid, so a leaf smaller than one cell still leaves a point;
 *   2. scan its (clamped) cell bounding box, keeping every cell whose centre
 *      falls inside the triangle.
 */
static void canvas_fill_tri(Canvas *cv, Tri t, uint8_t color)
{
    Pt centroid = tri_centroid(t);
    canvas_mark(cv, (int)lroundf(centroid.y), (int)lroundf(centroid.x), color);

    CellBox box = tri_cell_box(cv, t);
    for (int row = box.r0; row <= box.r1; row++) {
        for (int col = box.c0; col <= box.c1; col++) {
            Pt cell_centre = { (float)col + 0.5f, (float)row + 0.5f };
            if (point_in_tri(t, cell_centre))
                cv->cell[row][col] = color;
        }
    }
}

/*
 * gasket_paint — paint triangle t as a Sierpinski gasket `levels` deep, all leaves
 * in `color`.  Level 0 is a leaf (fill it); otherwise apply the subdivision rule
 * and recurse on the three corners one level shallower.
 */
static void gasket_paint(Canvas *cv, Tri t, int levels, uint8_t color)
{
    if (levels == 0) {
        canvas_fill_tri(cv, t, color);
        return;
    }
    Tri corner[3];
    tri_corners(t, corner);
    for (int i = 0; i < 3; i++)
        gasket_paint(cv, corner[i], levels - 1, color);
}

/*
 * gasket_build — THE processing: rebuild the cached image for a given depth.  The
 * first split is done here so the three top-level corners can take distinct hues
 * (COL_V1/2/3); each hue then carries down to all of that branch's leaves.  Total
 * leaves = 3^depth.
 */
static void gasket_build(Canvas *cv, int depth)
{
    canvas_clear(cv);
    Tri corner[3];
    tri_corners(base_triangle(cv->cols, cv->rows), corner);
    for (int i = 0; i < 3; i++)
        gasket_paint(cv, corner[i], depth - 1, (uint8_t)(COL_V1 + i));
}

/* ===================================================================== */
/* §7  scene — orchestration: depth + theme + canvas + dirty              */
/* ===================================================================== */

typedef struct {
    int    depth;    /* recursion depth — the triangle-count knob: 3^depth leaves    */
    int    theme;    /* palette index; changing it is a pure recolor, not a rebuild  */
    Canvas canvas;   /* §6 cached raster of the gasket at `depth`                     */
    bool   dirty;    /* depth or terminal size changed — the canvas must be rebuilt   */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->depth = DEPTH_DEFAULT;
    s->theme = 0;
    canvas_resize(&s->canvas, cols, rows);
    s->dirty = true;
}

/* scene_update — the only non-render processing: rebuild the canvas if (and only
 * if) depth or size changed.  Idle frames skip it entirely. */
static void scene_update(Scene *s)
{
    if (!s->dirty) return;
    gasket_build(&s->canvas, s->depth);
    s->dirty = false;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    canvas_resize(&s->canvas, cols, rows);
    s->dirty = true;
}

/* scene_set_depth — clamp to [DEPTH_MIN, DEPTH_MAX]; mark dirty on a real change. */
static void scene_set_depth(Scene *s, int depth)
{
    if (depth < DEPTH_MIN) depth = DEPTH_MIN;
    if (depth > DEPTH_MAX) depth = DEPTH_MAX;
    if (depth != s->depth) {
        s->depth = depth;
        s->dirty = true;
    }
}

/* scene_cycle_theme — a pure recolor: re-bind the palette, leave the canvas. */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

/* ===================================================================== */
/* §8  render — canvas → glyphs + HUD (READ-ONLY)                         */
/* ===================================================================== */

typedef struct { int rows, cols; } Screen;

/* render_canvas — blit the cached cells as '*'.  Read-only: never writes Canvas. */
static void render_canvas(const Canvas *cv)
{
    for (int row = 0; row < cv->rows; row++) {
        for (int col = 0; col < cv->cols; col++) {
            uint8_t c = cv->cell[row][col];
            if (c == 0) continue;
            attron(COLOR_PAIR((int)c) | A_BOLD);
            mvaddch(row, col, (chtype)'*');
            attroff(COLOR_PAIR((int)c) | A_BOLD);
        }
    }
}

/* hud_line — one HUD line at (row, x), truncated to the screen width so it can
 * never wrap or overrun, however long the string or narrow the terminal. */
static void hud_line(int row, int x, int cols, int pair, attr_t attr, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | attr);
}

/* leaf_count — 3^depth, the number of filled triangles shown. */
static int leaf_count(int depth)
{
    int n = 1;
    for (int i = 0; i < depth; i++) n *= 3;
    return n;
}

/*
 * render_hud — data on top, actions on the bottom:
 *   row 0      (yellow bold)  depth, triangle count (3^depth), theme
 *   row rows-1 (cyan bold)    every interactive key
 */
static void render_hud(const Screen *s, const Scene *sc)
{
    char buf[96];
    snprintf(buf, sizeof buf, " Sierpinski  depth:%d  triangles:%d  theme:%s ",
             sc->depth, leaf_count(sc->depth), k_themes[sc->theme].name);
    hud_line(0, 0, s->cols, COL_HUD, A_BOLD, buf);

    hud_line(s->rows - 1, 0, s->cols, COL_HINT, A_BOLD,
             " q:quit  +/-:depth  r:reset  t:theme ");
}

/* ===================================================================== */
/* §9  app — main loop                                                    */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;       /* cleared by SIGINT/SIGTERM or 'q' */
    volatile sig_atomic_t need_resize;   /* set by SIGWINCH                  */
} App;

/* The one global: signal handlers receive only an int, so the flags they set must
 * be reachable without a parameter (volatile sig_atomic_t).  Everything else is
 * reached through this object. */
static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

static void terminal_init(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);     /* getch() returns ERR instead of blocking */
    keypad(stdscr, TRUE);
    typeahead(-1);             /* don't let pending input interrupt the diff write */
    color_init();
}

/* app_apply_resize — after SIGWINCH: rebuild ncurses' screen model, re-read the
 * size, and mark the canvas stale so the next frame rebuilds at the new size. */
static void app_apply_resize(App *app)
{
    endwin();
    refresh();
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/* app_handle_key — translate one keypress into a scene action; false = quit. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case '+': case '=': scene_set_depth(&app->scene, app->scene.depth + 1); break;
    case '-': case '_': scene_set_depth(&app->scene, app->scene.depth - 1); break;

    case 'r': case 'R': scene_set_depth(&app->scene, DEPTH_DEFAULT); break;

    case 't': scene_cycle_theme(&app->scene, +1); break;
    case 'T': scene_cycle_theme(&app->scene, -1); break;

    default: break;
    }
    return true;
}

static void app_drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR)
        if (!app_handle_key(app, ch)) { app->running = 0; break; }
}

/* app_present — compose one frame: clear, blit the cached canvas, overlay the HUD,
 * and flush as a single terminal diff. */
static void app_present(App *app)
{
    erase();
    render_canvas(&app->scene.canvas);
    render_hud(&app->screen, &app->scene);
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    terminal_init();

    App *app     = &g_app;
    app->running = 1;
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    while (app->running) {
        if (app->need_resize) app_apply_resize(app);

        int64_t frame_start = clock_ns();

        app_drain_input(app);          /* 1. input   → depth/theme edits (sets dirty) */
        scene_update(&app->scene);     /* 2. process → rebuild the canvas if dirty    */
        app_present(app);              /* 3. render  → blit cached canvas + HUD       */

        clock_sleep_ns(NS_PER_SEC / RENDER_FPS - (clock_ns() - frame_start));
    }
    return 0;
}
