/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sierpinski.c — draws the Sierpinski triangle, the classic three-cornered fractal.
 *
 * The idea: take a triangle, cut it into four half-size triangles by joining the
 * midpoints of its sides, throw away the middle one (that gap is the hole), and
 * repeat on the three corners. Do this a few times and you get the familiar
 * self-similar gasket. The +/- keys change how many times we repeat.
 *
 * For the cousins that draw fractals by chance instead of by subdivision, see the
 * chaos-game demos barnsley.c and fern.c.
 *
 * Reference: Sierpinski, W. (1915), "Sur une courbe dont tout point est un point
 * de ramification" — the original construction. Peitgen, Jurgens & Saupe, "Chaos
 * and Fractals" (Springer) is the friendliest modern explanation. The triangle
 * filling follows Pineda's edge-function method (SIGGRAPH '88).
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra sierpinski.c -o sierpinski -lncurses -lm
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
    DEPTH_MIN     =   1,
    DEPTH_MAX     =   7,
    DEPTH_DEFAULT =   4,

    GRID_ROWS_MAX =  80,
    GRID_COLS_MAX = 300,

    RENDER_FPS    =  30,     /* idle redraw / input-poll rate */
    N_THEMES      =  10,
};

/*
 * The starting triangle, written as plain coordinates: bottom-left corner, then
 * bottom-right, then the peak in the top-center. It is an equilateral triangle one
 * unit wide; the height 0.866 is just how tall an equilateral triangle of width 1
 * happens to be.
 */
#define V1X  0.0f
#define V1Y  0.0f
#define V2X  1.0f
#define V2Y  0.0f
#define V3X  0.5f
#define V3Y  0.8660254f

/* Terminal characters are about twice as tall as they are wide, so a shape drawn
 * square would look squashed. We stretch the width by this factor to make the
 * triangle look properly equilateral on screen. */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL

/* Blank cells kept around the triangle so the HUD lines at top and bottom have room. */
#define MARGIN_ROWS 3
#define MARGIN_COLS 4

/*
 * A color theme: one bright color for each of the three top-level corners, so the
 * three big sub-triangles stand out from each other (and the pattern repeats that
 * tricolor at every smaller scale). The HUD has its own fixed colors and isn't part
 * of this trio.
 *   c[0] = bottom-left corner, c[1] = bottom-right, c[2] = top.
 */
typedef struct {
    const char *name;
    int c[3];    /* colors for 256-color terminals */
    int c8[3];   /* fallback colors for plain 8-color terminals */
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

/* Names for our ncurses color slots. The three corner colors get swapped out when
 * you change theme; the two HUD colors stay the same no matter the theme. */
typedef enum {
    COL_V1   = 1,   /* bottom-left corner */
    COL_V2   = 2,   /* bottom-right corner */
    COL_V3   = 3,   /* top corner */
    COL_HUD  = 4,   /* HUD readouts — bright yellow */
    COL_HINT = 5,   /* HUD key hints — bright cyan */
} ColorID;

/* Point the three corner color slots at the chosen theme's colors. */
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
    /* The HUD colors never change with the theme: bright yellow readouts, bright cyan hints. */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* start on the first theme */
}

/* ===================================================================== */
/* §4  geometry — the subdivision rule                                    */
/* ===================================================================== */

/* A point on screen. x is the column, y is the row. We keep them as floats (not
 * whole numbers) so that the midpoints stay accurate as we cut the triangle smaller
 * and smaller. */
typedef struct { float x, y; } Pt;

/* A triangle, given by its three corners. This is the one shape everything works on. */
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
 * This is the whole Sierpinski rule in one place. Join the midpoints of the three
 * sides and you've split the triangle into four smaller ones. We hand back the three
 * at the corners and simply never build the middle one — leaving it out is what makes
 * the hole. Every level of the fractal is just this same step applied again.
 */
static void tri_corners(Tri t, Tri out[3])
{
    Pt ab = midpoint(t.a, t.b);
    Pt bc = midpoint(t.b, t.c);
    Pt ca = midpoint(t.c, t.a);
    out[0] = (Tri){ t.a, ab,  ca  };   /* keeps corner a */
    out[1] = (Tri){ ab,  t.b, bc  };   /* keeps corner b */
    out[2] = (Tri){ ca,  bc,  t.c };   /* keeps corner c */
}

/*
 * Tells you which side of the line from a to b the point p is on. The result is
 * positive on one side, negative on the other, zero right on the line. This little
 * test is the standard trick for filling triangles (Pineda's "edge function").
 */
static float edge_side(Pt a, Pt b, Pt p)
{
    return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

/*
 * Is point p inside triangle t (edges count as inside)? It's inside when it sits on
 * the same side of all three edges. We check that the three side-tests never disagree
 * (no mix of positive and negative), which works whichever way the corners are wound.
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
/* §5  layout — where the base triangle sits on screen                    */
/* ===================================================================== */

/*
 * Work out the biggest triangle that fits this terminal, already placed in screen
 * coordinates. We pick the size that fits both the height and the width, center it
 * left-to-right, and sit it near the bottom. Because we build the whole fractal in
 * these screen coordinates, there's no separate math-to-screen conversion later.
 */
static Tri base_triangle(int cols, int rows)
{
    float fit_rows = (float)(rows - MARGIN_ROWS) / V3Y;
    float fit_cols = (float)(cols - MARGIN_COLS) / ASPECT_R;
    float scale_y  = fminf(fit_rows, fit_cols);
    float scale_x  = scale_y * ASPECT_R;

    float left     = (cols - scale_x) * 0.5f;   /* centers it left-to-right */
    float baseline = (float)(rows - 2);         /* the screen row that is the triangle's base */

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
 * The finished picture, stored one cell at a time. Each cell holds a color: 0 means
 * empty, COL_V1..V3 mean a filled triangle of that corner's color. gasket_build()
 * fills this in; the drawing code (§8) only reads it. Filling it is the only slow
 * part of the program, so we redo it only when we have to — never on a quiet frame.
 *
 *   cell  — the grid of colors, [row][col].
 *   rows, cols — how much of that grid is actually in use this run.
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

/* The little rectangle of cells a triangle might cover (kept inside the canvas).
 * Knowing it lets us check just those cells instead of the whole grid.
 *   r0,r1 = top and bottom rows; c0,c1 = left and right columns. */
typedef struct { int r0, r1, c0, c1; } CellBox;

/* Find that surrounding rectangle for triangle t, trimmed to stay on the canvas. */
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

/* Color one cell, but only if it's actually on the canvas. */
static void canvas_mark(Canvas *cv, int row, int col, uint8_t color)
{
    if (row >= 0 && row < cv->rows && col >= 0 && col < cv->cols)
        cv->cell[row][col] = color;
}

/*
 * Paint triangle t onto the canvas in the given color. First we color its center
 * point, so that even a triangle too small to cover a whole cell still leaves a mark.
 * Then we walk the cells around it and color any whose center lands inside.
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
 * Draw triangle t as a Sierpinski gasket, going `levels` deep, all in one color.
 * When there are no levels left we just fill the triangle. Otherwise we split it
 * into its three corners and do the same thing to each, one level shallower.
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
 * Build the whole picture from scratch for the chosen depth. We do the very first
 * split here ourselves so the three big corners can each get their own color; that
 * color then flows down into every smaller triangle in that branch.
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
/* §7  scene — what we're showing right now                               */
/* ===================================================================== */

/* Everything that describes the current view, in one place.
 *   depth  — how many times we split; more depth means more, smaller triangles.
 *   theme  — which color theme is active. Changing it just recolors, no rebuild.
 *   canvas — the picture we've already drawn for the current depth.
 *   dirty  — set when the depth or the window size changed, so we know the picture
 *            is out of date and needs rebuilding before the next frame. */
typedef struct {
    int    depth;
    int    theme;
    Canvas canvas;
    bool   dirty;
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
