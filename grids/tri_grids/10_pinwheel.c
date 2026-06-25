/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 10_pinwheel.c — a 1-2-√5 right triangle split into 5 similar copies, again
 * and again, into a pinwheel of nested rotated triangles. +/- adds/removes
 * layers of detail. Loosely inspired by the pinwheel tiling (children rotate
 * but, unlike the strict version, are not placed irrationally — see §5).
 *
 * Sister files: 01_equilateral.c (shared GridCtx/clock/screen skeleton),
 *               08_triforce.c (the 4-way split this builds on),
 *               12_penrose.c (another never-repeating tiling).
 * Refs: pinwheel tiling — https://en.wikipedia.org/wiki/Pinwheel_tiling
 *       Radin, "The Pinwheel Tilings of the Plane" (1994)
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

#define TARGET_FPS 60

#define CELL_W 2     /* sub-pixels per cell (width, height) */
#define CELL_H 4

#define DEPTH_DEFAULT 3
#define DEPTH_MIN     0
#define DEPTH_MAX     6

#define SIZE_FRAC_DEFAULT 0.85   /* fraction of the screen the figure fills */
#define SIZE_FRAC_MIN     0.30
#define SIZE_FRAC_MAX     0.95
#define SIZE_FRAC_STEP    0.05

#define MAX_DEPTH_LEVELS (DEPTH_MAX + 1)
#define N_THEMES         3

#define FPS_EWMA_ALPHA 0.05   /* small = steadier on-screen fps number */

#define PAIR_DEPTH_BASE  1                          /* one pair per split level */
#define PAIR_HUD        (PAIR_DEPTH_BASE + MAX_DEPTH_LEVELS)
#define PAIR_HINT       (PAIR_HUD + 1)

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

static const short PAL256[N_THEMES][MAX_DEPTH_LEVELS] = {
    /* spin   */ { 15, 39, 82, 226, 207, 196, 21 },
    /* paper  */ { 15, 21, 39, 82,  226, 196, 207 },
    /* mono   */ { 15, 15, 15, 15,  15,  15,  15 },
};
static const short PAL8[N_THEMES][MAX_DEPTH_LEVELS] = {
    { COLOR_WHITE, COLOR_CYAN,    COLOR_GREEN,  COLOR_YELLOW,
      COLOR_MAGENTA, COLOR_RED,   COLOR_BLUE },
    { COLOR_WHITE, COLOR_BLUE,    COLOR_CYAN,   COLOR_GREEN,
      COLOR_YELLOW, COLOR_RED,    COLOR_MAGENTA },
    { COLOR_WHITE, COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE,
      COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < MAX_DEPTH_LEVELS; i++) {
        short fg = (COLORS >= 256) ? PAL256[theme][i] : PAL8[theme][i];
        init_pair(PAIR_DEPTH_BASE + i, fg, -1);
    }
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 screen anchor & line drawing ── */

/* GridCtx — screen geometry plus the two live knobs (depth, size_frac). Built
 * fresh each frame so it tracks the terminal size. (Same skeleton as the other
 * tri_grids files.) */
typedef struct {
    int    rows, cols;   /* terminal size, in character cells */
    int    cw, ch;       /* one cell in sub-pixels (CELL_W, CELL_H) */
    int    ox, oy;       /* sub-pixel coords of screen centre = figure anchor */
    int    depth;        /* split levels to draw (more = finer detail) */
    double size_frac;    /* fraction of the screen the figure fills, 0..1 */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, int depth, double size_frac)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = (cols * CELL_W) / 2;
    g->oy = ((rows - 1) * CELL_H) / 2;
    g->depth     = depth;
    g->size_frac = size_frac;
}

/* pick the ASCII glyph that best matches a segment's slope */
static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

/* Bresenham line in sub-pixel space, clipped to the screen */
static void line_draw(const GridCtx *g, double px0, double py0,
                      double px1, double py1, int attr)
{
    char ch = slope_char(px1 - px0, py1 - py0);
    int x0 = (int)(px0 / CELL_W), y0 = (int)(py0 / CELL_H);
    int x1 = (int)(px1 / CELL_W), y1 = (int)(py1 / CELL_H);
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    attron(attr);
    for (;;) {
        if (x0 >= 0 && x0 < g->cols && y0 >= 0 && y0 < g->rows - 1)
            mvaddch(y0, x0, (chtype)(unsigned char)ch);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    attroff(attr);
}

/* ── §5 pinwheel subdivision (the distinct math) ── */

/* Tri — one triangle by its three corners (sub-pixel screen coords). Corner
 * roles are fixed so every child can be split the same way: 0 = sharp tip,
 * 1 = right-angle corner, 2 = other sharp tip. Children inherit the labelling,
 * which keeps the recursion uniform all the way down. */
typedef struct { double x[3], y[3]; } Tri;

/* drop P onto line A-B: the closest point F on the line. Used to find where a
 * perpendicular from the middle triangle's square corner meets its long side —
 * the cut that turns the leftover 4th piece into 2 children. */
static void foot_perp(double Px, double Py,
                      double Ax, double Ay, double Bx, double By,
                      double *Fx, double *Fy)
{
    double dx = Bx - Ax, dy = By - Ay;
    double len2 = dx*dx + dy*dy;
    double tt = ((Px - Ax) * dx + (Py - Ay) * dy) / len2;
    *Fx = Ax + tt * dx;
    *Fy = Ay + tt * dy;
}

static void tri_draw_edges(const GridCtx *g, Tri t, int depth)
{
    int attr = COLOR_PAIR(PAIR_DEPTH_BASE + depth) | (depth == 0 ? A_BOLD : 0);
    line_draw(g, t.x[0], t.y[0], t.x[1], t.y[1], attr);
    line_draw(g, t.x[1], t.y[1], t.x[2], t.y[2], attr);
    line_draw(g, t.x[2], t.y[2], t.x[0], t.y[0], attr);
}

/* split one 1-2-√5 triangle into its 5 similar children. The three edge
 * midpoints carve the usual 4 pieces; the flipped middle piece is cut once
 * more (foot_perp) into 2, giving 5. Recurse until depth == max_depth. */
static void subdivide(const GridCtx *g, Tri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(g, t, depth); return; }

    double m01x = (t.x[0] + t.x[1]) * 0.5, m01y = (t.y[0] + t.y[1]) * 0.5;
    double m12x = (t.x[1] + t.x[2]) * 0.5, m12y = (t.y[1] + t.y[2]) * 0.5;
    double m20x = (t.x[2] + t.x[0]) * 0.5, m20y = (t.y[2] + t.y[0]) * 0.5;

    /* three corner triangles, each half the size of the parent */
    Tri c0 = { {t.x[0], m01x, m20x}, {t.y[0], m01y, m20y} };
    Tri c1 = { {m01x, t.x[1], m12x}, {m01y, t.y[1], m12y} };
    Tri c2 = { {m20x, m12x, t.x[2]}, {m20y, m12y, t.y[2]} };

    /* cut the leftover middle triangle in two along the perpendicular foot */
    double fx, fy;
    foot_perp(m20x, m20y, m01x, m01y, m12x, m12y, &fx, &fy);
    Tri c3 = { {m20x, m01x, fx}, {m20y, m01y, fy} };
    Tri c4 = { {m20x, fx, m12x}, {m20y, fy, m12y} };

    subdivide(g, c0, depth + 1, max_depth);
    subdivide(g, c1, depth + 1, max_depth);
    subdivide(g, c2, depth + 1, max_depth);
    subdivide(g, c3, depth + 1, max_depth);
    subdivide(g, c4, depth + 1, max_depth);
}

/* the seed 1-2-√5 right triangle, sized and centred to the screen.
 * Corners: lower-left, lower-right (square corner), upper-right. */
static Tri scene_seed(const GridCtx *g)
{
    double pw = (double)g->cols * CELL_W;
    double ph = (double)(g->rows - 1) * CELL_H;
    double base = (pw < ph ? pw : ph) * g->size_frac * 0.5;   /* the long leg */
    double cxp = (double)g->ox;
    double cyp = (double)g->oy + base * 0.25;
    Tri t = {
        { cxp - base, cxp + base, cxp + base },
        { cyp,        cyp,        cyp - base * 0.5 },
    };
    return t;
}

/* ── §6 scene ── */

/* Scene — the live, viewer-tunable settings, gathered so reset/redraw stay simple. */
typedef struct {
    int    depth;        /* split levels to draw (+/-), 0..DEPTH_MAX */
    double size_frac;    /* figure size on screen ([ ]), 0..1 */
    int    theme;        /* active colour set (t) */
    int    paused;       /* p froze the view; 1 = paused */
} Scene;

static void scene_reset(Scene *s)
{
    s->depth     = DEPTH_DEFAULT;
    s->size_frac = SIZE_FRAC_DEFAULT;
    s->theme     = 0;
    s->paused    = 0;
}

static void hud_draw(const GridCtx *g, const Scene *s, double fps)
{
    long leaves = 1; for (int i = 0; i < s->depth; i++) leaves *= 5;
    char buf[128];
    snprintf(buf, sizeof buf,
             " depth:%d  leaves:%ld  size:%.2f  theme:%d  %5.1f fps  %s ",
             s->depth, leaves, s->size_frac, s->theme, fps,
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " +/-:depth  [/]:size  t:theme  r:reset  p:pause  q:quit  [10 pinwheel] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Scene *s, double fps)
{
    erase();
    subdivide(g, scene_seed(g), 0, s->depth);
    hud_draw(g, s, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §8 app ── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    Scene sc;
    scene_reset(&sc);
    screen_init(sc.theme);

    GridCtx g;
    ctx_init(&g, LINES, COLS, sc.depth, sc.size_frac);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();   /* let ncurses pick up the new size */
        }
        ctx_init(&g, LINES, COLS, sc.depth, sc.size_frac);

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           sc.paused ^= 1; break;
                case 'r':           scene_reset(&sc); color_init(sc.theme); break;
                case 't':
                    sc.theme = (sc.theme + 1) % N_THEMES;
                    color_init(sc.theme);
                    break;
                case '+': case '=':
                    if (sc.depth < DEPTH_MAX) { sc.depth++; } break;
                case '-':
                    if (sc.depth > DEPTH_MIN) { sc.depth--; } break;
                case '[':
                    if (sc.size_frac > SIZE_FRAC_MIN) { sc.size_frac -= SIZE_FRAC_STEP; } break;
                case ']':
                    if (sc.size_frac < SIZE_FRAC_MAX) { sc.size_frac += SIZE_FRAC_STEP; } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &sc, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
