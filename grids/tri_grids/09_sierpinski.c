/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 09_sierpinski.c — Sierpinski triangle (3-way midpoint subdivision)
 *
 * DEMO: One big equilateral triangle is recursively split into 3 corner
 *       sub-triangles via midpoint subdivision — the classic Sierpinski
 *       fractal. The centre (inverted) triangle that 08_triforce.c keeps
 *       is dropped, leaving the famous self-similar gasket. Use +/- to
 *       change depth (0..9). Triangle count grows as 3^N — much sparser
 *       than the triforce at the same depth.
 *
 * Study alongside: 08_triforce.c — same midpoint split, but keeps all 4
 *                  children (4^N leaves vs 3^N here).
 *                  07_barycentric.c — 6-way split (centroid-anchored).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, DEPTH, SIZE_FRAC
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — depth-keyed palette + HUD / hint
 *   §4 formula  — slope_char + Bresenham line_draw (same as 07/08)
 *   §5 subdivide — 3-way split (drops centre child)
 *   §6 scene    — seed_triangle + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  +/- depth   [/] size   r reset   t theme   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/09_sierpinski.c \
 *       -o 09_sierpinski -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Sierpinski triangle (Sierpiński 1915). Take the
 *                  midpoint subdivision of 08_triforce, but keep ONLY
 *                  the three corner children — drop the inverted centre.
 *                  The fractal limit set has Hausdorff dimension log₂3
 *                  ≈ 1.585, area zero, infinite perimeter. At finite
 *                  depth N the drawn figure has 3^N triangles, each at
 *                  scale 2⁻ᴺ.
 *
 * Formula        : Same midpoint subdivision as 08; only 3 of the 4
 *                  children are kept. Recursion depth controlled by +/-
 *                  keys (0..9).
 *
 * Edge chars     : Same Bresenham + slope_char as 07/08. Color keyed
 *                  to depth.
 *
 * Movement       : None — depth is the user-controlled parameter.
 *
 * References     :
 *   Sierpinski triangle  — https://en.wikipedia.org/wiki/Sierpinski_triangle
 *   Sierpiński 1915 — original construction (in French)
 *   Mandelbrot, "The Fractal Geometry of Nature" (1982) §6
 *   Iterated function systems — Hutchinson 1981
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Same midpoint split as the Triforce — but throw away the centre piece.
 * Three corner triangles remain at every step; recurse on each. The holes
 * left by all the discarded centres at every scale produce the famous
 * Sierpinski gasket — a fractal with finite circumference at any depth
 * but zero area in the limit.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Sierpinski is THE example of a self-similar fractal with non-integer
 * Hausdorff dimension. The dimension log₂3 ≈ 1.585 says the gasket is
 * "more than a 1-D curve, less than a 2-D area". Each refinement step
 * removes 1/4 of the remaining area, so after N steps the figure occupies
 * (3/4)^N of the original area, tending to zero — yet the perimeter
 * (sum of edge lengths) grows by a factor of 3/2 per step, tending to
 * infinity. Finite-area zero, infinite-length perimeter.
 *
 * Our renderer draws all leaves at chosen depth N. The "holes" in the
 * gasket are simply the regions not visited by any leaf — those are the
 * dropped centres at every level.
 *
 * DRAWING METHOD  (recursive emit)
 * ──────────────
 *  1. Pick DEPTH and SIZE_FRAC.
 *  2. Build seed equilateral triangle.
 *  3. subdivide(t, depth):
 *       if depth == max_depth: draw 3 edges
 *       else:
 *         compute 3 midpoints
 *         build 3 corner children — DROP the inverted centre
 *         recurse on each corner
 *  4. Bresenham line_draw per leaf edge.
 *
 * KEY FORMULAS
 * ────────────
 *  Edge midpoints, identical to 08_triforce:
 *    M01 = (V0+V1)/2,  M12 = (V1+V2)/2,  M20 = (V2+V0)/2
 *
 *  Three children kept (no centre):
 *    Corner-V0:  (V0, M01, M20)
 *    Corner-V1:  (M01, V1, M12)
 *    Corner-V2:  (M20, M12, V2)
 *
 *  Leaf count at depth N: 3^N
 *  Hausdorff dimension:   log₂3 = ln 3 / ln 2 ≈ 1.585
 *  Area at depth N:       (3/4)^N · A₀
 *  Perimeter at depth N:  (3/2)^N · P₀
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The centre is "dropped" simply by NOT recursing into it. Nothing to
 *    erase — we just draw fewer leaves than 08.
 *  • Stack depth at N=9 is fine (3^9 = 19683 leaves; deepest path is 9
 *    frames).
 *  • At small DEPTH the figure looks like a hollow triangle. Pump DEPTH
 *    up and the recursive holes appear at every scale.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At depth 0: 3^0 = 1 leaf — original triangle.
 *  At depth 1: 3 leaves — 3 corner triangles, centre HOLE visible.
 *  At depth 2: 9 leaves — each corner child further split into 3.
 *  At depth 5: 3^5 = 243 leaves; visible hole pattern at three scales.
 *
 *  Compare with 08_triforce at the same depth: triforce shows 4^5 = 1024
 *  leaves filling the entire triangle; sierpinski shows 243 leaves with
 *  the characteristic gasket holes.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define DEPTH_DEFAULT 4
#define DEPTH_MIN     0
#define DEPTH_MAX     9

#define SIZE_FRAC_DEFAULT 0.90
#define SIZE_FRAC_MIN     0.30
#define SIZE_FRAC_MAX     0.95
#define SIZE_FRAC_STEP    0.05

#define MAX_DEPTH_LEVELS (DEPTH_MAX + 1)
#define N_THEMES         3

#define PAIR_DEPTH_BASE  1
#define PAIR_HUD        (PAIR_DEPTH_BASE + MAX_DEPTH_LEVELS)
#define PAIR_HINT       (PAIR_HUD + 1)

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short PAL256[N_THEMES][MAX_DEPTH_LEVELS] = {
    /* electric */ { 15, 39, 82, 226, 196, 207, 21,  39,  15,  15 },
    /* fire     */ { 15, 226, 196, 207, 21,  39, 82, 226, 196, 207 },
    /* mono     */ { 15, 15, 15, 15, 15, 15, 15, 15, 15, 15 },
};
static const short PAL8[N_THEMES][MAX_DEPTH_LEVELS] = {
    { COLOR_WHITE, COLOR_CYAN,    COLOR_GREEN,  COLOR_YELLOW,
      COLOR_RED,   COLOR_MAGENTA, COLOR_BLUE,   COLOR_CYAN,
      COLOR_WHITE, COLOR_WHITE },
    { COLOR_WHITE, COLOR_YELLOW,  COLOR_RED,    COLOR_MAGENTA,
      COLOR_BLUE,  COLOR_CYAN,    COLOR_GREEN,  COLOR_YELLOW,
      COLOR_RED,   COLOR_MAGENTA },
    { COLOR_WHITE, COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < MAX_DEPTH_LEVELS; i++) {
        short fg = (COLORS >= 256) ? PAL256[theme][i] : PAL8[theme][i];
        init_pair(PAIR_DEPTH_BASE + i, fg, -1);
    }
    init_pair(PAIR_HUD,  COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT, COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — slope_char + Bresenham line                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

static void line_draw(int rows, int cols, double px0, double py0,
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
        if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows - 1)
            mvaddch(y0, x0, (chtype)(unsigned char)ch);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    attroff(attr);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  subdivide — 3-way recursive split                                   */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { double x[3], y[3]; } Tri;

static void tri_draw_edges(int rows, int cols, Tri t, int depth)
{
    int pair = PAIR_DEPTH_BASE + depth;
    int attr = COLOR_PAIR(pair) | (depth == 0 ? A_BOLD : 0);
    line_draw(rows, cols, t.x[0], t.y[0], t.x[1], t.y[1], attr);
    line_draw(rows, cols, t.x[1], t.y[1], t.x[2], t.y[2], attr);
    line_draw(rows, cols, t.x[2], t.y[2], t.x[0], t.y[0], attr);
}

/*
 * subdivide — keep only the 3 corner children. The inverted centre
 * (the "hole") is silently dropped, producing the gasket.
 */
static void subdivide(int rows, int cols, Tri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(rows, cols, t, depth); return; }
    double m01x = (t.x[0] + t.x[1]) * 0.5, m01y = (t.y[0] + t.y[1]) * 0.5;
    double m12x = (t.x[1] + t.x[2]) * 0.5, m12y = (t.y[1] + t.y[2]) * 0.5;
    double m20x = (t.x[2] + t.x[0]) * 0.5, m20y = (t.y[2] + t.y[0]) * 0.5;

    Tri c0 = { {t.x[0], m01x, m20x}, {t.y[0], m01y, m20y} };
    Tri c1 = { {m01x, t.x[1], m12x}, {m01y, t.y[1], m12y} };
    Tri c2 = { {m20x, m12x, t.x[2]}, {m20y, m12y, t.y[2]} };

    subdivide(rows, cols, c0, depth + 1, max_depth);
    subdivide(rows, cols, c1, depth + 1, max_depth);
    subdivide(rows, cols, c2, depth + 1, max_depth);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    depth;
    double size_frac;
    int    theme;
    int    paused;
} Scene;

static void scene_reset(Scene *s)
{
    s->depth     = DEPTH_DEFAULT;
    s->size_frac = SIZE_FRAC_DEFAULT;
    s->theme     = 0;
    s->paused    = 0;
}

static Tri scene_seed(const Scene *s, int rows, int cols)
{
    double pw = (double)cols * CELL_W;
    double ph = (double)(rows - 1) * CELL_H;
    double base = (pw < ph ? pw : ph) * s->size_frac;
    double h    = base * sqrt(3.0) * 0.5;
    double cxp  = pw * 0.5;
    double cyp  = ph * 0.5;
    Tri t = {
        { cxp,                 cxp - base * 0.5, cxp + base * 0.5 },
        { cyp - h * 2.0/3.0,   cyp + h / 3.0,    cyp + h / 3.0    },
    };
    return t;
}

static void scene_draw(int rows, int cols, const Scene *s, double fps)
{
    erase();
    Tri seed = scene_seed(s, rows, cols);
    subdivide(rows, cols, seed, 0, s->depth);

    long leaves = 1; for (int i = 0; i < s->depth; i++) leaves *= 3;
    char buf[128];
    snprintf(buf, sizeof buf,
             " depth:%d  leaves:%ld  size:%.2f  theme:%d  %5.1f fps  %s ",
             s->depth, leaves, s->size_frac, s->theme, fps,
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " +/-:depth  [/]:size  t:theme  r:reset  p:pause  q:quit  [09 sierpinski] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

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

    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
        }
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
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;

        scene_draw(rows, cols, &sc, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
