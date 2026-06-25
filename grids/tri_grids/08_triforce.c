/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 08_triforce.c — recursive 4-way subdivision: split a triangle at its edge
 * midpoints into 3 corner pieces + 1 flipped middle, then recurse on all four.
 * Leaf count is 4^depth. +/- tunes depth; [/] the fill size.
 *
 * Sisters: 07_barycentric.c (6-way split), 09_sierpinski.c (drops the middle).
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

#define CELL_W 2
#define CELL_H 4

#define DEPTH_DEFAULT 3
#define DEPTH_MIN     0
#define DEPTH_MAX     7

#define SIZE_FRAC_DEFAULT 0.85
#define SIZE_FRAC_MIN     0.30
#define SIZE_FRAC_MAX     0.95
#define SIZE_FRAC_STEP    0.05

#define MAX_DEPTH_LEVELS (DEPTH_MAX + 1)
#define N_THEMES         3

/* How much the on-screen fps number leans toward the latest frame. Small
 * value = smooth, slow-moving readout instead of a jittery one. */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_DEPTH_BASE  1
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

/* One color per recursion level — depth i uses pair PAIR_DEPTH_BASE+i.
 * PAL256 for rich terminals, PAL8 for the eight-color fallback. */
static const short PAL256[N_THEMES][MAX_DEPTH_LEVELS] = {
    /* sunset */ { 15, 226, 196, 207,  21,  39, 82,  15 },
    /* forest */ { 15,  82,  39,  21, 207, 196, 226, 82 },
    /* mono   */ { 15,  15,  15,  15,  15,  15, 15,  15 },
};
static const short PAL8[N_THEMES][MAX_DEPTH_LEVELS] = {
    { COLOR_WHITE, COLOR_YELLOW,  COLOR_RED,    COLOR_MAGENTA,
      COLOR_BLUE,  COLOR_CYAN,    COLOR_GREEN,  COLOR_WHITE },
    { COLOR_WHITE, COLOR_GREEN,   COLOR_CYAN,   COLOR_BLUE,
      COLOR_MAGENTA, COLOR_RED,   COLOR_YELLOW, COLOR_GREEN },
    { COLOR_WHITE, COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE },
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

/* ── §4 screen mapping & line drawing ── */

/* GridCtx — the drawing frame: terminal size, sub-pixel scale, and the centre
 * the seed triangle is built around. Sister: 07_barycentric.c. */
typedef struct {
    int    rows, cols;     /* terminal size in cells                    */
    int    cw, ch;         /* sub-pixels per cell — always CELL_W/CELL_H */
    int    ox, oy;         /* drawing centre, in sub-pixels             */
    int    depth;          /* recursion depth, 0..DEPTH_MAX             */
    double size_frac;      /* fraction of the screen the triangle fills */
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

/* ASCII glyph for a line's tilt: '-', '|', '/' or '\'. */
static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

/* Bresenham line between two pixels, clipped to the screen (bottom row stays
 * free for the hint bar). Integer steps only — never skips a cell. */
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

/* ── §5 subdivision — the four-way split ── */

/* A triangle: three corner points (x[0..2], y[0..2]). */
typedef struct { double x[3], y[3]; } Tri;

/* Midpoint of two points. */
static void midpoint(double ax, double ay, double bx, double by,
                     double *mx, double *my)
{
    *mx = (ax + bx) * 0.5;
    *my = (ay + by) * 0.5;
}

static void tri_draw_edges(const GridCtx *g, Tri t, int depth)
{
    int attr = COLOR_PAIR(PAIR_DEPTH_BASE + depth) | (depth == 0 ? A_BOLD : 0);
    line_draw(g, t.x[0], t.y[0], t.x[1], t.y[1], attr);
    line_draw(g, t.x[1], t.y[1], t.x[2], t.y[2], attr);
    line_draw(g, t.x[2], t.y[2], t.x[0], t.y[0], attr);
}

/* recipe — split t at its edge midpoints into 4: three corner triangles (same
 * orientation as the parent) + one flipped middle; recurse on all four until
 * max_depth, then draw. Keeping the middle is what makes this the Triforce
 * rather than a Sierpinski gasket. */
static void subdivide(const GridCtx *g, Tri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(g, t, depth); return; }

    double m01x, m01y, m12x, m12y, m20x, m20y;
    midpoint(t.x[0], t.y[0], t.x[1], t.y[1], &m01x, &m01y);
    midpoint(t.x[1], t.y[1], t.x[2], t.y[2], &m12x, &m12y);
    midpoint(t.x[2], t.y[2], t.x[0], t.y[0], &m20x, &m20y);

    Tri corner0 = { {t.x[0], m01x, m20x}, {t.y[0], m01y, m20y} };
    Tri corner1 = { {m01x, t.x[1], m12x}, {m01y, t.y[1], m12y} };
    Tri corner2 = { {m20x, m12x, t.x[2]}, {m20y, m12y, t.y[2]} };
    Tri middle  = { {m12x, m20x, m01x}, {m12y, m20y, m01y} };

    subdivide(g, corner0, depth + 1, max_depth);
    subdivide(g, corner1, depth + 1, max_depth);
    subdivide(g, corner2, depth + 1, max_depth);
    subdivide(g, middle,  depth + 1, max_depth);
}

/* The starting triangle: an upward equilateral, centred, sized by size_frac. */
static Tri scene_seed(const GridCtx *g)
{
    double pw = (double)g->cols * CELL_W;
    double ph = (double)(g->rows - 1) * CELL_H;
    double base = (pw < ph ? pw : ph) * g->size_frac;
    double h    = base * sqrt(3.0) * 0.5;
    double cxp  = (double)g->ox;
    double cyp  = (double)g->oy;
    Tri t = {
        { cxp,                 cxp - base * 0.5, cxp + base * 0.5 },
        { cyp - h * 2.0/3.0,   cyp + h / 3.0,    cyp + h / 3.0    },
    };
    return t;
}

/* ── §6 scene ── */

/* Live runtime state — the knobs the keys change. */
typedef struct {
    int    depth;      /* recursion depth, 0..DEPTH_MAX             */
    double size_frac;  /* fraction of the screen the triangle fills */
    int    theme;      /* active palette index                      */
    int    paused;     /* 1 = frozen (drawing still refreshes)      */
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
    long leaves = 1; for (int i = 0; i < s->depth; i++) leaves *= 4;
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
             " +/-:depth  [/]:size  t:theme  r:reset  p:pause  q:quit  [08 triforce] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Scene *s, double fps)
{
    erase();
    Tri seed = scene_seed(g);
    subdivide(g, seed, 0, s->depth);
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
            endwin(); refresh();
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
