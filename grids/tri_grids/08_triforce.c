/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 08_triforce.c — the Triforce split: cut a triangle into four, over and over
 *
 * Take one big triangle and split it into four smaller ones by joining the
 * midpoints of its edges (the shape you've seen as the Triforce). The three
 * corners point the same way as the parent; the middle one is flipped. Then
 * do the same to every piece. +/- changes how many times we split, so the
 * number of small triangles is 4 to the power of the depth.
 *
 * Study alongside: 07_barycentric.c — splits each triangle six ways instead.
 *                  09_sierpinski.c  — same four-way split, but it throws away
 *                  the flipped middle piece, leaving the famous gasket.
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

/* One color per recursion level, so deeper triangles read in a different
 * shade than their parent. PAL256 for rich terminals, PAL8 for the basic
 * eight-color fallback. */
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

/* ── §4 formula — screen layout + line drawing ── */

/*
 * GridCtx — everything we need to place and draw the pattern on screen:
 * how big the terminal is, how to map our coordinates onto cells, where the
 * centre of the drawing sits, and the two knobs the user controls (how many
 * times to split, and how much of the screen to fill).
 * 07_barycentric.c has the original version of this struct.
 */
typedef struct {
    int    rows, cols;     /* terminal size, in character cells            */
    int    cw, ch;         /* sub-cells per character (we draw finer than 1
                            * char wide); always CELL_W / CELL_H           */
    int    ox, oy;         /* centre of the drawing, in sub-cell units     */
    int    depth;          /* how many times to split, 0..DEPTH_MAX        */
    double size_frac;      /* fraction of the screen the triangle fills    */
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

/* Picks the ASCII character that best matches a line's tilt: -, |, / or \. */
static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

/* Draws a straight line between two points, one cell at a time. Uses
 * Bresenham's classic line algorithm — it walks the line with integer steps
 * only, so it stays fast and never skips a cell. We clip to the screen and
 * leave the bottom row free for the hint bar. */
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

/* ── §5 mesh — the four-way split ── */

/* A triangle: just its three corner points (x[0..2], y[0..2]). */
typedef struct { double x[3], y[3]; } Tri;

static void tri_draw_edges(const GridCtx *g, Tri t, int depth)
{
    int pair = PAIR_DEPTH_BASE + depth;
    int attr = COLOR_PAIR(pair) | (depth == 0 ? A_BOLD : 0);
    line_draw(g, t.x[0], t.y[0], t.x[1], t.y[1], attr);
    line_draw(g, t.x[1], t.y[1], t.x[2], t.y[2], attr);
    line_draw(g, t.x[2], t.y[2], t.x[0], t.y[0], attr);
}

/* The heart of it: split this triangle into four, then split each of those,
 * until we've gone as deep as the user asked, where we finally draw. */
static void subdivide(const GridCtx *g, Tri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(g, t, depth); return; }
    double m01x = (t.x[0] + t.x[1]) * 0.5, m01y = (t.y[0] + t.y[1]) * 0.5;
    double m12x = (t.x[1] + t.x[2]) * 0.5, m12y = (t.y[1] + t.y[2]) * 0.5;
    double m20x = (t.x[2] + t.x[0]) * 0.5, m20y = (t.y[2] + t.y[0]) * 0.5;

    /* Three small triangles, one tucked into each corner, pointing the
     * same way the parent did. */
    Tri c0 = { {t.x[0], m01x, m20x}, {t.y[0], m01y, m20y} };
    Tri c1 = { {m01x, t.x[1], m12x}, {m01y, t.y[1], m12y} };
    Tri c2 = { {m20x, m12x, t.x[2]}, {m20y, m12y, t.y[2]} };
    /* The middle triangle, made from the three midpoints — it points the
     * opposite way (the upside-down piece of the Triforce). */
    Tri cc = { {m12x, m20x, m01x}, {m12y, m20y, m01y} };

    subdivide(g, c0, depth + 1, max_depth);
    subdivide(g, c1, depth + 1, max_depth);
    subdivide(g, c2, depth + 1, max_depth);
    subdivide(g, cc, depth + 1, max_depth);
}

/* Builds the one big triangle we start from: an upward equilateral triangle
 * centred on screen, sized by size_frac. */
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

/* The handful of settings the user can change at runtime — the live state of
 * what's on screen. */
typedef struct {
    int    depth;      /* how many times to split, 0..DEPTH_MAX  */
    double size_frac;  /* fraction of the screen the triangle fills */
    int    theme;      /* which color palette is active          */
    int    paused;     /* 1 = frozen (drawing still refreshes)   */
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
        /* Terminal was resized: tear ncurses down and bring it back so it
         * picks up the new width and height. */
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
