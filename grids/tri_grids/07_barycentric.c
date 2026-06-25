/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 07_barycentric.c — subdivide one triangle by barycentric coordinates
 *
 * Every point inside a triangle is a weighted blend of its three corners:
 * weights (u,v,w) with u+v+w = 1. The splitting points are exactly such
 * blends — the centroid is (1/3,1/3,1/3), each side's midpoint sets one
 * weight to 0 and the other two to 1/2. Fan from the centroid to make 6
 * smaller triangles, then recurse. Depth-N has 6^N self-similar pieces.
 *
 * Sister: 01_equilateral.c (the shared tri/GridCtx skeleton); 08_triforce.c
 * splits the other classic way (midpoint triangle, no centroid).
 * Ref: Barycentric coords https://en.wikipedia.org/wiki/Barycentric_coordinate_system
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

#define DEPTH_DEFAULT 2
#define DEPTH_MIN     0
#define DEPTH_MAX     5

#define SIZE_FRAC_DEFAULT 0.85
#define SIZE_FRAC_MIN     0.30
#define SIZE_FRAC_MAX     0.95
#define SIZE_FRAC_STEP    0.05

#define MAX_DEPTH_LEVELS (DEPTH_MAX + 1)
#define N_THEMES         3

/* How much each new frame nudges the displayed fps. Small = steady number. */
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

static const short PAL256[N_THEMES][MAX_DEPTH_LEVELS] = {
    /* ocean */ {  15,  39,  21, 207, 196, 226 },
    /* fire  */ {  15, 226, 196, 207,  21,  39 },
    /* mono  */ {  15,  15,  15,  15,  15,  15 },
};
static const short PAL8[N_THEMES][MAX_DEPTH_LEVELS] = {
    { COLOR_WHITE, COLOR_CYAN,    COLOR_BLUE,    COLOR_MAGENTA, COLOR_RED,    COLOR_YELLOW },
    { COLOR_WHITE, COLOR_YELLOW,  COLOR_RED,     COLOR_MAGENTA, COLOR_BLUE,   COLOR_CYAN   },
    { COLOR_WHITE, COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE  },
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

/* ── §4 tri mapping & lattice ── */

/* GridCtx — the triangle picture for one frame. We never store the triangles,
 * so this struct (terminal size, centre, the two live knobs) is the whole
 * state: hand it to the drawing code and it redraws from scratch. */
typedef struct {
    int    rows, cols;   /* terminal size in cells */
    int    cw, ch;       /* sub-pixels per cell (CELL_W, CELL_H) — keeps the
                            grid square despite tall characters */
    int    ox, oy;       /* picture centre, in sub-pixels */
    int    depth;        /* how many times to split, 0..DEPTH_MAX */
    double size_frac;    /* triangle size as a fraction of the screen */
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

/* the line char nearest a segment's slant: '-' flat, '|' upright, '/' '\' the
 * diagonals. Squish by cell shape first so the choice looks right on screen. */
static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

/* draw a segment cell-by-cell with Bresenham (integer line stepping); the
 * whole segment uses one character, chosen from its slant. */
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

/* ── §5 subdivision ── */
/* No cursor here: you explore by depth (+/-) and size ([/]), not by stepping
 * triangles. We never store the mesh — each piece is drawn as it's generated. */

/* One triangle: its three corners, stored as x's and y's. */
typedef struct { double x[3], y[3]; } Tri;

/* recipe step 1 — barycentric point: blend the three corners by weights
 * (u,v,w), u+v+w = 1. Every split point is one of these blends. */
static void bary_point(Tri t, double u, double v, double w,
                       double *px, double *py)
{
    *px = u * t.x[0] + v * t.x[1] + w * t.x[2];
    *py = u * t.y[0] + v * t.y[1] + w * t.y[2];
}

/* Draw a triangle's three sides. Color comes from depth, so every triangle
   at the same split level looks alike; level 0 is drawn bold to stand out. */
static void tri_draw_edges(const GridCtx *g, Tri t, int depth)
{
    int pair = PAIR_DEPTH_BASE + depth;
    int attr = COLOR_PAIR(pair) | (depth == 0 ? A_BOLD : 0);
    line_draw(g, t.x[0], t.y[0], t.x[1], t.y[1], attr);
    line_draw(g, t.x[1], t.y[1], t.x[2], t.y[2], attr);
    line_draw(g, t.x[2], t.y[2], t.x[0], t.y[0], attr);
}

/* recipe step 2 — split, recurse, draw. At max depth, draw the triangle.
 * Otherwise find the centroid (1/3,1/3,1/3) and the three side midpoints
 * (each one weight 0, the others 1/2), then fan into 6 triangles that each
 * run centroid -> a corner -> the next side's midpoint, and split those too. */
static void subdivide(const GridCtx *g, Tri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(g, t, depth); return; }

    double cx,  cy;   bary_point(t, 1.0/3, 1.0/3, 1.0/3, &cx,  &cy);   /* centroid */
    double m01x, m01y; bary_point(t, 0.5, 0.5, 0.0, &m01x, &m01y);     /* side 0-1 */
    double m12x, m12y; bary_point(t, 0.0, 0.5, 0.5, &m12x, &m12y);     /* side 1-2 */
    double m20x, m20y; bary_point(t, 0.5, 0.0, 0.5, &m20x, &m20y);     /* side 2-0 */

    Tri subs[6] = {
        { {cx, t.x[0], m01x}, {cy, t.y[0], m01y} },
        { {cx, m01x, t.x[1]}, {cy, m01y, t.y[1]} },
        { {cx, t.x[1], m12x}, {cy, t.y[1], m12y} },
        { {cx, m12x, t.x[2]}, {cy, m12y, t.y[2]} },
        { {cx, t.x[2], m20x}, {cy, t.y[2], m20y} },
        { {cx, m20x, t.x[0]}, {cy, m20y, t.y[0]} },
    };
    for (int i = 0; i < 6; i++)
        subdivide(g, subs[i], depth + 1, max_depth);
}

/* the starting triangle: centred, point up, sized to a fraction of the
 * smaller screen dimension so it always fits. */
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

/* the live state the keys can change. */
typedef struct {
    int    depth;       /* how many times to split, 0..DEPTH_MAX */
    double size_frac;   /* triangle size, fraction of the screen */
    int    theme;       /* which color palette is active */
    int    paused;      /* 1 = frozen */
} Scene;

static void scene_reset(Scene *s)
{
    s->depth     = DEPTH_DEFAULT;
    s->size_frac = SIZE_FRAC_DEFAULT;
    s->theme     = 0;
    s->paused    = 0;
}

/* Status line top-right, key hints bottom-left. */
static void hud_draw(const GridCtx *g, const Scene *s, double fps)
{
    /* each split turns one triangle into six, so the count is 6 to the depth */
    long leaves = 1; for (int i = 0; i < s->depth; i++) leaves *= 6;
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
             " +/-:depth  [/]:size  t:theme  r:reset  p:pause  q:quit  [07 barycentric] ");
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
        /* terminal was resized: tear ncurses down and back up so it picks
           up the new size, otherwise the display stays corrupted */
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
