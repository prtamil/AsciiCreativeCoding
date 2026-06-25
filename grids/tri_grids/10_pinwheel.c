/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 10_pinwheel.c — a triangle that keeps splitting into 5 smaller copies of
 * itself, over and over, making a busy pinwheel-like pattern of nested
 * rotated triangles. Press +/- to add or remove layers of detail.
 *
 * The starting shape is a right triangle whose legs are 1 and 2 (so the
 * long side is √5). Every split makes 5 children that are all the same
 * 1-2-√5 shape, just smaller and turned. This is loosely inspired by the
 * real pinwheel tiling, but simplified — see the note in §5 for how it
 * differs from the strict version.
 *
 * Sister files: 08_triforce.c (the plain 4-way split this builds on) and
 *               12_penrose.c (another never-repeating tiling, golden-ratio).
 * References:   pinwheel tiling — https://en.wikipedia.org/wiki/Pinwheel_tiling
 *               Radin, "The Pinwheel Tilings of the Plane" (1994)
 *               Conway & Radin, "Quaquaversal Tilings and Rotations" (1998)
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
#define DEPTH_MAX     6

#define SIZE_FRAC_DEFAULT 0.85
#define SIZE_FRAC_MIN     0.30
#define SIZE_FRAC_MAX     0.95
#define SIZE_FRAC_STEP    0.05

#define MAX_DEPTH_LEVELS (DEPTH_MAX + 1)
#define N_THEMES         3

/* How much each new frame nudges the on-screen fps number, so it reads
 * steady instead of jittering. Smaller = smoother but slower to react. */
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

/* ── §4 formula ── */

/*
 * GridCtx — everything the drawing code needs to know about the screen
 * and how detailed the pattern should be. Built fresh each frame so it
 * always matches the current terminal size. (Same struct as 07_barycentric.c.)
 */
typedef struct {
    int    rows, cols;   /* terminal size, in character cells */
    int    cw, ch;       /* size of one cell in sub-pixels (width, height) */
    int    ox, oy;       /* sub-pixel coords of screen centre, where we anchor */
    int    depth;        /* how many times to split (more = finer detail) */
    double size_frac;    /* how much of the screen the whole figure fills, 0..1 */
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

static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

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

/*
 * foot_perp — given a point P and a line through A and B, find the spot on
 * that line directly "below" P (the closest point on the line to P). We use
 * it to find where a straight drop from the centre triangle's corner lands
 * on its long side — that's the cut that splits the centre into two halves.
 */
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

/* ── §5 mesh ── */
/*
 * Tri — one triangle, just its three corners as screen positions.
 * The corners always keep the same roles so the splitting code can treat
 * every triangle the same way: corner 0 is one sharp tip, corner 1 is the
 * square (right-angle) corner, and corner 2 is the other sharp tip.
 * Children get the same labelling, which is what lets the recursion stay
 * uniform all the way down.
 *
 * x[0..2], y[0..2] — the three corners, in sub-pixel screen coordinates.
 */
typedef struct { double x[3], y[3]; } Tri;

static void tri_draw_edges(const GridCtx *g, Tri t, int depth)
{
    int pair = PAIR_DEPTH_BASE + depth;
    int attr = COLOR_PAIR(pair) | (depth == 0 ? A_BOLD : 0);
    line_draw(g, t.x[0], t.y[0], t.x[1], t.y[1], attr);
    line_draw(g, t.x[1], t.y[1], t.x[2], t.y[2], attr);
    line_draw(g, t.x[2], t.y[2], t.x[0], t.y[0], attr);
}

/*
 * subdivide — replace one triangle with 5 smaller copies of itself, then
 * keep going until we hit the chosen depth. The trick: marking the three
 * edge midpoints carves out the usual 4 pieces, and the flipped middle
 * piece gets cut once more down the middle, giving 5 children in all.
 * Every child is the same 1-2-√5 shape as the parent, just smaller.
 */
static void subdivide(const GridCtx *g, Tri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(g, t, depth); return; }
    double m01x = (t.x[0] + t.x[1]) * 0.5, m01y = (t.y[0] + t.y[1]) * 0.5;
    double m12x = (t.x[1] + t.x[2]) * 0.5, m12y = (t.y[1] + t.y[2]) * 0.5;
    double m20x = (t.x[2] + t.x[0]) * 0.5, m20y = (t.y[2] + t.y[0]) * 0.5;

    /* The three corner triangles, each half the size of the parent. */
    Tri c0 = { {t.x[0], m01x, m20x}, {t.y[0], m01y, m20y} };
    Tri c1 = { {m01x, t.x[1], m12x}, {m01y, t.y[1], m12y} };
    Tri c2 = { {m20x, m12x, t.x[2]}, {m20y, m12y, t.y[2]} };

    /* Cut the leftover middle triangle in two: drop straight from its
     * square corner (m20) onto its long side (m01-m12) and split there. */
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

/*
 * scene_seed — the very first triangle everything grows from: a 1-2-√5
 * right triangle, sized and centred to fit the current screen. Corners
 * go lower-left, lower-right (the square corner), upper-right.
 */
static Tri scene_seed(const GridCtx *g)
{
    double pw = (double)g->cols * CELL_W;
    double ph = (double)(g->rows - 1) * CELL_H;
    double base = (pw < ph ? pw : ph) * g->size_frac * 0.5;   /* length of the long leg */
    double cxp = (double)g->ox;
    double cyp = (double)g->oy + base * 0.25;
    Tri t = {
        { cxp - base, cxp + base, cxp + base                },
        { cyp,        cyp,        cyp - base * 0.5          },
    };
    return t;
}

/* ── §6 scene ── */

/*
 * Scene — the handful of settings the viewer can change live, all in one
 * place so reset and redraw stay simple.
 */
typedef struct {
    int    depth;        /* how many split levels to draw (+/- keys), 0..DEPTH_MAX */
    double size_frac;    /* how big the figure is on screen ([ ] keys), 0..1 */
    int    theme;        /* which colour set is active (t key) */
    int    paused;       /* p key froze the view; 1 = paused */
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
            /* terminal was resized; this makes ncurses pick up the new size */
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
