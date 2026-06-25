/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 12_penrose.c — Penrose-style tiling by recursive deflation of Robinson
 * triangles. Start with one acute or obtuse half-tile; each round splits every
 * triangle at the golden-ratio point on one edge into one acute + one obtuse
 * child. The golden ratio is what makes the result aperiodic.
 *
 * Simplified cut: each triangle makes exactly two children (textbook Penrose
 * makes two or three) — same two shapes, same golden cut, same self-similarity.
 *
 * Sister files: 10_pinwheel.c, 09_sierpinski.c (other substitution tilings).
 * Refs: Penrose "Pentaplexity" (1979); Robinson (1971); Senechal §7 (1995).
 *       https://en.wikipedia.org/wiki/Penrose_tiling
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

#define PHI     1.6180339887498949
#define INV_PHI 0.6180339887498949

#define DEPTH_DEFAULT 4
#define DEPTH_MIN     0
#define DEPTH_MAX    10

#define SIZE_FRAC_DEFAULT 0.85
#define SIZE_FRAC_MIN     0.30
#define SIZE_FRAC_MAX     0.95
#define SIZE_FRAC_STEP    0.05

#define N_THEMES 3

/* How heavily to smooth the fps number so it doesn't jitter every frame. */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_ACUTE  1
#define PAIR_OBTUSE 2
#define PAIR_HUD    3
#define PAIR_HINT   4

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

static const short THEME_FG[N_THEMES][2] = {
    /* acute, obtuse */
    {  39, 207 },
    { 226,  21 },
    {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_MAGENTA },
    { COLOR_YELLOW,  COLOR_BLUE    },
    { COLOR_WHITE,   COLOR_CYAN    },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_a, fg_b;
    if (COLORS >= 256) { fg_a = THEME_FG[theme][0];   fg_b = THEME_FG[theme][1];   }
    else               { fg_a = THEME_FG_8[theme][0]; fg_b = THEME_FG_8[theme][1]; }
    init_pair(PAIR_ACUTE,  fg_a, -1);
    init_pair(PAIR_OBTUSE, fg_b, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 screen layout & line drawing ── */

#define TYPE_ACUTE   0
#define TYPE_OBTUSE  1

/* GridCtx — the tiling placed on this frame: screen size, centre (sub-pixel
 * units), and the three live knobs steered by keys. */
typedef struct {
    int    rows, cols;       /* terminal size, in cells */
    int    cw, ch;           /* sub-pixels per cell (CELL_W, CELL_H) */
    int    ox, oy;           /* centre of the drawing, in sub-pixels */
    int    depth;            /* deflation rounds, 0..DEPTH_MAX */
    double size_frac;        /* seed triangle's share of the screen, 0..1 */
    int    seed_type;        /* TYPE_ACUTE or TYPE_OBTUSE */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols,
                     int depth, double size_frac, int seed_type)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = (cols * CELL_W) / 2;
    g->oy = ((rows - 1) * CELL_H) / 2;
    g->depth     = depth;
    g->size_frac = size_frac;
    g->seed_type = seed_type;
}

/* the line char (- | / \) nearest a segment's slope */
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

/* ── §5 deflation — split each Robinson triangle into two smaller ones ── */

/* PTri — one half-tile. Corners are kept in a fixed order: 0 = apex (the sharp
 * 36° point for acute, the blunt 108° point for obtuse), 1 and 2 = base. Every
 * deflation rule preserves this order, so children come out apex-first. */
typedef struct {
    double x[3], y[3];      /* corner 0 = apex, 1 and 2 = base */
    int    type;            /* TYPE_ACUTE (thin) or TYPE_OBTUSE (wide) */
} PTri;

static void tri_draw_edges(const GridCtx *g, PTri t)
{
    int attr = COLOR_PAIR(t.type == TYPE_ACUTE ? PAIR_ACUTE : PAIR_OBTUSE);
    line_draw(g, t.x[0], t.y[0], t.x[1], t.y[1], attr);
    line_draw(g, t.x[1], t.y[1], t.x[2], t.y[2], attr);
    line_draw(g, t.x[2], t.y[2], t.x[0], t.y[0], attr);
}

/* the golden-ratio point along edge a->b — the cut always lands here */
static void golden_point(PTri t, int a, int b, double *px, double *py)
{
    *px = t.x[a] + (t.x[b] - t.x[a]) * INV_PHI;
    *py = t.y[a] + (t.y[b] - t.y[a]) * INV_PHI;
}

/* deflate one acute (thin) tile -> one obtuse + one acute child.
 * P sits on the leg apex->base-corner-1. out[0] wide: apex P, base (0,2).
 *                                         out[1] thin: apex 2, base (P,1). */
static void deflate_acute(PTri t, PTri out[2])
{
    double Px, Py; golden_point(t, 0, 1, &Px, &Py);
    out[0] = (PTri){ { Px, t.x[0], t.x[2] }, { Py, t.y[0], t.y[2] }, TYPE_OBTUSE };
    out[1] = (PTri){ { t.x[2], Px, t.x[1] }, { t.y[2], Py, t.y[1] }, TYPE_ACUTE  };
}

/* deflate one obtuse (wide) tile -> one acute + one obtuse child.
 * P sits on the long base corner-1->corner-2. out[0] thin: apex 1, base (0,P).
 *                                              out[1] wide: apex P, base (0,2). */
static void deflate_obtuse(PTri t, PTri out[2])
{
    double Px, Py; golden_point(t, 1, 2, &Px, &Py);
    out[0] = (PTri){ { t.x[1], t.x[0], Px }, { t.y[1], t.y[0], Py }, TYPE_ACUTE  };
    out[1] = (PTri){ { Px, t.x[0], t.x[2] }, { Py, t.y[0], t.y[2] }, TYPE_OBTUSE };
}

/* recurse: deflate to the requested depth, then draw the leaf triangles */
static void deflate(const GridCtx *g, PTri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(g, t); return; }

    PTri child[2];
    if (t.type == TYPE_ACUTE) deflate_acute(t, child);
    else                      deflate_obtuse(t, child);
    deflate(g, child[0], depth + 1, max_depth);
    deflate(g, child[1], depth + 1, max_depth);
}

/* the first triangle, centred with its apex up, sized to size_frac of the
 * screen — acute or obtuse per seed_type. Everything else deflates from this. */
static PTri scene_seed(const GridCtx *g)
{
    double pw = (double)g->cols * CELL_W;
    double ph = (double)(g->rows - 1) * CELL_H;
    double cxp = (double)g->ox;
    double cyp = (double)g->oy;

    if (g->seed_type == TYPE_ACUTE) {
        double leg    = (pw < ph ? pw : ph) * g->size_frac * 0.5;
        double base   = leg / PHI;
        double height = leg * sin(72.0 * M_PI / 180.0);
        PTri t = {
            { cxp,                cxp - base * 0.5,    cxp + base * 0.5 },
            { cyp - height * 0.5, cyp + height * 0.5,  cyp + height * 0.5 },
            TYPE_ACUTE
        };
        return t;
    } else {
        double leg    = (pw < ph ? pw : ph) * g->size_frac * 0.4;
        double base   = leg * PHI;
        double height = leg * sin(36.0 * M_PI / 180.0);
        PTri t = {
            { cxp,                cxp - base * 0.5,    cxp + base * 0.5 },
            { cyp - height * 0.5, cyp + height * 0.5,  cyp + height * 0.5 },
            TYPE_OBTUSE
        };
        return t;
    }
}

/* ── §6 scene ── */

/* Scene — the live state the keys steer. */
typedef struct {
    int    depth;           /* deflation rounds, 0..DEPTH_MAX */
    double size_frac;       /* seed triangle's share of the screen, 0..1 */
    int    seed_type;       /* TYPE_ACUTE or TYPE_OBTUSE */
    int    theme;           /* palette index, 0..N_THEMES-1 */
    int    paused;          /* nonzero while paused */
} Scene;

static void scene_reset(Scene *s)
{
    s->depth     = DEPTH_DEFAULT;
    s->size_frac = SIZE_FRAC_DEFAULT;
    s->seed_type = TYPE_ACUTE;
    s->theme     = 0;
    s->paused    = 0;
}

static void hud_draw(const GridCtx *g, const Scene *s, double fps)
{
    long leaves = 1; for (int i = 0; i < s->depth; i++) leaves *= 2;
    char buf[128];
    snprintf(buf, sizeof buf,
             " seed:%s  depth:%d  leaves~%ld  size:%.2f  theme:%d  %5.1f fps  %s ",
             s->seed_type == TYPE_ACUTE ? "acute(36°)" : "obtuse(108°)",
             s->depth, leaves, s->size_frac, s->theme, fps,
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " +/-:depth  [/]:size  s:swap-seed  t:theme  r:reset  p:pause  q:quit  [12 penrose] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Scene *s, double fps)
{
    erase();
    PTri seed = scene_seed(g);
    deflate(g, seed, 0, s->depth);
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
    ctx_init(&g, LINES, COLS, sc.depth, sc.size_frac, sc.seed_type);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
        }
        ctx_init(&g, LINES, COLS, sc.depth, sc.size_frac, sc.seed_type);

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           sc.paused ^= 1; break;
                case 'r':           scene_reset(&sc); color_init(sc.theme); break;
                case 's':
                    sc.seed_type = (sc.seed_type == TYPE_ACUTE)
                                    ? TYPE_OBTUSE : TYPE_ACUTE;
                    break;
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
