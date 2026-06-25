/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 12_penrose.c — a Penrose-style tiling drawn by splitting triangles forever.
 *
 * We start with one of two special triangles and keep cutting each one into
 * two smaller copies of the same family. After a few rounds the screen fills
 * with a pattern that never quite repeats — the hallmark of a Penrose tiling.
 *
 * Sister files: 10_pinwheel.c and 09_sierpinski.c — other "split a shape into
 * smaller copies of itself" demos to compare against.
 * The full math: https://en.wikipedia.org/wiki/Penrose_tiling
 */

/* ── the idea ───────────────────────────────────────────────────────────── *
 *
 * Two triangles do all the work. One is tall and thin (we call it "acute",
 * sharp point at the top). One is wide and shallow ("obtuse", blunt point at
 * the top). Both are built around the golden ratio, the famous number
 * φ ≈ 1.618 that shows up whenever something splits into a 1-to-bigger
 * proportion.
 *
 * The trick: each triangle can be cut into two smaller triangles drawn from
 * the same two-shape family. Cut those, cut their pieces, and so on. The
 * cut always lands at the golden-ratio point along one edge, which is why
 * the pattern never settles into a repeating grid — you can't tile the plane
 * periodically with an irrational ratio baked in. That "never repeats" quality
 * is what makes it a Penrose-style tiling.
 *
 * A note on honesty: this uses a simplified cut (each triangle makes exactly
 * two children) rather than the textbook Penrose rule (which makes two or
 * three). It looks aperiodic and shows the same core ideas — two shapes, a
 * golden-ratio cut, self-similarity — without the bookkeeping.
 *
 * The cut, in words:
 *   - For the thin (acute) triangle, mark a point along one leg at the
 *     golden-ratio spot, then join it up to form one wide child and one
 *     thin child.
 *   - For the wide (obtuse) triangle, mark a point along the long base at
 *     the golden-ratio spot, then join it up the same way.
 *   Either way: one acute child + one obtuse child, both smaller.
 *
 * The exact corner-by-corner recipe lives next to the substitute() function
 * in §5 — that's where the meaning of each vertex is spelled out.
 *
 * What you'll see: bump the depth up with +/-. Depth 0 is just the starting
 * triangle; each step doubles the triangle count, so depth 10 is ~1024 of
 * them. Around depth 4 and up the familiar Penrose motifs (suns, stars,
 * kites, darts) start to appear. Acute and obtuse triangles are drawn in
 * different colors so you can see which is which.
 *
 * References:
 *   Penrose, "Pentaplexity" (1979)
 *   Robinson, "Undecidability and Nonperiodicity for Tilings" (1971)
 *   Senechal, "Quasicrystals and Geometry" (1995) §7
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

/* ── §4 formula — screen layout + line drawing ── */

#define TYPE_ACUTE   0
#define TYPE_OBTUSE  1

/*
 * GridCtx — everything we need to place the tiling on this particular screen.
 * It bundles the terminal size, where the center of the drawing sits, and the
 * three knobs the user controls: how many rounds of cutting (depth), how big
 * the starting triangle is (size_frac), and which triangle we start from.
 */
typedef struct {
    int    rows, cols;       /* terminal size, in characters */
    int    cw, ch;           /* sub-cell resolution: how many sub-units fit in one character cell */
    int    ox, oy;           /* center of the drawing, in sub-cell units */
    int    depth;            /* how many rounds of cutting to do */
    double size_frac;        /* how much of the screen the starting triangle fills, 0..1 */
    int    seed_type;        /* which triangle we begin with: TYPE_ACUTE or TYPE_OBTUSE */
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

/* Pick the ASCII character (- | / \) that best matches the slope of a line. */
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

/* ── §5 mesh — cut each triangle into smaller ones ── */

/*
 * PTri — one triangle in the tiling, plus which family it belongs to.
 * We always keep the corners in a fixed order: corner 0 is the apex (the
 * sharp 36° point for acute, the blunt 108° point for obtuse), and corners
 * 1 and 2 are the two base corners. Every cut preserves this order so the
 * children come out with their apex in the right place automatically.
 */
typedef struct {
    double x[3], y[3];      /* corner 0 = apex, corners 1 and 2 = base */
    int    type;            /* TYPE_ACUTE (thin) or TYPE_OBTUSE (wide) */
} PTri;

static void tri_draw_edges(const GridCtx *g, PTri t)
{
    int pair = (t.type == TYPE_ACUTE) ? PAIR_ACUTE : PAIR_OBTUSE;
    int attr = COLOR_PAIR(pair);
    line_draw(g, t.x[0], t.y[0], t.x[1], t.y[1], attr);
    line_draw(g, t.x[1], t.y[1], t.x[2], t.y[2], attr);
    line_draw(g, t.x[2], t.y[2], t.x[0], t.y[0], attr);
}

/*
 * The heart of the demo: cut one triangle into two smaller ones, then do the
 * same to those, until we've gone as deep as the user asked. When we hit the
 * bottom we stop cutting and actually draw the triangle.
 *
 * Each cut marks a point P along one edge at the golden-ratio spot (that's
 * what INV_PHI is), then wires P up to the existing corners to form one acute
 * child and one obtuse child. The exact corners used:
 *
 *   Starting from a thin (acute) triangle:
 *     P sits along the leg from apex toward base corner 1.
 *     Wide child: apex at P, base across corners 0 and 2.
 *     Thin child: apex at corner 2, base across P and corner 1.
 *
 *   Starting from a wide (obtuse) triangle:
 *     P sits along the long base between corners 1 and 2.
 *     Thin child: apex at corner 1, base across corner 0 and P.
 *     Wide child: apex at P, base across corners 0 and 2.
 */
static void substitute(const GridCtx *g, PTri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(g, t); return; }

    if (t.type == TYPE_ACUTE) {
        double Px = t.x[0] + (t.x[1] - t.x[0]) * INV_PHI;
        double Py = t.y[0] + (t.y[1] - t.y[0]) * INV_PHI;
        PTri c0 = { { Px, t.x[0], t.x[2] }, { Py, t.y[0], t.y[2] }, TYPE_OBTUSE };
        PTri c1 = { { t.x[2], Px, t.x[1] }, { t.y[2], Py, t.y[1] }, TYPE_ACUTE  };
        substitute(g, c0, depth + 1, max_depth);
        substitute(g, c1, depth + 1, max_depth);
    } else {
        double Px = t.x[1] + (t.x[2] - t.x[1]) * INV_PHI;
        double Py = t.y[1] + (t.y[2] - t.y[1]) * INV_PHI;
        PTri c0 = { { t.x[1], t.x[0], Px }, { t.y[1], t.y[0], Py }, TYPE_ACUTE  };
        PTri c1 = { { Px, t.x[0], t.x[2] }, { Py, t.y[0], t.y[2] }, TYPE_OBTUSE };
        substitute(g, c0, depth + 1, max_depth);
        substitute(g, c1, depth + 1, max_depth);
    }
}

/*
 * Build the very first triangle — the one everything else is cut from. It's
 * centered on screen with its point at the top, and sized to fill the
 * fraction of the screen the user picked. Returns either a thin or wide
 * starting triangle depending on which seed type is selected.
 */
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

/*
 * Scene — the live state the user is steering: how deep to recurse, how big
 * the starting triangle is, which triangle to start from, the color theme,
 * and whether the demo is paused. This is what the keypresses in main() edit.
 */
typedef struct {
    int    depth;           /* rounds of cutting, 0..DEPTH_MAX */
    double size_frac;       /* starting triangle's share of the screen, 0..1 */
    int    seed_type;       /* starting triangle: TYPE_ACUTE or TYPE_OBTUSE */
    int    theme;           /* which color palette, 0..N_THEMES-1 */
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
    substitute(g, seed, 0, s->depth);
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
