/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 07_barycentric.c — recursive barycentric subdivision of one triangle
 *
 * DEMO: One big equilateral triangle is recursively split into 6 smaller
 *       triangles via "barycentric subdivision" — each step adds the
 *       centroid plus the three edge midpoints, then connects them.
 *       Use +/- to change recursion depth (0..5). At depth 0 you see the
 *       original triangle; at depth 1 the kisrhombille of one tri; at
 *       depth N there are 6^N leaf triangles.
 *
 * Study alongside: 04_30_60_90.c — single-level barycentric subdivision
 *                  applied uniformly to the equilateral tiling.
 *                  08_triforce.c — 4-way midpoint split (no centroid).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, DEPTH, SIZE_FRAC
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — depth-keyed palette + HUD / hint
 *   §4 formula  — slope_char + Bresenham line_draw
 *   §5 subdivide — 6-way recursive split, leaf emitter
 *   §6 scene    — seed_triangle + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  +/- depth   [/] size   r reset   t theme   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/07_barycentric.c \
 *       -o 07_barycentric -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Barycentric subdivision (algebraic topology). Take a
 *                  triangle with vertices V0, V1, V2. Compute centroid
 *                  C = (V0+V1+V2)/3 and midpoints Mij = (Vi+Vj)/2. The
 *                  six new triangles are
 *                    (C, V0, M01), (C, M01, V1), (C, V1, M12),
 *                    (C, M12, V2), (C, V2, M20), (C, M20, V0).
 *                  Each sub-triangle is a 30-60-90 right triangle.
 *                  Recurse for N levels: 6^N leaves.
 *
 * Formula        : Centroid:    C = (V0+V1+V2)/3
 *                  Midpoints:   Mij = (Vi+Vj)/2
 *                  Six children listed above; recursion depth controlled
 *                  by +/- keys (0..5).
 *
 * Edge chars     : Each leaf triangle draws its 3 edges as Bresenham
 *                  line segments in cell space. Slope→character map:
 *                    |angle| < 22.5°  → '-'
 *                    angle near ±90°  → '|'
 *                    otherwise        → '/' or '\\' (sign of dx·dy)
 *
 * Movement       : None — depth is the user-controlled parameter.
 *                  +/- inc/dec depth, [/] adjust seed triangle size.
 *
 * References     :
 *   Barycentric subdivision  — https://en.wikipedia.org/wiki/Barycentric_subdivision
 *   Hatcher, "Algebraic Topology" §2.1 — formal definition + properties
 *   Bresenham line algorithm — https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take a triangle. Find its centroid and the midpoint of each edge.
 * Connect the centroid to all 6 (3 vertices + 3 midpoints). You get 6
 * smaller triangles. Repeat on each. Result: a self-similar fractal
 * decomposition. We don't store any of the leaves — the recursion just
 * draws their edges directly.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Every depth-N leaf is a 30-60-90 right triangle with one vertex at the
 * parent's centroid, one at a parent's edge midpoint, and one at a parent
 * vertex. The centroid sits at the intersection of the three medians.
 * Walking around the centroid you see 6 sectors — exactly the kisrhombille
 * pattern you saw in 04_30_60_90.c, applied recursively.
 *
 * For depth N, leaf count = 6^N. Each leaf draws 3 edges. Many edges are
 * shared between leaves; we draw them twice without optimisation — both
 * Bresenham passes paint the same pixels with the same character.
 *
 * DRAWING METHOD  (recursive emit, the approach used here)
 * ──────────────
 *  1. Pick DEPTH and SIZE_FRAC (fraction of screen the seed fills).
 *  2. Build the seed triangle as 3 (x, y) pixel coords centered on screen.
 *  3. Recursive function subdivide(t, depth):
 *       if depth == max_depth: draw t's 3 edges with line_draw + slope_char
 *       else:
 *         compute C and 3 midpoints
 *         build 6 children
 *         recurse on each
 *  4. Bresenham line_draw for each leaf edge in cell-space.
 *
 * KEY FORMULAS
 * ────────────
 *  Centroid of triangle (V0, V1, V2):
 *    C = ((V0.x + V1.x + V2.x) / 3,  (V0.y + V1.y + V2.y) / 3)
 *
 *  Midpoint of edge (Vi, Vj):
 *    Mij = ((Vi.x + Vj.x) / 2,  (Vi.y + Vj.y) / 2)
 *
 *  Six children of (V0, V1, V2):
 *    (C, V0, M01), (C, M01, V1), (C, V1, M12),
 *    (C, M12, V2), (C, V2, M20), (C, M20, V0)
 *
 *  Slope → character (after aspect-correcting for CELL_W, CELL_H):
 *    angle = atan2(|dy|/CELL_H, |dx|/CELL_W)
 *    angle < 22.5°       → '-'
 *    angle > 67.5°       → '|'
 *    same sign (dx, dy)  → '\\'
 *    opposite signs      → '/'
 *
 *  Equilateral seed centered on screen with apex up:
 *    base = SIZE_FRAC · min(screen_w_pix, screen_h_pix)
 *    h    = base · √3 / 2
 *    V0 = (cx,            cy − 2·h/3)   ← apex
 *    V1 = (cx − base/2,   cy +   h/3)
 *    V2 = (cx + base/2,   cy +   h/3)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Recursion depth is bounded by stack size, which for 6^N at N=5 is
 *    fine (~7000 stack frames in the deepest path). N=6 would be 46656
 *    leaves — likely slow at 60 fps.
 *  • Bresenham draws each leaf edge in cell-space. Adjacent leaves share
 *    an edge; we draw it twice. The repaint is harmless (same character,
 *    same color) but doubles the line work. Tolerable at the depths used.
 *  • Color is keyed to depth, not leaf identity. All leaves at the same
 *    recursion depth are the same color — visually communicates the
 *    hierarchical structure.
 *  • Resize: recompute the seed triangle each frame from the current
 *    screen size. No state to recompute; the recursion is pure.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At depth 0: 6^0 = 1 leaf — the original equilateral. HUD shows
 *    "depth:0 leaves:1".
 *  At depth 1: 6 leaves, all 30-60-90s meeting at the equilateral's
 *    centroid. HUD shows "depth:1 leaves:6".
 *  At depth N: HUD shows leaves = 6^N.
 *
 *  Quick sanity: every leaf at depth ≥ 1 has at least one vertex at the
 *  parent's centroid (which is the parent's centroid at depth ≥ 2 too,
 *  recursively). So you should see 6 distinct edges meeting at every
 *  centroid in the figure.
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

#define DEPTH_DEFAULT 2
#define DEPTH_MIN     0
#define DEPTH_MAX     5

#define SIZE_FRAC_DEFAULT 0.85
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
    init_pair(PAIR_HUD,  COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT, COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — slope_char + Bresenham line                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * slope_char — pixel-space (dx, dy) → ASCII line character.
 *
 * Aspect-correct by normalising dx, dy by (CELL_W, CELL_H) before
 * computing the angle. Lines are bucketed into 4 classes:
 *   |angle| < 22.5°    → '-'
 *   |angle| > 67.5°    → '|'
 *   same-sign dx, dy   → '\\'
 *   opposite-sign      → '/'
 */
static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

/*
 * line_draw — Bresenham in cell space.
 *
 * Pixel endpoints (px0, py0) → (px1, py1) are converted to cell coords
 * by integer division by (CELL_W, CELL_H). The character is constant
 * along the line (chosen by slope_char of the pixel delta).
 */
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
/* §5  subdivide — 6-way recursive split                                   */
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
 * subdivide — emit leaves at max_depth, otherwise split into 6 children.
 *
 * Six children are formed by the centroid + 3 edge midpoints. Each
 * child is (C, V_i, M_ij) traversing around the parent counter-clockwise.
 */
static void subdivide(int rows, int cols, Tri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(rows, cols, t, depth); return; }
    double cx  = (t.x[0] + t.x[1] + t.x[2]) / 3.0;
    double cy  = (t.y[0] + t.y[1] + t.y[2]) / 3.0;
    double m01x = (t.x[0] + t.x[1]) * 0.5, m01y = (t.y[0] + t.y[1]) * 0.5;
    double m12x = (t.x[1] + t.x[2]) * 0.5, m12y = (t.y[1] + t.y[2]) * 0.5;
    double m20x = (t.x[2] + t.x[0]) * 0.5, m20y = (t.y[2] + t.y[0]) * 0.5;
    Tri subs[6] = {
        { {cx, t.x[0], m01x}, {cy, t.y[0], m01y} },
        { {cx, m01x, t.x[1]}, {cy, m01y, t.y[1]} },
        { {cx, t.x[1], m12x}, {cy, t.y[1], m12y} },
        { {cx, m12x, t.x[2]}, {cy, m12y, t.y[2]} },
        { {cx, t.x[2], m20x}, {cy, t.y[2], m20y} },
        { {cx, m20x, t.x[0]}, {cy, m20y, t.y[0]} },
    };
    for (int i = 0; i < 6; i++)
        subdivide(rows, cols, subs[i], depth + 1, max_depth);
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

/*
 * scene_seed — equilateral seed triangle centered on screen, apex up.
 *
 *   base = SIZE_FRAC · min(pw, ph)
 *   h    = base · √3 / 2
 *   V0 = (cx,           cy − 2·h/3)   ← apex
 *   V1 = (cx − base/2,  cy +   h/3)
 *   V2 = (cx + base/2,  cy +   h/3)
 */
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

    long leaves = 1; for (int i = 0; i < s->depth; i++) leaves *= 6;
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
             " +/-:depth  [/]:size  t:theme  r:reset  p:pause  q:quit  [07 barycentric] ");
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
