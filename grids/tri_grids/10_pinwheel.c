/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 10_pinwheel.c — pinwheel-inspired 5-way substitution of a 1-2-√5 triangle
 *
 * DEMO: A right triangle with legs 1 and 2 (hypotenuse √5) is recursively
 *       split into 5 sub-triangles. The split combines the standard 4-way
 *       midpoint subdivision with one extra cut: the inverted centre
 *       child is bisected by its altitude from the right-angle vertex.
 *       Use +/- to change recursion depth (0..6); leaf count grows as 5^N.
 *
 * Study alongside: 08_triforce.c — the underlying 4-way midpoint split.
 *                  12_penrose.c — another aperiodic substitution but
 *                  with golden-ratio (φ) splits and 2 prototiles.
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, DEPTH, SIZE_FRAC
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — depth-keyed palette + HUD / hint
 *   §4 formula  — slope_char + Bresenham line_draw + foot_perp
 *   §5 subdivide — 5-way recursive split
 *   §6 scene    — seed_triangle (1-2-√5) + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  +/- depth   [/] size   r reset   t theme   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/10_pinwheel.c \
 *       -o 10_pinwheel -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : "Pinwheel-style" substitution of a 1-2-√5 right triangle.
 *                  Step:
 *                    1. Compute midpoints M01, M12, M20 — the standard
 *                       4-way midpoint subdivision.
 *                    2. The inverted centre child (M01, M12, M20) is
 *                       similar to the parent. Find its right-angle vertex
 *                       and the foot of perpendicular F to the opposite
 *                       hypotenuse. Split the centre into halves using F.
 *                    3. Emit 5 children: 3 corner triangles + 2 altitude
 *                       halves of the centre.
 *                  All 5 children are similar to the parent (1:2:√5 ratio).
 *
 * NOTE: This is NOT strict Conway-Radin pinwheel. The original uses 5
 *       IDENTICALLY-scaled children at scale 1/√5 with specific rotations
 *       of arctan(1/2). This file uses a related substitution where the
 *       5 children are similar but at TWO scales (1/2 for the corners,
 *       1/√5 and 1/(2√5) for the altitude halves). Visually it's just as
 *       aperiodic-looking; the difference is bookkeeping not topology.
 *
 * Formula        : Midpoints + foot of perpendicular (see §4 foot_perp).
 *                  Recursion depth controlled by +/- keys (0..6).
 *
 * Edge chars     : Same Bresenham + slope_char as 07/08/09. Color keyed
 *                  to depth.
 *
 * Movement       : None — depth is the user-controlled parameter.
 *
 * References     :
 *   Pinwheel tiling — https://en.wikipedia.org/wiki/Pinwheel_tiling
 *   Radin, "The Pinwheel Tilings of the Plane" (1994)
 *   Conway & Radin, "Quaquaversal Tilings and Rotations" (1998)
 *   Sadun, "Topology of Tiling Spaces" (2008) §1
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take a 1-2-√5 right triangle. Apply midpoint subdivision (4 children).
 * The inverted centre child is itself a 1-2-√5 right triangle — split it
 * once more along its altitude from the right-angle corner. Net: 5 child
 * triangles per parent, all similar to the parent. Recurse on each.
 *
 * Why "pinwheel-inspired"? The strict Conway-Radin pinwheel rule produces
 * 5 children of EQUAL size with rotations of arctan(1/2) ≈ 26.57°
 * relative to the parent. That irrational rotation is what makes the
 * tiling truly aperiodic and produces the famous pinwheel pattern. Our
 * simpler 4+1-cut rule keeps the 5-way fan-out and the visual character
 * of nested rotated triangles, but loses the equal-size constraint.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * At every depth, the inverted centre child is a 1-2-√5 right triangle
 * rotated 180° from the parent. Cutting it along its altitude from the
 * right angle slices it into TWO smaller 1-2-√5 right triangles (a known
 * property of right triangles: the altitude from the right angle to the
 * hypotenuse splits the triangle into two similar to itself). So now we
 * have 3 corner children + 2 altitude halves = 5, all 1-2-√5.
 *
 * The corner children are at scale 1/2; the altitude halves are at
 * scales 1/√5 and 1/(2√5) respectively (corresponding to the cosine and
 * sine of the smaller acute angle).
 *
 * DRAWING METHOD  (recursive emit)
 * ──────────────
 *  1. Pick DEPTH and SIZE_FRAC.
 *  2. Build seed triangle as 3 (x, y) pixel coords (legs along screen axes,
 *     right angle at the lower-right corner — convention: v[1] is the
 *     right-angle vertex).
 *  3. subdivide(t, depth):
 *       if depth == max_depth: draw 3 edges
 *       else:
 *         compute 3 midpoints
 *         build 3 corner children (scale 1/2)
 *         compute foot of altitude from m20 to hypotenuse m01-m12
 *         build 2 altitude halves
 *         recurse on all 5
 *  4. Bresenham line_draw per leaf edge.
 *
 * KEY FORMULAS
 * ────────────
 *  Midpoints:
 *    M01 = (V0+V1)/2,  M12 = (V1+V2)/2,  M20 = (V2+V0)/2
 *
 *  Foot of perpendicular from point P to line segment AB:
 *    t = ((P − A) · (B − A)) / |B − A|²
 *    F = A + t · (B − A)
 *
 *  Five children:
 *    Corner-V0:  (V0, M01, M20)        — scale 1/2
 *    Corner-V1:  (M01, V1, M12)        — scale 1/2
 *    Corner-V2:  (M20, M12, V2)        — scale 1/2
 *    Centre half 1:  (M20, M01, F)     — scale 1/√5
 *    Centre half 2:  (M20, F,   M12)   — scale 1/(2√5)
 *
 *  Leaf count at depth N: 5^N
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The inverted centre's right angle is at M20 (opposite the longest
 *    side M01-M12). The altitude from M20 lands somewhere on M01-M12;
 *    foot_perp computes that exactly.
 *  • Different child scales mean the recursion at depth N has a mix of
 *    triangle sizes — UNLIKE 08_triforce where all leaves at depth N
 *    are the same size. Visual gives more variety.
 *  • 5^6 = 15625 leaves; 5^7 = 78125 — too slow at 60 fps. DEPTH_MAX = 6.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At depth 0: 1 leaf — the seed 1-2-√5 right triangle.
 *  At depth 1: 5 leaves — 3 corner + 2 altitude halves of the centre.
 *  At depth 2: 25 leaves; visible aperiodic-ish pattern.
 *  Leaf count formula: 5^N.
 *
 *  The hypotenuse of every leaf at depth N is ≈ √5/2^N (for corner
 *  children) or various rational multiples thereof (for altitude halves).
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

#define DEPTH_DEFAULT 3
#define DEPTH_MIN     0
#define DEPTH_MAX     6

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
    init_pair(PAIR_HUD,  COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT, COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — slope_char + Bresenham line + foot_perp                   */
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

/*
 * foot_perp — foot of perpendicular from point P to segment AB.
 *
 * THE FORMULA:
 *   t = ((P − A) · (B − A)) / |B − A|²
 *   F = A + t · (B − A)
 *
 * Used to find the altitude foot on the hypotenuse of the inverted centre.
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  subdivide — pinwheel-style 5-way split                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Triangle convention:
 *   t.v[0] = "v0", t.v[1] = "v1" (right-angle vertex), t.v[2] = "v2"
 * For the pinwheel parent (legs 2 and 1):
 *   v0 = acute angle arctan(1/2)
 *   v1 = right angle      (90°)
 *   v2 = acute angle arctan(2)
 * Children inherit the same role labelling so the recursion is uniform.
 */
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
 * subdivide — pinwheel-style 5-way split.
 *
 *   1. Compute midpoints m01, m12, m20.
 *   2. Three corner children (scale 1/2):
 *        c0: (v0, m01, m20)
 *        c1: (m01, v1, m12)            ← right angle stays at v1 slot
 *        c2: (m20, m12, v2)
 *   3. Inverted centre = (m01, m12, m20). Its right angle is at m20
 *      (opposite the longest side m01-m12). Compute the altitude foot
 *      F on m01-m12 from m20.
 *   4. Two altitude halves:
 *        c3: (m20, m01, F)
 *        c4: (m20, F,   m12)
 */
static void subdivide(int rows, int cols, Tri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(rows, cols, t, depth); return; }
    double m01x = (t.x[0] + t.x[1]) * 0.5, m01y = (t.y[0] + t.y[1]) * 0.5;
    double m12x = (t.x[1] + t.x[2]) * 0.5, m12y = (t.y[1] + t.y[2]) * 0.5;
    double m20x = (t.x[2] + t.x[0]) * 0.5, m20y = (t.y[2] + t.y[0]) * 0.5;

    /* Corner children (scale 1/2) */
    Tri c0 = { {t.x[0], m01x, m20x}, {t.y[0], m01y, m20y} };
    Tri c1 = { {m01x, t.x[1], m12x}, {m01y, t.y[1], m12y} };
    Tri c2 = { {m20x, m12x, t.x[2]}, {m20y, m12y, t.y[2]} };

    /* Altitude foot on hypotenuse m01-m12, dropped from m20 */
    double fx, fy;
    foot_perp(m20x, m20y, m01x, m01y, m12x, m12y, &fx, &fy);
    Tri c3 = { {m20x, m01x, fx}, {m20y, m01y, fy} };
    Tri c4 = { {m20x, fx, m12x}, {m20y, fy, m12y} };

    subdivide(rows, cols, c0, depth + 1, max_depth);
    subdivide(rows, cols, c1, depth + 1, max_depth);
    subdivide(rows, cols, c2, depth + 1, max_depth);
    subdivide(rows, cols, c3, depth + 1, max_depth);
    subdivide(rows, cols, c4, depth + 1, max_depth);
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
 * Seed: a 1-2-√5 right triangle centered on screen.
 *   v0 = lower-left  (acute, far end of long leg)
 *   v1 = lower-right (right angle)
 *   v2 = upper-right (acute, far end of short leg)
 */
static Tri scene_seed(const Scene *s, int rows, int cols)
{
    double pw = (double)cols * CELL_W;
    double ph = (double)(rows - 1) * CELL_H;
    double base = (pw < ph ? pw : ph) * s->size_frac * 0.5;   /* leg-2 = base */
    double cxp = pw * 0.5;
    double cyp = ph * 0.5 + base * 0.25;
    Tri t = {
        { cxp - base, cxp + base, cxp + base                },
        { cyp,        cyp,        cyp - base * 0.5          },
    };
    return t;
}

static void scene_draw(int rows, int cols, const Scene *s, double fps)
{
    erase();
    Tri seed = scene_seed(s, rows, cols);
    subdivide(rows, cols, seed, 0, s->depth);

    long leaves = 1; for (int i = 0; i < s->depth; i++) leaves *= 5;
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
             " +/-:depth  [/]:size  t:theme  r:reset  p:pause  q:quit  [10 pinwheel] ");
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
