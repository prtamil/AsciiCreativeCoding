/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_30_60_90.c — kisrhombille tiling (equilaterals subdivided into 6 right tris)
 *
 * DEMO: The equilateral grid from 01 is dressed with the three medians
 *       of each triangle. Each median connects a vertex to the midpoint
 *       of the opposite edge; together they cut every equilateral into 6
 *       congruent 30-60-90 right triangles. Arrow keys move the cursor
 *       between whole equilaterals (medians render automatically); the 6
 *       sub-triangles are visible by the median lines.
 *
 * Study alongside: 01_equilateral.c — same skew-lattice rasterizer.
 *                  03_double_diagonal.c — the analogous "kis" operation
 *                  applied to squares (6-way for triangles, 4-way for
 *                  squares).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, BORDER_W, MEDIAN_T
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: edge / median / cursor / HUD / hint
 *   §4 formula  — pixel ↔ skew lattice + edge + 3 median proximity
 *   §5 cursor   — TRI_DIR table (same as 01) + cursor_draw
 *   §6 scene    — grid_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   t theme   p pause
 *        +/- size        [/] border thickness   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/04_30_60_90.c \
 *       -o 04_30_60_90 -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Kisrhombille = "kis" (apply triangulation) of the
 *                  rhombille. Every equilateral triangle is split by its
 *                  three medians (vertex → opposite-edge midpoint) into 6
 *                  right triangles with angles 30°-60°-90°. The medians
 *                  meet at the centroid where six 60° angles complete
 *                  to 360°. Twelve 30-60-90s meet at every original vertex.
 *
 * Formula        : Same pixel→skew-lattice as 01_equilateral.c. Inside
 *                  each triangle, three additional median-line proximity
 *                  tests use signed-distance to line equations:
 *                    ▽ medians:  fa−fb=0,  fa+2·fb−1=0,  2·fa+fb−1=0
 *                    △ medians:  fa−fb=0,  2·fa+fb−2=0,  fa+2·fb−2=0
 *                  Distance is normalized by √(aL²+bL²) for each line.
 *
 * Edge chars     : Equilateral edges as in 01. Median characters keyed
 *                  to the median's slope:
 *                    ▽: '\\', '/', '|'
 *                    △: '\\', '|', '/'
 *                  (the diagonal fa=fb is shared between adjacent ▽ and △)
 *
 * Movement       : Same TRI_DIR as 01_equilateral.c — cursor walks whole
 *                  equilaterals; the 6 sub-triangles are visual only.
 *
 * References     :
 *   Kisrhombille tiling   — https://en.wikipedia.org/wiki/Kisrhombille_tiling
 *   30-60-90 triangle     — https://en.wikipedia.org/wiki/Special_right_triangle
 *   Conway, "Symmetries of Things" §22 — kis-/truncation operations
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take 01's equilateral grid. Inside every triangle, draw the three
 * medians. Each median is a straight line from a vertex to the midpoint
 * of the opposite edge — they all meet at the centroid. The result is
 * the "kisrhombille" tiling: each equilateral is now 6 small 30-60-90
 * right triangles. We don't store any of them; we add 3 extra distance
 * tests per pixel on top of 01's edge tests.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The medians live in lattice space as straight lines aL·fa + bL·fb + cL = 0.
 * The perpendicular distance from a fractional lattice point (fa, fb) to
 * such a line is |aL·fa + bL·fb + cL| / √(aL² + bL²). If this distance is
 * below MEDIAN_T, the pixel is "on" the median — paint it with the median
 * color and the angle-appropriate ASCII character.
 *
 * For ▽ (lower-left half of rhombus, fa+fb < 1):
 *   - Median from P00 to mid(P10, P01) lives on line  fa − fb = 0.
 *   - Median from P10 to mid(P00, P01) lives on line  fa + 2·fb − 1 = 0.
 *   - Median from P01 to mid(P00, P10) lives on line  2·fa + fb − 1 = 0.
 *
 * For △ (upper-right half, fa+fb ≥ 1):
 *   - Median from P11 to mid(P10, P01) lives on line  fa − fb = 0.
 *     (the SAME line as ▽'s first median — together they form one rhombus
 *     diagonal that passes through both centroids)
 *   - Median from P10 to mid(P11, P01) lives on line  2·fa + fb − 2 = 0.
 *   - Median from P01 to mid(P10, P11) lives on line  fa + 2·fb − 2 = 0.
 *
 * DRAWING METHOD  (raster scan, the approach used here)
 * ──────────────
 *  1. Pick TRI_SIZE; compute h = TRI_SIZE · √3/2.
 *  2. For every screen cell, run pixel→lattice as in 01.
 *  3. Compute the 3 edge weights (l₁, l₂, l₃) and minimum em + char ech.
 *  4. Compute the 3 median signed distances and minimum mm + char mch.
 *  5. Choose what to draw:
 *       em < BORDER_W and em ≤ mm  →  EDGE character (PAIR_BORDER)
 *       mm < MEDIAN_T               →  MEDIAN character (PAIR_MEDIAN)
 *       otherwise                   →  interior, skip
 *  6. Cursor highlight uses PAIR_CURSOR for whichever wins.
 *
 * KEY FORMULAS
 * ────────────
 *  Same lattice inverse as 01:  b = py/h,  a = px/size − 0.5·b.
 *  Triangle id and barycentric weights — also same as 01.
 *
 *  Median signed distances (lattice-perpendicular):
 *    ▽: m1 = |fa − fb| / √2
 *       m2 = |fa + 2·fb − 1| / √5
 *       m3 = |2·fa + fb − 1| / √5
 *    △: m1 = |fa − fb| / √2
 *       m2 = |2·fa + fb − 2| / √5
 *       m3 = |fa + 2·fb − 2| / √5
 *
 *  Median character choice:
 *    ▽: m1→'\\'  m2→'/'   m3→'|'
 *    △: m1→'\\'  m2→'|'   m3→'/'
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Lattice space is skew, so "lattice perpendicular distance" is not
 *    quite the same as "pixel perpendicular distance". The medians render
 *    with a slight thickness anisotropy. Acceptable for a teaching demo;
 *    a production renderer would use pixel-space line equations.
 *  • The diagonal fa = fb is the SAME line for ▽'s M1 and △'s M2 — at
 *    the rhombus boundary they merge into one continuous rendered line.
 *  • Edge wins ties with the median: we draw the equilateral edge if the
 *    edge weight is below border_w AND ≤ the median minimum. This keeps
 *    the outer triangles' edges visually dominant over internal medians.
 *  • MEDIAN_T tuned by trial — too small and medians vanish; too big and
 *    they bleed into the interior.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At lattice (fa, fb) = (0.5, 0.5) — the rhombus centre, on the diagonal:
 *    ▽ side test fa+fb=1 → pick △ branch (boundary, classifier picks one).
 *    △ M2 distance = |2·0.5 + 0.5 − 2| / √5 = 0.5/√5 ≈ 0.224 — far from M2.
 *    △ M3 distance = |0.5 + 2·0.5 − 2| / √5 = 0.5/√5 ≈ 0.224 — far from M3.
 *    △ M1 distance = |0.5 − 0.5| / √2 = 0 — exactly on M1.
 *  Result: draws '\\' (M1's character). ✓ The rhombus diagonal renders
 *  as a single '\\' through the centre.
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 18.0
#define TRI_SIZE_MIN      8.0
#define TRI_SIZE_MAX     48.0
#define TRI_SIZE_STEP     2.0

#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.30
#define BORDER_W_STEP    0.02

/* MEDIAN_T = perpendicular-distance threshold in lattice units. */
#define MEDIAN_T 0.05

#define N_THEMES 4

#define PAIR_BORDER 1
#define PAIR_MEDIAN 2
#define PAIR_CURSOR 3
#define PAIR_HUD    4
#define PAIR_HINT   5

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

static const short THEME_FG[N_THEMES][2] = {
    /* edge,  median */
    {  75,  39 },
    {  82, 226 },
    { 207, 196 },
    {  15,  87 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_BLUE   },
    { COLOR_GREEN,   COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_RED    },
    { COLOR_WHITE,   COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e, fg_m;
    if (COLORS >= 256) { fg_e = THEME_FG[theme][0];   fg_m = THEME_FG[theme][1];   }
    else               { fg_e = THEME_FG_8[theme][0]; fg_m = THEME_FG_8[theme][1]; }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_MEDIAN, fg_m, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — pixel ↔ lattice + edge + median                           */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * pixel_to_tri — same skew-lattice inverse as 01_equilateral.c §4.
 */
static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *up,
                         double *fa, double *fb)
{
    double h = size * sqrt(3.0) * 0.5;
    double b = py / h;
    double a = px / size - 0.5 * b;
    int    c = (int)floor(a);
    int    r = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c;
    *fb = b - (double)r;
    *up = (*fa + *fb >= 1.0) ? 1 : 0;
}

/*
 * tri_centroid_pixel — same as 01_equilateral.c §4.
 */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

/*
 * tri_edge_char — same as 01_equilateral.c §4.
 */
static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    if (up == 0) {
        l1 = 1.0 - fa - fb; ch1 = '/';
        l2 = fa;            ch2 = '\\';
        l3 = fb;            ch3 = '_';
    } else {
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa + fb - 1.0;  ch2 = '/';
        l3 = 1.0 - fa;       ch3 = '\\';
    }
    char   ch = ch1;
    double m  = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/*
 * tri_median_char — three median signed distances, return min and char.
 *
 * THE FORMULA (perpendicular distance to line aL·fa + bL·fb + cL = 0):
 *
 *   d = |aL·fa + bL·fb + cL| / √(aL² + bL²)
 *
 * Three lines per triangle (different per orientation). The smallest
 * distance picks the median character.
 */
static char tri_median_char(int up, double fa, double fb, double *out_min)
{
    static const double INV_SQRT2 = 0.70710678118654752440;
    static const double INV_SQRT5 = 0.44721359549995793928;
    double m1, m2, m3;
    char   ch1, ch2, ch3;
    if (up == 0) {                          /* ▽ */
        m1 = fabs(fa - fb)         * INV_SQRT2; ch1 = '\\';
        m2 = fabs(fa + 2.0*fb - 1.0) * INV_SQRT5; ch2 = '/';
        m3 = fabs(2.0*fa + fb - 1.0) * INV_SQRT5; ch3 = '|';
    } else {                                /* △ */
        m1 = fabs(fa - fb)         * INV_SQRT2; ch1 = '\\';
        m2 = fabs(2.0*fa + fb - 2.0) * INV_SQRT5; ch2 = '|';
        m3 = fabs(fa + 2.0*fb - 2.0) * INV_SQRT5; ch3 = '/';
    }
    char   ch = ch1;
    double m  = m1;
    if (m2 < m) { m = m2; ch = ch2; }
    if (m3 < m) { m = m3; ch = ch3; }
    *out_min = m;
    return ch;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    col, row, up;
    double tri_size;
    double border_w;
    int    theme;
    int    paused;
} Cursor;

/* Same TRI_DIR as 01_equilateral.c — see that file for the derivation. */
static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 }, {  0,  0,  0 } },
    /* RIGHT */ { {  0,  0,  1 }, { +1,  0,  0 } },
    /* UP    */ { {  0, -1,  1 }, {  0,  0,  0 } },
    /* DOWN  */ { {  0,  0,  1 }, {  0, +1,  0 } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->up = 0;
    cur->tri_size = TRI_SIZE_DEFAULT;
    cur->border_w = BORDER_W_DEFAULT;
    cur->theme    = 0;
    cur->paused   = 0;
}

static void cursor_step(Cursor *cur, int dir)
{
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0];
    cur->row += t[1];
    cur->up   = t[2];
}

static void cursor_draw(const Cursor *cur, int rows, int cols, int ox, int oy)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(cur->col, cur->row, cur->up, cur->tri_size,
                       &cx_pix, &cy_pix);
    int col = ox + (int)(cx_pix / CELL_W);
    int row = oy + (int)(cy_pix / CELL_H);
    if (col >= 0 && col < cols && row >= 0 && row < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void grid_draw(int rows, int cols, const Cursor *cur, int ox, int oy)
{
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;

            int    tC, tR, tU;
            double fa, fb, em, mm;
            pixel_to_tri(px, py, cur->tri_size, &tC, &tR, &tU, &fa, &fb);
            char ech = tri_edge_char(tU, fa, fb, &em);
            char mch = tri_median_char(tU, fa, fb, &mm);

            int on_cur = (tC == cur->col && tR == cur->row && tU == cur->up);
            if (em < cur->border_w && em <= mm) {
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ech);
                attroff(attr);
            } else if (mm < MEDIAN_T) {
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : COLOR_PAIR(PAIR_MEDIAN);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)mch);
                attroff(attr);
            }
        }
    }
}

static void scene_draw(int rows, int cols, const Cursor *cur, double fps)
{
    erase();
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur, ox, oy);
    cursor_draw(cur, rows, cols, ox, oy);

    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             cur->tri_size, cur->border_w, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [04 30-60-90] ");
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

    Cursor cur;
    cursor_reset(&cur);
    screen_init(cur.theme);

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
                case 'p':           cur.paused ^= 1; break;
                case 'r':           cursor_reset(&cur); color_init(cur.theme); break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_step(&cur, 0); break;
                case KEY_RIGHT: cursor_step(&cur, 1); break;
                case KEY_UP:    cursor_step(&cur, 2); break;
                case KEY_DOWN:  cursor_step(&cur, 3); break;
                case '+': case '=':
                    if (cur.tri_size < TRI_SIZE_MAX) { cur.tri_size += TRI_SIZE_STEP; } break;
                case '-':
                    if (cur.tri_size > TRI_SIZE_MIN) { cur.tri_size -= TRI_SIZE_STEP; } break;
                case '[':
                    if (cur.border_w > BORDER_W_MIN) { cur.border_w -= BORDER_W_STEP; } break;
                case ']':
                    if (cur.border_w < BORDER_W_MAX) { cur.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;

        scene_draw(rows, cols, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
