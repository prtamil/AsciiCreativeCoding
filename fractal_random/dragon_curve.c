/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * dragon_curve.c — Paper-Folding Dragon Curve
 *
 * The dragon curve is built by iterative right-folding of a paper strip:
 *
 *   Gen 1: [R]
 *   Gen 2: [R  R  L]
 *   Gen 3: [R  R  L  R  R  L  L]
 *
 * Each generation appends a right-turn midpoint plus the reverse-complement
 * of the existing sequence.  The result is a space-filling path that never
 *  self-intersects.
 *
 * Animation draws the path segment by segment.  Color encodes generation
 * depth: dark blue (gen 1-2) → cyan → green → yellow → white (gen 13).
 *
 * Keys: q quit  SPACE pause/resume  r restart  +/- speed  g/G generations
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fractal_random/dragon_curve.c \
 *       -o dragon_curve -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 curve  §5 draw  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Paper-folding sequence construction.
 *                  The turn sequence for generation n is:
 *                    T_n = T_{n-1}  R  reverse-complement(T_{n-1})
 *                  where R=right-turn and reverse-complement flips L↔R and
 *                  reverses order.  This can also be derived by looking at
 *                  the n-th bit of the folding number via: turn(k) = R if
 *                  k / (highest_bit) has the bit pattern ...100...
 *
 * Math           : After n folds the sequence has 2ⁿ−1 turns and 2ⁿ segments.
 *                  Gen 13 → 8191 turns, 8192 segments.  The path never
 *                  self-intersects (proven: each segment is unique).
 *                  Tiles the plane when continued: 4 copies of the dragon
 *                  curve at rotations 0°/90°/180°/270° fill the plane without
 *                  overlap, making it a rep-tile of order 4.
 *
 * Rendering      : Segments drawn one per frame using turtle graphics.
 *                  Color encodes position in the sequence (age of segment),
 *                  producing a visual record of the folding hierarchy.
 *                  Aspect correction: terminal cells are 2× taller than wide,
 *                  so horizontal steps are scaled by CELL_W/CELL_H ≈ 0.5.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_GEN_MIN    4
#define N_GEN_MAX   13
#define N_GEN_INIT  12

/* Max segments = 2^N_GEN_MAX - 1 = 8191 */
#define MAX_SEGS  8191
#define MAX_PTS   8192

#define SPEED_DEFAULT  8
#define RENDER_NS      (1000000000LL / 30)

enum {
    CP_G1 = 1,  /* dark blue    */
    CP_G2,       /* blue         */
    CP_G3,       /* cyan         */
    CP_G4,       /* green        */
    CP_G5,       /* yellow       */
    CP_G6,       /* orange       */
    CP_G7,       /* white        */
    CP_HUD,
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_G1,  18, -1);
        init_pair(CP_G2,  27, -1);
        init_pair(CP_G3,  51, -1);
        init_pair(CP_G4,  46, -1);
        init_pair(CP_G5, 226, -1);
        init_pair(CP_G6, 208, -1);
        init_pair(CP_G7, 231, -1);
        init_pair(CP_HUD, 226, -1);
    } else {
        init_pair(CP_G1, COLOR_BLUE,    -1);
        init_pair(CP_G2, COLOR_BLUE,    -1);
        init_pair(CP_G3, COLOR_CYAN,    -1);
        init_pair(CP_G4, COLOR_GREEN,   -1);
        init_pair(CP_G5, COLOR_YELLOW,  -1);
        init_pair(CP_G6, COLOR_YELLOW,  -1);
        init_pair(CP_G7, COLOR_WHITE,   -1);
        init_pair(CP_HUD, COLOR_YELLOW, -1);
    }
}

/* ===================================================================== */
/* §4  curve                                                              */
/* ===================================================================== */

/* turn[i]: 1 = clockwise (right), 0 = counter-clockwise (left) */
static int g_turns[MAX_SEGS];
static int g_px[MAX_PTS], g_py[MAX_PTS];  /* path in integer grid units  */
static int g_n_segs;
static int g_n_gen;

/* full path bounding box (stable scaling during animation) */
static int g_xmin, g_xmax, g_ymin, g_ymax;

/* direction vectors: 0=right 1=down 2=left 3=up (y increases downward) */
static const int DX[4] = { 1,  0, -1,  0 };
static const int DY[4] = { 0,  1,  0, -1 };

/*
 * seg_generation — generation in which segment i was first added.
 *
 * Pattern: seg 0 = gen 1; segs 1-2 = gen 2; segs 3-6 = gen 3; ...
 * Formula: g = floor(log2(i+1)) + 1.
 */
static int seg_generation(int i)
{
    int g = 1, n = i + 1;
    while (n > 1) { n >>= 1; g++; }
    return g;
}

static int seg_color(int i)
{
    int g = seg_generation(i);
    int ngen = g_n_gen > 1 ? g_n_gen - 1 : 1;
    int pair = 1 + (g - 1) * 6 / ngen;
    if (pair < 1) pair = 1;
    if (pair > 7) pair = 7;
    return pair;
}

static void curve_build(int n_gen)
{
    if (n_gen < N_GEN_MIN) n_gen = N_GEN_MIN;
    if (n_gen > N_GEN_MAX) n_gen = N_GEN_MAX;
    g_n_gen = n_gen;

    /* build turn sequence */
    g_turns[0] = 1;
    g_n_segs   = 1;

    for (int g = 2; g <= n_gen; g++) {
        int old_n = g_n_segs;
        g_turns[old_n] = 1;              /* midpoint: always right */
        for (int i = 0; i < old_n; i++) /* reverse-complement     */
            g_turns[old_n + 1 + i] = !g_turns[old_n - 1 - i];
        g_n_segs = old_n * 2 + 1;
    }

    /* build path */
    g_px[0] = 0; g_py[0] = 0;
    int dir = 0;
    for (int i = 0; i < g_n_segs; i++) {
        g_px[i+1] = g_px[i] + DX[dir];
        g_py[i+1] = g_py[i] + DY[dir];
        dir = g_turns[i] ? (dir + 1) % 4 : (dir + 3) % 4;
    }

    /* bounding box */
    g_xmin = g_px[0]; g_xmax = g_px[0];
    g_ymin = g_py[0]; g_ymax = g_py[0];
    for (int i = 1; i <= g_n_segs; i++) {
        if (g_px[i] < g_xmin) g_xmin = g_px[i];
        if (g_px[i] > g_xmax) g_xmax = g_px[i];
        if (g_py[i] < g_ymin) g_ymin = g_py[i];
        if (g_py[i] > g_ymax) g_ymax = g_py[i];
    }
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;
static int  g_draw_seg;
static int  g_speed = SPEED_DEFAULT;

static void scene_draw(void)
{
    int draw_to = g_draw_seg < g_n_segs ? g_draw_seg : g_n_segs;

    int xspan = g_xmax - g_xmin + 1;
    int yspan = g_ymax - g_ymin + 1;

    int usable_cols = g_cols - 2;
    int usable_rows = g_rows - 4;
    if (usable_cols < 1) usable_cols = 1;
    if (usable_rows < 1) usable_rows = 1;

    /*
     * Aspect correction: terminal cells are ~2× taller than wide.
     * Use x_scale = 2 × y_scale so a square grid unit looks square.
     * Constrain so the path fits in both dimensions.
     */
    float ys = (float)usable_rows / (float)yspan;
    float xs = (float)usable_cols / (float)xspan;
    float scale = ys;
    if (scale * 2.0f > xs) scale = xs * 0.5f;

    float x_scale = scale * 2.0f;
    float y_scale = scale;

    int x_off = 1 + (usable_cols - (int)(xspan * x_scale)) / 2;
    int y_off = 2 + (usable_rows - (int)(yspan * y_scale)) / 2;

    for (int i = 0; i < draw_to; i++) {
        int cp = seg_color(i);
        for (int pt = i; pt <= i + 1; pt++) {
            int col = (int)((g_px[pt] - g_xmin) * x_scale) + x_off;
            int row = (int)((g_py[pt] - g_ymin) * y_scale) + y_off;
            if (row >= 2 && row < g_rows && col >= 0 && col < g_cols) {
                attron(COLOR_PAIR(cp) | A_BOLD);
                mvaddch(row, col, (chtype)'#');
                attroff(COLOR_PAIR(cp) | A_BOLD);
            }
        }
    }

    /* HUD */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " DragonCurve  q:quit  spc:pause  r:restart  +/-:speed  g/G:gen");
    mvprintw(1, 0,
        " gen:%d  segs:%d/%d  speed:%d  %s",
        g_n_gen, draw_to, g_n_segs, g_speed,
        g_paused ? "PAUSED" : (draw_to >= g_n_segs ? "complete" : "drawing"));
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,  sig_h);
    signal(SIGTERM, sig_h);
    signal(SIGWINCH,sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);

    curve_build(N_GEN_INIT);
    g_draw_seg = 0;

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case ' ':  g_paused = !g_paused; break;
        case 'r': case 'R': g_draw_seg = 0; break;
        case '+': case '=':
            g_speed *= 2; if (g_speed > 1024) g_speed = 1024; break;
        case '-':
            g_speed /= 2; if (g_speed < 1) g_speed = 1; break;
        case 'g':
            curve_build(g_n_gen - 1); g_draw_seg = 0; break;
        case 'G':
            curve_build(g_n_gen + 1); g_draw_seg = 0; break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused && g_draw_seg < g_n_segs)
            g_draw_seg += g_speed;

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
