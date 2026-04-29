/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_log_spiral.c — logarithmic (equiangular) spiral grid
 *
 * DEMO: Spiral arms wind outward with a gap that grows with radius — the
 *       defining property of logarithmic spirals found in nautilus shells,
 *       galaxies, and sunflower seed arrangements.  A 'g' key switches to the
 *       golden spiral preset (a ≈ 0.3065), which matches Fibonacci phyllotaxis.
 *       +/- adjusts the growth rate; [/] changes the number of arms.
 *
 * Study alongside: 03_archimedean_spiral.c (constant-pitch spiral),
 *                  05_sunflower.c (phyllotaxis dot pattern)
 *
 * Section map:
 *   §1 config   — growth rate, arm count, golden preset, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID
 *   §4 coords   — cell_to_polar, angle_char
 *   §5 draw     — log-spiral phase test, N-arm trick
 *   §6 scene    — scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme   g golden-spiral preset
 *        +/- growth rate   [/] arm count
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids/04_log_spiral.c \
 *       -o 04_log_spiral -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Logarithmic spiral arm detection via the N-arm phase test.
 *
 *                  A logarithmic spiral satisfies r = b × e^(a×θ).
 *                  Rearranging: θ = ln(r/b) / a.  For N arms equally spaced
 *                  by 2π/N, the N-arm phase test is:
 *
 *                    phase = fmod(N × (θ − ln(r/b)/a) + N×2π, 2π)
 *                    on_spiral : phase < W || phase > 2π − W
 *
 *                  Why it works: on arm k, θ = ln(r/b)/a + k×2π/N, so
 *                  N×(θ − ln(r/b)/a) = k×2π.  fmod(k×2π, 2π) = 0. ✓
 *
 * Math           : The growth parameter a determines how quickly the spiral
 *                  expands.  After one full turn (Δθ = 2π), the radius
 *                  multiplies by e^(a×2π).  For the golden spiral:
 *
 *                    a = 2 × ln(φ) / π  ≈  0.3065  where φ = (1+√5)/2
 *
 *                  This gives a radial ratio of exactly φ² per half-turn —
 *                  the same ratio found in sunflower and pinecone phyllotaxis.
 *
 *                  Key difference from Archimedean (03): Archimedean gaps are
 *                  constant in pixels; log-spiral gaps grow proportionally to
 *                  the current radius (equal in log space).
 *
 * Rendering      : angle_char(theta) gives the tangent direction at each
 *                  point, producing a smooth curved-line appearance.  The
 *                  inner region is excluded (r < R_MIN) to avoid a smear.
 *
 * Performance    : O(rows × cols) per frame.  One log() per cell — slightly
 *                  more expensive than 03 but still imperceptible at terminal
 *                  resolution.
 *
 * References     :
 *   Logarithmic spiral — en.wikipedia.org/wiki/Logarithmic_spiral
 *   Golden spiral — en.wikipedia.org/wiki/Golden_spiral
 *   Phyllotaxis and φ — Prusinkiewicz & Lindenmayer "The Algorithmic Beauty
 *     of Plants" (1990), Chapter 4
 *   Spirals in nature — Livio 2002, "The Golden Ratio", chapter 5
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   A log-spiral winds outward with a gap that grows proportionally with
 *   radius.  At any point it crosses any radial line at the same angle
 *   α = arctan(1/a) — this "equiangular" property is found in nautilus
 *   shells, galaxies, and Fibonacci phyllotaxis.  Same N-arm phase test
 *   as 03, but with θ_predicted = log(r/R_MIN)/growth instead of r/a.
 *
 * HOW TO THINK ABOUT IT
 *   Archimedean spiral (03): each coil adds PITCH pixels — constant additive gap.
 *   Log-spiral (04): each coil multiplies radius by e^(a×2π) — constant ratio gap.
 *   Zooming out reveals self-similarity: the spiral looks the same at every scale.
 *
 *   Detection: at radius r the spiral passes through angle
 *     θ_predicted = ln(r / R_MIN) / growth
 *   The actual angle's deviation from θ_predicted (×N for N arms) is the phase.
 *
 * DRAWING METHOD
 *   1. dx = (col−ox)×CELL_W,  dy = (row−oy)×CELL_H
 *   2. r = √(dx²+dy²),  θ = atan2(dy,dx)
 *   3. If r < R_MIN: skip
 *   4. θ_norm = fmod(θ + 2π, 2π)
 *   5. θ_predicted = log(r / R_MIN) / growth     ← spiral angle at this r
 *   6. raw = N × (θ_norm − θ_predicted)
 *   7. phase = fmod(raw + N×2π, 2π)
 *   8. on_spiral = phase < SPIRAL_W  ||  phase > 2π − SPIRAL_W
 *   9. Draw angle_char(θ) if on_spiral.
 *
 * KEY FORMULAS
 *   Log-spiral equation: r = b × e^(a × θ),  b = R_MIN
 *     Rearranged: θ = ln(r/b) / a = θ_predicted.
 *
 *   Equiangular property:
 *     dr/dθ = a × r  →  tan(crossing angle) = r / (dr/dθ) = 1/a
 *     α = arctan(1/a).  At a=0.18: α≈80°.  At a=0.3065 (golden): α≈73°.
 *
 *   Golden spiral preset:
 *     a = 2 × ln(φ) / π ≈ 0.3065,  φ = (1+√5)/2 ≈ 1.618
 *     Each half-turn scales radius by φ², matching Fibonacci phyllotaxis ratios.
 *
 *   Comparison with Archimedean (03):
 *     03: θ_predicted = r / a          (linear in r)
 *     04: θ_predicted = ln(r/b) / a    (logarithmic in r)
 *     Same N-arm test; only θ_predicted differs.
 *
 * EDGE CASES TO WATCH
 *   • r < R_MIN: log undefined or negative.  Hard guard (continue).
 *   • growth → 0: θ_predicted → ±∞; spiral wound infinitely tight.
 *     Constrained to [GROWTH_MIN, GROWTH_MAX].
 *   • Same N×2π normalisation needed as 03 to keep phase in [0, 2π).
 *
 * HOW TO VERIFY
 *   growth=0.18, N=1, R_MIN=4, ox=40, oy=12.
 *
 *   Point on arm 0 at θ=0 after k full turns:
 *     r = R_MIN × e^(0.18 × 2πk)
 *     k=1: r = 4 × e^1.131 ≈ 4 × 3.099 ≈ 12.4 px  →  col=40+round(12.4/2)=46
 *   Check cell (col=46, row=12):
 *     dx=(46−40)×2=12, dy=0  →  r=12, θ=0  →  θ_norm=0
 *     θ_predicted = log(12/4)/0.18 = ln(3)/0.18 = 1.099/0.18 = 6.105
 *     raw = 1×(0 − 6.105) = −6.105  ≈ −2π + 0.178
 *     phase = fmod(−6.105 + 2π, 2π) = fmod(0.178, 2π) = 0.178
 *     0.178 < SPIRAL_W(0.22)  →  on_spiral = true  ✓
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS      30
#define CELL_W          2
#define CELL_H          4

/* Growth rate a: radial multiplier per radian.  r doubles every 2π/a rad.
 * Range [0.05, 0.80].  Golden spiral: a = 2*ln(φ)/π ≈ 0.3065.          */
#define GROWTH_DEFAULT  0.18
#define GROWTH_MIN      0.05
#define GROWTH_MAX      0.80
#define GROWTH_STEP     0.02

/* Golden ratio constant and the golden-spiral growth rate */
#define PHI             1.61803398874989484820
#define GROWTH_GOLDEN   (2.0 * log(PHI) / M_PI)   /* ≈ 0.3065 */

/* Inner anchor radius b (pixels).  Arm is undefined for r < R_MIN. */
#define R_MIN           4.0

/* Angular half-width of the spiral line (radians in N×phase space) */
#define SPIRAL_W        0.22

/* Number of spiral arms */
#define N_ARMS_DEFAULT   2
#define N_ARMS_MIN       1
#define N_ARMS_MAX       8

#define PAIR_GRID   1
#define PAIR_HUD    2
#define PAIR_LABEL  3

static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
#define N_THEMES 5

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec=(time_t)(ns/1000000000LL),
                          .tv_nsec=(long)(ns%1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg = COLORS >= 256 ? THEME_FG[theme][0] : THEME_FG[theme][1];
    init_pair(PAIR_GRID,  fg,                              -1);
    init_pair(PAIR_HUD,   COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_LABEL, COLORS>=256 ? 252 : COLOR_WHITE, -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  coords                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_polar — convert screen cell (col, row) to polar (r_px, theta).
 *
 * THE FORMULA:
 *   dx_px = (col − ox) × CELL_W,  dy_px = (row − oy) × CELL_H
 *   r_px  = √(dx_px² + dy_px²),   theta = atan2(dy_px, dx_px)  ∈ (−π, π]
 */
static void cell_to_polar(int col, int row, int ox, int oy,
                           double *r_px, double *theta)
{
    double dx = (double)(col - ox) * CELL_W;
    double dy = (double)(row - oy) * CELL_H;
    *r_px  = sqrt(dx*dx + dy*dy);
    *theta = atan2(dy, dx);
}

/*
 * angle_char — pick the ASCII line character that best matches orientation theta.
 *
 * THE FORMULA:
 *   a = fmod(theta + 2π, π)  ← fold into [0, π) (orientation, not direction)
 *   a ∈ [0, π/8) or [7π/8, π) → '-';  a ∈ [π/8, 3π/8) → '\'
 *   a ∈ [3π/8, 5π/8) → '|';          a ∈ [5π/8, 7π/8) → '/'
 */
static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  draw                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * grid_draw — sweep every cell, apply N-arm log-spiral phase test, draw.
 *
 * THE PIPELINE:
 *   for each cell:
 *     (r, θ) ← cell_to_polar(); if r < R_MIN: skip
 *     θ_norm     = fmod(θ + 2π, 2π)
 *     θ_predicted = log(r / R_MIN) / growth    spiral angle at this r
 *     raw        = N × (θ_norm − θ_predicted)  N-arm phase (unbounded)
 *     phase      = fmod(raw + N×2π, 2π)        normalised to [0, 2π)
 *     on_spiral  = phase < SPIRAL_W  ||  phase > 2π − SPIRAL_W
 *
 * Difference from Archimedean (03): θ_predicted uses log(r) not r directly.
 */
static void grid_draw(int rows, int cols, int ox, int oy,
                      double growth, int n_arms)
{
    double two_pi = 2.0 * M_PI;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double r_px, theta;
            cell_to_polar(col, row, ox, oy, &r_px, &theta);
            if (r_px < R_MIN) continue;

            double theta_norm = fmod(theta + two_pi, two_pi);

            /*
             * Log-spiral N-arm phase test:
             *   θ_predicted = ln(r/b) / a   (b = R_MIN)
             *   phase = N × (θ − θ_predicted)  mod 2π
             * On arm k: phase = N×k×2π/N = k×2π ≡ 0.  ✓
             */
            double theta_pred = log(r_px / R_MIN) / growth;
            double raw = (double)n_arms * (theta_norm - theta_pred);
            double phase = fmod(raw + (double)n_arms * two_pi, two_pi);

            if (phase < SPIRAL_W || phase > two_pi - SPIRAL_W) {
                char c = angle_char(theta);
                mvaddch(row, col, (chtype)(unsigned char)c);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(int rows, int cols, double growth, int n_arms,
                       bool golden, int theme, double fps, bool paused)
{
    int ox = cols / 2, oy = rows / 2;
    erase();
    grid_draw(rows, cols, ox, oy, growth, n_arms);

    char buf[80];
    snprintf(buf, sizeof buf, " %.1f fps  a:%.4f%s  arms:%d  %s ",
             fps, growth, golden ? "(golden)" : "",
             n_arms, paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 0,
        " q:quit  p:pause  t:theme(%d)  g:golden  +/-:growth  [/]:arms ",
        theme+1);
    attroff(COLOR_PAIR(PAIR_LABEL));

    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    int theme  = 0;
    screen_init(theme);

    int    rows   = LINES, cols = COLS;
    double growth = GROWTH_DEFAULT;
    int    n_arms = N_ARMS_DEFAULT;
    bool   golden = false;
    bool   paused = false;
    double fps = TARGET_FPS;
    int64_t t0 = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'p': paused = !paused; break;
        case 't': theme = (theme + 1) % N_THEMES; color_init(theme); break;
        case 'g':
            golden = !golden;
            growth = golden ? GROWTH_GOLDEN : GROWTH_DEFAULT;
            break;
        case '+': case '=':
            if (growth < GROWTH_MAX) { growth += GROWTH_STEP; golden = false; }
            break;
        case '-':
            if (growth > GROWTH_MIN) { growth -= GROWTH_STEP; golden = false; }
            break;
        case '[':
            if (n_arms > N_ARMS_MIN) n_arms--;
            break;
        case ']':
            if (n_arms < N_ARMS_MAX) n_arms++;
            break;
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;
        if (!paused)
            scene_draw(rows, cols, growth, n_arms, golden, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
