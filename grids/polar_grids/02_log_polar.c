/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_log_polar.c — logarithmic polar grid: exponentially-spaced rings
 *
 * DEMO: Rings are spaced so that each successive ring is a fixed RATIO
 *       further from the centre than the previous one (geometric progression).
 *       Inner rings are dense; outer rings grow increasingly wide.  This is
 *       the coordinate system of conformal optics, SIFT descriptors, and
 *       human retinal sampling.  +/- adjusts the growth ratio.
 *
 * Study alongside: 01_rings_spokes.c (linear ring spacing)
 *
 * Section map:
 *   §1 config   — R_MIN, log step (ratio), spoke count, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID
 *   §4 coords   — cell_to_polar, log_ring_phase
 *   §5 draw     — grid sweep, log-ring and spoke tests
 *   §6 scene    — scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme
 *        +/- growth ratio   [/] spoke count
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids/02_log_polar.c \
 *       -o 02_log_polar -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Logarithmic polar grid.  Instead of rings at equal pixel
 *                  intervals (r = k × SPACING), rings sit at exponentially
 *                  growing radii:
 *
 *                    r_k = R_MIN × RATIO^k
 *
 *                  Equivalently: ln(r / R_MIN) increases by ln(RATIO) per
 *                  ring.  To test membership, compute the continuous ring
 *                  index u = ln(r / R_MIN) / ln(RATIO) and check its
 *                  fractional part:
 *
 *                    u = ln(r / R_MIN) / LOG_STEP      (LOG_STEP = ln(RATIO))
 *                    on_ring : fmod(u, 1.0) < RING_W_U || > 1 − RING_W_U
 *
 *                  RING_W_U is a fractional width in "ring-index space", not
 *                  pixels.  This keeps all rings the same visual thickness in
 *                  log-space (thin inner rings would be imperceptible with a
 *                  fixed pixel width, so we use adaptive width here instead).
 *
 * Math           : The log-polar transform maps (r, θ) → (ln r, θ).  In this
 *                  space, concentric circles become horizontal lines — it is
 *                  a conformal (angle-preserving) map.  Biologically, the
 *                  human fovea has approximately log-polar sampling density.
 *                  Scale changes in Cartesian space become translations in
 *                  log-polar space — the basis of scale-invariant feature
 *                  descriptors (SIFT, SURF).
 *
 * Rendering      : Same angle_char() as 01_rings_spokes.  Rings near the
 *                  centre are tightly packed; outer rings are wide apart.
 *                  The visual is like a radar screen with fine inner detail
 *                  and coarse outer bins.
 *
 * Performance    : Same O(rows × cols) sweep as 01; adds one log() call
 *                  per cell.  Still imperceptibly fast at terminal resolution.
 *
 * References     :
 *   Log-polar transform — en.wikipedia.org/wiki/Log-polar_coordinates
 *   SIFT scale-invariant features — Lowe 2004, IJCV 60(2):91–110
 *   Human retinal sampling — Schwartz 1980, Biol. Cybernetics 37(4):199–208
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────���──────────── *
 *
 * CORE IDEA
 *   Replace the evenly-spaced rings of 01 with exponentially-spaced rings.
 *   Each ring is RATIO times further from the centre than the previous one.
 *   Detection: compute the continuous "ring index" u = log(r/R_MIN)/LOG_STEP;
 *   a cell is on a ring when u's fractional part is near 0 or 1.
 *
 * HOW TO THINK ABOUT IT
 *   Imagine zooming into the centre of a photo: each zoom level reveals the
 *   same structural detail.  Ring k is at r_k = R_MIN × RATIO^k, so each ring
 *   represents an equal multiplicative step in distance, not an additive one.
 *   The log-polar transform maps (r, θ) → (ln r, θ): circles become horizontal
 *   lines in log-space, and scale changes become translations.
 *
 *   Comparison with 01_rings_spokes:
 *     01: r_k = k × RING_SPACING   (CONSTANT pixel gap between rings)
 *     02: r_k = R_MIN × RATIO^k    (CONSTANT log gap; pixel gap grows with r)
 *
 * DRAWING METHOD
 *   1. dx = (col−ox)×CELL_W,  dy = (row−oy)×CELL_H
 *   2. r = √(dx²+dy²),  θ = atan2(dy,dx)
 *   3. If r ≤ R_MIN: skip (log undefined at or below inner anchor)
 *   4. u = log(r / R_MIN) / LOG_STEP     ← continuous ring index: u=0 at r=R_MIN
 *   5. frac = u − floor(u)               ← position within current ring interval
 *   6. on_ring = (frac < RING_W_U || frac > 1 − RING_W_U)
 *   7. Spoke test same as 01_rings_spokes.
 *   8. Draw intersection/ring/spoke/skip.
 *
 * KEY FORMULAS
 *   Ring placement: r_k = R_MIN × e^(k × LOG_STEP)
 *     LOG_STEP = ln(RATIO).  Each ring is e^LOG_STEP ≈ RATIO times the previous.
 *
 *   Ring detection: u = ln(r/R_MIN) / LOG_STEP  (inverse of r_k formula)
 *     If r = r_k exactly, then u = k (integer → frac=0 → on_ring).
 *
 *   Adaptive ring width: RING_W_U is a fraction of the log-space interval,
 *     NOT a pixel width.  Physical width at ring k:
 *       Δr = r_k × LOG_STEP × RING_W_U  (from d(log r)/dr = 1/r)
 *     So outer rings are wider in pixels — they remain visible at large radii.
 *
 * EDGE CASES TO WATCH
 *   • r ≤ R_MIN: log(r/R_MIN) ≤ 0 → u ≤ 0 → rings cluster at origin.
 *     Hard guard: skip cells with r ≤ R_MIN.
 *   • LOG_STEP = 0: division by zero.  Constrained to [LOG_STEP_MIN, LOG_STEP_MAX].
 *   • θ normalisation: same as 01 — add 2π before fmod to avoid negative phases.
 *
 * HOW TO VERIFY
 *   Terminal 80×24, R_MIN=4, LOG_STEP=0.25 (RATIO=e^0.25≈1.284), ox=40, oy=12.
 *
 *   Ring radii: r_0=4px, r_1≈5.1px, r_2≈6.6px, r_4≈10.9px, r_6≈17.9px.
 *
 *   Cell (col=49, row=12): dx=(49−40)×2=18, dy=0  →  r=18.0
 *     u = log(18/4) / 0.25 = ln(4.5) / 0.25 = 1.504 / 0.25 = 6.017
 *     frac = 0.017 < RING_W_U(0.08)  →  on_ring = true
 *     r_6 = 4 × e^(6×0.25) = 4 × e^1.5 = 4 × 4.482 ≈ 17.9 px  ← matches ✓
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

#define TARGET_FPS   30
#define CELL_W       2
#define CELL_H       4

/* Smallest ring radius in pixels.  Rings below R_MIN are suppressed. */
#define R_MIN               4.0

/* ln(RATIO): the log-step between consecutive rings.
 * RATIO = e^LOG_STEP.  LOG_STEP=0.25 → each ring is e^0.25 ≈ 1.28× further.
 * Range [0.10, 0.60]: small = dense inner rings; large = widely spaced.   */
#define LOG_STEP_DEFAULT    0.25
#define LOG_STEP_MIN        0.10
#define LOG_STEP_MAX        0.60
#define LOG_STEP_DELTA      0.05

/* Fractional thickness of a ring in log-ring-index space (0..1 per ring) */
#define RING_W_U            0.08

#define N_SPOKES_DEFAULT    12
#define N_SPOKES_MIN         4
#define N_SPOKES_MAX        36
#define SPOKE_W             0.10
#define SPOKE_MIN_R         3.0

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
 *   r_px  = √(dx_px² + dy_px²)    [Euclidean distance in pixel space]
 *   theta = atan2(dy_px, dx_px)    [angle ∈ (−π, π]]
 */
static void cell_to_polar(int col, int row, int ox, int oy,
                           double *r_px, double *theta)
{
    double dx = (double)(col - ox) * CELL_W;
    double dy = (double)(row - oy) * CELL_H;
    *r_px  = sqrt(dx*dx + dy*dy);
    *theta = atan2(dy, dx);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  draw                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * angle_char — pick the ASCII line character that best matches orientation theta.
 *
 * THE FORMULA:
 *   a = fmod(theta + 2π, π)  ← fold into [0, π) (lines have no direction)
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

/*
 * grid_draw — sweep every cell, apply log-ring and spoke tests, draw.
 *
 * THE PIPELINE:
 *   for each cell (col, row):
 *     (r, θ) ← cell_to_polar()
 *     if r ≤ R_MIN: skip
 *     u = log(r / R_MIN) / log_step     continuous log ring index
 *     frac = u − floor(u)               ∈ [0, 1)
 *     on_ring = frac < RING_W_U  ||  frac > 1 − RING_W_U
 *     on_spoke = same fmod test as 01 on θ_norm
 *     draw '+'/angle_char/skip
 */
static void grid_draw(int rows, int cols, int ox, int oy,
                      double log_step, int n_spokes)
{
    double spoke_angle = 2.0 * M_PI / (double)n_spokes;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double r_px, theta;
            cell_to_polar(col, row, ox, oy, &r_px, &theta);

            bool on_ring = false;
            if (r_px > R_MIN) {
                /* u = continuous log-ring index: u=0 at r=R_MIN, +1 per ring */
                double u = log(r_px / R_MIN) / log_step;
                double frac = u - floor(u);
                on_ring = (frac < RING_W_U || frac > 1.0 - RING_W_U);
            }

            double theta_norm  = fmod(theta + 2.0*M_PI, 2.0*M_PI);
            double spoke_phase = fmod(theta_norm, spoke_angle);
            bool on_spoke = (r_px > SPOKE_MIN_R) &&
                            (spoke_phase < SPOKE_W ||
                             spoke_phase > spoke_angle - SPOKE_W);

            if (!on_ring && !on_spoke) continue;

            char c = (on_ring && on_spoke) ? '+' : angle_char(theta);
            mvaddch(row, col, (chtype)(unsigned char)c);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(int rows, int cols, double log_step, int n_spokes,
                       int theme, double fps, bool paused)
{
    int ox = cols / 2, oy = rows / 2;
    erase();
    grid_draw(rows, cols, ox, oy, log_step, n_spokes);

    char buf[80];
    double ratio = exp(log_step);
    snprintf(buf, sizeof buf, " %.1f fps  ratio:%.2f  spokes:%d  %s ",
             fps, ratio, n_spokes, paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 0,
        " q:quit  p:pause  t:theme(%d)  +/-:ring-ratio  [/]:spokes ", theme+1);
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

    int theme = 0;
    screen_init(theme);

    int    rows = LINES, cols = COLS;
    double log_step = LOG_STEP_DEFAULT;
    int    n_spokes = N_SPOKES_DEFAULT;
    bool   paused   = false;
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
        case '+': case '=':
            if (log_step < LOG_STEP_MAX) log_step += LOG_STEP_DELTA;
            break;
        case '-':
            if (log_step > LOG_STEP_MIN) log_step -= LOG_STEP_DELTA;
            break;
        case '[':
            if (n_spokes > N_SPOKES_MIN) n_spokes -= (n_spokes > 8 ? 4 : 2);
            break;
        case ']':
            if (n_spokes < N_SPOKES_MAX) n_spokes += (n_spokes >= 8 ? 4 : 2);
            break;
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;
        if (!paused)
            scene_draw(rows, cols, log_step, n_spokes, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
