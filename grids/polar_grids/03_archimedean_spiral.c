/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_archimedean_spiral.c — Archimedean spiral grid (constant pitch)
 *
 * DEMO: One or more spiral arms wind outward at constant pitch — the gap
 *       between successive passes at any radius is always the same.  The
 *       N_ARMS trick renders all arms simultaneously with a single modular
 *       test per cell.  +/- adjusts the pitch (tighter vs looser coil);
 *       [/] changes the number of arms (1 arm = classic spiral, 2 = yin-yang
 *       style, 6 = pinwheel).
 *
 * Study alongside: 04_log_spiral.c (variable pitch), 01_rings_spokes.c
 *
 * Section map:
 *   §1 config   — pitch, arm count, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID
 *   §4 coords   — cell_to_polar, angle_char
 *   §5 draw     — Archimedean phase test, N-arm trick
 *   §6 scene    — scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme
 *        +/- pitch (pixels/turn)   [/] arm count
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids/03_archimedean_spiral.c \
 *       -o 03_archimedean_spiral -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Archimedean spiral arm detection via the N-arm phase test.
 *
 *                  An Archimedean spiral satisfies r = a × θ (one arm).
 *                  Rearranging: θ = r / a.  For N arms equally spaced by
 *                  2π/N, arm k satisfies θ = r/a + k × 2π/N.
 *
 *                  The N-arm phase trick: multiply the "offset" (θ − r/a) by
 *                  N and test whether the result is near a multiple of 2π:
 *
 *                    phase = fmod(N × (θ − r/a) + N×2π, 2π)
 *                    on_spiral : phase < W || phase > 2π − W
 *
 *                  Why it works: for point on arm k, θ = r/a + k×2π/N, so
 *                  N×(θ−r/a) = N×k×2π/N = k×2π.  fmod(k×2π, 2π) = 0. ✓
 *                  Any k tests as on-spiral, so ALL arms are detected at once.
 *
 * Math           : The pitch (gap between successive turns at fixed θ) is
 *                  PITCH = a × 2π for a single arm, or PITCH/N for N arms.
 *                  The parameter a = PITCH / (2π).  Pitch is set in pixels
 *                  and relates to RING_SPACING in 01_rings_spokes: equal
 *                  pitch gives equal turn-to-turn gap for any radius.
 *
 * Rendering      : Because the spiral is neither purely horizontal nor
 *                  vertical, angle_char(theta) naturally produces the
 *                  correct slanted line character at each point.
 *
 * Performance    : Same O(rows × cols) sweep as 01.
 *
 * References     :
 *   Archimedean spiral — en.wikipedia.org/wiki/Archimedean_spiral
 *   Spirals in nature — Livio 2002, "The Golden Ratio", chapter 5
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   An Archimedean spiral winds outward at constant pitch: at any radius, the
 *   gap between successive turns is always PITCH pixels.  The N-arm phase test
 *   detects ALL arms simultaneously with one modular expression — no loop.
 *
 * HOW TO THINK ABOUT IT
 *   For arm 0: r = a × θ.  Every point on the spiral satisfies θ = r/a.
 *   The "phase" of a cell (r, θ) is how far θ deviates from the spiral's
 *   prediction r/a.  Phase ≈ 0 → on the spiral.  Phase ≈ π → opposite side.
 *
 *   For N arms equally spaced by 2π/N, arm k satisfies θ = r/a + k×2π/N.
 *   Multiplying the phase by N maps all N arms to the SAME modular value 0.
 *   One test catches every arm.
 *
 * DRAWING METHOD
 *   1. dx = (col−ox)×CELL_W,  dy = (row−oy)×CELL_H
 *   2. r = √(dx²+dy²),  θ = atan2(dy,dx)
 *   3. If r < MIN_R: skip
 *   4. θ_norm = fmod(θ + 2π, 2π)            ← [0, 2π)
 *   5. a = PITCH / (2π)                      ← radial advance per radian
 *   6. raw = N × (θ_norm − r/a)             ← N-arm phase, unbounded
 *   7. phase = fmod(raw + N×2π, 2π)          ← normalise to [0, 2π)
 *   8. on_spiral = phase < SPIRAL_W  ||  phase > 2π − SPIRAL_W
 *   9. Draw angle_char(θ) if on_spiral.
 *
 * KEY FORMULAS
 *   Single arm: r = a × θ,  so  a = PITCH / (2π)
 *     After one turn Δθ = 2π the radius increases by a×2π = PITCH.  ✓
 *
 *   N-arm phase derivation:
 *     On arm k: θ = r/a + k×2π/N
 *     phase = N × (θ_norm − r/a)
 *           = N × (r/a + k×2π/N − r/a)     [substitute arm k equation]
 *           = k × 2π
 *     fmod(k×2π, 2π) = 0  →  every arm k tests as "on_spiral".  ✓
 *
 *   Why add N×2π before the final fmod?
 *     raw = N×(θ_norm − r/a).  r/a can be large → raw can be large negative.
 *     Adding N×2π shifts by N whole periods: the fractional part is unchanged,
 *     but the result is non-negative for typical values, keeping fmod well-behaved.
 *
 * EDGE CASES TO WATCH
 *   • PITCH=0: a=0 → r/a = ∞.  Constrained to [PITCH_MIN, PITCH_MAX].
 *   • Centre smear: near origin θ_norm − r/a changes rapidly.
 *     Hard guard: skip cells with r < MIN_R.
 *   • Large N: each arm's angular width is SPIRAL_W/N in θ space.  At N=8
 *     arms may look very thin.  Increase SPIRAL_W if arms disappear.
 *
 * HOW TO VERIFY
 *   PITCH=32, N=1, ox=40, oy=12.  a = 32/(2π) ≈ 5.093.
 *
 *   Point on arm 0 at θ=0 (rightward), k-th pass (k≥1):
 *     r = a × 2πk = PITCH × k  →  first pass at r=32px
 *     col = 40 + round(32/2) = 56.
 *   Check cell (col=56, row=12):
 *     dx=(56−40)×2=32, dy=0  →  r=32, θ=0  →  θ_norm=0
 *     raw = 1×(0 − 32/5.093) = −6.284 ≈ −2π
 *     phase = fmod(−6.284 + 2π, 2π) = fmod(0, 2π) = 0
 *     0 < SPIRAL_W(0.20)  →  on_spiral = true  ✓
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

/* Spiral pitch: pixel distance between successive turns (= a × 2π for 1 arm) */
#define PITCH_DEFAULT   32.0
#define PITCH_MIN        8.0
#define PITCH_MAX       80.0
#define PITCH_STEP       4.0

/* Angular half-width of the spiral line (radians in N×phase space) */
#define SPIRAL_W        0.20

/* Number of spiral arms */
#define N_ARMS_DEFAULT   1
#define N_ARMS_MIN       1
#define N_ARMS_MAX       8

/* Minimum pixel radius — avoids a dense smear at the very centre */
#define MIN_R            3.0

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
 * grid_draw — sweep every cell, apply N-arm Archimedean phase test, draw.
 *
 * THE PIPELINE:
 *   a = PITCH / (2π)                      per-radian radial advance
 *   for each cell:
 *     (r, θ) ← cell_to_polar(); if r < MIN_R: skip
 *     θ_norm = fmod(θ + 2π, 2π)
 *     raw    = N × (θ_norm − r/a)         N-arm phase (unbounded)
 *     phase  = fmod(raw + N×2π, 2π)       normalised to [0, 2π)
 *     on_spiral = phase < SPIRAL_W  ||  phase > 2π − SPIRAL_W
 */
static void grid_draw(int rows, int cols, int ox, int oy,
                      double pitch, int n_arms)
{
    /* a = pitch / (2π): the per-radian radial advance for a single arm */
    double a = pitch / (2.0 * M_PI);
    double two_pi = 2.0 * M_PI;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double r_px, theta;
            cell_to_polar(col, row, ox, oy, &r_px, &theta);
            if (r_px < MIN_R) continue;

            double theta_norm = fmod(theta + two_pi, two_pi);

            /*
             * N-arm phase test:
             *   phase = N × (θ − r/a)  mod 2π
             * On arm k: phase = N×k×2π/N = k×2π ≡ 0.  ✓
             */
            double raw = (double)n_arms * (theta_norm - r_px / a);
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

static void scene_draw(int rows, int cols, double pitch, int n_arms,
                       int theme, double fps, bool paused)
{
    int ox = cols / 2, oy = rows / 2;
    erase();
    grid_draw(rows, cols, ox, oy, pitch, n_arms);

    char buf[80];
    snprintf(buf, sizeof buf, " %.1f fps  pitch:%.0fpx  arms:%d  %s ",
             fps, pitch, n_arms, paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 0,
        " q:quit  p:pause  t:theme(%d)  +/-:pitch  [/]:arms ", theme+1);
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

    int theme   = 0;
    screen_init(theme);

    int    rows   = LINES, cols = COLS;
    double pitch  = PITCH_DEFAULT;
    int    n_arms = N_ARMS_DEFAULT;
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
        case '+': case '=':
            if (pitch < PITCH_MAX) pitch += PITCH_STEP;
            break;
        case '-':
            if (pitch > PITCH_MIN) pitch -= PITCH_STEP;
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
            scene_draw(rows, cols, pitch, n_arms, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
