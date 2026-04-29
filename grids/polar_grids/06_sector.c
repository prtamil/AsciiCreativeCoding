/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_sector.c — equal-area polar sector grid
 *
 * DEMO: Rings are placed at r_k = √k × R_UNIT so that each annular band
 *       (between consecutive rings) has the same area — like a bullseye
 *       target where every ring is equally "hard to hit".  Combined with
 *       uniform angular sectors the result is a grid where every cell covers
 *       the same area.  +/- adjusts the unit radius; [/] changes sector count.
 *
 * Study alongside: 01_rings_spokes.c (linear rings — unequal area),
 *                  02_log_polar.c (log rings — equal log-area)
 *
 * Section map:
 *   §1 config   — R_UNIT, sector count, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID
 *   §4 coords   — cell_to_polar, angle_char
 *   §5 draw     — equal-area ring test, sector grid
 *   §6 scene    — scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme
 *        +/- unit radius   [/] sector count
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids/06_sector.c \
 *       -o 06_sector -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Equal-area ring placement.  The k-th ring is at:
 *
 *                    r_k = √k × R_UNIT
 *
 *                  The annular area between ring k and ring k+1 is:
 *                    π × r_{k+1}² − π × r_k²
 *                    = π × R_UNIT² × ((k+1) − k) = π × R_UNIT²
 *
 *                  Constant!  Every annular band has area π×R_UNIT².
 *
 *                  Detection: for a cell with radius r_px, the continuous ring
 *                  index is k_float = (r_px / R_UNIT)².  We test whether k_float
 *                  is near an integer:
 *
 *                    frac = k_float − floor(k_float)
 *                    on_ring: frac < RING_W_F || frac > 1 − RING_W_F
 *
 *                  RING_W_F is a fractional threshold in "ring-index² space".
 *                  The visual ring width grows with radius (∝ √r) because
 *                  d(r²)/dr = 2r — a fixed Δ(k_float) maps to larger Δr at
 *                  larger r.  This keeps all rings visually present without
 *                  becoming invisible at large radii.
 *
 * Math           : Sector detection is identical to 01_rings_spokes: divide
 *                  [0, 2π) into N_SECTORS equal wedges of width 2π/N_SECTORS
 *                  and use the same fmod spoke test.
 *
 *                  Equal-area grids appear in:
 *                    • HEALPix (astronomy): equal-area pixels on the sphere
 *                    • Camera sensor binning: equal photon counts per cell
 *                    • Polling / bin statistics: each bin equally informative
 *
 * Rendering      : angle_char() gives the tangent direction for rings;
 *                  the same character for sector lines.  Ring/sector
 *                  intersections use '+'.
 *
 * Performance    : O(rows × cols) per frame with one sqrt per cell (for
 *                  r_px) plus a squaring r_px² / R_UNIT² — no extra log().
 *
 * References     :
 *   Equal-area projection — en.wikipedia.org/wiki/Equal-area_map
 *   HEALPix equal-area tessellation — Górski et al. 2005, ApJ 622:759
 *   Area of annulus — en.wikipedia.org/wiki/Annulus_(mathematics)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   Rings are placed at r_k = √k × R_UNIT so that each annular band has the
 *   same area: π × R_UNIT².  Combined with uniform angular sectors, every grid
 *   cell covers the same area — the standard equal-area polar grid.
 *
 * HOW TO THINK ABOUT IT
 *   A dartboard where each ring should be equally likely to be hit needs equal-
 *   area rings.  Linear spacing (01) makes outer rings larger → hit more often.
 *   The √k spacing equalises the probability.  Visually: inner rings are closer
 *   together (small Δr), outer rings farther apart — softer growth than log (02).
 *
 *   Detection: map r to "ring index squared" k_float = (r/R_UNIT)².  If k_float
 *   is near an integer, the cell is near ring k = floor(k_float).
 *
 * DRAWING METHOD
 *   1. dx = (col−ox)×CELL_W,  dy = (row−oy)×CELL_H
 *   2. r = √(dx²+dy²),  θ = atan2(dy,dx)
 *   3. If r < R_MIN: skip
 *   4. k_float = (r / R_UNIT)²             ← continuous ring index in k² space
 *   5. frac = k_float − floor(k_float)     ← position within current ring interval
 *   6. on_ring = (frac < RING_W_F || frac > 1 − RING_W_F)
 *   7. Sector test same as 01 spoke test.
 *   8. Draw intersection/ring/sector/skip.
 *
 * KEY FORMULAS
 *   Ring placement: r_k = √k × R_UNIT
 *     Equal-area proof: annular area from r_{k−1} to r_k
 *       = π r_k² − π r_{k−1}² = π R_UNIT²(k − (k−1)) = π R_UNIT²  (constant) ✓
 *
 *   Ring detection: k_float = (r / R_UNIT)²
 *     If r = r_k then k_float = k exactly (integer → frac=0 → on_ring).
 *
 *   Adaptive pixel width:
 *     dk = RING_W_F.  dr = r × dk (from d(r²/R_UNIT²)/dr = 2r/R_UNIT²).
 *     Outer rings are wider in pixels — they remain visible at large radii.
 *
 * EDGE CASES TO WATCH
 *   • r=0: k_float=0 → frac=0 → always on_ring.  Guard with R_MIN.
 *   • SECTOR_MIN_R: prevents smeared disc at origin for sector lines.
 *   • RING_W_F ≥ 0.5: every cell becomes "on_ring".  Keep < 0.3.
 *
 * HOW TO VERIFY
 *   R_UNIT=18px, ox=40, oy=12.  Rings at r_k = 18√k px:
 *     k=1: 18px,  k=2: 25.5px,  k=4: 36px,  k=9: 54px.
 *
 *   Cell (col=49, row=12): dx=(49−40)×2=18, dy=0  →  r=18px
 *     k_float = (18/18)² = 1.000  →  frac=0 < RING_W_F(0.06)  →  on_ring ✓
 *     θ=0  →  angle_char(0) = '-'  ✓
 *
 *   Cell (col=53, row=12): dx=(53−40)×2=26, dy=0  →  r=26px
 *     k_float = (26/18)² = 2.087  →  frac=0.087 > RING_W_F(0.06)
 *     NOT on_ring  (between ring 2 at 25.5px and ring 3 at 31.2px)  ✓
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

/* Unit radius: r_1 = R_UNIT (innermost ring).  Each ring k is at √k × R_UNIT. */
#define R_UNIT_DEFAULT  18.0
#define R_UNIT_MIN       6.0
#define R_UNIT_MAX      40.0
#define R_UNIT_STEP      2.0

/* Fractional width of a ring in k² space (k_float = (r/R_UNIT)²) */
#define RING_W_F        0.06

/* Sectors */
#define N_SECTORS_DEFAULT  12
#define N_SECTORS_MIN       4
#define N_SECTORS_MAX      36
#define SECTOR_W           0.10   /* radian half-width of a sector boundary */
#define SECTOR_MIN_R       3.0

/* Minimum radius — avoid centre smear */
#define R_MIN            3.0

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
 * grid_draw — sweep every cell, apply equal-area ring and sector tests, draw.
 *
 * THE PIPELINE:
 *   for each cell:
 *     (r, θ) ← cell_to_polar(); if r < R_MIN: skip
 *     k_float = (r / r_unit)²           continuous ring index in k² space
 *     frac    = k_float − floor(k_float)
 *     on_ring = frac < RING_W_F  ||  frac > 1 − RING_W_F
 *     on_sector = same fmod test as 01 spoke test on θ_norm
 *     draw '+'/angle_char/skip
 */
static void grid_draw(int rows, int cols, int ox, int oy,
                      double r_unit, int n_sectors)
{
    double sector_angle = 2.0 * M_PI / (double)n_sectors;
    double r_unit_sq    = r_unit * r_unit;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double r_px, theta;
            cell_to_polar(col, row, ox, oy, &r_px, &theta);
            if (r_px < R_MIN) continue;

            /*
             * Equal-area ring test:
             *   k_float = (r_px / R_UNIT)²  — continuous ring index
             *   on_ring: fractional part near 0 or 1
             */
            bool on_ring = false;
            {
                double k_float = (r_px * r_px) / r_unit_sq;
                double frac    = k_float - floor(k_float);
                on_ring = (frac < RING_W_F || frac > 1.0 - RING_W_F);
            }

            double theta_norm  = fmod(theta + 2.0*M_PI, 2.0*M_PI);
            double sector_phase = fmod(theta_norm, sector_angle);
            bool on_sector = (r_px > SECTOR_MIN_R) &&
                             (sector_phase < SECTOR_W ||
                              sector_phase > sector_angle - SECTOR_W);

            if (!on_ring && !on_sector) continue;

            char c = (on_ring && on_sector) ? '+' : angle_char(theta);
            mvaddch(row, col, (chtype)(unsigned char)c);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(int rows, int cols, double r_unit, int n_sectors,
                       int theme, double fps, bool paused)
{
    int ox = cols / 2, oy = rows / 2;
    erase();
    grid_draw(rows, cols, ox, oy, r_unit, n_sectors);

    char buf[80];
    snprintf(buf, sizeof buf, " %.1f fps  R_unit:%.0fpx  sectors:%d  %s ",
             fps, r_unit, n_sectors, paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 0,
        " q:quit  p:pause  t:theme(%d)  +/-:R-unit  [/]:sectors ", theme+1);
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

    int theme     = 0;
    screen_init(theme);

    int    rows      = LINES, cols = COLS;
    double r_unit    = R_UNIT_DEFAULT;
    int    n_sectors = N_SECTORS_DEFAULT;
    bool   paused    = false;
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
            if (r_unit < R_UNIT_MAX) r_unit += R_UNIT_STEP;
            break;
        case '-':
            if (r_unit > R_UNIT_MIN) r_unit -= R_UNIT_STEP;
            break;
        case '[':
            if (n_sectors > N_SECTORS_MIN)
                n_sectors -= (n_sectors > 8 ? 4 : 2);
            break;
        case ']':
            if (n_sectors < N_SECTORS_MAX)
                n_sectors += (n_sectors >= 8 ? 4 : 2);
            break;
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;
        if (!paused)
            scene_draw(rows, cols, r_unit, n_sectors, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
