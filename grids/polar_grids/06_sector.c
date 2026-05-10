/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_sector.c — equal-area polar sector grid
 *
 * DEMO: Rings are placed at r_k = √k × R_UNIT so that each annular band
 *       (between consecutive rings) has the same area — like a bullseye
 *       target where every ring is equally "hard to hit".  Combined with
 *       uniform angular sectors the result is a grid where every cell covers
 *       the same area.  An '@' cursor sits at one (ring, spoke) cell — arrows
 *       step it across the grid.  +/- adjusts unit radius; [/] sector count.
 *
 * Study alongside: 01_rings_spokes.c (linear rings — unequal area),
 *                  02_log_polar.c (log rings — equal log-area),
 *                  ../rect_grids/01_uniform_rect.c (the GridCtx template)
 *
 * Section map:
 *   §1 config   — R_UNIT, sector count, themes, EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID + HUD/HINT/CURSOR
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg + angle_char
 *   §5 cursor   — Cursor (ring, spoke) + cursor_reset / cursor_move / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme   r reset
 *        arrows move @   +/- unit radius   [/] sector count
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
 * Data-structure : Two structs — GridCtx (terminal extent, R_UNIT, n_sectors,
 *                  ox/oy) and Cursor (linear ring index, sector index).
 *                  ctx_to_screen places (ring, spoke) at the equal-area
 *                  midpoint √(ring + 0.5) × R_UNIT and the angular midpoint
 *                  (spoke + 0.5) × 2π/N_SECTORS.
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
 *   cell covers the same area — the standard equal-area polar grid.  Cursor
 *   address (ring k, spoke s) maps to the equal-area midpoint of cell (k, s).
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
 *   Cursor → screen (ring k, sector s):
 *     mid_radius = √(k + 0.5) × R_UNIT     (equal-area midpoint)
 *     theta_mid  = (s + 0.5) × (2π / N_SECTORS)
 *     cx = mid_radius × cos theta_mid;  cy = mid_radius × sin theta_mid
 *
 *   Adaptive pixel width:
 *     dk = RING_W_F.  dr = r × dk (from d(r²/R_UNIT²)/dr = 2r/R_UNIT²).
 *     Outer rings are wider in pixels — they remain visible at large radii.
 *
 * EDGE CASES TO WATCH
 *   • r=0: k_float=0 → frac=0 → always on_ring.  Guard with R_MIN.
 *   • SECTOR_MIN_R: prevents smeared disc at origin for sector lines.
 *   • RING_W_F ≥ 0.5: every cell becomes "on_ring".  Keep < 0.3.
 *   • Cursor max_ring re-derived in ctx_init from current R_UNIT.
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

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA   0.05

#define PAIR_GRID    1
#define PAIR_CURSOR  2
#define PAIR_HUD     3
#define PAIR_HINT    4

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
    init_pair(PAIR_GRID,   fg,                              -1);
    init_pair(PAIR_CURSOR, COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the equal-area ring/sector ↔ screen mapping  */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — equal-area sector geometry plus cursor bounds.
 *
 * r_unit = innermost ring radius; rings at r_k = √k × r_unit.
 * n_spokes (called n_sectors in the file's prose) = angular wedges.
 * max_ring = floor((r_visible / r_unit)² − 0.5).
 */
typedef struct {
    int rows, cols;

    double r_unit;         /* unit radius (pixels)                          */
    int    n_spokes;       /* number of equal-area angular sectors          */
    int    cell_w, cell_h;

    int    ox, oy;

    int    max_ring, max_spoke;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size.
 *
 * For a sample at the equal-area centre of (ring k, spoke s):
 *   mid_radius = √(k + 0.5) × r_unit ≤ r_visible
 *   k_max      = floor((r_visible / r_unit)² − 0.5)
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->r_unit   <= 0.0) g->r_unit   = R_UNIT_DEFAULT;
    if (g->n_spokes <= 0)   g->n_spokes = N_SECTORS_DEFAULT;

    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mr = 0;
    if (r_visible > g->r_unit) {
        double ratio = r_visible / g->r_unit;
        mr = (int)floor(ratio * ratio - 0.5);
        if (mr < 0) mr = 0;
    }
    g->max_ring  = mr;
    g->max_spoke = g->n_spokes - 1;
}

/*
 * ctx_to_screen — equal-area centre of (ring k, sector s).
 *
 * THE FORMULA:
 *   mid_radius = √(k + 0.5) × r_unit       (equal-area midpoint of annulus k)
 *   theta_mid  = (s + 0.5) × (2π / n_spokes)
 *   cx = mid_radius × cos theta_mid;  cy = mid_radius × sin theta_mid
 *   sc = ox + (int)round(cx / CELL_W);  sr = oy + (int)round(cy / CELL_H)
 *
 * The √(k+0.5) midpoint splits the annular area in two equal halves —
 * the natural centre for an equal-area cell, just as the arithmetic
 * midpoint splits a uniform-spaced annulus in 01_rings_spokes.
 */
static void ctx_to_screen(const GridCtx *g, int ring, int spoke,
                          int *sr, int *sc)
{
    double mid_radius = sqrt((double)ring + 0.5) * g->r_unit;
    double theta_mid  = ((double)spoke + 0.5) * (2.0 * M_PI / (double)g->n_spokes);
    double cx = mid_radius * cos(theta_mid);
    double cy = mid_radius * sin(theta_mid);
    *sc = g->ox + (int)round(cx / (double)g->cell_w);
    *sr = g->oy + (int)round(cy / (double)g->cell_h);
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

/*
 * ctx_draw_bg — sweep every cell, apply equal-area ring and sector tests, draw.
 *
 * THE PIPELINE:
 *   for each cell:
 *     dx = (col−ox)×CELL_W,  dy = (row−oy)×CELL_H
 *     r  = √(dx²+dy²),  θ = atan2(dy,dx);  if r < R_MIN: skip
 *     k_float = (r / r_unit)²           continuous ring index in k² space
 *     frac    = k_float − floor(k_float)
 *     on_ring = frac < RING_W_F  ||  frac > 1 − RING_W_F
 *     on_sector = same fmod test as 01 spoke test on θ_norm
 *     draw '+'/angle_char/skip
 */
static void ctx_draw_bg(const GridCtx *g)
{
    double sector_angle = 2.0 * M_PI / (double)g->n_spokes;
    double r_unit_sq    = g->r_unit * g->r_unit;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * g->cell_w;
            double dy = (double)(row - g->oy) * g->cell_h;
            double r_px = sqrt(dx*dx + dy*dy);
            if (r_px < R_MIN) continue;

            double theta = atan2(dy, dx);

            /*
             * Equal-area ring test:
             *   k_float = (r_px / R_UNIT)²  — continuous ring index
             *   on_ring: fractional part near 0 or 1
             */
            double k_float = (r_px * r_px) / r_unit_sq;
            double frac    = k_float - floor(k_float);
            bool on_ring = (frac < RING_W_F || frac > 1.0 - RING_W_F);

            double theta_norm   = fmod(theta + 2.0*M_PI, 2.0*M_PI);
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
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — (ring index, sector index).  Same shape as 01_rings_spokes;
 * only ctx_to_screen's radial law differs (equal-area, not uniform).
 */
typedef struct { int ring, spoke; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->ring  = g->max_ring / 2;
    cur->spoke = 0;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int d_ring, int d_spoke)
{
    int nr = cur->ring + d_ring;
    if (nr < 0)            nr = 0;
    if (nr > g->max_ring)  nr = g->max_ring;
    cur->ring = nr;

    int n = g->n_spokes > 0 ? g->n_spokes : 1;
    int ns = (cur->spoke + d_spoke) % n;
    if (ns < 0) ns += n;
    cur->spoke = ns;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->ring, cur->spoke, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     bool paused, double fps)
{
    char buf[112];
    snprintf(buf, sizeof buf,
             " ring:%d sector:%d  R_unit:%.0fpx  sectors:%d  th:%d  %5.1f fps  %s ",
             cur->ring, cur->spoke, g->r_unit, g->n_spokes,
             theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:R-unit  [/]:sectors ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       bool paused, double fps)
{
    erase();
    ctx_draw_bg(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    atexit(screen_cleanup);
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
    screen_init();
    color_init(theme);

    GridCtx g = {0};
    g.r_unit   = R_UNIT_DEFAULT;
    g.n_spokes = N_SECTORS_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    bool   paused = false;
    double fps    = TARGET_FPS;
    int64_t t0    = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            ctx_init(&g, LINES, COLS);
            cursor_reset(&cur, &g);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'p': paused = !paused; break;
        case 'r': cursor_reset(&cur, &g); break;
        case 't': theme = (theme + 1) % N_THEMES; color_init(theme); break;
        case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
        case '+': case '=':
            if (g.r_unit < R_UNIT_MAX) {
                g.r_unit += R_UNIT_STEP;
                ctx_init(&g, LINES, COLS);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case '-':
            if (g.r_unit > R_UNIT_MIN) {
                g.r_unit -= R_UNIT_STEP;
                ctx_init(&g, LINES, COLS);
            }
            break;
        case '[':
            if (g.n_spokes > N_SECTORS_MIN) {
                g.n_spokes -= (g.n_spokes > 8 ? 4 : 2);
                ctx_init(&g, LINES, COLS);
                if (cur.spoke > g.max_spoke) cur.spoke = g.max_spoke;
            }
            break;
        case ']':
            if (g.n_spokes < N_SECTORS_MAX) {
                g.n_spokes += (g.n_spokes >= 8 ? 4 : 2);
                ctx_init(&g, LINES, COLS);
            }
            break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;
        if (!paused)
            scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
