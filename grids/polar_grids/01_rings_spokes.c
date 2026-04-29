/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_rings_spokes.c — standard polar grid: concentric rings + radial spokes
 *
 * DEMO: Every screen cell is tested for proximity to a ring or spoke using
 *       polar coordinates.  Rings are detected when the pixel radius is a
 *       near-integer multiple of RING_SPACING.  Spokes are detected when
 *       the polar angle is a near-multiple of 2π/N_SPOKES.  Characters are
 *       chosen by angle: '-' horizontal, '|' vertical, '/' and '\' diagonal.
 *
 * Study alongside: 02_log_polar.c (logarithmic ring spacing)
 *                  grids/rect_grids/01_uniform_rect.c (analogous rect sweep)
 *
 * Section map:
 *   §1 config   — ring spacing, spoke count, thresholds, colour themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID
 *   §4 coords   — cell_to_polar: (col,row) → (r_px, theta)
 *   §5 draw     — grid sweep, on_ring / on_spoke tests, angle_char
 *   §6 scene    — scene_draw: background + HUD
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme
 *        +/- ring spacing   [/] spoke count
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids/01_rings_spokes.c \
 *       -o 01_rings_spokes -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Screen-sweep polar detection.  For every terminal cell
 *                  (col, row) the pixel-space distance r and angle θ from
 *                  the screen centre are computed.  Two Boolean tests decide
 *                  what to draw:
 *
 *                    on_ring : fmod(r, RING_SPACING) < RING_W
 *                              || fmod(r, RING_SPACING) > RING_SPACING−RING_W
 *                    on_spoke: fmod(θ_norm, spoke_angle) < SPOKE_W
 *                              || fmod(θ_norm, spoke_angle) > spoke_angle−SPOKE_W
 *
 *                  Both tests use modular arithmetic so they detect ALL rings
 *                  and ALL spokes simultaneously with a single expression each.
 *
 * Math           : Pixel coordinates respect terminal aspect ratio:
 *                    dx_px = (col − ox) × CELL_W
 *                    dy_px = (row − oy) × CELL_H
 *                    r     = √(dx_px² + dy_px²)   [circular in pixel space]
 *                    θ     = atan2(dy_px, dx_px)   [−π, π]
 *                  CELL_H / CELL_W = 2 compensates for the fact that terminal
 *                  rows are taller than columns — without it the grid appears
 *                  as ellipses, not circles.
 *
 * Rendering      : Character selection by angle makes spokes look like
 *                  actual lines: horizontal near ±0°, vertical near ±90°,
 *                  diagonals in between.  The ring character uses the same
 *                  angle_char() function so ring and spoke intersections join
 *                  smoothly.
 *
 * Performance    : O(rows × cols) per frame with one sqrt + one atan2 per
 *                  cell.  At 80×24 that is 1 920 cells — imperceptibly fast.
 *
 * References     :
 *   Polar coordinate system — en.wikipedia.org/wiki/Polar_coordinate_system
 *   Terminal aspect ratio   — CLAUDE.md §Coordinate/Physics Model
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   A polar grid divides the plane into concentric rings (equal distance
 *   bands) and radial spokes (equal angle wedges).  Every screen cell is
 *   either on a ring boundary, on a spoke boundary, both, or neither.
 *
 * HOW TO THINK ABOUT IT
 *   Picture the screen as a flat disc viewed from directly above.  The origin
 *   is the screen centre.  Moving right is angle 0°; moving down is 90°
 *   (terminal y grows downward, so atan2 follows screen-y = positive).
 *
 *   Two independent Boolean tests cover the entire grid:
 *     • Ring test: is the cell's radius a near-integer multiple of RING_SPACING?
 *     • Spoke test: is the cell's angle a near-integer multiple of 2π/N_SPOKES?
 *
 * DRAWING METHOD
 *   1. Compute pixel-space offset: dx = (col−ox)×CELL_W, dy = (row−oy)×CELL_H
 *   2. r = √(dx²+dy²),  θ = atan2(dy,dx)  ∈ (−π, π]
 *   3. Normalise θ to [0, 2π):  θ_norm = fmod(θ + 2π, 2π)
 *   4. Ring test:  ring_phase = fmod(r, RING_SPACING)
 *                  on_ring = ring_phase < RING_W  ||  ring_phase > RING_SPACING−RING_W
 *   5. Spoke test: spoke_phase = fmod(θ_norm, spoke_angle)
 *                  on_spoke = r > SPOKE_MIN_R  &&
 *                             (spoke_phase < SPOKE_W || spoke_phase > spoke_angle−SPOKE_W)
 *   6. Draw '+' at intersection, angle_char(θ) otherwise.
 *
 * KEY FORMULAS
 *   Aspect correction (why CELL_H=4, CELL_W=2):
 *     Terminal characters are ~2× taller than wide.  Without dy×CELL_H
 *     scaling, "equal radii" form ellipses on screen.  The ratio
 *     CELL_H/CELL_W = 2 matches the typical 2:1 terminal cell aspect,
 *     making the Euclidean distance circular rather than elliptical.
 *
 *   fmod dual-boundary trick:
 *     ring_phase ∈ [0, RING_SPACING).  It is near 0 at any integer multiple
 *     of RING_SPACING, and near RING_SPACING at the SAME boundary from the
 *     other side.  Both checks (< RING_W and > RING_SPACING−RING_W) catch
 *     both edges of each ring line with one fmod call.
 *
 *   Angular half-width SPOKE_W in radians:
 *     At radius r, a spoke of angular half-width SPOKE_W subtends arc r×SPOKE_W.
 *     Inner spokes look thinner; outer spokes look wider — natural for a
 *     constant-angle wedge.
 *
 * EDGE CASES TO WATCH
 *   • Centre smear: at r < SPOKE_MIN_R the spoke test would fill a solid disc.
 *     Guarded with (r_px > SPOKE_MIN_R).
 *   • θ range: atan2 returns (−π, π].  Adding 2π before fmod normalises to
 *     [0, 2π) — without this, fmod(−0.1, spoke_angle) ≈ −0.1 (negative),
 *     which is neither < SPOKE_W nor > spoke_angle−SPOKE_W → spoke goes missing.
 *   • Ring at r=0: ring_phase=0 → always on_ring at origin.  The SPOKE_MIN_R
 *     guard does NOT suppress this centre dot.
 *
 * HOW TO VERIFY
 *   Terminal 80×24, RING_SPACING=20, N_SPOKES=12, ox=40, oy=12.
 *
 *   Cell (col=60, row=12) — right of centre on horizontal axis:
 *     dx = (60−40)×2 = 40,  dy = (12−12)×4 = 0
 *     r = √(1600) = 40.0 px  ← exactly 2×RING_SPACING → ring_phase = 0
 *     on_ring = (0 < 1.6) = true
 *     θ = atan2(0,40) = 0  →  angle_char(0) = '-'  ✓
 *
 *   Cell (col=40, row=7) — directly above centre:
 *     dx = 0,  dy = (7−12)×4 = −20
 *     r = 20.0 px ← 1×RING_SPACING → on_ring = true
 *     θ = atan2(−20,0) = −π/2
 *     angle_char: fmod(−π/2 + 2π, π) = π/2 ∈ [3π/8, 5π/8) → '|'  ✓
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

#define TARGET_FPS    30

/* Terminal cell size in "pixels" — ratio CELL_H/CELL_W ≈ 2 makes circles
 * appear circular despite rows being taller than columns.              */
#define CELL_W        2
#define CELL_H        4

/* Ring geometry */
#define RING_SPACING_DEFAULT  20.0f  /* pixels between rings  */
#define RING_SPACING_MIN       8.0f
#define RING_SPACING_MAX      48.0f
#define RING_SPACING_STEP      4.0f
#define RING_W                 1.6f  /* pixel half-width of a ring line */

/* Spoke geometry */
#define N_SPOKES_DEFAULT  12
#define N_SPOKES_MIN       4
#define N_SPOKES_MAX      36
#define SPOKE_W            0.10      /* radian half-width of a spoke line */
#define SPOKE_MIN_R        3.0f     /* ignore centre blob */

/* Colour pairs */
#define PAIR_GRID   1
#define PAIR_HUD    2
#define PAIR_LABEL  3

/* Theme palette: 256-colour fg, 8-colour fallback */
static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
#define N_THEMES  5

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
 *   dx_px = (col − ox) × CELL_W      [pixel-space x offset]
 *   dy_px = (row − oy) × CELL_H      [pixel-space y offset]
 *   r_px  = √(dx_px² + dy_px²)       [Euclidean distance in pixel space]
 *   theta = atan2(dy_px, dx_px)       [angle ∈ (−π, π]]
 *
 * CELL_H/CELL_W = 2 compensates for terminal aspect: cells are ~2× taller
 * than wide, so un-scaled dy shrinks vertical distances by half, turning
 * circles into ellipses on screen.
 */
static void cell_to_polar(int col, int row, int ox, int oy,
                           float *r_px, double *theta)
{
    double dx = (double)(col - ox) * CELL_W;
    double dy = (double)(row - oy) * CELL_H;
    *r_px  = (float)sqrt(dx*dx + dy*dy);
    *theta = atan2(dy, dx); /* [-π, π] */
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  draw                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * angle_char — pick the ASCII line character that best matches orientation theta.
 *
 * THE FORMULA:
 *   a = fmod(theta + 2π, π)   ← fold (−π, π] into [0, π) (lines have no direction)
 *   a ∈ [0, π/8)  or  [7π/8, π) → '-'   (near-horizontal)
 *   a ∈ [π/8,  3π/8)           → '\'   (diagonal down-right)
 *   a ∈ [3π/8, 5π/8)           → '|'   (near-vertical)
 *   a ∈ [5π/8, 7π/8)           → '/'   (diagonal up-right)
 *
 * Why fold by π not 2π?  A line at angle α and α+π looks identical in ASCII
 * (no directed arrow, only orientation), so folding into [0,π) halves the cases.
 */
static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI); /* fold to [0, π) */
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/*
 * grid_draw — sweep every cell, apply ring and spoke tests, draw.
 *
 * THE PIPELINE:
 *   for each cell (col, row):
 *     (r, θ) ← cell_to_polar()             aspect-corrected polar coords
 *     ring_phase  = fmod(r, ring_spacing)   scalar distance to nearest ring
 *     spoke_phase = fmod(θ_norm, spoke_angle) scalar angle to nearest spoke
 *     on_ring  = ring_phase  < RING_W  ||  ring_phase  > spacing − RING_W
 *     on_spoke = r > MIN_R   &&  (spoke_phase < SPOKE_W || > angle − SPOKE_W)
 *     draw '+' if both; angle_char(θ) if either; skip if neither
 *
 * The dual-boundary test (< W || > step−W) fires at BOTH edges of each band,
 * so the drawn line has width 2×RING_W px centered on the ring radius.
 */
static void grid_draw(int rows, int cols, int ox, int oy,
                      float ring_spacing, int n_spokes)
{
    double spoke_angle = 2.0 * M_PI / (double)n_spokes;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            float  r_px;
            double theta;
            cell_to_polar(col, row, ox, oy, &r_px, &theta);

            /* ring test: r near any integer multiple of ring_spacing */
            float ring_phase = fmodf(r_px, ring_spacing);
            bool on_ring = (ring_phase < RING_W ||
                            ring_phase > ring_spacing - RING_W);

            /* spoke test: theta near any multiple of spoke_angle */
            double theta_norm = fmod(theta + 2.0*M_PI, 2.0*M_PI);
            double spoke_phase = fmod(theta_norm, spoke_angle);
            bool on_spoke = (r_px > SPOKE_MIN_R) &&
                            (spoke_phase < SPOKE_W ||
                             spoke_phase > spoke_angle - SPOKE_W);

            if (!on_ring && !on_spoke) continue;

            char c;
            if (on_ring && on_spoke) c = '+';
            else if (on_spoke)       c = angle_char(theta);
            else                     c = angle_char(theta); /* ring: tangent char */

            mvaddch(row, col, (chtype)(unsigned char)c);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(int rows, int cols, float ring_spacing, int n_spokes,
                       int theme, double fps, bool paused)
{
    int ox = cols / 2, oy = rows / 2;
    erase();
    grid_draw(rows, cols, ox, oy, ring_spacing, n_spokes);

    /* top-right HUD */
    char buf[80];
    snprintf(buf, sizeof buf, " %.1f fps  rings:%.0fpx  spokes:%d  %s ",
             fps, (double)ring_spacing, n_spokes, paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* bottom-left key strip */
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 0,
        " q:quit  p:pause  t:theme(%d)  +/-:ring-spacing  [/]:spokes ", theme+1);
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
    color_init(theme);
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

    int theme       = 0;
    screen_init(theme);

    int    rows = LINES, cols = COLS;
    float  ring_spacing = RING_SPACING_DEFAULT;
    int    n_spokes     = N_SPOKES_DEFAULT;
    bool   paused       = false;
    double fps          = TARGET_FPS;
    int64_t t0          = clock_ns();
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
        case 't':
            theme = (theme + 1) % N_THEMES;
            color_init(theme);
            break;
        case '+': case '=':
            if (ring_spacing < RING_SPACING_MAX) ring_spacing += RING_SPACING_STEP;
            break;
        case '-':
            if (ring_spacing > RING_SPACING_MIN) ring_spacing -= RING_SPACING_STEP;
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
            scene_draw(rows, cols, ring_spacing, n_spokes, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
