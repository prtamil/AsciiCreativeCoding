/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 07_elliptic.c — elliptic polar grid (confocal ellipses + hyperbolae)
 *
 * DEMO: Concentric ellipses replace the circular rings of 01_rings_spokes.
 *       The aspect ratio A:B can be adjusted with arrow keys, morphing from
 *       circles (A=B) through increasingly elongated ellipses.  The 'h' key
 *       overlays confocal hyperbolae — the orthogonal family to the ellipses
 *       — producing the classic confocal conic section pattern found in
 *       optics and electrostatics.
 *
 * Study alongside: 01_rings_spokes.c (circular polar — A=B special case),
 *                  06_sector.c (equal-area rings)
 *
 * Section map:
 *   §1 config   — semi-axes A, B, ring spacing, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID
 *   §4 coords   — cell_to_elliptic, angle_char
 *   §5 draw     — elliptic ring test, hyperbola overlay
 *   §6 scene    — scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme   h toggle-hyperbolae
 *        +/- ring spacing   a/z semi-axis A   s/x semi-axis B
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids/07_elliptic.c \
 *       -o 07_elliptic -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Elliptic coordinate grid.  Every cell is mapped to an
 *                  "elliptic radius":
 *
 *                    e_r = sqrt((dx_px/A)² + (dy_px/B)²)
 *
 *                  where A, B are the semi-axis scale factors.  Rings sit at
 *                  integer values of e_r (spaced by RING_SPACING):
 *
 *                    u = e_r / RING_SPACING
 *                    on_ring: fmod(u, 1.0) < RING_W_U || > 1 − RING_W_U
 *
 *                  When A = B this reduces to the standard circular rings of
 *                  01_rings_spokes (with u = r_px / RING_SPACING).
 *
 *                  The hyperbola family: confocal hyperbolae are the level
 *                  sets of the "hyperbolic coordinate" v = dx_px/A / e_r =
 *                  cos(elliptic_angle).  Boundaries of constant v form the
 *                  radial lines of the elliptic grid:
 *
 *                    v = (dx_px / A) / e_r   ∈ [−1, 1]
 *                    on_hyp: fmod(v_norm, HYPER_STEP) < HYPER_W
 *
 *                  Together, the two families are orthogonal at every point
 *                  (the ellipses and hyperbolae are at right angles).
 *
 * Math           : Elliptic coordinates (μ, ν) are defined by:
 *                    x = c × cosh μ × cos ν
 *                    y = c × sinh μ × sin ν
 *                  with focal distance c = √(A² − B²) (assumes A ≥ B).
 *                  This file uses the simpler "scaled-radius" form e_r =
 *                  sqrt((x/A)²+(y/B)²) which is the μ = const level set
 *                  for a conformal elliptic map.
 *
 *                  Physical motivation: the scalar potential of an elliptic
 *                  cylinder in electrostatics is constant on confocal
 *                  ellipses — this is the equipotential grid.
 *
 * Rendering      : Ring character follows angle_char(atan2(dy/B, dx/A)) —
 *                  the tangent to the ellipse at that point, not the raw
 *                  screen angle.  This keeps the ring characters aligned
 *                  with the ellipse rather than the circle.
 *
 * Performance    : O(rows × cols) per frame.  Two divisions per cell (dx/A,
 *                  dy/B) plus one sqrt — comparable to 01.
 *
 * References     :
 *   Elliptic coordinates — en.wikipedia.org/wiki/Elliptic_coordinate_system
 *   Confocal conics — en.wikipedia.org/wiki/Confocal_conic_sections
 *   Electrostatics of cylinders — Griffiths "Introduction to
 *     Electrodynamics" §3.3
 *   Elliptic integrals — Abramowitz & Stegun §17
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   Replace the circular polar rings with elliptic rings by dividing pixel
 *   coordinates by semi-axis factors A and B before computing the radius.
 *   The "elliptic radius" e_r = √((dx/A)²+(dy/B)²) is constant on confocal
 *   ellipses.  When A=B=1 this reduces exactly to the circular polar grid.
 *
 * HOW TO THINK ABOUT IT
 *   Take the circular grid (01) and independently scale the x-axis by A and
 *   the y-axis by B.  Every circle becomes an ellipse with semi-axis ratio A:B.
 *   The orthogonal family — curves that cross every ellipse at 90° — are the
 *   confocal hyperbolae, revealed by the 'h' key.
 *
 *   Physical analogy: elliptic conducting cylinders in electrostatics have
 *   confocal-ellipse equipotentials and confocal-hyperbola field lines.
 *
 * DRAWING METHOD
 *   1. dx_px = (col−ox)×CELL_W,  dy_px = (row−oy)×CELL_H
 *   2. u = dx_px / A,  v = dy_px / B          ← axis-scaled coordinates
 *   3. e_r = √(u²+v²)                          ← elliptic radius
 *   4. ell_theta = atan2(v, u)                 ← angle in scaled space
 *   5. Ring test: u_ring = e_r / RING_SPACING
 *                 frac = u_ring − floor(u_ring)
 *                 on_ring = (frac < RING_W_U || frac > 1−RING_W_U)
 *   6. Hyperbola test (if show_hyper):
 *                 cv = |cos(ell_theta)|         ← ∈ [0,1]
 *                 hfrac = fmod(cv, HYPER_STEP)
 *                 on_hyper = (hfrac < HYPER_W || hfrac > HYPER_STEP−HYPER_W)
 *   7. Draw '+' (intersection), colored hyperbola, ring, or skip.
 *
 * KEY FORMULAS
 *   Elliptic radius: e_r = √((dx_px/A)² + (dy_px/B)²)
 *     Level sets e_r=const are ellipses with semi-axes A×const and B×const.
 *     When A=B: e_r = √(dx²+dy²)/A = r/A (circular, just scaled).
 *
 *   Character direction: angle_char uses ell_theta = atan2(v, u).
 *     This is the angle in axis-scaled space, which aligns the character
 *     with the tangent to the ellipse at that point (not the raw screen angle).
 *
 *   Hyperbola detection: confocal hyperbolae are level sets of
 *     cos(ell_theta) = u / e_r
 *     fabs used so only magnitude matters (hyperbolae are symmetric about axes).
 *
 * EDGE CASES TO WATCH
 *   • A or B near 0: e_r becomes huge along the respective axis.  Constrained
 *     to [AXIS_MIN, AXIS_MAX].  E_R_MIN guards very small e_r near origin.
 *   • A=B: hyperbolae degenerate to two half-lines (the ±x axis).  Visually
 *     correct but looks like only two spokes — toggle 'h' off to clean up.
 *   • HYPER_STEP: 0.05–0.20 gives clean results; outside this range either
 *     too dense (overlapping) or too sparse (gaps between hyperbola lines).
 *
 * HOW TO VERIFY
 *   A=1.6, B=1.0, RING_SPACING=20, ox=40, oy=12.
 *
 *   Cell (col=56, row=12) — rightmost point of first ellipse ring:
 *     dx_px=(56−40)×2=32, dy_px=0
 *     u=32/1.6=20,  v=0  →  e_r=20.0
 *     u_ring=20/20=1.000  →  frac=0 < RING_W_U(0.07)  →  on_ring ✓
 *     ell_theta=atan2(0,20)=0  →  angle_char(0)='-'  ✓
 *
 *   Cell (col=40, row=7) — topmost point of same ellipse ring:
 *     dx_px=0, dy_px=(7−12)×4=−20
 *     u=0,  v=−20/1.0=−20  →  e_r=20.0  →  on_ring ✓  (same ring!)
 *     ell_theta=atan2(−20,0)=−π/2  →  angle_char=  '|'  ✓
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

/* Semi-axis scale factors (pixels).  Rings are ellipses with ratio A:B. */
#define AXIS_A_DEFAULT  1.6    /* x-axis scale factor (width stretch)  */
#define AXIS_B_DEFAULT  1.0    /* y-axis scale factor (height stretch) */
#define AXIS_MIN        0.5
#define AXIS_MAX        4.0
#define AXIS_STEP       0.1

/* Ring spacing in elliptic-radius units */
#define RING_SPACING_DEFAULT  20.0
#define RING_SPACING_MIN       8.0
#define RING_SPACING_MAX      48.0
#define RING_SPACING_STEP      4.0

/* Fractional ring width in u = e_r/RING_SPACING space */
#define RING_W_U        0.07

/* Hyperbola overlay: step between v-contours (v = cos of elliptic angle) */
#define HYPER_STEP      0.12   /* spacing between hyperbola lines in v ∈[0,1] */
#define HYPER_W         0.02   /* half-width in v space */

/* Minimum elliptic radius — avoids centre smear */
#define E_R_MIN         0.3

#define PAIR_GRID    1
#define PAIR_HUD     2
#define PAIR_LABEL   3
#define PAIR_HYPER   4   /* separate color for hyperbola family */

static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
/* Hyperbola highlight colors (paired with THEME_FG) */
static const short THEME_HFG[][2] = {
    {214, COLOR_YELLOW},
    {220, COLOR_YELLOW},
    {214, COLOR_YELLOW},
    {82,  COLOR_GREEN},
    {75,  COLOR_CYAN},
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
    short fg  = COLORS >= 256 ? THEME_FG[theme][0]  : THEME_FG[theme][1];
    short hfg = COLORS >= 256 ? THEME_HFG[theme][0] : THEME_HFG[theme][1];
    init_pair(PAIR_GRID,  fg,                              -1);
    init_pair(PAIR_HUD,   COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_LABEL, COLORS>=256 ? 252 : COLOR_WHITE, -1);
    init_pair(PAIR_HYPER, hfg,                             -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  coords                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_elliptic — compute elliptic radius e_r and tangent angle ell_theta.
 *
 * THE FORMULA:
 *   dx_px = (col − ox) × CELL_W,  dy_px = (row − oy) × CELL_H
 *   u = dx_px / axis_a,  v = dy_px / axis_b    ← axis-scaled coordinates
 *   e_r       = √(u² + v²)                     ← elliptic radius (const on ellipses)
 *   ell_theta = atan2(v, u)                     ← angle in scaled space
 *
 * Using atan2(v,u) rather than atan2(dy,dx) aligns angle_char with the
 * ellipse tangent direction, not the raw screen direction.
 */
static void cell_to_elliptic(int col, int row, int ox, int oy,
                              double axis_a, double axis_b,
                              double *e_r, double *ell_theta)
{
    double dx_px = (double)(col - ox) * CELL_W;
    double dy_px = (double)(row - oy) * CELL_H;
    double u     = dx_px / axis_a;
    double v     = dy_px / axis_b;
    *e_r      = sqrt(u*u + v*v);
    *ell_theta = atan2(v, u);     /* tangent-aligned angle for char selection */
}

/*
 * angle_char — pick the ASCII line character that best matches orientation theta.
 *
 * THE FORMULA:
 *   a = fmod(theta + 2π, π)  ← fold into [0, π) (orientation, not direction)
 *   a ∈ [0, π/8) or [7π/8, π) → '-';  a ∈ [π/8, 3π/8) → '\'
 *   a ∈ [3π/8, 5π/8) → '|';          a ∈ [5π/8, 7π/8) → '/'
 *
 * Called with ell_theta (scaled space angle), so characters align with
 * the ellipse tangent, not the circular tangent.
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
 * grid_draw — sweep every cell, apply elliptic ring and hyperbola tests, draw.
 *
 * THE PIPELINE:
 *   for each cell:
 *     (e_r, ell_theta) ← cell_to_elliptic(axis_a, axis_b)
 *     if e_r < E_R_MIN: skip
 *     u    = e_r / ring_spacing
 *     frac = u − floor(u)
 *     on_ring  = frac < RING_W_U  ||  frac > 1 − RING_W_U
 *     on_hyper = |cos(ell_theta)| stepped by HYPER_STEP  (if show_hyper)
 *     draw intersection(+)/hyperbola/ring/skip with separate color pairs
 */
static void grid_draw(int rows, int cols, int ox, int oy,
                      double axis_a, double axis_b,
                      double ring_spacing, bool show_hyper)
{
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double e_r, ell_theta;
            cell_to_elliptic(col, row, ox, oy, axis_a, axis_b,
                             &e_r, &ell_theta);
            if (e_r < E_R_MIN) continue;

            /* Elliptic ring test: u = e_r / ring_spacing */
            double u    = e_r / ring_spacing;
            double frac = u - floor(u);
            bool on_ring = (frac < RING_W_U || frac > 1.0 - RING_W_U);

            /* Hyperbola test: cos(ell_theta) stepped by HYPER_STEP */
            bool on_hyper = false;
            if (show_hyper) {
                double cv   = fabs(cos(ell_theta));   /* ∈ [0,1] */
                double hfrac = fmod(cv, HYPER_STEP);
                on_hyper = (hfrac < HYPER_W || hfrac > HYPER_STEP - HYPER_W);
            }

            if (!on_ring && !on_hyper) continue;

            char c = angle_char(ell_theta);
            if (on_ring && on_hyper) {
                attron(COLOR_PAIR(PAIR_HYPER));
                mvaddch(row, col, (chtype)(unsigned char)'+');
                attroff(COLOR_PAIR(PAIR_HYPER));
            } else if (on_hyper) {
                attron(COLOR_PAIR(PAIR_HYPER));
                mvaddch(row, col, (chtype)(unsigned char)c);
                attroff(COLOR_PAIR(PAIR_HYPER));
            } else {
                attron(COLOR_PAIR(PAIR_GRID));
                mvaddch(row, col, (chtype)(unsigned char)c);
                attroff(COLOR_PAIR(PAIR_GRID));
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(int rows, int cols,
                       double axis_a, double axis_b, double ring_spacing,
                       bool show_hyper, int theme, double fps, bool paused)
{
    int ox = cols / 2, oy = rows / 2;
    erase();
    grid_draw(rows, cols, ox, oy, axis_a, axis_b, ring_spacing, show_hyper);

    char buf[80];
    snprintf(buf, sizeof buf, " %.1f fps  A:%.1f B:%.1f  sp:%.0f  %s%s ",
             fps, axis_a, axis_b, ring_spacing,
             show_hyper ? "+hyper  " : "",
             paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 0,
        " q:quit  p:pause  t:theme(%d)  h:hyperbolae  +/-:spacing  a/z:A  s/x:B ",
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

    int    rows         = LINES, cols = COLS;
    double axis_a       = AXIS_A_DEFAULT;
    double axis_b       = AXIS_B_DEFAULT;
    double ring_spacing = RING_SPACING_DEFAULT;
    bool   show_hyper   = false;
    bool   paused       = false;
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
        case 'h': show_hyper = !show_hyper; break;
        case '+': case '=':
            if (ring_spacing < RING_SPACING_MAX) ring_spacing += RING_SPACING_STEP;
            break;
        case '-':
            if (ring_spacing > RING_SPACING_MIN) ring_spacing -= RING_SPACING_STEP;
            break;
        case 'a':
            if (axis_a < AXIS_MAX) axis_a += AXIS_STEP;
            break;
        case 'z':
            if (axis_a > AXIS_MIN) axis_a -= AXIS_STEP;
            break;
        case 's':
            if (axis_b < AXIS_MAX) axis_b += AXIS_STEP;
            break;
        case 'x':
            if (axis_b > AXIS_MIN) axis_b -= AXIS_STEP;
            break;
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;
        if (!paused)
            scene_draw(rows, cols, axis_a, axis_b, ring_spacing,
                       show_hyper, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
