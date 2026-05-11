/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 07_elliptic.c — elliptic polar grid (confocal ellipses + hyperbolae)
 *
 * DEMO: Concentric ellipses replace the circular rings of 01_rings_spokes.
 *       The aspect ratio A:B can be adjusted with arrow keys, morphing from
 *       circles (A=B) through increasingly elongated ellipses.  The 'h' key
 *       overlays confocal hyperbolae — the orthogonal family to the ellipses
 *       — producing the classic confocal conic section pattern found in
 *       optics and electrostatics.  An '@' cursor sits at one (ring, spoke)
 *       cell — arrows step it across the grid.
 *
 * Study alongside: 01_rings_spokes.c (circular polar — A=B special case),
 *                  06_sector.c (equal-area rings),
 *                  ../rect_grids/01_uniform_rect.c (the GridCtx template)
 *
 * Section map:
 *   §1 config   — semi-axes A, B, ring spacing, themes, EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID + HUD/HINT/CURSOR/HYPER
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg + angle_char
 *   §5 cursor   — Cursor (ring, spoke) + cursor_reset / cursor_move / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme   r reset   h toggle-hyperbolae
 *        arrows move @   +/- ring spacing   a/z semi-axis A   s/x semi-axis B
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
 *                  integer values of e_r (spaced by ring_spacing):
 *
 *                    u = e_r / ring_spacing
 *                    on_ring: fmod(u, 1.0) < RING_W_U || > 1 − RING_W_U
 *
 *                  When A = B this reduces to the standard circular rings of
 *                  01_rings_spokes (with u = r_px / ring_spacing).
 *
 *                  The hyperbola family: confocal hyperbolae are the level
 *                  sets of the "hyperbolic coordinate" v = dx_px/A / e_r =
 *                  cos(elliptic_angle).  Boundaries of constant v form the
 *                  radial lines of the elliptic grid:
 *
 *                    v = (dx_px / A) / e_r   ∈ [−1, 1]
 *                    on_hyp: fmod(v_norm, HYPER_STEP) < HYPER_W
 *
 *                  Together, the two families are orthogonal at every point.
 *
 * Data-structure : Two structs — GridCtx (terminal extent, axis_a, axis_b,
 *                  ring_spacing, show_hyper, n_spokes, cursor bounds) and
 *                  Cursor (ring, spoke). No grid array; the grid is computed
 *                  per pixel via the ring + hyperbola tests above. Mirrors
 *                  the (ring, spoke) cursor of 01_rings_spokes — only the
 *                  radial law in §4 differs.
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
 *   5. Ring test: u_ring = e_r / ring_spacing
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
 *   Cursor forward map (ring k, spoke s):
 *     theta_mid = (s + 0.5) × (2π / n_spokes)
 *     u_mid = (k + 0.5) × ring_spacing
 *     cx = u_mid × A × cos theta_mid;  cy = u_mid × B × sin theta_mid
 *     sc = ox + round(cx / CELL_W);  sr = oy + round(cy / CELL_H)
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
 *   A=1.6, B=1.0, ring_spacing=20, ox=40, oy=12.
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

/* Fractional ring width in u = e_r/ring_spacing space */
#define RING_W_U        0.07

/* Default spoke count for the cursor's angular index.  Drawing the rings
 * does not depend on n_spokes (rings are continuous ellipses), but the
 * cursor needs a discrete angular step so arrow keys snap cleanly. */
#define N_SPOKES_DEFAULT  12

/* Hyperbola overlay: step between v-contours (v = cos of elliptic angle) */
#define HYPER_STEP      0.12   /* spacing between hyperbola lines in v ∈[0,1] */
#define HYPER_W         0.02   /* half-width in v space */

/* Minimum elliptic radius — avoids centre smear */
#define E_R_MIN         0.3

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

/* Color pair IDs */
#define PAIR_GRID    1
#define PAIR_HUD     2   /* status bar (yellow)          */
#define PAIR_HINT    3   /* key-hint footer (cyan)       */
#define PAIR_CURSOR  4   /* bright '@'                   */
#define PAIR_HYPER   5   /* hyperbola family overlay     */

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

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg  = COLORS >= 256 ? THEME_FG[theme][0]  : THEME_FG[theme][1];
    short hfg = COLORS >= 256 ? THEME_HFG[theme][0] : THEME_HFG[theme][1];
    init_pair(PAIR_GRID,   fg,                                -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HYPER,  hfg,                                -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the elliptic ring + hyperbola pipeline        */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the elliptic polar grid plus cursor bounds.
 *
 * Carries axis_a/axis_b/ring_spacing as fields (not just #defines) so the
 * keyboard handlers can mutate them on the live struct and ctx_to_screen +
 * ctx_draw_bg pick up the new values immediately. n_spokes is the discrete
 * angular step the CURSOR uses; the rings themselves are continuous and do
 * not depend on it.
 */
typedef struct {
    int rows, cols;
    int cell_w, cell_h;
    int ox, oy;

    double axis_a;          /* x-axis scale factor (pixels) */
    double axis_b;          /* y-axis scale factor (pixels) */
    double ring_spacing;    /* spacing between rings in e_r units */
    bool   show_hyper;      /* hyperbola overlay toggle */

    int    n_spokes;        /* discrete angular cursor steps */
    int    max_ring, max_spoke;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size.
 *
 * max_ring is the largest k whose midpoint ellipse still fits inside the
 * screen rectangle along EITHER axis; we underestimate by using the smaller
 * of the two visible half-extents divided by (axis × ring_spacing).
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;

    if (g->axis_a       <= 0.0) g->axis_a       = AXIS_A_DEFAULT;
    if (g->axis_b       <= 0.0) g->axis_b       = AXIS_B_DEFAULT;
    if (g->ring_spacing <= 0.0) g->ring_spacing = RING_SPACING_DEFAULT;
    if (g->n_spokes     <= 0)   g->n_spokes     = N_SPOKES_DEFAULT;

    /* Visible half-extent in pixels along each axis */
    double hx_px = (double)cols * 0.5 * CELL_W;
    double hy_px = (double)rows * 0.5 * CELL_H;
    /* Largest ellipse-radius (in e_r units) that still fits along each axis */
    double max_u_x = hx_px / g->axis_a;
    double max_u_y = hy_px / g->axis_b;
    double max_u   = (max_u_x < max_u_y ? max_u_x : max_u_y);
    int mr = (int)floor(max_u / g->ring_spacing - 0.5);
    if (mr < 0) mr = 0;
    g->max_ring  = mr;
    g->max_spoke = g->n_spokes - 1;
}

/*
 * ctx_to_screen — equal-step centre of (ring k, spoke s).
 *
 * THE FORMULA:
 *   theta_mid = (s + 0.5) × (2π / n_spokes)
 *   u_mid     = (k + 0.5) × ring_spacing       (mid-radius of annulus k)
 *   cx = u_mid × axis_a × cos theta_mid        (un-scale back to pixel space)
 *   cy = u_mid × axis_b × sin theta_mid
 *   sc = ox + round(cx / CELL_W)
 *   sr = oy + round(cy / CELL_H)
 *
 * The midpoint convention places the cursor visually inside the annulus
 * rather than on a ring boundary, matching 01_rings_spokes.
 */
static void ctx_to_screen(const GridCtx *g, int ring, int spoke,
                          int *sr, int *sc)
{
    double theta_mid = ((double)spoke + 0.5) *
                       (2.0 * M_PI / (double)g->n_spokes);
    double u_mid     = ((double)ring + 0.5) * g->ring_spacing;
    double cx = u_mid * g->axis_a * cos(theta_mid);
    double cy = u_mid * g->axis_b * sin(theta_mid);
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

/*
 * ctx_draw_bg — sweep every cell, apply elliptic ring and hyperbola tests, draw.
 *
 * THE PIPELINE:
 *   for each cell:
 *     dx = (col-ox)×CELL_W,  dy = (row-oy)×CELL_H
 *     u  = dx / axis_a,      v = dy / axis_b
 *     e_r = √(u²+v²),  ell_theta = atan2(v, u);  if e_r < E_R_MIN: skip
 *     u_ring = e_r / ring_spacing
 *     frac   = u_ring − floor(u_ring)
 *     on_ring  = frac < RING_W_U  ||  frac > 1 − RING_W_U
 *     on_hyper = |cos(ell_theta)| stepped by HYPER_STEP   (if show_hyper)
 *     draw '+'/hyperbola/ring/skip with separate color pairs
 */
static void ctx_draw_bg(const GridCtx *g)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * g->cell_w;
            double dy = (double)(row - g->oy) * g->cell_h;
            double u  = dx / g->axis_a;
            double v  = dy / g->axis_b;
            double e_r       = sqrt(u*u + v*v);
            if (e_r < E_R_MIN) continue;
            double ell_theta = atan2(v, u);

            /* Elliptic ring test: u_ring = e_r / ring_spacing */
            double u_ring = e_r / g->ring_spacing;
            double frac   = u_ring - floor(u_ring);
            bool on_ring  = (frac < RING_W_U || frac > 1.0 - RING_W_U);

            /* Hyperbola test: cos(ell_theta) stepped by HYPER_STEP */
            bool on_hyper = false;
            if (g->show_hyper) {
                double cv    = fabs(cos(ell_theta));   /* ∈ [0,1] */
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
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — (ring index, spoke index).  Same shape as 01_rings_spokes;
 * only the radial law in ctx_to_screen differs (ellipse, not circle).
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
    if (sr < 0 || sr >= g->rows - 1 || sc < 0 || sc >= g->cols) return;
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, sc, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     double fps, bool paused)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " %.1f fps  A:%.1f B:%.1f  sp:%.0f  ring:%d/%d  spoke:%d  %s%s ",
             fps, g->axis_a, g->axis_b, g->ring_spacing,
             cur->ring, g->max_ring, cur->spoke,
             g->show_hyper ? "+hyper  " : "",
             paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme(%d)  r:reset  h:hyper  arrows:move "
             " +/-:spacing  a/z:A  s/x:B ",
             theme + 1);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       double fps, bool paused)
{
    erase();
    ctx_draw_bg(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, fps, paused);
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

    int theme = 0;
    screen_init(theme);

    GridCtx g = {0};
    ctx_init(&g, LINES, COLS);
    Cursor cur;
    cursor_reset(&cur, &g);

    bool paused = false;
    double fps = TARGET_FPS;
    int64_t t0 = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
            if (cur.ring  > g.max_ring)  cur.ring  = g.max_ring;
            if (cur.spoke > g.max_spoke) cur.spoke = g.max_spoke;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'p': paused = !paused; break;
        case 't': theme = (theme + 1) % N_THEMES; color_init(theme); break;
        case 'r': cursor_reset(&cur, &g); break;
        case 'h': g.show_hyper = !g.show_hyper; break;
        case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
        case '+': case '=':
            if (g.ring_spacing < RING_SPACING_MAX) {
                g.ring_spacing += RING_SPACING_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case '-':
            if (g.ring_spacing > RING_SPACING_MIN) {
                g.ring_spacing -= RING_SPACING_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case 'a':
            if (g.axis_a < AXIS_MAX) {
                g.axis_a += AXIS_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case 'z':
            if (g.axis_a > AXIS_MIN) {
                g.axis_a -= AXIS_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case 's':
            if (g.axis_b < AXIS_MAX) {
                g.axis_b += AXIS_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case 'x':
            if (g.axis_b > AXIS_MIN) {
                g.axis_b -= AXIS_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;
        if (!paused)
            scene_draw(&g, &cur, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
