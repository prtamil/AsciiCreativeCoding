/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_polar_scatter.c — four polar scatter strategies on a polar grid
 *
 * DEMO: Move the cursor and watch a live scatter preview update in real time.
 *       U/G/W/D select the distribution: U uniform-area, G radial Gaussian
 *       (clusters around the cursor ring), W angular wedge (fills the cursor
 *       sector), D ring-snap (objects gravitate to ring boundaries).  The
 *       preview regenerates whenever the cursor moves or a parameter changes.
 *       Press space to stamp the current preview into the permanent pool.
 *
 * Study alongside: 03_polar_spiral.c (parametric path placement),
 *                  grids/rect_grids_placement/04_scatter.c (rectangular version)
 *
 * Section map:
 *   §1 config   — scatter parameters: N, radii, sigma, wedge half-angle
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: grid, active, anchor, HUD (yellow), hint (cyan)
 *   §4 gridctx  — GridCtx (mode, origin), cell_to_polar, polar_to_screen
 *   §5 pool     — Pool: place, clear, draw, draw_preview, stamp
 *   §6 cursor   — Cursor (row, col, r, theta) + screen-space arrow move
 *   §7 mode     — draw_polar_bg: 7 inline polar background types in one switch
 *   §8 scatter  — gauss, scatter_uniform, scatter_gauss, scatter_wedge,
 *                  scatter_ringsnap
 *   §9 scene    — hud_draw + scene_draw
 *   §10 screen  — ncurses init / cleanup
 *   §11 app     — signals, resize, main loop
 *
 * Keys:  q/ESC quit   P pause   t theme   a/e prev/next background
 *        U/G/W/D select scatter mode (preview updates live)
 *        +/- N    [/] sigma (G) or wedge (W)
 *        space stamp preview to pool    C clear pool    r reset cursor
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids_placement/04_polar_scatter.c \
 *       -o 04_polar_scatter -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Four scatter strategies that differ only in how (r, θ)
 *                  is sampled.  Each converts the sample to a terminal cell
 *                  via polar_to_screen() and appends to the pool.
 *
 *                  U — Uniform area:
 *                    r  ~ sqrt(rand(r_min², r_max²))  — corrects radial bias
 *                    θ  ~ Uniform[0, 2π)
 *                    Without the sqrt correction, sampling r ~ Uniform
 *                    would over-weight the centre (small r has less area
 *                    per unit r but the same sampling probability).
 *
 *                  G — Radial Gaussian:
 *                    r  ~ |Normal(r_cursor, SIGMA_R)|
 *                    θ  ~ Uniform[0, 2π)
 *                    Objects cluster around the ring at radius r_cursor.
 *                    Box-Muller transform for the Normal deviate.
 *
 *                  W — Angular wedge:
 *                    r  ~ Uniform[R_INNER, R_OUTER]
 *                    θ  ~ Uniform[θ_cursor − WEDGE_HALF, θ_cursor + WEDGE_HALF]
 *                    Objects fill a pie-slice centred on the cursor angle.
 *
 *                  D — Ring-snap:
 *                    r  ~ nearest ring position k × RING_SNAP_SP + jitter
 *                    θ  ~ Uniform[0, 2π)
 *                    Objects appear to "snap" to ring boundaries.
 *
 * Math           : Uniform area correction: the disc of radius R has area
 *                  π × R².  Uniform sampling of r² (not r) gives uniform
 *                  areal density.  CDF inversion: r = sqrt(r_min² + rand × (r_max² − r_min²)).
 *
 *                  Box-Muller: U1, U2 ~ Uniform[0,1] →
 *                    Z = sqrt(−2 ln U1) × cos(2π U2)  ~ Normal(0,1)
 *
 * References     :
 *   Uniform disc sampling — en.wikipedia.org/wiki/Disk_sampling
 *   Box-Muller transform — en.wikipedia.org/wiki/Box-Muller_transform
 *   Rectangular analogue — grids/rect_grids_placement/04_scatter.c
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Four polar scatter strategies that differ only in how (r, θ) is sampled.
 * Every strategy converts its sample to a terminal cell via polar_to_screen()
 * and appends to the preview pool.  The dirty flag triggers regeneration
 * whenever cursor or parameters change; SPACE stamps preview → permanent pool.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine throwing darts at the polar grid with different aiming strategies:
 *
 *   U: blindfolded — completely random, but corrected so every ring band
 *      gets proportional coverage (outer bands are larger, so they get
 *      more darts when sampling uniformly over area, not over radius).
 *   G: aiming for a ring — most darts land near r_cursor with Gaussian
 *      falloff; the further from that ring the rarer the dart.
 *   W: aiming at a clock sector — darts land in a ±WEDGE_HALF wedge
 *      around the cursor angle.
 *   D: magnetised darts — they snap to ring boundaries regardless of aim.
 *
 * DRAWING METHOD
 * ──────────────
 * 1. Move cursor to set the scatter centre (r_cursor, θ_cursor).
 * 2. Select mode U/G/W/D; adjust N with +/−, param with [/].
 * 3. dirty=true triggers pool_clear(preview) + scatter_*(preview, ...).
 *    Preview drawn in amber (PAIR_ANCHOR) each frame.
 * 4. SPACE: pool_stamp copies preview → permanent pool (no regen needed).
 *
 * KEY FORMULAS
 * ────────────
 * gauss — Box-Muller transform:
 *   U1 = (rand+1)/(RAND_MAX+2) ∈ (0,1)   [+1 shift avoids log(0)]
 *   U2 = rand/(RAND_MAX+1)    ∈ [0,1)
 *   Z  = sqrt(−2 ln U1) × cos(2π U2)     ∼ Normal(0,1)
 *   return mean + sigma × Z
 *
 * scatter_uniform (U) — CDF-inversion for uniform areal density:
 *   Without correction, r~Uniform gives 4× more dots near r_min than r_max.
 *   Area of disc = π r², so sampling r² uniformly → uniform areal density.
 *   r = sqrt(R_INNER² + rand × (r_outer² − R_INNER²))
 *   θ ∈ [0, 2π) uniformly
 *
 * scatter_gauss (G) — Gaussian ring cluster:
 *   r = |Normal(r_cursor, SIGMA_R)|   [fabs keeps r > 0]
 *   r clamped to R_INNER
 *   θ ∈ [0, 2π) uniformly
 *   Slight pile-up at R_INNER when r_cursor is small and sigma is large.
 *
 * scatter_wedge (W) — angular wedge:
 *   r ∈ [R_INNER, r_outer] uniformly (no area correction — fills wedge)
 *   θ ∈ [θ_cursor − WEDGE_HALF, θ_cursor + WEDGE_HALF] uniformly
 *
 * scatter_ringsnap (D) — ring-snap with jitter:
 *   k ∈ {1, …, floor(r_outer / RING_SNAP_SP)} uniformly (integer)
 *   r = k × RING_SNAP_SP + jitter,  jitter ∈ [−RING_SNAP_JITTER, +JITTER]
 *   θ ∈ [0, 2π) uniformly
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 * - dirty flag: preview regenerates on cursor move, key, param change.
 *   pool_stamp (SPACE) does NOT set dirty, so repeated SPACE stamps the
 *   same pattern without regeneration (preserves expected behaviour).
 * - R_OUTER_FACTOR=0.42: on a large terminal (200×50) r_outer can exceed
 *   200px; with U mode the pool fills in one press.
 * - scatter_gauss: fabs(gauss(...)) causes pile-up at R_INNER when
 *   r_cursor is small.  This is visible as a bright spot at the inner radius.
 *
 * HOW TO VERIFY
 * ─────────────
 * Terminal 80×24 → ox=40, oy=12.
 * r_outer = 0.42 × sqrt((40×2)²+(12×4)²) = 0.42×sqrt(6400+2304) ≈ 39px
 *
 * scatter_uniform (U, N=1):
 *   r_min=R_INNER=8, r_max≈39, r_min²=64, r_max²≈1521, range≈1457
 *   rand=0.25: r=sqrt(64+0.25×1457)=sqrt(428)≈20.7; θ random  ✓
 *
 * scatter_ringsnap (D, r_outer≈39):
 *   n_rings=floor(39/20)=1 → only ring 1 (r≈20) available.
 *   All dots cluster near r=20±RING_SNAP_JITTER=±2px.
 *   On larger terminal (r_outer=80): n_rings=4; rings at 20,40,60,80px. ✓
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
#define CELL_W         2
#define CELL_H         4

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

/* Objects placed per key press */
#define N_SCATTER_DEFAULT  60
#define N_SCATTER_MIN      10
#define N_SCATTER_MAX     400
#define N_SCATTER_STEP     10

/* Radial bounds for U and W scatter */
#define R_INNER           8.0    /* pixels, minimum scatter radius */
#define R_OUTER_FACTOR    0.42   /* fraction of half-screen diagonal */

/* Radial Gaussian sigma (adjustable) */
#define SIGMA_R_DEFAULT   20.0
#define SIGMA_R_MIN        5.0
#define SIGMA_R_MAX       60.0
#define SIGMA_R_STEP       5.0

/* Angular wedge half-angle (adjustable, same variable as sigma in context) */
#define WEDGE_DEFAULT     (M_PI / 6.0)   /* 30° either side = 60° total  */
#define WEDGE_MIN         (M_PI / 18.0)  /* 10° */
#define WEDGE_MAX         (M_PI)         /* 180° */
#define WEDGE_STEP        (M_PI / 18.0)  /* 10° per step */

/* Ring-snap: spacing between snap targets (pixels) */
#define RING_SNAP_SP      20.0
#define RING_SNAP_JITTER   2.0   /* ± pixel jitter around each ring */

/*
 * BG_* constants — the natural ring/arm/seed laws of the 7 polar
 * backgrounds (mirror values used by polar_grids/01..07 and the
 * other polar placement files).  scatter_ringsnap dispatches on
 * g->mode and samples r from the matching law so the dots actually
 * land on the active grid's rings instead of an arbitrary fixed step.
 */
#define BG_RING_SP        20.0           /* mode 0: rings spacing (px)        */
#define BG_LOG_RMIN        4.0           /* mode 1: log-polar inner radius    */
#define BG_LOG_RATIO       0.25          /* mode 1: ln(RATIO) per ring        */
#define BG_ARCH_PITCH     32.0           /* mode 2: archimedean pitch (px)    */
#define BG_LOG_GROWTH    (2.0 * log(PHI) / M_PI)  /* mode 3: golden ratio    */
#define BG_LOG_R0          4.0           /* mode 3: log-spiral inner anchor   */
#define BG_SEED_SP         3.5           /* mode 4: sunflower seed spacing    */
#define BG_RUNIT          18.0           /* mode 5: equal-area R_UNIT (px)    */
#define BG_ELLIP_A         1.6           /* mode 6: elliptic x semi-axis      */
#define BG_ELLIP_B         1.0           /* mode 6: elliptic y semi-axis      */
#define BG_ELLIP_SP       20.0           /* mode 6: ring spacing in e_r       */

/* Object pool */
#define MAX_OBJ        2048
#define OBJ_GLYPH      'o'

#define PHI            1.61803398874989484820
#define GOLDEN_ANGLE  (2.0 * M_PI / (PHI * PHI))
#define N_BG_SEEDS     600

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_ANCHOR  3   /* amber — live scatter preview */
#define PAIR_HUD     4   /* status bar (yellow)  */
#define PAIR_HINT    5   /* key-hint footer (cyan) */

static const char *const SCATTER_NAMES[] = {
    "uniform", "radial-gauss", "wedge", "ring-snap",
};

static const char *const BG_NAMES[] = {
    "rings+spokes", "log-polar",  "archimedean",
    "log-spiral",   "sunflower",  "equal-area",  "elliptic",
};
#define N_BG_TYPES  7

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
    init_pair(PAIR_GRID,   fg,                               -1);
    init_pair(PAIR_ACTIVE, COLORS>=256 ? 255 : COLOR_WHITE,  -1);
    init_pair(PAIR_ANCHOR, COLORS>=256 ? 220 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int mode;
    int rows, cols;
    int cw, ch;
    int ox, oy;
    int max_ring, max_spoke;
} GridCtx;

static void ctx_init(GridCtx *g, int mode, int rows, int cols)
{
    memset(g, 0, sizeof *g);
    g->mode = mode; g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = cols / 2; g->oy = rows / 2;
    g->max_spoke = 12;
    int half_diag_px = (int)(sqrt((double)(g->ox*g->cw)*(g->ox*g->cw) +
                                   (double)(g->oy*g->ch)*(g->oy*g->ch)));
    g->max_ring = (int)(half_diag_px / 20.0);
}

static void cell_to_polar(int col, int row, int ox, int oy,
                           double *r_px, double *theta)
{
    double dx = (double)(col - ox) * CELL_W;
    double dy = (double)(row - oy) * CELL_H;
    *r_px  = sqrt(dx*dx + dy*dy);
    *theta = atan2(dy, dx);
}

static void polar_to_screen(double r, double theta, int ox, int oy,
                              int *col, int *row)
{
    *col = ox + (int)round(r * cos(theta) / CELL_W);
    *row = oy + (int)round(r * sin(theta) / CELL_H);
}

static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int row, col; char glyph; bool alive; } Obj;
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

static void pool_place(Pool *p, int row, int col,
                       int rows, int cols, char glyph)
{
    if (row < 0 || row >= rows-1 || col < 0 || col >= cols) return;
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ row, col, glyph, true };
}

static void pool_draw(const Pool *p)
{
    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        mvaddch(p->items[i].row, p->items[i].col,
                (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw_preview(const Pool *p)
{
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        mvaddch(p->items[i].row, p->items[i].col,
                (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
}

/* pool_stamp — append all preview objects into the permanent pool */
static void pool_stamp(Pool *dst, const Pool *src)
{
    for (int i = 0; i < src->count && dst->count < MAX_OBJ; i++)
        dst->items[dst->count++] = src->items[i];
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    row, col;
    double r, theta;
} Cursor;

static void cursor_sync_polar(Cursor *c, const GridCtx *g)
{
    cell_to_polar(c->col, c->row, g->ox, g->oy, &c->r, &c->theta);
}

static void cursor_reset(Cursor *c, const GridCtx *g)
{
    c->col = g->ox + (int)round(20.0 / CELL_W);
    c->row = g->oy;
    cursor_sync_polar(c, g);
}

static void cursor_move(Cursor *c, const GridCtx *g, int dr, int dc)
{
    int nr = c->row + dr, nc = c->col + dc;
    if (nr >= 0 && nr < g->rows-1) c->row = nr;
    if (nc >= 0 && nc < g->cols)   c->col = nc;
    cursor_sync_polar(c, g);
}

static void cursor_draw(const Cursor *c, const GridCtx *g)
{
    if (c->row < 0 || c->row >= g->rows-1 || c->col < 0 || c->col >= g->cols)
        return;
    attron(COLOR_PAIR(PAIR_ACTIVE) | A_REVERSE | A_BOLD);
    mvaddch(c->row, c->col, (chtype)'+');
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_REVERSE | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  mode  (polar background dispatcher)                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Mode 0 — rings + spokes (defaults from polar_grids/01_rings_spokes.c). */
static void bg_rings_spokes_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double sp = 20.0, rw = 1.6, sw = 0.10;
    const double sa = two_pi / 12.0;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            cell_to_polar(col, row, g->ox, g->oy, &r, &th);
            double rp  = fmod(r, sp);
            double tn  = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            if (rp < rw || rp > sp - rw ||
                (r > 3.0 && (sp2 < sw || sp2 > sa - sw)))
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Mode 1 — log-polar grid (defaults from polar_grids/02_log_polar.c). */
static void bg_log_polar_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double rmin = 4.0, ls = 0.25, rwu = 0.08, sw = 0.10;
    const double sa = two_pi / 12.0;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            cell_to_polar(col, row, g->ox, g->oy, &r, &th);
            bool on_r = false;
            if (r > rmin) {
                double u  = log(r / rmin) / ls;
                double fr = u - floor(u);
                on_r = fr < rwu || fr > 1.0 - rwu;
            }
            double tn  = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            if (on_r || (r > 3.0 && (sp2 < sw || sp2 > sa - sw)))
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Mode 2 — archimedean 2-arm spiral (polar_grids/03_archimedean_spiral.c). */
static void bg_archimedean_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double pitch = 32.0, sw = 0.20, rmin = 3.0;
    double a = pitch / two_pi;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            cell_to_polar(col, row, g->ox, g->oy, &r, &th);
            if (r < rmin) continue;
            double tn = fmod(th + two_pi, two_pi);
            double ph = fmod(2.0 * (tn - r / a) + 2.0 * two_pi, two_pi);
            if (ph < sw || ph > two_pi - sw)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Mode 3 — golden log-spiral, 2 arms (polar_grids/04_log_spiral.c). */
static void bg_log_spiral_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double growth = 2.0 * log(PHI) / M_PI;
    const double sw = 0.22, rmin = 4.0;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            cell_to_polar(col, row, g->ox, g->oy, &r, &th);
            if (r < rmin) continue;
            double tn = fmod(th + two_pi, two_pi);
            double tp = log(r / rmin) / growth;
            double ph = fmod(2.0 * (tn - tp) + 2.0 * two_pi, two_pi);
            if (ph < sw || ph > two_pi - sw)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Mode 4 — Vogel sunflower phyllotaxis (polar_grids/05_sunflower.c). */
static void bg_sunflower_draw(const GridCtx *g)
{
    const double sp = 3.5;
    bool *vis = calloc((size_t)(g->rows * g->cols), 1);
    if (!vis) return;

    for (int i = 0; i < N_BG_SEEDS; i++) {
        double r  = sqrt((double)i) * sp;
        double th = (double)i * GOLDEN_ANGLE;
        int    c  = g->ox + (int)round(r * cos(th) / CELL_W);
        int    rw = g->oy + (int)round(r * sin(th) / CELL_H);
        if (rw < 0 || rw >= g->rows - 1 || c < 0 || c >= g->cols) continue;
        if (vis[rw * g->cols + c]) continue;
        vis[rw * g->cols + c] = true;
        mvaddch(rw, c, (chtype)(unsigned char)'o');
    }
    free(vis);
}

/* Mode 5 — equal-area sector grid (polar_grids/06_sector.c). */
static void bg_equal_area_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double ru = 18.0, rwf = 0.06, sw = 0.10;
    const double sa = two_pi / 12.0;
    double rusq = ru * ru;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            cell_to_polar(col, row, g->ox, g->oy, &r, &th);
            if (r < 3.0) continue;
            double kf  = (r * r) / rusq;
            double fr  = kf - floor(kf);
            double tn  = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            if (fr < rwf || fr > 1.0 - rwf || sp2 < sw || sp2 > sa - sw)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Mode 6 — elliptic ring grid (polar_grids/07_elliptic.c). */
static void bg_elliptic_draw(const GridCtx *g)
{
    const double A = 1.6, B = 1.0, sp = 20.0, rwu = 0.07;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * CELL_W;
            double dy = (double)(row - g->oy) * CELL_H;
            double er = sqrt((dx/A)*(dx/A) + (dy/B)*(dy/B));
            if (er < 0.5) continue;
            double et = atan2(dy/B, dx/A);
            double fr = (er / sp) - floor(er / sp);
            if (fr < rwu || fr > 1.0 - rwu)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(et));
        }
    }
}

/*
 * draw_polar_bg — dispatch on g->mode and draw the matching polar grid.
 * Routes to one of seven bg_*_draw helpers (rings_spokes, log_polar,
 * archimedean, log_spiral, sunflower, equal_area, elliptic).  No mutation.
 */
static void draw_polar_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    switch (g->mode) {
    case 0: bg_rings_spokes_draw(g); break;
    case 1: bg_log_polar_draw   (g); break;
    case 2: bg_archimedean_draw (g); break;
    case 3: bg_log_spiral_draw  (g); break;
    case 4: bg_sunflower_draw   (g); break;
    case 5: bg_equal_area_draw  (g); break;
    case 6: bg_elliptic_draw    (g); break;
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scatter                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * gauss — Box-Muller transform: Normal(mean, sigma) from two uniforms.
 *
 * THE FORMULA:
 *   U1 = (rand+1)/(RAND_MAX+2) ∈ (0,1)   [+1 avoids log(0)]
 *   U2 = rand/(RAND_MAX+1)    ∈ [0,1)
 *   Z  = sqrt(−2 ln U1) × cos(2π U2)     ~ Normal(0,1)
 *   return mean + sigma × Z
 */
static double gauss(double mean, double sigma)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = (double)rand() / ((double)RAND_MAX + 1.0);
    double z  = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return mean + sigma * z;
}

/*
 * scatter_uniform (U) — CDF-inversion for uniform areal density.
 *
 * THE FORMULA:
 *   Area ∝ r², so sampling r² uniformly → uniform dots per unit area.
 *   r = sqrt(R_INNER² + rand × (r_outer² − R_INNER²))
 *   θ ∈ [0, 2π) uniformly
 */
static void scatter_uniform(Pool *pool, int n, double r_outer, const GridCtx *g)
{
    double r0sq = R_INNER  * R_INNER;
    double r1sq = r_outer  * r_outer;
    for (int i = 0; i < n; i++) {
        double r  = sqrt(r0sq + ((double)rand()/(double)RAND_MAX) * (r1sq - r0sq));
        double th = ((double)rand()/(double)RAND_MAX) * 2.0 * M_PI;
        int c, row;
        polar_to_screen(r, th, g->ox, g->oy, &c, &row);
        pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/*
 * scatter_gauss (G) — Gaussian ring cluster.
 *
 * THE FORMULA:
 *   r = |Normal(r_cursor, sigma)|   [fabs keeps r positive]
 *   r clamped to R_INNER minimum
 *   θ ∈ [0, 2π) uniformly
 */
static void scatter_gauss(Pool *pool, int n, double r_cursor, double sigma,
                           const GridCtx *g)
{
    for (int i = 0; i < n; i++) {
        double r  = fabs(gauss(r_cursor, sigma));
        if (r < R_INNER) r = R_INNER;
        double th = ((double)rand()/(double)RAND_MAX) * 2.0 * M_PI;
        int c, row;
        polar_to_screen(r, th, g->ox, g->oy, &c, &row);
        pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/*
 * scatter_wedge (W) — angular pie-slice filled uniformly.
 *
 * THE FORMULA:
 *   r ∈ [R_INNER, r_outer] uniformly (no area correction — fills the wedge)
 *   θ ∈ [theta_cursor − wedge_half, theta_cursor + wedge_half] uniformly
 */
static void scatter_wedge(Pool *pool, int n, double r_outer,
                           double theta_cursor, double wedge_half,
                           const GridCtx *g)
{
    for (int i = 0; i < n; i++) {
        double r  = R_INNER + ((double)rand()/(double)RAND_MAX) * (r_outer - R_INNER);
        double th = theta_cursor - wedge_half
                    + ((double)rand()/(double)RAND_MAX) * 2.0 * wedge_half;
        int c, row;
        polar_to_screen(r, th, g->ox, g->oy, &c, &row);
        pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/* One symmetric ±RING_SNAP_JITTER radial jitter sample (≈ ±2 px). */
static double ringsnap_jitter(void)
{
    return ((double)rand()/(double)RAND_MAX - 0.5) * 2.0 * RING_SNAP_JITTER;
}

/* Place a single (r, θ) sample with R_INNER clamp into the polar→screen path.
 * Used by every ringsnap_* mode helper EXCEPT mode 6 (elliptic), which has
 * its own non-circular conversion. */
static void ringsnap_place_polar(Pool *pool, double r, double th, const GridCtx *g)
{
    if (r < R_INNER) r = R_INNER;
    int c, row;
    polar_to_screen(r, th, g->ox, g->oy, &c, &row);
    pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
}

/* Mode 0 — linear rings: r = k × BG_RING_SP, θ uniform on [0, 2π). */
static void ringsnap_rings_spokes(Pool *pool, int n, double r_outer, const GridCtx *g)
{
    int n_rings = (int)(r_outer / BG_RING_SP);
    if (n_rings < 1) n_rings = 1;
    for (int i = 0; i < n; i++) {
        int    k  = 1 + rand() % n_rings;
        double r  = k * BG_RING_SP + ringsnap_jitter();
        double th = ((double)rand()/(double)RAND_MAX) * 2.0 * M_PI;
        ringsnap_place_polar(pool, r, th, g);
    }
}

/* Mode 1 — log rings: r = BG_LOG_RMIN × e^(k × BG_LOG_RATIO). */
static void ringsnap_log_polar(Pool *pool, int n, double r_outer, const GridCtx *g)
{
    int n_rings = (int)(log(r_outer / BG_LOG_RMIN) / BG_LOG_RATIO);
    if (n_rings < 1) n_rings = 1;
    for (int i = 0; i < n; i++) {
        int    k  = 1 + rand() % n_rings;
        double r  = BG_LOG_RMIN * exp(k * BG_LOG_RATIO) + ringsnap_jitter();
        double th = ((double)rand()/(double)RAND_MAX) * 2.0 * M_PI;
        ringsnap_place_polar(pool, r, th, g);
    }
}

/* Mode 2 — archimedean arm: r = (BG_ARCH_PITCH/2π)·t, θ = t. */
static void ringsnap_archimedean(Pool *pool, int n, double r_outer, const GridCtx *g)
{
    double a     = BG_ARCH_PITCH / (2.0 * M_PI);
    double t_max = r_outer / a;
    for (int i = 0; i < n; i++) {
        double t  = ((double)rand()/(double)RAND_MAX) * t_max;
        double r  = a * t + ringsnap_jitter();
        double th = t;
        ringsnap_place_polar(pool, r, th, g);
    }
}

/* Mode 3 — log-spiral arm: r = BG_LOG_R0·e^(BG_LOG_GROWTH·t), θ = t. */
static void ringsnap_log_spiral(Pool *pool, int n, double r_outer, const GridCtx *g)
{
    double t_max = log(r_outer / BG_LOG_R0) / BG_LOG_GROWTH;
    if (t_max < 0.0) t_max = 0.0;
    for (int i = 0; i < n; i++) {
        double t  = ((double)rand()/(double)RAND_MAX) * t_max;
        double r  = BG_LOG_R0 * exp(BG_LOG_GROWTH * t) + ringsnap_jitter();
        double th = t;
        ringsnap_place_polar(pool, r, th, g);
    }
}

/* Mode 4 — Vogel seeds: (r, θ) = (√i·BG_SEED_SP, i·GOLDEN_ANGLE) at random i. */
static void ringsnap_sunflower(Pool *pool, int n, double r_outer, const GridCtx *g)
{
    int max_seed = (int)((r_outer / BG_SEED_SP) * (r_outer / BG_SEED_SP));
    if (max_seed < 1)          max_seed = 1;
    if (max_seed > N_BG_SEEDS) max_seed = N_BG_SEEDS;
    for (int i = 0; i < n; i++) {
        int    seed = rand() % max_seed;
        double r    = sqrt((double)seed) * BG_SEED_SP + ringsnap_jitter();
        double th   = (double)seed * GOLDEN_ANGLE;
        ringsnap_place_polar(pool, r, th, g);
    }
}

/* Mode 5 — equal-area rings: r = √k × BG_RUNIT, θ uniform on [0, 2π). */
static void ringsnap_equal_area(Pool *pool, int n, double r_outer, const GridCtx *g)
{
    int n_rings = (int)((r_outer * r_outer) / (BG_RUNIT * BG_RUNIT));
    if (n_rings < 1) n_rings = 1;
    for (int i = 0; i < n; i++) {
        int    k  = 1 + rand() % n_rings;
        double r  = sqrt((double)k) * BG_RUNIT + ringsnap_jitter();
        double th = ((double)rand()/(double)RAND_MAX) * 2.0 * M_PI;
        ringsnap_place_polar(pool, r, th, g);
    }
}

/* Mode 6 — elliptic rings: e_r = k × BG_ELLIP_SP, then (dx, dy) = e_r·(A·cos, B·sin).
 * Bypasses ringsnap_place_polar because the screen conversion uses the ellipse axes,
 * not the standard polar→screen mapping. */
static void ringsnap_elliptic(Pool *pool, int n, double r_outer, const GridCtx *g)
{
    int n_rings = (int)(r_outer / BG_ELLIP_SP);
    if (n_rings < 1) n_rings = 1;
    for (int i = 0; i < n; i++) {
        int    k    = 1 + rand() % n_rings;
        double e_r  = k * BG_ELLIP_SP + ringsnap_jitter();
        double e_th = ((double)rand()/(double)RAND_MAX) * 2.0 * M_PI;
        int sc = g->ox + (int)round(e_r * BG_ELLIP_A * cos(e_th) / CELL_W);
        int sr = g->oy + (int)round(e_r * BG_ELLIP_B * sin(e_th) / CELL_H);
        pool_place(pool, sr, sc, g->rows, g->cols, OBJ_GLYPH);
    }
}

/*
 * scatter_ringsnap (D) — objects snap to the ACTIVE grid's natural rings
 * (or arms / seeds) plus a small radial jitter.  Dispatches on g->mode to
 * one of seven ringsnap_* helpers so the dots line up with the visible
 * background (rings_spokes, log_polar, archimedean, log_spiral,
 * sunflower, equal_area, elliptic).
 */
static void scatter_ringsnap(Pool *pool, int n, double r_outer,
                              const GridCtx *g)
{
    switch (g->mode) {
    case 0: ringsnap_rings_spokes(pool, n, r_outer, g); break;
    case 1: ringsnap_log_polar   (pool, n, r_outer, g); break;
    case 2: ringsnap_archimedean (pool, n, r_outer, g); break;
    case 3: ringsnap_log_spiral  (pool, n, r_outer, g); break;
    case 4: ringsnap_sunflower   (pool, n, r_outer, g); break;
    case 5: ringsnap_equal_area  (pool, n, r_outer, g); break;
    case 6: ringsnap_elliptic    (pool, n, r_outer, g); break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Bright bold yellow status (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Pool *pool, const Cursor *cur,
                     int scatter_type, int n, double param,
                     int theme, double fps, bool paused)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " %5.1f fps  r:%.0f  θ:%.0f°  [%s]  n:%d  p:%.1f  objs:%d  %s ",
             fps, cur->r, cur->theta*180.0/M_PI, SCATTER_NAMES[scatter_type],
             n, param, pool->count, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    mvprintw(0, 0, " %-13s ", BG_NAMES[g->mode]);
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " U/G/W/D:mode  +/-:N  [/]:param  spc:stamp  C:clear"
        "  a/e:bg  t:theme(%d)  q:quit ", theme + 1);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *pool, const Pool *preview,
                       const Cursor *cur,
                       int scatter_type, int n, double param,
                       int theme, double fps, bool paused)
{
    erase();
    draw_polar_bg(g);
    pool_draw(pool);
    pool_draw_preview(preview);
    cursor_draw(cur, g);
    hud_draw(g, pool, cur, scatter_type, n, param, theme, fps, paused);
    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10  screen                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
    srand((unsigned)time(NULL));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §11  app                                                                */
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

    int rows = LINES, cols = COLS;
    GridCtx ctx; ctx_init(&ctx, 0, rows, cols);
    Pool    pool;    pool_clear(&pool);
    Pool    preview; pool_clear(&preview);
    Cursor  cur;     cursor_reset(&cur, &ctx);

    int     scatter_type = 0;   /* 0=U 1=G 2=W 3=D */
    bool    dirty        = true;   /* regenerate preview before first draw */
    int     n_scat       = N_SCATTER_DEFAULT;
    double  sigma        = SIGMA_R_DEFAULT;
    double  wedge        = WEDGE_DEFAULT;

    bool    paused = false;
    double  fps    = TARGET_FPS;
    int64_t t0     = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
            ctx_init(&ctx, ctx.mode, rows, cols);
            cursor_sync_polar(&cur, &ctx);
            dirty = true;
        }

        double r_outer = sqrt((double)(ctx.ox*CELL_W)*(double)(ctx.ox*CELL_W) +
                              (double)(ctx.oy*CELL_H)*(double)(ctx.oy*CELL_H))
                         * R_OUTER_FACTOR;

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'P': paused = !paused; break;
        case 't': theme = (theme+1) % N_THEMES; color_init(theme); break;
        case 'a':
            ctx_init(&ctx, (ctx.mode-1+N_BG_TYPES) % N_BG_TYPES, rows, cols);
            dirty = true;   /* re-sample so any scatter visibly responds to bg */
            break;
        case 'e':
            ctx_init(&ctx, (ctx.mode+1) % N_BG_TYPES, rows, cols);
            dirty = true;
            break;
        case 'U': scatter_type = 0; dirty = true; break;
        case 'G': scatter_type = 1; dirty = true; break;
        case 'W': scatter_type = 2; dirty = true; break;
        case 'D': scatter_type = 3; dirty = true; break;
        case ' ': pool_stamp(&pool, &preview); break;
        case '+': case '=':
            if (n_scat < N_SCATTER_MAX) { n_scat += N_SCATTER_STEP; dirty = true; }
            break;
        case '-':
            if (n_scat > N_SCATTER_MIN) { n_scat -= N_SCATTER_STEP; dirty = true; }
            break;
        case '[':
            if (scatter_type == 1 && sigma > SIGMA_R_MIN) { sigma -= SIGMA_R_STEP; dirty = true; }
            if (scatter_type == 2 && wedge > WEDGE_MIN)   { wedge -= WEDGE_STEP;   dirty = true; }
            break;
        case ']':
            if (scatter_type == 1 && sigma < SIGMA_R_MAX) { sigma += SIGMA_R_STEP; dirty = true; }
            if (scatter_type == 2 && wedge < WEDGE_MAX)   { wedge += WEDGE_STEP;   dirty = true; }
            break;
        case 'C': pool_clear(&pool); break;
        case 'r': cursor_reset(&cur, &ctx); dirty = true; break;
        case KEY_UP:    cursor_move(&cur, &ctx, -1,  0); dirty = true; break;
        case KEY_DOWN:  cursor_move(&cur, &ctx, +1,  0); dirty = true; break;
        case KEY_LEFT:  cursor_move(&cur, &ctx,  0, -1); dirty = true; break;
        case KEY_RIGHT: cursor_move(&cur, &ctx,  0, +1); dirty = true; break;
        }

        /* regenerate preview only when cursor or params changed */
        if (dirty) {
            pool_clear(&preview);
            switch (scatter_type) {
            case 0: scatter_uniform  (&preview, n_scat, r_outer, &ctx); break;
            case 1: scatter_gauss    (&preview, n_scat, cur.r, sigma, &ctx); break;
            case 2: scatter_wedge    (&preview, n_scat, r_outer, cur.theta, wedge, &ctx); break;
            case 3: scatter_ringsnap (&preview, n_scat, r_outer, &ctx); break;
            }
            dirty = false;
        }

        double param = (scatter_type == 1) ? sigma
                     : (scatter_type == 2) ? wedge * 180.0 / M_PI
                     : 0.0;

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9/(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        if (!paused)
            scene_draw(&ctx, &pool, &preview, &cur,
                       scatter_type, n_scat, param, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
