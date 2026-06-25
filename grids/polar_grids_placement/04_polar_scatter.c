/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_polar_scatter.c — scatter dots on a polar grid four different ways.
 *
 * Move the cursor, pick a scatter style with U/G/W/D, and a live preview
 * follows along in amber. Press space to stamp the preview into a permanent
 * pool so you can build up a picture. The four styles only differ in how
 * each dot's distance-from-centre and angle are chosen at random.
 *
 * Sister files: 03_polar_spiral.c (placing dots along a spiral path) and
 * grids/rect_grids_placement/04_scatter.c (the same idea on a square grid).
 *
 * The "uniform area" trick (picking r from sqrt of a uniform value so the
 * middle doesn't get crowded) and the bell-curve generator come from:
 *   en.wikipedia.org/wiki/Disk_sampling
 *   en.wikipedia.org/wiki/Box-Muller_transform
 */

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

/* ── §1 config ── */

#define TARGET_FPS    30
#define CELL_W         2
#define CELL_H         4

/* How much each new frame nudges the displayed FPS number. Small value =
 * smooth, slow-to-react readout instead of a jittery one. */
#define FPS_EWMA_ALPHA  0.05

/* How many dots a scatter drops at once, and the range +/- can dial it to. */
#define N_SCATTER_DEFAULT  60
#define N_SCATTER_MIN      10
#define N_SCATTER_MAX     400
#define N_SCATTER_STEP     10

/* Inner and outer edge of the scatter ring for the U and W styles.
 * R_INNER keeps dots off the very centre; R_OUTER_FACTOR sets the outer
 * edge as a fraction of the distance from centre to corner. */
#define R_INNER           8.0    /* sub-pixels, closest a dot may land */
#define R_OUTER_FACTOR    0.42   /* fraction of half-screen diagonal */

/* Width of the bell curve for the G (Gaussian) style: bigger = dots spread
 * further from the cursor's ring. Adjustable with [ and ]. */
#define SIGMA_R_DEFAULT   20.0
#define SIGMA_R_MIN        5.0
#define SIGMA_R_MAX       60.0
#define SIGMA_R_STEP       5.0

/* Half-width of the pie slice for the W (wedge) style. Default fills 30 deg
 * to each side of the cursor, so a 60 deg slice. Adjustable with [ and ]. */
#define WEDGE_DEFAULT     (M_PI / 6.0)   /* 30 deg either side = 60 deg total */
#define WEDGE_MIN         (M_PI / 18.0)  /* 10 deg */
#define WEDGE_MAX         (M_PI)         /* 180 deg */
#define WEDGE_STEP        (M_PI / 18.0)  /* 10 deg per step */

/* Ring-snap style: gap between the rings dots snap to, and how far a dot may
 * wobble off its ring so the result doesn't look mechanically perfect. */
#define RING_SNAP_SP      20.0
#define RING_SNAP_JITTER   2.0   /* +/- sub-pixel wobble around each ring */

/*
 * Each polar background has its own rule for where its rings (or spiral arms,
 * or seeds) sit. These numbers copy those rules from polar_grids/01..07. The
 * ring-snap style reads them so its dots land exactly on whichever background
 * is showing, instead of on some unrelated fixed spacing.
 */
#define BG_RING_SP        20.0           /* rings+spokes: gap between rings   */
#define BG_LOG_RMIN        4.0           /* log-polar: innermost ring         */
#define BG_LOG_RATIO       0.25          /* log-polar: growth step per ring   */
#define BG_ARCH_PITCH     32.0           /* archimedean: gap between arm coils */
#define BG_LOG_GROWTH    (2.0 * log(PHI) / M_PI)  /* log-spiral: golden growth */
#define BG_LOG_R0          4.0           /* log-spiral: where the arm starts  */
#define BG_SEED_SP         3.5           /* sunflower: spacing between seeds   */
#define BG_RUNIT          18.0           /* equal-area: ring step             */
#define BG_ELLIP_A         1.6           /* elliptic: how stretched, sideways  */
#define BG_ELLIP_B         1.0           /* elliptic: how stretched, vertical  */
#define BG_ELLIP_SP       20.0           /* elliptic: gap between rings        */

/* The fixed-size store of placed dots, and the character each one draws as. */
#define MAX_OBJ        2048
#define OBJ_GLYPH      'o'

#define PHI            1.61803398874989484820
#define GOLDEN_ANGLE  (2.0 * M_PI / (PHI * PHI))
#define N_BG_SEEDS     600

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_ANCHOR  3   /* amber — the live preview */
#define PAIR_HUD     4   /* status line (yellow) */
#define PAIR_HINT    5   /* key hints (cyan) */

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

/* ── §2 clock ── */

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

/* ── §3 color ── */

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

/* ── §4 gridctx ── */

/*
 * Everything the drawing code needs to know about the current screen and
 * which polar background is showing. Recomputed whenever the terminal resizes
 * or the user switches backgrounds, then passed (read-only) to every helper.
 *   mode             which of the 7 backgrounds is active (0..6)
 *   rows, cols       terminal size in character cells
 *   cw, ch           cell width/height in sub-pixels — a character cell is
 *                    taller than it is wide, so we measure distances in these
 *                    units to keep circles looking round
 *   ox, oy           centre of the grid, in cells (where r = 0 lives)
 *   max_ring         how many rings fit before running off screen
 *   max_spoke        how many evenly-spaced spokes the background draws
 */
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

/* Pick a slash that roughly points along the angle, so grid lines look like
 * they curve instead of being a wall of identical characters. */
static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* ── §5 pool ── */

/*
 * One placed dot: where it sits on screen and what it looks like.
 *   row, col   its character cell on screen
 *   glyph      the character to draw (always OBJ_GLYPH here)
 *   alive      kept for future use; right now every dot stays alive
 */
typedef struct { int row, col; char glyph; bool alive; } Obj;

/*
 * A fixed-size bucket of dots. We use two of these: one permanent pool the
 * user is building up, and one short-lived preview that gets wiped and
 * refilled every time the cursor or settings change. No dynamic memory —
 * count just walks up to MAX_OBJ and stops.
 */
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

/* Copy the preview's dots into the permanent pool — what space:stamp does.
 * Stops early if the permanent pool is full. */
static void pool_stamp(Pool *dst, const Pool *src)
{
    for (int i = 0; i < src->count && dst->count < MAX_OBJ; i++)
        dst->items[dst->count++] = src->items[i];
}

/* ── §6 cursor ── */

/*
 * The crosshair the user steers. It lives on the screen grid (row, col), but
 * the scatter styles think in polar terms, so we also keep its distance from
 * centre and its angle. The two views are kept in sync by cursor_sync_polar.
 *   row, col   position in character cells
 *   r          distance from the grid centre, in sub-pixels
 *   theta      angle around the centre, in radians
 */
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

/* ── §7 mode ── */

/* Each bg_*_draw paints one style of polar background. They're tuned to match
 * the standalone demos in polar_grids/, so the scatter dots sit on the same
 * lines those demos draw. They only read the grid; they never change it. */

/* The plain rings-and-spokes background, like a dartboard. */
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

/* Rings that grow further apart the further out you go (log spacing). */
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

/* A two-armed spiral whose coils stay evenly spaced, like a coiled rope. */
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

/* A two-armed spiral that widens as it turns, the shape of a nautilus shell. */
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

/* Seeds laid out like a sunflower head, each turned by the golden angle. */
static void bg_sunflower_draw(const GridCtx *g)
{
    const double sp = 3.5;
    /* One flag per screen cell so we don't draw two seeds on the same spot.
     * Freed before we return, so nothing leaks. */
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

/* Rings spaced so every ring band covers the same area (rings bunch up as
 * you go out, since outer bands are naturally wider). */
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

/* Oval rings instead of round ones — squashed by the A and B stretch factors. */
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

/* Draw whichever background the user picked. */
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

/* ── §8 scatter ── */

/*
 * Draw a random number from a bell curve centred on mean, with sigma setting
 * its width — most results land near mean, far-out ones get rare. C only gives
 * us flat random numbers, so this is the standard recipe (the Box-Muller
 * transform) for turning two flat ones into a bell-curve one. The +1 nudge on
 * the first random value keeps it away from zero, since the math takes its log.
 */
static double gauss(double mean, double sigma)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = (double)rand() / ((double)RAND_MAX + 1.0);
    double z  = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return mean + sigma * z;
}

/*
 * U style: spread dots evenly across the whole disc. The catch is that a ring
 * far from the centre is bigger than one near it, so picking the distance
 * straight-up would crowd the middle. Taking the square root of a flat random
 * value cancels that out, giving the same dot density everywhere.
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
 * G style: cluster dots around the ring the cursor is on. Each dot's distance
 * is a bell-curve pick centred on the cursor's radius, so most land near that
 * ring and fewer stray away. The angle is fully random, so it's a fuzzy ring.
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
 * W style: fill a pie slice aimed at the cursor's angle. The distance is just
 * a flat random pick (no even-density trick here — we want the slice filled),
 * and the angle is random but kept within wedge_half either side of the cursor.
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

/* A small random wobble, plus or minus RING_SNAP_JITTER, so snapped dots
 * don't sit on a perfectly sharp ring. */
static double ringsnap_jitter(void)
{
    return ((double)rand()/(double)RAND_MAX - 0.5) * 2.0 * RING_SNAP_JITTER;
}

/* Drop one dot at the given distance and angle, nudged outward if it landed
 * too close to the centre. Every ring-snap style uses this except elliptic,
 * which needs its own oval placement. */
static void ringsnap_place_polar(Pool *pool, double r, double th, const GridCtx *g)
{
    if (r < R_INNER) r = R_INNER;
    int c, row;
    polar_to_screen(r, th, g->ox, g->oy, &c, &row);
    pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
}

/* Snap each dot onto one of the evenly-spaced rings, at a random angle. */
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

/* Snap onto the log-polar rings, which sit further apart the further out. */
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

/* Scatter dots along the archimedean spiral arm (distance grows with angle). */
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

/* Scatter dots along the log-spiral arm (the coils widen as they turn). */
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

/* Land each dot on a random sunflower seed position. */
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

/* Snap onto the equal-area rings, at a random angle. */
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

/* Snap onto the oval elliptic rings. Does its own placement instead of the
 * shared helper, because the screen position has to be squashed by the ellipse
 * stretch factors rather than mapped as a plain circle. */
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
 * D style: make the dots cling to whatever background is showing. It picks the
 * matching ring-snap helper for the active background, so the dots land on its
 * rings (or arms, or seeds) with just a little wobble, instead of floating free.
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

/* ── §9 scene ── */

/* Draw the on-screen readouts: stats top-right, background name top-left,
 * key hints along the bottom. */
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

/* ── §10 screen ── */

static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
    srand((unsigned)time(NULL));
}

/* ── §11 app ── */

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
    bool    dirty        = true;   /* true = preview needs rebuilding; set so
                                      the first frame already shows something */
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
            dirty = true;   /* rebuild so ring-snap follows the new background */
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

        /* Rebuild the preview only when something changed — note that stamping
         * (space) deliberately leaves dirty alone, so you can stamp the same
         * pattern several times without it re-randomising. */
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
