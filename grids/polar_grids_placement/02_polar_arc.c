/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_polar_arc.c — draw arcs and spokes on a polar (rings-and-rays) grid.
 *
 * You pick two points with 'p', then stamp the four shapes that bound a
 * polar cell: an arc (part of a ring), a spoke (part of a ray), a whole
 * ring, or a whole ray.  The HUD shows each point as (r, angle) so you can
 * watch the polar geometry while you draw.
 *
 * Sister files: 01_polar_direct.c (same cursor model, simpler),
 *               grids/rect_grids_placement/03_path.c (the square-grid version).
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

/* Smooths the FPS number so it doesn't jitter every frame. */
#define FPS_EWMA_ALPHA  0.05

/* How far apart to drop dots when drawing a spoke. Smaller than a cell so
 * the line has no gaps. */
#define SPOKE_PX_STEP   1.0
/* Smallest angle step for an arc. Without a floor, a huge ring's step would
 * round to zero and the loop would never end. */
#define ARC_STEP_MIN    0.005   /* radians */
/* Rings and rays start a little out from the centre to dodge the origin. */
#define R_OPS_MIN       4.0

/* How many dots we can hold. Bigger than file 01's pool because one ring or
 * arc can stamp hundreds at once. */
#define MAX_OBJ       4096
#define OBJ_GLYPH     '*'

#define PHI           1.61803398874989484820
#define GOLDEN_ANGLE  (2.0 * M_PI / (PHI * PHI))
#define N_BG_SEEDS    600

/* Don't let the cursor sit right on the centre, where angle is undefined. */
#define R_POLAR_MIN     4.0

/*
 * How far each arrow press moves the cursor, one set per background style
 * (same values as 01_polar_direct). The point: arrows step from one grid
 * line to the next, so a captured anchor lands on a visible ring or spoke
 * instead of somewhere in between.
 */
#define BG_RING_SP      20.0           /* rings+spokes: ring spacing (px)    */
#define BG_SPOKE_ANG    (M_PI / 6.0)   /* rings/log/sector/elliptic: 30°     */
#define BG_LOG_RATIO    0.25           /* log-polar: ln(RATIO) per ring      */
#define BG_ARCH_PITCH   32.0           /* archimedean: pitch per turn (px)   */
#define BG_ARCH_ANG     (M_PI / 4.0)   /* archimedean: along-curve step (45°)*/
#define BG_LOG_ANG      (M_PI / 4.0)   /* log-spiral: along-curve step (45°) */
#define BG_SEED_SP      3.5            /* sunflower: seed spacing (px)       */
#define BG_SEED_STEP    1              /* sunflower: seeds per LR press      */
#define BG_SEED_JUMP   13              /* sunflower: seeds per UD press      */
#define BG_RUNIT       18.0            /* equal-area: R_UNIT (px)            */
#define BG_ELLIP_A      1.6            /* elliptic: x semi-axis              */
#define BG_ELLIP_B      1.0            /* elliptic: y semi-axis              */
#define BG_ELLIP_SP    20.0            /* elliptic: ring spacing in e_r      */

/* Color pairs */
#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_ANCHOR  3   /* the A and B point markers */
#define PAIR_HUD     4   /* status bar (yellow)  */
#define PAIR_HINT    5   /* key-hint footer (cyan) */

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
 * GridCtx — everything the polar math needs to know about the screen.
 * It holds the terminal size, the centre point that polar (0,0) maps to,
 * the shape of one character cell, and which background style is showing.
 *
 *   mode              which background grid is active (0..6, see BG_NAMES)
 *   rows, cols        terminal size in characters
 *   cw, ch            one cell's width/height in pixels (a cell is taller
 *                     than wide, so circles need this to not look squashed)
 *   ox, oy            the centre cell — where radius 0 lives
 *   max_ring,         rough counts of how many rings/spokes fit on screen,
 *   max_spoke         shown in the HUD only
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

static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* ── §5 pool ── */

/* One stamped dot: where it sits and what character it shows. */
typedef struct { int row, col; char glyph; bool alive; } Obj;
/* The whole collection of dots drawn so far. Fixed size — no growing. */
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

/* Adds a dot if it's on screen. If the pool is full, the dot is dropped
 * (better than overflowing). */
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

/* ── §6 cursor ── */

/*
 * Cursor — where you're pointing, kept in polar terms.
 *
 * The real position is (r, theta): how far from the centre and at what
 * angle. The (row, col) cell is just that converted to a screen spot, so
 * we don't recompute it every time we draw. Arrow keys move the cursor
 * along whatever grid is showing, so a captured point lands on a real
 * grid line. Same layout as 01_polar_direct.c.
 *
 *   r, theta    distance (pixels) and angle (radians) from centre
 *   row, col    the screen cell that (r, theta) maps to
 *   seed_idx    which sunflower seed we're on — only the sunflower
 *               background uses this
 */
typedef struct {
    double r, theta;
    int    row, col;
    int    seed_idx;
} Cursor;

/* After r or theta changes, refresh the screen cell and keep it on screen. */
static void cursor_sync(Cursor *c, const GridCtx *g)
{
    polar_to_screen(c->r, c->theta, g->ox, g->oy, &c->col, &c->row);
    if (c->row < 0)          c->row = 0;
    if (c->row >= g->rows-1) c->row = g->rows-2;
    if (c->col < 0)          c->col = 0;
    if (c->col >= g->cols)   c->col = g->cols-1;
}

/*
 * Places the cursor on sunflower seed number seed_idx. Each seed sits a
 * little farther out and turned by the golden angle from the last — the
 * pattern real sunflowers use to pack seeds evenly (Vogel's model).
 */
static void cursor_apply_seed(Cursor *c, const GridCtx *g)
{
    if (c->seed_idx < 0)           c->seed_idx = 0;
    if (c->seed_idx >= N_BG_SEEDS) c->seed_idx = N_BG_SEEDS - 1;
    c->r     = sqrt((double)c->seed_idx) * BG_SEED_SP;
    c->theta = fmod((double)c->seed_idx * GOLDEN_ANGLE + 4.0*M_PI, 2.0*M_PI);
    if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
    cursor_sync(c, g);
}

static void cursor_reset(Cursor *c, const GridCtx *g)
{
    c->r = 20.0; c->theta = 0.0; c->seed_idx = 0;
    if (g->mode == 4) cursor_apply_seed(c, g);
    else              cursor_sync(c, g);
}

/* Tidy up after a move: fold the angle back into one full turn and stop r
 * from going below the minimum, then refresh the screen cell. Most modes
 * end with this; sunflower and elliptic do their own clamping instead. */
static void cursor_normalise_polar(Cursor *c, const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
    c->theta = fmod(c->theta + 4.0 * two_pi, two_pi);
    cursor_sync(c, g);
}

/* Rings+spokes: up/down hop to the next ring, left/right to the next spoke. */
static void cursor_move_rings_spokes(Cursor *c, const GridCtx *g, int key)
{
    switch (key) {
    case KEY_UP:    c->r -= BG_RING_SP;       break;
    case KEY_DOWN:  c->r += BG_RING_SP;       break;
    case KEY_LEFT:  c->theta -= BG_SPOKE_ANG; break;
    case KEY_RIGHT: c->theta += BG_SPOKE_ANG; break;
    default: return;
    }
    cursor_normalise_polar(c, g);
}

/* Log-polar: up/down scale the radius by a fixed factor (rings grow as you
 * go out), left/right hop to the next spoke. */
static void cursor_move_log_polar(Cursor *c, const GridCtx *g, int key)
{
    switch (key) {
    case KEY_UP:    c->r /= exp(BG_LOG_RATIO); break;
    case KEY_DOWN:  c->r *= exp(BG_LOG_RATIO); break;
    case KEY_LEFT:  c->theta -= BG_SPOKE_ANG;  break;
    case KEY_RIGHT: c->theta += BG_SPOKE_ANG;  break;
    default: return;
    }
    cursor_normalise_polar(c, g);
}

/* Archimedean spiral: left/right walk along the spiral itself, up/down jump
 * out or in by one full turn. */
static void cursor_move_archimedean(Cursor *c, const GridCtx *g, int key)
{
    double a = BG_ARCH_PITCH / (2.0 * M_PI);
    switch (key) {
    case KEY_LEFT:
        c->theta -= BG_ARCH_ANG;
        c->r     -= a * BG_ARCH_ANG;
        break;
    case KEY_RIGHT:
        c->theta += BG_ARCH_ANG;
        c->r     += a * BG_ARCH_ANG;
        break;
    case KEY_UP:    c->r -= BG_ARCH_PITCH; break;
    case KEY_DOWN:  c->r += BG_ARCH_PITCH; break;
    default: return;
    }
    cursor_normalise_polar(c, g);
}

/* Log-spiral: left/right walk along the curve, up/down flip to the other
 * arm (there are two, half a turn apart). */
static void cursor_move_log_spiral(Cursor *c, const GridCtx *g, int key)
{
    double scale = exp(2.0 * log(PHI) / M_PI * BG_LOG_ANG);
    switch (key) {
    case KEY_LEFT:
        c->theta -= BG_LOG_ANG;
        c->r     /= scale;
        break;
    case KEY_RIGHT:
        c->theta += BG_LOG_ANG;
        c->r     *= scale;
        break;
    case KEY_UP:   c->theta -= M_PI; break;  /* the two arms sit half a turn apart */
    case KEY_DOWN: c->theta += M_PI; break;
    default: return;
    }
    cursor_normalise_polar(c, g);
}

/* Sunflower: arrows step through seed numbers (left/right by one, up/down by
 * a bigger jump); the cursor snaps to that exact seed. */
static void cursor_move_sunflower(Cursor *c, const GridCtx *g, int key)
{
    switch (key) {
    case KEY_UP:    c->seed_idx -= BG_SEED_JUMP; break;
    case KEY_DOWN:  c->seed_idx += BG_SEED_JUMP; break;
    case KEY_LEFT:  c->seed_idx -= BG_SEED_STEP; break;
    case KEY_RIGHT: c->seed_idx += BG_SEED_STEP; break;
    default: return;
    }
    cursor_apply_seed(c, g);
}

/* Equal-area: up/down hop between rings spaced so each band holds the same
 * area (they bunch up as you go out), left/right hop between spokes. */
static void cursor_move_equal_area(Cursor *c, const GridCtx *g, int key)
{
    double kf = (c->r / BG_RUNIT) * (c->r / BG_RUNIT);
    switch (key) {
    case KEY_UP: {
        double k = ceil(kf) - 1.0;
        if (k < 1.0) k = 1.0;
        c->r = sqrt(k) * BG_RUNIT;
        break;
    }
    case KEY_DOWN: {
        double k = floor(kf) + 1.0;
        c->r = sqrt(k) * BG_RUNIT;
        break;
    }
    case KEY_LEFT:  c->theta -= BG_SPOKE_ANG; break;
    case KEY_RIGHT: c->theta += BG_SPOKE_ANG; break;
    default: return;
    }
    cursor_normalise_polar(c, g);
}

/* Elliptic: the rings are stretched ovals, so we first read the cursor in
 * the oval's own coordinates, hop to the next oval ring or spoke, then
 * convert that back to a screen cell. */
static void cursor_move_elliptic(Cursor *c, const GridCtx *g, int key)
{
    double dx   = (double)(c->col - g->ox) * CELL_W;
    double dy   = (double)(c->row - g->oy) * CELL_H;
    double e_r  = sqrt((dx/BG_ELLIP_A)*(dx/BG_ELLIP_A) +
                       (dy/BG_ELLIP_B)*(dy/BG_ELLIP_B));
    double e_th = atan2(dy/BG_ELLIP_B, dx/BG_ELLIP_A);

    switch (key) {
    case KEY_UP: {
        double k = ceil(e_r / BG_ELLIP_SP) - 1.0;
        e_r = k * BG_ELLIP_SP;
        if (e_r < 1.0) e_r = 1.0;
        break;
    }
    case KEY_DOWN: {
        double k = floor(e_r / BG_ELLIP_SP) + 1.0;
        e_r = k * BG_ELLIP_SP;
        break;
    }
    case KEY_LEFT:  e_th -= BG_SPOKE_ANG; break;
    case KEY_RIGHT: e_th += BG_SPOKE_ANG; break;
    default: return;
    }

    c->col = g->ox + (int)round(e_r * BG_ELLIP_A * cos(e_th) / CELL_W);
    c->row = g->oy + (int)round(e_r * BG_ELLIP_B * sin(e_th) / CELL_H);
    if (c->row < 0)          c->row = 0;
    if (c->row >= g->rows-1) c->row = g->rows-2;
    if (c->col < 0)          c->col = 0;
    if (c->col >= g->cols)   c->col = g->cols-1;
    cell_to_polar(c->col, c->row, g->ox, g->oy, &c->r, &c->theta);
    if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
}

/*
 * Hands the arrow key to whichever mover matches the current background, so
 * arrows always follow the shape of the grid you're looking at.
 */
static void cursor_move(Cursor *c, const GridCtx *g, int key)
{
    switch (g->mode) {
    case 0: cursor_move_rings_spokes(c, g, key); break;
    case 1: cursor_move_log_polar   (c, g, key); break;
    case 2: cursor_move_archimedean (c, g, key); break;
    case 3: cursor_move_log_spiral  (c, g, key); break;
    case 4: cursor_move_sunflower   (c, g, key); break;
    case 5: cursor_move_equal_area  (c, g, key); break;
    case 6: cursor_move_elliptic    (c, g, key); break;
    }
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

/* These seven functions just paint the faint background grid you draw on.
 * Each matches one full demo elsewhere; the file name says which. */

/* Rings + spokes — see polar_grids/01_rings_spokes.c. */
static void bg_rings_spokes_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double sp = 20.0, rw = 1.6, sw = 0.10;
    const double sa = two_pi / 12.0;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            cell_to_polar(col, row, g->ox, g->oy, &r, &th);
            double rp     = fmod(r, sp);
            bool   on_r   = rp < rw || rp > sp - rw;
            double tn     = fmod(th + two_pi, two_pi);
            double sp2    = fmod(tn, sa);
            bool   on_s   = r > 3.0 && (sp2 < sw || sp2 > sa - sw);
            if (on_r || on_s)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Log-polar grid — see polar_grids/02_log_polar.c. */
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
            bool   on_s = r > 3.0 && (sp2 < sw || sp2 > sa - sw);
            if (on_r || on_s)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Archimedean two-arm spiral — see polar_grids/03_archimedean_spiral.c. */
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
            double tn  = fmod(th + two_pi, two_pi);
            double raw = 2.0 * (tn - r / a);
            double ph  = fmod(raw + 2.0 * two_pi, two_pi);
            if (ph < sw || ph > two_pi - sw)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Golden log-spiral, two arms — see polar_grids/04_log_spiral.c. */
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
            double tn  = fmod(th + two_pi, two_pi);
            double tp  = log(r / rmin) / growth;
            double raw = 2.0 * (tn - tp);
            double ph  = fmod(raw + 2.0 * two_pi, two_pi);
            if (ph < sw || ph > two_pi - sw)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Sunflower seed pattern — see polar_grids/05_sunflower.c. */
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

/* Equal-area sector grid — see polar_grids/06_sector.c. */
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
            bool   on_r = fr < rwf || fr > 1.0 - rwf;
            double tn  = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            if (on_r || sp2 < sw || sp2 > sa - sw)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Elliptic (oval) ring grid — see polar_grids/07_elliptic.c. */
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
            double u  = er / sp, fr = u - floor(u);
            if (fr < rwu || fr > 1.0 - rwu)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(et));
        }
    }
}

/* Draws whichever background grid the current mode selects. */
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

/* ── §8 anchor ── */

/* How many points we've picked so far: none, one (A), or two (A and B). */
typedef enum { IDLE = 0, ONE = 1, TWO = 2 } AnchorState;

/*
 * AnchorCtx — the two points you select and remember between them.
 * Press 'p' to capture the cursor as point A, then again for point B; the
 * four shapes are drawn from these. We keep each point both in polar form
 * (r, angle) for the math and in screen form (row, col) for drawing markers.
 *
 *   state             how far through picking we are (none / A / A and B)
 *   r_a, theta_a      point A in polar form
 *   row_a, col_a      point A as a screen cell
 *   r_b, theta_b      point B in polar form
 *   row_b, col_b      point B as a screen cell
 */
typedef struct {
    AnchorState state;
    double r_a, theta_a;
    int    row_a, col_a;
    double r_b, theta_b;
    int    row_b, col_b;
} AnchorCtx;

/*
 * Draws an arc: the slice of point A's ring that runs from A's angle to B's.
 * It walks the angle in small steps, dropping one dot per step. The step is
 * sized so big rings (more cells around) get more dots and the line stays
 * solid; a floor on the step stops a giant ring from looping forever. The
 * walk always goes the same direction, so picking A past B gives the long
 * way round, not the short.
 */
static void arc_draw(Pool *pool, const AnchorCtx *ac, const GridCtx *g)
{
    double t0 = fmod(ac->theta_a + 2.0*M_PI, 2.0*M_PI);
    double t1 = fmod(ac->theta_b + 2.0*M_PI, 2.0*M_PI);
    if (t1 < t0) t1 += 2.0*M_PI;                     /* keep sweeping one direction */
    double step = CELL_W / (ac->r_a + 1.0);
    if (step < ARC_STEP_MIN) step = ARC_STEP_MIN;
    for (double t = t0; t <= t1 + step*0.5; t += step) {
        int c, r;
        polar_to_screen(ac->r_a, t, g->ox, g->oy, &c, &r);
        pool_place(pool, r, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/*
 * Draws a spoke: a straight line out from the centre at point A's angle,
 * running between the two points' distances. It steps the distance outward
 * in small pixel steps so the line has no gaps.
 */
static void spoke_draw(Pool *pool, const AnchorCtx *ac, const GridCtx *g)
{
    double r0 = fmin(ac->r_a, ac->r_b);
    double r1 = fmax(ac->r_a, ac->r_b);
    for (double r = r0; r <= r1; r += SPOKE_PX_STEP) {
        int c, row;
        polar_to_screen(r, ac->theta_a, g->ox, g->oy, &c, &row);
        pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/* Draws a full ring at point A's distance — an arc that goes all the way
 * around. Same stepping idea as arc_draw. */
static void ring_draw(Pool *pool, const AnchorCtx *ac, const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    double step = CELL_W / (ac->r_a + 1.0);
    if (step < ARC_STEP_MIN) step = ARC_STEP_MIN;
    for (double t = 0.0; t < two_pi; t += step) {
        int c, r;
        polar_to_screen(ac->r_a, t, g->ox, g->oy, &c, &r);
        pool_place(pool, r, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/*
 * Draws a full ray at point A's angle, from near the centre out past the
 * far corner of the screen, so it always reaches the edge. The far corner
 * is just the longest distance any visible cell can be.
 */
static void radial_draw(Pool *pool, const AnchorCtx *ac, const GridCtx *g)
{
    double r_max = sqrt(
        (double)(g->ox * CELL_W) * (double)(g->ox * CELL_W) +
        (double)(g->oy * CELL_H) * (double)(g->oy * CELL_H));
    for (double r = R_OPS_MIN; r <= r_max; r += SPOKE_PX_STEP) {
        int c, row;
        polar_to_screen(r, ac->theta_a, g->ox, g->oy, &c, &row);
        pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/*
 * One press of 'p' steps the picker forward: first press grabs point A,
 * second grabs point B, third clears back to the start so you can pick again.
 */
static void anchor_advance(AnchorCtx *ac, const Cursor *cur)
{
    switch (ac->state) {
    case IDLE:
        ac->r_a   = cur->r;   ac->theta_a = cur->theta;
        ac->row_a = cur->row; ac->col_a   = cur->col;
        ac->state = ONE;
        break;
    case ONE:
        ac->r_b   = cur->r;   ac->theta_b = cur->theta;
        ac->row_b = cur->row; ac->col_b   = cur->col;
        ac->state = TWO;
        break;
    case TWO:
        ac->state = IDLE;
        break;
    }
}

/* Marks the picked points: '@' for A, '#' for B. */
static void anchors_draw(const AnchorCtx *ac)
{
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    if (ac->state >= ONE)
        mvaddch(ac->row_a, ac->col_a, (chtype)'@');
    if (ac->state >= TWO)
        mvaddch(ac->row_b, ac->col_b, (chtype)'#');
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
}

/* ── §9 scene ── */

static const char *const STATE_NAMES[] = {"IDLE", "A-set", "B-set"};

/* Top-right status line, plus the key hints along the bottom. The hint line
 * changes with the picker state so it only offers keys that do something. */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     const AnchorCtx *ac, int theme, double fps, bool paused)
{
    char buf[96];
    double deg = cur->theta * 180.0 / M_PI;
    snprintf(buf, sizeof buf, " %5.1f fps  r:%.0f  θ:%.0f°  objs:%d  %s ",
             fps, cur->r, deg, p->count, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    mvprintw(0, 0, " %-13s %s ", BG_NAMES[g->mode], STATE_NAMES[ac->state]);
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);

    const char *ops = (ac->state == TWO)
        ? " a:arc  s:spoke  o:ring  x:radial  p:reset  C:clear"
        : " p:set-anchor  o:ring  x:radial  a/e:bg  q:quit";
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0, "%s  t:theme(%d) ", ops, theme + 1);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *pool, const Cursor *cur,
                       const AnchorCtx *ac, int theme, double fps, bool paused)
{
    erase();
    draw_polar_bg(g);
    pool_draw(pool);
    anchors_draw(ac);
    cursor_draw(cur, g);
    hud_draw(g, pool, cur, ac, theme, fps, paused);
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
    GridCtx   ctx; ctx_init(&ctx, 0, rows, cols);
    Pool      pool; pool_clear(&pool);
    AnchorCtx ac = { .state = IDLE };
    Cursor    cur; cursor_reset(&cur, &ctx);

    bool    paused = false;
    double  fps    = TARGET_FPS;
    int64_t t0     = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
            ctx_init(&ctx, ctx.mode, rows, cols);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'P': paused = !paused; break;   /* capital P so it won't clash with the 'p' picker */
        case 't': theme = (theme+1) % N_THEMES; color_init(theme); break;
        case 'a':
            if (ac.state == TWO) {
                arc_draw(&pool, &ac, &ctx);
            } else {
                ctx_init(&ctx, (ctx.mode - 1 + N_BG_TYPES) % N_BG_TYPES,
                         rows, cols);
                /* switching to sunflower: jump to the seed nearest the current radius */
                if (ctx.mode == 4) {
                    cur.seed_idx = (int)round((cur.r / BG_SEED_SP) *
                                              (cur.r / BG_SEED_SP));
                    cursor_apply_seed(&cur, &ctx);
                }
            }
            break;
        case 'e':
            ctx_init(&ctx, (ctx.mode + 1) % N_BG_TYPES, rows, cols);
            if (ctx.mode == 4) {
                cur.seed_idx = (int)round((cur.r / BG_SEED_SP) *
                                          (cur.r / BG_SEED_SP));
                cursor_apply_seed(&cur, &ctx);
            }
            break;
        case 'p': anchor_advance(&ac, &cur); break;
        case 's':
            if (ac.state == TWO) spoke_draw(&pool, &ac, &ctx);
            break;
        case 'o':
            if (ac.state >= ONE) ring_draw(&pool, &ac, &ctx);
            break;
        case 'x':
            if (ac.state >= ONE) radial_draw(&pool, &ac, &ctx);
            break;
        case 'C': pool_clear(&pool); break;
        case 'r': ac.state = IDLE; break;
        case KEY_UP:
        case KEY_DOWN:
        case KEY_LEFT:
        case KEY_RIGHT: cursor_move(&cur, &ctx, ch); break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9/(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        if (!paused)
            scene_draw(&ctx, &pool, &cur, &ac, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
