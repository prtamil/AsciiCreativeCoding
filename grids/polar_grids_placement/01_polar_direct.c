/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_polar_direct.c — move a cursor over a round (polar) grid and drop markers.
 *
 * You steer a cursor across one of seven round backgrounds (rings, spirals,
 * a sunflower, an ellipse, ...).  The arrow keys behave differently on each
 * grid so the cursor always moves the way that grid naturally wants — hop
 * between rings, walk along a spiral, jump seed to seed.  Space drops or
 * picks up a marker; 'a'/'e' flip to the previous/next background.
 *
 * Sister files: grids/rect_grids_placement/01_direct.c is the square-grid
 *               version of this same idea; grids/polar_grids/01_rings_spokes.c
 *               (through 07_elliptic.c) draw each background on its own.
 * References:   polar coordinates — en.wikipedia.org/wiki/Polar_coordinate_system
 *               object pool pattern — gameprogrammingpatterns.com/object-pool.html
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

#define TARGET_FPS     30
#define CELL_W          2     /* a terminal cell is roughly 2 wide... */
#define CELL_H          4     /* ...and 4 tall in pixels; correcting this keeps circles round */

/* How fast the on-screen FPS number settles: small = smooth but laggy. */
#define FPS_EWMA_ALPHA  0.05

/* Don't let the cursor sit right on the centre — angle is meaningless there. */
#define R_POLAR_MIN     4.0

/* Markers the user drops */
#define MAX_OBJ       256
#define OBJ_GLYPH     'o'

/* Golden ratio, used by the sunflower and the golden spiral. */
#define PHI            1.61803398874989484820
#define GOLDEN_ANGLE  (2.0 * M_PI / (PHI * PHI))

/* How many seeds the sunflower background draws. */
#define N_BG_SEEDS    600

/* How far one arrow press moves on each background.  Every grid has its
 * own natural step — ring spacing here, spiral pitch there, seed counts
 * for the sunflower, and so on. */
#define BG_RING_SP     20.0            /* rings+spokes: ring spacing (px)    */
#define BG_SPOKE_ANG   (M_PI / 6.0)   /* rings/log/sector/elliptic: 30°     */
#define BG_LOG_RATIO    0.25           /* log-polar: ln(RATIO) per ring      */
#define BG_ARCH_PITCH  32.0            /* archimedean: pitch per turn (px)   */
#define BG_ARCH_ANG    (M_PI / 4.0)   /* archimedean: along-curve step (45°)*/
#define BG_LOG_ANG     (M_PI / 4.0)   /* log-spiral: along-curve step (45°) */
#define BG_SEED_SP      3.5            /* sunflower: seed spacing (px)       */
#define BG_SEED_STEP    1              /* sunflower: seeds per LR press      */
#define BG_SEED_JUMP   13              /* sunflower: seeds per UD press      */
#define BG_RUNIT       18.0            /* equal-area: R_UNIT (px)            */
#define BG_ELLIP_A      1.6            /* elliptic: x semi-axis              */
#define BG_ELLIP_B      1.0            /* elliptic: y semi-axis              */
#define BG_ELLIP_SP    20.0            /* elliptic: ring spacing in e_r      */

static const char *const BG_MOVE_HINTS[] = {
    "ud:ring  lr:spoke",
    "ud:log-ring  lr:spoke",
    "lr:along-spiral  ud:turn",
    "lr:along-spiral  ud:arm",
    "lr:seed+1  ud:seed+13",
    "ud:eq-ring  lr:spoke",
    "ud:ellipse  lr:ell-ang",
};

/* Colour slots: background grid, markers/cursor, anchor, status bar, key hints. */
#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_ANCHOR  3
#define PAIR_HUD     4
#define PAIR_HINT    5

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
 * GridCtx — everything we need to know about the round grid right now: which
 * background is showing and where its centre sits on screen.  Rebuilt whenever
 * the user switches grids or the terminal is resized.
 *
 *   mode      — which background is active, 0..6 (see BG_NAMES).
 *   rows,cols — current terminal size; remembered so a resize can re-centre.
 *   cw,ch     — pixels per cell across and down; used to keep circles round.
 *   ox,oy     — the centre of the screen, the point everything is measured from.
 *   max_ring,
 *   max_spoke — rough "how many rings/spokes fit" counts, only shown in the HUD.
 *               The cursor itself is kept on-screen by clamping, not by these.
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
    /* roughly how many rings reach from the centre to the far corner */
    int half_diag_px = (int)(sqrt((double)(g->ox*g->cw)*(g->ox*g->cw) +
                                   (double)(g->oy*g->ch)*(g->oy*g->ch)));
    g->max_ring = (int)(half_diag_px / BG_RING_SP);
}

/*
 * cell_to_polar — given a cell on screen, find how far it is from the centre
 * and which direction it points (its distance r and angle theta).
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
 * polar_to_screen — the reverse: take a distance and angle and find which
 * cell on screen lands there.
 */
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

/* Obj — one dropped marker: where it sits, what it looks like, and whether
 * the slot is in use (alive==false means the slot is free to reuse). */
typedef struct { int row, col; char glyph; bool alive; } Obj;

/* Pool — all the markers, held in one fixed array so we never allocate while
 * running.  count is how many of the slots are currently filled. */
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

static int pool_find(const Pool *p, int row, int col)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].alive && p->items[i].row == row && p->items[i].col == col)
            return i;
    return -1;
}

static void pool_place(Pool *p, int row, int col, char glyph)
{
    if (pool_find(p, row, col) >= 0) return;
    if (p->count >= MAX_OBJ) return;
    p->items[p->count++] = (Obj){ row, col, glyph, true };
}

static void pool_remove(Pool *p, int row, int col)
{
    int i = pool_find(p, row, col);
    if (i < 0) return;
    p->items[i] = p->items[--p->count];
}

static void pool_toggle(Pool *p, int row, int col, char glyph)
{
    if (pool_find(p, row, col) >= 0) pool_remove(p, row, col);
    else                              pool_place(p, row, col, glyph);
}

static void pool_clear(Pool *p) { p->count = 0; }

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

/* ── §6 cursor ── */

/*
 * Cursor — where the user is pointing.  Its real position is the round one:
 * a distance from centre and an angle.  The cell (row, col) is just that
 * position rounded to the nearest character, recomputed after every move.
 *
 *   r, theta — distance from centre (pixels) and direction (radians).
 *   row, col — the screen cell the cursor lands on.
 *   seed_idx — only the sunflower uses this: which numbered seed we're on.
 */
typedef struct {
    double r, theta;
    int    row, col;
    int    seed_idx;
} Cursor;

/* After r or theta changes, work out the screen cell and keep it on-screen. */
static void cursor_sync(Cursor *c, const GridCtx *g)
{
    polar_to_screen(c->r, c->theta, g->ox, g->oy, &c->col, &c->row);
    if (c->row < 0)          c->row = 0;
    if (c->row >= g->rows-1) c->row = g->rows-2;
    if (c->col < 0)          c->col = 0;
    if (c->col >= g->cols)   c->col = g->cols-1;
}

/*
 * cursor_apply_seed — for the sunflower, snap the cursor exactly onto seed
 * number seed_idx.  Seeds spiral out by the golden angle, each a little
 * farther from the centre, the way real sunflower seeds pack.
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

/* Tidy up after a move: keep the angle in one full turn and don't let the
 * cursor reach the dead centre.  Shared by the ring/spiral modes; the
 * sunflower and ellipse clean up their own way. */
static void cursor_normalise_polar(Cursor *c, const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
    c->theta = fmod(c->theta + 4.0 * two_pi, two_pi);
    cursor_sync(c, g);
}

/* Rings + spokes: up/down hop to the next ring, left/right turn to the next spoke. */
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

/* Log-polar: rings get wider the farther out you go, so up/down scale the
 * distance by a fixed ratio instead of adding a fixed amount; left/right snap to spokes. */
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

/* Archimedean spiral: left/right slide along the curve (turn a little, step out a
 * matching little, so the cursor stays on the line); up/down jump a whole turn. */
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

/* Golden spiral: left/right slide along the curve (each turn multiplies the
 * distance); up/down jump straight across to the other of the two arms. */
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

/* Sunflower: move by seed number — left/right step one seed, up/down jump 13
 * (a Fibonacci number, so you hop along one of the visible spiral arms). */
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

/* Equal-area rings: every ring covers the same area, so outer ones sit closer
 * together.  Up/down snap to the next such ring; left/right snap to spokes. */
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

/* Ellipse: pretend the screen is stretched into a circle, move on that circle,
 * then squash the answer back so it lands on the real ellipse. */
static void cursor_move_elliptic(Cursor *c, const GridCtx *g, int key)
{
    double dx   = (double)(c->col - g->ox) * CELL_W;
    double dy   = (double)(c->row - g->oy) * CELL_H;
    double e_r  = sqrt((dx/BG_ELLIP_A)*(dx/BG_ELLIP_A) +
                       (dy/BG_ELLIP_B)*(dy/BG_ELLIP_B));
    double e_th = atan2(dy/BG_ELLIP_B, dx/BG_ELLIP_A);

    switch (key) {
    case KEY_UP: {
        /* snap inward to the next ellipse ring */
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

    /* squash the circle answer back onto the ellipse and onto a screen cell */
    c->col = g->ox + (int)round(e_r * BG_ELLIP_A * cos(e_th) / CELL_W);
    c->row = g->oy + (int)round(e_r * BG_ELLIP_B * sin(e_th) / CELL_H);
    if (c->row < 0)          c->row = 0;
    if (c->row >= g->rows-1) c->row = g->rows-2;
    if (c->col < 0)          c->col = 0;
    if (c->col >= g->cols)   c->col = g->cols-1;
    cell_to_polar(c->col, c->row, g->ox, g->oy, &c->r, &c->theta);
    if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
}

/* Hand the arrow key to whichever background is showing, so the cursor moves
 * the way that grid wants. */
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

/* Each bg_*_draw paints one background by walking every cell, turning it into
 * distance-and-angle, and lighting it up if it lands on a grid line.  The
 * line spacings here mirror the standalone demos in grids/polar_grids/. */

/* Rings + spokes. */
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

/* Log-polar: rings spaced so they look evenly spread under zoom. */
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

/* Archimedean spiral, two arms — even spacing between turns. */
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

/* Golden spiral, two arms — each turn farther out than the last. */
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

/* Sunflower: seeds placed by the golden angle, the way real seed heads pack. */
static void bg_sunflower_draw(const GridCtx *g)
{
    const double sp = BG_SEED_SP;
    /* one flag per cell so two seeds don't fight over the same spot; freed below */
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

/* Equal-area rings + spokes — every ring band covers the same area. */
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
            bool   on_s = sp2 < sw || sp2 > sa - sw;
            if (on_r || on_s)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
        }
    }
}

/* Ellipse rings — circles stretched wider than they are tall. */
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

/* Draw whichever background is currently selected. */
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

/* ── §8 scene ── */

/* The on-screen readouts: grid name and stats up top, key hints along the bottom. */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     int theme, double fps, bool paused)
{
    char buf[96];
    double deg = cur->theta * 180.0 / M_PI;
    snprintf(buf, sizeof buf, " %5.1f fps  r:%.0f  θ:%.0f°  objs:%d  %s ",
             fps, cur->r, deg, p->count, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    mvprintw(0, 0, " %-13s ", BG_NAMES[g->mode]);
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " %s  spc:place  a/e:bg  C:clear  r:reset  t:theme(%d)  q:quit ",
        BG_MOVE_HINTS[g->mode], theme + 1);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       int theme, double fps, bool paused)
{
    erase();
    draw_polar_bg(g);
    pool_draw(p);
    cursor_draw(cur, g);
    hud_draw(g, p, cur, theme, fps, paused);
    wnoutrefresh(stdscr); doupdate();
}

/* ── §9 screen ── */

static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

/* ── §10 app ── */

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
    GridCtx ctx;  ctx_init(&ctx, 0, rows, cols);
    Cursor  cur = { .r = 20.0, .theta = 0.0, .seed_idx = 0 };
    cursor_sync(&cur, &ctx);
    Pool    pool; pool_clear(&pool);

    bool    paused  = false;
    double  fps     = TARGET_FPS;
    int64_t t0      = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
            ctx_init(&ctx, ctx.mode, rows, cols);
            if (ctx.mode == 4) cursor_apply_seed(&cur, &ctx);
            else               cursor_sync(&cur, &ctx);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'p': paused = !paused; break;
        case 't': theme = (theme + 1) % N_THEMES; color_init(theme); break;
        case ' ': pool_toggle(&pool, cur.row, cur.col, OBJ_GLYPH); break;
        case 'C': pool_clear(&pool); break;
        case 'r': cursor_reset(&cur, &ctx); break;
        case 'a':
        case 'e': {
            int m = (ch == 'a') ? (ctx.mode - 1 + N_BG_TYPES) % N_BG_TYPES
                                : (ctx.mode + 1) % N_BG_TYPES;
            ctx_init(&ctx, m, rows, cols);
            /* switching to the sunflower: pick the seed nearest where we already are */
            if (ctx.mode == 4) {
                cur.seed_idx = (int)round((cur.r / BG_SEED_SP) * (cur.r / BG_SEED_SP));
                cursor_apply_seed(&cur, &ctx);
            }
            break;
        }
        case KEY_UP: case KEY_DOWN: case KEY_LEFT: case KEY_RIGHT:
            cursor_move(&cur, &ctx, ch);
            break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9/(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        if (!paused)
            scene_draw(&ctx, &pool, &cur, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
