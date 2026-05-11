/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_polar_arc.c — two-anchor arc and spoke drawing on a polar grid
 *
 * DEMO: Press 'p' to set anchor A, move, press 'p' again to set anchor B.
 *       Then press 'a' to draw an arc along the ring at r_A from θ_A to θ_B;
 *       's' to draw a radial spoke at θ_A from r_A to r_B; 'o' to stamp a
 *       full ring at r_A; 'x' for a full radial line at θ_A.  The HUD shows
 *       the anchor coordinates in (r, θ) so you can see the polar geometry.
 *
 * Study alongside: 01_polar_direct.c (cursor model),
 *                  grids/rect_grids_placement/03_path.c (analogous rect version)
 *
 * Section map:
 *   §1 config   — pool size, arc/spoke step, background names
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: grid, active, anchor, HUD (yellow), hint (cyan)
 *   §4 gridctx  — GridCtx (mode, origin), cell_to_polar, polar_to_screen
 *   §5 pool     — Pool: place, clear, draw
 *   §6 cursor   — Cursor (r, theta, row, col, seed_idx) + grid-snap arrow move
 *   §7 mode     — draw_polar_bg: 7 inline polar background types in one switch
 *   §8 anchor   — AnchorCtx, 3-state FSM, arc/spoke/ring/radial draw
 *   §9 scene    — hud_draw + scene_draw
 *   §10 screen  — ncurses init / cleanup
 *   §11 app     — signals, resize, main loop
 *
 * Keys:  q/ESC quit   P pause   t theme   a/e prev/next background
 *        p  advance state (IDLE→ONE→TWO→IDLE)
 *        In ONE or TWO: o full-ring   x full-spoke
 *        In TWO only:   a arc   s spoke
 *        C clear all   r reset anchors
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids_placement/02_polar_arc.c \
 *       -o 02_polar_arc -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Polar path rasterisation.  Given two polar anchors
 *                  A = (r_A, θ_A) and B = (r_B, θ_B) the four operations
 *                  are:
 *
 *                  Arc   — iterate θ from θ_A to θ_B at constant r = r_A,
 *                           placing one object per terminal cell:
 *                           step = CELL_W / r_A radians (≈ 1 cell per step).
 *
 *                  Spoke — iterate r from r_A to r_B at constant θ = θ_A,
 *                           step = 1.0 pixel (finer than CELL_W/CELL_H).
 *
 *                  Ring  — full arc at r_A over [0, 2π).
 *
 *                  Radial — spoke at θ_A from r=4 to max visible radius.
 *
 *                  All operations convert each (r, θ) sample to a terminal
 *                  cell via polar_to_screen() and append to the object pool.
 *
 * Data-structure : Pool with pool_place (no-duplicate check for draw ops;
 *                  duplicates are harmless since the pool is capped).
 *                  State machine: IDLE → ONE → TWO, advanced by 'p'.
 *
 * Math           : Arc length in terminal columns ≈ r × Δθ / CELL_W.
 *                  Step size CELL_W/r ensures ≤ 1 object per column at
 *                  any radius.  Spoke length in cells ≈ Δr / CELL_H.
 *
 * References     :
 *   Polar coordinate system — en.wikipedia.org/wiki/Polar_coordinate_system
 *   Rectangular analogue — grids/rect_grids_placement/03_path.c
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A polar "cell" is bounded by two arcs (at r_A and r_B) and two spokes
 * (at θ_A and θ_B).  This file lets you draw those four boundary types
 * individually using a two-anchor state machine.  Each operation converts
 * a parametric polar curve into a sequence of object-pool entries.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Anchor A = (r_A, θ_A) defines a specific ring and a specific ray.
 * Anchor B = (r_B, θ_B) defines a second ring and second ray.
 *
 *   Arc:    portion of ring r_A between rays θ_A and θ_B.
 *   Spoke:  segment of ray θ_A between rings r_A and r_B.
 *   Ring:   full ring at r_A (θ from 0 to 2π).
 *   Radial: full ray at θ_A from origin to screen edge.
 *
 * The state machine (IDLE→ONE→TWO) ensures both anchors are set before
 * drawing arc or spoke.  Ring and radial only need anchor ONE.
 *
 * DRAWING METHOD
 * ──────────────
 * 1. Move cursor, press 'p' → sets anchor A (state: IDLE→ONE).
 * 2. Move cursor, press 'p' → sets anchor B (state: ONE→TWO).
 * 3. Press 'a'=arc, 's'=spoke, 'o'=ring, 'x'=radial to draw.
 *    Each operation calls pool_place() for every sampled (r, θ).
 * 4. Press 'p' again → resets to IDLE.
 *
 * KEY FORMULAS
 * ────────────
 * arc_draw — angular step for one terminal column:
 *   step = CELL_W / (r_A + 1)          [radians per column at radius r_A]
 *   clamped to ARC_STEP_MIN=0.005 rad (prevents infinite loop at large r)
 *   Direction: always forward — if t1 < t0 add 2π (counterclockwise wrap)
 *   Arc cells ≈ Δθ × r_A / CELL_W
 *   Example: Δθ=π/2, r_A=20 → ≈ 20×1.571/2 ≈ 15 objects
 *
 * spoke_draw — pixel-step radial segment:
 *   step = SPOKE_PX_STEP = 1.0 px (finer than CELL_H=4; no gaps)
 *   r from min(r_A, r_B) to max(r_A, r_B) at constant θ = θ_A
 *   Spoke cells ≈ |r_B − r_A| / CELL_H
 *
 * ring_draw — full circle at r_A:
 *   Same step as arc_draw; t from 0 to 2π
 *   Objects ≈ 2π × r_A / CELL_W  (= π × r_A for CELL_W=2)
 *
 * radial_draw — full spoke from R_OPS_MIN to screen corner:
 *   r_max = sqrt((ox × CELL_W)² + (oy × CELL_H)²)   [screen diagonal in px]
 *   r from R_OPS_MIN to r_max by SPOKE_PX_STEP
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 * - Arc always goes counterclockwise (t1 += 2π when t1 < t0).  If θ_A and
 *   θ_B are close and θ_A > θ_B you get the long arc (> π), not the short.
 * - ARC_STEP_MIN=0.005: without this the step rounds to 0 for r_A > ~400,
 *   causing an infinite loop in arc_draw.
 * - MAX_OBJ=4096.  A full ring at r=200 needs ≈628 objects; at r=1300 the
 *   ring would overflow the pool.  pool_place silently caps at MAX_OBJ.
 *
 * HOW TO VERIFY
 * ─────────────
 * Terminal 80×24 → ox=40, oy=12.
 *
 * arc_draw (A at col=50,row=12 → r_A=20, θ_A=0; B at col=40,row=7):
 *   θ_B = atan2((7−12)×4, 0) = atan2(−20, 0) = −π/2
 *   t0=0, t1=fmod(−π/2+2π,2π)=3π/2≈4.712
 *   step=2/(20+1)≈0.095 rad → ≈50 objects covering 270°.
 *
 * ring_draw at r_A=20:
 *   step≈0.095; loop: 2π/0.095≈66 objects.
 *   At θ=0: col=50, row=12 ✓.  At θ=π/2: col=40, row=17 ✓.
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

/* Spoke rasterisation step in pixels (finer than cell size, no gaps) */
#define SPOKE_PX_STEP   1.0
/* Minimum arc angular step — prevents infinite loops at very large r */
#define ARC_STEP_MIN    0.005   /* radians */
/* Minimum radius for radial/ring operations */
#define R_OPS_MIN       4.0

/* Object pool — larger than 01 because arc/ring can produce many objects */
#define MAX_OBJ       4096
#define OBJ_GLYPH     '*'

#define PHI           1.61803398874989484820
#define GOLDEN_ANGLE  (2.0 * M_PI / (PHI * PHI))
#define N_BG_SEEDS    600

/* Minimum cursor radius — avoids origin singularity */
#define R_POLAR_MIN     4.0

/*
 * BG_* constants — per-mode step sizes for cursor_move (mirror 01_polar_direct).
 * Arrows snap the cursor to the natural intersections of each background grid,
 * so 'p'-anchor + 'o'-ring / 'x'-radial land on visible ring/spoke lines
 * instead of arbitrary screen positions.
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
#define PAIR_ANCHOR  3   /* anchor A and B markers */
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

/*
 * GridCtx — minimal polar context: terminal extent, cell aspect, origin,
 * which background mode is active, and informational max_ring/max_spoke
 * caps used by the HUD.
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int row, col; char glyph; bool alive; } Obj;
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

/* pool_place — append if in bounds; cap silently at MAX_OBJ */
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — POLAR-space cursor with cached screen coordinates.
 *
 * (r, theta) is the address; (row, col) is computed from it via cursor_sync().
 * Arrow keys snap to grid intersections via cursor_move's per-mode dispatch,
 * so when 'p' captures an anchor and 'o'/'x' draw a ring/radial at that
 * radius/angle, the result lines up with the visible polar grid.
 *
 * Same shape as 01_polar_direct.c.
 */
typedef struct {
    double r, theta;  /* polar position: pixels and radians        */
    int    row, col;  /* terminal cell derived from (r, theta)     */
    int    seed_idx;  /* Vogel seed index (sunflower bg only)      */
} Cursor;

/* Recompute (row,col) after polar coords change; clamp to screen. */
static void cursor_sync(Cursor *c, const GridCtx *g)
{
    polar_to_screen(c->r, c->theta, g->ox, g->oy, &c->col, &c->row);
    if (c->row < 0)          c->row = 0;
    if (c->row >= g->rows-1) c->row = g->rows-2;
    if (c->col < 0)          c->col = 0;
    if (c->col >= g->cols)   c->col = g->cols-1;
}

/*
 * cursor_apply_seed — update (r, θ, row, col) from seed_idx for sunflower bg.
 * Vogel model: r = sqrt(i) × spacing, θ = i × GOLDEN_ANGLE.
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

/* Wrap θ into [0, 2π) and clamp r to R_POLAR_MIN — shared tail of every
 * (r, θ)-mutating cursor mode (modes 0, 1, 2, 3, 5).  Mode 4 (sunflower)
 * and mode 6 (elliptic) use bespoke clamps that touch (row, col) directly. */
static void cursor_normalise_polar(Cursor *c, const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
    c->theta = fmod(c->theta + 4.0 * two_pi, two_pi);
    cursor_sync(c, g);
}

/* Mode 0 — rings+spokes: UP/DOWN snap to next ring, LEFT/RIGHT to next spoke. */
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

/* Mode 1 — log-polar: UP/DOWN multiply r by RATIO=e^0.25, LR snaps to spokes. */
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

/* Mode 2 — archimedean: LR walks ALONG the curve (Δr = a×Δθ); UD jumps one turn. */
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

/* Mode 3 — log-spiral: LR walks the curve, UD flips between the two golden arms. */
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
    case KEY_UP:   c->theta -= M_PI; break;  /* 2-arm: arms are π apart */
    case KEY_DOWN: c->theta += M_PI; break;
    default: return;
    }
    cursor_normalise_polar(c, g);
}

/* Mode 4 — sunflower: step seed index; cursor_apply_seed snaps to the exact seed. */
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

/* Mode 5 — equal-area: UP/DOWN snap to sqrt(k)×R_UNIT rings, LR = spokes. */
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

/* Mode 6 — elliptic: move in the (e_r, ell_θ) frame, then convert back to (col,row). */
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
 * cursor_move — grid-aware movement: dispatches per mode so arrows
 * always follow the natural geometry of the active background.
 * Routes to one of seven cursor_move_* helpers (one per BG_NAMES entry).
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
            bool   on_s = r > 3.0 && (sp2 < sw || sp2 > sa - sw);
            if (on_r || on_s)
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
            double tn  = fmod(th + two_pi, two_pi);
            double raw = 2.0 * (tn - r / a);
            double ph  = fmod(raw + 2.0 * two_pi, two_pi);
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
            double tn  = fmod(th + two_pi, two_pi);
            double tp  = log(r / rmin) / growth;
            double raw = 2.0 * (tn - tp);
            double ph  = fmod(raw + 2.0 * two_pi, two_pi);
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
            bool   on_r = fr < rwf || fr > 1.0 - rwf;
            double tn  = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            if (on_r || sp2 < sw || sp2 > sa - sw)
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
            double u  = er / sp, fr = u - floor(u);
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
/* §8  anchor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef enum { IDLE = 0, ONE = 1, TWO = 2 } AnchorState;

typedef struct {
    AnchorState state;
    double r_a, theta_a;   /* anchor A */
    int    row_a, col_a;
    double r_b, theta_b;   /* anchor B */
    int    row_b, col_b;
} AnchorCtx;

/*
 * arc_draw — rasterise arc of ring r_a from theta_a to theta_b.
 *
 * THE FORMULA:
 *   step = CELL_W / (r_a + 1)   [rad/col: ≈1 object per terminal column]
 *   step clamped to ARC_STEP_MIN=0.005 (prevents infinite loop at large r)
 *   Always counterclockwise: if t1 < t0 add 2π before iterating
 */
static void arc_draw(Pool *pool, const AnchorCtx *ac, const GridCtx *g)
{
    double t0 = fmod(ac->theta_a + 2.0*M_PI, 2.0*M_PI);
    double t1 = fmod(ac->theta_b + 2.0*M_PI, 2.0*M_PI);
    if (t1 < t0) t1 += 2.0*M_PI;                     /* always go forward */
    double step = CELL_W / (ac->r_a + 1.0);
    if (step < ARC_STEP_MIN) step = ARC_STEP_MIN;
    for (double t = t0; t <= t1 + step*0.5; t += step) {
        int c, r;
        polar_to_screen(ac->r_a, t, g->ox, g->oy, &c, &r);
        pool_place(pool, r, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/*
 * spoke_draw — rasterise radial segment at theta_a from r_a to r_b.
 *
 * THE FORMULA:
 *   step = SPOKE_PX_STEP = 1.0 px (finer than CELL_H=4 to avoid gaps)
 *   r from min(r_a, r_b) to max(r_a, r_b) at constant θ = theta_a
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

/* ring_draw — full 2π arc at r_a (same step formula as arc_draw) */
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
 * radial_draw — full spoke at theta_a from R_OPS_MIN to screen corner.
 *
 * THE FORMULA:
 *   r_max = sqrt((ox×CELL_W)² + (oy×CELL_H)²)   [screen diagonal in px]
 *   r from R_OPS_MIN to r_max by SPOKE_PX_STEP=1.0 px
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
 * anchor_advance — drive the IDLE → ONE → TWO → IDLE state machine.
 * IDLE: capture cursor as anchor A.  ONE: capture cursor as anchor B.
 * TWO: clear back to IDLE so the user can restart the two-anchor selection.
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

/* Draw anchor markers on screen */
static void anchors_draw(const AnchorCtx *ac)
{
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    if (ac->state >= ONE)
        mvaddch(ac->row_a, ac->col_a, (chtype)'@');
    if (ac->state >= TWO)
        mvaddch(ac->row_b, ac->col_b, (chtype)'#');
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const char *const STATE_NAMES[] = {"IDLE", "A-set", "B-set"};

/* Bright bold yellow status (top-right) + bold cyan key hints (bottom). */
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
        case 'P': paused = !paused; break;   /* capital P = pause to avoid clash */
        case 't': theme = (theme+1) % N_THEMES; color_init(theme); break;
        case 'a':
            if (ac.state == TWO) {
                arc_draw(&pool, &ac, &ctx);
            } else {
                ctx_init(&ctx, (ctx.mode - 1 + N_BG_TYPES) % N_BG_TYPES,
                         rows, cols);
                /* entering sunflower: snap seed_idx to nearest seed at current r */
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
