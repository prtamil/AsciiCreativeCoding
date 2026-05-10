/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_polar_direct.c — direct cursor placement on a polar grid background
 *
 * DEMO: A cursor navigates over one of 7 polar backgrounds, with arrow-key
 *       movement adapted to each grid type.  Rings/spokes grids snap to ring
 *       and spoke intersections; spirals walk along the curve; sunflower jumps
 *       between Vogel seeds by index; elliptic moves in (e_r, ell_θ) space.
 *       Space places or removes an object.  'a'/'e' cycles all 7 backgrounds.
 *
 * Study alongside: grids/rect_grids_placement/01_direct.c (rectangular version),
 *                  grids/polar_grids/01_rings_spokes.c (background drawing)
 *
 * Section map:
 *   §1 config   — pool size, per-grid step constants, background names/hints
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: grid, active object, anchor, HUD (yellow), hint (cyan)
 *   §4 gridctx  — GridCtx (mode + per-mode params), cell_to_polar, polar_to_screen
 *   §5 pool     — Pool: place, remove, toggle, find, clear, draw
 *   §6 cursor   — Cursor (r, theta, row, col, seed_idx), grid-aware move/reset
 *   §7 mode     — draw_polar_bg: 7 inline polar background types in one switch
 *   §8 scene    — hud_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *   §10 app     — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme
 *        arrows  move cursor (grid-aware: ring/spoke/spiral/seed/elliptic)
 *        space  place/remove   a/e  prev/next background   C clear   r reset
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids_placement/01_polar_direct.c \
 *       -o 01_polar_direct -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Grid-aware polar cursor.  Arrow-key movement is
 *                  dispatched per background type:
 *
 *                  Rings+spokes (0): UP/DOWN snap to next ring (±RING_SP);
 *                  LEFT/RIGHT snap to next spoke (±30°).
 *
 *                  Log-polar (1): UP/DOWN multiply/divide r by RATIO=e^0.25;
 *                  LEFT/RIGHT snap to next spoke (±30°).
 *
 *                  Archimedean (2): LEFT/RIGHT walk along the curve (Δθ +
 *                  matching Δr = a×Δθ); UP/DOWN jump one full pitch.
 *
 *                  Log-spiral (3): LEFT/RIGHT walk along the curve (Δθ +
 *                  r scaled by e^(growth×Δθ)); UP/DOWN flip between the
 *                  two golden arms (θ ± π).
 *
 *                  Sunflower (4): LEFT/RIGHT step seed index ±1; UP/DOWN
 *                  step by ±13 (one Fibonacci family of spirals).
 *
 *                  Equal-area (5): UP/DOWN snap to next equal-area ring
 *                  (r = sqrt(k) × R_UNIT); LEFT/RIGHT snap to spokes.
 *
 *                  Elliptic (6): UP/DOWN step e_r by ±BG_ELLIP_SP;
 *                  LEFT/RIGHT step ell_θ by ±30° in elliptic frame.
 *
 * Data-structure : Pool — fixed array of (row, col, glyph, alive).
 *                  pool_toggle: finds a match and removes (swap-last, O(1)),
 *                  or appends if not found.  Identical shape to the pool in
 *                  grids/rect_grids_placement/01_direct.c.
 *
 * GridContext    : A single GridCtx struct carries the mode plus origin/range
 *                  parameters.  Switching grids re-initialises GridCtx but
 *                  the pool is NOT cleared — placed objects persist across
 *                  modes (they may appear off-screen until you switch back).
 *
 * Rendering      : Background in PAIR_GRID (theme color), objects in
 *                  PAIR_ACTIVE (bright white), cursor in A_REVERSE.
 *
 * Performance    : Background O(rows × cols) per frame for sweep-based
 *                  types (0–3, 5–6); O(N_BG_SEEDS) for sunflower (type 4).
 *
 * References     :
 *   Polar coordinate system — en.wikipedia.org/wiki/Polar_coordinate_system
 *   Object pool pattern — gameprogrammingpatterns.com/object-pool.html
 *   Rectangular analogue — grids/rect_grids_placement/01_direct.c
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Seven background grids share the same polar coordinate system, but each
 * defines its own natural cursor movement.  On rings+spokes, UP/DOWN hop
 * between rings (r ± BG_RING_SP) and LEFT/RIGHT rotate between spokes
 * (θ ± 30°).  On the archimedean spiral, LEFT/RIGHT walk ALONG the curve
 * (both r and θ change so the cursor stays on the line).  On sunflower,
 * movement jumps between indexed seeds via the Vogel formula.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Two coordinate layers are in play: screen (row, col) and polar (r, θ in
 * pixels).  The cursor stores (r, θ); cursor_sync() converts these to
 * (row, col) after every move.  Exception: sunflower stores seed_idx i and
 * cursor_apply_seed() derives (r, θ, row, col) from i via the Vogel formula.
 *
 * Elliptic mode (mode=6) adds a third layer: (e_r, ell_θ) in the
 * ellipse's own stretched frame.  Arrows modify (e_r, ell_θ); the result
 * converts back via dx=e_r×A×cos(ell_θ), dy=e_r×B×sin(ell_θ).
 *
 * DRAWING METHOD
 * ──────────────
 * 1. draw_polar_bg() sweeps every cell (row,col), converts to polar,
 *    tests against the grid equation, draws if matched.
 * 2. pool_draw() draws placed objects from the pool.
 * 3. cursor_draw() draws '+' with A_REVERSE at (cursor.row, cursor.col).
 * 4. hud_draw() draws yellow status (top-right) + cyan key hints (bottom).
 *
 * KEY FORMULAS
 * ────────────
 * cell_to_polar (§4):
 *   dx = (col − ox) × CELL_W        [pixels; aspect correction]
 *   dy = (row − oy) × CELL_H
 *   r  = sqrt(dx² + dy²)
 *   θ  = atan2(dy, dx)               [−π to +π]
 *
 * polar_to_screen (§4):
 *   col = ox + round(r × cos(θ) / CELL_W)
 *   row = oy + round(r × sin(θ) / CELL_H)
 *
 * Archimedean cursor walk (mode=2):
 *   a = BG_ARCH_PITCH / (2π)         [pitch parameter, px/rad]
 *   Δr = a × Δθ                       [r advances proportionally to θ]
 *   → cursor stays on spiral after each LEFT/RIGHT press
 *
 * Log-spiral cursor walk (mode=3):
 *   growth = 2 × ln(φ) / π ≈ 0.3065
 *   new r = r × e^(growth × Δθ)      [radius scales exponentially]
 *   UP/DOWN: θ ± π switches between the two arms (arms are π apart)
 *
 * Equal-area ring snap (mode=5):
 *   k_float = (r / R_UNIT)²           [fractional ring index]
 *   UP:   k = ceil(k_float) − 1;   new r = sqrt(k) × R_UNIT
 *   DOWN: k = floor(k_float) + 1;  new r = sqrt(k) × R_UNIT
 *
 * Elliptic cursor (mode=6):
 *   e_r    = sqrt((dx/A)² + (dy/B)²)  [elliptic radius]
 *   ell_θ  = atan2(dy/B, dx/A)        [angle in normalised frame]
 *   convert back: col = ox + round(e_r × A × cos(ell_θ) / CELL_W)
 *                 row = oy + round(e_r × B × sin(ell_θ) / CELL_H)
 *
 * Vogel sunflower (cursor_apply_seed, mode=4):
 *   r = sqrt(i) × BG_SEED_SP          [equal-area seed density]
 *   θ = i × GOLDEN_ANGLE              [golden angle ≈ 137.508°]
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 * - r < R_POLAR_MIN=4 is clamped everywhere.  Without this, any radial
 *   move that overshoots the origin collapses r to 0 and the angle is
 *   lost permanently (atan2(0,0) is undefined).
 * - Log-spiral arm flip: UP/DOWN shifts θ by ±π.  With N=2 arms they
 *   are exactly π apart, so UP always lands on the "other" arm.
 * - Elliptic mode updates cursor.r / cursor.theta via cell_to_polar after
 *   each move; these values are used only if you switch mode from elliptic
 *   to another mode without resetting.
 *
 * HOW TO VERIFY
 * ─────────────
 * Terminal 80×24 → ox=40, oy=12.
 *
 * polar_to_screen:
 *   r=20, θ=0:   col=40+round(20/2)=50, row=12        ✓
 *   r=20, θ=π/2: col=40, row=12+round(20/4)=17        ✓
 *
 * Archimedean step (PITCH=32, start r=32, θ=0):
 *   a=32/(2π)≈5.09; press RIGHT (Δθ=π/4):
 *   Δr=5.09×0.785≈4.0 → new r=36, new θ=45°           ✓
 *
 * Equal-area snap (R_UNIT=18, start r=20):
 *   k_float=(20/18)²=1.235
 *   UP:   k=ceil(1.235)−1=1, new r=sqrt(1)×18=18      ✓
 *   DOWN: k=floor(1.235)+1=2, new r=sqrt(2)×18≈25.5   ✓
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

#define TARGET_FPS     30
#define CELL_W          2
#define CELL_H          4

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

#define R_POLAR_MIN     4.0   /* minimum cursor radius — avoids origin singularity */

/* Object pool */
#define MAX_OBJ       256
#define OBJ_GLYPH     'o'

/* Golden ratio constants (for sunflower and log-spiral backgrounds) */
#define PHI            1.61803398874989484820
#define GOLDEN_ANGLE  (2.0 * M_PI / (PHI * PHI))

/* Sunflower background seed count */
#define N_BG_SEEDS    600

/* ── Per-grid cursor step sizes ────────────────────────────────────────── *
 * Each background mode defines its own natural step: ring spacing for
 * circular grids, pitch for spirals, seed count for sunflower, etc.      */
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

/* Color pairs */
#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_ANCHOR  3
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
 * GridCtx — geometry for the current polar grid mode.
 *
 *   mode    — which of the 7 polar backgrounds is active (0..6).
 *   rows/cols — terminal extent (cached for resize handling).
 *   cw/ch   — terminal cell aspect (pixels per column / per row).
 *   ox/oy   — screen-space origin (centre of terminal).
 *   max_ring/max_spoke — informational caps for HUD; cursor uses R_POLAR_MIN
 *                        and screen clamping rather than a hard ring index.
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
    g->max_spoke = 12;     /* default spoke count for ring-style modes */
    /* approximate ring count fitting on screen */
    int half_diag_px = (int)(sqrt((double)(g->ox*g->cw)*(g->ox*g->cw) +
                                   (double)(g->oy*g->ch)*(g->oy*g->ch)));
    g->max_ring = (int)(half_diag_px / BG_RING_SP);
}

/*
 * cell_to_polar — convert screen cell (col, row) to polar (r_px, theta).
 *
 *   dx = (col − ox) × CELL_W      [pixel-space x offset]
 *   dy = (row − oy) × CELL_H      [pixel-space y offset]
 *   r  = sqrt(dx² + dy²)
 *   θ  = atan2(dy, dx)             ∈ (−π, π]
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
 * polar_to_screen — inverse of cell_to_polar; (r, θ) → (col, row).
 * Used by cursor_sync, sunflower seed placement, and (in 02–04) by
 * arc/spoke/spiral/scatter rasterisers.
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int row, col; char glyph; bool alive; } Obj;
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/*
 * cursor_move — grid-aware movement: dispatches per mode so arrows
 * always follow the natural geometry of the active background.
 */
static void cursor_move(Cursor *c, const GridCtx *g, int key)
{
    const double two_pi = 2.0 * M_PI;

    switch (g->mode) {

    case 0: /* rings+spokes: UP/DOWN = one ring, LEFT/RIGHT = one spoke */
        switch (key) {
        case KEY_UP:    c->r -= BG_RING_SP;       break;
        case KEY_DOWN:  c->r += BG_RING_SP;       break;
        case KEY_LEFT:  c->theta -= BG_SPOKE_ANG; break;
        case KEY_RIGHT: c->theta += BG_SPOKE_ANG; break;
        default: return;
        }
        if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
        c->theta = fmod(c->theta + 4.0*two_pi, two_pi);
        cursor_sync(c, g);
        break;

    case 1: /* log-polar: UP/DOWN = multiply r by RATIO=e^0.25, LEFT/RIGHT = spoke */
        switch (key) {
        case KEY_UP:    c->r /= exp(BG_LOG_RATIO);    break;
        case KEY_DOWN:  c->r *= exp(BG_LOG_RATIO);    break;
        case KEY_LEFT:  c->theta -= BG_SPOKE_ANG;     break;
        case KEY_RIGHT: c->theta += BG_SPOKE_ANG;     break;
        default: return;
        }
        if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
        c->theta = fmod(c->theta + 4.0*two_pi, two_pi);
        cursor_sync(c, g);
        break;

    case 2: { /* archimedean: LR walk along curve (Δr = a×Δθ), UD jump one turn */
        double a = BG_ARCH_PITCH / two_pi;
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
        if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
        c->theta = fmod(c->theta + 4.0*two_pi, two_pi);
        cursor_sync(c, g);
        break;
    }

    case 3: /* log-spiral: LR walk along curve, UD flip between 2 golden arms */
        switch (key) {
        case KEY_LEFT:
            c->theta -= BG_LOG_ANG;
            c->r     /= exp(2.0 * log(PHI) / M_PI * BG_LOG_ANG);
            break;
        case KEY_RIGHT:
            c->theta += BG_LOG_ANG;
            c->r     *= exp(2.0 * log(PHI) / M_PI * BG_LOG_ANG);
            break;
        case KEY_UP:   c->theta -= M_PI; break;  /* 2-arm: arms are π apart */
        case KEY_DOWN: c->theta += M_PI; break;
        default: return;
        }
        if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
        c->theta = fmod(c->theta + 4.0*two_pi, two_pi);
        cursor_sync(c, g);
        break;

    case 4: /* sunflower: step seed index; cursor_apply_seed snaps to exact seed */
        switch (key) {
        case KEY_UP:    c->seed_idx -= BG_SEED_JUMP; break;
        case KEY_DOWN:  c->seed_idx += BG_SEED_JUMP; break;
        case KEY_LEFT:  c->seed_idx -= BG_SEED_STEP; break;
        case KEY_RIGHT: c->seed_idx += BG_SEED_STEP; break;
        default: return;
        }
        cursor_apply_seed(c, g);
        break;

    case 5: { /* equal-area: UP/DOWN snap to sqrt(k)×R_UNIT ring, LR = spoke */
        switch (key) {
        case KEY_UP: {
            double kf = (c->r / BG_RUNIT) * (c->r / BG_RUNIT);
            double k  = ceil(kf) - 1.0;
            if (k < 1.0) k = 1.0;
            c->r = sqrt(k) * BG_RUNIT;
            break;
        }
        case KEY_DOWN: {
            double kf = (c->r / BG_RUNIT) * (c->r / BG_RUNIT);
            double k  = floor(kf) + 1.0;
            c->r = sqrt(k) * BG_RUNIT;
            break;
        }
        case KEY_LEFT:  c->theta -= BG_SPOKE_ANG; break;
        case KEY_RIGHT: c->theta += BG_SPOKE_ANG; break;
        default: return;
        }
        if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
        c->theta = fmod(c->theta + 4.0*two_pi, two_pi);
        cursor_sync(c, g);
        break;
    }

    case 6: { /* elliptic: move in (e_r, ell_θ) elliptic coordinate frame */
        double dx   = (double)(c->col - g->ox) * CELL_W;
        double dy   = (double)(c->row - g->oy) * CELL_H;
        double e_r  = sqrt((dx/BG_ELLIP_A)*(dx/BG_ELLIP_A) +
                           (dy/BG_ELLIP_B)*(dy/BG_ELLIP_B));
        double e_th = atan2(dy/BG_ELLIP_B, dx/BG_ELLIP_A);
        switch (key) {
        case KEY_UP: {
            /* snap to inner ring: ceil(e_r/sp)-1 steps in e_r space */
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
        /* convert back from elliptic to screen: dx=e_r×A×cos, dy=e_r×B×sin */
        c->col = g->ox + (int)round(e_r * BG_ELLIP_A * cos(e_th) / CELL_W);
        c->row = g->oy + (int)round(e_r * BG_ELLIP_B * sin(e_th) / CELL_H);
        if (c->row < 0)          c->row = 0;
        if (c->row >= g->rows-1) c->row = g->rows-2;
        if (c->col < 0)          c->col = 0;
        if (c->col >= g->cols)   c->col = g->cols-1;
        cell_to_polar(c->col, c->row, g->ox, g->oy, &c->r, &c->theta);
        if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
        break;
    }

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

/*
 * draw_polar_bg — dispatch on g->mode and draw the matching polar grid.
 * Each case is the drawing core from the corresponding grids/polar_grids/NN_*.c
 * file with default parameters.  No mutation; reads g for origin/extent only.
 */
static void draw_polar_bg(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols;
    int ox = g->ox, oy = g->oy;
    const double two_pi = 2.0 * M_PI;
    attron(COLOR_PAIR(PAIR_GRID));

    switch (g->mode) {

    case 0: { /* rings + spokes (default params from 01_rings_spokes.c) */
        const double sp = 20.0, rw = 1.6, sw = 0.10;
        const double sa = two_pi / 12.0;
        for (int row = 0; row < rows-1; row++)
          for (int col = 0; col < cols; col++) {
            double r, th; cell_to_polar(col, row, ox, oy, &r, &th);
            double rp = fmod(r, sp);
            bool on_r = rp < rw || rp > sp - rw;
            double tn = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            bool on_s = r > 3.0 && (sp2 < sw || sp2 > sa - sw);
            if (on_r || on_s)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
          }
        break;
    }

    case 1: { /* log-polar (02_log_polar.c defaults) */
        const double rmin = 4.0, ls = 0.25, rwu = 0.08, sw = 0.10;
        const double sa = two_pi / 12.0;
        for (int row = 0; row < rows-1; row++)
          for (int col = 0; col < cols; col++) {
            double r, th; cell_to_polar(col, row, ox, oy, &r, &th);
            bool on_r = false;
            if (r > rmin) {
                double u = log(r / rmin) / ls;
                double fr = u - floor(u);
                on_r = fr < rwu || fr > 1.0 - rwu;
            }
            double tn = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            bool on_s = r > 3.0 && (sp2 < sw || sp2 > sa - sw);
            if (on_r || on_s)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
          }
        break;
    }

    case 2: { /* archimedean spiral, 2 arms (03_archimedean_spiral.c defaults) */
        const double pitch = 32.0, sw = 0.20, rmin = 3.0;
        double a = pitch / two_pi;
        for (int row = 0; row < rows-1; row++)
          for (int col = 0; col < cols; col++) {
            double r, th; cell_to_polar(col, row, ox, oy, &r, &th);
            if (r < rmin) continue;
            double tn  = fmod(th + two_pi, two_pi);
            double raw = 2.0 * (tn - r / a);
            double ph  = fmod(raw + 2.0 * two_pi, two_pi);
            if (ph < sw || ph > two_pi - sw)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
          }
        break;
    }

    case 3: { /* log-spiral golden, 2 arms (04_log_spiral.c defaults) */
        const double growth = 2.0 * log(PHI) / M_PI;
        const double sw = 0.22, rmin = 4.0;
        for (int row = 0; row < rows-1; row++)
          for (int col = 0; col < cols; col++) {
            double r, th; cell_to_polar(col, row, ox, oy, &r, &th);
            if (r < rmin) continue;
            double tn  = fmod(th + two_pi, two_pi);
            double tp  = log(r / rmin) / growth;
            double raw = 2.0 * (tn - tp);
            double ph  = fmod(raw + 2.0 * two_pi, two_pi);
            if (ph < sw || ph > two_pi - sw)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
          }
        break;
    }

    case 4: { /* sunflower phyllotaxis (05_sunflower.c defaults) */
        const double sp = BG_SEED_SP;
        bool *vis = calloc((size_t)(rows * cols), 1);
        if (!vis) break;
        for (int i = 0; i < N_BG_SEEDS; i++) {
            double r  = sqrt((double)i) * sp;
            double th = (double)i * GOLDEN_ANGLE;
            int c  = ox + (int)round(r * cos(th) / CELL_W);
            int rw = oy + (int)round(r * sin(th) / CELL_H);
            if (rw < 0 || rw >= rows-1 || c < 0 || c >= cols) continue;
            if (vis[rw * cols + c]) continue;
            vis[rw * cols + c] = true;
            mvaddch(rw, c, (chtype)(unsigned char)'o');
        }
        free(vis);
        break;
    }

    case 5: { /* equal-area sectors (06_sector.c defaults) */
        const double ru = 18.0, rwf = 0.06, sw = 0.10;
        const double sa = two_pi / 12.0;
        double rusq = ru * ru;
        for (int row = 0; row < rows-1; row++)
          for (int col = 0; col < cols; col++) {
            double r, th; cell_to_polar(col, row, ox, oy, &r, &th);
            if (r < 3.0) continue;
            double kf = (r*r) / rusq;
            double fr = kf - floor(kf);
            bool on_r = fr < rwf || fr > 1.0 - rwf;
            double tn  = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            bool on_s = sp2 < sw || sp2 > sa - sw;
            if (on_r || on_s)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(th));
          }
        break;
    }

    case 6: { /* elliptic (07_elliptic.c defaults) */
        const double A = 1.6, B = 1.0, sp = 20.0, rwu = 0.07;
        for (int row = 0; row < rows-1; row++)
          for (int col = 0; col < cols; col++) {
            double dx = (double)(col - ox) * CELL_W;
            double dy = (double)(row - oy) * CELL_H;
            double er = sqrt((dx/A)*(dx/A) + (dy/B)*(dy/B));
            if (er < 0.5) continue;
            double et = atan2(dy/B, dx/A);
            double u  = er / sp, fr = u - floor(u);
            if (fr < rwu || fr > 1.0 - rwu)
                mvaddch(row, col, (chtype)(unsigned char)angle_char(et));
          }
        break;
    }

    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Bright bold yellow status (top-right) + bold cyan key hints (bottom). */
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
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
/* §10  app                                                                */
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
            /* entering sunflower: snap seed_idx to nearest seed at current r */
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
