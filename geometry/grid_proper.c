/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * grid_proper.c — rectangular, polar and hexagonal grids with a moving object
 *
 * DEMO: Three coordinate systems drawn on the terminal with a single '@' object
 *       bouncing between walls. Keys 1/2/3 switch grids live. The HUD shows the
 *       object's position translated into each grid's native coordinate system —
 *       the same physical point described as (col,row), (ring,sector), or (hx,hy).
 *
 * Study alongside: framework.c (canonical loop structure), rect_grid.c,
 *                  polar_grid.c, hex_grid.c (original single-grid versions)
 *
 * Section map:
 *   §1  config  — all constants for all three grids + object
 *   §2  clock   — monotonic timer + sleep
 *   §3  color   — unified 6-level palette (38 pairs total)
 *   §4  coords  — CELL_W/CELL_H, px_to_cell_x/y (object lives in pixel space)
 *   §5  object  — bouncing '@': state, tick, steer
 *   §6  rect    — rectangular grid: static +/-/| grid lines, wave colour
 *   §7  polar   — polar grid: rotating rings, tangent-direction arc chars
 *   §8  hex     — hexagonal offset-row grid: _/\ outline tiling
 *   §9  screen  — HUD, ncurses double-buffer flush
 *  §10  app     — signals, resize, main loop
 *
 * Keys:
 *   1 / 2 / 3    switch grid  (rectangular / polar / hexagonal)
 *   arrow keys   steer the object
 *   t            cycle theme  (6 themes)
 *   p            pause / resume
 *   r            reset object to centre
 *   q / ESC      quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra geometry/grid_proper.c -o grid_proper -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three coordinate systems for the same 2-D plane.
 *                  Rectangular — uniform square tiling: position = (col, row).
 *                  Polar       — rings × spokes: position = (radius r, angle θ).
 *                  Hexagonal   — offset-row tiling: position = (hx, hy) in screen
 *                  coords, converted to cube coords (q+r+s=0) for ring distance.
 *                  Switching grids shows how one physical point has a different
 *                  "address" depending on the active coordinate system.
 *
 * Rendering      : Each grid is drawn from pure geometry — no per-cell character
 *                  arrays. Rect computes +/-/| from (r%step, c%step). Polar uses
 *                  ring_char(angle) which picks |/-/\ by the tangent direction at
 *                  each sector position. Hex draws _/\ outlines per hex cell.
 *                  The colour wave is analytic (sine-wave interference) for rect
 *                  and distance-band for polar/hex — both computed per frame.
 *
 * Object         : Pixel-space physics {px,py,vx,vy}. px_to_cell_x/y converts
 *                  once per frame. The host cell in each grid gets OBJ_PAIR+BOLD.
 *
 * References     :
 *   Hexagonal grids (cube coords, offset layout) — redblobgames.com/grids/hexagons
 *   Polar coordinate system — Wikipedia: "Polar coordinate system"
 *   Terminal aspect ratio fix — this project's framework.c §4
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/*
 * All magic numbers live here. Change behaviour by editing this block only.
 */
enum {
    TARGET_FPS    = 60,

    /* Colour system — 6 themes × 6 levels = 36 pairs; obj + HUD after */
    N_THEMES      =  6,
    N_LEVELS      =  6,
    OBJ_PAIR      = N_THEMES * N_LEVELS + 1,   /* 37 — bright yellow    */
    HUD_PAIR      = N_THEMES * N_LEVELS + 2,   /* 38 — white            */

    /* Rectangular grid — spacing between drawn grid lines */
    RECT_ROW_STEP =  4,   /* horizontal lines every 4 rows               */
    RECT_COL_STEP =  8,   /* vertical lines every 8 columns              */

    /* Polar grid */
    N_RINGS       =  6,   /* concentric rings                            */
    MAX_SECTORS   = 128,  /* max spokes on any single ring               */

    /* Hexagonal grid — pointy-top offset-row layout
     * Each hex outline occupies 6 cols × 3 rows:
     *   row+0:  ____      (4 underscores at cols c+1..c+4)
     *   row+1: /    \     (slash at c, backslash at c+5)
     *   row+2: \____/     (backslash at c, 4 underscores, slash at c+5)
     */
    HEX_DX        =  6,   /* terminal columns per hex step               */
    HEX_DY        =  3,   /* terminal rows per hex step                  */
    HEX_MAX_COLS  = 80,
    HEX_MAX_ROWS  = 45,
};

/*
 * Terminal cell width / height ratio ≈ 0.47 (cells are ~2× taller than wide).
 * Used by the polar grid to make rings look circular, not elliptical.
 */
#define POLAR_ASPECT   0.47f

/*
 * Object physics constants.
 *   OBJ_DRAG        fraction of speed lost per second (exponential decay)
 *   OBJ_STEER_FORCE pixels/sec impulse added per arrow keypress
 *   OBJ_SPEED_MAX   terminal velocity — prevents runaway after many presses
 *   OBJ_BOUNCE_DAMP fraction of speed kept on wall impact
 */
#define OBJ_DRAG         0.40f
#define OBJ_STEER_FORCE  180.0f
#define OBJ_SPEED_MAX    280.0f
#define OBJ_BOUNCE_DAMP  0.75f

/*
 * 6-level 256-color ramps per theme (index 0 = brightest, 5 = dimmest).
 * Shared by rect (wave level), polar (ring index), hex (distance band).
 */
static const int g_pal[N_THEMES][N_LEVELS] = {
    { 231, 226, 220, 208, 202, 196 },   /* fire    */
    {  82,  46,  40,  34,  28,  22 },   /* matrix  */
    { 159,  51,  45,  33,  27,  21 },   /* ice     */
    { 207, 201, 165, 129,  93,  57 },   /* plasma  */
    { 220, 184, 178, 172, 136, 130 },   /* gold    */
    { 255, 251, 247, 243, 239, 235 },   /* mono    */
};
static const char *g_theme_names[N_THEMES] = {
    "fire", "matrix", "ice", "plasma", "gold", "mono"
};

/* PAIR(t, l): colour pair id for theme t, brightness level l */
#define PAIR(t, l)  ((t) * N_LEVELS + (l) + 1)

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / 1000000000LL),
        .tv_nsec = (long)  (ns % 1000000000LL),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();

    /* 8-color fallback: map 6 brightness levels to 4 basic colors */
    static const int fb8[N_LEVELS] = {
        COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_CYAN,  COLOR_CYAN,   COLOR_BLUE,
    };

    for (int t = 0; t < N_THEMES; t++)
        for (int l = 0; l < N_LEVELS; l++)
            init_pair(PAIR(t, l),
                      COLORS >= 256 ? g_pal[t][l] : fb8[l], -1);

    /* Object: bright yellow stands out against any grid theme */
    init_pair(OBJ_PAIR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(HUD_PAIR, COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the aspect-ratio fix                         */
/* ===================================================================== */

/*
 * WHY pixel space for the object:
 *   Terminal cells are ~8 px wide × 16 px tall. Moving by one cell in X
 *   travels 8 physical pixels; one cell in Y travels 16 physical pixels.
 *   Moving (dx=1, dy=1) per tick looks skewed. In pixel space, dx=dy means
 *   equal physical distance in both axes — correct diagonal motion.
 *
 *   The conversion px_to_cell_x/y happens exactly once, in scene_draw.
 *   Physics code never sees cell coordinates.
 *
 * See framework.c §4 for the full explanation including WHY floorf not roundf.
 */
#define CELL_W   8
#define CELL_H  16

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  object — the bouncing '@'                                          */
/* ===================================================================== */

/*
 * Obj — a single point-mass in pixel space.
 *
 *   px, py   position — pixels
 *   vx, vy   velocity — pixels / second
 */
typedef struct {
    float px, py;   /* position — pixels               */
    float vx, vy;   /* velocity — pixels / second      */
} Obj;

static void obj_init(Obj *o, int cols, int rows)
{
    o->px = (float)pw(cols) * 0.5f;
    o->py = (float)ph(rows - 2) * 0.5f;
    o->vx =  90.0f;
    o->vy =  55.0f;
}

/*
 * obj_tick() — advance object one timestep.
 *
 * Uses exponential drag: v(t) = v₀ × e^(−DRAG×t).
 * Per frame: v *= expf(−DRAG×dt) — frame-rate independent.
 *
 * WHY expf, not `vel *= 0.99`:
 *   A per-frame multiplier like 0.99 is applied once per frame regardless
 *   of dt. At 30 fps the drag is half as strong as at 60 fps — the object
 *   behaves differently depending on CPU load. expf(−DRAG×dt) is exact
 *   and produces identical behaviour at any frame rate.
 */
static void obj_tick(Obj *o, float dt, int cols, int rows)
{
    float decay = expf(-OBJ_DRAG * dt);
    o->vx *= decay;
    o->vy *= decay;

    float spd = sqrtf(o->vx * o->vx + o->vy * o->vy);
    if (spd > OBJ_SPEED_MAX) {
        float inv = OBJ_SPEED_MAX / spd;
        o->vx *= inv;
        o->vy *= inv;
    }

    o->px += o->vx * dt;
    o->py += o->vy * dt;

    float max_x = (float)pw(cols - 1);
    float max_y = (float)ph(rows - 2);

    if (o->px < 0)     { o->px = 0;     o->vx =  fabsf(o->vx) * OBJ_BOUNCE_DAMP; }
    if (o->px > max_x) { o->px = max_x; o->vx = -fabsf(o->vx) * OBJ_BOUNCE_DAMP; }
    if (o->py < 0)     { o->py = 0;     o->vy =  fabsf(o->vy) * OBJ_BOUNCE_DAMP; }
    if (o->py > max_y) { o->py = max_y; o->vy = -fabsf(o->vy) * OBJ_BOUNCE_DAMP; }
}

static void obj_steer(Obj *o, float dx, float dy)
{
    o->vx += dx * OBJ_STEER_FORCE;
    o->vy += dy * OBJ_STEER_FORCE;
}

/* ── end §5 — rect grid follows ── */

/* ===================================================================== */
/* §6  rectangular grid                                                   */
/* ===================================================================== */

/*
 * rect_draw() — draw a static +/-/| grid with a two-wave colour wash.
 *
 * Every cell is classified geometrically:
 *   r % RECT_ROW_STEP == 0  AND  c % RECT_COL_STEP == 0  →  '+'  (corner)
 *   r % RECT_ROW_STEP == 0                                →  '-'  (h-line)
 *   c % RECT_COL_STEP == 0                                →  '|'  (v-line)
 *   otherwise: skip (erase() already blanked the cell)
 *
 * Colour wave: two sine waves at different spatial frequencies combine to
 * create a slowly shifting interference pattern. The wave phases wx, wy
 * advance each frame so the colour ripples across the grid.
 */
static void rect_draw(int rows, int cols, int theme,
                      float wx, float wy,
                      int obj_col, int obj_row)
{
    for (int r = 0; r < rows - 1; r++) {
        bool on_hline = (r % RECT_ROW_STEP == 0);
        float wy_r = sinf((float)r * 0.26f + wy);
        for (int c = 0; c < cols; c++) {
            bool on_vline = (c % RECT_COL_STEP == 0);
            if (!on_hline && !on_vline) continue;

            char ch;
            if (on_hline && on_vline) ch = '+';
            else if (on_hline)        ch = '-';
            else                      ch = '|';

            float w     = (sinf((float)c * 0.18f + wx) + wy_r) * 0.25f + 0.5f;
            int   level = (int)(w * (float)(N_LEVELS - 1) + 0.5f);
            if (level < 0)          level = 0;
            if (level >= N_LEVELS)  level = N_LEVELS - 1;

            bool   is_obj = (r == obj_row && c == obj_col);
            int    pair   = is_obj ? OBJ_PAIR : PAIR(theme, level);
            attr_t attr   = COLOR_PAIR(pair) | (is_obj ? (attr_t)A_BOLD : (attr_t)0);

            attron(attr);
            mvaddch(r, c, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── end §6 — polar grid follows ── */

/* ===================================================================== */
/* §7  polar grid                                                         */
/* ===================================================================== */

/*
 * Polar grid state — N_RINGS concentric rings, each with its own:
 *   g_ring_r[i]     radius in terminal column units
 *   g_n_sectors[i]  number of evenly-spaced spokes on ring i
 *   g_ring_angle[i] current rotation offset in radians (advances per tick)
 *   g_rot_speed[i]  radians/second — alternating direction, faster inward
 *
 * Characters are computed from angle — no per-sector storage needed.
 */
static int   g_n_sectors[N_RINGS];
static float g_ring_r[N_RINGS];
static float g_ring_angle[N_RINGS];
static float g_rot_speed[N_RINGS];

/*
 * ring_char() — choose ASCII char that follows the tangent direction at angle.
 *
 * The tangent to a circle at angle θ points in direction (-sinθ, cosθ).
 * We correct the row component by POLAR_ASPECT (≈0.47) to account for the
 * fact that terminal cells are taller than wide — without this, all circles
 * would appear to have only horizontal tangents near 3 o'clock / 9 o'clock.
 *
 * Tangent col component dc = -sin(θ)
 * Tangent row component dr =  cos(θ) × POLAR_ASPECT
 *   slope = dr/dc → small slope → '-', large → '|', middle → '/' or '\'
 */
static char ring_char(float angle)
{
    float dc = -sinf(angle);
    float dr =  cosf(angle) * POLAR_ASPECT;
    if (fabsf(dc) < 0.15f) return '|';
    float slope = dr / dc;
    if (fabsf(slope) < 0.5f) return '-';
    return (slope > 0.0f) ? '\\' : '/';
}

static void polar_init(int rows, int cols)
{
    float max_r = fminf((float)(cols / 2 - 2),
                        (float)(rows / 2 - 1) / POLAR_ASPECT);
    float min_r = 3.0f;

    for (int i = 0; i < N_RINGS; i++) {
        float t     = (float)i / (float)(N_RINGS - 1);
        /* Quadratic spacing: denser rings near centre */
        g_ring_r[i] = min_r + (max_r - min_r) * (t * t * 0.4f + t * 0.6f);

        /* One sector per ~2 column-widths of arc for readable spacing */
        int n = (int)(2.f * (float)M_PI * g_ring_r[i] / 2.0f);
        if (n < 6)           n = 6;
        if (n > MAX_SECTORS) n = MAX_SECTORS;
        g_n_sectors[i] = n;

        /* Alternate direction; inner rings spin faster */
        float dir       = (i % 2 == 0) ? 1.f : -1.f;
        g_rot_speed[i]  = dir * (0.10f + (float)(N_RINGS - 1 - i) * 0.04f);
        g_ring_angle[i] = (float)(rand() % 628) / 100.f;
    }
}

/* polar_tick() — advance ring rotation angles only; chars come from geometry */
static void polar_tick(float dt)
{
    for (int i = 0; i < N_RINGS; i++)
        g_ring_angle[i] += g_rot_speed[i] * dt;
}

/*
 * polar_find_obj_cell() — find the (ring, sector) whose screen position
 *                         is closest to (obj_col, obj_row).
 *
 * Linear scan over all rings × sectors: O(Σ n_sectors) ≈ O(300).
 */
static void polar_find_obj_cell(int cx, int cy,
                                int obj_col, int obj_row,
                                int *out_ring, int *out_sec)
{
    float best = 1e9f;
    *out_ring = 0; *out_sec = 0;

    for (int i = 0; i < N_RINGS; i++) {
        float r = g_ring_r[i];
        int   n = g_n_sectors[i];
        for (int j = 0; j < n; j++) {
            float angle = g_ring_angle[i]
                        + 2.f * (float)M_PI * (float)j / (float)n;
            int col = cx + (int)(r * cosf(angle) + 0.5f);
            int row = cy + (int)(r * sinf(angle) * POLAR_ASPECT + 0.5f);
            float d2 = (float)((col - obj_col) * (col - obj_col) +
                               (row - obj_row) * (row - obj_row));
            if (d2 < best) { best = d2; *out_ring = i; *out_sec = j; }
        }
    }
}

/*
 * polar_draw() — render rotating rings with tangent-direction arc chars.
 *
 * Ring index i → palette level i (ring 0 = inner = brightest).
 * The (ring, sector) nearest the object gets OBJ_PAIR + A_BOLD.
 * Centre marked with '+' in the innermost colour.
 */
static void polar_draw(int rows, int cols, int theme,
                       int obj_ring, int obj_sec)
{
    int cx = cols / 2, cy = rows / 2;

    attron(COLOR_PAIR(PAIR(theme, 0)) | A_BOLD);
    mvaddch(cy, cx, '+');
    attroff(COLOR_PAIR(PAIR(theme, 0)) | A_BOLD);

    for (int i = 0; i < N_RINGS; i++) {
        float r = g_ring_r[i];
        int   n = g_n_sectors[i];

        for (int j = 0; j < n; j++) {
            float angle = g_ring_angle[i]
                        + 2.f * (float)M_PI * (float)j / (float)n;
            int col = cx + (int)(r * cosf(angle) + 0.5f);
            int row = cy + (int)(r * sinf(angle) * POLAR_ASPECT + 0.5f);

            if (row < 0 || row >= rows - 1) continue;
            if (col < 0 || col >= cols)     continue;

            bool   is_obj = (i == obj_ring && j == obj_sec);
            int    pair   = is_obj ? OBJ_PAIR : PAIR(theme, i);
            attr_t attr   = COLOR_PAIR(pair) | (is_obj ? (attr_t)A_BOLD : (attr_t)0);

            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ring_char(angle));
            attroff(attr);
        }
    }
}

/* ── end §7 — hex grid follows ── */

/* ===================================================================== */
/* §8  hexagonal grid                                                     */
/* ===================================================================== */

/*
 * Hex grid uses "offset-row" layout: odd rows (hy & 1) are shifted right
 * by HEX_DX/2 columns. This is "pointy-top hexagon" orientation.
 *
 * Top-left corner of hex (hx, hy) on screen:
 *   col = hx × HEX_DX + (hy & 1) × (HEX_DX / 2)
 *   row = hy × HEX_DY
 *
 * Each hex is drawn as a 6-wide × 3-tall outline:
 *   row+0:  ____      cols c+1..c+4
 *   row+1: /    \     col c  and  col c+5
 *   row+2: \____/     col c, cols c+1..c+4, col c+5
 *
 * Colour band = cube-coordinate distance from the centre hex, mod N_LEVELS.
 */
static int g_hx_count, g_hy_count;
static int g_cx_hex,   g_cy_hex;

/*
 * hex_dist() — cube-coordinate distance between two offset-row hex cells.
 *
 * Converts each hex (hx, hy) to cube coords (q, r, s where q+r+s = 0):
 *   q = hx − (hy − (hy & 1)) / 2,  r = hy
 * Distance = max(|Δq|, |Δr|, |Δq+Δr|).
 *
 * WHY cube coords: offset-row distance is asymmetric — stepping from an
 * even row costs a different (col, row) delta than from an odd row.
 * Cube coords are uniform in all 6 hex directions.
 */
static int hex_dist(int hx1, int hy1, int hx2, int hy2)
{
    int q1 = hx1 - (hy1 - (hy1 & 1)) / 2,  r1 = hy1;
    int q2 = hx2 - (hy2 - (hy2 & 1)) / 2,  r2 = hy2;
    int dq = q1 - q2, dr = r1 - r2, ds = dq + dr;
    int v = abs(dq);
    if (abs(dr) > v) v = abs(dr);
    if (abs(ds) > v) v = abs(ds);
    return v;
}

static void hex_init(int rows, int cols)
{
    g_hx_count = cols / HEX_DX + 1;
    g_hy_count = rows / HEX_DY;
    if (g_hx_count > HEX_MAX_COLS) g_hx_count = HEX_MAX_COLS;
    if (g_hy_count > HEX_MAX_ROWS) g_hy_count = HEX_MAX_ROWS;
    g_cx_hex = g_hx_count / 2;
    g_cy_hex = g_hy_count / 2;
}

/*
 * hex_screen_to_hex() — inverse of the draw formula.
 *
 * Given screen (col, row), find the hex cell (hx, hy) that maps to it.
 * Integer division snaps to the nearest hex cell.
 */
static void hex_screen_to_hex(int col, int row, int *out_hx, int *out_hy)
{
    int hy = row / HEX_DY;
    int hx = (col - (hy & 1) * (HEX_DX / 2)) / HEX_DX;
    if (hy < 0)           hy = 0;
    if (hy >= g_hy_count) hy = g_hy_count - 1;
    if (hx < 0)           hx = 0;
    if (hx >= g_hx_count) hx = g_hx_count - 1;
    *out_hy = hy;
    *out_hx = hx;
}

/*
 * hex_draw_one() — draw a single hex outline at top-left corner (c, r).
 *
 *   row r:    ____      (4 underscores, cols c+1..c+4)
 *   row r+1: /    \     (slash at c, backslash at c+5)
 *   row r+2: \____/     (backslash at c, 4 underscores, slash at c+5)
 *
 * All bounds-checked against (rows, cols) to handle partial hexes at edges.
 */
static void hex_draw_one(int c, int r, int rows, int cols, attr_t attr)
{
    attron(attr);

    /* top: ____ */
    if (r >= 0 && r < rows - 1)
        for (int k = 1; k <= 4; k++)
            if (c + k >= 0 && c + k < cols)
                mvaddch(r, c + k, '_');

    /* middle: /    \ */
    if (r + 1 >= 0 && r + 1 < rows - 1) {
        if (c     >= 0 && c     < cols) mvaddch(r + 1, c,     '/');
        if (c + 5 >= 0 && c + 5 < cols) mvaddch(r + 1, c + 5, '\\');
    }

    /* bottom: \____/ */
    if (r + 2 >= 0 && r + 2 < rows - 1) {
        if (c >= 0 && c < cols) mvaddch(r + 2, c, '\\');
        for (int k = 1; k <= 4; k++)
            if (c + k >= 0 && c + k < cols)
                mvaddch(r + 2, c + k, '_');
        if (c + 5 >= 0 && c + 5 < cols) mvaddch(r + 2, c + 5, '/');
    }

    attroff(attr);
}

static void hex_draw(int rows, int cols, int theme,
                     int obj_hx, int obj_hy)
{
    for (int hy = 0; hy < g_hy_count; hy++) {
        for (int hx = 0; hx < g_hx_count; hx++) {
            int col = hx * HEX_DX + (hy & 1) * (HEX_DX / 2);
            int row = hy * HEX_DY;

            if (row >= rows - 1 || col >= cols) continue;

            int    dist  = hex_dist(hx, hy, g_cx_hex, g_cy_hex);
            int    level = dist % N_LEVELS;
            bool   is_obj = (hx == obj_hx && hy == obj_hy);
            int    pair   = is_obj ? OBJ_PAIR : PAIR(theme, level);
            attr_t attr   = COLOR_PAIR(pair) | (is_obj ? (attr_t)A_BOLD : (attr_t)0);

            hex_draw_one(col, row, rows, cols, attr);
        }
    }
}

/* ── end §8 — screen layer follows ── */

/* ===================================================================== */
/* §9  screen — HUD + double-buffer flush                                 */
/* ===================================================================== */

/*
 * draw_hud() — two fixed UI lines.
 *
 * Top-right : fps | active grid name | theme | PAUSED flag
 * Bottom    : key hint strip (left) + object native coords (right)
 *
 * The native-coord display is the main teaching payoff: same screen point
 * described as (col,row), (ring,sector), or (hx,hy) depending on mode.
 */
static void draw_hud(int rows, int cols, float fps,
                     int mode, int theme, bool paused,
                     int obj_col, int obj_row,
                     int obj_ring, int obj_sec,
                     int obj_hx, int obj_hy)
{
    static const char *mode_names[3] = { "RECT", "POLAR", "HEX" };

    char top[72];
    snprintf(top, sizeof top, " %.0f fps  %s  %s%s ",
             (double)fps, mode_names[mode], g_theme_names[theme],
             paused ? "  PAUSED" : "");
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, cols - (int)strlen(top), "%s", top);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    attron(COLOR_PAIR(HUD_PAIR));
    mvprintw(rows - 1, 0,
             " 1/2/3:grid  arrows:steer  t:theme  p:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(HUD_PAIR));

    char coord[48];
    if (mode == 0)
        snprintf(coord, sizeof coord, " rect c=%d r=%d ", obj_col, obj_row);
    else if (mode == 1)
        snprintf(coord, sizeof coord, " polar ring=%d sec=%d ", obj_ring, obj_sec);
    else
        snprintf(coord, sizeof coord, " hex hx=%d hy=%d ", obj_hx, obj_hy);

    attron(COLOR_PAIR(OBJ_PAIR) | A_BOLD);
    mvprintw(rows - 1, cols - (int)strlen(coord), "%s", coord);
    attroff(COLOR_PAIR(OBJ_PAIR) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §10  app — signals, resize, main loop                                  */
/* ===================================================================== */

static volatile sig_atomic_t g_run = 1;
static volatile sig_atomic_t g_rsz = 0;
static void on_exit_signal  (int s) { (void)s; g_run = 0; }
static void on_resize_signal(int s) { (void)s; g_rsz = 1; }
static void cleanup(void)           { endwin(); }

/*
 * main() — the game loop.
 *
 * Loop order (same as framework.c):
 *   ① resize check
 *   ② dt measurement (capped at 100 ms)
 *   ③ tick object + polar rings (hex and rect have no per-frame state)
 *   ④ fps counter (500 ms window)
 *   ⑤ derive object position in each coordinate system
 *   ⑥ sleep to cap at 60 fps  ← BEFORE render
 *   ⑦ erase → draw grid → draw '@' → HUD → doupdate
 *   ⑧ poll input (non-blocking)
 */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    initscr();
    noecho(); cbreak(); curs_set(0);
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE); typeahead(-1);

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    color_init();

    int   mode   = 0;    /* 0=rect, 1=polar, 2=hex */
    int   theme  = 1;    /* start on matrix */
    bool  paused = false;
    float fps = 0.f, fps_acc = 0.f;
    int   fps_cnt = 0;
    float wave_x = 0.f, wave_y = 0.f;   /* rect sine-wave phases */

    Obj obj;
    obj_init(&obj, cols, rows);
    polar_init(rows, cols);
    hex_init(rows, cols);

    int64_t frame_ns = 1000000000LL / TARGET_FPS;
    int64_t last     = clock_ns();

    while (g_run) {

        /* ── ① resize ──────────────────────────────────────────────── */
        if (g_rsz) {
            g_rsz = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            polar_init(rows, cols);
            hex_init(rows, cols);
            obj_init(&obj, cols, rows);
            last = clock_ns();
        }

        /* ── ② dt ───────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        float   dt  = (float)(now - last) * 1e-9f;
        if (dt > 0.1f) dt = 0.1f;
        last = now;

        /* ── ③ tick ─────────────────────────────────────────────────── */
        if (!paused) {
            obj_tick(&obj, dt, cols, rows);
            wave_x += dt * 0.75f;
            wave_y += dt * 0.48f;
            polar_tick(dt);
        }

        /* ── ④ fps counter (500 ms window) ─────────────────────────── */
        fps_cnt++; fps_acc += dt;
        if (fps_acc >= 0.5f) {
            fps     = (float)fps_cnt / fps_acc;
            fps_cnt = 0; fps_acc = 0.f;
        }

        /* ── ⑤ derive object in each grid's native coordinates ──────── */
        int obj_col  = px_to_cell_x(obj.px);
        int obj_row  = px_to_cell_y(obj.py);
        int obj_ring = 0, obj_sec = 0;
        int obj_hx   = 0, obj_hy  = 0;

        polar_find_obj_cell(cols / 2, rows / 2,
                            obj_col, obj_row, &obj_ring, &obj_sec);
        hex_screen_to_hex(obj_col, obj_row, &obj_hx, &obj_hy);

        /* ── ⑥ sleep before render ──────────────────────────────────── */
        clock_sleep_ns(frame_ns - (clock_ns() - now));

        /* ── ⑦ draw ─────────────────────────────────────────────────── */
        erase();

        if (mode == 0) rect_draw(rows, cols, theme, wave_x, wave_y, obj_col, obj_row);
        if (mode == 1) polar_draw(rows, cols, theme, obj_ring, obj_sec);
        if (mode == 2) hex_draw(rows, cols, theme, obj_hx, obj_hy);

        /* Draw '@' on top — always last so it is never hidden by the grid */
        if (obj_row >= 0 && obj_row < rows - 1 &&
            obj_col >= 0 && obj_col < cols) {
            attron(COLOR_PAIR(OBJ_PAIR) | A_BOLD);
            mvaddch(obj_row, obj_col, '@');
            attroff(COLOR_PAIR(OBJ_PAIR) | A_BOLD);
        }

        draw_hud(rows, cols, fps, mode, theme, paused,
                 obj_col, obj_row, obj_ring, obj_sec, obj_hx, obj_hy);

        screen_present();

        /* ── ⑧ input ────────────────────────────────────────────────── */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27:  g_run = 0;                        break;
        case 'p': case 'P':           paused = !paused;                  break;
        case 't': case 'T':           theme = (theme + 1) % N_THEMES;   break;
        case 'r': case 'R':           obj_init(&obj, cols, rows);        break;
        case '1':                     mode = 0;                          break;
        case '2':                     mode = 1;                          break;
        case '3':                     mode = 2;                          break;
        case KEY_UP:    obj_steer(&obj,  0.f, -1.f);                    break;
        case KEY_DOWN:  obj_steer(&obj,  0.f,  1.f);                    break;
        case KEY_LEFT:  obj_steer(&obj, -1.f,  0.f);                    break;
        case KEY_RIGHT: obj_steer(&obj,  1.f,  0.f);                    break;
        default: break;
        }
    }

    endwin();
    return 0;
}
