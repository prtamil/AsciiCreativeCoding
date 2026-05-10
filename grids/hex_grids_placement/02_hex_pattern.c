/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_hex_pattern.c — stamp hex patterns (disc, ring, row, col) on a flat-top grid
 *
 * DEMO: A flat-top hex grid fills the screen. Navigate '@' with arrow keys.
 *       Press 1–4 to select a pattern, +/- to grow/shrink it, then SPACE to
 *       stamp the highlighted pattern into the pool. Four pattern modes:
 *       disc (all hexes within radius N), ring (exactly radius N), row
 *       (same r, |dq|≤N), col (same q, |dr|≤N).
 *
 * Study alongside: grids/hex_grids_placement/01_hex_direct.c (direct toggle),
 *                  grids/rect_grids_placement/02_patterns.c (same idea on rect)
 *
 * Section map:
 *   §1  config   — all tunable constants
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — color pairs: grid, cursor, object, preview, HUD, hint
 *   §4  gridctx  — GridCtx + cube_round, ctx_to_screen, hex_dist, ctx_draw_bg
 *   §5  pool     — Pool: place, clear, draw
 *   §6  cursor   — Cursor + axial movement, cursor_draw
 *   §7  patterns — PatMode, pat_test, pat_overlay, pat_stamp
 *   §8  scene    — hud_draw + scene_draw
 *   §9  screen   — ncurses init / cleanup
 *   §10 app      — signals, main loop
 *
 * Keys:  arrows:move  1-4:pattern  +/-:radius  spc:stamp  p:preview
 *        C:clear  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids_placement/02_hex_pattern.c \
 *       -o 02_hex_pattern -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pattern-fill placement via predicate filtering.  For a
 *                  cursor at (cur.q, cur.r) and radius N, iterate the bounding
 *                  box dq ∈ [−N, N], dr ∈ [−N, N] and call pat_test(mode,
 *                  dq, dr, N).  Cells that pass the predicate are added to
 *                  Pool.  The predicate uses hex_dist (axial distance) as the
 *                  key metric.
 *
 * Data-structure : Pool — flat array of Obj{q, r, glyph}.  pool_place adds
 *                  or overwrites a cell (no duplicates).  Clear is O(1).
 *
 * GridContext    : GridCtx carries the hex-specific geometry (hex_size,
 *                  border_w, screen origin ox/oy, terminal extent rows/cols).
 *
 * Rendering      : Four-pass: grid background → preview overlay → placed
 *                  objects → cursor '@'.  Preview shows the current pattern
 *                  shape before committing, so the user can preview before
 *                  stamping.
 *
 * Performance    : pat_overlay and pat_stamp iterate (2N+1)² cells max.  For
 *                  N=8 that is 289 calls — negligible inside the 60 fps budget.
 *
 * References     :
 *   Red Blob Games hex grid algorithms — https://www.redblobgames.com/grids/hexagons/
 *   Hex disc / ring / line             — https://www.redblobgames.com/grids/hexagons/#range
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every pattern is a PREDICATE on the axial displacement (dq, dr) from the
 * cursor.  If pat_test(mode, dq, dr, N) is true, the cell at (cur.q+dq,
 * cur.r+dr) belongs to the pattern.  All four modes use hex_dist — the single
 * number that captures "how many hex steps away" — as their primary selector:
 *
 *   DISC:  d ≤ N         — all hexes inside a circular region
 *   RING:  d == N        — the perimeter ring at exactly radius N
 *   ROW:   dr==0 && |dq| ≤ N — same axial row (r constant)
 *   COL:   dq==0 && |dr| ≤ N — same axial column (q constant)
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine holding a rubber stamp shaped like the pattern.  Pressing SPACE
 * "stamps" it: every hex whose offset from the cursor satisfies the predicate
 * gets an object glyph written to Pool.  The preview ('p' key) shows the stamp
 * outline in real time as you move, so you can see the shape before committing.
 *
 * The bounding box [-N, N]² in dq and dr is the worst-case search space.
 * For ROW and COL the bounding box is rectangular but the predicate filters it
 * down to a line.  For DISC the box contains ~22% extra cells outside the disc.
 * We always iterate the full box and let the predicate filter — simple and fast.
 *
 * DRAWING METHOD  (pattern preview and stamp)
 * ──────────────────────────────────────────
 *  Preview (each frame, if show_preview is on):
 *  1. For dr in [-N, N] and dq in [-N, N]:
 *  2.   if pat_test(mode, dq, dr, N) is false → skip
 *  3.   ctx_to_screen(cur.q+dq, cur.r+dr) → (col, row)
 *  4.   draw '.' at (row, col) in PAIR_PREVIEW
 *
 *  Stamp (on SPACE key):
 *  1. Same loop as preview
 *  2.   pool_place(&pool, cur.q+dq, cur.r+dr, glyph[mode])
 *  Objects persist across cursor moves and resize events.
 *
 * KEY FORMULAS
 * ────────────
 *  hex_dist (axial distance):
 *    d = (|dq| + |dr| + |dq + dr|) / 2
 *
 *  Pattern predicates (all use d = hex_dist(0,0,dq,dr)):
 *    DISC   d ≤ N         cell count = 3N² + 3N + 1
 *    RING   d == N        cell count = 6N  (1 for N=0)
 *    ROW    dr==0 && |dq| ≤ N  cell count = 2N + 1
 *    COL    dq==0 && |dr| ≤ N  cell count = 2N + 1
 *
 *  Bounding box iteration:
 *    dq ∈ [-N, N],  dr ∈ [-N, N]
 *    Total candidates = (2N+1)²,  e.g. N=3 → 49 candidates
 *
 *  Glyph per mode (so stamped regions remain visually distinct):
 *    DISC='*'  RING='o'  ROW='='  COL=':'
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • N=0: DISC and RING both yield {(0,0)} — one cell at the cursor.
 *    ROW and COL also yield one cell.  Stamping N=0 always places one object.
 *
 *  • RING at N=0: hex_dist(0,0,0,0)=0 == 0 = N → one cell.  6×0=0 formula
 *    fails, but the predicate (d==N) is still correct.
 *
 *  • MAX_OBJ cap: pool_place silently drops objects when pool is full.
 *    For N=8 disc: 3×64+3×8+1=217 cells — just fits in MAX_OBJ=256.
 *    For N≥9 disc: may overflow.  The cap prevents buffer overrun.
 *
 *  • Preview draws OVER pool objects each frame. This is intentional: the
 *    user needs to see the stamp shape at the current cursor, not the old data.
 *
 * HOW TO VERIFY  (N=2, cursor at (cur.q=0, cur.r=0), 80×24 terminal, size=14)
 * ─────────────
 *  DISC N=2: cells where d ≤ 2 → 3×4+6+1 = 19 cells.
 *    Bounding box [-2,2]²: 25 candidates.  6 fail (corners): e.g.
 *    (dq=+2, dr=-2): ds=−(+2)−(−2)=0, d=(2+2+0)/2=2 ≤ 2 → PASSES. ✓
 *    (dq=+2, dr=+1): d=(2+1+3)/2=3 > 2 → fails. ✓
 *
 *  RING N=2: cells where d == 2 → 12 cells.
 *    (dq=+2, dr=0): d=(2+0+2)/2=2 → PASSES.
 *    (dq=+1, dr=+1): d=(1+1+2)/2=2 → PASSES.
 *    (dq=0,  dr=+2): d=(0+2+2)/2=2 → PASSES.
 *    (dq=-1, dr=+2): d=(1+2+1)/2=2 → PASSES.
 *
 *  ROW N=2: dr=0, |dq| ≤ 2 → 5 cells: dq ∈ {-2,-1,0,+1,+2}.
 *
 *  COL N=2: dq=0, |dr| ≤ 2 → 5 cells: dr ∈ {-2,-1,0,+1,+2}.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ncurses.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define CELL_W              2
#define CELL_H              4

#define HEX_SIZE_DEFAULT   14.0
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35

/* Pattern radius range. N=8 disc = 217 cells; must fit in MAX_OBJ. */
#define PAT_N_DEFAULT       3
#define PAT_N_MIN           0
#define PAT_N_MAX           8

#define MAX_OBJ            256
#define FRAME_NS    16666667LL

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA      0.05

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

#define PAIR_GRID      1
#define PAIR_CURSOR    2
#define PAIR_OBJ       3
#define PAIR_PREVIEW   4   /* preview overlay (stamp shape before committing) */
#define PAIR_HUD       5   /* status bar (yellow)  */
#define PAIR_HINT      6   /* key-hint footer (cyan) */

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,    COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_OBJ,     COLORS >= 256 ? 214 : COLOR_RED,     -1);
    init_pair(PAIR_PREVIEW, COLORS >= 256 ?  82 : COLOR_GREEN,   -1);
    init_pair(PAIR_HUD,     COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HINT,    COLORS >= 256 ?  51 : COLOR_CYAN,    -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx — GridCtx + cube_round, ctx_to_screen, hex_dist, ctx_draw_bg */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    rows, cols;
    double hex_size;
    double border_w;
    int    ox, oy;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols,
                      double hex_size, double border_w)
{
    g->rows = rows; g->cols = cols;
    g->hex_size = hex_size;
    g->border_w = border_w;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
}

/*
 * cube_round — nearest integer hex to fractional cube position.
 *
 * THE FORMULA:
 *   Round all three; fix the component with the LARGEST rounding error to
 *   restore q+r+s=0. See 01_hex_direct.c §4 for the full derivation.
 */
static void cube_round(double fq, double fr, double fs, int *q, int *r)
{
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) { *q = -rr - rs; *r = rr; }
    else if (dr > ds)             { *q = rq; *r = -rq - rs; }
    else                          { *q = rq; *r = rr; }
}

/*
 * ctx_to_screen — flat-top forward matrix + aspect correction.
 *
 * THE FORMULA:
 *   cx = size × 3/2 × q,   cy = size × (√3/2 × q  +  √3 × r)
 *   col = ox + round(cx / CELL_W),   row = oy + round(cy / CELL_H)
 */
static void ctx_to_screen(const GridCtx *g, int q, int r, int *col, int *row)
{
    double sq3 = sqrt(3.0);
    double cx  = g->hex_size * 1.5 * (double)q;
    double cy  = g->hex_size * (sq3 * 0.5 * (double)q + sq3 * (double)r);
    *col = g->ox + (int)round(cx / CELL_W);
    *row = g->oy + (int)round(cy / CELL_H);
}

/*
 * hex_dist — axial distance between two hexes.
 *
 * THE FORMULA:
 *   dq = q2-q1,  dr = r2-r1
 *   d  = (|dq| + |dr| + |dq + dr|) / 2
 *
 * This equals the cube distance max(|dq|,|dr|,|ds|) where ds = -dq-dr.
 * The average-of-three form avoids the max and works in axial directly.
 */
static int hex_dist(int q1, int r1, int q2, int r2)
{
    int dq = q2 - q1, dr = r2 - r1;
    return (abs(dq) + abs(dr) + abs(dq + dr)) / 2;
}

/*
 * angle_char — radial angle → tangent ASCII line character.
 * Input: atan2(py-cy, px-cx) + π/2   (radial rotated to tangent direction).
 * Folded into [0,π): '-' '\\' '|' '/' '-'
 */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)        return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                              return '-';
}

/*
 * ctx_draw_bg — per-pixel pipeline: inverse matrix → cube_round → border test.
 * See 01_hex_direct.c §4 for the full pipeline comment.
 */
static void ctx_draw_bg(const GridCtx *g, int curq, int curr)
{
    double size  = g->hex_size;
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - g->border_w;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * CELL_W;
            double py = (double)(row - g->oy) * CELL_H;
            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;
            int q, r;
            cube_round(fq, fr, fs, &q, &r);
            double fS   = (double)(-q - r);
            double dist = fabs(fq - (double)q);
            double d2   = fabs(fr - (double)r);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;
            double cx    = size * 1.5 * (double)q;
            double cy    = size * (sq3_2 * (double)q + sq3 * (double)r);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);
            int on_cur = (q == curq && r == curr);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_GRID)   | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int q, r; char glyph; } Obj;
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

/*
 * pool_place — add or overwrite an object at (q, r); no duplicates.
 *
 * If (q, r) already exists in pool, update its glyph (stamp over old).
 * This means re-stamping a different pattern mode on the same cell updates
 * the glyph, showing the most recent stamp.
 */
static void pool_place(Pool *p, int q, int r, char glyph)
{
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].q == q && p->items[i].r == r) {
            p->items[i].glyph = glyph;
            return;
        }
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ q, r, glyph };
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        ctx_to_screen(g, p->items[i].q, p->items[i].r, &col, &row);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
}

/* ── end §5 — to understand cursor placement, read §6 ─────────────────── */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor — axial cursor and movement                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur) { cur->q = 0; cur->r = 0; }

static const int HEX_DIR[4][2] = {
    { 0, -1 }, { 0, +1 }, {-1, 0}, {+1, 0}
};

static void cursor_move(Cursor *cur, int dq, int dr)
{
    cur->q += dq; cur->r += dr;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx    = g->hex_size * 1.5    * (double)cur->q;
    double cy    = g->hex_size * (sq3_2 * (double)cur->q + sq3 * (double)cur->r);
    int col = g->ox + (int)(cx / CELL_W);
    int row = g->oy + (int)(cy / CELL_H);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  patterns — predicate, preview, stamp                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef enum { PAT_DISC=0, PAT_RING=1, PAT_ROW=2, PAT_COL=3, N_PAT=4 } PatMode;

/* One glyph per mode so stamped regions stay visually distinct.
 * Avoid '-' and '|' — they look identical to hex border characters. */
static const char PAT_GLYPH[N_PAT] = { '*', 'o', '=', ':' };
static const char *PAT_NAME[N_PAT] = { "disc", "ring", "row", "col" };

/*
 * pat_test — return 1 if displacement (dq, dr) is in the pattern.
 *
 * THE FORMULA (predicate per mode):
 *   d = hex_dist(0, 0, dq, dr)    ← distance from cursor to candidate
 *   DISC:  d ≤ N
 *   RING:  d == N
 *   ROW:   dr == 0  &&  |dq| ≤ N
 *   COL:   dq == 0  &&  |dr| ≤ N
 *
 * WHY bounding-box iteration + predicate instead of enumeration:
 *   Enumeration (e.g. walking the ring step-by-step) is faster but complex
 *   to implement and hard to extend.  The bounding-box approach is O((2N+1)²)
 *   — at most 289 cells for N=8 — which is negligible and easy to read.
 */
static int pat_test(PatMode mode, int dq, int dr, int N)
{
    int d = hex_dist(0, 0, dq, dr);
    switch (mode) {
    case PAT_DISC: return d <= N;
    case PAT_RING: return d == N;
    case PAT_ROW:  return dr == 0 && abs(dq) <= N;
    case PAT_COL:  return dq == 0 && abs(dr) <= N;
    default:       return 0;
    }
}

/*
 * pat_overlay — draw the full hex border for every pattern cell in PAIR_PREVIEW.
 *
 * THE FORMULA:
 *   Same per-pixel inverse-matrix loop as ctx_draw_bg, but filtered:
 *     if pat_test(mode, q-cur.q, r-cur.r, N) is false → skip this pixel
 *     if pixel is interior (dist < limit) → skip
 *     otherwise → draw border character in PAIR_PREVIEW | A_BOLD (bright green)
 *
 * WHY draw full borders rather than a single dot per hex:
 *   A single dot at the hex centre falls on an interior pixel and is drawn
 *   over by nothing — but it's tiny and dim.  Drawing the entire border ring
 *   of each pattern hex in bright green makes the pattern shape unmistakable:
 *   you see complete coloured outlines for every hex in the pattern.
 */
static void pat_overlay(const GridCtx *g, PatMode mode, int N,
                         int curq, int curr)
{
    double size  = g->hex_size;
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - g->border_w;
    attron(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * CELL_W;
            double py = (double)(row - g->oy) * CELL_H;
            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;
            int q, r;
            cube_round(fq, fr, fs, &q, &r);
            if (!pat_test(mode, q - curq, r - curr, N)) continue;
            double fS   = (double)(-q - r);
            double dist = fabs(fq - (double)q);
            double d2   = fabs(fr - (double)r);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;
            double cx    = size * 1.5 * (double)q;
            double cy    = size * (sq3_2 * (double)q + sq3 * (double)r);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
}

/*
 * pat_stamp — commit all pattern cells to the pool.
 *
 * THE FORMULA:
 *   Same iteration as pat_overlay.
 *   pool_place(cur.q+dq, cur.r+dr, PAT_GLYPH[mode]) for each passing cell.
 *   Glyphs: disc='*' ring='o' row='=' col=':'  (never '-'/'|' which look like grid lines).
 *   pool_place deduplicates — re-stamping the same cell updates its glyph.
 */
static void pat_stamp(Pool *pool, PatMode mode, int N, int curq, int curr)
{
    char glyph = PAT_GLYPH[mode];
    for (int dr = -N; dr <= N; dr++) {
        for (int dq = -N; dq <= N; dq++) {
            if (!pat_test(mode, dq, dr, N)) continue;
            pool_place(pool, curq + dq, curr + dr, glyph);
        }
    }
}

/* ── end §7 — to understand how paths use hex_dist, read 03_hex_path.c §7 */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    PatMode pat_mode;
    int     pat_n;
    int     show_preview;
} SceneCfg;

/* Bright bold yellow fps readout (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Pool *p, const SceneCfg *cfg,
                      double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " %s N:%d  obj:%d  %5.1f fps  prev:%s ",
             PAT_NAME[cfg->pat_mode], cfg->pat_n, p->count, fps,
             cfg->show_preview ? "on " : "off");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  1-4:pattern  +/-:N  spc:stamp  p:preview  C:clear  r:reset  q/ESC:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                        const SceneCfg *cfg, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    pool_draw(p, g);
    if (cfg->show_preview)
        pat_overlay(g, cfg->pat_mode, cfg->pat_n, cur->q, cur->r);
    cursor_draw(cur, g);
    hud_draw(g, p, cfg, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
    color_init();
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;

static void on_signal(int sig)
{
    if (sig == SIGINT || sig == SIGTERM) g_running = 0;
    if (sig == SIGWINCH)                 g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    screen_init();

    int rows, cols; getmaxyx(stdscr, rows, cols);
    GridCtx g;   ctx_init(&g, rows, cols, HEX_SIZE_DEFAULT, BORDER_W_DEFAULT);
    Cursor  cur; cursor_reset(&cur);
    Pool    pool; pool_clear(&pool);
    SceneCfg cfg = { PAT_DISC, PAT_N_DEFAULT, 1 };

    double fps = 60.0;
    int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            ctx_init(&g, rows, cols, g.hex_size, g.border_w);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'r': cursor_reset(&cur); break;
            case 'C': pool_clear(&pool); break;
            case 'p': cfg.show_preview ^= 1; break;
            case '1': cfg.pat_mode = PAT_DISC; break;
            case '2': cfg.pat_mode = PAT_RING; break;
            case '3': cfg.pat_mode = PAT_ROW;  break;
            case '4': cfg.pat_mode = PAT_COL;  break;
            case ' ': pat_stamp(&pool, cfg.pat_mode, cfg.pat_n, cur.q, cur.r); break;
            case KEY_UP:    cursor_move(&cur, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (cfg.pat_n < PAT_N_MAX) cfg.pat_n++;
                break;
            case '-':
                if (cfg.pat_n > PAT_N_MIN) cfg.pat_n--;
                break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;

        scene_draw(&g, &pool, &cur, &cfg, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
