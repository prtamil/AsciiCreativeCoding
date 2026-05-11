/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_hex_path.c — hex paths (line, ring walk, L-path) on a flat-top grid
 *
 * DEMO: A flat-top hex grid fills the screen.  Set a fixed endpoint A with
 *       the 'a' key, then move '@' to position B and press SPACE to stamp a
 *       path between them.  Three modes:  LINE (shortest hex path via lerp +
 *       cube_round),  RING (all hexes exactly N steps from cursor),  LPATH
 *       (q-axis leg first, then r-axis leg).
 *
 * Study alongside: grids/hex_grids_placement/02_hex_pattern.c (pattern stamp),
 *                  grids/rect_grids_placement/03_path.c (Bresenham on rect)
 *
 * Section map:
 *   §1  config   — all tunable constants
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — color pairs: grid, cursor, endpoint A/B, path, HUD, hint
 *   §4  gridctx  — GridCtx + cube_round, ctx_to_screen, hex_dist, ctx_draw_bg
 *   §5  pool     — Pool: place, clear, draw
 *   §6  cursor   — Cursor + axial movement, cursor_draw
 *   §7  paths    — HEX6 directions, hex_lerp_round, path_line, path_ring, path_lpath
 *   §8  scene    — hud_draw + scene_draw
 *   §9  screen   — ncurses init / cleanup
 *   §10 app      — signals, main loop
 *
 * Keys:  arrows:move  a:set-A  b:set-B  spc:stamp-path  1-3:mode  +/-:ring-N
 *        C:clear  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids_placement/03_hex_path.c \
 *       -o 03_hex_path -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three hex path algorithms.
 *                  LINE: Lerp both axial axes from A to B in N = hex_dist(A,B)
 *                  steps, calling cube_round at each t=i/N to snap fractional
 *                  cube coordinates to the nearest integer hex.
 *                  RING: Start N steps in one direction from cursor; walk N
 *                  steps in each of the 6 hex directions in sequence.
 *                  LPATH: Travel from A to B by changing q first (keeping r
 *                  fixed at aR), then changing r (keeping q fixed at bQ).
 *
 * Data-structure : Pool — flat array of Obj{q,r,glyph}.  pool_place adds
 *                  or overwrites (no duplicates).  Path stamping is additive.
 *
 * GridContext    : GridCtx carries the hex-specific geometry (hex_size,
 *                  border_w, screen origin ox/oy, terminal extent rows/cols).
 *
 * Rendering      : Five-pass: grid background → stamped path objects →
 *                  live preview → endpoint A/B markers (if set) → cursor '@'.
 *
 * Performance    : path_line is O(N).  path_ring is O(6N).  path_lpath is
 *                  O(|dq|+|dr|).  All are fast for any visible terminal.
 *
 * References     :
 *   Hex line drawing      — https://www.redblobgames.com/grids/hexagons/#line-drawing
 *   Hex ring algorithm    — https://www.redblobgames.com/grids/hexagons/#rings
 *   Lerp + cube_round     — https://www.redblobgames.com/grids/hexagons/#rounding
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * All three algorithms share a common structure: visit a sequence of hex
 * addresses and call pool_place on each one.  The difference is HOW the
 * addresses are generated:
 *
 *   LINE    — sample a continuous straight line at N+1 equally-spaced points
 *             and round each to the nearest integer hex.
 *   RING    — start at a known ring entry point and take N steps in each of
 *             6 directions — the geometry guarantees exactly 6N cells are visited.
 *   LPATH   — split the journey into two axis-aligned legs (change q, then r).
 *             This is the hex analogue of a rook's L-move in chess.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * LINE: Imagine stretching a rubber band from A to B in pixel space and
 * sampling it at N equally-spaced intervals.  Each sample is snapped to the
 * nearest hex centre via cube_round.  The resulting hex sequence is the
 * hexagonal equivalent of Bresenham's line — no gaps, no jumps.
 *
 * RING: Think of the ring as a hexagonal clock.  Start at the 6 o'clock
 * position (N steps south from cursor).  Then walk clockwise: N steps in each
 * of 6 directions.  After 6×N steps, you're back at the start.  Each of the
 * 6×N steps visits exactly one hex on the ring (no revisits).
 *
 * LPATH: Go right/left first (q-axis), then up/down (r-axis).  The corner
 * cell (bQ, aR) is where the two legs meet; it is visited once.
 *
 * DRAWING METHOD  (LINE — the key algorithm)
 * ──────────────────────────────────────────
 *  1. N = hex_dist(A, B).  If N=0, place one cell and return.
 *  2. For i = 0, 1, ..., N:
 *       t = i / N                           ← parameter in [0.0, 1.0]
 *       fq = aQ + (bQ − aQ) × t
 *       fr = aR + (bR − aR) × t
 *       fs = −fq − fr
 *  3. cube_round(fq, fr, fs) → integer hex (q, r).
 *  4. pool_place(q, r, glyph).
 *
 * KEY FORMULAS
 * ────────────
 *  hex_lerp_round (hex line sample at parameter t ∈ [0,1]):
 *    fq = aQ + (bQ−aQ)×t
 *    fr = aR + (bR−aR)×t
 *    fs = −fq − fr
 *    (q, r) = cube_round(fq, fr, fs)
 *
 *  hex_dist (step count for line, ring N):
 *    d = (|dq| + |dr| + |dq+dr|) / 2
 *
 *  hex_line cell count: N + 1 = hex_dist(A,B) + 1
 *
 *  Ring algorithm (radius N, centre = cursor):
 *    start: (cur.q, cur.r) + N × HEX6[4]   [N steps in direction 4]
 *    walk: for i in 0..5: N steps in HEX6[i] direction
 *    ring cell count: 6N  (1 for N=0)
 *
 *  HEX6 — 6 axial neighbor directions (flat-top):
 *    0:(+1, 0)  1:(0, +1)  2:(−1,+1)  3:(−1,0)  4:(0,−1)  5:(+1,−1)
 *
 *  L-path cell count: |bQ − aQ| + |bR − aR| + 1
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • hex_lerp_round uses EXACT lerp — not epsilon-nudged like some
 *    implementations.  Cube_round handles the boundary correctly because
 *    it fixes the largest-error component, which at t=0.5 defaults to S.
 *    No nudge is needed; the algorithm is already correct.
 *
 *  • Ring at N=0: start = cursor + 0×dir = cursor; walk 0 steps in each
 *    direction → places 0 cells.  Handle N=0 as a single-cell special case.
 *
 *  • L-path when A == B: both legs have length 0 → one cell placed.
 *
 *  • L-path corner: cell (bQ, aR) is included in the q-axis leg (its
 *    last cell) and excluded from the r-axis leg (which starts at aR+dR).
 *    This prevents double-placing the corner.
 *
 * HOW TO VERIFY  (cursor at (0,0), B at (3,0))
 * ─────────────
 *  LINE, A=(0,0) B=(3,0): d=3, t=0,1/3,2/3,1.
 *    t=0:   fq=0, fr=0 → (0,0)
 *    t=1/3: fq=1, fr=0 → (1,0)
 *    t=2/3: fq=2, fr=0 → (2,0)
 *    t=1:   fq=3, fr=0 → (3,0)
 *    → 4 cells on the E-axis. ✓
 *
 *  RING, N=1, cursor=(0,0):
 *    start = (0,0) + 1×HEX6[4] = (0,0)+(0,-1) = (0,-1)
 *    dir 0=(+1,0): (0,-1)→(1,-1)     dir 1=(0,+1): (1,-1)→(1,0)
 *    dir 2=(-1,+1): (1,0)→(0,1)      dir 3=(-1,0): (0,1)→(-1,1)
 *    dir 4=(0,-1): (-1,1)→(-1,0)     dir 5=(+1,-1): (-1,0)→(0,-1)  [back to start]
 *    → 6 cells visited, each once. ✓
 *
 *  LPATH, A=(0,0) B=(2,2):
 *    q-leg: q=0→2 at r=0 → (0,0),(1,0),(2,0)
 *    r-leg: r=1→2 at q=2 → (2,1),(2,2)   [skips corner (2,0) already placed]
 *    → 5 cells = |2-0|+|2-0|+1. ✓
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

/* Maximum ring radius (ring cells = 6N, must fit in MAX_OBJ). */
#define RING_N_DEFAULT      3
#define RING_N_MIN          0
#define RING_N_MAX         40   /* 6×40=240 < MAX_OBJ=256 */

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
#define PAIR_ENDPT_A   3   /* fixed endpoint A marker 'A'        */
#define PAIR_PATH      4   /* stamped path glyphs                */
#define PAIR_HUD       5   /* status bar (yellow)                */
#define PAIR_HINT      6   /* key-hint footer (cyan)             */
#define PAIR_ENDPT_B   7   /* fixed endpoint B marker 'B'        */
#define PAIR_PREVIEW   8   /* live path preview before stamping  */

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,    COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_ENDPT_A, COLOR_WHITE,                COLOR_RED);
    init_pair(PAIR_PATH,    COLORS >= 256 ? 214 : COLOR_RED,     -1);
    init_pair(PAIR_HUD,     COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HINT,    COLORS >= 256 ?  51 : COLOR_CYAN,    -1);
    init_pair(PAIR_ENDPT_B, COLOR_WHITE,                COLOR_MAGENTA);
    init_pair(PAIR_PREVIEW, COLORS >= 256 ?  82 : COLOR_GREEN,   -1);
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
 * Round all three; fix the component with the LARGEST error to restore
 * q+r+s=0.  See 01_hex_direct.c §4 for the full derivation.
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
 *   d = (|dq| + |dr| + |dq + dr|) / 2
 * Used by path_line to determine step count N = hex_dist(A, B).
 */
static int hex_dist(int q1, int r1, int q2, int r2)
{
    int dq = q2 - q1, dr = r2 - r1;
    return (abs(dq) + abs(dr) + abs(dq + dr)) / 2;
}

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

static void pool_place(Pool *p, int q, int r, char glyph)
{
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].q == q && p->items[i].r == r) {
            p->items[i].glyph = glyph; return;
        }
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ q, r, glyph };
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        ctx_to_screen(g, p->items[i].q, p->items[i].r, &col, &row);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

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
/* §7  paths — hex_lerp_round, path_line, path_ring, path_lpath           */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * HEX6 — all 6 axial neighbor directions for flat-top hex grid.
 * Used by path_ring for the ring-walk step directions.
 *
 *   index 0:(+1, 0) E    index 1:(0, +1) SE   index 2:(-1,+1) SW
 *   index 3:(-1, 0) W    index 4:(0, -1) NW   index 5:(+1,-1) NE
 *
 * Ring walk starts N steps in direction 4 (NW), then walks E, SE, SW, W,
 * NW, NE in sequence — visiting all 6N ring cells exactly once.
 */
static const int HEX6[6][2] = {
    {+1,  0}, { 0, +1}, {-1, +1},
    {-1,  0}, { 0, -1}, {+1, -1},
};

/*
 * hex_lerp_round — single sample of the hex line at parameter t ∈ [0,1].
 *
 * THE FORMULA:
 *   fq = aQ + (bQ − aQ) × t
 *   fr = aR + (bR − aR) × t
 *   fs = −fq − fr
 *   (q, r) = cube_round(fq, fr, fs)
 *
 * WHY linear lerp works: the hex plane embeds in 3-D cube space as a flat
 * diagonal plane.  A straight line in cube space crosses exactly the same
 * hexes as the shortest path between A and B — no detour, no diagonal bias.
 */
static void hex_lerp_round(double aQ, double aR, double bQ, double bR,
                            double t, int *q, int *r)
{
    double fq = aQ + (bQ - aQ) * t;
    double fr = aR + (bR - aR) * t;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, q, r);
}

/*
 * path_line — stamp the shortest hex path from (aQ,aR) to (bQ,bR).
 *
 * THE FORMULA:
 *   N = hex_dist(A, B)
 *   For i = 0..N: t = i/N → hex_lerp_round(A, B, t) → pool_place.
 *   Total cells = N + 1.
 */
static void path_line(Pool *pool, int aQ, int aR, int bQ, int bR,
                       char glyph)
{
    int N = hex_dist(aQ, aR, bQ, bR);
    if (N == 0) { pool_place(pool, aQ, aR, glyph); return; }
    for (int i = 0; i <= N; i++) {
        double t = (double)i / (double)N;
        int q, r;
        hex_lerp_round((double)aQ, (double)aR, (double)bQ, (double)bR, t, &q, &r);
        pool_place(pool, q, r, glyph);
    }
}

/*
 * path_ring — stamp all hexes exactly N steps from (cur.q, cur.r).
 *
 * THE FORMULA:
 *   Start: (cur.q, cur.r) + N × HEX6[4] = (cur.q + 0×N, cur.r + (−1)×N)
 *        = (cur.q, cur.r−N)
 *   Walk: for i in 0..5, take N steps in direction HEX6[i].
 *   Total cells: 6N.  Special case N=0: place cursor cell only.
 *
 * WHY this visits 6N cells without revisiting:
 *   Each leg of length N steps moves the walker along one edge of the ring.
 *   The 6 edges together complete a closed hexagonal loop.  The walker never
 *   enters the ring interior.
 */
static void path_ring(Pool *pool, int cQ, int cR, int N, char glyph)
{
    if (N == 0) { pool_place(pool, cQ, cR, glyph); return; }
    int q = cQ + HEX6[4][0] * N;
    int r = cR + HEX6[4][1] * N;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < N; j++) {
            pool_place(pool, q, r, glyph);
            q += HEX6[i][0];
            r += HEX6[i][1];
        }
    }
}

/*
 * path_lpath — q-axis leg then r-axis leg from (aQ,aR) to (bQ,bR).
 *
 * THE FORMULA:
 *   q-leg: step q from aQ to bQ at r=aR  (includes both endpoints)
 *   r-leg: step r from aR+dR to bR at q=bQ  (skips corner (bQ,aR))
 *   Total: |bQ−aQ| + |bR−aR| + 1 cells.
 *
 * WHY skip (bQ,aR) in the r-leg: pool_place already placed it in the q-leg.
 * Double-counting would not duplicate (pool_place deduplicates), but skipping
 * is clearer and avoids the redundant search.
 */
static void path_lpath(Pool *pool, int aQ, int aR, int bQ, int bR,
                        char glyph)
{
    /* q-axis leg: traverse from (aQ,aR) to (bQ,aR) */
    int dQ = (bQ >= aQ) ? 1 : -1;
    for (int q = aQ; q != bQ + dQ; q += dQ)
        pool_place(pool, q, aR, glyph);
    /* r-axis leg: traverse from (bQ,aR+dR) to (bQ,bR) — skips corner */
    if (aR != bR) {
        int dR = (bR >= aR) ? 1 : -1;
        for (int r = aR + dR; r != bR + dR; r += dR)
            pool_place(pool, bQ, r, glyph);
    }
}

/* ── end §7 — for scatter placement strategies, read 04_hex_scatter.c §7 */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef enum { PATH_LINE=0, PATH_RING=1, PATH_LPATH=2, N_PATH=3 } PathMode;

static const char *PATH_NAME[N_PATH] = { "line", "ring", "lpath" };

typedef struct {
    PathMode path_mode;
    int      ring_n;       /* radius for RING mode */
    int      has_a; int aQ, aR;   /* fixed endpoint A ('a' key) */
    int      has_b; int bQ, bR;   /* fixed endpoint B ('b' key) */
} SceneCfg;

static void cfg_init(SceneCfg *cfg)
{
    cfg->path_mode = PATH_LINE;
    cfg->ring_n    = RING_N_DEFAULT;
    cfg->has_a = 0; cfg->aQ = 0; cfg->aR = 0;
    cfg->has_b = 0; cfg->bQ = 0; cfg->bR = 0;
}

/* endpoint_marker — render a letter marker at a fixed endpoint hex. */
static void endpoint_marker(const GridCtx *g, int q, int r,
                             int pair, char label)
{
    int col, row;
    ctx_to_screen(g, q, r, &col, &row);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1) {
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, col, label);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* preview_dot — draw a single '.' at hex (q, r) if visible.  Caller owns the
 * COLOR_PAIR attron/attroff so all dots in a preview share one attribute set. */
static void preview_dot(const GridCtx *g, int q, int r)
{
    int col, row;
    ctx_to_screen(g, q, r, &col, &row);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
        mvaddch(row, col, '.');
}

/* Mirror path_line's lerp+round walk; only A→B is previewed (no-op without A). */
static void preview_line_draw(const GridCtx *g, const SceneCfg *cfg,
                               int bQ, int bR)
{
    if (!cfg->has_a) return;
    int N = hex_dist(cfg->aQ, cfg->aR, bQ, bR);
    for (int i = 0; i <= N; i++) {
        double t = (N > 0) ? (double)i / (double)N : 0.0;
        int q, r;
        hex_lerp_round((double)cfg->aQ, (double)cfg->aR,
                       (double)bQ,      (double)bR, t, &q, &r);
        preview_dot(g, q, r);
    }
}

/* Mirror path_ring's HEX6 walk; ring is always centered on the live cursor. */
static void preview_ring_draw(const GridCtx *g, const Cursor *cur,
                               const SceneCfg *cfg)
{
    if (cfg->ring_n == 0) {
        preview_dot(g, cur->q, cur->r);
        return;
    }
    int q = cur->q + HEX6[4][0] * cfg->ring_n;
    int r = cur->r + HEX6[4][1] * cfg->ring_n;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < cfg->ring_n; j++) {
            preview_dot(g, q, r);
            q += HEX6[i][0];
            r += HEX6[i][1];
        }
    }
}

/* Mirror path_lpath's q-leg-then-r-leg walk; r-leg starts at aR+dR to skip the
 * shared corner cell (bQ, aR) already drawn by the q-leg. */
static void preview_lpath_draw(const GridCtx *g, const SceneCfg *cfg,
                                int bQ, int bR)
{
    if (!cfg->has_a) return;
    int dQ = (bQ >= cfg->aQ) ? 1 : -1;
    for (int q = cfg->aQ; q != bQ + dQ; q += dQ)
        preview_dot(g, q, cfg->aR);
    if (cfg->aR != bR) {
        int dR = (bR >= cfg->aR) ? 1 : -1;
        for (int r = cfg->aR + dR; r != bR + dR; r += dR)
            preview_dot(g, bQ, r);
    }
}

/*
 * path_preview_draw — show in bright green where SPACE would stamp.
 *
 * Uses the same algorithm as the stamp functions but draws '.' at each
 * hex centre instead of committing to the pool.  B falls back to cursor
 * when not explicitly fixed ('b' not yet pressed).
 */
static void path_preview_draw(const GridCtx *g, const Cursor *cur,
                               const SceneCfg *cfg)
{
    int bQ = cfg->has_b ? cfg->bQ : cur->q;
    int bR = cfg->has_b ? cfg->bR : cur->r;

    attron(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
    switch (cfg->path_mode) {
        case PATH_LINE:  preview_line_draw  (g, cfg, bQ, bR); break;
        case PATH_RING:  preview_ring_draw  (g, cur, cfg);    break;
        case PATH_LPATH: preview_lpath_draw (g, cfg, bQ, bR); break;
        default: break;
    }
    attroff(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
}

static void scene_stamp(Pool *pool, const Cursor *cur, const SceneCfg *cfg)
{
    /* B falls back to cursor when 'b' has not been pressed */
    int bQ = cfg->has_b ? cfg->bQ : cur->q;
    int bR = cfg->has_b ? cfg->bR : cur->r;
    char glyph = (cfg->path_mode == PATH_LINE)  ? '*' :
                 (cfg->path_mode == PATH_RING)  ? 'o' : '+';
    switch (cfg->path_mode) {
    case PATH_LINE:
        if (cfg->has_a)
            path_line(pool, cfg->aQ, cfg->aR, bQ, bR, glyph);
        break;
    case PATH_RING:
        path_ring(pool, cur->q, cur->r, cfg->ring_n, glyph);
        break;
    case PATH_LPATH:
        if (cfg->has_a)
            path_lpath(pool, cfg->aQ, cfg->aR, bQ, bR, glyph);
        break;
    default: break;
    }
}

/* Bright bold yellow fps readout (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Pool *p, const SceneCfg *cfg,
                      double fps)
{
    char buf[96];
    if (cfg->path_mode == PATH_RING) {
        snprintf(buf, sizeof buf,
                 " ring N:%d  obj:%d  %5.1f fps ",
                 cfg->ring_n, p->count, fps);
    } else {
        snprintf(buf, sizeof buf,
                 " %s  A:%s B:%s  obj:%d  %5.1f fps ",
                 PATH_NAME[cfg->path_mode],
                 cfg->has_a ? "set" : "---",
                 cfg->has_b ? "set" : "---",
                 p->count, fps);
    }
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  a:set-A  b:set-B  spc:stamp  1:line  2:ring  3:lpath  +/-:N  C:clear  q/ESC:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                        const SceneCfg *cfg, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    pool_draw(p, g);
    path_preview_draw(g, cur, cfg);
    if (cfg->has_a)
        endpoint_marker(g, cfg->aQ, cfg->aR, PAIR_ENDPT_A, 'A');
    if (cfg->has_b)
        endpoint_marker(g, cfg->bQ, cfg->bR, PAIR_ENDPT_B, 'B');
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
    SceneCfg cfg; cfg_init(&cfg);

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
            case 'C':
                pool_clear(&pool);
                cfg.has_a = 0; cfg.has_b = 0; break;
            case 'a':
                cfg.aQ = cur.q; cfg.aR = cur.r; cfg.has_a = 1; break;
            case 'b':
                cfg.bQ = cur.q; cfg.bR = cur.r; cfg.has_b = 1; break;
            case ' ': scene_stamp(&pool, &cur, &cfg); break;
            case '1': cfg.path_mode = PATH_LINE;  break;
            case '2': cfg.path_mode = PATH_RING;  break;
            case '3': cfg.path_mode = PATH_LPATH; break;
            case KEY_UP:    cursor_move(&cur, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (cfg.ring_n < RING_N_MAX) cfg.ring_n++;
                break;
            case '-':
                if (cfg.ring_n > RING_N_MIN) cfg.ring_n--;
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
