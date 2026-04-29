/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_hex_path.c — hex paths (line, ring walk, L-path) on a flat-top grid
 *
 * DEMO: A flat-top hex grid fills the screen.  Set a fixed endpoint A with
 *       the 'a' key, then move '@' to position B and press SPACE to stamp a
 *       path between them.  Three modes:  LINE (shortest hex path via lerp +
 *       cube_round),  RING (all hexes exactly N steps from cursor),  LPATH
 *       (Q-axis leg first, then R-axis leg).
 *
 * Study alongside: grids/hex_grids_placement/02_hex_pattern.c (pattern stamp),
 *                  grids/rect_grids_placement/03_path.c (Bresenham on rect)
 *
 * Section map:
 *   §1  config   — all tunable constants
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — color pairs: grid, cursor, endpoint A, path, HUD, hint
 *   §4  coords   — cube_round, hex_to_screen, hex_dist, angle_char
 *   §5  pool     — HPool: place, clear, draw
 *   §6  bgrid    — flat-top hex rasterizer (grid_draw)
 *   §7  paths    — HEX6 directions, hex_lerp_round, path_line, path_ring, path_lpath
 *   §8  scene    — Scene struct, scene_draw
 *   §9  screen   — ncurses init, HUD, cleanup
 *  §10  app      — signals, main loop
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
 *                  LPATH: Travel from A to B by changing Q first (keeping R
 *                  fixed at aR), then changing R (keeping Q fixed at bQ).
 *
 * Data-structure : HPool — flat array of HObj{Q,R,glyph}.  pool_place adds
 *                  or overwrites (no duplicates).  Path stamping is additive.
 *
 * Rendering      : Four-pass: grid background → stamped path objects →
 *                  endpoint A marker (if set) → cursor '@'.
 *
 * Performance    : path_line is O(N).  path_ring is O(6N).  path_lpath is
 *                  O(|dQ|+|dR|).  All are fast for any visible terminal.
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
 *   LPATH   — split the journey into two axis-aligned legs (change Q, then R).
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
 * LPATH: Go right/left first (Q-axis), then up/down (R-axis).  The corner
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
 *  3. cube_round(fq, fr, fs) → integer hex (Q, R).
 *  4. pool_place(Q, R, glyph).
 *
 * KEY FORMULAS
 * ────────────
 *  hex_lerp_round (hex line sample at parameter t ∈ [0,1]):
 *    fq = aQ + (bQ−aQ)×t
 *    fr = aR + (bR−aR)×t
 *    fs = −fq − fr
 *    (Q, R) = cube_round(fq, fr, fs)
 *
 *  hex_dist (step count for line, ring N):
 *    d = (|dQ| + |dR| + |dQ+dR|) / 2
 *
 *  hex_line cell count: N + 1 = hex_dist(A,B) + 1
 *
 *  Ring algorithm (radius N, centre = cursor):
 *    start: (cQ, cR) + N × HEX6[4]   [N steps in direction 4]
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
 *  • L-path corner: cell (bQ, aR) is included in the Q-axis leg (its
 *    last cell) and excluded from the R-axis leg (which starts at aR+dR).
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
 *    Q-leg: Q=0→2 at R=0 → (0,0),(1,0),(2,0)
 *    R-leg: R=1→2 at Q=2 → (2,1),(2,2)   [skips corner (2,0) already placed]
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(int64_t ns) {
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
#define PAIR_HUD       5
#define PAIR_HINT      6
#define PAIR_ENDPT_B   7   /* fixed endpoint B marker 'B'        */
#define PAIR_PREVIEW   8   /* live path preview before stamping  */

static void color_init(void) {
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,    COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_ENDPT_A, COLOR_WHITE,                COLOR_RED);
    init_pair(PAIR_PATH,    COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HUD,     COLOR_BLACK,                COLOR_CYAN);
    init_pair(PAIR_HINT,    COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_ENDPT_B, COLOR_WHITE,                COLOR_MAGENTA);
    init_pair(PAIR_PREVIEW, COLORS >= 256 ?  82 : COLOR_GREEN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  coords — cube_round, hex_to_screen, hex_dist, angle_char           */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cube_round — nearest integer hex to fractional cube position.
 * Round all three; fix the component with the LARGEST error to restore
 * Q+R+S=0.  See 01_hex_direct.c §4 for the full derivation.
 */
static void cube_round(double fq, double fr, double fs, int *Q, int *R) {
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) { *Q = -rr - rs; *R = rr; }
    else if (dr > ds)             { *Q = rq; *R = -rq - rs; }
    else                          { *Q = rq; *R = rr; }
}

/*
 * hex_to_screen — flat-top forward matrix + aspect correction.
 *
 * THE FORMULA:
 *   cx = size × 3/2 × Q,   cy = size × (√3/2 × Q  +  √3 × R)
 *   col = ox + round(cx / CELL_W),   row = oy + round(cy / CELL_H)
 */
static void hex_to_screen(double size, int Q, int R, int ox, int oy,
                           int *col, int *row) {
    double sq3 = sqrt(3.0);
    double cx  = size * 1.5 * (double)Q;
    double cy  = size * (sq3 * 0.5 * (double)Q + sq3 * (double)R);
    *col = ox + (int)round(cx / CELL_W);
    *row = oy + (int)round(cy / CELL_H);
}

/*
 * hex_dist — axial distance between two hexes.
 *
 * THE FORMULA:
 *   d = (|dQ| + |dR| + |dQ + dR|) / 2
 * Used by path_line to determine step count N = hex_dist(A, B).
 */
static int hex_dist(int q1, int r1, int q2, int r2) {
    int dq = q2 - q1, dr = r2 - r1;
    return (abs(dq) + abs(dr) + abs(dq + dr)) / 2;
}

static char angle_char(double theta) {
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)        return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                              return '-';
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int Q, R; char glyph; } HObj;
typedef struct { HObj items[MAX_OBJ]; int count; } HPool;

static void pool_place(HPool *p, int Q, int R, char glyph) {
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].Q == Q && p->items[i].R == R) {
            p->items[i].glyph = glyph; return;
        }
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (HObj){ Q, R, glyph };
}

static void pool_clear(HPool *p) { p->count = 0; }

static void pool_draw(const HPool *p, double size, int ox, int oy,
                       int rows, int cols) {
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        hex_to_screen(size, p->items[i].Q, p->items[i].R, ox, oy, &col, &row);
        if (col >= 0 && col < cols && row >= 0 && row < rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  bgrid — flat-top hex rasterizer                                    */
/* ═══════════════════════════════════════════════════════════════════════ */

static void grid_draw(int rows, int cols, double size, double border_w,
                       int cQ, int cR, int ox, int oy) {
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - border_w;
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;
            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;
            int Q, R;
            cube_round(fq, fr, fs, &Q, &R);
            double fS   = (double)(-Q - R);
            double dist = fabs(fq - (double)Q);
            double d2   = fabs(fr - (double)R);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;
            double cx    = size * 1.5 * (double)Q;
            double cy    = size * (sq3_2 * (double)Q + sq3 * (double)R);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);
            int on_cur = (Q == cQ && R == cR);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_GRID)   | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
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
 *   (Q, R) = cube_round(fq, fr, fs)
 *
 * WHY linear lerp works: the hex plane embeds in 3-D cube space as a flat
 * diagonal plane.  A straight line in cube space crosses exactly the same
 * hexes as the shortest path between A and B — no detour, no diagonal bias.
 */
static void hex_lerp_round(double aQ, double aR, double bQ, double bR,
                            double t, int *Q, int *R) {
    double fq = aQ + (bQ - aQ) * t;
    double fr = aR + (bR - aR) * t;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, Q, R);
}

/*
 * path_line — stamp the shortest hex path from (aQ,aR) to (bQ,bR).
 *
 * THE FORMULA:
 *   N = hex_dist(A, B)
 *   For i = 0..N: t = i/N → hex_lerp_round(A, B, t) → pool_place.
 *   Total cells = N + 1.
 */
static void path_line(HPool *pool, int aQ, int aR, int bQ, int bR,
                       char glyph) {
    int N = hex_dist(aQ, aR, bQ, bR);
    if (N == 0) { pool_place(pool, aQ, aR, glyph); return; }
    for (int i = 0; i <= N; i++) {
        double t = (double)i / (double)N;
        int Q, R;
        hex_lerp_round((double)aQ, (double)aR, (double)bQ, (double)bR, t, &Q, &R);
        pool_place(pool, Q, R, glyph);
    }
}

/*
 * path_ring — stamp all hexes exactly N steps from (cQ, cR).
 *
 * THE FORMULA:
 *   Start: (cQ, cR) + N × HEX6[4] = (cQ + 0×N, cR + (−1)×N) = (cQ, cR−N)
 *   Walk: for i in 0..5, take N steps in direction HEX6[i].
 *   Total cells: 6N.  Special case N=0: place cursor cell only.
 *
 * WHY this visits 6N cells without revisiting:
 *   Each leg of length N steps moves the walker along one edge of the ring.
 *   The 6 edges together complete a closed hexagonal loop.  The walker never
 *   enters the ring interior.
 */
static void path_ring(HPool *pool, int cQ, int cR, int N, char glyph) {
    if (N == 0) { pool_place(pool, cQ, cR, glyph); return; }
    int Q = cQ + HEX6[4][0] * N;
    int R = cR + HEX6[4][1] * N;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < N; j++) {
            pool_place(pool, Q, R, glyph);
            Q += HEX6[i][0];
            R += HEX6[i][1];
        }
    }
}

/*
 * path_lpath — Q-axis leg then R-axis leg from (aQ,aR) to (bQ,bR).
 *
 * THE FORMULA:
 *   Q-leg: step Q from aQ to bQ at R=aR  (includes both endpoints)
 *   R-leg: step R from aR+dR to bR at Q=bQ  (skips corner (bQ,aR))
 *   Total: |bQ−aQ| + |bR−aR| + 1 cells.
 *
 * WHY skip (bQ,aR) in the R-leg: pool_place already placed it in the Q-leg.
 * Double-counting would not duplicate (pool_place deduplicates), but skipping
 * is clearer and avoids the redundant search.
 */
static void path_lpath(HPool *pool, int aQ, int aR, int bQ, int bR,
                        char glyph) {
    /* Q-axis leg: traverse from (aQ,aR) to (bQ,aR) */
    int dQ = (bQ >= aQ) ? 1 : -1;
    for (int Q = aQ; Q != bQ + dQ; Q += dQ)
        pool_place(pool, Q, aR, glyph);
    /* R-axis leg: traverse from (bQ,aR+dR) to (bQ,bR) — skips corner */
    if (aR != bR) {
        int dR = (bR >= aR) ? 1 : -1;
        for (int R = aR + dR; R != bR + dR; R += dR)
            pool_place(pool, bQ, R, glyph);
    }
}

/* ── end §7 — for scatter placement strategies, read 04_hex_scatter.c §7 */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef enum { PATH_LINE=0, PATH_RING=1, PATH_LPATH=2, N_PATH=3 } PathMode;

static const char *PATH_NAME[N_PATH] = { "line", "ring", "lpath" };

typedef struct {
    double   hex_size, border_w;
    int      cQ, cR;       /* movable cursor */
    PathMode path_mode;
    int      ring_n;       /* radius for RING mode */
    int      has_a; int aQ, aR;   /* fixed endpoint A ('a' key) */
    int      has_b; int bQ, bR;   /* fixed endpoint B ('b' key) */
    HPool    pool;
} Scene;

static void scene_init(Scene *s) {
    s->hex_size  = HEX_SIZE_DEFAULT;
    s->border_w  = BORDER_W_DEFAULT;
    s->cQ = 0; s->cR = 0;
    s->path_mode = PATH_LINE;
    s->ring_n    = RING_N_DEFAULT;
    s->has_a = 0; s->aQ = 0; s->aR = 0;
    s->has_b = 0; s->bQ = 0; s->bR = 0;
    pool_clear(&s->pool);
}

/* endpoint_marker — render a letter marker at a fixed endpoint hex. */
static void endpoint_marker(double size, int Q, int R, int ox, int oy,
                             int rows, int cols, int pair, char label) {
    int col, row;
    hex_to_screen(size, Q, R, ox, oy, &col, &row);
    if (col >= 0 && col < cols && row >= 0 && row < rows - 1) {
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, col, label);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/*
 * path_preview_draw — show in bright green where SPACE would stamp.
 *
 * Uses the same algorithm as the stamp functions but draws '.' at each
 * hex centre instead of committing to the pool.  B falls back to cursor
 * when not explicitly fixed ('b' not yet pressed).
 */
static void path_preview_draw(const Scene *s, int ox, int oy,
                               int rows, int cols) {
    int bQ = s->has_b ? s->bQ : s->cQ;
    int bR = s->has_b ? s->bR : s->cR;

    attron(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
    switch (s->path_mode) {
    case PATH_LINE: {
        if (!s->has_a) break;
        int N = hex_dist(s->aQ, s->aR, bQ, bR);
        for (int i = 0; i <= N; i++) {
            double t = (N > 0) ? (double)i / (double)N : 0.0;
            int Q, R;
            hex_lerp_round((double)s->aQ, (double)s->aR,
                           (double)bQ,    (double)bR, t, &Q, &R);
            int col, row;
            hex_to_screen(s->hex_size, Q, R, ox, oy, &col, &row);
            if (col >= 0 && col < cols && row >= 0 && row < rows - 1)
                mvaddch(row, col, '.');
        }
        break;
    }
    case PATH_RING: {
        /* Ring preview always shows at cursor regardless of A/B */
        if (s->ring_n == 0) {
            int col, row;
            hex_to_screen(s->hex_size, s->cQ, s->cR, ox, oy, &col, &row);
            if (col >= 0 && col < cols && row >= 0 && row < rows - 1)
                mvaddch(row, col, '.');
        } else {
            int Q = s->cQ + HEX6[4][0] * s->ring_n;
            int R = s->cR + HEX6[4][1] * s->ring_n;
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < s->ring_n; j++) {
                    int col, row;
                    hex_to_screen(s->hex_size, Q, R, ox, oy, &col, &row);
                    if (col >= 0 && col < cols && row >= 0 && row < rows - 1)
                        mvaddch(row, col, '.');
                    Q += HEX6[i][0];
                    R += HEX6[i][1];
                }
            }
        }
        break;
    }
    case PATH_LPATH: {
        if (!s->has_a) break;
        int dQ = (bQ >= s->aQ) ? 1 : -1;
        for (int Q = s->aQ; Q != bQ + dQ; Q += dQ) {
            int col, row;
            hex_to_screen(s->hex_size, Q, s->aR, ox, oy, &col, &row);
            if (col >= 0 && col < cols && row >= 0 && row < rows - 1)
                mvaddch(row, col, '.');
        }
        if (s->aR != bR) {
            int dR = (bR >= s->aR) ? 1 : -1;
            for (int R = s->aR + dR; R != bR + dR; R += dR) {
                int col, row;
                hex_to_screen(s->hex_size, bQ, R, ox, oy, &col, &row);
                if (col >= 0 && col < cols && row >= 0 && row < rows - 1)
                    mvaddch(row, col, '.');
            }
        }
        break;
    }
    default: break;
    }
    attroff(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
}

static void scene_draw(const Scene *s, int rows, int cols) {
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, s->hex_size, s->border_w, s->cQ, s->cR, ox, oy);
    pool_draw(&s->pool, s->hex_size, ox, oy, rows, cols);
    path_preview_draw(s, ox, oy, rows, cols);
    if (s->has_a)
        endpoint_marker(s->hex_size, s->aQ, s->aR, ox, oy,
                        rows, cols, PAIR_ENDPT_A, 'A');
    if (s->has_b)
        endpoint_marker(s->hex_size, s->bQ, s->bR, ox, oy,
                        rows, cols, PAIR_ENDPT_B, 'B');
    /* cursor '@' last */
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx = s->hex_size * 1.5    * (double)s->cQ;
    double cy = s->hex_size * (sq3_2 * (double)s->cQ + sq3 * (double)s->cR);
    int ccol = ox + (int)(cx / CELL_W);
    int crow = oy + (int)(cy / CELL_H);
    if (ccol >= 0 && ccol < cols && crow >= 0 && crow < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(crow, ccol, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

static void scene_stamp(Scene *s) {
    /* B falls back to cursor when 'b' has not been pressed */
    int bQ = s->has_b ? s->bQ : s->cQ;
    int bR = s->has_b ? s->bR : s->cR;
    char glyph = (s->path_mode == PATH_LINE)  ? '*' :
                 (s->path_mode == PATH_RING)  ? 'o' : '+';
    switch (s->path_mode) {
    case PATH_LINE:
        if (s->has_a)
            path_line(&s->pool, s->aQ, s->aR, bQ, bR, glyph);
        break;
    case PATH_RING:
        path_ring(&s->pool, s->cQ, s->cR, s->ring_n, glyph);
        break;
    case PATH_LPATH:
        if (s->has_a)
            path_lpath(&s->pool, s->aQ, s->aR, bQ, bR, glyph);
        break;
    default: break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void cleanup(void) { endwin(); }

static void screen_init(void) {
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
    color_init();
    atexit(cleanup);
}

static void screen_hud(const Scene *s, int rows, int cols, double fps) {
    char buf[96];
    if (s->path_mode == PATH_RING) {
        snprintf(buf, sizeof buf,
                 " ring N:%d  obj:%d  %5.1f fps ",
                 s->ring_n, s->pool.count, fps);
    } else {
        snprintf(buf, sizeof buf,
                 " %s  A:%s B:%s  obj:%d  %5.1f fps ",
                 PATH_NAME[s->path_mode],
                 s->has_a ? "set" : "---",
                 s->has_b ? "set" : "---",
                 s->pool.count, fps);
    }
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " q:quit  arrows:move  a:set-A  b:set-B  spc:stamp  1:line  2:ring  3:lpath  +/-:N  C:clear ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t running = 1, need_resize = 0;
static void sig_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) running = 0;
    if (sig == SIGWINCH)                 need_resize = 1;
}

static const int HEX_DIR[4][2] = {
    { 0, -1 }, { 0, +1 }, {-1, 0}, {+1, 0}
};

int main(void) {
    signal(SIGINT, sig_handler); signal(SIGTERM, sig_handler);
    signal(SIGWINCH, sig_handler);
    screen_init();

    int rows, cols; getmaxyx(stdscr, rows, cols);
    Scene sc; scene_init(&sc);
    int64_t prev = clock_ns(); double fps = 60.0;

    while (running) {
        if (need_resize) {
            need_resize = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: running = 0; break;
            case 'r': sc.cQ = 0; sc.cR = 0; break;
            case 'C':
                pool_clear(&sc.pool);
                sc.has_a = 0; sc.has_b = 0; break;
            case 'a':
                sc.aQ = sc.cQ; sc.aR = sc.cR; sc.has_a = 1; break;
            case 'b':
                sc.bQ = sc.cQ; sc.bR = sc.cR; sc.has_b = 1; break;
            case ' ': scene_stamp(&sc); break;
            case '1': sc.path_mode = PATH_LINE;  break;
            case '2': sc.path_mode = PATH_RING;  break;
            case '3': sc.path_mode = PATH_LPATH; break;
            case KEY_UP:
                sc.cQ += HEX_DIR[0][0]; sc.cR += HEX_DIR[0][1]; break;
            case KEY_DOWN:
                sc.cQ += HEX_DIR[1][0]; sc.cR += HEX_DIR[1][1]; break;
            case KEY_LEFT:
                sc.cQ += HEX_DIR[2][0]; sc.cR += HEX_DIR[2][1]; break;
            case KEY_RIGHT:
                sc.cQ += HEX_DIR[3][0]; sc.cR += HEX_DIR[3][1]; break;
            case '+': case '=':
                if (sc.ring_n < RING_N_MAX) sc.ring_n++;
                break;
            case '-':
                if (sc.ring_n > RING_N_MIN) sc.ring_n--;
                break;
            }
        }
        int64_t now = clock_ns();
        int64_t dt  = now - prev; prev = now;
        if (dt > 0) fps = fps * 0.9 + 1e9 / (double)dt * 0.1;

        erase();
        scene_draw(&sc, rows, cols);
        screen_hud(&sc, rows, cols, fps);
        screen_present();
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
