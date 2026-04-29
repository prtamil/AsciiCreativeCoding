/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_hex_pattern.c — stamp hex patterns (disc, ring, row, col) on a flat-top grid
 *
 * DEMO: A flat-top hex grid fills the screen. Navigate '@' with arrow keys.
 *       Press 1–4 to select a pattern, +/- to grow/shrink it, then SPACE to
 *       stamp the highlighted pattern into the pool. Four pattern modes:
 *       disc (all hexes within radius N), ring (exactly radius N), row
 *       (same R, |dQ|≤N), col (same Q, |dR|≤N).
 *
 * Study alongside: grids/hex_grids_placement/01_hex_direct.c (direct toggle),
 *                  grids/rect_grids_placement/02_patterns.c (same idea on rect)
 *
 * Section map:
 *   §1  config   — all tunable constants
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — color pairs: grid, cursor, object, preview, HUD, hint
 *   §4  coords   — cube_round, hex_to_screen, hex_dist, angle_char
 *   §5  pool     — HPool: place, clear, draw
 *   §6  bgrid    — flat-top hex rasterizer (grid_draw)
 *   §7  patterns — PatMode, pat_test, pat_preview, pat_stamp
 *   §8  scene    — Scene struct, scene_draw
 *   §9  screen   — ncurses init, HUD, cleanup
 *  §10  app      — signals, main loop
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
 *                  cursor at (cQ, cR) and radius N, iterate the bounding box
 *                  dQ ∈ [−N, N], dR ∈ [−N, N] and call pat_test(mode, dQ, dR, N).
 *                  Cells that pass the predicate are added to HPool.  The
 *                  predicate uses hex_dist (axial distance) as the key metric.
 *
 * Data-structure : HPool — flat array of HObj{Q, R, glyph}.  pool_place adds
 *                  or overwrites a cell (no duplicates).  Clear is O(1).
 *
 * Rendering      : Four-pass: grid background → preview dots → placed objects
 *                  → cursor '@'.  Preview dots show the current pattern shape
 *                  before committing, so the user can preview before stamping.
 *
 * Performance    : pat_preview and pat_stamp iterate (2N+1)² cells max.  For
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
 * Every pattern is a PREDICATE on the axial displacement (dQ, dR) from the
 * cursor.  If pat_test(mode, dQ, dR, N) is true, the cell at (cQ+dQ, cR+dR)
 * belongs to the pattern.  All four modes use hex_dist — the single number
 * that captures "how many hex steps away" — as their primary selector:
 *
 *   DISC:  d ≤ N         — all hexes inside a circular region
 *   RING:  d == N        — the perimeter ring at exactly radius N
 *   ROW:   dR==0 && |dQ| ≤ N — same axial row (R constant)
 *   COL:   dQ==0 && |dR| ≤ N — same axial column (Q constant)
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine holding a rubber stamp shaped like the pattern.  Pressing SPACE
 * "stamps" it: every hex whose offset from the cursor satisfies the predicate
 * gets an object glyph written to HPool.  The preview ('p' key) shows the stamp
 * outline in real time as you move, so you can see the shape before committing.
 *
 * The bounding box [-N, N]² in dQ and dR is the worst-case search space.
 * For ROW and COL the bounding box is rectangular but the predicate filters it
 * down to a line.  For DISC the box contains ~22% extra cells outside the disc.
 * We always iterate the full box and let the predicate filter — simple and fast.
 *
 * DRAWING METHOD  (pattern preview and stamp)
 * ──────────────────────────────────────────
 *  Preview (each frame, if show_preview is on):
 *  1. For dR in [-N, N] and dQ in [-N, N]:
 *  2.   if pat_test(mode, dQ, dR, N) is false → skip
 *  3.   hex_to_screen(cQ+dQ, cR+dR) → (col, row)
 *  4.   draw '.' at (row, col) in PAIR_PREVIEW
 *
 *  Stamp (on SPACE key):
 *  1. Same loop as preview
 *  2.   pool_place(&pool, cQ+dQ, cR+dR, glyph[mode])
 *  Objects persist across cursor moves and resize events.
 *
 * KEY FORMULAS
 * ────────────
 *  hex_dist (axial distance):
 *    d = (|dQ| + |dR| + |dQ + dR|) / 2
 *
 *  Pattern predicates (all use d = hex_dist(0,0,dQ,dR)):
 *    DISC   d ≤ N         cell count = 3N² + 3N + 1
 *    RING   d == N        cell count = 6N  (1 for N=0)
 *    ROW    dR==0 && |dQ| ≤ N  cell count = 2N + 1
 *    COL    dQ==0 && |dR| ≤ N  cell count = 2N + 1
 *
 *  Bounding box iteration:
 *    dQ ∈ [-N, N],  dR ∈ [-N, N]
 *    Total candidates = (2N+1)²,  e.g. N=3 → 49 candidates
 *
 *  Glyph per mode (so stamped regions remain visually distinct):
 *    DISC='*'  RING='o'  ROW='-'  COL='|'
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
 * HOW TO VERIFY  (N=2, cursor at (cQ=0, cR=0), 80×24 terminal, size=14)
 * ─────────────
 *  DISC N=2: cells where d ≤ 2 → 3×4+6+1 = 19 cells.
 *    Bounding box [-2,2]²: 25 candidates.  6 fail (corners): e.g.
 *    (dQ=+2, dR=-2): dS=−(+2)−(−2)=0, d=(2+2+0)/2=2 ≤ 2 → PASSES. ✓
 *    (dQ=+2, dR=+1): d=(2+1+3)/2=3 > 2 → fails. ✓
 *
 *  RING N=2: cells where d == 2 → 12 cells.
 *    (dQ=+2, dR=0): d=(2+0+2)/2=2 → PASSES.
 *    (dQ=+1, dR=+1): d=(1+1+2)/2=2 → PASSES.
 *    (dQ=0,  dR=+2): d=(0+2+2)/2=2 → PASSES.
 *    (dQ=-1, dR=+2): d=(1+2+1)/2=2 → PASSES.
 *
 *  ROW N=2: dR=0, |dQ| ≤ 2 → 5 cells: dQ ∈ {-2,-1,0,+1,+2}.
 *
 *  COL N=2: dQ=0, |dR| ≤ 2 → 5 cells: dR ∈ {-2,-1,0,+1,+2}.
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
#define PAIR_OBJ       3
#define PAIR_PREVIEW   4   /* preview dots (stamp outline before committing) */
#define PAIR_HUD       5
#define PAIR_HINT      6

static void color_init(void) {
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,    COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_OBJ,     COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_PREVIEW, COLORS >= 256 ?  82 : COLOR_GREEN,   -1);
    init_pair(PAIR_HUD,     COLOR_BLACK,                COLOR_CYAN);
    init_pair(PAIR_HINT,    COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  coords — cube_round, hex_to_screen, hex_dist, angle_char           */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cube_round — nearest integer hex to fractional cube position.
 *
 * THE FORMULA:
 *   Round all three; fix the component with the LARGEST rounding error to
 *   restore Q+R+S=0. See 01_hex_direct.c §4 for the full derivation.
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
 *   dQ = q2-q1,  dR = r2-r1
 *   d  = (|dQ| + |dR| + |dQ + dR|) / 2
 *
 * This equals the cube distance max(|dQ|,|dR|,|dS|) where dS = -dQ-dR.
 * The average-of-three form avoids the max and works in axial directly.
 */
static int hex_dist(int q1, int r1, int q2, int r2) {
    int dq = q2 - q1, dr = r2 - r1;
    return (abs(dq) + abs(dr) + abs(dq + dr)) / 2;
}

/*
 * angle_char — radial angle → tangent ASCII line character.
 * Input: atan2(py-cy, px-cx) + π/2   (radial rotated to tangent direction).
 * Folded into [0,π): '-' '\\' '|' '/' '-'
 */
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

/*
 * pool_place — add or overwrite an object at (Q, R); no duplicates.
 *
 * If (Q, R) already exists in pool, update its glyph (stamp over old).
 * This means re-stamping a different pattern mode on the same cell updates
 * the glyph, showing the most recent stamp.
 */
static void pool_place(HPool *p, int Q, int R, char glyph) {
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].Q == Q && p->items[i].R == R) {
            p->items[i].glyph = glyph;
            return;
        }
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (HObj){ Q, R, glyph };
}

static void pool_clear(HPool *p) { p->count = 0; }

static void pool_draw(const HPool *p, double size, int ox, int oy,
                       int rows, int cols) {
    attron(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        hex_to_screen(size, p->items[i].Q, p->items[i].R, ox, oy, &col, &row);
        if (col >= 0 && col < cols && row >= 0 && row < rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
}

/* ── end §5 — to understand the background grid drawing, read §6 ──────── */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  bgrid — flat-top hex rasterizer                                    */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * grid_draw — per-pixel pipeline: inverse matrix → cube_round → border test.
 * See 01_hex_direct.c §6 for the full pipeline comment.
 */
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
/* §7  patterns — predicate, preview, stamp                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef enum { PAT_DISC=0, PAT_RING=1, PAT_ROW=2, PAT_COL=3, N_PAT=4 } PatMode;

/* One glyph per mode so stamped regions stay visually distinct.
 * Avoid '-' and '|' — they look identical to hex border characters. */
static const char PAT_GLYPH[N_PAT] = { '*', 'o', '=', ':' };
static const char *PAT_NAME[N_PAT] = { "disc", "ring", "row", "col" };

/*
 * pat_test — return 1 if displacement (dQ, dR) is in the pattern.
 *
 * THE FORMULA (predicate per mode):
 *   d = hex_dist(0, 0, dQ, dR)    ← distance from cursor to candidate
 *   DISC:  d ≤ N
 *   RING:  d == N
 *   ROW:   dR == 0  &&  |dQ| ≤ N
 *   COL:   dQ == 0  &&  |dR| ≤ N
 *
 * WHY bounding-box iteration + predicate instead of enumeration:
 *   Enumeration (e.g. walking the ring step-by-step) is faster but complex
 *   to implement and hard to extend.  The bounding-box approach is O((2N+1)²)
 *   — at most 289 cells for N=8 — which is negligible and easy to read.
 */
static int pat_test(PatMode mode, int dQ, int dR, int N) {
    int d = hex_dist(0, 0, dQ, dR);
    switch (mode) {
    case PAT_DISC: return d <= N;
    case PAT_RING: return d == N;
    case PAT_ROW:  return dR == 0 && abs(dQ) <= N;
    case PAT_COL:  return dQ == 0 && abs(dR) <= N;
    default:       return 0;
    }
}

/*
 * pat_overlay — draw the full hex border for every pattern cell in PAIR_PREVIEW.
 *
 * THE FORMULA:
 *   Same per-pixel inverse-matrix loop as grid_draw, but filtered:
 *     if pat_test(mode, Q-cQ, R-cR, N) is false → skip this pixel entirely
 *     if pixel is interior (dist < limit) → skip
 *     otherwise → draw border character in PAIR_PREVIEW | A_BOLD (bright green)
 *
 * WHY draw full borders rather than a single dot per hex:
 *   A single dot at the hex centre falls on an interior pixel and is drawn
 *   over by nothing — but it's tiny and dim.  Drawing the entire border ring
 *   of each pattern hex in bright green makes the pattern shape unmistakable:
 *   you see complete coloured outlines for every hex in the pattern.
 */
static void pat_overlay(PatMode mode, int N, int cQ, int cR,
                         double size, double border_w, int ox, int oy,
                         int rows, int cols) {
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - border_w;
    attron(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;
            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;
            int Q, R;
            cube_round(fq, fr, fs, &Q, &R);
            if (!pat_test(mode, Q - cQ, R - cR, N)) continue;
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
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
}

/*
 * pat_stamp — commit all pattern cells to the pool.
 *
 * THE FORMULA:
 *   Same iteration as pat_preview.
 *   pool_place(cQ+dQ, cR+dR, PAT_GLYPH[mode]) for each passing cell.
 *   Glyphs: disc='*' ring='o' row='=' col=':'  (never '-'/'|' which look like grid lines).
 *   pool_place deduplicates — re-stamping the same cell updates its glyph.
 */
static void pat_stamp(HPool *pool, PatMode mode, int N, int cQ, int cR) {
    char glyph = PAT_GLYPH[mode];
    for (int dR = -N; dR <= N; dR++) {
        for (int dQ = -N; dQ <= N; dQ++) {
            if (!pat_test(mode, dQ, dR, N)) continue;
            pool_place(pool, cQ + dQ, cR + dR, glyph);
        }
    }
}

/* ── end §7 — to understand how paths use hex_dist, read 03_hex_path.c §7 */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    double  hex_size, border_w;
    int     cQ, cR;
    PatMode pat_mode;
    int     pat_n;
    int     show_preview;
    HPool   pool;
} Scene;

static void scene_init(Scene *s) {
    s->hex_size     = HEX_SIZE_DEFAULT;
    s->border_w     = BORDER_W_DEFAULT;
    s->cQ = 0; s->cR = 0;
    s->pat_mode     = PAT_DISC;
    s->pat_n        = PAT_N_DEFAULT;
    s->show_preview = 1;
    pool_clear(&s->pool);
}

static void scene_draw(const Scene *s, int rows, int cols) {
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, s->hex_size, s->border_w, s->cQ, s->cR, ox, oy);
    pool_draw(&s->pool, s->hex_size, ox, oy, rows, cols);
    if (s->show_preview)
        pat_overlay(s->pat_mode, s->pat_n, s->cQ, s->cR,
                    s->hex_size, s->border_w, ox, oy, rows, cols);
    /* cursor last — always visible on top */
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
    snprintf(buf, sizeof buf,
             " %s N:%d  obj:%d  %5.1f fps  prev:%s ",
             PAT_NAME[s->pat_mode], s->pat_n, s->pool.count, fps,
             s->show_preview ? "on " : "off");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " q:quit  arrows:move  1-4:pattern  +/-:N  spc:stamp  p:preview  C:clear ");
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
            case 'C': pool_clear(&sc.pool); break;
            case 'p': sc.show_preview ^= 1; break;
            case '1': sc.pat_mode = PAT_DISC; break;
            case '2': sc.pat_mode = PAT_RING; break;
            case '3': sc.pat_mode = PAT_ROW;  break;
            case '4': sc.pat_mode = PAT_COL;  break;
            case ' ': pat_stamp(&sc.pool, sc.pat_mode, sc.pat_n, sc.cQ, sc.cR); break;
            case KEY_UP:
                sc.cQ += HEX_DIR[0][0]; sc.cR += HEX_DIR[0][1]; break;
            case KEY_DOWN:
                sc.cQ += HEX_DIR[1][0]; sc.cR += HEX_DIR[1][1]; break;
            case KEY_LEFT:
                sc.cQ += HEX_DIR[2][0]; sc.cR += HEX_DIR[2][1]; break;
            case KEY_RIGHT:
                sc.cQ += HEX_DIR[3][0]; sc.cR += HEX_DIR[3][1]; break;
            case '+': case '=':
                if (sc.pat_n < PAT_N_MAX) sc.pat_n++;
                break;
            case '-':
                if (sc.pat_n > PAT_N_MIN) sc.pat_n--;
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
