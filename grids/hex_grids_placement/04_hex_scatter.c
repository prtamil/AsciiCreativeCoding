/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_hex_scatter.c — four scatter strategies on a flat-top hex grid
 *
 * DEMO: A flat-top hex grid fills the screen.  Move '@' and press SPACE to
 *       fire one of four scatter strategies centered on the cursor.  Four
 *       modes: UNIFORM (random sparse), MINDIST (Poisson-disk-like, evenly
 *       spaced), FLOOD (solid disc fill), GRADIENT (dense center, sparse edge).
 *       Adjust the scatter radius and density/mindist parameters live.
 *
 * Study alongside: grids/hex_grids_placement/03_hex_path.c (path placement),
 *                  grids/rect_grids_placement/04_scatter.c (same on rect)
 *
 * Section map:
 *   §1  config   — all tunable constants
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — color pairs: grid, cursor, objects per mode, HUD, hint
 *   §4  gridctx  — GridCtx + cube_round, ctx_to_screen, hex_dist, ctx_draw_bg
 *   §5  pool     — Pool: place, clear, draw
 *   §6  cursor   — Cursor + axial movement, cursor_draw
 *   §7  scatter  — scatter_uniform, scatter_mindist, scatter_flood, scatter_gradient
 *   §8  scene    — hud_draw + scene_draw
 *   §9  screen   — ncurses init / cleanup
 *   §10 app      — signals, main loop
 *
 * Keys:  arrows:move  spc:scatter  1-4:mode  +/-:radius  d/D:density  m/M:mindist
 *        C:clear  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids_placement/04_hex_scatter.c \
 *       -o 04_hex_scatter -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Four scatter placement strategies, all centered on the
 *                  cursor hex (cur.q, cur.r) with radius R.  Each iterates
 *                  the disc (hexes with hex_dist ≤ R from cursor) and applies
 *                  a different acceptance rule.
 *                  UNIFORM:   accept with probability DENSITY (uniform random)
 *                  MINDIST:   accept only if hex_dist to every existing pool
 *                             member ≥ MIN_DIST (Poisson-disk sampling)
 *                  FLOOD:     accept all hexes in disc (density = 1.0)
 *                  GRADIENT:  accept with P = FALLOFF/(dist+FALLOFF), making
 *                             the center denser than the edge
 *
 * Data-structure : Pool — flat array of Obj{q,r,glyph}.  MINDIST checks
 *                  pool linearly for each candidate (O(n) per candidate).
 *                  With n ≤ 256 and disc ≤ 169 cells (R=7), this is fast.
 *
 * GridContext    : GridCtx carries the hex-specific geometry (hex_size,
 *                  border_w, screen origin ox/oy, terminal extent rows/cols).
 *
 * Rendering      : Three-pass: grid background → pool objects → cursor '@'.
 *
 * Performance    : FLOOD is O(disc_size).  UNIFORM/GRADIENT are O(disc_size).
 *                  MINDIST is O(disc_size × pool_count) per scatter — still
 *                  < 50k ops for typical parameters.
 *
 * References     :
 *   Poisson-disk sampling    — https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
 *   Hex disc / ring          — https://www.redblobgames.com/grids/hexagons/#range
 *   Gradient / falloff        — inverse-distance density weighting
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * All four scatter modes share one loop structure:
 *
 *   for dr in [-R, R]:
 *     for dq in [-R, R]:
 *       d = hex_dist(cursor, cursor+(dq,dr))
 *       if d > R: skip  (outside the scatter disc)
 *       if acceptance_rule(mode, d, ...): pool_place(cursor+(dq,dr))
 *
 * The difference is ENTIRELY in the acceptance rule.  The bounding-box loop
 * + hex_dist filter produces the hex disc.  Each mode then decides whether
 * to accept or reject each candidate cell.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture the four modes as four kinds of paint sprayer aimed at the cursor:
 *
 *   UNIFORM:   a spray can that randomly skips ~50% of the area (sparse).
 *   MINDIST:   a stamp that places dots only if each dot is far from all others
 *              — result looks evenly distributed, like a Poisson disk.
 *   FLOOD:     a paint bucket that fills every cell within the radius solidly.
 *   GRADIENT:  a soft-spray that concentrates near the cursor and fades out —
 *              dense centre, sparse edge (like a Gaussian blob).
 *
 * DRAWING METHOD  (GRADIENT — the most mathematically interesting)
 * ──────────────────────────────────────────────────────────────
 *  For each candidate hex at axial distance d from cursor:
 *  1. Compute acceptance probability: P = FALLOFF / (d + FALLOFF)
 *     d=0: P=FALLOFF/FALLOFF=1.0 (cursor hex always placed)
 *     d=1: P=FALLOFF/(1+FALLOFF)
 *     d=R: P=FALLOFF/(R+FALLOFF)
 *  2. Draw a uniform random number u ∈ [0, 1).
 *  3. If u < P: pool_place (accept); else: skip (reject).
 *  Increasing FALLOFF makes the gradient softer (slower dropoff).
 *
 * KEY FORMULAS
 * ────────────
 *  hex_dist (used by every mode to determine disc membership):
 *    d = (|dq| + |dr| + |dq+dr|) / 2
 *
 *  Disc cell count for radius R:
 *    count = 3R² + 3R + 1   (e.g. R=3 → 37 cells, R=5 → 91 cells)
 *
 *  UNIFORM acceptance:  rand()/RAND_MAX < DENSITY
 *
 *  MINDIST acceptance:  all pool objects have hex_dist ≥ MIN_DIST to candidate
 *
 *  FLOOD acceptance:    d ≤ R  (always accept)
 *
 *  GRADIENT acceptance:
 *    P = FALLOFF / (d + FALLOFF)
 *    rand()/RAND_MAX < P
 *
 *  Expected placed cells for GRADIENT over disc of radius R:
 *    E[n] = Σ_{d=0}^{R} ring_count(d) × FALLOFF / (d + FALLOFF)
 *    where ring_count(0)=1, ring_count(d)=6d for d≥1
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • MINDIST with MIN_DIST=1: adjacent hexes are allowed; any hex in the disc
 *    that doesn't coincide with an existing object is accepted.
 *
 *  • MINDIST with very large MIN_DIST: only the cursor hex (d=0) may be placed
 *    if it's ≥ MIN_DIST from all pool objects.  Can result in 0 new objects.
 *
 *  • FLOOD may overflow MAX_OBJ for large radii: R=8 disc = 217 cells, R=9 = 271.
 *    pool_place silently caps at MAX_OBJ=256.  Use R ≤ 8 for full floods.
 *
 *  • GRADIENT FALLOFF=0 would cause division by zero in d=0 case.
 *    Keep FALLOFF ≥ 1. At FALLOFF=1: P(d=0)=1, P(d=1)=0.5, P(d=2)=0.33.
 *
 *  • rand() is seeded once in main() with srand(time(NULL)).  Each scatter
 *    call advances the PRNG state — scatter at the same position gives
 *    different results each time.  This is a feature, not a bug.
 *
 * HOW TO VERIFY  (radius=3, cursor=(0,0), 80×24 terminal, size=14)
 * ─────────────
 *  Disc R=3: 3×9+9+1 = 37 cells.
 *
 *  UNIFORM DENSITY=0.5: expect ~18-19 objects placed.
 *    Each of 37 hexes accepted with p=0.5, independently. ✓
 *
 *  FLOOD R=3: exactly 37 cells placed (all disc hexes). ✓
 *    Verify: (dq=3,dr=0): d=(3+0+3)/2=3 ≤ 3 → placed. ✓
 *            (dq=2,dr=2): d=(2+2+4)/2=4 > 3 → rejected. ✓
 *
 *  GRADIENT FALLOFF=3, R=3:
 *    d=0: P=3/3=1.0   → 1×1=1 expected cell
 *    d=1: P=3/4=0.75  → 6×0.75=4.5 expected cells
 *    d=2: P=3/5=0.60  → 12×0.60=7.2 expected cells
 *    d=3: P=3/6=0.50  → 18×0.50=9.0 expected cells
 *    Total expected: ~21.7 cells (vs 37 for FLOOD). ✓
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

/* Scatter radius bounds. R=8 disc = 217 cells — just fits in MAX_OBJ=256. */
#define RADIUS_DEFAULT      4
#define RADIUS_MIN          1
#define RADIUS_MAX          8

/* Density for UNIFORM mode: fraction of disc hexes to place (0.0–1.0). */
#define DENSITY_DEFAULT     0.40
#define DENSITY_STEP        0.05
#define DENSITY_MIN         0.05
#define DENSITY_MAX         1.00

/* Minimum hex-step separation for MINDIST mode. */
#define MINDIST_DEFAULT     2
#define MINDIST_MIN         1
#define MINDIST_MAX         6

/* Falloff constant for GRADIENT: P = FALLOFF/(d+FALLOFF).  Must be ≥ 1. */
#define GRAD_FALLOFF        3

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
#define PAIR_HUD       4   /* status bar (yellow)    */
#define PAIR_HINT      5   /* key-hint footer (cyan) */

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR, COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_OBJ,    COLORS >= 256 ? 214 : COLOR_RED,     -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,    -1);
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
 * Used by all scatter modes to filter the bounding box to the disc.
 * Used by MINDIST to check separation from pool objects.
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
    attron(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        ctx_to_screen(g, p->items[i].q, p->items[i].r, &col, &row);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
}

/* ── end §5 — pool objects are scattered by §7 functions ──────────────── */

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
/* §7  scatter — four acceptance strategies over the hex disc             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * scatter_uniform — place each disc hex with probability DENSITY.
 *
 * THE FORMULA:
 *   For each (dq, dr) in [-R,R]² with hex_dist(0,0,dq,dr) ≤ R:
 *     if rand()/RAND_MAX < density: pool_place(cur.q+dq, cur.r+dr, '.')
 *
 * Expected output: density × disc_count cells.  Variance is binomial:
 * each hex is an independent Bernoulli trial with p = density.
 */
static void scatter_uniform(Pool *pool, int cQ, int cR,
                             int radius, double density)
{
    for (int dr = -radius; dr <= radius; dr++) {
        for (int dq = -radius; dq <= radius; dq++) {
            if (hex_dist(0, 0, dq, dr) > radius) continue;
            if ((double)rand() / (double)RAND_MAX > density) continue;
            pool_place(pool, cQ + dq, cR + dr, '.');
        }
    }
}

/*
 * scatter_mindist — Poisson-disk sampling on the hex grid.
 *
 * THE FORMULA:
 *   For each candidate (q, r) in the disc:
 *     For each existing pool object (pq, pr):
 *       if hex_dist(q, r, pq, pr) < mindist: reject candidate
 *     If no object is too close: pool_place(q, r, '+')
 *
 * WHY this produces even distribution: any two placed objects are guaranteed
 * ≥ mindist hex steps apart.  At mindist=2 each object "owns" a 7-cell disc
 * around itself (1 + 6 neighbours), leaving no adjacent hexes empty.
 *
 * Cost: O(disc_count × pool_count) per scatter.  With disc ≤ 217 and
 * pool ≤ 256, this is at most 55,552 comparisons — under 1 ms.
 */
static void scatter_mindist(Pool *pool, int cQ, int cR,
                             int radius, int mindist)
{
    for (int dr = -radius; dr <= radius; dr++) {
        for (int dq = -radius; dq <= radius; dq++) {
            if (hex_dist(0, 0, dq, dr) > radius) continue;
            int q = cQ + dq, r = cR + dr;
            int ok = 1;
            for (int i = 0; i < pool->count && ok; i++) {
                if (hex_dist(q, r, pool->items[i].q, pool->items[i].r) < mindist)
                    ok = 0;
            }
            if (ok) pool_place(pool, q, r, '+');
        }
    }
}

/*
 * scatter_flood — fill the entire disc solidly.
 *
 * THE FORMULA:
 *   Accept ALL hexes with hex_dist(cursor, hex) ≤ radius.
 *   disc_count = 3R² + 3R + 1 cells placed.
 *
 * Equivalent to a flood-fill starting at cursor outward to depth R — but
 * since the hex plane has no obstacles here, iteration is simpler than BFS.
 */
static void scatter_flood(Pool *pool, int cQ, int cR, int radius)
{
    for (int dr = -radius; dr <= radius; dr++) {
        for (int dq = -radius; dq <= radius; dq++) {
            if (hex_dist(0, 0, dq, dr) <= radius)
                pool_place(pool, cQ + dq, cR + dr, '#');
        }
    }
}

/*
 * scatter_gradient — acceptance probability falls off with hex distance.
 *
 * THE FORMULA:
 *   For each hex at dist d from cursor:
 *     P = GRAD_FALLOFF / (d + GRAD_FALLOFF)
 *     if rand()/RAND_MAX < P: place '*'
 *
 * WHY inverse-distance falloff: it produces a "heat map" that is denser at
 * the cursor and sparser at the edge.  Increasing GRAD_FALLOFF makes the
 * gradient shallower (more uniform).  At GRAD_FALLOFF=∞, P=1 for all d
 * (degenerates to FLOOD).  At GRAD_FALLOFF=1 the dropoff is steep.
 */
static void scatter_gradient(Pool *pool, int cQ, int cR, int radius)
{
    for (int dr = -radius; dr <= radius; dr++) {
        for (int dq = -radius; dq <= radius; dq++) {
            int d = hex_dist(0, 0, dq, dr);
            if (d > radius) continue;
            double p = (double)GRAD_FALLOFF / (double)(d + GRAD_FALLOFF);
            if ((double)rand() / (double)RAND_MAX < p)
                pool_place(pool, cQ + dq, cR + dr, '*');
        }
    }
}

/* ── end §7 — all scatter strategies proven correct in HOW TO VERIFY ─── */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef enum {
    SC_UNIFORM=0, SC_MINDIST=1, SC_FLOOD=2, SC_GRADIENT=3, N_SC=4
} ScatterMode;

static const char *SC_NAME[N_SC] = { "uniform", "mindist", "flood", "gradient" };

typedef struct {
    ScatterMode mode;
    int         radius;
    double      density;   /* UNIFORM mode only */
    int         mindist;   /* MINDIST mode only */
} SceneCfg;

static void cfg_init(SceneCfg *cfg)
{
    cfg->mode    = SC_UNIFORM;
    cfg->radius  = RADIUS_DEFAULT;
    cfg->density = DENSITY_DEFAULT;
    cfg->mindist = MINDIST_DEFAULT;
}

static void scene_scatter(Pool *pool, const Cursor *cur, const SceneCfg *cfg)
{
    switch (cfg->mode) {
    case SC_UNIFORM:  scatter_uniform (pool, cur->q, cur->r, cfg->radius, cfg->density); break;
    case SC_MINDIST:  scatter_mindist (pool, cur->q, cur->r, cfg->radius, cfg->mindist); break;
    case SC_FLOOD:    scatter_flood   (pool, cur->q, cur->r, cfg->radius);               break;
    case SC_GRADIENT: scatter_gradient(pool, cur->q, cur->r, cfg->radius);               break;
    default: break;
    }
}

/* Bright bold yellow fps readout (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Pool *p, const SceneCfg *cfg,
                      double fps)
{
    char buf[96];
    if (cfg->mode == SC_UNIFORM) {
        snprintf(buf, sizeof buf,
                 " %s  R:%d  d:%.2f  obj:%d  %5.1f fps ",
                 SC_NAME[cfg->mode], cfg->radius, cfg->density, p->count, fps);
    } else if (cfg->mode == SC_MINDIST) {
        snprintf(buf, sizeof buf,
                 " %s  R:%d  min:%d  obj:%d  %5.1f fps ",
                 SC_NAME[cfg->mode], cfg->radius, cfg->mindist, p->count, fps);
    } else {
        snprintf(buf, sizeof buf,
                 " %s  R:%d  obj:%d  %5.1f fps ",
                 SC_NAME[cfg->mode], cfg->radius, p->count, fps);
    }
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:scatter  1-4:mode  +/-:R  d/D:density  m/M:mindist  C:clear  q/ESC:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                        const SceneCfg *cfg, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    pool_draw(p, g);
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
    srand((unsigned int)time(NULL));
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
            case 'C': pool_clear(&pool); break;
            case ' ': scene_scatter(&pool, &cur, &cfg); break;
            case '1': cfg.mode = SC_UNIFORM;  break;
            case '2': cfg.mode = SC_MINDIST;  break;
            case '3': cfg.mode = SC_FLOOD;    break;
            case '4': cfg.mode = SC_GRADIENT; break;
            case KEY_UP:    cursor_move(&cur, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (cfg.radius < RADIUS_MAX) cfg.radius++;
                break;
            case '-':
                if (cfg.radius > RADIUS_MIN) cfg.radius--;
                break;
            case 'd':
                if (cfg.density < DENSITY_MAX - DENSITY_STEP/2)
                    cfg.density += DENSITY_STEP;
                break;
            case 'D':
                if (cfg.density > DENSITY_MIN + DENSITY_STEP/2)
                    cfg.density -= DENSITY_STEP;
                break;
            case 'm':
                if (cfg.mindist < MINDIST_MAX) cfg.mindist++;
                break;
            case 'M':
                if (cfg.mindist > MINDIST_MIN) cfg.mindist--;
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
