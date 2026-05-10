/*
 * 01_flat_top.c — flat-top hexagonal grid with keyboard-controlled cursor
 *
 * DEMO: Fills the screen with flat-top hexagons. A '@' cursor starts at the
 *       origin hex and moves between adjacent hexes with arrow keys — the
 *       current hex border glows white-on-blue. Resize hexes with +/-.
 *
 * Study alongside: grids/hex_grids/02_pointy_top.c (same cursor logic,
 *                  different transform matrix),
 *                  grids/rect_grids/01_uniform_rect.c (the GridCtx template)
 *
 * Section map:
 *   §1 config   — tunable constants, EWMA alpha
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — ncurses color pairs (BORDER, CURSOR, HUD, HINT)
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_pixel_to_axial /
 *                 ctx_draw_bg + cube_round + angle_char
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw, HEX_DIR
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  t theme  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids/01_flat_top.c \
 *       -o 01_flat_top -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Flat-top hex tiling via cube coordinates (Q+R+S=0).
 *                  pixel→fractional cube via flat-top inverse matrix,
 *                  cube_round (fix-largest-error) → nearest hex (Q,R),
 *                  cube dist = max(|fq-Q|,|fr-R|,|fs-S|) → border/interior.
 *
 * Data-structure : Two structs — GridCtx (terminal extent, hex_size,
 *                  CELL_W/CELL_H, screen origin ox/oy) and Cursor (axial
 *                  pair (q, r); S = -q-r implicit). No grid array — every
 *                  pixel resolves its (Q,R) per frame from the flat-top
 *                  inverse matrix + cube_round.
 *
 * Cursor movement : Stored as axial (q, r). Arrow keys apply deltas from
 *                  HEX_DIR[4][2] — 4 of the 6 hex faces. See §5.
 *
 * Rendering      : Grid centered on screen: pixel origin = screen center.
 *                  Border char = angle_char(theta + π/2). Cursor hex border
 *                  drawn in PAIR_CURSOR; '@' drawn at hex centroid cell.
 *
 * References     :
 *   Red Blob Games hex guide — https://www.redblobgames.com/grids/hexagons/
 *   Cube rounding            — https://www.redblobgames.com/grids/hexagons/#rounding
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every screen pixel belongs to exactly one hexagon. The cube coordinate
 * system (Q+R+S=0) embeds a 2-D hex plane in 3-D space — but the constraint
 * Q+R+S=0 keeps us on a diagonal plane. Within this system, "which hex am
 * I closest to?" reduces to: round the fractional cube coordinates to the
 * nearest integers, with one correction step to restore Q+R+S=0. The cube
 * distance formula then tells us how close to the hex center we are, which
 * determines border vs. interior.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine coloring every hexagon a different color and asking: "for a given
 * pixel, which color does it inherit?" The answer is the color of the nearest
 * hex center. The boundary between two hexes is the set of pixels equidistant
 * from both centers — i.e., where cube distance = 0.5. Everything closer than
 * 0.5 − border_w is interior (invisible); everything between 0.5 − border_w
 * and 0.5 is "border" and gets a line character.
 *
 * You never store a hex grid array. You never loop over hexes. You ask the
 * formula at every pixel, per frame. No data structure, just arithmetic.
 *
 * DRAWING METHOD  (raster-scan pipeline)
 * ──────────────────────────────────────
 *  1. Center the grid: ox = cols/2, oy = (rows-1)/2.
 *     Pixel: px = (col-ox)*CELL_W, py = (row-oy)*CELL_H.
 *     Now hex (Q=0,R=0) maps to pixel (0,0) = screen center.
 *
 *  2. Flat-top inverse matrix — pixel → fractional cube:
 *       fq =  (2/3 * px) / size
 *       fr = (-px/3 + √3*py/3) / size
 *       fs = -fq - fr
 *
 *  3. cube_round — round all three, restore Q+R+S=0 by fixing the
 *     component with the largest rounding error:
 *       if dq is largest: Q = -rr - rs
 *       if dr is largest: R = -rq - rs
 *       otherwise:        S = -rq - rr   (S not stored; implicit)
 *
 *  4. Cube distance — how far is the pixel from hex (Q,R)?
 *       dist = max(|fq-Q|, |fr-R|, |fs-S|)
 *     0 = exact center, 0.5 = edge midpoint, 2/3 = vertex.
 *
 *  5. Threshold: dist < (0.5 - border_w) → interior → skip.
 *
 *  6. For border pixels: compute the hex center (flat-top forward matrix):
 *       cx = size * 3/2 * Q
 *       cy = size * (√3/2 * Q + √3 * R)
 *     Then the radial angle: theta = atan2(py-cy, px-cx).
 *
 *  7. Pick a line character: angle_char(theta + π/2).
 *     The +π/2 rotates from radial to tangent direction.
 *
 *  8. Draw in cursor color (Q==cur->q && R==cur->r) or border color.
 *
 * KEY FORMULAS
 * ────────────
 *  Flat-top inverse matrix (pixel → fractional cube):
 *    fq =  2/3 * px / size
 *    fr = (-1/3 * px + √3/3 * py) / size
 *
 *  Flat-top forward matrix (hex → pixel center):
 *    cx = size * 3/2 * Q
 *    cy = size * (√3/2 * Q  +  √3 * R)
 *
 *  Pixel → terminal cell (with centering):
 *    col = ox + (int)(cx / CELL_W)
 *    row = oy + (int)(cy / CELL_H)
 *
 *  Cube distance (border detection):
 *    dist = max(|fq-Q|, |fr-R|, |fs-S|)
 *    interior  if dist < 0.5 - border_w
 *    border    if dist ≥ 0.5 - border_w
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • cube_round: rounding three values independently can break Q+R+S=0.
 *    Always fix the component with the LARGEST error — that component was
 *    rounded away from its "true" value most, so enforcing the constraint
 *    via it keeps the other two more accurate.
 *
 *  • CELL_H / CELL_W = 4/2 = 2. Terminal characters are ~2× taller than
 *    wide in pixels. CELL_W=2, CELL_H=4 compensates so one pixel step is
 *    the same distance in both axes when projected to screen.
 *
 *  • ox = cols/2 is integer division. At odd column counts, the origin is
 *    shifted half a cell left. This is fine — the grid tiles infinitely.
 *
 *  • Last row (row == rows-1) is reserved for the HUD. Raster scan stops
 *    at row < rows-1.
 *
 *  • Resize: ctx_init recomputes ox/oy from the current rows/cols; the
 *    grid recenters automatically. cur->q/cur->r remain valid (hex coords
 *    are independent of terminal size).
 *
 * HOW TO VERIFY  (80×24 terminal, HEX_SIZE=14, CELL_W=2, CELL_H=4)
 * ─────────────
 *  Screen center: ox=40, oy=11.
 *  Origin hex (Q=0,R=0): cx=0, cy=0 → col=40, row=11. Cursor '@' at (11,40).
 *
 *  Pixel at cell (col=40, row=11): px=0, py=0 → fq=0, fr=0 → Q=0,R=0, dist=0.
 *  → interior pixel (dist < 0.4). Skipped — no character drawn there.
 *
 *  Pixel at cell (col=40, row=6): py=-20.
 *    fr ≈ -0.476.  dist≈0.476 ≥ 0.40 → BORDER. cy=0, cx=0.
 *    theta=atan2(-20-0, 0-0)=−π/2.
 *    angle_char(−π/2 + π/2) = angle_char(0) = '-'. ✓ Top of hex is horizontal.
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

#define CELL_W             2
#define CELL_H             4

#define HEX_SIZE_DEFAULT  14.0
#define HEX_SIZE_MIN       6.0
#define HEX_SIZE_MAX      40.0
#define HEX_SIZE_STEP      2.0

#define BORDER_W_DEFAULT   0.10
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.35
#define BORDER_W_STEP      0.02

#define N_THEMES           4
#define TICK_NS           16666667LL

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA     0.05

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

#define PAIR_BORDER   1
#define PAIR_CURSOR   2   /* cursor hex border + '@' character */
#define PAIR_HUD      3   /* yellow status bar */
#define PAIR_HINT     4   /* cyan key-hint footer */

static const short THEMES[N_THEMES][2] = {
    { COLOR_CYAN,   COLOR_BLACK },
    { COLOR_GREEN,  COLOR_BLACK },
    { COLOR_YELLOW, COLOR_BLACK },
    { COLOR_WHITE,  COLOR_BLACK },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_BORDER, THEMES[theme][0], THEMES[theme][1]);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the hex ↔ screen mapping                     */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active hex grid.
 *
 * The grid is centered on screen. For terminal cell (col, row):
 *   px = (col - ox) × CELL_W      ← pixel relative to screen center
 *   py = (row - oy) × CELL_H
 * with  ox = cols/2,  oy = (rows-1)/2.
 *
 * Hex (Q=0, R=0) center maps to pixel (0,0) = screen center.
 * CELL_H/CELL_W = 2 matches the ~2:1 terminal character aspect ratio,
 * so hexes appear circular rather than stretched vertically.
 *
 * border_w lives in GridCtx because ctx_draw_bg needs it; it's tunable per
 * frame from the main loop ([ / ]). hex_size is also tunable (+/-).
 *
 * max_q / max_r are advisory cursor bounds — the hex plane is infinite, so
 * "bounds" here means the largest Q/R that still places its center on
 * screen given the current hex_size. Cursor movement does not clamp by
 * default (panning beyond the visible area is allowed).
 */
typedef struct {
    /* terminal extent */
    int rows, cols;

    /* hex geometry */
    double hex_size;       /* pixel "radius" (distance from center to vertex)  */
    double border_w;       /* fraction of cube distance that counts as border  */
    int    cell_w, cell_h; /* sub-pixel scaling — CELL_W, CELL_H               */

    /* screen origin = pixel (0,0) */
    int    ox, oy;

    /* cursor bounds in axial space (computed from terminal size + hex_size) */
    int    max_q, max_r;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size.
 *
 * ox/oy are integer cell coordinates of the screen center.
 * max_q/max_r come from inverting the forward matrix at the screen edges:
 *   The widest hex column index that still fits is roughly
 *     cols * CELL_W / (2 * 1.5 * size).
 *   Same logic for rows with √3 spacing.
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows     = rows;
    g->cols     = cols;
    g->cell_w   = CELL_W;
    g->cell_h   = CELL_H;
    g->ox       = cols / 2;
    g->oy       = (rows - 1) / 2;
    /* Defer hex_size / border_w to scene defaults; ctx_init may be called
     * with stale values (e.g. resize) so caller sets them after. */
    if (g->hex_size <= 0.0) g->hex_size = HEX_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    g->max_q = (int)((double)cols * CELL_W / (3.0 * g->hex_size)) + 1;
    g->max_r = (int)((double)rows * CELL_H / (sqrt(3.0) * g->hex_size)) + 1;
}

/*
 * ctx_to_screen — center cell of hex (Q, R) in screen coordinates.
 *
 * THE FORMULA (flat-top forward matrix):
 *   cx_pix = size × 3/2 × Q
 *   cy_pix = size × (√3/2 × Q  +  √3 × R)
 *   sc = ox + (int)(cx_pix / CELL_W)
 *   sr = oy + (int)(cy_pix / CELL_H)
 *
 * Integer truncation (not round) keeps '@' slightly inside the hex interior
 * rather than on the border character, so it is always visible.
 */
static void ctx_to_screen(const GridCtx *g, int Q, int R, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * 1.5    * (double)Q;
    double cy_pix = g->hex_size * (sq3_2 * (double)Q + sq3 * (double)R);
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/*
 * cube_round — round fractional cube (fq,fr,fs) to integer (Q,R) restoring
 * Q+R+S=0 by fixing the component with the largest rounding error.
 *
 * Pure math helper — keeps its domain name. Used by ctx_pixel_to_axial and
 * by ctx_draw_bg (which inlines it for speed).
 */
static void cube_round(double fq, double fr, double fs, int *Q, int *R)
{
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) rq = -rr - rs;
    else if (dr > ds)             rr = -rq - rs;
    *Q = rq; *R = rr;
}

/*
 * ctx_pixel_to_axial — inverse map: terminal cell (sr, sc) → axial (Q, R).
 *
 * Reverses ctx_to_screen via the flat-top inverse matrix and cube_round.
 * Used by hit-testing (not by ctx_draw_bg, which inlines the math for
 * tighter inner-loop performance).
 */
__attribute__((unused))
static void ctx_pixel_to_axial(const GridCtx *g, int sr, int sc, int *Q, int *R)
{
    double sq3_3 = sqrt(3.0) / 3.0;
    double px = (double)(sc - g->ox) * g->cell_w;
    double py = (double)(sr - g->oy) * g->cell_h;
    double fq = (2.0/3.0 * px) / g->hex_size;
    double fr = (-1.0/3.0 * px + sq3_3 * py) / g->hex_size;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, Q, R);
}

/*
 * angle_char — map a tangent angle to the best-fit ASCII line character.
 *
 *   Input: theta = atan2(py-cy, px-cx) + π/2
 *
 *   WHY + π/2:
 *   atan2 gives the RADIAL angle from hex center to pixel.
 *   Adding π/2 converts it to the TANGENT angle — the direction the hex
 *   edge runs at that point. The character must align with the edge, not
 *   point toward the center.
 *
 *   The result is folded into [0, π) with fmod because ASCII characters
 *   are symmetric under 180° rotation ('-' looks the same upside-down).
 *
 *   After folding into [0°, 180°):
 *     [  0°,  22.5°) → '-'   near-horizontal  (flat-top hex top/bottom edges)
 *     [ 22.5°,  67.5°) → '\'  diagonal, \-slope (upper-left/lower-right edges)
 *     [ 67.5°, 112.5°) → '|'  near-vertical
 *     [112.5°, 157.5°) → '/'  diagonal, /-slope (lower-left/upper-right edges)
 *     [157.5°, 180°) → '-'   wraps back to horizontal
 */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)         return '-';
    else if (t < 3.0 * M_PI / 8.0)   return '\\';
    else if (t < 5.0 * M_PI / 8.0)   return '|';
    else if (t < 7.0 * M_PI / 8.0)   return '/';
    else                              return '-';
}

/*
 * ctx_draw_bg — rasterize the flat-top hex grid with cursor hex highlighted.
 *
 * THE PIPELINE (per screen cell):
 *
 *   (col, row)
 *      │
 *      ▼  Center the grid (origin hex → screen center)
 *   px = (col − ox) × CELL_W
 *   py = (row − oy) × CELL_H
 *      │
 *      ▼  Flat-top inverse matrix: pixel → fractional cube
 *   fq = (2/3 × px) / size
 *   fr = (−px/3 + √3·py/3) / size
 *   fs = −fq − fr
 *      │
 *      ▼  cube_round — see helper above
 *      │
 *      ▼  Cube distance: how far from hex (Q,R) center?
 *   fQ=Q, fR=R, fS=−Q−R
 *   dist = max(|fq−Q|, |fr−R|, |fs−S|)
 *      │
 *      ├── dist < (0.5 − border_w)  →  interior: skip (continue)
 *      │
 *      └── dist ≥ (0.5 − border_w)  →  border: draw character
 *            cx = size × 3/2 × Q         ← flat-top forward matrix (x)
 *            cy = size × (√3/2·Q + √3·R) ← flat-top forward matrix (y)
 *            theta = atan2(py−cy, px−cx)
 *            ch = angle_char(theta + π/2)
 *            color = PAIR_CURSOR if Q==cur->q && R==cur->r, else PAIR_BORDER
 *
 * cube_round is inlined here (rather than calling the helper) to avoid
 * function-call overhead in the per-pixel inner loop.
 */
static void ctx_draw_bg(const GridCtx *g, int cQ, int cR)
{
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double size  = g->hex_size;
    double limit = 0.5 - g->border_w;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;

            int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
            double dq = fabs((double)rq - fq);
            double dr = fabs((double)rr - fr);
            double ds = fabs((double)rs - fs);
            if      (dq > dr && dq > ds) rq = -rr - rs;
            else if (dr > ds)             rr = -rq - rs;
            int Q = rq, R = rr;

            double fQ = (double)Q, fR = (double)R, fS = (double)(-Q - R);
            double dist = fabs(fq - fQ);
            double d2   = fabs(fr - fR);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;

            double cx = size * 1.5 * fQ;
            double cy = size * (sq3_2 * fQ + sq3 * fR);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);

            int on_cur = (Q == cQ && R == cR);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — just (q, r) in axial hex space.
 *
 * Bounds and geometry live in GridCtx, not here — the cursor doesn't know
 * how big the grid is, just where in axial space the user is pointing.
 * The two structs compose: Cursor + GridCtx → screen position via
 * ctx_to_screen().
 *
 * S = -q-r is implicit; only q and r are stored.
 */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/*
 * HEX_DIR — movement vectors for arrow keys in axial (Q, R) space.
 *
 * The flat-top hex grid has 6 faces. Arrow keys cover 4 of them:
 *
 *                 UP: (Q=0, R=-1)
 *                       ↑
 *   LEFT: (Q=-1, R=0) ← ● → RIGHT: (Q=+1, R=0)
 *                       ↓
 *                DOWN: (Q=0, R=+1)
 *
 * Screen direction: RIGHT=east, LEFT=west, UP=upper-left, DOWN=lower-right.
 * The unmapped faces are NE (+1,-1) and SW (-1,+1) (diagonal in screen).
 * Same deltas apply for pointy-top (02), rhombille (06), trihex (07).
 *
 * WHY THESE DELTAS WORK:
 *   Flat-top forward matrix gives cx = 3/2·Q·s, cy = (√3/2·Q + √3·R)·s.
 *   Δ(Q=+1, R=0): Δcx = +3/2·s > 0 (moves right). ✓
 *   Δ(Q=0,  R=+1): Δcy = +√3·s > 0 (moves down in screen). ✓
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* 0 = UP    — R decreases */
    { 0, +1 },   /* 1 = DOWN  — R increases */
    {-1,  0 },   /* 2 = LEFT  — Q decreases */
    {+1,  0 },   /* 3 = RIGHT — Q increases */
};

/*
 * cursor_move — apply (dq, dr) in axial space.
 *
 * The hex plane is infinite; we do not clamp here. (max_q/max_r in GridCtx
 * are advisory — visible-extent only.)
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    (void)g;
    cur->q += dq;
    cur->r += dr;
}

/*
 * cursor_draw — place '@' at the center cell of the cursor hex.
 *
 * Uses ctx_to_screen for the hex→screen conversion. Drawn after
 * ctx_draw_bg so '@' sits on top of any border characters at the centroid.
 */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->q, cur->r, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Bright bold yellow status (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, g->hex_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    screen_init();
    int theme = 0, paused = 0;
    color_init(theme);

    GridCtx g = {0};
    g.hex_size = HEX_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    double fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 'r': cursor_reset(&cur, &g); break;
            case 't':
                theme = (theme + 1) % N_THEMES;
                color_init(theme);
                break;
            case KEY_UP:    cursor_move(&cur, &g, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (g.hex_size < HEX_SIZE_MAX) { g.hex_size += HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
            case '-':
                if (g.hex_size > HEX_SIZE_MIN) { g.hex_size -= HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
            case '[':
                if (g.border_w > BORDER_W_MIN) { g.border_w -= BORDER_W_STEP; } break;
            case ']':
                if (g.border_w < BORDER_W_MAX) { g.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
