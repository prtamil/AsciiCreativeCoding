/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_hex_direct.c — cursor-based direct object placement on a flat-top hex grid
 *
 * DEMO: A flat-top hexagonal grid fills the screen. Navigate '@' between
 *       hexagons with arrow keys. Press SPACE to toggle a '*' object at the
 *       cursor hex. Objects are stored by axial address (q, r) and survive
 *       resizes — they follow their hex when the terminal changes size.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c (background rasterizer),
 *                  grids/rect_grids_placement/01_direct.c (same idea on rect)
 *
 * Section map:
 *   §1  config   — all tunable constants
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — color pairs: grid, cursor, object, HUD, hint
 *   §4  gridctx  — GridCtx struct + ctx_init / ctx_to_screen / ctx_draw_bg
 *                  (also cube_round, hex_dist, angle_char helpers)
 *   §5  pool     — Pool: object array, toggle, clear, draw
 *   §6  cursor   — Cursor struct + axial movement table, cursor_draw
 *   §7  scene    — hud_draw + scene_draw
 *   §8  screen   — ncurses init / cleanup
 *   §9  app      — signals, main loop
 *
 * Keys:  arrows:move  spc:toggle  C:clear  r:reset  +/-:size  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids_placement/01_hex_direct.c \
 *       -o 01_hex_direct -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Flat-top hex cursor placement.  A cursor (cur.q, cur.r) in
 *                  axial coordinates moves with arrow keys.  SPACE toggles an
 *                  object record (q, r, glyph) in a flat Pool array.  Each frame:
 *                  ctx_draw_bg rasterizes the background per-pixel using the
 *                  inverse matrix; pool_draw projects each stored (q, r) back
 *                  to screen via the forward matrix.
 *
 * Data-structure : Pool — flat array of Obj{q, r, glyph}.  Removal swaps
 *                  the dead slot with the last item (O(1)).  Capacity MAX_OBJ
 *                  greatly exceeds the visible hex count on any terminal.
 *
 * GridContext    : GridCtx carries the hex-specific geometry (hex_size,
 *                  border_w, screen origin ox/oy, terminal extent rows/cols).
 *                  The same struct shape is shared with rect/tri/polar
 *                  placement files; only the fields differ.
 *
 * Rendering      : Three-pass: (1) ctx_draw_bg rasterizes hex borders per
 *                  pixel, (2) pool_draw renders each placed object at its hex
 *                  centre, (3) cursor_draw places '@' at cursor hex centre.
 *                  The cursor draws over objects so the player always sees
 *                  their position.
 *
 * Performance    : ctx_draw_bg is O(rows × cols) per frame — the dominant cost.
 *                  pool_draw is O(MAX_OBJ).  No dynamic allocation after init.
 *
 * References     :
 *   Red Blob Games hex guide  — https://www.redblobgames.com/grids/hexagons/
 *   Cube coordinates          — https://www.redblobgames.com/grids/hexagons/#coordinates-cube
 *   Object pool pattern       — gameprogrammingpatterns.com/object-pool.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two independent coordinate systems are always in play:
 *
 *   HEX space   — axial (q, r) integers.  The cursor and placed objects live
 *                 here.  Hex (0, 0) is the screen centre.  Distances between
 *                 hexes are hex_dist — independent of terminal size.
 *
 *   SCREEN space — (row, col) characters.  ncurses lives here.  The background
 *                  rasterizer visits every (row, col), converts to pixel,
 *                  applies the inverse matrix → (q, r), and draws a border
 *                  character if that pixel is near a hex edge.
 *
 * Placed objects know ONLY their (q, r) address.  To render them, apply the
 * forward matrix (hex centre → pixel) then divide by (CELL_W, CELL_H) to get
 * the screen cell.  This is the same formula cursor_draw uses for '@'.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of each hex as a named city.  Its name is (q, r).  Its pixel location
 * (cx, cy) is computed on demand from the forward matrix — it is never stored.
 * The background map (ctx_draw_bg) redraws each frame by asking every pixel
 * "which city owns you?" via the inverse matrix.  Placed objects are sticky
 * notes attached to a city name; they rerender at that city's pixel location.
 *
 * DRAWING METHOD  (forward matrix — hex → screen)
 * ──────────────────────────────────────────────
 *  Given hex (q, r) and radius parameter size:
 *
 *  1. Flat-top forward matrix (hex → pixel centre):
 *       cx = size × 3/2 × q
 *       cy = size × (√3/2 × q  +  √3 × r)
 *
 *  2. Terminal cell (centering offset ox = cols/2, oy = (rows-1)/2):
 *       col = ox + round(cx / CELL_W)
 *       row = oy + round(cy / CELL_H)
 *     cursor_draw uses truncation (int)(cx/CELL_W) to stay in interior.
 *
 *  3. Draw glyph at (row, col) if within visible bounds.
 *
 * KEY FORMULAS
 * ────────────
 *  Forward (hex → pixel centre):
 *    cx = size × 3/2 × q
 *    cy = size × (√3/2 × q  +  √3 × r)
 *
 *  Inverse (pixel → fractional cube):
 *    fq = (2/3 × px) / size
 *    fr = (−1/3 × px  +  √3/3 × py) / size
 *    fs = −fq − fr
 *
 *  cube_round (fractional cube → nearest integer hex):
 *    Round all three; fix the component with the LARGEST rounding error:
 *      if dq = max: q = −rr − rs
 *      if dr = max: r = −rq − rs
 *      otherwise:   q = rq, r = rr  (s = −q − r implicit)
 *
 *  hex_dist (axial distance between two hexes A and B):
 *    d = (|dq| + |dr| + |dq + dr|) / 2
 *    where dq = q_B − q_A, dr = r_B − r_A
 *
 *  Cube distance (border detection in ctx_draw_bg):
 *    dist = max(|fq − q|, |fr − r|, |fs − s|)
 *    border if dist ≥ 0.5 − border_w
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • cube_round: never skip the "fix largest error" step. Rounding fq, fr, fs
 *    independently breaks q+r+s=0. The fix step restores the constraint.
 *
 *  • pool_draw uses round() (not truncation). Objects must centre in their hex,
 *    not drift to the interior like '@'. One pixel of difference matters.
 *
 *  • CELL_H / CELL_W = 4/2 = 2. Terminal chars are ~2× taller than wide.
 *    Without this ratio, hexes appear squashed vertically on screen.
 *
 *  • Resize: ox/oy recompute from current rows/cols each frame. cur.q/cur.r and
 *    Pool (q, r) values stay valid — they live in hex space, not screen space.
 *
 * HOW TO VERIFY  (80×24 terminal, HEX_SIZE=14, CELL_W=2, CELL_H=4)
 * ─────────────
 *  Screen centre: ox=40, oy=11.
 *
 *  Cursor at (q=0, r=0):
 *    cx = 0, cy = 0 → col=40, row=11 → '@' at (11,40). ✓
 *
 *  Place object at (q=1, r=0):
 *    cx = 14×1.5×1 = 21,  cy = 14×(√3/2×1 + 0) ≈ 12.12
 *    col = 40 + round(21/2) = 50,  row = 11 + round(12.12/4) = 14
 *    → '*' at (14, 50). ✓
 *
 *  Move cursor RIGHT: cur.q += +1 → (q=1, r=0) → '@' at (14, 50).
 *  Draws over '*' — cursor always visible on top. ✓
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

/* Aspect correction: terminal chars are ~2× taller than wide.
 * CELL_W=2, CELL_H=4 → 2:1 ratio so hexes appear circular, not oval. */
#define CELL_W              2
#define CELL_H              4

#define HEX_SIZE_DEFAULT   14.0   /* hex radius in pixels (sub-cell units) */
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

/* Border width as a fraction of cube distance [0..0.5].
 * 0.10 ≈ 1–2 terminal cells of border visible inside each hex. */
#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35

#define MAX_OBJ            256   /* max placed objects; far more than visible */
#define FRAME_NS    16666667LL   /* ~60 fps                                   */

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

#define PAIR_GRID    1   /* dim hex border lines        */
#define PAIR_CURSOR  2   /* cursor hex + '@' character  */
#define PAIR_OBJ     3   /* placed object glyphs        */
#define PAIR_HUD     4   /* status bar (yellow)         */
#define PAIR_HINT    5   /* key hint footer (cyan)      */

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
/* §4  gridctx — GridCtx + cube_round, hex_to_screen, hex_dist, angle_char */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active hex grid.
 *
 * Hex-specific fields:
 *   hex_size  — hex radius in pixel sub-units (governs apparent cell size)
 *   border_w  — border thickness in fractional cube distance
 *   ox, oy    — screen origin (screen position of axial (0, 0))
 *
 * Same struct shape as the rect/tri/polar placement files; only the fields
 * differ.  Carrying these as fields (not globals) lets ctx_to_screen() work
 * for any GridCtx — the abstraction shared with the rest of the series.
 */
typedef struct {
    /* terminal extent */
    int    rows, cols;

    /* hex geometry */
    double hex_size;
    double border_w;

    /* screen origin (screen pos of axial 0,0) */
    int    ox, oy;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size and current hex parameters.
 *
 * ox/oy place axial (0,0) at the screen centre.  oy uses (rows-1)/2 to leave
 * the bottom row free for the HUD hint.
 */
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
 * cube_round — nearest integer hex to fractional cube position (fq, fr, fs).
 *
 * THE FORMULA:
 *   Round all three axes independently: rq=round(fq), rr=round(fr), rs=round(fs).
 *   Rounding can break q+r+s=0, so fix the component with the LARGEST error:
 *     dq=|rq-fq|, dr=|rr-fr|, ds=|rs-fs|
 *     if dq is max: q = -rr - rs   (recomputed to restore constraint)
 *     if dr is max: r = -rq - rs
 *     else:         q = rq, r = rr  (s = -q-r, implicit)
 *
 * WHY fix the largest-error component: it was rounded the most aggressively,
 * so reassigning it from the constraint loses the least accuracy in the other
 * two axes (which were rounded more conservatively).
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
 * ctx_to_screen — project hex (q, r) to terminal (col, row).
 *
 * THE FORMULA (flat-top forward matrix + aspect correction):
 *
 *   Pixel centre of hex (q, r):
 *     cx = size × 3/2 × q
 *     cy = size × (√3/2 × q  +  √3 × r)
 *
 *   Terminal cell (centering offset ox, oy):
 *     col = ox + round(cx / CELL_W)
 *     row = oy + round(cy / CELL_H)
 *
 * round() (not truncation) keeps object glyphs centred in their hex interior.
 * cursor_draw uses truncation to guarantee '@' stays inside the border ring.
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
 * angle_char — map a tangent angle to the best-fit ASCII line character.
 *
 * THE FORMULA:
 *   Input theta = atan2(py-cy, px-cx) + π/2  (radial → tangent direction).
 *   Fold into [0, π) — ASCII chars are symmetric under 180° rotation.
 *   Map: [0, π/8) → '-'  [π/8, 3π/8) → '\'  [3π/8, 5π/8) → '|'
 *        [5π/8, 7π/8) → '/'  [7π/8, π) → '-'
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
 * ctx_draw_bg — rasterize the flat-top hex background with cursor highlighted.
 *
 * THE PIPELINE (per screen cell):
 *
 *   (col, row) → pixel:  px = (col-ox)×CELL_W,  py = (row-oy)×CELL_H
 *      ↓
 *   Inverse matrix → fractional cube:
 *     fq = (2/3 × px) / size
 *     fr = (-px/3 + √3·py/3) / size
 *     fs = -fq - fr
 *      ↓
 *   cube_round → integer hex (q, r)
 *      ↓
 *   Cube distance: dist = max(|fq-q|, |fr-r|, |fs-(-q-r)|)
 *      ├── dist < 0.5-border_w → interior → skip
 *      └── dist ≥ 0.5-border_w → border:
 *            cx = size×1.5×q,  cy = size×(√3/2×q + √3×r)
 *            theta = atan2(py-cy, px-cx)
 *            ch = angle_char(theta + π/2)
 *            color = PAIR_CURSOR if q==cur.q && r==cur.r, else PAIR_GRID
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
            double fS = (double)(-q - r);

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
/* §5  pool — flat object array with O(1) removal                         */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int q, r; char glyph; } Obj;
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

static int pool_find(const Pool *p, int q, int r)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].q == q && p->items[i].r == r)
            return i;
    return -1;
}

/*
 * pool_toggle — add '*' at (q, r) if absent; remove it if present.
 *
 * Removal swaps the target with the last item and decrements count — O(1).
 * This avoids shifting the array but makes the pool unordered.  Draw order
 * doesn't matter for placement (each hex is unique), so this is fine.
 */
static void pool_toggle(Pool *p, int q, int r)
{
    int i = pool_find(p, q, r);
    if (i >= 0) {
        p->items[i] = p->items[--p->count];   /* swap-last remove */
        return;
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ q, r, '*' };
}

static void pool_clear(Pool *p) { p->count = 0; }

/*
 * pool_draw — render all objects using the flat-top forward matrix.
 *
 * THE FORMULA: for each (q, r) in pool → ctx_to_screen → bounds check → draw.
 * Drawing over the grid background is intentional: objects sit ON the grid.
 */
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

/* ── end §5 — to understand the cursor placement, read §6 ─────────────── */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor — axial cursor and movement                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — just (q, r) in axial hex space.
 *
 * Bounds are unclamped: the hex plane is infinite.  The pool stores objects
 * by (q, r) so they survive resize/movement off-screen.
 */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur) { cur->q = 0; cur->r = 0; }

/*
 * HEX_DIR — axial movement deltas for the 4 arrow keys.
 *
 * Flat-top hex: UP/DOWN move along r, LEFT/RIGHT move along q.
 * The 2 diagonal faces (NE = +q−r, SW = −q+r) are not mapped to keys.
 *
 *             UP: (0, -1)
 *  LEFT: (-1, 0)  ●  RIGHT: (+1, 0)
 *            DOWN: (0, +1)
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    — r decreases */
    { 0, +1 },   /* DOWN  — r increases */
    {-1,  0 },   /* LEFT  — q decreases */
    {+1,  0 },   /* RIGHT — q increases */
};

static void cursor_move(Cursor *cur, int dq, int dr)
{
    cur->q += dq; cur->r += dr;
}

/*
 * cursor_draw — place '@' at the interior centre of hex (cur.q, cur.r).
 *
 * THE FORMULA:
 *   cx_pix = size × 3/2 × cur.q
 *   cy_pix = size × (√3/2 × cur.q  +  √3 × cur.r)
 *   col = ox + (int)(cx_pix / CELL_W)   ← truncation stays in interior
 *   row = oy + (int)(cy_pix / CELL_H)
 */
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
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Bright bold yellow fps readout (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                      double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
             " q:%+d r:%+d  obj:%d  size:%.0f  %5.1f fps ",
             cur->q, cur->r, p->count, g->hex_size, fps);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:toggle  C:clear  r:reset  +/-:size  q/ESC:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                        double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    pool_draw(p, g);
    cursor_draw(cur, g);
    hud_draw(g, p, cur, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  screen                                                              */
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
/* §9  app                                                                 */
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
            case ' ': pool_toggle(&pool, cur.q, cur.r); break;
            case KEY_UP:    cursor_move(&cur, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (g.hex_size < HEX_SIZE_MAX) g.hex_size += HEX_SIZE_STEP;
                break;
            case '-':
                if (g.hex_size > HEX_SIZE_MIN) g.hex_size -= HEX_SIZE_STEP;
                break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;

        scene_draw(&g, &pool, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
