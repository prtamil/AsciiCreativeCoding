/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral.c — equilateral triangular grid, the base formula
 *
 * DEMO: Fills the screen with equilateral triangles tiled in alternating
 *       up triangle / down triangle pairs. A '@' cursor sits on the origin
 *       triangle. Arrow keys move it across edges to neighbouring
 *       triangles. Each arrow press traces ONE edge — UP/DOWN cross
 *       horizontal edges, LEFT/RIGHT cross slanted edges. Resize with +/-,
 *       border with [/]. This is the root file in the tri_grids series —
 *       every other file modifies one piece of it.
 *
 * Study alongside: grids/rect_grids/01_uniform_rect.c — same pixel-rasterize
 *                  pattern but on an axis-aligned lattice; this file uses
 *                  a 2-axis SKEW lattice because triangles tile with 60°
 *                  basis vectors instead of 90°.
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, BORDER_W, FPS_EWMA_ALPHA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: border / cursor / HUD / HINT
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_pixel_to_tri /
 *                 ctx_draw_bg + tri_centroid_pixel + tri_edge_char
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw, TRI_DIR
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   t theme   p pause
 *        +/- size        [/] border thickness   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/01_equilateral.c \
 *       -o 01_equilateral -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Equilateral triangle tiling via a 2-axis skew lattice.
 *                  Basis: v1 = (size, 0), v2 = (size/2, h)  with h = size·√3/2.
 *                  Each rhombus (col, row) holds two equilateral triangles —
 *                  down (apex-down, base on top) and up (apex-up, base on
 *                  bottom) separated by the diagonal fa + fb = 1 in lattice
 *                  space.
 *
 * Data-structure : Two structs — GridCtx (terminal extent, tri_size,
 *                  CELL_W/CELL_H, screen origin ox/oy, border_w) and
 *                  Cursor (col, row, up). No grid array — every pixel
 *                  resolves its (col, row, up) per frame from the skew
 *                  inverse.
 *
 * Formula        : pixel → lattice (skew inverse):
 *                    b = py / h
 *                    a = px/size − 0.5·b
 *                  (col, row, up) = (⌊a⌋, ⌊b⌋, fa+fb ≥ 1).
 *                  Six triangles meet at every vertex of the tiling, and
 *                  every pixel lies in exactly one triangle.
 *
 * Edge chars     : Barycentric weights inside the triangle pick the edge
 *                  character. The smallest weight indicates which edge is
 *                  closest (the edge OPPOSITE the smallest-weight vertex).
 *                    down: l1→'/' l2→'\\' l3→'_'
 *                    up  : l1→'_' l2→'/'  l3→'\\'
 *
 * Movement       : (col, row, up) walked by lookup table TRI_DIR[4][2].
 *                  Each arrow key crosses ONE specific edge of the current
 *                  triangle — or toggles within the rhombus when the
 *                  geometry has no edge in that direction. Two presses of
 *                  UP from the up-triangle moves up by one full strip.
 *
 * References     :
 *   Triangular tiling     — https://en.wikipedia.org/wiki/Triangular_tiling
 *   Barycentric coords    — https://en.wikipedia.org/wiki/Barycentric_coordinate_system
 *   Red Blob Games (hex)  — https://www.redblobgames.com/grids/hexagons/
 *   Coxeter, "Regular Polytopes" §4.6 (regular tessellations of the plane)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The equilateral triangular tiling is the dual of the hexagonal tiling —
 * 6 triangles meet at every vertex. We don't store any of them. Instead,
 * for every screen pixel, we ask "which triangle do I belong to?" and the
 * answer drops out of arithmetic on a 2-axis SKEW lattice. No grid array,
 * no loops over triangles. Topology is a function, not a data structure.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the plane covered by parallelograms (rhombuses) whose sides are
 * the lattice basis vectors v1 and v2. Each rhombus contains exactly two
 * equilateral triangles, separated by its short diagonal. To find which
 * triangle owns a pixel:
 *   1. Express the pixel in lattice coordinates (a, b) — like asking
 *      "how many v1-steps and v2-steps to reach this point?".
 *   2. The integer parts (⌊a⌋, ⌊b⌋) tell you which rhombus.
 *   3. The fractional parts (fa, fb) tell you WHICH HALF — above the
 *      diagonal fa+fb=1 is up, below is down.
 *
 * The skew comes from v2 = (size/2, h): one v2-step also slides you
 * size/2 in x. So the pixel→lattice inverse must "undo the shear" by
 * subtracting 0.5·b from a.
 *
 * DRAWING METHOD  (raster scan, the approach used here)
 * ──────────────
 *  1. Pick TRI_SIZE — the side length of one triangle in pixels.
 *     Compute h = TRI_SIZE · √3/2 (strip height in pixels).
 *  2. Loop every screen cell (row, col); convert to centred pixel:
 *       px = (col − ox) × CELL_W
 *       py = (row − oy) × CELL_H
 *  3. Skew inverse:  b = py / h,  a = px/size − 0.5·b.
 *  4. Floor + fractional split:  tC=⌊a⌋, tR=⌊b⌋, fa=a−tC, fb=b−tR.
 *  5. Half-rhombus:  tU = (fa + fb ≥ 1) ? up : down.
 *  6. Barycentric weights (l1, l2, l3) — see CONCEPTS for derivation.
 *  7. m = min(l1, l2, l3). If m ≥ BORDER_W → interior, skip.
 *     Otherwise pick the edge character by which weight is smallest.
 *  8. Draw the character in cursor-color if (tC,tR,tU) matches cursor,
 *     else in border-color.
 *
 * KEY FORMULAS
 * ────────────
 *  Forward  (lattice (a, b) → pixel):
 *    px = a·size + b·(size/2)  =  (a + 0.5·b) · size
 *    py = b · h                where h = size · √3/2
 *
 *  Inverse  (pixel → lattice):
 *    b = py / h
 *    a = px/size − 0.5·b
 *
 *  Triangle id from lattice:
 *    col = ⌊a⌋, row = ⌊b⌋
 *    up  = (fa + fb ≥ 1) ? up : down
 *
 *  Barycentric weights:
 *    down: l1 = 1−fa−fb,  l2 = fa,        l3 = fb
 *    up  : l1 = 1−fb,     l2 = fa+fb−1,   l3 = 1−fa
 *
 *  Centroid in lattice (for placing the '@' cursor):
 *    down: (col + 1/3, row + 1/3)
 *    up  : (col + 2/3, row + 2/3)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CELL_W=2, CELL_H=4 → cells are 2× taller than wide. With h = size·√3/2
 *    ≈ 0.866·size, the triangles render at correct equilateral aspect.
 *  • Strip height h is irrational; every lattice computation is float.
 *    The floor() at integer boundaries can be off by 1 ULP, giving 1-pixel
 *    jitter at the cursor. Acceptable for this demo.
 *  • Last terminal row (rows−1) is reserved for the HUD. Raster scan
 *    stops at row < rows−1.
 *  • Resize: ox/oy are recomputed each frame from rows/cols, so the grid
 *    re-centres automatically. Cursor (col, row, up) is independent of
 *    terminal size and survives resize.
 *  • UP from up-triangle has no horizontal edge above it. We toggle inside
 *    the rhombus instead — visually a small up-left step. Two consecutive
 *    UP presses traverse one full strip from any orientation.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At cursor (col, row, up) = (0, 0, down):
 *    centroid lattice = (1/3, 1/3)
 *    centroid pixel   = (1/3 + 0.5·1/3, 1/3) · (size, h)
 *                     = (size/2, h/3)
 *    For TRI_SIZE = 14 (h ≈ 12.12): centroid ≈ (7, 4) pixels →
 *      cell column ≈ 7/CELL_W = 3, cell row ≈ 4/CELL_H = 1.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS 60

/*
 * Cell dimensions — a "sub-pixel" model where each terminal character
 * holds CELL_W × CELL_H sub-pixels. With CELL_W=2, CELL_H=4 the cell is
 * 1:2 wide:tall in sub-pixels, matching real terminal-character aspect.
 */
#define CELL_W 2
#define CELL_H 4

/*
 * TRI_SIZE — side length of one equilateral triangle in pixel units.
 * Strip height h = TRI_SIZE · √3/2 ≈ 0.866 · TRI_SIZE.
 */
#define TRI_SIZE_DEFAULT 14.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

/*
 * BORDER_W — barycentric threshold for "near an edge".
 *   0.10 = thin crisp border with empty interiors.
 *   0.30 = fat border that nearly fills the triangle.
 */
#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.35
#define BORDER_W_STEP    0.02

#define N_THEMES 4

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA 0.05

/* Color pair IDs */
#define PAIR_BORDER 1   /* edge characters between triangles */
#define PAIR_CURSOR 2   /* cursor triangle border + '@' mark */
#define PAIR_HUD    3   /* yellow status bar (top right)     */
#define PAIR_HINT   4   /* cyan key hints (bottom left)      */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEME_FG[N_THEMES] = {
    /* 256-color preferred values, with 8-color fallback below */
     75,   /* steel blue   */
     82,   /* lime green   */
    214,   /* gold         */
     15,   /* bright white */
};
static const short THEME_FG_8[N_THEMES] = {
    COLOR_CYAN, COLOR_GREEN, COLOR_YELLOW, COLOR_WHITE,
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg = (COLORS >= 256) ? THEME_FG[theme] : THEME_FG_8[theme];
    init_pair(PAIR_BORDER, fg, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the pixel ↔ lattice ↔ triangle mapping       */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active triangular grid.
 *
 * The grid is centred on screen. For terminal cell (col, row):
 *   px = (col − ox) × CELL_W      ← pixel relative to screen centre
 *   py = (row − oy) × CELL_H
 * with  ox = cols/2,  oy = (rows−1)/2.
 *
 * Triangle (col=0, row=0, down) has its upper-left corner at pixel (0, 0).
 * CELL_H/CELL_W = 2 matches the ~2:1 terminal character aspect ratio.
 *
 * tri_size and border_w live here because ctx_draw_bg uses them; both are
 * tunable per frame from the main loop (+/-, [/]).
 *
 * max_col / max_row are advisory cursor bounds — the triangular plane is
 * infinite, so "bounds" here means the largest col/row that still places
 * its centroid on screen given the current tri_size.
 */
typedef struct {
    /* terminal extent */
    int rows, cols;

    /* triangle geometry */
    double tri_size;       /* side length in pixels                          */
    double border_w;       /* barycentric threshold for edge proximity       */
    int    cw, ch;         /* sub-pixel scaling — CELL_W, CELL_H             */

    /* screen origin = pixel (0,0) */
    int    ox, oy;

    /* advisory cursor bounds in lattice space */
    int    max_col, max_row;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size.
 *
 * ox/oy are integer cell coordinates of the screen centre.
 * tri_size and border_w default if unset, and are preserved across resize.
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->cw   = CELL_W;
    g->ch   = CELL_H;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
    if (g->tri_size <= 0.0) g->tri_size = TRI_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    g->max_col = (int)((double)cols * CELL_W / g->tri_size) + 1;
    g->max_row = (int)((double)rows * CELL_H / (sqrt(3.0) * 0.5 * g->tri_size)) + 1;
}

/*
 * ctx_pixel_to_tri — solve the skew-lattice inverse and pick the triangle.
 *
 * THE FORMULA (equilateral skew lattice):
 *
 *   h = size · √3 / 2                      ← strip height in px
 *   b = py / h                              ← v2-axis fractional steps
 *   a = px/size − 0.5·b                     ← v1-axis, undoing v2-shear
 *   col = ⌊a⌋,  row = ⌊b⌋
 *   fa  = a − col,  fb = b − row            ← fractional offsets ∈ [0,1)
 *   up  = (fa + fb ≥ 1)                     ← which half of the rhombus
 */
static void ctx_pixel_to_tri(const GridCtx *g, double px, double py,
                             int *col, int *row, int *up,
                             double *fa, double *fb)
{
    double h = g->tri_size * sqrt(3.0) * 0.5;
    double b = py / h;
    double a = px / g->tri_size - 0.5 * b;
    int    c = (int)floor(a);
    int    r = (int)floor(b);
    *col = c;
    *row = r;
    *fa  = a - (double)c;
    *fb  = b - (double)r;
    *up  = (*fa + *fb >= 1.0) ? 1 : 0;
}

/*
 * tri_centroid_pixel — pure forward map for the triangle centroid.
 *
 *   down centroid lattice = (col + 1/3, row + 1/3)
 *   up   centroid lattice = (col + 2/3, row + 2/3)
 *
 *   Forward map:  px = (a + 0.5·b)·size,  py = b·h
 *
 * Pure math helper — keeps its domain name. Used by ctx_to_screen.
 */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

/*
 * ctx_to_screen — terminal cell of the centroid of triangle (col, row, up).
 *
 * Integer truncation (not round) keeps '@' slightly inside the interior so
 * it never lands on a border character and stays visible.
 */
static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *sc = g->ox + (int)(cx_pix / g->cw);
    *sr = g->oy + (int)(cy_pix / g->ch);
}

/*
 * tri_edge_char — barycentric weights → edge proximity → ASCII character.
 *
 * Weights (derivations in MENTAL MODEL):
 *   down: l1 = 1−fa−fb,  l2 = fa,        l3 = fb
 *   up  : l1 = 1−fb,     l2 = fa+fb−1,   l3 = 1−fa
 *
 * The smallest weight names the edge OPPOSITE that vertex. Character map:
 *   down: l1→'/' l2→'\\' l3→'_'
 *   up  : l1→'_' l2→'/'  l3→'\\'
 *
 * Returns the smallest weight via *out_min. The character is returned;
 * if *out_min ≥ border_w the caller treats the cell as interior and skips.
 */
static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    if (up == 0) {           /* down */
        l1 = 1.0 - fa - fb;  ch1 = '/';
        l2 = fa;             ch2 = '\\';
        l3 = fb;             ch3 = '_';
    } else {                 /* up */
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa + fb - 1.0;  ch2 = '/';
        l3 = 1.0 - fa;       ch3 = '\\';
    }
    char   ch = ch1;
    double m  = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/*
 * ctx_draw_bg — raster scan: pixel→triangle→edge character at every cell.
 *
 * The whole grid is recomputed each frame with no stored data. One per-cell
 * call costs 4 multiplies, 1 floor, 3 compares. Resize is free.
 *
 * Cursor highlighting is folded into the same loop: when (tC, tR, tU)
 * matches the cursor, paint with PAIR_CURSOR instead of PAIR_BORDER.
 */
static void ctx_draw_bg(const GridCtx *g, int cC, int cR, int cU)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int    tC, tR, tU;
            double fa, fb, m;
            ctx_pixel_to_tri(g, px, py, &tC, &tR, &tU, &fa, &fb);
            char ch = tri_edge_char(tU, fa, fb, &m);
            if (m >= g->border_w) continue;

            int on_cur = (tC == cC && tR == cR && tU == cU);
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
 * Cursor — just (col, row, up) in lattice triangle space.
 *
 * Bounds and geometry live in GridCtx, not here — the cursor doesn't know
 * how big the grid is, just where in lattice space the user is pointing.
 * The two structs compose: Cursor + GridCtx → screen position via
 * ctx_to_screen().
 *
 * up ∈ {0=down-triangle, 1=up-triangle} within the rhombus (col, row).
 */
typedef struct { int col, row, up; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0;
    cur->row = 0;
    cur->up  = 0;
}

/*
 * TRI_DIR — arrow-key transition table indexed by [direction][current_up].
 * Each entry is (Δcol, Δrow, target_up).
 *
 *   direction:  0=LEFT  1=RIGHT  2=UP  3=DOWN
 *   up index :  0=down  1=up
 *
 *   LEFT   crosses the left slant edge
 *   RIGHT  crosses the right slant edge
 *   UP     crosses the top horizontal (down only); up toggles to down in same rhombus
 *   DOWN   crosses the bottom horizontal (up only); down toggles to up in same rhombus
 *
 * Two presses of UP·UP advance by one full strip from any orientation —
 * a clean idiom for vertical traversal on a triangular lattice.
 */
static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 },    /* down → up(col-1, row)             */
                  {  0,  0,  0 } },  /* up   → down(col, row)  toggle      */
    /* RIGHT */ { {  0,  0,  1 },    /* down → up(col, row)    toggle      */
                  { +1,  0,  0 } },  /* up   → down(col+1, row)            */
    /* UP    */ { {  0, -1,  1 },    /* down → up(col, row-1)  cross top   */
                  {  0,  0,  0 } },  /* up   → down(col, row)  toggle      */
    /* DOWN  */ { {  0,  0,  1 },    /* down → up(col, row)    toggle      */
                  {  0, +1,  0 } },  /* up   → down(col, row+1) cross bot  */
};

/*
 * cursor_move — apply the arrow-key transition for direction `dir`.
 *
 * The triangular plane is infinite; we do not clamp here. (max_col/max_row
 * in GridCtx are advisory — visible-extent only.)
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dir)
{
    (void)g;
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0];
    cur->row += t[1];
    cur->up   = t[2];
}

/*
 * cursor_draw — place '@' at the centroid cell of the current triangle.
 *
 * Uses ctx_to_screen for the triangle→screen conversion. Drawn after
 * ctx_draw_bg so '@' sits on top of any border characters at the centroid.
 */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->col, cur->row, cur->up, &sr, &sc);
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
    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             g->tri_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [01 equilateral] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->col, cur->row, cur->up);
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
    g.tri_size = TRI_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           paused ^= 1; break;
                case 'r':           cursor_reset(&cur, &g); break;
                case 't':
                    theme = (theme + 1) % N_THEMES;
                    color_init(theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, &g, 0); break;
                case KEY_RIGHT: cursor_move(&cur, &g, 1); break;
                case KEY_UP:    cursor_move(&cur, &g, 2); break;
                case KEY_DOWN:  cursor_move(&cur, &g, 3); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) { g.tri_size += TRI_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) { g.tri_size -= TRI_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '[':
                    if (g.border_w > BORDER_W_MIN) { g.border_w -= BORDER_W_STEP; } break;
                case ']':
                    if (g.border_w < BORDER_W_MAX) { g.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns(), dt = now - t0; t0 = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
