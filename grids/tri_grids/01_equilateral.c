/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral.c — equilateral triangular grid, the base formula
 *
 * DEMO: Fills the screen with equilateral triangles tiled in alternating
 *       up △ / down ▽ pairs. A '@' cursor sits on the origin triangle.
 *       Arrow keys move it across edges to neighbouring triangles. Each
 *       arrow press traces ONE edge — UP/DOWN cross horizontal edges,
 *       LEFT/RIGHT cross slanted edges. Resize with +/-, border with [/].
 *       This is the root file in the tri_grids series — every other
 *       file modifies one piece of it.
 *
 * Study alongside: grids/rect_grids/01_uniform_rect.c — same pixel-rasterize
 *                  pattern but on an axis-aligned lattice; this file uses
 *                  a 2-axis SKEW lattice because triangles tile with 60°
 *                  basis vectors instead of 90°.
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, BORDER_W
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs: border / cursor / HUD / hint
 *   §4 formula  — pixel ↔ lattice ↔ triangle (col, row, up)
 *   §5 cursor   — TRI_DIR table + cursor_step + cursor_draw
 *   §6 scene    — grid_draw + scene_draw
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
 *                  ▽ (apex-down, base on top) and △ (apex-up, base on bottom)
 *                  separated by the diagonal fa + fb = 1 in lattice space.
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
 *                    ▽: l₁→'/' (right slant), l₂→'\\' (left slant), l₃→'_'
 *                    △: l₁→'_' (bottom),     l₂→'/' (left slant), l₃→'\\'
 *
 * Movement       : (col, row, up) walked by lookup table TRI_DIR[4][2].
 *                  Each arrow key crosses ONE specific edge of the current
 *                  triangle — or toggles within the rhombus when the
 *                  geometry has no edge in that direction. Two presses of
 *                  UP from △ moves up by one full strip:
 *                    △ ─UP→ ▽(same rhombus) ─UP→ △(strip above).
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
 *      diagonal fa+fb=1 is △, below is ▽.
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
 *  5. Half-rhombus:  tU = (fa + fb ≥ 1) ? △ : ▽.
 *  6. Barycentric weights (l₁, l₂, l₃) — see CONCEPTS for derivation.
 *  7. m = min(l₁, l₂, l₃). If m ≥ BORDER_W → interior, skip.
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
 *    up  = (fa + fb ≥ 1) ? △ : ▽
 *
 *  Barycentric weights:
 *    ▽:  l₁ = 1−fa−fb,  l₂ = fa,        l₃ = fb
 *    △:  l₁ = 1−fb,     l₂ = fa+fb−1,   l₃ = 1−fa
 *
 *  Centroid in lattice (for placing the '@' cursor):
 *    ▽: (col + 1/3, row + 1/3)
 *    △: (col + 2/3, row + 2/3)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CELL_W=2, CELL_H=4 → cells are 2× taller than wide. With h = size·√3/2
 *    ≈ 0.866·size, the triangles render at correct equilateral aspect.
 *  • Strip height h is irrational; every lattice computation is float.
 *    The floor() at integer boundaries can be off by 1 ULP, giving 1-pixel
 *    jitter at the cursor. Acceptable for this demo; an industrial-grade
 *    implementation would round-with-tie-breaking.
 *  • Last terminal row (rows−1) is reserved for the HUD. Raster scan
 *    stops at row < rows−1.
 *  • Resize: ox/oy are recomputed each frame from rows/cols, so the grid
 *    re-centres automatically. Cursor (cC, cR, cU) is independent of
 *    terminal size and survives resize.
 *  • UP from △ has no horizontal edge above it. We toggle inside the
 *    rhombus instead — visually a small up-left step. Consistent two-press
 *    UP·UP traverses one full strip from any starting orientation.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At cursor (cC, cR, cU) = (0, 0, ▽):
 *    centroid lattice = (1/3, 1/3)
 *    centroid pixel   = (1/3 + 0.5·1/3, 1/3) · (size, h)
 *                     = (size/2, h/3)
 *    For TRI_SIZE = 14 (h ≈ 12.12): centroid ≈ (7, 4) pixels →
 *      cell column ≈ 7/CELL_W = 3, cell row ≈ 4/CELL_H = 1.
 *
 *  Quick sanity at fa=fb=0 (the corner P00 of any rhombus):
 *    ▽: l₁=1, l₂=0, l₃=0 → on edges P00-P10 (l₃) and P00-P01 (l₂).
 *    The min is 0, two characters tie; pick by argmin lexicographically
 *    (we choose l₂'s character '\\' if it ties l₃).
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
 * 1:2 wide:tall in sub-pixels, matching real terminal-character aspect
 * (~2× taller than wide). Triangles measured in pixel units therefore
 * appear isotropic — equilateral triangles look equilateral.
 */
#define CELL_W 2   /* sub-pixels per terminal column */
#define CELL_H 4   /* sub-pixels per terminal row    */

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

/* Color pair IDs */
#define PAIR_BORDER  1   /* edge characters between triangles */
#define PAIR_CURSOR  2   /* cursor triangle border + '@' mark */
#define PAIR_HUD     3   /* status bar (top right)            */
#define PAIR_HINT    4   /* key hints (bottom left)           */

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
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15  : COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 0   : COLOR_BLACK,  COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75  : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — pixel ↔ lattice ↔ triangle                                */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Grid is centred on screen. For terminal cell (col, row):
 *   px = (col − ox) × CELL_W      ← pixel relative to screen centre
 *   py = (row − oy) × CELL_H
 * with  ox = cols/2,  oy = (rows−1)/2.
 * Triangle (col=0, row=0, ▽) has its upper-left corner at pixel (0, 0).
 *
 * All 12 grids in this series share this §4 structure. Only the lattice
 * shape (basis vectors) and the half-rhombus split differ between files.
 */

/*
 * pixel_to_tri — solve the skew-lattice inverse and pick the triangle.
 *
 * THE FORMULA (equilateral skew lattice):
 *
 *   h = size · √3 / 2                      ← strip height in px
 *   b = py / h                              ← v2-axis fractional steps
 *   a = px/size − 0.5·b                     ← v1-axis, undoing v2-shear
 *   col = ⌊a⌋,  row = ⌊b⌋
 *   fa  = a − col,  fb = b − row            ← fractional offsets ∈ [0,1)
 *   up  = (fa + fb ≥ 1)                     ← which half of the rhombus
 *
 * Returns (col, row, up). Caller passes ox, oy for centring.
 */
static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *up,
                         double *fa, double *fb)
{
    double h = size * sqrt(3.0) * 0.5;
    double b = py / h;
    double a = px / size - 0.5 * b;
    int    c = (int)floor(a);
    int    r = (int)floor(b);
    *col = c;
    *row = r;
    *fa  = a - (double)c;
    *fb  = b - (double)r;
    *up  = (*fa + *fb >= 1.0) ? 1 : 0;
}

/*
 * tri_centroid_pixel — forward lattice→pixel for the triangle centroid.
 *
 *   ▽ centroid lattice = (col + 1/3, row + 1/3)
 *   △ centroid lattice = (col + 2/3, row + 2/3)
 *
 *   Forward map:  px = (a + 0.5·b)·size,  py = b·h
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
 * tri_edge_char — barycentric weights → edge proximity → ASCII character.
 *
 * Weights (derivations in MENTAL MODEL):
 *   ▽:  l₁ = 1−fa−fb,  l₂ = fa,        l₃ = fb
 *   △:  l₁ = 1−fb,     l₂ = fa+fb−1,   l₃ = 1−fa
 *
 * The smallest weight names the edge OPPOSITE that vertex. Character map:
 *   ▽:  l₁→'/'  l₂→'\\'  l₃→'_'
 *   △:  l₁→'_'  l₂→'/'   l₃→'\\'
 *
 * Returns the smallest weight via *out_min. The character is returned;
 * if *out_min ≥ border_w the caller treats the cell as interior and skips.
 */
static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    if (up == 0) {           /* ▽ */
        l1 = 1.0 - fa - fb;  ch1 = '/';
        l2 = fa;             ch2 = '\\';
        l3 = fb;             ch3 = '_';
    } else {                 /* △ */
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    col, row, up;        /* triangle id */
    double tri_size;            /* current side length in pixels */
    double border_w;            /* current edge threshold (barycentric) */
    int    theme;
    int    paused;
} Cursor;

/*
 * TRI_DIR — arrow-key transition table indexed by [direction][current_up].
 * Each entry is (Δcol, Δrow, target_up).
 *
 *   direction:  0=LEFT  1=RIGHT  2=UP  3=DOWN
 *   up index :  0=▽    1=△
 *
 *   LEFT   crosses the left slant edge
 *   RIGHT  crosses the right slant edge
 *   UP     crosses the top horizontal (▽ only); △ toggles to ▽ in same rhombus
 *   DOWN   crosses the bottom horizontal (△ only); ▽ toggles to △ in same rhombus
 *
 * Two presses of UP·UP advance by one full strip from any orientation —
 * a clean idiom for vertical traversal on a triangular lattice.
 */
static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 },    /* ▽ → △(col-1, row)             */
                  {  0,  0,  0 } },  /* △ → ▽(col,   row)  toggle      */
    /* RIGHT */ { {  0,  0,  1 },    /* ▽ → △(col,   row)  toggle      */
                  { +1,  0,  0 } },  /* △ → ▽(col+1, row)             */
    /* UP    */ { {  0, -1,  1 },    /* ▽ → △(col,   row-1) cross top */
                  {  0,  0,  0 } },  /* △ → ▽(col,   row)  toggle      */
    /* DOWN  */ { {  0,  0,  1 },    /* ▽ → △(col,   row)  toggle      */
                  {  0, +1,  0 } },  /* △ → ▽(col,   row+1) cross bot */
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->up = 0;
    cur->tri_size = TRI_SIZE_DEFAULT;
    cur->border_w = BORDER_W_DEFAULT;
    cur->theme    = 0;
    cur->paused   = 0;
}

static void cursor_step(Cursor *cur, int dir)
{
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0];
    cur->row += t[1];
    cur->up   = t[2];
}

/*
 * cursor_draw — place '@' at the centroid cell of the current triangle.
 *
 * Integer truncation (not round) keeps '@' slightly inside the interior
 * so it never lands on a border character and stays visible.
 */
static void cursor_draw(const Cursor *cur, int rows, int cols, int ox, int oy)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(cur->col, cur->row, cur->up, cur->tri_size,
                       &cx_pix, &cy_pix);
    int col = ox + (int)(cx_pix / CELL_W);
    int row = oy + (int)(cy_pix / CELL_H);
    if (col >= 0 && col < cols && row >= 0 && row < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * grid_draw — raster scan: pixel→triangle→edge character at every cell.
 *
 * The whole grid is recomputed each frame with no stored data. One per-cell
 * call costs 4 multiplies, 1 floor, 3 compares. Resize is free.
 */
static void grid_draw(int rows, int cols, const Cursor *cur, int ox, int oy)
{
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;

            int    tC, tR, tU;
            double fa, fb, m;
            pixel_to_tri(px, py, cur->tri_size, &tC, &tR, &tU, &fa, &fb);
            char ch = tri_edge_char(tU, fa, fb, &m);
            if (m >= cur->border_w) continue;

            int on_cur = (tC == cur->col && tR == cur->row && tU == cur->up);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

static void scene_draw(int rows, int cols, const Cursor *cur, double fps)
{
    erase();
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur, ox, oy);
    cursor_draw(cur, rows, cols, ox, oy);

    /* HUD — top right */
    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             cur->tri_size, cur->border_w, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Key hints — bottom left */
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [01 equilateral] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
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

    Cursor cur;
    cursor_reset(&cur);
    screen_init(cur.theme);

    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           cur.paused ^= 1; break;
                case 'r':           cursor_reset(&cur); color_init(cur.theme); break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_step(&cur, 0); break;
                case KEY_RIGHT: cursor_step(&cur, 1); break;
                case KEY_UP:    cursor_step(&cur, 2); break;
                case KEY_DOWN:  cursor_step(&cur, 3); break;
                case '+': case '=':
                    if (cur.tri_size < TRI_SIZE_MAX) { cur.tri_size += TRI_SIZE_STEP; } break;
                case '-':
                    if (cur.tri_size > TRI_SIZE_MIN) { cur.tri_size -= TRI_SIZE_STEP; } break;
                case '[':
                    if (cur.border_w > BORDER_W_MIN) { cur.border_w -= BORDER_W_STEP; } break;
                case ']':
                    if (cur.border_w < BORDER_W_MAX) { cur.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;

        scene_draw(rows, cols, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
