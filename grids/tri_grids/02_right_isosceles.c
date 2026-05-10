/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_right_isosceles.c — square grid bisected by one diagonal
 *
 * DEMO: Each terminal-screen square is split by a single '\' diagonal into
 *       an upper-right (UR) and lower-left (LL) right-isosceles triangle.
 *       A '@' cursor sits on the origin triangle. Arrow keys move it
 *       across edges — UP/DOWN cross horizontal, LEFT/RIGHT cross vertical,
 *       and a toggle across the diagonal where the geometry has no edge
 *       in that direction.
 *
 * Study alongside: 01_equilateral.c — same pixel-rasterize + cursor pattern;
 *                  this file uses an axis-aligned (NOT skew) lattice.
 *                  03_double_diagonal.c — both diagonals (4 tris/cell).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, CELL_SIZE, BORDER_W, FPS_EWMA_ALPHA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs: border / cursor / HUD / HINT
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
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/02_right_isosceles.c \
 *       -o 02_right_isosceles -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Half-rect tiling. Square cell of side `cell_size` split
 *                  by the '\' diagonal into two right-isosceles triangles
 *                  (legs equal, hypotenuse = leg·√2). Pixel→lattice is
 *                  axis-aligned — no shear.
 *
 * Data-structure : Two structs — GridCtx (terminal extent, cell_size,
 *                  CELL_W/CELL_H, screen origin ox/oy, border_w) and
 *                  Cursor (col, row, up). No grid array; every pixel
 *                  resolves its (col, row, up) per frame.
 *
 * Formula        : pixel → lattice (square cell):
 *                    a = px / size
 *                    b = py / size
 *                  (col, row) = (⌊a⌋, ⌊b⌋), and inside the unit square the
 *                  diagonal fa = fb separates UR (fa ≥ fb) from LL (fa < fb).
 *
 * Edge chars     : Barycentric weights pick the edge character.
 *                  UR (vertices A=(0,0), B=(1,0), C=(1,1)):
 *                    l_A → '|' (right edge), l_B → '\\' (diagonal),
 *                    l_C → '_' (top edge)
 *                  LL (vertices A=(0,0), C=(1,1), D=(0,1)):
 *                    l_A → '_' (bottom),    l_C → '|' (left edge),
 *                    l_D → '\\' (diagonal)
 *
 * Movement       : (col, row, up) walked by lookup table TRI_DIR[4][2].
 *                  Each square is reachable from any starting triangle in
 *                  at most 2 presses; the diagonal toggle is what handles
 *                  the directions that have no matching edge.
 *
 * References     :
 *   Right-isosceles tiling — https://en.wikipedia.org/wiki/Triangular_tiling
 *   Tetrakis (companion)   — https://en.wikipedia.org/wiki/Tetrakis_square_tiling
 *   Barycentric coords     — https://en.wikipedia.org/wiki/Barycentric_coordinate_system
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The plane is tiled with square cells, just like a chessboard. Inside
 * every square is a single diagonal — the '\' line — splitting it into
 * two right-angled triangles. To find which triangle owns a pixel, ask
 * "which square am I in?" then "am I above or below the diagonal?". Two
 * cheap modulo-style tests; no data structure, no array.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture the rectangular grid from rect_grids/01 — same uniform squares.
 * Then draw the '\' diagonal in each square. Now each square is split in
 * half. The upper-right half (UR) lives ABOVE the diagonal in screen
 * y-down terms (i.e., where x > y in fractional cell coords); the lower-
 * left half (LL) lives BELOW (where x < y).
 *
 * The same skew-free lattice as the rect grid: a = px/size, b = py/size.
 * No basis vectors needed because the cells are axis-aligned squares.
 *
 * DRAWING METHOD  (raster scan, the approach used here)
 * ──────────────
 *  1. Pick CELL_SIZE — the side length of one square in pixels.
 *  2. Loop every screen cell (row, col); convert to centred pixel.
 *  3. Lattice inverse:  a = px/size,  b = py/size.
 *  4. Floor + fractional split: tC=⌊a⌋, tR=⌊b⌋, fa=a−tC, fb=b−tR.
 *  5. Diagonal split:  tU = (fa ≥ fb) ? UR : LL.
 *  6. Compute barycentric weights (l1, l2, l3) for the chosen half.
 *  7. m = min(l1, l2, l3). If m ≥ BORDER_W → interior, skip.
 *     Otherwise pick the edge character by which weight is smallest.
 *  8. Draw in cursor color if (tC,tR,tU) matches cursor, else border.
 *
 * KEY FORMULAS
 * ────────────
 *  Forward  (cell (col, row) → pixel of upper-left corner):
 *    px = col · size,  py = row · size
 *
 *  Inverse  (pixel → cell + fractional offset):
 *    a = px / size,  b = py / size
 *    col = ⌊a⌋,  row = ⌊b⌋
 *    fa  = a − col,  fb = b − row
 *    up  = (fa ≥ fb) ? UR : LL
 *
 *  Barycentric weights:
 *    UR (A=(0,0), B=(1,0), C=(1,1)):
 *      l_A = 1−fa,  l_B = fa−fb,  l_C = fb
 *    LL (A=(0,0), C=(1,1), D=(0,1)):
 *      l_A = 1−fb,  l_C = fa,     l_D = fb−fa
 *
 *  Centroid for placing '@' cursor:
 *    UR: (col + 2/3, row + 1/3)
 *    LL: (col + 1/3, row + 2/3)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Floating-point exactness on the diagonal: when fa==fb the test
 *    "fa ≥ fb" arbitrarily picks UR. Either side works because the
 *    diagonal pixel is always a border anyway.
 *  • Last terminal row reserved for HUD; raster stops at row < rows−1.
 *  • Resize: ox/oy recompute each frame from rows/cols → grid recentres.
 *  • UP from LL has no horizontal top edge (LL's top is the diagonal).
 *    The toggle to UR in same square is geometrically a small up-right
 *    step, providing visual "up" motion via two presses of UP·UP.
 *  • CELL_SIZE measured in pixels (CELL_W·CELL_H sub-pixels per cell);
 *    minimum readable size is ~6 pixels for the diagonal to render.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At cursor (col, row, up) = (0, 0, LL):
 *    centroid lattice = (1/3, 2/3) → pixel (size/3, 2·size/3).
 *    For CELL_SIZE = 16: centroid ≈ (5.3, 10.7) px →
 *      cell column ≈ 5/CELL_W = 2, cell row ≈ 10/CELL_H = 2.
 *
 *  Quick edge-char sanity inside LL at (fa, fb) = (0.5, 0.6):
 *    l_A = 1−0.6 = 0.4   (far from bottom edge)
 *    l_C = 0.5           (far from left edge)
 *    l_D = 0.6−0.5 = 0.1 (close to diagonal)
 *  → min is l_D, character is '\\' — sitting near the diagonal. ✓
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS 60

/*
 * Cell sub-pixel model (see 01_equilateral.c §1 for derivation).
 * CELL_W=2, CELL_H=4 keeps shapes geometrically isotropic.
 */
#define CELL_W 2
#define CELL_H 4

#define CELL_SIZE_DEFAULT 16.0
#define CELL_SIZE_MIN      6.0
#define CELL_SIZE_MAX     40.0
#define CELL_SIZE_STEP     2.0

#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.35
#define BORDER_W_STEP    0.02

#define N_THEMES 4

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_HUD    3
#define PAIR_HINT   4

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

static const short THEME_FG[N_THEMES]   = {  75,  207, 214,  15 };
static const short THEME_FG_8[N_THEMES] = {
    COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW, COLOR_WHITE,
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg = (COLORS >= 256) ? THEME_FG[theme] : THEME_FG_8[theme];
    init_pair(PAIR_BORDER, fg, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the pixel ↔ lattice ↔ triangle mapping       */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active right-isosceles grid.
 *
 * Same centring scheme as 01_equilateral.c: ox = cols/2, oy = (rows-1)/2.
 * cell_size and border_w are tunable per frame from the main loop.
 */
typedef struct {
    /* terminal extent */
    int rows, cols;

    /* square geometry */
    double cell_size;      /* side length in pixels                          */
    double border_w;       /* barycentric threshold for edge proximity       */
    int    cw, ch;         /* sub-pixel scaling — CELL_W, CELL_H             */

    /* screen origin = pixel (0,0) */
    int    ox, oy;

    /* advisory cursor bounds in lattice space */
    int    max_col, max_row;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->cw   = CELL_W;
    g->ch   = CELL_H;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
    if (g->cell_size <= 0.0) g->cell_size = CELL_SIZE_DEFAULT;
    if (g->border_w  <= 0.0) g->border_w  = BORDER_W_DEFAULT;
    g->max_col = (int)((double)cols * CELL_W / g->cell_size) + 1;
    g->max_row = (int)((double)rows * CELL_H / g->cell_size) + 1;
}

/*
 * ctx_pixel_to_tri — axis-aligned lattice inverse + diagonal split.
 *
 * THE FORMULA (right-isosceles half-rect):
 *
 *   a = px / size,   b = py / size              ← unit-square coords
 *   col = ⌊a⌋,       row = ⌊b⌋
 *   fa  = a − col,   fb  = b − row              ← in [0, 1)
 *   up  = (fa ≥ fb) ? UR : LL                   ← '\' diagonal split
 *
 * UR has its right angle at (1,0); LL has its right angle at (0,1).
 */
static void ctx_pixel_to_tri(const GridCtx *g, double px, double py,
                             int *col, int *row, int *up,
                             double *fa, double *fb)
{
    double inv = 1.0 / g->cell_size;
    double a   = px * inv;
    double b   = py * inv;
    int    c   = (int)floor(a);
    int    r   = (int)floor(b);
    *col = c;  *row = r;
    *fa  = a - (double)c;
    *fb  = b - (double)r;
    *up  = (*fa >= *fb) ? 1 : 0;
}

/*
 * tri_centroid_pixel — pure forward map for cursor mark.
 *
 *   UR centroid lattice = ((0+1+1)/3, (0+0+1)/3) = (2/3, 1/3)
 *   LL centroid lattice = ((0+1+0)/3, (0+1+1)/3) = (1/3, 2/3)
 */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double a = (up == 1) ? ((double)col + 2.0/3.0) : ((double)col + 1.0/3.0);
    double b = (up == 1) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = a * size;
    *cy_pix = b * size;
}

/*
 * ctx_to_screen — terminal cell of the centroid of triangle (col, row, up).
 */
static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->cell_size, &cx_pix, &cy_pix);
    *sc = g->ox + (int)(cx_pix / g->cw);
    *sr = g->oy + (int)(cy_pix / g->ch);
}

/*
 * tri_edge_char — barycentric → edge character.
 *
 * UR weights:  l1 = 1−fa  → '|'   (right edge)
 *              l2 = fa−fb → '\\'  (diagonal)
 *              l3 = fb    → '_'   (top edge)
 *
 * LL weights:  l1 = 1−fb  → '_'   (bottom edge)
 *              l2 = fa    → '|'   (left edge)
 *              l3 = fb−fa → '\\'  (diagonal)
 */
static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    if (up == 1) {           /* UR */
        l1 = 1.0 - fa;       ch1 = '|';
        l2 = fa - fb;        ch2 = '\\';
        l3 = fb;             ch3 = '_';
    } else {                 /* LL */
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa;             ch2 = '|';
        l3 = fb - fa;        ch3 = '\\';
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
 * Cursor highlight folded into the same loop.
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
 * up ∈ {0=LL (lower-left), 1=UR (upper-right)} within the square (col, row).
 */
typedef struct { int col, row, up; } Cursor;

/*
 * TRI_DIR — arrow-key transitions.
 *   index 0:LEFT  1:RIGHT  2:UP  3:DOWN
 *   row   0:LL    1:UR
 *
 *   LEFT:   LL → UR(col-1, row)        — across left edge
 *           UR → LL same square         — across diagonal (fits "left")
 *   RIGHT:  UR → LL(col+1, row)        — across right edge
 *           LL → UR same square         — across diagonal (fits "right")
 *   UP:     UR → LL(col, row-1)        — across top edge
 *           LL → UR same square         — toggle (LL has no top edge)
 *   DOWN:   LL → UR(col, row+1)        — across bottom edge
 *           UR → LL same square         — toggle (UR has no bottom edge)
 */
static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 }, {  0,  0,  0 } },
    /* RIGHT */ { {  0,  0,  1 }, { +1,  0,  0 } },
    /* UP    */ { {  0,  0,  1 }, {  0, -1,  0 } },
    /* DOWN  */ { {  0, +1,  1 }, {  0,  0,  0 } },
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0;
    cur->row = 0;
    cur->up  = 0;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int dir)
{
    (void)g;
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0];
    cur->row += t[1];
    cur->up   = t[2];
}

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

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "UR" : "LL",
             g->cell_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [02 right isosceles] ");
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
    g.cell_size = CELL_SIZE_DEFAULT;
    g.border_w  = BORDER_W_DEFAULT;
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
                    if (g.cell_size < CELL_SIZE_MAX) { g.cell_size += CELL_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '-':
                    if (g.cell_size > CELL_SIZE_MIN) { g.cell_size -= CELL_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
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
