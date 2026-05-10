/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_isometric.c — equilateral grid rendered as solid colored blocks
 *
 * DEMO: Same triangular tiling as 01, but each triangle is filled with
 *       a solid color and the colors rotate through a 6-cycle so
 *       neighbouring triangles around any vertex spell out the
 *       characteristic "stacked cubes" isometric pattern. Arrow keys
 *       move the cursor across edges.
 *
 * Study alongside: 01_equilateral.c — same lattice, same cursor logic;
 *                  this file fills cells (background color) instead of
 *                  drawing edge characters.
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, FPS_EWMA_ALPHA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6-color rotating palette + cursor / HUD / HINT
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_pixel_to_tri /
 *                 ctx_draw_bg + tri_centroid_pixel + palette_index
 *   §5 cursor   — Cursor + cursor_reset / cursor_move, TRI_DIR (same as 01)
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   t theme   p pause
 *        +/- size        q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/05_isometric.c \
 *       -o 05_isometric -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : The equilateral triangular tiling is the standard
 *                  "isometric grid" used in graphics — three axes 120°
 *                  apart, six triangles meeting at every vertex. To make
 *                  the iso character pop visually, each triangle gets a
 *                  color from a 6-cycle indexed by (col + 2·row + up) mod 6.
 *                  Pairs of triangles that share a horizontal edge pick
 *                  adjacent indices, so the screen reads as stacked rhombi
 *                  (= cube faces in iso projection).
 *
 * Data-structure : Two structs — GridCtx (terminal extent, tri_size,
 *                  CELL_W/CELL_H, screen origin ox/oy) and Cursor
 *                  (col, row, up) — same as 01. No grid array.
 *
 * Formula        : Same skew-lattice pixel→triangle as 01_equilateral.c.
 *                  Then palette_index(col, row, up) = (col + 2·row + up) mod 6.
 *                  No edge characters — each cell is filled with the
 *                  triangle's background color.
 *
 * Edge chars     : None — the visual boundary between triangles is the
 *                  color discontinuity itself. No '/'/'\\'/'_' rendering.
 *
 * Movement       : Same TRI_DIR as 01_equilateral.c.
 *
 * References     :
 *   Isometric projection — https://en.wikipedia.org/wiki/Isometric_projection
 *   Triangular tiling    — https://en.wikipedia.org/wiki/Triangular_tiling
 *   Red Blob Games (iso) — https://www.redblobgames.com/grids/hexagons/  (axial = iso)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The equilateral triangular grid IS the isometric grid — the projection
 * of a 3-D cubic lattice viewed along its main diagonal. Six triangles
 * meet at every vertex; if we color them in a 6-cycle, the result reads
 * as stacked cubes with three visible faces (top, left, right) shaded
 * differently. No special projection math — just the lattice from 01
 * plus a hash that picks one of 6 colors per triangle.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a stack of unit cubes viewed from the (1,1,1) direction. Each
 * cube projects to a hexagon, and the hexagon decomposes into 3 rhombi
 * (the cube's 3 visible faces) which further decompose into 6 triangles.
 * Adjacent triangles in the same rhombus get similar colors; triangles in
 * different rhombi (= different cube faces) get contrasting colors.
 *
 * The hash (col + 2·row + up) mod 6 achieves this without any explicit
 * "which face am I on?" reasoning. The "·2" gives different colors to
 * triangles in adjacent strips at the same column; the "+up" splits
 * down vs up in the same rhombus into different colors. Cycling through 6
 * indices around any vertex produces the stacked-cube illusion.
 *
 * DRAWING METHOD  (raster scan, the approach used here)
 * ──────────────
 *  1. Pick TRI_SIZE — side length in pixels.
 *  2. Loop every screen cell; convert to centred pixel.
 *  3. Skew lattice inverse → (col, row, up) as in 01.
 *  4. p = palette_index(col, row, up).
 *  5. Draw a SPACE in the cell with background color = palette[p].
 *  6. If this cell is the cursor, overwrite with '@' in the cursor color.
 *
 * KEY FORMULAS
 * ────────────
 *  Same lattice inverse as 01_equilateral.c.
 *
 *  Palette hash:
 *    k = (col + 2·row + up) mod 6     (positive remainder)
 *
 *  Why "·2" and "+up": at any vertex of the triangular tiling, six
 *  triangles meet in a fan. Walking around the vertex CCW visits them
 *  in an order whose (col, row, up) sequence yields six DISTINCT residues
 *  mod 6. That's the stacked-cube property in a single line.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The "filled cell" rendering relies on the terminal's background-color
 *    paint. On terminals that don't honour color pairs for spaces, this
 *    file shows a blank screen. Modern xterm/iTerm/foot are fine.
 *  • Negative col or row: the modulo of a negative integer can be
 *    negative in C. We force a positive remainder with "if (k < 0) k += 6".
 *  • Cursor visibility: the '@' is drawn with the SAME background color
 *    as the underlying triangle, plus A_REVERSE — so it's always visible
 *    no matter which palette slot the triangle sits in.
 *  • CELL_W=2, CELL_H=4 produces the correct equilateral aspect.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At cursor (col, row, up) = (0, 0, 0): palette = (0+0+0) mod 6 = 0.
 *  Move cursor RIGHT → (0, 0, 1) → palette = 1.
 *  Move cursor RIGHT again → (1, 0, 0) → palette = 1. (Same color as the
 *    previous up — they are the same hex face in the iso analogy.)
 *  Move cursor RIGHT again → (1, 0, 1) → palette = 2.
 *  → Walking right cycles through 0, 1, 1, 2, 2, 3, 3, … (each color
 *  used twice in a row because adjacent down/up pairs share a face).
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

#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 16.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

#define N_PALETTE  6
#define N_THEMES   3

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_FILL_BASE  1                              /* 1..6 */
#define PAIR_CURSOR    (PAIR_FILL_BASE + N_PALETTE)
#define PAIR_HUD       (PAIR_CURSOR + 1)
#define PAIR_HINT      (PAIR_HUD + 1)

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

/*
 * Three iso-flavoured 6-cycle palettes. Each row is a 6-color rotation —
 * 256-color values (preferred) and 8-color fallback row.
 */
static const short PAL256[N_THEMES][N_PALETTE] = {
    /* warm  */ { 196, 214, 226, 118, 39, 129 },
    /* cool  */ {  21,  39,  82, 226, 207, 21  },
    /* mono  */ {  15,  87,  39,   0, 39, 87   },
};
static const short PAL8[N_THEMES][N_PALETTE] = {
    { COLOR_RED,  COLOR_YELLOW, COLOR_GREEN, COLOR_CYAN,    COLOR_BLUE,    COLOR_MAGENTA },
    { COLOR_BLUE, COLOR_CYAN,   COLOR_GREEN, COLOR_YELLOW,  COLOR_MAGENTA, COLOR_BLUE    },
    { COLOR_WHITE,COLOR_CYAN,   COLOR_BLUE,  COLOR_BLACK,   COLOR_BLUE,    COLOR_CYAN    },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < N_PALETTE; i++) {
        short bg = (COLORS >= 256) ? PAL256[theme][i] : PAL8[theme][i];
        /* fg=black so the SPACE character is invisible — only background shows */
        init_pair(PAIR_FILL_BASE + i, COLOR_BLACK, bg);
    }
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLACK);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx, pixel ↔ lattice + palette hash                   */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active iso (equilateral) grid.
 *
 * Same skew-lattice geometry as 01; no border_w because rendering is
 * solid-fill (no edge characters).
 */
typedef struct {
    /* terminal extent */
    int rows, cols;

    /* triangle geometry */
    double tri_size;       /* side length in pixels                          */
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
    if (g->tri_size <= 0.0) g->tri_size = TRI_SIZE_DEFAULT;
    g->max_col = (int)((double)cols * CELL_W / g->tri_size) + 1;
    g->max_row = (int)((double)rows * CELL_H / (sqrt(3.0) * 0.5 * g->tri_size)) + 1;
}

static void ctx_pixel_to_tri(const GridCtx *g, double px, double py,
                             int *col, int *row, int *up,
                             double *fa, double *fb)
{
    double h = g->tri_size * sqrt(3.0) * 0.5;
    double b = py / h;
    double a = px / g->tri_size - 0.5 * b;
    int    c = (int)floor(a);
    int    r = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c;
    *fb = b - (double)r;
    *up = (*fa + *fb >= 1.0) ? 1 : 0;
}

/*
 * tri_centroid_pixel — pure forward map for cursor mark (same as 01).
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

__attribute__((unused))
static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *sc = g->ox + (int)(cx_pix / g->cw);
    *sr = g->oy + (int)(cy_pix / g->ch);
}

/*
 * palette_index — assign each triangle a 6-cycle color slot.
 *
 *   k = (col + 2·row + up) mod 6
 *
 * The "·2" gives different colors to triangles in adjacent strips at the
 * same column; the "+up" gives different colors to down vs up in the same
 * rhombus. Around any vertex, the 6 distinct slots appear in cyclic order
 * — the visual signature of an isometric "stack of cubes".
 */
static int palette_index(int col, int row, int up)
{
    int k = col + 2 * row + up;
    k %= N_PALETTE;
    if (k < 0) k += N_PALETTE;
    return k;
}

/*
 * ctx_draw_bg — solid-fill rendering. Each cell gets the background color
 * matching its triangle's palette slot. Cursor cell overlaid with '@' in
 * reverse video so it is visible regardless of palette slot.
 */
static void ctx_draw_bg(const GridCtx *g, int cC, int cR, int cU)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int    tC, tR, tU;
            double fa, fb;
            ctx_pixel_to_tri(g, px, py, &tC, &tR, &tU, &fa, &fb);

            int p    = palette_index(tC, tR, tU);
            int pair = PAIR_FILL_BASE + p;
            attron(COLOR_PAIR(pair));
            mvaddch(row, col, ' ');
            attroff(COLOR_PAIR(pair));

            int on_cur = (tC == cC && tR == cR && tU == cU);
            if (on_cur) {
                attron(COLOR_PAIR(pair) | A_BOLD | A_REVERSE);
                mvaddch(row, col, '@');
                attroff(COLOR_PAIR(pair) | A_BOLD | A_REVERSE);
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — same as 01_equilateral.c: (col, row, up) on the equilateral
 * lattice. The visual cursor mark is rendered inside ctx_draw_bg (so it
 * lands on the correct palette colour), so this file does not need a
 * separate cursor_draw().
 */
typedef struct { int col, row, up; } Cursor;

/* Same TRI_DIR as 01_equilateral.c */
static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 }, {  0,  0,  0 } },
    /* RIGHT */ { {  0,  0,  1 }, { +1,  0,  0 } },
    /* UP    */ { {  0, -1,  1 }, {  0,  0,  0 } },
    /* DOWN  */ { {  0,  0,  1 }, {  0, +1,  0 } },
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             g->tri_size, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  +/-:size  t:theme  r:reset  p:pause  q:quit  [05 isometric] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->col, cur->row, cur->up);
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
