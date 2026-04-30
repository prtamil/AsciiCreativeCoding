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
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/02_right_isosceles.c \
 *       -o 02_right_isosceles -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Half-rect tiling. Square cell of side `size` split by
 *                  the '\' diagonal into two right-isosceles triangles
 *                  (legs equal, hypotenuse = leg·√2). Pixel→lattice is
 *                  axis-aligned — no shear.
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
 *  1. Pick TRI_SIZE — the side length of one square in pixels.
 *  2. Loop every screen cell (row, col); convert to centred pixel.
 *  3. Lattice inverse:  a = px/size,  b = py/size.
 *  4. Floor + fractional split: tC=⌊a⌋, tR=⌊b⌋, fa=a−tC, fb=b−tR.
 *  5. Diagonal split:  tU = (fa ≥ fb) ? UR : LL.
 *  6. Compute barycentric weights (l₁, l₂, l₃) for the chosen half.
 *  7. m = min(l₁, l₂, l₃). If m ≥ BORDER_W → interior, skip.
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
 *  • TRI_SIZE measured in pixels (CELL_W·CELL_H sub-pixels per cell);
 *    minimum readable size is ~6 pixels for the diagonal to render.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At cursor (cC, cR, cU) = (0, 0, LL):
 *    centroid lattice = (1/3, 2/3) → pixel (size/3, 2·size/3).
 *    For TRI_SIZE = 16: centroid ≈ (5.3, 10.7) px →
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

#define TRI_SIZE_DEFAULT 16.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.35
#define BORDER_W_STEP    0.02

#define N_THEMES 4

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
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — pixel ↔ lattice ↔ triangle                                */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * pixel_to_tri — axis-aligned lattice inverse + diagonal split.
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
static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *up,
                         double *fa, double *fb)
{
    double inv = 1.0 / size;
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
 * tri_centroid_pixel — forward map for the cursor mark.
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
 * tri_edge_char — barycentric → edge character.
 *
 * UR weights:  l₁ = 1−fa  → '|'   (right edge)
 *              l₂ = fa−fb → '\\'  (diagonal)
 *              l₃ = fb    → '_'   (top edge)
 *
 * LL weights:  l₁ = 1−fb  → '_'   (bottom edge)
 *              l₂ = fa    → '|'   (left edge)
 *              l₃ = fb−fa → '\\'  (diagonal)
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    col, row, up;
    double tri_size;
    double border_w;
    int    theme;
    int    paused;
} Cursor;

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

    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "UR" : "LL",
             cur->tri_size, cur->border_w, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [02 right isosceles] ");
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
