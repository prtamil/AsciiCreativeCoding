/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal_path.c — line-of-sight path on the tetrakis grid
 *
 * DEMO: Two markers — START (S) and END (E) — sit on a tetrakis grid
 *       (each square split by both diagonals into N/E/S/W wedges).
 *       Move '@' with arrows; 's' sets START at cursor, 'e' sets END.
 *       The path is computed by walking pixel coordinates along the
 *       centroid-to-centroid line and recording which wedge each
 *       sampled pixel lies in.
 *
 * Study alongside: 03_double_diagonal_direct.c (manual placement),
 *                  grids/tri_grids/03_double_diagonal.c (rasterizer),
 *                  02_right_isosceles_path.c (1-diagonal sibling).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, MAX_OBJ
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 7 pairs: edge / cursor / start / end / path / HUD / hint
 *   §4 gridctx  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg
 *   §5 pool     — Pool: place / find / clear / draw  (used as path)
 *   §6 cursor   — Cursor + TETRA_DIR + reset / move / draw + START / END
 *   §7 mode     — pixel walk between two centroids → wedge list
 *   §8 scene    — hud_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move  s:set-start  e:set-end  spc:clear-path
 *        +/-:size  t:theme  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/03_double_diagonal_path.c \
 *       -o 03_double_diagonal_path -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Line-rasterize between two wedge centroids in PIXEL
 *                  space; for each sampled pixel, ask pixel_to_tri
 *                  "which (col, row, wedge) triangle owns me?" and record
 *                  uniques. The result is the ordered set of N/E/S/W
 *                  wedges traversed by the straight line.
 *
 * Data-structure : Pool — flat array of Obj{col, row, wedge, glyph,
 *                  alive}. pool_place deduplicates via pool_find.
 *
 * Sampling step  : ~size/4 pixels — fine enough that no wedge is
 *                  skipped. Wedges are smaller than the underlying
 *                  square (4 per square), so the step must stay tight.
 *
 * References     :
 *   Tetrakis square tiling — https://en.wikipedia.org/wiki/Tetrakis_square_tiling
 *   Bresenham line — https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 *   Line-of-sight on grids — Red Blob Games
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Stretch a string from S to E across an X-cut grid of squares. The
 * path is the ordered list of wedges the string passes through. Sample
 * the string finely in pixel space; at each sample, classify the
 * underlying wedge with the same formula ctx_draw_bg uses.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The grid is a square lattice with both diagonals drawn — every
 * square holds 4 triangular wedges. Where the line crosses a side
 * it changes square; where it crosses a diagonal it changes wedge
 * inside the same square. The path captures every such transition
 * by sampling more often than half the wedge size.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. ctx_draw_bg — raster scan: pixel_to_tri → tri_edge_char draws
 *     '/', '\\', '|', '_' near edges of each wedge.
 *  3. pool_draw — '*' at each path entry's centroid screen cell.
 *  4. marker_draw — 'S' at start, 'E' at end.
 *  5. cursor_draw — '@' at the cursor address.
 *  6. hud_draw — top-right status, bottom-row hint.
 *
 *  path_compute runs only on START/END change or size change.
 *
 * KEY FORMULAS
 * ────────────
 *  Pixel → wedge:
 *    a = px / size,   b = py / size
 *    col = ⌊a⌋,        row = ⌊b⌋
 *    dx = fa-0.5,      dy = fb-0.5
 *    wedge = (|dx|>|dy|) ? (dx>0?E:W) : (dy>0?S:N)
 *
 *  Wedge centroid (1/3 of the way from apex to opposite edge midpoint):
 *    N: a = col+1/2,  b = row+1/6
 *    E: a = col+5/6,  b = row+1/2
 *    S: a = col+1/2,  b = row+5/6
 *    W: a = col+1/6,  b = row+1/2
 *    px = a · size,   py = b · size
 *
 *  Walk parameters: dist, n, step = size/4 — same as 02_*_path.
 *
 *  Walk loop:
 *    for i in 0..n:
 *      t  = i / n
 *      px = sx + t·dx,  py = sy + t·dy
 *      pixel_to_tri(px, py) → (tC, tR, tW)
 *      pool_place(tC, tR, tW)         // dedup
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Wedge density: 4 wedges per square means a horizontal line of
 *    one-square length can record 2–3 wedges. Keep step ≤ size/4.
 *  • Apex point: at the centre of any square the line passes through
 *    a single pixel where all 4 wedges meet. The classifier breaks
 *    the tie deterministically by ≥ on |dy|, so the path won't
 *    "flicker" between wedges.
 *  • Recompute on size change: '+'/'-' must call path_compute.
 *  • Zero-length, MAX_OBJ cap: identical to 02_*_path.
 *
 * HOW TO VERIFY
 * ─────────────
 *  START and END both at N wedge of (0,0): path size = 1.
 *  END at E wedge of (0,0): path size ≥ 2 (passes through the apex).
 *  END at N wedge of (1,0): path size = 2 (one square step right).
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

#define TRI_SIZE_DEFAULT 18.0
#define TRI_SIZE_MIN      8.0
#define TRI_SIZE_MAX     48.0
#define TRI_SIZE_STEP     2.0
#define BORDER_W          0.10

#define MAX_OBJ    1024
#define N_THEMES   4

#define FPS_EWMA_ALPHA 0.05

#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_START  3
#define PAIR_END    4
#define PAIR_PATH   5
#define PAIR_HUD    6
#define PAIR_HINT   7

static const char *DIR_NAME[4] = { "N", "E", "S", "W" };

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

static const short THEME_FG[N_THEMES][3] = {
    {  82,  82, 196 }, {  39, 226, 207 },
    { 207,  82,  39 }, {  15,  82, 196 },
};
static const short THEME_FG_8[N_THEMES][3] = {
    { COLOR_GREEN,   COLOR_GREEN,  COLOR_RED     },
    { COLOR_BLUE,    COLOR_YELLOW, COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_GREEN,  COLOR_BLUE    },
    { COLOR_WHITE,   COLOR_GREEN,  COLOR_RED     },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e, fg_s, fg_n;
    if (COLORS >= 256) {
        fg_e = THEME_FG[theme][0]; fg_s = THEME_FG[theme][1]; fg_n = THEME_FG[theme][2];
    } else {
        fg_e = THEME_FG_8[theme][0]; fg_s = THEME_FG_8[theme][1]; fg_n = THEME_FG_8[theme][2];
    }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_START,  fg_s, -1);
    init_pair(PAIR_END,    fg_n, -1);
    init_pair(PAIR_PATH,   COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    rows, cols;
    int    cw, ch;
    int    ox, oy;
    double tri_size;
    double border_w;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, double tri_size)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->tri_size = tri_size;
    g->border_w = BORDER_W;
}

static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *wedge,
                         double *fa, double *fb)
{
    double inv = 1.0 / size;
    double a = px * inv, b = py * inv;
    int    c = (int)floor(a), r = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c; *fb = b - (double)r;
    double dx = *fa - 0.5, dy = *fb - 0.5;
    double adx = fabs(dx), ady = fabs(dy);
    if (adx > ady) *wedge = (dx > 0.0) ? DIR_E : DIR_W;
    else           *wedge = (dy > 0.0) ? DIR_S : DIR_N;
}

static void tri_centroid_pixel(int col, int row, int wedge, double size,
                               double *cx, double *cy)
{
    double a, b;
    switch (wedge) {
        case DIR_N: a = 0.5;     b = 1.0/6.0; break;
        case DIR_E: a = 5.0/6.0; b = 0.5;     break;
        case DIR_S: a = 0.5;     b = 5.0/6.0; break;
        default:    a = 1.0/6.0; b = 0.5;     break;
    }
    *cx = ((double)col + a) * size;
    *cy = ((double)row + b) * size;
}

static char tri_edge_char(int wedge, double fa, double fb, double *out_min)
{
    double l1, l2, l3; char ch1, ch2, ch3;
    switch (wedge) {
        case DIR_N:
            l1 = 1.0 - fa - fb;     ch1 = '/';
            l2 = fa - fb;           ch2 = '\\';
            l3 = 2.0 * fb;          ch3 = '_';
            break;
        case DIR_E:
            l1 = fa - fb;           ch1 = '\\';
            l2 = fa + fb - 1.0;     ch2 = '/';
            l3 = 2.0 * (1.0 - fa);  ch3 = '|';
            break;
        case DIR_S:
            l1 = fb - fa;           ch1 = '\\';
            l2 = fa + fb - 1.0;     ch2 = '/';
            l3 = 2.0 * (1.0 - fb);  ch3 = '_';
            break;
        default:
            l1 = 1.0 - fa - fb;     ch1 = '\\';
            l2 = fb - fa;           ch2 = '/';
            l3 = 2.0 * fa;          ch3 = '|';
            break;
    }
    char ch = ch1; double m = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

static void ctx_to_screen(const GridCtx *g, int col, int row, int wedge,
                          int *scol, int *srow)
{
    double cx, cy;
    tri_centroid_pixel(col, row, wedge, g->tri_size, &cx, &cy);
    *scol = g->ox + (int)(cx / g->cw);
    *srow = g->oy + (int)(cy / g->ch);
}

static void ctx_draw_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;
            int tC, tR, tW; double fa, fb, m;
            pixel_to_tri(px, py, g->tri_size, &tC, &tR, &tW, &fa, &fb);
            char ch = tri_edge_char(tW, fa, fb, &m);
            if (m >= g->border_w) continue;
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int col, row, wedge; char glyph; bool alive; } Obj;
typedef struct { Obj items[MAX_OBJ];  int count; } Pool;

static int pool_find(const Pool *p, int col, int row, int wedge)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].alive
            && p->items[i].col == col
            && p->items[i].row == row
            && p->items[i].wedge == wedge)
            return i;
    return -1;
}

static void pool_place(Pool *p, int col, int row, int wedge, char glyph)
{
    if (pool_find(p, col, row, wedge) >= 0) return;
    if (p->count >= MAX_OBJ) return;
    p->items[p->count++] = (Obj){ col, row, wedge, glyph, true };
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].wedge,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int col, row, wedge;
    int sCol, sRow, sWedge;     /* START marker */
    int eCol, eRow, eWedge;     /* END   marker */
    int has_start, has_end;
    int theme;
    int paused;
} Cursor;

static const int TETRA_DIR[4][4][3] = {
    { {  0,  0, DIR_W }, {  0,  0, DIR_W }, {  0,  0, DIR_W }, { -1,  0, DIR_E } },
    { {  0,  0, DIR_E }, { +1,  0, DIR_W }, {  0,  0, DIR_E }, {  0,  0, DIR_E } },
    { {  0, -1, DIR_S }, {  0,  0, DIR_N }, {  0,  0, DIR_N }, {  0,  0, DIR_N } },
    { {  0,  0, DIR_S }, {  0,  0, DIR_S }, {  0, +1, DIR_N }, {  0,  0, DIR_S } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->wedge = DIR_N;
    cur->has_start = 0; cur->has_end = 0;
    cur->theme  = 0;
    cur->paused = 0;
}

static void cursor_move(Cursor *cur, int arrow)
{
    const int *t = TETRA_DIR[arrow][cur->wedge];
    cur->col   += t[0];
    cur->row   += t[1];
    cur->wedge  = t[2];
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sc, sr;
    ctx_to_screen(g, cur->col, cur->row, cur->wedge, &sc, &sr);
    if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) return;
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, sc, '@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

static void marker_draw(const GridCtx *g, int col, int row, int wedge,
                        char glyph, int pair)
{
    int sc, sr;
    ctx_to_screen(g, col, row, wedge, &sc, &sr);
    if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sr, sc, glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  mode — pixel walk between START and END centroids                   */
/* ═══════════════════════════════════════════════════════════════════════ */

static void path_compute(Pool *p, const Cursor *cur, const GridCtx *g)
{
    pool_clear(p);
    if (!cur->has_start || !cur->has_end) return;

    double sx, sy, ex, ey;
    tri_centroid_pixel(cur->sCol, cur->sRow, cur->sWedge, g->tri_size, &sx, &sy);
    tri_centroid_pixel(cur->eCol, cur->eRow, cur->eWedge, g->tri_size, &ex, &ey);

    double dx = ex - sx, dy = ey - sy;
    double dist = sqrt(dx*dx + dy*dy);
    if (dist < 1e-6) {
        pool_place(p, cur->sCol, cur->sRow, cur->sWedge, '*');
        return;
    }
    double step = g->tri_size * 0.25;
    int    n    = (int)(dist / step) + 1;
    for (int i = 0; i <= n; i++) {
        double t  = (double)i / (double)n;
        double px = sx + t * dx;
        double py = sy + t * dy;
        int    tC, tR, tW; double fa, fb;
        pixel_to_tri(px, py, g->tri_size, &tC, &tR, &tW, &fa, &fb);
        pool_place(p, tC, tR, tW, '*');
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, const Pool *path,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  path:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, DIR_NAME[cur->wedge],
             path->count, g->tri_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  s:set-start  e:set-end  spc:clear  +/-:size  t:theme  r:reset  q:quit  [03 path] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, const Pool *path,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(path, g);
    if (cur->has_start)
        marker_draw(g, cur->sCol, cur->sRow, cur->sWedge, 'S', PAIR_START);
    if (cur->has_end)
        marker_draw(g, cur->eCol, cur->eRow, cur->eWedge, 'E', PAIR_END);
    cursor_draw(cur, g);
    hud_draw(g, cur, path, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
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

    Cursor cur;  cursor_reset(&cur);
    Pool   path; pool_clear(&path);
    screen_init(cur.theme);

    GridCtx g;   ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS, g.tri_size);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r':
                    cursor_reset(&cur); pool_clear(&path);
                    ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);
                    color_init(cur.theme);
                    break;
                case ' ':
                    cur.has_start = 0; cur.has_end = 0; pool_clear(&path);
                    break;
                case 's':
                    cur.sCol = cur.col; cur.sRow = cur.row; cur.sWedge = cur.wedge;
                    cur.has_start = 1; path_compute(&path, &cur, &g);
                    break;
                case 'e':
                    cur.eCol = cur.col; cur.eRow = cur.row; cur.eWedge = cur.wedge;
                    cur.has_end = 1; path_compute(&path, &cur, &g);
                    break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, 0); break;
                case KEY_RIGHT: cursor_move(&cur, 1); break;
                case KEY_UP:    cursor_move(&cur, 2); break;
                case KEY_DOWN:  cursor_move(&cur, 3); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) {
                        g.tri_size += TRI_SIZE_STEP; path_compute(&path, &cur, &g);
                    } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) {
                        g.tri_size -= TRI_SIZE_STEP; path_compute(&path, &cur, &g);
                    } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA)
            + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &cur, &path, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
