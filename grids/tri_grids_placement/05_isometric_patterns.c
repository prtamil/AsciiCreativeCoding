/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_isometric_patterns.c — preset stamps on the iso (solid-fill) grid
 *
 * DEMO: Iso solid-colour triangular grid (6-cycle palette). Cursor moves
 *       with arrows. Press 1..5 to STAMP a preset pattern at the cursor:
 *         1 = RING    (6 triangles surrounding the cursor)
 *         2 = LINE    (8 triangles in a horizontal strip)
 *         3 = STAR    (RING + 6 outer triangles)
 *         4 = TRI     (cursor + 3 corner triangles forming a triforce)
 *         5 = SCATTER (10 random within 4-step radius)
 *       SPACE clears all objects. 'g' cycles the placed glyph.
 *
 * Study alongside: grids/tri_grids/05_isometric.c (rasterizer),
 *                  05_isometric_direct.c (manual SPACE-toggle on iso),
 *                  01_equilateral_patterns.c (same patterns, edges).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, MAX_OBJ
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — fill palette + cursor / HUD / hint
 *   §4 gridctx  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg
 *   §5 pool     — Pool: place / find / draw
 *   §6 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw
 *   §7 mode     — pattern offset tables + pattern_stamp + pattern_scatter
 *   §8 scene    — hud_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move  1..5:stamp  spc:clear  g:glyph
 *        +/-:size  t:theme  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/05_isometric_patterns.c \
 *       -o 05_isometric_patterns -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Stamp-based placement. Each pattern is a STATIC array
 *                  of (Δcol, Δrow, target_up) triples relative to the
 *                  cursor. Pressing a digit translates the array by the
 *                  cursor and inserts each entry into the pool.
 *
 * Data-structure : Pool — flat array of Obj{col, row, up, glyph, alive}.
 *                  Pattern tables are read-only in §7.
 *
 * The trick      : target_up is ABSOLUTE — orientation depends on (col +
 *                  row) parity in the equilateral lattice, so storing a
 *                  delta would flip the stamp's silhouette every other
 *                  position.
 *
 * Iso twist      : The background is solid-filled by palette_index.
 *                  Stamped glyphs render in REVERSE on the cell's palette
 *                  pair so they pop on any colour.
 *
 * References     :
 *   Triangular tiling — https://en.wikipedia.org/wiki/Triangular_tiling
 *   Object pool pattern — gameprogrammingpatterns.com/object-pool.html
 *   Linear congruential generator — Numerical Recipes ch. 7
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Same stamp mechanic as 01_equilateral_patterns; the underlying paint is
 * solid colour fills instead of edge characters. Patterns are unaware of
 * paint — they manipulate addresses (col, row, up). The iso colour wheel
 * decorates the background; stamped glyphs sit on top in REVERSE.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Rubber stamps on an iso "wall of cubes". Each pattern (RING, LINE,
 * STAR, TRI, SCATTER) is a fixed-offset list; pressing the stamp at the
 * cursor lands a glyph at every offset address. Cubes underneath keep
 * their colours; the glyph row pops in reverse video.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. ctx_draw_bg paints every cell with its palette colour.
 *  3. pool_draw — every placed object's glyph at its centroid cell using
 *     the same palette pair plus REVERSE.
 *  4. cursor_draw — '@' on top.
 *
 *  Stamping (only on key press):
 *    pattern_stamp(pool, PAT_xxx, cur.col, cur.row, glyph)
 *      for each entry (Δc, Δr, up_abs):
 *        pool_place(pool, cur.col+Δc, cur.row+Δr, up_abs, glyph)
 *
 * KEY FORMULAS
 * ────────────
 *  Pattern entry shape:  (Δcol, Δrow, target_up)        [3-tuple]
 *  Sentinel:             { 0xDEAD, 0, 0 }
 *
 *  Centroid lattice → pixel  (h = size · √3 / 2):
 *    ▽: a = col + 1/3,  b = row + 1/3
 *    △: a = col + 2/3,  b = row + 2/3
 *    px = (a + 0.5·b) · size,  py = b · h
 *
 *  Palette hash (paints background only):
 *    k = (col + 2·row + up) mod N_PALETTE
 *
 *  LCG step:
 *    g_seed = g_seed · 1103515245 + 12345
 *    frand  = ((g_seed >> 16) & 0x7FFF) / 32767.0
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Theme cycle: 't' rebuilds palette pairs; objects keep addresses.
 *  • Glyph cycle, MAX_OBJ cap, dedup: identical to the equilateral file.
 *  • SCATTER at high density on top of patterns can fill MAX_OBJ quickly.
 *    SPACE clears.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Press '1' (RING) at origin: 6 stamped glyphs around the cursor on
 *  visibly different palette colours. Press 't': background recolours but
 *  the same triangles still hold the glyphs.
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

#define MAX_OBJ    512
#define N_GLYPHS   6
#define N_PALETTE  6
#define N_THEMES   3

#define FPS_EWMA_ALPHA  0.05

#define PAIR_FILL_BASE  1
#define PAIR_CURSOR    (PAIR_FILL_BASE + N_PALETTE)
#define PAIR_HUD       (PAIR_CURSOR + 1)
#define PAIR_HINT      (PAIR_HUD + 1)

static const char GLYPHS[N_GLYPHS] = { '*', 'o', '+', '#', 'X', '%' };

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
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

static const short PAL256[N_THEMES][N_PALETTE] = {
    { 196, 214, 226, 118,  39, 129 },
    {  39,  45,  82, 226, 207,  51 },
    { 250, 244, 250, 244, 250, 244 },
};
static const short PAL8[N_THEMES][N_PALETTE] = {
    { COLOR_RED,   COLOR_YELLOW, COLOR_GREEN, COLOR_CYAN,    COLOR_BLUE,    COLOR_MAGENTA },
    { COLOR_BLUE,  COLOR_CYAN,   COLOR_GREEN, COLOR_YELLOW,  COLOR_MAGENTA, COLOR_BLUE    },
    { COLOR_WHITE, COLOR_CYAN,   COLOR_BLUE,  COLOR_WHITE,   COLOR_BLUE,    COLOR_CYAN    },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    for (int i = 0; i < N_PALETTE; i++) {
        short bg = (COLORS >= 256) ? PAL256[theme][i] : PAL8[theme][i];
        init_pair(PAIR_FILL_BASE + i, COLOR_BLACK, bg);
    }
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLACK);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx — GridCtx + pixel ↔ lattice + palette                       */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    rows, cols;
    int    cw, ch;
    double tri_size;
    int    ox, oy;
    int    max_col, max_row;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, double tri_size)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->tri_size = tri_size;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->max_col = cols / 2;
    g->max_row = rows / 2;
}

static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *up,
                         double *fa, double *fb)
{
    double h = size * sqrt(3.0) * 0.5;
    double b = py / h;
    double a = px / size - 0.5 * b;
    int    c = (int)floor(a);
    int    r = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c;
    *fb = b - (double)r;
    *up = (*fa + *fb >= 1.0) ? 1 : 0;
}

static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx, double *cy)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx = (a + 0.5 * b) * size;
    *cy = b * h;
}

static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *scol, int *srow)
{
    double cx, cy;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx, &cy);
    *scol = g->ox + (int)(cx / g->cw);
    *srow = g->oy + (int)(cy / g->ch);
}

static int palette_index(int col, int row, int up)
{
    int k = col + 2 * row + up;
    k %= N_PALETTE; if (k < 0) k += N_PALETTE;
    return k;
}

static void ctx_draw_bg(const GridCtx *g)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;
            int    tC, tR, tU;
            double fa, fb;
            pixel_to_tri(px, py, g->tri_size, &tC, &tR, &tU, &fa, &fb);
            int pair = PAIR_FILL_BASE + palette_index(tC, tR, tU);
            attron(COLOR_PAIR(pair));
            mvaddch(row, col, ' ');
            attroff(COLOR_PAIR(pair));
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int  col, row, up;
    char glyph;
    bool alive;
} Obj;

typedef struct {
    Obj items[MAX_OBJ];
    int count;
} Pool;

static void pool_clear(Pool *p) { p->count = 0; }

static int pool_find(const Pool *p, int col, int row, int up)
{
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].alive &&
            p->items[i].col == col && p->items[i].row == row &&
            p->items[i].up == up)
            return i;
    }
    return -1;
}

static void pool_place(Pool *p, int col, int row, int up, char glyph)
{
    if (p->count >= MAX_OBJ || pool_find(p, col, row, up) >= 0) return;
    p->items[p->count++] = (Obj){ col, row, up, glyph, true };
}

static void pool_draw(const Pool *p, const GridCtx *g)
{
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].up,
                      &sc, &sr);
        if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) continue;
        int pair = PAIR_FILL_BASE +
                   palette_index(p->items[i].col, p->items[i].row, p->items[i].up);
        attron(COLOR_PAIR(pair) | A_BOLD | A_REVERSE);
        mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD | A_REVERSE);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int col, row, up;
    int glyph_idx;
    int theme;
    int paused;
} Cursor;

static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 }, {  0,  0,  0 } },
    /* RIGHT */ { {  0,  0,  1 }, { +1,  0,  0 } },
    /* UP    */ { {  0, -1,  1 }, {  0,  0,  0 } },
    /* DOWN  */ { {  0,  0,  1 }, {  0, +1,  0 } },
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0; cur->row = 0; cur->up = 0;
    cur->glyph_idx = 0;
    cur->theme = 0;
    cur->paused = 0;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int dir)
{
    const int *t = TRI_DIR[dir][cur->up];
    int nc = cur->col + t[0];
    int nr = cur->row + t[1];
    if (nc < -g->max_col || nc > g->max_col) return;
    if (nr < -g->max_row || nr > g->max_row) return;
    cur->col = nc; cur->row = nr; cur->up = t[2];
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sc, sr;
    ctx_to_screen(g, cur->col, cur->row, cur->up, &sc, &sr);
    if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) return;
    int pair = PAIR_FILL_BASE + palette_index(cur->col, cur->row, cur->up);
    attron(COLOR_PAIR(pair) | A_BOLD | A_REVERSE);
    mvaddch(sr, sc, '@');
    attroff(COLOR_PAIR(pair) | A_BOLD | A_REVERSE);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  mode — pattern stamps (offset tables + scatter LCG)                 */
/* ═══════════════════════════════════════════════════════════════════════ */

#define PAT_END   { 0xDEAD, 0, 0 }
#define IS_END(p) ((p)[0] == 0xDEAD)

static const int PAT_RING[][3] = {
    {  0,  0,  0 }, {  0,  0,  1 }, { -1,  0,  1 }, { +1,  0,  0 },
    {  0, -1,  1 }, {  0, +1,  0 }, PAT_END
};
static const int PAT_LINE[][3] = {
    {  0,  0,  0 }, {  0,  0,  1 }, {  1,  0,  0 }, {  1,  0,  1 },
    {  2,  0,  0 }, {  2,  0,  1 }, {  3,  0,  0 }, {  3,  0,  1 }, PAT_END
};
static const int PAT_STAR[][3] = {
    {  0,  0,  0 }, {  0,  0,  1 }, { -1,  0,  1 }, { +1,  0,  0 },
    {  0, -1,  1 }, {  0, +1,  0 }, { -1, -1,  1 }, { +1, -1,  0 },
    { -1, +1,  1 }, { +1, +1,  0 }, { -2,  0,  1 }, { +2,  0,  0 }, PAT_END
};
static const int PAT_TRI[][3] = {
    {  0,  0,  0 }, {  0,  0,  1 }, { +1,  0,  0 }, {  0, +1,  1 }, PAT_END
};

static void pattern_stamp(Pool *pool, const int (*pat)[3],
                          int cC, int cR, char glyph)
{
    for (int i = 0; !IS_END(pat[i]); i++)
        pool_place(pool, cC + pat[i][0], cR + pat[i][1], pat[i][2], glyph);
}

static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

static void pattern_scatter(Pool *pool, int cC, int cR, char glyph)
{
    g_seed ^= (unsigned int)clock_ns();
    int n = 10, tries = 0;
    while (n > 0 && tries < 100) {
        int dC = (int)(frand() * 9) - 4;
        int dR = (int)(frand() * 9) - 4;
        int up = frand() > 0.5 ? 1 : 0;
        int prev = pool->count;
        pool_place(pool, cC + dC, cR + dR, up, glyph);
        if (pool->count > prev) n--;
        tries++;
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             p->count, GLYPHS[cur->glyph_idx], g->tri_size,
             cur->theme, fps, cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  1:ring 2:line 3:star 4:tri 5:scatter  spc:clear  g:glyph  +/-:size  q:quit  [05 patterns] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(p, g);
    cursor_draw(cur, g);
    hud_draw(g, cur, p, fps);
    wnoutrefresh(stdscr); doupdate();
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
    color_init(theme); atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    Cursor cur;
    Pool   pool;  pool_clear(&pool);
    GridCtx g;

    cur.col = 0; cur.row = 0; cur.up = 0;
    cur.glyph_idx = 0; cur.theme = 0; cur.paused = 0;
    screen_init(cur.theme);
    ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS;
    int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            ctx_init(&g, LINES, COLS, g.tri_size);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            char glyph = GLYPHS[cur.glyph_idx];
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r':
                    cursor_reset(&cur, &g); pool_clear(&pool);
                    color_init(cur.theme);
                    break;
                case ' ': pool_clear(&pool); break;
                case 'g': cur.glyph_idx = (cur.glyph_idx + 1) % N_GLYPHS; break;
                case '1': pattern_stamp(&pool, PAT_RING,  cur.col, cur.row, glyph); break;
                case '2': pattern_stamp(&pool, PAT_LINE,  cur.col, cur.row, glyph); break;
                case '3': pattern_stamp(&pool, PAT_STAR,  cur.col, cur.row, glyph); break;
                case '4': pattern_stamp(&pool, PAT_TRI,   cur.col, cur.row, glyph); break;
                case '5': pattern_scatter(&pool, cur.col, cur.row, glyph); break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, &g, 0); break;
                case KEY_RIGHT: cursor_move(&cur, &g, 1); break;
                case KEY_UP:    cursor_move(&cur, &g, 2); break;
                case KEY_DOWN:  cursor_move(&cur, &g, 3); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) {
                        g.tri_size += TRI_SIZE_STEP;
                    } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) {
                        g.tri_size -= TRI_SIZE_STEP;
                    } break;
            }
        }
        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&g, &cur, &pool, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
