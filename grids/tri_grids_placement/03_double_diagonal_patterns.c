/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal_patterns.c — preset stamps on the tetrakis grid
 *
 * DEMO: Cursor moves with arrows on a tetrakis grid (each square split
 *       by both diagonals into 4 N/E/S/W wedges). Press 1..5 to STAMP
 *       a preset pattern at the cursor:
 *         1 = RING    (4 wedges around the cursor's square)
 *         2 = LINE    (horizontal strip of wedges)
 *         3 = STAR    (RING + outer ring)
 *         4 = TRI     (3-wedge triangular cluster)
 *         5 = SCATTER (random wedges within a small box)
 *       SPACE clears all objects. 'g' cycles the placed glyph.
 *
 * Study alongside: 03_double_diagonal_direct.c (manual SPACE-toggle),
 *                  grids/tri_grids/03_double_diagonal.c (rasterizer),
 *                  02_right_isosceles_patterns.c (1-diagonal sibling).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, MAX_OBJ
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: edge / cursor / object / HUD / hint
 *   §4 gridctx  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg
 *   §5 pool     — Pool: place / remove / toggle / find / clear / draw
 *   §6 cursor   — Cursor + TETRA_DIR + reset / move / draw
 *   §7 mode     — pattern offset tables + pattern_stamp + pattern_scatter
 *   §8 scene    — hud_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move  1..5:stamp  spc:clear  g:glyph
 *        +/-:size  t:theme  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/03_double_diagonal_patterns.c \
 *       -o 03_double_diagonal_patterns -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Stamp-based placement. Each pattern is a STATIC array
 *                  of (Δcol, Δrow, target_wedge) triples relative to the
 *                  cursor. Pressing a digit translates the array by the
 *                  cursor and inserts each entry into the pool.
 *
 * Data-structure : Pool — flat array of Obj{col, row, wedge, glyph,
 *                  alive}. Pattern tables are read-only in §7. SCATTER
 *                  picks random offsets and a random wedge via LCG.
 *
 * The trick      : target_wedge is ABSOLUTE (one of N/E/S/W), not a
 *                  delta. Each (col, row) holds 4 wedges; the stamp's
 *                  silhouette must not rotate when translated, so we
 *                  store wedge directly per entry.
 *
 * References     :
 *   Tetrakis square tiling — https://en.wikipedia.org/wiki/Tetrakis_square_tiling
 *   Object pool pattern — gameprogrammingpatterns.com/object-pool.html
 *   Linear congruential generator — Numerical Recipes ch. 7
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A pattern is a static list of (Δcol, Δrow, wedge) entries. Pressing
 * '1' translates the RING list by the cursor and inserts each entry
 * into the pool. The cursor never moves; only objects appear.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think rubber stamps on X-cut graph paper. The RING stamp's ink dots
 * are pre-placed at the four wedges around the cursor's square; the
 * STAR stamp adds the outer ring. SCATTER generates a fresh random
 * stamp on each press.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. ctx_draw_bg — raster scan: pixel_to_tri → tri_edge_char draws
 *     '/', '\\', '|', '_' near each wedge's edges.
 *  3. pool_draw — every placed object's glyph at its wedge centroid.
 *  4. cursor_draw — '@' on top.
 *  5. hud_draw — top-right status, bottom-row hint.
 *
 *  Stamping (only on key press):
 *    pattern_stamp(pool, PAT_xxx, cur.col, cur.row, glyph)
 *      for each entry (Δc, Δr, wedge_abs):
 *        pool_place(pool, cur.col+Δc, cur.row+Δr, wedge_abs, glyph)
 *
 * KEY FORMULAS
 * ────────────
 *  Pattern entry shape:  (Δcol, Δrow, target_wedge)        [3-tuple]
 *  Sentinel:             { 0xDEAD, 0, 0 }
 *  Iteration:            for i in 0..; while !IS_END(pat[i])
 *
 *  Wedge centroid (used by pool_draw, see §4 of the file):
 *    N: a = col+1/2, b = row+1/6
 *    E: a = col+5/6, b = row+1/2
 *    S: a = col+1/2, b = row+5/6
 *    W: a = col+1/6, b = row+1/2
 *    px = a · size, py = b · size
 *
 *  Why ABSOLUTE target_wedge: every (col, row) holds all 4 wedges
 *  simultaneously, so the stamp's footprint is a fixed shape relative
 *  to the cursor regardless of where it lands. A delta would have no
 *  meaning — there is no "relative direction" between two wedges of
 *  the same square.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • MAX_OBJ cap: large STAR plus repeated SCATTER saturates; new
 *    entries silently dropped. SPACE clears.
 *  • Glyph cycle: glyph used by next stamp comes from
 *    GLYPHS[cur.glyph_idx] AT stamp time.
 *  • Pattern overlap: pool_place deduplicates; stamping a RING twice at
 *    the same cursor has no effect.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Press '1' (RING) at the origin: 4 wedges placed at (0,0,N), (0,0,E),
 *  (0,0,S), (0,0,W) — all four wedges of the cursor's square.
 *  Press '2' (LINE): a horizontal strip of wedges along row 0.
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

#define MAX_OBJ    512
#define N_GLYPHS   6
#define N_THEMES   4

#define FPS_EWMA_ALPHA 0.05

#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_OBJECT 3
#define PAIR_HUD    4
#define PAIR_HINT   5

static const char  GLYPHS[N_GLYPHS] = { '*', 'o', '+', '#', 'X', '%' };
static const char *DIR_NAME[4]      = { "N", "E", "S", "W" };

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

static const short THEME_FG[N_THEMES][2] = {
    {  82, 226 }, { 207, 226 }, { 207,  82 }, {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_GREEN,   COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_GREEN  },
    { COLOR_WHITE,   COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e = (COLORS >= 256) ? THEME_FG[theme][0] : THEME_FG_8[theme][0];
    short fg_o = (COLORS >= 256) ? THEME_FG[theme][1] : THEME_FG_8[theme][1];
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
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
    attron(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].wedge,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int col, row, wedge;
    int glyph_idx;
    int theme;
    int paused;
} Cursor;

static const int TETRA_DIR[4][4][3] = {
    /* LEFT  */ { {  0,  0, DIR_W }, {  0,  0, DIR_W }, {  0,  0, DIR_W }, { -1,  0, DIR_E } },
    /* RIGHT */ { {  0,  0, DIR_E }, { +1,  0, DIR_W }, {  0,  0, DIR_E }, {  0,  0, DIR_E } },
    /* UP    */ { {  0, -1, DIR_S }, {  0,  0, DIR_N }, {  0,  0, DIR_N }, {  0,  0, DIR_N } },
    /* DOWN  */ { {  0,  0, DIR_S }, {  0,  0, DIR_S }, {  0, +1, DIR_N }, {  0,  0, DIR_S } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->wedge = DIR_N;
    cur->glyph_idx = 0;
    cur->theme     = 0;
    cur->paused    = 0;
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  mode — pattern stamps                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

#define PAT_END   { 0xDEAD, 0, 0 }
#define IS_END(p) ((p)[0] == 0xDEAD)

static const int PAT_RING[][3] = {
    {  0,  0, DIR_N }, {  0,  0, DIR_E }, {  0,  0, DIR_S }, {  0,  0, DIR_W },
    PAT_END
};
static const int PAT_LINE[][3] = {
    {  0, 0, DIR_N }, {  0, 0, DIR_E }, {  0, 0, DIR_S }, {  0, 0, DIR_W },
    {  1, 0, DIR_N }, {  1, 0, DIR_E }, {  1, 0, DIR_S }, {  1, 0, DIR_W },
    PAT_END
};
static const int PAT_STAR[][3] = {
    {  0,  0, DIR_N }, {  0,  0, DIR_E }, {  0,  0, DIR_S }, {  0,  0, DIR_W },
    { -1,  0, DIR_E }, { +1,  0, DIR_W },
    {  0, -1, DIR_S }, {  0, +1, DIR_N },
    { -1, -1, DIR_S }, { +1, +1, DIR_N },
    PAT_END
};
static const int PAT_TRI[][3] = {
    {  0,  0, DIR_N }, {  0,  0, DIR_E }, {  0,  0, DIR_W },
    PAT_END
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
        int dC    = (int)(frand() * 9) - 4;
        int dR    = (int)(frand() * 9) - 4;
        int wedge = (int)(frand() * 4);
        int prev  = pool->count;
        pool_place(pool, cC + dC, cR + dR, wedge, glyph);
        if (pool->count > prev) n--;
        tries++;
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, const Pool *pool,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, DIR_NAME[cur->wedge],
             pool->count, GLYPHS[cur->glyph_idx], g->tri_size,
             cur->theme, fps, cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  1:ring 2:line 3:star 4:tri 5:scatter  spc:clear  g:glyph  +/-:size  q:quit  [03 patterns] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, const Pool *pool,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(pool, g);
    cursor_draw(cur, g);
    hud_draw(g, cur, pool, fps);
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
    Pool   pool; pool_clear(&pool);
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
            char glyph = GLYPHS[cur.glyph_idx];
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r':
                    cursor_reset(&cur); pool_clear(&pool);
                    ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);
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
                case KEY_LEFT:  cursor_move(&cur, 0); break;
                case KEY_RIGHT: cursor_move(&cur, 1); break;
                case KEY_UP:    cursor_move(&cur, 2); break;
                case KEY_DOWN:  cursor_move(&cur, 3); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) g.tri_size += TRI_SIZE_STEP;
                    break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) g.tri_size -= TRI_SIZE_STEP;
                    break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA)
            + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &cur, &pool, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
