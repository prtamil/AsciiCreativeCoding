/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_right_isosceles_patterns.c — preset pattern stamps on the half-rect grid
 *
 * DEMO: Cursor moves with arrows on a square grid bisected by '\' into
 *       UR / LL right-isosceles triangles. Press 1..5 to STAMP a preset
 *       pattern at the cursor:
 *         1 = RING    (cursor + neighbours)
 *         2 = LINE    (8-tri horizontal strip)
 *         3 = STAR    (RING + outer ring)
 *         4 = TRI     (cursor + 3 corner triangles)
 *         5 = SCATTER (10 random within 4-step radius)
 *       SPACE clears all objects. 'g' cycles the placed glyph.
 *
 * Study alongside: 02_right_isosceles_direct.c (manual SPACE-toggle),
 *                  grids/tri_grids/02_right_isosceles.c (rasterizer),
 *                  01_equilateral_patterns.c (same idea, equilateral).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, MAX_OBJ
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: edge / cursor / object / HUD / hint
 *   §4 gridctx  — GridCtx + pixel/centroid/edge formula
 *   §5 pool     — Pool: place / find / clear / draw
 *   §6 cursor   — Cursor + TRI_DIR + reset / move / draw
 *   §7 patterns — pattern offset tables + pattern_stamp + pattern_scatter
 *   §8 scene    — hud_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move  1..5:stamp  spc:clear  g:glyph
 *        +/-:size  t:theme  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/02_right_isosceles_patterns.c \
 *       -o 02_right_isosceles_patterns -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Stamp-based placement. Each pattern is a STATIC array
 *                  of (Δcol, Δrow, target_up) triples relative to the
 *                  cursor. Pressing a digit translates the array by the
 *                  cursor and inserts each entry into the pool.
 *
 * Data-structure : Pool — flat array of Obj{col, row, up, glyph, alive}.
 *                  Pattern tables are read-only in §7. SCATTER uses an
 *                  LCG (g_seed) to pick random offsets within a ±4 box.
 *
 * The trick      : target_up is ABSOLUTE (UR=1, LL=0), not a delta.
 *                  On the half-rect lattice every (col, row) holds one
 *                  UR and one LL — the stamp's silhouette must not
 *                  flip when translated, so we store UR/LL directly.
 *
 * References     :
 *   Half-rect tiling — https://en.wikipedia.org/wiki/Triangular_tiling
 *   Object pool pattern — gameprogrammingpatterns.com/object-pool.html
 *   Linear congruential generator — Numerical Recipes ch. 7
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A pattern is a static list of relative addresses. Pressing '1' is
 * "translate the RING list by the cursor and insert each entry into
 * the pool". The cursor never moves; only objects appear. Adding a
 * new pattern is just adding another array — pure compile-time data.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think rubber stamps on graph-paper-with-diagonals. Each pattern
 * (RING, LINE, STAR, TRI, SCATTER) is a stamp whose ink dots are
 * fixed offsets from a centre. Pressing the stamp at the cursor lands
 *   (cur->col + Δcol, cur->row + Δrow, target_up)
 * for each entry. SCATTER is the same idea but generates a random
 * stamp on every press.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. ctx_draw_bg — raster scan: pixel_to_tri → tri_edge_char draws
 *     '|', '_', '\\' near triangle edges.
 *  3. pool_draw — every placed object's glyph at its centroid cell.
 *  4. cursor_draw — '@' on top.
 *
 *  Stamping (only on key press, not per frame):
 *    pattern_stamp(pool, PAT_xxx, cur.col, cur.row, glyph)
 *      for each entry (Δc, Δr, up_abs):
 *        pool_place(pool, cur.col+Δc, cur.row+Δr, up_abs, glyph)
 *
 * KEY FORMULAS
 * ────────────
 *  Pattern entry shape:  (Δcol, Δrow, target_up)        [3-tuple]
 *  Sentinel:             { 0xDEAD, 0, 0 }
 *  Iteration:            for i in 0..; while !IS_END(pat[i])
 *
 *  SCATTER (LCG, see §7):
 *    g_seed ^= clock_ns()
 *    dC = floor(frand·9) - 4    ; dR = floor(frand·9) - 4
 *    up = (frand > 0.5) ? UR : LL
 *    pool_place(cur.col+dC, cur.row+dR, up, glyph)
 *
 *  Why ABSOLUTE target_up: the half-rect lattice always has both UR
 *  and LL at every (col, row); the choice is not parity-dependent
 *  (unlike the equilateral grid). Storing absolute orientations keeps
 *  the stamp's shape invariant under translation, period.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • MAX_OBJ cap: large STAR (12 entries) plus repeated SCATTER
 *    saturates; new entries silently dropped. SPACE clears.
 *  • Glyph cycle: glyph used by next stamp comes from
 *    GLYPHS[g->glyph_idx] AT stamp time. Already-stamped entries
 *    keep their original glyph.
 *  • Pattern overlap: pool_place deduplicates; stamping a RING twice
 *    has no effect (the entries already exist).
 *  • SCATTER bound: 100 retries, 10 entries — if the area near the
 *    cursor is already full, fewer than 10 dots may land.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Press '1' (RING): a small cluster of 6 entries around the cursor.
 *  Press '2' (LINE): 8 entries forming a horizontal strip across 4
 *  squares. Move the cursor and press '1' again — a new ring at the
 *  new address; the old ring remains because patterns ADD, not
 *  REPLACE.
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
#define BORDER_W   0.10
#define MAX_OBJ    512
#define N_GLYPHS   6
#define N_THEMES   4

#define FPS_EWMA_ALPHA 0.05

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_OBJECT 3
#define PAIR_HUD    4
#define PAIR_HINT   5

static const char GLYPHS[N_GLYPHS] = { '*', 'o', '+', '#', 'X', '%' };

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
    {  75, 226 }, { 207, 226 }, {  82, 207 }, {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_YELLOW  },
    { COLOR_MAGENTA, COLOR_YELLOW  },
    { COLOR_GREEN,   COLOR_MAGENTA },
    { COLOR_WHITE,   COLOR_CYAN    },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e = (COLORS >= 256) ? THEME_FG[theme][0] : THEME_FG_8[theme][0];
    short fg_o = (COLORS >= 256) ? THEME_FG[theme][1] : THEME_FG_8[theme][1];
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    rows, cols;
    double tri_size;
    int    cell_w, cell_h;
    int    ox, oy;
    double border_w;
    int    theme;
    int    paused;
    int    glyph_idx;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows     = rows;
    g->cols     = cols;
    g->tri_size = TRI_SIZE_DEFAULT;
    g->cell_w   = CELL_W;
    g->cell_h   = CELL_H;
    g->ox       = cols / 2;
    g->oy       = (rows - 1) / 2;
    g->border_w = BORDER_W;
    g->theme    = 0;
    g->paused   = 0;
    g->glyph_idx = 0;
}

static void ctx_resize(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->ox = cols / 2; g->oy = (rows - 1) / 2;
}

static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *up,
                         double *fa, double *fb)
{
    double inv = 1.0 / size;
    double a = px * inv, b = py * inv;
    int c = (int)floor(a), r = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c; *fb = b - (double)r;
    *up = (*fa >= *fb) ? 1 : 0;
}

static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double a = (up == 1) ? ((double)col + 2.0/3.0) : ((double)col + 1.0/3.0);
    double b = (up == 1) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = a * size; *cy_pix = b * size;
}

static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *scol = g->ox + (int)(cx_pix / g->cell_w);
    *srow = g->oy + (int)(cy_pix / g->cell_h);
}

static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3; char ch1, ch2, ch3;
    if (up == 1) {
        l1 = 1.0 - fa;       ch1 = '|';
        l2 = fa - fb;        ch2 = '\\';
        l3 = fb;             ch3 = '_';
    } else {
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa;             ch2 = '|';
        l3 = fb - fa;        ch3 = '\\';
    }
    char ch = ch1; double m = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

static void ctx_draw_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;
            int tC, tR, tU; double fa, fb, m;
            pixel_to_tri(px, py, g->tri_size, &tC, &tR, &tU, &fa, &fb);
            char ch = tri_edge_char(tU, fa, fb, &m);
            if (m >= g->border_w) continue;
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int col, row, up; char glyph; bool alive; } Obj;
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

static int pool_find(const Pool *p, int col, int row, int up)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].alive
            && p->items[i].col == col
            && p->items[i].row == row
            && p->items[i].up  == up)
            return i;
    return -1;
}

static void pool_place(Pool *p, int col, int row, int up, char glyph)
{
    if (pool_find(p, col, row, up) >= 0) return;
    if (p->count >= MAX_OBJ) return;
    p->items[p->count++] = (Obj){ col, row, up, glyph, true };
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].up,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int col, row, up; } Cursor;

static const int TRI_DIR[4][2][3] = {
    { { -1,  0,  1 }, {  0,  0,  0 } },
    { {  0,  0,  1 }, { +1,  0,  0 } },
    { {  0,  0,  1 }, {  0, -1,  0 } },
    { {  0, +1,  1 }, {  0,  0,  0 } },
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0; cur->row = 0; cur->up = 0;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int dcol, int drow, int dup)
{
    (void)g;
    cur->col += dcol; cur->row += drow; cur->up = dup;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sc, sr;
    ctx_to_screen(g, cur->col, cur->row, cur->up, &sc, &sr);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  patterns                                                            */
/* ═══════════════════════════════════════════════════════════════════════ */

#define PAT_END { 0xDEAD, 0, 0 }
#define IS_END(p) ((p)[0] == 0xDEAD)

static const int PAT_RING[][3] = {
    {  0,  0,  0 }, {  0,  0,  1 },
    { -1,  0,  1 }, { +1,  0,  0 },
    {  0, -1,  1 }, {  0, +1,  0 },
    PAT_END
};
static const int PAT_LINE[][3] = {
    {  0, 0, 0 }, {  0, 0, 1 }, {  1, 0, 0 }, {  1, 0, 1 },
    {  2, 0, 0 }, {  2, 0, 1 }, {  3, 0, 0 }, {  3, 0, 1 },
    PAT_END
};
static const int PAT_STAR[][3] = {
    {  0,  0,  0 }, {  0,  0,  1 },
    { -1,  0,  1 }, { +1,  0,  0 },
    {  0, -1,  1 }, {  0, +1,  0 },
    { -1, -1,  1 }, { +1, -1,  0 }, { -1, +1,  1 }, { +1, +1,  0 },
    { -2,  0,  1 }, { +2,  0,  0 },
    PAT_END
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

static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "UR" : "LL",
             p->count, GLYPHS[g->glyph_idx], g->tri_size,
             g->theme, fps, g->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  1:ring 2:line 3:star 4:tri 5:scatter  spc:clear  g:glyph  +/-:size  q:quit  [02 patterns] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(p, g);
    cursor_draw(cur, g);
    hud_draw(g, p, cur, fps);
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

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal); signal(SIGWINCH, on_signal);

    GridCtx g; ctx_init(&g, 0, 0);
    screen_init(g.theme);
    ctx_init(&g, LINES, COLS);

    Cursor cur; cursor_reset(&cur, &g);
    Pool   pool; pool_clear(&pool);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS; int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            ctx_resize(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            char glyph = GLYPHS[g.glyph_idx];
            const int *t;
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': g.paused ^= 1; break;
                case 'r': cursor_reset(&cur, &g); pool_clear(&pool); color_init(g.theme); break;
                case ' ': pool_clear(&pool); break;
                case 'g': g.glyph_idx = (g.glyph_idx + 1) % N_GLYPHS; break;
                case '1': pattern_stamp(&pool, PAT_RING,  cur.col, cur.row, glyph); break;
                case '2': pattern_stamp(&pool, PAT_LINE,  cur.col, cur.row, glyph); break;
                case '3': pattern_stamp(&pool, PAT_STAR,  cur.col, cur.row, glyph); break;
                case '4': pattern_stamp(&pool, PAT_TRI,   cur.col, cur.row, glyph); break;
                case '5': pattern_scatter(&pool, cur.col, cur.row, glyph); break;
                case 't': g.theme = (g.theme + 1) % N_THEMES; color_init(g.theme); break;
                case KEY_LEFT:  t = TRI_DIR[0][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_RIGHT: t = TRI_DIR[1][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_UP:    t = TRI_DIR[2][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_DOWN:  t = TRI_DIR[3][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) { g.tri_size += TRI_SIZE_STEP; } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) { g.tri_size -= TRI_SIZE_STEP; } break;
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
