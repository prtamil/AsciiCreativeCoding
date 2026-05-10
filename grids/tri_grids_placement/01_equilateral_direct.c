/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral_direct.c — direct object placement on the equilateral grid
 *
 * DEMO: An equilateral triangular grid fills the screen. Move '@' between
 *       triangles with arrow keys. Press SPACE to toggle a '*' object at
 *       the cursor triangle. Objects are stored by lattice address
 *       (col, row, up) and survive resize — they follow their triangle
 *       when the terminal changes size. 'g' cycles the placed glyph.
 *
 * Study alongside: grids/tri_grids/01_equilateral.c (background rasterizer),
 *                  grids/hex_grids_placement/01_hex_direct.c (same idea on hex).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, MAX_OBJ
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: edge / cursor / object / HUD / hint
 *   §4 gridctx  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg helpers
 *   §5 pool     — Pool: place / remove / toggle / find / clear / draw
 *   §6 cursor   — Cursor + TRI_DIR + cursor_reset / cursor_move / cursor_draw
 *   §7 scene    — hud_draw + scene_draw
 *   §8 screen   — ncurses init / cleanup
 *   §9 app      — signals, main loop
 *
 * Keys:  arrows:move  spc:toggle  g:glyph  C:clear  r:reset
 *        +/-:size  t:theme  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/01_equilateral_direct.c \
 *       -o 01_equilateral_direct -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Direct placement using a movable cursor. The cursor
 *                  holds (col, row, up) in TRIANGLE-lattice space. SPACE
 *                  toggles an object at that address; the object's screen
 *                  position is recomputed each frame from the centroid
 *                  formula in ctx_to_screen.
 *
 * Data-structure : Pool — flat array of Obj{col, row, up, glyph, alive}.
 *                  Removal swaps the dead slot with the last item (O(1)).
 *
 * Rendering      : Three-pass per frame:
 *                    (1) ctx_draw_bg rasterizes the equilateral background
 *                    (2) pool_draw renders each placed object at its centroid
 *                    (3) cursor_draw places '@' at the cursor centroid
 *                  The cursor draws over objects so the user always sees it.
 *
 * References     :
 *   Triangular tiling — https://en.wikipedia.org/wiki/Triangular_tiling
 *   Object pool pattern — gameprogrammingpatterns.com/object-pool.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two address spaces share the screen at once:
 *   1. The GRID — a function of every screen pixel via the equilateral
 *      skew lattice. No data; reformed every frame in ctx_draw_bg().
 *   2. The OBJECTS — a tiny array of (col, row, up, glyph) records in
 *      Pool. The cursor is just one more such address. SPACE toggles
 *      whether the cursor address is in the pool.
 * Drawing is grid first, then objects, then cursor. The cursor draws
 * on top so it is always visible.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the triangular tiling as an infinite address book — every
 * triangle has a unique 3-tuple (col, row, up). The screen is a window
 * onto that address book, re-centred each frame from
 * (ox=cols/2, oy=(rows-1)/2). Objects are tagged by ADDRESS, not by
 * pixel position; resizing the terminal moves the WINDOW, but the
 * objects keep their addresses and reappear at new pixel positions.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. ctx_draw_bg — raster scan: for every screen cell, pixel_to_tri →
 *     tri_edge_char → mvaddch when min-weight < border_w.
 *  3. pool_draw — for each object, ctx_to_screen → mvaddch the glyph
 *     at that screen cell.
 *  4. cursor_draw — same path as pool_draw but '@' on top.
 *  5. hud_draw; wnoutrefresh + doupdate.
 *
 * KEY FORMULAS
 * ────────────
 *  Cursor step (4-direction lookup, see TRI_DIR in §6):
 *    (cur->col, cur->row, cur->up) ← TRI_DIR[dir][cur->up]
 *
 *  Centroid lattice → pixel  (h = size · √3 / 2):
 *    ▽ centroid  a = col + 1/3,   b = row + 1/3
 *    △ centroid  a = col + 2/3,   b = row + 2/3
 *    px = (a + 0.5·b) · size,   py = b · h
 *
 *  Centroid pixel → screen cell:
 *    scol = ox + (int)(px / CELL_W)
 *    srow = oy + (int)(py / CELL_H)
 *
 *  Pool toggle (O(1) remove via swap-with-last):
 *    if (i = pool_find(col,row,up)) >= 0:
 *      pool->items[i] = pool->items[--pool->count]
 *    else:
 *      pool->items[pool->count++] = (Obj){col,row,up,glyph,true}
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • MAX_OBJ cap: silently dropped when the pool is full. 'C' clears
 *    the pool. Bumping MAX_OBJ has no other cost — the pool is one
 *    flat array, no allocation churn.
 *  • Resize: cursor and objects keep their lattice addresses, but the
 *    pixel positions shift because ox/oy are recomputed each frame.
 *    An object can fall off-screen if the new terminal is small — its
 *    address is still valid; the cursor can walk back to it.
 *  • Glyph cycle: changing the glyph affects only the NEXT placement.
 *    Already-placed objects keep the glyph they were stored with.
 *  • Centroid rounding: ctx_to_screen uses int truncation, which
 *    keeps '@' inside the interior so it never lands on a border char.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Place a '*' at the origin triangle, press +/- to change tri_size,
 *  then watch the glyph follow the same triangle to its new pixel
 *  position. The (col, row, up) address is preserved across resize and
 *  size change.
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

#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 14.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

#define BORDER_W   0.10
#define MAX_OBJ    256
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
    /* edge,  object */
    {  75, 226 },
    {  82, 207 },
    { 207,  82 },
    {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_YELLOW },
    { COLOR_GREEN,   COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_GREEN  },
    { COLOR_WHITE,   COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e = (COLORS >= 256) ? THEME_FG[theme][0]   : THEME_FG_8[theme][0];
    short fg_o = (COLORS >= 256) ? THEME_FG[theme][1]   : THEME_FG_8[theme][1];
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx — GridCtx + the equilateral skew lattice formula            */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active equilateral grid plus tunable state.
 *
 * Carrying tri_size/border_w/theme as fields (not just defines or globals)
 * lets ctx_to_screen() work for any GridCtx — the shared abstraction with
 * the rect / hex / polar placement files (see ../README.md).
 */
typedef struct {
    /* terminal extent */
    int    rows, cols;

    /* lattice geometry */
    double tri_size;        /* equilateral side length in pixel sub-units */
    int    cell_w, cell_h;  /* sub-pixels per terminal column / row       */

    /* screen origin (screen position of lattice (0,0) corner) */
    int    ox, oy;

    /* mode state */
    double border_w;        /* barycentric edge threshold                 */
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

/* Recompute the per-resize fields without disturbing tunables. */
static void ctx_resize(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
}

/*
 * pixel_to_tri — solve the skew-lattice inverse and pick the triangle.
 *
 *   h = size · √3 / 2
 *   b = py / h
 *   a = px/size − 0.5·b
 *   col = ⌊a⌋,  row = ⌊b⌋
 *   fa  = a − col,  fb = b − row
 *   up  = (fa + fb ≥ 1)                     ← which half of the rhombus
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
    *col = c; *row = r;
    *fa  = a - (double)c;
    *fb  = b - (double)r;
    *up  = (*fa + *fb >= 1.0) ? 1 : 0;
}

/*
 * tri_centroid_pixel — forward lattice→pixel for the triangle centroid.
 *   ▽ centroid lattice = (col + 1/3, row + 1/3)
 *   △ centroid lattice = (col + 2/3, row + 2/3)
 *   px = (a + 0.5·b)·size,  py = b·h
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
 * ctx_to_screen — map (col, row, up) to the terminal cell at the centroid.
 * Truncation (not round) keeps '@' inside the triangle interior.
 */
static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *scol = g->ox + (int)(cx_pix / g->cell_w);
    *srow = g->oy + (int)(cy_pix / g->cell_h);
}

/*
 * tri_edge_char — barycentric weights → edge proximity → ASCII character.
 *   ▽:  l₁ = 1−fa−fb,  l₂ = fa,        l₃ = fb
 *   △:  l₁ = 1−fb,     l₂ = fa+fb−1,   l₃ = 1−fa
 * Smallest weight names the edge OPPOSITE that vertex.
 *   ▽: l₁→'/'  l₂→'\\'  l₃→'_'
 *   △: l₁→'_'  l₂→'/'   l₃→'\\'
 */
static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    if (up == 0) {
        l1 = 1.0 - fa - fb; ch1 = '/';
        l2 = fa;            ch2 = '\\';
        l3 = fb;            ch3 = '_';
    } else {
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
 * The whole grid is recomputed each frame with no stored data.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;
            int    tC, tR, tU;
            double fa, fb, m;
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

static void pool_remove(Pool *p, int col, int row, int up)
{
    int i = pool_find(p, col, row, up);
    if (i < 0) return;
    p->items[i] = p->items[--p->count];
}

static void pool_toggle(Pool *p, int col, int row, int up, char glyph)
{
    if (pool_find(p, col, row, up) >= 0) pool_remove(p, col, row, up);
    else                                  pool_place(p, col, row, up, glyph);
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

/*
 * TRI_DIR — arrow-key transition table indexed by [direction][current_up].
 * Each entry is (Δcol, Δrow, target_up).
 *   direction:  0=LEFT  1=RIGHT  2=UP  3=DOWN
 *   up index :  0=▽    1=△
 */
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
}

/*
 * cursor_move — apply (dcol, drow, dup) deltas. The triangular lattice has
 * no global cursor bounds, so no clamp; addresses extend infinitely.
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dcol, int drow, int dup)
{
    (void)g;
    cur->col += dcol;
    cur->row += drow;
    cur->up   = dup;
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
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Bright bold yellow fps readout (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             p->count, GLYPHS[g->glyph_idx], g->tri_size,
             g->theme, fps, g->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:toggle  g:glyph  C:clear  +/-:size  t:theme  r:reset  q:quit  [01 equilateral direct] ");
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
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  screen                                                              */
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
/* §9  app                                                                 */
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

    GridCtx g; ctx_init(&g, 0, 0);
    screen_init(g.theme);
    ctx_init(&g, LINES, COLS);

    Cursor cur; cursor_reset(&cur, &g);
    Pool   pool; pool_clear(&pool);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_resize(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            const int *t;
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g.paused ^= 1; break;
                case 'r':          cursor_reset(&cur, &g); pool_clear(&pool);
                                   color_init(g.theme); break;
                case 'C':          pool_clear(&pool); break;
                case 'g':          g.glyph_idx = (g.glyph_idx + 1) % N_GLYPHS; break;
                case ' ':          pool_toggle(&pool, cur.col, cur.row, cur.up,
                                                GLYPHS[g.glyph_idx]); break;
                case 't':
                    g.theme = (g.theme + 1) % N_THEMES;
                    color_init(g.theme);
                    break;
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
        t0  = now;

        scene_draw(&g, &pool, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
