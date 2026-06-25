/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral_direct.c — drop markers onto a triangle grid
 *
 * A grid of equilateral triangles fills the screen. Move the '@' cursor
 * between triangles with the arrows, hit SPACE to leave a marker on the
 * triangle you're standing on. Each marker remembers WHICH triangle it
 * sits on (a col/row/up address), not where on screen it is — so when you
 * resize the window or change the triangle size, markers follow their
 * triangle to its new spot.
 *
 * Sister files: grids/tri_grids/01_equilateral.c (just the background),
 *               grids/hex_grids_placement/01_hex_direct.c (same idea, hexes).
 */

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

/* ── §1 config ── */

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

/* ── §2 clock ── */

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

/* ── §3 color ── */

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

/* ── §4 gridctx — the math that turns a triangle's address into a screen spot ── */

/*
 * GridCtx — everything we need to know to draw the grid and to figure out
 * where any triangle lands on screen. It's one bundle holding the window
 * size, how big the triangles are right now, where the grid's centre sits,
 * and the few knobs the user can twiddle (size, theme, current glyph).
 *
 * Why keep the size/border/theme in here instead of as plain globals: it
 * means the placement math takes a GridCtx and works the same way for the
 * sibling rect / hex / polar files (see ../README.md). One bundle, one rule.
 */
typedef struct {
    /* how big the terminal is, in character cells */
    int    rows, cols;

    /* triangle size and how many sub-pixels fit in one character cell.
     * We measure positions in tiny sub-pixels first, then divide down to
     * character cells, so triangles can be sized more smoothly than the
     * chunky terminal grid would otherwise allow. */
    double tri_size;        /* triangle side length, in sub-pixels */
    int    cell_w, cell_h;  /* sub-pixels per character column / row */

    /* where the grid's (0,0) corner sits on screen — we re-centre this on
     * the middle of the window every frame, which is what lets markers stay
     * put on their triangle when the window resizes */
    int    ox, oy;

    /* user-twiddled knobs */
    double border_w;        /* how thick the drawn triangle edges look (0..1ish);
                             * a cell is part of an edge if it's within this of one */
    int    theme;           /* which colour scheme, 0..N_THEMES-1 */
    int    paused;          /* 1 while paused (the 'p' key) */
    int    glyph_idx;       /* which marker character SPACE will drop next */
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

/* on a resize, redo only the size-dependent fields and leave the user's
 * knobs (triangle size, theme, glyph) exactly as they were */
static void ctx_resize(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
}

/*
 * Given a point on screen, work out which triangle it falls in. We undo the
 * slanted grid to get whole-number col/row (which diamond-shaped cell), plus
 * the leftover fractions fa/fb saying how far into that cell we are. Each
 * diamond is two triangles; if the fractions add past 1 we're in the upper
 * (△) half, otherwise the lower (▽) half. The math is the inverse of the
 * forward formula below; the formulas are kept alongside for reference.
 *
 *   h = size · √3 / 2;  b = py/h;  a = px/size − 0.5·b
 *   col = ⌊a⌋, row = ⌊b⌋;  fa = a−col, fb = b−row;  up = (fa+fb ≥ 1)
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
 * Find the pixel at the dead centre of a given triangle. A triangle's centre
 * sits one-third of the way in from its corners, so we nudge the address by
 * 1/3 (▽) or 2/3 (△) and run it through the slanted-grid formula. This is the
 * spot where we'll draw the cursor or a marker so it lands cleanly inside.
 *   ▽ centre at (col+1/3, row+1/3);  △ centre at (col+2/3, row+2/3)
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
 * Turn a triangle's address into the exact character cell to draw on. We
 * chop the fraction off (truncate, not round) on purpose — rounding can
 * nudge the mark onto a triangle edge, and we want it sitting inside.
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
 * Decide which slash to draw for a cell near a triangle edge, and report how
 * close to an edge it is. We measure how near the point is to each of the
 * three sides (three numbers, one per side; small means close). Whichever
 * side is nearest picks the character: '/' '\' or '_' to trace that edge.
 * The caller only draws when the smallest number is below border_w — i.e.
 * the cell is actually sitting on an edge rather than out in open space.
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
 * Paint the whole triangle grid. We walk every character cell, ask which
 * triangle and edge it belongs to, and drop a slash only on the edge cells.
 * Nothing is stored between frames — the grid is just redrawn from scratch,
 * which is why it can follow any window size for free.
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

/* ── §5 pool — the bag of placed markers ── */

/* One placed marker. It's pinned to a triangle by address, not to a screen
 * spot, so it stays put when the view changes.
 *   col, row, up — which triangle (up: 0 = ▽ lower, 1 = △ upper)
 *   glyph        — the character to draw for this marker
 *   alive        — true if this slot holds a real marker (vs. an empty slot) */
typedef struct { int col, row, up; char glyph; bool alive; } Obj;

/* The whole collection of markers: a plain fixed-size array plus a count of
 * how many are in use. No growing, no allocation — full at MAX_OBJ. We keep
 * the live ones packed at the front (slots 0..count-1), so a removal just
 * moves the last one into the gap. */
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

/* ── §6 cursor — the '@' you steer ── */

/* Where the '@' currently is: the address of one triangle. Same three
 * numbers a marker uses (col, row, and up = which half of the diamond). */
typedef struct { int col, row, up; } Cursor;

/*
 * Moving by one arrow press isn't a simple ±1 on a triangle grid: stepping
 * "right" from a downward triangle just flips you to its upward neighbour in
 * the same cell, while stepping "right" from an upward one moves a column
 * over. This table spells out every case so cursor_move doesn't have to.
 * Look it up by [which arrow][which half you're on] to get the change to
 * apply: how much col and row shift, and which half you end up on.
 *   arrows: 0=LEFT 1=RIGHT 2=UP 3=DOWN     half: 0=▽ (lower)  1=△ (upper)
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
 * Nudge the cursor by the change looked up in TRI_DIR. There's no edge to
 * bump into — the grid of addresses goes on forever, so we never clamp; the
 * cursor can simply wander off the visible window and back.
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

/* ── §7 scene ── */

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

/* ── §8 screen ── */

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

/* ── §9 app — signals and the main loop ── */

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
