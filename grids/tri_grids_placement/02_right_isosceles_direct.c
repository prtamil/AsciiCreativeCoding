/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_right_isosceles_direct.c
 *
 * A grid of squares, each cut in half by a '\' diagonal into two right
 * triangles: an upper-right one and a lower-left one. Move the '@' cursor
 * with the arrows; SPACE drops or removes a glyph on whichever half the
 * cursor sits on. Every glyph remembers which half-square it's on, so it
 * stays put when you resize the window or zoom the grid.
 *
 * Sister files: grids/tri_grids/02_right_isosceles.c shows the bare grid,
 * and 01_equilateral_direct.c does the same trick on 60-degree triangles.
 *
 * Sections: §1 settings  §2 clock  §3 color  §4 grid math  §5 glyph pool
 *           §6 cursor  §7 drawing  §8 ncurses setup  §9 main loop
 */

/* ── how the addressing works ──
 *
 * Picture ordinary graph paper, then slice every square along the '\'
 * diagonal so each square holds two triangles. We give each triangle its
 * own address: the square it lives in (col, row) plus which half it is
 * (up = 1 means the upper-right triangle, up = 0 the lower-left).
 *
 * Going from a pixel position to a triangle is easy here because the grid
 * lines up with the axes — no slanting to undo. Divide the pixel by the
 * square size to get a fractional position inside the square; the whole
 * part is the square, the leftover fractions (fa across, fb down) tell you
 * which side of the diagonal you're on. More across than down means the
 * upper-right half, otherwise the lower-left.
 *
 * To draw a triangle's glyph we aim for its center of mass: the upper-right
 * triangle sits at (col + 2/3, row + 1/3) and the lower-left at
 * (col + 1/3, row + 2/3), measured in square-widths.
 *
 * Moving the cursor with an arrow key sometimes just flips to the other
 * half of the same square and sometimes steps into a neighbour — the
 * TRI_DIR table in §6 spells out each case. Two presses in one direction
 * always equal one full square's worth of travel.
 *
 * Worth knowing:
 *  - On the exact diagonal we pick the upper-right half. That line is
 *    infinitely thin, so rounding can flip the choice for a single frame
 *    on resize. Harmless.
 *  - Terminal cells are taller than they are wide (2 wide, 4 tall here), so
 *    the triangles look stretched vertically on screen even though they're
 *    true right-isosceles triangles in pixel terms. Not a bug.
 *  - The pool holds a fixed number of glyphs; once full, new drops are
 *    ignored. 'C' clears them all.
 *
 * References: half-square triangular tiling
 *   https://en.wikipedia.org/wiki/Triangular_tiling
 * Object pool idea: gameprogrammingpatterns.com/object-pool.html
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

/* ── §1 settings ── */

#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 16.0
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
    start_color();
    use_default_colors();
    short fg_e = (COLORS >= 256) ? THEME_FG[theme][0] : THEME_FG_8[theme][0];
    short fg_o = (COLORS >= 256) ? THEME_FG[theme][1] : THEME_FG_8[theme][1];
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 grid math — squares split by one diagonal ── */

/* Everything the grid needs to know to draw itself and convert between
 * the math world and the screen. One of these lives for the whole run. */
typedef struct {
    int    rows, cols;      /* terminal size in character cells */
    double tri_size;        /* how big one square is, in pixel sub-units; bigger = zoomed in */
    int    cell_w, cell_h;  /* pixel sub-units per character cell (cells are taller than wide) */
    int    ox, oy;          /* screen cell the grid's origin (0,0) lands on — re-centred on resize */
    double border_w;        /* how close to an edge counts as "on the edge" when drawing lines */
    int    theme;           /* which color set is active (0..N_THEMES-1) */
    int    paused;          /* unused here; kept for parity with animated demos */
    int    glyph_idx;       /* which glyph SPACE will drop, an index into GLYPHS */
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
    double a   = px * inv;
    double b   = py * inv;
    int    c   = (int)floor(a);
    int    r   = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c;
    *fb = b - (double)r;
    *up = (*fa >= *fb) ? 1 : 0;     /* more across than down -> upper-right half, else lower-left */
}

static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double a = (up == 1) ? ((double)col + 2.0/3.0) : ((double)col + 1.0/3.0);
    double b = (up == 1) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = a * size;
    *cy_pix = b * size;
}

static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *scol = g->ox + (int)(cx_pix / g->cell_w);
    *srow = g->oy + (int)(cy_pix / g->cell_h);
}

/* Of the triangle's three edges, find the one this point is closest to and
 * return the line glyph for it. out_min is that distance, so the caller can
 * decide whether the point is close enough to an edge to draw the line. */
static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char ch1, ch2, ch3;
    if (up == 1) {                /* upper-right triangle */
        l1 = 1.0 - fa;       ch1 = '|';
        l2 = fa - fb;        ch2 = '\\';
        l3 = fb;             ch3 = '_';
    } else {                       /* lower-left triangle */
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

/* ── §5 glyph pool ── */

/* One placed glyph and where it lives. The address (col, row, up) is the
 * source of truth — the screen position is recomputed from it each frame,
 * so glyphs follow the grid through resizes and zooms. */
typedef struct {
    int  col, row, up;  /* which square, and which half (1 = upper-right, 0 = lower-left) */
    char glyph;         /* the character to draw */
    bool alive;         /* false = this slot is empty and skipped when drawing */
} Obj;

/* A fixed-size bag of placed glyphs. We keep all the live ones packed at the
 * front, so 'count' is exactly how many there are and removal just moves the
 * last one into the gap. */
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

/* ── §6 cursor ── */

/* Where the '@' is: which square (col, row) and which half (up). */
typedef struct { int col, row, up; } Cursor;

/* The movement rulebook. Look up [direction][current half] to get the change
 * to apply: {dcol, drow, new up}. Each arrow either flips to the other half of
 * the same square or steps into a neighbour, so two presses one way always
 * move a full square. Rows are LEFT, RIGHT, UP, DOWN; the inner pair is
 * indexed by whether you're currently on the lower-left (0) or upper-right (1)
 * half. */
static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 }, {  0,  0,  0 } },
    /* RIGHT */ { {  0,  0,  1 }, { +1,  0,  0 } },
    /* UP    */ { {  0,  0,  1 }, {  0, -1,  0 } },
    /* DOWN  */ { {  0, +1,  1 }, {  0,  0,  0 } },
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

/* ── §7 drawing ── */

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
             " arrows:move  spc:toggle  g:glyph  C:clear  +/-:size  t:theme  r:reset  q:quit  [02 right-isosceles direct] ");
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

/* ── §8 ncurses setup ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §9 main loop ── */

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
