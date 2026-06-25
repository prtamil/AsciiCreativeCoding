/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_isometric_direct.c — triangle grid where every tile is painted a solid
 * colour, and you drop glyphs onto tiles with the arrow keys.
 *
 * Six colours cycle around every shared corner, which tricks the eye into
 * seeing a wall of stacked 3-D cubes. Arrows move the '@' cursor between
 * triangles; SPACE toggles a glyph on the tile you're sitting on.
 *
 * Sister files: grids/tri_grids/05_isometric.c (the colour-fill grid on its
 *               own); 01_equilateral_direct.c (same cursor + object pool, but
 *               draws triangle edges instead of solid fills).
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

/* ── §1 config ── */

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 16.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

#define MAX_OBJ    256
#define N_GLYPHS   6
#define N_PALETTE  6
#define N_THEMES   3

#define FPS_EWMA_ALPHA  0.05

#define PAIR_FILL_BASE  1                              /* 1..6 */
#define PAIR_CURSOR    (PAIR_FILL_BASE + N_PALETTE)
#define PAIR_HUD       (PAIR_CURSOR + 1)
#define PAIR_HINT      (PAIR_HUD + 1)

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

static const short PAL256[N_THEMES][N_PALETTE] = {
    /* warm  */ { 196, 214, 226, 118,  39, 129 },
    /* cool  */ {  39,  45,  82, 226, 207,  51 },
    /* mono  */ { 250, 244, 250, 244, 250, 244 },
};
static const short PAL8[N_THEMES][N_PALETTE] = {
    { COLOR_RED,   COLOR_YELLOW, COLOR_GREEN, COLOR_CYAN,    COLOR_BLUE,    COLOR_MAGENTA },
    { COLOR_BLUE,  COLOR_CYAN,   COLOR_GREEN, COLOR_YELLOW,  COLOR_MAGENTA, COLOR_BLUE    },
    { COLOR_WHITE, COLOR_CYAN,   COLOR_BLUE,  COLOR_WHITE,   COLOR_BLUE,    COLOR_CYAN    },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < N_PALETTE; i++) {
        short bg = (COLORS >= 256) ? PAL256[theme][i] : PAL8[theme][i];
        /* black text on a colour: we only ever print a space, so all you see is the fill */
        init_pair(PAIR_FILL_BASE + i, COLOR_BLACK, bg);
    }
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLACK);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 gridctx ── */

/*
 * GridCtx — everything we need to know to place triangles on screen: how big
 * the terminal is, how big each triangle should be, where the grid is centred,
 * and how far the cursor is allowed to wander.
 *
 * The triangle size lives in here rather than a fixed #define so '+'/'-' can
 * resize the grid at runtime. The grid itself goes on forever in theory;
 * max_col/max_row just fence the cursor into the part you can actually see.
 */
typedef struct {
    int    rows, cols;            /* terminal size, in cells */
    int    cw, ch;                /* how many sub-pixels one cell is worth (=CELL_W, CELL_H) */
    double tri_size;              /* length of a triangle side, in pixels */
    int    ox, oy;                /* screen cell the grid is centred on */
    int    max_col, max_row;      /* how far the cursor may roam from centre */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, double tri_size)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->tri_size = tri_size;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    /* half a screen of triangles in each direction is plenty of room to move */
    g->max_col = cols / 2;
    g->max_row = rows / 2;
}

/* Given a point on screen, work out which triangle it lands in. */
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

/* Find the middle point of a given triangle, in screen pixels. */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx, double *cy)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx = (a + 0.5 * b) * size;
    *cy = b * h;
}

/* Turn a triangle address into the screen cell to draw its marker in. */
static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *scol, int *srow)
{
    double cx, cy;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx, &cy);
    *scol = g->ox + (int)(cx / g->cw);
    *srow = g->oy + (int)(cy / g->ch);
}

/*
 * Pick which of the six colours a triangle gets.
 *
 * The mix of column, row, and up/down is tuned so that the six triangles
 * meeting at any shared corner each get a different colour, going round in
 * order. That repeating ring of colours is what fools the eye into seeing
 * stacked cubes. The mod keeps the answer in 0..5 even left of the origin.
 */
static int palette_index(int col, int row, int up)
{
    int k = col + 2 * row + up;
    k %= N_PALETTE; if (k < 0) k += N_PALETTE;
    return k;
}

/*
 * Paint the whole grid: for every cell on screen, figure out which triangle
 * covers it and colour the cell with that triangle's colour.
 */
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

/* ── §5 pool ── */

/*
 * Obj — one glyph the user dropped on a triangle.
 *   col, row, up — which triangle it sits on
 *   glyph        — the character to draw there
 *   alive        — whether this slot is in use (false slots are skipped)
 */
typedef struct {
    int  col, row, up;
    char glyph;
    bool alive;
} Obj;

/*
 * Pool — the bag of placed glyphs, kept in a plain fixed array so we never
 * allocate memory while running. Holds up to MAX_OBJ; count is how many are
 * actually in use. Deleting one moves the last item into the gap, so the live
 * items always sit packed at the front.
 */
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

/* ── §6 cursor ── */

/*
 * Cursor — where the '@' is and what the user has chosen.
 *   col, row, up — which triangle the cursor is on
 *   glyph_idx    — which glyph SPACE will drop (index into GLYPHS)
 *   theme        — which colour set is active
 *   paused       — set by 'p'; shown in the HUD
 */
typedef struct {
    int col, row, up;
    int glyph_idx;
    int theme;
    int paused;
} Cursor;

/*
 * Where each arrow key takes you. Moving between triangles isn't a simple
 * "one cell over": stepping out of an up-pointing triangle lands you somewhere
 * different than stepping out of a down-pointing one. So this table is looked
 * up by both the arrow pressed and which way the current triangle points, and
 * gives back how much to shift column/row and which way the new triangle faces.
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

/* ── §7 mode ── */

static void mode_toggle_at_cursor(Pool *p, const Cursor *cur)
{
    pool_toggle(p, cur->col, cur->row, cur->up, GLYPHS[cur->glyph_idx]);
}

/* ── §8 scene ── */

/* The status line (cursor info + fps, top-right) and the key hints (bottom). */
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
             " arrows:move  spc:toggle  g:glyph  C:clear  +/-:size  t:theme  r:reset  q:quit  [05 isometric direct] ");
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
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §9 screen ── */

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

/* ── §10 app ── */

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
    Pool   pool;   pool_clear(&pool);
    GridCtx g;

    cur.col = 0; cur.row = 0; cur.up = 0;
    cur.glyph_idx = 0; cur.theme = 0; cur.paused = 0;
    screen_init(cur.theme);
    ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);
    cursor_reset(&cur, &g);

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
                    cursor_reset(&cur, &g); pool_clear(&pool);
                    color_init(cur.theme);
                    break;
                case 'C': pool_clear(&pool); break;
                case 'g': cur.glyph_idx = (cur.glyph_idx + 1) % N_GLYPHS; break;
                case ' ': mode_toggle_at_cursor(&pool, &cur); break;
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
