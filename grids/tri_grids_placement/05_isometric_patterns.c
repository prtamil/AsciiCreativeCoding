/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_isometric_patterns.c — drop preset shapes onto a colour-filled
 * triangular grid.
 *
 * Move the cursor with the arrows, then press 1..5 to stamp a ready-made
 * shape (a ring, a line, a star, a small triforce, or a random scatter) at
 * that spot. The grid itself is painted in solid colour blocks; the stamped
 * characters sit on top in reverse video so they show up on any colour.
 *
 * Sister files: grids/tri_grids/05_isometric.c (the plain solid-fill grid),
 * 05_isometric_direct.c (place one triangle at a time), and
 * 01_equilateral_patterns.c (same stamps drawn as edges, not fills).
 */

/* The big idea:
 *
 * Each preset shape is just a little list of "go this many columns over, this
 * many rows down, and land on the up- or down-pointing triangle." Pressing a
 * digit shifts that whole list to wherever the cursor is and drops a character
 * at every spot. The shapes don't care what colour anything is — they only
 * deal in grid addresses (column, row, which way the triangle points).
 *
 * One thing that catches people out: each list stores the FINAL orientation
 * it wants (up or down), not a relative flip. In this grid whether a triangle
 * points up or down depends on its position, so a relative flip would make the
 * shape look different every other spot you stamp it.
 *
 * To check it works: press '1' (the ring) anywhere and you should see six
 * characters around the cursor on different background colours. Press 't' to
 * recolour the grid — the same triangles keep their characters.
 *
 * References: triangular tiling (en.wikipedia.org/wiki/Triangular_tiling),
 * the object-pool idea (gameprogrammingpatterns.com/object-pool.html), and the
 * simple random-number recipe from Numerical Recipes ch. 7.
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

/* ── §2 clock ── */

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

/* ── §3 color ── */

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

/* ── §4 gridctx — grid layout, address↔screen, and the colour picker ── */

/* Everything we need to know about where the triangular grid sits on screen.
 * Holds the terminal size, how big each triangle is, where the grid's origin
 * lands, and how far the cursor is allowed to roam from that origin. */
typedef struct {
    int    rows, cols;      /* terminal size, in character cells            */
    int    cw, ch;          /* how many sub-units make one character cell    */
    double tri_size;        /* edge length of a triangle, in those sub-units */
    int    ox, oy;          /* screen cell the grid's (0,0) is centred on    */
    int    max_col, max_row;/* how far the cursor may move from the origin    */
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

/* ── §5 pool ── */

/* One stamped character living on a single triangle. */
typedef struct {
    int  col, row, up;  /* which triangle: column, row, and up- vs down-pointing */
    char glyph;         /* the character drawn there                             */
    bool alive;         /* false means this slot is free to overwrite            */
} Obj;

/* All the stamped characters, kept in one fixed-size array so we never need to
 * allocate memory while running. We only ever fill up to `count`; clearing the
 * screen just resets count back to zero. */
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

/* ── §6 cursor ── */

/* Where the user is pointing, plus the few settings the arrow keys and toggles
 * change. Kept separate from the grid so we can recolour or resize without
 * losing the cursor's spot. */
typedef struct {
    int col, row, up;   /* the triangle the cursor sits on                  */
    int glyph_idx;      /* which character 'g' will stamp next (into GLYPHS)*/
    int theme;          /* current colour theme                             */
    int paused;         /* set by 'p' (this demo has nothing to pause, but
                           the HUD shows the state)                         */
} Cursor;

/* Moving the cursor one step isn't a simple +/-1: stepping off an up triangle
 * lands you somewhere different than stepping off a down one. This table gives,
 * for each arrow and each starting orientation, the (column shift, row shift,
 * new orientation) that lands on the neighbour. */
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

/* ── §7 mode — the preset shapes and the random scatter ── */

/* Each shape list ends with this marker so the stamping loop knows to stop
 * (0xDEAD is just a value that never shows up as a real column offset). */
#define PAT_END   { 0xDEAD, 0, 0 }
#define IS_END(p) ((p)[0] == 0xDEAD)

/* The presets. Each row is (columns over, rows down, final orientation),
 * measured from the cursor. */
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

/* A tiny home-grown random number generator: returns a value in [0,1). Plenty
 * good enough for sprinkling characters; not for anything that needs real
 * randomness. */
static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

static void pattern_scatter(Pool *pool, int cC, int cR, char glyph)
{
    /* mix in the clock so each scatter looks different */
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

/* ── §8 scene ── */

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

/* ── §9 screen ── */

static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

/* ── §10 app ── */

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
