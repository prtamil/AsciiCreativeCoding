/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal_patterns.c — drop preset shapes onto an X-cut grid
 *
 * Move a cursor with the arrows over a grid where every square is sliced
 * by both diagonals into 4 little triangles (north/east/south/west). Press
 * a digit 1..5 to stamp a ready-made shape (ring, line, star, triangle, or
 * a random scatter) at the cursor. SPACE wipes everything; 'g' changes the
 * mark used.
 *
 * Sister files: 03_double_diagonal_direct.c (place one wedge at a time),
 *               grids/tri_grids/03_double_diagonal.c (just draws the grid),
 *               02_right_isosceles_patterns.c (same idea, one diagonal).
 * Tiling reference: https://en.wikipedia.org/wiki/Tetrakis_square_tiling
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

/* ── §4 gridctx ── */

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

/* Given a point on screen, work out which square it's in and which of the
 * four diagonal wedges it falls into. The wedge is decided by whichever
 * diagonal the point is closest to relative to the square's centre. */
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

/* Picks the line character ('/', '\', '|', '_') to draw a point near a
 * wedge's edge, and reports how close the point is to that edge in
 * *out_min — the caller only draws the outline, not the wedge interior. */
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

/* ── §5 pool ── */

/* One placed mark. It lives in a specific triangle: the square at
 * (col, row), and which of the 4 wedges (DIR_N/E/S/W) inside it.
 * glyph is the character to show; alive lets us tombstone an entry
 * without shuffling the array. */
typedef struct { int col, row, wedge; char glyph; bool alive; } Obj;

/* All the placed marks. Fixed-size array, no malloc — count says how
 * many slots are used; everything past count is unused. */
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

/* ── §6 cursor ── */

/* The thing you steer with the arrows, plus the demo's current settings.
 * col/row/wedge is which triangle it's pointing at. glyph_idx picks the
 * mark used by the next stamp (index into GLYPHS). theme is the colour
 * scheme; paused is the running/paused flag shown in the HUD. */
typedef struct {
    int col, row, wedge;
    int glyph_idx;
    int theme;
    int paused;
} Cursor;

/* Where one arrow press takes the cursor. The cursor sits in a wedge,
 * so "move left" means something different depending on which wedge
 * you're in — sometimes you just flip to the neighbouring wedge in the
 * same square, sometimes you step into the next square. Each entry is
 * (column step, row step, wedge you land in), indexed by [arrow][wedge]. */
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

/* ── §7 mode — pattern stamps ── */

/* A pattern is just a list of wedges to drop, written as
 * (column offset, row offset, which wedge) relative to the cursor.
 * PAT_END is a marker row that means "end of list" — 0xDEAD is just an
 * unmistakable value no real offset would ever be. */
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

/* A tiny home-made random number generator (0..1). Good enough for
 * scattering marks around; not for anything that needs real randomness. */
static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

static void pattern_scatter(Pool *pool, int cC, int cR, char glyph)
{
    g_seed ^= (unsigned int)clock_ns();   /* stir in the clock so each press differs */
    int n = 10, tries = 0;                /* want 10 marks, but give up after 100 attempts */
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

/* ── §8 scene ── */

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

/* ── §9 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
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
            endwin(); refresh();   /* ncurses needs this poke to pick up the new size */
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
