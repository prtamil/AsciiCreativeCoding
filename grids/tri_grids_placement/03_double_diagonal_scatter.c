/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal_scatter.c
 *
 * Sprinkles a cloud of random dots around the cursor on a grid where each
 * square is cut by both diagonals into four wedges (N/E/S/W), then colors
 * each dot by how far it sits from the cursor — near is warm, far is cool.
 * Move the cursor to repaint the same cloud through a new "distance" lens;
 * SPACE sprinkles a fresh cloud; +/- changes the density.
 *
 * Grid shape: tetrakis square tiling
 *   https://en.wikipedia.org/wiki/Tetrakis_square_tiling
 * Sister files: 03_double_diagonal_direct.c, 03_double_diagonal_patterns.c,
 *               02_right_isosceles_scatter.c (same idea, two-triangle squares).
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
#define BORDER_W   0.10
#define MAX_OBJ    1024
#define SCATTER_RADIUS 12
#define DENSITY_DEFAULT 120
#define DENSITY_MIN      20
#define DENSITY_MAX     500
#define DENSITY_STEP     20
#define N_BUCKETS  6
#define N_THEMES   3

#define FPS_EWMA_ALPHA 0.05

/* the four wedges a square's two diagonals cut it into */
#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_BUCK0  3   /* first of 6 gradient colors; bucket 0 = closest/warmest */
#define PAIR_HUD    9
#define PAIR_HINT   10

static const char *DIR_NAME[4] = { "N", "E", "S", "W" };

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
    struct timespec r = { .tv_sec = (time_t)(ns/1000000000LL), .tv_nsec = (long)(ns%1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

/* The six color stops each theme paints dots with, ordered near-to-far from
 * the cursor. THEME_GRAD is the rich 256-color version; THEME_GRAD_8 is the
 * fallback for terminals that only have 8 colors. */
static const short THEME_GRAD[N_THEMES][N_BUCKETS] = {
    { 196, 202, 214, 226,  82,  39 },
    {  39,  82, 226, 214, 202, 196 },
    {  15, 250, 244, 240, 236, 232 },
};
static const short THEME_GRAD_8[N_THEMES][N_BUCKETS] = {
    { COLOR_RED,  COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE },
    { COLOR_BLUE, COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,   COLOR_RED  },
    { COLOR_WHITE,COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE},
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    for (int i = 0; i < N_BUCKETS; i++) {
        short fg = (COLORS >= 256) ? THEME_GRAD[theme][i] : THEME_GRAD_8[theme][i];
        init_pair(PAIR_BUCK0 + i, fg, -1);
    }
    init_pair(PAIR_BORDER, COLORS >= 256 ? 248 : COLOR_WHITE, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 gridctx — the double-diagonal grid ── */

/* Everything the grid needs to draw itself and to turn a wedge's grid address
 * into a spot on the screen. One of these lives for the whole run; resizing the
 * terminal just refreshes a few of its fields. */
typedef struct {
    int    rows, cols;      /* size of the terminal, in character cells */
    double tri_size;        /* how big one grid square is, in sub-cell pixels */
    int    cell_w, cell_h;  /* pixels packed into one character cell (chars are tall) */
    int    ox, oy;          /* screen cell the grid is centered on (the origin) */
    double border_w;        /* how close to an edge counts as "on the edge" (0..1) */
    int    theme;           /* which color set is active (0..N_THEMES-1) */
    int    paused;          /* nonzero while the sim is frozen */
    int    density;         /* how many dots a fresh scatter aims for */
    int    scatter_radius;  /* how far from the cursor dots may land, in squares */
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
    g->density  = DENSITY_DEFAULT;
    g->scatter_radius = SCATTER_RADIUS;
}

static void ctx_resize(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->ox = cols / 2; g->oy = (rows - 1) / 2;
}

/* recipe step 1 (reverse) — a pixel -> which square (col,row) + which wedge +
 * where inside (fa,fb). Scale by size; the integer part names the square, the
 * fraction (fa,fb) locates the point. The wedge is whichever side the point
 * leans toward from the square's center: bigger |dx| => E/W, else N/S. */
static void screen_to_tri(double px, double py, double size,
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

/* the pixel at a wedge's centroid (1/6 in from the shared center toward each
 * outer edge). This is where the cursor/dot is drawn so it lands cleanly
 * inside the wedge. */
static void tri_centroid_pixel(int col, int row, int wedge, double size,
                               double *cx_pix, double *cy_pix)
{
    double a, b;
    switch (wedge) {
        case DIR_N: a = 0.5;     b = 1.0/6.0; break;
        case DIR_E: a = 5.0/6.0; b = 0.5;     break;
        case DIR_S: a = 0.5;     b = 5.0/6.0; break;
        default:    a = 1.0/6.0; b = 0.5;     break;
    }
    *cx_pix = ((double)col + a) * size;
    *cy_pix = ((double)row + b) * size;
}

static void tri_to_screen(const GridCtx *g, int col, int row, int wedge,
                          int *scol, int *srow)
{
    double cx, cy;
    tri_centroid_pixel(col, row, wedge, g->tri_size, &cx, &cy);
    *scol = g->ox + (int)(cx / g->cell_w);
    *srow = g->oy + (int)(cy / g->cell_h);
}

/* the line char for a point by nearest of the wedge's 3 sides (two diagonals
 * '/' '\\' plus the square's outer edge '_' or '|'). out_min returns the
 * distance to that side, so the caller skips open-space cells. */
static char edge_glyph(int wedge, double fa, double fb, double *out_min)
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

/* recipe step 2 — draw the grid: every cell -> its wedge -> a line char only on
 * the edge cells (interiors stay blank). Nothing stored; redrawn each frame. */
static void draw_lattice(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;
            int tC, tR, tW; double fa, fb, m;
            screen_to_tri(px, py, g->tri_size, &tC, &tR, &tW, &fa, &fb);
            char ch = edge_glyph(tW, fa, fb, &m);
            if (m >= g->border_w) continue;
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

/* ── §5 pool ── */

/* One scattered dot. col/row picks the square it lives in; wedge says which of
 * that square's four wedges it is. glyph is the character drawn for it, and
 * alive lets us skip dead entries. */
typedef struct { int col, row, wedge; char glyph; bool alive; } Obj;

/* The whole scatter cloud: a fixed-size bag of dots plus how many of the
 * MAX_OBJ slots are currently in use. No allocation — it's all here up front. */
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

/* The running state of the random-number maker. Changing this changes the
 * cloud; we mix in the clock each reseed so two clouds never look the same. */
static unsigned int g_seed = 1;
/* A tiny home-grown random-number maker (a classic LCG, Numerical Recipes
 * ch. 7). Returns a fresh value in 0..1 each call. We use our own instead of
 * rand() so a given seed always replays the exact same cloud. */
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

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

static int triangle_distance(int aC, int aR, int aW, int bC, int bR, int bW)
{
    int d  = abs(aC - bC) + abs(aR - bR);
    /* add a small bump for facing different ways: 0 same wedge, 1 a
     * quarter-turn apart, 2 facing opposite (the 4 wedges wrap around) */
    int dd = abs(aW - bW); if (dd > 2) dd = 4 - dd;
    return d + dd;
}

static int distance_bucket(int dist, int max_d)
{
    if (max_d <= 0) return 0;
    int b = (dist * N_BUCKETS) / (max_d + 1);
    if (b >= N_BUCKETS) b = N_BUCKETS - 1;
    return b;
}

/* ── §6 cursor ── */

/* Where the '@' marker sits. Same address scheme as Obj: col/row pick the
 * square, wedge picks which of the 4 wedges inside it. */
typedef struct { int col, row, wedge; } Cursor;

/* How an arrow key nudges the cursor. On the four-wedge grid one step depends
 * on which wedge you're in: an arrow sometimes just flips to the neighboring
 * wedge in the same square, and sometimes hops into the next square. Looked up
 * by direction (left/right/up/down) and by the current wedge, giving
 * {dcol, drow, new-wedge}. Index it as TRI_DIR[direction][cursor.wedge]. */
static const int TRI_DIR[4][4][3] = {
    { {  0,  0, DIR_W }, {  0,  0, DIR_W }, {  0,  0, DIR_W }, { -1,  0, DIR_E } },
    { {  0,  0, DIR_E }, { +1,  0, DIR_W }, {  0,  0, DIR_E }, {  0,  0, DIR_E } },
    { {  0, -1, DIR_S }, {  0,  0, DIR_N }, {  0,  0, DIR_N }, {  0,  0, DIR_N } },
    { {  0,  0, DIR_S }, {  0,  0, DIR_S }, {  0, +1, DIR_N }, {  0,  0, DIR_S } },
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0; cur->row = 0; cur->wedge = DIR_N;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int dcol, int drow, int dwedge)
{
    (void)g;
    cur->col += dcol; cur->row += drow; cur->wedge = dwedge;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sc, sr;
    tri_to_screen(g, cur->col, cur->row, cur->wedge, &sc, &sr);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §7 scatter — random dots around the cursor ── */

/* the distinct placement step: fill the pool with dots at random offsets
 * (within scatter_radius squares) and random wedges, all centered on the
 * cursor. The tries cap stops us looping forever when duplicate spots keep
 * getting rejected and the cloud can't reach the requested count. */
static void scatter_seed(Pool *sp, const GridCtx *g, const Cursor *cur)
{
    pool_clear(sp);
    g_seed ^= (unsigned int)clock_ns();
    int max = (g->density < MAX_OBJ) ? g->density : MAX_OBJ;
    int tries = 0;
    int R = g->scatter_radius;
    while (sp->count < max && tries < max * 4) {
        int dC    = (int)(frand() * (2 * R + 1)) - R;
        int dR    = (int)(frand() * (2 * R + 1)) - R;
        int wedge = (int)(frand() * 4);
        pool_place(sp, cur->col + dC, cur->row + dR, wedge, '*');
        tries++;
    }
}

static void scatter_draw(const Pool *sp, const GridCtx *g, const Cursor *cur)
{
    int max_d = g->scatter_radius * 2;
    for (int i = 0; i < sp->count; i++) {
        if (!sp->items[i].alive) continue;
        int sc, sr;
        tri_to_screen(g, sp->items[i].col, sp->items[i].row, sp->items[i].wedge,
                      &sc, &sr);
        if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) continue;
        int dist = triangle_distance(sp->items[i].col, sp->items[i].row, sp->items[i].wedge,
                                     cur->col, cur->row, cur->wedge);
        int b = distance_bucket(dist, max_d);
        attron(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
        mvaddch(sr, sc, (chtype)(unsigned char)sp->items[i].glyph);
        attroff(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
    }
}

/* ── §8 scene ── */

static void hud_draw(const GridCtx *g, const Pool *sp, const Cursor *cur,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  N:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, DIR_NAME[cur->wedge],
             sp->count, g->tri_size, g->theme, fps,
             g->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:reseed  +/-:density  t:theme  r:reset  q:quit  [03 scatter] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *sp, const Cursor *cur,
                       double fps)
{
    erase();
    draw_lattice(g);
    scatter_draw(sp, g, cur);
    cursor_draw(cur, g);
    hud_draw(g, sp, cur, fps);
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
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal); signal(SIGWINCH, on_signal);

    GridCtx g; ctx_init(&g, 0, 0);
    screen_init(g.theme);
    ctx_init(&g, LINES, COLS);

    Cursor cur; cursor_reset(&cur, &g);
    Pool   sp;  pool_clear(&sp);
    g_seed = (unsigned int)clock_ns();
    scatter_seed(&sp, &g, &cur);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS; int64_t t0 = clock_ns();
    while (g_running) {
        if (g_need_resize) { g_need_resize = 0; endwin(); refresh(); ctx_resize(&g, LINES, COLS); }
        int ch;
        while ((ch = getch()) != ERR) {
            const int *t;
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': g.paused ^= 1; break;
                case 'r':
                    cursor_reset(&cur, &g);
                    ctx_init(&g, LINES, COLS);   /* size, density, theme back to defaults */
                    scatter_seed(&sp, &g, &cur);
                    color_init(g.theme);
                    break;
                case ' ': scatter_seed(&sp, &g, &cur); break;
                case 't': g.theme = (g.theme + 1) % N_THEMES; color_init(g.theme); break;
                case KEY_LEFT:  t = TRI_DIR[0][cur.wedge]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_RIGHT: t = TRI_DIR[1][cur.wedge]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_UP:    t = TRI_DIR[2][cur.wedge]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_DOWN:  t = TRI_DIR[3][cur.wedge]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case '+': case '=':
                    if (g.density < DENSITY_MAX) { g.density += DENSITY_STEP; scatter_seed(&sp, &g, &cur); } break;
                case '-':
                    if (g.density > DENSITY_MIN) { g.density -= DENSITY_STEP; scatter_seed(&sp, &g, &cur); } break;
            }
        }
        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&g, &sp, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
