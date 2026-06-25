/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal_scatter.c
 *
 * Sprinkle a cloud of random dots near the cursor on a grid where each
 * square is cut by both diagonals into four wedges (N/E/S/W). Each dot
 * is colored by how far it sits from the cursor — close is warm, far is
 * cool. Moving the cursor just recolors the same dots; SPACE makes a new
 * cloud; +/- changes how many dots.
 *
 * Grid shape: tetrakis square tiling
 *   https://en.wikipedia.org/wiki/Tetrakis_square_tiling
 * Sister files: 03_double_diagonal_direct.c, 03_double_diagonal_patterns.c,
 *               02_right_isosceles_scatter.c.
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

#define MAX_OBJ          1024
#define SCATTER_RADIUS     12
#define DENSITY_DEFAULT   120
#define DENSITY_MIN        20
#define DENSITY_MAX       500
#define DENSITY_STEP       20
#define N_BUCKETS           6
#define N_THEMES            3

#define FPS_EWMA_ALPHA 0.05

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
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

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
    init_pair(PAIR_BORDER, COLORS >= 256 ? 248 : COLOR_WHITE,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 gridctx ── */

/* Everything needed to turn a grid square+wedge into a screen spot, and
 * back. Holds the screen size and where we pin the grid's origin, so the
 * grid stays centered as the terminal resizes. */
typedef struct {
    int    rows, cols;     /* terminal size, in character cells */
    int    cw, ch;         /* how many pixels one character cell is worth (width, height) */
    int    ox, oy;         /* screen cell that grid coordinate (0,0) maps to */
    double tri_size;       /* size of one grid square in pixels; bigger = zoomed in */
    double border_w;       /* how close to an edge counts as "on the line" when drawing borders */
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

/* ── §5 pool ── */

/* One scattered dot: which square (col,row), which of the 4 wedges, the
 * character to draw, and whether the slot is in use. */
typedef struct { int col, row, wedge; char glyph; bool alive; } Obj;
/* The whole cloud of dots in one fixed array — no allocation, just fill
 * up to count. A reseed resets count to 0 and refills. */
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

/* ── §6 cursor ── */

/* The user-driven marker. Its position is the center the dot cloud is
 * sprinkled around and measured from, so moving it recolors everything. */
typedef struct {
    int col, row, wedge;   /* where the '@' sits: square + which of the 4 wedges */
    int density;           /* how many dots to scatter on the next reseed */
    int theme;             /* which color palette is active (0..N_THEMES-1) */
    int paused;            /* unused here, kept so all the sister demos share one struct */
} Cursor;

/* Move lookup: given an arrow key and the wedge you're in, what's the next
 * spot? Each entry is the step (dcol, drow, new-wedge). Pressing an arrow
 * sometimes just flips to the neighboring wedge in the same square, and
 * sometimes hops into the next square — this table bakes in both. */
static const int TETRA_DIR[4][4][3] = {
    { {  0,  0, DIR_W }, {  0,  0, DIR_W }, {  0,  0, DIR_W }, { -1,  0, DIR_E } },
    { {  0,  0, DIR_E }, { +1,  0, DIR_W }, {  0,  0, DIR_E }, {  0,  0, DIR_E } },
    { {  0, -1, DIR_S }, {  0,  0, DIR_N }, {  0,  0, DIR_N }, {  0,  0, DIR_N } },
    { {  0,  0, DIR_S }, {  0,  0, DIR_S }, {  0, +1, DIR_N }, {  0,  0, DIR_S } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->wedge = DIR_N;
    cur->density = DENSITY_DEFAULT;
    cur->theme   = 0;
    cur->paused  = 0;
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

/* ── §7 mode — random scatter + distance bucket ── */

/* Our own tiny random-number generator so the demo is self-contained and
 * repeatable from a given seed; returns a value in [0,1). */
static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

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

static void scatter_seed(Pool *p, const Cursor *cur)
{
    pool_clear(p);
    g_seed ^= (unsigned int)clock_ns();   /* stir in the clock so each reseed differs */
    int max = (cur->density < MAX_OBJ) ? cur->density : MAX_OBJ;
    /* the tries cap stops us looping forever when duplicates keep getting
     * rejected and the cloud can't reach the requested count */
    int tries = 0;
    while (p->count < max && tries < max * 4) {
        int dC    = (int)(frand() * (2*SCATTER_RADIUS+1)) - SCATTER_RADIUS;
        int dR    = (int)(frand() * (2*SCATTER_RADIUS+1)) - SCATTER_RADIUS;
        int wedge = (int)(frand() * 4);
        pool_place(p, cur->col + dC, cur->row + dR, wedge, '*');
        tries++;
    }
}

static void scatter_draw(const Pool *p, const Cursor *cur, const GridCtx *g)
{
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].wedge,
                      &sc, &sr);
        if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) continue;
        int dist = triangle_distance(p->items[i].col, p->items[i].row, p->items[i].wedge,
                                     cur->col, cur->row, cur->wedge);
        int b = distance_bucket(dist, SCATTER_RADIUS * 2);
        attron(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
        mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
        attroff(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
    }
}

/* ── §8 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  N:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, DIR_NAME[cur->wedge],
             p->count, g->tri_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:reseed  +/-:density  t:theme  r:reset  q:quit  [03 scatter] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    scatter_draw(p, cur, g);
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
    g_seed = (unsigned int)clock_ns();
    screen_init(cur.theme);
    scatter_seed(&pool, &cur);

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
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r':
                    cursor_reset(&cur); scatter_seed(&pool, &cur);
                    ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);
                    color_init(cur.theme);
                    break;
                case ' ': scatter_seed(&pool, &cur); break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, 0); break;
                case KEY_RIGHT: cursor_move(&cur, 1); break;
                case KEY_UP:    cursor_move(&cur, 2); break;
                case KEY_DOWN:  cursor_move(&cur, 3); break;
                case '+': case '=':
                    if (cur.density < DENSITY_MAX) {
                        cur.density += DENSITY_STEP; scatter_seed(&pool, &cur);
                    } break;
                case '-':
                    if (cur.density > DENSITY_MIN) {
                        cur.density -= DENSITY_STEP; scatter_seed(&pool, &cur);
                    } break;
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
