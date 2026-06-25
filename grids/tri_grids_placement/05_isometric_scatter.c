/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_isometric_scatter.c — random dots on an isometric triangle grid,
 * coloured by how far each one sits from the cursor (warm = near, cool = far).
 * The background is the same solid-filled iso grid from tri_grids/05_isometric.c;
 * arrows walk the cursor, SPACE drops a fresh batch of dots, +/- changes how many.
 *
 * Sister files: grids/tri_grids/05_isometric.c (the grid + fill palette),
 *               05_isometric_direct.c, 01_equilateral_scatter.c.
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

#define TRI_SIZE_DEFAULT 14.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

#define MAX_OBJ        1024
#define SCATTER_RADIUS   12
#define DENSITY_DEFAULT 120
#define DENSITY_MIN      20
#define DENSITY_MAX     500
#define DENSITY_STEP     20
#define N_PALETTE 6
#define N_BUCKETS 6
#define N_THEMES  3

#define FPS_EWMA_ALPHA  0.05

/* Colour-pair slot numbers. The fill pairs colour the background grid;
 * the bucket pairs colour the scattered dots by distance band. */
#define PAIR_FILL_BASE  1                                 /* background grid: one pair per fill colour */
#define PAIR_BUCK0     (PAIR_FILL_BASE + N_PALETTE)       /* dots: one pair per distance band */
#define PAIR_CURSOR    (PAIR_BUCK0 + N_BUCKETS)
#define PAIR_HUD       (PAIR_CURSOR + 1)
#define PAIR_HINT      (PAIR_HUD + 1)

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
/* Dot colours, near to far. Kept bright so the dots stay readable on top of
 * any background tile. */
static const short GRAD256[N_BUCKETS] = { 15, 226, 214, 196, 207, 51 };
static const short GRAD8[N_BUCKETS]   = {
    COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_MAGENTA, COLOR_CYAN
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    for (int i = 0; i < N_PALETTE; i++) {
        short bg = (COLORS >= 256) ? PAL256[theme][i] : PAL8[theme][i];
        init_pair(PAIR_FILL_BASE + i, COLOR_BLACK, bg);
    }
    for (int i = 0; i < N_BUCKETS; i++) {
        short fg = (COLORS >= 256) ? GRAD256[i] : GRAD8[i];
        init_pair(PAIR_BUCK0 + i, fg, -1);
    }
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLACK);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 gridctx ── */

/* Everything needed to turn a triangle's grid address into a screen spot.
 * Built once at start-up (and again on resize) so the drawing code can stay
 * simple. */
typedef struct {
    int    rows, cols;        /* terminal size in characters */
    int    cw, ch;            /* how many sub-pixels wide/tall one character is */
    double tri_size;          /* edge length of one triangle, in sub-pixels */
    int    ox, oy;            /* where grid (0,0) lands on screen — roughly centre */
    int    max_col, max_row;  /* how far the cursor may roam from the origin */
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

/* One scattered dot, addressed by which triangle it sits in. */
typedef struct {
    int  col, row, up;   /* which triangle: column, row, and which half (up vs down) */
    char glyph;          /* the character drawn for it (always '*' here) */
    bool alive;          /* false = skip this slot when drawing */
} Obj;

/* The whole batch of dots. Plain fixed array — we never free or grow it, we
 * just refill it from the front on each reseed. */
typedef struct {
    Obj items[MAX_OBJ];
    int count;           /* how many slots are actually in use */
} Pool;

static void pool_clear(Pool *p) { p->count = 0; }

/* ── §6 cursor ── */

/* The '@' marker you steer, plus a few app-wide settings that ride along with
 * it since the same struct gets passed everywhere. */
typedef struct {
    int col, row, up;     /* which triangle the cursor is on */
    int density;          /* how many dots to drop on the next reseed */
    int theme, paused;    /* current colour theme; paused is unused here */
} Cursor;

/* Where each arrow press takes you, in triangle coordinates. Moving on this
 * grid is awkward: the step depends on whether you're on an up- or down-facing
 * triangle, so we look it up instead of computing it. First index is the
 * arrow (left/right/up/down), second is your current up/down half, then the
 * three numbers are the change to col, row, and the new up value. */
static const int TRI_DIR[4][2][3] = {
    { { -1,  0,  1 }, {  0,  0,  0 } },
    { {  0,  0,  1 }, { +1,  0,  0 } },
    { {  0, -1,  1 }, {  0,  0,  0 } },
    { {  0,  0,  1 }, {  0, +1,  0 } },
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0; cur->row = 0; cur->up = 0;
    cur->density = DENSITY_DEFAULT;
    cur->theme = 0; cur->paused = 0;
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

/* ── §7 mode — drop the dots and pick each one's colour by distance ── */

/* Our own tiny random-number generator (a classic LCG, Numerical Recipes
 * ch. 7). Built in rather than using rand() so a given seed always replays
 * the same scatter. Returns a number in [0, 1). */
static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

static int triangle_distance(int aC, int aR, int aU, int bC, int bR, int bU)
{
    int d = abs(aC - bC) + abs(aR - bR);
    if (aU != bU) d += 1;
    return d;
}

static int distance_bucket(int dist, int max_d)
{
    if (max_d <= 0) return 0;
    int b = (dist * N_BUCKETS) / (max_d + 1);
    if (b >= N_BUCKETS) b = N_BUCKETS - 1;
    return b;
}

static void scatter_seed(Pool *pool, const Cursor *cur)
{
    pool_clear(pool);
    g_seed ^= (unsigned int)clock_ns();   /* stir in the clock so each reseed looks new */
    int max = (cur->density < MAX_OBJ) ? cur->density : MAX_OBJ;
    int tries = 0;
    while (pool->count < max && tries < max * 4) {
        int dC = (int)(frand() * (2 * SCATTER_RADIUS + 1)) - SCATTER_RADIUS;
        int dR = (int)(frand() * (2 * SCATTER_RADIUS + 1)) - SCATTER_RADIUS;
        int up = frand() > 0.5 ? 1 : 0;
        pool->items[pool->count++] = (Obj){ cur->col + dC, cur->row + dR, up, '*', true };
        tries++;
    }
}

static void scatter_draw(const Pool *p, const Cursor *cur, const GridCtx *g)
{
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].up,
                      &sc, &sr);
        if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) continue;
        int dist = triangle_distance(p->items[i].col, p->items[i].row, p->items[i].up,
                                     cur->col, cur->row, cur->up);
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
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             p->count, g->tri_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:reseed  +/-:density  t:theme  r:reset  q:quit  [05 scatter] ");
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
    Pool   pool; pool_clear(&pool);
    GridCtx g;

    cur.col = 0; cur.row = 0; cur.up = 0;
    cur.density = DENSITY_DEFAULT;
    cur.theme = 0; cur.paused = 0;
    g_seed = (unsigned int)clock_ns();
    screen_init(cur.theme);
    ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);
    cursor_reset(&cur, &g);
    scatter_seed(&pool, &cur);

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
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r':
                    cursor_reset(&cur, &g); scatter_seed(&pool, &cur);
                    color_init(cur.theme);
                    break;
                case ' ': scatter_seed(&pool, &cur); break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, &g, 0); break;
                case KEY_RIGHT: cursor_move(&cur, &g, 1); break;
                case KEY_UP:    cursor_move(&cur, &g, 2); break;
                case KEY_DOWN:  cursor_move(&cur, &g, 3); break;
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
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&g, &cur, &pool, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
