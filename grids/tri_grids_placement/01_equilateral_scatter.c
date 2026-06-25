/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral_scatter.c
 *
 * Sprinkle a handful of random triangles around the cursor, then colour
 * each one by how far it sits from the cursor: close ones run warm, far
 * ones run cool. Move the cursor and the same dots just re-colour;
 * SPACE re-sprinkles a fresh batch.
 *
 * Sister demos worth reading next: 01_equilateral_direct.c (place by hand),
 * 01_equilateral_patterns.c (preset stamps), 01_equilateral_path.c (paths).
 * Random numbers use the classic linear-congruential recipe (Numerical
 * Recipes ch. 7).
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
#define MAX_OBJ    1024
#define SCATTER_RADIUS    12

#define DENSITY_DEFAULT  120
#define DENSITY_MIN       20
#define DENSITY_MAX      500
#define DENSITY_STEP      20

#define N_BUCKETS  6
#define N_THEMES   3

#define FPS_EWMA_ALPHA 0.05

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_BUCK0  3   /* nearest, hottest */
#define PAIR_BUCK1  4
#define PAIR_BUCK2  5
#define PAIR_BUCK3  6
#define PAIR_BUCK4  7
#define PAIR_BUCK5  8   /* farthest, coolest */
#define PAIR_HUD    9
#define PAIR_HINT   10

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
    /* 256-color: hot → cold */
    { 196, 202, 214, 226,  82,  39 },   /* warm */
    {  39,  82, 226, 214, 202, 196 },   /* cool */
    {  15, 250, 244, 240, 236, 232 },   /* mono dim */
};
static const short THEME_GRAD_8[N_THEMES][N_BUCKETS] = {
    { COLOR_RED,  COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE },
    { COLOR_BLUE, COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,   COLOR_RED  },
    { COLOR_WHITE,COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE},
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < N_BUCKETS; i++) {
        short fg = (COLORS >= 256) ? THEME_GRAD[theme][i] : THEME_GRAD_8[theme][i];
        init_pair(PAIR_BUCK0 + i, fg, -1);
    }
    init_pair(PAIR_BORDER, COLORS >= 256 ? 248 : COLOR_WHITE, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 gridctx ── */

/* Everything the demo needs to know to turn triangle coordinates into
 * characters on screen, plus the few knobs the user can tweak live. One
 * of these lives for the whole run and gets passed around read-only. */
typedef struct {
    int    rows, cols;         /* terminal size in characters             */
    double tri_size;           /* triangle edge length, in sub-pixels     */
    int    cell_w, cell_h;     /* sub-pixels packed into one character cell */
    int    ox, oy;             /* screen cell the grid origin sits on      */
    double border_w;           /* how close to an edge a point must be to
                                * get drawn as an edge character (0..1)    */
    int    theme;              /* which colour gradient is active (0..2)   */
    int    paused;             /* unused toggle kept for the 'p' key       */
    int    density;            /* how many triangles to scatter            */
    int    scatter_radius;     /* scatter reaches this far from the cursor,
                                * measured in lattice steps                */
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

/* recipe step 1 (reverse) — a pixel -> which triangle (col,row,up) + where
 * inside (fa,fb). Undo the slant, then fa+fb >= 1 picks the upper (△) half. */
static void screen_to_tri(double px, double py, double size,
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
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

/* a triangle's address -> the character cell to draw on (its centroid). */
static void tri_to_screen(const GridCtx *g, int col, int row, int up,
                          int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *scol = g->ox + (int)(cx_pix / g->cell_w);
    *srow = g->oy + (int)(cy_pix / g->cell_h);
}

/* the line char for a point by nearest of the 3 sides ('/','\\','_');
 * out_min returns that side's distance so the caller can skip interiors. */
static char edge_glyph(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char ch1, ch2, ch3;
    if (up == 0) {
        l1 = 1.0 - fa - fb; ch1 = '/';
        l2 = fa;            ch2 = '\\';
        l3 = fb;            ch3 = '_';
    } else {
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa + fb - 1.0;  ch2 = '/';
        l3 = 1.0 - fa;       ch3 = '\\';
    }
    char ch = ch1; double m = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/* recipe step 2 — draw the grid: every cell -> its triangle -> a slash only
 * on the edge cells (interiors stay blank). Nothing stored; redrawn each frame. */
static void draw_lattice(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;
            int    tC, tR, tU;
            double fa, fb, m;
            screen_to_tri(px, py, g->tri_size, &tC, &tR, &tU, &fa, &fb);
            char ch = edge_glyph(tU, fa, fb, &m);
            if (m >= g->border_w) continue;
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

/* ── §5 pool ── */

/* One scattered triangle. col/row name which cell of the triangular
 * lattice it sits in; up flags which of the two triangles in that cell
 * (1 = points up, 0 = points down). glyph is the character drawn for it. */
typedef struct { int col, row, up; char glyph; bool alive; } Obj;

/* A fixed-size bag of scattered triangles. items[0..count) are the live
 * ones; everything past count is stale and ignored. Sized for MAX_OBJ so
 * we never allocate while the demo is running. */
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

/* Our own tiny random-number generator so the scatter is repeatable from
 * a given seed and never depends on the system's rand(). Returns 0..1. */
static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

static void pool_clear(Pool *p) { p->count = 0; }

/* A cheap stand-in for "how far apart are these two triangles." It just
 * adds up the column and row gaps (plus one if they point opposite ways).
 * Good enough to drive colour over short ranges; not a true path length. */
static int triangle_distance(int aC, int aR, int aU, int bC, int bR, int bU)
{
    int d = abs(aC - bC) + abs(aR - bR);
    if (aU != bU) d += 1;
    return d;
}

/* Sort a distance into one of N_BUCKETS colour bands, 0 nearest. max_d is
 * the farthest we expect, so the gradient spreads evenly across it. */
static int distance_bucket(int dist, int max_d)
{
    if (max_d <= 0) return 0;
    int b = (dist * N_BUCKETS) / (max_d + 1);
    if (b >= N_BUCKETS) b = N_BUCKETS - 1;
    return b;
}

/* ── §6 cursor ── */

/* The '@' marker the user steers. Same col/row/up coordinates as a
 * scattered triangle — it's just the one we measure distances from. */
typedef struct { int col, row, up; } Cursor;

/* How one arrow-key press nudges the cursor. Moving between neighbouring
 * triangles isn't a simple grid step: where you land depends on which way
 * the current triangle points, so each direction has two rows here, one
 * for up-triangles and one for down-triangles. The three numbers are the
 * change to (col, row, up). Indexed TRI_DIR[direction][cur.up]. */
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
}

static void cursor_move(Cursor *cur, const GridCtx *g, int dcol, int drow, int dup)
{
    (void)g;
    cur->col += dcol; cur->row += drow; cur->up = dup;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sc, sr;
    tri_to_screen(g, cur->col, cur->row, cur->up, &sc, &sr);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §7 scatter — the distinct step: random triangles in a region ── */

/* SCATTER STRATEGY — fill the pool with `density` triangles at random
 * addresses drawn uniformly from a (2R+1)x(2R+1) square of lattice cells
 * centred on the cursor, each randomly up or down. The tries cap (max*4) is
 * a safety belt so a bad density value can't spin us forever. */
static void scatter_seed(Pool *sp, const GridCtx *g, const Cursor *cur)
{
    sp->count = 0;
    g_seed ^= (unsigned int)clock_ns();
    int n_target = (g->density < MAX_OBJ) ? g->density : MAX_OBJ;
    int tries = 0;
    int R = g->scatter_radius;
    while (sp->count < n_target && tries < n_target * 4) {
        int dcol = (int)(frand() * (2 * R + 1)) - R;
        int drow = (int)(frand() * (2 * R + 1)) - R;
        int up   = frand() > 0.5 ? 1 : 0;
        sp->items[sp->count++] =
            (Obj){ cur->col + dcol, cur->row + drow, up, '*', true };
        tries++;
    }
}

/* Draw each scattered triangle, coloured by its lattice distance from the
 * cursor (near = hot bucket, far = cool). */
static void scatter_draw(const Pool *sp, const GridCtx *g, const Cursor *cur)
{
    int max_d = g->scatter_radius * 2;
    for (int i = 0; i < sp->count; i++) {
        int sc, sr;
        tri_to_screen(g, sp->items[i].col, sp->items[i].row, sp->items[i].up,
                      &sc, &sr);
        if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) continue;

        int dist = triangle_distance(sp->items[i].col, sp->items[i].row, sp->items[i].up,
                                     cur->col, cur->row, cur->up);
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
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             sp->count, g->tri_size, g->theme, fps,
             g->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:reseed  +/-:density  t:theme  r:reset  q:quit  [01 scatter] ");
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

    GridCtx g; ctx_init(&g, 0, 0);
    screen_init(g.theme);
    ctx_init(&g, LINES, COLS);

    Cursor cur; cursor_reset(&cur, &g);
    Pool   sp;  pool_clear(&sp);
    g_seed = (unsigned int)clock_ns();
    scatter_seed(&sp, &g, &cur);

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
                case 'r':          cursor_reset(&cur, &g); scatter_seed(&sp, &g, &cur);
                                   color_init(g.theme); break;
                case ' ':          scatter_seed(&sp, &g, &cur); break;
                case 't':
                    g.theme = (g.theme + 1) % N_THEMES;
                    color_init(g.theme);
                    break;
                case KEY_LEFT:  t = TRI_DIR[0][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_RIGHT: t = TRI_DIR[1][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_UP:    t = TRI_DIR[2][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_DOWN:  t = TRI_DIR[3][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case '+': case '=':
                    if (g.density < DENSITY_MAX) {
                        g.density += DENSITY_STEP; scatter_seed(&sp, &g, &cur);
                    } break;
                case '-':
                    if (g.density > DENSITY_MIN) {
                        g.density -= DENSITY_STEP; scatter_seed(&sp, &g, &cur);
                    } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &sp, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
