/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral_scatter.c — random scatter colored by distance to cursor
 *
 * DEMO: A random scatter of N triangles fills a square region around the
 *       cursor. Each triangle is colored on a 6-stop gradient by its
 *       cell-distance from the cursor — closer = warm, farther = cool.
 *       Press SPACE to reseed; +/- to change density (N).
 *
 * Study alongside: 01_equilateral_direct.c (manual placement),
 *                  01_equilateral_patterns.c (preset stamps),
 *                  01_equilateral_path.c (line-of-sight path).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, MAX_OBJ, RADIUS, DENSITY
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — gradient palette (6 distance buckets)
 *   §4 gridctx  — GridCtx + pixel/centroid/edge formula
 *   §5 pool     — Pool: store (col, row, up) entries
 *   §6 cursor   — Cursor + TRI_DIR + reset / move / draw
 *   §7 scatter  — random spawn + Manhattan distance bucketing
 *   §8 scene    — hud_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move  spc:reseed  +/-:density   r:reset
 *        t:theme  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/01_equilateral_scatter.c \
 *       -o 01_equilateral_scatter -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Random scatter. Pick N random (Δcol, Δrow, up) within
 *                  ±RADIUS of the cursor; color by Manhattan-style cell
 *                  distance from cursor, bucketed into 6 gradient slots.
 *
 * Distance metric: |Δcol| + |Δrow| + (Δup ? 1 : 0). Cheap and roughly
 *                  matches actual edge-walk distance on the equilateral
 *                  lattice for short distances.
 *
 * Re-seeding     : SPACE re-randomises with a new seed (xor'd by clock).
 *                  Moving the cursor does NOT re-seed — but recolours
 *                  the existing scatter as the cursor moves.
 *
 * References     :
 *   Linear congruential generator — Numerical Recipes ch. 7
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two halves: STORAGE (random scatter, generated once per reseed) and
 * COLOURING (a pure function of cursor distance, recomputed every
 * frame). Moving the cursor never re-seeds; it only re-paints the
 * existing scatter through a different distance lens.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine sprinkling salt on a triangular tablecloth — the grains land
 * randomly inside a small square region. Now point a coloured spotlight
 * (the cursor) at the cloth; grains close to the beam glow warm,
 * grains farther away cool. Move the spotlight: same grains, different
 * colours. SPACE re-sprinkles.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. ctx_draw_bg — equilateral edge characters.
 *  3. For each scatter object:
 *       d = |obj.col - cur.col| + |obj.row - cur.row|
 *           + (obj.up != cur.up ? 1 : 0)
 *       bucket = min(d, N_BUCKETS - 1)
 *       attron(COLOR_PAIR(PAIR_BUCKET_0 + bucket))
 *       mvaddch(centroid_screen, '*')
 *  4. cursor_draw — '@' on top.
 *
 *  Reseed (only on SPACE or +/- density):
 *    pool->count = 0
 *    g_seed ^= clock_ns()
 *    for i in 0..density:
 *      dC = frand·(2·R+1) - R   ; dR = same
 *      up = (frand > 0.5) ? △ : ▽
 *      pool_place(cur.col+dC, cur.row+dR, up)   // dedup
 *
 * KEY FORMULAS
 * ────────────
 *  Manhattan-style cell distance (cheap, monotonic-ish on the
 *  equilateral lattice for short ranges):
 *    d = |Δcol| + |Δrow| + (Δup ? 1 : 0)
 *
 *  LCG step (Numerical Recipes ch. 7):
 *    g_seed = g_seed · 1103515245 + 12345    (mod 2³²)
 *    frand  = ((g_seed >> 16) & 0x7FFF) / 32767.0
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Manhattan vs true edge distance: on the equilateral lattice the
 *    true minimum-edges distance is more complex (depends on parity).
 *    Manhattan is a useful proxy for SMALL distances; for large ones
 *    the gradient may "bend" visually. Acceptable for a colouring
 *    demo, not for pathfinding.
 *  • Density cap: density values approaching MAX_OBJ saturate the pool;
 *    pool_place deduplicates, so "tries" may not reach the requested
 *    count. The visible scatter may look thinner than density would
 *    suggest at high values.
 *  • Reseed on cursor move: NOT reseed-triggering by design. Move
 *    cursor to "rotate the spotlight" without disturbing the scatter.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Place cursor in the middle of a freshly-seeded scatter — colours
 *  are warmest near '@', cooling outward. Walk the cursor to the edge:
 *  the same dots stay, but the warm/cool boundary follows the cursor.
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    rows, cols;
    double tri_size;
    int    cell_w, cell_h;
    int    ox, oy;
    double border_w;
    int    theme;
    int    paused;
    int    density;            /* scatter target count           */
    int    scatter_radius;     /* lattice half-extent of scatter */
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
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *scol = g->ox + (int)(cx_pix / g->cell_w);
    *srow = g->oy + (int)(cy_pix / g->cell_h);
}

static char tri_edge_char(int up, double fa, double fb, double *out_min)
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

static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

static void pool_clear(Pool *p) { p->count = 0; }

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int col, row, up; } Cursor;

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
    ctx_to_screen(g, cur->col, cur->row, cur->up, &sc, &sr);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scatter                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scatter_seed(Pool *sp, const GridCtx *g, const Cursor *cur)
{
    sp->count = 0;
    g_seed ^= (unsigned int)clock_ns();
    int max = (g->density < MAX_OBJ) ? g->density : MAX_OBJ;
    int tries = 0;
    int R = g->scatter_radius;
    while (sp->count < max && tries < max * 4) {
        int dC = (int)(frand() * (2 * R + 1)) - R;
        int dR = (int)(frand() * (2 * R + 1)) - R;
        int up = frand() > 0.5 ? 1 : 0;
        sp->items[sp->count++] = (Obj){ cur->col + dC, cur->row + dR, up, '*', true };
        tries++;
    }
}

static void scatter_draw(const Pool *sp, const GridCtx *g, const Cursor *cur)
{
    int max_d = g->scatter_radius * 2;
    for (int i = 0; i < sp->count; i++) {
        int sc, sr;
        ctx_to_screen(g, sp->items[i].col, sp->items[i].row, sp->items[i].up,
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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
    ctx_draw_bg(g);
    scatter_draw(sp, g, cur);
    cursor_draw(cur, g);
    hud_draw(g, sp, cur, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
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
