/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral_path.c — the straight-line path between two triangles
 *
 * Two markers sit on a triangle grid: START (S) and END (E). Move the '@'
 * cursor with the arrows, press 's' / 'e' to drop the markers, and the
 * program lights up every triangle a straight line from S to E passes
 * through — like a line of sight across triangle graph paper.
 *
 * Sister files: 01_equilateral_direct.c (place triangles by hand),
 *               01_equilateral_patterns.c (stamp preset shapes).
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
#define N_THEMES   4

#define FPS_EWMA_ALPHA 0.05

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_START  3
#define PAIR_END    4
#define PAIR_PATH   5
#define PAIR_HUD    6
#define PAIR_HINT   7

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

/* Each row is one theme's three colors: grid edges, the START marker, the
 * END marker. THEME_FG is for 256-color terminals; the _8 table is the
 * fallback for old 8-color ones. */
static const short THEME_FG[N_THEMES][3] = {
    {  75,  82, 196 },
    {  39, 226, 207 },
    { 207,  82,  39 },
    {  15,  82, 196 },
};
static const short THEME_FG_8[N_THEMES][3] = {
    { COLOR_CYAN,    COLOR_GREEN,   COLOR_RED     },
    { COLOR_BLUE,    COLOR_YELLOW,  COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_GREEN,   COLOR_BLUE    },
    { COLOR_WHITE,   COLOR_GREEN,   COLOR_RED     },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e, fg_s, fg_n;
    if (COLORS >= 256) {
        fg_e = THEME_FG[theme][0]; fg_s = THEME_FG[theme][1]; fg_n = THEME_FG[theme][2];
    } else {
        fg_e = THEME_FG_8[theme][0]; fg_s = THEME_FG_8[theme][1]; fg_n = THEME_FG_8[theme][2];
    }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_START,  fg_s, -1);
    init_pair(PAIR_END,    fg_n, -1);
    init_pair(PAIR_PATH,   COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 gridctx ── */

/*
 * Everything the grid needs to know about itself: how big the triangles
 * are, where the screen origin is, the current look, and where the two
 * path markers live. One bundle passed around instead of loose globals.
 */
typedef struct {
    int    rows, cols;        /* terminal size in character cells          */
    double tri_size;          /* triangle edge length, in pixel units      */
    int    cell_w, cell_h;    /* pixels per character cell (a cell is tall) */
    int    ox, oy;            /* screen cell where pixel (0,0) lands        */
    double border_w;          /* how close to an edge counts as "on it"    */
    int    theme;             /* which color set is active (0..N_THEMES-1)  */
    int    paused;            /* freeze flag (toggled by 'p')               */

    /* The two markers live here, not on the cursor, so the cursor stays a
     * pure "where am I pointing" value. Each marker is a triangle address
     * (col, row, up) plus a flag saying whether it's been placed yet. */
    int    sCol, sRow, sUp;   /* START triangle                            */
    int    eCol, eRow, eUp;   /* END triangle                              */
    int    has_start, has_end;
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
    g->sCol = g->sRow = g->sUp = 0;
    g->eCol = g->eRow = g->eUp = 0;
    g->has_start = 0;
    g->has_end   = 0;
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

/* ── §5 pool ── */

/*
 * One triangle on the path: its grid address (col, row, up) and the
 * character we draw at its center. 'alive' lets us skip slots without
 * compacting the array.
 */
typedef struct { int col, row, up; char glyph; bool alive; } Obj;

/*
 * The collected path: a fixed array of triangles plus how many are in use.
 * Refilled from scratch each time the markers move, so 'count' resets to 0
 * rather than freeing anything. Capped at MAX_OBJ — a very long line can
 * fill it and quietly stop adding.
 */
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

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].up,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

/* ── §6 cursor ── */

/* Where the '@' is pointing: a triangle address, nothing more. */
typedef struct { int col, row, up; } Cursor;

/*
 * Where each arrow key moves the cursor. Triangles point up or down, and a
 * key behaves differently depending on which way the current triangle
 * faces — so we look up [direction][up] to get the (dcol, drow, new-up)
 * step to apply. This little table replaces a nest of if/else branches.
 */
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

static void marker_draw(const GridCtx *g, int col, int row, int up,
                        char glyph, int pair)
{
    int sc, sr;
    ctx_to_screen(g, col, row, up, &sc, &sr);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sr, sc, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* ── §7 path — collect the triangles a straight S→E line crosses ── */

/*
 * Walk a straight line from the START triangle's center to the END's,
 * stopping every quarter-triangle to ask "which triangle is this point
 * in?" and saving each new answer. Stepping that finely means we never
 * jump clean over a triangle the line clips. Cheap to call: it just runs
 * when a marker or the size changes, not every frame.
 */
static void path_compute(Pool *p, const GridCtx *g)
{
    pool_clear(p);
    if (!g->has_start || !g->has_end) return;

    double sx, sy, ex, ey;
    tri_centroid_pixel(g->sCol, g->sRow, g->sUp, g->tri_size, &sx, &sy);
    tri_centroid_pixel(g->eCol, g->eRow, g->eUp, g->tri_size, &ex, &ey);

    double dx = ex - sx, dy = ey - sy;
    double dist = sqrt(dx*dx + dy*dy);
    /* START and END on the same spot: skip the walk (it would divide by a
     * zero step count) and just record that one triangle. */
    if (dist < 1e-6) {
        pool_place(p, g->sCol, g->sRow, g->sUp, '*');
        return;
    }
    double step = g->tri_size * 0.25;
    int    n    = (int)(dist / step) + 1;
    for (int i = 0; i <= n; i++) {
        double t = (double)i / (double)n;
        double px = sx + t * dx;
        double py = sy + t * dy;
        int    tC, tR, tU;
        double fa, fb;
        pixel_to_tri(px, py, g->tri_size, &tC, &tR, &tU, &fa, &fb);
        pool_place(p, tC, tR, tU, '*');
    }
}

/* ── §8 scene ── */

static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  path:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             p->count, g->tri_size, g->theme, fps,
             g->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  s:set-start  e:set-end  spc:clear  +/-:size  t:theme  r:reset  q:quit  [01 path] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(p, g);
    if (g->has_start) marker_draw(g, g->sCol, g->sRow, g->sUp, 'S', PAIR_START);
    if (g->has_end)   marker_draw(g, g->eCol, g->eRow, g->eUp, 'E', PAIR_END);
    cursor_draw(cur, g);
    hud_draw(g, p, cur, fps);
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

    Cursor cur;  cursor_reset(&cur, &g);
    Pool   path; pool_clear(&path);

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
                case 'r':          cursor_reset(&cur, &g); pool_clear(&path);
                                   g.has_start = 0; g.has_end = 0;
                                   color_init(g.theme); break;
                case ' ':          g.has_start = 0; g.has_end = 0;
                                   pool_clear(&path); break;
                case 's':
                    g.sCol = cur.col; g.sRow = cur.row; g.sUp = cur.up;
                    g.has_start = 1; path_compute(&path, &g);
                    break;
                case 'e':
                    g.eCol = cur.col; g.eRow = cur.row; g.eUp = cur.up;
                    g.has_end = 1; path_compute(&path, &g);
                    break;
                case 't':
                    g.theme = (g.theme + 1) % N_THEMES;
                    color_init(g.theme);
                    break;
                case KEY_LEFT:  t = TRI_DIR[0][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_RIGHT: t = TRI_DIR[1][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_UP:    t = TRI_DIR[2][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_DOWN:  t = TRI_DIR[3][cur.up]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) {
                        g.tri_size += TRI_SIZE_STEP; path_compute(&path, &g);
                    } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) {
                        g.tri_size -= TRI_SIZE_STEP; path_compute(&path, &g);
                    } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &path, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
