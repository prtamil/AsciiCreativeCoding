/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_isometric_path.c — shortest line of triangles between two points,
 * drawn on the solid-colour "stacked cubes" iso grid.
 *
 * Move the '@' cursor with the arrows. Press 's' to drop a start, 'e' to
 * drop an end; every triangle a straight line between them passes over
 * lights up with a bright '*'.
 *
 * Sister files: grids/tri_grids/05_isometric.c (the coloured grid itself),
 *               01_equilateral_path.c (same idea, outlined edges).
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

#define PATH_STEP_FRAC   0.25   /* sample the line every quarter-triangle */

#define MAX_OBJ   1024
#define N_PALETTE 6
#define N_THEMES  3

#define FPS_EWMA_ALPHA  0.05

#define PAIR_FILL_BASE 1                        /* six fill colours in slots 1..6 */
#define PAIR_PATH     (PAIR_FILL_BASE + N_PALETTE)
#define PAIR_START    (PAIR_PATH + 1)
#define PAIR_END      (PAIR_START + 1)
#define PAIR_CURSOR   (PAIR_END + 1)
#define PAIR_HUD      (PAIR_CURSOR + 1)
#define PAIR_HINT     (PAIR_HUD + 1)

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
    init_pair(PAIR_PATH,   COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_START,  COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_END,    COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLACK);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 tri mapping & lattice ── */

/* GridCtx — the iso triangle grid for one frame: terminal size, triangle
 * size, where the grid centre sits, and how far the cursor may roam. A
 * resize or size change just rewrites these in place. */
typedef struct {
    int    rows, cols;        /* terminal size, in character cells */
    int    cell_w, cell_h;    /* sub-pixels per character column / row */
    double tri_size;          /* edge length of a triangle, in pixels */
    int    ox, oy;            /* screen cell the grid is centred on */
    int    max_col, max_row;  /* how far the cursor may roam from centre */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, double tri_size)
{
    g->rows = rows; g->cols = cols;
    g->cell_w = CELL_W; g->cell_h = CELL_H;
    g->tri_size = tri_size;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->max_col = cols / 2;
    g->max_row = rows / 2;
}

/* a pixel -> which triangle (col,row,up) + where inside (fa,fb). Undo the
 * iso skew: a = px/size - b/2; fa+fb >= 1 picks the up (△) half. */
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

/* the pixel at a triangle's centre (1/3 in from its corners). Where the
 * cursor/marker draws and where a path-line starts and ends. */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

/* a triangle's address -> the character cell to draw on. */
static void tri_to_screen(const GridCtx *g, int col, int row, int up,
                          int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *scol = g->ox + (int)(cx_pix / g->cell_w);
    *srow = g->oy + (int)(cy_pix / g->cell_h);
}

/* which of the six fills a triangle gets. Mixing address + orientation makes
 * neighbours differ, so the grid reads as a wall of stacked cubes. */
static int palette_index(int col, int row, int up)
{
    int k = col + 2 * row + up;
    k %= N_PALETTE; if (k < 0) k += N_PALETTE;
    return k;
}

/* draw the grid: every cell -> its triangle -> a solid block of that
 * triangle's fill colour. Nothing stored; redrawn each frame. */
static void draw_lattice(const GridCtx *g)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;
            int    tC, tR, tU;
            double fa, fb;
            screen_to_tri(px, py, g->tri_size, &tC, &tR, &tU, &fa, &fb);
            int pair = PAIR_FILL_BASE + palette_index(tC, tR, tU);
            attron(COLOR_PAIR(pair));
            mvaddch(row, col, ' ');
            attroff(COLOR_PAIR(pair));
        }
    }
}

/* ── §5 pool — the line of path triangles ── */

/* One triangle the path passes through, ready to draw. */
typedef struct {
    int  col, row, up;   /* which triangle (address + orientation) */
    char glyph;          /* the character to stamp ('*') */
    bool alive;          /* false means this slot is free */
} Obj;

/* The full list of path triangles: a plain fixed array we refill from
 * scratch each time the path changes — no growing, no freeing. */
typedef struct {
    Obj items[MAX_OBJ];
    int count;
} Pool;

static void pool_clear(Pool *p) { p->count = 0; }

/* already on the path? skips duplicates when the line lingers in a triangle */
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
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        tri_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].up,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

/* ── §6 cursor — the '@' you steer, plus the start/end you drop ── */

/* The whole interactive state: where the '@' is, the start/end the user has
 * dropped, and the UI toggles. */
typedef struct {
    int col, row, up;                /* the '@' cursor's triangle */
    int sCol, sRow, sUp;             /* the start triangle (set by 's') */
    int eCol, eRow, eUp;             /* the end triangle (set by 'e') */
    int has_start, has_end;          /* have those been dropped yet? */
    int theme, paused;               /* current colour set; pause flag */
} Cursor;

/*
 * One arrow press isn't a simple ±1 on a triangle grid: an up triangle and a
 * down triangle have different neighbours. Look up [arrow][half] to get the
 * col/row shift and the half you end up on.
 *   arrows: 0=LEFT 1=RIGHT 2=UP 3=DOWN     half: 0=▽ (lower)  1=△ (upper)
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
    cur->has_start = 0; cur->has_end = 0;
    cur->theme = 0; cur->paused = 0;
}

/* nudge the cursor by the TRI_DIR change for this arrow; clamp to max_col/row
 * so it can't wander off the addressable grid. */
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
    tri_to_screen(g, cur->col, cur->row, cur->up, &sc, &sr);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD | A_REVERSE);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD | A_REVERSE);
    }
}

/* ── §7 placement — lay objects along a path ── */

/* path_lay — fill the pool with every triangle a straight line from start to
 * end crosses. Walk the line in PATH_STEP_FRAC steps, noting the triangle at
 * each sample; pool_place drops repeats so each triangle appears once. */
static void path_lay(Pool *pool, const Cursor *cur, const GridCtx *g)
{
    pool_clear(pool);
    if (!cur->has_start || !cur->has_end) return;
    double sx, sy, ex, ey;
    tri_centroid_pixel(cur->sCol, cur->sRow, cur->sUp, g->tri_size, &sx, &sy);
    tri_centroid_pixel(cur->eCol, cur->eRow, cur->eUp, g->tri_size, &ex, &ey);
    double dx = ex - sx, dy = ey - sy;
    double dist = sqrt(dx*dx + dy*dy);
    if (dist < 1e-6) {
        pool_place(pool, cur->sCol, cur->sRow, cur->sUp, '*');
        return;
    }
    double step = g->tri_size * PATH_STEP_FRAC;
    int n = (int)(dist / step) + 1;
    for (int i = 0; i <= n; i++) {
        double t = (double)i / (double)n;
        double px = sx + t * dx, py = sy + t * dy;
        int tC, tR, tU; double fa, fb;
        screen_to_tri(px, py, g->tri_size, &tC, &tR, &tU, &fa, &fb);
        pool_place(pool, tC, tR, tU, '*');
    }
}

static void marker_draw(const GridCtx *g, int col, int row, int up,
                        char glyph, int pair)
{
    int sc, sr;
    tri_to_screen(g, col, row, up, &sc, &sr);
    if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sr, sc, glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* ── §8 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  path:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             p->count, g->tri_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  s:set-start  e:set-end  spc:clear  +/-:size  t:theme  r:reset  q:quit  [05 path] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                       double fps)
{
    erase();
    draw_lattice(g);
    pool_draw(p, g);
    if (cur->has_start)
        marker_draw(g, cur->sCol, cur->sRow, cur->sUp, 'S', PAIR_START);
    if (cur->has_end)
        marker_draw(g, cur->eCol, cur->eRow, cur->eUp, 'E', PAIR_END);
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
    Pool   path; pool_clear(&path);
    GridCtx g;

    cur.col = 0; cur.row = 0; cur.up = 0;
    cur.has_start = 0; cur.has_end = 0; cur.theme = 0; cur.paused = 0;
    screen_init(cur.theme);
    ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS;
    int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            /* ncurses needs an endwin()/refresh() bounce to pick up the
             * terminal's new size after the window changes. */
            g_need_resize = 0; endwin(); refresh();
            ctx_init(&g, LINES, COLS, g.tri_size);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r':
                    cursor_reset(&cur, &g); pool_clear(&path);
                    color_init(cur.theme);
                    break;
                case ' ': cur.has_start = 0; cur.has_end = 0; pool_clear(&path); break;
                case 's':
                    cur.sCol = cur.col; cur.sRow = cur.row; cur.sUp = cur.up;
                    cur.has_start = 1;
                    path_lay(&path, &cur, &g);
                    break;
                case 'e':
                    cur.eCol = cur.col; cur.eRow = cur.row; cur.eUp = cur.up;
                    cur.has_end = 1;
                    path_lay(&path, &cur, &g);
                    break;
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
                        path_lay(&path, &cur, &g);
                    } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) {
                        g.tri_size -= TRI_SIZE_STEP;
                        path_lay(&path, &cur, &g);
                    } break;
            }
        }
        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&g, &cur, &path, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
