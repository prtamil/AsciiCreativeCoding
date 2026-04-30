/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_30_60_90_direct.c — direct object placement on the kisrhombille grid (equilaterals + 3 medians)
 *
 * DEMO: An equilateral triangular grid fills the screen. Move '@' between
 *       triangles with arrow keys. Press SPACE to toggle a '*' object at
 *       the cursor triangle. Objects are stored by lattice address
 *       (col, row, up) and survive resize — they follow their triangle
 *       when the terminal changes size. 'g' cycles the placed glyph.
 *
 * Study alongside: grids/tri_grids/01_equilateral.c (background rasterizer),
 *                  grids/hex_grids_placement/01_hex_direct.c (same idea on hex).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, MAX_OBJ
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: edge / cursor / object / HUD / hint
 *   §4 formula  — pixel ↔ lattice + triangle centroid + edge char
 *   §5 pool     — ObjectPool: place / remove / toggle / clear / draw
 *   §6 cursor   — TRI_DIR + cursor_step + cursor_draw
 *   §7 scene    — grid_draw + scene_draw
 *   §8 screen   — ncurses init / cleanup
 *   §9 app      — signals, main loop
 *
 * Keys:  arrows:move  spc:toggle  g:glyph  C:clear  r:reset
 *        +/-:size  t:theme  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/04_30_60_90_direct.c \
 *       -o 01_equilateral_direct -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Direct placement using a movable cursor. The cursor
 *                  holds (col, row, up) in TRIANGLE-lattice space. SPACE
 *                  toggles an object at that address; the object's screen
 *                  position is recomputed each frame from tri_centroid_pixel.
 *
 * Data-structure : ObjectPool — flat array of TObj{col, row, up, glyph}.
 *                  Removal swaps the dead slot with the last item (O(1)).
 *
 * Rendering      : Three-pass per frame:
 *                    (1) grid_draw rasterizes the equilateral background
 *                    (2) pool_draw renders each placed object at its centroid
 *                    (3) cursor_draw places '@' at the cursor centroid
 *                  The cursor draws over objects so the user always sees it.
 *
 * References     :
 *   Triangular tiling — https://en.wikipedia.org/wiki/Triangular_tiling
 *   Object pool pattern — gameprogrammingpatterns.com/object-pool.html
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
#define MEDIAN_T   0.05
#define MAX_OBJ    256
#define N_GLYPHS   6
#define N_THEMES   4

#define PAIR_BORDER 1
#define PAIR_MEDIAN 2
#define PAIR_CURSOR 3
#define PAIR_OBJECT 4
#define PAIR_HUD    5
#define PAIR_HINT   6

static const char GLYPHS[N_GLYPHS] = { '*', 'o', '+', '#', 'X', '%' };

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
    struct timespec r = { .tv_sec = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEME_FG[N_THEMES][2] = {
    /* edge,  object */
    {  75, 226 },
    {  82, 207 },
    { 207,  82 },
    {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_YELLOW },
    { COLOR_GREEN,   COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_GREEN  },
    { COLOR_WHITE,   COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e, fg_o;
    if (COLORS >= 256) { fg_e = THEME_FG[theme][0];   fg_o = THEME_FG[theme][1];   }
    else               { fg_e = THEME_FG_8[theme][0]; fg_o = THEME_FG_8[theme][1]; }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_MEDIAN, COLORS >= 256 ? 39 : COLOR_BLUE, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — pixel ↔ lattice (skew, equilateral)                       */
/* ═══════════════════════════════════════════════════════════════════════ */

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

static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    if (up == 0) {
        l1 = 1.0 - fa - fb; ch1 = '/';
        l2 = fa;            ch2 = '\\';
        l3 = fb;            ch3 = '_';
    } else {
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa + fb - 1.0;  ch2 = '/';
        l3 = 1.0 - fa;       ch3 = '\\';
    }
    char   ch = ch1;
    double m  = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/*
 * tri_median_char — three median signed distances inside the triangle.
 * See grids/tri_grids/04_30_60_90.c §4 for the line equations.
 */
static char tri_median_char(int up, double fa, double fb, double *out_min)
{
    static const double INV_SQRT2 = 0.70710678118654752440;
    static const double INV_SQRT5 = 0.44721359549995793928;
    double m1, m2, m3; char ch1, ch2, ch3;
    if (up == 0) {
        m1 = fabs(fa - fb)         * INV_SQRT2; ch1 = '\\';
        m2 = fabs(fa + 2.0*fb - 1) * INV_SQRT5; ch2 = '/';
        m3 = fabs(2.0*fa + fb - 1) * INV_SQRT5; ch3 = '|';
    } else {
        m1 = fabs(fa - fb)         * INV_SQRT2; ch1 = '\\';
        m2 = fabs(2.0*fa + fb - 2) * INV_SQRT5; ch2 = '|';
        m3 = fabs(fa + 2.0*fb - 2) * INV_SQRT5; ch3 = '/';
    }
    char ch = ch1; double m = m1;
    if (m2 < m) { m = m2; ch = ch2; }
    if (m3 < m) { m = m3; ch = ch3; }
    *out_min = m;
    return ch;
}

/* Map (col, row, up) → terminal cell, given centring offsets. */
static int tri_to_screen(int col, int row, int up, double size,
                         int ox, int oy, int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, size, &cx_pix, &cy_pix);
    *scol = ox + (int)(cx_pix / CELL_W);
    *srow = oy + (int)(cy_pix / CELL_H);
    return 1;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int  col, row, up;
    char glyph;
} TObj;

typedef struct {
    TObj objs[MAX_OBJ];
    int  count;
} ObjectPool;

static void pool_clear(ObjectPool *p) { p->count = 0; }

static int pool_find(const ObjectPool *p, int col, int row, int up)
{
    for (int i = 0; i < p->count; i++) {
        if (p->objs[i].col == col && p->objs[i].row == row && p->objs[i].up == up)
            return i;
    }
    return -1;
}

static void pool_remove_at(ObjectPool *p, int idx)
{
    if (idx < 0 || idx >= p->count) return;
    p->objs[idx] = p->objs[p->count - 1];
    p->count--;
}

/*
 * pool_toggle — if the cell already has an object, remove it; otherwise
 * append a new one with the chosen glyph.
 */
static void pool_toggle(ObjectPool *p, int col, int row, int up, char glyph)
{
    int idx = pool_find(p, col, row, up);
    if (idx >= 0) { pool_remove_at(p, idx); return; }
    if (p->count >= MAX_OBJ) return;
    p->objs[p->count++] = (TObj){ col, row, up, glyph };
}

static void pool_draw(const ObjectPool *p, double size,
                      int ox, int oy, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int sc, sr;
        tri_to_screen(p->objs[i].col, p->objs[i].row, p->objs[i].up,
                      size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->objs[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    col, row, up;
    double tri_size;
    int    glyph_idx;
    int    theme;
    int    paused;
} Cursor;

static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 }, {  0,  0,  0 } },
    /* RIGHT */ { {  0,  0,  1 }, { +1,  0,  0 } },
    /* UP    */ { {  0, -1,  1 }, {  0,  0,  0 } },
    /* DOWN  */ { {  0,  0,  1 }, {  0, +1,  0 } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->up = 0;
    cur->tri_size  = TRI_SIZE_DEFAULT;
    cur->glyph_idx = 0;
    cur->theme     = 0;
    cur->paused    = 0;
}

static void cursor_step(Cursor *cur, int dir)
{
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0]; cur->row += t[1]; cur->up = t[2];
}

static void cursor_draw(const Cursor *cur, int ox, int oy, int rows, int cols)
{
    int sc, sr;
    tri_to_screen(cur->col, cur->row, cur->up, cur->tri_size, ox, oy, &sc, &sr);
    if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void grid_draw(int rows, int cols, double size, int ox, int oy)
{
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;
            int    tC, tR, tU;
            double fa, fb, em, mm;
            pixel_to_tri(px, py, size, &tC, &tR, &tU, &fa, &fb);
            char ech = tri_edge_char(tU, fa, fb, &em);
            char mch = tri_median_char(tU, fa, fb, &mm);
            if (em < BORDER_W && em <= mm) {
                attron(COLOR_PAIR(PAIR_BORDER));
                mvaddch(row, col, (chtype)(unsigned char)ech);
                attroff(COLOR_PAIR(PAIR_BORDER));
            } else if (mm < MEDIAN_T) {
                attron(COLOR_PAIR(PAIR_MEDIAN));
                mvaddch(row, col, (chtype)(unsigned char)mch);
                attroff(COLOR_PAIR(PAIR_MEDIAN));
            }
        }
    }
}

static void scene_draw(int rows, int cols, const Cursor *cur,
                       const ObjectPool *pool, double fps)
{
    erase();
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur->tri_size, ox, oy);
    pool_draw(pool, cur->tri_size, ox, oy, rows, cols);
    cursor_draw(cur, ox, oy, rows, cols);

    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             pool->count, GLYPHS[cur->glyph_idx], cur->tri_size,
             cur->theme, fps, cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  spc:toggle  g:glyph  C:clear  +/-:size  t:theme  r:reset  q:quit  [04 30-60-90 direct] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  app                                                                 */
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

    Cursor     cur;     cursor_reset(&cur);
    ObjectPool pool;    pool_clear(&pool);
    screen_init(cur.theme);

    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          cur.paused ^= 1; break;
                case 'r':          cursor_reset(&cur); pool_clear(&pool);
                                   color_init(cur.theme); break;
                case 'C':          pool_clear(&pool); break;
                case 'g':          cur.glyph_idx = (cur.glyph_idx + 1) % N_GLYPHS; break;
                case ' ':          pool_toggle(&pool, cur.col, cur.row, cur.up,
                                                GLYPHS[cur.glyph_idx]); break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_step(&cur, 0); break;
                case KEY_RIGHT: cursor_step(&cur, 1); break;
                case KEY_UP:    cursor_step(&cur, 2); break;
                case KEY_DOWN:  cursor_step(&cur, 3); break;
                case '+': case '=':
                    if (cur.tri_size < TRI_SIZE_MAX) { cur.tri_size += TRI_SIZE_STEP; } break;
                case '-':
                    if (cur.tri_size > TRI_SIZE_MIN) { cur.tri_size -= TRI_SIZE_STEP; } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;

        scene_draw(rows, cols, &cur, &pool, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
