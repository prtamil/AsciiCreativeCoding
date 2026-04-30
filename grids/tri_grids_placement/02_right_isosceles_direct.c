/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_right_isosceles_direct.c — direct placement on the half-rect grid
 *
 * DEMO: Square cells split by a single '\' diagonal into UR / LL right-
 *       isosceles triangles. Move '@' with arrows; SPACE toggles a glyph
 *       at the cursor triangle. 'g' cycles glyphs.
 *
 * Study alongside: grids/tri_grids/02_right_isosceles.c (background),
 *                  01_equilateral_direct.c (same idea on equilateral).
 *
 * Section map:  §1 config  §2 clock  §3 color  §4 formula  §5 pool
 *               §6 cursor  §7 scene  §8 screen  §9 app
 *
 * Keys:  arrows:move  spc:toggle  g:glyph  C:clear  +/-:size
 *        t:theme  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/02_right_isosceles_direct.c \
 *       -o 02_right_isosceles_direct -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Cursor in (col, row, up) where up ∈ {LL=0, UR=1}.
 *                  Pixel→lattice axis-aligned: a=px/size, b=py/size.
 *                  Diagonal split: up = (fa ≥ fb) ? UR : LL.
 *                  Centroids:
 *                    UR (col+2/3, row+1/3),  LL (col+1/3, row+2/3).
 *
 * References     :
 *   Half-rect tiling — https://en.wikipedia.org/wiki/Triangular_tiling
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 16.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

#define BORDER_W   0.10
#define MAX_OBJ    256
#define N_GLYPHS   6
#define N_THEMES   4

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_OBJECT 3
#define PAIR_HUD    4
#define PAIR_HINT   5

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
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEME_FG[N_THEMES][2] = {
    {  75, 226 }, { 207, 226 }, {  82, 207 }, {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_YELLOW  },
    { COLOR_MAGENTA, COLOR_YELLOW  },
    { COLOR_GREEN,   COLOR_MAGENTA },
    { COLOR_WHITE,   COLOR_CYAN    },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e = (COLORS >= 256) ? THEME_FG[theme][0] : THEME_FG_8[theme][0];
    short fg_o = (COLORS >= 256) ? THEME_FG[theme][1] : THEME_FG_8[theme][1];
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — half-rect (axis-aligned, single diagonal)                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *up,
                         double *fa, double *fb)
{
    double inv = 1.0 / size;
    double a   = px * inv;
    double b   = py * inv;
    int    c   = (int)floor(a);
    int    r   = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c;
    *fb = b - (double)r;
    *up = (*fa >= *fb) ? 1 : 0;     /* 1 = UR, 0 = LL */
}

static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double a = (up == 1) ? ((double)col + 2.0/3.0) : ((double)col + 1.0/3.0);
    double b = (up == 1) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = a * size;
    *cy_pix = b * size;
}

static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char ch1, ch2, ch3;
    if (up == 1) {                /* UR */
        l1 = 1.0 - fa;       ch1 = '|';
        l2 = fa - fb;        ch2 = '\\';
        l3 = fb;             ch3 = '_';
    } else {                       /* LL */
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa;             ch2 = '|';
        l3 = fb - fa;        ch3 = '\\';
    }
    char ch = ch1; double m = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

static void tri_to_screen(int col, int row, int up, double size,
                          int ox, int oy, int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, size, &cx_pix, &cy_pix);
    *scol = ox + (int)(cx_pix / CELL_W);
    *srow = oy + (int)(cy_pix / CELL_H);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int col, row, up; char glyph; } TObj;
typedef struct { TObj objs[MAX_OBJ]; int count; } ObjectPool;

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
    /* UP    */ { {  0,  0,  1 }, {  0, -1,  0 } },
    /* DOWN  */ { {  0, +1,  1 }, {  0,  0,  0 } },
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
            double fa, fb, m;
            pixel_to_tri(px, py, size, &tC, &tR, &tU, &fa, &fb);
            char ch = tri_edge_char(tU, fa, fb, &m);
            if (m >= BORDER_W) continue;
            attron(COLOR_PAIR(PAIR_BORDER));
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(PAIR_BORDER));
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
             cur->col, cur->row, cur->up ? "UR" : "LL",
             pool->count, GLYPHS[cur->glyph_idx], cur->tri_size,
             cur->theme, fps, cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  spc:toggle  g:glyph  C:clear  +/-:size  t:theme  r:reset  q:quit  [02 right-isosceles direct] ");
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
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
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
