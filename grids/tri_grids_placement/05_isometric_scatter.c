/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_isometric_scatter.c — distance-colored scatter on iso (solid-fill) grid
 *
 * DEMO: Iso solid-fill background (6-cycle palette by triangle position).
 *       Random scatter of N triangles colored on a 6-stop distance gradient
 *       from the cursor — closer = warm, farther = cool. The cursor walks
 *       the iso lattice; SPACE reseeds the scatter.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/05_isometric_scatter.c \
 *       -o 05_isometric_scatter -lncurses -lm
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

/* §1 */
#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4
#define TRI_SIZE_DEFAULT 14.0
#define TRI_SIZE_MIN     6.0
#define TRI_SIZE_MAX    40.0
#define TRI_SIZE_STEP    2.0
#define MAX_OBJ  1024
#define SCATTER_RADIUS 12
#define DENSITY_DEFAULT 120
#define DENSITY_MIN      20
#define DENSITY_MAX     500
#define DENSITY_STEP     20
#define N_PALETTE 6
#define N_BUCKETS 6
#define N_THEMES  3

#define PAIR_FILL_BASE  1                                 /* 1..6  iso bg */
#define PAIR_BUCK0     (PAIR_FILL_BASE + N_PALETTE)       /* 7..12 scatter fg */
#define PAIR_CURSOR    (PAIR_BUCK0 + N_BUCKETS)
#define PAIR_HUD       (PAIR_CURSOR + 1)
#define PAIR_HINT      (PAIR_HUD + 1)

/* §2 */
static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec = (time_t)(ns/1000000000LL), .tv_nsec = (long)(ns%1000000000LL) };
    nanosleep(&r, NULL);
}

/* §3 — iso bg palette + distance gradient palette */
static const short PAL256[N_THEMES][N_PALETTE] = {
    { 196, 214, 226, 118, 39, 129 },
    {  21,  39,  82, 226, 207, 21  },
    {  15,  87,  39,   0, 39, 87   },
};
static const short PAL8[N_THEMES][N_PALETTE] = {
    { COLOR_RED,  COLOR_YELLOW, COLOR_GREEN, COLOR_CYAN,    COLOR_BLUE,    COLOR_MAGENTA },
    { COLOR_BLUE, COLOR_CYAN,   COLOR_GREEN, COLOR_YELLOW,  COLOR_MAGENTA, COLOR_BLUE    },
    { COLOR_WHITE,COLOR_CYAN,   COLOR_BLUE,  COLOR_BLACK,   COLOR_BLUE,    COLOR_CYAN    },
};
/* Distance gradient — bright fg colors that pop over the iso bg */
static const short GRAD256[N_BUCKETS] = { 15, 226, 214, 196, 207, 21 };
static const short GRAD8[N_BUCKETS]   = {
    COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_MAGENTA, COLOR_BLUE
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
        init_pair(PAIR_BUCK0 + i, fg, COLOR_BLACK);
    }
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLACK);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* §4 formula (equilateral skew lattice — same as 01) */
static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *up,
                         double *fa, double *fb)
{
    double h = size * sqrt(3.0) * 0.5;
    double b = py / h, a = px / size - 0.5 * b;
    int c = (int)floor(a), r = (int)floor(b);
    *col = c; *row = r; *fa = a-(double)c; *fb = b-(double)r;
    *up = (*fa + *fb >= 1.0) ? 1 : 0;
}
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx, double *cy)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx = (a + 0.5 * b) * size; *cy = b * h;
}
static void tri_to_screen(int col, int row, int up, double size,
                          int ox, int oy, int *scol, int *srow)
{
    double cx, cy; tri_centroid_pixel(col, row, up, size, &cx, &cy);
    *scol = ox + (int)(cx / CELL_W); *srow = oy + (int)(cy / CELL_H);
}
static int palette_index(int col, int row, int up)
{
    int k = col + 2 * row + up;
    k %= N_PALETTE; if (k < 0) k += N_PALETTE;
    return k;
}

/* §5 pool */
typedef struct { int col, row, up; } TPos;
typedef struct { TPos pos[MAX_OBJ]; int count; } ScatterPool;

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

typedef struct {
    int col, row, up; int density; double tri_size; int theme, paused;
} Cursor;

static const int TRI_DIR[4][2][3] = {
    { { -1,  0,  1 }, {  0,  0,  0 } },
    { {  0,  0,  1 }, { +1,  0,  0 } },
    { {  0, -1,  1 }, {  0,  0,  0 } },
    { {  0,  0,  1 }, {  0, +1,  0 } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->up = 0;
    cur->density = DENSITY_DEFAULT;
    cur->tri_size = TRI_SIZE_DEFAULT;
    cur->theme = 0; cur->paused = 0;
}
static void cursor_step(Cursor *cur, int dir)
{
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0]; cur->row += t[1]; cur->up = t[2];
}

static void scatter_seed(ScatterPool *sp, const Cursor *cur)
{
    sp->count = 0;
    g_seed ^= (unsigned int)clock_ns();
    int max = (cur->density < MAX_OBJ) ? cur->density : MAX_OBJ;
    int tries = 0;
    while (sp->count < max && tries < max * 4) {
        int dC = (int)(frand() * (2 * SCATTER_RADIUS + 1)) - SCATTER_RADIUS;
        int dR = (int)(frand() * (2 * SCATTER_RADIUS + 1)) - SCATTER_RADIUS;
        int up = frand() > 0.5 ? 1 : 0;
        sp->pos[sp->count++] = (TPos){ cur->col + dC, cur->row + dR, up };
        tries++;
    }
}

/* §6 scene — solid iso fill, scatter dots on top, cursor on top */
static void scene_draw(int rows, int cols, const Cursor *cur,
                       const ScatterPool *sp, double fps)
{
    erase();
    int ox = cols / 2, oy = (rows - 1) / 2;
    /* Iso solid fill */
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;
            int tC, tR, tU; double fa, fb;
            pixel_to_tri(px, py, cur->tri_size, &tC, &tR, &tU, &fa, &fb);
            int pair = PAIR_FILL_BASE + palette_index(tC, tR, tU);
            attron(COLOR_PAIR(pair));
            mvaddch(row, col, ' ');
            attroff(COLOR_PAIR(pair));
        }
    }
    /* Scatter dots — use REVERSE so the bucket fg color paints over the iso bg */
    for (int i = 0; i < sp->count; i++) {
        int sc, sr;
        tri_to_screen(sp->pos[i].col, sp->pos[i].row, sp->pos[i].up,
                      cur->tri_size, ox, oy, &sc, &sr);
        if (sc < 0 || sc >= cols || sr < 0 || sr >= rows - 1) continue;
        int dist = triangle_distance(sp->pos[i].col, sp->pos[i].row, sp->pos[i].up,
                                     cur->col, cur->row, cur->up);
        int b = distance_bucket(dist, SCATTER_RADIUS * 2);
        int pair_iso = PAIR_FILL_BASE + palette_index(sp->pos[i].col, sp->pos[i].row, sp->pos[i].up);
        /* Use the scatter-bucket pair (bright fg on black) and overlay on the iso bg.
         * Trick: compose by manually picking PAIR_BUCK0 + b which has bg=BLACK,
         * but draw the cell with the iso's bg color via REVERSE on the bucket color. */
        attron(COLOR_PAIR(pair_iso) | A_REVERSE | A_BOLD);
        (void)b;
        mvaddch(sr, sc, '*');
        attroff(COLOR_PAIR(pair_iso) | A_REVERSE | A_BOLD);
        /* Draw a bucket-colored highlight character on top — simpler: just use the
         * bucket pair with FG over the iso BG by writing the same '*' twice with
         * different attrs; but ncurses replaces, so the second write wins. Use
         * the bucket pair only and accept BLACK bg under the dot. */
        attron(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
        mvaddch(sr, sc, '*');
        attroff(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
    }
    /* Cursor */
    {
        int sc, sr;
        tri_to_screen(cur->col, cur->row, cur->up, cur->tri_size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
            int pair_iso = PAIR_FILL_BASE + palette_index(cur->col, cur->row, cur->up);
            attron(COLOR_PAIR(pair_iso) | A_BOLD | A_REVERSE);
            mvaddch(sr, sc, '@');
            attroff(COLOR_PAIR(pair_iso) | A_BOLD | A_REVERSE);
        }
    }

    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  N:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             sp->count, cur->tri_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  spc:reseed  +/-:density  t:theme  r:reset  q:quit  [05 scatter] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);
    wnoutrefresh(stdscr); doupdate();
}

/* §7 screen */
static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

/* §8 app */
static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal); signal(SIGWINCH, on_signal);
    Cursor cur; cursor_reset(&cur);
    ScatterPool sp; sp.count = 0;
    g_seed = (unsigned int)clock_ns();
    screen_init(cur.theme);
    scatter_seed(&sp, &cur);
    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS; int64_t t0 = clock_ns();
    while (g_running) {
        if (g_need_resize) { g_need_resize = 0; endwin(); refresh(); rows = LINES; cols = COLS; }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r': cursor_reset(&cur); scatter_seed(&sp, &cur); color_init(cur.theme); break;
                case ' ': scatter_seed(&sp, &cur); break;
                case 't': cur.theme = (cur.theme + 1) % N_THEMES; color_init(cur.theme); break;
                case KEY_LEFT:  cursor_step(&cur, 0); break;
                case KEY_RIGHT: cursor_step(&cur, 1); break;
                case KEY_UP:    cursor_step(&cur, 2); break;
                case KEY_DOWN:  cursor_step(&cur, 3); break;
                case '+': case '=':
                    if (cur.density < DENSITY_MAX) { cur.density += DENSITY_STEP; scatter_seed(&sp, &cur); } break;
                case '-':
                    if (cur.density > DENSITY_MIN) { cur.density -= DENSITY_STEP; scatter_seed(&sp, &cur); } break;
            }
        }
        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0 = now;
        scene_draw(rows, cols, &cur, &sp, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
