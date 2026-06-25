/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_hex_subdivision_path.c — draw the straight line between two wedges
 * on a hex grid where each hex is cut into 6 pie slices.
 *
 * Pick a start slice (s) and an end slice (e); the program traces the
 * straight line between them and lights up every slice it passes through.
 *
 * Sister files: grids/tri_grids/06_hex_subdivision.c (draws the grid),
 *               06_hex_subdivision_direct.c (place slices by hand),
 *               01_equilateral_path.c (same idea on a triangle grid).
 * Hex coordinate math: https://www.redblobgames.com/grids/hexagons/
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

#define HEX_SIZE_DEFAULT 16.0
#define HEX_SIZE_MIN      8.0
#define HEX_SIZE_MAX     40.0
#define HEX_SIZE_STEP     2.0

#define BORDER_W      0.10
#define RADIUS_T_FRAC 0.12

#define MAX_OBJ   1024
#define N_THEMES  4

#define FPS_EWMA_ALPHA  0.05

#define PAIR_BORDER 1
#define PAIR_RADIUS 2
#define PAIR_CURSOR 3
#define PAIR_START  4
#define PAIR_END    5
#define PAIR_PATH   6
#define PAIR_HUD    7
#define PAIR_HINT   8

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

static const short THEME_FG[N_THEMES][4] = {
    /* colors in order: hex outline, the 3 inner diagonals, start marker, end marker */
    {  75,  39,  82, 196 },
    {  82, 226, 226, 207 },
    { 207, 196,  82,  39 },
    {  87, 226,  82, 196 },
};
static const short THEME_FG_8[N_THEMES][4] = {
    { COLOR_CYAN,    COLOR_BLUE,   COLOR_GREEN,  COLOR_RED     },
    { COLOR_GREEN,   COLOR_YELLOW, COLOR_YELLOW, COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_RED,    COLOR_GREEN,  COLOR_BLUE    },
    { COLOR_WHITE,   COLOR_CYAN,   COLOR_GREEN,  COLOR_RED     },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e, fg_r, fg_s, fg_n;
    if (COLORS >= 256) {
        fg_e = THEME_FG[theme][0]; fg_r = THEME_FG[theme][1];
        fg_s = THEME_FG[theme][2]; fg_n = THEME_FG[theme][3];
    } else {
        fg_e = THEME_FG_8[theme][0]; fg_r = THEME_FG_8[theme][1];
        fg_s = THEME_FG_8[theme][2]; fg_n = THEME_FG_8[theme][3];
    }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_RADIUS, fg_r, -1);
    init_pair(PAIR_START,  fg_s, -1);
    init_pair(PAIR_END,    fg_n, -1);
    init_pair(PAIR_PATH,   COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 gridctx ── */

/*
 * Everything we need to turn hex coordinates into screen positions and back.
 * Built once at startup and rebuilt on resize or when the hexes change size.
 */
typedef struct {
    int    rows, cols;            /* terminal size, in characters */
    int    cw, ch;               /* width and height of one character cell, in sub-pixels */
    double hex_size;             /* radius of a hex, in pixels — bigger means fewer, larger hexes */
    int    ox, oy;               /* screen cell that hex (0,0) sits on */
    int    max_q, max_r;         /* how far the cursor may roam from the centre */
    /* border_w: how thick the hex outline is, as a fraction of a hex.
     * radius_t_frac: how close a point must be to a diagonal to count as on it. */
    double border_w, radius_t_frac;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, double hex_size)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->hex_size = hex_size;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->max_q = cols / 2;
    g->max_r = rows / 2;
    g->border_w      = BORDER_W;
    g->radius_t_frac = RADIUS_T_FRAC;
}

static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if (t < M_PI / 8.0)         return '-';
    if (t < 3.0 * M_PI / 8.0)   return '\\';
    if (t < 5.0 * M_PI / 8.0)   return '|';
    if (t < 7.0 * M_PI / 8.0)   return '/';
    return '-';
}

static int sector_of(double dx, double dy)
{
    double ang = atan2(dy, dx);
    int s = (int)floor((ang + M_PI / 6.0) / (M_PI / 3.0));
    s %= 6; if (s < 0) s += 6;
    return s;
}

static void pixel_to_hex(double px, double py, double size,
                         int *Q, int *R, double *dist)
{
    double sq3 = sqrt(3.0), sq3_3 = sq3 / 3.0;
    double fq = (2.0 / 3.0 * px) / size;
    double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
    double fs = -fq - fr;
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) rq = -rr - rs;
    else if (dr > ds)             rr = -rq - rs;
    *Q = rq; *R = rr;
    double fQ = (double)*Q, fR = (double)*R, fS = (double)(-*Q - *R);
    double d = fabs(fq - fQ);
    double d2 = fabs(fr - fR);
    double d3 = fabs(fs - fS);
    if (d2 > d) d = d2;
    if (d3 > d) d = d3;
    *dist = d;
}

static void hex_centre_pixel(int Q, int R, double size,
                              double *cx, double *cy)
{
    double sq3 = sqrt(3.0);
    *cx = size * 1.5      * (double)Q;
    *cy = size * (sq3*0.5 * (double)Q + sq3 * (double)R);
}

static void wedge_centroid_pixel(int Q, int R, int sector, double size,
                                 double *cx_pix, double *cy_pix)
{
    double cx, cy;
    hex_centre_pixel(Q, R, size, &cx, &cy);
    double ang = (double)sector * M_PI / 3.0;
    double r   = size * sqrt(3.0) / 3.0;
    *cx_pix = cx + r * cos(ang);
    *cy_pix = cy + r * sin(ang);
}

static void ctx_to_screen(const GridCtx *g, int q, int r, int sector,
                          int *scol, int *srow)
{
    double cx, cy;
    wedge_centroid_pixel(q, r, sector, g->hex_size, &cx, &cy);
    *scol = g->ox + (int)(cx / g->cw);
    *srow = g->oy + (int)(cy / g->ch);
}

static void ctx_draw_bg(const GridCtx *g)
{
    double sq3 = sqrt(3.0), sq3_2 = sq3 * 0.5;
    double limit_inner = 0.5 - g->border_w;
    double radius_t    = g->hex_size * g->radius_t_frac * 0.5;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;
            int Q, R; double dist;
            pixel_to_hex(px, py, g->hex_size, &Q, &R, &dist);
            double cx, cy;
            hex_centre_pixel(Q, R, g->hex_size, &cx, &cy);
            double dxp = px - cx, dyp = py - cy;
            if (dist >= limit_inner) {
                double theta = atan2(dyp, dxp);
                char ch = angle_char(theta + M_PI / 2.0);
                attron(COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                continue;
            }
            double r0 = fabs(dyp);
            double r1 = fabs(0.5 * dyp - sq3_2 * dxp);
            double r2 = fabs(0.5 * dyp + sq3_2 * dxp);
            char rch = '-'; double rmin = r0;
            if (r1 < rmin) { rmin = r1; rch = '/';  }
            if (r2 < rmin) { rmin = r2; rch = '\\'; }
            if (rmin < radius_t) {
                attron(COLOR_PAIR(PAIR_RADIUS));
                mvaddch(row, col, (chtype)(unsigned char)rch);
                attroff(COLOR_PAIR(PAIR_RADIUS));
            }
        }
    }
}

/* ── §5 pool ── */

/* One wedge on the path: which hex (q,r) and which of its 6 slices (sector). */
typedef struct {
    int  q, r, sector;
    char glyph;                  /* character drawn for it, '*' here */
    bool alive;                  /* false if this slot was cleared */
} Obj;

/* The whole path as a plain list of wedges, filled fresh each time it's recomputed. */
typedef struct {
    Obj items[MAX_OBJ];
    int count;
} Pool;

static void pool_clear(Pool *p) { p->count = 0; }

static int pool_find(const Pool *p, int q, int r, int sector)
{
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].alive &&
            p->items[i].q == q && p->items[i].r == r &&
            p->items[i].sector == sector)
            return i;
    }
    return -1;
}

static void pool_place(Pool *p, int q, int r, int sector, char glyph)
{
    if (p->count >= MAX_OBJ || pool_find(p, q, r, sector) >= 0) return;
    p->items[p->count++] = (Obj){ q, r, sector, glyph, true };
}

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].q, p->items[i].r, p->items[i].sector,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

/* ── §6 cursor ── */

/*
 * The user's current selection plus the two chosen endpoints.
 * The cursor (@) is where they're pointing now; sQ/sR/sSec and eQ/eR/eSec
 * are the start and end wedges they've locked in with 's' and 'e'.
 */
typedef struct {
    int q, r, sector;            /* wedge the cursor is on right now */
    int sQ, sR, sSec;            /* start wedge, valid only when has_start */
    int eQ, eR, eSec;            /* end wedge, valid only when has_end */
    int has_start, has_end;      /* whether each endpoint has been set */
    int theme, paused;           /* current color theme; paused is unused here */
} Cursor;

/* Arrow keys map to these whole-hex steps. */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0; cur->r = 0; cur->sector = 0;
    cur->has_start = 0; cur->has_end = 0;
    cur->theme = 0; cur->paused = 0;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int idx)
{
    int nq = cur->q + HEX_DIR[idx][0];
    int nr = cur->r + HEX_DIR[idx][1];
    if (nq < -g->max_q || nq > g->max_q) return;
    if (nr < -g->max_r || nr > g->max_r) return;
    cur->q = nq; cur->r = nr;
}

static void cursor_rotate_sector(Cursor *cur, int delta)
{
    cur->sector = (cur->sector + delta + 6) % 6;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sc, sr;
    ctx_to_screen(g, cur->q, cur->r, cur->sector, &sc, &sr);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §7 mode — path state machine + line walk ── */

/*
 * Walk the straight line from the start wedge to the end wedge in small steps,
 * and at each step note which wedge that point lands in. Re-run whenever the
 * endpoints or the hex size change.
 */
static void path_compute(Pool *pool, const Cursor *cur, const GridCtx *g)
{
    pool_clear(pool);
    if (!cur->has_start || !cur->has_end) return;
    double sx, sy, ex, ey;
    wedge_centroid_pixel(cur->sQ, cur->sR, cur->sSec, g->hex_size, &sx, &sy);
    wedge_centroid_pixel(cur->eQ, cur->eR, cur->eSec, g->hex_size, &ex, &ey);
    double dx = ex - sx, dy = ey - sy;
    double dist = sqrt(dx*dx + dy*dy);
    if (dist < 1e-6) {
        pool_place(pool, cur->sQ, cur->sR, cur->sSec, '*');
        return;
    }
    double step = g->hex_size * 0.25;
    int n = (int)(dist / step) + 1;
    for (int i = 0; i <= n; i++) {
        double t = (double)i / (double)n;
        double px = sx + t * dx, py = sy + t * dy;
        int Q, R; double dd;
        pixel_to_hex(px, py, g->hex_size, &Q, &R, &dd);
        double cx, cy; hex_centre_pixel(Q, R, g->hex_size, &cx, &cy);
        int s = sector_of(px - cx, py - cy);
        pool_place(pool, Q, R, s, '*');
    }
}

static void marker_draw(const GridCtx *g, int q, int r, int s,
                        char glyph, int pair)
{
    int sc, sr;
    ctx_to_screen(g, q, r, s, &sc, &sr);
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
             " Q:%+d R:%+d sec:%d  path:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, cur->sector,
             p->count, g->hex_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:hex  ,/.: sector  s:set-start  e:set-end  spc:clear  +/-:size  t:theme  q:quit  [06 path] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(p, g);
    if (cur->has_start)
        marker_draw(g, cur->sQ, cur->sR, cur->sSec, 'S', PAIR_START);
    if (cur->has_end)
        marker_draw(g, cur->eQ, cur->eR, cur->eSec, 'E', PAIR_END);
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

    cur.q = 0; cur.r = 0; cur.sector = 0;
    cur.has_start = 0; cur.has_end = 0; cur.theme = 0; cur.paused = 0;
    screen_init(cur.theme);
    ctx_init(&g, LINES, COLS, HEX_SIZE_DEFAULT);
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS;
    int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            ctx_init(&g, LINES, COLS, g.hex_size);
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
                case ' ':
                    cur.has_start = 0; cur.has_end = 0; pool_clear(&path);
                    break;
                case 's':
                    cur.sQ = cur.q; cur.sR = cur.r; cur.sSec = cur.sector;
                    cur.has_start = 1;
                    path_compute(&path, &cur, &g);
                    break;
                case 'e':
                    cur.eQ = cur.q; cur.eR = cur.r; cur.eSec = cur.sector;
                    cur.has_end = 1;
                    path_compute(&path, &cur, &g);
                    break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_UP:    cursor_move(&cur, &g, 0); break;
                case KEY_DOWN:  cursor_move(&cur, &g, 1); break;
                case KEY_LEFT:  cursor_move(&cur, &g, 2); break;
                case KEY_RIGHT: cursor_move(&cur, &g, 3); break;
                case ',': case '<': cursor_rotate_sector(&cur, -1); break;
                case '.': case '>': cursor_rotate_sector(&cur, +1); break;
                case '+': case '=':
                    if (g.hex_size < HEX_SIZE_MAX) {
                        g.hex_size += HEX_SIZE_STEP;
                        path_compute(&path, &cur, &g);
                    } break;
                case '-':
                    if (g.hex_size > HEX_SIZE_MIN) {
                        g.hex_size -= HEX_SIZE_STEP;
                        path_compute(&path, &cur, &g);
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
