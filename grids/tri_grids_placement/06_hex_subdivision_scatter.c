/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_hex_subdivision_scatter.c — distance-coloured scatter on hex-subdivision
 *
 * DEMO: A random scatter of N wedges fills a region around the cursor on
 *       a flat-top hex grid where each hex is split into 6 wedges by 3
 *       long diagonals. Each wedge is coloured on a 6-stop gradient by
 *       hex-axial distance from the cursor — closer = warm, farther =
 *       cool. Move '@' with arrows; ',' / '.' rotate the cursor sector.
 *       SPACE reseeds; +/- changes density.
 *
 * Study alongside: grids/tri_grids/06_hex_subdivision.c (rasterizer),
 *                  06_hex_subdivision_direct.c (manual placement),
 *                  01_equilateral_scatter.c (same idea on tri).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, HEX_SIZE, SCATTER_RADIUS, DENSITY
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6-bucket gradient palette + cursor / HUD / hint
 *   §4 gridctx  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg
 *   §5 pool     — Pool: place / find / draw  (scatter entries)
 *   §6 cursor   — Cursor + cursor_reset / cursor_move / cursor_rotate
 *   §7 mode     — scatter_seed (LCG random spawn) + bucket colouring
 *   §8 scene    — hud_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move-hex  ,/.:rotate  spc:reseed  +/-:density
 *        r:reset  t:theme  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/06_hex_subdivision_scatter.c \
 *       -o 06_hex_subdivision_scatter -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Random scatter on the hex-subdivision grid. Pick N
 *                  random (ΔQ, ΔR, sector) within ±SCATTER_RADIUS of the
 *                  cursor; colour by hex-axial distance bucketed into 6
 *                  gradient slots.
 *
 * Data-structure : Pool — flat array of Obj{q, r, sector, glyph='*',
 *                  alive}. Bucket assignment is recomputed every frame
 *                  from cursor's current position; entries change only
 *                  on reseed.
 *
 * Distance metric: Hex axial (cube) distance for the (q, r) part; plus a
 *                  small sector-mismatch penalty if Δsector ≠ 0. This is
 *                  the proper hex graph distance — exact, not a Manhattan
 *                  proxy.
 *
 * Re-seeding     : SPACE re-randomises with a new seed (xor'd by clock).
 *                  +/- density also reseeds. Cursor movement does NOT.
 *
 * References     :
 *   Hexagonal coordinates — https://www.redblobgames.com/grids/hexagons/
 *   Hex distance — Red Blob Games "hex grid distance"
 *   Linear congruential generator — Numerical Recipes ch. 7
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two halves: STORAGE (random scatter of (q, r, sector) addresses,
 * generated once per reseed) and COLOURING (a pure function of cursor
 * distance, recomputed every frame). Moving the cursor never reseeds; it
 * only repaints the same scatter through a different distance lens.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Sprinkle salt on a soccer-ball-pattern grid with diameter cuts —
 * grains land in random wedges within a disc around the cursor's hex.
 * Now point a coloured spotlight at the cloth: grains close to the beam
 * glow warm, farther ones cool. Move the spotlight: same grains,
 * different colours.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. ctx_draw_bg — hex border via angle_char + 3-radii proximity
 *     overlay.
 *  3. For each scatter entry:
 *       d = hex_dist(obj.q - cur.q, obj.r - cur.r)
 *           + (obj.sector != cur.sector ? 1 : 0)
 *       bucket = min(d, N_BUCKETS - 1)
 *       attron(COLOR_PAIR(PAIR_BUCKET_0 + bucket))
 *       mvaddch(wedge_centroid_screen, '*')
 *  4. cursor_draw — '@' on top.
 *
 *  Reseed (only on SPACE or +/- density):
 *    pool->count = 0
 *    g_seed ^= clock_ns()
 *    for i in 0..density:
 *      dQ = floor(frand·(2·R+1)) - R    ; dR = same
 *      s  = (int)floor(frand · 6)       // 0..5
 *      pool_place(cur.q+dQ, cur.r+dR, s)   // dedup
 *
 * KEY FORMULAS
 * ────────────
 *  Hex axial (cube) distance:
 *    d_hex = max(|ΔQ|, |ΔR|, |ΔQ + ΔR|)
 *  This is exact for the hex graph (NOT a Manhattan approximation).
 *
 *  Wedge centroid (used by pool_draw):
 *    angle = sector · π/3
 *    radius = size · √3 / 3
 *    cx_w   = cx + radius · cos(angle)
 *    cy_w   = cy + radius · sin(angle)
 *
 *  LCG step:
 *    g_seed = g_seed · 1103515245 + 12345
 *    frand  = ((g_seed >> 16) & 0x7FFF) / 32767.0
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Hex distance vs sector: the sector mismatch only adds 1 unit; two
 *    wedges of the same hex can be visually quite distant but share
 *    d_hex = 0. The "+1 if Δsector" term keeps neighbours inside one
 *    hex from all collapsing to the same colour bucket.
 *  • SCATTER_RADIUS is in HEX cells, not pixels.
 *  • Density vs MAX_OBJ: dedup and high density may give fewer dots
 *    than requested.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Place cursor at the centre of a fresh scatter — colours warmest near
 *  '@'. Walk the cursor a few hexes outward: same dots persist; the
 *  warm/cool boundary follows the cursor. Press ',' to rotate cursor
 *  sector — the bucket of any dot in the cursor's hex shifts by 0 or 1
 *  depending on whether its sector now matches.
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

#define HEX_SIZE_DEFAULT 16.0
#define HEX_SIZE_MIN      8.0
#define HEX_SIZE_MAX     40.0
#define HEX_SIZE_STEP     2.0

#define BORDER_W      0.10
#define RADIUS_T_FRAC 0.12

#define MAX_OBJ        1024
#define SCATTER_RADIUS    8
#define DENSITY_DEFAULT  80
#define DENSITY_MIN      20
#define DENSITY_MAX     400
#define DENSITY_STEP     20
#define N_BUCKETS  6
#define N_THEMES   3

#define FPS_EWMA_ALPHA  0.05

#define PAIR_BORDER 1
#define PAIR_RADIUS 2
#define PAIR_CURSOR 3
#define PAIR_BUCK0  4   /* 4..9  (6 buckets) */
#define PAIR_HUD    10
#define PAIR_HINT   11

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEME_GRAD[N_THEMES][N_BUCKETS] = {
    { 196, 202, 214, 226,  82,  39 },
    {  39,  82, 226, 214, 202, 196 },
    { 250, 247, 244, 250, 247, 244 },
};
static const short THEME_GRAD_8[N_THEMES][N_BUCKETS] = {
    { COLOR_RED,   COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE },
    { COLOR_BLUE,  COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,   COLOR_RED  },
    { COLOR_WHITE, COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    for (int i = 0; i < N_BUCKETS; i++) {
        short fg = (COLORS >= 256) ? THEME_GRAD[theme][i] : THEME_GRAD_8[theme][i];
        init_pair(PAIR_BUCK0 + i, fg, -1);
    }
    init_pair(PAIR_BORDER, COLORS >= 256 ?  75 : COLOR_CYAN,  -1);
    init_pair(PAIR_RADIUS, COLORS >= 256 ?  39 : COLOR_BLUE,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    rows, cols;
    int    cw, ch;
    double hex_size;
    int    ox, oy;
    int    max_q, max_r;
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

    attron(COLOR_PAIR(PAIR_BORDER));
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
                mvaddch(row, col, (chtype)(unsigned char)ch);
                continue;
            }
            double r0 = fabs(dyp);
            double r1 = fabs(0.5 * dyp - sq3_2 * dxp);
            double r2 = fabs(0.5 * dyp + sq3_2 * dxp);
            char rch = '-'; double rmin = r0;
            if (r1 < rmin) { rmin = r1; rch = '/';  }
            if (r2 < rmin) { rmin = r2; rch = '\\'; }
            if (rmin < radius_t) mvaddch(row, col, (chtype)(unsigned char)rch);
        }
    }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int  q, r, sector;
    char glyph;
    bool alive;
} Obj;

typedef struct {
    Obj items[MAX_OBJ];
    int count;
} Pool;

static void pool_clear(Pool *p) { p->count = 0; }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int q, r, sector;
    int density;
    int theme, paused;
} Cursor;

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
    cur->density = DENSITY_DEFAULT;
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  mode — scatter (LCG spawn + bucket colouring)                       */
/* ═══════════════════════════════════════════════════════════════════════ */

static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

/* Cube distance between two hexes (axial → cube via s = -q - r). */
static int hex_distance(int aQ, int aR, int bQ, int bR)
{
    int aS = -aQ - aR, bS = -bQ - bR;
    int dq = abs(aQ - bQ);
    int dr = abs(aR - bR);
    int ds = abs(aS - bS);
    int m = (dq > dr) ? dq : dr;
    if (ds > m) m = ds;
    return m;
}

static int wedge_distance(int aQ, int aR, int aS, int bQ, int bR, int bS)
{
    int hd = hex_distance(aQ, aR, bQ, bR);
    int sd = abs(aS - bS); if (sd > 3) sd = 6 - sd;
    return hd + sd;
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
    g_seed ^= (unsigned int)clock_ns();
    int max = (cur->density < MAX_OBJ) ? cur->density : MAX_OBJ;
    int tries = 0;
    while (pool->count < max && tries < max * 4) {
        int dQ = (int)(frand() * (2 * SCATTER_RADIUS + 1)) - SCATTER_RADIUS;
        int dR = (int)(frand() * (2 * SCATTER_RADIUS + 1)) - SCATTER_RADIUS;
        int s  = (int)(frand() * 6);
        pool->items[pool->count++] = (Obj){ cur->q + dQ, cur->r + dR, s, '*', true };
        tries++;
    }
}

static void scatter_draw(const Pool *p, const Cursor *cur, const GridCtx *g)
{
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].q, p->items[i].r, p->items[i].sector,
                      &sc, &sr);
        if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) continue;
        int dist = wedge_distance(p->items[i].q, p->items[i].r, p->items[i].sector,
                                  cur->q, cur->r, cur->sector);
        int b = distance_bucket(dist, SCATTER_RADIUS * 2);
        attron(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
        mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
        attroff(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d sec:%d  N:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, cur->sector,
             p->count, g->hex_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:hex  ,/.:sector  spc:reseed  +/-:density  t:theme  r:reset  q:quit  [06 scatter] ");
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

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

    cur.q = 0; cur.r = 0; cur.sector = 0;
    cur.density = DENSITY_DEFAULT;
    cur.theme = 0; cur.paused = 0;
    g_seed = (unsigned int)clock_ns();
    screen_init(cur.theme);
    ctx_init(&g, LINES, COLS, HEX_SIZE_DEFAULT);
    cursor_reset(&cur, &g);
    scatter_seed(&pool, &cur);

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
                    cursor_reset(&cur, &g); scatter_seed(&pool, &cur);
                    color_init(cur.theme);
                    break;
                case ' ': scatter_seed(&pool, &cur); break;
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
