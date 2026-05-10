/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_hex_subdivision_direct.c — direct placement on hex-with-radii grid
 *
 * DEMO: Flat-top hex grid where each hex is split into 6 equilateral
 *       wedges by 3 long diagonals through the centre. The cursor lives
 *       at (Q, R, sector) where (Q, R) are axial hex coords and sector
 *       ∈ 0..5 (CCW from +x). Arrow keys move the hex; ',' / '.' rotate
 *       the cursor sub-triangle within the hex. SPACE toggles a glyph
 *       at the cursor wedge.
 *
 * Study alongside: grids/tri_grids/06_hex_subdivision.c (rasterizer +
 *                  hex inverse via cube-rounding),
 *                  grids/hex_grids/01_flat_top.c (base hex grid).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, HEX_SIZE, BORDER_W, RADIUS_T_FRAC
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6 pairs: edge / radius / cursor / object / HUD / hint
 *   §4 gridctx  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg
 *   §5 pool     — Pool: place / remove / toggle / find / clear / draw
 *   §6 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw
 *   §7 mode     — direct: SPACE toggles via pool_toggle
 *   §8 scene    — hud_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move-hex  ,/.:rotate-sector  spc:toggle  g:glyph
 *        C:clear  +/-:size  t:theme  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/06_hex_subdivision_direct.c \
 *       -o 06_hex_subdivision_direct -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Direct placement on a flat-top hex grid where each
 *                  hex is sliced into 6 equilateral wedges by three
 *                  diameters. The cursor address is (q, r, sector). q
 *                  and r are axial hex coordinates; sector is rotated by
 *                  ',' (CCW) and '.' (CW). SPACE toggles an object at
 *                  that address.
 *
 * Data-structure : Pool — flat array of Obj{q, r, sector, glyph, alive}.
 *                  pool_toggle removes via swap-with-last (O(1)).
 *
 * Pixel→hex      : Cube-rounding (round (q, r, s) with s = -q - r, then
 *                  snap the largest absolute residual back to its
 *                  neighbours). See grids/tri_grids/06_hex_subdivision.c.
 *
 * Sector address : The CURSOR carries the sector — the user rotates it
 *                  with ',' / '.'. pixel_to_hex does NOT classify the
 *                  sector here (only the direct-placement variant). Wedge
 *                  centroids sit at sector·60° from the hex centre.
 *
 * References     :
 *   Hexagonal coordinates — https://www.redblobgames.com/grids/hexagons/
 *   Hex axial system — https://en.wikipedia.org/wiki/Hexagonal_coordinate_systems
 *   Object pool pattern — gameprogrammingpatterns.com/object-pool.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two address levels:
 *   1. The HEX  — a flat-top tiling with axial coordinates (q, r).
 *   2. The WEDGE — one of 6 equilateral sub-triangles inside the hex,
 *      indexed by sector ∈ 0..5 measured CCW from +x.
 * The cursor carries (q, r, sector). Arrows change (q, r); ',' / '.'
 * rotate the sector within the current hex. SPACE pins an object at the
 * current address.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a soccer-ball pattern of hexes; inside each hex draw three
 * lines from corner to corner through the centre, splitting it into six
 * equilateral wedges. The cursor walks hexes; the sector is the "clock
 * hand" inside the hex pointing to the active wedge. Objects are stored
 * by (q, r, sector) — not by pixel position — so they survive resize and
 * size change.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. ctx_draw_bg — for each screen cell:
 *       pixel_to_hex → (q, r, dist)
 *       hex_centre_pixel → (cx, cy)
 *       if dist > limit_inner: render hex border via angle_char
 *       else: test proximity to the 3 radii; if near, render '/' '\\' '-'
 *  3. pool_draw — for each placed object, glyph at wedge centroid screen
 *     cell.
 *  4. cursor_draw — '@' on top.
 *
 * KEY FORMULAS
 * ────────────
 *  Pixel → axial (flat-top, with size = hex side length):
 *    fq = (2/3 · px) / size
 *    fr = (-1/3 · px + (√3/3) · py) / size
 *    Cube-round (fq, fr, fs = -fq-fr) → integer (q, r).
 *
 *  Hex centre → pixel:
 *    cx = size · 1.5 · q
 *    cy = size · ((√3/2) · q + √3 · r)
 *
 *  Wedge centroid (1/3 of the way from centre to a vertex):
 *    angle = sector · π/3
 *    radius = size · √3 / 3
 *    cx_w   = cx + radius · cos(angle)
 *    cy_w   = cy + radius · sin(angle)
 *
 *  Cursor step (hex movement):
 *    HEX_DIR[idx] for idx ∈ {LEFT=2, RIGHT=3, UP=0, DOWN=1}
 *    cur->q += HEX_DIR[idx][0]
 *    cur->r += HEX_DIR[idx][1]
 *
 *  Sector rotate:
 *    cur->sector = (cur->sector + Δ + 6) mod 6
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Arrow→hex mapping: the hex grid has 6 neighbours, not 4. This file
 *    uses ONLY 4 cardinal arrow keys, mapping to 4 of those 6. The two
 *    diagonal hex neighbours are reachable by chaining two key-presses.
 *  • Sector rotation does NOT change q or r — only the cursor's wedge
 *    inside the same hex.
 *  • dist threshold: limit_inner = 0.5 - BORDER_W. dist near 0.5 means
 *    the pixel is near a hex edge; dist near 0 means near the centre.
 *  • Aspect: CELL_W=2, CELL_H=4 keeps wedges roughly equilateral on
 *    screen even though terminal characters are taller than wide.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Place a glyph at sector 0 (east wedge). Press ',' six times — the
 *  cursor returns to sector 0; the glyph is still where you placed it.
 *  Press LEFT — cursor moves to (q-1, r, sector=0); the glyph stays.
 *  Press +/- to resize: glyph still anchored to its wedge.
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

#define MAX_OBJ    256
#define N_GLYPHS   6
#define N_THEMES   4

#define FPS_EWMA_ALPHA  0.05

#define PAIR_BORDER 1
#define PAIR_RADIUS 2
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

static const short THEME_FG[N_THEMES][3] = {
    /* edge,  radius, object */
    {  75,  39, 226 },
    {  82, 226, 207 },
    { 207, 196,  82 },
    {  87, 207, 226 },
};
static const short THEME_FG_8[N_THEMES][3] = {
    { COLOR_CYAN,    COLOR_BLUE,   COLOR_YELLOW  },
    { COLOR_GREEN,   COLOR_YELLOW, COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_RED,    COLOR_GREEN   },
    { COLOR_WHITE,   COLOR_CYAN,   COLOR_YELLOW  },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e, fg_r, fg_o;
    if (COLORS >= 256) {
        fg_e = THEME_FG[theme][0];
        fg_r = THEME_FG[theme][1];
        fg_o = THEME_FG[theme][2];
    } else {
        fg_e = THEME_FG_8[theme][0];
        fg_r = THEME_FG_8[theme][1];
        fg_o = THEME_FG_8[theme][2];
    }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_RADIUS, fg_r, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15 : COLOR_WHITE, COLOR_BLUE);
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

/*
 * angle_char — copy from hex_grids/01_flat_top.c.
 *
 * Maps a tangent angle to the best-fit ASCII line character. Folded into
 * [0, π) because line characters are symmetric under 180° flip.
 */
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

/*
 * pixel_to_hex — cube-round to (Q, R); also returns dist (max axial
 * residual) so the caller can tell how close to a hex edge the pixel is.
 */
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
    double d  = fabs(fq - fQ);
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

/* ctx_to_screen — (q, r, sector) → screen cell of the wedge centroid. */
static void ctx_to_screen(const GridCtx *g, int q, int r, int sector,
                          int *scol, int *srow)
{
    double cx, cy;
    wedge_centroid_pixel(q, r, sector, g->hex_size, &cx, &cy);
    *scol = g->ox + (int)(cx / g->cw);
    *srow = g->oy + (int)(cy / g->ch);
}

/*
 * ctx_draw_bg — paint hex borders + 3 inner radii.
 *
 * Per-pixel raster scan: identify which hex owns the cell, then either
 * draw the border angle character or test proximity to the three
 * centre-to-vertex diagonals.
 */
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
    if (pool_find(p, q, r, sector) >= 0) return;
    if (p->count >= MAX_OBJ) return;
    p->items[p->count++] = (Obj){ q, r, sector, glyph, true };
}

static void pool_remove(Pool *p, int q, int r, int sector)
{
    int i = pool_find(p, q, r, sector);
    if (i < 0) return;
    p->items[i] = p->items[--p->count];
}

static void pool_toggle(Pool *p, int q, int r, int sector, char glyph)
{
    if (pool_find(p, q, r, sector) >= 0) pool_remove(p, q, r, sector);
    else                                  pool_place(p, q, r, sector, glyph);
}

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].q, p->items[i].r, p->items[i].sector,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int q, r, sector;
    int glyph_idx;
    int theme;
    int paused;
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
    cur->glyph_idx = 0;
    cur->theme = 0;
    cur->paused = 0;
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
/* §7  mode — direct: SPACE toggles via pool_toggle                        */
/* ═══════════════════════════════════════════════════════════════════════ */

static void mode_toggle_at_cursor(Pool *p, const Cursor *cur)
{
    pool_toggle(p, cur->q, cur->r, cur->sector, GLYPHS[cur->glyph_idx]);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d sec:%d  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, cur->sector,
             p->count, GLYPHS[cur->glyph_idx], g->hex_size,
             cur->theme, fps, cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:hex  ,/.: sector  spc:toggle  g:glyph  C:clear  +/-:size  t:theme  r:reset  q:quit  [06 hex direct] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, const Pool *p,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(p, g);
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
    cur.glyph_idx = 0; cur.theme = 0; cur.paused = 0;
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
                    cursor_reset(&cur, &g); pool_clear(&pool);
                    color_init(cur.theme);
                    break;
                case 'C': pool_clear(&pool); break;
                case 'g': cur.glyph_idx = (cur.glyph_idx + 1) % N_GLYPHS; break;
                case ' ': mode_toggle_at_cursor(&pool, &cur); break;
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
                    } break;
                case '-':
                    if (g.hex_size > HEX_SIZE_MIN) {
                        g.hex_size -= HEX_SIZE_STEP;
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
