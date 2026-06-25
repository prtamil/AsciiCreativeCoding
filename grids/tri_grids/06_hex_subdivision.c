/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_hex_subdivision.c — flat-top hexagons, each sliced into 6 triangles.
 *
 * Draws a grid of hexagons and cuts each one into 6 pie-slice triangles by
 * running three lines through its centre. Arrow keys move a cursor from hex
 * to hex; ',' and '.' spin the cursor around the 6 slices of its hex.
 *
 * Sister files: grids/hex_grids/01_flat_top.c (the plain hex grid this builds
 *   on) and 04_30_60_90.c (the same "slice each tile up" idea, but on
 *   triangles instead of hexagons).
 * Background: Red Blob Games hex guide — https://www.redblobgames.com/grids/hexagons/
 */

/* ── how it works ──
 *
 * There is no grid stored in memory. For every character cell on screen we
 * work out, from its position alone, which hexagon it falls in and which of
 * that hex's 6 slices it falls in. Two bits of math do the whole job:
 *
 *   - Which hex?  A hexagon's corners are the same distance from its centre
 *     as its edges are long, so the three lines joining opposite corners cut
 *     it into 6 identical triangles, like slices of a pie. To find the hex,
 *     we use the standard flat-top hex lookup from hex_grids/01 (round the
 *     cell's position to the nearest hex).
 *
 *   - Which slice?  Take the cell's offset from the hex centre and ask which
 *     way it points (the angle, measured counter-clockwise from due east).
 *     Sort that angle into one of six 60-degree wedges — that's the slice.
 *
 * Drawing each cell: if it's near the hex's outline, paint a border character
 * angled to follow the edge. If it's near one of the three centre lines,
 * paint that line. Otherwise leave it blank (unless it's the cursor's slice,
 * which gets a faint dot so you can still see which slice is selected).
 *
 * One thing to watch: the slice test only runs for inside-the-hex cells. If
 * we let the centre lines run on past the outline, they'd leak into the
 * neighbouring hexagons' borders.
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

#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.30
#define BORDER_W_STEP    0.02

/* How wide the three centre lines are drawn, as a fraction of the hex size.
 * Picked so they look about as thick as the hex border. */
#define RADIUS_T_FRAC 0.12

#define N_THEMES 4

/* How much each frame nudges the on-screen fps number toward the latest
 * reading. Small value = a steady, slow-moving display instead of jitter. */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_BORDER 1
#define PAIR_RADIUS 2
#define PAIR_CURSOR 3
#define PAIR_HUD    4
#define PAIR_HINT   5

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

/* Each theme: a colour for the hex outline and a colour for the centre lines.
 * THEME_FG is the rich 256-colour version; THEME_FG_8 is the fallback for
 * terminals that only have 8 colours. */
static const short THEME_FG[N_THEMES][2] = {
    /* outline, centre-lines */
    {  75,  39 },
    {  82, 226 },
    { 207, 196 },
    {  15,  87 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_BLUE   },
    { COLOR_GREEN,   COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_RED    },
    { COLOR_WHITE,   COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e, fg_r;
    if (COLORS >= 256) { fg_e = THEME_FG[theme][0];   fg_r = THEME_FG[theme][1];   }
    else               { fg_e = THEME_FG_8[theme][0]; fg_r = THEME_FG_8[theme][1]; }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_RADIUS, fg_r, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — find the hex and slice for any point ── */

/*
 * GridCtx — everything we need to know to lay the hex grid out on screen.
 *
 * Bundled together so any drawing routine can be handed one struct and work
 * out where every hexagon goes. The grid is recentred (and these numbers
 * recomputed) on startup, resize, and whenever the size is changed.
 */
typedef struct {
    /* Terminal size, in character cells. */
    int rows, cols;

    /* hex_size: how big one hexagon is, in pixels (centre to corner).
     * border_w: how thick the hex outline is, as a fraction (0..0.5) — bigger
     *   means a chunkier border.
     * cw, ch: how many pixels one character cell is worth across and down,
     *   so wide-looking cells still produce round-looking hexes. */
    double hex_size;
    double border_w;
    int    cw, ch;

    /* Where pixel (0,0) lands on screen — the grid's anchor point, kept in
     * the middle so the layout stays centred. */
    int    ox, oy;

    /* Rough hint for how far the cursor can roam and still be on screen. The
     * hex plane is endless, so these aren't hard limits — just the largest
     * hex coordinates whose centre still fits at the current size. */
    int    max_q, max_r;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->cw   = CELL_W;
    g->ch   = CELL_H;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
    if (g->hex_size <= 0.0) g->hex_size = HEX_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    g->max_q = (int)((double)cols * CELL_W / (3.0 * g->hex_size)) + 1;
    g->max_r = (int)((double)rows * CELL_H / (sqrt(3.0) * g->hex_size)) + 1;
}

/* Picks the ASCII character ( - \ | / ) that best matches the slope of a
 * line pointing in the given direction. Copied from hex_grids/01_flat_top.c.
 * A line looks the same flipped 180 degrees, so opposite angles share a
 * character. */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)         return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                              return '-';
}

/* Which of the 6 pie slices does this offset point into? Takes how far a
 * point sits to the right (dx) and below (dy) the hex centre and returns a
 * slice number 0..5. Slice 0 faces right; the rest follow counter-clockwise,
 * each spanning 60 degrees. */
static int sector_of(double dx, double dy)
{
    double ang = atan2(dy, dx);
    int s = (int)floor((ang + M_PI / 6.0) / (M_PI / 3.0));
    s %= 6; if (s < 0) s += 6;
    return s;
}

/* Given a pixel, find which hexagon it lands in. Returns that hex's
 * coordinates (Q, R) plus dist: how far out toward the hex's edge the pixel
 * is, where small means near the centre and ~0.5 means near the outline.
 * This is the standard flat-top hex lookup from hex_grids/01 — snap the
 * pixel to the nearest of the three hex axes, then nudge the result so the
 * coordinates stay consistent. */
static void ctx_pixel_to_hex(const GridCtx *g, double px, double py,
                             int *Q, int *R, double *dist)
{
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double fq = (2.0 / 3.0 * px) / g->hex_size;
    double fr = (-1.0/3.0 * px + sq3_3 * py) / g->hex_size;
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

/* The reverse of the lookup above: given a hexagon's coordinates (Q, R),
 * work out the pixel where its centre sits. */
static void hex_centre_pixel(int Q, int R, double size,
                              double *cx, double *cy)
{
    double sq3 = sqrt(3.0);
    *cx = size * 1.5      * (double)Q;
    *cy = size * (sq3*0.5 * (double)Q + sq3 * (double)R);
}

/* Finds the character cell sitting at the middle of one pie slice, so we can
 * drop the cursor marker there. The middle of a slice is partway out from
 * the hex centre toward the slice's outer edge. */
static void ctx_to_screen(const GridCtx *g, int Q, int R, int sector,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    hex_centre_pixel(Q, R, g->hex_size, &cx_pix, &cy_pix);
    double ang = (double)sector * M_PI / 3.0;
    double r   = g->hex_size * sqrt(3.0) / 3.0;
    double mx  = cx_pix + r * cos(ang);
    double my  = cy_pix + r * sin(ang);
    *sc = g->ox + (int)(mx / g->cw);
    *sr = g->oy + (int)(my / g->ch);
}

/* Draws the whole grid by walking every character cell and deciding what,
 * if anything, belongs there: a hex outline, one of the three centre lines,
 * or nothing. The cursor's slice is drawn in a highlight colour. */
static void ctx_draw_bg(const GridCtx *g, int cQ, int cR, int cSector)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double limit_inner = 0.5 - g->border_w;
    double radius_t    = g->hex_size * RADIUS_T_FRAC * 0.5;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int Q, R;
            double dist;
            ctx_pixel_to_hex(g, px, py, &Q, &R, &dist);

            double cx, cy;
            hex_centre_pixel(Q, R, g->hex_size, &cx, &cy);
            double dxp = px - cx, dyp = py - cy;

            int on_cur_hex = (Q == cQ && R == cR);
            int sector     = sector_of(dxp, dyp);
            int on_cur_sec = (on_cur_hex && sector == cSector);

            /* Near the hex's outline: draw an edge piece. */
            if (dist >= limit_inner) {
                double theta = atan2(dyp, dxp);
                char ch = angle_char(theta + M_PI / 2.0);
                int attr = on_cur_sec ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                       : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
                continue;
            }

            /* Inside the hex: how close are we to each of the three centre
             * lines? Keep the nearest one; if it's close enough, draw it. */
            double r0 = fabs(dyp);
            double r1 = fabs(0.5 * dyp - sq3_2 * dxp);
            double r2 = fabs(0.5 * dyp + sq3_2 * dxp);
            char rch = '-'; double rmin = r0;
            if (r1 < rmin) { rmin = r1; rch = '/'; }
            if (r2 < rmin) { rmin = r2; rch = '\\'; }
            if (rmin < radius_t) {
                int attr = on_cur_sec ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                       : COLOR_PAIR(PAIR_RADIUS);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)rch);
                attroff(attr);
                continue;
            }

            /* Blank cell, but it's in the cursor's slice — leave a faint dot
             * so the selected slice is still visible when it has no lines. */
            if (on_cur_sec) {
                attron(COLOR_PAIR(PAIR_CURSOR));
                mvaddch(row, col, '.');
                attroff(COLOR_PAIR(PAIR_CURSOR));
            }
        }
    }
}

/* ── §5 cursor ── */

/*
 * Cursor — where the user's selection sits: which hexagon (q, r) and which
 * of its 6 slices (sector, 0..5).
 *
 * It holds nothing about screen size or hex size — that lives in GridCtx.
 * Hand both to ctx_to_screen() to turn a selection into a screen position.
 */
typedef struct { int q, r, sector; } Cursor;

/* The four arrow-key moves, as steps in hex coordinates (up/down/left/right). */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q      = 0;
    cur->r      = 0;
    cur->sector = 0;
}

/* Steps the cursor one hex in an arrow direction. No edge to bump into — the
 * grid is endless, so the cursor can wander off screen if pushed far enough. */
static void cursor_move(Cursor *cur, const GridCtx *g, int idx)
{
    (void)g;
    cur->q += HEX_DIR[idx][0];
    cur->r += HEX_DIR[idx][1];
}

static void cursor_rotate(Cursor *cur, const GridCtx *g, int delta)
{
    (void)g;
    cur->sector = (cur->sector + delta + 6) % 6;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->q, cur->r, cur->sector, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d sec:%d  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, cur->sector, g->hex_size, g->border_w,
             theme, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:hex  ,/.: sector  +/-:size  [/]:border  t:theme  r:reset  q:quit  [06 hex subdivision] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r, cur->sector);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    atexit(screen_cleanup);
}

/* ── §8 app ── */

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

    screen_init();
    int theme = 0, paused = 0;
    color_init(theme);

    GridCtx g = {0};
    g.hex_size = HEX_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           paused ^= 1; break;
                case 'r':           cursor_reset(&cur, &g); break;
                case 't':
                    theme = (theme + 1) % N_THEMES;
                    color_init(theme);
                    break;
                case KEY_UP:    cursor_move(&cur, &g, 0); break;
                case KEY_DOWN:  cursor_move(&cur, &g, 1); break;
                case KEY_LEFT:  cursor_move(&cur, &g, 2); break;
                case KEY_RIGHT: cursor_move(&cur, &g, 3); break;
                case ',': case '<': cursor_rotate(&cur, &g, -1); break;
                case '.': case '>': cursor_rotate(&cur, &g, +1); break;
                case '+': case '=':
                    if (g.hex_size < HEX_SIZE_MAX) { g.hex_size += HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '-':
                    if (g.hex_size > HEX_SIZE_MIN) { g.hex_size -= HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '[':
                    if (g.border_w > BORDER_W_MIN) { g.border_w -= BORDER_W_STEP; } break;
                case ']':
                    if (g.border_w < BORDER_W_MAX) { g.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns(), dt = now - t0; t0 = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
