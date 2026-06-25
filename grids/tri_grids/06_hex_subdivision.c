/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_hex_subdivision.c — flat-top hexagons, each sliced into 6 triangles.
 *
 * THE RECIPE: no grid is stored. Per screen cell, screen_to_hex maps the pixel
 * back to its hexagon (Q,R) and how far out it sits (dist); slice_of takes the
 * offset from the hex centre and bins its angle into one of six 60° wedges.
 * draw_lattice paints the hex outline (dist near the rim) or, inside, whichever
 * of the three centre lines is nearest (nearest_radius) — those three lines are
 * what cut each hex into 6 pie-slice triangles, the idea distinct to this file.
 *
 * Sister: tri_grids/01_equilateral.c (same per-cell screen->grid->glyph trick);
 *   hex_grids/01_flat_top.c (the plain hex lookup this reuses).
 * Refs: Red Blob Games hex guide — https://www.redblobgames.com/grids/hexagons/
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

/* ── §4 hex mapping & lattice ── */

/* GridCtx — the hex grid for one frame. Centred on screen, recomputed on
 * startup/resize/size-change. hex_size/border_w live here (not as constants)
 * because +/- and [/] tune them live. The plane is infinite, so max_q/r are a
 * rough on-screen reach, not hard limits. */
typedef struct {
    int    rows, cols;       /* terminal size in cells */
    double hex_size;         /* hex centre-to-corner, sub-pixels */
    double border_w;         /* outline thickness, as a fraction 0..0.5 */
    int    cw, ch;           /* sub-pixels per cell (CELL_W, CELL_H) */
    int    ox, oy;           /* where pixel (0,0) lands — the centred anchor */
    int    max_q, max_r;     /* rough on-screen reach, not a hard boundary */
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

/* ASCII char ( - \ | / ) matching the slope of a line at angle theta. A line
 * looks the same flipped 180°, so opposite angles share a char. */
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

/* the slice idea, distinct to this file — bin the offset's angle into one of
 * six 60° wedges. Slice 0 faces right (east); the rest go counter-clockwise. */
static int slice_of(double dx, double dy)
{
    double ang = atan2(dy, dx);
    int s = (int)floor((ang + M_PI / 6.0) / (M_PI / 3.0));
    s %= 6; if (s < 0) s += 6;
    return s;
}

/* recipe step 1 (reverse) — a pixel -> which hexagon (Q,R) it lands in, plus
 * dist: how far out toward the rim (~0 centre, ~0.5 outline). Standard flat-top
 * hex lookup: snap to the nearest of the three axes, then fix up coordinates. */
static void screen_to_hex(const GridCtx *g, double px, double py,
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

/* the pixel where a hexagon's centre sits (forward: hex -> point) */
static void hex_centre_pixel(int Q, int R, double size,
                              double *cx, double *cy)
{
    double sq3 = sqrt(3.0);
    *cx = size * 1.5      * (double)Q;
    *cy = size * (sq3*0.5 * (double)Q + sq3 * (double)R);
}

/* the terminal cell at a slice's middle — partway out from the hex centre
 * toward the slice's outer edge, so '@' lands inside the slice. */
static void hex_to_screen(const GridCtx *g, int Q, int R, int slice,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    hex_centre_pixel(Q, R, g->hex_size, &cx_pix, &cy_pix);
    double ang = (double)slice * M_PI / 3.0;
    double r   = g->hex_size * sqrt(3.0) / 3.0;
    double mx  = cx_pix + r * cos(ang);
    double my  = cy_pix + r * sin(ang);
    *sc = g->ox + (int)(mx / g->cw);
    *sr = g->oy + (int)(my / g->ch);
}

/* the subdivision math, distinct to this file — the three lines through the
 * hex centre that cut it into 6 slices. Returns the distance to the nearest of
 * them (in out_min) and its glyph, so a small out_min means "on a centre line". */
static char nearest_radius(double dx, double dy, double *out_min)
{
    double sq3_2 = sqrt(3.0) * 0.5;
    double r0 = fabs(dy);                       /* horizontal line */
    double r1 = fabs(0.5 * dy - sq3_2 * dx);    /* / line */
    double r2 = fabs(0.5 * dy + sq3_2 * dx);    /* \ line */
    char rch = '-'; double rmin = r0;
    if (r1 < rmin) { rmin = r1; rch = '/'; }
    if (r2 < rmin) { rmin = r2; rch = '\\'; }
    *out_min = rmin;
    return rch;
}

/* recipe step 2 — draw the grid: every cell -> its hex and slice -> a glyph.
 * Near the rim, a hex outline; inside, the nearest centre line (or nothing).
 * A blank cursor slice still gets a faint dot so the selection stays visible. */
static void draw_lattice(const GridCtx *g, int cQ, int cR, int cSlice)
{
    double limit_inner = 0.5 - g->border_w;
    double radius_t    = g->hex_size * RADIUS_T_FRAC * 0.5;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int Q, R;
            double dist;
            screen_to_hex(g, px, py, &Q, &R, &dist);

            double cx, cy;
            hex_centre_pixel(Q, R, g->hex_size, &cx, &cy);
            double dxp = px - cx, dyp = py - cy;

            int slice      = slice_of(dxp, dyp);
            int on_cur_sec = (Q == cQ && R == cR && slice == cSlice);

            /* near the rim: an outline piece, angled to follow the edge */
            if (dist >= limit_inner) {
                char ch = angle_char(atan2(dyp, dxp) + M_PI / 2.0);
                int attr = on_cur_sec ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                       : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
                continue;
            }

            /* inside: draw the nearest centre line if we're close enough */
            double rmin;
            char rch = nearest_radius(dxp, dyp, &rmin);
            if (rmin < radius_t) {
                int attr = on_cur_sec ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                       : COLOR_PAIR(PAIR_RADIUS);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)rch);
                attroff(attr);
                continue;
            }

            if (on_cur_sec) {
                attron(COLOR_PAIR(PAIR_CURSOR));
                mvaddch(row, col, '.');
                attroff(COLOR_PAIR(PAIR_CURSOR));
            }
        }
    }
}

/* ── §5 cursor ── */

/* Cursor — the selection: which hexagon (q,r) and which of its 6 slices
 * (sector, 0..5). Pair with a GridCtx and run through hex_to_screen. */
typedef struct { int q, r, sector; } Cursor;

/* arrow-key moves as steps in hex coordinates */
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

/* recipe step 3 — step the cursor one hex. No clamp; the plane is infinite. */
static void cursor_move(Cursor *cur, const GridCtx *g, int idx)
{
    (void)g;
    cur->q += HEX_DIR[idx][0];
    cur->r += HEX_DIR[idx][1];
}

/* spin the selection around the 6 slices of the current hex */
static void cursor_rotate(Cursor *cur, const GridCtx *g, int delta)
{
    (void)g;
    cur->sector = (cur->sector + delta + 6) % 6;
}

/* put '@' in the cursor's slice; after the grid so it lands on top */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    hex_to_screen(g, cur->q, cur->r, cur->sector, &sr, &sc);
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
    draw_lattice(g, cur->q, cur->r, cur->sector);
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
