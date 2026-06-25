/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_hex_scatter.c — four ways to scatter objects across a hex grid
 *
 * Move the '@' cursor over a flat-top hex grid and press SPACE to "spray"
 * objects around it. Four sprays: uniform (random and sparse), mindist
 * (evenly spaced, no two too close), flood (fills the whole circle solid),
 * and gradient (thick near the cursor, thinning toward the edge). The spray
 * radius and the density/spacing knobs are all adjustable live.
 *
 * Sister files: 03_hex_path.c (placing along a path on this same grid),
 *               ../rect_grids_placement/04_scatter.c (the same idea on squares).
 * The even-spacing spray follows Poisson-disk sampling
 * (Bridson 2007: https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf).
 * Hex math (distance, circles): https://www.redblobgames.com/grids/hexagons/
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ncurses.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ── */

#define CELL_W              2
#define CELL_H              4

#define HEX_SIZE_DEFAULT   14.0
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35

/* How big the spray circle is, in hex steps. The cap is 8 because an 8-step
 * circle is 217 cells, which still fits under the 256-object limit below. */
#define RADIUS_DEFAULT      4
#define RADIUS_MIN          1
#define RADIUS_MAX          8

/* Uniform spray: chance each cell in the circle gets an object (0=none, 1=all). */
#define DENSITY_DEFAULT     0.40
#define DENSITY_STEP        0.05
#define DENSITY_MIN         0.05
#define DENSITY_MAX         1.00

/* Even-spacing spray: how many hex steps apart objects must stay. */
#define MINDIST_DEFAULT     2
#define MINDIST_MIN         1
#define MINDIST_MAX         6

/* Gradient spray: controls how fast the spray thins out from the center.
 * Bigger = thins slower (more even). Must stay at least 1 so we never divide
 * by zero at the center cell. */
#define GRAD_FALLOFF        3

/* Most objects we'll ever hold; extra placements past this are dropped. */
#define MAX_OBJ            256
#define FRAME_NS    16666667LL   /* one frame, ~60 per second, in nanoseconds */

/* How heavily to smooth the on-screen fps number so it doesn't jump around. */
#define FPS_EWMA_ALPHA      0.05

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 color ── */

#define PAIR_GRID      1
#define PAIR_CURSOR    2
#define PAIR_OBJ       3
#define PAIR_HUD       4   /* status bar (yellow)    */
#define PAIR_HINT      5   /* key-hint footer (cyan) */

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR, COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_OBJ,    COLORS >= 256 ? 214 : COLOR_RED,     -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,    -1);
}

/* ── §4 gridctx ── */

/* Everything needed to turn a hex coordinate into a spot on the screen, and
 * back. Bundled together so the drawing code can pass one thing around. */
typedef struct {
    int    rows, cols;   /* terminal size in characters */
    double hex_size;     /* how big one hex is drawn, in sub-cell units */
    double border_w;     /* fraction of each hex left blank as a gap (0..0.5) */
    int    ox, oy;       /* screen cell that the center hex (0,0) sits on */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols,
                      double hex_size, double border_w)
{
    g->rows = rows; g->cols = cols;
    g->hex_size = hex_size;
    g->border_w = border_w;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
}

/*
 * Snaps a fuzzy in-between hex position to the nearest real hex cell. Hex
 * coordinates come in threes that must always sum to zero; rounding each one
 * separately can break that, so we fix up whichever one we rounded worst.
 * Full derivation lives in 01_hex_direct.c §4.
 */
static void cube_round(double fq, double fr, double fs, int *q, int *r)
{
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) { *q = -rr - rs; *r = rr; }
    else if (dr > ds)             { *q = rq; *r = -rq - rs; }
    else                          { *q = rq; *r = rr; }
}

/* Hex coordinate (q, r) -> which screen cell to draw it on. */
static void ctx_to_screen(const GridCtx *g, int q, int r, int *col, int *row)
{
    double sq3 = sqrt(3.0);
    double cx  = g->hex_size * 1.5 * (double)q;
    double cy  = g->hex_size * (sq3 * 0.5 * (double)q + sq3 * (double)r);
    *col = g->ox + (int)round(cx / CELL_W);
    *row = g->oy + (int)round(cy / CELL_H);
}

/*
 * How many hex steps apart two cells are. This is the workhorse of the whole
 * file: every spray uses it to keep only the cells inside the circle, and the
 * even-spacing spray uses it to check nothing is too close to nothing else.
 */
static int hex_dist(int q1, int r1, int q2, int r2)
{
    int dq = q2 - q1, dr = r2 - r1;
    return (abs(dq) + abs(dr) + abs(dq + dr)) / 2;
}

/* Picks the ASCII line character ( - \ | / ) that best matches a given angle,
 * so a hex edge looks like it's leaning the right way. */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)        return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                              return '-';
}

/* Draws the empty hex grid that fills the screen. For each screen cell it
 * works out which hex it belongs to; cells near a hex's edge get an outline
 * character, cells deep inside a hex are left blank. The hex under the cursor
 * is tinted to stand out. */
static void ctx_draw_bg(const GridCtx *g, int curq, int curr)
{
    double size  = g->hex_size;
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - g->border_w;
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * CELL_W;
            double py = (double)(row - g->oy) * CELL_H;
            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;
            int q, r;
            cube_round(fq, fr, fs, &q, &r);
            double fS   = (double)(-q - r);
            double dist = fabs(fq - (double)q);
            double d2   = fabs(fr - (double)r);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;
            double cx    = size * 1.5 * (double)q;
            double cy    = size * (sq3_2 * (double)q + sq3 * (double)r);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);
            int on_cur = (q == curq && r == curr);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_GRID)   | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── §5 pool ── */

/* One placed object: which hex it sits on (q, r) and the character to draw. */
typedef struct { int q, r; char glyph; } Obj;

/* The bag of everything that's been scattered so far. Fixed-size array, no
 * heap; once it's full (count == MAX_OBJ) further placements are ignored.
 * 'count' is how many of the 'items' slots are actually in use. */
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

/* Adds an object at a hex, or just updates the glyph if one's already there,
 * so the same cell never gets two objects. Silently does nothing if the bag
 * is full. */
static void pool_place(Pool *p, int q, int r, char glyph)
{
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].q == q && p->items[i].r == r) {
            p->items[i].glyph = glyph; return;
        }
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ q, r, glyph };
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        ctx_to_screen(g, p->items[i].q, p->items[i].r, &col, &row);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
}

/* ── §6 cursor ── */

/* Where the '@' marker is on the grid, in hex coordinates. */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur) { cur->q = 0; cur->r = 0; }

/* The (q, r) step for each arrow key: up, down, left, right in that order. */
static const int HEX_DIR[4][2] = {
    { 0, -1 }, { 0, +1 }, {-1, 0}, {+1, 0}
};

static void cursor_move(Cursor *cur, int dq, int dr)
{
    cur->q += dq; cur->r += dr;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx    = g->hex_size * 1.5    * (double)cur->q;
    double cy    = g->hex_size * (sq3_2 * (double)cur->q + sq3 * (double)cur->r);
    int col = g->ox + (int)(cx / CELL_W);
    int row = g->oy + (int)(cy / CELL_H);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §7 scatter ── */
/*
 * The four sprays all walk the same circle of hexes around the cursor (every
 * cell within 'radius' steps). What differs is the rule each one uses to
 * decide whether a given cell gets an object. That single rule is the whole
 * personality of each spray.
 */

/* Uniform spray: flip a weighted coin for every cell, place a '.' if it wins.
 * Higher density = more cells win, but they're scattered with no pattern. */
static void scatter_uniform(Pool *pool, int cQ, int cR,
                             int radius, double density)
{
    for (int dr = -radius; dr <= radius; dr++) {
        for (int dq = -radius; dq <= radius; dq++) {
            if (hex_dist(0, 0, dq, dr) > radius) continue;
            if ((double)rand() / (double)RAND_MAX > density) continue;
            pool_place(pool, cQ + dq, cR + dr, '.');
        }
    }
}

/*
 * Even-spacing spray (Poisson-disk sampling): place a '+' only where it stays
 * at least 'mindist' steps from every object already down. That guarantee is
 * what makes the result look tidy and evenly spread instead of clumpy. We
 * check each candidate against the whole bag, which is cheap here (a few tens
 * of thousands of comparisons at most, well under a millisecond).
 */
static void scatter_mindist(Pool *pool, int cQ, int cR,
                             int radius, int mindist)
{
    for (int dr = -radius; dr <= radius; dr++) {
        for (int dq = -radius; dq <= radius; dq++) {
            if (hex_dist(0, 0, dq, dr) > radius) continue;
            int q = cQ + dq, r = cR + dr;
            int ok = 1;
            for (int i = 0; i < pool->count && ok; i++) {
                if (hex_dist(q, r, pool->items[i].q, pool->items[i].r) < mindist)
                    ok = 0;
            }
            if (ok) pool_place(pool, q, r, '+');
        }
    }
}

/* Flood spray: no coin flip, no spacing check — fill every cell in the circle
 * solid with '#'. The simplest rule of the four. */
static void scatter_flood(Pool *pool, int cQ, int cR, int radius)
{
    for (int dr = -radius; dr <= radius; dr++) {
        for (int dq = -radius; dq <= radius; dq++) {
            if (hex_dist(0, 0, dq, dr) <= radius)
                pool_place(pool, cQ + dq, cR + dr, '#');
        }
    }
}

/* Gradient spray: like the uniform spray, but the win chance shrinks the
 * farther a cell is from the cursor. The center cell always wins; outer cells
 * win less and less, giving a '*' blob that's dense in the middle and fades at
 * the rim. GRAD_FALLOFF sets how gentle the fade is. */
static void scatter_gradient(Pool *pool, int cQ, int cR, int radius)
{
    for (int dr = -radius; dr <= radius; dr++) {
        for (int dq = -radius; dq <= radius; dq++) {
            int d = hex_dist(0, 0, dq, dr);
            if (d > radius) continue;
            double p = (double)GRAD_FALLOFF / (double)(d + GRAD_FALLOFF);
            if ((double)rand() / (double)RAND_MAX < p)
                pool_place(pool, cQ + dq, cR + dr, '*');
        }
    }
}

/* ── §8 scene ── */

/* Which of the four sprays is currently selected. N_SC is the count, handy
 * for sizing the name table below. */
typedef enum {
    SC_UNIFORM=0, SC_MINDIST=1, SC_FLOOD=2, SC_GRADIENT=3, N_SC=4
} ScatterMode;

static const char *SC_NAME[N_SC] = { "uniform", "mindist", "flood", "gradient" };

/* The live settings the user tweaks with the keys. Some knobs only matter for
 * one spray (density for uniform, mindist for even-spacing); they're kept here
 * regardless so switching sprays remembers each one's last value. */
typedef struct {
    ScatterMode mode;
    int         radius;    /* spray circle size, in hex steps */
    double      density;   /* uniform spray only: win chance per cell */
    int         mindist;   /* even-spacing spray only: minimum gap in steps */
} SceneCfg;

static void cfg_init(SceneCfg *cfg)
{
    cfg->mode    = SC_UNIFORM;
    cfg->radius  = RADIUS_DEFAULT;
    cfg->density = DENSITY_DEFAULT;
    cfg->mindist = MINDIST_DEFAULT;
}

static void scene_scatter(Pool *pool, const Cursor *cur, const SceneCfg *cfg)
{
    switch (cfg->mode) {
    case SC_UNIFORM:  scatter_uniform (pool, cur->q, cur->r, cfg->radius, cfg->density); break;
    case SC_MINDIST:  scatter_mindist (pool, cur->q, cur->r, cfg->radius, cfg->mindist); break;
    case SC_FLOOD:    scatter_flood   (pool, cur->q, cur->r, cfg->radius);               break;
    case SC_GRADIENT: scatter_gradient(pool, cur->q, cur->r, cfg->radius);               break;
    default: break;
    }
}

/* Draws the two overlays: current spray + settings + fps in the top-right,
 * and the key reminders along the bottom. */
static void hud_draw(const GridCtx *g, const Pool *p, const SceneCfg *cfg,
                      double fps)
{
    char buf[96];
    if (cfg->mode == SC_UNIFORM) {
        snprintf(buf, sizeof buf,
                 " %s  R:%d  d:%.2f  obj:%d  %5.1f fps ",
                 SC_NAME[cfg->mode], cfg->radius, cfg->density, p->count, fps);
    } else if (cfg->mode == SC_MINDIST) {
        snprintf(buf, sizeof buf,
                 " %s  R:%d  min:%d  obj:%d  %5.1f fps ",
                 SC_NAME[cfg->mode], cfg->radius, cfg->mindist, p->count, fps);
    } else {
        snprintf(buf, sizeof buf,
                 " %s  R:%d  obj:%d  %5.1f fps ",
                 SC_NAME[cfg->mode], cfg->radius, p->count, fps);
    }
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:scatter  1-4:mode  +/-:R  d/D:density  m/M:mindist  C:clear  q/ESC:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                        const SceneCfg *cfg, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    pool_draw(p, g);
    cursor_draw(cur, g);
    hud_draw(g, p, cfg, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §9 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
    color_init();
    atexit(screen_cleanup);
}

/* ── §10 app ── */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;

static void on_signal(int sig)
{
    if (sig == SIGINT || sig == SIGTERM) g_running = 0;
    if (sig == SIGWINCH)                 g_need_resize = 1;
}

int main(void)
{
    srand((unsigned int)time(NULL));
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    screen_init();

    int rows, cols; getmaxyx(stdscr, rows, cols);
    GridCtx g;   ctx_init(&g, rows, cols, HEX_SIZE_DEFAULT, BORDER_W_DEFAULT);
    Cursor  cur; cursor_reset(&cur);
    Pool    pool; pool_clear(&pool);
    SceneCfg cfg; cfg_init(&cfg);

    double fps = 60.0;
    int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            /* Terminal was resized: the endwin/refresh pair is the ncurses
             * way to pick up the new window size before we re-measure it. */
            g_need_resize = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            ctx_init(&g, rows, cols, g.hex_size, g.border_w);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'r': cursor_reset(&cur); break;
            case 'C': pool_clear(&pool); break;
            case ' ': scene_scatter(&pool, &cur, &cfg); break;
            case '1': cfg.mode = SC_UNIFORM;  break;
            case '2': cfg.mode = SC_MINDIST;  break;
            case '3': cfg.mode = SC_FLOOD;    break;
            case '4': cfg.mode = SC_GRADIENT; break;
            case KEY_UP:    cursor_move(&cur, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (cfg.radius < RADIUS_MAX) cfg.radius++;
                break;
            case '-':
                if (cfg.radius > RADIUS_MIN) cfg.radius--;
                break;
            case 'd':
                if (cfg.density < DENSITY_MAX - DENSITY_STEP/2)
                    cfg.density += DENSITY_STEP;
                break;
            case 'D':
                if (cfg.density > DENSITY_MIN + DENSITY_STEP/2)
                    cfg.density -= DENSITY_STEP;
                break;
            case 'm':
                if (cfg.mindist < MINDIST_MAX) cfg.mindist++;
                break;
            case 'M':
                if (cfg.mindist > MINDIST_MIN) cfg.mindist--;
                break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;

        scene_draw(&g, &pool, &cur, &cfg, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
