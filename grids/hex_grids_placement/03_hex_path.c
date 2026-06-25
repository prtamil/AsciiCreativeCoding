/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_hex_path.c — draw paths between hexes on a flat-top hex grid.
 *
 * Set a start point A with 'a', move the '@' cursor to a second spot, and
 * press SPACE to stamp a path. Three flavours: a straight line, a ring of
 * hexes a fixed number of steps away, and an L-shaped path that turns once.
 *
 * Sister files: 02_hex_pattern.c (stamping a pattern), and
 *               rect_grids_placement/03_path.c (the same idea on a square grid).
 * The path math comes from redblobgames.com/grids/hexagons (line drawing,
 * rings, and coordinate rounding).
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

/* A ring of radius N holds 6N hexes, so cap N so the ring still fits the pool. */
#define RING_N_DEFAULT      3
#define RING_N_MIN          0
#define RING_N_MAX         40   /* 6*40 = 240, under the 256 pool limit */

#define MAX_OBJ            256
#define FRAME_NS    16666667LL

/* How quickly the on-screen FPS number reacts; smaller = steadier reading. */
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
#define PAIR_ENDPT_A   3   /* the 'A' start marker         */
#define PAIR_PATH      4   /* stamped path glyphs          */
#define PAIR_HUD       5   /* status bar (yellow)          */
#define PAIR_HINT      6   /* key-hint footer (cyan)       */
#define PAIR_ENDPT_B   7   /* the 'B' end marker           */
#define PAIR_PREVIEW   8   /* the green dots shown before you stamp */

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,    COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_ENDPT_A, COLOR_WHITE,                COLOR_RED);
    init_pair(PAIR_PATH,    COLORS >= 256 ? 214 : COLOR_RED,     -1);
    init_pair(PAIR_HUD,     COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HINT,    COLORS >= 256 ?  51 : COLOR_CYAN,    -1);
    init_pair(PAIR_ENDPT_B, COLOR_WHITE,                COLOR_MAGENTA);
    init_pair(PAIR_PREVIEW, COLORS >= 256 ?  82 : COLOR_GREEN,   -1);
}

/* ── §4 gridctx ── */

/*
 * Everything we need to turn a hex address into a spot on the screen, and to
 * draw the empty grid. One of these is built once at startup and rebuilt on
 * resize.
 */
typedef struct {
    int    rows, cols;   /* terminal size, in characters */
    double hex_size;     /* radius of one hex, in screen sub-pixels */
    double border_w;     /* how thick the hex outline looks, 0..0.5 of a hex */
    int    ox, oy;       /* where hex (0,0) lands on screen, in characters */
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
 * Snap a fractional hex position to the nearest real hex. We round each of
 * the three coordinates, but rounding can break the rule that they must sum
 * to zero, so we re-derive whichever one we rounded the hardest. See
 * 01_hex_direct.c §4 for the worked-out reasoning.
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

/*
 * Find which character cell a hex's centre lands on. The math is the standard
 * flat-top hex placement, then we divide by the cell size so the answer comes
 * out in terminal rows and columns instead of sub-pixels.
 */
static void ctx_to_screen(const GridCtx *g, int q, int r, int *col, int *row)
{
    double sq3 = sqrt(3.0);
    double cx  = g->hex_size * 1.5 * (double)q;
    double cy  = g->hex_size * (sq3 * 0.5 * (double)q + sq3 * (double)r);
    *col = g->ox + (int)round(cx / CELL_W);
    *row = g->oy + (int)round(cy / CELL_H);
}

/*
 * How many steps it takes to walk from one hex to another. The straight-line
 * path uses this to know how many points to drop along the way.
 */
static int hex_dist(int q1, int r1, int q2, int r2)
{
    int dq = q2 - q1, dr = r2 - r1;
    return (abs(dq) + abs(dr) + abs(dq + dr)) / 2;
}

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

/* One stamped cell: its hex address (q, r) and the character drawn there. */
typedef struct { int q, r; char glyph; } Obj;

/*
 * The collection of everything stamped so far. It's a plain fixed array, no
 * dynamic memory, so paths just keep adding to it up to MAX_OBJ. There's only
 * ever one hex per address — placing on an occupied spot overwrites it.
 */
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

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
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        ctx_to_screen(g, p->items[i].q, p->items[i].r, &col, &row);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

/* ── §6 cursor ── */

/* Where the '@' currently sits, as a hex address. */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur) { cur->q = 0; cur->r = 0; }

/* The four arrow-key moves, in hex terms: up, down, left, right. */
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

/* ── §7 paths ── */

/*
 * The six steps to a hex's neighbours, going around clockwise. The ring walk
 * leans on the order: start one side away, then take a run of steps in each
 * direction in turn, and you trace the whole ring without ever doubling back.
 *
 *   0:(+1, 0) E    1:(0, +1) SE   2:(-1,+1) SW
 *   3:(-1, 0) W    4:(0, -1) NW   5:(+1,-1) NE
 */
static const int HEX6[6][2] = {
    {+1,  0}, { 0, +1}, {-1, +1},
    {-1,  0}, { 0, -1}, {+1, -1},
};

/*
 * Pick the single hex that lands part-way from A to B, where t=0 is A and t=1
 * is B. We slide along the straight line between them and snap to the nearest
 * hex. Because of how hex coordinates work, that straight line passes through
 * exactly the hexes of the true shortest path, with no diagonal favouritism.
 */
static void hex_lerp_round(double aQ, double aR, double bQ, double bR,
                            double t, int *q, int *r)
{
    double fq = aQ + (bQ - aQ) * t;
    double fr = aR + (bR - aR) * t;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, q, r);
}

/*
 * Stamp the shortest line of hexes from A to B. We figure out how many steps
 * apart they are, then drop one hex at each evenly-spaced point along the way.
 */
static void path_line(Pool *pool, int aQ, int aR, int bQ, int bR,
                       char glyph)
{
    int N = hex_dist(aQ, aR, bQ, bR);
    if (N == 0) { pool_place(pool, aQ, aR, glyph); return; }
    for (int i = 0; i <= N; i++) {
        double t = (double)i / (double)N;
        int q, r;
        hex_lerp_round((double)aQ, (double)aR, (double)bQ, (double)bR, t, &q, &r);
        pool_place(pool, q, r, glyph);
    }
}

/*
 * Stamp every hex that sits exactly N steps away from the centre — a hexagon
 * outline. We jump out to one corner of the ring, then walk its six sides in
 * order. Each side is a straight run of N steps, so we trace the whole loop
 * once and land back where we started. N=0 is just the centre hex by itself.
 */
static void path_ring(Pool *pool, int cQ, int cR, int N, char glyph)
{
    if (N == 0) { pool_place(pool, cQ, cR, glyph); return; }
    int q = cQ + HEX6[4][0] * N;
    int r = cR + HEX6[4][1] * N;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < N; j++) {
            pool_place(pool, q, r, glyph);
            q += HEX6[i][0];
            r += HEX6[i][1];
        }
    }
}

/*
 * Stamp an L-shaped path: go straight along one axis to line up with B, turn
 * once at the corner, then go straight along the other axis to reach B. The
 * second leg starts just past the corner so we don't stamp it twice.
 */
static void path_lpath(Pool *pool, int aQ, int aR, int bQ, int bR,
                        char glyph)
{
    /* First leg: slide along q until we're under B, keeping r fixed. */
    int dQ = (bQ >= aQ) ? 1 : -1;
    for (int q = aQ; q != bQ + dQ; q += dQ)
        pool_place(pool, q, aR, glyph);
    /* Second leg: slide along r up to B, starting one past the shared corner. */
    if (aR != bR) {
        int dR = (bR >= aR) ? 1 : -1;
        for (int r = aR + dR; r != bR + dR; r += dR)
            pool_place(pool, bQ, r, glyph);
    }
}

/* For scatter-placement strategies, see 04_hex_scatter.c §7. */

/* ── §8 scene ── */

/* Which of the three path shapes is active. */
typedef enum { PATH_LINE=0, PATH_RING=1, PATH_LPATH=2, N_PATH=3 } PathMode;

static const char *PATH_NAME[N_PATH] = { "line", "ring", "lpath" };

/*
 * The current state of the demo: which path mode we're in, the ring size, and
 * the two endpoints the user has pinned down. has_a / has_b say whether each
 * endpoint has actually been set yet; until then we fall back to the cursor.
 */
typedef struct {
    PathMode path_mode;
    int      ring_n;              /* ring radius, only used in ring mode */
    int      has_a; int aQ, aR;   /* start point A, set with the 'a' key */
    int      has_b; int bQ, bR;   /* end point B, set with the 'b' key   */
} SceneCfg;

static void cfg_init(SceneCfg *cfg)
{
    cfg->path_mode = PATH_LINE;
    cfg->ring_n    = RING_N_DEFAULT;
    cfg->has_a = 0; cfg->aQ = 0; cfg->aR = 0;
    cfg->has_b = 0; cfg->bQ = 0; cfg->bR = 0;
}

/* Draw the 'A' or 'B' letter on the hex the user pinned down. */
static void endpoint_marker(const GridCtx *g, int q, int r,
                             int pair, char label)
{
    int col, row;
    ctx_to_screen(g, q, r, &col, &row);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1) {
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, col, label);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* Put one '.' on a hex if it's on screen. The caller sets the colour once and
 * leaves it on, so a whole preview shares the same attribute. */
static void preview_dot(const GridCtx *g, int q, int r)
{
    int col, row;
    ctx_to_screen(g, q, r, &col, &row);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
        mvaddch(row, col, '.');
}

/* Same walk as path_line, but drawn as dots. Does nothing until A is set. */
static void preview_line_draw(const GridCtx *g, const SceneCfg *cfg,
                               int bQ, int bR)
{
    if (!cfg->has_a) return;
    int N = hex_dist(cfg->aQ, cfg->aR, bQ, bR);
    for (int i = 0; i <= N; i++) {
        double t = (N > 0) ? (double)i / (double)N : 0.0;
        int q, r;
        hex_lerp_round((double)cfg->aQ, (double)cfg->aR,
                       (double)bQ,      (double)bR, t, &q, &r);
        preview_dot(g, q, r);
    }
}

/* Same walk as path_ring, drawn as dots, always centred on the live cursor. */
static void preview_ring_draw(const GridCtx *g, const Cursor *cur,
                               const SceneCfg *cfg)
{
    if (cfg->ring_n == 0) {
        preview_dot(g, cur->q, cur->r);
        return;
    }
    int q = cur->q + HEX6[4][0] * cfg->ring_n;
    int r = cur->r + HEX6[4][1] * cfg->ring_n;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < cfg->ring_n; j++) {
            preview_dot(g, q, r);
            q += HEX6[i][0];
            r += HEX6[i][1];
        }
    }
}

/* Same walk as path_lpath, drawn as dots; the second leg starts past the
 * corner so the turn isn't drawn twice. */
static void preview_lpath_draw(const GridCtx *g, const SceneCfg *cfg,
                                int bQ, int bR)
{
    if (!cfg->has_a) return;
    int dQ = (bQ >= cfg->aQ) ? 1 : -1;
    for (int q = cfg->aQ; q != bQ + dQ; q += dQ)
        preview_dot(g, q, cfg->aR);
    if (cfg->aR != bR) {
        int dR = (bR >= cfg->aR) ? 1 : -1;
        for (int r = cfg->aR + dR; r != bR + dR; r += dR)
            preview_dot(g, bQ, r);
    }
}

/*
 * Show, in green dots, exactly where SPACE would stamp right now. If B hasn't
 * been pinned with 'b', the cursor stands in for it so the preview follows the
 * '@' as you move.
 */
static void path_preview_draw(const GridCtx *g, const Cursor *cur,
                               const SceneCfg *cfg)
{
    int bQ = cfg->has_b ? cfg->bQ : cur->q;
    int bR = cfg->has_b ? cfg->bR : cur->r;

    attron(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
    switch (cfg->path_mode) {
        case PATH_LINE:  preview_line_draw  (g, cfg, bQ, bR); break;
        case PATH_RING:  preview_ring_draw  (g, cur, cfg);    break;
        case PATH_LPATH: preview_lpath_draw (g, cfg, bQ, bR); break;
        default: break;
    }
    attroff(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
}

static void scene_stamp(Pool *pool, const Cursor *cur, const SceneCfg *cfg)
{
    /* If B was never pinned, treat the cursor as B. */
    int bQ = cfg->has_b ? cfg->bQ : cur->q;
    int bR = cfg->has_b ? cfg->bR : cur->r;
    char glyph = (cfg->path_mode == PATH_LINE)  ? '*' :
                 (cfg->path_mode == PATH_RING)  ? 'o' : '+';
    switch (cfg->path_mode) {
    case PATH_LINE:
        if (cfg->has_a)
            path_line(pool, cfg->aQ, cfg->aR, bQ, bR, glyph);
        break;
    case PATH_RING:
        path_ring(pool, cur->q, cur->r, cfg->ring_n, glyph);
        break;
    case PATH_LPATH:
        if (cfg->has_a)
            path_lpath(pool, cfg->aQ, cfg->aR, bQ, bR, glyph);
        break;
    default: break;
    }
}

/* Status line top-right (mode, endpoints, fps) and key hints along the bottom. */
static void hud_draw(const GridCtx *g, const Pool *p, const SceneCfg *cfg,
                      double fps)
{
    char buf[96];
    if (cfg->path_mode == PATH_RING) {
        snprintf(buf, sizeof buf,
                 " ring N:%d  obj:%d  %5.1f fps ",
                 cfg->ring_n, p->count, fps);
    } else {
        snprintf(buf, sizeof buf,
                 " %s  A:%s B:%s  obj:%d  %5.1f fps ",
                 PATH_NAME[cfg->path_mode],
                 cfg->has_a ? "set" : "---",
                 cfg->has_b ? "set" : "---",
                 p->count, fps);
    }
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  a:set-A  b:set-B  spc:stamp  1:line  2:ring  3:lpath  +/-:N  C:clear  q/ESC:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                        const SceneCfg *cfg, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    pool_draw(p, g);
    path_preview_draw(g, cur, cfg);
    if (cfg->has_a)
        endpoint_marker(g, cfg->aQ, cfg->aR, PAIR_ENDPT_A, 'A');
    if (cfg->has_b)
        endpoint_marker(g, cfg->bQ, cfg->bR, PAIR_ENDPT_B, 'B');
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
            g_need_resize = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            ctx_init(&g, rows, cols, g.hex_size, g.border_w);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'r': cursor_reset(&cur); break;
            case 'C':
                pool_clear(&pool);
                cfg.has_a = 0; cfg.has_b = 0; break;
            case 'a':
                cfg.aQ = cur.q; cfg.aR = cur.r; cfg.has_a = 1; break;
            case 'b':
                cfg.bQ = cur.q; cfg.bR = cur.r; cfg.has_b = 1; break;
            case ' ': scene_stamp(&pool, &cur, &cfg); break;
            case '1': cfg.path_mode = PATH_LINE;  break;
            case '2': cfg.path_mode = PATH_RING;  break;
            case '3': cfg.path_mode = PATH_LPATH; break;
            case KEY_UP:    cursor_move(&cur, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (cfg.ring_n < RING_N_MAX) cfg.ring_n++;
                break;
            case '-':
                if (cfg.ring_n > RING_N_MIN) cfg.ring_n--;
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
