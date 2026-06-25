/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_hex_path.c — lay objects along a PATH between two hexes on a flat-top grid.
 *
 * Set start A with 'a', move the '@' cursor to a second hex, press SPACE to
 * stamp. Three path flavours: a straight hex line (line-draw via lerp +
 * cube_round), a ring N steps out, and an L-shaped path that turns once.
 *
 * Shares its hex mapping / lattice / pool layers with 01_hex_direct.c (§4..§6).
 * Sister files: 02_hex_pattern.c (stamp a pattern), rect_grids_placement/03_path.c.
 * Path math: https://www.redblobgames.com/grids/hexagons/ (line draw, rings).
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

#define CELL_W              2     /* pixels per char cell (2 wide x 4 tall) — */
#define CELL_H              4     /* undoes the cell's tall aspect so hexes look round */

#define HEX_SIZE_DEFAULT   14.0   /* centre-to-corner, pixels; bigger = fewer hexes */
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

#define BORDER_W_DEFAULT    0.10  /* outline band width, 0..0.5 (0 = none, 0.5 = solid) */
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35

/* A ring of radius N holds 6N hexes, so cap N so the ring still fits the pool. */
#define RING_N_DEFAULT      3
#define RING_N_MIN          0
#define RING_N_MAX         40     /* 6*40 = 240, under the 256 pool limit */

#define MAX_OBJ           256     /* room for far more objects than fit on screen */
#define FRAME_NS    16666667LL    /* one frame at ~60 fps, in nanoseconds */

#define FPS_EWMA_ALPHA      0.05  /* small = steadier on-screen fps number */

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

#define PAIR_GRID      1   /* the hex outlines                      */
#define PAIR_CURSOR    2   /* the hex you're on + the '@'           */
#define PAIR_ENDPT_A   3   /* the 'A' start marker                  */
#define PAIR_PATH      4   /* stamped path glyphs                   */
#define PAIR_HUD       5
#define PAIR_HINT      6
#define PAIR_ENDPT_B   7   /* the 'B' end marker                    */
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

/* ── §4 hex mapping & lattice ── */

/* GridCtx — the hex grid for one frame: window size, hex size/look, and where
 * hex (0,0) lands on screen (recomputed on resize so the grid stays centred). */
typedef struct {
    int    rows, cols;        /* terminal size, in character cells */
    double hex_size;          /* centre-to-corner distance, pixels */
    double border_w;          /* outline band width, 0..0.5 */
    int    ox, oy;            /* screen cell that hex (0,0) sits on */
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

/* one hex's centre in pixels, measured from the grid centre (flat-top layout) */
static void hex_center_pixel(double size, int q, int r, double *cx, double *cy)
{
    double sq3 = sqrt(3.0);
    *cx = size * 1.5 * (double)q;
    *cy = size * (sq3 * 0.5 * (double)q + sq3 * (double)r);
}

/* recipe step 1 — hex address (q,r) -> the screen cell at its centre. Rounds to
 * nearest so a dropped object lands dead-centre in its hex. */
static void axial_to_screen(const GridCtx *g, int q, int r, int *col, int *row)
{
    double cx, cy;
    hex_center_pixel(g->hex_size, q, r, &cx, &cy);
    *col = g->ox + (int)round(cx / CELL_W);
    *row = g->oy + (int)round(cy / CELL_H);
}

/* the reverse — a pixel offset from grid centre -> fractional hex (q,r,s).
 * s = -q-r, so the three coordinates always sum to zero. */
static void screen_to_axial_frac(const GridCtx *g, double px, double py,
                                 double *fq, double *fr, double *fs)
{
    *fq = (2.0 / 3.0 * px) / g->hex_size;
    *fr = (-1.0 / 3.0 * px + sqrt(3.0) / 3.0 * py) / g->hex_size;
    *fs = -*fq - *fr;
}

/* snap fractional (q,r,s) to the nearest real hex. Rounding each on its own can
 * push their sum off zero, so we re-derive whichever we rounded most. */
static void cube_round(double fq, double fr, double fs, int *q, int *r)
{
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) { *q = -rr - rs; *r = rr; }
    else if (dr > ds)            { *q = rq; *r = -rq - rs; }
    else                         { *q = rq; *r = rr; }
}

/* how near a hex edge a fractional point is: 0 at the centre, 0.5 at an edge */
static double hex_edge_distance(double fq, double fr, double fs, int q, int r)
{
    double dq = fabs(fq - (double)q);
    double dr = fabs(fr - (double)r);
    double ds = fabs(fs - (double)(-q - r));
    double d = dq;
    if (dr > d) d = dr;
    if (ds > d) d = ds;
    return d;
}

/* the ASCII glyph whose slant lies along an edge at angle theta: '-' flattish,
 * '|' steep, '/' and '\' between. The glyphs look the same flipped 180°, so we
 * fold theta into [0,pi). */
static char edge_glyph(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)        return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                            return '-';
}

/* recipe step 2 — draw the grid: for every screen cell, find its hex and how
 * near an edge it sits. Near an edge -> an outline glyph; deep inside -> blank.
 * The cursor's hex is drawn in its own colour. */
static void draw_lattice(const GridCtx *g, int curq, int curr)
{
    double limit = 0.5 - g->border_w;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * CELL_W;
            double py = (double)(row - g->oy) * CELL_H;

            double fq, fr, fs;
            screen_to_axial_frac(g, px, py, &fq, &fr, &fs);
            int q, r;
            cube_round(fq, fr, fs, &q, &r);

            if (hex_edge_distance(fq, fr, fs, q, r) < limit) continue;  /* inside the hex */

            double cx, cy;
            hex_center_pixel(g->hex_size, q, r, &cx, &cy);
            char ch = edge_glyph(atan2(py - cy, px - cx) + M_PI / 2.0);

            int attr = (q == curq && r == curr)
                       ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                       : (COLOR_PAIR(PAIR_GRID)   | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── §5 pool — the objects placed on the grid ── */

/* Obj — one stamped cell: which hex it's on (q,r) and its glyph. Storing the
 * hex address (not a screen spot) is what keeps it stuck to its hex on resize.
 * Pool — all objects packed into the front of items[]; count is how many. */
typedef struct { int q, r; char glyph; } Obj;
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

/* place an object on this hex, overwriting whatever was already there. At most
 * one object lives on a hex, so a path that crosses itself never doubles up. */
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

/* draw every object on top of the grid, each at its hex's centre */
static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        axial_to_screen(g, p->items[i].q, p->items[i].r, &col, &row);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

/* ── §6 cursor ── */

/* Cursor — where the player is, as a hex address (q,r). Unbounded: it can roam
 * off-screen and back, since the grid is conceptually endless. */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur) { cur->q = 0; cur->r = 0; }

/* what each arrow key adds to (q,r). Four arrows reach four of a hex's six
 * neighbours; the two diagonals are reached by combining moves. */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_move(Cursor *cur, int dq, int dr)
{
    cur->q += dq; cur->r += dr;
}

/* draw the '@' on the player's hex. Unlike objects this truncates instead of
 * rounding, nudging '@' just inside the hex so it never lands on the outline. */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    double cx, cy;
    hex_center_pixel(g->hex_size, cur->q, cur->r, &cx, &cy);
    int col = g->ox + (int)(cx / CELL_W);
    int row = g->oy + (int)(cy / CELL_H);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §7 paths — THIS FILE'S STEP: lay objects along a path ── */

/* steps to walk from one hex to another — how many points a line needs */
static int hex_dist(int q1, int r1, int q2, int r2)
{
    int dq = q2 - q1, dr = r2 - r1;
    return (abs(dq) + abs(dr) + abs(dq + dr)) / 2;
}

/* the six unit steps to a hex's neighbours, clockwise. The ring walk leans on
 * the order: start one side out, then take a run in each direction in turn, and
 * you trace the whole ring without doubling back.
 *   0:(+1,0)E  1:(0,+1)SE  2:(-1,+1)SW  3:(-1,0)W  4:(0,-1)NW  5:(+1,-1)NE */
static const int HEX6[6][2] = {
    {+1,  0}, { 0, +1}, {-1, +1},
    {-1,  0}, { 0, -1}, {+1, -1},
};

/* hex line-draw, one point — slide t∈[0,1] from A to B in fractional hex space,
 * then snap to the nearest hex. The straight line passes through exactly the
 * hexes of the true shortest path, with no diagonal favouritism. */
static void hex_lerp(double aQ, double aR, double bQ, double bR,
                     double t, int *q, int *r)
{
    double fq = aQ + (bQ - aQ) * t;
    double fr = aR + (bR - aR) * t;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, q, r);
}

/* PATH strategy 1 — the shortest line of hexes from A to B: N+1 evenly-spaced
 * lerp points along the straight line between them. */
static void hex_line(Pool *pool, int aQ, int aR, int bQ, int bR, char glyph)
{
    int N = hex_dist(aQ, aR, bQ, bR);
    if (N == 0) { pool_place(pool, aQ, aR, glyph); return; }
    for (int i = 0; i <= N; i++) {
        double t = (double)i / (double)N;
        int q, r;
        hex_lerp((double)aQ, (double)aR, (double)bQ, (double)bR, t, &q, &r);
        pool_place(pool, q, r, glyph);
    }
}

/* PATH strategy 2 — every hex exactly N steps from the centre (a hexagon
 * outline): jump out to one corner, then walk the six sides of N steps each. */
static void hex_ring(Pool *pool, int cQ, int cR, int N, char glyph)
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

/* PATH strategy 3 — an L-shaped path: slide along q to line up under B, turn
 * once at the corner, then slide along r to B. The second leg starts one past
 * the corner so the turn isn't stamped twice. */
static void hex_lpath(Pool *pool, int aQ, int aR, int bQ, int bR, char glyph)
{
    int dQ = (bQ >= aQ) ? 1 : -1;
    for (int q = aQ; q != bQ + dQ; q += dQ)
        pool_place(pool, q, aR, glyph);
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

/* The demo's placement state: path mode, ring size, and the two pinned
 * endpoints. has_a / has_b say whether each is set; until then we use the
 * cursor in its place. */
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
    axial_to_screen(g, q, r, &col, &row);
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
    axial_to_screen(g, q, r, &col, &row);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
        mvaddch(row, col, '.');
}

/* Same walk as hex_line, but drawn as dots. Does nothing until A is set. */
static void preview_line_draw(const GridCtx *g, const SceneCfg *cfg,
                               int bQ, int bR)
{
    if (!cfg->has_a) return;
    int N = hex_dist(cfg->aQ, cfg->aR, bQ, bR);
    for (int i = 0; i <= N; i++) {
        double t = (N > 0) ? (double)i / (double)N : 0.0;
        int q, r;
        hex_lerp((double)cfg->aQ, (double)cfg->aR,
                 (double)bQ,      (double)bR, t, &q, &r);
        preview_dot(g, q, r);
    }
}

/* Same walk as hex_ring, drawn as dots, always centred on the live cursor. */
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

/* Same walk as hex_lpath, drawn as dots; the second leg starts past the
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

/* Show, in green dots, exactly where SPACE would stamp right now. If B hasn't
 * been pinned with 'b', the cursor stands in for it so the preview follows the
 * '@' as you move. */
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

/* stamp the active path into the pool. If B was never pinned, the cursor is B. */
static void scene_stamp(Pool *pool, const Cursor *cur, const SceneCfg *cfg)
{
    int bQ = cfg->has_b ? cfg->bQ : cur->q;
    int bR = cfg->has_b ? cfg->bR : cur->r;
    char glyph = (cfg->path_mode == PATH_LINE)  ? '*' :
                 (cfg->path_mode == PATH_RING)  ? 'o' : '+';
    switch (cfg->path_mode) {
    case PATH_LINE:
        if (cfg->has_a)
            hex_line(pool, cfg->aQ, cfg->aR, bQ, bR, glyph);
        break;
    case PATH_RING:
        hex_ring(pool, cur->q, cur->r, cfg->ring_n, glyph);
        break;
    case PATH_LPATH:
        if (cfg->has_a)
            hex_lpath(pool, cfg->aQ, cfg->aR, bQ, bR, glyph);
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
    draw_lattice(g, cur->q, cur->r);
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
