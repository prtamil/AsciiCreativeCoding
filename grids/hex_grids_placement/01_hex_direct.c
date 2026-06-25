/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_hex_direct.c — move a cursor around a hex grid and drop objects on it.
 *
 * A grid of flat-topped hexagons fills the screen. Arrow keys move the '@'
 * cursor from hex to hex; SPACE drops or removes a '*' on the current hex.
 * Each object remembers which hex it's on (not where on screen), so resizing
 * the terminal keeps it stuck to the same hex.
 *
 * Sister files: grids/hex_grids/01_flat_top.c (the grid drawing on its own),
 *               grids/rect_grids_placement/01_direct.c (same idea, square grid).
 * Hex coordinate math: https://www.redblobgames.com/grids/hexagons/
 */

/*
 * THE BIG PICTURE
 *
 * There are two ways of saying "where" in this program, and we constantly
 * translate between them:
 *
 *   - Hex address (q, r): which hexagon. The cursor and every dropped object
 *     are stored this way. The centre hex is (0, 0). This doesn't change when
 *     the window resizes.
 *
 *   - Screen position (row, col): which character cell ncurses draws into.
 *
 * Going from a hex to a screen cell is the "forward" direction: plug (q, r)
 * into a formula and out comes the centre of that hex on screen. Going the
 * other way — from a screen cell back to which hex covers it — is the
 * "inverse" direction, and it's how we paint the grid lines: we ask every cell
 * "which hex are you part of, and are you near its edge?".
 *
 * Handy mental image: each hex is a town with a name (q, r). We never write
 * down where a town sits on screen — we work it out from the name whenever we
 * need it. A dropped object is a sticky note pinned to a town's name, so it
 * always reappears wherever that town currently is.
 *
 * The trickiest bit is the inverse step. Turning a screen cell into a hex name
 * first gives fractional, in-between numbers; cube_round() snaps them to the
 * nearest real hex. See cube_round() for why that needs a fix-up step.
 *
 * Aspect note: terminal characters are about twice as tall as they are wide,
 * so CELL_W=2 / CELL_H=4 squash the math the right amount to make hexes look
 * round on screen instead of stretched.
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

/* Terminal characters are taller than they are wide, so we treat each cell as
 * 2 wide by 4 tall sub-units. That 1:2 shape keeps hexes looking round. */
#define CELL_W              2
#define CELL_H              4

#define HEX_SIZE_DEFAULT   14.0   /* how big each hex is; bigger = fewer on screen */
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

/* How thick the hex outlines are. 0 = no border, 0.5 = solid hexes.
 * 0.10 leaves a thin outline with open space inside each hex. */
#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35

#define MAX_OBJ            256   /* room for far more objects than fit on screen */
#define FRAME_NS    16666667LL   /* one frame at ~60 fps, in nanoseconds */

/* How much the on-screen fps number leans on its old value vs. the latest
 * frame. Small = smooth and slow to react, so the readout doesn't jitter. */
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

#define PAIR_GRID    1   /* the hex outlines            */
#define PAIR_CURSOR  2   /* the hex you're on + the '@' */
#define PAIR_OBJ     3   /* dropped '*' objects         */
#define PAIR_HUD     4   /* status bar, top-right        */
#define PAIR_HINT    5   /* key hints, bottom row        */

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

/*
 * GridCtx — everything the drawing code needs to know about the current grid:
 * how big the window is, how big the hexes are, and where the centre sits.
 * We pass this around instead of using globals so the same helpers could draw
 * any grid (the sister rect/tri/polar files reuse the same idea).
 *
 *   rows, cols  — size of the terminal in character cells.
 *   hex_size    — how big each hexagon is, in sub-cell units (see CELL_W/H).
 *   border_w    — how thick the hex outlines are, 0..0.5 (see BORDER_W_*).
 *   ox, oy      — where hex (0, 0) lands on screen. Recomputed on every resize
 *                 so the grid stays centred.
 */
typedef struct {
    int    rows, cols;        /* terminal size, in character cells */
    double hex_size;          /* hex size, in sub-cell units */
    double border_w;          /* outline thickness, 0..0.5 */
    int    ox, oy;            /* screen cell that hex (0, 0) sits on */
} GridCtx;

/*
 * Centre the grid in the current window. The bottom row is left out of the
 * vertical centring so the key-hint line at the bottom doesn't overlap a hex.
 */
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
 * Snap an in-between hex position to the nearest real hex.
 *
 * Hex math uses three numbers (fq, fr, fs) that must always add up to zero.
 * Rounding each to the nearest whole number can break that, so we round all
 * three, then re-derive whichever one we trusted least — the one that moved
 * furthest when we rounded it. Fixing the worst-off number keeps the two we
 * rounded more cleanly intact. (Red Blob Games, hex rounding.)
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
 * Work out which screen cell sits at the centre of hex (q, r).
 *
 * This is the "forward" direction: hex name in, screen position out. We round
 * to the nearest cell so a dropped object lands smack in the middle of its
 * hex. (cursor_draw deliberately rounds the other way to keep '@' off the
 * outline.) Standard flat-top hex layout from the Red Blob Games guide.
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
 * Pick the ASCII character that best matches the slope of a line.
 *
 * Given which way an edge runs, choose from -, \, |, / so the outline looks
 * like a real hexagon instead of a blob. A line and the same line flipped 180°
 * look identical, so we only care about the angle within a half-turn.
 */
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

/*
 * Draw the whole hex grid, one screen cell at a time.
 *
 * For each cell we run the "inverse" step: figure out which hex covers it and
 * how far it is from that hex's centre. Cells near the middle are blank; cells
 * out near a hex edge get an outline character (angle_char picks which one).
 * The hex the cursor is on is drawn in the cursor colour so it stands out.
 *
 * This visits every cell every frame, so it's the most expensive thing here —
 * but it's what lets the grid follow a resize for free.
 */
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
            double fS = (double)(-q - r);

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

/*
 * Obj — one dropped object: which hex it's on (q, r) and what to draw ('*').
 * Storing the hex address, not a screen position, is what lets objects stick
 * to their hex through resizes.
 *
 * Pool — the whole collection of dropped objects, kept in a plain array.
 *   items — the objects, packed into the front of the array.
 *   count — how many slots in items are actually used. Capped at MAX_OBJ.
 */
typedef struct { int q, r; char glyph; } Obj;
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

static int pool_find(const Pool *p, int q, int r)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].q == q && p->items[i].r == r)
            return i;
    return -1;
}

/*
 * Drop an object on this hex, or pick it back up if one's already there.
 *
 * To remove an object we just move the last one into its slot — no shuffling
 * the rest of the array down. That leaves the objects in a jumbled order, but
 * order never matters here since each hex holds at most one.
 */
static void pool_toggle(Pool *p, int q, int r)
{
    int i = pool_find(p, q, r);
    if (i >= 0) {
        p->items[i] = p->items[--p->count];   /* fill the gap with the last one */
        return;
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ q, r, '*' };
}

static void pool_clear(Pool *p) { p->count = 0; }

/*
 * Draw every dropped object. Each one's hex address is turned into a screen
 * cell and drawn on top of the grid, since objects sit on top of it.
 */
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

/*
 * Cursor — where the player is: just a hex address (q, r).
 *
 * There's no limit on how far it can go — the grid is conceptually endless, so
 * the cursor can roam off the visible screen and come back.
 */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur) { cur->q = 0; cur->r = 0; }

/*
 * HEX_DIR — how each arrow key nudges the cursor's hex address.
 *
 * On a flat-top grid the four arrows give you up/down (changing r) and
 * left/right (changing q). A hex actually has six sides, so the two diagonal
 * neighbours have no key — you reach them by combining moves.
 *
 *             UP: (0, -1)
 *  LEFT: (-1, 0)  *  RIGHT: (+1, 0)
 *            DOWN: (0, +1)
 */
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

/*
 * Draw the '@' on the hex the player is standing on.
 *
 * It uses the same hex-to-screen math as everything else, but rounds toward
 * zero rather than to nearest, which nudges the '@' just inside the hex so it
 * never lands on the outline.
 */
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

/* ── §7 scene ── */

/* Status line at top-right (cursor address, object count, fps) and the list of
 * keys along the bottom. */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                      double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
             " q:%+d r:%+d  obj:%d  size:%.0f  %5.1f fps ",
             cur->q, cur->r, p->count, g->hex_size, fps);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:toggle  C:clear  r:reset  +/-:size  q/ESC:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                        double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    pool_draw(p, g);
    cursor_draw(cur, g);
    hud_draw(g, p, cur, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §8 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
    color_init();
    atexit(screen_cleanup);
}

/* ── §9 app ── */

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
            case 'C': pool_clear(&pool); break;
            case ' ': pool_toggle(&pool, cur.q, cur.r); break;
            case KEY_UP:    cursor_move(&cur, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (g.hex_size < HEX_SIZE_MAX) g.hex_size += HEX_SIZE_STEP;
                break;
            case '-':
                if (g.hex_size > HEX_SIZE_MIN) g.hex_size -= HEX_SIZE_STEP;
                break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;

        scene_draw(&g, &pool, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
