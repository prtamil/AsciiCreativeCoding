/*
 * 04_ring_distance.c — colour a hex grid by how far each hex is from a movable '@'.
 *
 * No grid is stored. For each screen cell we ask "which hex is this pixel in,
 * and how many hex steps is that hex from the cursor?" The step count is the
 * "ring distance": the cursor is ring 0, its 6 neighbours ring 1, and so on, so
 * the colours form concentric hexagon rings that follow the cursor (§4).
 *
 * Sister file: grids/hex_grids/01_flat_top.c — shares the hex vocabulary below.
 * Reference:   Red Blob Games hex distances,
 *              https://www.redblobgames.com/grids/hexagons/#distances
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

#define CELL_W             2
#define CELL_H             4

#define HEX_SIZE_DEFAULT  14.0
#define HEX_SIZE_MIN       6.0
#define HEX_SIZE_MAX      40.0
#define HEX_SIZE_STEP      2.0

#define BORDER_W_DEFAULT   0.12
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.35
#define BORDER_W_STEP      0.02

#define N_RING_COLORS      6   /* colours repeat once you pass this many rings */

#define TICK_NS           16666667LL

#define FPS_EWMA_ALPHA     0.05   /* small = steadier on-screen fps number */

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

#define PAIR_RING    1   /* one colour per ring; rings beyond the palette reuse these */
#define PAIR_CURSOR  7   /* the hex the '@' sits on */
#define PAIR_HUD     8   /* yellow status bar */
#define PAIR_HINT    9   /* cyan key-hint footer */

static const short RING_FG[N_RING_COLORS] = {
    COLOR_WHITE, COLOR_CYAN, COLOR_GREEN,
    COLOR_YELLOW, COLOR_MAGENTA, COLOR_BLUE,
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < N_RING_COLORS; i++)
        init_pair(PAIR_RING + i, RING_FG[i], COLOR_BLACK);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 hex mapping & lattice ── */

/* GridCtx — the whole hex grid for one frame. The (q,r)=(0,0) hex sits at
 * screen centre; every other hex is measured outward from it. Keys +/- and
 * [/] write hex_size / border_w; the rest is recomputed on resize. */
typedef struct {
    int    rows, cols;      /* terminal size in character cells */
    double hex_size;        /* centre-to-corner distance, pixels; bigger = fewer hexes */
    double border_w;        /* outline band width, 0..0.5; how near an edge counts as edge */
    int    cell_w, cell_h;  /* pixels per char cell (2 vs 4) — undoes the cell's tall aspect */
    int    ox, oy;          /* screen-centre cell; the (0,0) hex lives here */
    int    max_q, max_r;    /* rough furthest visible hex; advisory, never clamped */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = (rows - 1) / 2;
    /* keep the user's chosen size/border across a resize; only seed if unset */
    if (g->hex_size <= 0.0) g->hex_size = HEX_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    g->max_q = (int)((double)cols * CELL_W / (3.0 * g->hex_size)) + 1;
    g->max_r = (int)((double)rows * CELL_H / (sqrt(3.0) * g->hex_size)) + 1;
}

/* one hex's centre in pixels, measured from the grid centre (flat-top layout) */
static void hex_center_pixel(double size, int q, int r, double *cx, double *cy)
{
    *cx = size * 1.5 * (double)q;
    *cy = size * (sqrt(3.0) * 0.5 * (double)q + sqrt(3.0) * (double)r);
}

/* recipe step 1 — hex address (q,r) -> the screen cell at its centre */
static void axial_to_screen(const GridCtx *g, int q, int r, int *sr, int *sc)
{
    double cx, cy;
    hex_center_pixel(g->hex_size, q, r, &cx, &cy);
    *sc = g->ox + (int)(cx / g->cell_w);
    *sr = g->oy + (int)(cy / g->cell_h);
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
    if      (dq > dr && dq > ds) rq = -rr - rs;
    else if (dr > ds)            rr = -rq - rs;
    *q = rq; *r = rr;
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

/* the ring-distance step — fewest single-hex steps from (q,r) to (cq,cr), the
 * number that drives every colour. The three axes (q, r, s = -q-r) always sum
 * to zero, so the biggest gap along any one axis is the answer: stepping
 * diagonally shrinks two axes at once. */
static int hex_distance(int q, int r, int cq, int cr)
{
    int dq = q - cq, dr = r - cr, ds = -dq - dr;
    int a = abs(dq), b = abs(dr), c = abs(ds);
    return a > b ? (a > c ? a : c) : (b > c ? b : c);
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

/* recipe step 2 — draw the grid: for every screen cell, find its hex and its
 * ring distance from the cursor. Inside the hex -> a colour-filled space, so
 * each ring reads as a solid band; near an edge -> an outline glyph in the same
 * ring colour. Ring 0 (the cursor's hex) gets its own colour. */
static void draw_lattice(const GridCtx *g, int cur_q, int cur_r)
{
    double limit = 0.5 - g->border_w;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            double fq, fr, fs;
            screen_to_axial_frac(g, px, py, &fq, &fr, &fs);
            int q, r;
            cube_round(fq, fr, fs, &q, &r);

            int ring = hex_distance(q, r, cur_q, cur_r);
            int pair = (ring == 0) ? PAIR_CURSOR
                                   : PAIR_RING + (ring % N_RING_COLORS);

            if (hex_edge_distance(fq, fr, fs, q, r) < limit) {
                attron(COLOR_PAIR(pair));
                mvaddch(row, col, ' ');               /* inside: solid colour band */
                attroff(COLOR_PAIR(pair));
            } else {
                double cx, cy;
                hex_center_pixel(g->hex_size, q, r, &cx, &cy);
                char ch = edge_glyph(atan2(py - cy, px - cx) + M_PI / 2.0);
                attron(COLOR_PAIR(pair) | A_BOLD);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(pair) | A_BOLD);
            }
        }
    }
}

/* ── §5 cursor ── */

/* Cursor — which hex is selected, named by its (q,r). The third coordinate is
 * always -q-r, so it isn't stored. The rings recolour from this every frame. */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/* what each arrow key adds to (q,r) to step onto a neighbouring hex. A flat-top
 * hex has six neighbours; the arrows reach four (the others sit on diagonals).
 * The same four steps work for sister grids 01–03. */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* up    */
    { 0, +1 },   /* down  */
    {-1,  0 },   /* left  */
    {+1,  0 },   /* right */
};

/* recipe step 3 — move the cursor by one hex. The grid is unbounded, so no clamp. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    (void)g;
    cur->q += dq;
    cur->r += dr;
}

/* drop the '@' on the selected hex; called after the grid so it lands on top */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    axial_to_screen(g, cur->q, cur->r, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, int paused, double fps)
{
    int dist_from_origin = hex_distance(cur->q, cur->r, 0, 0);
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  dist-from-origin:%d  size:%.0f  %5.1f fps  %s ",
             cur->q, cur->r, dist_from_origin, g->hex_size, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  r:reset  arrows:move cursor  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int paused, double fps)
{
    erase();
    draw_lattice(g, cur->q, cur->r);
    cursor_draw(cur, g);
    hud_draw(g, cur, paused, fps);
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
    color_init();
    int paused = 0;

    GridCtx g = {0};
    g.hex_size = HEX_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    double fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            /* endwin/refresh is how ncurses picks up the new terminal size */
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 'r': cursor_reset(&cur, &g); break;
            case KEY_UP:    cursor_move(&cur, &g, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, HEX_DIR[3][0], HEX_DIR[3][1]); break;
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

        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
