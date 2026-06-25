/*
 * 07_trihexagonal.c — a hex grid and a triangle grid drawn on top of each other.
 *
 * Lay a flat-top hexagon grid (cyan) and a triangle grid (green) so their
 * corners land on the same spots. Together they make the trihexagonal tiling,
 * the star-of-David / snowflake pattern. Toggle each grid with h/t and steer an
 * '@' cursor through the hexes with the arrow keys.
 *
 * Sister files: hex_grids/01_flat_top.c (the hex drawing), 05_triangular.c
 * (the triangle drawing). Trihexagonal tiling: en.wikipedia.org/wiki/Trihexagonal_tiling
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

/* Each terminal cell is split into this many tiny units so the math has room
   to work; we draw in those units, not whole characters. */
#define CELL_W              2
#define CELL_H              4

/* How long a hex side is, in those tiny units. The triangles use the same
   length so the two grids share corners. */
#define HEX_SIZE_DEFAULT   14.0
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

/* How thick the grid lines look, for both grids. */
#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35
#define BORDER_W_STEP       0.02

#define TICK_NS            16666667LL    /* one frame at ~60 frames per second */

/* How much each new frame nudges the on-screen fps number; small means the
   readout glides instead of jumping around. */
#define FPS_EWMA_ALPHA      0.05

/* Color slots */
#define PAIR_HEX     1   /* cyan  — hex grid lines */
#define PAIR_TRI     2   /* green — triangle grid lines */
#define PAIR_CURSOR  3   /* the cursor hex and the '@' */
#define PAIR_HUD     4
#define PAIR_HINT    5

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
    struct timespec ts = { .tv_sec  = (time_t)(ns / 1000000000LL),
                           .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&ts, NULL);
}

/* ── §3 color ── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_HEX,    COLOR_CYAN,  COLOR_BLACK);
    init_pair(PAIR_TRI,    COLOR_GREEN, COLOR_BLACK);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — the hex grid and the triangle grid ── */

/*
 * GridCtx — everything the drawing code needs: the size and placement of the
 * tiling, and how far the cursor is allowed to wander.
 *
 * The cursor lives in hex coordinates (q, r), the same scheme as 01_flat_top:
 * q steps roughly east, r steps roughly south-east. The two on/off toggles and
 * the line thickness sit here because the background painter reads them to
 * decide what to draw.
 */
typedef struct {
    /* size of the terminal, in characters */
    int rows, cols;

    /* hex side length, plus how many tiny units make up one character cell */
    double hex_size;
    int    cell_w, cell_h;

    /* the character cell the tiling is centred on */
    int ox, oy;

    /* how far the cursor may roam, in hex steps each way from centre (generous —
       it's just a leash, not a tight box) */
    int max_q, max_r;

    /* look-and-feel knobs */
    double border_w;       /* line thickness, shared by both grids */
    bool   show_hex;       /* is the hex grid switched on? */
    bool   show_tri;       /* is the triangle grid switched on? */
} GridCtx;

/* Work out the placement and cursor leash from the terminal size and hex size.
   Re-run this on startup and after every resize. */
static void ctx_init(GridCtx *g, int rows, int cols,
                     double hex_size, double border_w,
                     bool show_hex, bool show_tri)
{
    g->rows = rows; g->cols = cols;
    g->hex_size = hex_size;
    g->cell_w = CELL_W; g->cell_h = CELL_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->border_w = border_w;
    g->show_hex = show_hex;
    g->show_tri = show_tri;

    /* Let the cursor reach any hex that fits on screen, plus a little slack. */
    g->max_q = (int)((cols * CELL_W) / (1.5 * hex_size)) + 2;
    g->max_r = (int)((rows * CELL_H) / (sqrt(3.0) * hex_size)) + 2;
}

/* Find the character cell at the centre of hex (q, r). Stepping q moves you
   east, stepping r moves you south-east; the same hex-to-screen math as
   01_flat_top. Rounding down lands the '@' inside the hex, not on a wall. */
static void ctx_to_screen(const GridCtx *g, int q, int r, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * 1.5      * (double)q;
    double cy_pix = g->hex_size * (sq3_2   * (double)q + sq3 * (double)r);
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/* Pick the dash/slash/pipe character that best matches the slope of a line at
   this angle, so the wall looks like it points the right way. (Same trick as
   01_flat_top, explained there.) */
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

/* How close a point is to the nearest triangle grid line: 0 means right on a
   line, 0.5 means as far from any line as you can get. (Same as 05_triangular.) */
static double edge_frac(double v)
{
    double t = fmod(v, 1.0);
    if (t < 0.0) t += 1.0;
    return t < 0.5 ? t : 1.0 - t;
}

/*
 * Draw the cyan hexagon walls — the first of the two grids. For each character
 * cell we ask "which hex are you in, and are you near its edge?" and draw a wall
 * piece if so. The hex the cursor sits in is drawn in the cursor color instead.
 * (The hex-finding math is the same as 01_flat_top.)
 *
 * This runs before the triangle grid, so wherever the two grids cross, the
 * triangle lines painted next will cover these cells.
 */
static void hex_layer(const GridCtx *g, int cur_q, int cur_r)
{
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - g->border_w;
    double size  = g->hex_size;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;

            int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
            double dq = fabs((double)rq - fq);
            double dr = fabs((double)rr - fr);
            double ds = fabs((double)rs - fs);
            if      (dq > dr && dq > ds) rq = -rr - rs;
            else if (dr > ds)             rr = -rq - rs;
            int Q = rq, R = rr;

            double fQ = (double)Q, fR = (double)R, fS = (double)(-Q - R);
            double dist = fabs(fq - fQ);
            double d2   = fabs(fr - fR);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;

            double cx = size * 1.5 * fQ;
            double cy = size * (sq3_2 * fQ + sq3 * fR);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);

            int on_cur = (Q == cur_q && R == cur_r);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_HEX)    | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/*
 * Draw the green triangle grid — the second of the two grids. The triangle grid
 * is just three sets of parallel lines crossing at 60 degrees: one flat, one
 * tilted one way, one tilted the other. For each cell we measure how close it is
 * to the nearest line in each set, and if it's close enough to any of them we
 * draw a wall sloped to match that set. Using the same size as the hexes makes
 * the corners line up.
 *
 * Painted after the hexes, so at crossings the green triangle lines sit on top.
 */
static void tri_layer(const GridCtx *g)
{
    /* spacing between neighbouring triangle lines */
    double h   = g->hex_size * sqrt(3.0) * 0.5;
    double sq3 = sqrt(3.0);

    attron(COLOR_PAIR(PAIR_TRI) | A_BOLD);
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            double n1 = py / h;
            double n2 = (sq3 * px + py) / h;
            double n3 = (-sq3 * px + py) / h;

            double d1 = edge_frac(n1);
            double d2 = edge_frac(n2);
            double d3 = edge_frac(n3);

            double dmin = d1;
            char   ch   = '-';
            if (d2 < dmin) { dmin = d2; ch = '/';  }
            if (d3 < dmin) { dmin = d3; ch = '\\'; }

            if (dmin < g->border_w) {
                mvaddch(row, col, (chtype)(unsigned char)ch);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_TRI) | A_BOLD);
}

/* Paint the whole background: hexes first, then triangles, each only if it's
   switched on. Only the hex pass cares about the cursor — the triangles are
   just decoration you can't walk on. */
static void ctx_draw_bg(const GridCtx *g, int cur_q, int cur_r)
{
    if (g->show_hex) hex_layer(g, cur_q, cur_r);
    if (g->show_tri) tri_layer(g);
}

/* ── §5 cursor ── */

/*
 * Cursor — which hex the '@' is sitting in, held as a (q, r) hex address.
 * It only travels the hex grid; the triangles are scenery. How far it can go
 * is stored over in GridCtx (max_q, max_r).
 */
typedef struct { int q, r; } Cursor;

/*
 * HEX_DIR — what one arrow-key press adds to (q, r), one row per direction:
 * up, down, left, right. Up/down nudge r; left/right nudge q. (Same table as
 * 01_flat_top.)
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/* Move the cursor by one step, but don't let it stray past its leash. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    int nq = cur->q + dq;
    int nr = cur->r + dr;
    if (nq >= -g->max_q && nq <= g->max_q) cur->q = nq;
    if (nr >= -g->max_r && nr <= g->max_r) cur->r = nr;
}

/* Stamp the '@' at the centre of the cursor's hex. Drawn last so it's always on
   top; since both grids only draw walls, the hex centre is empty and the '@'
   shows cleanly no matter which grids are on. */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->q, cur->r, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

/* The status readout (top-right) and the key hints (bottom). */
static void hud_draw(const GridCtx *g, const Cursor *cur,
                     int paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  size:%.0f  hex:%s  tri:%s  %5.1f fps  %s ",
             cur->q, cur->r, g->hex_size,
             g->show_hex ? "ON " : "OFF",
             g->show_tri ? "ON " : "OFF",
             fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  h:hex  t:tri  r:reset  arrows:move  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
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

    double hex_size = HEX_SIZE_DEFAULT;
    double border_w = BORDER_W_DEFAULT;
    bool   show_hex = true;
    bool   show_tri = true;
    int    paused   = 0;

    GridCtx g;     ctx_init(&g, LINES, COLS, hex_size, border_w,
                            show_hex, show_tri);
    Cursor  cur;   cursor_reset(&cur, &g);
    color_init();

    double  fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS, hex_size, border_w,
                     show_hex, show_tri);
            cursor_reset(&cur, &g);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 'h': show_hex = !show_hex; g.show_hex = show_hex; break;
            case 't': show_tri = !show_tri; g.show_tri = show_tri; break;
            case 'r': cursor_reset(&cur, &g); break;
            case KEY_UP:    cursor_move(&cur, &g, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (hex_size < HEX_SIZE_MAX) {
                    hex_size += HEX_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, hex_size, border_w,
                             show_hex, show_tri);
                }
                break;
            case '-':
                if (hex_size > HEX_SIZE_MIN) {
                    hex_size -= HEX_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, hex_size, border_w,
                             show_hex, show_tri);
                }
                break;
            case '[':
                if (border_w > BORDER_W_MIN) {
                    border_w -= BORDER_W_STEP;
                    g.border_w = border_w;
                }
                break;
            case ']':
                if (border_w < BORDER_W_MAX) {
                    border_w += BORDER_W_STEP;
                    g.border_w = border_w;
                }
                break;
            }
        }

        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0) {
            fps = fps * (1.0 - FPS_EWMA_ALPHA)
                + (1e9 / (double)dt) * FPS_EWMA_ALPHA;
        }

        scene_draw(&g, &cur, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
