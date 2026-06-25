/*
 * 03_axial.c — a flat-top hex grid with every hexagon labeled by its (Q,R)
 * axial coordinates. Drive the cursor with the arrows; each coordinate axis
 * (Q=0, R=0, S=0) glows a different color. S = -Q-R is derived, never stored.
 *
 * Sister file: grids/hex_grids/01_flat_top.c — same grid, mapping, and cursor;
 *              this one adds axis coloring and coordinate labels on top.
 * Reference:   https://www.redblobgames.com/grids/hexagons/#coordinates-axial
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

#define HEX_SIZE_DEFAULT  20.0
#define HEX_SIZE_MIN      10.0
#define HEX_SIZE_MAX      50.0
#define HEX_SIZE_STEP      2.0

#define BORDER_W_DEFAULT   0.08
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.25
#define BORDER_W_STEP      0.02

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

#define PAIR_DEFAULT  1
#define PAIR_Q_AXIS   2   /* cyan   — hexes where Q is 0   */
#define PAIR_R_AXIS   3   /* green  — hexes where R is 0   */
#define PAIR_S_AXIS   4   /* yellow — hexes where S is 0   */
#define PAIR_ORIGIN   5   /* white  — the (0,0) hex        */
#define PAIR_CURSOR   6   /* selected hex: white on blue   */
#define PAIR_HUD      7
#define PAIR_HINT     8

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_DEFAULT, COLOR_WHITE,  COLOR_BLACK);
    init_pair(PAIR_Q_AXIS,  COLOR_CYAN,   COLOR_BLACK);
    init_pair(PAIR_R_AXIS,  COLOR_GREEN,  COLOR_BLACK);
    init_pair(PAIR_S_AXIS,  COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_ORIGIN,  COLOR_WHITE,  COLOR_BLACK);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,     COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,    COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* which color a hex gets. First match wins: the cursor is checked before any
 * axis so a cursor parked on an axis still reads as the cursor. */
static int hex_color(int q, int r, int cur_q, int cur_r)
{
    if (q == cur_q && r == cur_r) return PAIR_CURSOR;
    if (q == 0 && r == 0)         return PAIR_ORIGIN;
    if (q == 0)                   return PAIR_Q_AXIS;
    if (r == 0)                   return PAIR_R_AXIS;
    if (-q - r == 0)              return PAIR_S_AXIS;
    return PAIR_DEFAULT;
}

/* ── §4 hex mapping & lattice ── */

/* GridCtx — the whole hex grid for one frame. The (q,r)=(0,0) hex sits at
 * screen centre; every other hex is measured outward from it. Same layout as
 * 01_flat_top.c. */
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

/* the ASCII glyph whose slant lies along an edge at angle theta: '-' flattish,
 * '|' steep, '/' and '\' between. Glyphs look the same flipped 180°, so we
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

/* recipe step 2a — draw the hex outlines: for every screen cell, find its hex
 * and how near an edge it sits. Near an edge -> an outline glyph tinted by the
 * hex's axis; deep inside -> blank, leaving room for the labels in step 2b. */
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

            if (hex_edge_distance(fq, fr, fs, q, r) < limit) continue;  /* inside the hex */

            double cx, cy;
            hex_center_pixel(g->hex_size, q, r, &cx, &cy);
            char ch = edge_glyph(atan2(py - cy, px - cx) + M_PI / 2.0);

            int pair = hex_color(q, r, cur_q, cur_r);
            int attr = (pair == PAIR_ORIGIN || pair == PAIR_CURSOR)
                       ? (COLOR_PAIR(pair) | A_BOLD) : COLOR_PAIR(pair);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* recipe step 2b — stamp each hex's "q,r" label in its middle. Sweep a (q,r)
 * range wide enough to cover the screen, skipping any whose centre lands
 * off-screen. Cursor/origin labels are bold; ordinary labels dimmed so the
 * coloured outlines stay dominant. */
static void draw_labels(const GridCtx *g, int cur_q, int cur_r)
{
    int q_max = (int)(g->cols * g->cell_w / (1.5 * g->hex_size)) + 3;
    int r_max = (int)(g->rows * g->cell_h / (sqrt(3.0) * g->hex_size)) + 3;

    for (int q = -q_max; q <= q_max; q++) {
        for (int r = -r_max; r <= r_max; r++) {
            int sr, sc;
            axial_to_screen(g, q, r, &sr, &sc);
            if (sr < 1 || sr >= g->rows - 1) continue;
            if (sc < 2 || sc >= g->cols - 2) continue;

            char buf[12];
            int len = snprintf(buf, sizeof buf, "%d,%d", q, r);
            int lx  = sc - len / 2;
            if (lx < 0 || lx + len >= g->cols) continue;

            int pair = hex_color(q, r, cur_q, cur_r);
            int attr = (pair == PAIR_CURSOR) ? (COLOR_PAIR(pair) | A_BOLD | A_REVERSE)
                     : (pair == PAIR_ORIGIN) ? (COLOR_PAIR(pair) | A_BOLD)
                     :                          (COLOR_PAIR(pair) | A_DIM);
            attron(attr);
            mvprintw(sr, lx, "%s", buf);
            attroff(attr);
        }
    }
}

/* ── §5 cursor ── */

/* Cursor — which hex is selected, named by its (q,r). The third coordinate is
 * always -q-r, so it isn't stored. Same struct as 01_flat_top.c. */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/* what each arrow key adds to (q,r): right/left change q, down/up change r, so
 * a label's two numbers tick up and down as you move. Same table as 01. */
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

/* drop the '@' on the selected hex; called after the labels so it lands on top */
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
    int s = -cur->q - cur->r;
    char buf[96];
    snprintf(buf, sizeof buf,
             " cursor Q:%+d R:%+d S:%+d  size:%.0f  %5.1f fps  %s ",
             cur->q, cur->r, s, g->hex_size, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  r:reset  arrows:move  +/-:size  [/]:border "
             " cyan=Q  green=R  yellow=S  white=origin ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int paused, double fps)
{
    erase();
    draw_lattice(g, cur->q, cur->r);   /* step 2a: outlines */
    draw_labels (g, cur->q, cur->r);   /* step 2b: labels   */
    cursor_draw (cur, g);              /* step 3:  '@'      */
    hud_draw    (g, cur, paused, fps);
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
