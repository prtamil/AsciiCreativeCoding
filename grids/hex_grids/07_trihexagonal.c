/*
 * 07_trihexagonal.c — a flat-top hex grid and a triangle grid drawn on top of
 * each other, so their corners coincide: the trihexagonal (star-of-David) tiling.
 *
 * The hex layer reuses the §4 vocabulary of 01_flat_top.c verbatim. The triangle
 * layer is the one genuinely different step (see tri_line_distance / draw_triangles):
 * three families of parallel lines at 60°, spaced to share the hexes' corners.
 *
 * Sister files: 01_flat_top.c (hex math), 05_triangular.c (triangle math).
 * Trihexagonal tiling: en.wikipedia.org/wiki/Trihexagonal_tiling
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
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

#define BORDER_W_DEFAULT    0.10   /* line thickness, shared by both grids */
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35
#define BORDER_W_STEP       0.02

#define TICK_NS            16666667LL

#define FPS_EWMA_ALPHA      0.05   /* small = steadier on-screen fps number */

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
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
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

/* ── §4 hex mapping & lattice ── */

/* GridCtx — the whole tiling for one frame. The (q,r)=(0,0) hex sits at screen
 * centre; the triangle grid shares the same size so their corners coincide. The
 * two on/off toggles live here because the lattice painter reads them. */
typedef struct {
    int    rows, cols;      /* terminal size in character cells */
    double hex_size;        /* centre-to-corner distance, pixels; bigger = fewer hexes */
    double border_w;        /* outline band width, 0..0.5; how near an edge counts as edge */
    int    cell_w, cell_h;  /* pixels per char cell (2 vs 4) — undoes the cell's tall aspect */
    int    ox, oy;          /* screen-centre cell; the (0,0) hex lives here */
    int    max_q, max_r;    /* cursor leash, in hex steps each way from centre */
    bool   show_hex;        /* is the hex grid switched on? */
    bool   show_tri;        /* is the triangle grid switched on? */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols,
                     double hex_size, double border_w,
                     bool show_hex, bool show_tri)
{
    g->rows   = rows;
    g->cols   = cols;
    g->hex_size = hex_size;
    g->border_w = border_w;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = (rows - 1) / 2;
    g->show_hex = show_hex;
    g->show_tri = show_tri;
    g->max_q = (int)((cols * CELL_W) / (1.5 * hex_size)) + 2;
    g->max_r = (int)((rows * CELL_H) / (sqrt(3.0) * hex_size)) + 2;
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
 * '|' steep, '/' and '\' between. Folded into [0,pi) since glyphs look the same
 * flipped 180°. */
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

/* recipe step 2a — the cyan hex grid, identical to 01_flat_top: for every cell,
 * find its hex and how near an edge it sits; near an edge -> an outline glyph.
 * The cursor's hex is drawn in its own colour. Runs first, so the triangle pass
 * overpaints it at crossings. */
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

            int attr = (q == cur_q && r == cur_r)
                       ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                       : (COLOR_PAIR(PAIR_HEX)    | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* how near a triangle-grid line a coordinate is: 0 right on a line, 0.5 furthest
 * from any line. Each line family is one such coordinate; near any of them = a
 * wall. */
static double tri_line_distance(double n)
{
    double t = fmod(n, 1.0);
    if (t < 0.0) t += 1.0;
    return t < 0.5 ? t : 1.0 - t;
}

/* recipe step 2b — the distinct trihexagonal step: the green triangle grid, three
 * families of parallel lines at 60° (one flat, two tilted). The spacing h = the
 * hexes' apothem, so the triangle corners land on the hex corners. For each cell
 * we take the smallest distance to the three families; near enough -> a wall
 * sloped to match. Painted after the hexes, so it sits on top at crossings. */
static void draw_triangles(const GridCtx *g)
{
    double h   = g->hex_size * sqrt(3.0) * 0.5;   /* line spacing = hex apothem */
    double sq3 = sqrt(3.0);

    attron(COLOR_PAIR(PAIR_TRI) | A_BOLD);
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            double d_flat = tri_line_distance(py / h);
            double d_up   = tri_line_distance(( sq3 * px + py) / h);
            double d_down = tri_line_distance((-sq3 * px + py) / h);

            double dmin = d_flat;
            char   ch   = '-';
            if (d_up   < dmin) { dmin = d_up;   ch = '/';  }
            if (d_down < dmin) { dmin = d_down; ch = '\\'; }

            if (dmin < g->border_w)
                mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_TRI) | A_BOLD);
}

/* paint both grids, each only if switched on; hexes first so triangles overpaint */
static void draw_grid(const GridCtx *g, int cur_q, int cur_r)
{
    if (g->show_hex) draw_lattice(g, cur_q, cur_r);
    if (g->show_tri) draw_triangles(g);
}

/* ── §5 cursor ── */

/* Cursor — which hex is selected, named by its (q,r). It only travels the hex
 * grid; the triangles are scenery. The leash is in GridCtx (max_q, max_r). */
typedef struct { int q, r; } Cursor;

/* what each arrow key adds to (q,r) to step onto a neighbouring hex; same four
 * steps as sister grids 01, 06. */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* up    */
    { 0, +1 },   /* down  */
    {-1,  0 },   /* left  */
    {+1,  0 },   /* right */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/* recipe step 3 — move the cursor by one hex, clamped to the leash. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    int nq = cur->q + dq;
    int nr = cur->r + dr;
    if (nq >= -g->max_q && nq <= g->max_q) cur->q = nq;
    if (nr >= -g->max_r && nr <= g->max_r) cur->r = nr;
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
    draw_grid(g, cur->q, cur->r);
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
            /* endwin/refresh is how ncurses picks up the new terminal size */
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
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
