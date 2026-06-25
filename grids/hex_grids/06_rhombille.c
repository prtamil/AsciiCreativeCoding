/*
 * 06_rhombille.c — rhombille tiling: each flat-top hex split into 3 rhombi.
 *
 * Same flat-top hex lattice as 01_flat_top.c, but inside every hex we also draw
 * three spokes from the centre out to alternate corners (120° apart). Those
 * spokes carve each hex into three diamonds; the diamonds of neighbouring hexes
 * line up into the classic "stack of cubes" illusion. An '@' marks the selected
 * hex and moves with the arrows.
 *
 * Sister files: 01_flat_top.c (the hex border this builds on),
 *               05_triangular.c (the triangular cousin of this tiling).
 * Reference:    Rhombille tiling, https://en.wikipedia.org/wiki/Rhombille_tiling
 *               Red Blob Games hex guide, https://www.redblobgames.com/grids/hexagons/
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <signal.h>
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

#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.30
#define BORDER_W_STEP       0.02

#define SPOKE_W_DEFAULT     2.5    /* half-thickness of an inner spoke, pixels */
#define SPOKE_W_MIN         0.5
#define SPOKE_W_MAX         6.0
#define SPOKE_W_STEP        0.5

#define N_THEMES            4
#define TICK_NS            16666667LL

#define FPS_EWMA_ALPHA      0.05   /* small = steadier on-screen fps number */

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

#define PAIR_BORDER   1
#define PAIR_SPOKE    2   /* the three diamond-splitting lines inside each hex */
#define PAIR_CURSOR   3   /* selected hex's outline, spokes, and its '@' */
#define PAIR_HUD      4
#define PAIR_HINT     5

static const short THEMES[N_THEMES][2] = {
    { COLOR_CYAN,   COLOR_BLACK },
    { COLOR_GREEN,  COLOR_BLACK },
    { COLOR_YELLOW, COLOR_BLACK },
    { COLOR_WHITE,  COLOR_BLACK },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_BORDER, THEMES[theme][0], THEMES[theme][1]);
    init_pair(PAIR_SPOKE,  THEMES[theme][0], THEMES[theme][1]);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 hex mapping & lattice ── */

/* GridCtx — the whole hex grid for one frame. The (q,r)=(0,0) hex sits at
 * screen centre; every other hex is measured outward from it. Keys +/- write
 * hex_size and [/] writes spoke_w_px; the rest is recomputed on resize. */
typedef struct {
    int    rows, cols;      /* terminal size in character cells */
    double hex_size;        /* centre-to-corner distance, pixels; bigger = fewer hexes */
    double border_w;        /* outline band width, 0..0.5; how near an edge counts as edge */
    double spoke_w_px;      /* half-thickness of an inner spoke, pixels */
    int    cell_w, cell_h;  /* pixels per char cell (2 vs 4) — undoes the cell's tall aspect */
    int    ox, oy;          /* screen-centre cell; the (0,0) hex lives here */
    int    max_q, max_r;    /* cursor roam limit in each axis (generous) */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols,
                     double hex_size, double border_w, double spoke_w_px)
{
    g->rows       = rows;
    g->cols       = cols;
    g->hex_size   = hex_size;
    g->border_w   = border_w;
    g->spoke_w_px = spoke_w_px;
    g->cell_w     = CELL_W;
    g->cell_h     = CELL_H;
    g->ox         = cols / 2;
    g->oy         = (rows - 1) / 2;
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

/* The three rhombille spokes: lines from the hex centre out to alternate
 * corners, 120° apart. SPOKE_DIR is each spoke's unit direction on screen
 * (right, up-left, down-left — y points down, so these aren't textbook angles);
 * SPOKE_GLYPH is the line glyph that points that way. Drawing these three is
 * what splits one hex into three rhombi. */
static const double SPOKE_DIR[3][2] = {
    {  1.0,  0.0       },   /* right     -> '-'  */
    { -0.5, -0.8660254 },   /* up-left   -> '\\' */
    { -0.5,  0.8660254 },   /* down-left -> '/'  */
};
static const char SPOKE_GLYPH[3] = { '-', '\\', '/' };

/* which spoke (if any) a pixel offset (vx,vy) from the hex centre lies on:
 * project onto each spoke direction; on it when the projection runs from centre
 * (0) to corner (size) and the perpendicular distance is within the band.
 * Returns the spoke index 0..2, or -1 if the pixel is on no spoke. */
static int spoke_at(double vx, double vy, double size, double half_w)
{
    for (int k = 0; k < 3; k++) {
        double along = vx * SPOKE_DIR[k][0] + vy * SPOKE_DIR[k][1];
        double perp  = fabs(vx * SPOKE_DIR[k][1] - vy * SPOKE_DIR[k][0]);
        if (along >= 0.0 && along <= size && perp <= half_w) return k;
    }
    return -1;
}

/* recipe step 2 — draw the grid: for every screen cell find its hex. Near the
 * rim -> outline glyph; deep inside -> test the three spokes and draw one if the
 * cell sits on it (this is the rhombille split). The cursor's hex uses its own
 * colour. */
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

            double cx, cy;
            hex_center_pixel(g->hex_size, q, r, &cx, &cy);
            int on_cur = (q == cur_q && r == cur_r);

            if (hex_edge_distance(fq, fr, fs, q, r) >= limit) {
                char ch = edge_glyph(atan2(py - cy, px - cx) + M_PI / 2.0);
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
            } else {
                int k = spoke_at(px - cx, py - cy, g->hex_size, g->spoke_w_px);
                if (k >= 0) {
                    int attr = on_cur ? COLOR_PAIR(PAIR_CURSOR)
                                      : COLOR_PAIR(PAIR_SPOKE);
                    attron(attr);
                    mvaddch(row, col, (chtype)(unsigned char)SPOKE_GLYPH[k]);
                    attroff(attr);
                }
            }
        }
    }
}

/* ── §5 cursor ── */

/* Cursor — which hex is selected, named by its (q,r). The third coordinate is
 * always -q-r, so it isn't stored. Pair with a GridCtx to find it on screen. */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/* what each arrow key adds to (q,r) to step onto a neighbouring hex. The same
 * four steps work for sister grids 01, 02, 07. */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* up    */
    { 0, +1 },   /* down  */
    {-1,  0 },   /* left  */
    {+1,  0 },   /* right */
};

/* recipe step 3 — move the cursor by one hex, refusing to step past the roam limits. */
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

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  size:%.0f  spoke:%.1f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, g->hex_size, g->spoke_w_px, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:spoke ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    draw_lattice(g, cur->q, cur->r);
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

    double hex_size = HEX_SIZE_DEFAULT;
    double border_w = BORDER_W_DEFAULT;
    double spoke_w  = SPOKE_W_DEFAULT;
    int    theme    = 0;
    int    paused   = 0;

    GridCtx g;   ctx_init(&g, LINES, COLS, hex_size, border_w, spoke_w);
    Cursor  cur; cursor_reset(&cur, &g);
    color_init(theme);

    double  fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            /* endwin/refresh is how ncurses picks up the new terminal size */
            endwin(); refresh();
            ctx_init(&g, LINES, COLS, hex_size, border_w, spoke_w);
            cursor_reset(&cur, &g);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 't':
                theme = (theme + 1) % N_THEMES;
                color_init(theme);
                break;
            case 'r': cursor_reset(&cur, &g); break;
            case KEY_UP:    cursor_move(&cur, &g, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (hex_size < HEX_SIZE_MAX) {
                    hex_size += HEX_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, hex_size, border_w, spoke_w);
                }
                break;
            case '-':
                if (hex_size > HEX_SIZE_MIN) {
                    hex_size -= HEX_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, hex_size, border_w, spoke_w);
                }
                break;
            case '[':
                if (spoke_w > SPOKE_W_MIN) { spoke_w -= SPOKE_W_STEP; g.spoke_w_px = spoke_w; }
                break;
            case ']':
                if (spoke_w < SPOKE_W_MAX) { spoke_w += SPOKE_W_STEP; g.spoke_w_px = spoke_w; }
                break;
            }
        }

        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
