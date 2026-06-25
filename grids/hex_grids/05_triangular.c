/*
 * 05_triangular.c — a screen of equilateral triangles with a movable cursor.
 *
 * No grid is stored. A triangle tiling is three sets of parallel lines (flat,
 * up-right, up-left); for each screen cell we ask "which stripe of each set am
 * I in, and is this pixel near a line?" and answer with arithmetic — that is §4.
 *
 * Sister file: hex_grids/01_flat_top.c — same skeleton; a hex grid is the flip
 * side of this tiling. Only the mapping math differs.
 * Reference:   https://mathworld.wolfram.com/TriangularGrid.html
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

/* ── §1 config ── */

#define CELL_W              2
#define CELL_H              4

#define TRI_SIZE_DEFAULT   20.0   /* triangle side length, pixels; bigger = fewer triangles */
#define TRI_SIZE_MIN        8.0
#define TRI_SIZE_MAX       60.0
#define TRI_SIZE_STEP       4.0

#define BORDER_W_DEFAULT    0.10  /* how near a line counts as edge: 0 on the line, 0.5 mid-triangle */
#define BORDER_W_MIN        0.02
#define BORDER_W_MAX        0.45
#define BORDER_W_STEP       0.02

#define N_THEMES            4
#define TICK_NS            16666667LL

#define FPS_EWMA_ALPHA      0.05   /* small = steadier on-screen fps number */

#define PAIR_BORDER   1
#define PAIR_CURSOR   2   /* selected triangle's edges and its '@' */
#define PAIR_HUD      3
#define PAIR_HINT     4

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
    init_pair(PAIR_CURSOR, COLOR_WHITE,      COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 triangle mapping & lattice ── */

/* GridCtx — the whole triangle grid for one frame. Stripe (si,sj)=(0,0) sits at
 * screen centre; every other triangle is named by its stripe numbers measured
 * from there. Keys +/- and [/] write tri_size / border_w; the rest is
 * recomputed on resize. */
typedef struct {
    int    rows, cols;      /* terminal size in character cells */
    double tri_size;        /* triangle side length, pixels; bigger = fewer triangles */
    double border_w;        /* outline band width, 0..0.5; how near a line counts as edge */
    int    cell_w, cell_h;  /* pixels per char cell (2 vs 4) — undoes the cell's tall aspect */
    int    ox, oy;          /* screen-centre cell; the (0,0) triangle lives here */
    int    max_si, max_sj;  /* furthest stripe the cursor may reach; off past it is harmless */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols,
                     double tri_size, double border_w)
{
    g->rows     = rows;
    g->cols     = cols;
    g->tri_size = tri_size;
    g->border_w = border_w;
    g->cell_w   = CELL_W;
    g->cell_h   = CELL_H;
    g->ox       = cols / 2;
    g->oy       = (rows - 1) / 2;
    /* generous stripe limits — enough to cover the screen with room to spare */
    double h = tri_size * sqrt(3.0) * 0.5;
    g->max_si = (int)((double)(rows * CELL_H) / h) + 4;
    g->max_sj = (int)((double)((cols * CELL_W) + (rows * CELL_H)) / h) + 4;
}

/* one triangle's centre in pixels, measured from the grid centre. si counts
 * rows down (row height h); sj counts diagonal steps along the row. */
static void tri_center_pixel(double size, int si, int sj, double *cx, double *cy)
{
    double h = size * sqrt(3.0) * 0.5;
    *cx = (double)(sj - si) * size * 0.5;
    *cy = ((double)si + 0.5) * h;
}

/* recipe step 1 — triangle address (si,sj) -> the screen cell at its centre */
static void tri_to_screen(const GridCtx *g, int si, int sj, int *sr, int *sc)
{
    double cx, cy;
    tri_center_pixel(g->tri_size, si, sj, &cx, &cy);
    *sc = g->ox + (int)(cx / g->cell_w);
    *sr = g->oy + (int)(cy / g->cell_h);
}

/* the reverse — a pixel offset from grid centre -> position along each of the
 * three line-sets (flat, up-right, up-left), counted in row-heights. */
static void screen_to_tristripe(const GridCtx *g, double px, double py,
                                double *n1, double *n2, double *n3)
{
    double h   = g->tri_size * sqrt(3.0) * 0.5;
    double sq3 = sqrt(3.0);
    *n1 =          py  / h;   /* flat lines     */
    *n2 = ( sq3 * px + py) / h;   /* up-right lines */
    *n3 = (-sq3 * px + py) / h;   /* up-left lines  */
}

/* how near a line a stripe value sits: 0 right on a line, 0.5 mid-triangle.
 * (The +1 folds C's fmod, which can go negative, back into [0,1).) */
static double edge_frac(double v)
{
    double t = fmod(v, 1.0);
    if (t < 0.0) t += 1.0;
    return t < 0.5 ? t : 1.0 - t;
}

/* the ASCII glyph for whichever line-set this cell is closest to: '-' flat,
 * '/' up-right, '\' up-left. Returns the closest distance via *dmin. */
static char edge_glyph(double n1, double n2, double n3, double *dmin)
{
    double d1 = edge_frac(n1), d2 = edge_frac(n2), d3 = edge_frac(n3);
    double d = d1;
    char   ch = '-';
    if (d2 < d) { d = d2; ch = '/';  }
    if (d3 < d) { d = d3; ch = '\\'; }
    *dmin = d;
    return ch;
}

/* recipe step 2 — draw the grid: for every screen cell, find its stripes and
 * how near a line it sits. Near a line -> an edge glyph; deep inside -> blank.
 * The cursor's triangle is filled and its edges drawn in its own colour. */
static void draw_lattice(const GridCtx *g, int cur_si, int cur_sj)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            double n1, n2, n3;
            screen_to_tristripe(g, px, py, &n1, &n2, &n3);

            /* two stripes floored name the triangle uniquely */
            int si = (int)floor(n1);
            int sj = (int)floor(n2);
            int on_cur = (si == cur_si && sj == cur_sj);

            double dmin;
            char ch = edge_glyph(n1, n2, n3, &dmin);

            if (dmin < g->border_w) {
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
            } else if (on_cur) {
                attron(COLOR_PAIR(PAIR_CURSOR));
                mvaddch(row, col, ' ');
                attroff(COLOR_PAIR(PAIR_CURSOR));
            }
        }
    }
}

/* ── §5 cursor ── */

/* Cursor — which triangle is selected, named by its stripe numbers (si, sj):
 * si counts rows down the screen, sj the diagonal step along that row. Its own
 * scheme — not the hex (q,r). Pair with a GridCtx to find it on screen. */
typedef struct { int si, sj; } Cursor;

/* what each arrow key adds to (si,sj) to step onto an edge-sharing neighbour.
 * up/down cross a flat edge to the next row; left/right cross a slanted edge
 * along the diagonal — never a diagonal jump. */
static const int TRI_DIR[4][2] = {
    {-1,  0 },   /* up    */
    {+1,  0 },   /* down  */
    { 0, -1 },   /* left  */
    { 0, +1 },   /* right */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->si = 0;
    cur->sj = 0;
}

/* recipe step 3 — move the cursor by one triangle, clamped to grid limits */
static void cursor_move(Cursor *cur, const GridCtx *g, int dsi, int dsj)
{
    int nsi = cur->si + dsi;
    int nsj = cur->sj + dsj;
    if (nsi >= -g->max_si && nsi <= g->max_si) cur->si = nsi;
    if (nsj >= -g->max_sj && nsj <= g->max_sj) cur->sj = nsj;
}

/* drop the '@' on the selected triangle; called after the grid so it lands on top */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    tri_to_screen(g, cur->si, cur->sj, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur,
                     int theme, int paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " stripe si:%+d sj:%+d  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->si, cur->sj, g->tri_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur,
                       int theme, int paused, double fps)
{
    erase();
    draw_lattice(g, cur->si, cur->sj);
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

    double tri_size = TRI_SIZE_DEFAULT;
    double border_w = BORDER_W_DEFAULT;
    int    theme    = 0;
    int    paused   = 0;

    GridCtx g;     ctx_init(&g, LINES, COLS, tri_size, border_w);
    Cursor  cur;   cursor_reset(&cur, &g);
    color_init(theme);

    double  fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS, tri_size, border_w);
            cursor_reset(&cur, &g);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 'r': cursor_reset(&cur, &g); break;
            case 't':
                theme = (theme + 1) % N_THEMES;
                color_init(theme);
                break;
            /* arrow keys move the cursor by one step (see TRI_DIR in §5) */
            case KEY_UP:    cursor_move(&cur, &g, TRI_DIR[0][0], TRI_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, TRI_DIR[1][0], TRI_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, TRI_DIR[2][0], TRI_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, TRI_DIR[3][0], TRI_DIR[3][1]); break;
            case '+': case '=':
                if (tri_size < TRI_SIZE_MAX) {
                    tri_size += TRI_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, tri_size, border_w);
                }
                break;
            case '-':
                if (tri_size > TRI_SIZE_MIN) {
                    tri_size -= TRI_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, tri_size, border_w);
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

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
