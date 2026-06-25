/*
 * 05_triangular.c — a grid made of equilateral triangles you can move a cursor around.
 *
 * Arrow keys hop a '@' between neighbouring triangles; the one you're on is
 * highlighted. The cursor's spot is stored as two "stripe" numbers (si, sj),
 * a counting scheme made just for triangle grids.
 *
 * Sister file: hex_grids/01_flat_top.c — a hex grid is the flip side of this
 * tiling (one hex centre per triangle corner, and vice versa).
 *
 * How it works, in one breath: a rectangle grid is built from two sets of
 * parallel lines (across and down). A triangle grid needs THREE sets, running
 * flat, up-to-the-right, and up-to-the-left. For every screen cell we ask how
 * close it is to a line in each of the three sets; the nearest one decides
 * whether to draw an edge ('-', '/', '\') and which way it leans. There's no
 * stored grid array — we recompute everything from the cell's position each
 * frame.
 *
 * References (the kind the code can't show you):
 *   Triangular tiling     — https://mathworld.wolfram.com/TriangularGrid.html
 *   Hex/triangle flip side — https://www.redblobgames.com/grids/hexagons/#map-storage
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

/* One terminal cell counts as this many tiny "pixels" of room — the geometry
 * math works in pixels so the triangles aren't chained to character size. */
#define CELL_W              2
#define CELL_H              4

/* How big each triangle is, in pixels. The one knob that sets the scale. */
#define TRI_SIZE_DEFAULT   20.0
#define TRI_SIZE_MIN        8.0
#define TRI_SIZE_MAX       60.0
#define TRI_SIZE_STEP       4.0

/* How close to a line counts as "on the border". 0 = right on the line,
 * 0.5 = dead centre of a triangle. Bigger value = fatter drawn edges. */
#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.02
#define BORDER_W_MAX        0.45
#define BORDER_W_STEP       0.02

#define N_THEMES            4
#define TICK_NS            16666667LL    /* one frame at ~60 per second */

/* The on-screen fps reading is smoothed so it doesn't jitter; smaller = calmer. */
#define FPS_EWMA_ALPHA      0.05

/* Colour slots */
#define PAIR_BORDER   1   /* the triangle edges (takes the theme colour) */
#define PAIR_CURSOR   2   /* the highlighted triangle (white on blue) */
#define PAIR_HUD      3   /* status line (yellow) */
#define PAIR_HINT     4   /* key reminders at the bottom (cyan) */

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

/* ── §4 formula — the triangle grid and how a screen cell maps onto it ── */

/*
 * GridCtx — everything we need to know about the current triangle grid:
 * how big the triangles are, how big the terminal is, and how far the cursor
 * is allowed to roam.
 *
 * Why bundle it instead of using globals? So the math functions take a grid
 * as an argument and work for any grid — the same shape is reused across the
 * rectangle/hex/triangle/polar demos in this folder.
 */
typedef struct {
    /* size of the terminal, counted in character cells */
    int rows, cols;

    /* tri_size: triangle side length in pixels.
     * cell_w/cell_h: how many pixels one terminal cell is worth. */
    double tri_size;
    int    cell_w, cell_h;

    /* where the grid's centre sits, as a cell position — used to centre
     * the whole tiling on screen */
    int ox, oy;

    /* how far the cursor may travel, in stripe-number space. Kept generous
     * so the cursor can reach any triangle that's visible; going past the
     * edge just parks it off-screen. */
    int max_si, max_sj;

    /* the "how close counts as an edge" threshold (see BORDER_W in §1) */
    double border_w;
} GridCtx;

/* Fills in a GridCtx for the current terminal size and triangle size.
 * Call it again after a resize or after the user changes the triangle size. */
static void ctx_init(GridCtx *g, int rows, int cols,
                     double tri_size, double border_w)
{
    g->rows = rows; g->cols = cols;
    g->tri_size = tri_size;
    g->cell_w = CELL_W; g->cell_h = CELL_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->border_w = border_w;

    /* Pick cursor limits big enough to cover the whole screen with room to
     * spare — rough is fine, since stepping off-screen does no harm. */
    int max_pix_w = cols * CELL_W;
    int max_pix_h = rows * CELL_H;
    double h = tri_size * sqrt(3.0) * 0.5;
    g->max_si = (int)(max_pix_h / h) + 4;
    g->max_sj = (int)((max_pix_w + max_pix_h) / h) + 4;
}

/* Given a triangle's two stripe numbers (si, sj), find the screen cell at its
 * centre. si picks the row going down; sj picks how far along that row to go.
 * This is the reverse of the per-pixel stripe math done in ctx_draw_bg(). */
static void ctx_to_screen(const GridCtx *g, int si, int sj, int *sr, int *sc)
{
    double h    = g->tri_size * sqrt(3.0) * 0.5;
    double py_c = ((double)si + 0.5) * h;
    double px_c = (double)(sj - si) * g->tri_size * 0.5;
    *sr = g->oy + (int)(py_c / g->cell_h);
    *sc = g->ox + (int)(px_c / g->cell_w);
}

/* How close is v to the nearest whole number? Returns 0 right on a line and
 * 0.5 at the midpoint between two. We use it to tell "near an edge" from
 * "deep inside a triangle". (The +1 fixes C's fmod, which can go negative.) */
static double edge_frac(double v)
{
    double t = fmod(v, 1.0);
    if (t < 0.0) t += 1.0;
    return t < 0.5 ? t : 1.0 - t;
}

/*
 * Draws the whole triangle grid and shades the triangle the cursor sits on.
 *
 * The idea: for each screen cell, measure its distance to the nearest line in
 * each of the three line-sets (flat, up-right, up-left). Whichever set is
 * closest wins and decides the edge character ('-', '/', or '\', leaning the
 * same way the real line does). If the closest distance is small, we're near
 * an edge and draw it; otherwise we're inside a triangle and leave it blank —
 * unless it's the cursor's triangle, which gets a coloured fill.
 *
 * The cursor's own (si, sj) is passed in; this grid type doesn't store it.
 */
static void ctx_draw_bg(const GridCtx *g, int cur_si, int cur_sj)
{
    double h   = g->tri_size * sqrt(3.0) * 0.5;
    double sq3 = sqrt(3.0);

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            /* position relative to each of the three line-sets */
            double n1 = py / h;
            double n2 = (sq3 * px + py) / h;
            double n3 = (-sq3 * px + py) / h;

            /* which triangle is this cell in? two of the three numbers,
             * rounded down, name it uniquely */
            int cell_si = (int)floor(n1);
            int cell_sj = (int)floor(n2);
            int on_cur  = (cell_si == cur_si && cell_sj == cur_sj);

            double d1 = edge_frac(n1);
            double d2 = edge_frac(n2);
            double d3 = edge_frac(n3);

            /* keep the closest line; its character leans the matching way */
            double dmin = d1;
            char   ch   = '-';
            if (d2 < dmin) { dmin = d2; ch = '/';  }
            if (d3 < dmin) { dmin = d3; ch = '\\'; }

            if (dmin < g->border_w) {
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
            } else if (on_cur) {
                /* inside the cursor's triangle: paint the highlight */
                attron(COLOR_PAIR(PAIR_CURSOR));
                mvaddch(row, col, ' ');
                attroff(COLOR_PAIR(PAIR_CURSOR));
            }
        }
    }
}

/* ── §5 cursor ── */

/*
 * Cursor — where the user is pointing, as the two stripe numbers (si, sj)
 * that name one triangle.
 *
 * si: which row of triangles (counts down the screen).
 * sj: which diagonal stripe along that row.
 *
 * It deliberately knows nothing about grid size — the limits live in GridCtx.
 * Hand both to ctx_to_screen() to turn (si, sj) into a screen position.
 */
typedef struct { int si, sj; } Cursor;

/*
 * TRI_DIR — the four arrow-key moves, written as changes to (si, sj).
 *
 *           UP: si-1
 *              ^
 *   LEFT: sj-1 < @ > RIGHT: sj+1
 *              v
 *          DOWN: si+1
 *
 * Up/down step to the next row of triangles (crossing a flat edge); left/right
 * step along the diagonal (crossing a slanted edge). Either way you land on a
 * triangle that shares an edge with the one you left — never a diagonal jump.
 *
 * Note: this is its own counting scheme, unrelated to the (Q,R) scheme hex
 * grids use, so don't reach for the hex movement table here.
 */
static const int TRI_DIR[4][2] = {
    {-1,  0 },   /* UP */
    {+1,  0 },   /* DOWN */
    { 0, -1 },   /* LEFT */
    { 0, +1 },   /* RIGHT */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->si = 0;
    cur->sj = 0;
}

/* Nudges the cursor by one step, refusing to wander past the grid limits. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dsi, int dsj)
{
    int nsi = cur->si + dsi;
    int nsj = cur->sj + dsj;
    if (nsi >= -g->max_si && nsi <= g->max_si) cur->si = nsi;
    if (nsj >= -g->max_sj && nsj <= g->max_sj) cur->sj = nsj;
}

/* Drops a bold '@' on the cursor's triangle, but only if it's actually
 * on screen. */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->si, cur->sj, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

/* Draws the two text overlays: a status line top-right and the key reminders
 * along the bottom. */
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
    ctx_draw_bg(g, cur->si, cur->sj);
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
