/*
 * 02_pointy_top.c — a pointy-top hex grid you can move a cursor around with arrow keys.
 *
 * Same idea as 01_flat_top.c, but each hexagon is turned 30° so a corner points
 * straight up and the flat sides face left and right. Everything about the hex
 * coordinate math is the same; only the little formula that turns screen pixels
 * into hex coordinates (and back) is different.
 *
 * Sister file: grids/hex_grids/01_flat_top.c (read it first — same cursor, same keys).
 * The pointy-top formulas come from redblobgames.com/grids/hexagons/#hex-to-pixel-pointy
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

#define BORDER_W_DEFAULT   0.10
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.35
#define BORDER_W_STEP      0.02

#define N_THEMES           4
#define TICK_NS           16666667LL

/* How much each new frame's speed nudges the FPS number on screen, so it
 * reads as a smooth average instead of jittering frame to frame. */
#define FPS_EWMA_ALPHA     0.05

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
#define PAIR_CURSOR   2
#define PAIR_HUD      3   /* yellow status bar */
#define PAIR_HINT     4   /* cyan key-hint footer */

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
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — turning hex coordinates into screen positions, and back ── */

/*
 * GridCtx — everything we need to know to draw the current grid: how big the
 * terminal is, how big each hex is, and where the grid is centered on screen.
 * This is the one place that knows the pointy-top orientation; the rest of the
 * file just asks it where things go. It holds settings, not the grid contents
 * (there is no stored grid — hexes are computed on the fly).
 */
typedef struct {
    /* Size of the terminal window, in character cells. */
    int rows, cols;

    /* How the hexes look. hex_size is the hex "radius" in pixels (bigger =
     * bigger hexes). border_w is how thick the outline is, as a fraction of a
     * hex (0.10 means the outer 10% of each hex draws its border). cell_w and
     * cell_h are how many pixels wide/tall one character cell counts as, so we
     * can pretend characters are little square pixels. */
    double hex_size;
    double border_w;
    int    cell_w, cell_h;

    /* The character cell that sits at pixel (0,0) — the middle of the screen,
     * which we treat as the center of hex (0,0). */
    int    ox, oy;

    /* Rough limit on how far the cursor can roam, just so we don't wander off
     * forever. The hex plane is really infinite; these are only a guide. */
    int    max_q, max_r;
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
    g->max_q = (int)((double)cols * CELL_W / (sqrt(3.0) * g->hex_size)) + 1;
    g->max_r = (int)((double)rows * CELL_H / (3.0      * g->hex_size)) + 1;
}

/*
 * Given a hex's coordinates, find which character cell sits at its center.
 * This is the "back door": hex coordinates in, screen position out. The
 * formula is the pointy-top one; flat-top would use a different one.
 */
static void ctx_to_screen(const GridCtx *g, int Q, int R, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * (sq3   * (double)Q + sq3_2 * (double)R);
    double cy_pix = g->hex_size * 1.5    * (double)R;
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/*
 * Snap a fractional hex coordinate to the nearest real hex. Hex coordinates
 * come in three numbers that must always add up to zero; rounding each one
 * separately can break that, so we fix up whichever was rounded the hardest.
 */
static void cube_round(double fq, double fr, double fs, int *Q, int *R)
{
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) rq = -rr - rs;
    else if (dr > ds)             rr = -rq - rs;
    *Q = rq; *R = rr;
}

/*
 * The "front door": given a character cell on screen, figure out which hex
 * that point falls inside. Currently unused, kept as the mirror image of
 * ctx_to_screen for anyone who needs to map clicks back to hexes.
 */
__attribute__((unused))
static void ctx_pixel_to_axial(const GridCtx *g, int sr, int sc, int *Q, int *R)
{
    double sq3_3 = sqrt(3.0) / 3.0;
    double px = (double)(sc - g->ox) * g->cell_w;
    double py = (double)(sr - g->oy) * g->cell_h;
    double fq = (sq3_3 * px - 1.0/3.0 * py) / g->hex_size;
    double fr = (2.0/3.0 * py) / g->hex_size;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, Q, R);
}

/*
 * Pick the line character that best matches the slope of an edge: a flat edge
 * gets '-', a steep one gets '|', and the slanted ones get '/' or '\'. We feed
 * it the direction from the hex center out to the pixel, so it works the same
 * for pointy-top and flat-top — the angles just come out differently.
 */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)         return '-';
    else if (t < 3.0 * M_PI / 8.0)   return '\\';
    else if (t < 5.0 * M_PI / 8.0)   return '|';
    else if (t < 7.0 * M_PI / 8.0)   return '/';
    else                              return '-';
}

/*
 * Draw all the hex outlines. We don't store a grid — instead we walk every
 * character cell on screen, ask "which hex does this point belong to, and is
 * it near that hex's edge?", and if so draw an edge character there. The hex
 * the cursor is sitting on gets drawn in the cursor color so it stands out.
 */
static void ctx_draw_bg(const GridCtx *g, int cQ, int cR)
{
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double size  = g->hex_size;
    double limit = 0.5 - g->border_w;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            /* Which hex does this pixel fall in? (pointy-top front door) */
            double fq = (sq3_3 * px - 1.0/3.0 * py) / size;
            double fr = (2.0/3.0 * py) / size;
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
            if (dist < limit) continue;  /* well inside a hex — leave it blank */

            /* Near an edge: find this hex's center so we know which way the
             * edge runs, then pick the matching line character. */
            double cx = size * (sq3 * fQ + sq3_2 * fR);
            double cy = size * 1.5 * fR;
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);

            int on_cur = (Q == cQ && R == cR);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── §5 cursor ── */

/*
 * Where the '@' currently sits, named by hex coordinates (q, r). That's all the
 * state the cursor needs — its pixel position is worked out from these when we
 * draw. Same little struct as 01_flat_top, because hex coordinates don't care
 * which way the hexes are turned.
 */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/*
 * The four arrow-key steps, written as changes to (q, r): up, down, left,
 * right. These are the same numbers as in 01_flat_top, even though the hexes
 * are turned, because the steps are in hex coordinates, not screen pixels —
 * "one hex right" is the same move either way. (The two diagonal hex
 * neighbours aren't reachable here; only four directions are wired up.)
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    (void)g;
    cur->q += dq;
    cur->r += dr;
}

/* Draw the '@' marker at the center of whichever hex the cursor is on. */
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

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, g->hex_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
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
    int theme = 0, paused = 0;
    color_init(theme);

    GridCtx g = {0};
    g.hex_size = HEX_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    double fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        /* Window resized: bounce ncurses so it learns the new size, then
         * re-center the grid to fit. */
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
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

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
