/*
 * 01_flat_top.c — a screen full of flat-top hexagons with a movable cursor.
 *
 * Draws hexagons across the whole terminal. An '@' marker starts on the
 * middle hex; arrow keys hop it to a neighbouring hex, and that hex's outline
 * lights up white-on-blue. +/- resize the hexes, [/] thicken the outline.
 *
 * The trick: there's no grid stored anywhere. For each screen cell we just ask
 * "which hexagon is this pixel inside, and is it near an edge?" and answer it
 * with arithmetic. The how-and-why of that math lives next to §4.
 *
 * Sister files: 02_pointy_top.c (same idea, hexes turned 90°),
 *               rect_grids/01_uniform_rect.c (the GridCtx pattern this copies).
 * Reference:    Red Blob Games hex guide, https://www.redblobgames.com/grids/hexagons/
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

/* How much each new frame nudges the on-screen fps number. Small = steady. */
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
#define PAIR_CURSOR   2   /* the selected hex's outline and its '@' marker */
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

/* ── §4 formula — turning a screen cell into "which hex, how near the edge" ── */

/*
 * GridCtx — everything we need to know about the hex grid right now: how big
 * the terminal is, how big the hexes are, and where the middle of the screen
 * sits. The drawing code reads these to place hexes; the menu keys (+/-, [/])
 * write to them. Bundled together so one struct fully describes a frame.
 *
 * The grid always sits centred: the hex at (Q=0,R=0) lands dead-centre, and
 * everything else is measured outward from there.
 */
typedef struct {
    /* Terminal size in character cells, refreshed on every resize. */
    int rows, cols;

    /* Hex shape and look. */
    double hex_size;       /* hex size: distance from a hex's centre to a corner, in pixels.
                              Bigger = fewer, larger hexes. Tuned by +/-.       */
    double border_w;       /* outline thickness, 0..0.5: how wide the "near an edge"
                              band is. Bigger = chunkier outlines. Tuned by [/]. */
    int    cell_w, cell_h; /* pixels per character cell. A terminal cell is taller than
                              it is wide, so these two differ to keep hexes from
                              looking squashed (set to CELL_W / CELL_H).         */

    /* Where the centre of the screen is, in character cells. The (0,0) hex
       lives here, and pixel positions are measured relative to it. */
    int    ox, oy;

    /* Rough hint at the furthest hex still visible. Advisory only — the cursor
       can wander past these, and nothing clamps to them. */
    int    max_q, max_r;
} GridCtx;

/* Recompute the centre and visible extent after a resize or a size change. */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows     = rows;
    g->cols     = cols;
    g->cell_w   = CELL_W;
    g->cell_h   = CELL_H;
    g->ox       = cols / 2;
    g->oy       = (rows - 1) / 2;
    /* Only fill in size/border if the caller hasn't — on a resize they're
     * already set and we must not stomp the user's chosen values. */
    if (g->hex_size <= 0.0) g->hex_size = HEX_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    g->max_q = (int)((double)cols * CELL_W / (3.0 * g->hex_size)) + 1;
    g->max_r = (int)((double)rows * CELL_H / (sqrt(3.0) * g->hex_size)) + 1;
}

/*
 * Given a hex's (Q,R) coordinates, find the screen cell at its centre.
 * We chop the result down to a whole cell rather than rounding, which nudges
 * the '@' a hair toward the hex's middle so it never lands on an outline char.
 */
static void ctx_to_screen(const GridCtx *g, int Q, int R, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * 1.5    * (double)Q;
    double cy_pix = g->hex_size * (sq3_2 * (double)Q + sq3 * (double)R);
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/*
 * Snap a not-quite-integer hex coordinate to the nearest real hex.
 *
 * Hexes are addressed with three numbers (q, r, s) that must always add up to
 * zero. Round each one on its own and that sum can drift off zero, so we
 * re-derive whichever of the three we rounded most aggressively from the other
 * two — that fixes the sum while disturbing the answer the least. Caller gets
 * back just q and r; s is whatever makes them sum to zero.
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
 * The reverse of ctx_to_screen: hand it a screen cell, get back which hex
 * covers it. Handy for "what did the user click?" style lookups. The main
 * draw loop doesn't call this — it does the same math inline to stay fast.
 */
__attribute__((unused))
static void ctx_pixel_to_axial(const GridCtx *g, int sr, int sc, int *Q, int *R)
{
    double sq3_3 = sqrt(3.0) / 3.0;
    double px = (double)(sc - g->ox) * g->cell_w;
    double py = (double)(sr - g->oy) * g->cell_h;
    double fq = (2.0/3.0 * px) / g->hex_size;
    double fr = (-1.0/3.0 * px + sq3_3 * py) / g->hex_size;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, Q, R);
}

/*
 * Pick the ASCII character whose slant best matches an edge running at the
 * given angle: '-' for flattish, '|' for steep, '/' and '\' in between. Since
 * those glyphs look the same flipped 180°, we only care about angle up to half
 * a turn — that's the fmod-into-[0,pi) step. The caller passes the direction
 * the edge runs, so the character lies along the outline instead of across it.
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
 * Draw the whole grid by walking every screen cell and asking, for each one:
 * which hexagon am I in, and how close to its edge am I? If a cell sits near a
 * hex edge we draw an outline character there; if it's deep inside, we leave it
 * blank. The cursor's hex gets a different colour so it stands out.
 *
 * "How close to the edge" is a single number from the hex coordinates: 0 at
 * the centre, 0.5 at an edge. Anything past 0.5 - border_w counts as edge.
 *
 * cube_round's logic is copied in by hand here instead of calling it — this
 * runs once per screen cell, so skipping the function call keeps it snappy.
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
 * Cursor — which hexagon the user currently has selected, named by its (q, r)
 * coordinates. That's all the cursor knows: not screen pixels, not grid size,
 * just "which hex." To find it on screen, pair it with a GridCtx and run it
 * through ctx_to_screen. (The third hex coordinate is always -q-r, so there's
 * no need to store it.)
 */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/*
 * HEX_DIR — what each arrow key adds to the cursor's (q, r) to step onto a
 * neighbouring hex. A flat-top hex touches six others; the four arrow keys
 * reach four of them (the two skipped neighbours sit on screen diagonals).
 * Up/down move the cursor straight up and down on screen; left/right move it
 * sideways. The same four steps work for the sister grids (02, 06, 07).
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* up    */
    { 0, +1 },   /* down  */
    {-1,  0 },   /* left  */
    {+1,  0 },   /* right */
};

/* Step the cursor by one hex. No edge of the world to bump into, so no clamp. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    (void)g;
    cur->q += dq;
    cur->r += dr;
}

/* Drop the '@' on the selected hex. Called after the grid so it draws on top. */
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

/* Yellow status line up top, cyan key reminders along the bottom. */
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
        if (g_need_resize) {
            g_need_resize = 0;
            /* The endwin/refresh shuffle is how ncurses learns the new
               terminal size after the window changed. */
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
