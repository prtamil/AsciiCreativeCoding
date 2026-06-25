/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal.c — squares cut into 4 triangles, one per direction.
 *
 * Each square gets both diagonals drawn in, splitting it into 4 triangles
 * that meet at the centre. Each triangle points one way — N, E, S, or W —
 * and the arrow keys walk the @ cursor between them.
 *
 * Sister files: 02_right_isosceles.c (one diagonal, 2 triangles per square),
 *               04_30_60_90.c (the same idea for a triangular grid).
 * Background: "tetrakis square tiling" on Wikipedia; Conway, Burgiel &
 *             Goodman-Strauss, "The Symmetries of Things" (2008) §22.
 */

/*
 * THE BIG PICTURE
 *
 * There is no grid stored in memory. Instead, for every character cell on
 * screen we ask: which square is this, and which of its 4 triangles does it
 * fall in? Both answers come straight from the pixel's position, recomputed
 * fresh each frame.
 *
 * To find the triangle, stand at the centre of a square. The two diagonals
 * cut it into 4 wedges, like slicing a sandwich corner to corner both ways:
 *   N is the top wedge, E the right, S the bottom, W the left.
 * Which wedge a point lands in just depends on whether it's further from the
 * centre horizontally or vertically: more horizontal -> E or W (left/right
 * tells which), more vertical -> N or S.
 *
 * To draw, we only paint the thin outlines, not the solid interiors. For each
 * pixel we measure how close it is to the triangle's three edges; if it's
 * hugging an edge we draw a slash, backslash, underscore or bar, otherwise we
 * leave it blank. Edge distances use barycentric weights — a standard way to
 * say "how close am I to each side of a triangle" (see the Barycentric
 * coordinates article on Wikipedia).
 *
 * Good to know:
 *  - Right on a diagonal the two wedges tie; we just pick one, it's invisible.
 *  - Make squares too small (below size 8) and the inner triangles vanish.
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

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define CELL_SIZE_DEFAULT 18.0
#define CELL_SIZE_MIN      8.0
#define CELL_SIZE_MAX     48.0
#define CELL_SIZE_STEP     2.0

#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.35
#define BORDER_W_STEP    0.02

/* Wedge direction indices */
#define WEDGE_N 0
#define WEDGE_E 1
#define WEDGE_S 2
#define WEDGE_W 3

#define N_THEMES 4

/* How fast the on-screen FPS number reacts; low = smooth and steady. */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_HUD    3
#define PAIR_HINT   4

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

static const short THEME_FG[N_THEMES]   = {  82, 207, 214,  15 };
static const short THEME_FG_8[N_THEMES] = {
    COLOR_GREEN, COLOR_MAGENTA, COLOR_YELLOW, COLOR_WHITE,
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg = (COLORS >= 256) ? THEME_FG[theme] : THEME_FG_8[theme];
    init_pair(PAIR_BORDER, fg, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — which square, which triangle, which edge ── */

/*
 * GridCtx — everything we need to draw the grid this frame.
 *
 * There's no stored grid; this just holds the current size and placement so
 * any pixel can be turned into a square + triangle. cell_size and border_w
 * are live-tunable, so the picture redraws when the user zooms or thickens
 * the outlines. Centring matches its sister file 02.
 */
typedef struct {
    /* size of the terminal, in character cells */
    int rows, cols;

    double cell_size;      /* one square's side length, in pixels            */
    double border_w;       /* how close to an edge counts as "on the line";
                              bigger = thicker outlines                      */
    int    cw, ch;         /* pixels packed into one character cell (a cell
                              is taller than it is wide), = CELL_W, CELL_H   */

    int    ox, oy;         /* screen cell that sits at pixel (0,0)           */

    int    max_col, max_row;  /* rough how-far-the-grid-reaches hint         */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->cw   = CELL_W;
    g->ch   = CELL_H;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
    if (g->cell_size <= 0.0) g->cell_size = CELL_SIZE_DEFAULT;
    if (g->border_w  <= 0.0) g->border_w  = BORDER_W_DEFAULT;
    g->max_col = (int)((double)cols * CELL_W / g->cell_size) + 1;
    g->max_row = (int)((double)rows * CELL_H / g->cell_size) + 1;
}

/*
 * Given a pixel, work out which square it's in and which of the 4 triangles.
 * The square comes from dividing by the square size; the triangle comes from
 * asking whether the pixel sits further from the square's centre sideways or
 * up/down. Also hands back where inside the square it landed (fa, fb, each
 * 0..1) so the caller can measure edge distances.
 */
static void ctx_pixel_to_tri(const GridCtx *g, double px, double py,
                             int *col, int *row, int *wedge,
                             double *fa, double *fb)
{
    double inv = 1.0 / g->cell_size;
    double a   = px * inv;
    double b   = py * inv;
    int    c   = (int)floor(a);
    int    r   = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c;
    *fb = b - (double)r;

    double dx = *fa - 0.5, dy = *fb - 0.5;
    double adx = fabs(dx), ady = fabs(dy);
    if (adx > ady) *wedge = (dx > 0.0) ? WEDGE_E : WEDGE_W;
    else           *wedge = (dy > 0.0) ? WEDGE_S : WEDGE_N;
}

/*
 * The opposite trip: given a triangle, find the pixel at its middle. Used to
 * place the @ marker on the triangle the cursor is sitting on. Each triangle's
 * middle sits a fixed fraction into its square (the N triangle's middle is
 * near its top edge, and so on).
 */
static void tri_centroid_pixel(int col, int row, int wedge, double size,
                               double *cx_pix, double *cy_pix)
{
    double a, b;
    switch (wedge) {
        case WEDGE_N: a = 0.5;       b = 1.0/6.0;  break;
        case WEDGE_E: a = 5.0/6.0;   b = 0.5;      break;
        case WEDGE_S: a = 0.5;       b = 5.0/6.0;  break;
        default:      a = 1.0/6.0;   b = 0.5;      break;  /* W */
    }
    *cx_pix = ((double)col + a) * size;
    *cy_pix = ((double)row + b) * size;
}

static void ctx_to_screen(const GridCtx *g, int col, int row, int wedge,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, wedge, g->cell_size, &cx_pix, &cy_pix);
    *sc = g->ox + (int)(cx_pix / g->cw);
    *sr = g->oy + (int)(cy_pix / g->ch);
}

/*
 * Pick the outline character for a pixel, and report how close it is to the
 * nearest edge. Each triangle has three edges, each drawn with its own glyph:
 * a slash, a backslash, and a flat one (underscore or bar). We measure the
 * distance to all three (smaller = closer), keep the nearest, and return its
 * glyph. The caller draws it only when that distance is small enough.
 */
static char tri_edge_char(int wedge, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    switch (wedge) {
        case WEDGE_N:
            l1 = 1.0 - fa - fb; ch1 = '/';
            l2 = fa - fb;       ch2 = '\\';
            l3 = 2.0 * fb;      ch3 = '_';
            break;
        case WEDGE_E:
            l1 = fa - fb;       ch1 = '\\';
            l2 = fa + fb - 1.0; ch2 = '/';
            l3 = 2.0 * (1.0 - fa); ch3 = '|';
            break;
        case WEDGE_S:
            l1 = fb - fa;       ch1 = '\\';
            l2 = fa + fb - 1.0; ch2 = '/';
            l3 = 2.0 * (1.0 - fb); ch3 = '_';
            break;
        default: /* WEDGE_W */
            l1 = 1.0 - fa - fb; ch1 = '\\';
            l2 = fb - fa;       ch2 = '/';
            l3 = 2.0 * fa;      ch3 = '|';
            break;
    }
    char   ch = ch1;
    double m  = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

static void ctx_draw_bg(const GridCtx *g, int cC, int cR, int cWedge)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int    tC, tR, tW;
            double fa, fb, m;
            ctx_pixel_to_tri(g, px, py, &tC, &tR, &tW, &fa, &fb);
            char ch = tri_edge_char(tW, fa, fb, &m);
            if (m >= g->border_w) continue;

            int on_cur = (tC == cC && tR == cR && tW == cWedge);
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
 * Cursor — where the @ marker currently sits.
 *
 * col, row say which square; wedge says which of its 4 triangles
 * (0=N, 1=E, 2=S, 3=W — the direction that triangle points).
 */
typedef struct { int col, row, wedge; } Cursor;

/*
 * TETRA_DIR — the movement rules, baked into a lookup table.
 *
 * Read it as TETRA_DIR[which arrow][current triangle] -> {shift in column,
 * shift in row, new triangle}. Arrows are 0:LEFT 1:RIGHT 2:UP 3:DOWN;
 * triangles are 0:N 1:E 2:S 3:W.
 *
 * The idea: an arrow nudges the cursor that way. Usually it just hops to the
 * neighbouring triangle inside the same square. But if you're already against
 * the square's outer edge in that direction, it steps into the next square and
 * lands on the triangle facing back the way you came. Example: in the W
 * triangle pressing LEFT crosses into the previous square's E triangle.
 */
static const int TETRA_DIR[4][4][3] = {
    /* LEFT  */ { {  0,  0, WEDGE_W }, {  0,  0, WEDGE_W }, {  0,  0, WEDGE_W }, { -1,  0, WEDGE_E } },
    /* RIGHT */ { {  0,  0, WEDGE_E }, { +1,  0, WEDGE_W }, {  0,  0, WEDGE_E }, {  0,  0, WEDGE_E } },
    /* UP    */ { {  0, -1, WEDGE_S }, {  0,  0, WEDGE_N }, {  0,  0, WEDGE_N }, {  0,  0, WEDGE_N } },
    /* DOWN  */ { {  0,  0, WEDGE_S }, {  0,  0, WEDGE_S }, {  0, +1, WEDGE_N }, {  0,  0, WEDGE_S } },
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col   = 0;
    cur->row   = 0;
    cur->wedge = WEDGE_N;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int arrow)
{
    (void)g;
    const int *t = TETRA_DIR[arrow][cur->wedge];
    cur->col  += t[0];
    cur->row  += t[1];
    cur->wedge = t[2];
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->col, cur->row, cur->wedge, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static const char *WEDGE_NAME[4] = { "N", "E", "S", "W" };

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, WEDGE_NAME[cur->wedge],
             g->cell_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [03 double diagonal] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->col, cur->row, cur->wedge);
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
    g.cell_size = CELL_SIZE_DEFAULT;
    g.border_w  = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           paused ^= 1; break;
                case 'r':           cursor_reset(&cur, &g); break;
                case 't':
                    theme = (theme + 1) % N_THEMES;
                    color_init(theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, &g, 0); break;
                case KEY_RIGHT: cursor_move(&cur, &g, 1); break;
                case KEY_UP:    cursor_move(&cur, &g, 2); break;
                case KEY_DOWN:  cursor_move(&cur, &g, 3); break;
                case '+': case '=':
                    if (g.cell_size < CELL_SIZE_MAX) { g.cell_size += CELL_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '-':
                    if (g.cell_size > CELL_SIZE_MIN) { g.cell_size -= CELL_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '[':
                    if (g.border_w > BORDER_W_MIN) { g.border_w -= BORDER_W_STEP; } break;
                case ']':
                    if (g.border_w < BORDER_W_MAX) { g.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns(), dt = now - t0; t0 = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
