/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_right_isosceles.c — a square grid where every square is cut in half
 * by one diagonal slash, giving two right-triangle halves per square.
 * A '@' cursor walks between the half-triangles with the arrow keys.
 *
 * Sister files: 01_equilateral.c (same draw + cursor idea, but a slanted
 * grid) and 03_double_diagonal.c (both diagonals, four triangles per cell).
 *
 * Wikipedia: "Triangular tiling", "Tetrakis square tiling", and
 * "Barycentric coordinate system" cover the geometry used here.
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

/*
 * Terminal characters are taller than they are wide, so we treat each one
 * as a 2-wide, 4-tall block of "sub-pixels". Measuring in those units keeps
 * the triangles looking square instead of stretched. (01_equilateral.c §1
 * works through where these numbers come from.)
 */
#define CELL_W 2
#define CELL_H 4

#define CELL_SIZE_DEFAULT 16.0
#define CELL_SIZE_MIN      6.0
#define CELL_SIZE_MAX     40.0
#define CELL_SIZE_STEP     2.0

#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.35
#define BORDER_W_STEP    0.02

#define N_THEMES 4

/* How much each new frame nudges the on-screen FPS number. Small = steadier,
 * slower-to-react reading instead of a jittery one. */
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

static const short THEME_FG[N_THEMES]   = {  75,  207, 214,  15 };
static const short THEME_FG_8[N_THEMES] = {
    COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW, COLOR_WHITE,
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

/* ── §4 formula — turning a screen position into "which triangle is here" ── */

/*
 * GridCtx — everything we need to know about the grid right now: how big
 * the terminal is, how large each square is, where the grid is centred, and
 * how thick to draw the triangle edges. The main loop tweaks the size and
 * border live, so this bundle is recomputed whenever those change or the
 * window resizes.
 */
typedef struct {
    /* How many rows and columns of characters the terminal has. */
    int rows, cols;

    /* The square itself. */
    double cell_size;      /* side of one square, in sub-pixels               */
    double border_w;       /* how close to an edge counts as "on the edge"
                              (0 = hairline, bigger = fatter border lines)    */
    int    cw, ch;         /* sub-pixels per character — copies of CELL_W/H   */

    /* Where pixel (0,0) lands on screen, so the grid sits centred. */
    int    ox, oy;

    /* Rough how-far-the-grid-reaches counts; just a hint, not a hard fence. */
    int    max_col, max_row;
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
 * Given a point on screen, work out which square it's in and which half of
 * that square. Dividing by the square size tells us the column/row; the
 * leftover fractions say where inside the square we landed. If we're more to
 * the right than down (fa >= fb) we're in the upper-right half, otherwise the
 * lower-left half — that's just asking which side of the '\' diagonal we're on.
 */
static void ctx_pixel_to_tri(const GridCtx *g, double px, double py,
                             int *col, int *row, int *up,
                             double *fa, double *fb)
{
    double inv = 1.0 / g->cell_size;
    double a   = px * inv;
    double b   = py * inv;
    int    c   = (int)floor(a);
    int    r   = (int)floor(b);
    *col = c;  *row = r;
    *fa  = a - (double)c;
    *fb  = b - (double)r;
    *up  = (*fa >= *fb) ? 1 : 0;
}

/*
 * Find the middle (centroid) of a triangle, in sub-pixels — the spot where
 * we drop the '@' so the cursor sits visually inside the right half.
 */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double a = (up == 1) ? ((double)col + 2.0/3.0) : ((double)col + 1.0/3.0);
    double b = (up == 1) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = a * size;
    *cy_pix = b * size;
}

/*
 * Which character cell on screen holds the middle of a given triangle —
 * used to place the cursor mark.
 */
static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->cell_size, &cx_pix, &cy_pix);
    *sc = g->ox + (int)(cx_pix / g->cw);
    *sr = g->oy + (int)(cy_pix / g->ch);
}

/*
 * Decide which border character a point should be, and how far it is from the
 * nearest edge. Each triangle has three edges; we measure the distance to all
 * three, and the closest one wins. The character names that edge: '|' for a
 * vertical side, '_' for a horizontal side, '\\' for the slanted diagonal.
 * The caller uses the returned distance to tell border pixels from interior.
 */
static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    if (up == 1) {           /* upper-right half */
        l1 = 1.0 - fa;       ch1 = '|';
        l2 = fa - fb;        ch2 = '\\';
        l3 = fb;             ch3 = '_';
    } else {                 /* lower-left half */
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa;             ch2 = '|';
        l3 = fb - fa;        ch3 = '\\';
    }
    char   ch = ch1;
    double m  = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/*
 * Draw the whole grid: walk every character cell, figure out which triangle
 * it falls in, and if it's close enough to an edge, print that edge's
 * character. The triangle the cursor is sitting on gets the highlight colour,
 * so we check for it in the same pass instead of a second loop.
 */
static void ctx_draw_bg(const GridCtx *g, int cC, int cR, int cU)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int    tC, tR, tU;
            double fa, fb, m;
            ctx_pixel_to_tri(g, px, py, &tC, &tR, &tU, &fa, &fb);
            char ch = tri_edge_char(tU, fa, fb, &m);
            if (m >= g->border_w) continue;

            int on_cur = (tC == cC && tR == cR && tU == cU);
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
 * Cursor — where the '@' is right now: which square (col, row) and which
 * half of it (up: 0 = lower-left triangle, 1 = upper-right triangle).
 */
typedef struct { int col, row, up; } Cursor;

/*
 * TRI_DIR — the movement rulebook. Look up [direction][which half you're in]
 * and it tells you the new square and half: the first two numbers add to your
 * column and row, the third sets the new half.
 *
 * Directions are 0:LEFT 1:RIGHT 2:UP 3:DOWN; the inner pair is your current
 * half (0:lower-left, 1:upper-right). Most moves just step across the edge
 * facing that way. But a triangle doesn't have an edge in every direction —
 * e.g. the lower-left half has no flat top — so those presses instead flip you
 * to the other half of the same square, which lands you on the side you wanted.
 */
static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 }, {  0,  0,  0 } },
    /* RIGHT */ { {  0,  0,  1 }, { +1,  0,  0 } },
    /* UP    */ { {  0,  0,  1 }, {  0, -1,  0 } },
    /* DOWN  */ { {  0, +1,  1 }, {  0,  0,  0 } },
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0;
    cur->row = 0;
    cur->up  = 0;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int dir)
{
    (void)g;
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0];
    cur->row += t[1];
    cur->up   = t[2];
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->col, cur->row, cur->up, &sr, &sc);
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
    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "UR" : "LL",
             g->cell_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [02 right isosceles] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->col, cur->row, cur->up);
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
