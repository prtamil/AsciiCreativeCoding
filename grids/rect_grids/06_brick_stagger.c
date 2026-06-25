/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_brick_stagger.c — a grid laid out like bricks in a wall.
 *
 * Same as a plain rectangular grid, but every other row is nudged right by
 * half a cell so the cells overlap their neighbours like brickwork. Arrow
 * keys move the '@' around so you can see how the stagger shifts the lines.
 *
 * Compare with: 01_uniform_rect.c (no stagger) and 07_half_brick_vert.c.
 * Brick bond patterns: en.wikipedia.org/wiki/Brick#Bonds
 * Same offset trick, applied to hex grids: redblobgames.com/grids/hexagons
 *
 * The whole idea: horizontal lines never move — they stretch all the way
 * across at the same rows as always. Only the vertical lines shift, and only
 * on odd rows, where they slide right by half a cell. That single nudge is
 * the entire difference from a plain grid.
 */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

#define TARGET_FPS  30
#define CELL_W      10   /* width of a cell, in characters; must be even so half splits cleanly */
#define CELL_H       4   /* height of a cell, in rows */
#define HALF_W      (CELL_W / 2)   /* how far odd rows slide right: half a cell */

/* Higher = the fps number jumps more; lower = it glides. Just for the readout. */
#define FPS_EWMA_ALPHA  0.05

/* Color pair IDs */
#define PAIR_GRID    1   /* grid lines                   */
#define PAIR_ACTIVE  2   /* highlighted cell fill        */
#define PAIR_CURSOR  3   /* bright '@'                   */
#define PAIR_HUD     4   /* status bar (yellow)          */
#define PAIR_HINT    5   /* key-hint footer (cyan)       */

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

static void color_init(void)
{
    start_color(); use_default_colors();
    init_pair(PAIR_GRID,   COLORS >= 256 ? 214 : COLOR_RED,    -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — turning a cell (row,col) into a spot on screen ── */

/*
 * GridCtx — everything we need to know about the grid's size and layout.
 *
 * Filled in once at startup (and again on resize), then handed read-only to
 * the drawing code. We stash the cell size and stagger here instead of using
 * the #defines directly, so the math below reads off one struct.
 *
 *   rows, cols   — how big the terminal is right now, in characters.
 *   cw, ch       — width and height of one cell, in characters.
 *   half_w       — how far odd rows slide right; always cw / 2.
 *   max_r, max_c — the last row/column whose cell still fits on screen.
 *                  max_c leaves room for the stagger so odd rows don't run
 *                  off the right edge.
 */
typedef struct {
    int rows, cols;
    int cw, ch;
    int half_w;
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->half_w = HALF_W;
    g->max_r = (rows - 1) / CELL_H - 1;
    /* Odd rows lean half a cell to the right, so leave that much slack on the
     * right or the cursor could walk off the visible edge. */
    g->max_c = (cols - HALF_W) / CELL_W - 1;
}

/*
 * Where does cell (r,c) sit on screen? This one nudge — adding half a cell on
 * odd rows — is the whole brick effect; everything else matches a plain grid.
 */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw + (r % 2) * g->half_w;
}

/*
 * Pick the line character to draw at one screen spot: '+', '-', '|', or blank.
 * Horizontal lines sit at the same rows everywhere. Vertical lines, though,
 * depend on which row band we're in — on odd rows they're slid right by half
 * a cell to match the bricks above and below.
 */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    bool h = (sr % g->ch == 0);

    int row_idx = sr / g->ch;             /* which row band this line falls in */
    bool v = (row_idx % 2 == 0)
             ? (sc % g->cw == 0)             /* even row: lines line up normally */
             : (sc % g->cw == g->half_w);    /* odd row: lines shifted half a cell */

    if (h && v) return '+';
    if (h)      return '-';
    if (v)      return '|';
    return ' ';
}

static void ctx_draw_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < g->rows - 1; sr++)
        for (int sc = 0; sc < g->cols; sc++) {
            char ch = ctx_grid_char(g, sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/* Cursor — which cell the '@' is sitting in, as a (row, col) pair. */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r = g->max_r / 2;
    cur->c = g->max_c / 2;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= 0 && nr <= g->max_r) cur->r = nr;
    if (nc >= 0 && nc <= g->max_c) cur->c = nc;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->r, cur->c, &sr, &sc);

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = 1; dr < g->ch; dr++)
        for (int dc = 1; dc < g->cw; dc++)
            mvaddch(sr + dr, sc + dc, (chtype)'.');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr + g->ch / 2, sc + g->cw / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    int sr, sc; ctx_to_screen(g, cur->r, cur->c, &sr, &sc);
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  screen_col=%d  stagger=%d ",
        fps, cur->r, cur->c, sc, (cur->r % 2) * g->half_w);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [06 brick stagger  HALF_W=%d] ",
        g->half_w);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 screen ── */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(); atexit(screen_cleanup);
}

/* ── §8 app ── */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
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
    GridCtx g;     ctx_init(&g, LINES, COLS);
    Cursor  cur;   cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS;
    int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
            cursor_reset(&cur, &g);
        }
        int ch = getch();
        switch (ch) {
            case 'q': case 27:  g_running = 0;                  break;
            case 'r':           cursor_reset(&cur, &g);          break;
            case KEY_UP:        cursor_move(&cur, &g, -1,  0);   break;
            case KEY_DOWN:      cursor_move(&cur, &g, +1,  0);   break;
            case KEY_LEFT:      cursor_move(&cur, &g,  0, -1);   break;
            case KEY_RIGHT:     cursor_move(&cur, &g,  0, +1);   break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
