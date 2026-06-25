/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_uniform_rect.c — a plain rectangular grid you can walk a cursor around.
 *
 * Evenly spaced lines split the terminal into equal boxes; arrow keys move
 * the '@' from box to box. This is the simplest grid in the series — every
 * other file here is this same skeleton with one piece swapped out.
 *
 * The trick: we never store a grid. We just ask, at each screen spot, "is
 * this row or column on a line?" using leftover-after-division. Sister files
 * and the shared GridCtx/Cursor idea live in ../README.md; 02_square.c shows
 * how to make the boxes look square.
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

/*
 * How big each box is, measured in terminal characters: CELL_W columns wide,
 * CELL_H rows tall. Terminal characters are about twice as tall as they are
 * wide, so 8 wide by 4 tall ends up looking roughly square on screen.
 * 02_square.c works out that aspect ratio properly.
 */
#define CELL_W  8
#define CELL_H  4

/* How much to trust each new frame when smoothing the FPS number on screen.
 * Small value = steady reading that ignores one-off hiccups. */
#define FPS_EWMA_ALPHA  0.05

/* Color slots: which color each part of the screen uses. */
#define PAIR_GRID    1   /* the grid lines       */
#define PAIR_ACTIVE  2   /* fill of the box you're standing in */
#define PAIR_CURSOR  3   /* the '@' itself       */
#define PAIR_HUD     4   /* fps readout (yellow) */
#define PAIR_HINT    5   /* key hints (cyan)     */

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

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — turning a cell (r,c) into a screen spot ── */

/*
 * GridCtx — everything we need to know about the current grid: how big the
 * terminal is, how big a box is, and how far the cursor is allowed to roam.
 *
 * We keep the box size as fields here instead of reading the #defines
 * directly, so the grid functions work on whatever GridCtx you hand them.
 * That's the shared idea the placement files lean on too
 * (../rect_grids_placement/01_direct.c, ../README.md).
 *
 *   rows, cols   — terminal size, in characters.
 *   cw, ch       — box width and height, in characters.
 *   max_r, max_c — the furthest cell the cursor can reach: the last whole
 *                  box that fully fits, leaving the bottom row for the hint.
 */
typedef struct {
    int rows, cols;
    int cw, ch;
    int max_r, max_c;
} GridCtx;

/* Work out the grid's measurements from the current terminal size.
 * We stop one row short of the bottom so the key-hint line has room, and we
 * drop any half-box at the right/bottom edge so the cursor can't land in it. */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->max_r = (rows - 1) / CELL_H - 1;
    g->max_c = cols / CELL_W - 1;
}

/* Where on screen does cell (r,c) start (its top-left corner)? Each step
 * right is cw columns over, each step down is ch rows down; cell (0,0) sits
 * at the very top-left. This one tiny mapping is the only thing that differs
 * between the grids in this series. */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/* What should we draw at this exact screen spot? A spot lands on a line when
 * its row or column divides evenly into the box size (nothing left over).
 * Lines crossing -> '+', a horizontal run -> '-', a vertical run -> '|',
 * and anything inside a box -> blank. */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    bool h = (sr % g->ch == 0);
    bool v = (sc % g->cw == 0);
    if (h && v) return '+';
    if (h)      return '-';
    if (v)      return '|';
    return ' ';
}

/* Paint the whole grid by asking the question above at every screen spot.
 * It's one check per character, which is plenty fast at terminal sizes. */
static void ctx_draw_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            char ch = ctx_grid_char(g, sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/*
 * Cursor — which box the user is currently standing in, as a row and column.
 *
 * It deliberately doesn't know how big the grid is; the limits live in
 * GridCtx. Pair the two together and ctx_to_screen() turns this (r,c) into an
 * actual spot on screen.
 *
 *   r, c — the box coordinates, counting from the top-left box (0,0).
 */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r = g->max_r / 2;
    cur->c = g->max_c / 2;
}

/* Nudge the cursor by (dr,dc), but never let it step off the grid: each axis
 * only moves if the new spot is still in bounds. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= 0 && nr <= g->max_r) cur->r = nr;
    if (nc >= 0 && nc <= g->max_c) cur->c = nc;
}

/* Highlight the box the cursor is in: dot in every inside cell (the space
 * between the four lines), then drop the '@' right in the middle. */
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

/* The two bits of text overlaid on the grid: fps and current cell up top,
 * the key reminders along the bottom. */
static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[64];
    snprintf(buf, sizeof buf, " %.1f fps  cell(%d,%d) ", fps, cur->r, cur->c);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  r:reset  q/ESC:quit  [01 uniform rect] ");
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
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
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
