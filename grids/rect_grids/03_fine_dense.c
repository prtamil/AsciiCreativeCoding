/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_fine_dense.c — a rectangular grid drawn with the smallest cells that
 * still leave room inside. Same drawing math as 01_uniform_rect.c, but the
 * cells are tiny (4 chars wide, 2 tall), so the screen fills up with many
 * more of them. The point: how packed the grid looks is just the cell size.
 *
 * Sister files: 01_uniform_rect.c (the base version), 04_coarse_sparse.c
 * (the opposite — big roomy cells).
 *
 * Move the '@' with the arrow keys, r resets it, q or ESC quits.
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
 * The cell size, in screen characters: 4 wide, 2 tall. This is about as
 * small as it gets — one cell has a border on every side and exactly one
 * blank row by three blank cols inside. Shrink either number and there's
 * no room left inside the borders for the '@'.
 */
#define CELL_W  4
#define CELL_H  2

/* How heavily to smooth the FPS number on screen so it doesn't jitter. */
#define FPS_EWMA_ALPHA  0.05

/* Color slots */
#define PAIR_GRID    1   /* the grid lines               */
#define PAIR_ACTIVE  2   /* the cell the '@' sits in     */
#define PAIR_CURSOR  3   /* the '@' itself               */
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
    /* Grid lines get a calmer color — there are so many of them that a
     * bright one would drown out the '@'. */
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_BLUE,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula ── */

/*
 * GridCtx — everything we need to know about the grid on screen right now:
 * how big the terminal is, how big one cell is, and how far the '@' is
 * allowed to roam. Recomputed whenever the window changes size.
 */
typedef struct {
    int rows, cols;   /* size of the terminal, in characters             */
    int cw, ch;       /* one cell's width and height, in characters      */

    /* The highest cell row/col the '@' can land on — the last whole cell
     * that still fits. We leave the bottom row free for the key hints,
     * so the usable height is rows-1, not rows. */
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->max_r = (rows - 1) / CELL_H - 1;
    g->max_c = cols / CELL_W - 1;
}

/* Turn a cell's (row, col) into where its top-left corner sits on screen:
 * just multiply by the cell size. Cell 0 starts at the very top-left. */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/* Decide what character belongs at one screen spot. Every cell-height row
 * is a horizontal line, every cell-width column is a vertical line; where
 * they cross we draw a corner, and everywhere else is blank. */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    bool h = (sr % g->ch == 0);
    bool v = (sc % g->cw == 0);
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

/* Cursor — which cell the '@' is currently in, as a (row, col) pair. */
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

/* Because cells are only 2 tall, each one has just a single blank row
 * inside. We fill that row across with dashes to highlight the cell, then
 * drop the '@' in the middle of it. */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->r, cur->c, &sr, &sc);

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dc = 1; dc < g->cw; dc++)
        mvaddch(sr + 1, sc + dc, (chtype)'-');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* the '@', centred on that one interior row */
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr + 1, sc + g->cw / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[64];
    snprintf(buf, sizeof buf, " %.1f fps  cell(%d,%d)  grid%dx%d ",
             fps, cur->r, cur->c, g->max_c + 1, g->max_r + 1);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [03 fine dense  %dx%d cells] ",
        g->cw, g->ch);
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
