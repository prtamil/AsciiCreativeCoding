/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 07_half_brick_vert.c — a grid of bricks stacked in columns, like a wall
 * turned on its side. Every other column of cells is nudged down by half a
 * cell, so the horizontal joints zig-zag instead of lining up.
 *
 * It's file 06_brick_stagger with the axes swapped: there the rows shift
 * sideways; here the columns shift up/down. Worth reading the two together.
 * Also see 01_uniform_rect.c for the plain, un-shifted grid this builds on.
 *
 * Move the @ with the arrow keys, r resets, q/ESC quits.
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
#define CELL_W       8   /* how wide one cell is, in screen columns */
#define CELL_H       6   /* how tall one cell is, in screen rows; keep it even */
#define HALF_H      (CELL_H / 2)   /* how far odd columns slide down: half a cell */

/* The on-screen FPS number jumps around frame to frame, so we smooth it:
   each frame nudges the shown value a little toward the latest reading. */
#define FPS_EWMA_ALPHA  0.05

/* Names for the five color slots we set up below. */
#define PAIR_GRID    1   /* the grid lines               */
#define PAIR_ACTIVE  2   /* fill inside the selected cell */
#define PAIR_CURSOR  3   /* the bright '@'               */
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
    init_pair(PAIR_GRID,   COLORS >= 256 ? 220 : COLOR_YELLOW, -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — turning a cell's (row, col) into a spot on screen ── */

/*
 * GridCtx — everything we need to know to draw the grid and keep the cursor
 * inside it: the terminal size, the cell size, how far odd columns slide down,
 * and the furthest cell the cursor is allowed to reach. We bundle it into one
 * struct and pass it around, so the drawing code reads these values from here
 * instead of reaching for the file-level #defines — that keeps the math in one
 * place and makes it easy to swap in a different grid size later.
 */
typedef struct {
    int rows, cols;   /* terminal size, in screen rows and columns */

    int cw, ch;       /* one cell's size: cw wide, ch tall (screen chars) */

    int half_h;       /* the down-shift on odd columns; always ch / 2 */

    /* Furthest cell the cursor may sit on. We stop one short of the edge so a
       whole cell always fits; max_r already leaves room for the odd-column
       overhang at the bottom. */
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->half_h = HALF_H;
    /* Odd columns hang HALF_H lower, so leave that much extra room at the
       bottom or the lowest cell would run off the screen. */
    g->max_r = (rows - 1 - HALF_H) / CELL_H - 1;
    g->max_c = cols / CELL_W - 1;
}

/*
 * Find where a cell's top-left corner lands on screen. The trick of this whole
 * demo lives here: odd-numbered columns get pushed down by half a cell, which
 * is what makes the bricks stagger. Even columns sit at their plain position.
 */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch + (c % 2) * g->half_h;
    *sc = c * g->cw;
}

/*
 * For one screen spot, decide which line character (if any) belongs there.
 * Vertical lines fall on the same even spacing everywhere. Horizontal lines
 * are the staggered part: in odd columns the cells sit half a cell lower, so
 * their joints land half a cell lower too. We pick the line position based on
 * which column we're in.
 */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    bool v = (sc % g->cw == 0);

    int col_idx = sc / g->cw;
    bool h = (col_idx % 2 == 0)
             ? (sr % g->ch == 0)
             : (sr % g->ch == g->half_h);

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

/* Where the @ is right now, given as a cell (row, col) rather than a screen
   spot — ctx_to_screen turns it into pixels when it's time to draw. */
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
        " %.1f fps  cell(%d,%d)  screen_row=%d  stagger=%d ",
        fps, cur->r, cur->c, sr, (cur->c % 2) * g->half_h);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [07 half-brick vert  HALF_H=%d] ",
        g->half_h);
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
