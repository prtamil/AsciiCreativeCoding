/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 11_checkerboard.c — a grid of cells coloured like a chessboard.
 *
 * Same rectangular grid as 01_uniform_rect, but every other cell is shaded
 * dark. The light/dark choice comes from one rule: a cell is dark when its
 * row + column is odd. The '@' cursor only sits on light cells, so the arrows
 * jump two cells at a time to skip past the dark ones.
 *
 * Study alongside: 01_uniform_rect.c (the plain grid this builds on).
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

#define TARGET_FPS   30
#define CELL_W        8
#define CELL_H        4
#define DARK_FILL   '#'    /* what we paint inside a dark cell */

/* How fast the on-screen FPS number reacts: smaller = steadier, less jumpy. */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_DARK    1
#define PAIR_LIGHT   2
#define PAIR_CURSOR  3
#define PAIR_BORDER  4
#define PAIR_HUD     5
#define PAIR_HINT    6

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
    init_pair(PAIR_DARK,   COLORS >= 256 ? 242 : COLOR_WHITE,  -1);
    init_pair(PAIR_LIGHT,  COLORS >= 256 ? 255 : COLOR_WHITE,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_BORDER, COLORS >= 256 ?  24 : COLOR_CYAN,   -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — turning a cell address into a screen position ── */

/*
 * Everything we need to know about the grid's current layout. Recomputed from
 * the terminal size at startup and on every resize, then passed around so the
 * drawing code never has to guess where a cell sits on screen.
 */
typedef struct {
    int rows, cols;     /* terminal size in characters */
    int cw, ch;         /* one cell's width and height in characters */
    int max_r, max_c;   /* furthest cell the cursor may reach; kept even so
                           it always lands on a light cell (see cursor_reset) */
} GridCtx;

/*
 * Works out the grid layout from the terminal size. max_r/max_c are rounded
 * down to even so the cursor can never get stuck on a dark cell.
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    int gr = (rows - 1) / CELL_H - 1;
    int gc = cols / CELL_W - 1;
    g->max_r = gr & ~1;    /* round down to even so the last cell stays light */
    g->max_c = gc & ~1;
}

/* Where does cell (r,c) start on the screen? Same mapping as the plain grid. */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    bool h = (sr % g->ch == 0);
    bool v = (sc % g->cw == 0);
    if (h && v) return '+';
    if (h)      return '-';
    if (v)      return '|';
    return ' ';
}

/*
 * Light or dark? A cell is dark (returns 1) when its row + column is odd,
 * light (returns 0) when even. Because each step sideways or down flips that
 * even/odd-ness, neighbouring cells always come out opposite — a chessboard.
 */
static int cell_parity(int r, int c) { return (r + c) % 2; }

/* Paint the dark squares first, then lay the grid lines over the top so the
 * borders are never swallowed by the fill. */
static void ctx_draw_bg(const GridCtx *g)
{
    int gr = (g->rows - 1) / g->ch;
    int gc = g->cols / g->cw;

    /* shade the dark cells */
    for (int r = 0; r < gr; r++) {
        for (int c = 0; c < gc; c++) {
            int sr, sc; ctx_to_screen(g, r, c, &sr, &sc);
            bool dark = (cell_parity(r, c) == 1);
            if (dark) {
                attron(COLOR_PAIR(PAIR_DARK));
                for (int dr = 1; dr < g->ch; dr++)
                    for (int dc = 1; dc < g->cw; dc++)
                        mvaddch(sr + dr, sc + dc, (chtype)(unsigned char)DARK_FILL);
                attroff(COLOR_PAIR(PAIR_DARK));
            }
        }
    }

    /* draw the grid lines over the fill */
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int sr = 0; sr < g->rows - 1; sr++)
        for (int sc = 0; sc < g->cols; sc++) {
            char ch = ctx_grid_char(g, sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

/* ── §5 cursor ── */

/* Which cell the '@' currently sits on, as a (row, column) address. */
typedef struct { int r, c; } Cursor;

/* Drop the cursor near the middle, snapped to even coords so it lands light. */
static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r = (g->max_r / 2) & ~1;
    cur->c = (g->max_c / 2) & ~1;
}

/*
 * Move two cells per keypress, not one. A single step would land on a dark
 * cell; stepping by two skips it and keeps the cursor on light cells only.
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + 2 * dr, nc = cur->c + 2 * dc;
    if (nr >= 0 && nr <= g->max_r) cur->r = nr;
    if (nc >= 0 && nc <= g->max_c) cur->c = nc;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc; ctx_to_screen(g, cur->r, cur->c, &sr, &sc);
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr + g->ch / 2, sc + g->cw / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  parity=%d ",
        fps, cur->r, cur->c, cell_parity(cur->r, cur->c));
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move(2 steps)  r:reset  q/ESC:quit  [11 checkerboard  (r+c)%%2] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, fps);
    wnoutrefresh(stdscr); doupdate();
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
            g_need_resize = 0; endwin(); refresh();
            ctx_init(&g, LINES, COLS);
            cursor_reset(&cur, &g);
        }
        int ch = getch();
        switch (ch) {
            case 'q': case 27: g_running = 0;              break;
            case 'r':          cursor_reset(&cur, &g);     break;
            case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
            case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
            case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
            case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
        }
        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&g, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
