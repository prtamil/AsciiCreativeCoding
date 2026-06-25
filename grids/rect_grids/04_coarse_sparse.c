/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_coarse_sparse.c — a rectangular grid with big cells, each one labelled
 * with its own (row,col) so the grid reads like a map. Big cells make the
 * point clear: a grid cell is just a named patch of the screen. Move the @
 * around with the arrow keys.
 *
 * Sister files: 03_fine_dense.c (tiny cells), 01_uniform_rect.c (the basic
 * version this builds on).
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

/* Each cell is 20 wide and 6 tall, so there's plenty of empty space inside
 * the borders to print a label. On a normal 80x24 terminal that's about a
 * 4-by-3 grid of cells — small enough to wander around comfortably. */
#define CELL_W  20
#define CELL_H   6

/* The frame rate shown in the corner jitters frame to frame, so we average
 * it gently over time to give a steady, readable number. Smaller = smoother. */
#define FPS_EWMA_ALPHA  0.05

/* Named slots for our colours. */
#define PAIR_GRID    1   /* the grid lines               */
#define PAIR_ACTIVE  2   /* the filled-in cell under @    */
#define PAIR_CURSOR  3   /* the bright @ itself           */
#define PAIR_HUD     4   /* the status readout (yellow)   */
#define PAIR_LABEL   5   /* the (row,col) text in a cell  */
#define PAIR_HINT    6   /* the key-hints footer (cyan)   */

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
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_LABEL,  COLORS >= 256 ? 246 : COLOR_WHITE,  -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — turning a (row,col) into a screen position ── */

/*
 * GridCtx — everything we need to know to lay the grid out and keep the
 * cursor in bounds. Computed once at startup (and again on resize), then
 * passed around read-only. Same idea as 01_uniform_rect; only the cell size
 * is bigger here.
 */
typedef struct {
    int rows, cols;   /* size of the terminal, in characters              */
    int cw, ch;       /* size of one cell: width and height in characters */

    /* The highest cell the cursor may sit on. We leave the bottom screen
     * row free for the key-hints footer, so only whole cells that fit above
     * it count. */
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->max_r = (rows - 1) / CELL_H - 1;
    g->max_c = cols / CELL_W - 1;
}

/* Where on the screen does cell (r,c) start? Its top-left corner is simply
 * the cell number times the cell size. Everything we draw in a cell — the
 * label, the @ — is measured from this corner. */
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

/* Where the @ currently sits, as a cell coordinate (which row, which column).
 * Not pixels — just which cell. */
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
            mvaddch(sr + dr, sc + dc, (chtype)' ');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* the cell under the cursor shows its own label, in the highlight colour */
    char lbl[24]; snprintf(lbl, sizeof lbl, "(%d,%d)", cur->r, cur->c);
    attron(COLOR_PAIR(PAIR_ACTIVE));
    mvprintw(sr + 1, sc + 2, "%s", lbl);
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr + g->ch / 2, sc + g->cw / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ── §6 scene ── */

/* Print each cell's own coordinate near its top-left, just inside the border.
 * The cell under the cursor is skipped here — cursor_draw paints that one in
 * its own colour, so we'd just be drawing over it. */
static void labels_draw(const GridCtx *g, const Cursor *cur)
{
    int gr = (g->rows - 1) / g->ch;
    int gc = g->cols / g->cw;

    for (int r = 0; r < gr; r++) {
        for (int c = 0; c < gc; c++) {
            if (r == cur->r && c == cur->c) continue;  /* cursor paints this one */
            int sr, sc;
            ctx_to_screen(g, r, c, &sr, &sc);
            char lbl[24];
            snprintf(lbl, sizeof lbl, "(%d,%d)", r, c);
            attron(COLOR_PAIR(PAIR_LABEL));
            mvprintw(sr + 1, sc + 2, "%s", lbl);
            attroff(COLOR_PAIR(PAIR_LABEL));
        }
    }
}

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[64];
    snprintf(buf, sizeof buf, " %.1f fps  at(%d,%d) ", fps, cur->r, cur->c);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [04 coarse sparse  %dx%d cell] ",
        g->cw, g->ch);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g);
    labels_draw(g, cur);
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
            /* the window changed size — tear ncurses down and bring it back
             * so LINES/COLS are correct, then recompute the grid to fit */
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
