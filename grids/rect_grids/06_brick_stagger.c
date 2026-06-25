/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_brick_stagger.c — a grid laid out like bricks in a wall.
 *
 * Same as 01_uniform_rect.c, but odd rows slide right by half a cell so cells
 * overlap their neighbours like brickwork. Horizontal lines never move; only
 * the vertical lines shift, and only on odd rows. That one nudge is the whole
 * difference from a plain grid.
 *
 * Sister files: 01_uniform_rect.c (no stagger), 07_half_brick_vert.c.
 * Brick bond patterns: en.wikipedia.org/wiki/Brick#Bonds
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

#define CELL_W  10    /* box size in characters; CELL_W must be even so half splits cleanly */
#define CELL_H   4

#define HALF_W  (CELL_W / 2)   /* how far odd rows slide right: half a cell */

#define FPS_EWMA_ALPHA  0.05   /* small = steadier on-screen fps number */

#define PAIR_GRID    1   /* grid lines */
#define PAIR_ACTIVE  2   /* fill of the box you're in */
#define PAIR_CURSOR  3   /* the '@' */
#define PAIR_HUD     4
#define PAIR_HINT    5

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
    init_pair(PAIR_GRID,   COLORS >= 256 ? 214 : COLOR_RED,    -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 rect mapping & lattice ── */

/* GridCtx — the grid for one frame: terminal size, box size, the stagger, and
 * how far the cursor may roam. max_c leaves a half-cell of slack on the right
 * so odd rows don't run off the visible edge. */
typedef struct {
    int rows, cols;      /* terminal size in characters */
    int cw, ch;          /* box width and height in characters */
    int half_w;          /* odd-row shift; always cw / 2 */
    int max_r, max_c;    /* furthest cell the cursor can reach */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->half_w = HALF_W;
    g->max_r = (rows - 1) / CELL_H - 1;
    g->max_c = (cols - HALF_W) / CELL_W - 1;
}

/* recipe step 1 — cell (r,c) -> its top-left corner on screen.
 * Odd rows add half a cell of horizontal shift — the whole brick effect. */
static void cell_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    int stagger = (r % 2) * g->half_w;
    *sr = r * g->ch;
    *sc = c * g->cw + stagger;
}

/* which glyph belongs at a screen spot. Horizontal lines sit at the same rows
 * everywhere; vertical lines slide right by half a cell on odd row bands. */
static char grid_glyph_at(const GridCtx *g, int sr, int sc)
{
    bool h = (sr % g->ch == 0);

    int row_band = sr / g->ch;
    bool v = (row_band % 2 == 0)
             ? (sc % g->cw == 0)            /* even band: lines line up normally */
             : (sc % g->cw == g->half_w);   /* odd band: lines shifted half a cell */

    if (h && v) return '+';
    if (h)      return '-';
    if (v)      return '|';
    return ' ';
}

/* recipe step 2 — draw the grid by asking grid_glyph_at at every screen spot */
static void draw_lattice(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            char ch = grid_glyph_at(g, sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/* Cursor — which box the user is in, as (r,c) from the top-left box (0,0). */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r = g->max_r / 2;
    cur->c = g->max_c / 2;
}

/* recipe step 3 — move the cursor, clamped so it never steps off the grid */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= 0 && nr <= g->max_r) cur->r = nr;
    if (nc >= 0 && nc <= g->max_c) cur->c = nc;
}

/* highlight the cursor's box: dot the interior, then drop '@' in the middle */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    cell_to_screen(g, cur->r, cur->c, &sr, &sc);

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
    int sr, sc; cell_to_screen(g, cur->r, cur->c, &sr, &sc);
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
    draw_lattice(g);
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
