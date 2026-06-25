/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 13_dot.c — the bare-minimum grid: just a dot at every corner, no lines.
 *
 * We draw a '.' wherever a row line and a column line would cross, and
 * nothing in between. Your eye fills in the missing grid on its own. An '@'
 * cursor walks from cell to cell; the four dots around it light up green so
 * you can see which cell you're in.
 *
 * Sister file: 01_uniform_rect.c draws the same corners but ALSO draws the
 * connecting lines. The only difference here is AND instead of OR (see §4).
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

/* How far apart the dots sit. This is also the cell size: a cell is one
 * DOT_H tall and one DOT_W wide, with a dot at each of its four corners. */
#define DOT_W   6    /* columns between dots */
#define DOT_H   3    /* rows between dots    */

/* The fps number on screen jumps around frame to frame, so we smooth it:
 * each frame nudges the old value a little toward the new one. */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_DOT     1   /* ordinary dots              */
#define PAIR_CORNER  2   /* the four dots around cursor */
#define PAIR_CURSOR  3
#define PAIR_HUD     4
#define PAIR_HINT    5

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
    init_pair(PAIR_DOT,    COLORS >= 256 ? 252 : COLOR_WHITE,  -1);
    init_pair(PAIR_CORNER, COLORS >= 256 ?  46 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — where dots go and which cell holds a screen position ── */

/* Everything we need to turn cell coordinates into screen positions and back.
 * Filled once at startup (and again on resize) from the terminal size, so the
 * drawing code never has to re-derive any of it.
 *   rows, cols     terminal size right now, in characters
 *   cw, ch         dot spacing — columns and rows between dots (a copy of
 *                  DOT_W / DOT_H, so the math reads off the struct)
 *   max_r, max_c   highest cell index the cursor may sit on; keeps it from
 *                  walking off the bottom or right edge */
typedef struct {
    int rows, cols;
    int cw, ch;
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = DOT_W;  g->ch = DOT_H;
    g->max_r = (rows - 1) / DOT_H - 1;
    g->max_c = cols / DOT_W - 1;
}

static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/* A spot gets a dot only where a row line AND a column line cross — that's
 * the whole trick. Sister file 01 draws when EITHER lines up (so you get the
 * full lattice of lines); using AND keeps just the crossings. */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    bool on_dot = (sr % g->ch == 0) && (sc % g->cw == 0);
    return on_dot ? '.' : ' ';
}

/* Is this screen spot one of the four corner dots of the cursor's cell?
 * Those corners sit on the cell's own row/column lines and the next ones over. */
static bool is_cursor_corner(const GridCtx *g, int sr, int sc, int cr, int cc)
{
    bool r_ok = (sr == cr * g->ch || sr == (cr + 1) * g->ch);
    bool c_ok = (sc == cc * g->cw || sc == (cc + 1) * g->cw);
    return r_ok && c_ok;
}

/* Middle of a cell — where the '@' sits, in the empty space between dots. */
static void cell_centre(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    ctx_to_screen(g, r, c, sr, sc);
    *sr += g->ch / 2;
    *sc += g->cw / 2;
}

/* Paint every dot. The four around the cursor's cell turn into a green '+'
 * instead of a plain '.', which is why we pass the cursor's cell in. */
static void ctx_draw_bg(const GridCtx *g, int cr, int cc)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            if (ctx_grid_char(g, sr, sc) != '.') continue;
            if (is_cursor_corner(g, sr, sc, cr, cc)) {
                attron(COLOR_PAIR(PAIR_CORNER) | A_BOLD);
                mvaddch(sr, sc, (chtype)'+');
                attroff(COLOR_PAIR(PAIR_CORNER) | A_BOLD);
            } else {
                attron(COLOR_PAIR(PAIR_DOT));
                mvaddch(sr, sc, (chtype)'.');
                attroff(COLOR_PAIR(PAIR_DOT));
            }
        }
    }
}

/* ── §5 cursor ── */

/* Which cell the '@' is on, as a (row, column) cell index — not a screen
 * position. §4 turns these into pixels when it's time to draw. */
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
    int sr, sc; cell_centre(g, cur->r, cur->c, &sr, &sc);
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, sc, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[64];
    snprintf(buf, sizeof buf, " %.1f fps  cell(%d,%d) ", fps, cur->r, cur->c);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [13 dot grid  corners only: sr%%H==0 && sc%%W==0] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g, cur->r, cur->c);
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
