/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_hierarchical.c — a two-level grid, like graph paper.
 *
 * Bright thick lines mark off the big squares; faint thin lines fill in the
 * small ones between them. You steer a cursor one small cell at a time, and
 * the status bar tells you both which small cell you're in and which big
 * square it sits inside.
 *
 * Sister files: 01_uniform_rect.c (just one grid level), 04_coarse_sparse.c.
 *
 * The trick worth knowing: every big-square line also lands exactly on a
 * small-cell line, because the big step is a whole number of small steps.
 * So when we decide how to paint a spot, we ask "is this a big line?" first
 * and only fall back to "small line?" — otherwise the big lines would never
 * get their bold style.
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

#define TARGET_FPS    30

/* The small cell: the smallest square, and how far the cursor moves per key. */
#define MINOR_W       4    /* cols per minor cell */
#define MINOR_H       2    /* rows per minor cell */

/*
 * How many small cells fit across one big square. The big step is just this
 * many small steps, so a big line always sits on a small line too. Bump it
 * up for bigger squares (3 -> 3x3, 5 -> 5x5, ...).
 */
#define MAJOR_FACTOR  4
#define MAJOR_W       (MINOR_W * MAJOR_FACTOR)
#define MAJOR_H       (MINOR_H * MAJOR_FACTOR)

/* Smooths the on-screen fps number so it doesn't jitter every frame. */
#define FPS_EWMA_ALPHA  0.05

/* Color pair IDs */
#define PAIR_MINOR   1   /* faint thin lines (small grid)  */
#define PAIR_MAJOR   2   /* bright thick lines (big grid)  */
#define PAIR_ACTIVE  3   /* cell the cursor sits in        */
#define PAIR_CURSOR  4   /* the '@' marker                 */
#define PAIR_HUD     5   /* status bar (yellow)            */
#define PAIR_HINT    6   /* key-hint footer (cyan)         */

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
    init_pair(PAIR_MINOR,  COLORS >= 256 ?  87 : COLOR_BLUE,   -1);
    init_pair(PAIR_MAJOR,  COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  22 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — turning a cell into a screen spot ── */

/*
 * Everything we need to know about the grid for one terminal size: how big
 * the squares are and how far the cursor is allowed to roam. Filled once at
 * startup (and again on resize), then read all over the place.
 */
typedef struct {
    /* How big the terminal is right now, in characters. */
    int rows, cols;

    /* Small-cell size in characters. This is also one cursor step. */
    int cw, ch;

    /* Big-square size in characters — always a whole number of small cells. */
    int mw, mh;
    int factor;                /* small cells per big square (MAJOR_FACTOR) */

    /* Farthest the cursor can go: the last whole small cell that still fits.
       We reserve the bottom row for the key-hint footer, hence rows-1. */
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = MINOR_W; g->ch = MINOR_H;
    g->factor = MAJOR_FACTOR;
    g->mw = MAJOR_W; g->mh = MAJOR_H;
    g->max_r = (rows - 1) / MINOR_H - 1;
    g->max_c = cols / MINOR_W - 1;
}

/* Where does small cell (r,c) land on screen? Just scale by the cell size. */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/*
 * What kind of grid spot is a screen position?
 *   LEVEL_NONE  — empty interior, draw nothing here.
 *   LEVEL_MINOR — sits on a thin small-grid line only.
 *   LEVEL_MAJOR — sits on a thick big-square line (which is also a small line).
 * The numbers are ordered so "more important" is bigger, but we never rely on
 * the arithmetic — we just compare against the names.
 */
typedef enum { LEVEL_NONE=0, LEVEL_MINOR=1, LEVEL_MAJOR=2 } GridLevel;

/*
 * Classify one screen spot. We must ask "big line?" before "small line?":
 * every big line also falls on a small line, so checking small first would
 * steal the big lines and they'd never get their bold look. is_h / is_v come
 * back saying whether the spot lies on a horizontal line, a vertical one, or
 * both (a crossing).
 */

static GridLevel ctx_grid_level(const GridCtx *g, int sr, int sc,
                                bool *is_h, bool *is_v)
{
    bool mj_h = (sr % g->mh == 0);
    bool mj_v = (sc % g->mw == 0);
    bool mn_h = (sr % g->ch == 0);
    bool mn_v = (sc % g->cw == 0);

    if (mj_h || mj_v) { *is_h = mj_h; *is_v = mj_v; return LEVEL_MAJOR; }
    if (mn_h || mn_v) { *is_h = mn_h; *is_v = mn_v; return LEVEL_MINOR; }
    *is_h = *is_v = false;
    return LEVEL_NONE;
}

/* Paint the whole grid: big lines bright and bold, small lines faint. */
static void ctx_draw_bg(const GridCtx *g)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            bool is_h, is_v;
            GridLevel lvl = ctx_grid_level(g, sr, sc, &is_h, &is_v);
            if (lvl == LEVEL_NONE) continue;

            char ch = ' ';
            if (is_h && is_v) ch = '+';
            else if (is_h)    ch = (lvl == LEVEL_MAJOR) ? '=' : '-';
            else if (is_v)    ch = '|';

            if (lvl == LEVEL_MAJOR) {
                attron(COLOR_PAIR(PAIR_MAJOR) | A_BOLD);
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_MAJOR) | A_BOLD);
            } else {
                attron(COLOR_PAIR(PAIR_MINOR));
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_MINOR));
            }
        }
    }
}

/* ── §5 cursor ── */

/* Where the '@' is, counted in small cells from the top-left corner. */
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
    for (int dc = 1; dc < g->cw; dc++)
        mvaddch(sr + 1, sc + dc, (chtype)' ');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr + 1, sc + g->cw / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    /* Split the cursor's small-cell address into "which big square" and
       "where inside that square" to show all three on the status bar. */
    int maj_r = cur->r / g->factor, maj_c = cur->c / g->factor;
    int loc_r = cur->r % g->factor, loc_c = cur->c % g->factor;
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  minor(%d,%d)  major(%d,%d)  local(%d,%d) ",
        fps, cur->r, cur->c, maj_r, maj_c, loc_r, loc_c);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [05 hierarchical  factor=%d] ",
        g->factor);
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
