/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 12_ruled.c — ruled lines only, like the horizontal lines on notebook paper.
 *
 * Only horizontal lines are drawn. The cursor jumps from line to line going
 * up/down, but slides freely left/right anywhere along a line — so the up/down
 * axis is stepped while the left/right axis is continuous.
 *
 * Study alongside 01_uniform_rect.c (the same idea but with vertical lines too).
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

/* How many screen rows sit between one ruled line and the next. */
#define LINE_STEP    3

/* How far left/right one arrow press moves. Bigger = quicker travel. */
#define COL_STEP     4

/* How hard the shown FPS number is smoothed, so it doesn't jitter. */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_LINE    1   /* ruled lines                    */
#define PAIR_ACTIVE  2   /* cursor's line highlight        */
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
    init_pair(PAIR_LINE,   COLORS >= 256 ? 252 : COLOR_WHITE,  -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  27 : COLOR_BLUE,   -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 rect mapping & lattice ── */

/* GridCtx — the grid for one frame. Only horizontal rules exist, spaced
 * line_step rows apart. The two axes are deliberately lopsided: up/down is
 * stepped by whole lines, left/right is free, so we carry max_line (a line
 * index) but max_col (a raw screen column the cursor may land on). */
typedef struct {
    int rows, cols;        /* terminal size in characters */
    int line_step;         /* screen rows from one ruled line to the next */
    int max_line, max_col; /* furthest line / column the cursor can reach */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->line_step = LINE_STEP;
    g->max_line = (rows - 1) / LINE_STEP - 1;
    g->max_col  = cols - 1;
}

/* recipe step 1 — line index -> the screen row it rules */
static int cell_to_screen(const GridCtx *g, int line)
{
    return line * g->line_step;
}

/* which glyph belongs at a screen spot: every line_step-th row is a rule. */
static char grid_glyph_at(const GridCtx *g, int sr, int sc)
{
    (void)sc;
    return (sr % g->line_step == 0) ? '-' : ' ';
}

/* recipe step 2 — draw every ruled line, the cursor's own line highlighted */
static void draw_lattice(const GridCtx *g, int active_line)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        if (grid_glyph_at(g, sr, 0) != '-') continue;
        int line = sr / g->line_step;
        int pair = (line == active_line) ? PAIR_ACTIVE : PAIR_LINE;
        attron(COLOR_PAIR(pair));
        for (int sc = 0; sc < g->cols; sc++)
            mvaddch(sr, sc, (chtype)'-');
        attroff(COLOR_PAIR(pair));
    }
}

/* ── §5 cursor ── */

/* Cursor — the @ marker. Its two axes behave differently:
 *   line — stepped axis: which ruled line, a whole index (0 = top line).
 *   col  — free axis: a raw screen column it may slide to anywhere. */
typedef struct {
    int line;
    int col;
} Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->line = 0;
    cur->col  = g->cols / 2;
}

/* recipe step 3 — move the cursor, clamped to the screen:
 * the stepped axis hops one whole line, the free axis slides by COL_STEP. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dline, int dcol)
{
    int stepped = cur->line + dline;
    int free    = cur->col  + dcol * COL_STEP;
    if (stepped >= 0 && stepped <= g->max_line) cur->line = stepped;
    if (free    >= 0 && free    <= g->max_col)  cur->col  = free;
}

/* drop '@' on the cursor's ruled line at its free column */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr = cell_to_screen(g, cur->line);
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, cur->col, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  line=%d  col=%d  screen(%d,%d) ",
        fps, cur->line, cur->col, cell_to_screen(g, cur->line), cur->col);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " up/dn:line  left/right:col(+%d)  r:reset  q/ESC:quit  [12 ruled  step=%d] ",
        COL_STEP, LINE_STEP);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    draw_lattice(g, cur->line);
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
