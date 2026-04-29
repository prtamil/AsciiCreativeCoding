/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 12_ruled.c — ruled lines only (horizontal stripes, no vertical lines)
 *
 * DEMO: Only horizontal lines are drawn — like notebook ruled paper.
 *       The player snaps vertically to lines (rows), but moves freely
 *       along each line (free column). This is a degenerate 2-D grid:
 *       it divides the Y axis but leaves the X axis continuous.
 *
 * Study alongside: 01_uniform_rect.c (adds vertical lines), 13_dot.c
 *
 * Section map:
 *   §1 config   — LINE_STEP (row spacing between lines)
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs
 *   §4 formula  — only sr % LINE_STEP == 0 for lines; free column
 *   §5 player   — snaps to line rows; free column within [0, cols-1]
 *   §6 scene    — draw_ruled(), draw_player()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/12_ruled.c \
 *       -o 12_ruled -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : The 1-D lattice applied only to the Y axis (rows).
 *                  Grid lines at: sr % LINE_STEP == 0
 *                  No constraint on the X axis — player column is free.
 *
 * Degenerate grid: A "ruled" grid is a rectangular grid with CELL_W → ∞.
 *                  Each "cell" is an infinite horizontal strip.
 *                  Practical interpretation: LINE_STEP rows between lines.
 *
 * Player position: Two independent coordinates:
 *                    line  — which ruled line the player is on (integer)
 *                    col   — horizontal position along the line (integer, free)
 *
 *                  screen_row = line * LINE_STEP  (on a ruled line)
 *                  screen_col = col               (free: 0 .. COLS-1)
 *
 * Movement:
 *   UP/DOWN   → line ± 1        (jump to adjacent line)
 *   LEFT/RIGHT → col ± COL_STEP (move along the line)
 *
 * References     :
 *   Ruled paper — en.wikipedia.org/wiki/Ruled_paper
 *   1-D lattice  — en.wikipedia.org/wiki/Lattice_(group)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ────────��
 * A ruled grid is a rectangular grid with CELL_W set to infinity.  There
 * is only ONE family of lines: horizontal.  The Y axis is divided into
 * discrete "lines" (rows); the X axis is continuous.  The player has two
 * INDEPENDENT coordinates: a discrete line index and a free column position.
 *
 * HOW TO THINK ABOUT IT — TWO INDEPENDENT AXES
 * ─────────────────────────────────────────────
 * Think of a musical staff, a notebook, or a time chart.  Lines divide
 * the vertical space; horizontal space is free.  The player sits ON a
 * ruled line, at any horizontal position along it.
 *
 * The two independent coordinates:
 *   line  (discrete):  which ruled line the player is on.
 *                      Ranges from 0 to (LINES-2)/LINE_STEP - 1.
 *   col   (integer):   horizontal position in screen columns.
 *                      Ranges from 0 to COLS-1.
 *
 * Movement is NOT symmetric:
 *   UP/DOWN keys → change line by ±1           (jump to next/prev line)
 *   LEFT/RIGHT   → change col  by ±COL_STEP    (slide along the line)
 *
 * There is no cell_to_screen in the usual sense:
 *   screen_row = line * LINE_STEP      (discrete Y)
 *   screen_col = col                   (free X, no scaling needed)
 *
 * DRAWING METHOD
 * ──────────────
 *  Per screen row sr:
 *    on_line = (sr % LINE_STEP == 0)
 *    if on_line AND sr == active_line * LINE_STEP:
 *        draw the active line with ACTIVE color (different from inactive)
 *    else if on_line:
 *        draw a regular ruled line with GRID color
 *    else:
 *        skip (empty space between lines)
 *
 *  For each ruled line: fill the entire screen row with '-' characters.
 *  The active line can use a different character or color to show the
 *  player's current line.
 *
 *  Then draw '@' at the player's (line*LINE_STEP, col) position on top.
 *
 * KEY FORMULAS
 * ────────────
 *  Line position:
 *    screen_row = line * LINE_STEP     (maps line index to screen row)
 *    line       = screen_row / LINE_STEP  (inverse: screen row -> line)
 *
 *  Line membership test (same formula as all other grids):
 *    is_ruled_line(sr) = (sr % LINE_STEP == 0)
 *
 *  Number of visible lines:
 *    num_lines = (LINES - 1) / LINE_STEP
 *
 *  Column movement:
 *    new_col = clamp(col + dc * COL_STEP, 0, COLS-1)
 *
 *  This is a degenerate rectangular grid:
 *    CELL_H = LINE_STEP,   CELL_W = ∞ (no vertical lines)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • LINE_STEP=1: every screen row is a ruled line — solid horizontal
 *    fill.  There is no space between lines.  Not useful as a grid.
 *    Minimum: LINE_STEP=2 for visible inter-line space.
 *
 *  • COL_STEP=1: very slow left/right movement.  Increase COL_STEP
 *    for faster navigation across the line.  COL_STEP=4 or 8 works well.
 *
 *  • The player is always on a ruled line (screen_row = line*LINE_STEP).
 *    Never place '@' between lines.  The line index is always an integer.
 *
 *  • There is no column grid structure.  If you add a column marker or
 *    cursor it must be drawn explicitly — there is no modular condition
 *    for the column position.
 *
 *  • After terminal resize: recompute max_line = (LINES-1)/LINE_STEP - 1
 *    and clamp player.line to the new max.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Count the number of ruled lines on a 24-row terminal with LINE_STEP=3:
 *    Lines at rows: 0, 3, 6, 9, 12, 15, 18, 21 -> 8 lines.
 *    Formula: (24-1) / 3 = 7.66 -> floor = 7... but including row 0:
 *    Actually: rows 0 through 21 with step 3 = 8 lines (0,3,...,21).
 *    floor((LINES-1) / LINE_STEP) + 1 = floor(23/3) + 1 = 7 + 1 = 8. ✓
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS   30

/*
 * LINE_STEP — rows between consecutive ruled lines.
 * Compare to CELL_H in 01_uniform_rect: same formula, only in Y.
 */
#define LINE_STEP    3    /* screen rows between ruled lines */

/*
 * COL_STEP — how many columns to advance per LEFT/RIGHT keypress.
 * With free column movement, a step > 1 makes navigation faster.
 */
#define COL_STEP     4

#define PAIR_LINE    1   /* ruled lines                    */
#define PAIR_ACTIVE  2   /* player's line highlight        */
#define PAIR_PLAYER  3
#define PAIR_HUD     4

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void color_init(void)
{
    start_color(); use_default_colors();
    init_pair(PAIR_LINE,   COLORS >= 256 ? 252 : COLOR_WHITE,  -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  27 : COLOR_BLUE,   -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * RULED GRID FORMULA:
 *
 * There is NO cell_to_screen: the player position is (line, col)
 * which maps directly to screen as:
 *   screen_row = line * LINE_STEP
 *   screen_col = col                  (free — no CELL_W scale)
 *
 * Grid line detection: a screen row sr is a ruled line when:
 *   sr % LINE_STEP == 0
 *
 * No vertical line condition — that's what makes it "ruled" not "rect".
 */
static bool is_ruled_line(int sr) { return (sr % LINE_STEP == 0); }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  player                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int line;       /* which ruled line (0 = top, 1 = next, ...)  */
    int col;        /* free horizontal position in screen columns  */
    int max_line;
    int max_col;
} Player;

static void player_reset(Player *p, int rows, int cols)
{
    p->max_line = (rows - 1) / LINE_STEP - 1;
    p->max_col  = cols - 1;
    p->line = p->max_line / 2;
    p->col  = cols / 2;
}

/*
 * player_move — two independent axes:
 *   UP/DOWN   → line ± 1
 *   LEFT/RIGHT → col ± COL_STEP, clamped to [0, max_col]
 */
static void player_move(Player *p, int dline, int dcol)
{
    int nl = p->line + dline;
    int nc = p->col  + dcol * COL_STEP;
    if (nl >= 0 && nl <= p->max_line) p->line = nl;
    if (nc >= 0 && nc <= p->max_col)  p->col  = nc;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void draw_ruled(int rows, int cols, int active_line)
{
    for (int sr = 0; sr < rows - 1; sr++) {
        if (!is_ruled_line(sr)) continue;
        int line = sr / LINE_STEP;
        if (line == active_line) {
            attron(COLOR_PAIR(PAIR_ACTIVE));
            for (int sc = 0; sc < cols; sc++)
                mvaddch(sr, sc, (chtype)'-');
            attroff(COLOR_PAIR(PAIR_ACTIVE));
        } else {
            attron(COLOR_PAIR(PAIR_LINE));
            for (int sc = 0; sc < cols; sc++)
                mvaddch(sr, sc, (chtype)'-');
            attroff(COLOR_PAIR(PAIR_LINE));
        }
    }
}

static void draw_player(const Player *p)
{
    int sr = p->line * LINE_STEP;
    attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
    mvaddch(sr, p->col, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Player *p, double fps)
{
    erase();
    draw_ruled(rows, cols, p->line);
    draw_player(p);

    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  line=%d  col=%d  screen(%d,%d) ",
        fps, p->line, p->col, p->line * LINE_STEP, p->col);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " up/dn:line  left/right:col(+%d)  r:reset  q:quit  [12 ruled  step=%d] ",
        COL_STEP, LINE_STEP);
    attroff(COLOR_PAIR(PAIR_HUD) | A_DIM);

    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(); atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

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
    int rows = LINES, cols = COLS;
    Player player; player_reset(&player, rows, cols);
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS; int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
            player_reset(&player, rows, cols);
        }
        int ch = getch();
        switch (ch) {
            case 'q': case 27: g_running = 0;                    break;
            case 'r':          player_reset(&player, rows, cols); break;
            case KEY_UP:    player_move(&player, -1,  0); break;
            case KEY_DOWN:  player_move(&player, +1,  0); break;
            case KEY_LEFT:  player_move(&player,  0, -1); break;
            case KEY_RIGHT: player_move(&player,  0, +1); break;
        }
        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (now - t0 + 1)) * 0.05; t0 = now;
        scene_draw(rows, cols, &player, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
