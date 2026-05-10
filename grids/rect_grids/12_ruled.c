/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 12_ruled.c — ruled lines only (horizontal stripes, no vertical lines)
 *
 * DEMO: Only horizontal lines are drawn — like notebook ruled paper.
 *       The cursor snaps vertically to lines (rows), but moves freely
 *       along each line (free column). This is a degenerate 2-D grid:
 *       it divides the Y axis but leaves the X axis continuous.
 *
 * Study alongside: 01_uniform_rect.c (adds vertical lines), 13_dot.c
 *
 * Section map:
 *   §1 config   — LINE_STEP (row spacing), COL_STEP (free-axis step), EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs (line, active, cursor, HUD, HINT)
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_grid_char / ctx_draw_bg
 *   §5 cursor   — Cursor (line, col) — two INDEPENDENT axes
 *   §6 scene    — hud_draw + scene_draw
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
 *                  No constraint on the X axis — cursor column is free.
 *
 * Degenerate grid: A "ruled" grid is a rectangular grid with CELL_W → ∞.
 *                  Each "cell" is an infinite horizontal strip.
 *                  Practical interpretation: LINE_STEP rows between lines.
 *
 * Cursor position: Two independent coordinates:
 *                    line  — which ruled line the cursor is on (integer)
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
 * ──────────
 * A ruled grid is a rectangular grid with CELL_W set to infinity.  There
 * is only ONE family of lines: horizontal.  The Y axis is divided into
 * discrete "lines" (rows); the X axis is continuous.  The cursor has two
 * INDEPENDENT coordinates: a discrete line index and a free column position.
 *
 * HOW TO THINK ABOUT IT — TWO INDEPENDENT AXES
 * ─────────────────────────────────────────────
 * Think of a musical staff, a notebook, or a time chart.  Lines divide
 * the vertical space; horizontal space is free.  The cursor sits ON a
 * ruled line, at any horizontal position along it.
 *
 * The two independent coordinates:
 *   line  (discrete):  which ruled line the cursor is on.
 *                      Ranges from 0 to (LINES-2)/LINE_STEP - 1.
 *   col   (integer):   horizontal position in screen columns.
 *                      Ranges from 0 to COLS-1.
 *
 * Movement is NOT symmetric:
 *   UP/DOWN keys → change line by ±1           (jump to next/prev line)
 *   LEFT/RIGHT   → change col  by ±COL_STEP    (slide along the line)
 *
 * There is no ctx_to_screen in the usual sense:
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
 *  Then draw '@' at the cursor's (line*LINE_STEP, col) position on top.
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
 *    fill.  Minimum: LINE_STEP=2 for visible inter-line space.
 *
 *  • COL_STEP=1: very slow left/right movement.  Increase COL_STEP
 *    for faster navigation across the line.  COL_STEP=4 or 8 works well.
 *
 *  • The cursor is always on a ruled line (screen_row = line*LINE_STEP).
 *    Never place '@' between lines.  The line index is always an integer.
 *
 *  • There is no column grid structure.  If you add a column marker or
 *    pointer it must be drawn explicitly — there is no modular condition
 *    for the column position.
 *
 *  • After terminal resize: recompute max_line = (LINES-1)/LINE_STEP - 1
 *    and clamp cursor.line to the new max.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Count the number of ruled lines on a 24-row terminal with LINE_STEP=3:
 *    Lines at rows: 0, 3, 6, 9, 12, 15, 18, 21 -> 8 lines.
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

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_LINE    1   /* ruled lines                    */
#define PAIR_ACTIVE  2   /* cursor's line highlight        */
#define PAIR_CURSOR  3
#define PAIR_HUD     4
#define PAIR_HINT    5

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
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the ruled-line geometry                      */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — terminal extent + ruled-line spacing + cursor bounds.
 *
 * Note the asymmetry: line_step is the discrete Y-step; the X axis is free,
 * so no "cw" is carried.  max_line bounds the vertical line index;
 * max_col is just cols-1 (the cursor can sit anywhere along a line).
 */
typedef struct {
    int rows, cols;
    int line_step;           /* screen rows between ruled lines */
    int max_line, max_col;   /* cursor bounds */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->line_step = LINE_STEP;
    g->max_line = (rows - 1) / LINE_STEP - 1;
    g->max_col  = cols - 1;
}

/*
 * RULED GRID FORMULA — the cursor's (line, col) maps directly to screen as:
 *
 *   screen_row = line * line_step
 *   screen_col = col                  (free — no CELL_W scale)
 *
 * Grid line detection: a screen row sr is a ruled line when:
 *   sr % line_step == 0
 *
 * No vertical line condition — that's what makes it "ruled" not "rect".
 */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    (void)sc;
    return (sr % g->line_step == 0) ? '-' : ' ';
}

/*
 * ctx_draw_bg — paint each ruled row with '-', highlighting the active line.
 * The active line uses PAIR_ACTIVE; others use PAIR_LINE.
 */
static void ctx_draw_bg(const GridCtx *g, int active_line)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        if (ctx_grid_char(g, sr, 0) != '-') continue;
        int line = sr / g->line_step;
        int pair = (line == active_line) ? PAIR_ACTIVE : PAIR_LINE;
        attron(COLOR_PAIR(pair));
        for (int sc = 0; sc < g->cols; sc++)
            mvaddch(sr, sc, (chtype)'-');
        attroff(COLOR_PAIR(pair));
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — TWO INDEPENDENT AXES (kept separate from the (r,c) of other files):
 *   line — discrete vertical index (which ruled line)
 *   col  — free horizontal position in screen columns
 */
typedef struct {
    int line;
    int col;
} Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->line = 0;
    cur->col  = g->cols / 2;
}

/*
 * cursor_move — two independent axes:
 *   UP/DOWN    → dline ± 1
 *   LEFT/RIGHT → dcol ± 1, scaled by COL_STEP, clamped to [0, max_col]
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dline, int dcol)
{
    int nl = cur->line + dline;
    int nc = cur->col  + dcol * COL_STEP;
    if (nl >= 0 && nl <= g->max_line) cur->line = nl;
    if (nc >= 0 && nc <= g->max_col)  cur->col  = nc;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr = cur->line * g->line_step;
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, cur->col, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  line=%d  col=%d  screen(%d,%d) ",
        fps, cur->line, cur->col, cur->line * g->line_step, cur->col);
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
    ctx_draw_bg(g, cur->line);
    cursor_draw(cur, g);
    hud_draw(g, cur, fps);
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
