/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_hierarchical.c — two-level grid (minor cells inside major cells)
 *
 * DEMO: Major grid lines (thick, bright) every MAJOR_FACTOR minor cells.
 *       Minor grid lines (thin, dim) fill in between. Looks like graph
 *       paper. The player moves in minor-cell steps; the HUD shows both
 *       the minor cell address and the major cell it belongs to.
 *
 * Study alongside: 01_uniform_rect.c (single level), 04_coarse_sparse.c
 *
 * Section map:
 *   §1 config   — MINOR_W, MINOR_H, MAJOR_FACTOR (major = factor * minor)
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: minor-grid, major-grid, active, player, HUD
 *   §4 formula  — grid_char_level() returns which level a line belongs to
 *   §5 player   — struct, move, reset
 *   §6 scene    — draw_grid(), draw_player()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/05_hierarchical.c \
 *       -o 05_hierarchical -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two nested uniform grids sharing the same origin.
 *                  Minor grid: step = (MINOR_W, MINOR_H) — fine subdivision.
 *                  Major grid: step = (MAJOR_W, MAJOR_H) = MAJOR_FACTOR * minor.
 *                  A screen position can be on a minor line, a major line, or
 *                  neither. Major lines are a subset of minor lines.
 *
 * Two-level test : For screen position (sr, sc):
 *                    on_minor_h = (sr % MINOR_H == 0)
 *                    on_minor_v = (sc % MINOR_W == 0)
 *                    on_major_h = (sr % MAJOR_H == 0)   ← implies on_minor_h
 *                    on_major_v = (sc % MAJOR_W == 0)   ← implies on_minor_v
 *                  Test major FIRST; if not major, fall through to minor.
 *                  MAJOR_H = MINOR_H * MAJOR_FACTOR → every MAJOR_FACTOR-th
 *                  minor line is also a major line.
 *
 * Player address : minor cell (mr, mc).
 *                  major cell it lives in: (mr / MAJOR_FACTOR, mc / MAJOR_FACTOR)
 *                  local index within major cell: (mr % MAJOR_FACTOR, mc % MAJOR_FACTOR)
 *
 * References     :
 *   Graph paper — en.wikipedia.org/wiki/Graph_paper
 *   Multi-resolution grids in games — redblobgames.com (search "hierarchical grid")
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two grids coexist at the same screen positions.  The major grid has a
 * large step; the minor grid has a small step.  Because MAJOR_H = MINOR_H *
 * FACTOR, every major line position is ALSO a minor line position — major
 * lines are a subset of minor lines.  The drawing trick: test major first
 * so that shared positions get the major visual style, not the minor one.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of graph paper: thin grey lines every 1mm, thick red lines every
 * 5mm.  The 5mm lines ARE at 1mm positions (5, 10, 15, ...).  If you draw
 * thin lines first and then thick lines on top, you get the right result.
 * But in one-pass rendering (one character per screen position), you must
 * CLASSIFY each position as "major" or "minor-only" or "neither", testing
 * in that priority order.
 *
 *   Each position is in exactly one category:
 *     MAJOR:      sr%MAJOR_H==0  OR  sc%MAJOR_W==0
 *     MINOR-ONLY: (sr%MINOR_H==0 OR sc%MINOR_W==0) AND NOT major
 *     INTERIOR:   neither
 *
 * DRAWING METHOD
 * ──────────────
 *  Per screen position (sr, sc), classify then draw:
 *
 *  1. Check major conditions:
 *       is_major_h = (sr % MAJOR_H == 0)
 *       is_major_v = (sc % MAJOR_W == 0)
 *     If either is true → draw with MAJOR style, stop.
 *
 *  2. Check minor conditions (only reached if NOT major):
 *       is_minor_h = (sr % MINOR_H == 0)
 *       is_minor_v = (sc % MINOR_W == 0)
 *     If either is true → draw with MINOR style, stop.
 *
 *  3. Otherwise → interior, skip.
 *
 *  WHY TEST MAJOR FIRST: At sr=8 with MAJOR_H=8, MINOR_H=2:
 *    sr%MAJOR_H = 8%8 = 0 (major!) AND sr%MINOR_H = 8%2 = 0 (also minor).
 *    If you tested minor first, this position would be classified as minor.
 *    Testing major first gives it the correct major classification.
 *
 * KEY FORMULAS
 * ────────────
 *  Setup:
 *    MAJOR_W = MINOR_W * MAJOR_FACTOR
 *    MAJOR_H = MINOR_H * MAJOR_FACTOR
 *
 *  Classification (test major before minor!):
 *    on_major_h = (sr % MAJOR_H == 0)    <- implies on_minor_h
 *    on_major_v = (sc % MAJOR_W == 0)    <- implies on_minor_v
 *    on_minor_h = (sr % MINOR_H == 0)    <- true even at major positions
 *    on_minor_v = (sc % MINOR_W == 0)
 *
 *  Player dual address:
 *    minor_cell_row = r
 *    minor_cell_col = c
 *    major_cell_row = r / MAJOR_FACTOR   (integer division)
 *    major_cell_col = c / MAJOR_FACTOR
 *    local_row_in_major = r % MAJOR_FACTOR
 *    local_col_in_major = c % MAJOR_FACTOR
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • If MAJOR_FACTOR=1: MAJOR_H == MINOR_H.  Every minor line is a major
 *    line.  The grid degenerates to a single level — all lines get the
 *    major style.  Minimum useful MAJOR_FACTOR is 2.
 *
 *  • Large MAJOR_FACTOR (e.g. 8) with small MINOR_H (e.g. 2): the major
 *    lines are far apart (every 16 rows).  On a 24-row terminal you may
 *    see only 2 major horizontal lines.
 *
 *  • Character choice matters: major and minor must be visually distinct.
 *    '='  vs '-' for horizontal, '|' vs ':' for vertical works well.
 *    Using colors is even better — see §3.
 *
 *  • Correct formula: only works if MAJOR_H is an EXACT multiple of MINOR_H.
 *    If MAJOR_H = MINOR_H * FACTOR + remainder, the subset property breaks
 *    and some major lines won't align with minor lines.
 *
 * HOW TO VERIFY
 * ─────────────
 *  With MAJOR_FACTOR=4, MINOR_H=2, MAJOR_H=8 on a 24-row terminal:
 *    Minor lines at rows: 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22 (12 lines)
 *    Major lines at rows: 0, 8, 16 (3 lines — subset of minor ✓)
 *  Count them on screen to confirm.
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

#define TARGET_FPS    30

/* Minor cell: the movement unit and smallest grid division */
#define MINOR_W       4    /* cols per minor cell */
#define MINOR_H       2    /* rows per minor cell */

/*
 * MAJOR_FACTOR — how many minor cells fit inside one major cell.
 * Major step = MAJOR_FACTOR * minor step.
 *   MAJOR_W = MINOR_W * MAJOR_FACTOR = 4 * 4 = 16 cols
 *   MAJOR_H = MINOR_H * MAJOR_FACTOR = 2 * 4 =  8 rows
 * Change MAJOR_FACTOR to 3 for a 3×3 subdivision, 5 for 5×5, etc.
 */
#define MAJOR_FACTOR  4
#define MAJOR_W       (MINOR_W * MAJOR_FACTOR)
#define MAJOR_H       (MINOR_H * MAJOR_FACTOR)

#define PAIR_MINOR   1   /* thin dim lines for minor grid  */
#define PAIR_MAJOR   2   /* thick bright lines for major grid */
#define PAIR_ACTIVE  3   /* highlighted active cell        */
#define PAIR_PLAYER  4   /* '@'                            */
#define PAIR_HUD     5

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
    init_pair(PAIR_MINOR,  COLORS >= 256 ?  87 : COLOR_BLUE,   -1);
    init_pair(PAIR_MAJOR,  COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  22 : COLOR_GREEN,  -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — minor cell (r,c) to screen.
 *
 *   screen_row = r * MINOR_H
 *   screen_col = c * MINOR_W
 *
 * Exactly the same formula as 01_uniform_rect — just MINOR_H/MINOR_W
 * instead of CELL_H/CELL_W.
 */
static void cell_to_screen(int r, int c, int *sr, int *sc)
{
    *sr = r * MINOR_H;
    *sc = c * MINOR_W;
}

/*
 * grid_level — returns 2 (major line), 1 (minor line only), or 0 (interior).
 *
 * TWO-LEVEL DETECTION FORMULA:
 *
 *   on_major_h = (sr % MAJOR_H == 0)   ← sr is a multiple of MAJOR_H
 *   on_major_v = (sc % MAJOR_W == 0)
 *   on_minor_h = (sr % MINOR_H == 0)   ← also true for major lines
 *   on_minor_v = (sc % MINOR_W == 0)
 *
 * Check MAJOR first because every major line is also a minor line.
 * If we checked minor first, we'd never see major-only lines.
 */
typedef enum { LEVEL_NONE=0, LEVEL_MINOR=1, LEVEL_MAJOR=2 } GridLevel;

static GridLevel grid_level(int sr, int sc, bool *is_h, bool *is_v)
{
    bool mj_h = (sr % MAJOR_H == 0);
    bool mj_v = (sc % MAJOR_W == 0);
    bool mn_h = (sr % MINOR_H == 0);
    bool mn_v = (sc % MINOR_W == 0);

    if (mj_h || mj_v) { *is_h = mj_h; *is_v = mj_v; return LEVEL_MAJOR; }
    if (mn_h || mn_v) { *is_h = mn_h; *is_v = mn_v; return LEVEL_MINOR; }
    *is_h = *is_v = false;
    return LEVEL_NONE;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  player                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int r, c, max_r, max_c; } Player;

static void player_reset(Player *p, int rows, int cols)
{
    p->max_r = (rows - 1) / MINOR_H - 1;
    p->max_c = cols / MINOR_W - 1;
    p->r = p->max_r / 2;
    p->c = p->max_c / 2;
}
static void player_move(Player *p, int dr, int dc)
{
    int nr = p->r + dr, nc = p->c + dc;
    if (nr >= 0 && nr <= p->max_r) p->r = nr;
    if (nc >= 0 && nc <= p->max_c) p->c = nc;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void draw_grid(int rows, int cols)
{
    for (int sr = 0; sr < rows - 1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            bool is_h, is_v;
            GridLevel lvl = grid_level(sr, sc, &is_h, &is_v);
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

static void draw_player(const Player *p)
{
    int sr, sc;
    cell_to_screen(p->r, p->c, &sr, &sc);

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dc = 1; dc < MINOR_W; dc++)
        mvaddch(sr + 1, sc + dc, (chtype)' ');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
    mvaddch(sr + 1, sc + MINOR_W / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Player *p, double fps)
{
    erase();
    draw_grid(rows, cols);
    draw_player(p);

    /* Show both address levels in HUD */
    int maj_r = p->r / MAJOR_FACTOR, maj_c = p->c / MAJOR_FACTOR;
    int loc_r = p->r % MAJOR_FACTOR, loc_c = p->c % MAJOR_FACTOR;
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  minor(%d,%d)  major(%d,%d)  local(%d,%d) ",
        fps, p->r, p->c, maj_r, maj_c, loc_r, loc_c);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " arrows:move  r:reset  q:quit  [05 hierarchical  factor=%d] ",
        MAJOR_FACTOR);
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
