/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_hierarchical.c — two-level grid (minor cells inside major cells)
 *
 * DEMO: Major grid lines (thick, bright) every MAJOR_FACTOR minor cells.
 *       Minor grid lines (thin, dim) fill in between. Looks like graph
 *       paper. The cursor moves in minor-cell steps; the HUD shows both
 *       the minor cell address and the major cell it belongs to.
 *
 * Study alongside: 01_uniform_rect.c (single level), 04_coarse_sparse.c
 *
 * Section map:
 *   §1 config   — MINOR_W, MINOR_H, MAJOR_FACTOR (major = factor * minor)
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — minor / major / active / cursor / HUD / HINT pairs
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_grid_level / ctx_draw_bg
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
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
 * Cursor address : minor cell (mr, mc).
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
 *  Cursor dual address:
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

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

/* Color pair IDs */
#define PAIR_MINOR   1   /* thin dim lines for minor grid     */
#define PAIR_MAJOR   2   /* thick bright lines for major grid */
#define PAIR_ACTIVE  3   /* highlighted active cell           */
#define PAIR_CURSOR  4   /* '@'                               */
#define PAIR_HUD     5   /* status bar (yellow)               */
#define PAIR_HINT    6   /* key-hint footer (cyan)            */

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
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the cell ↔ screen mapping                    */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of a TWO-LEVEL grid plus cursor bounds.
 *
 * cw/ch are the MINOR cell size (movement unit). Major step is derived as
 * cw*factor / ch*factor and stored as mw/mh for the level test in §6.
 */
typedef struct {
    /* terminal extent */
    int rows, cols;

    /* cell size in screen characters — MINOR cell = movement unit */
    int cw, ch;

    /* major cell size (= minor * factor) */
    int mw, mh;
    int factor;                /* MAJOR_FACTOR */

    /* cursor bounds — last whole minor cell that fits in (rows-1) × cols */
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

/*
 * ctx_to_screen — minor cell (r,c) to screen.
 *
 *   screen_row = r * ch
 *   screen_col = c * cw
 *
 * Exactly the same formula as 01_uniform_rect — just MINOR_H/MINOR_W
 * instead of CELL_H/CELL_W.
 */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/*
 * ctx_grid_level — returns 2 (major line), 1 (minor line only), or 0.
 *
 * TWO-LEVEL DETECTION FORMULA:
 *
 *   on_major_h = (sr % mh == 0)   ← sr is a multiple of MAJOR_H
 *   on_major_v = (sc % mw == 0)
 *   on_minor_h = (sr % ch == 0)   ← also true for major lines
 *   on_minor_v = (sc % cw == 0)
 *
 * Check MAJOR first because every major line is also a minor line.
 * If we checked minor first, we'd never see major-only lines.
 */
typedef enum { LEVEL_NONE=0, LEVEL_MINOR=1, LEVEL_MAJOR=2 } GridLevel;

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

/*
 * ctx_draw_bg — paint both grid levels, with major drawn in the bright pair.
 */
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    /* Show both address levels in HUD */
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
