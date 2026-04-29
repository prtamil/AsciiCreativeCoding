/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 07_half_brick_vert.c — vertical brick (staggered column) grid
 *
 * DEMO: Odd-numbered columns shift down by half a cell height. This is
 *       the exact transpose of 06_brick_stagger: the row formula gets the
 *       (col % 2) offset, while the column formula stays linear.
 *       Compare the two files side-by-side to see the symmetry.
 *
 * Study alongside: 06_brick_stagger.c (horizontal version), 01_uniform_rect.c
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, HALF_H = CELL_H/2
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs
 *   §4 formula  — vertical stagger: sr = r*CH + (c%2)*HALF_H
 *   §5 player   — struct, move, reset
 *   §6 scene    — draw_grid(), draw_player()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/07_half_brick_vert.c \
 *       -o 07_half_brick_vert -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Transpose of 06_brick_stagger.
 *                  Even cols align normally.
 *                  Odd  cols shift DOWN by HALF_H = CELL_H / 2.
 *
 * Stagger formula: ONLY the vertical part of cell_to_screen changes:
 *
 *   screen_row = r * CELL_H + (c % 2) * HALF_H   ← +HALF_H on odd cols
 *   screen_col = c * CELL_W                        ← unchanged from 01
 *
 * Grid line draw : Horizontal lines are now column-dependent:
 *                    even cols: h_line at sr % CELL_H == 0
 *                    odd  cols: h_line at sr % CELL_H == HALF_H
 *                  Vertical lines: sc % CELL_W == 0 (unchanged)
 *
 * Symmetry       : 06_brick_stagger swaps (r,c) and (CELL_H,CELL_W) roles.
 *                  The only code difference is:
 *                    06: screen_col += (r%2)*HALF_W   v-lines row-dependent
 *                    07: screen_row += (c%2)*HALF_H   h-lines col-dependent
 *
 * References     :
 *   Vertical brick bond — same concept as 06, transposed axis
 *   Offset hex grids (flat-top orientation) — redblobgames.com/grids/hexagons
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * This is the EXACT transpose of 06_brick_stagger.  Swap "row" and "col"
 * everywhere in file 06, swap CELL_H and CELL_W, swap HALF_W and HALF_H,
 * and you get this file.  The vertical lines don't move; the horizontal
 * lines stagger up/down based on which column band you are in.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Rotate the brick wall 90°.  Now the bricks are stacked vertically and
 * each column of bricks shifts up or down by half a cell height.
 *
 * The two modes:
 *   Mode 0 (even column bands): horizontal lines at sr % CELL_H == 0
 *   Mode 1 (odd  column bands): horizontal lines at sr % CELL_H == HALF_H
 *
 * Column band: col_band = sc / CELL_W.  Parity of col_band selects mode.
 *
 * Symmetry table — all code differences between 06 and 07:
 *   06 (brick horiz)       07 (brick vert)
 *   ──────────────         ──────────────
 *   stagger on rows        stagger on cols
 *   screen_col += (r%2)*HALF_W    screen_row += (c%2)*HALF_H
 *   on_h = sr%CH==0  (fixed)      on_v = sc%CW==0  (fixed)
 *   on_v  depends on row_band     on_h  depends on col_band
 *   max_c = (cols-HALF_W)/CW      max_r = (rows-HALF_H)/CH
 *
 * DRAWING METHOD
 * ──────────────
 *  Per screen position (sr, sc):
 *
 *  1. Vertical lines (unchanged from uniform rect):
 *       on_v = (sc % CELL_W == 0)
 *
 *  2. Horizontal lines (column-band dependent):
 *       col_band = sc / CELL_W
 *       if (col_band % 2 == 0):
 *           on_h = (sr % CELL_H == 0)
 *       else:
 *           on_h = (sr % CELL_H == HALF_H)
 *
 *  3. Character: on_h && on_v -> '+',  on_h -> '-',  on_v -> '|'
 *
 * KEY FORMULAS
 * ────────────
 *  Forward (cell -> screen top-left):
 *    screen_row = r * CELL_H + (c % 2) * HALF_H
 *    screen_col = c * CELL_W                        <- unchanged
 *
 *  Horizontal line condition at (sr, sc):
 *    col_band = sc / CELL_W
 *    on_h     = (sr % CELL_H == (col_band % 2) * HALF_H)
 *
 *  HALF_H = CELL_H / 2     (CELL_H must be even)
 *
 *  Player row bound (odd cols extend further down):
 *    max_r = (rows - 1 - HALF_H) / CELL_H - 1
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CELL_H must be even for a symmetric stagger.  Odd CELL_H gives
 *    HALF_H = CELL_H/2 rounded down — the shift is asymmetric.
 *
 *  • The bottom of odd columns extends HALF_H rows further than even
 *    columns.  Subtract HALF_H from available height in player_reset().
 *
 *  • Vertical lines span the FULL screen height regardless of col band.
 *    Do NOT stagger the vertical lines (sc%CELL_W==0 always, no shift).
 *
 *  • The ambiguity at vertical lines (sc%CELL_W==0) is the same as the
 *    horizontal ambiguity in 06: the line "belongs" to the col_band to
 *    its right (sc/CELL_W gives the next band index).
 *
 * HOW TO VERIFY
 * ─────────────
 *  Look at the first two columns of cells:
 *    Col 0 (even): horizontal line at rows 0, CELL_H, 2*CELL_H, ...
 *    Col 1 (odd):  horizontal line at rows HALF_H, CELL_H+HALF_H, ...
 *  The odd-col joints should be exactly between the even-col joints.
 *  Compare visually with 06_brick_stagger rotated 90°.
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

#define TARGET_FPS  30
#define CELL_W       8   /* cols per cell */
#define CELL_H       6   /* rows per cell; must be even for HALF_H */
#define HALF_H      (CELL_H / 2)   /* = 3: the vertical stagger offset */

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
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
    init_pair(PAIR_GRID,   COLORS >= 256 ? 220 : COLOR_YELLOW, -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — THE VERTICAL STAGGER FORMULA:
 *
 *   screen_row = r * CELL_H + (c % 2) * HALF_H   ← row shifts on odd cols
 *   screen_col = c * CELL_W                        ← unchanged from 01
 *
 * Compare to 06_brick_stagger:
 *   06:  screen_col = c * CELL_W + (r % 2) * HALF_W   ← col shifts on odd rows
 *   07:  screen_row = r * CELL_H + (c % 2) * HALF_H   ← row shifts on odd cols
 * Perfect axis swap.
 */
static void cell_to_screen(int r, int c, int *sr, int *sc)
{
    *sr = r * CELL_H + (c % 2) * HALF_H;
    *sc = c * CELL_W;
}

/*
 * grid_char — grid line detection for vertically staggered grid.
 *
 * Vertical lines: same as 01 — sc % CELL_W == 0
 *
 * Horizontal lines depend on which column band sc falls in:
 *   col_index = sc / CELL_W
 *   even col: h_line at sr % CELL_H == 0
 *   odd  col: h_line at sr % CELL_H == HALF_H
 *             (because the cell is shifted DOWN by HALF_H)
 */
static char grid_char(int sr, int sc)
{
    bool v = (sc % CELL_W == 0);

    int col_idx = sc / CELL_W;
    bool h = (col_idx % 2 == 0)
             ? (sr % CELL_H == 0)
             : (sr % CELL_H == HALF_H);

    if (h && v) return '+';
    if (h)      return '-';
    if (v)      return '|';
    return ' ';
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  player                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int r, c, max_r, max_c; } Player;

static void player_reset(Player *p, int rows, int cols)
{
    /* Odd columns extend HALF_H further down, so reduce max row */
    p->max_r = (rows - 1 - HALF_H) / CELL_H - 1;
    p->max_c = cols / CELL_W - 1;
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
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < rows - 1; sr++)
        for (int sc = 0; sc < cols; sc++) {
            char ch = grid_char(sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    attroff(COLOR_PAIR(PAIR_GRID));
}

static void draw_player(const Player *p)
{
    int sr, sc;
    cell_to_screen(p->r, p->c, &sr, &sc);

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = 1; dr < CELL_H; dr++)
        for (int dc = 1; dc < CELL_W; dc++)
            mvaddch(sr + dr, sc + dc, (chtype)'.');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
    mvaddch(sr + CELL_H / 2, sc + CELL_W / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Player *p, double fps)
{
    erase();
    draw_grid(rows, cols);
    draw_player(p);

    int sr, sc; cell_to_screen(p->r, p->c, &sr, &sc);
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  screen_row=%d  stagger=%d ",
        fps, p->r, p->c, sr, (p->c % 2) * HALF_H);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " arrows:move  r:reset  q:quit  [07 half-brick vert  HALF_H=%d] ", HALF_H);
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
