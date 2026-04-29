/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_fine_dense.c — tight small-cell rectangular grid
 *
 * DEMO: Same formula as 01_uniform_rect but CELL_W=4, CELL_H=2 — the
 *       minimum usable cell size. With cells this small the grid fills
 *       the terminal with many more rows and columns, demonstrating how
 *       parameter choice alone controls grid density.
 *
 * Study alongside: 01_uniform_rect.c (base), 04_coarse_sparse.c (large)
 *
 * Section map:
 *   §1 config   — CELL_W=4, CELL_H=2 (minimum practical cell)
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs
 *   §4 formula  — same formula as 01; only constants differ
 *   §5 player   — struct, move, reset
 *   §6 scene    — draw_grid(), draw_player()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/03_fine_dense.c \
 *       -o 03_fine_dense -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Identical to 01_uniform_rect. Only CELL_W and CELL_H
 *                  change. This file exists to show how the same formula
 *                  produces a completely different visual density.
 *
 * Minimum cell   : With CELL_H=2 and CELL_W=4 the interior of each cell
 *                  (inside the border lines) is exactly 1 row × 3 cols.
 *                  Going smaller would leave no interior space — the lines
 *                  would touch and the grid would become unreadable.
 *
 * Cell count     : On an 80×24 terminal:
 *                    cols / CELL_W = 80 / 4 = 20 columns
 *                    rows / CELL_H = 22 / 2 = 11 rows  (220 cells total)
 *                  Compare to 01_uniform_rect (8×4): 10 × 5 = 50 cells.
 *                  Same formula, 4.4× more cells just from halving cell size.
 *
 * Player cell    : With CELL_H=2 the interior is only 1 row tall.
 *                  The '@' sits on the single interior row; no dot fill.
 *
 * References     :
 *   Resolution vs cell-size tradeoff — any grid-game design article
 *   Same formula as 01_uniform_rect — see §4 there for full derivation
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The formula is IDENTICAL to 01_uniform_rect.  Only the constants differ.
 * This file exists to teach one thing: grid density is purely a function of
 * cell size.  Halving cell size quadruples the number of cells on screen.
 * No structural change — just a parameter change.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of cell size as a zoom level.  Large cells = zoomed out (few cells,
 * lots of interior space per cell).  Small cells = zoomed in (many cells,
 * tiny interior).  The same formula renders both; you just dial CELL_W/H.
 *
 * Density formula:   cells_on_screen = (COLS / CELL_W) * ((LINES-1) / CELL_H)
 *   CELL_W=8, CELL_H=4  on 80×24:  10 *  5 =  50 cells
 *   CELL_W=4, CELL_H=2  on 80×24:  20 * 11 = 220 cells    (4.4× more)
 *   CELL_W=2, CELL_H=1  on 80×24:  40 * 23 = 920 cells    (18× more!)
 *
 * DRAWING METHOD
 * ──────────────
 *  Exactly as in 01_uniform_rect.  The only decision is choosing CELL_W/H:
 *
 *  1. Decide the MINIMUM acceptable interior space per cell:
 *       interior_rows = CELL_H - 1      (rows inside the borders)
 *       interior_cols = CELL_W - 1
 *     For just a single char of interior: CELL_H=2, CELL_W=2.
 *     For a small '.' dot interior:       CELL_H=2, CELL_W=4 (3 interior cols).
 *
 *  2. Draw using the standard modular test.
 *
 *  3. For the player cell: with CELL_H=2, interior is only 1 row.
 *     Place '@' in that single interior row at the column centre.
 *     There is no room for '.' fill; just the '@' character.
 *
 * KEY FORMULAS
 * ────────────
 *  Same as 01.  The relationship to understand:
 *
 *  Interior size:
 *    interior_rows = CELL_H - 1      (CELL_H=2 -> 1 interior row)
 *    interior_cols = CELL_W - 1      (CELL_W=4 -> 3 interior cols)
 *
 *  First interior position of cell (r, c):
 *    first_interior_row = r * CELL_H + 1
 *    first_interior_col = c * CELL_W + 1
 *
 *  '@' placement (centre of interior row):
 *    at_row = r * CELL_H + 1              (CELL_H=2: only row available)
 *    at_col = c * CELL_W + CELL_W / 2
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CELL_H=1: the horizontal line IS the only row.  There is NO interior.
 *    Every row is a grid line — you cannot place '@' in the interior.
 *    Minimum: CELL_H=2 so you have at least 1 interior row.
 *
 *  • CELL_W=2: interior is 1 col wide — you can fit exactly one character.
 *    CELL_W=3: interior is 2 cols — you can fit '@' but not a label.
 *    CELL_W=4: interior is 3 cols — fits '@' centred and one space each side.
 *
 *  • Dense grids and ncurses performance: at CELL_W=2, CELL_H=1 you call
 *    mvaddch() for nearly every screen position.  On slow terminals this
 *    can reduce FPS.  Use erase() not clear(), and wnoutrefresh/doupdate
 *    to minimise actual writes.
 *
 *  • Player bounds recomputation after resize: the dense grid has many more
 *    cells, so resizing is more likely to need player_reset().
 *
 * HOW TO VERIFY
 * ─────────────
 *  Count '+' characters in the top row (sr=0).  With CELL_W=4 on 80 cols:
 *  '+' at cols 0, 4, 8, 12, ..., 76, 80 → 21 corners in row 0.
 *  Formula: floor(COLS / CELL_W) + 1 = 80/4 + 1 = 21. ✓
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

/*
 * CELL_W=4, CELL_H=2 — smallest useful cell.
 * Interior = (CELL_H-1)=1 row × (CELL_W-1)=3 cols.
 * Halving either value below this erases the cell interior entirely.
 */
#define CELL_W  4
#define CELL_H  2

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
    /* Dimmer grid color — with dense lines a bright color overwhelms */
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_BLUE,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — THE FORMULA (unchanged from 01_uniform_rect):
 *
 *   screen_row = r * CELL_H    (= r * 2)
 *   screen_col = c * CELL_W    (= c * 4)
 *
 * grid_char — unchanged from 01:
 *   sr % 2 == 0  →  horizontal line
 *   sc % 4 == 0  →  vertical line
 *
 * Every 2nd row and every 4th column is a grid line.
 * The 80×24 terminal has 11 horizontal lines and 20 vertical lines.
 */
static void cell_to_screen(int r, int c, int *sr, int *sc)
{
    *sr = r * CELL_H;
    *sc = c * CELL_W;
}

static char grid_char(int sr, int sc)
{
    bool h = (sr % CELL_H == 0);
    bool v = (sc % CELL_W == 0);
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
    p->max_r = (rows - 1) / CELL_H - 1;
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

    /* CELL_H=2: interior is exactly row sr+1, cols sc+1..sc+3 */
    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dc = 1; dc < CELL_W; dc++)
        mvaddch(sr + 1, sc + dc, (chtype)'-');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* '@' at centre of interior */
    attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
    mvaddch(sr + 1, sc + CELL_W / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Player *p, double fps)
{
    erase();
    draw_grid(rows, cols);
    draw_player(p);

    char buf[64];
    snprintf(buf, sizeof buf, " %.1f fps  cell(%d,%d)  grid%dx%d ",
             fps, p->r, p->c, p->max_c + 1, p->max_r + 1);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " arrows:move  r:reset  q:quit  [03 fine dense  %dx%d cells] ",
        CELL_W, CELL_H);
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
