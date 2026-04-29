/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_brick_stagger.c — horizontal brick (staggered row) grid
 *
 * DEMO: Odd-numbered rows shift right by half a cell width — exactly like
 *       bricks in a wall. The cell_to_screen formula adds (row % 2) * HALF_W
 *       to the column. Move '@' to see how the stagger affects neighbours.
 *
 * Study alongside: 01_uniform_rect.c (no stagger), 07_half_brick_vert.c
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, HALF_W = CELL_W/2
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs
 *   §4 formula  — stagger formula: sc = c*CW + (r%2)*HALF_W
 *   §5 player   — struct, move (movement is row/col unchanged)
 *   §6 scene    — draw_grid(), draw_player()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/06_brick_stagger.c \
 *       -o 06_brick_stagger -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Rectangular grid with an alternating column offset.
 *                  Even rows (r % 2 == 0) align normally.
 *                  Odd  rows (r % 2 == 1) shift right by HALF_W = CELL_W / 2.
 *
 * Stagger formula: ONLY the horizontal part of cell_to_screen changes:
 *
 *   screen_row = r * CELL_H                      ← unchanged from 01
 *   screen_col = c * CELL_W + (r % 2) * HALF_W   ← +HALF_W on odd rows
 *
 *   (r % 2) evaluates to 0 for even rows, 1 for odd rows.
 *   Multiplying by HALF_W gives the conditional shift cleanly.
 *
 * Grid line draw : Vertical lines are now row-dependent:
 *                    even rows: vertical line at sc % CELL_W == 0
 *                    odd  rows: vertical line at (sc + HALF_W) % CELL_W == 0
 *                               → sc % CELL_W == CELL_W - HALF_W == HALF_W
 *                  Horizontal lines: sr % CELL_H == 0 (unchanged)
 *
 * Movement       : Arrow keys change (r,c) exactly as in 01_uniform_rect.
 *                  The stagger is purely visual — the logical grid topology
 *                  (each cell has 4 neighbours) is unchanged.
 *
 * References     :
 *   Brick bond pattern — en.wikipedia.org/wiki/Brick#Bonds
 *   Offset coordinates for hex grids (same idea) — redblobgames.com/grids/hexagons
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Start with a uniform rectangular grid.  Now push every odd row
 * horizontally by half a cell width (HALF_W).  The horizontal lines
 * don't move — they still span the full screen at the same rows.
 * ONLY the vertical lines shift, and they shift by different amounts
 * depending on which row band you are in.  That's the entire change.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Visualise a wall of bricks: even rows have joints at positions
 * 0, CW, 2*CW, ...; odd rows have joints at CW/2, 3*CW/2, 5*CW/2, ...
 * The mortar lines between rows (horizontal) are the same in both cases.
 * The mortar lines within a row (vertical) are staggered.
 *
 * The key insight: the grid has TWO "modes" alternating by row index.
 * Mode 0 (even rows): vertical lines at sc % CELL_W == 0
 * Mode 1 (odd  rows): vertical lines at sc % CELL_W == HALF_W
 *
 * You switch between modes using `row_band = sr / CELL_H`.
 * The parity of row_band determines which mode is active.
 *
 * DRAWING METHOD
 * ──────────────
 *  Per screen position (sr, sc):
 *
 *  1. Horizontal lines (unchanged from uniform rect):
 *       on_h = (sr % CELL_H == 0)
 *
 *  2. Vertical lines (row-band dependent):
 *       row_band = sr / CELL_H           <- which row are we in?
 *       if (row_band % 2 == 0):          <- even row band
 *           on_v = (sc % CELL_W == 0)
 *       else:                            <- odd row band: shifted
 *           on_v = (sc % CELL_W == HALF_W)
 *
 *  3. Character:  on_h && on_v -> '+',  on_h -> '-',  on_v -> '|'
 *
 *  THE AMBIGUITY AT HORIZONTAL LINES:
 *  When sr % CELL_H == 0, the screen row is at the boundary between two
 *  row bands.  sr/CELL_H gives the index of the NEXT (lower) row band.
 *  This is consistent because integer division truncates, so row sr=0
 *  gives band 0, row sr=CELL_H gives band 1, etc.  The horizontal line
 *  at sr = k*CELL_H "belongs" to the row band k (below the line).
 *
 * KEY FORMULAS
 * ────────────
 *  Forward (cell -> screen top-left):
 *    screen_row = r * CELL_H
 *    screen_col = c * CELL_W + (r % 2) * HALF_W     <- stagger on odd rows
 *
 *  Vertical line condition at screen position (sr, sc):
 *    band  = sr / CELL_H
 *    on_v  = (sc % CELL_W == (band % 2 == 0 ? 0 : HALF_W))
 *    Simplified: on_v = (sc % CELL_W == (band % 2) * HALF_W)
 *
 *  HALF_W:
 *    HALF_W = CELL_W / 2       (use integer division; CELL_W must be even)
 *
 *  Player cell right edge (important for grid_cols bound):
 *    odd rows extend to  (grid_cols-1)*CELL_W + HALF_W + CELL_W
 *    so: grid_cols = (screen_cols - HALF_W) / CELL_W
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CELL_W must be even: HALF_W = CELL_W/2.  If CELL_W is odd,
 *    HALF_W rounds down and the stagger is slightly less than half.
 *    The visual result is close but not perfectly symmetric.
 *
 *  • The last column of cells on odd rows extends HALF_W extra pixels
 *    to the right.  The player_reset() bound must subtract HALF_W from
 *    the available width: max_c = (cols - HALF_W) / CELL_W - 1.
 *    Forgetting this lets the player move into an off-screen cell.
 *
 *  • Rendering the rightmost vertical line of odd rows: since odd rows
 *    are shifted right by HALF_W, their rightmost vertical line is at
 *    sc = (max_c+1)*CELL_W + HALF_W.  If this exceeds COLS, the line
 *    is partially off-screen.  The raster scan stops at COLS naturally.
 *
 *  • Horizontal lines span the FULL screen width regardless of row band.
 *    They do not stagger.  A common mistake is shifting horizontal lines
 *    too — don't: horizontal lines should always be at sr%CELL_H==0.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Look at the top two rows of cells:
 *    Row 0 (even): vertical line at sc=0 and sc=CELL_W.
 *    Row 1 (odd):  vertical line at sc=HALF_W and sc=CELL_W+HALF_W.
 *  The odd-row joints should fall exactly between the even-row joints.
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
#define CELL_W      10   /* cols per cell; must be even for HALF_W */
#define CELL_H       4   /* rows per cell */
#define HALF_W      (CELL_W / 2)   /* = 5: the stagger offset */

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
    init_pair(PAIR_GRID,   COLORS >= 256 ? 214 : COLOR_RED,    -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — THE STAGGER FORMULA:
 *
 *   screen_row = r * CELL_H
 *   screen_col = c * CELL_W + (r % 2) * HALF_W
 *
 * Decompose the column part:
 *   base_col   = c * CELL_W          ← where the cell would be without stagger
 *   stagger    = (r % 2) * HALF_W    ← 0 for even rows, HALF_W for odd rows
 *   screen_col = base_col + stagger
 *
 * This is the ONLY change from 01_uniform_rect.
 */
static void cell_to_screen(int r, int c, int *sr, int *sc)
{
    *sr = r * CELL_H;
    *sc = c * CELL_W + (r % 2) * HALF_W;
}

/*
 * grid_char — grid line detection adapted for staggered rows.
 *
 * Horizontal lines: same as before — sr % CELL_H == 0
 *
 * Vertical lines depend on which row band sr falls in:
 *   row_index = sr / CELL_H  (integer division → which row the line is in)
 *   even row: v_line at sc % CELL_W == 0
 *   odd  row: v_line at sc % CELL_W == HALF_W
 *             (because the cell starts HALF_W to the right)
 *
 * At a horizontal grid line (sr % CELL_H == 0), it is a boundary between
 * two row bands. We draw a full '-' line here without gaps.
 */
static char grid_char(int sr, int sc)
{
    bool h = (sr % CELL_H == 0);

    /* which row band does this screen row belong to? */
    int row_idx = sr / CELL_H;
    bool v = (row_idx % 2 == 0)
             ? (sc % CELL_W == 0)           /* even row: normal alignment */
             : (sc % CELL_W == HALF_W);     /* odd row: shifted by HALF_W */

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
    /* Account for the stagger: odd rows are shifted right by HALF_W,
     * so the last column must be at most (cols - HALF_W) / CELL_W - 1 */
    p->max_c = (cols - HALF_W) / CELL_W - 1;
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

    /* Show stagger amount in HUD */
    int sr, sc; cell_to_screen(p->r, p->c, &sr, &sc);
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  screen_col=%d  stagger=%d ",
        fps, p->r, p->c, sc, (p->r % 2) * HALF_W);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " arrows:move  r:reset  q:quit  [06 brick stagger  HALF_W=%d] ", HALF_W);
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
