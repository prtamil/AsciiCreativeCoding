/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_uniform_rect.c — standard rectangular grid, the base formula
 *
 * DEMO: Uniformly spaced horizontal and vertical lines divide the terminal
 *       into equal rectangles. Move '@' between cells with arrow keys.
 *       This is the root formula — every other file in this series modifies
 *       exactly one part of it.
 *
 * Study alongside: 02_square.c (aspect correction), 05_hierarchical.c
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H and constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs: grid / active-cell / player / HUD
 *   §4 formula  — cell_to_screen() and grid_char(): the core mapping
 *   §5 player   — struct, move, reset
 *   §6 scene    — draw_grid(), draw_player(), scene_draw()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/01_uniform_rect.c \
 *       -o 01_uniform_rect -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Cartesian product of two uniform 1-D lattices.
 *                  Column lines at x = k * CELL_W for integer k.
 *                  Row    lines at y = k * CELL_H for integer k.
 *                  Every cell (r,c) occupies the region:
 *                    rows [r*CELL_H .. (r+1)*CELL_H)
 *                    cols [c*CELL_W .. (c+1)*CELL_W)
 *
 * Formula        : cell_to_screen — the anchor (top-left) of cell (r,c):
 *                    screen_row = r * CELL_H
 *                    screen_col = c * CELL_W
 *                  This is a pure scale. Grid space = screen space / cell_size.
 *
 * Grid lines     : A screen position (sr, sc) sits on a line when:
 *                    sr % CELL_H == 0  →  horizontal line  '-'
 *                    sc % CELL_W == 0  →  vertical line    '|'
 *                    both              →  corner           '+'
 *
 * Movement       : delta (dr,dc) ∈ {(±1,0),(0,±1)}, then clamp to bounds.
 *                  Exactly 4-connected — neighbours share a full edge.
 *
 * References     :
 *   Cartesian coordinate system — en.wikipedia.org/wiki/Cartesian_coordinate_system
 *   Terminal grid model — this project's documentation/Architecture.md §4
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A rectangular grid is the visual result of asking ONE question at every
 * screen position: "Is my row a multiple of CELL_H, or my column a multiple
 * of CELL_W?"  That single modular test — applied per-pixel — produces the
 * entire grid with no arrays, no loops over cells, no stored data.
 * Everything derives from arithmetic on screen coordinates.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture the terminal as graph paper.  The paper itself has no content —
 * only a repeating pattern of intersecting lines.  The lines appear wherever
 * a screen coordinate is divisible by the cell step.  Between any two
 * consecutive vertical lines there are exactly (CELL_W - 1) blank columns;
 * between any two horizontal lines there are (CELL_H - 1) blank rows.
 * The "cell" is just the rectangular space between four lines.
 *
 * You never store a grid.  You never allocate cells.  You ask the formula.
 *
 * DRAWING METHOD  (raster-scan, the approach used here)
 * ──────────────
 *  1. Pick CELL_W and CELL_H — the spacing between lines in each axis.
 *  2. Loop over every screen row sr from 0 to LINES-2 (leave bottom for HUD).
 *  3. Loop over every screen column sc from 0 to COLS-1.
 *  4. Evaluate:
 *       on_h = (sr % CELL_H == 0)      <- this row is a horizontal line
 *       on_v = (sc % CELL_W == 0)      <- this col is a vertical line
 *  5. Choose the character:
 *       on_h && on_v  =>  '+'    (two lines cross here)
 *       on_h only     =>  '-'    (horizontal segment)
 *       on_v only     =>  '|'    (vertical segment)
 *       neither       =>  ' '    (interior — skip the mvaddch call)
 *  6. Draw.  Done.  No other data structure needed.
 *
 * KEY FORMULAS
 * ────────────
 *  Forward  (cell index  -> screen top-left corner):
 *    screen_row = r * CELL_H
 *    screen_col = c * CELL_W
 *
 *  Inverse  (screen position -> which cell contains it):
 *    cell_row = sr / CELL_H      (integer division truncates)
 *    cell_col = sc / CELL_W
 *
 *  On a grid line:
 *    horizontal line  at sr when  sr % CELL_H == 0
 *    vertical   line  at sc when  sc % CELL_W == 0
 *
 *  Interior of cell (r, c):
 *    rows  r*CELL_H + 1  ..  (r+1)*CELL_H - 1
 *    cols  c*CELL_W + 1  ..  (c+1)*CELL_W - 1
 *
 *  Number of visible complete cells:
 *    grid_cols = COLS  / CELL_W
 *    grid_rows = (LINES-1) / CELL_H
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CELL_H=1 or CELL_W=1: every position is a grid line — the "grid"
 *    becomes a solid mesh of '+' and '-' and '|'.  No usable interior.
 *    Minimum for a visible cell interior: CELL_H >= 2, CELL_W >= 2.
 *
 *  • Off-by-one on the last row: row LINES-1 is reserved for the HUD.
 *    Stop the raster scan at sr < LINES-1, not sr < LINES.
 *
 *  • Terminal resize: CELL_H and CELL_W are constants, but LINES and COLS
 *    change.  After resize, recompute grid_rows/cols and clamp the player
 *    position to the new bounds — otherwise the player can be off-screen.
 *
 *  • Cell border ownership: the top border of cell (r,c) is at screen row
 *    r*CELL_H.  That border row is ALSO the bottom border of cell (r-1,c).
 *    One screen row serves two cells.  This is correct — don't double-draw.
 *
 *  • The bottom-right cell may be incomplete if COLS or LINES is not a
 *    perfect multiple of CELL_W or CELL_H.  The raster scan handles this
 *    automatically (it just stops at the screen edge), but player_reset()
 *    must account for it by using integer division without +1.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Vertical line count  = floor(COLS / CELL_W) + 1  (including col 0)
 *  Horizontal line count= floor((LINES-1) / CELL_H) + 1
 *  For an 80x24 terminal with CELL_W=8, CELL_H=4:
 *    vertical lines at cols 0,8,16,24,32,40,48,56,64,72  -> 10 lines
 *    horizontal lines at rows 0,4,8,12,16,20              ->  6 lines
 *
 *  Quick sanity checks:
 *    (sr=0, sc=0) -> '+' (both conditions true)
 *    (sr=1, sc=0) -> '|' (on_v only)
 *    (sr=0, sc=1) -> '-' (on_h only)
 *    (sr=1, sc=1) -> ' ' (interior)
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
 * Cell dimensions in terminal characters.
 *
 *   CELL_W — columns per cell (distance between vertical grid lines)
 *   CELL_H — rows    per cell (distance between horizontal grid lines)
 *
 * Terminal characters are roughly 2× taller than wide in pixels.
 * With CELL_W=8, CELL_H=4:
 *   pixel width  = 8  * ~8 px  = ~64 px
 *   pixel height = 4  * ~16 px = ~64 px
 * → cells appear approximately square on screen.
 * See 02_square.c for the exact aspect-ratio derivation.
 */
#define CELL_W  8   /* terminal columns per grid cell */
#define CELL_H  4   /* terminal rows    per grid cell */

/* Color pair IDs */
#define PAIR_GRID    1   /* dim grid lines         */
#define PAIR_ACTIVE  2   /* highlighted cell fill  */
#define PAIR_PLAYER  3   /* bright '@'             */
#define PAIR_HUD     4   /* status bar             */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void color_init(void)
{
    start_color();
    use_default_colors();
    /* 8-color fallback values after the 256-color values */
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — cell ↔ screen mapping                                    */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — top-left corner of cell (r,c) in screen coordinates.
 *
 * THE FORMULA (uniform rectangular grid):
 *
 *   screen_row = r * CELL_H
 *   screen_col = c * CELL_W
 *
 * Reading it: to go one cell right (+c), move CELL_W columns right.
 *             to go one cell down  (+r), move CELL_H rows   down.
 * The origin cell (0,0) maps to screen corner (0,0).
 *
 * All 14 grids in this series share this §4 structure.
 * Only the formula inside changes.
 */
static void cell_to_screen(int r, int c, int *sr, int *sc)
{
    *sr = r * CELL_H;
    *sc = c * CELL_W;
}

/*
 * grid_char — what character to draw at screen position (sr, sc).
 *
 * FORMULA (grid line detection via modular arithmetic):
 *
 *   on_h_line = (sr % CELL_H == 0)   ← screen row is a multiple of CELL_H
 *   on_v_line = (sc % CELL_W == 0)   ← screen col is a multiple of CELL_W
 *
 * Character selection:
 *   both  → '+'   corner where a row-line and col-line cross
 *   h     → '-'   horizontal segment
 *   v     → '|'   vertical segment
 *   none  → ' '   interior of a cell
 *
 * Why modular arithmetic? Because cell_to_screen maps integer cell coords
 * to multiples of CELL_H/CELL_W. The grid lines ARE those multiples.
 */
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

typedef struct {
    int r, c;        /* cell-space position (row, col in the grid) */
    int max_r;       /* grid bounds — set from terminal size        */
    int max_c;
} Player;

static void player_reset(Player *p, int rows, int cols)
{
    p->max_r = (rows - 1) / CELL_H - 1;   /* last whole cell that fits */
    p->max_c = cols / CELL_W - 1;
    p->r = p->max_r / 2;
    p->c = p->max_c / 2;
}

/*
 * player_move — apply (dr, dc) and clamp to grid bounds.
 *
 * MOVEMENT FORMULA:
 *   new_r = clamp(r + dr,  0, max_r)
 *   new_c = clamp(c + dc,  0, max_c)
 *
 * (dr,dc) comes from arrow key: UP=(-1,0), DOWN=(+1,0),
 *                                LEFT=(0,-1), RIGHT=(0,+1).
 * The clamp prevents the player from leaving the visible grid.
 */
static void player_move(Player *p, int dr, int dc)
{
    int nr = p->r + dr, nc = p->c + dc;
    if (nr >= 0 && nr <= p->max_r) p->r = nr;
    if (nc >= 0 && nc <= p->max_c) p->c = nc;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * draw_grid — raster scan: evaluate grid_char() at every screen position.
 *
 * This is the "analytical" rendering method: instead of storing cell data
 * in an array, we compute whether each screen position is a grid line
 * directly from the formula. Works for any terminal size without resizing
 * any data structure.
 */
static void draw_grid(int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < rows - 1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            char ch = grid_char(sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/*
 * draw_player — fill the active cell and place '@' at its centre.
 *
 * Cell interior = rows (sr+1 .. sr+CELL_H-1) × cols (sc+1 .. sc+CELL_W-1)
 * (the region strictly inside the grid lines).
 *
 * Centre formula:
 *   centre_row = sr + CELL_H / 2
 *   centre_col = sc + CELL_W / 2
 */
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

    /* HUD — top right */
    char buf[64];
    snprintf(buf, sizeof buf, " %.1f fps  cell(%d,%d) ", fps, p->r, p->c);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Key hints — bottom left */
    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0, " arrows:move  r:reset  q:quit  [01 uniform rect] ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_DIM);

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
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

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
    double fps = TARGET_FPS;
    int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            player_reset(&player, rows, cols);
        }
        int ch = getch();
        switch (ch) {
            case 'q': case 27:  g_running = 0;                       break;
            case 'r':           player_reset(&player, rows, cols);    break;
            case KEY_UP:        player_move(&player, -1,  0);         break;
            case KEY_DOWN:      player_move(&player, +1,  0);         break;
            case KEY_LEFT:      player_move(&player,  0, -1);         break;
            case KEY_RIGHT:     player_move(&player,  0, +1);         break;
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (now - t0 + 1)) * 0.05;
        t0  = now;

        scene_draw(rows, cols, &player, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
