/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_coarse_sparse.c — large-cell rectangular grid with coordinate labels
 *
 * DEMO: CELL_W=20, CELL_H=6. The large interior space of each cell is
 *       used to display its (row,col) coordinate label, making the grid
 *       read like a map. This shows what a grid cell actually IS:
 *       a named region of screen space.
 *
 * Study alongside: 03_fine_dense.c (small cells), 01_uniform_rect.c (base)
 *
 * Section map:
 *   §1 config   — CELL_W=20, CELL_H=6
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs
 *   §4 formula  — same formula as 01; label placement formula added
 *   §5 player   — struct, move, reset
 *   §6 scene    — draw_grid(), draw_labels(), draw_player()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/04_coarse_sparse.c \
 *       -o 04_coarse_sparse -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Same Cartesian grid formula. The large cell size lets us
 *                  place a text label inside each cell, turning the grid
 *                  into a visible coordinate system.
 *
 * Label position : The label "(r,c)" is placed at:
 *                    label_row = sr + 1             (first interior row)
 *                    label_col = sc + 1             (first interior col)
 *                  where (sr, sc) = cell_to_screen(r, c).
 *                  With CELL_H=6 there are 5 interior rows — plenty of space.
 *
 * Why large cells : A real-world use case is a map grid (room layout,
 *                  dungeon floor, spreadsheet). The coordinate label inside
 *                  each cell is the first step toward named grid regions.
 *
 * References     :
 *   Grid-based maps in games — www.redblobgames.com/pathfinding/grids/graphs.html
 *   Same formula as 01_uniform_rect — see §4 there for derivation
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * When cells are large, they become REGIONS — named, addressable areas of
 * screen space.  The grid lines are now "walls" and the cell interior is
 * "floor".  A text label "(row,col)" placed inside each cell turns the
 * grid into a navigable coordinate map.  This is the mental model behind
 * map grids, spreadsheets, dungeon rooms, and game tile editors.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Stop thinking of cells as tiny pixels.  Think of them as rooms in a
 * building.  Each room has:
 *   - An address: (row, col) — its grid coordinate.
 *   - A floor area: the interior characters.
 *   - Walls: the grid line characters that border it.
 *
 * The cell formula maps the room address to its top-left wall corner.
 * Everything else about the room is computed relative to that corner.
 *
 *   top-left corner: (r*CELL_H, c*CELL_W)
 *   top-right corner: (r*CELL_H, (c+1)*CELL_W)
 *   bottom-left: ((r+1)*CELL_H, c*CELL_W)
 *   floor (interior top-left): (r*CELL_H+1, c*CELL_W+1)
 *   floor size: (CELL_H-1) rows  x  (CELL_W-1) cols
 *   centre: (r*CELL_H + CELL_H/2, c*CELL_W + CELL_W/2)
 *
 * DRAWING METHOD
 * ──────────────
 *  Phase 1 — draw grid lines (same as 01_uniform_rect):
 *    raster scan; on_h = sr%CELL_H==0; on_v = sc%CELL_W==0.
 *
 *  Phase 2 — fill cell content:
 *    For each cell (r, c), compute top-left corner (sr, sc) via cell_to_screen.
 *    Place label at (sr + 1, sc + 2) — first interior row, slight left indent.
 *    Place '@' at centre: (sr + CELL_H/2, sc + CELL_W/2).
 *
 *  ORDER MATTERS: draw grid lines first, then content on top.
 *  If you draw content first and grid lines second, the '+'/'−'/'|'
 *  characters will overwrite the first column/row of your label.
 *  Alternatively: draw grid lines ONLY on borders and skip interior positions.
 *
 * KEY FORMULAS
 * ────────────
 *  Cell top-left corner (sr, sc):
 *    sr = r * CELL_H,   sc = c * CELL_W
 *
 *  Label position (top-left of interior, with indent):
 *    label_row = sr + 1                <- first interior row
 *    label_col = sc + 2                <- 2 cols from left border
 *
 *  Cell centre (for '@'):
 *    centre_row = sr + CELL_H / 2
 *    centre_col = sc + CELL_W / 2
 *
 *  Label character budget (max label length before truncation):
 *    max_label_len = CELL_W - 3        (2 left indent + 1 right margin)
 *    For CELL_W=20: max_label_len = 17 chars — plenty for "(rr,cc)".
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Label truncation: if CELL_W < label_length + 3, the label will overflow
 *    into the right border or neighbouring cell.  Use snprintf with a buffer
 *    sized generously (at least 24 bytes for "(row,col)" with 2-digit coords).
 *
 *  • Player cell: skip the label draw for the player's cell — the player
 *    draws its own content.  Otherwise you get label text drawn under '@'.
 *
 *  • Iterating cells vs. raster scan: for coarse grids it is more efficient
 *    to iterate cells (two nested loops over r,c) than to raster-scan every
 *    pixel and check if it is in a cell interior.
 *
 *  • Bottom/right partial cells: if LINES or COLS is not a multiple of
 *    CELL_H or CELL_W, the last row or column of cells is truncated.
 *    The label may be clipped.  Guard: only draw label if sr+1 < LINES-1
 *    and sc+2 < COLS.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Each visible cell should show exactly one "(r,c)" label.  If labels
 *  appear twice or overlap: check draw order (grid lines must come after
 *  interior fill, or you must deliberately not draw over interior).
 *  If label is missing from last column: increase buffer size or check
 *  the max_c computation in player_reset.
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
 * Large cells — the interior (excluding border) has room for text:
 *   interior height = CELL_H - 1 = 5 rows
 *   interior width  = CELL_W - 1 = 19 cols
 * An 80×24 terminal holds 4 cols × 3 rows = 12 cells (easy to navigate).
 */
#define CELL_W  20
#define CELL_H   6

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_PLAYER  3
#define PAIR_HUD     4
#define PAIR_LABEL   5   /* coordinate labels inside cells */

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
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
    init_pair(PAIR_LABEL,  COLORS >= 256 ? 246 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — THE FORMULA (unchanged from 01_uniform_rect):
 *
 *   screen_row = r * CELL_H    (= r * 6)
 *   screen_col = c * CELL_W    (= c * 20)
 *
 * LABEL PLACEMENT FORMULA:
 *   A text label "(r,c)" is placed at the first interior position:
 *     label_row = sr + 1           ← one row below top border
 *     label_col = sc + 2           ← two cols right of left border
 *
 *   Centre the '@' using:
 *     centre_row = sr + CELL_H / 2   (= sr + 3)
 *     centre_col = sc + CELL_W / 2   (= sc + 10)
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

/*
 * draw_labels — place "(row,col)" text in the top-left of every visible cell.
 *
 * Label position formula:
 *   label_row = cell_screen_row + 1   (first interior row)
 *   label_col = cell_screen_col + 2   (first interior col, slight indent)
 */
static void draw_labels(int rows, int cols, const Player *player)
{
    int gr = (rows - 1) / CELL_H;
    int gc = cols / CELL_W;

    for (int r = 0; r < gr; r++) {
        for (int c = 0; c < gc; c++) {
            if (r == player->r && c == player->c) continue;  /* player draws own cell */
            int sr, sc;
            cell_to_screen(r, c, &sr, &sc);
            char lbl[24];
            snprintf(lbl, sizeof lbl, "(%d,%d)", r, c);
            attron(COLOR_PAIR(PAIR_LABEL));
            mvprintw(sr + 1, sc + 2, "%s", lbl);
            attroff(COLOR_PAIR(PAIR_LABEL));
        }
    }
}

static void draw_player(const Player *p)
{
    int sr, sc;
    cell_to_screen(p->r, p->c, &sr, &sc);

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = 1; dr < CELL_H; dr++)
        for (int dc = 1; dc < CELL_W; dc++)
            mvaddch(sr + dr, sc + dc, (chtype)' ');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* Label in active cell */
    char lbl[24]; snprintf(lbl, sizeof lbl, "(%d,%d)", p->r, p->c);
    attron(COLOR_PAIR(PAIR_ACTIVE));
    mvprintw(sr + 1, sc + 2, "%s", lbl);
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
    mvaddch(sr + CELL_H / 2, sc + CELL_W / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Player *p, double fps)
{
    erase();
    draw_grid(rows, cols);
    draw_labels(rows, cols, p);
    draw_player(p);

    char buf[64];
    snprintf(buf, sizeof buf, " %.1f fps  at(%d,%d) ", fps, p->r, p->c);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " arrows:move  r:reset  q:quit  [04 coarse sparse  %dx%d cell] ",
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
