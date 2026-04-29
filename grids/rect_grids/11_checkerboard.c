/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 11_checkerboard.c — checkerboard pattern (alternating cell fill)
 *
 * DEMO: Same rectangular grid as 01_uniform_rect but cells alternate
 *       between filled ('#') and empty (' '). The fill rule is
 *       (r + c) % 2 — the parity of the cell address. The player '@'
 *       can only stand on light cells (parity 0); arrow keys skip over
 *       dark cells, moving two steps at once in the correct direction.
 *
 * Study alongside: 01_uniform_rect.c (grid formula), 13_dot.c
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, fill characters
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: dark cell, light cell, player, border, HUD
 *   §4 formula  — cell fill rule: (r+c)%2; grid line same as 01
 *   §5 player   — moves in steps of 2 to stay on same parity
 *   §6 scene    — draw_grid(), draw_player()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/11_checkerboard.c \
 *       -o 11_checkerboard -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Standard rectangular grid with a parity-based cell fill.
 *                  The grid lines are identical to 01_uniform_rect.
 *                  The fill rule adds visual structure without changing geometry.
 *
 * Fill formula   :
 *   parity = (r + c) % 2
 *   parity == 0 → light cell   (player can stand here)
 *   parity == 1 → dark cell    (blocked / filled)
 *
 *   Why (r+c)%2? Moving right (+c=1) flips parity. Moving down (+r=1) also
 *   flips parity. So every neighbour has the opposite parity — checkerboard.
 *
 * Movement rule  : The player only occupies light cells (parity 0).
 *   Since every 1-step move flips parity, the player moves 2 steps at a time:
 *   new_r = r + 2*dr,  new_c = c + 2*dc
 *   This keeps (new_r + new_c) % 2 == (r + c) % 2 == 0.
 *
 * Alternative    : Let player move 1 step (to dark cells too) — remove the
 *   *2 multiplier in player_move and change the step in player_reset.
 *
 * References     :
 *   Checkerboard pattern — en.wikipedia.org/wiki/Checkerboard
 *   Chess board colouring — en.wikipedia.org/wiki/Chessboard
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The grid lines are IDENTICAL to 01_uniform_rect.  The only new thing is
 * a FILL RULE that colours alternating cells: parity = (r + c) % 2.  Even-
 * parity cells are light; odd-parity cells are dark.  The structure of the
 * grid is unchanged — only the interior of each cell gets shaded.
 *
 * HOW TO THINK ABOUT IT — PARITY AND NEIGHBOURS
 * ───────────────────────────────────────────────
 * Parity is additive: every step that changes exactly one of (r, c) by ±1
 * flips the parity.  This is why a checkerboard works:
 *
 *   parity(r,   c  ) = (r + c)     % 2
 *   parity(r+1, c  ) = (r + c + 1) % 2 = 1 - parity(r,c)   <- flipped!
 *   parity(r,   c+1) = (r + c + 1) % 2 = 1 - parity(r,c)   <- flipped!
 *   parity(r+1, c+1) = (r + c + 2) % 2 = parity(r,c)        <- same!
 *
 * Therefore: every orthogonal neighbour has opposite parity (checkerboard).
 *            every diagonal  neighbour has the same parity.
 *
 * This is a fundamental property used in:
 *   - Chess/checkers boards
 *   - Graph bipartite colouring
 *   - Cellular automata parity rules
 *   - Maze generation (walls on odd cells)
 *
 * DRAWING METHOD
 * ──────────────
 *  Phase 1 — cell fill:
 *    For each cell (r, c):
 *      if (r + c) % 2 == 1:  fill interior with DARK_FILL character ('#')
 *      else:                 leave interior empty
 *    Fill order: row-major scan over cells; inside each cell, fill all
 *    interior positions (sr+1..sr+CH-1) × (sc+1..sc+CW-1).
 *
 *  Phase 2 — grid lines (on top of fill):
 *    Exactly as 01_uniform_rect: raster scan, sr%CH==0 or sc%CW==0.
 *    Drawing grid lines AFTER fill ensures borders are always visible.
 *
 *  Alternatively: draw borders first, then fill — same visual result if
 *  the fill skips the border positions.
 *
 * KEY FORMULAS
 * ────────────
 *  Parity:        parity(r, c) = (r + c) % 2       (0=light, 1=dark)
 *  Fill condition: if parity == 1 -> fill interior
 *
 *  Player parity stays constant during movement (by design):
 *    parity(r + 2*dr, c + 2*dc) = (r+2dr + c+2dc) % 2
 *                                = (r+c + 2*(dr+dc)) % 2
 *                                = (r+c) % 2          <- same!
 *    Moving 2 steps preserves parity. Moving 1 step flips it.
 *
 *  Player always starts on a light cell (parity=0):
 *    r=0, c=0 -> (0+0)%2 = 0 (light) ✓
 *    r=2, c=0 -> (2+0)%2 = 0 ✓
 *    r=1, c=1 -> (1+1)%2 = 0 ✓  (diagonal move keeps parity)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Draw order: if you draw fill after grid lines, the fill overwrites
 *    the border characters.  Fix: draw fill first, then borders on top.
 *    Or: use a fill that skips the exact border rows and cols
 *    (dr from 1 to CELL_H-1, dc from 1 to CELL_W-1, not 0 to CELL_H).
 *
 *  • Player start cell: player_reset() sets r and c to even values so
 *    parity is 0 (light cell).  The mask `& ~1` rounds down to even:
 *      r = (some_value) & ~1   -> always even
 *    If r and c are both even, (r+c)%2=0 always (light cell). ✓
 *
 *  • The 2-step movement means the player jumps over dark cells entirely.
 *    If you want the player to enter dark cells too, remove the *2 factor
 *    in player_move and the & ~1 mask in player_reset.
 *
 *  • On resize, if the new grid_rows/cols is odd, max_r/max_c may be
 *    odd.  The & ~1 mask in player_reset keeps the player on even coords.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Cell (0,0): even parity -> light (empty interior).
 *  Cell (0,1): odd  parity -> dark (filled with '#').
 *  Cell (1,0): odd  parity -> dark.
 *  Cell (1,1): even parity -> light.
 *  The pattern should look exactly like a physical checkerboard.
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
#define CELL_W        8
#define CELL_H        4
#define DARK_FILL   '#'    /* character used to fill dark cells */

#define PAIR_DARK    1
#define PAIR_LIGHT   2
#define PAIR_PLAYER  3
#define PAIR_BORDER  4
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
    init_pair(PAIR_DARK,   COLORS >= 256 ? 242 : COLOR_WHITE,  -1);
    init_pair(PAIR_LIGHT,  COLORS >= 256 ? 255 : COLOR_WHITE,  -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_BORDER, COLORS >= 256 ?  24 : COLOR_CYAN,   -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — unchanged from 01_uniform_rect.
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

/*
 * cell_parity — CHECKERBOARD FILL FORMULA:
 *
 *   parity(r, c) = (r + c) % 2
 *
 *   0 → light cell   1 → dark cell
 *
 * Proof it produces a checkerboard:
 *   Right neighbour (r, c+1): parity = (r + c + 1) % 2 = 1 - parity(r,c) ✓
 *   Down  neighbour (r+1, c): parity = (r + c + 1) % 2 = 1 - parity(r,c) ✓
 *   Diagonal (r+1, c+1):      parity = (r + c + 2) % 2 = parity(r,c)     ✓ same!
 * Every orthogonal neighbour has opposite parity — true checkerboard.
 */
static int cell_parity(int r, int c) { return (r + c) % 2; }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  player                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int r, c, max_r, max_c; } Player;

static void player_reset(Player *p, int rows, int cols)
{
    int gr = (rows - 1) / CELL_H - 1;
    int gc = cols / CELL_W - 1;
    p->max_r = gr & ~1;    /* round down to even so start is on parity-0 cell */
    p->max_c = gc & ~1;
    p->r = (p->max_r / 2) & ~1;
    p->c = (p->max_c / 2) & ~1;
}

/*
 * player_move — PARITY-PRESERVING MOVEMENT:
 *
 *   new_r = r + 2*dr,  new_c = c + 2*dc
 *
 *   Moving 2 steps keeps parity: (r+2dr + c+2dc) % 2 == (r+c) % 2 ✓
 *   The player always stays on light (parity-0) cells.
 */
static void player_move(Player *p, int dr, int dc)
{
    int nr = p->r + 2 * dr, nc = p->c + 2 * dc;
    if (nr >= 0 && nr <= p->max_r) p->r = nr;
    if (nc >= 0 && nc <= p->max_c) p->c = nc;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void draw_grid(int rows, int cols)
{
    int gr = (rows - 1) / CELL_H;
    int gc = cols / CELL_W;

    /* Fill cell interiors first */
    for (int r = 0; r < gr; r++) {
        for (int c = 0; c < gc; c++) {
            int sr, sc; cell_to_screen(r, c, &sr, &sc);
            bool dark = (cell_parity(r, c) == 1);
            if (dark) {
                attron(COLOR_PAIR(PAIR_DARK));
                for (int dr = 1; dr < CELL_H; dr++)
                    for (int dc = 1; dc < CELL_W; dc++)
                        mvaddch(sr + dr, sc + dc, (chtype)(unsigned char)DARK_FILL);
                attroff(COLOR_PAIR(PAIR_DARK));
            }
        }
    }

    /* Draw grid borders on top */
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int sr = 0; sr < rows - 1; sr++)
        for (int sc = 0; sc < cols; sc++) {
            char ch = grid_char(sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

static void draw_player(const Player *p)
{
    int sr, sc; cell_to_screen(p->r, p->c, &sr, &sc);
    attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
    mvaddch(sr + CELL_H / 2, sc + CELL_W / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Player *p, double fps)
{
    erase();
    draw_grid(rows, cols);
    draw_player(p);

    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  parity=%d ", fps, p->r, p->c, cell_parity(p->r,p->c));
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " arrows:move(2 steps)  r:reset  q:quit  [11 checkerboard  (r+c)%%2] ");
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
