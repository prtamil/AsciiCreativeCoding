/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 11_checkerboard.c — checkerboard pattern (alternating cell fill)
 *
 * DEMO: Same rectangular grid as 01_uniform_rect but cells alternate
 *       between filled ('#') and empty (' '). The fill rule is
 *       (r + c) % 2 — the parity of the cell address. The cursor '@'
 *       can only stand on light cells (parity 0); arrow keys skip over
 *       dark cells, moving two steps at once in the correct direction.
 *
 * Study alongside: 01_uniform_rect.c (grid formula), 13_dot.c
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, fill characters, EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6 pairs (dark, light, cursor, border, HUD, HINT)
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_grid_char / ctx_draw_bg
 *   §5 cursor   — Cursor + cursor_reset / cursor_move (parity-preserving) / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
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
 *   parity == 0 → light cell   (cursor can stand here)
 *   parity == 1 → dark cell    (blocked / filled)
 *
 *   Why (r+c)%2? Moving right (+c=1) flips parity. Moving down (+r=1) also
 *   flips parity. So every neighbour has the opposite parity — checkerboard.
 *
 * Movement rule  : The cursor only occupies light cells (parity 0).
 *   Since every 1-step move flips parity, the cursor moves 2 steps at a time:
 *   new_r = r + 2*dr,  new_c = c + 2*dc
 *   This keeps (new_r + new_c) % 2 == (r + c) % 2 == 0.
 *
 * Alternative    : Let cursor move 1 step (to dark cells too) — remove the
 *   *2 multiplier in cursor_move and change the step in cursor_reset.
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
 *
 *  Phase 2 — grid lines (on top of fill):
 *    Exactly as 01_uniform_rect: raster scan, sr%CH==0 or sc%CW==0.
 *    Drawing grid lines AFTER fill ensures borders are always visible.
 *
 * KEY FORMULAS
 * ────────────
 *  Parity:        parity(r, c) = (r + c) % 2       (0=light, 1=dark)
 *  Fill condition: if parity == 1 -> fill interior
 *
 *  Cursor parity stays constant during movement (by design):
 *    parity(r + 2*dr, c + 2*dc) = (r+2dr + c+2dc) % 2
 *                                = (r+c + 2*(dr+dc)) % 2
 *                                = (r+c) % 2          <- same!
 *    Moving 2 steps preserves parity. Moving 1 step flips it.
 *
 *  Cursor always starts on a light cell (parity=0):
 *    r=0, c=0 -> (0+0)%2 = 0 (light) ✓
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Draw order: if you draw fill after grid lines, the fill overwrites
 *    the border characters.  Fix: draw fill first, then borders on top.
 *
 *  • Cursor start cell: cursor_reset() sets r and c to even values so
 *    parity is 0 (light cell).  The mask `& ~1` rounds down to even.
 *
 *  • The 2-step movement means the cursor jumps over dark cells entirely.
 *    If you want the cursor to enter dark cells too, remove the *2 factor
 *    in cursor_move and the & ~1 mask in cursor_reset.
 *
 *  • On resize, if the new grid_rows/cols is odd, max_r/max_c may be
 *    odd.  The & ~1 mask in cursor_reset keeps the cursor on even coords.
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

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_DARK    1
#define PAIR_LIGHT   2
#define PAIR_CURSOR  3
#define PAIR_BORDER  4
#define PAIR_HUD     5
#define PAIR_HINT    6

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
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_BORDER, COLORS >= 256 ?  24 : COLOR_CYAN,   -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the cell ↔ screen mapping                    */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int rows, cols;
    int cw, ch;
    int max_r, max_c;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size.
 * max_r/max_c are masked to even so the cursor always lands on parity-0 cells.
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    int gr = (rows - 1) / CELL_H - 1;
    int gc = cols / CELL_W - 1;
    g->max_r = gr & ~1;    /* round down to even so end cell is parity-0 */
    g->max_c = gc & ~1;
}

/*
 * ctx_to_screen — unchanged from 01_uniform_rect.
 */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    bool h = (sr % g->ch == 0);
    bool v = (sc % g->cw == 0);
    if (h && v) return '+';
    if (h)      return '-';
    if (v)      return '|';
    return ' ';
}

/*
 * cell_parity — CHECKERBOARD FILL FORMULA:
 *
 *   parity(r, c) = (r + c) % 2     0 → light cell   1 → dark cell
 *
 * Every orthogonal neighbour has opposite parity — true checkerboard.
 */
static int cell_parity(int r, int c) { return (r + c) % 2; }

/*
 * ctx_draw_bg — paint dark-cell fill, then grid borders on top.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    int gr = (g->rows - 1) / g->ch;
    int gc = g->cols / g->cw;

    /* Fill cell interiors first */
    for (int r = 0; r < gr; r++) {
        for (int c = 0; c < gc; c++) {
            int sr, sc; ctx_to_screen(g, r, c, &sr, &sc);
            bool dark = (cell_parity(r, c) == 1);
            if (dark) {
                attron(COLOR_PAIR(PAIR_DARK));
                for (int dr = 1; dr < g->ch; dr++)
                    for (int dc = 1; dc < g->cw; dc++)
                        mvaddch(sr + dr, sc + dc, (chtype)(unsigned char)DARK_FILL);
                attroff(COLOR_PAIR(PAIR_DARK));
            }
        }
    }

    /* Draw grid borders on top */
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int sr = 0; sr < g->rows - 1; sr++)
        for (int sc = 0; sc < g->cols; sc++) {
            char ch = ctx_grid_char(g, sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r = (g->max_r / 2) & ~1;
    cur->c = (g->max_c / 2) & ~1;
}

/*
 * cursor_move — PARITY-PRESERVING MOVEMENT:
 *
 *   new_r = r + 2*dr,  new_c = c + 2*dc
 *
 *   Moving 2 steps keeps parity: (r+2dr + c+2dc) % 2 == (r+c) % 2 ✓
 *   The cursor always stays on light (parity-0) cells.
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + 2 * dr, nc = cur->c + 2 * dc;
    if (nr >= 0 && nr <= g->max_r) cur->r = nr;
    if (nc >= 0 && nc <= g->max_c) cur->c = nc;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc; ctx_to_screen(g, cur->r, cur->c, &sr, &sc);
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr + g->ch / 2, sc + g->cw / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  parity=%d ",
        fps, cur->r, cur->c, cell_parity(cur->r, cur->c));
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move(2 steps)  r:reset  q/ESC:quit  [11 checkerboard  (r+c)%%2] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g);
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
