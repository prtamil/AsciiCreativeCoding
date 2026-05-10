/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_uniform_rect.c — standard rectangular grid, the base formula
 *
 * DEMO: Uniformly spaced horizontal and vertical lines divide the terminal
 *       into equal rectangles. Move '@' between cells with arrow keys.
 *       This is the root formula — every other file in this series modifies
 *       exactly one part of it.
 *
 * Study alongside: ../README.md (the GridCtx + Cursor abstraction),
 *                  02_square.c (aspect correction), 05_hierarchical.c
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, color pairs, EWMA constant
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — grid / active / cursor / HUD / HINT pairs
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_grid_char / ctx_draw_bg
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
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
 *                  Column lines at x = k * cw for integer k.
 *                  Row    lines at y = k * ch for integer k.
 *                  Every cell (r,c) occupies the region:
 *                    rows [r*ch .. (r+1)*ch)
 *                    cols [c*cw .. (c+1)*cw)
 *
 * Data-structure : Two structs — GridCtx (cell size, terminal size, cursor
 *                  bounds) and Cursor (just (r, c)). No grid array; the
 *                  grid is computed per pixel via modular arithmetic.
 *                  See ../README.md "The two primitives" for why this same
 *                  shape is reused across rect / hex / tri / polar.
 *
 * Formula        : ctx_to_screen — the anchor (top-left) of cell (r,c):
 *                    screen_row = r * ch
 *                    screen_col = c * cw
 *                  This is a pure scale. Grid space = screen space / cell_size.
 *
 * Grid lines     : A screen position (sr, sc) sits on a line when:
 *                    sr % ch == 0  →  horizontal line  '-'
 *                    sc % cw == 0  →  vertical line    '|'
 *                    both          →  corner           '+'
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
 * screen position: "Is my row a multiple of ch, or my column a multiple
 * of cw?"  That single modular test — applied per-pixel — produces the
 * entire grid with no arrays, no loops over cells, no stored data.
 * Everything derives from arithmetic on screen coordinates.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture the terminal as graph paper.  The paper itself has no content —
 * only a repeating pattern of intersecting lines.  The lines appear wherever
 * a screen coordinate is divisible by the cell step.  Between any two
 * consecutive vertical lines there are exactly (cw - 1) blank columns;
 * between any two horizontal lines there are (ch - 1) blank rows.
 * The "cell" is just the rectangular space between four lines.
 *
 * You never store a grid.  You never allocate cells.  You ask the formula.
 *
 * DRAWING METHOD  (raster-scan, the approach used here)
 * ──────────────
 *  1. Pick cw and ch — the spacing between lines in each axis.
 *  2. Loop over every screen row sr from 0 to rows-2 (leave bottom for HUD).
 *  3. Loop over every screen column sc from 0 to cols-1.
 *  4. Evaluate:
 *       on_h = (sr % ch == 0)      <- this row is a horizontal line
 *       on_v = (sc % cw == 0)      <- this col is a vertical line
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
 *    screen_row = r * ch
 *    screen_col = c * cw
 *
 *  Inverse  (screen position -> which cell contains it):
 *    cell_row = sr / ch      (integer division truncates)
 *    cell_col = sc / cw
 *
 *  On a grid line:
 *    horizontal line  at sr when  sr % ch == 0
 *    vertical   line  at sc when  sc % cw == 0
 *
 *  Interior of cell (r, c):
 *    rows  r*ch + 1  ..  (r+1)*ch - 1
 *    cols  c*cw + 1  ..  (c+1)*cw - 1
 *
 *  Number of visible complete cells:
 *    grid_cols = cols  / cw
 *    grid_rows = (rows-1) / ch
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • ch=1 or cw=1: every position is a grid line — the "grid" becomes a
 *    solid mesh of '+' and '-' and '|'.  Minimum for a visible interior:
 *    ch >= 2, cw >= 2.
 *
 *  • Off-by-one on the last row: row rows-1 is reserved for the HUD hint.
 *    Stop the raster scan at sr < rows-1, not sr < rows.
 *
 *  • Terminal resize: ch and cw are constants, but rows and cols change.
 *    After resize, ctx_init() recomputes max_r/max_c and cursor_reset()
 *    re-clamps the cursor.
 *
 *  • Cell border ownership: the top border of cell (r,c) is at screen row
 *    r*ch.  That row is ALSO the bottom border of cell (r-1,c).
 *    One screen row serves two cells.  Don't double-draw.
 *
 *  • The bottom-right cell may be incomplete if cols or rows is not a
 *    perfect multiple of cw or ch.  The raster scan handles this
 *    automatically (it just stops at the screen edge); ctx_init() uses
 *    integer division without +1 to exclude the partial cell.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Vertical line count   = floor(cols / cw) + 1   (including col 0)
 *  Horizontal line count = floor((rows-1) / ch) + 1
 *  For an 80x24 terminal with cw=8, ch=4:
 *    vertical   lines at cols 0,8,16,24,32,40,48,56,64,72  -> 10 lines
 *    horizontal lines at rows 0,4,8,12,16,20               ->  6 lines
 *
 *  Quick sanity checks via ctx_grid_char():
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
 * With CELL_W=8, CELL_H=4 the cells appear approximately square.
 * See 02_square.c for the exact aspect-ratio derivation.
 */
#define CELL_W  8
#define CELL_H  4

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

/* Color pair IDs */
#define PAIR_GRID    1   /* dim grid lines               */
#define PAIR_ACTIVE  2   /* highlighted cell fill        */
#define PAIR_CURSOR  3   /* bright '@'                   */
#define PAIR_HUD     4   /* status bar (yellow)          */
#define PAIR_HINT    5   /* key-hint footer (cyan)       */

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
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the cell ↔ screen mapping                    */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active grid plus cursor bounds.
 *
 * cw/ch are carried as fields (not just #defines) so the rest of the file
 * does not depend on global constants — ctx_to_screen() works for any
 * GridCtx value, which is the abstraction shared with placement files
 * (see ../rect_grids_placement/01_direct.c and ../README.md).
 */
typedef struct {
    /* terminal extent */
    int rows, cols;

    /* cell size in screen characters */
    int cw, ch;

    /* cursor bounds — last whole cell that fits in (rows-1) × cols */
    int max_r, max_c;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size.
 *
 * max_r leaves the bottom row free for the HUD hint.
 * Integer division truncates: an incomplete bottom-right cell is excluded
 * from the cursor's reachable range.
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->max_r = (rows - 1) / CELL_H - 1;
    g->max_c = cols / CELL_W - 1;
}

/*
 * ctx_to_screen — top-left corner of cell (r,c) in screen coordinates.
 *
 * THE FORMULA (uniform rectangular grid):
 *   screen_row = r * ch
 *   screen_col = c * cw
 *
 * Reading it: to go one cell right (+c), move cw columns right.
 *             to go one cell down  (+r), move ch rows   down.
 * The origin cell (0,0) maps to screen corner (0,0).
 *
 * All 14 grids in this series share this §4 structure.
 * Only the formula inside changes.
 */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/*
 * ctx_grid_char — what character belongs at screen position (sr, sc).
 *
 * Grid line detection via modular arithmetic:
 *   on_h = (sr % ch == 0)
 *   on_v = (sc % cw == 0)
 *
 * Character selection:
 *   both  → '+'   corner
 *   h     → '-'   horizontal segment
 *   v     → '|'   vertical segment
 *   none  → ' '   interior of a cell
 */
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
 * ctx_draw_bg — paint the grid background.
 *
 * Per-pixel raster scan: evaluates ctx_grid_char() at every screen position.
 * O(rows × cols), invisible at terminal sizes.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            char ch = ctx_grid_char(g, sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — just (r, c) in cell space.
 *
 * Bounds live in GridCtx, not here — the cursor doesn't know how big the
 * grid is, just where in it the user is pointing. The two structs compose:
 * Cursor + GridCtx → screen position via ctx_to_screen().
 */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r = g->max_r / 2;
    cur->c = g->max_c / 2;
}

/*
 * cursor_move — apply (dr, dc) and clamp to GridCtx bounds.
 *
 * Arrow-key dispatch: UP=(-1,0), DOWN=(+1,0), LEFT=(0,-1), RIGHT=(0,+1).
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= 0 && nr <= g->max_r) cur->r = nr;
    if (nc >= 0 && nc <= g->max_c) cur->c = nc;
}

/*
 * cursor_draw — fill the active cell with '.' and place '@' at its centre.
 *
 * Cell interior = rows (sr+1 .. sr+ch-1) × cols (sc+1 .. sc+cw-1)
 * (the region strictly inside the four grid lines).
 *
 * Centre formula:
 *   centre_row = sr + ch / 2
 *   centre_col = sc + cw / 2
 */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->r, cur->c, &sr, &sc);

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = 1; dr < g->ch; dr++)
        for (int dc = 1; dc < g->cw; dc++)
            mvaddch(sr + dr, sc + dc, (chtype)'.');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr + g->ch / 2, sc + g->cw / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Bright bold yellow fps readout (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[64];
    snprintf(buf, sizeof buf, " %.1f fps  cell(%d,%d) ", fps, cur->r, cur->c);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  r:reset  q/ESC:quit  [01 uniform rect] ");
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
