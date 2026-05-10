/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 13_dot.c — dot grid (intersection points only, no lines)
 *
 * DEMO: The minimal grid: only a '.' drawn at each cell corner
 *       (row*CELL_H, col*CELL_W). No connecting lines at all.
 *       The cursor '@' moves between corners; its 4 surrounding dots
 *       are highlighted to show which cell it occupies.
 *
 * Study alongside: 01_uniform_rect.c (same formula, adds lines), 12_ruled.c
 *
 * Section map:
 *   §1 config   — DOT_W, DOT_H (dot spacing), EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs (dot, corner, cursor, HUD, HINT)
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_grid_char / ctx_draw_bg
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/13_dot.c \
 *       -o 13_dot -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : The rectangular lattice rendered at minimum fidelity.
 *                  Only the intersection points (corners) are drawn.
 *                  All grid structure is implicit — the viewer infers the
 *                  lines from the dot positions alone.
 *
 * Dot formula    : A dot is drawn at screen position (sr, sc) when:
 *                    sr % DOT_H == 0  AND  sc % DOT_W == 0
 *                  This is the AND of both conditions from 01_uniform_rect.
 *                  01 draws a character when either condition holds;
 *                  13 draws only when BOTH hold (the corners only).
 *
 * Cell corners   : Cell (r,c) has 4 corners at:
 *                    top-left     (r*DOT_H,       c*DOT_W)
 *                    top-right    (r*DOT_H,   (c+1)*DOT_W)
 *                    bottom-left  ((r+1)*DOT_H,   c*DOT_W)
 *                    bottom-right ((r+1)*DOT_H, (c+1)*DOT_W)
 *                  These 4 dots define a cell without any connecting lines.
 *
 * Cursor cell    : When the cursor is at (cur->r, cur->c), its 4 corner dots
 *                  are highlighted. The cursor '@' sits at the cell centre:
 *                    centre_row = cur->r*DOT_H + DOT_H/2
 *                    centre_col = cur->c*DOT_W + DOT_W/2
 *
 * References     :
 *   Dot paper — en.wikipedia.org/wiki/Dot_paper
 *   Lattice points — en.wikipedia.org/wiki/Lattice_point
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The uniform rectangular grid (01) draws when EITHER the row OR the column
 * is on a grid line.  The dot grid draws when BOTH are on a grid line
 * simultaneously.  This logical AND instead of OR selects only the corner
 * positions — every connecting line segment is silently omitted.
 * Yet the human eye reconstructs the grid from the dots alone.
 *
 * HOW TO THINK ABOUT IT — AND vs OR
 * ────────────────────────────────────
 * Grid type vs. condition:
 *
 *   01_uniform_rect:  on_h OR  on_v  -> '-', '|', '+'     (lines + corners)
 *   13_dot:           on_h AND on_v  -> '.' only           (corners only)
 *
 * The "grid lines" in 01 are all positions where at least one coordinate
 * is a multiple of the step.  The "dots" are where BOTH coordinates are
 * multiples — the lattice points (the intersections of those lines).
 *
 * This is the distinction between:
 *   - A LATTICE: the set of discrete points at (k*DH, j*DW) for integers k,j.
 *   - A GRID:    the lattice PLUS all line segments connecting lattice points.
 *
 * The dot grid renders only the lattice.  The grid structure is implicit —
 * inferred by the viewer.  This is both minimal and elegant.
 *
 * DRAWING METHOD
 * ──────────────
 *  Per screen position (sr, sc):
 *
 *  1. on_dot = (sr % DOT_H == 0) AND (sc % DOT_W == 0)
 *  2. If on_dot AND is a cursor corner -> draw '+' with highlight color.
 *  3. If on_dot AND NOT cursor corner  -> draw '.' with dim color.
 *  4. Otherwise                        -> skip.
 *
 *  The cursor cell highlight uses the 4 CORNER DOTS — not fill, not lines.
 *  This keeps the minimal aesthetic while still showing the active cell.
 *
 *  Corner test for cell (cur->r, cur->c):
 *    is_corner_row = (sr == cur->r*DOT_H)  OR (sr == (cur->r+1)*DOT_H)
 *    is_corner_col = (sc == cur->c*DOT_W)  OR (sc == (cur->c+1)*DOT_W)
 *    is_corner = is_corner_row AND is_corner_col
 *
 * KEY FORMULAS
 * ────────────
 *  Dot condition (the only formula):
 *    is_dot(sr, sc) = (sr % DOT_H == 0) && (sc % DOT_W == 0)
 *
 *  Comparison with 01_uniform_rect:
 *    01:  on_h = (sr%CH==0),  on_v = (sc%CW==0)
 *         draw when on_h OR on_v
 *    13:  draw when on_h AND on_v   (AND is the only change!)
 *
 *  Number of dots:
 *    dot_rows = floor((LINES-1) / DOT_H) + 1
 *    dot_cols = floor(COLS / DOT_W) + 1
 *    total    = dot_rows * dot_cols
 *
 *  Cell top-left from cell index:
 *    sr = r * DOT_H,   sc = c * DOT_W
 *  Cell centre:
 *    sr = r * DOT_H + DOT_H/2,   sc = c * DOT_W + DOT_W/2
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DOT_H=1 and DOT_W=1: every position is a dot.  The screen fills
 *    completely with '.' characters.  No visible grid structure.
 *    For a visible grid: DOT_H >= 2, DOT_W >= 3.
 *
 *  • The cell is defined by its 4 corner dots but has no drawn border.
 *    "Which cell" contains a screen position (sr, sc)?
 *      cell_row = sr / DOT_H    (integer division)
 *      cell_col = sc / DOT_W
 *    Screen positions BETWEEN dots belong to a cell but have no character.
 *
 *  • Cursor '@' placement: the cell centre is NOT at a dot position (unless
 *    DOT_H is even and DOT_W is even and the centre falls on a multiple).
 *    In general, the '@' sits in empty space — this is correct, since the
 *    cell interior has no drawn characters.
 *
 *  • Dot character choice: '.' uses only 1 cell.  On many terminals, a
 *    middle-dot U+00B7 is more visually pleasing but requires correct
 *    locale settings.  Use '.' for maximum portability.
 *
 * HOW TO VERIFY
 * ─────────────
 *  The number of dots in the top row (sr=0) = floor(COLS/DOT_W) + 1.
 *  With DOT_W=6, COLS=80: floor(80/6)+1 = 13+1 = 14 dots in row 0.
 *  No characters should appear between the dots on row 0.
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
 * DOT_W, DOT_H — spacing between dots (= cell size).
 * The dot formula is: (sr % DOT_H == 0) && (sc % DOT_W == 0).
 */
#define DOT_W   6    /* columns between dots */
#define DOT_H   3    /* rows between dots    */

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_DOT     1   /* regular grid dots          */
#define PAIR_CORNER  2   /* highlighted cell corners   */
#define PAIR_CURSOR  3
#define PAIR_HUD     4
#define PAIR_HINT    5

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
    init_pair(PAIR_DOT,    COLORS >= 256 ? 252 : COLOR_WHITE,  -1);
    init_pair(PAIR_CORNER, COLORS >= 256 ?  46 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
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

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = DOT_W;  g->ch = DOT_H;
    g->max_r = (rows - 1) / DOT_H - 1;
    g->max_c = cols / DOT_W - 1;
}

static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/*
 * ctx_grid_char — DOT GRID FORMULA:
 *
 *   on_dot = (sr % DOT_H == 0) && (sc % DOT_W == 0)
 *
 * Compare to 01_uniform_rect:
 *   01: draws when (sr%CELL_H==0) OR (sc%CELL_W==0)   — lines
 *   13: draws when (sr%DOT_H==0) AND (sc%DOT_W==0)    — corners only
 */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    bool on_dot = (sr % g->ch == 0) && (sc % g->cw == 0);
    return on_dot ? '.' : ' ';
}

/*
 * is_cursor_corner — test if (sr,sc) is one of the 4 corners of cell (cr,cc).
 *
 * The 4 corner rows are:  cr*DOT_H  and  (cr+1)*DOT_H
 * The 4 corner cols are:  cc*DOT_W  and  (cc+1)*DOT_W
 */
static bool is_cursor_corner(const GridCtx *g, int sr, int sc, int cr, int cc)
{
    bool r_ok = (sr == cr * g->ch || sr == (cr + 1) * g->ch);
    bool c_ok = (sc == cc * g->cw || sc == (cc + 1) * g->cw);
    return r_ok && c_ok;
}

/* Cell centre (used for '@' placement) */
static void cell_centre(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    ctx_to_screen(g, r, c, sr, sc);
    *sr += g->ch / 2;
    *sc += g->cw / 2;
}

/*
 * ctx_draw_bg — draw all dots; corners of the cursor's cell get highlight.
 * Takes the cursor coords so it can paint the corner highlight in PAIR_CORNER.
 */
static void ctx_draw_bg(const GridCtx *g, int cr, int cc)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            if (ctx_grid_char(g, sr, sc) != '.') continue;
            if (is_cursor_corner(g, sr, sc, cr, cc)) {
                attron(COLOR_PAIR(PAIR_CORNER) | A_BOLD);
                mvaddch(sr, sc, (chtype)'+');
                attroff(COLOR_PAIR(PAIR_CORNER) | A_BOLD);
            } else {
                attron(COLOR_PAIR(PAIR_DOT));
                mvaddch(sr, sc, (chtype)'.');
                attroff(COLOR_PAIR(PAIR_DOT));
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
    int sr, sc; cell_centre(g, cur->r, cur->c, &sr, &sc);
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, sc, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[64];
    snprintf(buf, sizeof buf, " %.1f fps  cell(%d,%d) ", fps, cur->r, cur->c);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [13 dot grid  corners only: sr%%H==0 && sc%%W==0] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g, cur->r, cur->c);
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
