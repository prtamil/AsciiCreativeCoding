/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 10_crosshatch.c — two diagonal grids overlaid (cross-hatch pattern)
 *
 * DEMO: Two independent sets of diagonal lines: '/' lines and '\' lines
 *       each with their own spacing. Together they create a cross-hatch
 *       (tartan/mesh) pattern. The cursor moves on an underlying
 *       rectangular grid whose cells are the diamonds formed by the
 *       intersection of the two diagonal families.
 *
 * Study alongside: 08_diamond.c (single diagonal system), 01_uniform_rect.c
 *
 * Section map:
 *   §1 config   — STEP_A, STEP_B (line spacings), CELL_W/CELL_H, EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6 pairs (slash, back, cross, cursor, HUD, HINT)
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_grid_char / ctx_draw_bg
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/10_crosshatch.c \
 *       -o 10_crosshatch -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two families of parallel diagonal lines.
 *                  Family A: '/' lines — rise from right to left.
 *                  Family B: '\' lines — fall from left to right.
 *                  Each family is defined by a single modular condition.
 *
 * Family A — '/' lines:
 *   A '/' line through a terminal has the property that (sc + sr) is
 *   constant along the line (moving right = moving down).
 *   Family A has lines wherever: (sc + sr) % STEP_A == 0
 *
 * Family B — '\' lines:
 *   A '\' line has (sc - sr) constant along it.
 *   Family B: (sc - sr + BIG) % STEP_B == 0   (BIG avoids negative %)
 *
 * Intersection:  Both conditions true → draw 'X' (or '+')
 *
 * Cursor grid:   The cross-hatch creates diamond-shaped cells of size
 *                STEP_A × STEP_B. The cursor moves in a rectangular grid
 *                aligned to the underlying coordinate system. Cell (r,c)
 *                maps to the diamond centred at screen (r*CELL_H, c*CELL_W).
 *
 * References     :
 *   Hatching — en.wikipedia.org/wiki/Hatching
 *   Cross-hatch shading — en.wikipedia.org/wiki/Crosshatching
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two completely independent line families coexist on screen.  Each family
 * is defined by a single modular condition involving a sum or difference of
 * the screen coordinates.  Together they form a diamond-mesh cross-hatch.
 * The families do not share a formula or data — they are composed by
 * evaluating both independently for each screen position.
 *
 * HOW TO THINK ABOUT IT — THE SUM AND DIFFERENCE INVARIANTS
 * ──────────────────────────────────────────────────────────
 * Think about what stays CONSTANT as you move along each line type:
 *
 *   '/' line: moving right (+sc=+1) moves up (-sr=-1).
 *             The value (sc + sr) does not change along a '/' line.
 *             Each '/' line is a LEVEL SET of (sc + sr).
 *             Family A: draw '/' at all positions where (sc+sr) % STEP_A == 0.
 *
 *   '\' line: moving right (+sc=+1) moves down (+sr=+1).
 *             The value (sc - sr) does not change along a '\' line.
 *             Each '\' line is a level set of (sc - sr).
 *             Family B: draw '\' where safe_mod(sc-sr, STEP_B) == 0.
 *
 *   A family of parallel '/' lines = all points where (sc+sr) = k*STEP_A.
 *   A family of parallel '\' lines = all points where (sc-sr) = k*STEP_B.
 *   Both true => draw 'X' at the intersection.
 *
 * DRAWING METHOD
 * ──────────────
 *  Per screen position (sr, sc):
 *
 *  1. on_slash = ((sc + sr) % STEP_A == 0)
 *  2. on_back  = (((sc - sr) % STEP_B + STEP_B) % STEP_B == 0)
 *     (The +STEP_B before the final % ensures non-negative result.)
 *
 *  3. Draw:
 *     on_slash && on_back  ->  'X'  or '+'
 *     on_slash only        ->  '/'
 *     on_back only         ->  '\'
 *     neither              ->  ' '
 *
 * KEY FORMULAS
 * ────────────
 *  '/' family invariant:  (sc + sr) = constant along each line.
 *    Line membership:  (sc + sr) % STEP_A == 0
 *
 *  '\' family invariant:  (sc - sr) = constant along each line.
 *    Line membership:  safe_mod(sc - sr, STEP_B) == 0
 *
 *  Number of '/' lines crossing the screen (width W, height H):
 *    approximately (W + H) / STEP_A  (diagonal span / step)
 *
 *  Diamond cell diagonals:
 *    The two families create diamond cells.  The diagonals of each cell
 *    are STEP_A and STEP_B screen positions long.  If STEP_A == STEP_B,
 *    cells are rhombuses with equal diagonals (rotated squares).
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Negative modulo for '\' lines: (sc - sr) is negative in the lower-
 *    left region of the screen (large sr, small sc).  C's % operator
 *    returns negative for negative dividends.  The fix:
 *      ((sc - sr) % STEP_B + STEP_B) % STEP_B
 *    Forgetting this creates a gap in the '\' family for sc < sr.
 *
 *  • STEP_A=1 or STEP_B=1: every position is a line of that family —
 *    the entire screen fills with diagonal stripes, no empty interior.
 *    Minimum for visible diamond cells: STEP >= 3.
 *
 *  • STEP_A and STEP_B with common factors: if gcd(STEP_A, STEP_B) > 1,
 *    the 'X' intersections cluster in regular patterns because positions
 *    that are multiples of both steps repeat more often.  Coprime values
 *    (gcd=1, e.g. STEP_A=6, STEP_B=5) give the most uniform mesh.
 *
 *  • '+' vs 'X' at intersections: 'X' is the natural crosshatch character
 *    but visually heavy.  Using '+' makes intersections look like a grid.
 *
 * HOW TO VERIFY
 * ─────────────
 *  '/' lines in the first screen column (sc=0):
 *    on_slash when (0 + sr) % STEP_A == 0, i.e., sr = 0, STEP_A, 2*STEP_A,...
 *  '\' lines in the first screen row (sr=0):
 *    on_back  when sc % STEP_B == 0, i.e., sc = 0, STEP_B, 2*STEP_B, ...
 *  These are independent — changing STEP_A never affects the '\' spacing.
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
 * STEP_A — spacing between '/' lines. Every STEP_A positions along (sc+sr).
 * STEP_B — spacing between '\' lines. Every STEP_B positions along (sc-sr).
 * Setting STEP_A != STEP_B creates a non-square diamond mesh.
 */
#define STEP_A   6    /* '/' family spacing  */
#define STEP_B   4    /* '\' family spacing  */

/* Cursor moves on a rect grid that approximates the diamond cell size */
#define CELL_W   (STEP_A)
#define CELL_H   (STEP_B)

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_SLASH   1   /* '/' lines  */
#define PAIR_BACK    2   /* '\' lines  */
#define PAIR_CROSS   3   /* 'X' at intersections */
#define PAIR_CURSOR  4
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
    init_pair(PAIR_SLASH,  COLORS >= 256 ?  32 : COLOR_CYAN,   -1);
    init_pair(PAIR_BACK,   COLORS >= 256 ? 129 : COLOR_MAGENTA,-1);
    init_pair(PAIR_CROSS,  COLORS >= 256 ?  46 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the cell ↔ screen mapping                    */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — terminal extent + cell size + cursor bounds.
 * The cell space is the rectangular grid the cursor moves on.
 */
typedef struct {
    int rows, cols;
    int cw, ch;          /* underlying rect cell size */
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->max_r = (rows - 1) / CELL_H - 1;
    g->max_c = cols / CELL_W - 1;
}

/*
 * ctx_to_screen — same as 01_uniform_rect: the cursor's underlying rect.
 */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/*
 * crosshatch_test — TWO INDEPENDENT LINE FAMILIES.
 *
 * Family A — '/' lines (sum-constant):
 *   on_slash = ((sc + sr) % STEP_A == 0)
 *
 * Family B — '\' lines (difference-constant):
 *   on_back = ((sc - sr + BIG) % STEP_B == 0)    (+BIG keeps modulo non-negative)
 *
 * Together: the two families create a mesh of diamond cells.
 * The cells are where NEITHER condition holds.
 */
static void crosshatch_test(int sr, int sc, bool *slash, bool *back)
{
    *slash = ((sc + sr) % STEP_A == 0);
    *back  = (((sc - sr) % STEP_B + STEP_B) % STEP_B == 0);
}

/*
 * ctx_draw_bg — draw both diagonal families with per-family colour.
 * '/' uses PAIR_SLASH, '\' uses PAIR_BACK, intersections use PAIR_CROSS.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            bool s, b; crosshatch_test(sr, sc, &s, &b);
            if (s && b) {
                attron(COLOR_PAIR(PAIR_CROSS) | A_BOLD);
                mvaddch(sr, sc, (chtype)'X');
                attroff(COLOR_PAIR(PAIR_CROSS) | A_BOLD);
            } else if (s) {
                attron(COLOR_PAIR(PAIR_SLASH));
                mvaddch(sr, sc, (chtype)'/');
                attroff(COLOR_PAIR(PAIR_SLASH));
            } else if (b) {
                attron(COLOR_PAIR(PAIR_BACK));
                mvaddch(sr, sc, (chtype)'\\');
                attroff(COLOR_PAIR(PAIR_BACK));
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
    int sr, sc; ctx_to_screen(g, cur->r, cur->c, &sr, &sc);
    int cr = sr + g->ch / 2, cc = sc + g->cw / 2;
    if (cr >= 0 && cc >= 0) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(cr, cc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  /step=%d \\step=%d ",
        fps, cur->r, cur->c, STEP_A, STEP_B);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [10 crosshatch  /=(sc+sr)%%A  \\=(sc-sr)%%B] ");
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
