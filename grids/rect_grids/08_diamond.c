/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 08_diamond.c — a square grid tipped 45 degrees so every cell is a diamond.
 * Cell (0,0) sits dead centre; arrows move the '@' in screen directions.
 *
 * Same skeleton as 01_uniform_rect.c (GridCtx, Cursor, cell_to_screen,
 * grid_glyph_at, draw_lattice). Only the mapping math differs: it's a 45deg
 * rotation, so a cell's screen spot depends on (c - r) across and (c + r) down.
 *
 * Sister files: 01_uniform_rect.c (the un-rotated grid), 09_isometric.c.
 * Geometry the code can't supply: en.wikipedia.org/wiki/Rotation_matrix,
 * redblobgames.com/grids/hexagons.
 */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

#define TARGET_FPS  30

#define DW     4   /* half a diamond's width  in chars; DW=4,DH=2 makes a cell */
#define DH     2   /* half a diamond's height in chars; look square (chars are 2:1) */

#define RANGE  8   /* how many cells the '@' may roam from centre, each way */

#define FPS_EWMA_ALPHA  0.05   /* small = steadier on-screen fps number */

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_CURSOR  3
#define PAIR_HUD     4
#define PAIR_HINT    5

/* ── §2 clock ── */

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

/* ── §3 color ── */

static void color_init(void)
{
    start_color(); use_default_colors();
    init_pair(PAIR_GRID,   COLORS >= 256 ?  87 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 diamond mapping & lattice ── */

/* GridCtx — the grid for one frame: terminal size, half-cell size, the screen
 * centre cell (0,0) lands on, and how far the cursor may roam. */
typedef struct {
    int rows, cols;      /* terminal size in characters */
    int cw, ch;          /* half-width, half-height of a diamond (DW, DH) */
    int ox, oy;          /* screen spot of cell (0,0): dead centre */
    int range;           /* cursor roam limit from centre, in cells */
    int max_r, max_c;    /* same limit per axis (both = range) */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = DW;     g->ch = DH;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->range = RANGE;
    g->max_r = RANGE;
    g->max_c = RANGE;
}

/* recipe step 1 — cell (r,c) -> screen spot. The 45deg rotation: difference
 * (c - r) sets how far across, sum (c + r) sets how far down. */
static void cell_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sc = g->ox + (c - r) * g->cw;
    *sr = g->oy + (c + r) * g->ch;
}

/* C's % can return negative; this keeps it in [0,b) so the "on a line" tests
 * below don't fail in the lower-left quarter (else: gaps and stray lines). */
static int safe_mod(int a, int b) { return ((a % b) + b) % b; }

/* which glyph belongs at a screen spot: the rotation run backwards. A spot is
 * on a '/' edge or a '\' edge when its distance from centre lands on a diamond
 * boundary; crossing -> '+'. (Edge tests simplified for DW=4, DH=2.) */
static char grid_glyph_at(const GridCtx *g, int sr, int sc)
{
    int u = sc - g->ox;
    int v = sr - g->oy;
    bool c_line = (safe_mod(u + 2 * v, 8) == 0);
    bool r_line = (safe_mod(2 * v - u, 8) == 0);
    if (c_line && r_line) return '+';
    if (c_line)           return '/';
    if (r_line)           return '\\';
    return ' ';
}

/* does a screen spot fall inside the diamond at cell (pr,pc)? Rotation run
 * backwards to recover the spot's row/col, kept as integers scaled by 16
 * (= 2*cw*ch) so there's no floating point. */
static bool in_cell(const GridCtx *g, int sr, int sc, int pr, int pc)
{
    int u = sc - g->ox, v = sr - g->oy;
    int col_x16 = u * g->ch + v * g->cw;
    int row_x16 = v * g->cw - u * g->ch;
    int denom   = 2 * g->cw * g->ch;          /* one whole cell = 16 */
    return (col_x16 > pc * denom && col_x16 <= (pc + 1) * denom &&
            row_x16 > pr * denom && row_x16 <= (pr + 1) * denom);
}

/* recipe step 2 — draw the grid by asking grid_glyph_at at every screen spot */
static void draw_lattice(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < g->rows - 1; sr++)
        for (int sc = 0; sc < g->cols; sc++) {
            char ch = grid_glyph_at(g, sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/* Cursor — which diamond the user is on, as (r,c) with (0,0) at centre. Pair
 * with a GridCtx and run through cell_to_screen to place it on screen. */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->r = 0; cur->c = 0;
}

/* recipe step 3 — move the cursor, clamped so it never leaves the range. The
 * arrows hand us diagonal grid steps that come out straight on screen (see main). */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= -g->range && nr <= g->range) cur->r = nr;
    if (nc >= -g->range && nc <= g->range) cur->c = nc;
}

/* fill the selected diamond's interior, then drop '@' in its middle */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int csr, csc; cell_to_screen(g, cur->r, cur->c, &csr, &csc);
    int span_r = g->ch * 2, span_c = g->cw * 2;

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = -span_r; dr <= span_r; dr++) {
        for (int dc = -span_c; dc <= span_c; dc++) {
            int sr = csr + dr, sc = csc + dc;
            if (sr < 0 || sr >= g->rows - 1 || sc < 0 || sc >= g->cols) continue;
            if (!in_cell(g, sr, sc, cur->r, cur->c)) continue;
            if (grid_glyph_at(g, sr, sc) == ' ')
                mvaddch(sr, sc, (chtype)'.');
        }
    }
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* '@' one half-cell down so it sits in the diamond's middle */
    if (csr >= 0 && csr < g->rows - 1 && csc >= 0 && csc < g->cols) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(csr + g->ch, csc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    int csr, csc; cell_to_screen(g, cur->r, cur->c, &csr, &csc);
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  screen(%d,%d) ",
        fps, cur->r, cur->c, csc, csr);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move (screen-dir)  r:reset  q/ESC:quit  [08 diamond  DW=%d DH=%d] ",
        g->cw, g->ch);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    draw_lattice(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, fps);
    wnoutrefresh(stdscr); doupdate();
}

/* ── §7 screen ── */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(); atexit(screen_cleanup);
}

/* ── §8 app ── */

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
        }
        int ch = getch();
        switch (ch) {
            case 'q': case 27: g_running = 0;                 break;
            case 'r':          cursor_reset(&cur, &g);        break;
            /* diagonal grid steps that come out as plain screen moves */
            case KEY_RIGHT: cursor_move(&cur, &g, -1, +1);    break;
            case KEY_LEFT:  cursor_move(&cur, &g, +1, -1);    break;
            case KEY_UP:    cursor_move(&cur, &g, -1, -1);    break;
            case KEY_DOWN:  cursor_move(&cur, &g, +1, +1);    break;
        }
        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&g, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
