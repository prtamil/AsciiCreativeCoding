/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 09_isometric.c — a tilted grid drawn the way 2D games like SimCity do it.
 *
 * Cells are wide flat diamonds seen at an angle. The projection spreads a
 * cell-step sideways and marches it down; the lattice reverses it with two
 * modulo line-tests. Arrow keys walk the cursor one cell along the grid axes.
 *
 * Sister files: 08_diamond.c (same math, square 45° cells), 01_uniform_rect.c.
 * Reference: en.wikipedia.org/wiki/Isometric_projection
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

/* IW, IH set the cell shape: how far one cell-step moves sideways and down.
 * Cell ends up 2*IW=16 wide, 2*IH=4 tall — the flat 2:1 game-isometric look.
 * IW=4, IH=2 gives the square 45° diamond of 08_diamond. */
#define IW     8   /* half-cell width  — cell is 2*IW=16 chars wide */
#define IH     2   /* half-cell height — cell is 2*IH= 4 rows  tall */

#define MODULUS  (2 * IW * IH)   /* line-test wrap-around = 2*IW*IH = 32 */

#define RANGE  4   /* cursor stays within -RANGE..+RANGE on each axis */

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
    init_pair(PAIR_GRID,   COLORS >= 256 ?  48 : COLOR_GREEN,  -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 iso mapping & lattice ── */

/* GridCtx — the grid for one frame: terminal size, half-cell size, where cell
 * (0,0) lands, and how far the cursor may roam. */
typedef struct {
    int rows, cols;      /* terminal size in characters */
    int cw, ch;          /* half-cell width and height (IW, IH) */
    int ox, oy;          /* screen spot of cell (0,0) — the grid centre */
    int max_r, max_c;    /* furthest cell the cursor can reach from centre */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = IW;     g->ch = IH;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->max_r = RANGE;
    g->max_c = RANGE;
}

/* recipe step 1 — cell (r,c) -> its centre-top spot on screen. The whole
 * projection: col spreads with (c-r), row marches down with (c+r). */
static void cell_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sc = g->ox + (c - r) * g->cw;
    *sr = g->oy + (c + r) * g->ch;
}

/* C's % can return negative; this lands in 0..b-1 so the line checks still fire
 * in the upper-left quadrant where u and v go negative. */
static int safe_mod(int a, int b) { return ((a % b) + b) % b; }

/* which glyph belongs at a screen spot: run the projection backwards into two
 * line families. '/' line and '\' line crossing -> '+', else one stroke. */
static char grid_glyph_at(const GridCtx *g, int sr, int sc)
{
    int u = sc - g->ox;
    int v = sr - g->oy;
    bool c_line = (safe_mod(u * g->ch + v * g->cw, MODULUS) == 0);
    bool r_line = (safe_mod(v * g->cw - u * g->ch, MODULUS) == 0);
    if (c_line && r_line) return '+';
    if (c_line)           return '/';
    if (r_line)           return '\\';
    return ' ';
}

/* does screen spot (sr,sc) fall inside cell (pr,pc)'s diamond? Reverse the
 * projection to a scaled (r,c); each cell owns its far edge but not its near
 * edge, so neighbours don't fight over the shared grid line. */
static bool in_cursor_cell(const GridCtx *g, int sr, int sc, int pr, int pc)
{
    int u = sc - g->ox, v = sr - g->oy;
    int cn = u * g->ch + v * g->cw;          /* scaled-up c for this spot */
    int rn = v * g->cw - u * g->ch;          /* scaled-up r for this spot */
    return (cn > pc * MODULUS && cn <= (pc + 1) * MODULUS &&
            rn > pr * MODULUS && rn <= (pr + 1) * MODULUS);
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

/* Cursor — which cell the user is in, in grid coordinates from centre (0,0). */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->r = 0; cur->c = 0;
}

/* recipe step 3 — step one cell along a grid axis, clamped: r and c must both
 * stay in range or nothing moves. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= -g->max_r && nr <= g->max_r && nc >= -g->max_c && nc <= g->max_c) {
        cur->r = nr;
        cur->c = nc;
    }
}

/* highlight the cursor's cell: dot every spot that tests inside the diamond,
 * then drop '@' in the middle. */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int csr, csc; cell_to_screen(g, cur->r, cur->c, &csr, &csc);
    int span_r = g->ch * 2 + 1;   /* tall enough to cover the 4-row cell */
    int span_c = g->cw * 2;       /* wide enough to cover the 16-col cell */

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = -span_r; dr <= span_r; dr++) {
        for (int dc = -span_c; dc <= span_c; dc++) {
            int sr = csr + dr, sc = csc + dc;
            if (sr < 0 || sr >= g->rows - 1 || sc < 0 || sc >= g->cols) continue;
            if (!in_cursor_cell(g, sr, sc, cur->r, cur->c)) continue;
            if (grid_glyph_at(g, sr, sc) == ' ')
                mvaddch(sr, sc, (chtype)'.');
        }
    }
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* '@' at the cell middle — tested against the edges (not the top corner) so
     * it still shows when the corner has scrolled off the top. */
    int centre_sc = g->ox + (cur->c - cur->r) * g->cw;
    int centre_sr = g->oy + (cur->c + cur->r + 1) * g->ch;
    if (centre_sr >= 0 && centre_sr < g->rows - 1 &&
        centre_sc >= 0 && centre_sc < g->cols) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(centre_sr, centre_sc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf, " %.1f fps  cell(%d,%d)  IW=%d IH=%d ",
             fps, cur->r, cur->c, g->cw, g->ch);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " right/left:c+-1  up/down:r-+1 (grid-axis)  r:reset  q/ESC:quit  [09 iso] ");
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

/* Set from signals only, so they're flagged volatile sig_atomic_t. */
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
            case 'q': case 27: g_running = 0;              break;
            case 'r':          cursor_reset(&cur, &g);     break;
            case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
            case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
            case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
            case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
        }
        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&g, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
