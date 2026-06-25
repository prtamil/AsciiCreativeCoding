/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 09_isometric.c — a tilted grid drawn the way 2D games like SimCity do it.
 *
 * We place square cells on screen so they look like flat, wide diamonds seen
 * from an angle. Arrow keys walk a cursor one cell at a time along the grid's
 * own axes. The math is the same as 08_diamond.c — only the cell shape differs.
 *
 * Study alongside: 08_diamond.c (same idea, square 45° cells).
 * Reference: en.wikipedia.org/wiki/Isometric_projection
 */

/*
 * THE BIG PICTURE
 *
 * Each grid cell sits at whole-number coordinates (r, c). To draw it we turn
 * those into a screen spot:
 *     column = origin_col + (c - r) * IW
 *     row    = origin_row + (c + r) * IH
 * IW and IH are how far one cell-step stretches sideways and downward. Make a
 * cell wide and shallow (IW big, IH small) and it looks tilted, like a game's
 * floor tile. Make them equal-ish and it looks like a square diamond.
 *
 * Here IW=8, IH=2, so cells are 16 chars wide and 4 rows tall — the classic
 * 2:1 game-isometric look (about a 26.6° tilt once you account for terminal
 * chars being taller than they are wide).
 *
 * Drawing the lines: for every screen spot we run the formula backwards to
 * ask "does a grid line pass through here?" Two checks — one for the lines
 * heading one way ('/'), one for the other way ('\'). Where both lines cross
 * we draw a corner '+'. With these proportions the two line families land in
 * different spots, so you actually see the '/' and '\' strokes, not just dots.
 *
 * Moving the cursor: arrow keys change one grid coordinate at a time (like a
 * normal grid), not one screen direction. A move only happens if it keeps the
 * cursor inside the allowed range — both r and c must stay in bounds or
 * neither moves.
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

/*
 * IW, IH set the cell shape: how far one cell-step moves sideways and down.
 * The cell ends up 2*IW=16 chars wide and 2*IH=4 rows tall — the wide, flat
 * look of game isometric tiles. Want a different tilt? Shrink IW to stand the
 * cells up, or shrink IH to flatten them. IW=4, IH=2 gives the square 45°
 * diamond from 08_diamond.
 */
#define IW     8   /* half-cell width  — cell is 2*IW=16 chars wide */
#define IH     2   /* half-cell height — cell is 2*IH= 4 rows  tall */

/* The number the line-finding math wraps around; just 2*IW*IH worked out. */
#define MODULUS  (2 * IW * IH)   /* = 32 */

/*
 * How many cells the cursor may roam from the centre, each direction. At the
 * far diagonal corner that's about 2*RANGE*IW = 64 chars off-centre, fine on a
 * wide terminal. If it ever lands off-screen the '@' draw simply skips it, so
 * a narrow terminal won't crash — it just won't show the cursor out there.
 */
#define RANGE  4   /* cursor stays within -RANGE .. +RANGE on each axis */

/* How much to smooth the FPS number so it doesn't jitter every frame. */
#define FPS_EWMA_ALPHA  0.05

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

/* ── §4 formula — turning a cell (r,c) into a screen spot, and back ── */

/*
 * GridCtx — everything we need to know to place this grid on the screen.
 * Filled once at startup (and again on resize), then read-only while drawing.
 */
typedef struct {
    int rows, cols;     /* size of the terminal, in chars */
    int cw, ch;         /* half-cell width and height (IW, IH copied here) */
    int ox, oy;         /* where cell (0,0) lands — the grid's centre point */

    /* How far the cursor may wander from the centre. range is the limit per
     * axis; max_r / max_c are the same value kept for clarity at call sites. */
    int range;
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = IW;     g->ch = IH;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->range = RANGE;
    g->max_r = RANGE;
    g->max_c = RANGE;
}

/* Where does cell (r,c) land on screen? This is the whole projection.
 *   col = ox + (c - r) * IW,   row = oy + (c + r) * IH */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sc = g->ox + (c - r) * g->cw;
    *sr = g->oy + (c + r) * g->ch;
}

/* C's % can return negative; this always lands in 0..b-1 so the line checks
 * still fire in the upper-left quadrant where u and v go negative. */
static int safe_mod(int a, int b) { return ((a % b) + b) % b; }

/*
 * Which grid mark belongs at screen spot (sr,sc)? We run the projection
 * backwards: one test says "a '/' line passes here", the other "a '\' line
 * passes here". Both true means a corner '+'; one true means that single
 * stroke; neither means blank. With IW=8, IH=2 the two families fall in
 * different spots, so you see the strokes, not just the corners.
 */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
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

/*
 * Does screen spot (sr,sc) fall inside the diamond of cell (pr,pc)?
 * Running the projection backwards gives a scaled-up (r,c) for the spot; if
 * both land in this cell's slice of that range, the spot is inside. Each cell
 * owns its far edge but not its near edge, so neighbours don't fight over the
 * shared grid line.
 */
static bool in_cursor_cell(const GridCtx *g, int sr, int sc, int pr, int pc)
{
    int u = sc - g->ox, v = sr - g->oy;
    int cn = u * g->ch + v * g->cw;          /* scaled-up c for this spot */
    int rn = v * g->cw - u * g->ch;          /* scaled-up r for this spot */
    return (cn > pc * MODULUS && cn <= (pc + 1) * MODULUS &&
            rn > pr * MODULUS && rn <= (pr + 1) * MODULUS);
}

/* Draw the grid: visit every screen spot, ask what mark goes there, and stamp
 * it. We skip blanks so we don't paint over the background. */
static void ctx_draw_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < g->rows - 1; sr++)
        for (int sc = 0; sc < g->cols; sc++) {
            char ch = ctx_grid_char(g, sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/* The highlighted cell the user is standing on, in grid coordinates. */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->r = 0; cur->c = 0;
}

/* Step the cursor one cell along a grid axis. The move only counts if it keeps
 * the cursor in range — both r and c must stay valid, or nothing moves. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= -g->range && nr <= g->range && nc >= -g->range && nc <= g->range) {
        cur->r = nr;
        cur->c = nc;
    }
}

/*
 * Show where the cursor is: dot-fill the highlighted cell, then drop an '@' in
 * the middle. We scan a small box around the cell and dot every empty spot that
 * tests as inside it, so the fill follows the diamond shape exactly.
 */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int csr, csc; ctx_to_screen(g, cur->r, cur->c, &csr, &csc);
    int span_r = g->ch * 2 + 1;   /* tall enough to cover the 4-row cell */
    int span_c = g->cw * 2;       /* wide enough to cover the 16-col cell */

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = -span_r; dr <= span_r; dr++) {
        for (int dc = -span_c; dc <= span_c; dc++) {
            int sr = csr + dr, sc = csc + dc;
            if (sr < 0 || sr >= g->rows - 1 || sc < 0 || sc >= g->cols) continue;
            if (!in_cursor_cell(g, sr, sc, cur->r, cur->c)) continue;
            if (ctx_grid_char(g, sr, sc) == ' ')
                mvaddch(sr, sc, (chtype)'.');
        }
    }
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* Place '@' at the cell's middle. We check the middle (not the top corner)
     * against the screen edges, so the marker still shows when the corner has
     * scrolled off the top. */
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
    ctx_draw_bg(g);
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
