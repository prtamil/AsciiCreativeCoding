/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 08_diamond.c — a square grid tipped 45 degrees so every cell looks like a
 * diamond. The middle of the screen is cell (0,0). Arrow keys move the '@'
 * the way you'd expect on screen: right is right, up is up — even though the
 * grid underneath is rotated.
 *
 * Sister files: 01_uniform_rect.c (the same grid before rotating), 09_isometric.c
 */

/* ── how the rotation works ──
 *
 * Start with a normal grid where a cell's row r and column c map straight to
 * the screen. Tip the whole thing 45 degrees and the cells turn into diamonds.
 * The neat part: after that tilt, a cell's screen position only depends on the
 * SUM and the DIFFERENCE of its r and c.
 *
 *   how far across the screen  comes from  (c - r)
 *   how far down the screen    comes from  (c + r)
 *
 * So moving right in the grid (+c) actually nudges the cell right AND down on
 * screen, and moving up the grid nudges it right AND up. The grid's axes now
 * point along the diagonals instead of straight across and down.
 *
 * Drawing the lines is the reverse trip. We can't just test "is this column a
 * multiple of the cell width" like the un-rotated grid does, because the lines
 * run diagonally now. Instead we walk every screen cell, work backwards to ask
 * which diamond it falls on, and if it lands exactly on a diamond edge we draw
 * a '/' or a '\'. The drawing functions below spell out that backwards math.
 *
 * References (for the geometry, which the code can't explain on its own):
 *   Isometric projection — en.wikipedia.org/wiki/Isometric_projection
 *   Diamond/isometric grids in games — redblobgames.com/grids/hexagons
 *   Rotation matrix — en.wikipedia.org/wiki/Rotation_matrix
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
 * Half the width and half the height of one diamond, measured in screen
 * characters. Terminal characters are about twice as tall as they are wide,
 * so DW=4, DH=2 makes the diamonds look square instead of squashed.
 */
#define DW     4
#define DH     2

/* The '@' is allowed to roam this many cells out from the centre, each way. */
#define RANGE  8

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
    init_pair(PAIR_GRID,   COLORS >= 256 ?  87 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — cell <-> screen mapping ── */

/*
 * GridCtx — everything a drawing function needs to know about where the grid
 * sits on screen, bundled so nothing has to reach for global constants.
 *
 *   rows, cols    size of the terminal right now, in characters
 *   cw, ch        half-width and half-height of a diamond (copies of DW, DH);
 *                 kept as fields so the math reads off the struct, not globals
 *   ox, oy        the screen spot that cell (0,0) lands on — we put it dead
 *                 centre so the grid spreads out evenly in every direction
 *   range         how far the '@' may wander from centre, in cells
 *   max_r, max_c  same limit split per axis (both equal range here)
 */
typedef struct {
    int rows, cols;
    int cw, ch;
    int ox, oy;
    int range;
    int max_r, max_c;
} GridCtx;

/* Work out where the grid sits given the current terminal size. */
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

/*
 * Turn a cell's (row, column) into a spot on the screen. The whole rotation
 * boils down to two lines: the difference (c - r) decides how far across,
 * the sum (c + r) decides how far down. Cell (0,0) lands at the centre.
 */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sc = g->ox + (c - r) * g->cw;
    *sr = g->oy + (c + r) * g->ch;
}

/*
 * C's % can hand back a negative remainder, which breaks the "lands exactly on
 * a line" tests below. This wraps it so the answer is always 0 or positive —
 * without it, the lower-left quarter of the grid grows gaps and stray lines.
 */
static int safe_mod(int a, int b) { return ((a % b) + b) % b; }

/*
 * Decide what character, if any, belongs at one screen spot. We measure how
 * far the spot is from the centre, then check whether it sits right on one of
 * the diamond edges. There are two families of edges running in the two
 * diagonal directions: one shows up as '/', the other as '\', and where they
 * cross we draw a '+'. Everywhere else is blank.
 *
 * (The exact "on an edge" test is the rotation math run backwards, simplified
 * for DW=4, DH=2.)
 */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    int u = sc - g->ox;
    int v = sr - g->oy;
    bool c_line = (safe_mod(u + 2 * v, 8) == 0);
    bool r_line = (safe_mod(2 * v - u, 8) == 0);
    if (c_line && r_line) return '+';
    if (c_line)           return '/';
    if (r_line)           return '\\';
    (void)g;
    return ' ';
}

/*
 * Does this screen spot fall inside one particular diamond, the one at cell
 * (pr, pc)? We run the rotation backwards to recover which row and column the
 * spot belongs to, then check it lands in that cell's range. Used to fill in
 * the currently-selected diamond. (Everything is kept as whole numbers scaled
 * by 16 so there's no floating point and no rounding surprises.)
 */
static bool in_cursor_cell(const GridCtx *g, int sr, int sc, int pr, int pc)
{
    int u = sc - g->ox, v = sr - g->oy;
    int cn = u * g->ch + v * g->cw;          /* the spot's column, x16 */
    int rn = v * g->cw - u * g->ch;          /* the spot's row,    x16 */
    int denom = 2 * g->cw * g->ch;           /* one whole cell = 16    */
    return (cn > pc * denom && cn <= (pc + 1) * denom &&
            rn > pr * denom && rn <= (pr + 1) * denom);
}

/* Paint the diamond grid by asking every screen spot what it should show. */
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

/*
 * Cursor — which diamond the player is sitting on, as a grid row and column.
 * That's all the state we need; how far it's allowed to move lives in the
 * GridCtx's range, and where it lands on screen is recomputed each frame.
 */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->r = 0; cur->c = 0;
}

/*
 * Move the cursor by a (row, column) step, but ignore any step that would
 * push it past the allowed range. The arrow keys hand us steps that look
 * diagonal in grid terms but come out straight on screen — see where the
 * keys are wired up in the main loop for which step goes with which arrow.
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= -g->range && nr <= g->range) cur->r = nr;
    if (nc >= -g->range && nc <= g->range) cur->c = nc;
}

/* Fill in the selected diamond, then drop the '@' in its middle. */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int csr, csc; ctx_to_screen(g, cur->r, cur->c, &csr, &csc);
    int span_r = g->ch * 2, span_c = g->cw * 2;

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = -span_r; dr <= span_r; dr++) {
        for (int dc = -span_c; dc <= span_c; dc++) {
            int sr = csr + dr, sc = csc + dc;
            if (sr < 0 || sr >= g->rows - 1 || sc < 0 || sc >= g->cols) continue;
            if (!in_cursor_cell(g, sr, sc, cur->r, cur->c)) continue;
            char ch = ctx_grid_char(g, sr, sc);
            if (ch == ' ')
                mvaddch(sr, sc, (chtype)'.');
        }
    }
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* drop the '@' one half-cell down so it sits in the diamond's middle */
    if (csr >= 0 && csr < g->rows - 1 && csc >= 0 && csc < g->cols) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(csr + g->ch, csc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  screen(%d,%d) ",
        fps, cur->r, cur->c,
        g->ox + (cur->c - cur->r) * g->cw,
        g->oy + (cur->c + cur->r) * g->ch);
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
            /* odd-looking grid steps that come out as plain screen moves */
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
