/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 10_crosshatch.c — two diagonal line families woven into a cross-hatch mesh.
 *
 * Nothing is stored: at each screen spot we run two modulo tests — '/' lines
 * keep (sc+sr) constant, '\' lines keep (sc-sr) constant — and draw '/','\','X'.
 *
 * Sister files: 01_uniform_rect.c (the skeleton), 08_diamond.c (one family).
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

/* How far apart the lines sit. Bigger number = wider gaps. Picking different
 * values for the two sets makes the diamonds taller or wider than square. */
#define STEP_A   6    /* gap between '/' lines */
#define STEP_B   4    /* gap between '\' lines */

/* The cursor's grid roughly matches one diamond per cell. */
#define CELL_W   (STEP_A)
#define CELL_H   (STEP_B)

/* Smooths the on-screen FPS number so it doesn't jitter every frame. */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_SLASH   1   /* '/' lines */
#define PAIR_BACK    2   /* '\' lines */
#define PAIR_CROSS   3   /* 'X' where they cross */
#define PAIR_CURSOR  4
#define PAIR_HUD     5
#define PAIR_HINT    6

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
    init_pair(PAIR_SLASH,  COLORS >= 256 ?  32 : COLOR_CYAN,   -1);
    init_pair(PAIR_BACK,   COLORS >= 256 ? 129 : COLOR_MAGENTA,-1);
    init_pair(PAIR_CROSS,  COLORS >= 256 ?  46 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 rect mapping & lattice ── */

/* GridCtx — the grid for one frame: terminal size, cursor-cell size, and how
 * far the cursor may roam (last whole cell that fits, bottom row left free). */
typedef struct {
    int rows, cols;      /* terminal size in characters */
    int cw, ch;          /* cursor-cell width and height (CELL_W / CELL_H) */
    int max_r, max_c;    /* furthest cell the cursor can reach */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->max_r = (rows - 1) / CELL_H - 1;
    g->max_c = cols / CELL_W - 1;
}

/* recipe step 1 — cell (r,c) -> its top-left corner on screen */
static void cell_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/* The distinct idea: two diagonal line families, each a modulo test on a
 * screen spot. A '/' line keeps the sum (sc+sr) constant, a '\' line keeps the
 * difference (sc-sr) constant — so spacing the lines = "every STEP steps of"
 * that quantity. The "+ STEP_B" dance fixes C's sign-keeping % on (sc-sr) < 0,
 * which would otherwise drop the '\' lines in the lower-left. */
static bool on_slash_line(int sr, int sc) { return (sc + sr) % STEP_A == 0; }
static bool on_back_line(int sr, int sc)  { return ((sc - sr) % STEP_B + STEP_B) % STEP_B == 0; }

/* which glyph belongs at a screen spot: both families cross -> 'X', else the
 * single family that passes through, else blank. */
static char grid_glyph_at(int sr, int sc)
{
    bool slash = on_slash_line(sr, sc);
    bool back  = on_back_line(sr, sc);
    if (slash && back) return 'X';
    if (slash)         return '/';
    if (back)          return '\\';
    return ' ';
}

static int glyph_pair(char ch)
{
    if (ch == 'X')  return PAIR_CROSS;
    if (ch == '/')  return PAIR_SLASH;
    return PAIR_BACK;
}

/* recipe step 2 — draw the mesh by asking grid_glyph_at at every screen spot */
static void draw_lattice(const GridCtx *g)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            char ch = grid_glyph_at(sr, sc);
            if (ch == ' ') continue;
            int pair = glyph_pair(ch);
            chtype attr = COLOR_PAIR(pair) | (ch == 'X' ? A_BOLD : A_NORMAL);
            attron(attr);
            mvaddch(sr, sc, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── §5 cursor ── */

/* Cursor — which cell the user is in, as (r,c) from the top-left cell (0,0).
 * Pair with a GridCtx and run through cell_to_screen to land on screen. */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r = g->max_r / 2;
    cur->c = g->max_c / 2;
}

/* recipe step 3 — move the cursor, clamped so it never steps off the grid */
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= 0 && nr <= g->max_r) cur->r = nr;
    if (nc >= 0 && nc <= g->max_c) cur->c = nc;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc; cell_to_screen(g, cur->r, cur->c, &sr, &sc);
    int cr = sr + g->ch / 2, cc = sc + g->cw / 2;
    if (cr >= 0 && cc >= 0) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(cr, cc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

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
