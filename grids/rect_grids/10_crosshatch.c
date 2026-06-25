/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 10_crosshatch.c — two sets of diagonal lines overlaid to make a cross-hatch.
 *
 * One set of '/' lines and one set of '\' lines, each with its own spacing,
 * laid on top of each other. They weave into a diamond mesh (like tartan).
 * A cursor hops between the diamond cells. The trick: we never store any
 * lines — for each screen spot we just ask two yes/no questions and draw.
 *
 * Study alongside: 08_diamond.c (one diagonal set), 01_uniform_rect.c.
 * The pattern idea: en.wikipedia.org/wiki/Crosshatching.
 *
 * The two questions, per screen spot (row sr, column sc):
 *   on a '/' line?  -> sum (sc + sr) lands on a multiple of STEP_A.
 *   on a '\' line?  -> difference (sc - sr) lands on a multiple of STEP_B.
 * Why those? Walk along a '/' line and the sum stays put; walk along a '\'
 * line and the difference stays put. So "every STEP_A steps of the sum"
 * spaces out the '/' lines, and likewise STEP_B for the '\' lines. Both true
 * at once means the two lines cross, so we draw an 'X' there.
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

/* ── §4 formula — turning a cursor cell into a screen spot, and drawing lines ── */

/*
 * GridCtx — everything we need to know about the current terminal so the
 * cursor stays inside it. Recomputed whenever the window is resized.
 *   rows, cols     — terminal size in characters.
 *   cw, ch         — size of one cursor cell (matches CELL_W / CELL_H).
 *   max_r, max_c   — highest legal cursor cell, so it can't walk off-screen.
 */
typedef struct {
    int rows, cols;
    int cw, ch;
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->max_r = (rows - 1) / CELL_H - 1;
    g->max_c = cols / CELL_W - 1;
}

static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    *sr = r * g->ch;
    *sc = c * g->cw;
}

/*
 * Answers the two yes/no questions for one screen spot: is it on a '/' line,
 * and is it on a '\' line? The "+ STEP_B" dance is there because (sc - sr) can
 * go negative, and C's % keeps the sign — without it the '\' lines would
 * vanish in the lower-left of the screen.
 */
static void crosshatch_test(int sr, int sc, bool *slash, bool *back)
{
    *slash = ((sc + sr) % STEP_A == 0);
    *back  = (((sc - sr) % STEP_B + STEP_B) % STEP_B == 0);
}

/* Walks every screen spot and paints '/', '\', or 'X' where the lines fall. */
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

/* ── §5 cursor ── */

/* Where the '@' sits, in cursor-cell coordinates (not screen pixels). */
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
