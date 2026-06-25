/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 14_origin.c — a grid with math-style (x,y) axes drawn through the centre.
 *
 * Move the '@' cursor with the arrows; the HUD shows where it sits in math
 * coordinates and which quadrant it's in. Unlike the terminal, +Y points UP
 * here, like graph paper. Sister file: 01_uniform_rect.c (plain grid, no axes).
 *
 * The one trick: on a terminal, row 0 is the top and rows grow downward, but
 * in math, y grows upward. So when we turn a math point into a screen row we
 * subtract instead of add — that single minus sign flips the Y axis upright.
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
 * How big one math unit is on screen. Cells are taller than they are wide,
 * so we use more columns than rows per unit to keep each step looking square.
 */
#define UNIT_W   8    /* screen cols per 1 math unit */
#define UNIT_H   4    /* screen rows per 1 math unit */

/* How sharply the FPS readout reacts to each frame: small = smooth, slow. */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_GRID   1   /* faint background grid lines */
#define PAIR_XAXIS  2   /* the horizontal axis         */
#define PAIR_YAXIS  3   /* the vertical axis           */
#define PAIR_CURSOR 4
#define PAIR_QUAD   5   /* the I/II/III/IV labels      */
#define PAIR_HUD    6
#define PAIR_HINT   7

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
    init_pair(PAIR_GRID,   COLORS >= 256 ? 244 : COLOR_WHITE,  -1);
    init_pair(PAIR_XAXIS,  COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_YAXIS,  COLORS >= 256 ?  46 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_QUAD,   COLORS >= 256 ?  39 : COLOR_CYAN,   -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — turning math points into screen positions ── */

/*
 * Everything the drawing code needs to know about the current grid layout:
 * how big the terminal is, how many cells make one math unit, where the
 * centre (the origin) sits on screen, and how far the cursor may roam.
 *
 *   rows, cols    terminal size, in characters
 *   cw, ch        one math unit's width and height, in screen chars
 *   ox, oy        where math (0,0) lands on screen — its column and row
 *   range         cursor stays within [-range, +range] on both axes, so it
 *                 never wanders off the visible grid
 *   max_r, max_c  copies of range, kept only to match the shared grid template
 */
typedef struct {
    int rows, cols;
    int cw, ch;
    int ox, oy;
    int range;
    int max_r, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = UNIT_W; g->ch = UNIT_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->range = (rows < cols) ? rows / (2 * UNIT_H) - 1 : cols / (2 * UNIT_W) - 1;
    g->max_r = g->range;
    g->max_c = g->range;
}

/*
 * Where does a math point (mx, my) land on screen? Step right from the centre
 * for x, and step up for y — and "up" on a terminal means a smaller row, which
 * is why y is subtracted, not added. That minus is the whole Y flip.
 */
static void ctx_to_screen(const GridCtx *g, int my, int mx, int *sr, int *sc)
{
    *sc = g->ox + mx * g->cw;
    *sr = g->oy - my * g->ch;
}

/* The reverse trip: given a screen cell, which math point is it? */
static void screen_to_math(const GridCtx *g, int sr, int sc, int *mx, int *my)
{
    *mx = (sc - g->ox) / g->cw;
    *my = (g->oy - sr) / g->ch;
}

/*
 * What kind of mark belongs at a given screen cell, listed from most to least
 * important. The origin wins over the axes, and the axes win over plain grid
 * lines, so a cell that's both an axis and a grid line shows the axis.
 *
 *   GC_NONE     blank space, nothing to draw
 *   GC_GRID     a faint background grid line or crossing
 *   GC_XAXIS    the horizontal axis through the centre
 *   GC_YAXIS    the vertical axis through the centre
 *   GC_ORIGIN   the single centre point where both axes meet
 */
typedef enum { GC_NONE, GC_GRID, GC_XAXIS, GC_YAXIS, GC_ORIGIN } GridCharType;

static GridCharType ctx_grid_char_type(const GridCtx *g, int sr, int sc, char *out_ch)
{
    bool on_x = (sr == g->oy);
    bool on_y = (sc == g->ox);
    if (on_x && on_y) { *out_ch = 'O'; return GC_ORIGIN; }
    if (on_x)         { *out_ch = '='; return GC_XAXIS; }
    if (on_y)         { *out_ch = '|'; return GC_YAXIS; }

    int dr = sr - g->oy, dc = sc - g->ox;
    bool gh = (dr % g->ch == 0);
    bool gv = (dc % g->cw == 0);
    if (gh && gv) { *out_ch = '+'; return GC_GRID; }
    if (gh)       { *out_ch = '-'; return GC_GRID; }
    if (gv)       { *out_ch = ':'; return GC_GRID; }
    *out_ch = ' ';
    return GC_NONE;
}

/* Same question as above, but when you only want the character, not its kind. */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    char ch; ctx_grid_char_type(g, sr, sc, &ch);
    return ch;
}

/* Paints the whole grid and axes, giving each cell the colour its kind earns. */
static void ctx_draw_bg(const GridCtx *g)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            char ch; GridCharType t = ctx_grid_char_type(g, sr, sc, &ch);
            if (t == GC_NONE) continue;
            if (t == GC_ORIGIN) {
                attron(COLOR_PAIR(PAIR_XAXIS) | A_BOLD);
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_XAXIS) | A_BOLD);
            } else if (t == GC_XAXIS) {
                attron(COLOR_PAIR(PAIR_XAXIS));
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_XAXIS));
            } else if (t == GC_YAXIS) {
                attron(COLOR_PAIR(PAIR_YAXIS));
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_YAXIS));
            } else {
                attron(COLOR_PAIR(PAIR_GRID));
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_GRID));
            }
        }
    }
}

/* Drops a Roman-numeral label into the middle of each of the four corners. */
static void labels_draw(const GridCtx *g)
{
    int r, c;
    attron(COLOR_PAIR(PAIR_QUAD) | A_DIM);
    /* I: right and above the centre */
    ctx_to_screen(g, 2,  2, &r, &c);  mvprintw(r, c, "I");
    /* II: left and above */
    ctx_to_screen(g, 2, -3, &r, &c);  mvprintw(r, c, "II");
    /* III: left and below */
    ctx_to_screen(g, -2, -3, &r, &c); mvprintw(r, c, "III");
    /* IV: right and below */
    ctx_to_screen(g, -2,  2, &r, &c); mvprintw(r, c, "IV");
    attroff(COLOR_PAIR(PAIR_QUAD) | A_DIM);
}

/* ── §5 cursor ── */

/*
 * The '@' the user steers, stored in math coordinates rather than screen ones.
 *
 *   mx   horizontal position; positive is right of centre
 *   my   vertical position; positive is ABOVE centre, math-style
 *
 * How far it can travel is decided by GridCtx.range, not stored here.
 */
typedef struct { int mx, my; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->mx = 0; cur->my = 0;
}

/*
 * Nudges the cursor by one step, but only if it would stay inside the grid;
 * a step off the edge is simply ignored so the '@' can't escape.
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dmx, int dmy)
{
    int nx = cur->mx + dmx, ny = cur->my + dmy;
    if (nx >= -g->range && nx <= g->range) cur->mx = nx;
    if (ny >= -g->range && ny <= g->range) cur->my = ny;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc; ctx_to_screen(g, cur->my, cur->mx, &sr, &sc);
    /* Round-trip back to math coords as a sanity check; result is unused. */
    int vx, vy; screen_to_math(g, sr, sc, &vx, &vy);
    (void)vx; (void)vy;
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, sc, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ── §6 scene ── */

static const char *quadrant_name(int mx, int my)
{
    if (mx == 0 || my == 0) return "axis";
    if (mx > 0 && my > 0) return "I";
    if (mx < 0 && my > 0) return "II";
    if (mx < 0 && my < 0) return "III";
    return "IV";
}

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  math(%+d,%+d)  Q%s ",
        fps, cur->mx, cur->my, quadrant_name(cur->mx, cur->my));
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [14 origin  screen_row=oy-my*UNIT_H] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g);
    labels_draw(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, fps);
    wnoutrefresh(stdscr); doupdate();
}

/* ctx_grid_char isn't called anywhere; this touches it so the compiler
 * doesn't warn about an unused function. */
static void ctx_grid_char_ref(void) { (void)ctx_grid_char; }

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
    ctx_grid_char_ref();   /* keeps the unused helper from being flagged */

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
            /* Arrows move one math unit; UP raises my, so '@' rises on screen. */
            case KEY_UP:    cursor_move(&cur, &g,  0, +1); break;
            case KEY_DOWN:  cursor_move(&cur, &g,  0, -1); break;
            case KEY_LEFT:  cursor_move(&cur, &g, -1,  0); break;
            case KEY_RIGHT: cursor_move(&cur, &g, +1,  0); break;
        }
        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&g, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
