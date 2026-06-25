/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 14_origin.c — a math-axis grid: origin pinned at the screen centre, with
 * +Y pointing UP like graph paper instead of down like a terminal.
 *
 * The distinct mapping: screen_col = ox + mx*UNIT_W, screen_row = oy - my*UNIT_H.
 * That minus on the row is the whole Y flip — math y grows up, terminal rows
 * grow down. Sister file: 01_uniform_rect.c (plain grid, no axes, no origin).
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

#define UNIT_W   8    /* screen cols per 1 math unit; cells are wider than tall, */
#define UNIT_H   4    /* so more cols than rows per unit keeps a step looking square */

#define FPS_EWMA_ALPHA  0.05   /* small = steadier on-screen fps number */

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

/* ── §4 math-axis mapping & lattice ── */

/* GridCtx — the grid for one frame: terminal size, unit size, where the origin
 * lands on screen, and how far the cursor may roam from it.
 *   ox, oy        screen column/row where math (0,0) lands (the centre)
 *   range         cursor stays in [-range, +range] on both axes
 *   max_r, max_c  copies of range, kept to match the shared rect template */
typedef struct {
    int rows, cols;      /* terminal size in characters */
    int cw, ch;          /* one math unit's width and height, in screen chars */
    int ox, oy;          /* screen position of the origin */
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

/* recipe step 1 — math point (mx,my) -> its screen cell. Step right for x,
 * step UP for y; "up" is a smaller row, so y is subtracted. That is the Y flip. */
static void cell_to_screen(const GridCtx *g, int mx, int my, int *sr, int *sc)
{
    *sc = g->ox + mx * g->cw;
    *sr = g->oy - my * g->ch;
}

/* what a screen cell is, by priority: origin beats axes, axes beat grid lines,
 * so a cell that is both axis and grid line shows the axis. */
typedef enum { GC_NONE, GC_GRID, GC_XAXIS, GC_YAXIS, GC_ORIGIN } GridCharType;

static GridCharType grid_type_at(const GridCtx *g, int sr, int sc, char *out_ch)
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

static int pair_for_type(GridCharType t)
{
    switch (t) {
        case GC_ORIGIN: return PAIR_XAXIS;   /* origin painted as bold x-axis */
        case GC_XAXIS:  return PAIR_XAXIS;
        case GC_YAXIS:  return PAIR_YAXIS;
        default:        return PAIR_GRID;
    }
}

/* recipe step 2 — draw the axes & grid by asking grid_type_at at every spot */
static void draw_lattice(const GridCtx *g)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            char ch; GridCharType t = grid_type_at(g, sr, sc, &ch);
            if (t == GC_NONE) continue;
            chtype attr = COLOR_PAIR(pair_for_type(t));
            if (t == GC_ORIGIN) attr |= A_BOLD;
            attron(attr);
            mvaddch(sr, sc, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* a Roman-numeral label dropped into the middle of each quadrant */
static void labels_draw(const GridCtx *g)
{
    int r, c;
    attron(COLOR_PAIR(PAIR_QUAD) | A_DIM);
    cell_to_screen(g,  2,  2, &r, &c);  mvprintw(r, c, "I");     /* upper-right */
    cell_to_screen(g, -3,  2, &r, &c);  mvprintw(r, c, "II");    /* upper-left  */
    cell_to_screen(g, -3, -2, &r, &c);  mvprintw(r, c, "III");   /* lower-left  */
    cell_to_screen(g,  2, -2, &r, &c);  mvprintw(r, c, "IV");    /* lower-right */
    attroff(COLOR_PAIR(PAIR_QUAD) | A_DIM);
}

/* ── §5 cursor ── */

/* Cursor — the '@', stored in math coords. mx>0 is right of centre, my>0 is
 * ABOVE centre (math-style). How far it can travel lives in GridCtx.range. */
typedef struct { int mx, my; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->mx = 0; cur->my = 0;
}

/* recipe step 3 — move the cursor, clamped so it never steps off the grid */
static void cursor_move(Cursor *cur, const GridCtx *g, int dmx, int dmy)
{
    int nx = cur->mx + dmx, ny = cur->my + dmy;
    if (nx >= -g->range && nx <= g->range) cur->mx = nx;
    if (ny >= -g->range && ny <= g->range) cur->my = ny;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc; cell_to_screen(g, cur->mx, cur->my, &sr, &sc);
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
    draw_lattice(g);
    labels_draw(g);
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
