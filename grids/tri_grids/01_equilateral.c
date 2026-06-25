/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral.c — fill the screen with equilateral triangles and walk a
 * cursor across them with the arrow keys. We never store the triangles; for
 * every screen cell we just ask "which triangle is this?" and compute the
 * answer. This is the root of the tri_grids series — the other files change
 * one piece of it.
 *
 * Sister file: grids/rect_grids/01_uniform_rect.c does the same per-cell trick
 * on a plain square grid. Here the grid is slanted, because triangles tile at
 * 60-degree angles instead of square corners.
 *
 * References (things the code can't tell you):
 *   Triangular tiling   — https://en.wikipedia.org/wiki/Triangular_tiling
 *   Barycentric coords  — https://en.wikipedia.org/wiki/Barycentric_coordinate_system
 *   Red Blob Games (hex)— https://www.redblobgames.com/grids/hexagons/
 *   Coxeter, "Regular Polytopes" §4.6 (regular tilings of the plane)
 */

/* ── The idea, in plain words ──
 *
 * Picture the plane covered with slim diamonds (rhombuses), each one split
 * down the middle into two triangles: a "down" triangle (point at the bottom)
 * and an "up" triangle (point at the top). The whole thing is one repeating
 * pattern, so we don't keep a list of triangles. For any screen cell we work
 * backwards: where does this point land in the diamond pattern, and which half
 * of its diamond is it in?
 *
 * Finding a cell's triangle:
 *   1. Convert the screen cell to a point in the slanted grid — basically
 *      "how many steps along each of the two grid directions to get here?".
 *      Call those two numbers a and b. One grid direction is slanted, so we
 *      have to undo that slant when we compute a (subtract half of b).
 *   2. The whole-number parts of a and b say which diamond we're in.
 *   3. The leftover fractional parts say where inside the diamond we are.
 *      If they add up past the halfway line it's the up triangle, else down.
 *
 * Picking the character to draw:
 *   Each triangle has three corners. From the fractional position we get three
 *   "how close to each corner" weights. The smallest weight points at the edge
 *   we're nearest, and that edge decides which line character to draw: '/', '\'
 *   or '_'. If we're not near any edge (deep in the middle), draw nothing — the
 *   triangle interiors stay empty so you see clean outlines.
 *
 * Moving the cursor:
 *   Each arrow press steps the cursor across one edge into a neighbour, looked
 *   up in a small table. When there's no edge in that direction, it flips to
 *   the other triangle in the same diamond instead. Handy quirk: pressing UP
 *   twice always climbs one full row of triangles, whichever half you started
 *   in.
 *
 * Things to keep in mind:
 *  • Terminal characters are about twice as tall as wide, so a cell is 2 wide
 *    by 4 tall in our sub-pixel units. That's what makes the triangles look
 *    correctly equilateral instead of squashed.
 *  • All the grid math is floating point, so the cursor can jitter by a pixel
 *    right on a boundary. Harmless for a demo.
 *  • The bottom row is the HUD, so the triangle fill stops one row short.
 *  • Resizing just re-centers the grid; the cursor's position is independent of
 *    the terminal size, so it survives a resize.
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ── */

#define TARGET_FPS 60

/*
 * We pretend each terminal character is a little block of CELL_W by CELL_H
 * sub-pixels, so we can do smoother math than one-character steps allow. 2 wide
 * by 4 tall matches the roughly 1:2 shape of a real terminal character.
 */
#define CELL_W 2
#define CELL_H 4

/* How big one triangle is (its side length), plus how far +/- nudges it. */
#define TRI_SIZE_DEFAULT 14.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

/*
 * How close to an edge a point must be before we draw the edge. Small = thin
 * crisp outlines with hollow middles; large = fat outlines that nearly fill
 * the triangle. The [/] keys nudge it.
 */
#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.35
#define BORDER_W_STEP    0.02

#define N_THEMES 4

/* The fps number is smoothed so it doesn't twitch every frame. */
#define FPS_EWMA_ALPHA 0.05

/* Color pair IDs */
#define PAIR_BORDER 1   /* the triangle outlines             */
#define PAIR_CURSOR 2   /* the triangle the cursor is on + the '@' */
#define PAIR_HUD    3   /* yellow status bar (top right)     */
#define PAIR_HINT   4   /* cyan key hints (bottom left)      */

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

static const short THEME_FG[N_THEMES] = {
    /* 256-color preferred values, with 8-color fallback below */
     75,   /* steel blue   */
     82,   /* lime green   */
    214,   /* gold         */
     15,   /* bright white */
};
static const short THEME_FG_8[N_THEMES] = {
    COLOR_CYAN, COLOR_GREEN, COLOR_YELLOW, COLOR_WHITE,
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg = (COLORS >= 256) ? THEME_FG[theme] : THEME_FG_8[theme];
    init_pair(PAIR_BORDER, fg, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula ── */

/*
 * GridCtx — everything needed to map screen cells to triangles right now.
 *
 * The grid is kept centered on the screen, and the cell at the very center is
 * treated as the origin point (0,0) of the grid. From any terminal cell we get
 * a point relative to that center, then ask which triangle it falls in.
 *
 * tri_size and border_w live here (not as fixed constants) because the user can
 * tune them live with +/- and [/], and the drawing code reads them each frame.
 *
 * The plane of triangles is infinite, so max_col/max_row aren't hard limits —
 * they're just a rough "how far out is still on screen" estimate at the current
 * triangle size.
 */
typedef struct {
    int    rows, cols;     /* terminal size in cells                          */

    double tri_size;       /* triangle side length, in sub-pixels            */
    double border_w;       /* how close to an edge still counts as "on it"   */
    int    cw, ch;         /* sub-pixels per cell (copies of CELL_W, CELL_H) */

    int    ox, oy;         /* the center cell, used as grid origin (0,0)     */

    int    max_col, max_row; /* rough on-screen reach, not a hard boundary   */
} GridCtx;

/* Re-fits the grid to the current terminal size. Keeps the user's chosen
 * tri_size/border_w if they're already set, so a resize doesn't reset them. */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->cw   = CELL_W;
    g->ch   = CELL_H;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
    if (g->tri_size <= 0.0) g->tri_size = TRI_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    g->max_col = (int)((double)cols * CELL_W / g->tri_size) + 1;
    g->max_row = (int)((double)rows * CELL_H / (sqrt(3.0) * 0.5 * g->tri_size)) + 1;
}

/*
 * The heart of the demo: given a point on screen, work out which triangle it's
 * in. We measure how many steps along each grid direction reach the point (a
 * and b), undoing the grid's slant for the first one. The whole-number parts
 * say which diamond; the leftover fractions (fa, fb) say where inside it, and
 * whether their sum has crossed the halfway line tells up-triangle from down.
 */
static void ctx_pixel_to_tri(const GridCtx *g, double px, double py,
                             int *col, int *row, int *up,
                             double *fa, double *fb)
{
    double h = g->tri_size * sqrt(3.0) * 0.5;
    double b = py / h;
    double a = px / g->tri_size - 0.5 * b;
    int    c = (int)floor(a);
    int    r = (int)floor(b);
    *col = c;
    *row = r;
    *fa  = a - (double)c;
    *fb  = b - (double)r;
    *up  = (*fa + *fb >= 1.0) ? 1 : 0;
}

/* Finds the middle point of a given triangle, in pixels. This is the forward
 * direction of the grid math (triangle -> point), used to place the '@'. */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

/*
 * Which terminal cell sits at the middle of a triangle. We chop off the
 * fraction instead of rounding on purpose: it nudges the '@' a hair toward the
 * inside, so it never lands exactly on an outline character and get hidden.
 */
static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *sc = g->ox + (int)(cx_pix / g->cw);
    *sr = g->oy + (int)(cy_pix / g->ch);
}

/*
 * Picks the line character for a point inside a triangle, based on which edge
 * it's closest to: '/', '\', or '_'. It works out a "distance to each corner"
 * and the smallest one points at the nearest edge. It also hands back that
 * smallest distance in *out_min, so the caller can skip points that sit too
 * deep in the middle to be part of an outline.
 */
static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    if (up == 0) {           /* down */
        l1 = 1.0 - fa - fb;  ch1 = '/';
        l2 = fa;             ch2 = '\\';
        l3 = fb;             ch3 = '_';
    } else {                 /* up */
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa + fb - 1.0;  ch2 = '/';
        l3 = 1.0 - fa;       ch3 = '\\';
    }
    char   ch = ch1;
    double m  = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/*
 * Draws the whole triangle grid by visiting every cell, asking which triangle
 * it's in, and stamping the right outline character (or nothing for interiors).
 * Nothing is stored between frames — it's all recomputed, which is why a resize
 * costs nothing. While we're at it, if a cell belongs to the cursor's triangle
 * we paint it in the cursor color instead of the normal one.
 */
static void ctx_draw_bg(const GridCtx *g, int cC, int cR, int cU)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int    tC, tR, tU;
            double fa, fb, m;
            ctx_pixel_to_tri(g, px, py, &tC, &tR, &tU, &fa, &fb);
            char ch = tri_edge_char(tU, fa, fb, &m);
            if (m >= g->border_w) continue;

            int on_cur = (tC == cC && tR == cR && tU == cU);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── §5 cursor ── */

/*
 * Cursor — which triangle the user is pointing at, and nothing more.
 *
 * col/row pick the diamond; up says which half of it (0 = the down triangle,
 * 1 = the up triangle). It deliberately knows nothing about screen size or
 * pixels — pair it with a GridCtx and ctx_to_screen() turns it into a spot on
 * the screen.
 */
typedef struct { int col, row, up; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0;
    cur->row = 0;
    cur->up  = 0;
}

/*
 * TRI_DIR — the move table for the arrow keys. Look up [which arrow][which half
 * you're in now] and you get the change to apply: how col and row shift, and
 * which half you end up in. When an arrow has no edge to cross in that
 * direction, the entry just flips you to the other half of the same diamond.
 *   arrow order : 0=LEFT 1=RIGHT 2=UP 3=DOWN
 *   half order  : 0=down triangle, 1=up triangle
 */
static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 },    /* down → up(col-1, row)             */
                  {  0,  0,  0 } },  /* up   → down(col, row)  toggle      */
    /* RIGHT */ { {  0,  0,  1 },    /* down → up(col, row)    toggle      */
                  { +1,  0,  0 } },  /* up   → down(col+1, row)            */
    /* UP    */ { {  0, -1,  1 },    /* down → up(col, row-1)  cross top   */
                  {  0,  0,  0 } },  /* up   → down(col, row)  toggle      */
    /* DOWN  */ { {  0,  0,  1 },    /* down → up(col, row)    toggle      */
                  {  0, +1,  0 } },  /* up   → down(col, row+1) cross bot  */
};

/* Steps the cursor one triangle in the given direction. No edge-of-grid
 * clamping on purpose — the plane of triangles goes on forever. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dir)
{
    (void)g;
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0];
    cur->row += t[1];
    cur->up   = t[2];
}

/* Puts the '@' in the middle of the cursor's triangle. Called after the grid is
 * drawn so it sits on top of any outline character that shares that cell. */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->col, cur->row, cur->up, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

/* The on-screen overlay: status readout top-right, key hints along the bottom.
 * Bold and bright so it stays readable over the triangles. */
static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             g->tri_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [01 equilateral] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->col, cur->row, cur->up);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    atexit(screen_cleanup);
}

/* ── §8 app ── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

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
    int theme = 0, paused = 0;
    color_init(theme);

    GridCtx g = {0};
    g.tri_size = TRI_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           paused ^= 1; break;
                case 'r':           cursor_reset(&cur, &g); break;
                case 't':
                    theme = (theme + 1) % N_THEMES;
                    color_init(theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, &g, 0); break;
                case KEY_RIGHT: cursor_move(&cur, &g, 1); break;
                case KEY_UP:    cursor_move(&cur, &g, 2); break;
                case KEY_DOWN:  cursor_move(&cur, &g, 3); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) { g.tri_size += TRI_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) { g.tri_size -= TRI_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '[':
                    if (g.border_w > BORDER_W_MIN) { g.border_w -= BORDER_W_STEP; } break;
                case ']':
                    if (g.border_w < BORDER_W_MAX) { g.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns(), dt = now - t0; t0 = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
