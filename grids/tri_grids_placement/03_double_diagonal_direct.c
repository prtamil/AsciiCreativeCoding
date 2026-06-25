/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal_direct.c — drop markers onto a tetrakis (double-diagonal) grid
 *
 * A grid of square cells fills the screen, each square cut by BOTH diagonals
 * into 4 triangular wedges (N/E/S/W). Move the '@' cursor with the arrows,
 * hit SPACE to leave a marker on the wedge you're standing on. Each marker
 * remembers WHICH wedge it sits on (a col/row/wedge address), not where on
 * screen it is — so on resize or size change, markers follow their wedge.
 *
 * Sister files: grids/tri_grids/03_double_diagonal.c (just the background),
 *               02_right_isosceles_direct.c (1 diagonal, 2 wedges).
 * Tetrakis square tiling: https://en.wikipedia.org/wiki/Tetrakis_square_tiling
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

/* ── §1 config ── */

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 18.0
#define TRI_SIZE_MIN      8.0
#define TRI_SIZE_MAX     48.0
#define TRI_SIZE_STEP     2.0

#define BORDER_W   0.10
#define MAX_OBJ    256
#define N_GLYPHS   6
#define N_THEMES   4

#define FPS_EWMA_ALPHA 0.05

/* The 4 wedges, named by which compass way their point faces */
#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_OBJECT 3
#define PAIR_HUD    4
#define PAIR_HINT   5

static const char  GLYPHS[N_GLYPHS] = { '*', 'o', '+', '#', 'X', '%' };
static const char *DIR_NAME[4]      = { "N", "E", "S", "W" };

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

static const short THEME_FG[N_THEMES][2] = {
    /* edge,  object */
    {  82, 226 },
    { 207, 226 },
    { 207,  82 },
    {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_GREEN,   COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_GREEN  },
    { COLOR_WHITE,   COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e = (COLORS >= 256) ? THEME_FG[theme][0]   : THEME_FG_8[theme][0];
    short fg_o = (COLORS >= 256) ? THEME_FG[theme][1]   : THEME_FG_8[theme][1];
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 tri mapping & lattice ── */

/* GridCtx — the tetrakis grid for one frame, plus the user's live knobs.
 * Positions are measured in sub-pixels first, then divided down to character
 * cells, so square size can change smoothly. Centred on (ox,oy) so markers
 * stay on their wedge across a resize. */
typedef struct {
    int    rows, cols;      /* terminal size in character cells */
    double tri_size;        /* square side length, in sub-pixels */
    int    cell_w, cell_h;  /* sub-pixels per character column / row */
    int    ox, oy;          /* grid (0,0) on screen; re-centred each frame */
    double border_w;        /* a cell is on an edge if it's within this of one */
    int    theme;           /* colour scheme, 0..N_THEMES-1 */
    int    paused;          /* 1 while paused ('p') */
    int    glyph_idx;       /* which marker char SPACE drops next */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows     = rows;
    g->cols     = cols;
    g->tri_size = TRI_SIZE_DEFAULT;
    g->cell_w   = CELL_W;
    g->cell_h   = CELL_H;
    g->ox       = cols / 2;
    g->oy       = (rows - 1) / 2;
    g->border_w = BORDER_W;
    g->theme    = 0;
    g->paused   = 0;
    g->glyph_idx = 0;
}

/* on a resize, redo only the size-dependent fields and leave the user's
 * knobs (square size, theme, glyph) exactly as they were */
static void ctx_resize(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
}

/* recipe step 1 (reverse) — a pixel -> which square (col,row) + which wedge +
 * where inside (fa,fb). Lean from the square's centre: whichever axis it leans
 * the most (left/right vs up/down) names the wedge. */
static void screen_to_tri(double px, double py, double size,
                         int *col, int *row, int *wedge,
                         double *fa, double *fb)
{
    double inv = 1.0 / size;
    double a = px * inv;
    double b = py * inv;
    int    c = (int)floor(a);
    int    r = (int)floor(b);
    *col = c; *row = r;
    *fa  = a - (double)c;
    *fb  = b - (double)r;
    double dx = *fa - 0.5, dy = *fb - 0.5;
    if (fabs(dx) > fabs(dy)) *wedge = (dx > 0.0) ? DIR_E : DIR_W;
    else                     *wedge = (dy > 0.0) ? DIR_S : DIR_N;
}

/* the pixel at a wedge's centre, a third of the way out from the square's
 * centre toward its edge. This is where the cursor/marker is drawn so it
 * lands cleanly inside. */
static void tri_centroid_pixel(int col, int row, int wedge, double size,
                               double *cx_pix, double *cy_pix)
{
    double a, b;
    switch (wedge) {
        case DIR_N: a = 0.5;     b = 1.0/6.0; break;
        case DIR_E: a = 5.0/6.0; b = 0.5;     break;
        case DIR_S: a = 0.5;     b = 5.0/6.0; break;
        default:    a = 1.0/6.0; b = 0.5;     break;
    }
    *cx_pix = ((double)col + a) * size;
    *cy_pix = ((double)row + b) * size;
}

/* a wedge's address -> the character cell to draw on. Truncates (not rounds)
 * so the mark sits inside the wedge, never on an edge. */
static void tri_to_screen(const GridCtx *g, int col, int row, int wedge,
                          int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, wedge, g->tri_size, &cx_pix, &cy_pix);
    *scol = g->ox + (int)(cx_pix / g->cell_w);
    *srow = g->oy + (int)(cy_pix / g->cell_h);
}

/* the line char for a point by nearest of the wedge's 3 sides: '/', '\', '_', '|'.
 * out_min returns the distance to that side, so the caller skips open-space
 * cells (only draws when it's below border_w). */
static char edge_glyph(int wedge, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    switch (wedge) {
        case DIR_N:
            l1 = 1.0 - fa - fb;     ch1 = '/';
            l2 = fa - fb;           ch2 = '\\';
            l3 = 2.0 * fb;          ch3 = '_';
            break;
        case DIR_E:
            l1 = fa - fb;           ch1 = '\\';
            l2 = fa + fb - 1.0;     ch2 = '/';
            l3 = 2.0 * (1.0 - fa);  ch3 = '|';
            break;
        case DIR_S:
            l1 = fb - fa;           ch1 = '\\';
            l2 = fa + fb - 1.0;     ch2 = '/';
            l3 = 2.0 * (1.0 - fb);  ch3 = '_';
            break;
        default: /* DIR_W */
            l1 = 1.0 - fa - fb;     ch1 = '\\';
            l2 = fb - fa;           ch2 = '/';
            l3 = 2.0 * fa;          ch3 = '|';
            break;
    }
    char   ch = ch1;
    double m  = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/* recipe step 2 — draw the grid: every cell -> its wedge -> a line char only
 * on the edge cells (interiors stay blank). Nothing stored; redrawn each frame. */
static void draw_lattice(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;
            int    tC, tR, tW;
            double fa, fb, m;
            screen_to_tri(px, py, g->tri_size, &tC, &tR, &tW, &fa, &fb);
            char ch = edge_glyph(tW, fa, fb, &m);
            if (m >= g->border_w) continue;
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

/* ── §5 pool — the bag of placed markers ── */

/* One placed marker. It's pinned to a wedge by address, not to a screen
 * spot, so it stays put when the view changes.
 *   col, row, wedge — which wedge (wedge: DIR_N/E/S/W)
 *   glyph           — the character to draw for this marker
 *   alive           — true if this slot holds a real marker (vs. an empty slot) */
typedef struct { int col, row, wedge; char glyph; bool alive; } Obj;

/* The whole collection of markers: a plain fixed-size array plus a count of
 * how many are in use. No growing, no allocation — full at MAX_OBJ. We keep
 * the live ones packed at the front (slots 0..count-1), so a removal just
 * moves the last one into the gap. */
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

static int pool_find(const Pool *p, int col, int row, int wedge)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].alive
            && p->items[i].col   == col
            && p->items[i].row   == row
            && p->items[i].wedge == wedge)
            return i;
    return -1;
}

static void pool_place(Pool *p, int col, int row, int wedge, char glyph)
{
    if (pool_find(p, col, row, wedge) >= 0) return;
    if (p->count >= MAX_OBJ) return;
    p->items[p->count++] = (Obj){ col, row, wedge, glyph, true };
}

static void pool_remove(Pool *p, int col, int row, int wedge)
{
    int i = pool_find(p, col, row, wedge);
    if (i < 0) return;
    p->items[i] = p->items[--p->count];
}

static void pool_toggle(Pool *p, int col, int row, int wedge, char glyph)
{
    if (pool_find(p, col, row, wedge) >= 0) pool_remove(p, col, row, wedge);
    else                                    pool_place(p, col, row, wedge, glyph);
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        tri_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].wedge,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
}

/* ── §6 cursor — the '@' you steer ── */

/* Where the '@' currently is: the address of one wedge. Same three numbers a
 * marker uses (col, row, and wedge = which quarter of the square). */
typedef struct { int col, row, wedge; } Cursor;

/*
 * Moving by one arrow press isn't a simple ±1 on a tetrakis grid. Pressing an
 * arrow moves you toward that compass direction: usually you just flip to the
 * neighbouring wedge in the same square, but if your wedge already points that
 * way you hop into the next square and land in the wedge pointing back (e.g.
 * LEFT while in the W wedge carries you into the E wedge of the square left).
 * This table spells out every case so cursor_move doesn't have to.
 * Look it up by [which arrow][which wedge you're on] to get the change to
 * apply: how much col and row shift, and which wedge you end up on.
 *   arrows: 0=LEFT 1=RIGHT 2=UP 3=DOWN     wedge: DIR_N/E/S/W
 */
static const int TRI_DIR[4][4][3] = {
    /* LEFT  */ { {  0,  0, DIR_W }, {  0,  0, DIR_W }, {  0,  0, DIR_W }, { -1,  0, DIR_E } },
    /* RIGHT */ { {  0,  0, DIR_E }, { +1,  0, DIR_W }, {  0,  0, DIR_E }, {  0,  0, DIR_E } },
    /* UP    */ { {  0, -1, DIR_S }, {  0,  0, DIR_N }, {  0,  0, DIR_N }, {  0,  0, DIR_N } },
    /* DOWN  */ { {  0,  0, DIR_S }, {  0,  0, DIR_S }, {  0, +1, DIR_N }, {  0,  0, DIR_S } },
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0; cur->row = 0; cur->wedge = DIR_N;
}

/*
 * Nudge the cursor by the change looked up in TRI_DIR. There's no edge to
 * bump into — the grid of addresses goes on forever, so we never clamp; the
 * cursor can simply wander off the visible window and back.
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dcol, int drow, int wedge)
{
    (void)g;
    cur->col  += dcol;
    cur->row  += drow;
    cur->wedge = wedge;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sc, sr;
    tri_to_screen(g, cur->col, cur->row, cur->wedge, &sc, &sr);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §7 scene ── */

/* Bright bold yellow fps readout (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, DIR_NAME[cur->wedge],
             p->count, GLYPHS[g->glyph_idx], g->tri_size,
             g->theme, fps, g->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:toggle  g:glyph  C:clear  +/-:size  t:theme  r:reset  q:quit  [03 double diagonal direct] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       double fps)
{
    erase();
    draw_lattice(g);
    pool_draw(p, g);
    cursor_draw(cur, g);
    hud_draw(g, p, cur, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §8 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §9 app — signals and the main loop ── */

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

    GridCtx g; ctx_init(&g, 0, 0);
    screen_init(g.theme);
    ctx_init(&g, LINES, COLS);

    Cursor cur; cursor_reset(&cur, &g);
    Pool   pool; pool_clear(&pool);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_resize(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            const int *t;
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g.paused ^= 1; break;
                case 'r':          cursor_reset(&cur, &g); pool_clear(&pool);
                                   color_init(g.theme); break;
                case 'C':          pool_clear(&pool); break;
                case 'g':          g.glyph_idx = (g.glyph_idx + 1) % N_GLYPHS; break;
                case ' ':          pool_toggle(&pool, cur.col, cur.row, cur.wedge,
                                                GLYPHS[g.glyph_idx]); break;
                case 't':
                    g.theme = (g.theme + 1) % N_THEMES;
                    color_init(g.theme);
                    break;
                case KEY_LEFT:  t = TRI_DIR[0][cur.wedge]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_RIGHT: t = TRI_DIR[1][cur.wedge]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_UP:    t = TRI_DIR[2][cur.wedge]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case KEY_DOWN:  t = TRI_DIR[3][cur.wedge]; cursor_move(&cur, &g, t[0], t[1], t[2]); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) { g.tri_size += TRI_SIZE_STEP; } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) { g.tri_size -= TRI_SIZE_STEP; } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &pool, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
