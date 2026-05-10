/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal_direct.c — direct placement on tetrakis (4-wedge) grid
 *
 * DEMO: Each square is split by BOTH diagonals into 4 right-isosceles
 *       triangles labelled by the direction their apex points: N, E, S,
 *       W. Move '@' with arrows; SPACE toggles a glyph at the cursor
 *       wedge. An arrow key moves toward that compass direction —
 *       within the current square if possible, jumping squares when at
 *       the matching apex.
 *
 * Study alongside: grids/tri_grids/03_double_diagonal.c (rasterizer),
 *                  02_right_isosceles_direct.c (1 diagonal, 2 wedges).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, MAX_OBJ, DIR_N/E/S/W
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: edge / cursor / object / HUD / hint
 *   §4 gridctx  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg
 *   §5 pool     — Pool: place / remove / toggle / find / clear / draw
 *   §6 cursor   — Cursor + TETRA_DIR + reset / move / draw
 *   §7 mode     — direct toggle (lives in main loop)
 *   §8 scene    — hud_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move  spc:toggle  g:glyph  C:clear  r:reset
 *        +/-:size  t:theme  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/03_double_diagonal_direct.c \
 *       -o 03_double_diagonal_direct -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Tetrakis (a.k.a. "kis-square") tiling — every square
 *                  cut by both diagonals into 4 right-isosceles wedges
 *                  meeting at the centre. The cursor lives in
 *                  (col, row, wedge) where wedge ∈ {N, E, S, W}; SPACE
 *                  toggles an object at that address.
 *
 * Data-structure : Pool — flat array of Obj{col, row, wedge, glyph,
 *                  alive}. pool_remove swaps the dead slot with the
 *                  last item (O(1) removal, O(n) find).
 *
 * Wedge picker   : Translate (fa, fb) so the square centre is the origin
 *                  by computing (dx, dy) = (fa-0.5, fb-0.5). The wedge
 *                  is named by which axis dominates and the sign:
 *                    |dx| > |dy|, dx > 0 → E
 *                    |dx| > |dy|, dx ≤ 0 → W
 *                    |dy| ≥ |dx|, dy > 0 → S
 *                    |dy| ≥ |dx|, dy ≤ 0 → N
 *
 * References     :
 *   Tetrakis square tiling — https://en.wikipedia.org/wiki/Tetrakis_square_tiling
 *   Object pool pattern — gameprogrammingpatterns.com/object-pool.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The grid is square cells, each cut into 4 wedges by the two diagonals.
 * An address is (col, row, wedge) where wedge ∈ {N, E, S, W} — the apex
 * direction of the wedge. The cursor walks wedges; SPACE pins a glyph
 * at the current wedge. The grid lives only as arithmetic; objects
 * live in an array of (col, row, wedge) records.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a square cake split into 4 triangular slices by an X cut.
 * Each slice points outward in one cardinal direction. The cursor sits
 * in one slice; pressing N steps to the slice that points N — INSIDE
 * the same square if the current wedge is on the SOUTH side, or by
 * jumping to the square above when already pointing N. The TETRA_DIR
 * table encodes all 16 (arrow, current wedge) → (Δcol, Δrow, new wedge)
 * transitions.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. ctx_draw_bg — raster scan: for every screen cell, pixel_to_tri
 *     classifies the wedge, tri_edge_char picks an ASCII glyph by
 *     barycentric proximity to the wedge's three edges (two diagonals
 *     and one square side).
 *  3. pool_draw — for each placed object, glyph at wedge centroid
 *     screen cell.
 *  4. cursor_draw — '@' on top.
 *  5. hud_draw — top-right status, bottom-row hint.
 *
 * KEY FORMULAS
 * ────────────
 *  Pixel → square + wedge:
 *    a = px / size,   b = py / size
 *    col = ⌊a⌋,        row = ⌊b⌋
 *    fa  = a - col,    fb  = b - row
 *    dx  = fa - 0.5,   dy  = fb - 0.5
 *    if |dx| > |dy|:   wedge = (dx > 0) ? E : W
 *    else:             wedge = (dy > 0) ? S : N
 *
 *  Centroid lattice → pixel  (each centroid is 1/3 from the apex):
 *    N: (col + 1/2,    row + 1/6) · size
 *    E: (col + 5/6,    row + 1/2) · size
 *    S: (col + 1/2,    row + 5/6) · size
 *    W: (col + 1/6,    row + 1/2) · size
 *
 *  Cursor step: TETRA_DIR[arrow][cur->wedge] → (Δcol, Δrow, new_wedge).
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Centre tie: at fa == fb == 0.5 the cursor sits exactly at the
 *    apex meeting point. The classifier then resolves via the |dx| vs
 *    |dy| comparison (using ≥), defaulting to N/S over E/W. Harmless.
 *  • Glyph cycle, MAX_OBJ cap, resize behaviour: identical to
 *    01_equilateral_direct.c.
 *  • Aspect: CELL_W=2, CELL_H=4 → cells are 1:2 wide:tall. Wedges are
 *    right-isosceles in PIXEL space; on screen they appear taller than
 *    wide. This is a faithful aspect-corrected rendering, not a bug.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Place a glyph in the N wedge of (0,0). Press DOWN: cursor → S
 *  wedge of (0,0) (same square). Press DOWN again: cursor → N wedge
 *  of (0,1) (next row). Two DOWNs traverse one whole square.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 18.0
#define TRI_SIZE_MIN      8.0
#define TRI_SIZE_MAX     48.0
#define TRI_SIZE_STEP     2.0
#define BORDER_W          0.10

#define MAX_OBJ    256
#define N_GLYPHS   6
#define N_THEMES   4

#define FPS_EWMA_ALPHA 0.05

/* Apex direction indices */
#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_OBJECT 3
#define PAIR_HUD    4   /* status bar (yellow 226) */
#define PAIR_HINT   5   /* key-hint footer (cyan 51) */

static const char  GLYPHS[N_GLYPHS] = { '*', 'o', '+', '#', 'X', '%' };
static const char *DIR_NAME[4]      = { "N", "E", "S", "W" };

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEME_FG[N_THEMES][2] = {
    {  82, 226 }, { 207, 226 }, { 207,  82 }, {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_GREEN,   COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_GREEN  },
    { COLOR_WHITE,   COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e = (COLORS >= 256) ? THEME_FG[theme][0] : THEME_FG_8[theme][0];
    short fg_o = (COLORS >= 256) ? THEME_FG[theme][1] : THEME_FG_8[theme][1];
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx — tetrakis pixel↔lattice + background raster                */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — tetrakis geometry + screen origin.
 *
 * tri_size lives here so the rest of the file does not depend on the
 * cursor for pure-grid math; the cursor only owns the user's address.
 */
typedef struct {
    int    rows, cols;
    int    cw, ch;          /* terminal-cell size in sub-pixels */
    int    ox, oy;          /* lattice origin in screen chars */
    double tri_size;        /* square side length in pixels */
    double border_w;        /* edge-render threshold (lattice units) */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, double tri_size)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->tri_size = tri_size;
    g->border_w = BORDER_W;
}

/*
 * pixel_to_tri — square cell + wedge classifier.
 *
 *   a = px / size,   b = py / size
 *   col = ⌊a⌋, row = ⌊b⌋, fa = a-col, fb = b-row
 *   dx = fa - ½, dy = fb - ½
 *   |dx| > |dy|, dx > 0  →  E
 *   |dx| > |dy|, dx < 0  →  W
 *   |dy| ≥ |dx|, dy > 0  →  S
 *   |dy| ≥ |dx|, dy < 0  →  N
 */
static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *wedge,
                         double *fa, double *fb)
{
    double inv = 1.0 / size;
    double a = px * inv, b = py * inv;
    int    c = (int)floor(a), r = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c; *fb = b - (double)r;
    double dx = *fa - 0.5, dy = *fb - 0.5;
    double adx = fabs(dx), ady = fabs(dy);
    if (adx > ady) *wedge = (dx > 0.0) ? DIR_E : DIR_W;
    else           *wedge = (dy > 0.0) ? DIR_S : DIR_N;
}

/*
 * tri_centroid_pixel — forward map for cursor / object / mark.
 *
 *   N: (½, 1/6)    E: (5/6, ½)    S: (½, 5/6)    W: (1/6, ½)
 */
static void tri_centroid_pixel(int col, int row, int wedge, double size,
                               double *cx, double *cy)
{
    double a, b;
    switch (wedge) {
        case DIR_N: a = 0.5;     b = 1.0/6.0; break;
        case DIR_E: a = 5.0/6.0; b = 0.5;     break;
        case DIR_S: a = 0.5;     b = 5.0/6.0; break;
        default:    a = 1.0/6.0; b = 0.5;     break;
    }
    *cx = ((double)col + a) * size;
    *cy = ((double)row + b) * size;
}

/*
 * tri_edge_char — barycentric weights → edge character per wedge.
 *
 * Each wedge has C=(½,½) as apex; its two diagonal half-edges meet at C.
 *   N (A=(0,0), B=(1,0), C=(½,½)):
 *     l_A = 1−fa−fb,    l_B = fa−fb,        l_C = 2·fb
 *     l_A → '/'   l_B → '\\'  l_C → '_'
 *   E (A=(1,0), B=(1,1), C=(½,½)):
 *     l_A = fa−fb,      l_B = fa+fb−1,      l_C = 2·(1−fa)
 *     l_A → '\\'  l_B → '/'   l_C → '|'
 *   S (A=(0,1), B=(1,1), C=(½,½)):
 *     l_A = fb−fa,      l_B = fa+fb−1,      l_C = 2·(1−fb)
 *     l_A → '\\'  l_B → '/'   l_C → '_'
 *   W (A=(0,0), B=(0,1), C=(½,½)):
 *     l_A = 1−fa−fb,    l_B = fb−fa,        l_C = 2·fa
 *     l_A → '\\'  l_B → '/'   l_C → '|'
 */
static char tri_edge_char(int wedge, double fa, double fb, double *out_min)
{
    double l1, l2, l3; char ch1, ch2, ch3;
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
    char ch = ch1; double m = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/* Map (col, row, wedge) → terminal cell. */
static void ctx_to_screen(const GridCtx *g, int col, int row, int wedge,
                          int *scol, int *srow)
{
    double cx, cy;
    tri_centroid_pixel(col, row, wedge, g->tri_size, &cx, &cy);
    *scol = g->ox + (int)(cx / g->cw);
    *srow = g->oy + (int)(cy / g->ch);
}

static void ctx_draw_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;
            int tC, tR, tW; double fa, fb, m;
            pixel_to_tri(px, py, g->tri_size, &tC, &tR, &tW, &fa, &fb);
            char ch = tri_edge_char(tW, fa, fb, &m);
            if (m >= g->border_w) continue;
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int col, row, wedge; char glyph; bool alive; } Obj;
typedef struct { Obj items[MAX_OBJ];  int count; } Pool;

static int pool_find(const Pool *p, int col, int row, int wedge)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].alive
            && p->items[i].col == col
            && p->items[i].row == row
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
        ctx_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].wedge,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int col, row, wedge;
    int glyph_idx;
    int theme;
    int paused;
} Cursor;

/*
 * TETRA_DIR — arrow-key transitions (Δcol, Δrow, target_wedge).
 *   index 0:LEFT  1:RIGHT  2:UP  3:DOWN
 *   row   0:N     1:E      2:S   3:W
 *
 * Arrow press moves the cursor "toward" the compass direction. If the
 * current triangle's apex already points that way and its base edge is
 * the boundary, jump to the matching triangle in the adjacent square
 * (apex flipped). Otherwise toggle to the matching triangle in the same
 * square.
 *   W + LEFT  → E in (col-1, row)        N + LEFT → W in same square
 *   N + UP    → S in (col, row-1)        E + UP   → N in same square
 */
static const int TETRA_DIR[4][4][3] = {
    /* LEFT  */ { {  0,  0, DIR_W }, {  0,  0, DIR_W }, {  0,  0, DIR_W }, { -1,  0, DIR_E } },
    /* RIGHT */ { {  0,  0, DIR_E }, { +1,  0, DIR_W }, {  0,  0, DIR_E }, {  0,  0, DIR_E } },
    /* UP    */ { {  0, -1, DIR_S }, {  0,  0, DIR_N }, {  0,  0, DIR_N }, {  0,  0, DIR_N } },
    /* DOWN  */ { {  0,  0, DIR_S }, {  0,  0, DIR_S }, {  0, +1, DIR_N }, {  0,  0, DIR_S } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->wedge = DIR_N;
    cur->glyph_idx = 0;
    cur->theme     = 0;
    cur->paused    = 0;
}

static void cursor_move(Cursor *cur, int arrow)
{
    const int *t = TETRA_DIR[arrow][cur->wedge];
    cur->col   += t[0];
    cur->row   += t[1];
    cur->wedge  = t[2];
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sc, sr;
    ctx_to_screen(g, cur->col, cur->row, cur->wedge, &sc, &sr);
    if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) return;
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, sc, '@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  mode — direct toggle (logic in main loop)                           */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Direct mode is the "identity" placement: SPACE toggles a single object
 * at the cursor's current (col, row, wedge). All mode logic lives in
 * the main loop's switch — there is nothing to abstract here. */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, const Pool *pool,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, DIR_NAME[cur->wedge],
             pool->count, GLYPHS[cur->glyph_idx], g->tri_size,
             cur->theme, fps, cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:toggle  g:glyph  C:clear  +/-:size  t:theme  r:reset  q:quit  [03 double diagonal direct] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, const Pool *pool,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(pool, g);
    cursor_draw(cur, g);
    hud_draw(g, cur, pool, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

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

    Cursor cur;  cursor_reset(&cur);
    Pool   pool; pool_clear(&pool);
    screen_init(cur.theme);

    GridCtx g;   ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS, g.tri_size);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r':
                    cursor_reset(&cur); pool_clear(&pool);
                    ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);
                    color_init(cur.theme);
                    break;
                case 'C': pool_clear(&pool); break;
                case 'g': cur.glyph_idx = (cur.glyph_idx + 1) % N_GLYPHS; break;
                case ' ':
                    pool_toggle(&pool, cur.col, cur.row, cur.wedge,
                                GLYPHS[cur.glyph_idx]);
                    break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, 0); break;
                case KEY_RIGHT: cursor_move(&cur, 1); break;
                case KEY_UP:    cursor_move(&cur, 2); break;
                case KEY_DOWN:  cursor_move(&cur, 3); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) g.tri_size += TRI_SIZE_STEP;
                    break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) g.tri_size -= TRI_SIZE_STEP;
                    break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA)
            + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &cur, &pool, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
