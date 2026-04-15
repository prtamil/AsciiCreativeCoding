/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sandpile.c — Bak-Tang-Wiesenfeld Abelian Sandpile
 *
 * Grains are dropped one at a time at the centre of the grid.
 * Any cell with ≥ 4 grains topples: it loses 4 grains and each of its four
 * cardinal neighbours gains 1.  Grains that fall off the border are lost.
 * Toppling cascades until every cell holds 0–3 grains (the stable state).
 *
 * The emergent pattern exhibits self-organised criticality:
 *   — avalanche sizes follow a power-law (1/f) distribution
 *   — most grain drops trigger tiny, local topplings
 *   — occasionally a single grain tips a cascade across the entire pile
 *   — the long-run stable pattern has intricate fractal / quasi-crystalline
 *     symmetry that was not designed in — it arises purely from the rule
 *
 * Colour by grain count (stable state):
 *   0 grains — blank              (background)
 *   1 grain  — dim blue      '.'  (sparse)
 *   2 grains — green         '+'  (medium)
 *   3 grains — bright gold   '#'  (near-critical)
 *
 * Avalanche cells (≥ 4) are shown in bright red during the animated
 * avalanche mode ('v' key) so you can watch the wave propagate.
 *
 * Keys:
 *   r         reset (clear grid, restart drop counter)
 *   space     pause / resume
 *   s         cycle view: split → top → side
 *   v         toggle avalanche visualisation (slow per-step vs instant)
 *   + / =     more drops per frame
 *   - / _     fewer drops per frame
 *   q / Q     quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fractal_random/sandpile.c -o sandpile -lncurses
 *
 * Sections: §1 config  §2 clock  §3 color  §4 grid  §5 sim
 *           §6 scene   §7 screen §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Abelian Sandpile Model (Bak, Tang & Wiesenfeld, 1987).
 *                  A grain is dropped on a chosen cell.  If any cell has
 *                  ≥ 4 grains, it "topples": loses 4 grains and gives 1 to
 *                  each of its 4 (von Neumann) neighbours.  Toppling can
 *                  cascade into an "avalanche" affecting many cells.
 *
 * Physics/Math   : The sandpile is a canonical example of Self-Organised
 *                  Criticality (SOC): without any tuning, the system
 *                  spontaneously evolves to a critical state where avalanche
 *                  size distributions follow a power law P(s) ∝ s^(−τ) with
 *                  τ ≈ 1.0 for 2D abelian sandpile.
 *                  "Abelian" means the final state is independent of the order
 *                  in which topplings are processed — a deep mathematical
 *                  property proved by Dhar (1990).
 *
 * Performance    : O(avalanche_size) per grain drop.  Average avalanche size
 *                  grows as the pile approaches the critical state.  At
 *                  criticality, large avalanches are rare but unbounded —
 *                  the expected cost per drop diverges logarithmically.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config ──────────────────────────────────────────────────────────── */

#define TICK_NS       33333333LL   /* ~30 fps                               */
#define MAX_ROWS      128
#define MAX_COLS      320
#define QMAX          (MAX_ROWS * MAX_COLS + 1)  /* avalanche queue size   */

#define DROPS_DEF     10           /* grain drops per frame (default)       */
#define DROPS_MAX     500          /* maximum drops per frame               */

/* characters for each grain count */
static const char GRAIN_CH[4] = { ' ', '.', '+', '#' };

enum { CP_G1=1, CP_G2, CP_G3, CP_TOPPLE, CP_HUD };

/* view modes */
#define VIEW_SPLIT 0   /* top + side panels (default) */
#define VIEW_TOP   1   /* top-down only               */
#define VIEW_SIDE  2   /* side histogram only         */
#define N_VIEWS    3
static int g_view = VIEW_SPLIT;

/* ── themes ──────────────────────────────────────────────────────────────── */

#define N_THEMES 10

/*
 * Theme colors for 4 content pairs + HUD:
 *   c[0] = CP_G1   (1 grain  — sparse)
 *   c[1] = CP_G2   (2 grains — medium)
 *   c[2] = CP_G3   (3 grains — near-critical, bright)
 *   c[3] = CP_TOPPLE (toppling ≥4 — hottest)
 *   c[4] = CP_HUD
 */
typedef struct {
    const char *name;
    int c[5];    /* 256-color */
    int c8[5];   /* 8-color fallback */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Electric",
      {  39, 51, 226, 201,  87 },
      { COLOR_CYAN,   COLOR_CYAN,   COLOR_YELLOW,  COLOR_MAGENTA, COLOR_CYAN   } },
    { "Matrix",
      {  28, 46, 118,  82,  46 },
      { COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN  } },
    { "Nova",
      {  21, 39, 117, 231,  51 },
      { COLOR_BLUE,   COLOR_CYAN,   COLOR_CYAN,    COLOR_WHITE,   COLOR_CYAN   } },
    { "Poison",
      { 100,148, 190,  82, 154 },
      { COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW,  COLOR_GREEN,   COLOR_YELLOW } },
    { "Ocean",
      {  24, 38,  45, 159,  39 },
      { COLOR_BLUE,   COLOR_CYAN,   COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN   } },
    { "Fire",
      { 196,208, 226, 231, 208 },
      { COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW,  COLOR_WHITE,   COLOR_YELLOW } },
    { "Gold",
      { 136,178, 220, 231, 226 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,  COLOR_WHITE,   COLOR_YELLOW } },
    { "Ice",
      {  30, 45, 159, 231, 123 },
      { COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE,   COLOR_WHITE,   COLOR_CYAN   } },
    { "Nebula",
      {  93,141, 183, 231,  87 },
      { COLOR_MAGENTA,COLOR_CYAN,   COLOR_WHITE,   COLOR_WHITE,   COLOR_CYAN   } },
    { "Lava",
      { 124,196, 214, 226, 214 },
      { COLOR_RED,    COLOR_RED,    COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW } },
};

static int g_theme = 0;

/* ── §2 clock ───────────────────────────────────────────────────────────── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 color ───────────────────────────────────────────────────────────── */

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(CP_G1,     th->c[0], -1);
        init_pair(CP_G2,     th->c[1], -1);
        init_pair(CP_G3,     th->c[2], -1);
        init_pair(CP_TOPPLE, th->c[3], -1);
        init_pair(CP_HUD,    th->c[4], -1);
    } else {
        init_pair(CP_G1,     th->c8[0], -1);
        init_pair(CP_G2,     th->c8[1], -1);
        init_pair(CP_G3,     th->c8[2], -1);
        init_pair(CP_TOPPLE, th->c8[3], -1);
        init_pair(CP_HUD,    th->c8[4], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(g_theme);
}

/* ── §4 grid ────────────────────────────────────────────────────────────── */

static uint8_t g_grid[MAX_ROWS][MAX_COLS];
static int     g_rows, g_cols, g_ca_rows;
static int     g_drops, g_paused, g_vis;  /* g_vis: show avalanche steps   */
static long long g_total_drops;
static long      g_last_avalanche;

/* avalanche BFS queue */
static int     g_qr   [QMAX];
static int     g_qc   [QMAX];
static uint8_t g_inq  [MAX_ROWS][MAX_COLS];
static int     g_qhead, g_qtail;

/* drop-point (centre) */
static int g_cr, g_cc;

static void grid_clear(void)
{
    memset(g_grid, 0, sizeof(g_grid));
    g_total_drops   = 0;
    g_last_avalanche = 0;
    g_cr = g_ca_rows / 2;
    g_cc = g_cols    / 2;
}

/* ── §5 simulation ──────────────────────────────────────────────────────── */

static void enq(int r, int c)
{
    if (g_inq[r][c]) return;
    g_inq[r][c] = 1;
    g_qr[g_qtail] = r;
    g_qc[g_qtail] = c;
    g_qtail = (g_qtail + 1) % QMAX;
}

/* Run the avalanche to completion.
 * Returns the number of topple events.
 * If g_vis is set, returns after one "wave" (one pass through the current
 * queue front) so the caller can render between waves.                       */
static long avalanche_step(int full)
{
    long topples = 0;

    while (g_qhead != g_qtail) {
        int r = g_qr[g_qhead];
        int c = g_qc[g_qhead];
        g_qhead = (g_qhead + 1) % QMAX;
        g_inq[r][c] = 0;

        if (g_grid[r][c] < 4) continue;  /* stabilised since enqueue        */

        g_grid[r][c] -= 4;
        topples++;

        static const int DR[4] = {-1,  0, 1, 0};
        static const int DC[4] = { 0,  1, 0,-1};
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (nr >= 0 && nr < g_ca_rows && nc >= 0 && nc < g_cols) {
                g_grid[nr][nc]++;
                if (g_grid[nr][nc] >= 4) enq(nr, nc);
            }
        }
        if (g_grid[r][c] >= 4) enq(r, c);   /* still unstable: re-enqueue  */

        if (!full) break;   /* one topple per visual frame                  */
    }
    return topples;
}

static int avalanche_pending(void) { return g_qhead != g_qtail; }

static void drop_grain(void)
{
    g_grid[g_cr][g_cc]++;
    g_total_drops++;
    if (g_grid[g_cr][g_cc] >= 4) {
        memset(g_inq, 0, sizeof(g_inq));
        g_qhead = g_qtail = 0;
        enq(g_cr, g_cc);
    }
}

static void sim_tick(void)
{
    if (g_paused) return;

    if (g_vis) {
        /* animated mode: one topple per frame, drop new grain when stable   */
        if (avalanche_pending()) {
            g_last_avalanche += avalanche_step(0);
        } else {
            drop_grain();
            g_last_avalanche = 0;
        }
    } else {
        /* instant mode: drop g_drops grains, run each to full stability     */
        for (int i = 0; i < g_drops; i++) {
            drop_grain();
            if (avalanche_pending())
                g_last_avalanche = avalanche_step(1);
        }
    }
}

/* ── §6 scene ───────────────────────────────────────────────────────────── */

/*
 * scene_top() — top-down view of the grain grid.
 *
 * Draws g_ca_rows rows of the grid into screen rows [y0, y0+h).
 * The view is centred on the drop point (g_cr) so the pile is always
 * visible even when h < g_ca_rows (split mode).
 */
static void scene_top(int y0, int h)
{
    if (h <= 0) return;

    /* centre the visible window on the drop row */
    int start_r = g_cr - h / 2;
    if (start_r < 0)               start_r = 0;
    if (start_r + h > g_ca_rows)   start_r = g_ca_rows - h;
    if (start_r < 0)               start_r = 0;

    for (int dy = 0; dy < h; dy++) {
        int r = start_r + dy;
        if (r >= g_ca_rows) break;
        for (int c = 0; c < g_cols - 1; c++) {
            int g = g_grid[r][c];
            if (g == 0) { mvaddch(y0 + dy, c, ' '); continue; }
            int cp; char ch; attr_t bold = 0;
            if (g >= 4) { cp = CP_TOPPLE; ch = '*'; bold = A_BOLD; }
            else { cp = CP_G1 + g - 1; ch = GRAIN_CH[g]; bold = (g == 3) ? A_BOLD : 0; }
            attron(COLOR_PAIR(cp) | bold);
            mvaddch(y0 + dy, c, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(cp) | bold);
        }
    }

    /* drop-point marker */
    int drop_dy = g_cr - start_r;
    if (drop_dy >= 0 && drop_dy < h && g_grid[g_cr][g_cc] == 0) {
        attron(COLOR_PAIR(CP_TOPPLE) | A_BOLD);
        mvaddch(y0 + drop_dy, g_cc, (chtype)(unsigned char)'@');
        attroff(COLOR_PAIR(CP_TOPPLE) | A_BOLD);
    }
}

/*
 * scene_side() — side-profile histogram.
 *
 * For each terminal column c, sums all grains in that column across every
 * grid row.  The total is drawn as a vertical bar rising from the bottom of
 * the panel [y0, y0+h), scaled so the tallest column fills the panel.
 *
 * Color gradient (bottom → top of bar):
 *   lower third  → blue  (CP_G1) — sparse base
 *   middle third → green (CP_G2) — body
 *   upper third  → gold  (CP_G3) — peak
 *
 * The resulting shape is a bell curve / diamond — the sandpile's silhouette
 * viewed from the right.
 */
static void scene_side(int y0, int h)
{
    if (h <= 1) return;

    /*
     * Fixed scale: each cell holds 0-3 grains stably, so the theoretical
     * maximum per column is g_ca_rows * 3.  Using a fixed scale means bars
     * physically grow upward as grains accumulate — the centre column rises
     * first, then neighbours fill in, producing the sandpile's bell-curve
     * silhouette growing from the bottom.
     *
     * Dynamic normalisation (old approach) made every frame look the same
     * because the relative shape barely changes between frames.
     */
    long max_possible = (long)g_ca_rows * 3;
    if (max_possible < 1) max_possible = 1;

    for (int c = 0; c < g_cols - 1; c++) {
        long col_sum = 0;
        for (int r = 0; r < g_ca_rows; r++)
            col_sum += g_grid[r][c];

        int bar_h = (int)(col_sum * h / max_possible);
        if (bar_h > h) bar_h = h;

        for (int dy = 0; dy < h; dy++) {
            int screen_row = y0 + h - 1 - dy;   /* dy=0 → bottom of panel */
            if (dy < bar_h) {
                float frac = (float)dy / (float)(h > 1 ? h - 1 : 1);
                int    cp   = (frac < 0.33f) ? CP_G1 :
                              (frac < 0.66f) ? CP_G2 : CP_G3;
                attr_t bold = (frac >= 0.66f) ? A_BOLD : 0;
                attron(COLOR_PAIR(cp) | bold);
                mvaddch(screen_row, c, '|');
                attroff(COLOR_PAIR(cp) | bold);
            } else {
                mvaddch(screen_row, c, ' ');
            }
        }
    }
}

/*
 * scene_render() — dispatch to the correct view(s).
 *
 *   VIEW_TOP  : full-screen top-down grid
 *   VIEW_SIDE : full-screen side histogram
 *   VIEW_SPLIT: top 60 % top-down  |  separator  |  bottom 40 % histogram
 */
static const char *k_view_names[] = { "SPLIT", "TOP", "SIDE" };

static void scene_render(void)
{
    int draw_h = g_ca_rows;

    switch (g_view) {
    case VIEW_TOP:
        scene_top(0, draw_h);
        break;

    case VIEW_SIDE:
        scene_side(0, draw_h);
        break;

    case VIEW_SPLIT: {
        int top_h  = draw_h * 3 / 5;
        if (top_h  < 2) top_h = 2;
        int sep    = top_h;
        int side_y = sep + 1;
        int side_h = draw_h - side_y;

        scene_top(0, top_h);

        /* separator bar */
        attron(COLOR_PAIR(CP_HUD));
        for (int c = 0; c < g_cols; c++) mvaddch(sep, c, '-');
        mvprintw(sep, 2, "[ TOP ^ ]--[ SIDE v ]");
        attroff(COLOR_PAIR(CP_HUD));

        scene_side(side_y, side_h);
        break;
    }
    }
}

static void scene_hud(void)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(g_rows - 1, 0,
             " grains=%-8lld  avl=%-7ld  rate=%d/f  [%s]  t:%-8s  %s  %s"
             "  r:reset  +/-:rate  s:view  v:vis  spc:pause  q:quit",
             (long long)g_total_drops, g_last_avalanche, g_drops,
             k_view_names[g_view],
             k_themes[g_theme].name,
             g_vis    ? "VIS " : "    ",
             g_paused ? "PAUSED" : "      ");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §7 screen ──────────────────────────────────────────────────────────── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

static void screen_resize(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    g_rows    = rows;
    g_cols    = cols;
    g_ca_rows = rows - 1;   /* full screen minus HUD row                    */
    if (g_ca_rows < 1) g_ca_rows = 1;
    grid_clear();
    erase();
}

/* ── §8 app ─────────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);

    screen_init();
    g_drops  = DROPS_DEF;
    g_paused = 0;
    g_vis    = 0;
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));
    screen_resize();

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize();
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': g_running = 0;               break;
            case ' ':           g_paused ^= 1;                break;
            case 'r':           grid_clear(); erase();        break;
            case 's': case 'S': g_view = (g_view + 1) % N_VIEWS; break;
            case 't': case 'T':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply(g_theme);
                break;
            case 'v': case 'V': g_vis ^= 1;                  break;
            case '+': case '=':
                if (g_drops < DROPS_MAX) g_drops++;
                break;
            case '-': case '_':
                if (g_drops > 1) g_drops--;
                break;
            }
        }

        sim_tick();
        erase();
        scene_render();
        scene_hud();
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
