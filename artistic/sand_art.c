/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sand_art.c -- Hourglass sand art with flippable gravity
 *
 * A proper hourglass shape is drawn centred on the terminal.
 * Five layers of coloured sand fill the top bulb and fall through the
 * narrow neck into the bottom bulb.  Press SPACE or F to flip gravity --
 * the sand avalanches back upward through the neck and a new landscape
 * forms.  Press R to pour fresh sand layers into the top bulb at any time.
 *
 * Physics: falling-sand cellular automaton.
 * Each tick every grain tries to move one cell toward gravity; if blocked
 * it tries the two diagonal-forward cells in random order.  The scan sweeps
 * away from gravity so a moved grain is never processed twice per frame.
 *
 * Container: a centred hourglass whose walls taper linearly from full
 * width at top/bottom to a narrow neck at the centre row.  Everything
 * outside the hourglass is terminal background -- only the two bulbs and
 * the neck are accessible to sand.
 *
 * Keys:
 *   SPACE / f     flip gravity (DOWN <--> UP)
 *   r             refill top bulb with fresh coloured layers
 *   t             cycle theme (5 themes)
 *   p             pause / resume
 *   q / ESC       quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/sand_art.c \
 *       -o sand_art -lncurses
 *
 * Sections:
 *   [1] config  [2] clock  [3] color  [4] hourglass
 *   [5] sand    [6] scene  [7] app
 */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* [1] config                                                             */
/* ===================================================================== */

#define RENDER_NS       (1000000000LL / 30)
#define ROWS_MAX        80
#define COLS_MAX        300

/* Cell types */
#define CELL_VOID       0   /* outside hourglass -- never touched          */
#define CELL_WALL       1   /* hourglass outline -- blocks sand            */
#define CELL_EMPTY      2   /* interior empty -- sand may move here        */
#define SAND_FIRST      3   /* sand colours: SAND_FIRST .. SAND_FIRST+N_SAND-1 */
#define N_SAND          5

/* Gravity: only vertical */
typedef enum { G_DOWN = 0, G_UP = 1 } GravDir;
static const int k_gdr[2] = {  1, -1 };   /* row delta per gravity     */

/* Color pair IDs */
enum {
    CP_WALL = 1,
    CP_S1, CP_S2, CP_S3, CP_S4, CP_S5,
    CP_HUD,
    N_CP
};

#define N_THEMES 5

typedef struct {
    const char *name;
    int wall,   wall8;
    int sand [N_SAND];
    int sand8[N_SAND];
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Desert",
       94, COLOR_YELLOW,
      { 214, 178, 136, 172, 231 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED, COLOR_WHITE } },

    { "Ocean",
       24, COLOR_BLUE,
      {  51,  45,  39, 117, 231 },
      { COLOR_CYAN, COLOR_CYAN, COLOR_BLUE, COLOR_BLUE, COLOR_WHITE } },

    { "Forest",
       22, COLOR_GREEN,
      {  46, 118, 190, 226, 231 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_YELLOW, COLOR_WHITE } },

    { "Fire",
       52, COLOR_RED,
      { 196, 202, 208, 220, 231 },
      { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE } },

    { "Neon",
      240, COLOR_WHITE,
      { 201,  51,  93,  46, 226 },
      { COLOR_MAGENTA, COLOR_CYAN, COLOR_MAGENTA, COLOR_GREEN, COLOR_YELLOW } },
};

static int g_theme = 0;

/* ===================================================================== */
/* [2] clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* [3] color                                                              */
/* ===================================================================== */

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(CP_WALL, th->wall, COLOR_BLACK);
        for (int i = 0; i < N_SAND; i++)
            init_pair(CP_S1 + i, th->sand[i], -1);
        init_pair(CP_HUD, th->sand[0], -1);
    } else {
        init_pair(CP_WALL, th->wall8, COLOR_BLACK);
        for (int i = 0; i < N_SAND; i++)
            init_pair(CP_S1 + i, th->sand8[i], -1);
        init_pair(CP_HUD, th->sand8[0], -1);
    }
}
static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(g_theme);
}

/* ===================================================================== */
/* [4] hourglass                                                          */
/* ===================================================================== */

static uint8_t g_grid[ROWS_MAX][COLS_MAX];
static int     g_rows, g_cols;

/* Hourglass geometry -- computed once per resize */
static int g_hg_top;      /* first row of hourglass                      */
static int g_hg_bot;      /* last row of hourglass (excl HUD)            */
static int g_hg_mid;      /* centre row (neck)                           */
static int g_hg_cx;       /* centre column                               */
static int g_hg_hw_max;   /* half-width at top / bottom                  */
static int g_hg_neck;     /* half-width at neck                          */

static void hourglass_params(void)
{
    int usable   = g_rows - 1;            /* rows excluding HUD           */
    int hh       = usable * 88 / 100;     /* hourglass height (88%)       */
    int margin   = (usable - hh) / 2;

    g_hg_top    = margin;
    g_hg_bot    = margin + hh - 1;
    g_hg_mid    = (g_hg_top + g_hg_bot) / 2;
    g_hg_cx     = g_cols / 2;
    g_hg_hw_max = g_cols * 33 / 100;     /* 33% of width on each side    */
    g_hg_neck   = g_hg_hw_max / 7;
    if (g_hg_neck < 3) g_hg_neck = 3;
}

/*
 * Half-width of the hourglass interior at row r.
 * Linearly tapers from g_hg_hw_max at top/bottom to g_hg_neck at mid.
 */
static int hg_hw(int r)
{
    int dist   = abs(r - g_hg_mid);
    int span   = g_hg_mid - g_hg_top;
    if (span <= 0) span = 1;
    int hw = g_hg_neck + (g_hg_hw_max - g_hg_neck) * dist / span;
    return hw;
}

/*
 * Wall character based on position in the hourglass outline.
 *
 * Top / bottom rows:  '-'
 * Upper-left wall:    '\\'  (tapers inward going down)
 * Upper-right wall:   '/'
 * Lower-left wall:    '/'   (opens outward going down)
 * Lower-right wall:   '\\'
 */
static chtype wall_char(int r, int c)
{
    if (r == g_hg_top || r == g_hg_bot)
        return (chtype)'-';

    bool left     = (c < g_hg_cx);
    bool top_half = (r < g_hg_mid);

    if ( top_half &&  left) return (chtype)'\\';
    if ( top_half && !left) return (chtype)'/';
    if (!top_half &&  left) return (chtype)'/';
    return (chtype)'\\';
}

/*
 * (Re)build the grid:
 *  - CELL_VOID  everywhere outside the hourglass
 *  - CELL_WALL  along the hourglass outline
 *  - CELL_EMPTY in the interior of each row
 */
static void grid_build(void)
{
    hourglass_params();
    memset(g_grid, CELL_VOID, sizeof g_grid);

    for (int r = g_hg_top; r <= g_hg_bot; r++) {
        int hw = hg_hw(r);
        int lo = g_hg_cx - hw;
        int hi = g_hg_cx + hw;
        if (lo < 0)         lo = 0;
        if (hi >= g_cols)   hi = g_cols - 1;

        if (r == g_hg_top || r == g_hg_bot) {
            /* Horizontal bars */
            for (int c = lo; c <= hi; c++)
                g_grid[r][c] = CELL_WALL;
        } else {
            /* Side walls + interior */
            g_grid[r][lo] = CELL_WALL;
            g_grid[r][hi] = CELL_WALL;
            for (int c = lo + 1; c < hi; c++)
                g_grid[r][c] = CELL_EMPTY;
        }
    }
}

/* Texture character for a sand grain based on local density */
static chtype sand_char(int r, int c)
{
    int n = 0;
    for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++) {
            if (!dy && !dx) continue;
            int nr = r + dy, nc = c + dx;
            if (nr < 0 || nr >= g_rows || nc < 0 || nc >= g_cols) continue;
            if (g_grid[nr][nc] >= SAND_FIRST) n++;
        }
    if (n >= 6) return (chtype)'#';
    if (n >= 4) return (chtype)'o';
    if (n >= 2) return (chtype)'+';
    return (chtype)'.';
}

/* ===================================================================== */
/* [5] sand                                                               */
/* ===================================================================== */

static GravDir g_grav   = G_DOWN;
static bool    g_paused = false;

/*
 * Clear all sand, then fill the upper bulb with N_SAND horizontal
 * colour bands (80 % of the bulb height so grains start falling
 * immediately on the first frame).
 */
static void sand_fill(void)
{
    /* Clear existing sand */
    for (int r = 0; r < g_rows; r++)
        for (int c = 0; c < g_cols; c++)
            if (g_grid[r][c] >= SAND_FIRST)
                g_grid[r][c] = CELL_EMPTY;

    /* Fill the top bulb (rows g_hg_top+1 .. g_hg_mid-1) */
    int fill_top = g_hg_top + 1;
    int fill_bot = g_hg_mid - 1;
    int fill_h   = fill_bot - fill_top + 1;
    int use_h    = fill_h * 80 / 100;   /* 80% of the top bulb          */
    if (use_h < 1) use_h = 1;

    for (int r = fill_top; r < fill_top + use_h; r++) {
        int band  = (r - fill_top) * N_SAND / use_h;
        if (band >= N_SAND) band = N_SAND - 1;
        uint8_t color = (uint8_t)(SAND_FIRST + band);

        int hw = hg_hw(r);
        int lo = g_hg_cx - hw + 1;
        int hi = g_hg_cx + hw - 1;
        for (int c = lo; c <= hi; c++)
            if (g_grid[r][c] == CELL_EMPTY)
                g_grid[r][c] = color;
    }
}

/*
 * Try to move the grain at (r, c) one step toward gravity.
 *
 * The grain can only enter a cell that is CELL_EMPTY (not CELL_VOID or
 * CELL_WALL).  Diagonals are tried in random order to remove left/right
 * bias.
 */
static void grain_step(int r, int c, int gdr)
{
    uint8_t type = g_grid[r][c];
    if (type < SAND_FIRST) return;

    /* Primary: straight toward gravity */
    int nr = r + gdr;
    if (nr >= 0 && nr < g_rows && g_grid[nr][c] == CELL_EMPTY) {
        g_grid[nr][c] = type;
        g_grid[r][c]  = CELL_EMPTY;
        return;
    }

    /* Diagonals: toward gravity + left/right, random order */
    int da = -1, db = 1;
    if (rand() & 1) { da = 1; db = -1; }

    int na, nb;
    na = c + da;
    if (nr >= 0 && nr < g_rows && na >= 0 && na < g_cols
            && g_grid[nr][na] == CELL_EMPTY) {
        g_grid[nr][na] = type;
        g_grid[r][c]   = CELL_EMPTY;
        return;
    }
    nb = c + db;
    if (nr >= 0 && nr < g_rows && nb >= 0 && nb < g_cols
            && g_grid[nr][nb] == CELL_EMPTY) {
        g_grid[nr][nb] = type;
        g_grid[r][c]   = CELL_EMPTY;
    }
}

/*
 * One CA tick.  Scan away from gravity so a moved grain is not processed
 * twice: DOWN → scan bottom-to-top; UP → scan top-to-bottom.
 */
static void sand_tick(void)
{
    int gdr = k_gdr[g_grav];

    if (g_grav == G_DOWN) {
        for (int r = g_hg_bot - 1; r > g_hg_top; r--)
            for (int c = g_hg_cx - g_hg_hw_max; c <= g_hg_cx + g_hg_hw_max; c++)
                grain_step(r, c, gdr);
    } else {
        for (int r = g_hg_top + 1; r < g_hg_bot; r++)
            for (int c = g_hg_cx - g_hg_hw_max; c <= g_hg_cx + g_hg_hw_max; c++)
                grain_step(r, c, gdr);
    }
}

/* ===================================================================== */
/* [6] scene                                                              */
/* ===================================================================== */

static void scene_draw(void)
{
    for (int r = g_hg_top; r <= g_hg_bot; r++) {
        int hw = hg_hw(r);
        int lo = g_hg_cx - hw;
        int hi = g_hg_cx + hw;

        for (int c = lo; c <= hi; c++) {
            uint8_t cell = g_grid[r][c];

            if (cell == CELL_WALL) {
                attron(COLOR_PAIR(CP_WALL) | A_BOLD);
                mvaddch(r, c, wall_char(r, c));
                attroff(COLOR_PAIR(CP_WALL) | A_BOLD);
            } else if (cell >= SAND_FIRST) {
                int  cp   = CP_S1 + (int)(cell - SAND_FIRST);
                chtype ch = sand_char(r, c);
                attr_t at = COLOR_PAIR(cp);
                if (ch == '.' || ch == '+') at |= A_BOLD;
                attron(at);
                mvaddch(r, c, ch);
                attroff(at);
            }
            /* CELL_EMPTY / CELL_VOID: draw nothing (terminal background) */
        }
    }

    /* HUD */
    const char *grav_str = (g_grav == G_DOWN) ? "DOWN [v]" : "UP   [^]";
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    for (int c = 0; c < g_cols; c++) mvaddch(g_rows - 1, c, ' ');
    mvprintw(g_rows - 1, 1,
        "SandArt  q:quit  spc/f:flip-gravity:%s  r:refill"
        "  t:%-7s  p:pause",
        grav_str, k_themes[g_theme].name);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* [7] app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();

    getmaxyx(stdscr, g_rows, g_cols);
    grid_build();
    sand_fill();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            grid_build();
            sand_fill();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27:
            g_quit = 1; break;
        case ' ': case 'f': case 'F':
            g_grav = (g_grav == G_DOWN) ? G_UP : G_DOWN; break;
        case 'r': case 'R':
            sand_fill(); break;
        case 'p': case 'P':
            g_paused = !g_paused; break;
        case 't': case 'T':
            g_theme = (g_theme + 1) % N_THEMES;
            theme_apply(g_theme);
            break;
        default: break;
        }

        long long now = clock_ns();
        if (!g_paused) sand_tick();
        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
