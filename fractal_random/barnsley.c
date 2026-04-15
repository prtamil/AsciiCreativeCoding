/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * barnsley.c — Barnsley IFS (Iterated Function System) fractal
 *
 * The chaos game: repeatedly apply a randomly chosen affine transform
 *   x' = a*x + b*y + e
 *   y' = c*x + d*y + f
 * accumulating hit counts on a density grid.  Log-normalised density
 * drives 4 brightness levels rendered as ASCII characters.
 *
 * Five built-in presets:
 *   0  Barnsley Fern       — the classic leaf-shaped fern
 *   1  Sierpinski          — Sierpinski triangle via IFS
 *   2  Levy C Curve        — recursive C-curve
 *   3  Dragon Curve        — Heighway dragon
 *   4  Fractal Tree        — symmetric branching tree
 *
 * Keys:
 *   q            quit
 *   p / space    pause / resume
 *   n            next preset
 *   N            prev preset
 *   t            next theme
 *   T            prev theme
 *   r            reset / clear grid
 *   +            increase iterations per frame
 *   -            decrease iterations per frame
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra barnsley.c -o barnsley -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  IFS presets
 *   §5  chaos game / grid
 *   §6  render
 *   §7  hud
 *   §8  app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Chaos game (IFS attractor via random iteration).
 *                  Rather than recursively subdividing regions, the attractor
 *                  is found by iterating: pick a random transform, apply it to
 *                  the current point, plot the result.  After discarding the
 *                  first few transient iterates (burn-in), the orbit is on
 *                  the attractor.  Due to Barnsley's theorem: any IFS with
 *                  a contractivity condition < 1 has a unique compact attractor.
 *
 * Math           : Each transform is an affine map T_i(x,y) = A_i·[x,y]ᵀ + b_i
 *                  where A_i is a 2×2 matrix.  The probability p_i of choosing
 *                  transform i should be proportional to |det(A_i)| for uniform
 *                  density across the attractor parts.  For the Barnsley fern,
 *                  stem (T₁, tiny |det|) needs only p₁=1% while main leaflets
 *                  (T₂, |det|≈0.85) need p₂=85%.
 *
 * Data-structure : Density grid: rather than drawing individual points,
 *                  accumulate hit counts per cell and log-normalise for display.
 *                  This allows millions of points per pixel with graceful saturation.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define GRID_ROWS_MAX  80
#define GRID_COLS_MAX  300

#define ITERS_MIN         1000
#define ITERS_MAX        30000
#define ITERS_DEFAULT     8000
#define ITERS_STEP        1000

#define HITS_CAP  60000u

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define FRAME_NS    (NS_PER_SEC / 30)

/* Number of presets and themes */
#define N_PRESETS   5
#define N_THEMES    5

/* LCG random — fast, no stdlib dependency per iteration */
static uint32_t g_lcg = 12345u;

static inline uint32_t lcg_next(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return g_lcg;
}

/* Returns uniform float in [0, 1) */
static inline float lcg_f(void)
{
    return (float)(lcg_next() >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Color pair IDs:
 *   CP_HUD = 1
 *   CP_L1  = 2   dim  (lowest density)
 *   CP_L2  = 3
 *   CP_L3  = 4
 *   CP_L4  = 5   brightest
 */
enum {
    CP_HUD = 1,
    CP_L1  = 2,
    CP_L2  = 3,
    CP_L3  = 4,
    CP_L4  = 5,
};

typedef struct {
    const char *name;
    int fg256[4];        /* L1..L4 in 256-color mode */
    int fg8[4];          /* L1..L4 in 8-color fallback */
    int hud256;
    int hud8;
} Theme;

static const Theme k_themes[N_THEMES] = {
    {
        "Fern",
        { 22, 34, 46, 154 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
        46, COLOR_GREEN,
    },
    {
        "Fire",
        { 124, 196, 208, 226 },
        { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW },
        208, COLOR_YELLOW,
    },
    {
        "Ice",
        { 17, 27, 51, 123 },
        { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN },
        51, COLOR_CYAN,
    },
    {
        "Plasma",
        { 54, 129, 201, 231 },
        { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN, COLOR_WHITE },
        201, COLOR_CYAN,
    },
    {
        "Mono",
        { 240, 245, 250, 255 },
        { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        250, COLOR_WHITE,
    },
};

static int g_theme = 0;

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(CP_HUD, th->hud256,    COLOR_BLACK);
        init_pair(CP_L1,  th->fg256[0],  COLOR_BLACK);
        init_pair(CP_L2,  th->fg256[1],  COLOR_BLACK);
        init_pair(CP_L3,  th->fg256[2],  COLOR_BLACK);
        init_pair(CP_L4,  th->fg256[3],  COLOR_BLACK);
    } else {
        init_pair(CP_HUD, th->hud8,    COLOR_BLACK);
        init_pair(CP_L1,  th->fg8[0],  COLOR_BLACK);
        init_pair(CP_L2,  th->fg8[1],  COLOR_BLACK);
        init_pair(CP_L3,  th->fg8[2],  COLOR_BLACK);
        init_pair(CP_L4,  th->fg8[3],  COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(g_theme);
}

/* ===================================================================== */
/* §4  IFS presets                                                        */
/* ===================================================================== */

/* One affine transform within an IFS */
typedef struct {
    float a, b, c, d, e, f;
    float cum;   /* cumulative probability for selection */
} Transform;

/* One preset: up to 4 transforms, view extents */
typedef struct {
    const char *name;
    int         n_transforms;
    Transform   tf[4];
    float       x_min, x_max;
    float       y_min, y_max;
} Preset;

static const Preset k_presets[N_PRESETS] = {
    {
        "Barnsley Fern",
        4,
        {
            { 0.00f,  0.00f,  0.00f,  0.16f,  0.00f,  0.00f,  0.01f },
            { 0.85f,  0.04f, -0.04f,  0.85f,  0.00f,  1.60f,  0.86f },
            { 0.20f, -0.26f,  0.23f,  0.22f,  0.00f,  1.60f,  0.93f },
            {-0.15f,  0.28f,  0.26f,  0.24f,  0.00f,  0.44f,  1.00f },
        },
        -2.6f, 2.6f,  -0.2f, 10.2f,
    },
    {
        "Sierpinski",
        3,
        {
            { 0.5f, 0.0f, 0.0f, 0.5f,  0.0f,  0.0f,  0.333f },
            { 0.5f, 0.0f, 0.0f, 0.5f,  1.0f,  0.0f,  0.667f },
            { 0.5f, 0.0f, 0.0f, 0.5f,  0.5f,  0.87f, 1.000f },
            { 0.0f, 0.0f, 0.0f, 0.0f,  0.0f,  0.0f,  1.000f },  /* unused */
        },
        -0.1f, 1.1f,  -0.1f, 1.1f,
    },
    {
        "Levy C Curve",
        2,
        {
            { 0.5f, -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  0.5f },
            { 0.5f,  0.5f, -0.5f,  0.5f,  0.5f,  0.5f,  1.0f },
            { 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  1.0f },  /* unused */
            { 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  1.0f },  /* unused */
        },
        -0.5f, 1.5f,  -0.5f, 1.5f,
    },
    {
        "Dragon Curve",
        2,
        {
            {  0.5f, -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  0.5f },
            { -0.5f,  0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  1.0f },
            {  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  1.0f },  /* unused */
            {  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  1.0f },  /* unused */
        },
        -0.2f, 1.3f,  -0.7f, 0.7f,
    },
    {
        "Fractal Tree",
        4,
        {
            { 0.00f,  0.00f,  0.00f,  0.50f,  0.00f,  0.00f,  0.05f },
            { 0.42f, -0.42f,  0.42f,  0.42f,  0.00f,  0.20f,  0.45f },
            { 0.42f,  0.42f, -0.42f,  0.42f,  0.00f,  0.20f,  0.85f },
            { 0.00f,  0.00f,  0.00f,  0.75f,  0.00f,  0.20f,  1.00f },
        },
        -1.1f, 1.1f,   0.0f, 2.0f,
    },
};

/* ===================================================================== */
/* §5  chaos game / grid                                                  */
/* ===================================================================== */

static uint16_t g_hits[GRID_ROWS_MAX][GRID_COLS_MAX];
static int      g_rows    = 24;
static int      g_cols    = 80;
static int      g_preset  = 0;
static int      g_iters   = ITERS_DEFAULT;

/* Current orbit position */
static float g_cx = 0.0f, g_cy = 0.0f;

static void grid_reset(void)
{
    memset(g_hits, 0, sizeof g_hits);
    g_cx = 0.1f;
    g_cy = 0.0f;
}

/*
 * chaos_step — run g_iters IFS iterations and accumulate into g_hits.
 * Uses LCG for transform selection.
 */
static void chaos_step(void)
{
    const Preset *p = &k_presets[g_preset];
    int draw_rows = g_rows - 1;   /* top row is HUD */
    if (draw_rows < 1) draw_rows = 1;

    float x_range = p->x_max - p->x_min;
    float y_range = p->y_max - p->y_min;
    if (x_range < 1e-6f) x_range = 1e-6f;
    if (y_range < 1e-6f) y_range = 1e-6f;

    float cx = g_cx, cy = g_cy;

    for (int i = 0; i < g_iters; i++) {
        float rnd = lcg_f();

        /* Select transform via cumulative probability */
        const Transform *tf = &p->tf[p->n_transforms - 1];
        for (int k = 0; k < p->n_transforms; k++) {
            if (rnd <= p->tf[k].cum) {
                tf = &p->tf[k];
                break;
            }
        }

        float nx = tf->a * cx + tf->b * cy + tf->e;
        float ny = tf->c * cx + tf->d * cy + tf->f;
        cx = nx;
        cy = ny;

        /* Map (cx, cy) to grid — row 0 is HUD, so content starts at row 1 */
        int col = (int)((cx - p->x_min) / x_range * (float)(g_cols - 1));
        int row = 1 + (int)((p->y_max - cy) / y_range * (float)(draw_rows - 1));

        if (col >= 0 && col < g_cols && row >= 1 && row < g_rows) {
            uint16_t h = g_hits[row][col];
            if (h < (uint16_t)HITS_CAP)
                g_hits[row][col] = h + 1u;
        }
    }

    g_cx = cx;
    g_cy = cy;
}

/* ===================================================================== */
/* §6  render                                                             */
/* ===================================================================== */

static void render_grid(void)
{
    /* Scan for max hits */
    uint32_t max_hits = 1;
    for (int r = 1; r < g_rows && r < GRID_ROWS_MAX; r++) {
        for (int c = 0; c < g_cols && c < GRID_COLS_MAX; c++) {
            if (g_hits[r][c] > max_hits)
                max_hits = g_hits[r][c];
        }
    }

    double log_max = log1p((double)max_hits);
    if (log_max < 1e-12) log_max = 1e-12;

    /*
     * Density tiers (normalised log density t ∈ [0, 1]):
     *   t < 0.15:          skip (background)
     *   0.15 ≤ t < 0.35:   L1  '.'
     *   0.35 ≤ t < 0.55:   L2  ':'
     *   0.55 ≤ t < 0.75:   L3  '+'
     *   0.75 ≤ t:          L4  '@'  (bold)
     */
    for (int row = 1; row < g_rows && row < GRID_ROWS_MAX; row++) {
        for (int col = 0; col < g_cols && col < GRID_COLS_MAX; col++) {
            uint16_t h = g_hits[row][col];
            if (h == 0) continue;

            double t = log1p((double)h) / log_max;
            if (t < 0.15) continue;

            int    cp;
            chtype ch;
            attr_t extra = 0;

            if (t < 0.35) {
                cp = CP_L1; ch = '.';
            } else if (t < 0.55) {
                cp = CP_L2; ch = ':';
            } else if (t < 0.75) {
                cp = CP_L3; ch = '+';
            } else {
                cp = CP_L4; ch = '@'; extra = A_BOLD;
            }

            attron(COLOR_PAIR(cp) | extra);
            mvaddch(row, col, ch);
            attroff(COLOR_PAIR(cp) | extra);
        }
    }
}

/* ===================================================================== */
/* §7  hud                                                                */
/* ===================================================================== */

static void draw_hud(double fps, bool paused)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
             " [%d/5] %-14s  theme:%-7s  spd:%5d  %5.1f fps  %s",
             g_preset + 1,
             k_presets[g_preset].name,
             k_themes[g_theme].name,
             g_iters,
             fps,
             paused ? "PAUSED" : "      ");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT  || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)                g_resize = 1;
}

static void cleanup(void) { endwin(); }

static void do_resize(void)
{
    endwin();
    refresh();
    getmaxyx(stdscr, g_rows, g_cols);
    if (g_rows > GRID_ROWS_MAX) g_rows = GRID_ROWS_MAX;
    if (g_cols > GRID_COLS_MAX) g_cols = GRID_COLS_MAX;
    grid_reset();
    g_resize = 0;
}

int main(void)
{
    /* Seed LCG from monotonic clock */
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        g_lcg = (uint32_t)(ts.tv_nsec ^ (ts.tv_sec * 123456789LL));
        if (g_lcg == 0) g_lcg = 1u;
    }

    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();

    getmaxyx(stdscr, g_rows, g_cols);
    if (g_rows > GRID_ROWS_MAX) g_rows = GRID_ROWS_MAX;
    if (g_cols > GRID_COLS_MAX) g_cols = GRID_COLS_MAX;
    grid_reset();

    bool      paused      = false;
    long long fps_accum   = 0;
    int       fps_count   = 0;
    double    fps_display = 0.0;

    while (!g_quit) {

        if (g_resize) do_resize();

        long long frame_start = clock_ns();

        /* Input */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27:
                g_quit = 1;
                break;
            case 'p': case ' ':
                paused = !paused;
                break;
            case 'n':
                g_preset = (g_preset + 1) % N_PRESETS;
                grid_reset();
                break;
            case 'N':
                g_preset = (g_preset - 1 + N_PRESETS) % N_PRESETS;
                grid_reset();
                break;
            case 't':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply(g_theme);
                break;
            case 'T':
                g_theme = (g_theme - 1 + N_THEMES) % N_THEMES;
                theme_apply(g_theme);
                break;
            case 'r': case 'R':
                grid_reset();
                break;
            case '+': case '=':
                g_iters += ITERS_STEP;
                if (g_iters > ITERS_MAX) g_iters = ITERS_MAX;
                break;
            case '-':
                g_iters -= ITERS_STEP;
                if (g_iters < ITERS_MIN) g_iters = ITERS_MIN;
                break;
            default:
                break;
            }
        }

        /* Simulation */
        if (!paused)
            chaos_step();

        /* Draw */
        erase();
        render_grid();
        draw_hud(fps_display, paused);
        wnoutrefresh(stdscr);
        doupdate();

        /* FPS accounting */
        long long frame_end = clock_ns();
        long long frame_dur = frame_end - frame_start;
        fps_accum += frame_dur;
        fps_count++;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)fps_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_accum = 0;
            fps_count = 0;
        }

        clock_sleep_ns(FRAME_NS - frame_dur);
    }

    return 0;
}
