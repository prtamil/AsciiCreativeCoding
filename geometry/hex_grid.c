/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hex_grid.c — hexagonal character grid
 *
 * Characters are placed at the centres of a hexagonal tiling.
 * Offset-row layout: odd rows are shifted right by half a hex width.
 * Colour is assigned by ring distance from the centre hex, creating
 * concentric colour bands that radiate outward like a target.
 * Each character changes at an independent random rate.
 *
 * Themes (t): fire · matrix · ice · plasma · gold · mono
 * Speed  (s): slow · medium · fast · frenzy
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grid/hex_grid.c -o hex_grid -lncurses -lm
 *
 * Keys:
 *   t     cycle theme
 *   s     cycle speed
 *   p     pause / resume
 *   q / ESC  quit
 */

#define _POSIX_C_SOURCE 199309L
#include <ncurses.h>
#include <stdlib.h>
#include <time.h>
#include <signal.h>

/* ── §1  config ─────────────────────────────────────────────────────────── */

#define TARGET_FPS      60

/*
 * Hex layout (pointy-top offset rows):
 *   HEX_DX = column advance per hex (in terminal columns)
 *   HEX_DY = row advance per hex (in terminal rows)
 *   Odd rows shift right by HEX_DX/2 columns.
 *
 * Terminal cells are ~2.1× taller than wide.  For the tiling to look like
 * regular hexagons in physical pixels:
 *   physical hex height / physical hex width ≈ √3 / 2 ≈ 0.866
 *   hex_dy_px / hex_dx_px = 0.866
 *   (HEX_DY * cell_h) / (HEX_DX * cell_w) = 0.866
 *   HEX_DY / HEX_DX = 0.866 * cell_w/cell_h = 0.866 * 0.47 ≈ 0.41
 *
 * With HEX_DX=4, HEX_DY=2: ratio = 0.5 — close enough, hexes look round.
 */
#define HEX_DX          4      /* column step per hex                        */
#define HEX_DY          2      /* row step per hex                           */
#define HEX_MAX_COLS   64      /* max hex columns                            */
#define HEX_MAX_ROWS   38      /* max hex rows                               */
#define N_BAND_COLORS   8      /* colour bands (by distance from centre)     */
#define N_THEMES        6
#define N_SPEEDS        4
#define CH_FIRST       0x21
#define CH_RANGE        94

static const float  g_speeds[N_SPEEDS]      = { 2.f, 8.f, 24.f, 60.f };
static const char  *g_speed_names[N_SPEEDS] = { "slow","medium","fast","frenzy" };

/*
 * 8-band ramps — index 0 = centre (brightest), 7 = outermost ring (dimmest).
 * Colours radiate outward like a bullseye.
 */
static const int g_pal[N_THEMES][N_BAND_COLORS] = {
    { 231,226,220,214,208,202,196,160 },  /* fire    */
    {  82, 46, 40, 34, 28, 22,  2,  0 },  /* matrix  */
    { 159, 51, 45, 33, 27, 21, 19, 17 },  /* ice     */
    { 207,201,165,129, 93, 57, 56, 53 },  /* plasma  */
    { 220,184,178,172,136,130, 94, 52 },  /* gold    */
    { 255,251,247,243,239,235,234,232 },  /* mono    */
};
static const char *g_theme_names[N_THEMES] = {
    "fire","matrix","ice","plasma","gold","mono"
};

#define PAIR(t,b)  ((t)*N_BAND_COLORS + (b) + 1)
#define HUD_PAIR   (N_THEMES*N_BAND_COLORS + 1)

/* ── §2  clock ──────────────────────────────────────────────────────────── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3  color init ──────────────────────────────────────────────────────── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        for (int t = 0; t < N_THEMES; t++)
            for (int b = 0; b < N_BAND_COLORS; b++)
                init_pair(PAIR(t,b), g_pal[t][b], -1);
    } else {
        static const int fb[N_BAND_COLORS] = {
            COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_CYAN,
            COLOR_CYAN,  COLOR_BLUE,   COLOR_BLUE,   COLOR_WHITE
        };
        for (int t = 0; t < N_THEMES; t++)
            for (int b = 0; b < N_BAND_COLORS; b++)
                init_pair(PAIR(t,b), fb[b], -1);
    }
    init_pair(HUD_PAIR, 15, -1);
}

/* ── §4  hex grid state ──────────────────────────────────────────────────── */

typedef struct { char ch; float timer; float rate; } HCell;
static HCell g_hex[HEX_MAX_ROWS][HEX_MAX_COLS];

static int g_hx_count;   /* active hex columns */
static int g_hy_count;   /* active hex rows    */
static int g_cx_hex;     /* centre hex column index */
static int g_cy_hex;     /* centre hex row index    */

/*
 * Hex ring distance (cube coordinate distance / Chebyshev approximation).
 * For an offset-row layout, convert (hx, hy) to cube coords first:
 *   cube_q = hx - (hy - (hy&1)) / 2
 *   cube_r = hy
 *   cube_s = -cube_q - cube_r
 *   dist   = max(|q|, |r|, |s|)
 */
static int hex_dist(int hx1, int hy1, int hx2, int hy2)
{
    int q1 = hx1 - (hy1 - (hy1 & 1)) / 2;
    int r1 = hy1;
    int q2 = hx2 - (hy2 - (hy2 & 1)) / 2;
    int r2 = hy2;
    int dq = q1 - q2, dr = r1 - r2, ds = dq + dr;
    int v = abs(dq); if (abs(dr) > v) v = abs(dr); if (abs(ds) > v) v = abs(ds);
    return v;
}

static void hex_init(int rows, int cols, float base_rate)
{
    /* How many hex cells fit in the terminal */
    g_hx_count = (cols / HEX_DX) + 1;
    g_hy_count = (rows / HEX_DY);
    if (g_hx_count > HEX_MAX_COLS) g_hx_count = HEX_MAX_COLS;
    if (g_hy_count > HEX_MAX_ROWS) g_hy_count = HEX_MAX_ROWS;

    g_cx_hex = g_hx_count / 2;
    g_cy_hex = g_hy_count / 2;

    for (int hy = 0; hy < g_hy_count; hy++) {
        for (int hx = 0; hx < g_hx_count; hx++) {
            float rate = base_rate * (0.4f + (float)(rand() % 1000) / 1000.f * 1.2f);
            g_hex[hy][hx].ch    = (char)(CH_FIRST + rand() % CH_RANGE);
            g_hex[hy][hx].rate  = rate;
            g_hex[hy][hx].timer = (float)(rand() % 1000) / (1000.f * rate);
        }
    }
}

static void hex_tick(float dt)
{
    for (int hy = 0; hy < g_hy_count; hy++) {
        for (int hx = 0; hx < g_hx_count; hx++) {
            HCell *c = &g_hex[hy][hx];
            c->timer -= dt;
            if (c->timer <= 0.f) {
                c->ch    = (char)(CH_FIRST + rand() % CH_RANGE);
                c->timer = 1.f / c->rate;
            }
        }
    }
}

/*
 * Draw each hex cell's character at its screen position.
 * Screen position of hex (hx, hy):
 *   col = hx * HEX_DX + (hy & 1) * (HEX_DX/2)
 *   row = hy * HEX_DY
 * Colour band = min(dist_from_centre, N_BAND_COLORS-1).
 */
static void hex_draw(int rows, int cols, int theme)
{
    for (int hy = 0; hy < g_hy_count; hy++) {
        for (int hx = 0; hx < g_hx_count; hx++) {
            int col = hx * HEX_DX + (hy & 1) * (HEX_DX / 2);
            int row = hy * HEX_DY;

            if (row >= rows - 1) continue;
            if (col >= cols)     continue;

            int dist = hex_dist(hx, hy, g_cx_hex, g_cy_hex);
            int band = dist % N_BAND_COLORS;

            attron(COLOR_PAIR(PAIR(theme, band)));
            mvaddch(row, col, (chtype)(unsigned char)g_hex[hy][hx].ch);
            attroff(COLOR_PAIR(PAIR(theme, band)));
        }
    }
}

/* ── §5  HUD + signals ───────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run = 1;
static volatile sig_atomic_t g_rsz = 0;
static void on_sigint  (int s) { (void)s; g_run = 0; }
static void on_sigwinch(int s) { (void)s; g_rsz = 1; }

static void draw_hud(int rows, float fps, int theme, int spd, int paused)
{
    attron(COLOR_PAIR(HUD_PAIR));
    mvprintw(rows - 1, 1,
        " fps:%.0f  theme:%s  speed:%s%s  [t]theme [s]speed [p]pause [q]quit ",
        (double)fps, g_theme_names[theme], g_speed_names[spd],
        paused ? "  PAUSED" : "");
    attroff(COLOR_PAIR(HUD_PAIR));
}

/* ── §6  main ────────────────────────────────────────────────────────────── */

int main(void)
{
    srand((unsigned)(clock_ns() & 0xFFFFFFFF));
    signal(SIGINT, on_sigint); signal(SIGWINCH, on_sigwinch);
    initscr(); cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE); typeahead(-1);

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    color_init();

    int   theme  = 0;
    int   spd    = 2;
    int   paused = 0;
    float fps = 0.f, fps_acc = 0.f;
    int   fps_cnt = 0;
    long long frame_ns = 1000000000LL / TARGET_FPS;
    long long last = clock_ns();

    hex_init(rows, cols, g_speeds[spd]);

    while (g_run) {
        if (g_rsz) {
            g_rsz = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            hex_init(rows, cols, g_speeds[spd]);
        }

        long long now = clock_ns();
        float dt = (float)(now - last) * 1e-9f;
        if (dt > 0.1f) dt = 0.1f;
        last = now;

        if (!paused) hex_tick(dt);

        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 0.5f) {
            fps = (float)fps_cnt / fps_acc;
            fps_acc = 0.f; fps_cnt = 0;
        }

        long long t0 = clock_ns();
        erase();
        hex_draw(rows, cols, theme);
        draw_hud(rows, fps, theme, spd, paused);
        wnoutrefresh(stdscr); doupdate();

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_run = 0;                         break;
        case 'p': case 'P': paused = !paused;                           break;
        case 't': case 'T': theme = (theme + 1) % N_THEMES;            break;
        case 's': case 'S':
            spd = (spd + 1) % N_SPEEDS;
            hex_init(rows, cols, g_speeds[spd]);
            break;
        }

        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }
    endwin();
    return 0;
}
