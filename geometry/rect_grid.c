/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rect_grid.c — rectangular character grid
 *
 * Every terminal cell holds a character that flips at its own random rate.
 * Two sinusoidal colour waves wash across the grid independently, painting
 * each cell a shade from the active theme palette.
 *
 * Themes (t): fire · matrix · ice · plasma · gold · mono
 * Speed  (s): slow · medium · fast · frenzy
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grid/rect_grid.c -o rect_grid -lncurses -lm
 *
 * Keys:
 *   t     cycle theme
 *   s     cycle speed
 *   p     pause / resume
 *   q / ESC  quit
 */

#define _POSIX_C_SOURCE 199309L
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <signal.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1  config ─────────────────────────────────────────────────────────── */

#define TARGET_FPS       60
#define GRID_MAX_COLS   240
#define GRID_MAX_ROWS    70
#define N_THEME_COLORS    6
#define N_THEMES          6
#define N_SPEEDS          4
#define CH_FIRST       0x21    /* '!'                                        */
#define CH_RANGE         94    /* printable ASCII '!' .. '~'                 */

/* rate = character changes per second */
static const float  g_speeds[N_SPEEDS]      = { 2.f, 10.f, 30.f, 60.f };
static const char  *g_speed_names[N_SPEEDS] = { "slow","medium","fast","frenzy" };

/* 256-color foreground indices per theme (darkest → brightest) */
static const int g_pal[N_THEMES][N_THEME_COLORS] = {
    { 196, 202, 208, 214, 220, 226 },   /* fire    */
    {  22,  28,  34,  40,  46,  82 },   /* matrix  */
    {  17,  21,  27,  33,  51, 159 },   /* ice     */
    {  53,  90, 127, 164, 201, 207 },   /* plasma  */
    {  94, 130, 136, 172, 178, 220 },   /* gold    */
    { 235, 239, 243, 247, 251, 255 },   /* mono    */
};
static const char *g_theme_names[N_THEMES] = {
    "fire","matrix","ice","plasma","gold","mono"
};

#define PAIR(t,c)  ((t)*N_THEME_COLORS + (c) + 1)
#define HUD_PAIR   (N_THEMES*N_THEME_COLORS + 1)

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
            for (int c = 0; c < N_THEME_COLORS; c++)
                init_pair(PAIR(t,c), g_pal[t][c], -1);
    } else {
        static const int fb[N_THEME_COLORS] = {
            COLOR_RED, COLOR_RED, COLOR_YELLOW,
            COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE
        };
        for (int t = 0; t < N_THEMES; t++)
            for (int c = 0; c < N_THEME_COLORS; c++)
                init_pair(PAIR(t,c), fb[c], -1);
    }
    init_pair(HUD_PAIR, 15, -1);
}

/* ── §4  grid ────────────────────────────────────────────────────────────── */

/*
 * Each cell stores its current character, a countdown timer, and a base
 * change rate.  The wave colour is computed dynamically at draw time so no
 * colour needs to be stored per cell — saves memory and lets the wave move
 * independently of character updates.
 */
typedef struct { char ch; float timer; float rate; } Cell;
static Cell g_grid[GRID_MAX_ROWS][GRID_MAX_COLS];

static void grid_init(int rows, int cols, float base_rate)
{
    for (int r = 0; r < rows && r < GRID_MAX_ROWS; r++) {
        for (int c = 0; c < cols && c < GRID_MAX_COLS; c++) {
            float rate = base_rate * (0.4f + (float)(rand() % 1000) / 1000.f * 1.2f);
            g_grid[r][c].ch    = (char)(CH_FIRST + rand() % CH_RANGE);
            g_grid[r][c].rate  = rate;
            g_grid[r][c].timer = (float)(rand() % 1000) / (1000.f * rate);
        }
    }
}

static void grid_tick(int rows, int cols, float dt)
{
    for (int r = 0; r < rows && r < GRID_MAX_ROWS; r++) {
        for (int c = 0; c < cols && c < GRID_MAX_COLS; c++) {
            Cell *cell = &g_grid[r][c];
            cell->timer -= dt;
            if (cell->timer <= 0.f) {
                cell->ch    = (char)(CH_FIRST + rand() % CH_RANGE);
                cell->timer = 1.f / cell->rate;
            }
        }
    }
}

/*
 * Colour wave: two sinusoids at different frequencies and phases combine to
 * create a slowly shifting interference pattern across the grid.
 * The result maps to a palette index 0..N_THEME_COLORS-1.
 */
static void grid_draw(int rows, int cols, int theme, float wx, float wy)
{
    float last_wx[GRID_MAX_COLS];   /* cache wave x component per column */
    for (int c = 0; c < cols && c < GRID_MAX_COLS; c++)
        last_wx[c] = sinf((float)c * 0.18f + wx);

    for (int r = 0; r < rows - 1 && r < GRID_MAX_ROWS; r++) {
        float wr = sinf((float)r * 0.26f + wy);
        for (int c = 0; c < cols && c < GRID_MAX_COLS; c++) {
            float w   = (last_wx[c] + wr) * 0.5f * 0.5f + 0.5f;  /* 0..1 */
            int   idx = (int)(w * (float)(N_THEME_COLORS - 1) + 0.5f);
            if (idx < 0) idx = 0;
            if (idx >= N_THEME_COLORS) idx = N_THEME_COLORS - 1;
            attron(COLOR_PAIR(PAIR(theme, idx)));
            mvaddch(r, c, (chtype)(unsigned char)g_grid[r][c].ch);
            attroff(COLOR_PAIR(PAIR(theme, idx)));
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

    int   theme = 1;   /* start on matrix */
    int   spd   = 2;   /* fast */
    int   paused = 0;
    float fps = 0.f, fps_acc = 0.f;
    int   fps_cnt = 0;
    float wave_x = 0.f, wave_y = 0.f;
    long long frame_ns = 1000000000LL / TARGET_FPS;
    long long last = clock_ns();

    grid_init(rows, cols, g_speeds[spd]);

    while (g_run) {
        if (g_rsz) {
            g_rsz = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            grid_init(rows, cols, g_speeds[spd]);
        }

        long long now = clock_ns();
        float dt = (float)(now - last) * 1e-9f;
        if (dt > 0.1f) dt = 0.1f;
        last = now;

        if (!paused) {
            grid_tick(rows, cols, dt);
            wave_x += dt * 0.75f;
            wave_y += dt * 0.48f;
        }

        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 0.5f) {
            fps = (float)fps_cnt / fps_acc;
            fps_acc = 0.f; fps_cnt = 0;
        }

        long long t0 = clock_ns();
        erase();
        grid_draw(rows, cols, theme, wave_x, wave_y);
        draw_hud(rows, fps, theme, spd, paused);
        wnoutrefresh(stdscr); doupdate();

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_run = 0;                         break;
        case 'p': case 'P': paused = !paused;                           break;
        case 't': case 'T': theme = (theme + 1) % N_THEMES;            break;
        case 's': case 'S':
            spd = (spd + 1) % N_SPEEDS;
            grid_init(rows, cols, g_speeds[spd]);
            break;
        }

        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }
    endwin();
    return 0;
}
