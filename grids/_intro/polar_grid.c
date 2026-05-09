/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * polar_grid.c — polar / radial character grid
 *
 * Characters are placed at the intersections of concentric rings and radial
 * spokes.  Each ring rotates at its own angular speed (alternating direction)
 * and is coloured with a distinct shade from the active theme.
 * Characters at every cell change independently at random rates.
 *
 * The grid looks like a sonar screen or a mandala that slowly breathes.
 *
 * Themes (t): fire · matrix · ice · plasma · gold · mono
 * Speed  (s): slow · medium · fast · frenzy
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grid/polar_grid.c -o polar_grid -lncurses -lm
 *
 * Keys:
 *   t     cycle theme
 *   s     cycle speed
 *   p     pause / resume
 *   q / ESC  quit
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Polar coordinate grid with per-ring differential rotation.
 *                  Each ring rotates at a different angular velocity — alternating
 *                  clockwise/counterclockwise — creating a mesmerising gear-like
 *                  visual.  This demonstrates coordinate system transforms:
 *                  (ring, spoke) → (r, θ) → (x, y) → (col, row).
 *
 * Math           : Polar → Cartesian → terminal coordinates:
 *                    x = r · cos(θ + ω_ring · t)  [in pixels]
 *                    y = r · sin(θ + ω_ring · t)  [in pixels]
 *                    col = x / CELL_W + centre_col
 *                    row = y / CELL_H + centre_row
 *                  CELL_H / CELL_W ≈ 2: rows are compressed relative to columns,
 *                  so the radial distance uses r_y = r × (CELL_W / CELL_H) for
 *                  circular appearance despite non-square cells.
 *
 * Rendering      : Ring index maps to colour pair (concentric colour bands).
 *                  Characters at each cell change independently; adjacent cells
 *                  on the same ring change at similar rates (ring "breathing").
 * ─────────────────────────────────────────────────────────────────────── */

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

#define TARGET_FPS      60
#define N_RINGS         10     /* concentric rings                           */
#define MAX_SECTORS    128     /* max spokes on any single ring              */
#define N_THEME_COLORS  N_RINGS  /* one colour per ring (10 shades)         */
#define N_THEMES         6
#define N_SPEEDS         4
#define ASPECT          0.47f  /* terminal cell width / height               */
#define CH_FIRST       0x21
#define CH_RANGE         94

static const float  g_speeds[N_SPEEDS]      = { 1.f, 5.f, 15.f, 40.f };
static const char  *g_speed_names[N_SPEEDS] = { "slow","medium","fast","frenzy" };

/*
 * 10-shade ramps per theme.  Index 0 = innermost (brightest),
 * index 9 = outermost (dimmest). Gradient fades outward.
 */
static const int g_pal[N_THEMES][N_RINGS] = {
    { 255,226,220,214,208,202,196,160,124, 88 },  /* fire    */
    {  82, 46, 40, 34, 28, 22,  2,  2,  0,  0 },  /* matrix  */
    { 159, 51, 45, 33, 27, 21, 20, 19, 18, 17 },  /* ice     */
    { 207,201,165,129, 93, 57, 56, 55, 54, 53 },  /* plasma  */
    { 220,184,178,172,136,130, 94, 58, 52, 52 },  /* gold    */
    { 255,251,247,243,239,235,238,232,232,232 },  /* mono    */
};
static const char *g_theme_names[N_THEMES] = {
    "fire","matrix","ice","plasma","gold","mono"
};

#define PAIR(t,c)  ((t)*N_RINGS + (c) + 1)
#define HUD_PAIR   (N_THEMES*N_RINGS + 1)

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
            for (int c = 0; c < N_RINGS; c++)
                init_pair(PAIR(t,c), g_pal[t][c], -1);
    } else {
        static const int fb[N_RINGS] = {
            COLOR_WHITE, COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW,
            COLOR_CYAN,  COLOR_CYAN,  COLOR_BLUE,   COLOR_BLUE,
            COLOR_WHITE, COLOR_WHITE
        };
        for (int t = 0; t < N_THEMES; t++)
            for (int c = 0; c < N_RINGS; c++)
                init_pair(PAIR(t,c), fb[c], -1);
    }
    init_pair(HUD_PAIR, 15, -1);
}

/* ── §4  polar grid state ────────────────────────────────────────────────── */

typedef struct { char ch; float timer; float rate; } PCell;

static PCell  g_rings[N_RINGS][MAX_SECTORS];
static int    g_n_sectors[N_RINGS];
static float  g_ring_r[N_RINGS];       /* radius in column units            */
static float  g_ring_angle[N_RINGS];   /* current rotation offset (radians) */
static float  g_rot_speed[N_RINGS];    /* radians per second                 */

static void polar_init(int rows, int cols, float base_rate)
{
    int cx = cols / 2, cy = rows / 2;

    /* Outermost ring must fit inside the terminal.
     * In physical pixels: max_r_px = min(cx*cell_w, cy*cell_h).
     * In column units: max_r = min(cx, cy * cell_h/cell_w) = min(cx, cy/ASPECT). */
    float max_r = fminf((float)(cx - 2), (float)(cy - 1) / ASPECT);
    float min_r = 2.5f;

    for (int i = 0; i < N_RINGS; i++) {
        float t = (float)i / (float)(N_RINGS - 1);
        /* Quadratic spacing: more rings near centre */
        float r = min_r + (max_r - min_r) * (t * t * 0.4f + t * 0.6f);
        g_ring_r[i] = r;

        /* Sectors: one per ~2.2 column-widths of arc */
        int n = (int)(2.f * (float)M_PI * r / 2.2f);
        if (n < 5)          n = 5;
        if (n > MAX_SECTORS) n = MAX_SECTORS;
        g_n_sectors[i] = n;

        /* Alternating rotation, faster inward */
        float dir   = (i % 2 == 0) ? 1.f : -1.f;
        float speed = 0.12f + (float)(N_RINGS - 1 - i) * 0.04f;
        g_rot_speed[i]  = dir * speed;
        g_ring_angle[i] = (float)(rand() % 628) / 100.f;

        for (int j = 0; j < n; j++) {
            float rate = base_rate * (0.4f + (float)(rand() % 1000) / 1000.f * 1.2f);
            g_rings[i][j].ch    = (char)(CH_FIRST + rand() % CH_RANGE);
            g_rings[i][j].rate  = rate;
            g_rings[i][j].timer = (float)(rand() % 1000) / (1000.f * rate);
        }
    }

    (void)cx; (void)cy;
}

static void polar_tick(float dt, float speed)
{
    for (int i = 0; i < N_RINGS; i++) {
        g_ring_angle[i] += g_rot_speed[i] * dt;

        int n = g_n_sectors[i];
        for (int j = 0; j < n; j++) {
            PCell *c = &g_rings[i][j];
            c->timer -= dt * speed;
            if (c->timer <= 0.f) {
                c->ch    = (char)(CH_FIRST + rand() % CH_RANGE);
                c->timer = 1.f / c->rate;
            }
        }
    }
}

static void polar_draw(int rows, int cols, int theme)
{
    int cx = cols / 2, cy = rows / 2;

    for (int i = 0; i < N_RINGS; i++) {
        int   n    = g_n_sectors[i];
        float r    = g_ring_r[i];
        float base = g_ring_angle[i];
        int   pair = PAIR(theme, i);

        for (int j = 0; j < n; j++) {
            float angle = base + 2.f * (float)M_PI * (float)j / (float)n;
            int col = cx + (int)(r * cosf(angle) + 0.5f);
            int row = cy + (int)(r * sinf(angle) * ASPECT + 0.5f);

            if (row < 0 || row >= rows - 1) continue;
            if (col < 0 || col >= cols)     continue;

            attron(COLOR_PAIR(pair));
            mvaddch(row, col, (chtype)(unsigned char)g_rings[i][j].ch);
            attroff(COLOR_PAIR(pair));
        }
    }

    /* Centre dot */
    if (cy >= 0 && cy < rows - 1 && cx >= 0 && cx < cols) {
        attron(COLOR_PAIR(PAIR(theme, 0)) | A_BOLD);
        mvaddch(cy, cx, '+');
        attroff(COLOR_PAIR(PAIR(theme, 0)) | A_BOLD);
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
    int   spd    = 1;
    int   paused = 0;
    float fps = 0.f, fps_acc = 0.f;
    int   fps_cnt = 0;
    long long frame_ns = 1000000000LL / TARGET_FPS;
    long long last = clock_ns();

    polar_init(rows, cols, g_speeds[spd]);

    while (g_run) {
        if (g_rsz) {
            g_rsz = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            polar_init(rows, cols, g_speeds[spd]);
        }

        long long now = clock_ns();
        float dt = (float)(now - last) * 1e-9f;
        if (dt > 0.1f) dt = 0.1f;
        last = now;

        if (!paused) polar_tick(dt, g_speeds[spd]);

        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 0.5f) {
            fps = (float)fps_cnt / fps_acc;
            fps_acc = 0.f; fps_cnt = 0;
        }

        long long t0 = clock_ns();
        erase();
        polar_draw(rows, cols, theme);
        draw_hud(rows, fps, theme, spd, paused);
        wnoutrefresh(stdscr); doupdate();

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_run = 0;                         break;
        case 'p': case 'P': paused = !paused;                           break;
        case 't': case 'T': theme = (theme + 1) % N_THEMES;            break;
        case 's': case 'S':
            spd = (spd + 1) % N_SPEEDS;
            polar_init(rows, cols, g_speeds[spd]);
            break;
        }

        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }
    endwin();
    return 0;
}
