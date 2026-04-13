/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * string_art.c — Mathematical String Art
 *
 * N nails on a visible circle; threads connect nail i → round(i×k) mod N.
 * k drifts slowly through [2, 7.5]:
 *
 *   k ≈ 2  →  Cardioid       (1 cusp)
 *   k ≈ 3  →  Nephroid       (2 cusps)
 *   k ≈ 4  →  Deltoid        (3 cusps)
 *   k ≈ 5  →  Astroid        (4 cusps)
 *   k ≈ 6  →  5-Epicycloid
 *   k ≈ 7  →  6-Epicycloid
 *
 * Rendering:
 *   ·         circle rim  (the "board" the nails are pinned to)
 *   o  bold   nail        (white, drawn on top of rim and threads)
 *   - | / \   thread      (slope-based char; each nail gets its own colour
 *                          cycling through a 12-step rainbow palette)
 *
 * k slows near integer values so each named shape dwells for a few seconds.
 *
 * Nail count presets ([ / ]):  6  8  12  16  20  30  60  90  120  200
 * With few nails each thread is individually visible; with 200 nails the
 * dense envelope traces the curve boundary.
 *
 * Keys:
 *   q / Q     quit
 *   space     pause / resume
 *   r / R     reset  (k → 2)
 *   [ / ]     fewer / more nails  (steps through preset list)
 *   + / =     increase k drift speed  × 1.5
 *   - / _     decrease k drift speed  ÷ 1.5
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/string_art.c \
 *       -o string_art -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 coords  §5 sim
 *           §6 scene   §7 screen §8 app
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

#define TICK_NS       33333333LL   /* ~30 fps                             */
#define N_NAILS_MAX   200
#define K_START       2.0f
#define K_END         7.5f
#define K_SPEED_DEF   0.0008f      /* k increment per tick at full speed  */
#define K_SPEED_MIN   0.0001f
#define K_SPEED_MAX   0.010f
#define CELL_W        8
#define CELL_H        16
#define MAX_ROWS      128
#define MAX_COLS      320
#define RIM_STEPS     2000         /* sample points for circle rim        */

/* Thread colour pairs occupy slots CP_T0 … CP_T0+N_TCOLS-1.
 * Non-thread pairs sit below that range. */
#define N_TCOLS  12
#define CP_T0    10

/* 12-step rainbow (256-color) */
static const short TCOL_256[N_TCOLS] = {
    196, 202, 208, 214, 226, 154, 46, 51, 45, 27, 93, 201
};
/* 8-color fallback cycle */
static const short TCOL_8[N_TCOLS] = {
    COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
    COLOR_YELLOW, COLOR_GREEN, COLOR_GREEN, COLOR_CYAN,
    COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA
};

enum { CP_RIM = 1, CP_NAIL = 2, CP_HUD = 3 };

/* available nail counts */
static const int NAIL_PRESETS[] = { 6, 8, 12, 16, 20, 30, 60, 90, 120, 200 };
#define N_PRESETS  ((int)(sizeof(NAIL_PRESETS) / sizeof(NAIL_PRESETS[0])))

static const char *shape_name(float k)
{
    switch ((int)(k + 0.12f)) {
        case 2:  return "Cardioid";
        case 3:  return "Nephroid";
        case 4:  return "Deltoid";
        case 5:  return "Astroid";
        case 6:  return "5-Epicycloid";
        case 7:  return "6-Epicycloid";
        default: return "Morphing";
    }
}

/* ── §2 clock ────────────────────────────────────────────────────────── */

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

/* ── §3 color ────────────────────────────────────────────────────────── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_RIM,  245, -1);  /* mid-grey rim  */
        init_pair(CP_NAIL, 231, -1);  /* white nails   */
        init_pair(CP_HUD,   82, -1);  /* green HUD     */
        for (int i = 0; i < N_TCOLS; i++)
            init_pair(CP_T0 + i, TCOL_256[i], -1);
    } else {
        init_pair(CP_RIM,  COLOR_WHITE,  -1);
        init_pair(CP_NAIL, COLOR_YELLOW, -1);
        init_pair(CP_HUD,  COLOR_GREEN,  -1);
        for (int i = 0; i < N_TCOLS; i++)
            init_pair(CP_T0 + i, TCOL_8[i], -1);
    }
}

/* ── §4 coords ───────────────────────────────────────────────────────── */

static int   g_nail_cx[N_NAILS_MAX];
static int   g_nail_cy[N_NAILS_MAX];
static int   g_rows, g_cols;
static float g_r_px, g_cx_px, g_cy_px;
static int   g_n_nails;

static void compute_nails(int rows, int cols, int n)
{
    g_rows    = rows;
    g_cols    = cols;
    g_n_nails = n;
    g_r_px    = fminf((float)cols * CELL_W, (float)rows * CELL_H) * 0.44f;
    g_cx_px   = (float)cols * CELL_W * 0.5f;
    g_cy_px   = (float)rows * CELL_H * 0.5f;
    for (int i = 0; i < n; i++) {
        float a  = 2.0f * (float)M_PI * i / n;
        int   cx = (int)(g_cx_px + g_r_px * cosf(a)) / CELL_W;
        int   cy = (int)(g_cy_px + g_r_px * sinf(a)) / CELL_H;
        g_nail_cx[i] = cx < 0 ? 0 : cx >= cols ? cols - 1 : cx;
        g_nail_cy[i] = cy < 0 ? 0 : cy >= rows ? rows - 1 : cy;
    }
}

/* ── §5 simulation ───────────────────────────────────────────────────── */

static float g_k;
static float g_k_speed;
static int   g_paused;
static int   g_nail_preset;

static float wrap_k(float k)
{
    float range = K_END - K_START;
    while (k >= K_END)   k -= range;
    while (k <  K_START) k += range;
    return k;
}

static void sim_init(void)
{
    g_k          = K_START;
    g_k_speed    = K_SPEED_DEF;
    g_paused     = 0;
    g_nail_preset = N_PRESETS - 1;  /* start at 200 nails */
}

static void sim_tick(void)
{
    if (g_paused) return;
    /*
     * Speed modulation: slowest at integer k (0.15×) so named shapes
     * are visible long enough to read.  dist ∈ [0, 0.5].
     */
    float dist = fabsf(g_k - roundf(g_k));
    float mult = 0.15f + 1.70f * (dist * dist * 4.0f);
    if (mult > 1.0f) mult = 1.0f;
    g_k = wrap_k(g_k + g_k_speed * mult);
}

/* ── §6 scene ────────────────────────────────────────────────────────── */

/*
 * One character represents each thread based on its visual slope.
 * dy is multiplied by 2 to account for 2:1 cell aspect ratio so that
 * a "visual 45°" line is classified correctly.
 */
static char thread_char(int x0, int y0, int x1, int y1)
{
    int dx = x1 - x0, dy = y1 - y0;
    if (dx == 0 && dy == 0) return '+';
    float vs = (dx != 0) ? fabsf((float)dy * 2.0f / (float)dx) : 1e9f;
    if (vs < 0.577f) return '-';
    if (vs > 1.732f) return '|';
    return ((dx > 0) == (dy > 0)) ? '\\' : '/';
}

static void bres_draw(int x0, int y0, int x1, int y1, chtype ch)
{
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if ((unsigned)x0 < (unsigned)g_cols &&
            (unsigned)y0 < (unsigned)g_rows)
            mvaddch(y0, x0, ch);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void scene_draw(void)
{
    int n = g_n_nails;

    /* ── 1. threads (underneath nails) ──────────────────────────────── */
    for (int i = 0; i < n; i++) {
        int j = (int)roundf((float)i * g_k) % n;
        if (j < 0) j += n;
        chtype ch = (chtype)(unsigned char)
                    thread_char(g_nail_cx[i], g_nail_cy[i],
                                g_nail_cx[j], g_nail_cy[j]);
        /* each nail index gets its own colour from the 12-step rainbow */
        attron(COLOR_PAIR(CP_T0 + i % N_TCOLS));
        bres_draw(g_nail_cx[i], g_nail_cy[i],
                  g_nail_cx[j], g_nail_cy[j], ch);
        attroff(COLOR_PAIR(CP_T0 + i % N_TCOLS));
    }

    /* ── 2. circle rim (overdraw threads at boundary) ────────────────── */
    attron(COLOR_PAIR(CP_RIM) | A_DIM);
    for (int a = 0; a < RIM_STEPS; a++) {
        float angle = 2.0f * (float)M_PI * a / RIM_STEPS;
        int cx = (int)(g_cx_px + g_r_px * cosf(angle)) / CELL_W;
        int cy = (int)(g_cy_px + g_r_px * sinf(angle)) / CELL_H;
        if ((unsigned)cx < (unsigned)g_cols &&
            (unsigned)cy < (unsigned)g_rows)
            mvaddch(cy, cx, '.');
    }
    attroff(COLOR_PAIR(CP_RIM) | A_DIM);

    /* ── 3. nails (bright white, on top of everything) ───────────────── */
    attron(COLOR_PAIR(CP_NAIL) | A_BOLD);
    for (int i = 0; i < n; i++)
        mvaddch(g_nail_cy[i], g_nail_cx[i], 'o');
    attroff(COLOR_PAIR(CP_NAIL) | A_BOLD);
}

static void scene_hud(void)
{
    if (g_rows < 2) return;
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(g_rows - 1, 0,
             " k=%.3f [%-12s] nails=%3d  spd=%.4f  %s"
             "  spc:pause  r:reset  [/]:nails  +/-:spd  q:quit ",
             (double)g_k, shape_name(g_k), g_n_nails,
             (double)g_k_speed,
             g_paused ? "PAUSED" : "      ");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §7 screen ───────────────────────────────────────────────────────── */

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
    compute_nails(rows, cols, NAIL_PRESETS[g_nail_preset]);
    erase();
}

/* ── §8 app ──────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running     = 0;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);

    screen_init();
    sim_init();         /* sets g_nail_preset before screen_resize uses it */
    screen_resize();

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin();
            refresh();
            screen_resize();
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': g_running = 0; break;
            case ' ':           g_paused ^= 1; break;
            case 'r': case 'R': g_k = K_START; break;
            case '+': case '=':
                g_k_speed *= 1.5f;
                if (g_k_speed > K_SPEED_MAX) g_k_speed = K_SPEED_MAX;
                break;
            case '-': case '_':
                g_k_speed /= 1.5f;
                if (g_k_speed < K_SPEED_MIN) g_k_speed = K_SPEED_MIN;
                break;
            case ']':
                if (g_nail_preset < N_PRESETS - 1) {
                    g_nail_preset++;
                    compute_nails(g_rows, g_cols, NAIL_PRESETS[g_nail_preset]);
                }
                break;
            case '[':
                if (g_nail_preset > 0) {
                    g_nail_preset--;
                    compute_nails(g_rows, g_cols, NAIL_PRESETS[g_nail_preset]);
                }
                break;
            }
        }

        sim_tick();
        erase();
        scene_draw();
        scene_hud();
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
