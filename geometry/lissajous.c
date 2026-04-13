/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lissajous.c — Harmonograph / Lissajous Figures
 *
 * Two perpendicular damped oscillators:
 *   x(t) = sin(fx·t + phase)·exp(−decay·t)
 *   y(t) = sin(fy·t)       ·exp(−decay·t)
 *
 * The phase offset drifts slowly, morphing the Lissajous figure
 * through figure-8s, stars, petals, and spirals.  Drift slows near
 * symmetric phases so each named shape dwells on screen before the
 * figure transitions.  After a full 2π phase sweep the program
 * auto-advances to the next rational frequency ratio.
 *
 * Ratios:  1:2 Figure-8 · 2:3 Trefoil · 3:4 Star   · 1:3 Clover
 *          3:5 Pentagram · 2:5 Five-petal · 4:5 Crown · 1:4 Eye
 *
 * Each ratio's T_MAX scales to N_LOOPS=4 cycles of the slower
 * oscillator, and DECAY adjusts so amplitude reaches ~1% at T_MAX —
 * giving an identical spiral depth regardless of frequency ratio.
 *
 * Rendering: draw the full decaying spiral each frame (oldest innermost
 * first, newest outermost last).  Four brightness levels map to the four
 * decay loops; the outermost loop is always brightest.
 *
 * Keys:
 *   space      pause / resume
 *   n / p      next / prev ratio (resets phase to 0)
 *   + / =      faster phase drift
 *   - / _      slower phase drift
 *   c          cycle color themes (Golden / Ice / Ember / Neon)
 *   q / Q      quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/lissajous.c -o lissajous -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 sim  §5 scene  §6 screen  §7 app
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define TICK_NS       33333333LL   /* ~30 fps                              */
#define N_CURVE_PTS   2500         /* parametric samples per frame          */
#define N_LOOPS       4            /* complete cycles of slower oscillator  */
#define DECAY_TOTAL   4.5f         /* total exponent → amp ≈ 1% at T_MAX   */
#define CELL_ASPECT   2.0f         /* physical char height / width ratio    */
#define AMP_FRAC      0.92f        /* fraction of screen half-size used     */

/* Color pairs: N_THEMES × N_LEVELS + HUD */
#define N_LEVELS      4            /* one brightness level per decay loop   */
#define N_THEMES      4
#define CP_IDX(t,l)   ((t)*N_LEVELS + (l) + 1)
#define CP_HUD        (N_THEMES * N_LEVELS + 1)

/* Phase drift (rad/tick; 2π ÷ (DRIFT × 30fps) = seconds per ratio) */
#define DRIFT_DEFAULT 0.004f       /* 2π in ~1047 ticks ≈ 35 s             */
#define DRIFT_MIN     0.0005f
#define DRIFT_MAX     0.08f
#define DRIFT_STEP    1.6f

/*
 * Drift slow-down near symmetric phases.
 *
 * Symmetric figures occur every π/max(fx,fy) radians of phase.
 * Within DWELL_WIDTH of a symmetric phase, drift is multiplied to
 * DWELL_SPEED.  Linear ramp from DWELL_SPEED at the key phase to 1.0
 * at DWELL_WIDTH fraction of the period away.
 */
#define DWELL_WIDTH   0.25f        /* fraction of key period to slow down   */
#define DWELL_SPEED   0.25f        /* speed multiplier at the key phase     */

#define N_RATIOS      8

/* ===================================================================== */
/* §2  clock                                                              */
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
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Four themes, each with four brightness levels.
 * Level 0 = outermost loop (full amplitude, brightest).
 * Level 3 = innermost loop (tiny amplitude, dimmest).
 */
static const int k_theme[N_THEMES][N_LEVELS] = {
    /* Golden: bright-yellow → gold → dark-orange → brown   */
    { 226, 220, 136,  94 },
    /* Ice:    bright-cyan  → teal → dark-teal  → navy      */
    {  51,  38,  23,  17 },
    /* Ember:  white → orange → red → dark-red              */
    { 231, 208, 196,  88 },
    /* Neon:   bright-green → green → dark-green → very-dark*/
    { 118,  82,  28,  22 },
};
static const char *k_theme_names[N_THEMES] = {
    "Golden", "Ice", "Ember", "Neon"
};

/* Character for each brightness level, brightest first */
static const char k_lev_ch[N_LEVELS] = { '#', '*', '+', '.' };

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        for (int t = 0; t < N_THEMES; t++)
            for (int l = 0; l < N_LEVELS; l++)
                init_pair(CP_IDX(t, l), k_theme[t][l], -1);
        init_pair(CP_HUD, 244, -1);
    } else {
        for (int i = 1; i <= N_THEMES * N_LEVELS; i++)
            init_pair(i, COLOR_YELLOW, -1);
        init_pair(CP_HUD, COLOR_WHITE, -1);
    }
}

/* ===================================================================== */
/* §4  sim                                                                */
/* ===================================================================== */

/* Rational frequency pairs and their shape-family names */
static const float k_ratio[N_RATIOS][2] = {
    {1,2}, {2,3}, {3,4}, {1,3}, {3,5}, {2,5}, {4,5}, {1,4}
};
static const char *k_ratio_names[N_RATIOS] = {
    "1:2 Figure-8",  "2:3 Trefoil",   "3:4 Star",    "1:3 Clover",
    "3:5 Pentagram", "2:5 Five-petal", "4:5 Crown",   "1:4 Eye",
};

typedef struct {
    int   ratio;
    float phase_x;   /* drifts 0 → 2π, then wraps and advances ratio */
    float drift;     /* base phase advance per tick (rad)              */
    int   theme;
    bool  paused;
} Scene;

/*
 * Phase drift with slow-down near symmetric phases.
 *
 * For ratio a:b, symmetric Lissajous figures occur at phase multiples
 * of π / max(a,b).  Drift is reduced to DWELL_SPEED × base within
 * DWELL_WIDTH of each symmetric phase, then linearly ramps back to 1×.
 */
static float eff_drift(const Scene *s)
{
    float fx  = k_ratio[s->ratio][0];
    float fy  = k_ratio[s->ratio][1];
    float kp  = (float)M_PI / fmaxf(fx, fy);   /* key phase period */
    float pm  = fmodf(s->phase_x, kp);
    float d   = fminf(pm, kp - pm);             /* dist to nearest key */
    float frac = d / (kp * DWELL_WIDTH);
    float mul  = DWELL_SPEED + (1.0f - DWELL_SPEED) * fminf(frac, 1.0f);
    return s->drift * mul;
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    s->phase_x += eff_drift(s);
    if (s->phase_x >= 2.0f * (float)M_PI) {
        s->phase_x -= 2.0f * (float)M_PI;
        s->ratio    = (s->ratio + 1) % N_RATIOS;
    }
}

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

/*
 * Draw the full harmonograph spiral for the current phase.
 *
 * T_MAX = N_LOOPS × 2π / min(fx,fy)  — exactly N_LOOPS cycles of the
 * slower oscillator, regardless of ratio.
 *
 * DECAY = DECAY_TOTAL / T_MAX  — amplitude always reaches ~1% at T_MAX.
 *
 * age = i / (N−1) where i=0 is the outer/newest sample (t=0, full amp)
 * and i=N−1 is the inner/oldest (t=T_MAX, ~0 amp).  Oldest drawn first
 * so newest overwrites on shared cells.
 */
static void scene_draw(const Scene *s, int rows, int cols)
{
    float fx    = k_ratio[s->ratio][0];
    float fy    = k_ratio[s->ratio][1];
    float t_max = (float)N_LOOPS * 2.0f * (float)M_PI / fminf(fx, fy);
    float decay = DECAY_TOTAL / t_max;

    /* Aspect-correct radii: figure fills screen without distortion */
    float max_rx = (cols - 2) * 0.5f;
    float max_ry = (rows - 2) * 0.5f;
    float ry = fminf(max_rx / CELL_ASPECT, max_ry) * AMP_FRAC;
    float rx = ry * CELL_ASPECT;
    float cx = cols  * 0.5f;
    float cy = (rows - 1) * 0.5f;   /* -1 for HUD row */

    for (int i = N_CURVE_PTS - 1; i >= 0; i--) {
        float t   = (float)i / (float)(N_CURVE_PTS - 1) * t_max;
        float amp = expf(-decay * t);
        if (amp < 0.01f) continue;

        float x = amp * sinf(fx * t + s->phase_x);
        float y = amp * sinf(fy * t);

        int c = (int)(cx + x * rx + 0.5f);
        int r = (int)(cy + y * ry + 0.5f);
        if (r < 0 || r >= rows - 1 || c < 0 || c >= cols) continue;

        /*
         * age 0 (i=0, outer, newest) → level 0 (brightest '#')
         * age 1 (i=N-1, inner, oldest) → level 3 (dimmest '.')
         */
        float age = (float)i / (float)(N_CURVE_PTS - 1);
        int   lev = (int)(age * (float)N_LEVELS);
        if (lev >= N_LEVELS) lev = N_LEVELS - 1;

        char   ch   = k_lev_ch[lev];
        attr_t attr = COLOR_PAIR(CP_IDX(s->theme, lev));
        if (lev < 2) attr |= A_BOLD;

        attron(attr);
        mvaddch(r, c, (chtype)(unsigned char)ch);
        attroff(attr);
    }
}

static void scene_hud(const Scene *s, int rows)
{
    float phase_pi = s->phase_x / (float)M_PI;   /* display as multiples of π */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows - 1, 0,
             " %-16s  phase=%.2fπ  drift=%.4f  %s"
             "  n/p:ratio  +/-:drift  c:theme  spc:pause  q:quit",
             k_ratio_names[s->ratio], phase_pi,
             s->drift, k_theme_names[s->theme]);
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

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

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

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

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Scene s = {
        .ratio   = 0,
        .phase_x = 0.0f,
        .drift   = DRIFT_DEFAULT,
        .theme   = 0,
        .paused  = false,
    };

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            erase();
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': g_running = 0;                               break;
            case ' ':           s.paused ^= 1;                               break;
            case 'c': case 'C': s.theme = (s.theme + 1) % N_THEMES;         break;
            case 'n': case 'N':
                s.ratio   = (s.ratio + 1) % N_RATIOS;
                s.phase_x = 0.0f;
                break;
            case 'p': case 'P':
                s.ratio   = (s.ratio + N_RATIOS - 1) % N_RATIOS;
                s.phase_x = 0.0f;
                break;
            case '+': case '=':
                s.drift *= DRIFT_STEP;
                if (s.drift > DRIFT_MAX) s.drift = DRIFT_MAX;
                break;
            case '-': case '_':
                s.drift /= DRIFT_STEP;
                if (s.drift < DRIFT_MIN) s.drift = DRIFT_MIN;
                break;
            }
        }

        scene_tick(&s);
        erase();
        scene_draw(&s, rows, cols);
        scene_hud(&s, rows);
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }

    return 0;
}
