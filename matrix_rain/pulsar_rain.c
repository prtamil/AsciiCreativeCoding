/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pulsar_rain.c  —  Pulsar Star
 *
 * Models a rotating neutron star (pulsar): two opposing beams sweep
 * continuously around a central core like a lighthouse.  As each beam
 * rotates it leaves an angular wake of fading matrix characters behind it.
 *
 * Visual structure
 * ----------------
 *   CORE    — single '@' at screen centre, always drawn on top.
 *   BEAM    — two bright radial lines 180° apart, sweeping together.
 *   WAKE    — WAKE_LEN angular slots trailing each beam; characters fade
 *             from HOT → BRIGHT → MID → DARK → FADE as the angular
 *             distance from the beam head grows.
 *
 * Physics
 * -------
 *   beam_angle advances by spin radians every tick.
 *   Render interpolation:
 *     draw_angle = beam_angle + spin * alpha
 *   projects the beam to its exact fractional-tick position so rotation
 *   is smooth at 60 fps even with a 20 Hz physics rate.
 *
 *   For each beam, WAKE_LEN+1 direction vectors are pre-computed once
 *   per draw call (only 34 trig calls total for both beams).  Each of
 *   N_RADII radial samples is then placed using pre-computed cos/sin —
 *   no per-cell trig.
 *
 *   The char cache (N_RADII × (WAKE_LEN+1)) is refreshed ~75% per tick
 *   so every character shimmers as the beam sweeps past.
 *
 *   Draw order within each beam: wake slots drawn dim→bright (k descending
 *   to k=0) so the head always wins at any cell where inner slots overlap.
 *   Core '@' drawn last, always on top of both beams.
 *
 * Keys:
 *   q / ESC    quit
 *   t          cycle theme
 *   + / =      spin faster
 *   -          spin slower
 *   ]          add a beam    (1 → 16, evenly spaced at 360°/N)
 *   [          remove a beam
 *   r          reset
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/pulsar_rain.c -o pulsar_rain -lncurses -lm
 *
 * §1  config
 * §2  clock
 * §3  theme / color
 * §4  pulsar  — beam + wake + core
 * §5  screen
 * §6  app / main
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ================================================================ */
/* §1  config                                                        */
/* ================================================================ */

enum {
    N_RADII    = 80,   /* radial samples along each beam           */
    WAKE_LEN   = 16,   /* angular wake depth (slots behind head)   */
    SIM_FPS    = 20,
    RENDER_FPS = 60,
};

/*
 * WAKE_STEP  — angular gap between wake slots (radians).
 *   At r = 20 cells, one slot spans 20 × 0.05 = 1 cell → solid coverage.
 *   Total wake arc = WAKE_LEN × WAKE_STEP = 16 × 0.05 = 0.8 rad ≈ 46°.
 */
#define WAKE_STEP   0.05f

/*
 * Spin rates in radians / tick (1 tick = 1/SIM_FPS seconds).
 *   SPIN_DEF = 0.15 rad/tick × 20 Hz = 3.0 rad/s ≈ 0.48 rot/s.
 */
#define SPIN_MIN    0.03f
#define SPIN_DEF    0.15f
#define SPIN_MAX    0.60f
#define SPIN_STEP   0.03f

/* Beam count range.  16 beams = 22.5° apart; wakes overlap and merge. */
#define BEAMS_MIN   1
#define BEAMS_DEF   2
#define BEAMS_MAX  16

/*
 * ASPECT — terminal cell height/width ratio correction.
 * Baking it into sin_a keeps every position formula as:
 *   col = cx + r * cos_a
 *   row = cy + r * sin_a     (sin_a = sin(θ) × ASPECT)
 */
#define ASPECT   0.45f

#ifndef M_PI
#define M_PI     3.14159265358979323846
#endif

#define NSPS     1000000000LL
#define NSPM     1000000LL
#define TICK_NS  (NSPS / SIM_FPS)
#define FRAME_NS (NSPS / RENDER_FPS)

/* ================================================================ */
/* §2  clock                                                         */
/* ================================================================ */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NSPS + (int64_t)ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = {
        .tv_sec  = (time_t)(ns / NSPS),
        .tv_nsec = (long)  (ns % NSPS),
    };
    nanosleep(&r, NULL);
}

/* ================================================================ */
/* §3  theme / color                                                 */
/* ================================================================ */

enum {
    CP_FADE   = 1,
    CP_DARK   = 2,
    CP_MID    = 3,
    CP_BRIGHT = 4,
    CP_HOT    = 5,
    CP_HEAD   = 6,   /* beam head — white bold              */
    CP_CORE   = 7,   /* centre '@'                          */
    CP_HUD    = 8,
};

typedef struct { const char *name; int fg[5]; } Theme;

static const Theme k_themes[] = {
    { "green",  {  22,  28,  34,  40,  82 } },
    { "amber",  {  94, 130, 172, 214, 220 } },
    { "blue",   {  17,  19,  21,  33,  51 } },
    { "plasma", {  53,  57,  93, 129, 201 } },
    { "fire",   {  52,  88, 124, 160, 196 } },
};
#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static const int k_fallback[5][5] = {
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   },
    { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW  },
    { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN    },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA },
    { COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED     },
};

static bool g_has_256;

static void theme_apply(int idx)
{
    const int *fg = g_has_256 ? k_themes[idx].fg : k_fallback[idx];
    init_pair(CP_FADE,   fg[0], COLOR_BLACK);
    init_pair(CP_DARK,   fg[1], COLOR_BLACK);
    init_pair(CP_MID,    fg[2], COLOR_BLACK);
    init_pair(CP_BRIGHT, fg[3], COLOR_BLACK);
    init_pair(CP_HOT,    fg[4], COLOR_BLACK);
    init_pair(CP_HEAD,   COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_CORE,   COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_HUD,    COLOR_WHITE, COLOR_BLACK);
}

/*
 * wake_attr() — shade by angular distance k from the beam head.
 * k = 0 is the leading edge; k = WAKE_LEN is the trailing tail.
 */
static attr_t wake_attr(int k)
{
    if (k == 0)             return COLOR_PAIR(CP_HEAD)   | A_BOLD;
    if (k == 1)             return COLOR_PAIR(CP_HOT)    | A_BOLD;
    if (k == 2)             return COLOR_PAIR(CP_BRIGHT) | A_BOLD;
    if (k <= WAKE_LEN / 2) return COLOR_PAIR(CP_MID);
    if (k <= WAKE_LEN - 2) return COLOR_PAIR(CP_DARK);
    return COLOR_PAIR(CP_FADE) | A_DIM;
}

/* ================================================================ */
/* §4  pulsar                                                        */
/* ================================================================ */

static const char k_charset[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";
#define CHARSET_LEN (int)(sizeof k_charset - 1)

static char rand_ch(void) { return k_charset[rand() % CHARSET_LEN]; }

/*
 * Pulsar — full simulation state.
 *
 * angle   : current beam angle (radians), advances by spin each tick.
 * spin    : rotation rate (radians / tick).
 * cache   : [N_RADII][WAKE_LEN+1] character cache.
 *           cache[ri][0]  = char displayed at the beam head at radius ri.
 *           cache[ri][k]  = char k angular slots behind the head.
 *           Refreshed ~75% per tick (shimmer).
 * max_r   : radius of the farthest screen corner from centre (isotropic).
 *           Used to space N_RADII samples across the full beam length.
 */
typedef struct {
    float angle;
    float spin;
    int   n_beams;               /* number of beams, evenly spaced at 2π/n */
    char  cache[N_RADII][WAKE_LEN + 1];
    float max_r;
    float r_step;     /* max_r / N_RADII, cached */
    int   cx, cy;
} Pulsar;

static void pulsar_init(Pulsar *p, int cols, int rows, float spin, int n_beams)
{
    memset(p, 0, sizeof *p);
    p->cx      = cols / 2;
    p->cy      = rows / 2;
    p->spin    = spin;
    p->angle   = 0.0f;
    p->n_beams = n_beams;

    /* Distance from centre to farthest corner in isotropic cell units */
    float hx = (float)cols * 0.5f;
    float hy = (float)rows * 0.5f / ASPECT;
    p->max_r  = sqrtf(hx * hx + hy * hy) * 1.05f;
    p->r_step = p->max_r / (float)N_RADII;

    for (int ri = 0; ri < N_RADII; ri++)
        for (int k = 0; k <= WAKE_LEN; k++)
            p->cache[ri][k] = rand_ch();
}

static void pulsar_tick(Pulsar *p)
{
    p->angle += p->spin;
    if (p->angle > 2.0f * (float)M_PI)
        p->angle -= 2.0f * (float)M_PI;

    /* Shimmer: refresh ~75% of cache characters each tick */
    for (int ri = 0; ri < N_RADII; ri++)
        for (int k = 0; k <= WAKE_LEN; k++)
            if (rand() % 4 != 0) p->cache[ri][k] = rand_ch();
}

/*
 * draw_beam() — draw one rotating beam + its angular wake.
 *
 * Algorithm:
 *   1. Pre-compute direction vectors for the WAKE_LEN+1 angular slots.
 *      Each slot at angle (base − k × WAKE_STEP), baking ASPECT into sin.
 *      Only WAKE_LEN+1 = 17 trig pairs needed per beam, regardless of
 *      how many radial samples are drawn.
 *
 *   2. For each radial sample ri, draw slots k = WAKE_LEN..0 (dim→bright).
 *      Descending order so that at very small r (where multiple slots
 *      round to the same cell), the brighter head slot (k=0) wins by
 *      being drawn LAST.
 */
static void draw_beam(const Pulsar *p, float base_angle, int cols, int rows)
{
    /* Pre-compute direction vectors for all WAKE_LEN+1 angular slots */
    float cw[WAKE_LEN + 1], sw[WAKE_LEN + 1];
    for (int k = 0; k <= WAKE_LEN; k++) {
        float wa = base_angle - (float)k * WAKE_STEP;
        cw[k] = cosf(wa);
        sw[k] = sinf(wa) * ASPECT;
    }

    /*
     * Draw dim tail → bright head so the head wins at any cell
     * shared by multiple slots (happens near centre where WAKE_STEP
     * is smaller than one cell width).
     */
    for (int ri = 0; ri < N_RADII; ri++) {
        float r = (float)(ri + 1) * p->r_step;

        for (int k = WAKE_LEN; k >= 0; k--) {
            int col = p->cx + (int)roundf(r * cw[k]);
            int row = p->cy + (int)roundf(r * sw[k]);
            if (col < 0 || col >= cols || row < 0 || row >= rows) continue;

            attr_t attr = wake_attr(k);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)p->cache[ri][k]);
            attroff(attr);
        }
    }
}

static void pulsar_draw(const Pulsar *p, float alpha, int cols, int rows)
{
    /*
     * Project beam angle forward by (spin × alpha) for sub-tick smoothness.
     * Because spin is constant within a tick, forward extrapolation is exact.
     */
    float draw_angle = p->angle + p->spin * alpha;

    /*
     * N beams evenly spaced at 2π/N radians apart.
     *   N=1 → single sweeping beam
     *   N=2 → classic pulsar pair (0° and 180°)
     *   N=3 → tri-blade (0°, 120°, 240°)
     *   N=4 → cross (0°, 90°, 180°, 270°)
     *   …
     * Beams drawn 0..N-1; later beams overwrite earlier ones at shared
     * cells, which is fine since all share the same brightness gradient.
     */
    float step = 2.0f * (float)M_PI / (float)p->n_beams;
    for (int b = 0; b < p->n_beams; b++)
        draw_beam(p, draw_angle + (float)b * step, cols, rows);

    /* Core '@' drawn LAST — always wins over all beams */
    if (p->cx >= 0 && p->cx < cols && p->cy >= 0 && p->cy < rows) {
        attron(COLOR_PAIR(CP_CORE) | A_BOLD);
        mvaddch(p->cy, p->cx, '@');
        attroff(COLOR_PAIR(CP_CORE) | A_BOLD);
    }
}

/* ================================================================ */
/* §5  screen                                                        */
/* ================================================================ */

static void screen_init(int *cols, int *rows)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    start_color();
    use_default_colors();
    getmaxyx(stdscr, *rows, *cols);
}

static void screen_resize(int *cols, int *rows)
{
    endwin();
    refresh();
    getmaxyx(stdscr, *rows, *cols);
}

static void screen_hud(int cols, double fps, float spin, int n_beams,
                        int theme_idx)
{
    /* Display spin as rotations-per-second so it reads physically */
    double rps = (double)spin * SIM_FPS / (2.0 * M_PI);
    char buf[80];
    snprintf(buf, sizeof buf, " %.0ffps  %.2f rps  %dB  [%s]  +/- [ ] t r q ",
             fps, rps, n_beams, k_themes[theme_idx].name);
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void cleanup(void) { endwin(); }

/* ================================================================ */
/* §6  app / main                                                    */
/* ================================================================ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running     = 0;
}

int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    int cols, rows;
    screen_init(&cols, &rows);
    g_has_256 = (COLORS >= 256);

    int theme_idx = 0;
    theme_apply(theme_idx);

    float spin    = SPIN_DEF;
    int   n_beams = BEAMS_DEF;
    Pulsar psr;
    pulsar_init(&psr, cols, rows, spin, n_beams);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_disp    = 0.0;

    while (g_running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (g_need_resize) {
            screen_resize(&cols, &rows);
            pulsar_init(&psr, cols, rows, spin, n_beams);
            frame_time    = clock_ns();
            sim_accum     = 0;
            g_need_resize = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 150 * NSPM) dt = 150 * NSPM;

        /* ── physics accumulator ─────────────────────────────────── */
        sim_accum += dt;
        while (sim_accum >= TICK_NS) {
            pulsar_tick(&psr);
            sim_accum -= TICK_NS;
        }

        /*
         * alpha = sim_accum / TICK_NS ∈ [0, 1)
         * Passed to pulsar_draw() which adds spin×alpha to beam_angle,
         * projecting the rotation to the exact sub-tick elapsed time.
         */
        float alpha = (float)sim_accum / (float)TICK_NS;

        /* ── fps counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NSPM) {
            fps_disp    = (double)frame_count
                        / ((double)fps_accum / (double)NSPS);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap ───────────────────────────────────────────── */
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));

        /* ── draw ────────────────────────────────────────────────── */
        erase();
        pulsar_draw(&psr, alpha, cols, rows);
        screen_hud(cols, fps_disp, psr.spin, psr.n_beams, theme_idx);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27:
            g_running = 0;
            break;
        case 't': case 'T':
            theme_idx = (theme_idx + 1) % THEME_COUNT;
            theme_apply(theme_idx);
            break;
        case '+': case '=':
            spin += SPIN_STEP;
            if (spin > SPIN_MAX) spin = SPIN_MAX;
            psr.spin = spin;
            break;
        case '-':
            spin -= SPIN_STEP;
            if (spin < SPIN_MIN) spin = SPIN_MIN;
            psr.spin = spin;
            break;
        case ']':
            n_beams++;
            if (n_beams > BEAMS_MAX) n_beams = BEAMS_MAX;
            psr.n_beams = n_beams;
            break;
        case '[':
            n_beams--;
            if (n_beams < BEAMS_MIN) n_beams = BEAMS_MIN;
            psr.n_beams = n_beams;
            break;
        case 'r': case 'R':
            pulsar_init(&psr, cols, rows, spin, n_beams);
            break;
        default:
            break;
        }
    }

    return 0;
}
