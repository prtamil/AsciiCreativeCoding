/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * galaxy.c — Spiral Galaxy Simulation
 *
 * Stars orbit a central mass with a flat rotation curve (every star has the
 * same tangential speed regardless of radius, just like real galaxies).
 * Because inner stars complete orbits faster than outer ones (differential
 * rotation), the arms gradually wind up — exactly what happens in real spiral
 * galaxies over hundreds of millions of years.
 *
 * Stars are NOT simulated with mutual gravity — each moves in a smooth
 * circular orbit.  The spiral structure emerges purely from the initial
 * placement on logarithmic spiral arms and the differential rotation.
 *
 * Character key (density → glyph):
 *   .  ,  :  o  O  0  @     sparse → dense
 *
 * Colour key (radial zone):
 *   CORE  — bright bulge (white / yellow-white)
 *   DISK  — spiral arms (cyan / blue)
 *   HALO  — outer disc  (grey / dim)
 *
 * Keys:
 *   q/ESC   quit         p/Space  pause/resume     r  reset current arms
 *   a       more arms (2→3→4→2)   A  fewer arms
 *   t/T     next/prev theme        +/-  orbit speed faster/slower
 *   ]/[     FPS up/down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/galaxy.c -o galaxy -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 galaxy  §5 draw  §6 screen  §7 app
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define ROWS_MAX      128
#define COLS_MAX      512
#define N_STARS       3000   /* total stars */
#define ARMS_DEF      2      /* starting arm count */
#define ARMS_MIN      2
#define ARMS_MAX      4
#define N_THEMES      5

#define STEPS_DEF     2      /* physics steps per rendered frame */
#define STEPS_MIN     1
#define STEPS_MAX    16
#define SIM_FPS_DEF  20
#define SIM_FPS_MIN   5
#define SIM_FPS_MAX  60
#define SIM_FPS_STEP  5

/*
 * Flat rotation curve: v_circ = V0 (constant for all r).
 * Angular velocity: omega(r) = V0 / r  →  inner orbits faster.
 * V0 is in units of (normalized_radius / step).
 */
#define V0_DEF        0.006f

/* Brightness accumulator decays by this factor once per rendered frame */
#define DECAY         0.82f

/* Log-spiral tightness (higher = more wound) */
#define WINDING       1.0f

/* Angular scatter around each arm (rad, ±half-width) */
#define ARM_SCATTER   0.25f

#define SPEED_DEF     1.0f
#define SPEED_MIN     0.1f
#define SPEED_MAX     5.0f
#define SPEED_STEP    0.1f

#define NS_PER_SEC    1000000000LL
#define TICK_NS(f)    (NS_PER_SEC / (f))

static const float PI = 3.14159265358979f;

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                            .tv_nsec = (long)(ns % NS_PER_SEC) };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * Three radial colour zones:
 *   CP_CORE — bright stellar bulge (innermost ~10% of radius)
 *   CP_DISK — spiral arms and disc  (10%..65% of radius)
 *   CP_HALO — outer diffuse halo    (> 65% of radius)
 *
 * Character encodes local density; colour encodes radial position.
 */
enum { CP_CORE=1, CP_DISK, CP_HALO, CP_HUD };

typedef struct {
    short core256, disk256, halo256;   /* 256-colour fg on black bg */
    short core8,   disk8,   halo8;     /* 8-colour fg fallback */
    const char *name;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0 MilkyWay  — white core, cyan arms, grey halo */
    { 231,  39, 240,  COLOR_WHITE, COLOR_CYAN,    COLOR_WHITE,   "MilkyWay" },
    /* 1 Starburst — yellow core, blue arms, dark grey halo */
    { 226,  33, 238,  COLOR_YELLOW, COLOR_BLUE,   COLOR_WHITE,   "Starburst" },
    /* 2 Nebula    — white core, pink arms, purple halo */
    { 231, 207,  92,  COLOR_WHITE,  COLOR_MAGENTA, COLOR_MAGENTA, "Nebula" },
    /* 3 Infrared  — white core, red arms, dark-red halo */
    { 231, 196,  52,  COLOR_WHITE,  COLOR_RED,     COLOR_RED,     "Infrared" },
    /* 4 Aurora    — white core, bright-green arms, dark-green halo */
    { 231,  46,  22,  COLOR_WHITE,  COLOR_GREEN,   COLOR_GREEN,   "Aurora" },
};

static bool g_has_256;
static int  g_theme = 0;

static void theme_apply(int ti)
{
    const Theme *t = &k_themes[ti];
    if (g_has_256) {
        init_pair(CP_CORE, t->core256, COLOR_BLACK);
        init_pair(CP_DISK, t->disk256, COLOR_BLACK);
        init_pair(CP_HALO, t->halo256, COLOR_BLACK);
    } else {
        init_pair(CP_CORE, t->core8, COLOR_BLACK);
        init_pair(CP_DISK, t->disk8, COLOR_BLACK);
        init_pair(CP_HALO, t->halo8, COLOR_BLACK);
    }
    init_pair(CP_HUD, g_has_256 ? 255 : COLOR_WHITE,
                      g_has_256 ? 236 : COLOR_BLACK);
}

/* ===================================================================== */
/* §4  galaxy — stars and simulation                                      */
/* ===================================================================== */

/*
 * Each star is a point mass in a fixed circular orbit.
 *   r     — orbital radius, normalised so 1.0 = edge of the display
 *   theta — current angle (radians)
 *   omega — angular velocity (rad/step) = V0 / r
 *
 * No mutual gravitational interaction; the spiral pattern comes entirely
 * from the initial placement and the radial dependence of omega.
 */
typedef struct {
    float r;
    float theta;
    float omega;
} Star;

static Star  g_stars[N_STARS];
static int   g_narms  = ARMS_DEF;
static float g_v0     = V0_DEF;
static float g_speed  = SPEED_DEF;
static int   g_rows, g_cols;
static int   g_steps  = STEPS_DEF;
static int   g_sim_fps = SIM_FPS_DEF;
static long  g_tick   = 0;
static bool  g_paused = false;

/*
 * g_bright[r][c] — accumulated star density at each screen cell.
 * Added to each step (weighted by 1/g_steps so total per frame is constant).
 * Multiplied by DECAY once per rendered frame → exponential fade trail.
 */
static float g_bright[ROWS_MAX][COLS_MAX];

/* xorshift32 RNG */
static uint32_t g_rng = 12345u;
static inline uint32_t rng_next(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng <<  5;
    return g_rng;
}
static inline float rng_float(void)
{
    return (float)(rng_next() >> 8) / (float)(1u << 24);
}

/* Box-Muller normal sample */
static float rng_gauss(void)
{
    float u1 = rng_float() + 1e-6f;
    float u2 = rng_float();
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI * u2);
}

/* ------------------------------------------------------------------ */

static void galaxy_init(int narms)
{
    g_narms = narms;
    memset(g_bright, 0, sizeof g_bright);
    g_tick = 0;

    int n = 0;

    /*
     * BULGE — 20% of stars: tightly concentrated around the centre.
     * Radius drawn from a half-Gaussian (always positive) so stars pile up
     * near r=0 to form the bright nuclear region.
     */
    int n_bulge = N_STARS / 5;
    for (int i = 0; i < n_bulge; i++) {
        float r = fabsf(rng_gauss() * 0.07f);
        if (r < 0.004f) r = 0.004f;
        if (r > 0.20f)  r = 0.20f;
        g_stars[n++] = (Star){ r, rng_float() * 2.0f * PI, g_v0 / r };
    }

    /*
     * ARMS — 70% of stars: placed on logarithmic spirals.
     * For arm k (0-indexed), the initial angle at radius r is:
     *   theta = (k * 2π/narms) + WINDING * ln(r / r_min) + noise
     * This is a logarithmic spiral: r = r_min * exp((theta - start) / WINDING)
     */
    int n_arm_total = N_STARS * 7 / 10;
    for (int i = 0; i < n_arm_total; i++) {
        int   arm   = i % narms;
        float a_off = arm * (2.0f * PI / (float)narms);
        float r     = 0.08f + rng_float() * 0.87f;   /* 0.08 .. 0.95 */
        float theta = a_off + WINDING * logf(r / 0.08f)
                      + (rng_float() - 0.5f) * (2.0f * ARM_SCATTER);
        g_stars[n++] = (Star){ r, theta, g_v0 / r };
    }

    /*
     * HALO — remaining ~10%: scattered uniformly at large radii.
     * These represent field stars and the outer diffuse disc.
     */
    while (n < N_STARS) {
        float r = 0.35f + rng_float() * 0.70f;
        if (r > 1.05f) r = 1.05f;
        g_stars[n++] = (Star){ r, rng_float() * 2.0f * PI, g_v0 / r };
    }
}

/* ------------------------------------------------------------------ */

static void galaxy_step(void)
{
    /*
     * Screen centre and scale factors.
     * ry = rx * 0.5 corrects for terminal cells being ~2× taller than wide,
     * making the galaxy appear circular rather than vertically stretched.
     */
    int   cx = g_cols / 2, cy = g_rows / 2;
    float rx = g_cols * 0.44f;
    float ry = rx * 0.50f;

    /*
     * Weight per star per step: dividing by g_steps keeps total brightness
     * per frame constant regardless of how many physics steps we compute.
     */
    float w = 1.0f / (float)g_steps;

    for (int i = 0; i < N_STARS; i++) {
        /* Advance along circular orbit */
        g_stars[i].theta += g_stars[i].omega * g_speed;

        /* Convert polar → Cartesian → screen coordinates */
        float x = g_stars[i].r * cosf(g_stars[i].theta);
        float y = g_stars[i].r * sinf(g_stars[i].theta);
        int   sx = cx + (int)(x * rx + 0.5f);
        int   sy = cy + (int)(y * ry + 0.5f);

        if (sx >= 0 && sx < g_cols && sy >= 0 && sy < g_rows)
            g_bright[sy][sx] += w;
    }
    g_tick++;
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static void scene_draw(void)
{
    /*
     * Decay the brightness grid once per rendered frame.
     * Steady-state brightness for a cell receiving f stars/frame:
     *   B_ss = f / (1 - DECAY)
     * With DECAY=0.82, B_ss = f * 5.56  →  bright core, faint halo.
     */
    for (int r = 0; r < g_rows; r++)
        for (int c = 0; c < g_cols; c++)
            g_bright[r][c] *= DECAY;

    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    int draw_rows = (g_rows < rows - 2) ? g_rows : rows - 2;
    int draw_cols = (g_cols < cols)     ? g_cols : cols;

    /*
     * Find peak brightness so we can normalise to [0,1].
     * This auto-calibrates the display for any star density or speed.
     */
    float b_max = 0.1f;
    for (int r = 0; r < draw_rows; r++)
        for (int c = 0; c < draw_cols; c++)
            if (g_bright[r][c] > b_max) b_max = g_bright[r][c];

    /* Radial colour zone boundaries (normalised galaxy radius) */
    int   cx = g_cols / 2, cy = g_rows / 2;
    float rx = g_cols * 0.44f, ry = rx * 0.50f;
    float r_core = 0.10f;    /* < 10% → core colour */
    float r_disk = 0.65f;    /* 10%..65% → disk colour; > 65% → halo */

    for (int r = 0; r < draw_rows; r++) {
        for (int c = 0; c < draw_cols; c++) {
            float b = g_bright[r][c];
            if (b < 0.02f * b_max) continue;   /* below noise floor — skip */

            /* Normalised brightness 0..1 → character */
            float t = b / b_max;
            char  ch;
            int   bold = 0;
            if      (t < 0.12f) ch = '.';
            else if (t < 0.25f) ch = ',';
            else if (t < 0.40f) ch = ':';
            else if (t < 0.55f) ch = 'o';
            else if (t < 0.70f) ch = 'O';
            else if (t < 0.85f) { ch = '0'; bold = A_BOLD; }
            else                { ch = '@'; bold = A_BOLD; }

            /* Normalised screen radius → colour zone */
            float dx = (float)(c - cx) / rx;
            float dy = (float)(r - cy) / ry;
            float rn = sqrtf(dx*dx + dy*dy);
            int cp = (rn < r_core) ? CP_CORE
                   : (rn < r_disk) ? CP_DISK
                                   : CP_HALO;

            attron(COLOR_PAIR(cp) | bold);
            mvaddch(r, c, (chtype)ch);
            attroff(COLOR_PAIR(cp) | bold);
        }
    }

    /* Galactic centre — always mark even when no stars land exactly here */
    if (cy < draw_rows && cx < draw_cols) {
        attron(COLOR_PAIR(CP_CORE) | A_BOLD);
        mvaddch(cy, cx, '*');
        attroff(COLOR_PAIR(CP_CORE) | A_BOLD);
    }

    /* ── HUD row 1 ── */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(rows - 2, 0,
        " Spiral Galaxy  arms:%d  speed:%.1fx  theme:%-10s"
        "  tick:%-6ld  fps:%d  steps:%d  %s",
        g_narms, g_speed, k_themes[g_theme].name,
        g_tick, g_sim_fps, g_steps,
        g_paused ? "[PAUSED]" : "inner orbits faster -- watch arms wind up");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* ── HUD row 2 ── */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows - 1, 0,
        "  [a/A]arms  [+/-]speed  []/[]fps  [t/T]theme  "
        "[r]reset  [p]pause  [q]quit   "
        "Chars: .=sparse  @=dense   Colour: CORE/DISK/HALO");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void handle_sigwinch(int s) { (void)s; g_resize = 1; }
static void handle_sigterm (int s) { (void)s; g_quit   = 1; }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    start_color();
    g_has_256 = (COLORS >= 256);
    theme_apply(g_theme);
}

static void screen_resize(void)
{
    endwin(); refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
    g_cols = (cols      < COLS_MAX) ? cols      : COLS_MAX;
    g_resize = 0;
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGTERM,  handle_sigterm);
    signal(SIGINT,   handle_sigterm);

    g_rng = (uint32_t)time(NULL) ^ 0xC0DE4A1Au;
    screen_init();

    { int rows, cols; getmaxyx(stdscr, rows, cols);
      g_rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
      g_cols = (cols      < COLS_MAX) ? cols      : COLS_MAX; }

    galaxy_init(g_narms);
    int64_t next_tick = clock_ns();

    while (!g_quit) {

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_quit = 1; break;
            case 'p': case ' ': g_paused = !g_paused; break;
            case 'r': galaxy_init(g_narms); break;

            case 'a':
                g_narms = (g_narms < ARMS_MAX) ? g_narms + 1 : ARMS_MIN;
                galaxy_init(g_narms);
                break;
            case 'A':
                g_narms = (g_narms > ARMS_MIN) ? g_narms - 1 : ARMS_MAX;
                galaxy_init(g_narms);
                break;

            case 't':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply(g_theme);
                break;
            case 'T':
                g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
                theme_apply(g_theme);
                break;

            case '+': case '=':
                if (g_speed < SPEED_MAX - 0.01f) g_speed += SPEED_STEP;
                break;
            case '-':
                if (g_speed > SPEED_MIN + 0.01f) g_speed -= SPEED_STEP;
                break;

            case ']':
                if (g_sim_fps < SIM_FPS_MAX) g_sim_fps += SIM_FPS_STEP;
                break;
            case '[':
                if (g_sim_fps > SIM_FPS_MIN) g_sim_fps -= SIM_FPS_STEP;
                break;

            case '>':
                if (g_steps < STEPS_MAX) g_steps++;
                break;
            case '<':
                if (g_steps > STEPS_MIN) g_steps--;
                break;
            }
        }

        /* ── resize ── */
        if (g_resize) { screen_resize(); galaxy_init(g_narms); }

        /* ── simulate ── */
        int64_t now = clock_ns();
        if (!g_paused && now >= next_tick) {
            for (int s = 0; s < g_steps; s++) galaxy_step();
            next_tick = now + TICK_NS(g_sim_fps);
        }

        /* ── render ── */
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();

        /* ── sleep until next frame ── */
        clock_sleep_ns(next_tick - clock_ns() - 1000000LL);
    }

    endwin();
    return 0;
}
