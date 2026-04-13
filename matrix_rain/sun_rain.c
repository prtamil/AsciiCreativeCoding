/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sun_rain.c  —  Matrix Rain Sun
 *
 * A single '@' burns at the screen centre.  180 independent radial
 * streams of matrix characters shoot outward from it in all directions
 * — continuous solar wind with no circle, no disc, no border.
 *
 * Each stream has a random stagger offset so they appear at different
 * distances from the core at any given moment, producing a stochastic
 * field of beams rather than a single synchronised burst.
 *
 * Coordinate system
 * -----------------
 *   r_off is in isotropic cell units (1 unit = 1 column width).
 *   y-coordinates are compressed by ASPECT = 0.45 so streams at
 *   oblique angles trace visually equal distances horizontally and
 *   vertically:
 *
 *     screen_col = cx + r * cos(θ)
 *     screen_row = cy + r * sin(θ) * ASPECT
 *
 *   cos_a and sin_a (= sin(θ) × ASPECT) are baked in at ray init,
 *   so every position computation is just:
 *     col = cx + ri * cos_a
 *     row = cy + ri * sin_a
 *
 * Render interpolation
 * --------------------
 *   draw_r_off = r_off + speed × alpha
 *
 *   alpha = sim_accum / TICK_NS ∈ [0, 1) — sub-tick fraction.
 *   Because each ray travels at constant speed, forward extrapolation
 *   is exact.  At 20 Hz physics / 60 Hz render, motion stays smooth.
 *
 * Keys:  q/ESC quit   t theme   r reset
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/sun_rain.c -o sun_rain -lncurses -lm
 *
 * §1  config
 * §2  clock
 * §3  theme / color
 * §4  solar ray — one radial stream from centre
 * §5  sun       — 180 rays + core character
 * §6  screen
 * §7  app / main
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

/* ================================================================== */
/* §1  config                                                          */
/* ================================================================== */

enum {
    N_RAYS      = 180,  /* radial streams — one every 2°            */
    RAY_TRAIL   =  16,  /* characters per ray tail                  */
    SIM_FPS     =  20,
    RENDER_FPS  =  60,
};

/*
 * ASPECT — corrects for terminal cells being taller than wide.
 * Standard cell ≈ 8 × 16 px → raw ratio = 0.5; 0.45 gives a
 * slightly rounder spread on most terminals.
 */
#define ASPECT   0.45f

#ifndef M_PI
#define M_PI     3.14159265358979323846
#endif

#define NSPS     1000000000LL
#define NSPM     1000000LL
#define TICK_NS  (NSPS / SIM_FPS)
#define FRAME_NS (NSPS / RENDER_FPS)

/* ================================================================== */
/* §2  clock                                                           */
/* ================================================================== */

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

/* ================================================================== */
/* §3  theme / color                                                   */
/* ================================================================== */

enum {
    CP_FADE   = 1,   /* dimmest tail end              */
    CP_DARK   = 2,
    CP_MID    = 3,
    CP_BRIGHT = 4,
    CP_HOT    = 5,   /* one cell behind head          */
    CP_HEAD   = 6,   /* stream head (white)           */
    CP_CORE   = 7,   /* centre @ character            */
    CP_HUD    = 8,
};

typedef struct {
    const char *name;
    int fg[5];           /* FADE / DARK / MID / BRIGHT / HOT */
} Theme;

static const Theme k_themes[] = {
    { "solar",  { 130, 166, 202, 214, 220 } },   /* amber / gold  */
    { "green",  {  22,  28,  34,  40,  82 } },
    { "nova",   {  17,  33,  51, 159, 255 } },
    { "plasma", {  53,  57,  93, 129, 201 } },
    { "fire",   {  52,  88, 124, 160, 196 } },
};
#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static const int k_fallback[5][5] = {
    { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW  },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   },
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
 * dist_attr() — 6-level shade gradient by distance from stream head.
 * Same scheme as matrix_rain.c and pulsar_rain.c.
 */
static attr_t dist_attr(int dist, int tlen)
{
    if (dist == 0)          return COLOR_PAIR(CP_HEAD)   | A_BOLD;
    if (dist == 1)          return COLOR_PAIR(CP_HOT)    | A_BOLD;
    if (dist == 2)          return COLOR_PAIR(CP_BRIGHT) | A_BOLD;
    if (dist <= tlen / 2)  return COLOR_PAIR(CP_MID);
    if (dist <= tlen - 2)  return COLOR_PAIR(CP_DARK);
    return COLOR_PAIR(CP_FADE) | A_DIM;
}

/* ================================================================== */
/* §4  solar ray                                                       */
/* ================================================================== */

static const char k_charset[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";
#define CHARSET_LEN (int)(sizeof k_charset - 1)

static char rand_ch(void) { return k_charset[rand() % CHARSET_LEN]; }

/*
 * SolarRay — one continuous radial stream from the screen centre.
 *
 * r_off: signed distance from centre to the ray's head.
 *   r_off < 0  →  pre-emergence (stagger delay, not yet visible).
 *   r_off ≥ 1  →  head is at r_off cells from centre; tail extends
 *                  inward.  Trail stops at ri < 1.0 to preserve the
 *                  centre cell for the '@' core character.
 *
 * cos_a, sin_a: baked direction components (sin_a includes ASPECT).
 *   col = cx + ri * cos_a
 *   row = cy + ri * sin_a
 */
typedef struct {
    float cos_a, sin_a;
    float r_off;            /* head distance from centre            */
    float speed;            /* cells / tick                         */
    char  cache[RAY_TRAIL];
} SolarRay;

static void solar_ray_reset(SolarRay *ray, float angle, float max_r)
{
    ray->cos_a = cosf(angle);
    ray->sin_a = sinf(angle) * ASPECT;

    /*
     * Stagger: start at a random negative offset so all 180 rays
     * don't emerge simultaneously.  Range: 0 to −55% of max_r.
     */
    int stagger_range = (int)(max_r * 0.55f) + 2;
    ray->r_off = -(float)(rand() % stagger_range);
    ray->speed = 1.5f + (float)(rand() % 26) * 0.1f;  /* 1.5 – 4.0 */
    for (int i = 0; i < RAY_TRAIL; i++) ray->cache[i] = rand_ch();
}

/*
 * solar_ray_tick() — advance one physics step.
 * Returns false when the tail has cleared the screen (ray is dead).
 */
static bool solar_ray_tick(SolarRay *ray, float max_r)
{
    ray->r_off += ray->speed;
    for (int i = 0; i < RAY_TRAIL; i++)
        if (rand() % 4 != 0) ray->cache[i] = rand_ch();   /* shimmer */
    return (ray->r_off - (float)RAY_TRAIL) < max_r;
}

/*
 * solar_ray_draw() — paint one ray at the alpha-interpolated position.
 *
 *   ri = draw_r_off − i      (i = 0 is head, i = RAY_TRAIL−1 is tail)
 *
 * Trail stops at ri < 1.0 so the centre cell is never overwritten —
 * the '@' drawn last in sun_draw() always stays on top.
 */
static void solar_ray_draw(const SolarRay *ray, float draw_r_off,
                            int cx, int cy, int cols, int rows)
{
    for (int i = 0; i < RAY_TRAIL; i++) {
        float ri = draw_r_off - (float)i;
        if (ri < 1.0f) break;      /* leave centre cell for '@'   */

        int col = cx + (int)roundf(ri * ray->cos_a);
        int row = cy + (int)roundf(ri * ray->sin_a);
        if (col < 0 || col >= cols || row < 0 || row >= rows) continue;

        attr_t attr = dist_attr(i, RAY_TRAIL);
        attron(attr);
        mvaddch(row, col, (chtype)(unsigned char)ray->cache[i]);
        attroff(attr);
    }
}

/* ================================================================== */
/* §5  sun                                                             */
/* ================================================================== */

typedef struct {
    SolarRay rays[N_RAYS];
    float    max_r;       /* upper-bound travel distance for any ray */
    int      cx, cy;      /* screen centre (cells)                   */
} Sun;

static void sun_init(Sun *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cx    = cols / 2;
    s->cy    = rows / 2;

    /*
     * max_r: conservative upper bound so rays travel until their tails
     * clear even the farthest corner from the centre.
     *
     *   Horizontal ray:   must reach cx or cols−cx   ≤ cols
     *   Vertical ray:     must reach cy/ASPECT        ≤ rows/ASPECT
     *   Diagonal: bounded by cols + rows/ASPECT
     */
    s->max_r = (float)cols + (float)rows * (1.0f / ASPECT);

    for (int i = 0; i < N_RAYS; i++) {
        float angle = (float)i * (2.0f * (float)M_PI / (float)N_RAYS);
        solar_ray_reset(&s->rays[i], angle, s->max_r);
    }
}

static void sun_tick(Sun *s)
{
    for (int i = 0; i < N_RAYS; i++) {
        float angle = (float)i * (2.0f * (float)M_PI / (float)N_RAYS);
        if (!solar_ray_tick(&s->rays[i], s->max_r))
            solar_ray_reset(&s->rays[i], angle, s->max_r);
    }
}

static void sun_draw(const Sun *s, float alpha, int cols, int rows)
{
    /* Pass 1 — 180 radial streams, drawn outward from centre */
    for (int i = 0; i < N_RAYS; i++) {
        float dr = s->rays[i].r_off + s->rays[i].speed * alpha;
        solar_ray_draw(&s->rays[i], dr, s->cx, s->cy, cols, rows);
    }

    /*
     * Pass 2 — core character, always on top.
     * Drawn last so it is never overwritten by any ray trail.
     */
    if (s->cx >= 0 && s->cx < cols && s->cy >= 0 && s->cy < rows) {
        attron(COLOR_PAIR(CP_CORE) | A_BOLD);
        mvaddch(s->cy, s->cx, '@');
        attroff(COLOR_PAIR(CP_CORE) | A_BOLD);
    }
}

/* ================================================================== */
/* §6  screen                                                          */
/* ================================================================== */

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

static void screen_hud(int cols, double fps, int theme_idx)
{
    char buf[72];
    snprintf(buf, sizeof buf, " %.0ffps  [%s]  t=theme  r=reset  q=quit ",
             fps, k_themes[theme_idx].name);
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void cleanup(void) { endwin(); }

/* ================================================================== */
/* §7  app / main                                                      */
/* ================================================================== */

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

    Sun sun;
    sun_init(&sun, cols, rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_disp    = 0.0;

    while (g_running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (g_need_resize) {
            screen_resize(&cols, &rows);
            sun_init(&sun, cols, rows);
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
            sun_tick(&sun);
            sim_accum -= TICK_NS;
        }

        /*
         * Render interpolation alpha.
         *   alpha = sim_accum / TICK_NS   ∈ [0.0, 1.0)
         * draw_r_off = r_off + speed × alpha projects each ray's head
         * forward by the sub-tick fraction elapsed since last physics step.
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
        sun_draw(&sun, alpha, cols, rows);
        screen_hud(cols, fps_disp, theme_idx);
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
        case 'r': case 'R':
            sun_init(&sun, cols, rows);
            break;
        default:
            break;
        }
    }

    return 0;
}
