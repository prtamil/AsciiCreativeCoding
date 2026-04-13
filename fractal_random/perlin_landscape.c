/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * perlin_landscape.c — 2-D Parallax Landscape
 *
 * Three independent Perlin noise terrain layers scroll at different
 * speeds (parallax illusion of depth):
 *
 *   Far   — distant mountains  slow  dark blue-grey silhouette
 *   Mid   — rolling hills      medium  olive green
 *   Near  — foreground ground  full speed  bright green
 *
 * Each layer is a 1-D fractal Brownian motion height profile.
 * Painter's algorithm: sky → far → mid → near (each overwrites).
 * Stars are scattered deterministically in the upper sky.
 *
 * Keys: q quit  p pause  r reset  ← → speed / direction  +/- noise zoom
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/perlin_landscape.c \
 *       -o perlin_landscape -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 noise  §5 terrain  §6 draw  §7 app
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

#define RENDER_NS  (1000000000LL / 30)
#define HUD_ROWS    2

/* Parallax: fraction of camera speed each layer scrolls at */
#define SCROLL_FAR    0.12f
#define SCROLL_MID    0.38f
#define SCROLL_NEAR   1.00f

/* Noise frequency: horizontal "stretch" of each landscape layer.
 * Smaller = wider features (smoother). */
static float g_freq = 1.0f;           /* zoom multiplier */
#define BASE_FREQ_FAR   0.009f
#define BASE_FREQ_MID   0.018f
#define BASE_FREQ_NEAR  0.040f

/* Terrain vertical bands (fraction of draw_rows from top).
 * Peak floats up from BASE by up to AMP * noise. */
#define BASE_FAR   0.60f
#define AMP_FAR    0.28f
#define BASE_MID   0.70f
#define AMP_MID    0.22f
#define BASE_NEAR  0.80f
#define AMP_NEAR   0.16f

/* Camera */
static float g_scroll = 0.f;
static float g_speed  = 0.6f;   /* terminal columns per frame */
#define SPEED_MAX  4.f
#define SPEED_MIN  0.05f

/* Color pairs */
enum {
    CP_SKY_HI = 1,   /* top sky        */
    CP_SKY_LO,       /* horizon sky    */
    CP_STAR,          /* stars          */
    CP_FAR_E,         /* far edge       */
    CP_FAR_F,         /* far fill       */
    CP_MID_E,         /* mid edge       */
    CP_MID_F,         /* mid fill       */
    CP_NEAR_E,        /* near edge      */
    CP_NEAR_F,        /* near fill      */
    CP_NEAR_B,        /* near base      */
    CP_HUD
};

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

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_SKY_HI, 17,  -1);  /* deep navy      */
        init_pair(CP_SKY_LO, 18,  -1);  /* dark blue      */
        init_pair(CP_STAR,  250,  -1);  /* light grey     */
        init_pair(CP_FAR_E,  24,  -1);  /* steel blue edge */
        init_pair(CP_FAR_F,  17,  -1);  /* dark navy fill  */
        init_pair(CP_MID_E,  22,  -1);  /* dark green edge */
        init_pair(CP_MID_F,  22,  -1);  /* dark green fill */
        init_pair(CP_NEAR_E, 34,  -1);  /* bright green edge */
        init_pair(CP_NEAR_F, 28,  -1);  /* medium green    */
        init_pair(CP_NEAR_B, 22,  -1);  /* deep green base */
        init_pair(CP_HUD,   244,  -1);
    } else {
        init_pair(CP_SKY_HI, COLOR_BLUE,   -1);
        init_pair(CP_SKY_LO, COLOR_CYAN,   -1);
        init_pair(CP_STAR,   COLOR_WHITE,  -1);
        init_pair(CP_FAR_E,  COLOR_BLUE,   -1);
        init_pair(CP_FAR_F,  COLOR_BLUE,   -1);
        init_pair(CP_MID_E,  COLOR_GREEN,  -1);
        init_pair(CP_MID_F,  COLOR_GREEN,  -1);
        init_pair(CP_NEAR_E, COLOR_GREEN,  -1);
        init_pair(CP_NEAR_F, COLOR_GREEN,  -1);
        init_pair(CP_NEAR_B, COLOR_GREEN,  -1);
        init_pair(CP_HUD,    COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  Perlin gradient noise (2-D; use y=constant per layer)             */
/* ===================================================================== */

#define PERM 256
static unsigned char g_p[PERM * 2];
static float g_gx[PERM * 2], g_gy[PERM * 2];   /* gradient vectors (doubled for wrap) */

static void noise_init(void)
{
    for (int i = 0; i < PERM; i++) {
        g_p[i] = (unsigned char)i;
        float a = (float)rand() / RAND_MAX * 2.f * (float)M_PI;
        g_gx[i] = cosf(a);
        g_gy[i] = sinf(a);
    }
    for (int i = PERM - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        unsigned char t = g_p[i]; g_p[i] = g_p[j]; g_p[j] = t;
        float tx = g_gx[i]; g_gx[i] = g_gx[j]; g_gx[j] = tx;
        float ty = g_gy[i]; g_gy[i] = g_gy[j]; g_gy[j] = ty;
    }
    for (int i = 0; i < PERM; i++) {
        g_p[i + PERM] = g_p[i];
        g_gx[i + PERM] = g_gx[i];
        g_gy[i + PERM] = g_gy[i];
    }
}

static float fade(float t) { return t * t * t * (t * (t * 6.f - 15.f) + 10.f); }
static float lerp(float a, float b, float t) { return a + t * (b - a); }

static float perlin(float x, float y)
{
    int xi = (int)floorf(x) & (PERM - 1);
    int yi = (int)floorf(y) & (PERM - 1);
    float xf = x - floorf(x), yf = y - floorf(y);
    float u = fade(xf), v = fade(yf);

    int aa = g_p[g_p[xi  ] + yi  ];
    int ab = g_p[g_p[xi  ] + yi+1];
    int ba = g_p[g_p[xi+1] + yi  ];
    int bb = g_p[g_p[xi+1] + yi+1];

    float r =
        lerp(lerp(g_gx[aa]*xf       + g_gy[aa]*yf,
                  g_gx[ba]*(xf-1.f) + g_gy[ba]*yf,       u),
             lerp(g_gx[ab]*xf       + g_gy[ab]*(yf-1.f),
                  g_gx[bb]*(xf-1.f) + g_gy[bb]*(yf-1.f), u), v);
    return r;
}

/* Fractal Brownian Motion: 5 octaves → result in roughly [-0.5, 0.5] */
static float fbm(float x, float y)
{
    float v = 0.f, a = 0.6f, f = 1.f, m = 0.f;
    for (int o = 0; o < 5; o++) {
        v += a * perlin(x * f, y * f);
        m += a; a *= 0.5f; f *= 2.f;
    }
    return v / m;   /* normalised to ≈ [-0.5, 0.5] */
}

/* ===================================================================== */
/* §5  terrain                                                            */
/* ===================================================================== */

static int g_rows, g_cols;
static bool g_paused;

/* Deterministic star hash — no storage, no flicker */
static bool is_star(int col, int row)
{
    unsigned h = (unsigned)col * 2654435761u ^ (unsigned)row * 2246822519u;
    return (h & 0x1FF) < 3;   /* ~0.6% density */
}

/* Sample one layer's peak row for this column.
 * yoffset keeps layers independent from each other in noise space. */
static int layer_row(float scroll_frac, float freq,
                     float base, float amp,
                     float col, float yoffset, int draw_rows)
{
    float x = g_scroll * scroll_frac + col * freq * g_freq;
    float n = fbm(x, yoffset);          /* ≈ [-0.5, 0.5] */
    float h = base - (n + 0.5f) * amp; /* fraction from top */
    int r = (int)(h * (float)draw_rows);
    if (r < 0) r = 0;
    if (r >= draw_rows) r = draw_rows - 1;
    return r;
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static void scene_draw(void)
{
    int draw_rows = g_rows - HUD_ROWS;
    if (draw_rows < 1) draw_rows = 1;

    for (int col = 0; col < g_cols; col++) {

        int rf = layer_row(SCROLL_FAR,  BASE_FREQ_FAR,  BASE_FAR,  AMP_FAR,
                           (float)col, 0.0f,  draw_rows);
        int rm = layer_row(SCROLL_MID,  BASE_FREQ_MID,  BASE_MID,  AMP_MID,
                           (float)col, 8.3f,  draw_rows);
        int rn = layer_row(SCROLL_NEAR, BASE_FREQ_NEAR, BASE_NEAR, AMP_NEAR,
                           (float)col, 16.7f, draw_rows);

        /* Enforce visual ordering so layers never invert */
        if (rm < rf + 1) rm = rf + 1;
        if (rn < rm + 1) rn = rm + 1;
        if (rf >= draw_rows) rf = draw_rows - 1;
        if (rm >= draw_rows) rm = draw_rows - 1;
        if (rn >= draw_rows) rn = draw_rows - 1;

        for (int r = 0; r < draw_rows; r++) {
            int screen_row = r + HUD_ROWS;
            int cp; chtype ch;

            if (r < rf) {
                /* ── sky ── */
                float sky_t = (float)r / (float)(rf > 0 ? rf : 1);
                if (sky_t < 0.55f && is_star(col, r)) {
                    cp = CP_STAR; ch = '.';
                } else {
                    cp = (sky_t < 0.50f) ? CP_SKY_HI : CP_SKY_LO;
                    ch = ' ';
                }
            } else if (r == rf) {
                /* ── far mountain edge ── */
                cp = CP_FAR_E; ch = '^';
            } else if (r < rm) {
                /* ── far mountain fill ── */
                cp = CP_FAR_F;
                ch = (r == rf + 1) ? ':' : '.';
            } else if (r == rm) {
                /* ── mid hill edge ── */
                cp = CP_MID_E; ch = '^';
            } else if (r < rn) {
                /* ── mid hill fill ── */
                cp = CP_MID_F;
                ch = (r == rm + 1) ? ':' : '#';
            } else if (r == rn) {
                /* ── near ground edge ── */
                cp = CP_NEAR_E; ch = '~';
            } else {
                /* ── near ground fill — darker toward base ── */
                cp = (r < rn + 3) ? CP_NEAR_F : CP_NEAR_B;
                ch = '#';
            }

            attron(COLOR_PAIR(cp));
            mvaddch(screen_row, col, ch);
            attroff(COLOR_PAIR(cp));
        }
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Landscape  q:quit  p:pause  r:reset  ←/→:speed+dir  +/-:zoom");
    mvprintw(1, 0,
        " scroll=%.0f  speed=%.2f  zoom=%.1fx  %s",
        (double)g_scroll, (double)g_speed, (double)g_freq,
        g_paused ? "PAUSED" : "scrolling");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §7  app                                                                */
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
    noise_init();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R':
            g_scroll = 0.f; g_speed = 0.6f; g_freq = 1.0f;
            noise_init();
            break;
        case KEY_RIGHT:
            g_speed += 0.1f; if (g_speed > SPEED_MAX) g_speed = SPEED_MAX; break;
        case KEY_LEFT:
            g_speed -= 0.1f; if (g_speed < -SPEED_MAX) g_speed = -SPEED_MAX; break;
        case '+': case '=':
            g_freq *= 1.25f; if (g_freq > 8.f) g_freq = 8.f; break;
        case '-':
            g_freq *= 0.8f; if (g_freq < 0.125f) g_freq = 0.125f; break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused) g_scroll += g_speed;

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
