/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * newton_fractal.c — Newton's Method Fractal  (z⁴ − 1 = 0)
 *
 * Apply Newton's root-finding iteration to f(z) = z⁴ − 1:
 *
 *   z  ←  z − f(z)/f′(z)  =  (3z⁴ + 1) / (4z³)
 *
 * The four roots are  1, −1, i, −i.  Each point in the complex plane
 * converges to one of them; the boundary between basins of attraction
 * is a fractal.  Colour encodes which root; brightness encodes speed.
 *
 * Inside set (non-converging) → black.
 *
 * Keys: q quit  ←→↑↓ pan  +/- zoom  r reset  1/2/3/4 zoom to each root
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fractal_random/newton_fractal.c \
 *       -o newton_fractal -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 fractal  §5 draw  §6 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define MAX_ITER     64
#define TOL          1e-5f
#define RENDER_NS    (1000000000LL / 30)
#define ZOOM_FACTOR  1.30f
#define PAN_FRAC     0.15f

#define INIT_X_MIN  (-2.0f)
#define INIT_X_MAX  ( 2.0f)
#define INIT_Y_MIN  (-1.5f)
#define INIT_Y_MAX  ( 1.5f)

/* per-root zoom presets */
static const float ROOT_ZOOM[4][4] = {
    { 0.6f, 1.4f, -0.4f,  0.4f },   /* root 1:  +1 */
    {-1.4f,-0.6f, -0.4f,  0.4f },   /* root 2:  -1 */
    {-0.4f, 0.4f,  0.6f,  1.4f },   /* root 3:  +i */
    {-0.4f, 0.4f, -1.4f, -0.6f },   /* root 4:  -i */
};

enum {
    /* dark/bright pairs for each of 4 roots */
    CP_R1D = 1,  /* root +1  dark  — dark red     */
    CP_R1B,      /* root +1  bright — bright red   */
    CP_R2D,      /* root -1  dark  — dark blue     */
    CP_R2B,      /* root -1  bright — bright blue  */
    CP_R3D,      /* root +i  dark  — dark yellow   */
    CP_R3B,      /* root +i  bright — yellow       */
    CP_R4D,      /* root -i  dark  — dark green    */
    CP_R4B,      /* root -i  bright — green        */
    CP_SET,      /* did not converge — black        */
    CP_HUD,
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
        init_pair(CP_R1D,  88, -1);   /* dark red    */
        init_pair(CP_R1B, 196, -1);   /* bright red  */
        init_pair(CP_R2D,  18, -1);   /* dark blue   */
        init_pair(CP_R2B,  21, -1);   /* bright blue */
        init_pair(CP_R3D, 136, -1);   /* dark yellow */
        init_pair(CP_R3B, 226, -1);   /* yellow      */
        init_pair(CP_R4D,  22, -1);   /* dark green  */
        init_pair(CP_R4B,  46, -1);   /* bright green*/
        init_pair(CP_SET, COLOR_BLACK, COLOR_BLACK);
        init_pair(CP_HUD, 226, -1);
    } else {
        init_pair(CP_R1D, COLOR_RED,     -1);
        init_pair(CP_R1B, COLOR_RED,     -1);
        init_pair(CP_R2D, COLOR_BLUE,    -1);
        init_pair(CP_R2B, COLOR_BLUE,    -1);
        init_pair(CP_R3D, COLOR_YELLOW,  -1);
        init_pair(CP_R3B, COLOR_YELLOW,  -1);
        init_pair(CP_R4D, COLOR_GREEN,   -1);
        init_pair(CP_R4B, COLOR_GREEN,   -1);
        init_pair(CP_SET, COLOR_BLACK, COLOR_BLACK);
        init_pair(CP_HUD, COLOR_YELLOW, -1);
    }
}

/* ===================================================================== */
/* §4  fractal                                                            */
/* ===================================================================== */

static int g_rows, g_cols;
static float g_xmin = INIT_X_MIN, g_xmax = INIT_X_MAX;
static float g_ymin = INIT_Y_MIN, g_ymax = INIT_Y_MAX;

/* four roots of z^4 - 1 = 0 */
static const float ROOTS_RE[4] = {  1.f,  -1.f,  0.f,  0.f };
static const float ROOTS_IM[4] = {  0.f,   0.f,  1.f, -1.f };

/*
 * newton_iter — apply Newton's method to z^4 - 1 = 0.
 *
 * Returns root index [0..3] that was reached, or -1 if not converged.
 * *n_iter receives the iteration count (used for brightness).
 */
static int newton_iter(float cr, float ci, int *n_iter)
{
    float re = cr, im = ci;
    for (int n = 0; n < MAX_ITER; n++) {
        /* z^2 */
        float re2 = re*re - im*im;
        float im2 = 2.f*re*im;
        /* z^3 = z^2 * z */
        float re3 = re2*re - im2*im;
        float im3 = re2*im + im2*re;
        /* z^4 = z^2 * z^2 */
        float re4 = re2*re2 - im2*im2;
        float im4 = 2.f*re2*im2;
        /* numerator: 3z^4 + 1 */
        float num_re = 3.f*re4 + 1.f;
        float num_im = 3.f*im4;
        /* denominator: 4z^3 */
        float den_re = 4.f*re3;
        float den_im = 4.f*im3;
        float denom  = den_re*den_re + den_im*den_im;
        if (denom < 1e-12f) break;
        /* z_new = num / den */
        re = (num_re*den_re + num_im*den_im) / denom;
        im = (num_im*den_re - num_re*den_im) / denom;
        /* convergence check */
        for (int r = 0; r < 4; r++) {
            float dr = re - ROOTS_RE[r];
            float di = im - ROOTS_IM[r];
            if (dr*dr + di*di < TOL) {
                *n_iter = n + 1;
                return r;
            }
        }
    }
    *n_iter = MAX_ITER;
    return -1;
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static void map_color(int root, int n, int *cp_out, chtype *ch_out)
{
    if (root < 0) { *cp_out = CP_SET; *ch_out = ' '; return; }

    float t = (float)n / (float)MAX_ITER;
    /* fast convergence = bright, slow = dark */
    int bright = (t < 0.25f);

    /* dark/bright offset: 0 for dark, 1 for bright */
    *cp_out = CP_R1D + root * 2 + (bright ? 1 : 0);

    if (t < 0.15f)      *ch_out = '.';
    else if (t < 0.40f) *ch_out = ':';
    else if (t < 0.70f) *ch_out = '+';
    else                *ch_out = '#';
}

static void fractal_draw(void)
{
    int hud = 2;
    int draw_rows = g_rows - hud;
    if (draw_rows < 1) return;

    float xs = g_xmax - g_xmin;
    float ys = g_ymax - g_ymin;

    for (int cy = 0; cy < draw_rows; cy++) {
        float ci = g_ymin + ys * (float)cy / (float)(draw_rows - 1);
        for (int cx = 0; cx < g_cols; cx++) {
            float cr = g_xmin + xs * (float)cx / (float)(g_cols - 1);
            int n;
            int root = newton_iter(cr, ci, &n);
            int cp; chtype ch;
            map_color(root, n, &cp, &ch);
            attron(COLOR_PAIR(cp) | A_BOLD);
            mvaddch(cy + hud, cx, ch);
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }
}

static void hud_draw(void)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " NewtonFractal  q:quit  ←→↑↓:pan  +/-:zoom  r:reset  1-4:root-zoom");
    mvprintw(1, 0,
        " x:[%.4f,%.4f]  y:[%.4f,%.4f]  iter:%d  roots: +1(red) -1(blue) +i(yel) -i(grn)",
        g_xmin, g_xmax, g_ymin, g_ymax, MAX_ITER);
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  app                                                                */
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
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
        }

        int ch = getch();
        float xw = g_xmax - g_xmin;
        float yw = g_ymax - g_ymin;
        float xp = xw * PAN_FRAC;
        float yp = yw * PAN_FRAC;

        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'r': case 'R':
            g_xmin = INIT_X_MIN; g_xmax = INIT_X_MAX;
            g_ymin = INIT_Y_MIN; g_ymax = INIT_Y_MAX;
            break;
        case '1': case '2': case '3': case '4': {
            int i = ch - '1';
            g_xmin = ROOT_ZOOM[i][0]; g_xmax = ROOT_ZOOM[i][1];
            g_ymin = ROOT_ZOOM[i][2]; g_ymax = ROOT_ZOOM[i][3];
            break;
        }
        case KEY_LEFT:  g_xmin -= xp; g_xmax -= xp; break;
        case KEY_RIGHT: g_xmin += xp; g_xmax += xp; break;
        case KEY_UP:    g_ymin -= yp; g_ymax -= yp; break;
        case KEY_DOWN:  g_ymin += yp; g_ymax += yp; break;
        case '+': case '=': {
            float xc = (g_xmin+g_xmax)*.5f, yc = (g_ymin+g_ymax)*.5f;
            g_xmin = xc - xw/ZOOM_FACTOR*.5f; g_xmax = xc + xw/ZOOM_FACTOR*.5f;
            g_ymin = yc - yw/ZOOM_FACTOR*.5f; g_ymax = yc + yw/ZOOM_FACTOR*.5f;
            break;
        }
        case '-': {
            float xc = (g_xmin+g_xmax)*.5f, yc = (g_ymin+g_ymax)*.5f;
            g_xmin = xc - xw*ZOOM_FACTOR*.5f; g_xmax = xc + xw*ZOOM_FACTOR*.5f;
            g_ymin = yc - yw*ZOOM_FACTOR*.5f; g_ymax = yc + yw*ZOOM_FACTOR*.5f;
            break;
        }
        default: break;
        }

        long long now = clock_ns();
        erase();
        hud_draw();
        fractal_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
