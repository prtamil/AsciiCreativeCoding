/*
 * burning_ship.c — Burning Ship Fractal
 *
 * Escape-time iteration identical to Mandelbrot except the real and
 * imaginary parts are forced positive before squaring:
 *
 *   z ← (|Re(z)| + i|Im(z)|)²  +  c
 *
 * Expanding:  Re(z_new) = Re(z)² − Im(z)² + Re(c)
 *             Im(z_new) = 2|Re(z)||Im(z)| + Im(c)
 *
 * The absolute-value fold creates a characteristic "ship" shape with a
 * mast-like spire and flame-shaped filaments below.  The most detailed
 * region is near c ≈ −1.77 − 0.04i.
 *
 * A fire palette maps escape speed to colour:
 *   inside set → black  ·  slow escape → dark red/red  ·  fast → yellow/white
 *
 * Keys:  q quit   ← → ↑ ↓ pan   + - zoom   r reset   f fly-to ship
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fractal_random/burning_ship.c \
 *       -o burning_ship -lncurses -lm
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define MAX_ITER      96
#define RENDER_NS    (1000000000LL / 30)
#define CELL_W        8     /* pixels per terminal column (aspect correction) */
#define CELL_H       16     /* pixels per terminal row                        */
#define ZOOM_FACTOR   1.30f
#define PAN_FRAC      0.15f

/* Initial view: shows the full fractal including the main body and spire */
#define INIT_X_MIN  (-2.60f)
#define INIT_X_MAX  ( 1.40f)
#define INIT_Y_MIN  (-2.00f)
#define INIT_Y_MAX  ( 0.80f)

/*
 * "Ship" zoom preset — the characteristic vessel in the lower body.
 *
 * COORDINATE NOTE: burning_ship_iter() negates ci before iterating
 * (ci = -ci) to flip the image vertically so the ship appears hull-down.
 * That means the actual imaginary part used = -ci_passed.
 *
 * The ship hull lives at actual Im ≈ [-0.10, +0.025].
 * So ci_passed must be in  [-0.025, +0.100]  (opposite signs).
 *
 * x_range (0.28) ≈ 2 × y_range (0.13) for the ~2:1 terminal pixel ratio.
 */
#define SHIP_X_MIN  (-1.910f)
#define SHIP_X_MAX  (-1.630f)
#define SHIP_Y_MIN  (-0.025f)
#define SHIP_Y_MAX  ( 0.105f)

enum {
    CP_SET = 1,   /* inside set — black             */
    CP_F1,        /* very slow escape — dark red    */
    CP_F2,        /* slow escape      — red         */
    CP_F3,        /* mid escape       — orange      */
    CP_F4,        /* fast escape      — yellow      */
    CP_F5,        /* very fast escape — white       */
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
        init_pair(CP_SET, COLOR_BLACK, COLOR_BLACK);
        init_pair(CP_F1,   52, -1);   /* dark red     */
        init_pair(CP_F2,  196, -1);   /* bright red   */
        init_pair(CP_F3,  208, -1);   /* orange       */
        init_pair(CP_F4,  226, -1);   /* yellow       */
        init_pair(CP_F5,  231, -1);   /* bright white */
        init_pair(CP_HUD, 244, -1);   /* grey         */
    } else {
        init_pair(CP_SET, COLOR_BLACK, COLOR_BLACK);
        init_pair(CP_F1,  COLOR_RED,    -1);
        init_pair(CP_F2,  COLOR_RED,    -1);
        init_pair(CP_F3,  COLOR_YELLOW, -1);
        init_pair(CP_F4,  COLOR_YELLOW, -1);
        init_pair(CP_F5,  COLOR_WHITE,  -1);
        init_pair(CP_HUD, COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  fractal                                                            */
/* ===================================================================== */

static int g_rows, g_cols;
static float g_x_min = INIT_X_MIN, g_x_max = INIT_X_MAX;
static float g_y_min = INIT_Y_MIN, g_y_max = INIT_Y_MAX;

/*
 * burning_ship_iter() — iteration count for a single point c = (cr, ci).
 *
 * The fractal is typically shown flipped vertically (ci negated) so the
 * "ship" hull appears at the bottom.  We apply that convention here by
 * negating ci before the loop.
 */
static int burning_ship_iter(float cr, float ci)
{
    ci = -ci;   /* flip: shows ship right-side-up */
    float re = 0.f, im = 0.f;
    for (int n = 0; n < MAX_ITER; n++) {
        float abs_re = re < 0.f ? -re : re;
        float abs_im = im < 0.f ? -im : im;
        float re2 = re*re - im*im + cr;
        float im2 = 2.f * abs_re * abs_im + ci;
        re = re2; im = im2;
        if (re*re + im*im > 4.f) return n;
    }
    return MAX_ITER;
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

/*
 * map_color() — map iteration count to fire-palette color + character.
 *
 * The fire palette runs: black (set) → dark-red → red → orange → yellow → white.
 * Characters get denser as escape speed increases (more outside detail).
 */
static void map_color(int iter, int *cp_out, chtype *ch_out)
{
    if (iter == MAX_ITER) { *cp_out = CP_SET; *ch_out = ' '; return; }

    /* normalised escape fraction; 0 = barely escaped, 1 = escaped instantly */
    float t = (float)iter / (float)MAX_ITER;

    if      (t < 0.12f) { *cp_out = CP_F1; *ch_out = '.'; }
    else if (t < 0.30f) { *cp_out = CP_F2; *ch_out = ':'; }
    else if (t < 0.55f) { *cp_out = CP_F3; *ch_out = '+'; }
    else if (t < 0.80f) { *cp_out = CP_F4; *ch_out = '*'; }
    else                { *cp_out = CP_F5; *ch_out = '#'; }
}

static void fractal_draw(void)
{
    int hud = 2;   /* rows used by HUD at top */
    int draw_rows = g_rows - hud;
    if (draw_rows < 1) return;

    float x_span = g_x_max - g_x_min;
    float y_span = g_y_max - g_y_min;

    for (int cy = 0; cy < draw_rows; cy++) {
        float ci = g_y_min + y_span * (float)cy / (float)(draw_rows - 1);
        for (int cx = 0; cx < g_cols; cx++) {
            float cr = g_x_min + x_span * (float)cx / (float)(g_cols - 1);
            int iter = burning_ship_iter(cr, ci);
            int cp; chtype ch;
            map_color(iter, &cp, &ch);
            if (cp == CP_SET) {
                attron(COLOR_PAIR(cp));
                mvaddch(cy + hud, cx, ch);
                attroff(COLOR_PAIR(cp));
            } else {
                attron(COLOR_PAIR(cp) | A_BOLD);
                mvaddch(cy + hud, cx, ch);
                attroff(COLOR_PAIR(cp) | A_BOLD);
            }
        }
    }
}

static void hud_draw(void)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " BurningShip  q:quit  ←→↑↓:pan  +/-:zoom  r:reset  f:fly-to-ship");
    mvprintw(1, 0,
        " x:[%.4f,%.4f]  y:[%.4f,%.4f]  iter:%d",
        g_x_min, g_x_max, g_y_min, g_y_max, MAX_ITER);
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
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);

    long long last = clock_ns();

    while (!g_quit) {

        /* ── resize ─────────────────────────────────────────────── */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            last = clock_ns();
            continue;
        }

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        float xw = g_x_max - g_x_min;
        float yw = g_y_max - g_y_min;
        float xp = xw * PAN_FRAC;
        float yp = yw * PAN_FRAC;

        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;

        case 'r': case 'R':
            g_x_min = INIT_X_MIN; g_x_max = INIT_X_MAX;
            g_y_min = INIT_Y_MIN; g_y_max = INIT_Y_MAX;
            break;

        case 'f': case 'F':
            /* fly to the ship's hull region */
            g_x_min = SHIP_X_MIN; g_x_max = SHIP_X_MAX;
            g_y_min = SHIP_Y_MIN; g_y_max = SHIP_Y_MAX;
            break;

        case KEY_LEFT:  g_x_min -= xp; g_x_max -= xp; break;
        case KEY_RIGHT: g_x_min += xp; g_x_max += xp; break;
        case KEY_UP:    g_y_min -= yp; g_y_max -= yp; break;
        case KEY_DOWN:  g_y_min += yp; g_y_max += yp; break;

        case '+': case '=': {
            float xc = (g_x_min + g_x_max) * .5f;
            float yc = (g_y_min + g_y_max) * .5f;
            g_x_min = xc - xw / ZOOM_FACTOR * .5f;
            g_x_max = xc + xw / ZOOM_FACTOR * .5f;
            g_y_min = yc - yw / ZOOM_FACTOR * .5f;
            g_y_max = yc + yw / ZOOM_FACTOR * .5f;
            break;
        }
        case '-': {
            float xc = (g_x_min + g_x_max) * .5f;
            float yc = (g_y_min + g_y_max) * .5f;
            g_x_min = xc - xw * ZOOM_FACTOR * .5f;
            g_x_max = xc + xw * ZOOM_FACTOR * .5f;
            g_y_min = yc - yw * ZOOM_FACTOR * .5f;
            g_y_max = yc + yw * ZOOM_FACTOR * .5f;
            break;
        }
        default: break;
        }

        /* ── draw ────────────────────────────────────────────────── */
        long long now = clock_ns();
        last = now;
        (void)last;

        erase();
        hud_draw();
        fractal_draw();
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }

    return 0;
}
