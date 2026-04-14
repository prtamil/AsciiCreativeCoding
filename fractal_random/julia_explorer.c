/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * julia_explorer.c — Interactive Julia Set Explorer
 *
 * Split-screen layout:
 *   LEFT  — Mandelbrot set (precomputed at start/resize)
 *           A crosshair marks the selected c = cr + ci·i
 *   RIGHT — Julia set for current c, redrawn every frame
 *           z → z² + c,  escape radius 2,  MAX_ITER 128
 *
 * Moving the crosshair on the Mandelbrot map with arrow keys instantly
 * redraws the Julia panel.  Every pixel of the Julia set is recomputed
 * analytically each frame, so response to keystrokes is immediate.
 *
 * Auto-wander ('a') orbits c along a slow ellipse through the Mandelbrot
 * boundary, revealing many distinct Julia morphologies automatically.
 *
 * Keys:
 *   q / ESC     quit
 *   Arrow keys  move c  (fine step 0.015)
 *   H J K L     move c  (coarse step 0.08)
 *   t / T       next / prev color theme  (Fire/Ice/Matrix/Plasma/Mono)
 *   r           reset c → −0.7 + 0.27i  (Douady rabbit)
 *   z / Z       Julia view: zoom in / out
 *   a           toggle auto-wander
 *   p / spc     pause / resume auto-wander
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra julia_explorer.c -o julia_explorer -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color / themes
 *   §4  mandelbrot panel  (precomputed)
 *   §5  julia panel  (per-frame recompute)
 *   §6  draw helpers
 *   §7  app / main
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

#define RENDER_FPS   30
#define RENDER_NS   (1000000000LL / RENDER_FPS)
#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL

enum {
    MAX_ITER      = 128,
    N_LEVELS      = 8,      /* color bands: 0 = bg (not drawn), 1–7 = gradient */
    N_THEMES      = 5,
    GRID_ROWS_MAX = 80,
    GRID_COLS_MAX = 300,

    CP_HUD   = 1,
    CP_DIV   = 2,
    CP_XHAIR = 3,
    CP_LEV1  = 4,    /* pairs CP_LEV1 .. CP_LEV1+6  →  levels 1..7 */
};

/* Terminal aspect: cell height ≈ 2 × cell width */
#define ASPECT_R    2.0f

/* Julia view defaults */
#define J_IM_HALF_DEF  1.5f
#define J_ZOOM_IN      0.85f
#define J_ZOOM_OUT    (1.0f / J_ZOOM_IN)
#define J_IM_HALF_MIN  0.25f
#define J_IM_HALF_MAX  3.0f

/* Mandelbrot fixed view: classic framing */
#define M_RE_MIN    (-2.5f)
#define M_RE_MAX    ( 1.0f)
#define M_IM_HALF    1.25f   /* ±1.25 imaginary */

/* Interaction step sizes */
#define FINE_STEP   0.015f
#define COARSE_STEP 0.08f

/* Auto-wander: c orbits a squished ellipse (keeps it near the boundary) */
#define WANDER_R      0.72f
#define WANDER_YSHRK  0.65f
#define WANDER_SPEED  0.006f   /* rad/frame  ≈ 35 s per orbit at 30 fps */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + (int64_t)ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color / themes                                                     */
/* ===================================================================== */

/*
 * Each theme: 8 xterm-256 foreground colors for levels 0..7.
 * Level 0 is unused (background — cells at this level are not drawn).
 * Level 7 also gets A_BOLD for a bright "inside the set" glow.
 */
static const struct {
    const char *name;
    short       fg[N_LEVELS];
} k_themes[N_THEMES] = {
    { "Fire",   { -1, 124, 196, 202, 208, 220, 226, 231 } },
    { "Ice",    { -1,  17,  21,  27,  33,  39,  51, 231 } },
    { "Matrix", { -1,  22,  28,  34,  40,  46,  82, 231 } },
    { "Plasma", { -1,  54,  93, 129, 165, 201, 213, 231 } },
    { "Mono",   { -1, 235, 239, 243, 247, 251, 255, 231 } },
};

/* Characters assigned to each level (denser → brighter → higher level) */
static const char k_chars[N_LEVELS] = { ' ', '.', ',', ':', '+', '#', '@', '*' };

/* 8-color terminal fallback palette */
static const short k_fb8[N_LEVELS] = {
    -1, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
    COLOR_GREEN, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE,
};

static void theme_apply(int t)
{
    for (int l = 1; l < N_LEVELS; l++) {
        short fg = (COLORS >= 256) ? k_themes[t].fg[l] : k_fb8[l];
        init_pair((short)(CP_LEV1 + l - 1), fg, COLOR_BLACK);
    }
}

static void colors_init(int theme)
{
    start_color();
    init_pair(CP_HUD,    51, COLOR_BLACK);   /* cyan    */
    init_pair(CP_DIV,   240, COLOR_BLACK);   /* gray    */
    init_pair(CP_XHAIR, (COLORS >= 256) ? 226 : COLOR_YELLOW, COLOR_BLACK);
    theme_apply(theme);
}

/* ===================================================================== */
/* §4  mandelbrot panel  (precomputed)                                    */
/* ===================================================================== */

/*
 * g_mbuf[row][col] = color level 0..7 for the Mandelbrot set.
 * Occupies the left g_mw columns of the terminal.
 * Recomputed at init and on every SIGWINCH resize.
 */
static uint8_t g_mbuf[GRID_ROWS_MAX][GRID_COLS_MAX / 2 + 2];
static int g_rows, g_cols, g_mw, g_jw;   /* terminal & panel sizes */

/*
 * escape_level — map an escape iteration count to a color level.
 *
 *   iter == MAX_ITER  →  7  (inside the set — brightest)
 *   fast escape       →  0  (background, not drawn)
 *   slow escape       →  1..6 by fraction of MAX_ITER
 */
static uint8_t escape_level(int iter)
{
    if (iter >= MAX_ITER) return (uint8_t)(N_LEVELS - 1);
    float frac = (float)iter / (float)MAX_ITER;
    if (frac < 0.07f) return 0;
    int band = 1 + (int)((frac - 0.07f) / 0.93f * (float)(N_LEVELS - 2) + 0.5f);
    if (band < 1)          band = 1;
    if (band > N_LEVELS - 2) band = N_LEVELS - 2;
    return (uint8_t)band;
}

static void mandelbrot_compute(void)
{
    float re_range = M_RE_MAX - M_RE_MIN;
    float im_range = 2.0f * M_IM_HALF;
    int   mw1 = (g_mw > 1) ? g_mw - 1 : 1;
    int   rows1 = (g_rows > 1) ? g_rows - 1 : 1;

    for (int row = 0; row < g_rows && row < GRID_ROWS_MAX; row++) {
        float ci = M_IM_HALF - (float)row / (float)rows1 * im_range;
        for (int col = 0; col < g_mw && col < GRID_COLS_MAX / 2 + 2; col++) {
            float cr = M_RE_MIN + (float)col / (float)mw1 * re_range;
            float zr = 0.f, zi = 0.f;
            int iter;
            for (iter = 0; iter < MAX_ITER; iter++) {
                float zr2 = zr * zr - zi * zi + cr;
                zi = 2.f * zr * zi + ci;
                zr = zr2;
                if (zr * zr + zi * zi > 4.f) break;
            }
            g_mbuf[row][col] = escape_level(iter);
        }
    }
}

/* Map c = (cr, ci) to a pixel position in the Mandelbrot panel */
static void c_to_mpanel(float cr, float ci, int *out_col, int *out_row)
{
    float re_range = M_RE_MAX - M_RE_MIN;
    float im_range = 2.0f * M_IM_HALF;
    *out_col = (int)((cr - M_RE_MIN) / re_range * (float)(g_mw - 1) + 0.5f);
    *out_row = (int)((M_IM_HALF - ci) / im_range * (float)(g_rows - 1) + 0.5f);
}

/* ===================================================================== */
/* §5  julia panel  (per-frame recompute)                                 */
/* ===================================================================== */

/*
 * Julia set for c = (cr, ci), drawn into the right panel.
 * Right panel columns: [g_mw+1 .. g_cols-1]
 *
 * View: im ∈ [−im_half, +im_half],
 *       re ∈ [−re_half, +re_half] where re_half corrects for aspect ratio.
 */
static float g_j_im_half = J_IM_HALF_DEF;

static void julia_draw(float cr, float ci)
{
    if (g_jw <= 0 || g_rows <= 0) return;

    float im_half = g_j_im_half;
    float re_half = im_half * (float)g_jw / (float)g_rows / ASPECT_R;
    int   jw1     = (g_jw > 1)   ? g_jw   - 1 : 1;
    int   rows1   = (g_rows > 1) ? g_rows  - 1 : 1;

    for (int row = 0; row < g_rows; row++) {
        float im = im_half - (float)row / (float)rows1 * (2.0f * im_half);
        for (int col = 0; col < g_jw; col++) {
            float re = -re_half + (float)col / (float)jw1 * (2.0f * re_half);
            float zr = re, zi = im;
            int iter;
            for (iter = 0; iter < MAX_ITER; iter++) {
                float zr2 = zr * zr - zi * zi + cr;
                zi = 2.f * zr * zi + ci;
                zr = zr2;
                if (zr * zr + zi * zi > 4.f) break;
            }
            uint8_t lev = escape_level(iter);
            if (lev == 0) continue;
            attr_t attr = COLOR_PAIR(CP_LEV1 + (int)lev - 1);
            if (lev == N_LEVELS - 1) attr |= A_BOLD;
            attron(attr);
            mvaddch(row, g_mw + 1 + col, (chtype)(unsigned char)k_chars[lev]);
            attroff(attr);
        }
    }
}

/* ===================================================================== */
/* §6  draw helpers                                                       */
/* ===================================================================== */

static void mandelbrot_draw(void)
{
    for (int row = 0; row < g_rows; row++) {
        for (int col = 0; col < g_mw; col++) {
            uint8_t lev = g_mbuf[row][col];
            if (lev == 0) continue;
            attr_t attr = COLOR_PAIR(CP_LEV1 + (int)lev - 1);
            if (lev == N_LEVELS - 1) attr |= A_BOLD;
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)k_chars[lev]);
            attroff(attr);
        }
    }
}

static void divider_draw(void)
{
    attron(COLOR_PAIR(CP_DIV));
    for (int row = 0; row < g_rows; row++)
        mvaddch(row, g_mw, '|');
    attroff(COLOR_PAIR(CP_DIV));
}

static void crosshair_draw(int xr, int xc)
{
    /* Clamp to Mandelbrot panel */
    if (xc < 0 || xc >= g_mw || xr < 0 || xr >= g_rows) return;

    attron(COLOR_PAIR(CP_XHAIR) | A_BOLD);

    /* Horizontal arms */
    for (int c = xc - 4; c <= xc + 4; c++) {
        if (c < 0 || c >= g_mw || c == xc) continue;
        mvaddch(xr, c, '-');
    }
    /* Vertical arms */
    for (int r = xr - 3; r <= xr + 3; r++) {
        if (r < 0 || r >= g_rows || r == xr) continue;
        mvaddch(r, xc, '|');
    }
    /* Centre */
    mvaddch(xr, xc, '+');

    attroff(COLOR_PAIR(CP_XHAIR) | A_BOLD);
}

static void labels_draw(float cr, float ci, int theme, bool wander, bool paused,
                        double fps)
{
    /* Left panel label */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 1, " MANDELBROT ");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Right panel label (top of Julia panel) */
    char jlabel[64];
    snprintf(jlabel, sizeof jlabel, " JULIA  c = %.3f%+.3fi ",
             (double)cr, (double)ci);
    int jx = g_mw + 2;
    if (jx + (int)strlen(jlabel) < g_cols) {
        attron(COLOR_PAIR(CP_HUD) | A_BOLD);
        mvprintw(0, jx, "%s", jlabel);
        attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
    }

    /* Bottom status bar */
    char status[128];
    float zoom = J_IM_HALF_DEF / g_j_im_half;
    snprintf(status, sizeof status,
             " [q]quit [arrows/HJKL]move  [t]theme:%s  [z/Z]zoom:%.2fx"
             "  [a]wander%s%s  %.0ffps ",
             k_themes[theme].name, (double)zoom,
             wander ? ":ON" : "",
             paused ? " PAUSE" : "",
             fps);
    int sx = (g_cols - (int)strlen(status)) / 2;
    if (sx < 0) sx = 0;
    int hrow = (g_rows > 1) ? g_rows - 1 : 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(hrow, sx, "%s", status);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* §7  app / main                                                         */
/* ===================================================================== */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_exit(int s)   { (void)s; g_running     = 0; }
static void sig_resize(int s) { (void)s; g_need_resize = 1; }
static void cleanup(void)     { endwin(); }

static void do_resize(void)
{
    endwin();
    refresh();
    getmaxyx(stdscr, g_rows, g_cols);
    if (g_rows < 5)            g_rows = 5;
    if (g_rows > GRID_ROWS_MAX) g_rows = GRID_ROWS_MAX;
    if (g_cols < 10)           g_cols = 10;
    if (g_cols > GRID_COLS_MAX) g_cols = GRID_COLS_MAX;
    g_mw = g_cols / 2;
    g_jw = g_cols - g_mw - 1;   /* -1 for the divider column */
    mandelbrot_compute();
    g_need_resize = 0;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_exit);
    signal(SIGTERM,  sig_exit);
    signal(SIGWINCH, sig_resize);

    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);

    int theme = 0;
    colors_init(theme);

    /* Initial terminal size & precompute Mandelbrot */
    getmaxyx(stdscr, g_rows, g_cols);
    if (g_rows > GRID_ROWS_MAX) g_rows = GRID_ROWS_MAX;
    if (g_cols > GRID_COLS_MAX) g_cols = GRID_COLS_MAX;
    g_mw = g_cols / 2;
    g_jw = g_cols - g_mw - 1;
    mandelbrot_compute();

    /* State */
    float c_re  = -0.7f, c_im  = 0.27f;   /* Douady rabbit default */
    bool  wander = false, paused = false;
    float wander_t = 0.f;

    double  fps        = 0.0;
    int     frame_cnt  = 0;
    int64_t fps_clock  = clock_ns();

    while (g_running) {

        if (g_need_resize) do_resize();

        int64_t frame_start = clock_ns();

        /* ---------- input ---------- */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            /* quit */
            case 'q': case 'Q': case 27: g_running = 0; break;

            /* fine c movement — arrow keys */
            case KEY_UP:    c_im += FINE_STEP;  wander = false; break;
            case KEY_DOWN:  c_im -= FINE_STEP;  wander = false; break;
            case KEY_LEFT:  c_re -= FINE_STEP;  wander = false; break;
            case KEY_RIGHT: c_re += FINE_STEP;  wander = false; break;

            /* coarse c movement — HJKL */
            case 'K': c_im += COARSE_STEP; wander = false; break;
            case 'J': c_im -= COARSE_STEP; wander = false; break;
            case 'H': c_re -= COARSE_STEP; wander = false; break;
            case 'L': c_re += COARSE_STEP; wander = false; break;

            /* theme */
            case 't': theme = (theme + 1)            % N_THEMES;
                      theme_apply(theme); break;
            case 'T': theme = (theme + N_THEMES - 1) % N_THEMES;
                      theme_apply(theme); break;

            /* reset */
            case 'r': c_re = -0.7f; c_im = 0.27f; wander = false; break;

            /* Julia zoom: z = in (smaller im_half), Z = out */
            case 'z': {
                g_j_im_half *= J_ZOOM_IN;
                if (g_j_im_half < J_IM_HALF_MIN) g_j_im_half = J_IM_HALF_MIN;
                break;
            }
            case 'Z': {
                g_j_im_half *= J_ZOOM_OUT;
                if (g_j_im_half > J_IM_HALF_MAX) g_j_im_half = J_IM_HALF_MAX;
                break;
            }

            /* auto-wander */
            case 'a': case 'A':
                wander = !wander;
                if (wander) {
                    /* Sync wander_t to current c position so there's no jump */
                    wander_t = atan2f(c_im / WANDER_YSHRK, c_re / WANDER_R);
                }
                break;

            /* pause wander */
            case 'p': case 'P': case ' ':
                paused = !paused;
                break;

            default: break;
            }
        }

        /* ---------- wander update ---------- */
        if (wander && !paused) {
            wander_t += WANDER_SPEED;
            c_re = WANDER_R * cosf(wander_t);
            c_im = WANDER_R * sinf(wander_t) * WANDER_YSHRK;
        }

        /* ---------- draw ---------- */
        erase();
        mandelbrot_draw();
        divider_draw();
        julia_draw(c_re, c_im);

        /* Crosshair on Mandelbrot panel */
        int xc, xr;
        c_to_mpanel(c_re, c_im, &xc, &xr);
        crosshair_draw(xr, xc);

        /* Labels (drawn last so they overlay the fractal) */
        labels_draw(c_re, c_im, theme, wander, paused, fps);

        wnoutrefresh(stdscr);
        doupdate();

        /* ---------- FPS counter ---------- */
        frame_cnt++;
        int64_t now = clock_ns();
        if (now - fps_clock >= 500LL * NS_PER_MS) {
            fps       = (double)frame_cnt
                      / ((double)(now - fps_clock) / (double)NS_PER_SEC);
            frame_cnt = 0;
            fps_clock = now;
        }

        /* ---------- frame cap ---------- */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(RENDER_NS - elapsed);
    }

    return 0;
}
