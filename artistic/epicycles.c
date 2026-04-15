/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * epicycles.c — Fourier epicycles: DFT of a parametric path → animated
 *               rotating arm chain whose tip traces the original shape.
 *
 * Shapes: Heart  Star  Trefoil  Figure-8  Butterfly  (cycle with n/p)
 *
 * Algorithm
 * ---------
 *   1. Sample the chosen shape into N_SAMPLES complex points z[k].
 *   2. Compute DFT: Z[n] = Σ_k z[k]·exp(-2πi·n·k/N)
 *   3. Sort coefficients by amplitude |Z[n]|/N  (largest arm first).
 *   4. Animate: at angle φ, arm n contributes
 *        (|Z[n]|/N) · exp(i·(freq_n·φ + arg Z[n]))
 *      chained from the pivot, tip traces the reconstructed path.
 *   5. Auto-add one epicycle every AUTO_ADD_FRAMES to show convergence.
 *
 * Keys
 * ----
 *   q / ESC   quit
 *   space     pause
 *   n / p     next / prev shape
 *   + / -     add / remove one epicycle
 *   r         reset (restart from 1 epicycle)
 *   a         toggle auto-add
 *   c         toggle orbit circles
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/epicycles.c -o epicycles -lncurses -lm
 *
 * Sections:  §1 config  §2 clock  §3 color  §4 shapes  §5 DFT
 *            §6 scene   §7 screen §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Discrete Fourier Transform (DFT) of a complex path.
 *                  A 2-D curve is encoded as z[k] = x[k] + i·y[k].  The
 *                  DFT Z[n] = Σ_k z[k]·exp(-2πi·n·k/N) decomposes the curve
 *                  into N rotating phasors (epicycles).  Sorting by |Z[n]|
 *                  gives the greedy best-N approximation at each step.
 *
 * Math           : Each DFT bin Z[n] defines a circle of radius |Z[n]|/N
 *                  rotating at angular frequency n rev/cycle with initial
 *                  phase arg(Z[n]).  Chaining these circles reproduces the
 *                  original path exactly when all N arms are active
 *                  (Fourier completeness).  Convergence speed depends on
 *                  signal smoothness: smooth curves need fewer arms than
 *                  curves with sharp corners (Gibbs phenomenon).
 *
 * Performance    : DFT is O(N²) computed once on shape change; per-frame
 *                  cost is O(N_active) arm evaluations + O(TRAIL_LEN)
 *                  trail draw — both negligible at N≤256.
 *
 * Rendering      : Arm chain drawn with ncurses mvaddch; trail stored in
 *                  a ring buffer (TRAIL_LEN entries) so only the most recent
 *                  600 tip positions are kept — O(1) append and draw.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_SAMPLES       256     /* DFT input points — also max epicycles  */
#define TRAIL_LEN       600     /* ring-buffer trail for the tip path      */
#define RENDER_FPS       30
#define RENDER_NS       (1000000000LL / RENDER_FPS)
#define CYCLE_FRAMES    360     /* render frames per full shape trace      */
#define AUTO_ADD_FRAMES  12     /* frames between auto-adding one epicycle */
#define N_CIRCLES         6     /* how many arm orbits to draw             */
#define SHAPE_SCALE      0.38f  /* fraction of min(pw,ph)/2 for shape size */
#define CELL_W            8
#define CELL_H           16

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

enum {
    CP_ARM_HI  = 1,   /* large-amplitude arm  — bright white              */
    CP_ARM_MID = 2,   /* medium arm           — cyan                       */
    CP_ARM_LO  = 3,   /* tiny arm             — dark grey                  */
    CP_CIRCLE  = 4,   /* orbit circle dots    — dim grey                   */
    CP_TRAIL_1 = 5,   /* trail newest         — bright yellow              */
    CP_TRAIL_2 = 6,   /* trail mid            — orange                     */
    CP_TRAIL_3 = 7,   /* trail old            — red                        */
    CP_BOB     = 8,   /* tip bob              — bright white               */
    CP_PIVOT   = 9,   /* pivot marker                                      */
    CP_HUD     = 10,
    CP_SHAPE   = 11,
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_ARM_HI,  231, -1);
        init_pair(CP_ARM_MID,  51, -1);
        init_pair(CP_ARM_LO,  238, -1);
        init_pair(CP_CIRCLE,  236, -1);
        init_pair(CP_TRAIL_1, 226, -1);
        init_pair(CP_TRAIL_2, 208, -1);
        init_pair(CP_TRAIL_3, 196, -1);
        init_pair(CP_BOB,     231, -1);
        init_pair(CP_PIVOT,   220, -1);
        init_pair(CP_HUD,     244, -1);
        init_pair(CP_SHAPE,   220, -1);
    } else {
        init_pair(CP_ARM_HI,  COLOR_WHITE,  -1);
        init_pair(CP_ARM_MID, COLOR_CYAN,   -1);
        init_pair(CP_ARM_LO,  COLOR_WHITE,  -1);
        init_pair(CP_CIRCLE,  COLOR_WHITE,  -1);
        init_pair(CP_TRAIL_1, COLOR_YELLOW, -1);
        init_pair(CP_TRAIL_2, COLOR_YELLOW, -1);
        init_pair(CP_TRAIL_3, COLOR_RED,    -1);
        init_pair(CP_BOB,     COLOR_WHITE,  -1);
        init_pair(CP_PIVOT,   COLOR_YELLOW, -1);
        init_pair(CP_HUD,     COLOR_WHITE,  -1);
        init_pair(CP_SHAPE,   COLOR_YELLOW, -1);
    }
}

/* ===================================================================== */
/* §4  shapes                                                             */
/* ===================================================================== */

typedef struct { float re, im; } Cplx;

static const char *k_shape_names[] = {
    "Heart", "Star (5pt)", "Trefoil", "Figure-8", "Butterfly"
};
#define N_SHAPES 5

static void sample_shape(int si, Cplx *out, int N)
{
    for (int k = 0; k < N; k++) {
        float t = (float)k / (float)N * 2.0f * (float)M_PI;
        float x = 0.f, y = 0.f;

        switch (si) {
        case 0:   /* heart */
            x =  16.f * sinf(t)*sinf(t)*sinf(t) / 17.f;
            y = -(13.f*cosf(t) - 5.f*cosf(2.f*t)
                  - 2.f*cosf(3.f*t) - cosf(4.f*t)) / 17.f;
            break;

        case 1: { /* 5-pointed star — linear interpolation between vertices */
            float norm = t / (2.f*(float)M_PI) * 10.f;  /* 0..10 */
            int   seg  = (int)floorf(norm) % 10;
            float frac = norm - floorf(norm);
            float base = -(float)M_PI / 2.f;             /* start at top */
            float a0   = base + (float) seg    * (float)M_PI / 5.f;
            float a1   = base + (float)(seg+1) * (float)M_PI / 5.f;
            float r0   = (seg % 2 == 0) ? 1.f : 0.42f;
            float r1   = (seg % 2 == 0) ? 0.42f : 1.f;
            x = r0*cosf(a0) + (r1*cosf(a1) - r0*cosf(a0)) * frac;
            y = r0*sinf(a0) + (r1*sinf(a1) - r0*sinf(a0)) * frac;
            break;
        }
        case 2:   /* 2-D trefoil projection */
            x = (sinf(t) + 2.f*sinf(2.f*t)) / 3.2f;
            y = (cosf(t) - 2.f*cosf(2.f*t)) / 3.2f;
            break;

        case 3:   /* figure-8 / lemniscate */
            x =  sinf(t);
            y =  sinf(t) * cosf(t);
            break;

        case 4: { /* butterfly curve */
            float e = expf(cosf(t)) - 2.f*cosf(4.f*t)
                      - powf(sinf(t/12.f), 5.f);
            x =  sinf(t) * e / 5.f;
            y = -cosf(t) * e / 5.f;
            break;
        }
        }
        out[k].re = x;
        out[k].im = y;
    }
}

/* ===================================================================== */
/* §5  DFT + epicycle sorting                                             */
/* ===================================================================== */

/*
 * O(N²) DFT using the twiddle-factor recurrence.
 * Computes one cos/sin per frequency bin (instead of N²), then iterates
 * the twiddle by complex multiplication — no extra trig in the inner loop.
 */
static void compute_dft(const Cplx *in, Cplx *out, int N)
{
    for (int n = 0; n < N; n++) {
        /* twiddle: W = exp(-2πi·n/N) */
        float tw_re = cosf(-2.f*(float)M_PI*(float)n/(float)N);
        float tw_im = sinf(-2.f*(float)M_PI*(float)n/(float)N);
        float w_re = 1.f, w_im = 0.f;   /* W^k, starts at W^0 = 1 */
        float re = 0.f,   im = 0.f;
        for (int k = 0; k < N; k++) {
            re += in[k].re * w_re - in[k].im * w_im;
            im += in[k].re * w_im + in[k].im * w_re;
            /* advance W^k → W^(k+1) */
            float tmp = w_re*tw_re - w_im*tw_im;
            w_im      = w_re*tw_im + w_im*tw_re;
            w_re      = tmp;
        }
        out[n].re = re;
        out[n].im = im;
    }
}

typedef struct {
    float amp;    /* |Z[n]| / N  — arm length in normalised units          */
    float phase;  /* arg(Z[n])                                             */
    int   freq;   /* rotation rate (turns per shape cycle); may be negative*/
} Epicycle;

static int epic_cmp(const void *a, const void *b)
{
    float da = ((const Epicycle *)a)->amp;
    float db = ((const Epicycle *)b)->amp;
    return (da < db) - (da > db);   /* descending by amplitude            */
}

static Epicycle g_epics[N_SAMPLES];
static int      g_n_epics;         /* always N_SAMPLES after build        */
static int      g_n_active;        /* arms currently animated (1..N)      */

static void build_epicycles(int shape_idx)
{
    Cplx samples[N_SAMPLES], dft[N_SAMPLES];
    sample_shape(shape_idx, samples, N_SAMPLES);
    compute_dft(samples, dft, N_SAMPLES);

    float inv_N = 1.f / (float)N_SAMPLES;
    for (int n = 0; n < N_SAMPLES; n++) {
        float re  = dft[n].re, im = dft[n].im;
        int   f   = (n <= N_SAMPLES/2) ? n : n - N_SAMPLES;  /* signed freq */
        g_epics[n] = (Epicycle){ sqrtf(re*re + im*im) * inv_N,
                                  atan2f(im, re), f };
    }
    qsort(g_epics, N_SAMPLES, sizeof(Epicycle), epic_cmp);
    g_n_epics = N_SAMPLES;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    float px[TRAIL_LEN], py[TRAIL_LEN];
    int   head, count;
} Trail;

static void trail_push(Trail *t, float px, float py)
{
    t->px[t->head] = px;  t->py[t->head] = py;
    t->head = (t->head + 1) % TRAIL_LEN;
    if (t->count < TRAIL_LEN) t->count++;
}
static void trail_clear(Trail *t) { t->head = t->count = 0; }

/* scene globals */
static int    g_rows, g_cols;
static float  g_cx, g_cy;      /* pivot, pixel space                      */
static float  g_scale;         /* normalised → pixel                       */
static float  g_phi;           /* animation angle [0, 2π)                  */
static int    g_frame;
static int    g_auto_cnt;
static bool   g_paused;
static bool   g_auto_add;
static bool   g_show_circles;
static int    g_shape;
static Trail  g_trail;

/* tip positions for the arm chain (pixel space, N_SAMPLES+1 entries) */
static float g_tips_px[N_SAMPLES + 1];
static float g_tips_py[N_SAMPLES + 1];

static void scene_compute_chain(void)
{
    float x = g_cx, y = g_cy;
    g_tips_px[0] = x;  g_tips_py[0] = y;
    for (int i = 0; i < g_n_active; i++) {
        float ang = (float)g_epics[i].freq * g_phi + g_epics[i].phase;
        float r   = g_epics[i].amp * g_scale;
        x += r * cosf(ang);
        y += r * sinf(ang);
        g_tips_px[i+1] = x;
        g_tips_py[i+1] = y;
    }
}

static void scene_reset(int shape_idx)
{
    g_shape    = shape_idx;
    g_phi      = 0.f;
    g_frame    = 0;
    g_auto_cnt = 0;
    g_n_active = 1;
    trail_clear(&g_trail);
    build_epicycles(g_shape);
    scene_compute_chain();
}

static void scene_init(int rows, int cols)
{
    g_rows = rows;  g_cols = cols;
    g_cx   = (float)(cols * CELL_W) * 0.5f;
    g_cy   = (float)(rows * CELL_H) * 0.5f;
    float mn = fminf((float)(cols * CELL_W), (float)(rows * CELL_H));
    g_scale = mn * SHAPE_SCALE;
    g_paused      = false;
    g_auto_add    = true;
    g_show_circles = true;
    scene_reset(g_shape);
}

static void scene_tick(void)
{
    if (g_paused) return;
    g_frame++;

    /* auto-add one epicycle at intervals until all are active */
    if (g_auto_add && g_n_active < g_n_epics) {
        g_auto_cnt++;
        if (g_auto_cnt >= AUTO_ADD_FRAMES) {
            g_auto_cnt = 0;
            g_n_active++;
        }
    }

    /* advance animation angle */
    g_phi += 2.f * (float)M_PI / (float)CYCLE_FRAMES;
    if (g_phi >= 2.f * (float)M_PI) {
        g_phi -= 2.f * (float)M_PI;
        trail_clear(&g_trail);   /* clean slate each completed cycle */
    }

    scene_compute_chain();

    /* record tip in trail */
    trail_push(&g_trail,
               g_tips_px[g_n_active],
               g_tips_py[g_n_active]);
}

/* ── draw helpers ────────────────────────────────────────────────────── */

static int px_cx(float px) { return (int)(px / CELL_W + 0.5f); }
static int px_cy(float py) { return (int)(py / CELL_H + 0.5f); }

static void draw_line(int x0, int y0, int x1, int y1, attr_t attr)
{
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = x0<x1?1:-1, sy = y0<y1?1:-1;
    int err = dx - dy;
    for (;;) {
        if (x0>=0 && x0<g_cols && y0>=0 && y0<g_rows) {
            int   e2 = 2*err;
            bool  bx = e2 > -dy, by = e2 < dx;
            chtype ch = (bx && by) ? (sx==sy ? '\\' : '/')
                      : bx ? '-' : '|';
            attron(attr);
            mvaddch(y0, x0, ch);
            attroff(attr);
        }
        if (x0==x1 && y0==y1) break;
        int e2 = 2*err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/*
 * draw_orbit() — draw the orbit ellipse for an arm whose pivot is at
 * (pcx, pcy) in cell coords and whose amplitude is r_px pixels.
 * Because cells are CELL_H:CELL_W ≈ 2:1, a circular orbit in pixel
 * space appears as an ellipse in cell space.
 */
static void draw_orbit(float piv_px, float piv_py, float r_px)
{
    if (r_px < (float)CELL_W * 0.5f) return;   /* too small to bother   */
    float rx = r_px / (float)CELL_W;            /* cell-space semi-axes  */
    float ry = r_px / (float)CELL_H;
    int   pcx = px_cx(piv_px), pcy = px_cy(piv_py);
    int   steps = (int)(2.f*(float)M_PI * fmaxf(rx, ry)) + 4;
    attr_t attr = COLOR_PAIR(CP_CIRCLE);
    for (int i = 0; i < steps; i++) {
        float a = 2.f*(float)M_PI*(float)i/(float)steps;
        int x = pcx + (int)(rx * cosf(a) + 0.5f);
        int y = pcy + (int)(ry * sinf(a) + 0.5f);
        if (x>=0 && x<g_cols && y>=0 && y<g_rows) {
            attron(attr);
            mvaddch(y, x, '.');
            attroff(attr);
        }
    }
}

static void scene_draw(void)
{
    /* ── 1. orbit circles (back layer) ──────────────────────────────── */
    if (g_show_circles) {
        int nc = g_n_active < N_CIRCLES ? g_n_active : N_CIRCLES;
        for (int i = 0; i < nc; i++) {
            float r_px = g_epics[i].amp * g_scale;
            draw_orbit(g_tips_px[i], g_tips_py[i], r_px);
        }
    }

    /* ── 2. trail ────────────────────────────────────────────────────── */
    int draw = g_trail.count;
    int start = (g_trail.head - draw + TRAIL_LEN) % TRAIL_LEN;
    for (int i = 0; i < draw; i++) {
        int idx = (start + i) % TRAIL_LEN;
        int cx  = px_cx(g_trail.px[idx]);
        int cy  = px_cy(g_trail.py[idx]);
        if (cx<0 || cx>=g_cols || cy<0 || cy>=g_rows) continue;
        float age = (float)i / (float)draw;   /* 0=oldest, 1=newest */
        int   cp  = age > 0.70f ? CP_TRAIL_1
                  : age > 0.35f ? CP_TRAIL_2 : CP_TRAIL_3;
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvaddch(cy, cx, '*');
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }

    /* ── 3. arm chain ────────────────────────────────────────────────── */
    for (int i = 0; i < g_n_active; i++) {
        float r_px = g_epics[i].amp * g_scale;
        int   cp   = r_px > g_scale * 0.10f ? CP_ARM_HI
                   : r_px > g_scale * 0.02f ? CP_ARM_MID : CP_ARM_LO;
        draw_line(px_cx(g_tips_px[i]),   px_cy(g_tips_py[i]),
                  px_cx(g_tips_px[i+1]), px_cy(g_tips_py[i+1]),
                  COLOR_PAIR(cp) | A_BOLD);
    }

    /* ── 4. tip bob ──────────────────────────────────────────────────── */
    int bx = px_cx(g_tips_px[g_n_active]);
    int by = px_cy(g_tips_py[g_n_active]);
    if (bx>=0 && bx<g_cols && by>=0 && by<g_rows) {
        attron(COLOR_PAIR(CP_BOB) | A_BOLD);
        mvaddch(by, bx, '@');
        attroff(COLOR_PAIR(CP_BOB) | A_BOLD);
    }

    /* ── 5. pivot marker ─────────────────────────────────────────────── */
    int piv_cx = px_cx(g_cx), piv_cy = px_cy(g_cy);
    if (piv_cx>=0 && piv_cx<g_cols && piv_cy>=0 && piv_cy<g_rows) {
        attron(COLOR_PAIR(CP_PIVOT) | A_BOLD);
        mvaddch(piv_cy, piv_cx, '+');
        attroff(COLOR_PAIR(CP_PIVOT) | A_BOLD);
    }

    /* ── 6. HUD ──────────────────────────────────────────────────────── */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
             " Epicycles  q quit  n/p shape  +/- epics  r reset  a auto  c circles  spc pause");
    attroff(COLOR_PAIR(CP_HUD));
    attron(COLOR_PAIR(CP_SHAPE) | A_BOLD);
    mvprintw(1, 0, " %-12s  epics: %3d / %d  %s  %s",
             k_shape_names[g_shape], g_n_active, g_n_epics,
             g_auto_add    ? "auto"    : "manual",
             g_show_circles ? "circles" : "");
    attroff(COLOR_PAIR(CP_SHAPE) | A_BOLD);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

static void screen_init(void)
{
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT  || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)                g_resize = 1;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    screen_init();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_shape = 0;
    scene_init(rows, cols);

    long long last_ns = clock_ns();

    while (!g_quit) {
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            scene_init(rows, cols);
            last_ns = clock_ns();
            continue;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1;                         break;
        case ' ':                    g_paused = !g_paused;                break;
        case 'n': case 'N': scene_reset((g_shape + 1) % N_SHAPES);       break;
        case 'p': case 'P': scene_reset((g_shape + N_SHAPES-1) % N_SHAPES); break;
        case '+': case '=':
            if (g_n_active < g_n_epics) g_n_active++;
            break;
        case '-':
            if (g_n_active > 1) { g_n_active--; trail_clear(&g_trail); } break;
        case 'r': case 'R': scene_reset(g_shape);                        break;
        case 'a': case 'A': g_auto_add = !g_auto_add;                    break;
        case 'c': case 'C': g_show_circles = !g_show_circles;            break;
        }

        long long now_ns = clock_ns();
        long long dt     = now_ns - last_ns;
        last_ns = now_ns;
        if (dt > 100000000LL) dt = 100000000LL;

        /* one tick per render frame keeps the code simple; physics is trivial */
        scene_tick();

        erase();
        scene_draw();
        screen_present();

        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }
    return 0;
}
