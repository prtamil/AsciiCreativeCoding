/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fourier_draw.c — Fourier Epicycle Path Reconstruction
 *
 * DFT of a sampled closed path → animated epicycle arm chain whose tip traces
 * the original shape.  A ghost overlay of the source samples stays on screen
 * so you can watch the reconstruction converge toward the target.
 * An energy bar shows the fraction of signal power captured by the active arms.
 *
 * Shapes   : Square  Arrow  Star-7  Cardioid  Lissajous 3:2  Rose r=cos(2t)
 * Keys     : q/ESC quit   spc pause   n/p shape   +/- arms   r reset
 *            a auto-add   c circles   g ghost path
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/fourier_draw.c -o fourier_draw -lncurses -lm
 *
 * Sections  §1 config  §2 clock  §3 color  §4 paths  §5 DFT
 *           §6 scene   §7 screen §8 app
 *
 * Design note
 * -----------
 * This program deliberately differs from epicycles.c:
 *   • Shapes are defined as sampled point arrays (polygons + closed curves),
 *     not smooth parametric formulas.  This exposes Gibbs-phenomenon ringing
 *     near discontinuities — most visible on Square and Arrow.
 *   • The ghost path overlay shows the true source so the viewer can gauge
 *     how well the active arms reconstruct the original.
 *   • The energy bar is derived from Parseval's theorem: sorted-by-amplitude
 *     epicycles give a greedy optimal approximation; bar tracks cumulative
 *     power fraction.
 */

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

#define N_SAMPLES       256     /* DFT input size; also max epicycles          */
#define TRAIL_LEN       512     /* ring-buffer length for tip trail             */
#define RENDER_FPS       30
#define RENDER_NS       (1000000000LL / RENDER_FPS)
#define CYCLE_FRAMES    360     /* render frames per full shape trace           */
#define AUTO_ADD_FRAMES   8     /* frames between auto-adding one epicycle      */
#define N_CIRCLES         5     /* max orbit rings drawn                        */
#define SHAPE_SCALE      0.38f  /* fraction of min(pw,ph)/2 used for shape size */
#define CELL_W            8     /* physical pixels per terminal column           */
#define CELL_H           16     /* physical pixels per terminal row              */

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
    CP_ARM_HI  =  1,   /* large-amplitude arm  — bright white                  */
    CP_ARM_MID =  2,   /* medium arm           — aqua                           */
    CP_ARM_LO  =  3,   /* tiny arm             — dark grey                      */
    CP_CIRCLE  =  4,   /* orbit ring dots      — very dark grey                 */
    CP_TRAIL_1 =  5,   /* trail newest         — yellow                         */
    CP_TRAIL_2 =  6,   /* trail mid            — orange                         */
    CP_TRAIL_3 =  7,   /* trail oldest         — red                            */
    CP_BOB     =  8,   /* tip dot              — bright white                   */
    CP_PIVOT   =  9,   /* pivot marker         — gold                           */
    CP_HUD     = 10,   /* HUD text             — grey                           */
    CP_GHOST   = 11,   /* ghost sample dots    — dim blue-grey                  */
    CP_SHAPE   = 12,   /* shape name / status  — lavender                       */
    CP_ENERGY  = 13,   /* energy bar fill      — green                          */
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_ARM_HI,  231, -1);   /* xterm bright white   */
        init_pair(CP_ARM_MID,  87, -1);   /* aqua                 */
        init_pair(CP_ARM_LO,  238, -1);   /* dark grey            */
        init_pair(CP_CIRCLE,  235, -1);   /* near-black grey      */
        init_pair(CP_TRAIL_1, 226, -1);   /* yellow               */
        init_pair(CP_TRAIL_2, 208, -1);   /* orange               */
        init_pair(CP_TRAIL_3, 196, -1);   /* red                  */
        init_pair(CP_BOB,     231, -1);   /* white                */
        init_pair(CP_PIVOT,   220, -1);   /* gold                 */
        init_pair(CP_HUD,     244, -1);   /* mid grey             */
        init_pair(CP_GHOST,   240, -1);   /* dim blue-grey        */
        init_pair(CP_SHAPE,   147, -1);   /* lavender             */
        init_pair(CP_ENERGY,   46, -1);   /* bright green         */
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
        init_pair(CP_GHOST,   COLOR_WHITE,  -1);
        init_pair(CP_SHAPE,   COLOR_CYAN,   -1);
        init_pair(CP_ENERGY,  COLOR_GREEN,  -1);
    }
}

/* ===================================================================== */
/* §4  paths — sample_path() fills N complex points for each shape       */
/* ===================================================================== */

typedef struct { float re, im; } Cplx;

static const char *k_shape_names[] = {
    "Square", "Arrow", "Star-7", "Cardioid", "Lissajous 3:2", "Rose r=cos(2t)"
};
#define N_SHAPES 6

/*
 * sample_poly() — linearly interpolate around a polygon with n_verts vertices.
 *
 * Vertices are given in normalized coordinates.  The result is N_SAMPLES
 * evenly-spaced complex points along the boundary.  Sharp corners produce
 * Gibbs ringing when reconstructed with few Fourier terms.
 */
static void sample_poly(const float *vx, const float *vy, int nv,
                        Cplx *out, int N)
{
    for (int i = 0; i < N; i++) {
        float t    = (float)i / (float)N * (float)nv;
        int   seg  = (int)t % nv;
        float frac = t - floorf(t);
        int   nxt  = (seg + 1) % nv;
        out[i].re  = vx[seg] + (vx[nxt] - vx[seg]) * frac;
        out[i].im  = vy[seg] + (vy[nxt] - vy[seg]) * frac;
    }
}

static void sample_path(int si, Cplx *out, int N)
{
    switch (si) {

    case 0: {
        /*
         * Square — 4 corners, linear sides.
         * Sharp 90° corners require many harmonics → prominent Gibbs overshoot
         * near corners with few arms, then convergence to a clean square.
         */
        static const float sx[] = {-1.f,  1.f,  1.f, -1.f};
        static const float sy[] = {-1.f, -1.f,  1.f,  1.f};
        sample_poly(sx, sy, 4, out, N);
        break;
    }

    case 1: {
        /*
         * Arrow (pointing up) — 7-vertex polygon.
         * The notched tail creates two more sharp re-entrant corners.
         */
        static const float ax[] = { 0.f,  .65f,  .30f,  .30f, -.30f, -.30f, -.65f};
        static const float ay[] = { 1.f,  .10f,  .10f, -.85f, -.85f,  .10f,  .10f};
        sample_poly(ax, ay, 7, out, N);
        break;
    }

    case 2: {
        /*
         * 7-pointed star — alternating outer (r=1) and inner (r=0.40) radii.
         * More spiky than a 5-point star; needs more terms to resolve the tips.
         */
        float vx[14], vy[14];
        for (int i = 0; i < 14; i++) {
            float r = (i % 2 == 0) ? 1.f : 0.40f;
            float a = -(float)M_PI/2.f + (float)i * (float)M_PI / 7.f;
            vx[i] = r * cosf(a);
            vy[i] = r * sinf(a);
        }
        sample_poly(vx, vy, 14, out, N);
        break;
    }

    case 3: {
        /*
         * Cardioid: r = 1 - cos(t) (polar), recentered so the cusp is at left.
         * Smooth but asymmetric; centroid is offset — we subtract it numerically.
         */
        float cx = 0.f;
        for (int k = 0; k < N; k++) {
            float t    = 2.f*(float)M_PI*(float)k/(float)N;
            float r    = 0.5f * (1.f - cosf(t));
            out[k].re  = r * cosf(t);
            out[k].im  = r * sinf(t);
            cx        += out[k].re;
        }
        cx /= (float)N;
        for (int k = 0; k < N; k++) out[k].re -= cx;
        break;
    }

    case 4: {
        /*
         * Lissajous 3:2 with π/4 phase offset.
         * Closes at t=2π; produces a self-intersecting figure-8-like knot.
         */
        for (int k = 0; k < N; k++) {
            float t   = 2.f*(float)M_PI*(float)k/(float)N;
            out[k].re = sinf(3.f*t + (float)M_PI/4.f);
            out[k].im = sinf(2.f*t);
        }
        break;
    }

    case 5: {
        /*
         * 4-petal rose: r = cos(2t).
         * r can be negative (back-projection); all 4 petals are traced over
         * [0, 2π].  Ghost dots reveal the 4-fold symmetry.
         */
        for (int k = 0; k < N; k++) {
            float t   = 2.f*(float)M_PI*(float)k/(float)N;
            float r   = cosf(2.f*t);
            out[k].re = r * cosf(t);
            out[k].im = r * sinf(t);
        }
        break;
    }

    }
}

/* ===================================================================== */
/* §5  DFT + epicycle sorting                                            */
/* ===================================================================== */

/*
 * O(N²) DFT using the twiddle recurrence (one cos/sin per freq, then
 * complex multiply to advance the twiddle factor).
 */
static void compute_dft(const Cplx *in, Cplx *out, int N)
{
    for (int n = 0; n < N; n++) {
        float tw_re = cosf(-2.f*(float)M_PI*(float)n/(float)N);
        float tw_im = sinf(-2.f*(float)M_PI*(float)n/(float)N);
        float w_re = 1.f, w_im = 0.f;
        float re   = 0.f, im   = 0.f;
        for (int k = 0; k < N; k++) {
            re += in[k].re * w_re - in[k].im * w_im;
            im += in[k].re * w_im + in[k].im * w_re;
            float tmp = w_re*tw_re - w_im*tw_im;
            w_im      = w_re*tw_im + w_im*tw_re;
            w_re      = tmp;
        }
        out[n].re = re;
        out[n].im = im;
    }
}

typedef struct {
    float amp;    /* normalised arm length = |Z[n]| / N                       */
    float phase;  /* arg(Z[n])                                                */
    int   freq;   /* signed rotation rate (cycles per shape trace)            */
} Epicycle;

static int epic_cmp(const void *a, const void *b)
{
    float da = ((const Epicycle *)a)->amp;
    float db = ((const Epicycle *)b)->amp;
    return (da < db) - (da > db);   /* descending by amplitude               */
}

static Epicycle g_epics[N_SAMPLES];
static int      g_n_epics;
static int      g_n_active;

/*
 * g_cum_energy[k] = fraction of total signal power captured by the first
 *                   k+1 epicycles (sorted by amplitude, so this is the
 *                   greedy-optimal approximation quality).
 *
 * Parseval connection: total_energy = Σ amp² = (1/N²) Σ |Z[n]|²
 *                                    = (1/N) Σ |x[k]|²
 */
static float g_cum_energy[N_SAMPLES];
static float g_total_energy;

/* raw source samples stored for ghost overlay */
static Cplx g_ghost[N_SAMPLES];

static void build_epicycles(int shape_idx)
{
    Cplx dft[N_SAMPLES];
    sample_path(shape_idx, g_ghost, N_SAMPLES);
    compute_dft(g_ghost, dft, N_SAMPLES);

    float inv_N = 1.f / (float)N_SAMPLES;
    for (int n = 0; n < N_SAMPLES; n++) {
        float re = dft[n].re, im = dft[n].im;
        int   f  = (n <= N_SAMPLES/2) ? n : n - N_SAMPLES;   /* signed freq */
        g_epics[n] = (Epicycle){ sqrtf(re*re + im*im)*inv_N, atan2f(im,re), f };
    }
    qsort(g_epics, N_SAMPLES, sizeof(Epicycle), epic_cmp);
    g_n_epics = N_SAMPLES;

    /* build cumulative energy table */
    g_total_energy = 0.f;
    for (int n = 0; n < N_SAMPLES; n++)
        g_total_energy += g_epics[n].amp * g_epics[n].amp;

    float acc = 0.f;
    for (int n = 0; n < N_SAMPLES; n++) {
        acc += g_epics[n].amp * g_epics[n].amp;
        g_cum_energy[n] = (g_total_energy > 0.f) ? acc / g_total_energy : 0.f;
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/* ── trail ring buffer ──────────────────────────────────────────────── */

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

/* ── scene globals ──────────────────────────────────────────────────── */

static int   g_rows, g_cols;
static float g_cx,   g_cy;      /* pivot in pixel space                      */
static float g_scale;           /* normalised unit → pixels                   */
static float g_phi;             /* animation angle [0, 2π)                    */
static int   g_auto_cnt;
static bool  g_paused;
static bool  g_auto_add;
static bool  g_show_circles;
static bool  g_show_ghost;
static int   g_shape;
static Trail g_trail;

/* tip positions along the arm chain, in pixel space (N_SAMPLES+1 entries) */
static float g_tips_px[N_SAMPLES + 1];
static float g_tips_py[N_SAMPLES + 1];

/* ── coordinate helpers ─────────────────────────────────────────────── */

static int px_cx(float px) { return (int)(px / (float)CELL_W + 0.5f); }
static int px_cy(float py) { return (int)(py / (float)CELL_H + 0.5f); }

/* ── chain computation ──────────────────────────────────────────────── */

static void scene_compute_chain(void)
{
    float x = g_cx, y = g_cy;
    g_tips_px[0] = x;  g_tips_py[0] = y;
    for (int i = 0; i < g_n_active; i++) {
        float ang  = (float)g_epics[i].freq * g_phi + g_epics[i].phase;
        float r    = g_epics[i].amp * g_scale;
        x += r * cosf(ang);
        y += r * sinf(ang);
        g_tips_px[i+1] = x;
        g_tips_py[i+1] = y;
    }
}

/* ── init / reset ───────────────────────────────────────────────────── */

static void scene_reset(int shape_idx)
{
    g_shape    = shape_idx;
    g_phi      = 0.f;
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
    g_scale      = mn * SHAPE_SCALE;
    g_paused     = false;
    g_auto_add   = true;
    g_show_circles = true;
    g_show_ghost = true;
    scene_reset(0);
}

/* ── tick ───────────────────────────────────────────────────────────── */

static void scene_tick(void)
{
    if (g_paused) return;

    /* auto-add one arm every AUTO_ADD_FRAMES until all are active */
    if (g_auto_add && g_n_active < g_n_epics) {
        g_auto_cnt++;
        if (g_auto_cnt >= AUTO_ADD_FRAMES) {
            g_auto_cnt = 0;
            g_n_active++;
        }
    }

    /* advance animation angle; clear trail at each full cycle */
    g_phi += 2.f * (float)M_PI / (float)CYCLE_FRAMES;
    if (g_phi >= 2.f * (float)M_PI) {
        g_phi -= 2.f * (float)M_PI;
        trail_clear(&g_trail);
    }

    scene_compute_chain();
    trail_push(&g_trail, g_tips_px[g_n_active], g_tips_py[g_n_active]);
}

/* ── draw helpers ───────────────────────────────────────────────────── */

/*
 * draw_line_seg() — Bresenham line with direction-aware character selection.
 *   Mostly-horizontal segments → '-'
 *   Mostly-vertical   segments → '|'
 *   Diagonal (\ or /)         → '\\' / '/'
 */
static void draw_line_seg(int x0, int y0, int x1, int y1, attr_t attr)
{
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = x0<x1?1:-1, sy = y0<y1?1:-1;
    int err = dx - dy;
    for (;;) {
        if (x0>=0 && x0<g_cols && y0>=0 && y0<g_rows) {
            int  e2 = 2*err;
            bool bx = e2 > -dy, by = e2 < dx;
            chtype ch = (bx && by) ? (sx==sy ? '\\' : '/')
                      :  bx ? '-' : '|';
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
 * draw_orbit() — dotted ellipse for the orbit of one arm.
 * Pixel-space circle appears as a cell-space ellipse because CELL_H > CELL_W.
 */
static void draw_orbit(float piv_px, float piv_py, float r_px)
{
    if (r_px < (float)CELL_W * 0.5f) return;   /* skip sub-cell orbits */
    float rx    = r_px / (float)CELL_W;
    float ry    = r_px / (float)CELL_H;
    int   pcx   = px_cx(piv_px), pcy = px_cy(piv_py);
    int   steps = (int)(2.f*(float)M_PI * fmaxf(rx, ry)) + 4;
    for (int i = 0; i < steps; i++) {
        float a = 2.f*(float)M_PI*(float)i/(float)steps;
        int x   = pcx + (int)(rx*cosf(a)+0.5f);
        int y   = pcy + (int)(ry*sinf(a)+0.5f);
        if (x>=0 && x<g_cols && y>=0 && y<g_rows) {
            attron(COLOR_PAIR(CP_CIRCLE));
            mvaddch(y, x, '.');
            attroff(COLOR_PAIR(CP_CIRCLE));
        }
    }
}

/* ── main draw function ─────────────────────────────────────────────── */

static void scene_draw(void)
{
    /* ── 1. ghost path: source sample dots ─────────────────────────── */
    /*
     * Every other sample is drawn to keep density readable.
     * Shows exactly where the "true" shape lies so the viewer can
     * compare the epicycle reconstruction quality directly.
     */
    if (g_show_ghost) {
        attron(COLOR_PAIR(CP_GHOST));
        for (int k = 0; k < N_SAMPLES; k += 2) {
            float px = g_cx + g_ghost[k].re * g_scale;
            float py = g_cy + g_ghost[k].im * g_scale;
            int   cx = px_cx(px), cy = px_cy(py);
            if (cx>=0 && cx<g_cols && cy>=0 && cy<g_rows)
                mvaddch(cy, cx, ':');
        }
        attroff(COLOR_PAIR(CP_GHOST));
    }

    /* ── 2. orbit circles ───────────────────────────────────────────── */
    if (g_show_circles) {
        int nc = g_n_active < N_CIRCLES ? g_n_active : N_CIRCLES;
        for (int i = 0; i < nc; i++)
            draw_orbit(g_tips_px[i], g_tips_py[i], g_epics[i].amp * g_scale);
    }

    /* ── 3. trail ───────────────────────────────────────────────────── */
    {
        int draw  = g_trail.count;
        int start = (g_trail.head - draw + TRAIL_LEN) % TRAIL_LEN;
        for (int i = 0; i < draw; i++) {
            int   idx = (start + i) % TRAIL_LEN;
            int   cx  = px_cx(g_trail.px[idx]);
            int   cy  = px_cy(g_trail.py[idx]);
            if (cx<0||cx>=g_cols||cy<0||cy>=g_rows) continue;
            float age = (float)i / (float)draw;   /* 0 = oldest, 1 = newest */
            int   cp  = age > 0.70f ? CP_TRAIL_1
                      : age > 0.35f ? CP_TRAIL_2 : CP_TRAIL_3;
            attron(COLOR_PAIR(cp)|A_BOLD);
            mvaddch(cy, cx, '*');
            attroff(COLOR_PAIR(cp)|A_BOLD);
        }
    }

    /* ── 4. arm chain ───────────────────────────────────────────────── */
    for (int i = 0; i < g_n_active; i++) {
        float r_px = g_epics[i].amp * g_scale;
        int   cp   = r_px > g_scale * 0.10f ? CP_ARM_HI
                   : r_px > g_scale * 0.02f ? CP_ARM_MID : CP_ARM_LO;
        draw_line_seg(px_cx(g_tips_px[i]),   px_cy(g_tips_py[i]),
                      px_cx(g_tips_px[i+1]), px_cy(g_tips_py[i+1]),
                      COLOR_PAIR(cp) | A_BOLD);
    }

    /* ── 5. tip bob ─────────────────────────────────────────────────── */
    {
        int bx = px_cx(g_tips_px[g_n_active]);
        int by = px_cy(g_tips_py[g_n_active]);
        if (bx>=0&&bx<g_cols&&by>=0&&by<g_rows) {
            attron(COLOR_PAIR(CP_BOB)|A_BOLD);
            mvaddch(by, bx, '@');
            attroff(COLOR_PAIR(CP_BOB)|A_BOLD);
        }
    }

    /* ── 6. pivot marker ────────────────────────────────────────────── */
    {
        int pcx = px_cx(g_cx), pcy = px_cy(g_cy);
        if (pcx>=0&&pcx<g_cols&&pcy>=0&&pcy<g_rows) {
            attron(COLOR_PAIR(CP_PIVOT)|A_BOLD);
            mvaddch(pcy, pcx, '+');
            attroff(COLOR_PAIR(CP_PIVOT)|A_BOLD);
        }
    }

    /* ── 7. HUD ─────────────────────────────────────────────────────── */

    /* key hints */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " FourierDraw  q:quit  n/p:shape  +/-:arms  r:reset  a:auto  c:circles  g:ghost  spc:pause");
    attroff(COLOR_PAIR(CP_HUD));

    /* shape name + arm count + energy percentage */
    float efrac = (g_n_active > 0 && g_total_energy > 0.f)
                ? g_cum_energy[g_n_active - 1] : 0.f;

    attron(COLOR_PAIR(CP_SHAPE)|A_BOLD);
    mvprintw(1, 0, " %-18s  arms: %3d/%d  energy: %5.1f%%  %s  %s  %s",
             k_shape_names[g_shape],
             g_n_active, g_n_epics,
             efrac * 100.f,
             g_auto_add     ? "auto"    : "manual",
             g_show_circles ? "circles" : "      ",
             g_show_ghost   ? "ghost"   : "     ");
    attroff(COLOR_PAIR(CP_SHAPE)|A_BOLD);

    /* energy bar — greedy power fraction of active epicycles */
    {
        int bar_col = 1, bar_row = 2, bar_w = 42;
        attron(COLOR_PAIR(CP_HUD));
        mvaddch(bar_row, bar_col, '[');
        attroff(COLOR_PAIR(CP_HUD));

        int filled = (int)(efrac * (float)(bar_w - 2));
        attron(COLOR_PAIR(CP_ENERGY)|A_BOLD);
        for (int i = 0; i < filled; i++)
            mvaddch(bar_row, bar_col + 1 + i, '=');
        attroff(COLOR_PAIR(CP_ENERGY)|A_BOLD);

        attron(COLOR_PAIR(CP_HUD));
        for (int i = filled; i < bar_w - 2; i++)
            mvaddch(bar_row, bar_col + 1 + i, '-');
        mvaddch(bar_row, bar_col + bar_w - 1, ']');
        mvprintw(bar_row, bar_col + bar_w + 1, "power");
        attroff(COLOR_PAIR(CP_HUD));
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

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
    scene_init(rows, cols);

    long long last_ns = clock_ns();

    while (!g_quit) {

        /* ── resize ─────────────────────────────────────────────────── */
        if (g_resize) {
            g_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_init(rows, cols);
            last_ns = clock_ns();
            continue;
        }

        /* ── input ───────────────────────────────────────────────────── */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27:
            g_quit = 1;
            break;
        case ' ':
            g_paused = !g_paused;
            break;
        case 'n': case 'N':
            scene_reset((g_shape + 1) % N_SHAPES);
            break;
        case 'p': case 'P':
            scene_reset((g_shape + N_SHAPES - 1) % N_SHAPES);
            break;
        case '+': case '=':
            if (g_n_active < g_n_epics) g_n_active++;
            break;
        case '-':
            if (g_n_active > 1) { g_n_active--; trail_clear(&g_trail); }
            break;
        case 'r': case 'R':
            scene_reset(g_shape);
            break;
        case 'a': case 'A':
            g_auto_add = !g_auto_add;
            break;
        case 'c': case 'C':
            g_show_circles = !g_show_circles;
            break;
        case 'g': case 'G':
            g_show_ghost = !g_show_ghost;
            break;
        default: break;
        }

        /* ── tick ────────────────────────────────────────────────────── */
        long long now_ns = clock_ns();
        long long dt     = now_ns - last_ns;
        last_ns = now_ns;
        if (dt > 100000000LL) dt = 100000000LL;   /* pause guard: cap at 100 ms */
        (void)dt;

        scene_tick();

        /* ── draw + present ──────────────────────────────────────────── */
        erase();
        scene_draw();
        screen_present();

        /* ── frame cap ───────────────────────────────────────────────── */
        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }

    return 0;
}
