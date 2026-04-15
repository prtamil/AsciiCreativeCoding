/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fourier_art.c — Real-time Fourier Drawing
 *
 * Draw any path on screen with cursor keys, press ENTER, and watch an
 * epicycle arm chain reconstruct your drawing in real time.
 *
 * Algorithm:
 *   1. User traces a path with arrow keys; raw cursor positions recorded.
 *   2. On ENTER: resample path to N_SAMPLES points using arc-length
 *      parameterisation (equal spacing along the drawn path).
 *   3. Compute O(N²) DFT: Z[n] = Σ_k z[k]·exp(-2πi·n·k/N)
 *   4. Sort N epicycles by amplitude |Z[n]|/N (largest arm first).
 *   5. Animate: at angle φ, arm n contributes
 *        amp_n · exp(i·(freq_n·φ + phase_n))
 *      chained from the path centroid. Tip traces the reconstructed curve.
 *   6. Press r to draw a new path.
 *
 * DRAW mode keys:
 *   Arrow / WASD   move cursor and record path
 *   ENTER / g      compute DFT → animate
 *   c              clear path, start over
 *   q / ESC        quit
 *
 * PLAY mode keys:
 *   r              back to draw mode
 *   p / space      pause / resume
 *   + / -          add / remove one epicycle
 *   c              toggle orbit circles
 *   t / T          next / previous theme
 *   a              toggle auto-add
 *   q / ESC        quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/fourier_art.c \
 *       -o fourier_art -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 dft  §5 scene  §6 screen  §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Interactive path-recording → DFT epicycle reconstruction.
 *                  User traces a free-form path; on ENTER the path is
 *                  resampled to N_SAMPLES points using arc-length
 *                  parameterisation (uniform spacing along the drawn curve),
 *                  then an O(N²) DFT converts it to epicycle coefficients.
 *
 * Math           : Arc-length resampling: cumulative chord-length distances
 *                  are computed, then uniform sample positions are mapped
 *                  back to original points via linear interpolation.  This
 *                  ensures the DFT receives uniform-time samples regardless
 *                  of how fast the user drew, eliminating velocity artefacts.
 *                  DFT of complex path: z[k] = x[k] + i·y[k]; Z[n] gives
 *                  the n-th epicycle amplitude and initial phase.
 *
 * Performance    : DFT is computed once (O(N²)) at draw-time, not per frame.
 *                  Per-frame cost is O(N_active) arm chain evaluations.
 *                  RAW_MAX=8192 points buffered during draw; resampled to
 *                  N_SAMPLES=256 before DFT to bound computation.
 *
 * Rendering      : Dual-mode: DRAW mode (cursor + recorded trail) and PLAY
 *                  mode (epicycle arm chain + reconstructed tip trail).
 *                  Orbit circles toggled with 'c' show each arm's radius.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_SAMPLES       256     /* DFT resolution — also max epicycles    */
#define RAW_MAX        8192     /* max raw cursor points collected         */
#define TRAIL_LEN       600     /* ring-buffer for the tip path trail      */
#define RENDER_FPS       30
#define RENDER_NS       (1000000000LL / RENDER_FPS)
#define CYCLE_FRAMES    300     /* frames per full reconstruction cycle    */
#define AUTO_ADD_FRAMES   8     /* frames between auto-adding one epicycle */
#define N_CIRCLES         6     /* orbit circles drawn (largest arms)      */
#define CELL_W            8     /* pixels per terminal cell, x             */
#define CELL_H           16     /* pixels per terminal cell, y             */
#define ROWS_MAX        128
#define COLS_MAX        512
#define N_THEMES          5

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
/* §3  color / theme                                                      */
/* ===================================================================== */

enum {
    CP_ARM_HI  = 1,   /* large arm          */
    CP_ARM_MID = 2,   /* medium arm         */
    CP_ARM_LO  = 3,   /* tiny arm           */
    CP_CIRCLE  = 4,   /* orbit circle dots  */
    CP_TRAIL_1 = 5,   /* trail newest       */
    CP_TRAIL_2 = 6,   /* trail middle       */
    CP_TRAIL_3 = 7,   /* trail oldest       */
    CP_BOB     = 8,   /* tip '@'            */
    CP_PIVOT   = 9,   /* centre pivot       */
    CP_HUD     = 10,  /* status bar         */
    CP_CURSOR  = 11,  /* draw-mode cursor   */
    CP_PATH    = 12,  /* draw-mode path     */
};

typedef struct {
    /* 256-color fg for each role */
    short arm_hi, arm_mid, arm_lo;
    short circle, trail1, trail2, trail3;
    short bob, pivot, hud, cursor, path;
    /* 8-color fallbacks */
    short arm_hi8, arm_mid8, arm_lo8;
    short circle8, trail18, trail28, trail38;
    short bob8, pivot8, hud8, cursor8, path8;
    const char *name;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0 Classic — white arms, yellow/orange trail */
    { 231, 51, 238,  236,  226, 208, 196,  231, 220, 244, 226, 250,
      COLOR_WHITE, COLOR_CYAN, COLOR_WHITE,
      COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_WHITE, COLOR_YELLOW, COLOR_WHITE,
      "Classic" },
    /* 1 Fire — red/orange arms, bright trail */
    { 208, 196, 88,   236,  226, 214, 196,  231, 208, 244, 226, 202,
      COLOR_RED, COLOR_RED, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE, COLOR_RED, COLOR_WHITE, COLOR_YELLOW, COLOR_RED,
      "Fire" },
    /* 2 Neon — magenta/violet arms, pink trail */
    { 207, 165, 93,   236,  219, 213, 201,  231, 207, 244, 219, 201,
      COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_WHITE, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_WHITE, COLOR_MAGENTA, COLOR_WHITE, COLOR_MAGENTA, COLOR_MAGENTA,
      "Neon" },
    /* 3 Ocean — teal/blue arms, cyan trail */
    { 123, 45, 25,    236,  159, 87,  51,   231, 45,  244, 123, 39,
      COLOR_CYAN, COLOR_BLUE, COLOR_BLUE,
      COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE,
      COLOR_WHITE, COLOR_CYAN, COLOR_WHITE, COLOR_CYAN, COLOR_BLUE,
      "Ocean" },
    /* 4 Matrix — green arms, lime trail */
    { 118, 46, 22,    236,  154, 118, 46,   231, 118, 244, 154, 82,
      COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_WHITE, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_WHITE, COLOR_GREEN, COLOR_WHITE, COLOR_GREEN, COLOR_GREEN,
      "Matrix" },
};

static int g_theme = 0;

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(CP_ARM_HI,  th->arm_hi,  -1);
        init_pair(CP_ARM_MID, th->arm_mid, -1);
        init_pair(CP_ARM_LO,  th->arm_lo,  -1);
        init_pair(CP_CIRCLE,  th->circle,  -1);
        init_pair(CP_TRAIL_1, th->trail1,  -1);
        init_pair(CP_TRAIL_2, th->trail2,  -1);
        init_pair(CP_TRAIL_3, th->trail3,  -1);
        init_pair(CP_BOB,     th->bob,     -1);
        init_pair(CP_PIVOT,   th->pivot,   -1);
        init_pair(CP_HUD,     th->hud,     -1);
        init_pair(CP_CURSOR,  th->cursor,  -1);
        init_pair(CP_PATH,    th->path,    -1);
    } else {
        init_pair(CP_ARM_HI,  th->arm_hi8,  -1);
        init_pair(CP_ARM_MID, th->arm_mid8, -1);
        init_pair(CP_ARM_LO,  th->arm_lo8,  -1);
        init_pair(CP_CIRCLE,  th->circle8,  -1);
        init_pair(CP_TRAIL_1, th->trail18,  -1);
        init_pair(CP_TRAIL_2, th->trail28,  -1);
        init_pair(CP_TRAIL_3, th->trail38,  -1);
        init_pair(CP_BOB,     th->bob8,     -1);
        init_pair(CP_PIVOT,   th->pivot8,   -1);
        init_pair(CP_HUD,     th->hud8,     -1);
        init_pair(CP_CURSOR,  th->cursor8,  -1);
        init_pair(CP_PATH,    th->path8,    -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(0);
}

/* ===================================================================== */
/* §4  DFT + epicycles                                                    */
/* ===================================================================== */

typedef struct { float re, im; } Cplx;

/*
 * O(N²) DFT using twiddle recurrence.
 * W = exp(-2πi·n/N); advance by complex multiply instead of re-computing.
 */
static void compute_dft(const Cplx *in, Cplx *out, int N)
{
    for (int n = 0; n < N; n++) {
        float tw_re = cosf(-2.f*(float)M_PI*(float)n/(float)N);
        float tw_im = sinf(-2.f*(float)M_PI*(float)n/(float)N);
        float w_re  = 1.f, w_im = 0.f;
        float re    = 0.f, im   = 0.f;
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
    float amp;    /* |Z[n]| / N  */
    float phase;  /* arg(Z[n])   */
    int   freq;   /* signed rotation rate */
} Epicycle;

static int epic_cmp(const void *a, const void *b)
{
    float da = ((const Epicycle *)a)->amp;
    float db = ((const Epicycle *)b)->amp;
    return (da < db) - (da > db);   /* descending */
}

static Epicycle g_epics[N_SAMPLES];
static int      g_n_epics  = 0;
static int      g_n_active = 0;

/*
 * Resample the raw pixel-space path to N equally arc-length-spaced points,
 * then center on origin.  Stores result in out[0..N-1].
 * Returns the scale factor: max |z| of the centered path.
 */
static float resample_and_center(const float *rx, const float *ry, int n,
                                  Cplx *out, int N)
{
    /* arc lengths */
    float *arc = malloc((size_t)n * sizeof(float));
    if (!arc) return 1.f;
    arc[0] = 0.f;
    for (int i = 1; i < n; i++) {
        float dx = rx[i] - rx[i-1], dy = ry[i] - ry[i-1];
        arc[i] = arc[i-1] + sqrtf(dx*dx + dy*dy);
    }
    float total = arc[n-1];
    if (total < 0.001f) total = 1.f;

    /* resample */
    int j = 0;
    for (int k = 0; k < N; k++) {
        float s = (float)k / (float)N * total;
        while (j < n-2 && arc[j+1] < s) j++;
        float span = arc[j+1] - arc[j];
        float t    = (span > 0.001f) ? (s - arc[j]) / span : 0.f;
        out[k].re  = rx[j] + t * (rx[j+1] - rx[j]);
        out[k].im  = ry[j] + t * (ry[j+1] - ry[j]);
    }
    free(arc);

    /* center */
    float mean_re = 0.f, mean_im = 0.f;
    for (int k = 0; k < N; k++) { mean_re += out[k].re; mean_im += out[k].im; }
    mean_re /= (float)N; mean_im /= (float)N;
    float max_r = 0.f;
    for (int k = 0; k < N; k++) {
        out[k].re -= mean_re; out[k].im -= mean_im;
        float r = sqrtf(out[k].re*out[k].re + out[k].im*out[k].im);
        if (r > max_r) max_r = r;
    }
    return (max_r > 0.001f) ? max_r : 1.f;
}

static void build_epicycles(const Cplx *samples, int N)
{
    Cplx dft[N_SAMPLES];
    compute_dft(samples, dft, N);
    float inv_N = 1.f / (float)N;
    for (int n = 0; n < N; n++) {
        float re = dft[n].re, im = dft[n].im;
        int   f  = (n <= N/2) ? n : n - N;   /* signed frequency */
        g_epics[n] = (Epicycle){ sqrtf(re*re + im*im) * inv_N,
                                  atan2f(im, re), f };
    }
    qsort(g_epics, N, sizeof(Epicycle), epic_cmp);
    g_n_epics  = N;
    g_n_active = 1;
}

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

typedef enum { STATE_DRAW, STATE_PLAY } AppState;

/* ── shared ───────────────────────────────────────────────────────── */

static int   g_rows, g_cols;
static AppState g_state = STATE_DRAW;

/* ── draw mode ────────────────────────────────────────────────────── */

static int   g_cur_col, g_cur_row;        /* cursor in cell space      */
static float g_raw_px[RAW_MAX];           /* raw path in pixel space   */
static float g_raw_py[RAW_MAX];
static int   g_raw_n  = 0;
static uint8_t g_drawn[ROWS_MAX][COLS_MAX]; /* painted cells for display */
static bool  g_draw_err = false;           /* too-few-points error flag */

/* ── play mode ────────────────────────────────────────────────────── */

typedef struct {
    float px[TRAIL_LEN], py[TRAIL_LEN];
    int   head, count;
} Trail;

static Trail g_trail;
static float g_phi;
static int   g_frame;
static int   g_auto_cnt;
static bool  g_paused;
static bool  g_auto_add;
static bool  g_show_circles;
static float g_pivot_px, g_pivot_py;     /* screen centre, pixel space */
static float g_scale;                     /* normalized → pixel         */
static float g_tips_px[N_SAMPLES + 1];
static float g_tips_py[N_SAMPLES + 1];

/* ── helpers ──────────────────────────────────────────────────────── */

static int px_cx(float px) { return (int)(px / CELL_W + 0.5f); }
static int px_cy(float py) { return (int)(py / CELL_H + 0.5f); }

static void trail_push(Trail *t, float px, float py)
{
    t->px[t->head] = px;  t->py[t->head] = py;
    t->head = (t->head + 1) % TRAIL_LEN;
    if (t->count < TRAIL_LEN) t->count++;
}
static void trail_clear(Trail *t) { t->head = t->count = 0; }

/* draw a Bresenham line with given attr */
static void draw_line_attr(int x0, int y0, int x1, int y1, attr_t attr)
{
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = x0<x1?1:-1, sy = y0<y1?1:-1;
    int err = dx - dy;
    for (;;) {
        if (x0>=0 && x0<g_cols && y0>=0 && y0<g_rows-1) {
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

/* draw orbit ellipse around pivot (px, py) with radius r_px pixels */
static void draw_orbit(float piv_px, float piv_py, float r_px)
{
    if (r_px < (float)CELL_W * 0.5f) return;
    float rx = r_px / (float)CELL_W;
    float ry = r_px / (float)CELL_H;
    int   pcx = px_cx(piv_px), pcy = px_cy(piv_py);
    int   steps = (int)(2.f*(float)M_PI * fmaxf(rx, ry)) + 4;
    attr_t attr = COLOR_PAIR(CP_CIRCLE);
    for (int i = 0; i < steps; i++) {
        float a = 2.f*(float)M_PI*(float)i/(float)steps;
        int x = pcx + (int)(rx * cosf(a) + 0.5f);
        int y = pcy + (int)(ry * sinf(a) + 0.5f);
        if (x>=0 && x<g_cols && y>=0 && y<g_rows-1) {
            attron(attr);
            mvaddch(y, x, '.');
            attroff(attr);
        }
    }
}

/* ── draw mode functions ──────────────────────────────────────────── */

static void draw_mode_reset(void)
{
    g_cur_col = g_cols / 2;
    g_cur_row = g_rows / 2;
    g_raw_n   = 0;
    g_draw_err = false;
    memset(g_drawn, 0, sizeof g_drawn);
}

/* record current cursor position into raw buffer */
static void draw_record(void)
{
    if (g_raw_n >= RAW_MAX) return;
    float px = (float)g_cur_col * CELL_W;
    float py = (float)g_cur_row * CELL_H;
    /* skip if identical to last point */
    if (g_raw_n > 0 &&
        g_raw_px[g_raw_n-1] == px && g_raw_py[g_raw_n-1] == py)
        return;
    g_raw_px[g_raw_n] = px;
    g_raw_py[g_raw_n] = py;
    g_raw_n++;
    /* mark cell as drawn */
    if (g_cur_row >= 0 && g_cur_row < ROWS_MAX &&
        g_cur_col >= 0 && g_cur_col < COLS_MAX)
        g_drawn[g_cur_row][g_cur_col] = 1;
}

/* move cursor by (dc, dr) and record */
static void draw_move(int dc, int dr)
{
    int nc = g_cur_col + dc;
    int nr = g_cur_row + dr;
    if (nc < 0) nc = 0;
    if (nc >= g_cols) nc = g_cols - 1;
    if (nr < 0) nr = 0;
    if (nr >= g_rows - 1) nr = g_rows - 2;
    if (nc == g_cur_col && nr == g_cur_row) return;
    g_cur_col = nc;
    g_cur_row = nr;
    draw_record();
}

/* try to switch to PLAY mode; returns false if too few points */
static bool draw_compute(void)
{
    if (g_raw_n < 4) { g_draw_err = true; return false; }
    g_draw_err = false;

    Cplx samples[N_SAMPLES];
    float max_r = resample_and_center(g_raw_px, g_raw_py, g_raw_n,
                                       samples, N_SAMPLES);
    build_epicycles(samples, N_SAMPLES);

    /* set pivot and scale */
    g_pivot_px = (float)g_cols * CELL_W * 0.5f;
    g_pivot_py = (float)(g_rows - 1) * CELL_H * 0.5f;
    float screen_r = fminf((float)g_cols * CELL_W,
                            (float)(g_rows - 1) * CELL_H) * 0.40f;
    g_scale = screen_r / max_r;

    g_phi      = 0.f;
    g_frame    = 0;
    g_auto_cnt = 0;
    g_paused   = false;
    g_auto_add = true;
    g_show_circles = true;
    trail_clear(&g_trail);

    /* compute initial chain */
    float x = g_pivot_px, y = g_pivot_py;
    g_tips_px[0] = x; g_tips_py[0] = y;
    for (int i = 0; i < g_n_active; i++) {
        float ang = (float)g_epics[i].freq * g_phi + g_epics[i].phase;
        float r   = g_epics[i].amp * g_scale;
        x += r * cosf(ang); y += r * sinf(ang);
        g_tips_px[i+1] = x; g_tips_py[i+1] = y;
    }

    g_state = STATE_PLAY;
    return true;
}

static void draw_mode_draw(void)
{
    /* path cells */
    for (int r = 0; r < g_rows - 1 && r < ROWS_MAX; r++) {
        for (int c = 0; c < g_cols && c < COLS_MAX; c++) {
            if (!g_drawn[r][c]) continue;
            attron(COLOR_PAIR(CP_PATH));
            mvaddch(r, c, '.');
            attroff(COLOR_PAIR(CP_PATH));
        }
    }

    /* cursor */
    if (g_cur_row < g_rows-1) {
        attron(COLOR_PAIR(CP_CURSOR) | A_BOLD);
        mvaddch(g_cur_row, g_cur_col, '@');
        attroff(COLOR_PAIR(CP_CURSOR) | A_BOLD);
    }

    /* HUD */
    move(g_rows - 1, 0);
    clrtoeol();
    attron(COLOR_PAIR(CP_HUD));
    if (g_draw_err) {
        printw(" FourierArt  Draw more (need >=4 pts)  arrows:move  ENTER:play  c:clear  q:quit");
    } else {
        printw(" FourierArt  arrows/WASD:draw  ENTER/g:play  c:clear  q:quit  pts:%d/%d",
               g_raw_n, RAW_MAX);
    }
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── play mode functions ──────────────────────────────────────────── */

static void play_tick(void)
{
    if (g_paused) return;
    g_frame++;

    /* auto-add one epicycle at intervals */
    if (g_auto_add && g_n_active < g_n_epics) {
        g_auto_cnt++;
        if (g_auto_cnt >= AUTO_ADD_FRAMES) {
            g_auto_cnt = 0;
            g_n_active++;
        }
    }

    g_phi += 2.f * (float)M_PI / (float)CYCLE_FRAMES;
    if (g_phi >= 2.f * (float)M_PI) {
        g_phi -= 2.f * (float)M_PI;
        trail_clear(&g_trail);
    }

    /* recompute chain */
    float x = g_pivot_px, y = g_pivot_py;
    g_tips_px[0] = x; g_tips_py[0] = y;
    for (int i = 0; i < g_n_active; i++) {
        float ang = (float)g_epics[i].freq * g_phi + g_epics[i].phase;
        float r   = g_epics[i].amp * g_scale;
        x += r * cosf(ang); y += r * sinf(ang);
        g_tips_px[i+1] = x; g_tips_py[i+1] = y;
    }

    trail_push(&g_trail, g_tips_px[g_n_active], g_tips_py[g_n_active]);
}

static void play_draw(void)
{
    /* 1. orbit circles */
    if (g_show_circles) {
        int nc = g_n_active < N_CIRCLES ? g_n_active : N_CIRCLES;
        for (int i = 0; i < nc; i++) {
            float r_px = g_epics[i].amp * g_scale;
            draw_orbit(g_tips_px[i], g_tips_py[i], r_px);
        }
    }

    /* 2. trail */
    int draw  = g_trail.count;
    int start = (g_trail.head - draw + TRAIL_LEN) % TRAIL_LEN;
    for (int i = 0; i < draw; i++) {
        int idx = (start + i) % TRAIL_LEN;
        int cx  = px_cx(g_trail.px[idx]);
        int cy  = px_cy(g_trail.py[idx]);
        if (cx<0 || cx>=g_cols || cy<0 || cy>=g_rows-1) continue;
        float age = (float)i / (float)(draw > 1 ? draw : 1);
        int   cp  = age > 0.70f ? CP_TRAIL_1
                  : age > 0.35f ? CP_TRAIL_2 : CP_TRAIL_3;
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvaddch(cy, cx, '*');
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }

    /* 3. arm chain */
    for (int i = 0; i < g_n_active; i++) {
        float r_px = g_epics[i].amp * g_scale;
        int   cp   = r_px > g_scale * 0.08f ? CP_ARM_HI
                   : r_px > g_scale * 0.02f ? CP_ARM_MID : CP_ARM_LO;
        draw_line_attr(px_cx(g_tips_px[i]),   px_cy(g_tips_py[i]),
                       px_cx(g_tips_px[i+1]), px_cy(g_tips_py[i+1]),
                       COLOR_PAIR(cp) | A_BOLD);
    }

    /* 4. tip */
    int bx = px_cx(g_tips_px[g_n_active]);
    int by = px_cy(g_tips_py[g_n_active]);
    if (bx>=0 && bx<g_cols && by>=0 && by<g_rows-1) {
        attron(COLOR_PAIR(CP_BOB) | A_BOLD);
        mvaddch(by, bx, '@');
        attroff(COLOR_PAIR(CP_BOB) | A_BOLD);
    }

    /* 5. pivot */
    int pcx = px_cx(g_pivot_px), pcy = px_cy(g_pivot_py);
    if (pcx>=0 && pcx<g_cols && pcy>=0 && pcy<g_rows-1) {
        attron(COLOR_PAIR(CP_PIVOT) | A_BOLD);
        mvaddch(pcy, pcx, '+');
        attroff(COLOR_PAIR(CP_PIVOT) | A_BOLD);
    }

    /* 6. HUD */
    move(g_rows - 1, 0);
    clrtoeol();
    attron(COLOR_PAIR(CP_HUD));
    printw(" FourierArt  r:redraw  p:pause  +/-:epics(%d/%d)  c:circles  t:%s  a:auto  q:quit%s",
           g_n_active, g_n_epics,
           k_themes[g_theme].name,
           g_paused ? "  [PAUSED]" : "");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  screen                                                             */
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

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit_flag  = 0;
static volatile sig_atomic_t g_resize_flag = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit_flag   = 1;
    if (s == SIGWINCH)               g_resize_flag = 1;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    screen_init();
    getmaxyx(stdscr, g_rows, g_cols);
    draw_mode_reset();

    long long last_ns = clock_ns();

    while (!g_quit_flag) {
        /* ── resize ── */
        if (g_resize_flag) {
            g_resize_flag = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            if (g_state == STATE_DRAW) {
                draw_mode_reset();
            } else {
                /* rescale pivot for new terminal size */
                g_pivot_px = (float)g_cols * CELL_W * 0.5f;
                g_pivot_py = (float)(g_rows - 1) * CELL_H * 0.5f;
                float screen_r = fminf((float)g_cols * CELL_W,
                                        (float)(g_rows - 1) * CELL_H) * 0.40f;
                /* preserve relative scale */
                float old_sr = fminf((float)g_cols * CELL_W,
                                      (float)(g_rows - 1) * CELL_H) * 0.40f;
                g_scale = g_scale * (screen_r / (old_sr > 0 ? old_sr : 1));
                trail_clear(&g_trail);
            }
            last_ns = clock_ns();
            continue;
        }

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (g_state == STATE_DRAW) {
                switch (ch) {
                case 'q': case 'Q': case 27: g_quit_flag = 1; break;
                case KEY_UP:    case 'w': case 'W': draw_move( 0,-1); break;
                case KEY_DOWN:  case 's': case 'S': draw_move( 0, 1); break;
                case KEY_LEFT:  case 'a': case 'A': draw_move(-1, 0); break;
                case KEY_RIGHT: case 'd': case 'D': draw_move( 1, 0); break;
                case '\n': case '\r': case 'g': case 'G':
                    draw_compute();
                    break;
                case 'c': case 'C':
                    draw_mode_reset();
                    break;
                }
            } else { /* STATE_PLAY */
                switch (ch) {
                case 'q': case 'Q': case 27: g_quit_flag = 1; break;
                case 'r': case 'R':
                    g_state = STATE_DRAW;
                    draw_mode_reset();
                    break;
                case ' ': case 'p': case 'P':
                    g_paused = !g_paused;
                    break;
                case '+': case '=':
                    if (g_n_active < g_n_epics) {
                        g_n_active++;
                        trail_clear(&g_trail);
                    }
                    break;
                case '-':
                    if (g_n_active > 1) {
                        g_n_active--;
                        trail_clear(&g_trail);
                    }
                    break;
                case 'c': case 'C':
                    g_show_circles = !g_show_circles;
                    break;
                case 'a': case 'A':
                    g_auto_add = !g_auto_add;
                    break;
                case 't':
                    g_theme = (g_theme + 1) % N_THEMES;
                    theme_apply(g_theme);
                    break;
                case 'T':
                    g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
                    theme_apply(g_theme);
                    break;
                }
            }
        }

        /* ── tick ── */
        if (g_state == STATE_PLAY)
            play_tick();

        /* ── draw ── */
        erase();
        if (g_state == STATE_DRAW)
            draw_mode_draw();
        else
            play_draw();
        wnoutrefresh(stdscr);
        doupdate();

        /* ── FPS cap ── */
        long long now_ns = clock_ns();
        clock_sleep_ns(RENDER_NS - (now_ns - last_ns));
        last_ns = clock_ns();
    }
    return 0;
}
