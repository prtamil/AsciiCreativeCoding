/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fft_vis.c — Cooley-Tukey radix-2 DIT FFT visualiser
 *
 * Demonstrates the FFT algorithm vs naive DFT O(N²):
 *
 *   DFT:  X[k] = Σ_{n=0}^{N-1} x[n] · exp(-2πi·k·n/N)      O(N²)
 *   FFT:  same result, butterfly decomposition                 O(N log N)
 *
 * Cooley-Tukey radix-2 DIT (Decimation-In-Time):
 *   Split x[n] into even/odd halves → recurse → combine with twiddle factors
 *   W_N^k = exp(-2πi·k/N) = cos(2πk/N) − i·sin(2πk/N)
 *
 *   X[k]     = E[k] + W_N^k · O[k]       k = 0 … N/2−1
 *   X[k+N/2] = E[k] − W_N^k · O[k]
 *
 * Layout (two panels):
 *   TOP    — time-domain signal x[n] as vertical bar chart
 *   BOTTOM — frequency-domain |X[k]| as vertical bar chart
 *
 * The input signal is a sum of 3 sine waves whose amplitudes and
 * frequencies you can tune with keys. Watch peaks appear/disappear
 * in the frequency panel as you change the signal.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   1/2/3     toggle sine component 1 / 2 / 3
 *   +/-       increase / decrease frequency of selected component (j/k)
 *   j/k       select component
 *   n         add white noise burst
 *   t         cycle colour theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/fft_vis.c -o fft_vis -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 fft  §4 signal  §5 draw  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Cooley-Tukey radix-2 DIT (Decimation-In-Time) FFT.
 *                  Recursively splits the N-point DFT into two N/2-point
 *                  DFTs of even- and odd-indexed samples, then combines
 *                  with twiddle factors W_N^k = exp(-2πi·k/N).
 *                  Complexity: O(N log₂ N) multiplications vs O(N²) for
 *                  the naive DFT — for N=128: 896 vs 16384 operations.
 *
 * Math           : DFT linearity: sum of sinusoids → sum of delta spikes
 *                  in the frequency domain.  |X[k]| is the amplitude of
 *                  the k-th harmonic; arg(X[k]) is its phase.  The display
 *                  shows only the magnitude spectrum (one-sided: bins 0…N/2).
 *                  Parseval's theorem: Σ|x[n]|² = (1/N)·Σ|X[k]|² — total
 *                  power is conserved between domains.
 *
 * Performance    : N_FFT must be a power of 2 for the radix-2 butterfly.
 *                  In-place bit-reversal permutation reorders samples before
 *                  the butterfly stages, requiring no auxiliary buffer.
 *
 * Rendering      : Two-panel layout: time domain (top) and frequency domain
 *                  (bottom), both as vertical bar charts drawn column-by-
 *                  column.  Bar heights are normalised to panel height each
 *                  frame so the display auto-scales with terminal size.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_FFT       128        /* FFT size — must be power of 2           */
#define N_COMPS       3        /* number of sine components               */
#define RENDER_FPS   30
#define RENDER_NS   (1000000000LL / RENDER_FPS)
#define NOISE_AMP    0.3f      /* noise burst amplitude                   */
#define N_THEMES      4

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
/* §3  Cooley-Tukey radix-2 DIT FFT (in-place, iterative)               */
/* ===================================================================== */

/*
 * Bit-reversal permutation: swap sample[i] with sample[bit_reverse(i, log2 N)]
 * so the butterfly stages can operate in-place without extra buffers.
 */
static void bit_reverse_permute(float *re, float *im, int N)
{
    int bits = 0;
    for (int tmp = N; tmp > 1; tmp >>= 1) bits++;

    for (int i = 0; i < N; i++) {
        int rev = 0;
        for (int b = 0; b < bits; b++)
            rev |= ((i >> b) & 1) << (bits - 1 - b);
        if (rev > i) {
            float t;
            t = re[i]; re[i] = re[rev]; re[rev] = t;
            t = im[i]; im[i] = im[rev]; im[rev] = t;
        }
    }
}

/*
 * Iterative radix-2 DIT FFT.
 * Stage s processes butterfly width len = 2^s.
 * Each butterfly pair (j, j+half) uses twiddle W = exp(-2πi·j/len).
 *
 * Butterfly:
 *   t  = W · X[j+half]
 *   X[j+half] = X[j] − t
 *   X[j]      = X[j] + t
 */
static void fft(float *re, float *im, int N)
{
    bit_reverse_permute(re, im, N);

    for (int len = 2; len <= N; len <<= 1) {
        int half = len >> 1;
        float ang = -2.0f * (float)M_PI / (float)len;
        float wr0 = cosf(ang), wi0 = sinf(ang);   /* twiddle step W^1  */

        for (int i = 0; i < N; i += len) {
            float wr = 1.0f, wi = 0.0f;            /* W^0 = 1           */
            for (int j = 0; j < half; j++) {
                /* t = W^j * X[i+j+half] */
                float tr = wr * re[i+j+half] - wi * im[i+j+half];
                float ti = wr * im[i+j+half] + wi * re[i+j+half];
                re[i+j+half] = re[i+j] - tr;
                im[i+j+half] = im[i+j] - ti;
                re[i+j]     += tr;
                im[i+j]     += ti;
                /* advance twiddle: W^{j+1} = W^j * W^1 */
                float nwr = wr * wr0 - wi * wi0;
                wi = wr * wi0 + wi * wr0;
                wr = nwr;
            }
        }
    }
}

/* ===================================================================== */
/* §4  signal generation                                                  */
/* ===================================================================== */

typedef struct {
    float freq;      /* frequency bin index (0 … N_FFT/2)     */
    float amp;       /* amplitude 0..1                         */
    bool  on;
} Comp;

static float g_noise[N_FFT];
static float g_noise_decay = 0.0f;

static void build_signal(const Comp comps[], float *out, float phase)
{
    for (int n = 0; n < N_FFT; n++) {
        float v = 0.0f;
        for (int c = 0; c < N_COMPS; c++) {
            if (!comps[c].on) continue;
            float t = (float)n / (float)N_FFT;
            v += comps[c].amp * sinf(2.0f * (float)M_PI * comps[c].freq * t
                                     + phase * comps[c].freq);
        }
        v += g_noise[n] * g_noise_decay;
        out[n] = v;
    }
}

/* ===================================================================== */
/* §5  drawing                                                            */
/* ===================================================================== */

/* 5 ncurses colour pairs per theme: bg, time-pos, time-neg, freq-low, freq-high */
static const short themes[N_THEMES][5][2] = {
    /* 0 — cyan/green */
    {{-1,-1},{51,16},{39,16},{46,16},{82,16}},
    /* 1 — fire */
    {{-1,-1},{196,16},{202,16},{208,16},{226,16}},
    /* 2 — purple */
    {{-1,-1},{201,16},{171,16},{141,16},{231,16}},
    /* 3 — mono */
    {{-1,-1},{252,16},{245,16},{240,16},{255,16}},
};

#define CP_BG    1
#define CP_TPOS  2
#define CP_TNEG  3
#define CP_FLOW  4
#define CP_FHIGH 5

static void init_colors(int theme)
{
    for (int i = 0; i < 5; i++)
        init_pair((short)(i + 1),
                  themes[theme][i][0],
                  themes[theme][i][1]);
}

static volatile sig_atomic_t g_resize = 0;
static void on_resize(int s) { (void)s; g_resize = 1; }

/* Draw a single vertical bar at column x, from row base upward by h rows.
 * Positive bars grow upward from base, negative downward. */
static void draw_bar(int x, int base, int h, bool positive, int cp)
{
    if (h <= 0) return;
    attron(COLOR_PAIR(cp));
    for (int dy = 0; dy < h; dy++) {
        int row = positive ? (base - dy) : (base + dy + 1);
        if (row < 0 || row >= LINES) continue;
        mvaddch(row, x, positive ? '|' : '.');
    }
    attroff(COLOR_PAIR(cp));
}

/* ===================================================================== */
/* §6  app                                                                */
/* ===================================================================== */

typedef struct {
    Comp   comps[N_COMPS];
    int    sel;            /* selected component index    */
    int    theme;
    bool   paused;
    float  phase;          /* animation phase             */
    float  re[N_FFT];
    float  im[N_FFT];
    float  sig[N_FFT];
    float  mag[N_FFT/2];   /* FFT magnitudes (only first N/2 bins)        */
    float  mag_max;
    float  sig_max;
} App;

static void app_init(App *a)
{
    memset(a, 0, sizeof(*a));
    a->comps[0] = (Comp){  4.0f, 1.0f, true };   /* low frequency  */
    a->comps[1] = (Comp){ 11.0f, 0.6f, true };   /* mid frequency  */
    a->comps[2] = (Comp){ 23.0f, 0.4f, true };   /* high frequency */
    a->sel   = 0;
    a->theme = 0;
}

static void app_compute(App *a)
{
    build_signal(a->comps, a->sig, a->phase);

    /* copy to FFT input */
    for (int n = 0; n < N_FFT; n++) {
        a->re[n] = a->sig[n];
        a->im[n] = 0.0f;
    }
    fft(a->re, a->im, N_FFT);

    /* compute magnitudes for first N/2 bins (Nyquist) */
    a->mag_max = 1e-6f;
    a->sig_max = 1e-6f;
    for (int k = 0; k < N_FFT / 2; k++) {
        a->mag[k] = sqrtf(a->re[k]*a->re[k] + a->im[k]*a->im[k]) / (float)(N_FFT/2);
        if (a->mag[k] > a->mag_max) a->mag_max = a->mag[k];
    }
    for (int n = 0; n < N_FFT; n++) {
        float av = fabsf(a->sig[n]);
        if (av > a->sig_max) a->sig_max = av;
    }
}

static void app_draw(const App *a)
{
    int rows = LINES, cols = COLS;
    int panel_h = (rows - 4) / 2;   /* rows available for each panel     */
    int time_top = 1;
    int time_mid = time_top + panel_h;   /* baseline for time panel       */
    int freq_top = time_mid + 2;
    int freq_base= freq_top + panel_h;   /* baseline (bottom) for freq    */

    erase();

    /* ── title ── */
    attron(A_BOLD);
    mvprintw(0, 0, "FFT Visualiser  N=%d  O(N log N)=%d ops vs DFT O(N²)=%d ops",
             N_FFT,
             (int)(N_FFT * log2(N_FFT + 0.1f)),
             N_FFT * N_FFT);
    attroff(A_BOLD);

    /* ── panel labels ── */
    mvprintw(time_top - 1 < 0 ? 0 : time_top, 0, "TIME DOMAIN  x[n]");
    mvprintw(freq_top,            0, "FREQ DOMAIN  |X[k]|  (bins 0..%d)", N_FFT/2 - 1);

    /* ── time panel ── */
    int tw = cols < N_FFT ? cols : N_FFT;
    for (int n = 0; n < tw; n++) {
        float v  = a->sig[n] / (a->sig_max + 1e-6f);
        int   h  = (int)(fabsf(v) * (float)panel_h + 0.5f);
        bool  pos = v >= 0.0f;
        draw_bar(n, time_mid, h, pos, pos ? CP_TPOS : CP_TNEG);
        /* baseline tick */
        mvaddch(time_mid + 1, n, '-');
    }

    /* ── freq panel ── */
    int fw = cols < N_FFT / 2 ? cols : N_FFT / 2;
    /* scale: 2 pixels per bin if cols allows */
    int bin_w = (cols / (N_FFT / 2)) < 2 ? 1 : 2;
    for (int k = 0; k < fw / bin_w && k < N_FFT / 2; k++) {
        float v = a->mag[k] / (a->mag_max + 1e-6f);
        int   h = (int)(v * (float)panel_h + 0.5f);
        int   cp = h > panel_h / 2 ? CP_FHIGH : CP_FLOW;
        for (int bx = 0; bx < bin_w; bx++)
            draw_bar(k * bin_w + bx, freq_base, h, true, cp);
    }

    /* ── component status ── */
    int sy = rows - 2;
    mvprintw(sy, 0, "Components: ");
    for (int c = 0; c < N_COMPS; c++) {
        if (c == a->sel) attron(A_REVERSE);
        if (a->comps[c].on) attron(A_BOLD);
        printw("[%d] f=%.0f a=%.1f%s  ",
               c + 1, a->comps[c].freq, a->comps[c].amp,
               a->comps[c].on ? "" : " OFF");
        attroff(A_BOLD); attroff(A_REVERSE);
    }

    mvprintw(rows - 1, 0,
             "1/2/3 toggle | j/k select | +/- freq | n noise | t theme | SPC pause | q quit");

    if (a->paused) {
        attron(A_REVERSE | A_BOLD);
        mvprintw(0, cols - 8, " PAUSED ");
        attroff(A_REVERSE | A_BOLD);
    }

    refresh();
}

static volatile sig_atomic_t g_quit = 0;
static void on_signal(int s) { (void)s; g_quit = 1; }

int main(void)
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_resize);

    initscr();
    cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    start_color();
    use_default_colors();

    App a;
    app_init(&a);
    init_colors(a.theme);

    long long next = clock_ns();

    while (!g_quit) {
        if (g_resize) { g_resize = 0; endwin(); refresh(); }

        int ch = getch();
        switch (ch) {
            case 'q': case 27: g_quit = 1; break;
            case ' ': a.paused = !a.paused; break;
            case '1': a.comps[0].on = !a.comps[0].on; break;
            case '2': a.comps[1].on = !a.comps[1].on; break;
            case '3': a.comps[2].on = !a.comps[2].on; break;
            case 'j': a.sel = (a.sel + 1) % N_COMPS; break;
            case 'k': a.sel = (a.sel + N_COMPS - 1) % N_COMPS; break;
            case '+': case '=':
                a.comps[a.sel].freq += 1.0f;
                if (a.comps[a.sel].freq > N_FFT/2 - 1)
                    a.comps[a.sel].freq = N_FFT/2 - 1;
                break;
            case '-':
                a.comps[a.sel].freq -= 1.0f;
                if (a.comps[a.sel].freq < 1.0f) a.comps[a.sel].freq = 1.0f;
                break;
            case 'n':
                for (int i = 0; i < N_FFT; i++)
                    g_noise[i] = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * NOISE_AMP;
                g_noise_decay = 1.0f;
                break;
            case 't':
                a.theme = (a.theme + 1) % N_THEMES;
                init_colors(a.theme);
                break;
        }

        long long now = clock_ns();
        if (now >= next) {
            if (!a.paused) {
                a.phase += 0.06f;
                if (g_noise_decay > 0.0f) g_noise_decay -= 0.03f;
                if (g_noise_decay < 0.0f) g_noise_decay = 0.0f;
            }
            app_compute(&a);
            app_draw(&a);
            next += RENDER_NS;
        } else {
            clock_sleep_ns(next - now);
        }
    }

    endwin();
    return 0;
}
