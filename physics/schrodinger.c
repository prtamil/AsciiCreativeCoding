/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * schrodinger.c — 1-D Quantum Wavefunction
 *
 * Evolves ψ(x,t) under the time-dependent Schrödinger equation:
 *
 *   iℏ ∂ψ/∂t = [ -ℏ²/2m ∂²/∂x² + V(x) ] ψ
 *
 * Discretised with the Crank-Nicolson scheme (unconditionally stable):
 *   (I + iΔt/2 H) ψ(t+Δt) = (I - iΔt/2 H) ψ(t)
 *
 * Solved each step with the Thomas algorithm (tridiagonal complex system).
 *
 * Display:
 *   Blue  — Re(ψ)   Red  — Im(ψ)   White — |ψ|² (probability density)
 *   Bottom row shows the potential V(x).
 *
 * Presets (1-4):
 *   1 Free packet     — Gaussian wave packet, no barrier
 *   2 Barrier tunnel  — Finite rectangular barrier (quantum tunnelling)
 *   3 Harmonic well   — Quadratic V(x); packet bounces
 *   4 Double slit     — Two narrow gaps in a wall; interference pattern
 *
 * Keys: q quit  p pause  r reset  1/2/3/4 preset  +/- energy  SPACE kick
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/schrodinger.c \
 *       -o schrodinger -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 wavefunction  §5 draw  §6 app
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

#define N_GRID   512       /* number of spatial grid points (power of 2) */
#define X_MIN    0.f
#define X_MAX    1.f
#define DX       ((X_MAX - X_MIN) / (float)(N_GRID - 1))
#define DT       0.0002f   /* time step (in natural units ℏ=m=1)          */
#define STEPS_PER_FRAME 20

#define HBAR     1.f
#define MASS     1.f

/* Potential barrier height (in energy units) */
#define V_BARRIER  2000.f
#define V_WALL    50000.f

#define RENDER_NS   (1000000000LL / 30)
#define HUD_ROWS    3

typedef struct { const char *name; int id; } Preset;
static const Preset PRESETS[] = {
    {"Free",     0},
    {"Barrier",  1},
    {"Harmonic", 2},
    {"D-Slit",   3},
};
#define N_PRESETS 4

static int g_preset = 1;    /* default: barrier */

enum { CP_RE=1, CP_IM, CP_PROB, CP_VPOT, CP_HUD };

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
        init_pair(CP_RE,   27, -1);   /* blue  — Re(ψ)         */
        init_pair(CP_IM,  196, -1);   /* red   — Im(ψ)         */
        init_pair(CP_PROB,231, -1);   /* white — |ψ|²          */
        init_pair(CP_VPOT,220, -1);   /* yellow — potential V  */
        init_pair(CP_HUD, 244, -1);
    } else {
        init_pair(CP_RE,   COLOR_BLUE,   -1);
        init_pair(CP_IM,   COLOR_RED,    -1);
        init_pair(CP_PROB, COLOR_WHITE,  -1);
        init_pair(CP_VPOT, COLOR_YELLOW, -1);
        init_pair(CP_HUD,  COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  wavefunction                                                       */
/* ===================================================================== */

static float g_re[N_GRID], g_im[N_GRID];   /* ψ = re + i·im */
static float g_V[N_GRID];                  /* potential      */
static float g_k0 = 200.f;                 /* initial wavenumber (energy) */

/* Thomas algorithm for complex tridiagonal system:
 * a[i]*x[i-1] + b[i]*x[i] + c[i]*x[i+1] = d[i]
 * All coefficients real except d and x (complex). */
typedef struct { float re, im; } Cx;
static inline Cx cx_add(Cx a, Cx b) { return (Cx){a.re+b.re, a.im+b.im}; }
static inline Cx cx_sub(Cx a, Cx b) { return (Cx){a.re-b.re, a.im-b.im}; }
static inline Cx cx_mul(Cx a, Cx b)
    { return (Cx){a.re*b.re-a.im*b.im, a.re*b.im+a.im*b.re}; }
static inline Cx cx_div(Cx a, Cx b) {
    float d = b.re*b.re + b.im*b.im;
    return (Cx){(a.re*b.re+a.im*b.im)/d, (a.im*b.re-a.re*b.im)/d};
}
static inline Cx cx_scale(float s, Cx a) { return (Cx){s*a.re, s*a.im}; }
static inline Cx real_cx(float r) { return (Cx){r, 0.f}; }

static Cx g_cx_tmp[N_GRID], g_cx_cp[N_GRID]; /* workspace for Thomas */

/* Build potential V for preset */
static void build_potential(void)
{
    for (int i = 0; i < N_GRID; i++) g_V[i] = 0.f;

    /* Absorbing walls at edges */
    int wall = N_GRID / 20;
    for (int i = 0; i < wall; i++) {
        g_V[i] = V_WALL;
        g_V[N_GRID-1-i] = V_WALL;
    }

    if (g_preset == 1) {
        /* Finite rectangular barrier in middle */
        int c = N_GRID/2, w = N_GRID/60;
        for (int i = c-w; i <= c+w; i++) g_V[i] = V_BARRIER;

    } else if (g_preset == 2) {
        /* Harmonic oscillator: V = 0.5 * k * (x-0.5)^2 */
        for (int i = 0; i < N_GRID; i++) {
            float x = (float)i / (float)(N_GRID-1) - 0.5f;
            g_V[i] += 0.5f * 8000.f * x * x;
        }

    } else if (g_preset == 3) {
        /* Double slit: wall with two gaps */
        int c = N_GRID/2;
        int w = N_GRID / 100 + 1;   /* slit width */
        int sep = N_GRID / 12;       /* slit separation */
        for (int i = c-2; i <= c+2; i++) {
            g_V[i] = V_WALL;         /* wall strip */
        }
        /* Open two slits */
        int s1 = N_GRID/2 - sep/2;
        int s2 = N_GRID/2 + sep/2;
        for (int i = s1-w; i <= s1+w; i++) g_V[i] = 0.f;
        for (int i = s2-w; i <= s2+w; i++) g_V[i] = 0.f;
    }
    /* preset 0 = free (walls only) */
}

/* Initialize Gaussian wave packet */
static void init_wavepacket(void)
{
    float x0, sigma;
    switch (g_preset) {
    case 0: x0 = 0.2f; sigma = 0.05f; g_k0 = 150.f; break;
    case 1: x0 = 0.2f; sigma = 0.05f; g_k0 = 200.f; break;
    case 2: x0 = 0.3f; sigma = 0.07f; g_k0 = 50.f;  break;
    case 3: x0 = 0.2f; sigma = 0.04f; g_k0 = 200.f; break;
    default: x0 = 0.2f; sigma = 0.05f; g_k0 = 150.f; break;
    }

    float norm = 0.f;
    for (int i = 0; i < N_GRID; i++) {
        float x = (float)i / (float)(N_GRID-1);
        float envelope = expf(-(x-x0)*(x-x0) / (2.f*sigma*sigma));
        g_re[i] = envelope * cosf(g_k0 * x);
        g_im[i] = envelope * sinf(g_k0 * x);
        norm += g_re[i]*g_re[i] + g_im[i]*g_im[i];
    }
    /* normalize */
    float s = 1.f / sqrtf(norm * DX);
    for (int i = 0; i < N_GRID; i++) { g_re[i] *= s; g_im[i] *= s; }

    /* Dirichlet BC */
    g_re[0] = g_re[N_GRID-1] = 0.f;
    g_im[0] = g_im[N_GRID-1] = 0.f;
}

static void load_preset(int p)
{
    g_preset = p;
    build_potential();
    init_wavepacket();
}

/*
 * One Crank-Nicolson step.
 * The Hamiltonian discretised: H[i] = -0.5/dx^2 * (ψ[i-1]-2ψ[i]+ψ[i+1]) + V[i]*ψ[i]
 * CN: (I + i·dt/2·H)ψ_new = (I - i·dt/2·H)ψ_old
 *
 * Tridiagonal coefficients (all real for the matrix, RHS complex):
 *   alpha = -i·dt/(4·dx²)   (off-diagonal, constant)
 *   beta[i] = 1 + i·dt/2·(1/dx² + V[i]/2)   (diagonal)
 */
static void schrodinger_step(void)
{
    float r = DT / (4.f * DX * DX);   /* r = dt/(4 dx^2) */

    /* Construct RHS: b[i] = (I - i·dt/2·H)ψ[i] */
    /* b[i] = ψ[i] - i·dt/2·H·ψ[i]
     *      = ψ[i] - i·r·(ψ[i-1]-2ψ[i]+ψ[i+1]) + i·dt/2·V[i]·ψ[i]
     * Written in real/imag:
     *   re(b) = re(ψ) + r*(im(ψ[i-1])-2im(ψ[i])+im(ψ[i+1])) - dt/2*V*im(ψ)
     *   im(b) = im(ψ) - r*(re(ψ[i-1])-2re(ψ[i])+re(ψ[i+1])) + dt/2*V*re(ψ)
     */

    /* Diagonals for the LHS tridiagonal:
     *   sub = super = -i*r  →  (0, -r)
     *   diag[i] = 1 + i*(2r + dt/2*V[i])  →  (1, 2r + dt/2*V[i])
     */
    Cx sub  = {0.f, -r};
    Cx sup  = {0.f, -r};

    /* Forward sweep */
    for (int i = 1; i < N_GRID-1; i++) {
        Cx d_i = {1.f, 2.f*r + 0.5f*DT*g_V[i]};

        /* RHS */
        float re_b = g_re[i] + r*(g_im[i-1]-2.f*g_im[i]+g_im[i+1])
                     - 0.5f*DT*g_V[i]*g_im[i];
        float im_b = g_im[i] - r*(g_re[i-1]-2.f*g_re[i]+g_re[i+1])
                     + 0.5f*DT*g_V[i]*g_re[i];
        Cx b_i = {re_b, im_b};

        if (i == 1) {
            g_cx_cp[i] = d_i;
            g_cx_tmp[i] = b_i;
        } else {
            Cx w = cx_div(sub, g_cx_cp[i-1]);
            g_cx_cp[i] = cx_sub(d_i, cx_mul(w, sup));
            g_cx_tmp[i] = cx_sub(b_i, cx_mul(w, g_cx_tmp[i-1]));
        }
    }

    /* Back substitution */
    int last = N_GRID-2;
    Cx x_cur = cx_div(g_cx_tmp[last], g_cx_cp[last]);
    g_re[last] = x_cur.re; g_im[last] = x_cur.im;

    for (int i = last-1; i >= 1; i--) {
        Cx prev = {g_re[i+1], g_im[i+1]};
        x_cur = cx_div(cx_sub(g_cx_tmp[i], cx_mul(sup, prev)), g_cx_cp[i]);
        g_re[i] = x_cur.re; g_im[i] = x_cur.im;
    }
    /* BC */
    g_re[0] = g_re[N_GRID-1] = 0.f;
    g_im[0] = g_im[N_GRID-1] = 0.f;

    /* Suppress unused warning */
    (void)real_cx; (void)cx_add; (void)cx_scale;
}

/* Compute reflection/transmission (split at grid midpoint) */
static void compute_RT(float *R_out, float *T_out)
{
    float left = 0.f, right = 0.f;
    int mid = N_GRID / 2;
    for (int i = 1; i < mid; i++)
        left += (g_re[i]*g_re[i] + g_im[i]*g_im[i]) * DX;
    for (int i = mid; i < N_GRID-1; i++)
        right += (g_re[i]*g_re[i] + g_im[i]*g_im[i]) * DX;
    float total = left + right + 1e-10f;
    *R_out = left  / total;
    *T_out = right / total;
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;

static void scene_draw(void)
{
    int draw_rows = g_rows - HUD_ROWS;
    int mid_row   = HUD_ROWS + draw_rows / 2;

    /* Find max values for scaling */
    float max_prob = 0.f, max_V = 0.f;
    for (int i = 0; i < N_GRID; i++) {
        float prob = g_re[i]*g_re[i] + g_im[i]*g_im[i];
        if (prob > max_prob) max_prob = prob;
        if (g_V[i] > max_V && g_V[i] < V_WALL * 0.5f) max_V = g_V[i];
    }
    if (max_prob < 1e-12f) max_prob = 1e-12f;
    if (max_V < 1.f)       max_V = 1.f;

    /* Each terminal column covers N_GRID/g_cols grid points */
    float pts_per_col = (float)N_GRID / (float)g_cols;

    for (int col = 0; col < g_cols; col++) {
        int gi = (int)((float)col * pts_per_col);
        if (gi >= N_GRID) gi = N_GRID-1;

        float re   = g_re[gi];
        float im   = g_im[gi];
        float prob = re*re + im*im;
        float V    = g_V[gi];

        /* --- |ψ|² probability bar (from midline up) --- */
        int prob_h = (int)(prob / max_prob * (float)(draw_rows / 2));
        for (int dr = 0; dr < prob_h && (mid_row - dr) >= HUD_ROWS; dr++) {
            attron(COLOR_PAIR(CP_PROB));
            mvaddch(mid_row - dr, col, '|');
            attroff(COLOR_PAIR(CP_PROB));
        }

        /* --- Re(ψ) blue --- */
        int re_row = mid_row - (int)(re / sqrtf(max_prob) * (float)(draw_rows/4));
        if (re_row >= HUD_ROWS && re_row < g_rows) {
            attron(COLOR_PAIR(CP_RE));
            mvaddch(re_row, col, '.');
            attroff(COLOR_PAIR(CP_RE));
        }

        /* --- Im(ψ) red --- */
        int im_row = mid_row - (int)(im / sqrtf(max_prob) * (float)(draw_rows/4));
        if (im_row >= HUD_ROWS && im_row < g_rows) {
            attron(COLOR_PAIR(CP_IM));
            mvaddch(im_row, col, ',');
            attroff(COLOR_PAIR(CP_IM));
        }

        /* --- Potential V bottom row --- */
        if (V > 0.f && V < V_WALL * 0.5f) {
            int v_h = (int)(V / max_V * (float)(draw_rows / 6));
            if (v_h < 1) v_h = 1;
            for (int dr = 0; dr < v_h; dr++) {
                int vrow = g_rows - 1 - dr;
                if (vrow >= HUD_ROWS) {
                    attron(COLOR_PAIR(CP_VPOT));
                    mvaddch(vrow, col, '#');
                    attroff(COLOR_PAIR(CP_VPOT));
                }
            }
        } else if (V >= V_WALL * 0.5f) {
            attron(COLOR_PAIR(CP_VPOT) | A_BOLD);
            mvaddch(g_rows-1, col, '|');
            attroff(COLOR_PAIR(CP_VPOT) | A_BOLD);
        }
    }

    float R = 0.f, T = 0.f;
    compute_RT(&R, &T);

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Schrodinger  q:quit  p:pause  r:reset  1-4:preset  +/-:energy  spc:kick");
    mvprintw(1, 0,
        " [%d]%-9s  k0=%.0f  R=%.2f T=%.2f  %s",
        g_preset+1, PRESETS[g_preset].name, (double)g_k0,
        (double)R, (double)T, g_paused ? "PAUSED" : "running");
    mvprintw(2, 0,
        " Blue=Re  Red=Im  White=|ψ|²  Yellow=V(x)");
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
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);
    load_preset(g_preset);

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
        case 'r': case 'R': load_preset(g_preset); break;
        case '1': load_preset(0); break;
        case '2': load_preset(1); break;
        case '3': load_preset(2); break;
        case '4': load_preset(3); break;
        case '+': case '=':
            g_k0 *= 1.2f; if (g_k0 > 600.f) g_k0 = 600.f;
            load_preset(g_preset); break;
        case '-':
            g_k0 *= 0.833f; if (g_k0 < 30.f) g_k0 = 30.f;
            load_preset(g_preset); break;
        case ' ':
            /* Add momentum kick */
            for (int i = 0; i < N_GRID; i++) {
                float x = (float)i / (float)(N_GRID-1);
                float kre = cosf(100.f * x), kim = sinf(100.f * x);
                float nr = g_re[i]*kre - g_im[i]*kim;
                float ni = g_re[i]*kim + g_im[i]*kre;
                g_re[i] = nr; g_im[i] = ni;
            }
            break;
        default: break;
        }

        long long now = clock_ns();
        if (!g_paused)
            for (int s = 0; s < STEPS_PER_FRAME; s++) schrodinger_step();

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
