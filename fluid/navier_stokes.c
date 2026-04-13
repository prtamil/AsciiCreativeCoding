/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * navier_stokes.c — Stable Fluid (Jos Stam 1999)
 *
 * Velocity (u,v) + dye density on a grid.  Each frame:
 *   1. Add forces / dye sources
 *   2. Diffuse (Gauss-Seidel)
 *   3. Project (divergence-free via Poisson solve)
 *   4. Advect (semi-Lagrangian back-trace)
 *   5. Project again
 *
 * Arrow keys inject a velocity "wind" source at screen centre.
 * SPACE drops a dye blob at a random position.
 * 1/2/3 select dye color (blue/green/red).
 *
 * Keys: q quit  p pause  r reset  SPACE dye  ← → ↑ ↓ wind  +/- viscosity
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/navier_stokes.c \
 *       -o navier_stokes -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 fluid  §5 draw  §6 app
 */

#define _POSIX_C_SOURCE 200809L
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

#define N       80      /* grid cells per side (N×N internal, with 2 ghost cols) */
#define ITER    16      /* Gauss-Seidel iterations */
#define DT      0.05f
#define DIFF    0.0001f /* diffusion coefficient (viscosity) */
#define VISC_INIT 0.00001f
#define FORCE   50.f
#define SOURCE  50.f

#define RENDER_NS   (1000000000LL / 30)
#define HUD_ROWS    2

static float g_visc = VISC_INIT;
static int   g_dye_ch = 0;  /* 0=blue 1=green 2=red */

enum { CP_D0=1, CP_D1, CP_D2, CP_D3,
       CP_G0, CP_G1, CP_G2, CP_G3,
       CP_R0, CP_R1, CP_R2, CP_R3,
       CP_HUD };

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
        /* blue dye */
        init_pair(CP_D0,  17, -1); init_pair(CP_D1,  21, -1);
        init_pair(CP_D2,  27, -1); init_pair(CP_D3,  51, -1);
        /* green dye */
        init_pair(CP_G0,  22, -1); init_pair(CP_G1,  28, -1);
        init_pair(CP_G2,  34, -1); init_pair(CP_G3,  46, -1);
        /* red dye */
        init_pair(CP_R0,  88, -1); init_pair(CP_R1, 124, -1);
        init_pair(CP_R2, 160, -1); init_pair(CP_R3, 196, -1);
        init_pair(CP_HUD,244, -1);
    } else {
        init_pair(CP_D0, COLOR_BLUE,  -1); init_pair(CP_D1, COLOR_BLUE,  -1);
        init_pair(CP_D2, COLOR_CYAN,  -1); init_pair(CP_D3, COLOR_CYAN,  -1);
        init_pair(CP_G0, COLOR_GREEN, -1); init_pair(CP_G1, COLOR_GREEN, -1);
        init_pair(CP_G2, COLOR_GREEN, -1); init_pair(CP_G3, COLOR_GREEN, -1);
        init_pair(CP_R0, COLOR_RED,   -1); init_pair(CP_R1, COLOR_RED,   -1);
        init_pair(CP_R2, COLOR_YELLOW,-1); init_pair(CP_R3, COLOR_YELLOW,-1);
        init_pair(CP_HUD,COLOR_WHITE, -1);
    }
}

/* Return color pair base (0=blue,1=green,2=red) for dye channel */
static int dye_cp_base(int ch)
{
    if (ch == 1) return CP_G0;
    if (ch == 2) return CP_R0;
    return CP_D0;
}

/* ===================================================================== */
/* §4  fluid                                                              */
/* ===================================================================== */

#define SZ  ((N+2)*(N+2))
#define IX(i,j) ((i)+(N+2)*(j))

static float u[SZ], v[SZ];         /* velocity */
static float u_prev[SZ], v_prev[SZ];
static float dens[SZ], dens_prev[SZ];  /* dye density */

static void set_bnd(int b, float *x)
{
    for (int i = 1; i <= N; i++) {
        x[IX(0,  i)] = (b==1) ? -x[IX(1,i)] : x[IX(1,i)];
        x[IX(N+1,i)] = (b==1) ? -x[IX(N,i)] : x[IX(N,i)];
        x[IX(i,0  )] = (b==2) ? -x[IX(i,1)] : x[IX(i,1)];
        x[IX(i,N+1)] = (b==2) ? -x[IX(i,N)] : x[IX(i,N)];
    }
    x[IX(0,  0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0,  1)]);
    x[IX(0,  N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0,  N)]);
    x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);
    x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);
}

static void lin_solve(int b, float *x, float *x0, float a, float c)
{
    for (int k = 0; k < ITER; k++) {
        for (int j = 1; j <= N; j++)
            for (int i = 1; i <= N; i++)
                x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+
                                                 x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
        set_bnd(b, x);
    }
}

static void diffuse(int b, float *x, float *x0, float diff_coef, float dt)
{
    float a = dt * diff_coef * (float)(N*N);
    /* Warm-start GS from x0 — critical when x was a project() work array */
    memcpy(x, x0, SZ * sizeof(float));
    lin_solve(b, x, x0, a, 1.f + 4.f*a);
}

static void advect(int b, float *d, float *d0, float *uu, float *vv, float dt)
{
    float dt0 = dt * (float)N;
    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            float x = (float)i - dt0*uu[IX(i,j)];
            float y = (float)j - dt0*vv[IX(i,j)];
            if (x < 0.5f) x = 0.5f; else if (x > (float)N+0.5f) x = (float)N+0.5f;
            if (y < 0.5f) y = 0.5f; else if (y > (float)N+0.5f) y = (float)N+0.5f;
            int i0 = (int)x, i1 = i0+1;
            int j0 = (int)y, j1 = j0+1;
            float s1 = x - (float)i0, s0 = 1.f-s1;
            float t1 = y - (float)j0, t0 = 1.f-t1;
            d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])
                        +s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
        }
    }
    set_bnd(b, d);
}

static void project(float *uu, float *vv, float *p, float *div)
{
    float h = 1.f / (float)N;
    for (int j = 1; j <= N; j++)
        for (int i = 1; i <= N; i++) {
            div[IX(i,j)] = -0.5f*h*(uu[IX(i+1,j)]-uu[IX(i-1,j)]
                                   +vv[IX(i,j+1)]-vv[IX(i,j-1)]);
            p[IX(i,j)] = 0.f;
        }
    set_bnd(0, div); set_bnd(0, p);
    lin_solve(0, p, div, 1.f, 4.f);
    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            uu[IX(i,j)] -= 0.5f*(p[IX(i+1,j)]-p[IX(i-1,j)])/(float)N;
            vv[IX(i,j)] -= 0.5f*(p[IX(i,j+1)]-p[IX(i,j-1)])/(float)N;
        }
    }
    set_bnd(1, uu); set_bnd(2, vv);
}

static void fluid_step(void)
{
    /* velocity step */
    diffuse(1, u_prev, u, g_visc, DT); diffuse(2, v_prev, v, g_visc, DT);
    project(u_prev, v_prev, u, v);
    advect(1, u, u_prev, u_prev, v_prev, DT);
    advect(2, v, v_prev, u_prev, v_prev, DT);
    project(u, v, u_prev, v_prev);

    /* density step */
    diffuse(0, dens_prev, dens, DIFF, DT);
    advect(0, dens, dens_prev, u, v, DT);
}

static void add_source_at(int i, int j, float fu, float fv, float fd)
{
    if (i < 1 || i > N || j < 1 || j > N) return;
    u[IX(i,j)]    += DT * fu * FORCE;
    v[IX(i,j)]    += DT * fv * FORCE;
    dens[IX(i,j)] += DT * fd * SOURCE;
}

static void fluid_reset(void)
{
    memset(u,         0, sizeof u);
    memset(v,         0, sizeof v);
    memset(u_prev,    0, sizeof u_prev);
    memset(v_prev,    0, sizeof v_prev);
    memset(dens,      0, sizeof dens);
    memset(dens_prev, 0, sizeof dens_prev);
}

/* Two counter-rotating auto-emitters inject dye + swirling velocity every frame */
static float g_angle = 0.f;

static void inject_sources(void)
{
    g_angle += 0.04f;
    /* emitter 1 — left third */
    int i1 = N/3, j1 = N/2;
    float fu1 =  cosf(g_angle) * 1.5f;
    float fv1 =  sinf(g_angle) * 1.5f;
    add_source_at(i1, j1, fu1, fv1, 3.f);
    /* emitter 2 — right third (counter-rotate) */
    int i2 = 2*N/3, j2 = N/2;
    float fu2 = -cosf(g_angle) * 1.5f;
    float fv2 = -sinf(g_angle) * 1.5f;
    add_source_at(i2, j2, fu2, fv2, 3.f);
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;

/* Map grid cell (i,j) → terminal (col, row).  Grid is i=1..N (x), j=1..N (y). */
static int gc2col(int i) { return (i-1) * (g_cols)   / N; }
static int gc2row(int j) { return (N-j) * (g_rows - HUD_ROWS) / N + HUD_ROWS; }

static void scene_draw(void)
{
    /* Find max density for dynamic normalization */
    float max_d = 0.001f;
    for (int j = 1; j <= N; j++)
        for (int i = 1; i <= N; i++)
            if (dens[IX(i,j)] > max_d) max_d = dens[IX(i,j)];

    int base = dye_cp_base(g_dye_ch);
    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            float d = dens[IX(i,j)] / max_d;   /* normalised 0..1 */
            if (d < 0.02f) continue;
            int col = gc2col(i);
            int row = gc2row(j);
            if (row < HUD_ROWS || row >= g_rows || col < 0 || col >= g_cols) continue;
            int shade; chtype ch;
            if (d < 0.2f)      { shade = 0; ch = '.'; }
            else if (d < 0.5f) { shade = 1; ch = ':'; }
            else if (d < 0.8f) { shade = 2; ch = '+'; }
            else               { shade = 3; ch = '#'; }
            attron(COLOR_PAIR(base + shade));
            mvaddch(row, col, ch);
            attroff(COLOR_PAIR(base + shade));
        }
    }

    static const char *dye_names[] = { "blue", "green", "red" };
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " NavierStokes  q:quit  p:pause  r:reset  spc:dye  arrows:wind  +/-:visc  1/2/3:color");
    mvprintw(1, 0,
        " grid=%dx%d  visc=%.2e  dye=%s  %s",
        N, N, (double)g_visc, dye_names[g_dye_ch],
        g_paused ? "PAUSED" : "running");
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
    fluid_reset();

    /* Pre-warm: seed the simulation before first frame so dye is visible immediately */
    for (int pw = 0; pw < 80; pw++) { inject_sources(); fluid_step(); }

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
        }

        int ch = getch();
        int ci = N/2, cj = N/2;
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': fluid_reset(); break;
        case ' ':
            {
                int ri = 1 + rand() % N;
                int rj = 1 + rand() % N;
                add_source_at(ri, rj, 0.f, 0.f, 5.f);
            }
            break;
        case KEY_LEFT:  add_source_at(ci, cj, -1.f, 0.f, 1.f); break;
        case KEY_RIGHT: add_source_at(ci, cj,  1.f, 0.f, 1.f); break;
        case KEY_UP:    add_source_at(ci, cj,  0.f,-1.f, 1.f); break;
        case KEY_DOWN:  add_source_at(ci, cj,  0.f, 1.f, 1.f); break;
        case '+': case '=':
            g_visc *= 2.f; if (g_visc > 0.1f) g_visc = 0.1f; break;
        case '-':
            g_visc *= 0.5f; if (g_visc < 1e-7f) g_visc = 1e-7f; break;
        case '1': g_dye_ch = 0; break;
        case '2': g_dye_ch = 1; break;
        case '3': g_dye_ch = 2; break;
        default: break;
        }

        long long now = clock_ns();
        if (!g_paused) { inject_sources(); fluid_step(); }

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
