/*
 * reaction_wave.c — FitzHugh-Nagumo Excitable Medium
 *
 * Two-variable PDE on the terminal grid:
 *
 *   du/dt = u − u³/3 − v  +  D·∇²u  +  I(impulse)
 *   dv/dt = ε·(u + a − b·v)
 *
 * At rest (u≈−1.2, v≈−0.625) the medium is quiescent.  A localised
 * suprathreshold stimulus kicks u upward; the activator u fires, recovers
 * through the inhibitor v, and the wave propagates outward as a ring.
 * With suitable initial conditions a rotating spiral wave appears.
 *
 * 4 presets (1-4):
 *   1 TARGET RINGS — concentric expanding rings from the centre
 *   2 DOUBLE       — two offset sources → collision / annihilation
 *   3 SPIRAL       — rotating spiral from a broken ring
 *   4 PLANE WAVE   — wave front sweeping left to right
 *
 * Keys:  q quit   spc trigger impulse at centre   1-4 preset
 *        + - sim speed   p pause   r reset to resting state
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/reaction_wave.c \
 *       -o reaction_wave -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 grid  §5 presets  §6 scene  §7 app
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* FitzHugh-Nagumo parameters (excitable regime) */
#define FN_A    0.70f   /* threshold shift                       */
#define FN_B    0.80f   /* inhibitor feedback                    */
#define FN_EPS  0.08f   /* ratio of time scales (slow inhibitor) */
#define FN_D    0.10f   /* activator diffusion coefficient        */
#define FN_DT   0.04f   /* explicit Euler timestep               */

/* Resting-state fixed point (u* satisfies bu*³/3 − (b−1)u* = a) */
#define U_REST  (-1.20f)
#define V_REST  (-0.625f)
#define U_KICK   2.00f   /* value injected for a suprathreshold stimulus */

#define STEPS_PER_FRAME  8    /* sim steps per rendered frame              */
#define RENDER_NS        (1000000000LL / 30)

#define GRID_W_MAX  320
#define GRID_H_MAX  100
#define HUD_ROWS      2

enum {
    CP_REST = 1,  /* u ≈ -1.2  resting   — dark blue        */
    CP_REC,       /* u ≈ -0.5  recovery  — medium blue       */
    CP_RISE,      /* u ≈  0    threshold — cyan              */
    CP_WAVE,      /* u ≈  1    active    — bright white      */
    CP_FRONT,     /* u ≈  2    peak      — bold white        */
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
        init_pair(CP_REST,  17, -1);    /* very dark blue    */
        init_pair(CP_REC,   19, -1);    /* dark blue         */
        init_pair(CP_RISE,  51, -1);    /* cyan              */
        init_pair(CP_WAVE, 195, -1);    /* pale cyan-white   */
        init_pair(CP_FRONT,231, -1);    /* bright white      */
        init_pair(CP_HUD,  244, -1);    /* grey              */
    } else {
        init_pair(CP_REST,  COLOR_BLUE,  -1);
        init_pair(CP_REC,   COLOR_BLUE,  -1);
        init_pair(CP_RISE,  COLOR_CYAN,  -1);
        init_pair(CP_WAVE,  COLOR_CYAN,  -1);
        init_pair(CP_FRONT, COLOR_WHITE, -1);
        init_pair(CP_HUD,   COLOR_WHITE, -1);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

static float g_u[GRID_H_MAX][GRID_W_MAX];
static float g_v[GRID_H_MAX][GRID_W_MAX];
static float g_u2[GRID_H_MAX][GRID_W_MAX];
static float g_v2[GRID_H_MAX][GRID_W_MAX];
static int   g_gh, g_gw;   /* actual grid dimensions */

static void grid_fill_rest(void)
{
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++) {
            g_u[r][c] = U_REST;
            g_v[r][c] = V_REST;
        }
}

/*
 * grid_step() — one explicit Euler step of the FN PDE.
 *
 * Laplacian: 5-point stencil with Neumann (zero-flux) boundary conditions.
 * Boundary cells use interior neighbours only (reflected ghost cells).
 */
static void grid_step(void)
{
    for (int r = 0; r < g_gh; r++) {
        for (int c = 0; c < g_gw; c++) {
            float u = g_u[r][c], v = g_v[r][c];

            /* 5-point Laplacian with clamped neighbours */
            int ru = r > 0        ? r-1 : 0;
            int rd = r < g_gh-1   ? r+1 : g_gh-1;
            int cl = c > 0        ? c-1 : 0;
            int cr = c < g_gw-1   ? c+1 : g_gw-1;
            float lap = g_u[ru][c] + g_u[rd][c] + g_u[r][cl] + g_u[r][cr]
                        - 4.f * u;

            float du = (u - u*u*u/3.f - v) + FN_D*lap;
            float dv = FN_EPS * (u + FN_A - FN_B*v);

            g_u2[r][c] = u + FN_DT * du;
            g_v2[r][c] = v + FN_DT * dv;
        }
    }
    /* swap buffers */
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++) {
            g_u[r][c] = g_u2[r][c];
            g_v[r][c] = g_v2[r][c];
        }
}

/* inject a circular suprathreshold stimulus centred at (cr, cc), radius R */
static void inject(int cr, int cc, int R)
{
    for (int r = cr - R; r <= cr + R; r++)
        for (int c = cc - R; c <= cc + R; c++) {
            int dr = r-cr, dc = c-cc;
            if (dr*dr + dc*dc > R*R) continue;
            if (r < 0||r>=g_gh||c < 0||c>=g_gw) continue;
            g_u[r][c] = U_KICK;
        }
}

/* ===================================================================== */
/* §5  presets                                                            */
/* ===================================================================== */

static const char *g_preset_name = "RINGS";

static void preset_rings(void)
{
    g_preset_name = "TARGET RINGS";
    grid_fill_rest();
    inject(g_gh/2, g_gw/2, 4);
}

static void preset_double(void)
{
    g_preset_name = "DOUBLE SOURCE";
    grid_fill_rest();
    inject(g_gh/2, g_gw/3,   4);
    inject(g_gh/2, g_gw*2/3, 4);
}

static void preset_spiral(void)
{
    g_preset_name = "SPIRAL";
    grid_fill_rest();
    /* full ring stimulus + broken-ring phase shift creates spiral */
    inject(g_gh/2, g_gw/2, 5);
    /* erase the left half so the ring is broken → tip becomes rotor */
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw/2; c++)
            if (g_u[r][c] > U_REST + 0.5f)
                g_u[r][c] = U_REST;
}

static void preset_plane(void)
{
    g_preset_name = "PLANE WAVE";
    grid_fill_rest();
    for (int r = 0; r < g_gh; r++) {
        g_u[r][0] = U_KICK;
        g_u[r][1] = U_KICK;
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

static bool g_paused = false;
static int  g_speed  = STEPS_PER_FRAME;

static void scene_draw(int rows, int cols)
{
    for (int r = HUD_ROWS; r < g_gh && r < rows; r++) {
        for (int c = 0; c < g_gw && c < cols; c++) {
            float u = g_u[r][c];
            int   cp;
            chtype ch;
            if      (u < -0.80f) { cp = CP_REST;  ch = ' '; }
            else if (u < -0.20f) { cp = CP_REC;   ch = '.'; }
            else if (u <  0.50f) { cp = CP_RISE;  ch = ':'; }
            else if (u <  1.20f) { cp = CP_WAVE;  ch = '+'; }
            else                 { cp = CP_FRONT; ch = '#'; }

            attron(COLOR_PAIR(cp) | (cp == CP_FRONT ? A_BOLD : 0));
            mvaddch(r, c, ch);
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " FitzHugh-Nagumo  q:quit  1:rings  2:double  3:spiral  4:plane"
        "  spc:impulse  p:pause  +/-:speed  r:reset");
    mvprintw(1, 0,
        " preset:%-14s  speed:%dx  a=%.2f b=%.2f ε=%.2f D=%.2f dt=%.3f  %s",
        g_preset_name, g_speed,
        FN_A, FN_B, FN_EPS, FN_D, FN_DT,
        g_paused ? "PAUSED" : "running");
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

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_gh = rows < GRID_H_MAX ? rows : GRID_H_MAX;
    g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
    preset_rings();

    long long last = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            g_gh = rows < GRID_H_MAX ? rows : GRID_H_MAX;
            g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
            preset_rings();
            last = clock_ns();
            continue;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1;              break;
        case 'p': case 'P':          g_paused = !g_paused;    break;
        case ' ':   inject(g_gh/2, g_gw/2, 4);                break;
        case '1':   preset_rings();                            break;
        case '2':   preset_double();                           break;
        case '3':   preset_spiral();                           break;
        case '4':   preset_plane();                            break;
        case 'r': case 'R':          grid_fill_rest();         break;
        case '+': case '=':
            g_speed++; if (g_speed > 20) g_speed = 20;        break;
        case '-':
            g_speed--; if (g_speed < 1)  g_speed = 1;         break;
        default: break;
        }

        long long now = clock_ns();
        last = now;
        (void)last;

        if (!g_paused)
            for (int s = 0; s < g_speed; s++) grid_step();

        erase();
        scene_draw(rows, cols);
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }

    return 0;
}
