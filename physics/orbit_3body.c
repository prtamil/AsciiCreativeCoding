/*
 * orbit_3body.c — Three-Body Gravitational Orbits
 *
 * Three equal masses under mutual Newtonian gravity, integrated with
 * Velocity Verlet.  Starts with the stable figure-8 solution (Chenciner–
 * Montgomery 2000).  Key 'x' adds a random perturbation to show chaos.
 *
 * Trails fade with age: newest point is bright, oldest is dim.
 * Color encodes speed: slow → blue  fast → white.
 *
 * G = M = 1.  Time step dt = 0.001 normalised units.
 *
 * Keys: q quit  p pause  r reset  x perturb  +/- trail length  SPACE zoom
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/orbit_3body.c \
 *       -o orbit_3body -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 physics  §5 draw  §6 app
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

#define CELL_W    8
#define CELL_H   16

#define G_CONST   1.0f
#define MASS      1.0f
#define SOFTENING 0.02f     /* softening to prevent singularity */
#define DT        0.001f    /* integration time step            */
#define STEPS_PER_FRAME  8

#define TRAIL_MAX   200
#define RENDER_NS   (1000000000LL / 60)
#define HUD_ROWS     2

/* figure-8 initial conditions (normalized units) */
static const float INIT_X[3]  = {  0.97000436f, 0.f, -0.97000436f };
static const float INIT_Y[3]  = { -0.24308753f, 0.f,  0.24308753f };
static const float INIT_VX[3] = {  0.46620368f, -0.93240737f,  0.46620368f };
static const float INIT_VY[3] = {  0.43236573f, -0.86473146f,  0.43236573f };

enum { CP_B0=1, CP_B1, CP_B2, CP_B3, CP_G0, CP_G1, CP_G2, CP_G3,
       CP_R0, CP_R1, CP_R2, CP_R3, CP_HUD };

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
/* §3  color — 4 shades per body: dark→bright                            */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        /* body 0: blue gradient */
        init_pair(CP_B0,  17, -1);
        init_pair(CP_B1,  20, -1);
        init_pair(CP_B2,  27, -1);
        init_pair(CP_B3,  51, -1);
        /* body 1: green gradient */
        init_pair(CP_G0,  22, -1);
        init_pair(CP_G1,  28, -1);
        init_pair(CP_G2,  34, -1);
        init_pair(CP_G3,  46, -1);
        /* body 2: red/orange gradient */
        init_pair(CP_R0,  88, -1);
        init_pair(CP_R1, 124, -1);
        init_pair(CP_R2, 196, -1);
        init_pair(CP_R3, 208, -1);
        init_pair(CP_HUD,244, -1);
    } else {
        init_pair(CP_B0, COLOR_BLUE,    -1); init_pair(CP_B1, COLOR_BLUE, -1);
        init_pair(CP_B2, COLOR_CYAN,    -1); init_pair(CP_B3, COLOR_CYAN, -1);
        init_pair(CP_G0, COLOR_GREEN,   -1); init_pair(CP_G1, COLOR_GREEN,-1);
        init_pair(CP_G2, COLOR_GREEN,   -1); init_pair(CP_G3, COLOR_GREEN,-1);
        init_pair(CP_R0, COLOR_RED,     -1); init_pair(CP_R1, COLOR_RED,  -1);
        init_pair(CP_R2, COLOR_YELLOW,  -1); init_pair(CP_R3, COLOR_YELLOW,-1);
        init_pair(CP_HUD,COLOR_WHITE,   -1);
    }
}

static int body_cp(int body, int shade) /* shade 0=dim 3=bright */
{
    return CP_B0 + body * 4 + shade;
}

/* ===================================================================== */
/* §4  physics                                                            */
/* ===================================================================== */

static float g_x[3], g_y[3];     /* positions  */
static float g_vx[3], g_vy[3];   /* velocities */
static float g_ax[3], g_ay[3];   /* accelerations */

/* trail ring buffers */
static float g_tx[3][TRAIL_MAX], g_ty[3][TRAIL_MAX];
static int   g_trail_head;
static int   g_trail_len = 80;

static float g_sim_time = 0.f;
static bool  g_perturbed = false;

static void compute_accel(void)
{
    for (int i = 0; i < 3; i++) g_ax[i] = g_ay[i] = 0.f;
    for (int i = 0; i < 3; i++) {
        for (int j = i+1; j < 3; j++) {
            float dx = g_x[j] - g_x[i];
            float dy = g_y[j] - g_y[i];
            float r2 = dx*dx + dy*dy + SOFTENING*SOFTENING;
            float r  = sqrtf(r2);
            float f  = G_CONST * MASS * MASS / r2;
            float fx = f * dx / r;
            float fy = f * dy / r;
            g_ax[i] += fx / MASS;  g_ay[i] += fy / MASS;
            g_ax[j] -= fx / MASS;  g_ay[j] -= fy / MASS;
        }
    }
}

static void reset_figure8(void)
{
    for (int i = 0; i < 3; i++) {
        g_x[i]  = INIT_X[i];  g_y[i]  = INIT_Y[i];
        g_vx[i] = INIT_VX[i]; g_vy[i] = INIT_VY[i];
    }
    compute_accel();
    g_sim_time  = 0.f;
    g_perturbed = false;
    g_trail_head = 0;
    for (int b = 0; b < 3; b++)
        for (int k = 0; k < TRAIL_MAX; k++)
            g_tx[b][k] = g_ty[b][k] = -9999.f;
}

/* Velocity Verlet integration */
static void physics_step(void)
{
    float ax0[3], ay0[3];
    for (int i = 0; i < 3; i++) { ax0[i] = g_ax[i]; ay0[i] = g_ay[i]; }

    /* update positions */
    for (int i = 0; i < 3; i++) {
        g_x[i]  += g_vx[i]*DT + 0.5f*ax0[i]*DT*DT;
        g_y[i]  += g_vy[i]*DT + 0.5f*ay0[i]*DT*DT;
    }

    compute_accel();

    /* update velocities */
    for (int i = 0; i < 3; i++) {
        g_vx[i] += 0.5f*(ax0[i]+g_ax[i])*DT;
        g_vy[i] += 0.5f*(ay0[i]+g_ay[i])*DT;
    }

    g_sim_time += DT;
}

static void record_trail(void)
{
    for (int b = 0; b < 3; b++) {
        g_tx[b][g_trail_head] = g_x[b];
        g_ty[b][g_trail_head] = g_y[b];
    }
    g_trail_head = (g_trail_head + 1) % TRAIL_MAX;
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;
static float g_zoom = 1.0f;

/* Map sim coord to terminal cell */
static int sim_to_col(float sx)
{
    float scale = (float)(g_cols * CELL_W) / (g_zoom * 4.0f);
    float px = sx * scale + g_cols * CELL_W * 0.5f;
    return (int)(px / CELL_W);
}

static int sim_to_row(float sy)
{
    float scale = (float)((g_rows - HUD_ROWS) * CELL_H) / (g_zoom * 2.0f);
    float py = sy * scale + (g_rows - HUD_ROWS) * CELL_H * 0.5f;
    return (int)(py / CELL_H) + HUD_ROWS;
}

static void scene_draw(void)
{
    /* draw trails */
    for (int b = 0; b < 3; b++) {
        for (int k = 0; k < g_trail_len; k++) {
            int idx = (g_trail_head - 1 - k + TRAIL_MAX) % TRAIL_MAX;
            float tx = g_tx[b][idx], ty = g_ty[b][idx];
            if (tx < -9000.f) continue;
            int col = sim_to_col(tx);
            int row = sim_to_row(ty);
            if (row < HUD_ROWS || row >= g_rows || col < 0 || col >= g_cols) continue;
            float age = (float)k / (float)g_trail_len;
            int shade = (int)(3.f * (1.f - age));  /* 3=newest 0=oldest */
            if (shade < 0) shade = 0;
            attron(COLOR_PAIR(body_cp(b, shade)));
            mvaddch(row, col, shade == 3 ? '*' : shade == 2 ? '+' : '.');
            attroff(COLOR_PAIR(body_cp(b, shade)));
        }
    }

    /* draw bodies */
    static const chtype BODY_CH[3] = { '@', '@', '@' };
    for (int b = 0; b < 3; b++) {
        int col = sim_to_col(g_x[b]);
        int row = sim_to_row(g_y[b]);
        if (row < HUD_ROWS || row >= g_rows || col < 0 || col >= g_cols) continue;
        attron(COLOR_PAIR(body_cp(b, 3)) | A_BOLD);
        mvaddch(row, col, BODY_CH[b]);
        attroff(COLOR_PAIR(body_cp(b, 3)) | A_BOLD);
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " 3Body  q:quit  p:pause  r:reset  x:perturb  +/-:trail  spc:zoom");
    mvprintw(1, 0,
        " t=%.2f  zoom=%.1f  trail=%d  %s  %s",
        g_sim_time, g_zoom, g_trail_len,
        g_perturbed ? "[PERTURBED-CHAOTIC]" : "[figure-8]",
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
    reset_figure8();

    long long frame_time = clock_ns();
    int trail_timer = 0;

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            frame_time = clock_ns();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': reset_figure8(); break;
        case 'x': case 'X':
            for (int i = 0; i < 3; i++) {
                g_vx[i] += ((float)rand()/RAND_MAX - .5f) * 0.1f;
                g_vy[i] += ((float)rand()/RAND_MAX - .5f) * 0.1f;
            }
            compute_accel();
            g_perturbed = true;
            break;
        case '+': case '=':
            g_trail_len += 20; if (g_trail_len > TRAIL_MAX) g_trail_len = TRAIL_MAX; break;
        case '-':
            g_trail_len -= 20; if (g_trail_len < 10) g_trail_len = 10; break;
        case ' ':
            g_zoom = (g_zoom < 1.5f) ? 2.0f : 1.0f; break;
        default: break;
        }

        long long now = clock_ns();
        (void)(now - frame_time);
        frame_time = now;

        if (!g_paused) {
            for (int s = 0; s < STEPS_PER_FRAME; s++) physics_step();
            if (++trail_timer >= 2) { record_trail(); trail_timer = 0; }
        }

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
