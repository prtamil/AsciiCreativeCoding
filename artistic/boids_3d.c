/*
 * boids_3d.c — 3-D Boid Flocking with Perspective Projection
 *
 * Classic Reynolds boids (separation, alignment, cohesion) in 3-D space,
 * projected onto the terminal with a slowly rotating camera.  Depth cues
 * via character density: '.' far, '+' medium, 'o' close, '@' nearest.
 *
 * Boids wrap at the box boundary [-1,1]³.  Aspect ratio correction applied
 * during projection (terminal cells are ~2× taller than wide).
 *
 * Keys: q quit  p pause  r reset  +/- boid count  [/] flock speed
 *       SPACE toggle auto-rotate  ← → ↑ ↓ manual camera rotation
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/boids_3d.c \
 *       -o boids_3d -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 boids  §5 draw  §6 app
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

#define N_BOIDS_MAX    120
#define N_BOIDS_INIT    80
#define BOX_SIZE         1.0f

#define SEP_RADIUS   0.15f
#define ALI_RADIUS   0.40f
#define COH_RADIUS   0.50f
#define SEP_WEIGHT   2.0f
#define ALI_WEIGHT   1.0f
#define COH_WEIGHT   0.8f
#define MAX_SPEED    0.012f
#define MIN_SPEED    0.004f

#define CAM_DIST     3.0f     /* camera Z distance from origin */
#define CAM_ROT_SPD  0.005f   /* auto-rotation speed (rad/tick) */
#define FOV_SCALE    1.5f     /* field-of-view scale factor    */

#define SIM_FPS      60
#define RENDER_NS    (1000000000LL / SIM_FPS)
#define HUD_ROWS     2

enum { CP_FAR=1, CP_MED, CP_CLOSE, CP_NEAR, CP_HUD };

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
        init_pair(CP_FAR,   17, -1);   /* dark blue  — distant  */
        init_pair(CP_MED,   27, -1);   /* blue       — medium   */
        init_pair(CP_CLOSE, 51, -1);   /* cyan       — close    */
        init_pair(CP_NEAR, 231, -1);   /* white      — nearest  */
        init_pair(CP_HUD,  244, -1);
    } else {
        init_pair(CP_FAR,   COLOR_BLUE,  -1);
        init_pair(CP_MED,   COLOR_BLUE,  -1);
        init_pair(CP_CLOSE, COLOR_CYAN,  -1);
        init_pair(CP_NEAR,  COLOR_WHITE, -1);
        init_pair(CP_HUD,   COLOR_WHITE, -1);
    }
}

/* ===================================================================== */
/* §4  boids                                                              */
/* ===================================================================== */

typedef struct { float x, y, z, vx, vy, vz; } Boid;

static Boid g_boids[N_BOIDS_MAX];
static int  g_n_boids = N_BOIDS_INIT;

static void boids_init(void)
{
    for (int i = 0; i < g_n_boids; i++) {
        Boid *b = &g_boids[i];
        b->x  = ((float)rand()/RAND_MAX - .5f) * 2.f * BOX_SIZE;
        b->y  = ((float)rand()/RAND_MAX - .5f) * 2.f * BOX_SIZE;
        b->z  = ((float)rand()/RAND_MAX - .5f) * 2.f * BOX_SIZE;
        float spd = MIN_SPEED + (float)rand()/RAND_MAX*(MAX_SPEED-MIN_SPEED);
        float theta = (float)rand()/RAND_MAX * (float)(2*M_PI);
        float phi   = (float)rand()/RAND_MAX * (float)M_PI - (float)(M_PI/2);
        b->vx = spd * cosf(phi) * cosf(theta);
        b->vy = spd * cosf(phi) * sinf(theta);
        b->vz = spd * sinf(phi);
    }
}

static float len3(float x, float y, float z)
{
    return sqrtf(x*x + y*y + z*z);
}

static void boids_tick(void)
{
    static float ax[N_BOIDS_MAX], ay[N_BOIDS_MAX], az[N_BOIDS_MAX];

    for (int i = 0; i < g_n_boids; i++) {
        float sep_x=0, sep_y=0, sep_z=0;
        float ali_x=0, ali_y=0, ali_z=0;
        float coh_x=0, coh_y=0, coh_z=0;
        int ali_n=0, coh_n=0;
        Boid *a = &g_boids[i];

        for (int j = 0; j < g_n_boids; j++) {
            if (i == j) continue;
            Boid *b = &g_boids[j];
            float dx = a->x - b->x, dy = a->y - b->y, dz = a->z - b->z;
            float d = len3(dx, dy, dz);
            if (d < SEP_RADIUS && d > 0.f) {
                sep_x += dx/d; sep_y += dy/d; sep_z += dz/d;
            }
            if (d < ALI_RADIUS) {
                ali_x += b->vx; ali_y += b->vy; ali_z += b->vz; ali_n++;
            }
            if (d < COH_RADIUS) {
                coh_x += b->x; coh_y += b->y; coh_z += b->z; coh_n++;
            }
        }
        if (ali_n > 0) { ali_x /= ali_n; ali_y /= ali_n; ali_z /= ali_n; }
        if (coh_n > 0) {
            coh_x = coh_x/coh_n - a->x;
            coh_y = coh_y/coh_n - a->y;
            coh_z = coh_z/coh_n - a->z;
        }

        ax[i] = SEP_WEIGHT*sep_x + ALI_WEIGHT*ali_x + COH_WEIGHT*coh_x;
        ay[i] = SEP_WEIGHT*sep_y + ALI_WEIGHT*ali_y + COH_WEIGHT*coh_y;
        az[i] = SEP_WEIGHT*sep_z + ALI_WEIGHT*ali_z + COH_WEIGHT*coh_z;
    }

    for (int i = 0; i < g_n_boids; i++) {
        Boid *b = &g_boids[i];
        b->vx += ax[i] * 0.01f;
        b->vy += ay[i] * 0.01f;
        b->vz += az[i] * 0.01f;

        float spd = len3(b->vx, b->vy, b->vz);
        if (spd > MAX_SPEED) { b->vx *= MAX_SPEED/spd; b->vy *= MAX_SPEED/spd; b->vz *= MAX_SPEED/spd; }
        if (spd < MIN_SPEED && spd > 1e-6f) { b->vx *= MIN_SPEED/spd; b->vy *= MIN_SPEED/spd; b->vz *= MIN_SPEED/spd; }

        b->x += b->vx; b->y += b->vy; b->z += b->vz;

        /* wrap box */
        if (b->x >  BOX_SIZE) b->x -= 2.f*BOX_SIZE;
        if (b->x < -BOX_SIZE) b->x += 2.f*BOX_SIZE;
        if (b->y >  BOX_SIZE) b->y -= 2.f*BOX_SIZE;
        if (b->y < -BOX_SIZE) b->y += 2.f*BOX_SIZE;
        if (b->z >  BOX_SIZE) b->z -= 2.f*BOX_SIZE;
        if (b->z < -BOX_SIZE) b->z += 2.f*BOX_SIZE;
    }
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int   g_rows, g_cols;
static float g_cam_yaw   = 0.f;
static float g_cam_pitch = 0.15f;
static bool  g_autorot   = true;
static bool  g_paused;

/* Rotate (x,y,z) around Y axis by angle, then X axis by pitch */
static void rotate_point(float wx, float wy, float wz,
                          float yaw, float pitch,
                          float *ox, float *oy, float *oz)
{
    /* yaw around Y */
    float rx = wx*cosf(yaw) + wz*sinf(yaw);
    float ry = wy;
    float rz = -wx*sinf(yaw) + wz*cosf(yaw);
    /* pitch around X */
    *ox = rx;
    *oy = ry*cosf(pitch) - rz*sinf(pitch);
    *oz = ry*sinf(pitch) + rz*cosf(pitch);
}

static void scene_draw(void)
{
    if (g_autorot) g_cam_yaw += CAM_ROT_SPD;

    int cx = g_cols / 2, cy = (g_rows - HUD_ROWS) / 2 + HUD_ROWS;
    float scale_x = (float)(g_cols / 2) * FOV_SCALE;
    float scale_y = (float)((g_rows - HUD_ROWS) / 2) * FOV_SCALE;

    for (int i = 0; i < g_n_boids; i++) {
        float rx, ry, rz;
        rotate_point(g_boids[i].x, g_boids[i].y, g_boids[i].z,
                     g_cam_yaw, g_cam_pitch, &rx, &ry, &rz);

        float z_cam = rz + CAM_DIST;
        if (z_cam < 0.1f) continue;

        /* perspective projection — note y is inverted (screen y=0 is top) */
        float sx = rx / z_cam * scale_x + cx;
        float sy = -ry / z_cam * scale_y * 0.5f + cy;  /* *0.5 aspect correct */

        int col = (int)sx, row = (int)sy;
        if (row < HUD_ROWS || row >= g_rows || col < 0 || col >= g_cols) continue;

        /* depth cue */
        float t = 1.f - z_cam / (CAM_DIST + BOX_SIZE + 0.5f);
        int cp; chtype ch;
        if      (t < 0.25f) { cp = CP_FAR;   ch = '.'; }
        else if (t < 0.50f) { cp = CP_MED;   ch = '+'; }
        else if (t < 0.75f) { cp = CP_CLOSE; ch = 'o'; }
        else                { cp = CP_NEAR;  ch = '@'; }

        attron(COLOR_PAIR(cp) | (t > 0.5f ? A_BOLD : 0));
        mvaddch(row, col, ch);
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Boids3D  q:quit  p:pause  r:reset  +/-:count  [/]:speed  spc:rot  ←→↑↓:cam");
    mvprintw(1, 0,
        " N=%d  yaw=%.2f  pitch=%.2f  %s",
        g_n_boids, g_cam_yaw, g_cam_pitch,
        g_paused ? "PAUSED" : (g_autorot ? "auto-rotate" : "manual"));
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
    boids_init();

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
        case 'r': case 'R': boids_init(); break;
        case '+': case '=':
            g_n_boids += 10; if (g_n_boids > N_BOIDS_MAX) g_n_boids = N_BOIDS_MAX;
            boids_init();
            break;
        case '-':
            g_n_boids -= 10; if (g_n_boids < 10) g_n_boids = 10;
            boids_init();
            break;
        case ' ': g_autorot = !g_autorot; break;
        case KEY_LEFT:  g_cam_yaw   -= 0.05f; break;
        case KEY_RIGHT: g_cam_yaw   += 0.05f; break;
        case KEY_UP:    g_cam_pitch += 0.05f; break;
        case KEY_DOWN:  g_cam_pitch -= 0.05f; break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused) boids_tick();

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
