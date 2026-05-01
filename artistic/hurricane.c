/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hurricane.c — top-down hurricane / cyclone with calm eye
 *
 * DEMO: Bird's-eye view of a tropical cyclone. A pool of cloud particles
 *       orbits a central eye following the Rankine vortex velocity
 *       profile — solid-body rotation inside the eyewall, decaying
 *       1/r outside. Each particle slowly spirals inward (radial
 *       inflow) and is recycled when it reaches the eye boundary,
 *       respawning at a random outer radius. Three concentric "bands"
 *       of different colour intensity give a satellite-view texture:
 *       outer rim dim grey, middle band brighter cloud, eyewall white.
 *       The eye itself is a calm dark spot at the centre. The screen
 *       reads as a still-frame of a hurricane satellite image, but
 *       animated.
 *
 * Study alongside: artistic/fire_tornado.c (rotation, but side-view),
 *                  fluid/navier_stokes.c (real fluid dynamics),
 *                  geometry/voronoi.c (radial-distance ideas).
 *
 * Section map:
 *   §1 config    — particle count, eye/eyewall/outer radii, palette
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 5-stop cloud palette (eye dark → eyewall white)
 *   §4 random    — frand
 *   §5 vortex    — Rankine velocity profile + radial inflow + glyph
 *   §6 cloud     — Cloud particle struct + tick + draw
 *   §7 hurricane — Hurricane state, position, lifecycle
 *   §8 scene     — eye marker + clouds + HUD
 *   §9 screen    — ncurses init / cleanup
 *  §10 app       — signals, dt tracking, key handling, main loop
 *
 * Keys:  [/]   particle count
 *        -/+   spin speed
 *        ,/.   eye radius
 *        i     toggle inflow (clouds spiral in vs orbit fixed radius)
 *        t     cycle theme   r reseed   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/hurricane.c \
 *       -o hurricane -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Each cloud particle is stored in polar coordinates
 *                 around the screen centre: (radius, theta, age). Each
 *                 frame: theta advances by ω(r)·dt where the angular
 *                 velocity ω comes from the Rankine vortex profile —
 *                 solid-body rotation `ω = ω_max · r/R_eye` inside the
 *                 eyewall, and `ω = ω_max · R_eye/r` outside (so v(r)
 *                 = ω_max·R_eye is constant in the eyewall, then
 *                 decays as 1/r). Radial inflow pulls particles
 *                 inward at a small constant rate; particles that
 *                 cross the eye are recycled to the outer radius.
 *
 *                 Glyph and colour come from radius:
 *                   inside eye        → dark calm  (rare, just '.')
 *                   eyewall           → bright white '#' (high winds)
 *                   middle band       → mid-tone    'o' '*'
 *                   outer rim         → dim grey    '`' '.'
 *                 Plus a wind-aligned slope character (atan2 of velocity)
 *                 picked from a small set so the swirl direction is
 *                 visible even on a still frame.
 *
 * Data-structure: Hurricane holds Cloud[N_CLOUDS_MAX] inline. Each
 *                 Cloud carries (radius, theta, density). No allocation
 *                 after init.
 *
 * Rendering     : Per frame: erase, paint each cloud at its (radius,
 *                 theta) projection — with cell-aspect correction so the
 *                 hurricane reads as round, not vertically squashed.
 *
 * Performance   : O(N_CLOUDS) per frame. 600 particles is fine.
 *
 * References    :
 *   Rankine, "A Manual of Applied Mechanics" (1858) §625 — the original
 *     two-zone vortex model used in atmospheric science to first order.
 *   Holland, "An Analytic Model of the Wind and Pressure Profiles in
 *     Hurricanes" *Monthly Weather Review* 108 (1980) — modern profile;
 *     Rankine is the textbook simplification.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS         60

#define N_CLOUDS_MAX       800
#define N_CLOUDS_DEFAULT   500
#define N_CLOUDS_MIN       100
#define N_CLOUDS_STEP      100

/* Vortex profile. Radii are in cells. */
#define EYE_RADIUS_DEFAULT    4.0f
#define EYE_RADIUS_MIN        2.0f
#define EYE_RADIUS_MAX        8.0f
#define EYE_RADIUS_STEP       1.0f
#define OUTER_RADIUS_FRAC     0.85f   /* fraction of min(rows, cols/2)    */

#define OMEGA_MAX_DEFAULT     1.6f    /* rad/sec at the eyewall            */
#define OMEGA_MAX_MIN         0.5f
#define OMEGA_MAX_MAX         4.0f
#define OMEGA_MAX_STEP        0.25f
#define INFLOW_RATE           0.6f    /* radius units/sec inward            */

/* Cell aspect — terminal cells ~2× tall as wide. Stretch r·cos(θ) so
 * the hurricane is visually round on screen. */
#define ASPECT_X              2.0f

#define DT_CAP_S              0.10f
#define N_THEMES              4

/* Colour pairs */
#define PAIR_OUTER    1
#define PAIR_BAND     2
#define PAIR_EYEWALL  3
#define PAIR_EYE      4
#define PAIR_HUD      5
#define PAIR_HINT     6

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* 4-zone palette per theme: outer / band / eyewall / eye.
 *
 * Brightness gradient is encoded in the colour values themselves
 * (outer = mid-bright, eyewall = white) plus A_BOLD on the eyewall —
 * we no longer apply A_DIM to the outer band, since on most terminals
 * dim × medium-saturation = invisible.
 *
 * Eye colour is a fixed visible gray (240) regardless of theme — the
 * eye is a static "calm centre" feature, not part of the storm hue. */
static const short PAL_256[N_THEMES][4] = {
    /* 0 satellite white — light gray → bright white                   */
    { 250, 254, 231, 240 },
    /* 1 blue storm      — sky blue → cyan → white                     */
    {  39,  45,  51, 240 },
    /* 2 gold storm      — orange → gold → white                       */
    { 178, 214, 231, 240 },
    /* 3 magenta storm   — pink → light pink → white                   */
    { 175, 213, 231, 240 },
};
static const short PAL_8[N_THEMES][4] = {
    { COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE },
    { COLOR_BLUE,    COLOR_CYAN,    COLOR_WHITE,   COLOR_WHITE },
    { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE,   COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    init_pair(PAIR_OUTER,   x256 ? PAL_256[theme][0] : PAL_8[theme][0], -1);
    init_pair(PAIR_BAND,    x256 ? PAL_256[theme][1] : PAL_8[theme][1], -1);
    init_pair(PAIR_EYEWALL, x256 ? PAL_256[theme][2] : PAL_8[theme][2], -1);
    init_pair(PAIR_EYE,     x256 ? PAL_256[theme][3] : PAL_8[theme][3], -1);
    init_pair(PAIR_HUD,     x256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,    x256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  random                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  vortex profile                                                      */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * rankine_omega — Rankine vortex angular velocity at radius r.
 *   Inside  eye/eyewall (r ≤ R_eye):  ω = ω_max · r / R_eye   (solid body)
 *   Outside              (r >  R_eye): ω = ω_max · R_eye / r  (free vortex)
 * The cusp at r = R_eye is where the maximum tangential wind v = ω_max·R_eye
 * occurs — physically the eyewall.
 */
static float rankine_omega(float r, float r_eye, float omega_max)
{
    if (r < 1e-3f) return omega_max * 1e-3f / r_eye;
    if (r <= r_eye) return omega_max * r / r_eye;
    return omega_max * r_eye / r;
}

/*
 * radial_zone — pick a band index (0..3) by radius.
 *   0 = outer rim, 1 = mid band, 2 = eyewall, 3 = inside eye.
 */
static int radial_zone(float r, float r_eye, float r_outer)
{
    if (r < r_eye * 0.7f)            return 3;   /* inside eye         */
    if (r < r_eye * 1.15f)           return 2;   /* eyewall            */
    if (r < r_outer * 0.6f)          return 1;   /* mid band           */
    return 0;                                    /* outer rim          */
}

/*
 * wind_glyph — directional character for a particle moving at angle
 * `theta_vel` (atan2 of (vx, vy)). 8 sectors map to 8 glyphs that
 * trace the swirl direction even on a still frame.
 */
static char wind_glyph(float vx, float vy)
{
    static const char G[8] = { '-', '\\', '|', '/', '-', '\\', '|', '/' };
    float ang = atan2f(vy, vx);                  /* -π..π */
    if (ang < 0) ang += 2.0f * (float)M_PI;
    int sect = (int)(ang / (2.0f * (float)M_PI / 8.0f));
    if (sect < 0) sect = 0;
    if (sect > 7) sect = 7;
    return G[sect];
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cloud                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    float r;          /* radius from eye centre (cells)                   */
    float theta;      /* radians                                            */
    int   alive;
} Cloud;

static void cloud_respawn(Cloud *c, float r_outer)
{
    /* Bias toward larger radii (uniform in r²). */
    c->r     = r_outer * sqrtf(0.4f + 0.6f * frand());
    c->theta = frand() * 2.0f * (float)M_PI;
    c->alive = 1;
}

/*
 * cloud_tick — advance theta via Rankine omega; pull r inward by the
 * inflow rate. Recycle when the cloud crosses the eye into nothingness.
 */
static void cloud_tick(Cloud *c, float r_eye, float r_outer,
                       float omega_max, int inflow, float dt)
{
    if (!c->alive) {
        cloud_respawn(c, r_outer);
        return;
    }
    c->theta += rankine_omega(c->r, r_eye, omega_max) * dt;
    if (inflow) c->r -= INFLOW_RATE * dt;
    /* Wrap theta. */
    if (c->theta >  2.0f * (float)M_PI) c->theta -= 2.0f * (float)M_PI;
    if (c->theta < -2.0f * (float)M_PI) c->theta += 2.0f * (float)M_PI;
    /* Recycle if pulled into the eye or pushed out beyond the outer rim. */
    if (c->r < r_eye * 0.5f || c->r > r_outer * 1.05f)
        cloud_respawn(c, r_outer);
}

static void cloud_draw(const Cloud *c, int cx, int cy,
                       float r_eye, float r_outer,
                       float omega_max,
                       int rows, int cols)
{
    if (!c->alive) return;
    /* Project polar to screen with horizontal aspect stretch. */
    float sx = (float)cx + ASPECT_X * c->r * cosf(c->theta);
    float sy = (float)cy + c->r * sinf(c->theta);
    int   sr = (int)sy, sc = (int)sx;
    if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) return;

    int   zone = radial_zone(c->r, r_eye, r_outer);

    /* Skip the eye interior — that gives a clean empty centre, the way
     * a real cyclone reads on satellite imagery. The static eye marker
     * (`draw_eye`) is the only thing painted in that region. */
    if (zone == 3) return;

    short pair  = (zone == 2) ? PAIR_EYEWALL
                : (zone == 1) ? PAIR_BAND
                :                PAIR_OUTER;
    chtype attr = COLOR_PAIR(pair);
    if (zone == 2) attr |= A_BOLD;          /* eyewall pops */

    /* Velocity direction → glyph. v = ω×r in tangential direction. */
    float omega = rankine_omega(c->r, r_eye, omega_max);
    float vx = -sinf(c->theta) * omega * c->r * ASPECT_X;
    float vy =  cosf(c->theta) * omega * c->r;
    char  ch = wind_glyph(vx, vy);

    attron(attr);
    mvaddch(sr, sc, (chtype)(unsigned char)ch);
    attroff(attr);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  hurricane                                                           */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    Cloud clouds[N_CLOUDS_MAX];
    int   n;

    float r_eye;
    float r_outer;
    float omega_max;
    int   inflow;          /* 1 = clouds spiral inward, 0 = orbit fixed */

    int   theme;
    int   paused;
} Hurricane;

static void hurricane_geometry(Hurricane *h, int rows, int cols)
{
    /* Outer radius is the largest circle that fits inside the screen,
     * accounting for cell-aspect: horizontal extent scales by ASPECT_X. */
    float max_h = (float)(rows - 2) * 0.5f;
    float max_w = (float)cols * 0.5f / ASPECT_X;
    float bound = (max_h < max_w) ? max_h : max_w;
    h->r_outer = bound * OUTER_RADIUS_FRAC;
}

static void hurricane_reseed(Hurricane *h)
{
    for (int i = 0; i < h->n; i++) cloud_respawn(&h->clouds[i], h->r_outer);
}

static void hurricane_tick(Hurricane *h, float dt)
{
    for (int i = 0; i < h->n; i++)
        cloud_tick(&h->clouds[i], h->r_eye, h->r_outer,
                   h->omega_max, h->inflow, dt);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Faint eye marker — three '.' at the very centre, dim. */
static void draw_eye(int cx, int cy, int rows, int cols)
{
    if (cy < 0 || cy >= rows - 1) return;
    attron(COLOR_PAIR(PAIR_EYE) | A_BOLD);
    if (cx >= 0 && cx < cols)            mvaddch(cy, cx,     (chtype)'.');
    if (cx - 2 >= 0 && cx - 2 < cols)    mvaddch(cy, cx - 2, (chtype)'.');
    if (cx + 2 >= 0 && cx + 2 < cols)    mvaddch(cy, cx + 2, (chtype)'.');
    attroff(COLOR_PAIR(PAIR_EYE) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Hurricane *h, double fps)
{
    erase();
    int cx = cols / 2;
    int cy = (rows - 1) / 2;

    for (int i = 0; i < h->n; i++)
        cloud_draw(&h->clouds[i], cx, cy,
                   h->r_eye, h->r_outer, h->omega_max,
                   rows, cols);

    draw_eye(cx, cy, rows, cols);

    /* HUD */
    char buf[160];
    snprintf(buf, sizeof buf,
             " clouds:%d  spin:%.2f  eye:%.0f  inflow:%s  theme:%d  "
             "%5.1f fps  %s ",
             h->n, h->omega_max, h->r_eye,
             h->inflow ? "on " : "off",
             h->theme, fps,
             h->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " [/]:clouds  -/+:spin  ,/.:eye  i:inflow  t:theme  "
             "r:reset  p:pause  q:quit  [hurricane] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

static Hurricane g_h;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_h.n         = N_CLOUDS_DEFAULT;
    g_h.r_eye     = EYE_RADIUS_DEFAULT;
    g_h.omega_max = OMEGA_MAX_DEFAULT;
    g_h.inflow    = 1;
    g_h.theme     = 0;

    screen_init(g_h.theme);
    int rows = LINES, cols = COLS;
    hurricane_geometry(&g_h, rows, cols);
    hurricane_reseed(&g_h);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            hurricane_geometry(&g_h, rows, cols);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g_h.paused ^= 1; break;
                case 'r':          hurricane_reseed(&g_h); break;
                case 't':          g_h.theme = (g_h.theme + 1) % N_THEMES;
                                   color_init(g_h.theme); break;
                case 'i':          g_h.inflow ^= 1; break;
                case '[':
                    if (g_h.n - N_CLOUDS_STEP >= N_CLOUDS_MIN)
                        g_h.n -= N_CLOUDS_STEP;
                    break;
                case ']':
                    if (g_h.n + N_CLOUDS_STEP <= N_CLOUDS_MAX) {
                        int old = g_h.n;
                        g_h.n += N_CLOUDS_STEP;
                        for (int i = old; i < g_h.n; i++)
                            cloud_respawn(&g_h.clouds[i], g_h.r_outer);
                    }
                    break;
                case '-':
                    if (g_h.omega_max - OMEGA_MAX_STEP >= OMEGA_MAX_MIN)
                        g_h.omega_max -= OMEGA_MAX_STEP;
                    break;
                case '+': case '=':
                    if (g_h.omega_max + OMEGA_MAX_STEP <= OMEGA_MAX_MAX)
                        g_h.omega_max += OMEGA_MAX_STEP;
                    break;
                case ',':
                    if (g_h.r_eye - EYE_RADIUS_STEP >= EYE_RADIUS_MIN)
                        g_h.r_eye -= EYE_RADIUS_STEP;
                    break;
                case '.':
                    if (g_h.r_eye + EYE_RADIUS_STEP <= EYE_RADIUS_MAX)
                        g_h.r_eye += EYE_RADIUS_STEP;
                    break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_h.paused) hurricane_tick(&g_h, dt);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_h, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
