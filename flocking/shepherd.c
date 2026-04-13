/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shepherd.c — Shepherd and Sheep Herding
 *
 * A flock of sheep grazes using boid rules (separation + alignment + cohesion).
 * You are the shepherd (#) — move with arrow keys to herd the sheep.
 * Sheep within FLEE_RADIUS panic and scatter away from you.
 *
 * Sheep characters (based on speed):
 *   o          calm / slow grazing
 *   < > ^ v    moving (cardinal direction)
 *   / \        moving (diagonal)
 *   O          fleeing (fast, bold, red-orange)
 *
 * Shepherd: # (bright yellow, bold)
 *
 * Keys: q quit  p pause  r reset  arrows move shepherd
 *       +/- add/remove sheep  f toggle flee-radius ring
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/shepherd.c \
 *       -o shepherd -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 coords  §5 sheep  §6 shepherd  §7 scene  §8 app
 */

#define _GNU_SOURCE
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

/* Terminal cell sub-pixel geometry (same as flocking.c) */
#define CELL_W   8
#define CELL_H  16

#define SIM_FPS      60
#define RENDER_NS    (1000000000LL / 60)
#define HUD_ROWS     2

/* Sheep counts */
#define N_SHEEP_DEFAULT  25
#define N_SHEEP_MIN       5
#define N_SHEEP_MAX      60

/* Speeds in pixels/second */
#define SHEEP_SPEED      200.0f
#define SHEEP_SPEED_VAR   0.25f   /* ±25% cruise speed variation */
#define SHEPHERD_SPEED   320.0f   /* faster than sheep — you can catch them */
#define MIN_SPEED         60.0f
#define MAX_SPEED        520.0f

/* Boid perception */
#define PERCEPTION_RADIUS  160.0f
#define SEPARATION_RADIUS   55.0f

/* Steering weights */
#define W_SEPARATION   2.0f
#define W_ALIGNMENT    0.8f
#define W_COHESION     0.4f
#define W_FLEE         7.0f    /* must dominate cohesion so sheep actually scatter */
#define MAX_STEER      140.0f

/* Shepherd influence */
#define FLEE_RADIUS    180.0f   /* sheep panic within this distance of shepherd */
#define PANIC_RADIUS    70.0f   /* extra boost when very close */

/* Bounce damping at screen edges */
#define BOUNCE_DAMP    0.6f

enum {
    CP_SHEEP_CALM = 1,  /* white/grey — grazing */
    CP_SHEEP_MOVE,      /* light green — moving */
    CP_SHEEP_FLEE,      /* orange/red  — fleeing */
    CP_SHEPHERD,        /* bright yellow, bold */
    CP_RING,            /* dim ring showing flee radius */
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
    struct timespec req = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_SHEEP_CALM, 252,  -1);  /* light grey   — idle        */
        init_pair(CP_SHEEP_MOVE, 118,  -1);  /* bright green — moving      */
        init_pair(CP_SHEEP_FLEE, 208,  -1);  /* orange       — panicking   */
        init_pair(CP_SHEPHERD,   226,  -1);  /* bright yellow — you        */
        init_pair(CP_RING,       238,  -1);  /* dark grey    — radius ring */
        init_pair(CP_HUD,        244,  -1);
    } else {
        init_pair(CP_SHEEP_CALM, COLOR_WHITE,   -1);
        init_pair(CP_SHEEP_MOVE, COLOR_GREEN,   -1);
        init_pair(CP_SHEEP_FLEE, COLOR_RED,     -1);
        init_pair(CP_SHEPHERD,   COLOR_YELLOW,  -1);
        init_pair(CP_RING,       COLOR_WHITE,   -1);
        init_pair(CP_HUD,        COLOR_WHITE,   -1);
    }
}

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

static inline float px_w(int cols) { return (float)(cols * CELL_W); }
static inline float px_h(int rows) { return (float)(rows * CELL_H); }
static inline int   to_col(float px) { return (int)floorf(px / CELL_W + 0.5f); }
static inline int   to_row(float py) { return (int)floorf(py / CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  sheep                                                              */
/* ===================================================================== */

typedef struct {
    float px, py;
    float vx, vy;
    float cruise;   /* personal cruise speed */
    bool  fleeing;  /* set each tick — drives character choice */
} Sheep;

/* Pick a display character from velocity direction + flee state */
static chtype sheep_char(const Sheep *s)
{
    if (s->fleeing) return 'O';

    float spd = hypotf(s->vx, s->vy);
    if (spd < SHEEP_SPEED * 0.35f) return 'o';

    /* 8-sector direction glyph */
    static const char dirs[8] = {'<', '\\', '^', '/', '>', '\\', 'v', '/'};
    float angle   = atan2f(s->vy, s->vx);
    float shifted = angle + (float)M_PI + (float)M_PI / 8.0f;
    int   sector  = (int)floorf(shifted / ((float)M_PI / 4.0f)) % 8;
    return (chtype)dirs[sector];
}

static void sheep_spawn(Sheep *s, float px, float py)
{
    s->px     = px;
    s->py     = py;
    s->fleeing = false;
    float var = 1.0f - SHEEP_SPEED_VAR
              + 2.0f * SHEEP_SPEED_VAR * ((float)rand() / RAND_MAX);
    s->cruise = SHEEP_SPEED * var;

    /* Random direction */
    float angle = (float)rand() / RAND_MAX * 2.0f * (float)M_PI;
    s->vx = cosf(angle) * s->cruise;
    s->vy = sinf(angle) * s->cruise;
}

static void sheep_clamp(Sheep *s)
{
    float mag = hypotf(s->vx, s->vy);
    if (mag < 0.001f) { s->vx = MIN_SPEED; s->vy = 0.0f; return; }
    if (mag < MIN_SPEED) { s->vx = s->vx/mag*MIN_SPEED; s->vy = s->vy/mag*MIN_SPEED; }
    else if (mag > MAX_SPEED) { s->vx = s->vx/mag*MAX_SPEED; s->vy = s->vy/mag*MAX_SPEED; }
}

/* Bounce off screen boundaries — sheep get cornered when herded */
static void sheep_bounce(Sheep *s, float max_px, float max_py)
{
    if (s->px < 0.0f)      { s->px = 0.0f;    s->vx = fabsf(s->vx) * BOUNCE_DAMP; }
    if (s->px > max_px)    { s->px = max_px;   s->vx = -fabsf(s->vx) * BOUNCE_DAMP; }
    if (s->py < 0.0f)      { s->py = 0.0f;     s->vy = fabsf(s->vy) * BOUNCE_DAMP; }
    if (s->py > max_py)    { s->py = max_py;   s->vy = -fabsf(s->vy) * BOUNCE_DAMP; }
}

/* ===================================================================== */
/* §6  shepherd                                                           */
/* ===================================================================== */

typedef struct {
    float px, py;   /* position in pixel space */
    int   dir_x;    /* -1/0/+1 — set from held arrow keys each frame */
    int   dir_y;
} Shepherd;

static void shepherd_init(Shepherd *sh, float max_px, float max_py)
{
    sh->px    = max_px * 0.5f;
    sh->py    = max_py * 0.5f;
    sh->dir_x = 0;
    sh->dir_y = 0;
}

static void shepherd_move(Shepherd *sh, float dt, float max_px, float max_py)
{
    sh->px += (float)sh->dir_x * SHEPHERD_SPEED * dt;
    sh->py += (float)sh->dir_y * SHEPHERD_SPEED * dt;
    if (sh->px < 0.0f)    sh->px = 0.0f;
    if (sh->px > max_px)  sh->px = max_px;
    if (sh->py < 0.0f)    sh->py = 0.0f;
    if (sh->py > max_py)  sh->py = max_py;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

static Sheep    g_sheep[N_SHEEP_MAX];
static int      g_n    = N_SHEEP_DEFAULT;
static Shepherd g_shep;
static bool     g_paused;
static bool     g_show_ring = true;
static int      g_rows, g_cols;
static int      g_fleeing_count;

static void scene_init(void)
{
    float max_px = px_w(g_cols);
    float max_py = px_h(g_rows - HUD_ROWS);
    shepherd_init(&g_shep, max_px, max_py);

    for (int i = 0; i < g_n; i++) {
        /* Scatter across screen avoiding the center where shepherd starts */
        float ox, oy;
        do {
            ox = ((float)rand() / RAND_MAX) * max_px;
            oy = ((float)rand() / RAND_MAX) * max_py;
        } while (hypotf(ox - g_shep.px, oy - g_shep.py) < FLEE_RADIUS * 1.5f);
        sheep_spawn(&g_sheep[i], ox, oy);
    }
}

/* Compute new velocity for sheep[idx] using boids + flee */
static void sheep_steer(int idx, float *out_vx, float *out_vy)
{
    Sheep *b = &g_sheep[idx];
    float max_px = px_w(g_cols);
    float max_py = px_h(g_rows - HUD_ROWS);

    float sep_x = 0.f, sep_y = 0.f; int sep_n = 0;
    float ali_vx = 0.f, ali_vy = 0.f;
    float coh_dx = 0.f, coh_dy = 0.f; int nbr_n = 0;

    for (int j = 0; j < g_n; j++) {
        if (j == idx) continue;
        float dx = g_sheep[j].px - b->px;
        float dy = g_sheep[j].py - b->py;
        float dist = hypotf(dx, dy);
        if (dist >= PERCEPTION_RADIUS || dist < 0.001f) continue;

        ali_vx += g_sheep[j].vx;
        ali_vy += g_sheep[j].vy;
        coh_dx += dx;
        coh_dy += dy;
        nbr_n++;

        if (dist < SEPARATION_RADIUS) {
            float c = (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS;
            sep_x -= (dx / dist) * c;
            sep_y -= (dy / dist) * c;
            sep_n++;
        }
    }

    float steer_x = 0.f, steer_y = 0.f;

    if (sep_n > 0) {
        float sm = hypotf(sep_x, sep_y);
        if (sm > 0.001f) {
            steer_x += (sep_x/sm) * W_SEPARATION * MAX_STEER;
            steer_y += (sep_y/sm) * W_SEPARATION * MAX_STEER;
        }
    }
    if (nbr_n > 0) {
        float am = hypotf(ali_vx, ali_vy);
        if (am > 0.001f) {
            steer_x += ((ali_vx/am)*b->cruise - b->vx) * W_ALIGNMENT;
            steer_y += ((ali_vy/am)*b->cruise - b->vy) * W_ALIGNMENT;
        }
        float cm = hypotf(coh_dx, coh_dy);
        if (cm > 0.001f) {
            steer_x += ((coh_dx/cm)*b->cruise - b->vx) * W_COHESION;
            steer_y += ((coh_dy/cm)*b->cruise - b->vy) * W_COHESION;
        }
    }

    /* Flee from shepherd */
    float sdx  = b->px - g_shep.px;
    float sdy  = b->py - g_shep.py;
    float sdist = hypotf(sdx, sdy);
    b->fleeing = (sdist < FLEE_RADIUS);

    if (b->fleeing && sdist > 0.001f) {
        float weight = W_FLEE * (1.0f + (sdist < PANIC_RADIUS ? 2.0f : 0.0f));
        /* Clamp flee displacement to visible bounds to avoid sheep escaping */
        float tx = b->px + (sdx/sdist) * FLEE_RADIUS;
        float ty = b->py + (sdy/sdist) * FLEE_RADIUS;
        if (tx < 0.f) tx = 0.f; else if (tx > max_px) tx = max_px;
        if (ty < 0.f) ty = 0.f; else if (ty > max_py) ty = max_py;
        float fdx = tx - b->px, fdy = ty - b->py;
        float fm = hypotf(fdx, fdy);
        if (fm > 0.001f) {
            steer_x += (fdx/fm) * weight * MAX_STEER;
            steer_y += (fdy/fm) * weight * MAX_STEER;
        }
    }

    /* Clamp total steer */
    float smag = hypotf(steer_x, steer_y);
    if (smag > MAX_STEER * W_FLEE) {
        steer_x = steer_x / smag * MAX_STEER * W_FLEE;
        steer_y = steer_y / smag * MAX_STEER * W_FLEE;
    }

    *out_vx = b->vx + steer_x;
    *out_vy = b->vy + steer_y;
    (void)max_px; (void)max_py;
}

static void scene_tick(float dt)
{
    float max_px = px_w(g_cols);
    float max_py = px_h(g_rows - HUD_ROWS);

    shepherd_move(&g_shep, dt, max_px, max_py);

    /* Two-phase boid update */
    float new_vx[N_SHEEP_MAX], new_vy[N_SHEEP_MAX];
    for (int i = 0; i < g_n; i++)
        sheep_steer(i, &new_vx[i], &new_vy[i]);

    g_fleeing_count = 0;
    for (int i = 0; i < g_n; i++) {
        g_sheep[i].vx = new_vx[i];
        g_sheep[i].vy = new_vy[i];
        sheep_clamp(&g_sheep[i]);
        g_sheep[i].px += g_sheep[i].vx * dt;
        g_sheep[i].py += g_sheep[i].vy * dt;
        sheep_bounce(&g_sheep[i], max_px, max_py);
        if (g_sheep[i].fleeing) g_fleeing_count++;
    }
}

static void scene_draw(void)
{
    /* Optional flee-radius ring around shepherd */
    if (g_show_ring) {
        int sc = to_col(g_shep.px);
        int sr = to_row(g_shep.py) + HUD_ROWS;
        int rx = (int)(FLEE_RADIUS / CELL_W);
        int ry = (int)(FLEE_RADIUS / CELL_H);
        attron(COLOR_PAIR(CP_RING));
        for (int a = 0; a < 64; a++) {
            float ang = (float)a * 2.0f * (float)M_PI / 64.0f;
            int rc = sc + (int)(rx * cosf(ang));
            int rr = sr + (int)(ry * sinf(ang));
            if (rr >= HUD_ROWS && rr < g_rows && rc >= 0 && rc < g_cols)
                mvaddch(rr, rc, '.');
        }
        attroff(COLOR_PAIR(CP_RING));
    }

    /* Draw sheep */
    for (int i = 0; i < g_n; i++) {
        int col = to_col(g_sheep[i].px);
        int row = to_row(g_sheep[i].py) + HUD_ROWS;
        if (row < HUD_ROWS || row >= g_rows || col < 0 || col >= g_cols) continue;

        int cp; attr_t attr = A_NORMAL;
        if (g_sheep[i].fleeing) {
            cp   = CP_SHEEP_FLEE;
            attr = A_BOLD;
        } else if (hypotf(g_sheep[i].vx, g_sheep[i].vy) > SHEEP_SPEED * 0.4f) {
            cp = CP_SHEEP_MOVE;
        } else {
            cp = CP_SHEEP_CALM;
        }

        attron(COLOR_PAIR(cp) | attr);
        mvaddch(row, col, sheep_char(&g_sheep[i]));
        attroff(COLOR_PAIR(cp) | attr);
    }

    /* Draw shepherd */
    {
        int sc = to_col(g_shep.px);
        int sr = to_row(g_shep.py) + HUD_ROWS;
        if (sr >= HUD_ROWS && sr < g_rows && sc >= 0 && sc < g_cols) {
            attron(COLOR_PAIR(CP_SHEPHERD) | A_BOLD);
            mvaddch(sr, sc, '#');
            attroff(COLOR_PAIR(CP_SHEPHERD) | A_BOLD);
        }
    }

    /* HUD */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Shepherd  q:quit  p:pause  r:reset  arrows:move  +/-:sheep  f:ring");
    mvprintw(1, 0,
        " sheep:%d  fleeing:%d  shepherd:#  calm:o  moving:<>^v  fleeing:O  %s",
        g_n, g_fleeing_count,
        g_paused ? "PAUSED" : "herding");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §8  app                                                                */
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
    scene_init();

    long long last = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            scene_init();
            last = clock_ns();
        }

        /* Drain input — track arrow direction for this frame */
        g_shep.dir_x = 0;
        g_shep.dir_y = 0;
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_quit = 1;            break;
            case 'p': case 'P': g_paused = !g_paused;            break;
            case 'r': case 'R': scene_init();                     break;
            case 'f': case 'F': g_show_ring = !g_show_ring;      break;
            case '+': case '=':
                if (g_n < N_SHEEP_MAX) {
                    float max_px = px_w(g_cols);
                    float max_py = px_h(g_rows - HUD_ROWS);
                    sheep_spawn(&g_sheep[g_n++],
                        ((float)rand()/RAND_MAX) * max_px,
                        ((float)rand()/RAND_MAX) * max_py);
                }
                break;
            case '-':
                if (g_n > N_SHEEP_MIN) g_n--;
                break;
            case KEY_LEFT:  g_shep.dir_x = -1; break;
            case KEY_RIGHT: g_shep.dir_x =  1; break;
            case KEY_UP:    g_shep.dir_y = -1; break;
            case KEY_DOWN:  g_shep.dir_y =  1; break;
            default: break;
            }
        }

        long long now = clock_ns();
        float dt = (float)(now - last) * 1e-9f;
        last = now;
        if (dt > 0.05f) dt = 0.05f;

        if (!g_paused)
            scene_tick(dt);

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
