/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/* dune_rocket.c — Harkonnen siege: homing missiles pound the Arrakis desert
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/dune_rocket.c -o dune_rocket -lncurses -lm
 *
 * Keys: q quit | p pause | r reset | +/- launch rate | Space salvo
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Homing missile simulation with proportional navigation.
 *                  Each rocket steers toward a target using a turn rate
 *                  (TURN_RATE) that bends the velocity vector each tick.
 *                  Lateral wobble (WOBBLE_AMP × sin(WOBBLE_FREQ × t)) adds
 *                  visual realism to the flight path.
 *
 * Physics        : Rocket speed increases from ROCKET_SPEED0 toward
 *                  ROCKET_SPEEDMAX each frame (acceleration phase).
 *                  Trajectory is integrated with explicit Euler:
 *                    pos += vel × dt;  vel direction bent by TURN_RATE.
 *
 * Rendering      : Rocket orientation mapped to one of 8 ASCII direction
 *                  chars (▲↗→↘▼↙←↖); trail stored as fixed-length ring
 *                  buffer (TRAIL_LEN=30) of past positions drawn at
 *                  decreasing brightness to show motion blur.
 *                  Explosion sparks are simple ballistic particles with
 *                  gravity applied each tick.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── §1 config ───────────────────────────────────────────────────────────── */
#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TICK_NS         33333333L   /* 30 Hz */
#define MAX_ROCKETS     28
#define MAX_EXPLOSIONS  16
#define N_SPARKS        22
#define TRAIL_LEN       30
#define MAX_PORTS        8
#define SHIP_TOP_ROW     1          /* topmost ship row */
#define SHIP_ROWS        5          /* ship height in rows */

#define ROCKET_SPEED0   15.0f       /* launch speed (cells/sec) */
#define ROCKET_SPEEDMAX 44.0f       /* terminal speed */
#define TURN_RATE        3.0f       /* steering sharpness */
#define WOBBLE_AMP       0.28f      /* lateral wobble (radians equiv.) */
#define WOBBLE_FREQ      4.0f       /* oscillations per second */
#define LAUNCH_RATE0     1.1f       /* seconds between auto-launches */

#define MAX_COLS 512

/* ── §2 clock ────────────────────────────────────────────────────────────── */
static long long clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns) {
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 color ────────────────────────────────────────────────────────────── */
enum {
    CP_SHIP_HULL = 1, CP_SHIP_PORT,
    CP_RKT_HEAD, CP_RKT_HOT, CP_RKT_MID, CP_RKT_DIM,
    CP_EXP_CORE, CP_EXP_MID, CP_EXP_DIM,
    CP_GROUND, CP_SAND, CP_SCORCH,
    CP_HUD
};

static void color_init(void) {
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_SHIP_HULL, 250,  236);  /* light gray / dark gray */
        init_pair(CP_SHIP_PORT, 196,  236);  /* bright red / dark gray */
        init_pair(CP_RKT_HEAD,  231,   -1);  /* bright white */
        init_pair(CP_RKT_HOT,   214,   -1);  /* orange */
        init_pair(CP_RKT_MID,   202,   -1);  /* red-orange */
        init_pair(CP_RKT_DIM,    88,   -1);  /* dark red */
        init_pair(CP_EXP_CORE,  231,   -1);  /* white */
        init_pair(CP_EXP_MID,   226,   -1);  /* yellow */
        init_pair(CP_EXP_DIM,   208,   -1);  /* orange */
        init_pair(CP_GROUND,    136,   -1);  /* sandy gold */
        init_pair(CP_SAND,       94,   -1);  /* dark sand */
        init_pair(CP_SCORCH,     52,   -1);  /* very dark red */
        init_pair(CP_HUD,       232,  250);  /* dark text on silver bar */
    } else {
        init_pair(CP_SHIP_HULL, COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_SHIP_PORT, COLOR_RED,    COLOR_BLACK);
        init_pair(CP_RKT_HEAD,  COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_RKT_HOT,   COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_RKT_MID,   COLOR_RED,    COLOR_BLACK);
        init_pair(CP_RKT_DIM,   COLOR_RED,    COLOR_BLACK);
        init_pair(CP_EXP_CORE,  COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_EXP_MID,   COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_EXP_DIM,   COLOR_RED,    COLOR_BLACK);
        init_pair(CP_GROUND,    COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_SAND,      COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_SCORCH,    COLOR_RED,    COLOR_BLACK);
        init_pair(CP_HUD,       COLOR_BLACK,  COLOR_WHITE);
    }
}

/* ── §4 terrain ──────────────────────────────────────────────────────────── */
static int   g_terrain[MAX_COLS];  /* ground row at each column */
static int   g_scorch[MAX_COLS];   /* scorch intensity 0..4 */
static float g_scorch_decay;

static void terrain_init(int cols, int rows) {
    for (int c = 0; c < cols; c++) {
        float x = (float)c / (float)(cols > 1 ? cols - 1 : 1);
        /* four frequencies of sine for realistic dune profile */
        float h = sinf(x * (float)M_PI * 2.3f)         * 2.0f
                + sinf(x * (float)M_PI * 5.7f  + 0.73f) * 1.3f
                + sinf(x * (float)M_PI * 12.1f + 1.47f) * 0.7f
                + sinf(x * (float)M_PI * 27.3f + 0.23f) * 0.3f;
        int gr = (rows - 7) + (int)(h + 0.5f);
        if (gr < rows - 12) gr = rows - 12;
        if (gr > rows - 4)  gr = rows - 4;
        g_terrain[c] = gr;
    }
    memset(g_scorch, 0, sizeof g_scorch);
    g_scorch_decay = 0.f;
}

static void terrain_scorch(float cx, float radius) {
    int c0 = (int)(cx - radius);
    int c1 = (int)(cx + radius + 1.f);
    for (int c = c0; c <= c1; c++) {
        if (c < 0 || c >= MAX_COLS) continue;
        float d = fabsf((float)c - cx) / radius;
        if (d >= 1.f) continue;
        int s = 4 - (int)(d * 4.f);
        if (s > g_scorch[c]) g_scorch[c] = s;
    }
}

static void terrain_tick(float dt, int cols) {
    g_scorch_decay += dt;
    if (g_scorch_decay > 3.0f) {
        g_scorch_decay = 0.f;
        for (int c = 0; c < cols; c++)
            if (g_scorch[c] > 0) g_scorch[c]--;
    }
}

/* ── §5 ship ─────────────────────────────────────────────────────────────── */
static int   g_ship_left, g_ship_right;
static int   g_port_x[MAX_PORTS];
static float g_port_flash[MAX_PORTS];
static int   g_n_ports;
static int   g_launch_row;          /* row rockets spawn from */

static void ship_init(int cols) {
    int w = (cols * 3) / 4;
    if (w > 82) w = 82;
    if (w < 30) w = 30;
    int cx       = cols / 2;
    g_ship_left  = cx - w / 2;
    g_ship_right = g_ship_left + w;
    g_launch_row = SHIP_TOP_ROW + SHIP_ROWS;

    g_n_ports = MAX_PORTS;
    int span  = w - 8;
    int div   = g_n_ports > 1 ? g_n_ports - 1 : 1;
    for (int i = 0; i < g_n_ports; i++) {
        g_port_x[i]    = g_ship_left + 4 + span * i / div;
        g_port_flash[i] = 0.f;
    }
}

static void ship_draw(void) {
    int L = g_ship_left, R = g_ship_right;
    int row = SHIP_TOP_ROW;

    /* title row */
    const char *title = "HARKONNEN  CARRIER";
    int tx = (L + R) / 2 - (int)strlen(title) / 2;
    attron(COLOR_PAIR(CP_SHIP_HULL) | A_BOLD);
    mvprintw(row, tx, "%s", title);
    row++;

    /* top border */
    mvaddch(row, L, ACS_ULCORNER);
    for (int c = L + 1; c < R; c++) mvaddch(row, c, ACS_HLINE);
    mvaddch(row, R, ACS_URCORNER);
    row++;

    /* hull body — hatched armor texture */
    attroff(A_BOLD);
    mvaddch(row, L, ACS_VLINE);
    for (int c = L + 1; c < R; c++) {
        char bc = (((c - L) * 3 + row) % 7 < 2) ? '#' : '=';
        mvaddch(row, c, (chtype)(unsigned char)bc);
    }
    mvaddch(row, R, ACS_VLINE);
    row++;

    /* bottom border: launch port notches with ACS_TTEE */
    attron(A_BOLD);
    mvaddch(row, L, ACS_LLCORNER);
    for (int c = L + 1; c < R; c++) {
        int is_port = 0;
        for (int p = 0; p < g_n_ports; p++)
            if (c == g_port_x[p]) { is_port = 1; break; }
        mvaddch(row, c, is_port ? ACS_TTEE : ACS_HLINE);
    }
    mvaddch(row, R, ACS_LRCORNER);
    row++;

    /* port shafts: flash red on launch, otherwise gray */
    attroff(A_BOLD);
    for (int p = 0; p < g_n_ports; p++) {
        if (g_port_flash[p] > 0.f) {
            attron(COLOR_PAIR(CP_SHIP_PORT) | A_BOLD);
            mvaddch(row, g_port_x[p], '*');
            attroff(COLOR_PAIR(CP_SHIP_PORT) | A_BOLD);
        } else {
            attron(COLOR_PAIR(CP_SHIP_HULL));
            mvaddch(row, g_port_x[p], ACS_VLINE);
            attroff(COLOR_PAIR(CP_SHIP_HULL));
        }
    }

    attroff(COLOR_PAIR(CP_SHIP_HULL) | A_BOLD);
}

/* ── §6 rockets ──────────────────────────────────────────────────────────── */
typedef enum { RS_FLYING = 0, RS_DONE } RState;

typedef struct {
    float  x, y;                   /* position (cells) */
    float  vx, vy;                 /* velocity (cells/sec) */
    float  tx, ty;                 /* ground target */
    float  wobble_ph;              /* wobble phase (radians) */
    float  wobble_freq;
    float  wobble_amp;
    float  speed;                  /* current speed magnitude */
    RState state;
    int    port;
    /* trail ring buffer */
    float  tbx[TRAIL_LEN];
    float  tby[TRAIL_LEN];
    int    t_head;                 /* next-write index */
    int    t_len;                  /* filled entries 0..TRAIL_LEN */
    int    active;
} Rocket;

static Rocket g_rockets[MAX_ROCKETS];

/* velocity → slope character for the rocket nose */
static char dir_char_rocket(float vx, float vy) {
    float a = atan2f(vy, vx) * (180.f / (float)M_PI);
    if (a >  -22.5f && a <=   22.5f) return '>';
    if (a >   22.5f && a <=   67.5f) return '\\';
    if (a >   67.5f && a <=  112.5f) return 'v';
    if (a >  112.5f && a <=  157.5f) return '/';
    if (a >  157.5f || a <= -157.5f) return '<';
    if (a > -157.5f && a <= -112.5f) return '\\';
    if (a > -112.5f && a <=  -67.5f) return '^';
    return '/';  /* -67.5 .. -22.5 → up-right */
}

/* velocity → slope character for trail segments */
static char dir_char_trail(float vx, float vy) {
    float ax = fabsf(vx), ay = fabsf(vy);
    if (ay > ax * 1.73f) return '|';
    if (ax > ay * 1.73f) return '-';
    return (vx * vy > 0.f) ? '\\' : '/';
}

static void rocket_launch(int port, float tx, float ty) {
    for (int i = 0; i < MAX_ROCKETS; i++) {
        if (g_rockets[i].active) continue;
        Rocket *r = &g_rockets[i];
        memset(r, 0, sizeof *r);
        r->active = 1;
        r->x      = (float)g_port_x[port];
        r->y      = (float)g_launch_row + 0.5f;
        /* slight random sideways lean at launch */
        float lean = ((float)(rand() % 201) - 100.f) * 0.018f;
        r->vx         = lean;
        r->vy         = ROCKET_SPEED0;
        r->tx         = tx;
        r->ty         = ty;
        r->wobble_ph  = (float)(rand() % 628) * 0.01f;
        r->wobble_freq = WOBBLE_FREQ * (0.65f + (float)(rand() % 70) * 0.01f);
        r->wobble_amp  = WOBBLE_AMP  * (0.55f + (float)(rand() % 90) * 0.01f);
        r->speed      = ROCKET_SPEED0;
        r->state      = RS_FLYING;
        r->port       = port;
        g_port_flash[port] = 0.15f;
        return;
    }
}

static void rocket_update(Rocket *r, float dt, int cols, int rows) {
    if (!r->active || r->state == RS_DONE) return;

    /* record current position in trail */
    r->tbx[r->t_head] = r->x;
    r->tby[r->t_head] = r->y;
    r->t_head = (r->t_head + 1) % TRAIL_LEN;
    if (r->t_len < TRAIL_LEN) r->t_len++;

    /* target vector */
    float dx   = r->tx - r->x;
    float dy   = r->ty - r->y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < 1.5f) { r->state = RS_DONE; return; }

    /* desired direction (unit) */
    float ddx = dx / dist, ddy = dy / dist;

    /* current direction (unit) */
    float spd = sqrtf(r->vx * r->vx + r->vy * r->vy);
    float cdx = (spd > 0.01f) ? r->vx / spd : 0.f;
    float cdy = (spd > 0.01f) ? r->vy / spd : 1.f;

    /* steer toward target — blend current heading toward desired */
    float tr = fminf(TURN_RATE * dt, 0.99f);
    cdx += (ddx - cdx) * tr;
    cdy += (ddy - cdy) * tr;
    float nlen = sqrtf(cdx * cdx + cdy * cdy);
    if (nlen > 0.0001f) { cdx /= nlen; cdy /= nlen; }

    /* lateral wobble fades as rocket nears target (final dive is straight) */
    float wscale  = fminf(1.f, dist / 10.f);
    float wobble  = r->wobble_amp * wscale * sinf(r->wobble_ph);
    r->wobble_ph += r->wobble_freq * dt;

    /* add wobble perpendicular to heading, then re-normalise so speed
       stays constant and wobble is purely a direction perturbation */
    float wdx = cdx - cdy * wobble;   /* perpendicular = (-cdy, cdx) */
    float wdy = cdy + cdx * wobble;
    float wlen = sqrtf(wdx * wdx + wdy * wdy);
    if (wlen > 0.0001f) { wdx /= wlen; wdy /= wlen; }

    /* accelerate toward terminal speed */
    r->speed = fminf(r->speed + 20.f * dt, ROCKET_SPEEDMAX);

    r->vx = wdx * r->speed;
    r->vy = wdy * r->speed;
    r->x += r->vx * dt;
    r->y += r->vy * dt;

    /* terrain / bounds hit */
    int ic = (int)(r->x + 0.5f);
    if (ic < 0 || ic >= cols)          { r->state = RS_DONE; return; }
    if (r->y >= (float)g_terrain[ic])  { r->state = RS_DONE; return; }
    if (r->y >= (float)(rows - 2))     { r->state = RS_DONE; return; }
}

/* ── §7 explosions ───────────────────────────────────────────────────────── */
typedef struct {
    float x, y, vx, vy;
    float life, max_life;
    int   active;
} Spark;

typedef struct {
    float x, y;
    float life, max_life;
    float ring_r;               /* expanding shockwave radius */
    Spark sparks[N_SPARKS];
    int   active;
} Explosion;

static Explosion g_explosions[MAX_EXPLOSIONS];

static void explosion_spawn(float x, float y) {
    terrain_scorch(x, 5.5f);
    for (int i = 0; i < MAX_EXPLOSIONS; i++) {
        if (g_explosions[i].active) continue;
        Explosion *e = &g_explosions[i];
        memset(e, 0, sizeof *e);
        e->active   = 1;
        e->x        = x;
        e->y        = y;
        e->life     = 2.0f;
        e->max_life = 2.0f;
        e->ring_r   = 0.f;
        for (int s = 0; s < N_SPARKS; s++) {
            float ang = (float)s / (float)N_SPARKS * 2.f * (float)M_PI
                      + ((float)(rand() % 100) - 50.f) * 0.01f;
            float spd = 4.f + (float)(rand() % 900) * 0.01f;
            /* clamp downward component — sparks burst mostly up and sideways */
            float sa  = sinf(ang);
            if (sa > 0.25f) sa = 0.25f;
            e->sparks[s].vx       = cosf(ang) * spd;
            e->sparks[s].vy       = sa * spd - 3.f;
            e->sparks[s].x        = x + ((float)(rand() % 5) - 2.f) * 0.3f;
            e->sparks[s].y        = y;
            e->sparks[s].life     = 0.35f + (float)(rand() % 70) * 0.01f;
            e->sparks[s].max_life = e->sparks[s].life;
            e->sparks[s].active   = 1;
        }
        return;
    }
}

static void explosion_update(Explosion *e, float dt) {
    if (!e->active) return;
    e->life -= dt;
    if (e->life <= 0.f) { e->active = 0; return; }
    e->ring_r += 14.f * dt;
    for (int s = 0; s < N_SPARKS; s++) {
        Spark *sp = &e->sparks[s];
        if (!sp->active) continue;
        sp->life -= dt;
        if (sp->life <= 0.f) { sp->active = 0; continue; }
        sp->vy += 7.f * dt;   /* gravity pulls sparks down */
        sp->x  += sp->vx * dt;
        sp->y  += sp->vy * dt;
    }
}

/* ── §8 draw ─────────────────────────────────────────────────────────────── */
static void draw_terrain(int cols, int rows) {
    for (int c = 0; c < cols; c++) {
        int gr = g_terrain[c];
        if (gr < 0 || gr >= rows) continue;

        /* surface character from neighbour slope */
        int prev_gr = (c > 0)        ? g_terrain[c - 1] : gr;
        int next_gr = (c < cols - 1) ? g_terrain[c + 1] : gr;
        int slope   = next_gr - prev_gr;
        char sch = (slope < -1) ? '/' : (slope > 1) ? '\\' : '_';

        if (g_scorch[c] > 0) {
            /* scorch overrides surface */
            attron(COLOR_PAIR(CP_SCORCH) | A_BOLD);
            mvaddch(gr, c, g_scorch[c] >= 3 ? '*' : '.');
            attroff(COLOR_PAIR(CP_SCORCH) | A_BOLD);
        } else {
            attron(COLOR_PAIR(CP_GROUND) | A_BOLD);
            mvaddch(gr, c, (chtype)(unsigned char)sch);
            attroff(COLOR_PAIR(CP_GROUND) | A_BOLD);
        }

        /* sand fill below surface line */
        attron(COLOR_PAIR(CP_SAND));
        for (int r = gr + 1; r < rows - 1; r++) {
            char fc = ((c + r * 3) % 11 < 2) ? '.' : ' ';
            mvaddch(r, c, (chtype)(unsigned char)fc);
        }
        attroff(COLOR_PAIR(CP_SAND));
    }
}

static void draw_rockets(void) {
    for (int i = 0; i < MAX_ROCKETS; i++) {
        Rocket *r = &g_rockets[i];
        if (!r->active || r->state == RS_DONE) continue;

        /* trail: iterate oldest → newest */
        for (int t = 0; t < r->t_len; t++) {
            int idx = (r->t_head - r->t_len + t + TRAIL_LEN * 2) % TRAIL_LEN;
            int tr  = (int)(r->tby[idx] + 0.5f);
            int tc  = (int)(r->tbx[idx] + 0.5f);
            if (tr < 0 || tc < 0) continue;

            /* direction from this point to the next (gives correct slope char) */
            float dvx, dvy;
            if (t < r->t_len - 1) {
                int nxt = (r->t_head - r->t_len + t + 1 + TRAIL_LEN * 2) % TRAIL_LEN;
                dvx = r->tbx[nxt] - r->tbx[idx];
                dvy = r->tby[nxt] - r->tby[idx];
            } else {
                dvx = r->vx; dvy = r->vy;
            }
            char tch = dir_char_trail(dvx, dvy);

            /* age fraction 0=oldest 1=newest — drives brightness */
            float af = (r->t_len > 1) ? (float)t / (float)(r->t_len - 1) : 1.f;
            int   pair; attr_t attr = 0;
            if      (af > 0.70f) { pair = CP_RKT_HOT; attr = A_BOLD; }
            else if (af > 0.38f) { pair = CP_RKT_MID; }
            else                 { pair = CP_RKT_DIM;  attr = A_DIM;  }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(tr, tc, (chtype)(unsigned char)tch);
            attroff(COLOR_PAIR(pair) | attr);
        }

        /* rocket head */
        int hr = (int)(r->y + 0.5f);
        int hc = (int)(r->x + 0.5f);
        if (hr >= 0 && hc >= 0) {
            attron(COLOR_PAIR(CP_RKT_HEAD) | A_BOLD);
            mvaddch(hr, hc, (chtype)(unsigned char)dir_char_rocket(r->vx, r->vy));
            attroff(COLOR_PAIR(CP_RKT_HEAD) | A_BOLD);
        }
    }
}

static void draw_explosions(int cols, int rows) {
    for (int i = 0; i < MAX_EXPLOSIONS; i++) {
        Explosion *e = &g_explosions[i];
        if (!e->active) continue;
        float frac = e->life / e->max_life;  /* 1=fresh 0=dying */

        /* expanding shockwave ring (aspect-corrected ellipse) */
        if (e->ring_r < 22.f) {
            int n_pts = (int)(e->ring_r * 6.28f) + 12;
            int pair  = (frac > 0.55f) ? CP_EXP_CORE
                      : (frac > 0.25f) ? CP_EXP_MID : CP_EXP_DIM;
            char rch  = (frac > 0.55f) ? '*'
                      : (frac > 0.25f) ? '+' : '.';
            for (int p = 0; p < n_pts; p++) {
                float a  = (float)p / (float)n_pts * 2.f * (float)M_PI;
                int   rc = (int)(e->x + cosf(a) * e->ring_r + 0.5f);
                /* * 0.45 corrects for cell aspect ratio so ring looks circular */
                int   rr = (int)(e->y + sinf(a) * e->ring_r * 0.45f + 0.5f);
                if (rr < 0 || rr >= rows || rc < 0 || rc >= cols) continue;
                attron(COLOR_PAIR(pair));
                mvaddch(rr, rc, (chtype)(unsigned char)rch);
                attroff(COLOR_PAIR(pair));
            }
        }

        /* bright core flash on fresh explosions */
        if (frac > 0.65f) {
            int er = (int)(e->y + 0.5f);
            int ec = (int)(e->x + 0.5f);
            attron(COLOR_PAIR(CP_EXP_CORE) | A_BOLD);
            if (er >= 0 && er < rows && ec >= 0 && ec < cols)
                mvaddch(er, ec, '#');
            if (er - 1 >= 0) {
                mvaddch(er - 1, ec, '*');
                if (ec - 1 >= 0)    mvaddch(er - 1, ec - 1, '+');
                if (ec + 1 < cols)  mvaddch(er - 1, ec + 1, '+');
            }
            if (ec - 1 >= 0   && er < rows) mvaddch(er, ec - 1, '*');
            if (ec + 1 < cols && er < rows) mvaddch(er, ec + 1, '*');
            attroff(COLOR_PAIR(CP_EXP_CORE) | A_BOLD);
        }

        /* spark particles */
        for (int s = 0; s < N_SPARKS; s++) {
            Spark *sp = &e->sparks[s];
            if (!sp->active) continue;
            int sr  = (int)(sp->y + 0.5f);
            int sco = (int)(sp->x + 0.5f);
            if (sr < 0 || sr >= rows || sco < 0 || sco >= cols) continue;
            float sf  = sp->life / sp->max_life;
            int pair  = (sf > 0.6f) ? CP_EXP_CORE
                      : (sf > 0.3f) ? CP_EXP_MID : CP_EXP_DIM;
            char sch  = (sf > 0.6f) ? '*' : (sf > 0.3f) ? '+' : '.';
            attron(COLOR_PAIR(pair));
            mvaddch(sr, sco, (chtype)(unsigned char)sch);
            attroff(COLOR_PAIR(pair));
        }
    }
}

static void draw_hud(float rate, int n_active, int rows, int cols) {
    /* fill entire last row with the HUD background first */
    attron(COLOR_PAIR(CP_HUD));
    for (int c = 0; c < cols; c++) mvaddch(rows - 1, c, ' ');
    mvprintw(rows - 1, 1,
             "q:quit  p:pause  r:reset  +/-:rate(%.1fs)  "
             "rockets:%d/%d  Space:salvo",
             rate, n_active, MAX_ROCKETS);
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §9 scene ────────────────────────────────────────────────────────────── */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;
static void on_sigint(int s)   { (void)s; g_running = 0; }
static void on_sigwinch(int s) { (void)s; g_need_resize = 1; }

static float g_launch_rate  = LAUNCH_RATE0;
static float g_launch_timer = 0.f;
static int   g_paused       = 0;

static int count_active(void) {
    int n = 0;
    for (int i = 0; i < MAX_ROCKETS; i++) if (g_rockets[i].active) n++;
    return n;
}

static void scene_reset(int cols, int rows) {
    memset(g_rockets,    0, sizeof g_rockets);
    memset(g_explosions, 0, sizeof g_explosions);
    terrain_init(cols, rows);
    ship_init(cols);
    g_launch_timer = 0.3f;
}

static void scene_tick(float dt, int cols, int rows) {
    /* decay port flash timers */
    for (int p = 0; p < g_n_ports; p++)
        if (g_port_flash[p] > 0.f) g_port_flash[p] -= dt;

    /* auto-launch */
    g_launch_timer -= dt;
    if (g_launch_timer <= 0.f && count_active() < MAX_ROCKETS - 2) {
        int   port = rand() % g_n_ports;
        int   tc   = 2 + rand() % (cols - 4);
        float ty   = (float)g_terrain[tc] - 0.3f;
        rocket_launch(port, (float)tc, ty);
        /* jitter interval so launches don't feel mechanical */
        g_launch_timer = g_launch_rate * (0.4f + (float)(rand() % 120) * 0.01f);
    }

    /* update rockets; spawn explosion on impact */
    for (int i = 0; i < MAX_ROCKETS; i++) {
        Rocket *r = &g_rockets[i];
        if (!r->active) continue;
        rocket_update(r, dt, cols, rows);
        if (r->state == RS_DONE) {
            explosion_spawn(r->x, r->y);
            r->active = 0;
        }
    }

    for (int i = 0; i < MAX_EXPLOSIONS; i++)
        explosion_update(&g_explosions[i], dt);

    terrain_tick(dt, cols);
}

/* fire all ports at once — one rocket per port toward random targets */
static void scene_salvo(int cols) {
    for (int p = 0; p < g_n_ports && count_active() < MAX_ROCKETS - 1; p++) {
        int   tc = 2 + rand() % (cols - 4);
        float ty = (float)g_terrain[tc] - 0.3f;
        rocket_launch(p, (float)tc, ty);
    }
}

/* ── §10 main ────────────────────────────────────────────────────────────── */
int main(void) {
    srand((unsigned)time(NULL));
    signal(SIGINT,   on_sigint);
    signal(SIGTERM,  on_sigint);
    signal(SIGWINCH, on_sigwinch);

    initscr();
    noecho();
    cbreak();
    curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    color_init();

    int rows = LINES, cols = COLS;
    scene_reset(cols, rows);

    long long prev = clock_ns();

    while (g_running) {
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0; break;
            case 'p': case 'P': g_paused = !g_paused; break;
            case 'r': case 'R': scene_reset(cols, rows); break;
            case ' ':            scene_salvo(cols); break;
            case '+': case '=':
                g_launch_rate = fmaxf(0.2f, g_launch_rate - 0.2f); break;
            case '-':
                g_launch_rate = fminf(5.0f, g_launch_rate + 0.2f); break;
            case KEY_RESIZE: g_need_resize = 1; break;
            }
        }

        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            scene_reset(cols, rows);
        }

        long long now = clock_ns();
        float dt = (float)(now - prev) * 1e-9f;
        if (dt > 0.1f) dt = 0.1f;
        prev = now;

        if (!g_paused) scene_tick(dt, cols, rows);

        erase();
        draw_terrain(cols, rows);
        draw_explosions(cols, rows);
        draw_rockets();
        ship_draw();
        draw_hud(g_launch_rate, count_active(), rows, cols);
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }

    endwin();
    return 0;
}
