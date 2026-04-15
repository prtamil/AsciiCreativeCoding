/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * elastic_collision.c — Hard Sphere Billiards
 *
 * 25 discs with randomised radii and velocities bouncing off walls and
 * each other.  Perfectly elastic collisions preserve kinetic energy and
 * momentum.  After N² pair checks each tick, discs flash red on impact.
 *
 * Physics in pixel space (CELL_W=8, CELL_H=16 px per terminal cell).
 * Disc mass proportional to radius² (equal density).
 *
 * Keys: q quit  p pause  r reset  +/- sim speed  SPACE add impulse
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/elastic_collision.c \
 *       -o elastic_collision -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 physics  §5 draw  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Impulse-based elastic collision resolution.
 *                  For each overlapping pair, a single impulse along the
 *                  collision normal simultaneously adjusts both velocities
 *                  so the constraint (non-penetration + elastic restitution)
 *                  is satisfied in one step.
 *
 * Physics        : Conservation laws for elastic collisions:
 *                  (1) Conservation of momentum: m₁v₁ + m₂v₂ = const
 *                  (2) Conservation of kinetic energy: ½m₁v₁² + ½m₂v₂² = const
 *                  Combined, for collision along normal n̂:
 *                    impulse J = 2·m₁·m₂/(m₁+m₂) · Δv·n̂
 *                  Velocities updated: v₁ -= J/m₁·n̂;  v₂ += J/m₂·n̂
 *
 * Performance    : O(N²) pair checks per tick — acceptable for N=25.
 *                  For N>100 broad-phase (spatial hash or sweep-and-prune)
 *                  would reduce to O(N) average checks.
 *
 * Mass model     : mass = r²  (area of 2D disc × uniform density).
 *                  Heavier discs (larger radius) deflect smaller ones
 *                  more, matching intuition about billiard balls.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define CELL_W    8
#define CELL_H   16
#define N_DISCS   25

/* R_MIN/R_MAX: disc radius range in pixels.
 * R_MIN = 1.5 cells wide — large enough to be visible on screen.
 * R_MAX = 4 cells wide — at N=25 discs, max radius must not make placement
 * impossible; 4 cells keeps most random layouts placeable within 200 tries. */
#define R_MIN    (CELL_W * 1.5f)
#define R_MAX    (CELL_W * 4.0f)

/* V_MAX: initial speed cap (px/s).  180 px/s at CELL_W=8 ≈ 22.5 cells/s.
 * At 60 fps that's 0.375 cells per frame — fast enough to look dynamic
 * but slow enough that a disc doesn't skip over another in one tick.      */
#define V_MAX     180.f

/* FLASH_S: impact flash duration (seconds).
 * 0.4 s is just above typical human reaction time (~0.25 s), making
 * collision flashes easy to notice without looking persistent.            */
#define FLASH_S    0.4f

/* SIM_FPS: physics tick rate.  120 Hz gives dt ≈ 8.3 ms — small enough
 * that at V_MAX a disc moves only 1.5 px per tick, preventing tunnelling. */
#define SIM_FPS    120
#define RENDER_NS  (1000000000LL / 60)   /* 60 fps render period (ns) */
#define HUD_ROWS    2

enum { CP_SLOW=1, CP_MED, CP_FAST, CP_FLASH, CP_HUD };

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
        init_pair(CP_SLOW,  21, -1);   /* blue   — slow  */
        init_pair(CP_MED,   51, -1);   /* cyan   — med   */
        init_pair(CP_FAST, 231, -1);   /* white  — fast  */
        init_pair(CP_FLASH,196, -1);   /* red    — flash */
        init_pair(CP_HUD,  244, -1);
    } else {
        init_pair(CP_SLOW,  COLOR_BLUE,   -1);
        init_pair(CP_MED,   COLOR_CYAN,   -1);
        init_pair(CP_FAST,  COLOR_WHITE,  -1);
        init_pair(CP_FLASH, COLOR_RED,    -1);
        init_pair(CP_HUD,   COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  physics                                                            */
/* ===================================================================== */

typedef struct {
    float x, y;     /* center, pixel space  */
    float vx, vy;   /* velocity, px/s       */
    float r;         /* radius, px           */
    float mass;      /* = r²                 */
    float flash;     /* time remaining flash */
} Disc;

static Disc g_disc[N_DISCS];
static float g_pw, g_ph;   /* pixel world size */
static long long g_coll_total = 0;

static void disc_init(float pw, float ph)
{
    g_pw = pw; g_ph = ph;
    for (int i = 0; i < N_DISCS; i++) {
        Disc *d = &g_disc[i];
        d->r    = R_MIN + (float)rand() / RAND_MAX * (R_MAX - R_MIN);
        d->mass = d->r * d->r;
        d->flash = 0.f;
        /* Place discs without initial overlap (simple retry) */
        int tries = 0;
        do {
            d->x = d->r + (float)rand() / RAND_MAX * (pw - 2*d->r);
            d->y = d->r + (float)rand() / RAND_MAX * (ph - 2*d->r);
            tries++;
            bool ok = true;
            for (int j = 0; j < i && ok; j++) {
                float dx = d->x - g_disc[j].x, dy = d->y - g_disc[j].y;
                if (dx*dx + dy*dy < (d->r + g_disc[j].r) * (d->r + g_disc[j].r))
                    ok = false;
            }
            if (ok) break;
        } while (tries < 200);

        float speed = 60.f + (float)rand() / RAND_MAX * (V_MAX - 60.f);
        float angle = (float)rand() / RAND_MAX * 6.28318f;
        d->vx = speed * cosf(angle);
        d->vy = speed * sinf(angle);
    }
    g_coll_total = 0;
}

static void disc_tick(float dt)
{
    /* move */
    for (int i = 0; i < N_DISCS; i++) {
        Disc *d = &g_disc[i];
        d->x += d->vx * dt;
        d->y += d->vy * dt;
        d->flash -= dt;
        if (d->flash < 0.f) d->flash = 0.f;

        /* wall bounces */
        if (d->x - d->r < 0.f)  { d->x = d->r;        d->vx =  fabsf(d->vx); }
        if (d->x + d->r > g_pw) { d->x = g_pw - d->r; d->vx = -fabsf(d->vx); }
        if (d->y - d->r < 0.f)  { d->y = d->r;        d->vy =  fabsf(d->vy); }
        if (d->y + d->r > g_ph) { d->y = g_ph - d->r; d->vy = -fabsf(d->vy); }
    }

    /* disc-disc collisions: O(N²) — fine for N=25 */
    for (int i = 0; i < N_DISCS - 1; i++) {
        for (int j = i + 1; j < N_DISCS; j++) {
            Disc *a = &g_disc[i], *b = &g_disc[j];
            float dx = b->x - a->x, dy = b->y - a->y;
            float dist2 = dx*dx + dy*dy;
            float min_d = a->r + b->r;
            if (dist2 >= min_d * min_d || dist2 < 1e-6f) continue;

            float dist = sqrtf(dist2);
            float nx = dx / dist, ny = dy / dist;

            /* Separate overlapping discs */
            float overlap = min_d - dist;
            float ma = a->mass, mb = b->mass;
            float total = ma + mb;
            a->x -= nx * overlap * mb / total;
            a->y -= ny * overlap * mb / total;
            b->x += nx * overlap * ma / total;
            b->y += ny * overlap * ma / total;

            /* Elastic collision: exchange momentum along the collision normal n̂.
             * dv  = relative velocity projected onto n̂ (>0 means approaching).
             * imp = impulse magnitude: 2·m₁·m₂/(m₁+m₂) · (dv projected)
             *       Derived from simultaneous conservation of momentum and KE.
             * Skip if dv ≤ 0: bodies already separating after overlap resolution. */
            float dv = (a->vx - b->vx)*nx + (a->vy - b->vy)*ny;
            if (dv <= 0.f) continue;   /* already separating — do not re-collide */

            float imp = 2.f * ma * mb / total * dv;  /* impulse = 2m₁m₂/(m₁+m₂)·Δv */
            a->vx -= imp / ma * nx;    /* a loses momentum in normal direction    */
            a->vy -= imp / ma * ny;
            b->vx += imp / mb * nx;    /* b gains equal-and-opposite momentum     */
            b->vy += imp / mb * ny;

            a->flash = FLASH_S;
            b->flash = FLASH_S;
            g_coll_total++;
        }
    }
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;
static int  g_speed = 1;

static int px_to_cell_x(float px) { return (int)(px / CELL_W + .5f); }
static int px_to_cell_y(float py) { return (int)(py / CELL_H + .5f) + HUD_ROWS; }

/* Draw a filled disc outline using the 'O' character at center + rim chars */
static void draw_disc(const Disc *d, int cp)
{
    int cx = px_to_cell_x(d->x);
    int cy = px_to_cell_y(d->y);
    int rx = (int)(d->r / CELL_W + .5f);
    int ry = (int)(d->r / CELL_H + .5f);
    if (rx < 1) rx = 1;
    if (ry < 1) ry = 1;

    attron(COLOR_PAIR(cp) | A_BOLD);

    /* center */
    if (cy >= HUD_ROWS && cy < g_rows && cx >= 0 && cx < g_cols)
        mvaddch(cy, cx, 'O');

    /* horizontal extent */
    for (int dc = -rx; dc <= rx; dc++) {
        int tc = cx + dc;
        if (tc < 0 || tc >= g_cols) continue;
        /* top & bottom of ellipse */
        float frac = 1.f - ((float)(dc*dc)) / ((float)(rx*rx));
        if (frac < 0.f) frac = 0.f;
        int dy_px = (int)(ry * sqrtf(frac));
        for (int sign = -1; sign <= 1; sign += 2) {
            int tr = cy + sign * dy_px;
            if (tr < HUD_ROWS || tr >= g_rows) continue;
            mvaddch(tr, tc, (dc == 0 || dc == -rx || dc == rx) ? '|' : '-');
        }
    }

    attroff(COLOR_PAIR(cp) | A_BOLD);
}

static void scene_draw(void)
{
    for (int i = 0; i < N_DISCS; i++) {
        const Disc *d = &g_disc[i];
        float speed = sqrtf(d->vx*d->vx + d->vy*d->vy);
        int cp;
        /* Speed-based colour: slow → blue, mid → cyan, fast → white.
         * Thresholds are 40% and 80% of V_MAX so the three tiers roughly
         * divide the visible speed range evenly. Flash overrides all.     */
        if (d->flash > 0.f)          cp = CP_FLASH;  /* just collided — red  */
        else if (speed < V_MAX*.4f)  cp = CP_SLOW;   /* < 72 px/s  → blue   */
        else if (speed < V_MAX*.8f)  cp = CP_MED;    /* 72–144 px/s → cyan  */
        else                         cp = CP_FAST;   /* > 144 px/s → white  */
        draw_disc(d, cp);
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " ElasticCollision  q:quit  p:pause  r:reset  +/-:speed  spc:impulse");
    mvprintw(1, 0,
        " N=%d  collisions:%lld  speed:%dx  %s",
        N_DISCS, (long long)g_coll_total, g_speed,
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
    float pw = (float)(g_cols * CELL_W);
    float ph = (float)((g_rows - HUD_ROWS) * CELL_H);
    disc_init(pw, ph);

    long long frame_time = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            pw = (float)(g_cols * CELL_W);
            ph = (float)((g_rows - HUD_ROWS) * CELL_H);
            disc_init(pw, ph);
            frame_time = clock_ns();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': disc_init(pw, ph); break;
        case '+': case '=': g_speed++; if (g_speed > 8) g_speed = 8; break;
        case '-': g_speed--; if (g_speed < 1) g_speed = 1; break;
        case ' ':
            /* random impulse to all discs */
            for (int i = 0; i < N_DISCS; i++) {
                float a = (float)rand() / RAND_MAX * 6.28318f;
                g_disc[i].vx += V_MAX * .5f * cosf(a);
                g_disc[i].vy += V_MAX * .5f * sinf(a);
            }
            break;
        default: break;
        }

        long long now = clock_ns();
        long long dt_ns = now - frame_time;
        frame_time = now;
        if (dt_ns > 100000000LL) dt_ns = 100000000LL;
        float dt = (float)dt_ns * 1e-9f;

        if (!g_paused)
            for (int s = 0; s < g_speed; s++) disc_tick(dt / g_speed);

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
