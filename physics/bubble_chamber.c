/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bubble_chamber.c — Charged Particles in a Magnetic Field
 *
 * Simulates a bubble chamber: charged particles travel through a region of
 * uniform magnetic field (B perpendicular to the screen) and leave curved
 * ionisation tracks as they lose energy.
 *
 * Physics
 *   Lorentz force (2D, B along z-axis):
 *     Rotate velocity by  omega = (q/m_eff) * B  each step.
 *     v' = R(omega) · v   — exact rotation matrix; no Euler spiral drift.
 *   Ionisation drag:
 *     |v| *= (1 − DRAG)  each step  →  orbit spirals inward.
 *   Cyclotron radius:
 *     r = |v| / |omega|   →  light particles curl tight, heavy ones arc gently.
 *
 * Particle types  (q/m_eff tuned for clear visual curvature on a terminal)
 *   e⁻  electron   qm = −0.20   tight blue spirals
 *   e⁺  positron   qm = +0.20   tight red spirals  (opposite curl to e⁻)
 *   μ   muon       qm = −0.07   medium green arc
 *   π   pion       qm = +0.045  wide yellow arc
 *   p   proton     qm = +0.022  barely curves, cyan
 *
 * Trails are ring buffers drawn with age-faded characters:
 *   O head  * fresh  + medium  . fading
 *
 * Keys
 *   n  burst from centre     e  burst from edge
 *   b/B  field strength      Space  flip field direction
 *   t/T  cycle spawn type    r  reset   p  pause   q  quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/bubble_chamber.c \
 *       -o bubble_chamber -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 physics  §5 scene  §6 draw  §7 app
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

#define MAX_PARTICLES   20
#define TRAIL_LEN      300    /* ring-buffer length per particle            */
#define N_TYPES          5
#define HUD_ROWS         2

/* magnetic field */
#define B_INIT        1.0f    /* default field strength                     */
#define B_MIN         0.1f
#define B_MAX         4.0f
#define B_STEP        0.1f

/* particle motion */
#define V_SPAWN       2.2f    /* initial speed (physics units / step)       */
#define V_SPREAD      0.4f    /* ± random fraction of V_SPAWN               */
#define DRAG          0.003f  /* fractional speed lost per step (ionisation)*/
#define SPEED_DEAD    0.22f   /* particle "stops" below this speed          */

/* spawn */
#define BURST_MIN      2      /* particles per burst                        */
#define BURST_MAX      5

/* timing */
#define STEPS_PER_FRAME   4
#define RENDER_NS  (1000000000LL / 30)

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

/*
 * Each particle type has an effective q/m ratio (tuned for terminal scale)
 * and its own color pair.  Sign of qm determines curl direction:
 *   qm < 0  →  clockwise (with B > 0)
 *   qm > 0  →  counter-clockwise
 *
 * Cyclotron radius at default B=1, V_SPAWN=2.2:
 *   r = V_SPAWN / |qm * B| = 2.2 / |qm|
 *   electron: r ≈ 11 cols   muon: r ≈ 31   pion: r ≈ 49   proton: r ≈ 100
 */
typedef struct {
    const char *name;      /* display name                  */
    const char *symbol;    /* 2-char symbol shown in HUD    */
    float       qm;        /* effective charge/mass ratio   */
    short       c256;      /* 256-color foreground          */
    short       c8;        /* 8-color fallback              */
} PType;

static const PType k_types[N_TYPES] = {
    { "electron", "e-", -0.200f,  51, COLOR_BLUE   },
    { "positron", "e+", +0.200f, 196, COLOR_RED    },
    { "muon",     "mu", -0.070f,  46, COLOR_GREEN  },
    { "pion",     "pi", +0.045f, 226, COLOR_YELLOW },
    { "proton",   "p ", +0.022f, 159, COLOR_CYAN   },
};

/* color pairs: 1..N_TYPES for particle types, N_TYPES+1 for HUD */
#define CP_HUD  (N_TYPES + 1)

static bool g_256color;

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256color = (COLORS >= 256);
    for (int i = 0; i < N_TYPES; i++) {
        short fg = g_256color ? k_types[i].c256 : k_types[i].c8;
        init_pair(1 + i, fg, -1);
    }
    init_pair(CP_HUD, g_256color ? 244 : COLOR_WHITE, -1);
}

static inline int particle_cp(int kind) { return 1 + kind; }

/* ===================================================================== */
/* §4  physics                                                            */
/* ===================================================================== */

/*
 * Trail ring buffer: thead is the next write slot.
 * Reading newest-to-oldest: index = (thead - 1 - i + TRAIL_LEN) % TRAIL_LEN
 * for i in [0, tlen-1],  i=0 being the most recently recorded point.
 */
typedef struct {
    float x, y;              /* current position (col, row, floats)        */
    float vx, vy;            /* velocity                                   */
    int   kind;              /* index into k_types                         */
    bool  alive;
    float tx[TRAIL_LEN];     /* trail x ring buffer                        */
    float ty[TRAIL_LEN];     /* trail y ring buffer                        */
    int   thead;             /* next write index                           */
    int   tlen;              /* current number of recorded trail points    */
} Particle;

static Particle g_p[MAX_PARTICLES];
static float    g_B       = B_INIT;   /* field strength (may be negative)  */
static bool     g_paused  = false;
static int      g_spawn_kind = -1;    /* −1 = random                       */
static int      g_rows, g_cols;

/*
 * particle_step() — advance one physics step.
 *
 * Velocity rotation  R(omega):
 *   [ cos ω  -sin ω ] [ vx ]     ω = qm * B
 *   [ sin ω   cos ω ] [ vy ]
 *
 * This is an exact solution of the Lorentz ODE for constant B and produces
 * perfect circles without the energy drift of a simple Euler step.
 * Drag applied after rotation gives the inward spiral.
 */
static void particle_step(Particle *p)
{
    if (!p->alive) return;

    float omega = k_types[p->kind].qm * g_B;
    float ca = cosf(omega), sa = sinf(omega);

    /* exact velocity rotation */
    float nvx = p->vx * ca - p->vy * sa;
    float nvy = p->vx * sa + p->vy * ca;

    /* ionisation energy loss */
    p->vx = nvx * (1.f - DRAG);
    p->vy = nvy * (1.f - DRAG);

    /* advance position */
    p->x += p->vx;
    p->y += p->vy;

    /* record trail point */
    p->tx[p->thead] = p->x;
    p->ty[p->thead] = p->y;
    p->thead = (p->thead + 1) % TRAIL_LEN;
    if (p->tlen < TRAIL_LEN) p->tlen++;

    /* death: too slow */
    float spd2 = p->vx * p->vx + p->vy * p->vy;
    if (spd2 < SPEED_DEAD * SPEED_DEAD) p->alive = false;
}

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

/* Simple xorshift32 — srand() seeded at startup, rand() used throughout */

static void scene_reset(void)
{
    memset(g_p, 0, sizeof g_p);
}

/*
 * init_particle() — fill one particle slot.
 * cx, cy  : spawn centre (physics col/row coordinates)
 * angle   : initial velocity direction (radians)
 * kind    : particle type (−1 = random)
 */
static void init_particle(Particle *p, float cx, float cy,
                           float angle, int kind)
{
    memset(p, 0, sizeof *p);
    p->x    = cx;
    p->y    = cy;
    p->kind = (kind < 0) ? rand() % N_TYPES : kind;

    float speed = V_SPAWN * (1.f - V_SPREAD/2.f
                  + V_SPREAD * ((float)rand() / (float)RAND_MAX));
    p->vx   = cosf(angle) * speed;
    p->vy   = sinf(angle) * speed;
    p->alive  = true;
}

/*
 * find_dead_slot() — return index of first non-alive particle, or −1 if full.
 */
static int find_dead_slot(void)
{
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (!g_p[i].alive) return i;
    return -1;
}

/*
 * spawn_burst_centre() — n particles from screen centre, random directions.
 * Models a head-on collision vertex.
 */
static void spawn_burst_centre(int n)
{
    float cx = (float)g_cols * 0.5f;
    float cy = (float)(g_rows - HUD_ROWS) * 0.5f;
    int   k  = (g_spawn_kind < 0) ? -1 : g_spawn_kind;

    for (int i = 0; i < n; i++) {
        int slot = find_dead_slot();
        if (slot < 0) break;
        float angle = ((float)rand() / (float)RAND_MAX) * 2.f * 3.14159265f;
        init_particle(&g_p[slot], cx, cy, angle, k);
    }
}

/*
 * spawn_burst_edge() — n particles entering from a random screen edge,
 * velocity directed inward ± 30 deg.  Models beam entering the chamber.
 */
static void spawn_burst_edge(int n)
{
    int   edge  = rand() % 4;   /* 0=top 1=bottom 2=left 3=right */
    float cx, cy, base_angle;
    float W = (float)g_cols;
    float H = (float)(g_rows - HUD_ROWS);

    switch (edge) {
    case 0: cx = W * ((float)rand()/(float)RAND_MAX); cy = 0;   base_angle =  0.5f * 3.14159265f; break;
    case 1: cx = W * ((float)rand()/(float)RAND_MAX); cy = H-1; base_angle = -0.5f * 3.14159265f; break;
    case 2: cx = 0;   cy = H * ((float)rand()/(float)RAND_MAX); base_angle =  0.0f; break;
    default:cx = W-1; cy = H * ((float)rand()/(float)RAND_MAX); base_angle =  3.14159265f; break;
    }

    int k = (g_spawn_kind < 0) ? -1 : g_spawn_kind;
    for (int i = 0; i < n; i++) {
        int slot = find_dead_slot();
        if (slot < 0) break;
        float spread = (((float)rand()/(float)RAND_MAX) - 0.5f) * 1.047f; /* ±30 deg */
        init_particle(&g_p[slot], cx, cy, base_angle + spread, k);
    }
}

static void scene_step(void)
{
    for (int i = 0; i < MAX_PARTICLES; i++)
        particle_step(&g_p[i]);
}

static int alive_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (g_p[i].alive) n++;
    return n;
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

/*
 * draw_particle() — render trail ring buffer then the live particle head.
 *
 * Trail age fraction  age = i / tlen,  i=0 newest, i=tlen-1 oldest:
 *   age < 0.25   '*'  bold   (bright fresh track)
 *   age < 0.55   '+'  normal (ionisation trail)
 *   age < 0.80   '.'  normal (fading track)
 *   age ≥ 0.80   skip        (too old, fully faded)
 *
 * Live head drawn as 'O' bold, one cell ahead of the trail.
 */
static void draw_particle(const Particle *p, int draw_rows)
{
    /* nothing to draw if no trail and dead */
    if (!p->alive && p->tlen == 0) return;

    int cp = particle_cp(p->kind);

    /* trail: i=0 is most recent recorded point */
    for (int i = 0; i < p->tlen; i++) {
        float age = (float)i / (float)(p->tlen > 1 ? p->tlen : 1);
        if (age >= 0.80f) break;   /* all remaining points are older */

        int idx = (p->thead - 1 - i + TRAIL_LEN * 2) % TRAIL_LEN;
        int col = (int)(p->tx[idx] + 0.5f);
        int row = (int)(p->ty[idx] + 0.5f) + HUD_ROWS;
        if (col < 0 || col >= g_cols || row < HUD_ROWS || row >= draw_rows) continue;

        chtype ch;
        attr_t attr = (attr_t)COLOR_PAIR(cp);
        if      (age < 0.25f) { ch = '*'; attr |= A_BOLD; }
        else if (age < 0.55f) { ch = '+'; }
        else                  { ch = '.'; }

        attron(attr);
        mvaddch(row, col, ch);
        attroff(attr);
    }

    /* live head */
    if (p->alive) {
        int col = (int)(p->x + 0.5f);
        int row = (int)(p->y + 0.5f) + HUD_ROWS;
        if (col >= 0 && col < g_cols && row >= HUD_ROWS && row < draw_rows) {
            attron(COLOR_PAIR(cp) | A_BOLD);
            mvaddch(row, col, 'O');
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }
}

static void scene_draw(void)
{
    int draw_rows = g_rows;   /* full screen; HUD at top, tracks below */

    for (int i = 0; i < MAX_PARTICLES; i++)
        draw_particle(&g_p[i], draw_rows);

    /* ── HUD ── */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " BubbleChamber  q:quit  p:pause  r:reset"
        "  n:burst-centre  e:burst-edge  b/B:field  Space:flip  t/T:type");

    /* particle type legend */
    char type_buf[64] = {0};
    int  off = 0;
    for (int i = 0; i < N_TYPES; i++) {
        off += snprintf(type_buf + off, sizeof type_buf - (size_t)off,
                        "%s%s", i == 0 ? "" : " ",
                        k_types[i].symbol);
    }
    mvprintw(1, 0,
        " B=%.2f%s  alive=%d/%d  spawn=[%s]  %s",
        fabsf(g_B), g_B < 0 ? "(flipped)" : "",
        alive_count(), MAX_PARTICLES,
        g_spawn_kind < 0 ? "rand" : k_types[g_spawn_kind].symbol,
        g_paused ? "PAUSED" : "running");
    attroff(COLOR_PAIR(CP_HUD));

    /* type key in distinct colors */
    int hud_x = 52;   /* approximate position after "spawn=[" label */
    mvprintw(1, 0,
        " B=%.2f%s  alive=%d/%d  spawn=",
        fabsf(g_B), g_B < 0 ? "(flipped)" : "",
        alive_count(), MAX_PARTICLES);
    attroff(COLOR_PAIR(CP_HUD));

    /* color-coded spawn indicator */
    if (g_spawn_kind < 0) {
        attron(COLOR_PAIR(CP_HUD));
        addstr("[rand]");
        attroff(COLOR_PAIR(CP_HUD));
    } else {
        attron(COLOR_PAIR(particle_cp(g_spawn_kind)) | A_BOLD);
        printw("[%s]", k_types[g_spawn_kind].symbol);
        attroff(COLOR_PAIR(particle_cp(g_spawn_kind)) | A_BOLD);
    }

    attron(COLOR_PAIR(CP_HUD));
    printw("  %s", g_paused ? "PAUSED" : "running");
    attroff(COLOR_PAIR(CP_HUD));

    (void)hud_x;   /* suppress unused-variable warning */
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
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();

    getmaxyx(stdscr, g_rows, g_cols);
    scene_reset();
    /* initial event: mixed burst from centre */
    spawn_burst_centre(BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1));

    long long next_frame = clock_ns();

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

        case 'r': case 'R':
            scene_reset();
            spawn_burst_centre(BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1));
            break;

        case 'n': case 'N': {
            int n = BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1);
            spawn_burst_centre(n);
            break;
        }
        case 'e': case 'E': {
            int n = BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1);
            spawn_burst_edge(n);
            break;
        }

        case 'b':
            g_B += B_STEP; if (g_B > B_MAX) g_B = B_MAX; break;
        case 'B':
            g_B -= B_STEP; if (fabsf(g_B) < B_MIN) g_B = (g_B < 0) ? -B_MIN : B_MIN; break;

        case ' ':
            g_B = -g_B;   /* flip field direction — reverses all curls */
            break;

        case 't':
            /* cycle forward: rand → e- → e+ → mu → pi → p → rand */
            g_spawn_kind = (g_spawn_kind + 1 + 1) % (N_TYPES + 1) - 1;
            break;
        case 'T':
            g_spawn_kind = (g_spawn_kind + N_TYPES + 1) % (N_TYPES + 1) - 1;
            break;

        default: break;
        }

        long long now = clock_ns();
        if (!g_paused && now >= next_frame) {
            for (int s = 0; s < STEPS_PER_FRAME; s++)
                scene_step();
            next_frame = now + RENDER_NS;
        }

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(next_frame - clock_ns());
    }
    return 0;
}
