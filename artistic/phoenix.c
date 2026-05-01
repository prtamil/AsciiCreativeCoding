/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * phoenix.c — phoenix in flight with death and rebirth lifecycle
 *
 * DEMO: A bird-shaped formation of fire particles soars across the
 *       screen. The body is composed of fixed anchor points (head,
 *       neck, two wings, two trailing tail feathers); each anchor
 *       carries a cloud of jittery particles that paint the silhouette
 *       in flame. Wings beat via a slow sinusoid in time, raising and
 *       lowering the wing anchors so the bird flaps as it flies.
 *
 *       The phoenix follows a four-phase lifecycle:
 *         FLY   — cross the screen left to right (or right to left)
 *         DIE   — slow to a halt mid-screen, body collapses inward
 *         ASH   — bright burst, particles fall as cooling ash
 *         BIRTH — a new spark ignites at the horizon, body reforms
 *       and then back to FLY. One full cycle is ~30 seconds.
 *
 * Study alongside: artistic/fire_tornado.c (heat ramp + cylindrical pool),
 *                  artistic/volcano.c (ballistic ash particles for the
 *                  DIE/ASH phases).
 *
 * Section map:
 *   §1 config    — pool sizes, body anchors, lifecycle timings, palette
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — heat ramp per theme
 *   §4 random    — frand
 *   §5 anchor    — body template (anchor offsets relative to head)
 *   §6 particle  — Particle struct + tick / draw
 *   §7 phoenix   — Phoenix state + lifecycle FSM + emission
 *   §8 scene     — orchestrates draw order + HUD
 *   §9 screen    — ncurses init / cleanup
 *  §10 app       — signals, dt tracking, key handling, main loop
 *
 * Keys:  +/-   flight speed
 *        [/]   wing span scale
 *        ,/.   particle density
 *        s     skip to next phase
 *        t     cycle theme   r reset   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/phoenix.c \
 *       -o phoenix -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Body is a fixed array of anchor points in body-local
 *                 coordinates (head at origin, wings extending left and
 *                 right, tail trailing behind). Each frame:
 *                 (1) advance the head along its path; (2) compute each
 *                 anchor's world position by translating + applying the
 *                 wing-flap angle (a sinusoid in `world_time`);
 *                 (3) re-bind every particle to a random anchor and
 *                 jitter it within a small radius around that anchor's
 *                 world position. The bird is therefore drawn fresh
 *                 each frame from anchor + jitter; particles don't
 *                 carry memory of where they were.
 *
 *                 Lifecycle is a 4-state FSM driven by `phase_time`:
 *                 FLY → DIE → ASH → BIRTH → FLY. ASH releases the
 *                 anchor binding — particles continue in free flight
 *                 with gravity, cooling. BIRTH binds them back as the
 *                 silhouette reassembles.
 *
 * Data-structure: Phoenix holds Particle[N_PART_MAX] inline; each
 *                 particle has position, velocity, temperature, and an
 *                 anchor index used during the bound phases. A small
 *                 fixed-size BODY_ANCHORS array describes the silhouette.
 *
 * Rendering     : Per frame: erase, draw all particles with heat-ramp
 *                 char + colour. No back/front split — the bird is
 *                 small enough that simple draw order is fine.
 *
 * Performance   : O(N_PART) per frame. At N=300 it's a few hundred
 *                 mvaddch — trivial.
 *
 * References    :
 *   Reeves, "Particle Systems" SIGGRAPH (1983).
 *   Reynolds, "Steering Behaviors for Autonomous Characters" GDC (1999)
 *     — formation flight inspires the anchor-based body model here.
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

/* Particle pool. */
#define N_PART_MAX         500
#define N_PART_DEFAULT     280
#define N_PART_MIN          80
#define N_PART_STEP         40

/* Body geometry — anchors are placed relative to the head at (0,0).
 * x is forward axis (positive = forward), y is vertical (negative = up).
 * Coordinates are in cell units. */
#define BODY_HEAD_X         0
#define BODY_HEAD_Y         0
#define BODY_LEN            12.0f      /* head-to-tail length in cells   */
#define WING_SPAN_DEFAULT    12.0f
#define WING_SPAN_MIN         4.0f
#define WING_SPAN_MAX        22.0f
#define WING_SPAN_STEP        2.0f

#define WING_FLAP_AMP         3.0f     /* vertical wing tip travel        */
#define WING_FLAP_HZ          1.5f     /* flaps per second                 */
#define HEAD_BOB_AMP          0.6f     /* small head bob with wing beat   */

/* Path */
#define FLIGHT_SPEED_DEFAULT 14.0f     /* cells/sec                        */
#define FLIGHT_SPEED_MIN      4.0f
#define FLIGHT_SPEED_MAX     30.0f
#define FLIGHT_SPEED_STEP     2.0f
#define FLIGHT_AMP            5.0f     /* vertical sine while flying      */
#define FLIGHT_FREQ           0.4f     /* rad/sec                          */

#define ASPECT_X              2.0f     /* horizontal stretch on draw      */

/* Lifecycle timings (seconds). */
#define T_FLY                10.0f
#define T_DIE                 2.5f
#define T_ASH                 5.0f
#define T_BIRTH               2.5f

/* Particle physics. */
#define JITTER_R              0.7f     /* anchor jitter radius (cells)    */
#define COOL_RATE_FLY         0.0f     /* during FLY, no cooling           */
#define COOL_RATE_ASH         0.55f    /* during ASH, fast cool            */
#define GRAVITY_ASH          12.0f     /* cells/sec² during ASH            */

#define DT_CAP_S              0.10f
#define N_THEMES              4

/* Colour pair IDs */
#define PAIR_HEAT_0  1
#define PAIR_HEAT_1  2
#define PAIR_HEAT_2  3
#define PAIR_HEAT_3  4
#define PAIR_HEAT_4  5
#define PAIR_HUD     6
#define PAIR_HINT    7

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

/* Four distinct dominant hues so cycling 't' is unmistakable. The dim
 * end is bright enough to be visible (was 17/52 — near-black). */
static const short HEAT_256[N_THEMES][5] = {
    /* 0 fire    — red → orange → yellow → white   */
    { 124, 196, 208, 226, 231 },
    /* 1 blue    — navy → blue → cyan → white      */
    {  25,  33,  39,  51, 231 },
    /* 2 green   — dark green → lime → yellow → white */
    {  22,  28,  82, 154, 231 },
    /* 3 purple  — magenta → pink → white          */
    {  53,  91, 165, 213, 231 },
};
static const short HEAT_8[N_THEMES][5] = {
    { COLOR_RED,     COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < 5; i++) {
        short fg = x256 ? HEAT_256[theme][i] : HEAT_8[theme][i];
        init_pair((short)(PAIR_HEAT_0 + i), fg, -1);
    }
    init_pair(PAIR_HUD,  x256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT, x256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  random                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void) { return (float)rand() / (float)RAND_MAX; }
static float frand_signed(void) { return frand() * 2.0f - 1.0f; }

static int heat_bucket(float t)
{
    int b = (int)(t * 4.99f);
    if (b < 0) b = 0;
    if (b > 4) b = 4;
    return b;
}

static const char HEAT_GLYPH[5] = { '`', '.', '*', 'o', '#' };

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  anchor — body template                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Anchor type encodes role for animation:
 *   ANCHOR_BODY     — fixed offset from head (head, neck, body, tail)
 *   ANCHOR_WING_L   — left wing tip; flapping motion applied
 *   ANCHOR_WING_R   — right wing tip; flapping motion applied
 *   ANCHOR_TAIL     — trailing feather; sways with wing beat
 */
enum AnchorType {
    AT_BODY = 0,
    AT_WING_L,
    AT_WING_R,
    AT_TAIL,
};

typedef struct {
    float           dx, dy;     /* offset from head in body-local cells   */
    float           wing_frac;  /* 0..1 — fraction of wing span used by   *
                                 * this anchor (1.0 = wing tip)            */
    enum AnchorType type;
    int             weight;     /* particle-spawn weight                   */
} Anchor;

/* Body template — head at (0,0), forward = +x, up = -y. Coordinates
 * are in cells. Wing anchors carry wing_frac from base (~0.2) to tip
 * (1.0) so we paint the whole wing in flame, not just the tip. */
static const Anchor BODY_ANCHORS[] = {
    /* head + neck + body axis (forward of head) */
    { -1.0f,  0.0f, 0,      AT_BODY,    6 },     /* head                 */
    { -2.5f, -0.5f, 0,      AT_BODY,    5 },     /* neck                 */
    { -4.5f,  0.0f, 0,      AT_BODY,    8 },     /* breast               */
    { -7.0f,  0.5f, 0,      AT_BODY,    6 },     /* mid-body             */
    { -9.5f,  0.5f, 0,      AT_BODY,    4 },     /* lower body           */

    /* tail feathers — trail behind, sway with wing beat */
    {-12.0f,  0.0f, 0,      AT_TAIL,    4 },
    {-13.5f, -1.5f, 0,      AT_TAIL,    3 },     /* upper tail feather   */
    {-13.5f,  1.5f, 0,      AT_TAIL,    3 },     /* lower tail feather   */
    {-15.0f,  0.0f, 0,      AT_TAIL,    2 },     /* tail tip             */

    /* left wing — multiple anchors from shoulder to tip */
    { -3.5f, -1.0f, 0.20f,  AT_WING_L,  4 },
    { -4.5f, -2.5f, 0.45f,  AT_WING_L,  4 },
    { -5.5f, -4.0f, 0.70f,  AT_WING_L,  3 },
    { -7.0f, -5.0f, 1.00f,  AT_WING_L,  3 },     /* tip                  */
    { -3.0f, -2.0f, 0.30f,  AT_WING_L,  3 },     /* leading edge         */
    { -5.0f, -3.0f, 0.60f,  AT_WING_L,  3 },

    /* right wing — mirror of left */
    { -3.5f,  1.0f, 0.20f,  AT_WING_R,  4 },
    { -4.5f,  2.5f, 0.45f,  AT_WING_R,  4 },
    { -5.5f,  4.0f, 0.70f,  AT_WING_R,  3 },
    { -7.0f,  5.0f, 1.00f,  AT_WING_R,  3 },
    { -3.0f,  2.0f, 0.30f,  AT_WING_R,  3 },
    { -5.0f,  3.0f, 0.60f,  AT_WING_R,  3 },
};
#define N_ANCHORS (int)(sizeof(BODY_ANCHORS) / sizeof(BODY_ANCHORS[0]))

/* Compute total weight for weighted-random anchor selection. */
static int anchor_total_weight(void)
{
    int s = 0;
    for (int i = 0; i < N_ANCHORS; i++) s += BODY_ANCHORS[i].weight;
    return s;
}

/* Pick a random anchor index by weight. */
static int anchor_pick(int total)
{
    int r = rand() % total;
    for (int i = 0; i < N_ANCHORS; i++) {
        r -= BODY_ANCHORS[i].weight;
        if (r < 0) return i;
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  particle                                                            */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    float x, y;
    float vx, vy;
    float temp;
    int   anchor_idx;     /* during bound phases: which anchor we follow   */
    int   alive;
} Particle;

static void particle_draw(const Particle *p, int rows, int cols)
{
    if (!p->alive) return;
    int sr = (int)p->y, sc = (int)p->x;
    if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) return;
    int bucket = heat_bucket(p->temp);
    attron(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
    mvaddch(sr, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
    attroff(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  phoenix                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

enum Phase { PH_FLY, PH_DIE, PH_ASH, PH_BIRTH };

typedef struct {
    Particle parts[N_PART_MAX];
    int      n;

    /* Head trajectory. */
    float head_x;
    float head_y;
    float dir;             /* +1 right, -1 left                          */

    /* Knobs. */
    float flight_speed;
    float wing_span;

    /* Lifecycle FSM. */
    enum Phase phase;
    float      phase_time;
    float      world_time;

    int   theme;
    int   paused;
} Phoenix;

/*
 * anchor_world — given anchor index, head position, dir, wing_span and
 * world time, compute the anchor's world position. Wing anchors have
 * an additional flap term scaled by wing_frac and dir (the bird mirrors
 * y-flip when flying right-to-left).
 */
static void anchor_world(const Phoenix *ph, int idx, float *out_x, float *out_y)
{
    const Anchor *a = &BODY_ANCHORS[idx];
    float dx = a->dx;
    float dy = a->dy;

    /* Wing span scaling — multiply the y component for wing anchors. */
    if (a->type == AT_WING_L || a->type == AT_WING_R)
        dy *= ph->wing_span / WING_SPAN_DEFAULT;

    /* Wing flap. Sinusoid in time, scaled by wing_frac. Left/right
     * wings flap in phase (both up, both down), as real birds do. */
    if (a->type == AT_WING_L || a->type == AT_WING_R) {
        float flap = sinf(ph->world_time * 2.0f * (float)M_PI * WING_FLAP_HZ);
        dy += flap * WING_FLAP_AMP * a->wing_frac
            * (a->type == AT_WING_L ? -1.0f : 1.0f);
    }

    /* Head bob with wing beat. */
    float bob = sinf(ph->world_time * 2.0f * (float)M_PI * WING_FLAP_HZ);
    dy += bob * HEAD_BOB_AMP * 0.3f;

    /* Mirror x when flying left (head → tail goes right instead). */
    *out_x = ph->head_x + dx * (-ph->dir) * ASPECT_X;
    *out_y = ph->head_y + dy;
}

/*
 * particle_bind — set particle position to a small jitter around the
 * given anchor's world position. Used during FLY and BIRTH (partial).
 *
 * Temperature varies by body part so the heat ramp is visible across
 * the bird, not uniformly white:
 *   wing tips    → bucket 4  (white-hot)
 *   wing base    → bucket 2  (mid heat)
 *   body         → bucket 2-3
 *   tail         → bucket 1-2 (red/orange)
 * This is what makes 't' theme cycling visible — different heat
 * buckets show theme colours.
 */
static void particle_bind(Particle *p, const Phoenix *ph)
{
    float ax, ay;
    anchor_world(ph, p->anchor_idx, &ax, &ay);
    p->x  = ax + frand_signed() * JITTER_R * ASPECT_X;
    p->y  = ay + frand_signed() * JITTER_R;
    p->vx = 0;
    p->vy = 0;

    enum AnchorType type = BODY_ANCHORS[p->anchor_idx].type;
    float base_t;
    if (type == AT_WING_L || type == AT_WING_R) {
        /* Wing tips white, roots cooler — wing_frac=1.0 → 0.95,
         * wing_frac=0.20 → 0.51. */
        base_t = 0.45f + 0.50f * BODY_ANCHORS[p->anchor_idx].wing_frac;
    } else if (type == AT_BODY) {
        base_t = 0.55f;
    } else { /* AT_TAIL */
        base_t = 0.35f;
    }
    p->temp = base_t + 0.10f * (frand() - 0.5f);
    if (p->temp < 0.10f) p->temp = 0.10f;
    if (p->temp > 1.00f) p->temp = 1.00f;
    p->alive = 1;
}

static void phoenix_reset(Phoenix *ph, int rows, int cols)
{
    (void)cols;
    int total_w = anchor_total_weight();
    for (int i = 0; i < ph->n; i++) {
        ph->parts[i].anchor_idx = anchor_pick(total_w);
        ph->parts[i].alive = 1;
    }
    ph->head_x = -10.0f;                          /* start off-screen left */
    ph->head_y = (float)rows / 3.0f;
    ph->dir    = 1.0f;
    ph->phase  = PH_FLY;
    ph->phase_time = 0.0f;
    ph->world_time = 0.0f;
    /* Seed positions immediately so frame 0 isn't a black flash. */
    for (int i = 0; i < ph->n; i++) particle_bind(&ph->parts[i], ph);
}

/*
 * phoenix_tick — drive particle motion based on the current phase.
 *
 *   FLY   — head advances along dir; particles continuously rebind to
 *           anchors (bird is drawn fresh each frame).
 *   DIE   — head decelerates; particles stay bound but jitter shrinks.
 *   ASH   — particles released; gravity + cooling. Head fixed.
 *   BIRTH — head moves to start position; particles fade in by gradually
 *           rebinding to anchors with rising temp.
 */
static void phoenix_tick(Phoenix *ph, float dt, int rows, int cols)
{
    ph->world_time += dt;
    ph->phase_time += dt;

    int total_w = anchor_total_weight();

    switch (ph->phase) {
    case PH_FLY: {
        /* Head moves; vertical sinusoid for soaring feel. */
        ph->head_x += ph->flight_speed * ph->dir * dt;
        ph->head_y  = (float)rows / 3.0f
                    + sinf(ph->world_time * FLIGHT_FREQ) * FLIGHT_AMP;
        /* Rebind every particle every frame. */
        for (int i = 0; i < ph->n; i++) {
            if (frand() < 0.4f)  /* refresh anchor occasionally */
                ph->parts[i].anchor_idx = anchor_pick(total_w);
            particle_bind(&ph->parts[i], ph);
        }
        if (ph->phase_time >= T_FLY) {
            ph->phase = PH_DIE;
            ph->phase_time = 0.0f;
        }
        break;
    }
    case PH_DIE: {
        /* Slow to a halt at screen centre. */
        float slow = 1.0f - (ph->phase_time / T_DIE);
        if (slow < 0) slow = 0;
        ph->head_x += ph->flight_speed * ph->dir * dt * slow;
        ph->head_y  = (float)rows / 3.0f
                    + sinf(ph->world_time * FLIGHT_FREQ) * FLIGHT_AMP * slow;
        /* Particles still bound, jitter shrinking, intensity rising. */
        for (int i = 0; i < ph->n; i++) {
            float ax, ay;
            anchor_world(ph, ph->parts[i].anchor_idx, &ax, &ay);
            float r = JITTER_R * (0.5f + 0.5f * slow);
            ph->parts[i].x = ax + frand_signed() * r * ASPECT_X;
            ph->parts[i].y = ay + frand_signed() * r;
            ph->parts[i].temp = 0.95f + 0.05f * frand();
            ph->parts[i].alive = 1;
        }
        if (ph->phase_time >= T_DIE) {
            /* Burst — release particles with outward radial velocity. */
            for (int i = 0; i < ph->n; i++) {
                Particle *p = &ph->parts[i];
                float dx = p->x - ph->head_x;
                float dy = p->y - ph->head_y;
                float l  = sqrtf(dx * dx + dy * dy);
                if (l < 1e-3f) { dx = frand_signed(); dy = frand_signed(); l = 1.0f; }
                float speed = 6.0f + frand() * 14.0f;
                p->vx = dx / l * speed * ASPECT_X;
                p->vy = dy / l * speed * 0.7f;
                p->temp = 1.0f;
            }
            ph->phase = PH_ASH;
            ph->phase_time = 0.0f;
        }
        break;
    }
    case PH_ASH: {
        /* Free flight + gravity + cool. */
        for (int i = 0; i < ph->n; i++) {
            Particle *p = &ph->parts[i];
            p->vy += GRAVITY_ASH * dt;
            p->x  += p->vx * dt;
            p->y  += p->vy * dt;
            p->temp -= COOL_RATE_ASH * dt;
            if (p->temp <= 0 || p->y >= rows - 1) p->alive = 0;
        }
        if (ph->phase_time >= T_ASH) {
            /* Reset for BIRTH at the opposite edge. */
            ph->dir    = -ph->dir;
            ph->head_x = (ph->dir > 0) ? -8.0f : (float)(cols + 8);
            ph->head_y = (float)rows / 3.0f;
            for (int i = 0; i < ph->n; i++) {
                ph->parts[i].anchor_idx = anchor_pick(total_w);
                ph->parts[i].alive = 0;
                ph->parts[i].temp  = 0.0f;
            }
            ph->phase = PH_BIRTH;
            ph->phase_time = 0.0f;
        }
        break;
    }
    case PH_BIRTH: {
        /* Particles fade in over time; bind incrementally. */
        float t = ph->phase_time / T_BIRTH;
        if (t > 1.0f) t = 1.0f;
        int n_active = (int)(t * ph->n);
        for (int i = 0; i < ph->n; i++) {
            if (i < n_active) {
                if (!ph->parts[i].alive) {
                    ph->parts[i].anchor_idx = anchor_pick(total_w);
                    particle_bind(&ph->parts[i], ph);
                } else {
                    particle_bind(&ph->parts[i], ph);
                }
            } else {
                ph->parts[i].alive = 0;
            }
        }
        if (ph->phase_time >= T_BIRTH) {
            ph->phase = PH_FLY;
            ph->phase_time = 0.0f;
        }
        break;
    }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const char *phase_name(enum Phase p)
{
    switch (p) {
        case PH_FLY:   return "FLY  ";
        case PH_DIE:   return "DIE  ";
        case PH_ASH:   return "ASH  ";
        case PH_BIRTH: return "BIRTH";
    }
    return "?";
}

static void scene_draw(int rows, int cols, const Phoenix *ph, double fps)
{
    erase();
    for (int i = 0; i < ph->n; i++)
        particle_draw(&ph->parts[i], rows, cols);

    /* HUD */
    char buf[160];
    snprintf(buf, sizeof buf,
             " phase:%s  parts:%d  speed:%.0f  span:%.0f  theme:%d  "
             "%5.1f fps  %s ",
             phase_name(ph->phase), ph->n, ph->flight_speed,
             ph->wing_span, ph->theme, fps,
             ph->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " +/-:speed  [/]:span  ,/.:density  s:skip  t:theme  "
             "r:reset  p:pause  q:quit  [phoenix] ");
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

static Phoenix g_ph;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_ph.n            = N_PART_DEFAULT;
    g_ph.flight_speed = FLIGHT_SPEED_DEFAULT;
    g_ph.wing_span    = WING_SPAN_DEFAULT;
    g_ph.theme        = 0;

    screen_init(g_ph.theme);
    int rows = LINES, cols = COLS;
    phoenix_reset(&g_ph, rows, cols);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g_ph.paused ^= 1; break;
                case 'r':          phoenix_reset(&g_ph, rows, cols); break;
                case 't':          g_ph.theme = (g_ph.theme + 1) % N_THEMES;
                                   color_init(g_ph.theme); break;
                case 's':
                    /* Skip to next phase. */
                    g_ph.phase_time = 1e6f;
                    break;
                case '+': case '=':
                    if (g_ph.flight_speed + FLIGHT_SPEED_STEP <= FLIGHT_SPEED_MAX)
                        g_ph.flight_speed += FLIGHT_SPEED_STEP;
                    break;
                case '-':
                    if (g_ph.flight_speed - FLIGHT_SPEED_STEP >= FLIGHT_SPEED_MIN)
                        g_ph.flight_speed -= FLIGHT_SPEED_STEP;
                    break;
                case '[':
                    if (g_ph.wing_span - WING_SPAN_STEP >= WING_SPAN_MIN)
                        g_ph.wing_span -= WING_SPAN_STEP;
                    break;
                case ']':
                    if (g_ph.wing_span + WING_SPAN_STEP <= WING_SPAN_MAX)
                        g_ph.wing_span += WING_SPAN_STEP;
                    break;
                case ',':
                    if (g_ph.n - N_PART_STEP >= N_PART_MIN)
                        g_ph.n -= N_PART_STEP;
                    break;
                case '.':
                    if (g_ph.n + N_PART_STEP <= N_PART_MAX) {
                        g_ph.n += N_PART_STEP;
                        int total_w = anchor_total_weight();
                        for (int i = 0; i < g_ph.n; i++) {
                            g_ph.parts[i].anchor_idx = anchor_pick(total_w);
                            particle_bind(&g_ph.parts[i], &g_ph);
                        }
                    }
                    break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_ph.paused) phoenix_tick(&g_ph, dt, rows, cols);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_ph, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
