/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * genetic_rocket.c — genetic-algorithm rocket evolution
 *
 * DEMO: A target circle sits at the top of the screen; a launch pad sits
 *       at the bottom. A population of rockets is launched each
 *       generation, each rocket carrying a fixed genome of `LIFESPAN`
 *       force vectors that it applies one per simulation tick. Most of
 *       generation 0 fly chaotically — random forces produce random
 *       paths. Fitness is proportional to how close each rocket gets to
 *       the target (with a hit bonus and a crash penalty). The fittest
 *       rockets are picked as parents; their genomes are spliced and
 *       mutated to produce the next generation. After 30–80 generations
 *       most of the population reliably hits the target.
 *
 *       Press `f` to fast-forward (one full generation per frame); the
 *       hit count climbs quickly. Press `f` again for normal speed
 *       to watch a single generation fly. `r` reseeds back to gen 0.
 *
 * Study alongside: artistic/neural_net_vis.c (other Ai-folder vis),
 *                  flocking/flocking.c (similar agent + force model).
 *
 * Section map:
 *   §1 config    — POP_SIZE, LIFESPAN, mutation rate, target geometry
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — target / rocket / trail / best / HUD palette per theme
 *   §4 random    — frand, unit-vector helper
 *   §5 rocket    — Rocket struct, init / launch / tick / fitness
 *   §6 ga        — World struct, breed (crossover + mutation), evolve
 *   §7 scene     — draw_target + draw_rockets + scene_draw + HUD
 *   §8 screen    — ncurses init / cleanup
 *   §9 app       — signals, key handling, main loop
 *
 * Keys:  f      toggle fast-forward (1 gen/frame vs 1 tick/frame)
 *        s      toggle trails
 *        [/]    decrease/increase population (20..100, step 5)
 *        -/+    decrease/increase mutation rate (0.001..0.10)
 *        t      cycle theme   r reset gen 0   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra Ai/genetic_rocket.c \
 *       -o genetic_rocket -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Holland's classical genetic algorithm (1975), in the
 *                 "Smart Rockets" formulation popularised by Daniel
 *                 Shiffman ("Nature of Code" ch. 9). Each rocket carries
 *                 a fixed-length genome — an array of LIFESPAN 2-D force
 *                 vectors. At simulation tick `i` the rocket adds
 *                 `genes[i]` to its velocity. Fitness is
 *                     f = 1 / (distance_to_target + 1)
 *                 with a ×10 bonus for hits and a ×0.1 penalty for
 *                 crashes. Selection is fitness-proportional via a
 *                 mating pool (each rocket appears `k = 100·f/f_max`
 *                 times in the pool; uniform random pick). Crossover is
 *                 single-point: gene[0..mid) from parent A, gene[mid..)
 *                 from parent B. Per-gene mutation replaces a gene with
 *                 a fresh random vector with probability MUTATION_RATE.
 *
 * Data-structure: Population is a fixed array of Rocket; each Rocket
 *                 holds its genome inline (LIFESPAN × 8 bytes), its
 *                 phenotype state (position, velocity, age), status
 *                 flags (alive, hit_target, crashed) and a small cyclic
 *                 trail for visualisation. The World struct ties
 *                 everything together — population, generation counter,
 *                 target/launch positions, current tick within the
 *                 generation, and UI state.
 *
 * Rendering     : Per frame, in this draw order: target circle, launch
 *                 pad, trails (dim), rockets (bright, with the best of
 *                 the previous generation in a contrasting accent), HUD
 *                 with generation / hits / best fitness / mutation rate.
 *
 * Performance   : O(POP_SIZE · LIFESPAN) per generation for the sim; the
 *                 evolve step is O(POP_SIZE²) in the mating-pool build
 *                 but the constants are tiny — POP=50, pool ≤ 5000 is
 *                 microseconds. Memory: each rocket ≈ 1 KB (genome +
 *                 trail), so POP=100 fits in 100 KB.
 *
 * References    :
 *   Holland, "Adaptation in Natural and Artificial Systems" (1975).
 *   Shiffman, "The Nature of Code" ch. 9 — Smart Rockets formulation.
 *   Goldberg, "Genetic Algorithms in Search, Optimization, and Machine
 *     Learning" (1989) — fitness-proportional selection theory.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each rocket carries an INSTRUCTION TAPE — `LIFESPAN` force vectors,
 * applied one per simulation tick. The whole flight is genetically
 * determined. Rockets that end the flight near the target leave more
 * descendants; rockets that crash leave very few. Repeat for 50–80
 * generations and the population converges on tapes that hit the target.
 *
 * The "intelligence" is entirely in the genome — there is no real-time
 * steering, no goal-seeking heuristic, no neural net. Just random tapes,
 * fitness-weighted selection, and crossover-with-mutation. That's it.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine printing 50 random Etch-A-Sketch programs, running each on a
 * fresh rocket, scoring how close to the target each one ended up. Now
 * pick parents in proportion to their score, splice their programs at a
 * random midpoint, throw a few random bytes in for mutation, print 50
 * new programs. Run again. Score again. Splice again. After 50 rounds
 * the average program is a near-perfect target hitter, even though no
 * one ever told the program HOW to hit — it only ever knew whether it
 * succeeded.
 *
 * EVOLUTION ALGORITHM
 * ───────────────────
 *  1. Generation 0: every rocket gets a fresh random genome.
 *  2. Run all rockets for `LIFESPAN` ticks (or until all dead).
 *  3. Score: fitness = (1/(distance+1))² · hit_bonus · crash_penalty.
 *  4. Build a mating pool — each rocket gets `k = 100·f/f_max` slots.
 *  5. Breed `n_pop` children: pick two parents at random from the pool,
 *     splice genomes at a random midpoint, mutate each gene with
 *     probability `mutation_rate`.
 *  6. Replace population with children, relaunch all, increment gen.
 *
 * KEY FORMULAS
 * ────────────
 *  Fitness          : f = (1/(distance + 1))²
 *                     ×10  if hit_target
 *                     ×0.1 if crashed
 *
 *  Pool slot count  : k_i = floor(100 · f_i / f_max)    (min 1)
 *
 *  Crossover        : child.genes[i] = (i < mid) ? a.genes[i] : b.genes[i]
 *                     mid is a fresh random index per child.
 *
 *  Mutation         : if frand() < mutation_rate
 *                     replace genes[i] with a fresh random unit force
 *
 *  Rocket physics   : v += genes[age]
 *                     |v| capped at MAX_VEL
 *                     pos += v
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Premature convergence — mutation rate too low: the population
 *    locks onto one bad genome and never improves. Bump mutation with
 *    `+`. 0.01 is a good starting point; up to 0.05 if stuck.
 *
 *  • Pool overflow — at high `n_pop` the mating pool can hit
 *    `POOL_CAPACITY = pop · 100 = 5000` slots. Defensive cap in
 *    ga_evolve prevents UB; selection just gets slightly biased.
 *
 *  • All-crashers gen 0 — if every rocket crashes, max_f is tiny but
 *    still positive (the 0.1 penalty floor). Selection picks the
 *    "least bad" crashers; their genomes become the seed for gen 1.
 *
 *  • Zero-length genome — LIFESPAN = 0 would skip every gene-apply
 *    and rockets would never move. Keep LIFESPAN ≥ 30 for any visible
 *    flight at typical screen sizes.
 *
 *  • Resize during a generation — target/launch positions update via
 *    `world_position`, but rockets in flight keep their old positions
 *    and may crash on the new bounds. Acceptable: the next generation
 *    starts fresh with the new geometry.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Default config (pop=50, lifespan=100, mutation=0.01) should show:
 *    • gen 0:   chaos, hits ≈ 0–2 / 50
 *    • gen 10:  some convergence, hits ≈ 5–10
 *    • gen 30:  most arc toward target, hits ≈ 20–35
 *    • gen 80:  nearly all hit, hits ≈ 45–50
 *
 *  Press `f` to fast-forward (1 generation per frame). At 30 fps you
 *  should reach gen 80 in ~3 seconds and see the population nearly
 *  saturated with hitters.
 *
 *  If hits never grow past gen 0: mutation too low, or fitness function
 *  not pressuring enough. Try `+` to raise mutation.
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

#define TARGET_FPS          30

/* Population size — caps the static rocket array. */
#define POP_DEFAULT         50
#define POP_MIN             20
#define POP_MAX            100
#define POP_STEP             5

/* Genome length — how many ticks each rocket flies before evolution. */
#define LIFESPAN           100

/* Force per gene, velocity cap. Force is random unit vector × MAX_FORCE. */
#define MAX_FORCE         0.05f
#define MAX_VEL            0.6f

/* Mutation rate range — fraction of genes replaced with fresh random per
 * crossover. 0.01 = ~1 gene in 100 mutates per child. */
#define MUTATION_DEFAULT  0.01f
#define MUTATION_MIN     0.001f
#define MUTATION_MAX      0.10f
#define MUTATION_STEP    0.005f

/* Visual extras */
#define TRAIL_LEN            5
#define TARGET_RADIUS      2.5f      /* in cell-y units; horiz is 2× */
#define LAUNCH_OFFSET        2       /* cells above the bottom edge  */

/* Mating pool — each rocket gets up to this many slots. */
#define POOL_SLOTS_MAX     100
#define POOL_CAPACITY      (POP_MAX * POOL_SLOTS_MAX)

#define DT_CAP_S           0.10f
#define N_THEMES           4

/* Colour pair IDs */
#define PAIR_TARGET   1
#define PAIR_ROCKET   2
#define PAIR_TRAIL    3
#define PAIR_BEST     4
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

/* Per-theme [target, rocket, trail, best] for 256-color and 8-color. */
static const short THEME_FG_256[N_THEMES][4] = {
    { 196,  51,  39, 226 },   /* red target / cyan rocket / teal trail / gold best */
    { 220,  82,  34, 207 },   /* gold target / lime rocket / forest trail / pink best */
    { 207, 226, 178,  51 },   /* pink target / yellow rocket / honey trail / aqua best */
    {  51, 207, 134, 220 },   /* aqua target / pink rocket / orchid trail / gold best */
};
static const short THEME_FG_8[N_THEMES][4] = {
    { COLOR_RED,     COLOR_CYAN,    COLOR_BLUE,    COLOR_YELLOW  },
    { COLOR_YELLOW,  COLOR_GREEN,   COLOR_GREEN,   COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_YELLOW,  COLOR_YELLOW,  COLOR_CYAN    },
    { COLOR_CYAN,    COLOR_MAGENTA, COLOR_MAGENTA, COLOR_YELLOW  },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int   x256 = (COLORS >= 256);
    short tg   = x256 ? THEME_FG_256[theme][0] : THEME_FG_8[theme][0];
    short rk   = x256 ? THEME_FG_256[theme][1] : THEME_FG_8[theme][1];
    short tr   = x256 ? THEME_FG_256[theme][2] : THEME_FG_8[theme][2];
    short bs   = x256 ? THEME_FG_256[theme][3] : THEME_FG_8[theme][3];
    init_pair(PAIR_TARGET, tg, -1);
    init_pair(PAIR_ROCKET, rk, -1);
    init_pair(PAIR_TRAIL,  tr, -1);
    init_pair(PAIR_BEST,   bs, -1);
    init_pair(PAIR_HUD,    x256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   x256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  random                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/* Uniformly random unit vector via rejection sampling — simpler than the
 * polar form and avoids the angular bias of independent uniform [-1,1]
 * components. Used to seed each gene with a random direction. */
static void rand_unit(float *out_x, float *out_y)
{
    float x, y, l2;
    do {
        x = 2.0f * frand() - 1.0f;
        y = 2.0f * frand() - 1.0f;
        l2 = x * x + y * y;
    } while (l2 > 1.0f || l2 < 1e-6f);
    float l = sqrtf(l2);
    *out_x = x / l;
    *out_y = y / l;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  rocket                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { float fx, fy; } Gene;

typedef struct {
    Gene  genes[LIFESPAN];      /* immutable across one generation        */
    float px, py;               /* position (cell units, float)           */
    float vx, vy;               /* velocity                                */
    int   age;                  /* current tick within the generation     */
    int   alive;                /* still simulating?                       */
    int   hit_target;           /* fitness bonus flag                      */
    int   crashed;              /* fitness penalty flag                    */
    float fitness;              /* recomputed at end of generation         */

    /* Visual trail — most recent positions, newest first.                 */
    float tx[TRAIL_LEN];
    float ty[TRAIL_LEN];
    int   trail_count;
} Rocket;

/* Fresh genome — all genes set to random unit vector × MAX_FORCE. */
static void rocket_init_genome(Rocket *r)
{
    for (int i = 0; i < LIFESPAN; i++) {
        float ux, uy;
        rand_unit(&ux, &uy);
        r->genes[i].fx = ux * MAX_FORCE;
        r->genes[i].fy = uy * MAX_FORCE;
    }
}

/* Reset phenotype state to launch — keeps the genome unchanged. */
static void rocket_launch(Rocket *r, int launch_x, int launch_y)
{
    r->px = (float)launch_x;
    r->py = (float)launch_y;
    r->vx = 0.0f;
    r->vy = 0.0f;
    r->age = 0;
    r->alive = 1;
    r->hit_target = 0;
    r->crashed = 0;
    r->fitness = 0.0f;
    r->trail_count = 0;
}

/*
 * rocket_tick — one simulation step. Apply the gene at index `age`,
 * advance position, push the previous position onto the trail. Mark the
 * rocket dead on target-hit, off-screen, or end of genome.
 */
static void rocket_tick(Rocket *r, float tx, float ty, float trad,
                        int rows, int cols)
{
    if (!r->alive) return;
    if (r->age >= LIFESPAN) { r->alive = 0; return; }

    /* Push current position onto the trail (newest first). */
    if (r->trail_count < TRAIL_LEN) r->trail_count++;
    for (int i = TRAIL_LEN - 1; i > 0; i--) {
        r->tx[i] = r->tx[i - 1];
        r->ty[i] = r->ty[i - 1];
    }
    r->tx[0] = r->px;
    r->ty[0] = r->py;

    /* Apply gene; clamp velocity to MAX_VEL. */
    Gene *g = &r->genes[r->age];
    r->vx += g->fx;
    r->vy += g->fy;
    float v2 = r->vx * r->vx + r->vy * r->vy;
    if (v2 > MAX_VEL * MAX_VEL) {
        float f = MAX_VEL / sqrtf(v2);
        r->vx *= f;
        r->vy *= f;
    }
    r->px += r->vx;
    r->py += r->vy;
    r->age++;

    /* Target hit? Use cell-aspect-corrected distance: each cell is ~2×
     * taller than wide, so the "circle" is a 2:1 ellipse in cells. */
    float dx = (r->px - tx) * 0.5f;
    float dy = (r->py - ty);
    if (dx * dx + dy * dy <= trad * trad) {
        r->hit_target = 1;
        r->alive = 0;
        return;
    }

    /* Off-screen → crash. */
    if (r->px < 0 || r->px >= cols || r->py < 0 || r->py >= rows - 1) {
        r->crashed = 1;
        r->alive = 0;
    }
}

/*
 * rocket_fitness — proximity-based with bonuses and penalties.
 *
 *   f = 1 / (distance + 1)     in [0, 1]
 *   ×10 if hit_target          → strong selection toward hitters
 *   ×0.1 if crashed            → weak but non-zero so crashers still
 *                                  contribute occasional genes
 *
 * Squaring at the end amplifies selection pressure without going so
 * extreme that the population loses diversity.
 */
static float rocket_fitness(const Rocket *r, float tx, float ty)
{
    float dx = (r->px - tx) * 0.5f;
    float dy = (r->py - ty);
    float dist = sqrtf(dx * dx + dy * dy);
    float f = 1.0f / (dist + 1.0f);
    if (r->hit_target) f *= 10.0f;
    if (r->crashed)    f *= 0.10f;
    return f * f;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  ga                                                                  */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    Rocket  pop[POP_MAX];
    int     n;                  /* current population size                */
    int     generation;
    int     tick;               /* current tick within this generation    */
    int     n_hits;              /* hits accumulated this generation       */
    int     best_idx;            /* index of best rocket from PREVIOUS gen */
    float   best_fitness;        /* its fitness                            */

    float   mutation_rate;
    int     target_x, target_y;
    int     launch_x, launch_y;

    int     paused;
    int     fast;                /* 1 generation per frame                 */
    int     show_trails;
    int     theme;
} World;

/*
 * ga_breed — single-point crossover of parent A and parent B genomes,
 * with per-gene mutation. The split point is randomised per child so
 * different children inherit different combinations.
 */
static void ga_breed(Rocket *child, const Rocket *a, const Rocket *b,
                     float mut_rate)
{
    int mid = rand() % LIFESPAN;
    for (int i = 0; i < LIFESPAN; i++) {
        child->genes[i] = (i < mid) ? a->genes[i] : b->genes[i];
        if (frand() < mut_rate) {
            float ux, uy;
            rand_unit(&ux, &uy);
            child->genes[i].fx = ux * MAX_FORCE;
            child->genes[i].fy = uy * MAX_FORCE;
        }
    }
}

/*
 * ga_evolve — finish the current generation:
 *   1. Compute fitness for every rocket; track the best.
 *   2. Build a fitness-proportional mating pool (each rocket gets up to
 *      POOL_SLOTS_MAX entries scaled by f / f_max).
 *   3. Build the next generation in a temp array (so parents are still
 *      readable while we breed).
 *   4. Copy back, increment generation, reset rockets to launch state.
 *
 * The temp array is static to avoid stack pressure (POP_MAX × Rocket
 * is ~100 KB, fine but cleaner as data segment).
 */
static int  g_pool[POOL_CAPACITY];
static Rocket g_next[POP_MAX];

static void ga_evolve(World *w)
{
    /* 1. Fitness + best tracking. */
    float max_f = 0.0f;
    int   best  = 0;
    for (int i = 0; i < w->n; i++) {
        w->pop[i].fitness = rocket_fitness(&w->pop[i],
                                           (float)w->target_x,
                                           (float)w->target_y);
        if (w->pop[i].fitness > max_f) {
            max_f = w->pop[i].fitness;
            best  = i;
        }
    }
    w->best_fitness = max_f;
    w->best_idx     = best;

    /* 2. Mating pool — number of slots per rocket scaled by fitness. */
    int pool_size = 0;
    if (max_f <= 0.0f) max_f = 1.0f;   /* defensive */
    for (int i = 0; i < w->n; i++) {
        int k = (int)((w->pop[i].fitness / max_f) * POOL_SLOTS_MAX);
        if (k < 1) k = 1;
        for (int j = 0; j < k && pool_size < POOL_CAPACITY; j++)
            g_pool[pool_size++] = i;
    }

    /* 3. Breed the next generation. */
    for (int i = 0; i < w->n; i++) {
        int a = g_pool[rand() % pool_size];
        int b = g_pool[rand() % pool_size];
        ga_breed(&g_next[i], &w->pop[a], &w->pop[b], w->mutation_rate);
        rocket_launch(&g_next[i], w->launch_x, w->launch_y);
    }

    /* 4. Copy back, advance counters. */
    memcpy(w->pop, g_next, sizeof(Rocket) * (size_t)w->n);
    w->generation++;
    w->tick   = 0;
    w->n_hits = 0;
    w->best_idx = -1;   /* "best" highlight lasts only the gen it was set */
}

/* Generation-0 seed: brand-new random genomes for every rocket. */
static void world_reseed(World *w)
{
    for (int i = 0; i < w->n; i++) {
        rocket_init_genome(&w->pop[i]);
        rocket_launch(&w->pop[i], w->launch_x, w->launch_y);
    }
    w->generation   = 0;
    w->tick         = 0;
    w->n_hits       = 0;
    w->best_idx     = -1;
    w->best_fitness = 0.0f;
}

/*
 * world_step — one simulation tick: advance every alive rocket; on end
 * of LIFESPAN or all-dead, evolve.
 */
static void world_step(World *w, int rows, int cols)
{
    int any_alive = 0;
    for (int i = 0; i < w->n; i++) {
        if (!w->pop[i].alive) continue;
        rocket_tick(&w->pop[i],
                    (float)w->target_x, (float)w->target_y,
                    TARGET_RADIUS, rows, cols);
        if (w->pop[i].hit_target) w->n_hits++;
        if (w->pop[i].alive)      any_alive = 1;
    }
    w->tick++;
    if (w->tick >= LIFESPAN || !any_alive) {
        ga_evolve(w);
        /* After evolve, immediately mark best from PREVIOUS gen for
         * highlight on this frame. ga_evolve recorded best_idx. */
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Draw the target as a 2:1 cell-aspect ellipse so it reads as a circle. */
static void draw_target(int rows, int cols, int tx, int ty)
{
    attron(COLOR_PAIR(PAIR_TARGET) | A_BOLD);
    int ry = (int)(TARGET_RADIUS + 0.5f);
    int rx = ry * 2;
    for (int sr = ty - ry; sr <= ty + ry; sr++) {
        if (sr < 0 || sr >= rows - 1) continue;
        for (int sc = tx - rx; sc <= tx + rx; sc++) {
            if (sc < 0 || sc >= cols) continue;
            float dx = (sc - tx) * 0.5f;
            float dy = (sr - ty);
            if (dx * dx + dy * dy <= TARGET_RADIUS * TARGET_RADIUS)
                mvaddch(sr, sc, (chtype)'@');
        }
    }
    attroff(COLOR_PAIR(PAIR_TARGET) | A_BOLD);
}

static void draw_launch(int lx, int ly, int rows, int cols)
{
    if (ly < 0 || ly >= rows - 1) return;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    if (lx - 1 >= 0   ) mvaddch(ly, lx - 1, (chtype)'=');
    if (lx     <  cols) mvaddch(ly, lx,     (chtype)'^');
    if (lx + 1 <  cols) mvaddch(ly, lx + 1, (chtype)'=');
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_trails(const World *w, int rows, int cols)
{
    if (!w->show_trails) return;
    attron(COLOR_PAIR(PAIR_TRAIL));
    for (int i = 0; i < w->n; i++) {
        const Rocket *r = &w->pop[i];
        for (int t = 0; t < r->trail_count; t++) {
            int sr = (int)r->ty[t];
            int sc = (int)r->tx[t];
            if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;
            mvaddch(sr, sc, (chtype)'.');
        }
    }
    attroff(COLOR_PAIR(PAIR_TRAIL));
}

static void draw_rockets(const World *w, int rows, int cols)
{
    for (int i = 0; i < w->n; i++) {
        const Rocket *r = &w->pop[i];
        if (!r->alive && !r->hit_target) continue;
        int sr = (int)r->py;
        int sc = (int)r->px;
        if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;
        int is_best = (i == w->best_idx);
        int pair    = is_best ? PAIR_BEST : PAIR_ROCKET;
        char glyph  = is_best ? '@' : '^';
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sr, sc, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

static void scene_draw(int rows, int cols, const World *w, double fps)
{
    erase();
    draw_target(rows, cols, w->target_x, w->target_y);
    draw_launch(w->launch_x, w->launch_y, rows, cols);
    draw_trails(w, rows, cols);
    draw_rockets(w, rows, cols);

    /* HUD — top right */
    char buf[160];
    snprintf(buf, sizeof buf,
             " gen:%d  pop:%d  hits:%d/%d  best:%.2f  mut:%.3f  "
             "%5.1f fps  %s ",
             w->generation, w->n, w->n_hits, w->n,
             w->best_fitness, w->mutation_rate, fps,
             w->paused ? "PAUSED " :
             w->fast   ? "FAST   " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Key hints — bottom left */
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " [/]:pop  -/+:mut  f:fast  s:trails  t:theme  r:reset  "
             "p:pause  q:quit  [genetic_rocket] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  screen                                                              */
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

static void world_position(World *w, int rows, int cols)
{
    w->target_x = cols / 2;
    w->target_y = 3;
    w->launch_x = cols / 2;
    w->launch_y = rows - LAUNCH_OFFSET;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

static World g_world;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_world.n             = POP_DEFAULT;
    g_world.mutation_rate = MUTATION_DEFAULT;
    g_world.theme         = 0;
    g_world.show_trails   = 1;

    screen_init(g_world.theme);

    int rows = LINES, cols = COLS;
    world_position(&g_world, rows, cols);
    world_reseed(&g_world);

    const int64_t FRAME_NS    = 1000000000LL / TARGET_FPS;
    double  fps               = TARGET_FPS;
    int64_t t_fps_prev        = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            world_position(&g_world, rows, cols);
            /* Don't reseed on resize — only relocate target/launch. */
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           g_world.paused ^= 1; break;
                case 'f':           g_world.fast ^= 1;   break;
                case 's':           g_world.show_trails ^= 1; break;
                case 'r':           world_reseed(&g_world); break;
                case 't':           g_world.theme = (g_world.theme + 1) % N_THEMES;
                                    color_init(g_world.theme); break;
                case '+': case '=':
                    if (g_world.mutation_rate + MUTATION_STEP <= MUTATION_MAX)
                        g_world.mutation_rate += MUTATION_STEP;
                    break;
                case '-':
                    if (g_world.mutation_rate - MUTATION_STEP >= MUTATION_MIN)
                        g_world.mutation_rate -= MUTATION_STEP;
                    break;
                case '[':
                    if (g_world.n - POP_STEP >= POP_MIN) {
                        g_world.n -= POP_STEP;
                        world_reseed(&g_world);
                    } break;
                case ']':
                    if (g_world.n + POP_STEP <= POP_MAX) {
                        g_world.n += POP_STEP;
                        world_reseed(&g_world);
                    } break;
            }
        }

        /* Simulation: 1 tick/frame normal; up to LIFESPAN ticks/frame
         * in fast mode (so a full generation runs in one render). */
        if (!g_world.paused) {
            int steps = g_world.fast ? LIFESPAN : 1;
            for (int s = 0; s < steps; s++) {
                world_step(&g_world, rows, cols);
                if (g_world.tick == 0) break;   /* just evolved; redraw */
            }
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_world, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
