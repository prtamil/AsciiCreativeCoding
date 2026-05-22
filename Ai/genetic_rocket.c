/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * genetic_rocket.c — genetic-algorithm rocket evolution
 *
 * DEMO: A target sits at the top of the screen and a launch pad at the
 *       bottom. Each rocket carries a fixed-length genome of force
 *       vectors applied one per tick — early generations fly chaotically,
 *       but fitness-proportional breeding with mutation reliably hits
 *       the target after 30-80 generations. Press `f` to fast-forward
 *       a full generation per frame.
 *
 * Study alongside: flocking/flocking.c (similar agent + force model)
 *
 * Section map:
 *   §1 config    — population, lifespan, mutation, target geometry
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — themed palettes + spec HUD/hint pairs
 *   §4 random    — frand + uniform unit-vector  (cell-space, no §4 coords)
 *   §5 rocket    — Genome / Phenotype / Trail composed into Rocket;
 *                   genome init, launch, tick, fitness
 *   §6 ga        — Population / Generation / GAParams / WorldGeom /
 *                   WorldUI composed into Scene; ga_breed, ga_evolve,
 *                   scene setup + step
 *   §7 scene     — target / launch / trails / rockets + HUD + hint
 *   §8 screen    — ncurses init / cleanup
 *   §9 app       — FrameTimer + Scene globals; signals, keys, main loop
 *
 * Keys:  f      toggle fast-forward (1 gen/frame vs 1 tick/frame)
 *        s      toggle trails
 *        [/]    decrease/increase population (20..100, step 5)
 *        -/+    decrease/increase mutation rate (0.001..0.10)
 *        t      cycle theme
 *        r      reset to gen 0
 *        p      pause
 *        q/ESC  quit
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
 *                 genes[i] to its velocity. Fitness is
 *                     f = 1 / (distance_to_target + 1)
 *                 with a x10 bonus for hits and a x0.1 penalty for
 *                 crashes. Selection is fitness-proportional via a
 *                 mating pool (each rocket appears k = 100*f/f_max
 *                 times; uniform random pick). Crossover is single-
 *                 point: gene[0..mid) from parent A, gene[mid..) from
 *                 parent B. Per-gene mutation replaces a gene with a
 *                 fresh random unit force with probability mutation_rate.
 *
 * Data-structure: A Scene owns the whole program state. Each Rocket
 *                 is composed of three concept-sized pieces:
 *
 *                   Genome    — immutable instruction tape (LIFESPAN Genes)
 *                   Phenotype — physical state DURING one generation
 *                                 (px/py, vx/vy, age, alive/hit/crashed,
 *                                  fitness)
 *                   Trail     — visualisation-only ring buffer
 *
 *                 The naming maps to the GA vocabulary: the genotype is
 *                 fixed across a generation, the phenotype expresses it
 *                 through physics, the trail is just for the viewer.
 *
 *                 Scene composes five concept-sized sub-structs plus
 *                 Screen + FrameTimer:
 *
 *                   Population  — rockets[POP_MAX] + n
 *                   Generation  — number, tick, n_hits, best_fitness
 *                   GAParams    — mutation_rate
 *                   WorldGeom   — target_x/y, launch_x/y
 *                   WorldUI     — paused, fast, show_trails, theme
 *                   Screen      — cols, rows
 *                   FrameTimer  — frame budget + EWMA fps
 *
 *                 Plus the volatile sig_atomic_t flags (running,
 *                 need_resize) for signal handlers. No heap allocation
 *                 post-init.
 *
 * Rendering     : Per frame, in this draw order: target circle, launch
 *                 pad, trails (dim), rockets (bright). The rocket
 *                 currently closest to the target is recoloured as the
 *                 in-flight leader so the eye has something to track.
 *
 * Performance   : O(POP_SIZE * LIFESPAN) per generation for the sim; the
 *                 evolve step is O(POP_SIZE^2) in mating-pool build but
 *                 the constants are tiny — POP=50, pool <= 5000 is
 *                 microseconds. Memory: each rocket ~1 KB (genome +
 *                 trail), so POP=100 fits in ~100 KB.
 *
 * References    : grouped by the concepts this file teaches. Eight
 *                  entries — one or two strong picks per topic.
 *
 *   ── Genetic algorithms — theory & textbooks (Algorithm, §6) ──────
 *   Holland, "Adaptation in Natural and Artificial Systems" (MIT
 *     Press, reprint 1992; original Univ. Michigan, 1975). The
 *     originating work on genetic algorithms — schema theorem,
 *     building-block hypothesis, and the selection / crossover /
 *     mutation triad this file uses.
 *   Goldberg, "Genetic Algorithms in Search, Optimization, and
 *     Machine Learning" (Addison-Wesley, 1989). The canonical GA
 *     textbook. Fitness-proportional selection theory; explains
 *     why squaring fitness (as rocket_fitness() does) sharpens
 *     selection pressure and speeds convergence.
 *   Mitchell, "An Introduction to Genetic Algorithms" (MIT Press,
 *     1996). Modern, clear introduction — pair with Goldberg for
 *     a theory vs. practice perspective.
 *   Eiben & Smith, "Introduction to Evolutionary Computing"
 *     (Springer, 2nd ed., 2015). Broader treatment placing GA
 *     among related evolutionary methods (ES, EP, GP, DE).
 *
 *   ── Smart Rockets — the formulation used here (§5, §6) ───────────
 *   Shiffman, "The Nature of Code" ch. 9 (free online at
 *     natureofcode.com). The exact "Smart Rockets" formulation
 *     this file follows — instruction-tape genome, fitness
 *     = 1/(distance+1), mating-pool selection, single-point
 *     crossover with per-gene mutation.
 *
 *   ── Numerical method (§4 random) ─────────────────────────────────
 *   Marsaglia, "Choosing a point from the surface of a sphere"
 *     (Annals of Mathematical Statistics 43(2):645–646, 1972).
 *     The rejection-sampling unit-vector method rand_unit() uses;
 *     avoids the diagonal bias that uniform-component normalisation
 *     would otherwise introduce.
 *
 *   ── Rendering (§3, §7) ───────────────────────────────────────────
 *   Padala, "NCURSES Programming HOWTO" (The Linux Documentation
 *     Project). Practical reference for the ncurses API used in
 *     §3 (color_init) and §7 (mvaddch, attron, A_BOLD).
 *     https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *
 *   ── Game loop & numerical timing (§9 main loop) ──────────────────
 *   Fiedler ("Gaffer on Games"), "Fix Your Timestep!" (2004). The
 *     classic write-up of the dt-cap + EWMA-fps pattern in main();
 *     explains why a smoothed fps reading and capped frame budget
 *     keep the GA stable at any frame rate.
 *     https://gafferongames.com/post/fix_your_timestep/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each rocket carries an INSTRUCTION TAPE — LIFESPAN force vectors,
 * applied one per simulation tick. The whole flight is genetically
 * determined. Rockets that end the flight near the target leave more
 * descendants; rockets that crash leave very few. Repeat for 50-80
 * generations and the population converges on tapes that hit the target.
 *
 * The "intelligence" is entirely in the genome — no real-time steering,
 * no goal-seeking heuristic, no neural net. Just random tapes, fitness-
 * weighted selection, and crossover-with-mutation.
 *
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Generation 0: every rocket gets a fresh random genome.
 *  2. Run all rockets for LIFESPAN ticks (or until all dead).
 *  3. Score: fitness = (1/(distance+1))^2 * hit_bonus * crash_penalty.
 *  4. Build a mating pool — each rocket gets k = 100*f/f_max slots.
 *  5. Breed n_pop children: pick two parents at random from the pool,
 *     splice genomes at a random midpoint, mutate each gene with
 *     probability mutation_rate.
 *  6. Replace population with children, relaunch all, increment gen.
 *
 * KEY FORMULAS
 * ────────────
 *  Fitness          : f = (1/(distance + 1))^2
 *                     x10  if hit_target
 *                     x0.1 if crashed
 *
 *  Pool slot count  : k_i = floor(100 * f_i / f_max)    (min 1)
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
 *
 * Background you need
 * ───────────────────
 *   - Newton's laws (constant force = constant acceleration).
 *   - Random number generation (rand()/RAND_MAX → uniform [0,1]).
 *   - Cell-space simulation (no §4 coords; positions ARE cells).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Neural networks. Pure GA, no learning function.
 *   - Reinforcement learning (TD, Q-learning, policy gradient).
 *     GAs are POPULATION-based search, not gradient descent.
 *   - Differential evolution / genetic programming. Both are
 *     successors of GA but use different gene representations.
 *   - Calculus / gradients. The fitness function need not be
 *     differentiable — that's GA's selling point.
 *
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

/* Force per gene, velocity cap. Force is random unit vector x MAX_FORCE. */
#define MAX_FORCE         0.05f
#define MAX_VEL            0.6f

/* Mutation rate range — fraction of genes replaced with fresh random per
 * crossover. 0.01 ~ 1 gene in 100 mutates per child. */
#define MUTATION_DEFAULT  0.01f
#define MUTATION_MIN     0.001f
#define MUTATION_MAX      0.10f
#define MUTATION_STEP    0.005f

/* Visual */
#define TRAIL_LEN            5
#define TARGET_RADIUS      2.5f      /* in cell-y units; horiz is 2x */
#define LAUNCH_OFFSET        2       /* cells above the bottom edge  */

/* Mating pool — each rocket gets up to this many slots. */
#define POOL_SLOTS_MAX     100
#define POOL_CAPACITY      (POP_MAX * POOL_SLOTS_MAX)

#define N_THEMES           4

/* ── §1.1 fitness shaping ──────────────────────────────────────── */

/* Fitness = ( 1 / (distance + FITNESS_DIST_OFFSET) ) ²
 *           × FITNESS_HIT_BONUS    if hit_target
 *           × FITNESS_CRASH_PENALTY if crashed
 *
 * The squaring (in rocket_fitness) sharpens selection pressure; the
 * +1 offset keeps the inverse from blowing up at distance 0. */
#define FITNESS_DIST_OFFSET   1.0f
#define FITNESS_HIT_BONUS    10.0f      /* multiplier — favours hits     */
#define FITNESS_CRASH_PENALTY 0.10f     /* multiplier — punishes crashes */

/* ── §1.2 cell aspect ratio ───────────────────────────────────── */

/* Terminal cells are ~2× taller than wide. Multiplying x distances by
 * 0.5 in fitness + hit tests undoes the aspect ratio so a "circle"
 * drawn as a 2:1 ellipse acts circular for hit detection. */
#define CELL_ASPECT_X         0.5f

/* ── §1.3 main-loop pacing ─────────────────────────────────────── */

#define NS_PER_SEC      1000000000LL

/* EWMA fps: fps = fps · RETAIN + instantaneous · NEW. ~20-frame
 * effective window (1 / (1 - 0.95)). Stable enough for the HUD
 * digits not to jitter. */
#define EWMA_RETAIN     0.95
#define EWMA_NEW        0.05

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

/* Per-theme [target, rocket, trail, best] in 256-color and 8-color. */
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

/* PAIR_HUD/PAIR_HINT are fixed across themes per CLAUDE.md HUD spec —
 * bright yellow status + bright cyan hint, both on default background.
 * Choosing these out-of-theme keeps the HUD readable against ANY
 * animation behind it. */
static void color_init(int theme)
{
    start_color(); use_default_colors();
    int   x256 = (COLORS >= 256);
    short tg = x256 ? THEME_FG_256[theme][0] : THEME_FG_8[theme][0];
    short rk = x256 ? THEME_FG_256[theme][1] : THEME_FG_8[theme][1];
    short tr = x256 ? THEME_FG_256[theme][2] : THEME_FG_8[theme][2];
    short bs = x256 ? THEME_FG_256[theme][3] : THEME_FG_8[theme][3];
    init_pair(PAIR_TARGET, tg, -1);
    init_pair(PAIR_ROCKET, rk, -1);
    init_pair(PAIR_TRAIL,  tr, -1);
    init_pair(PAIR_BEST,   bs, -1);
    init_pair(PAIR_HUD,    x256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   x256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  random                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/*
 * rand_unit — uniformly random unit vector via rejection sampling.
 * Independent uniform [-1,1] components would over-sample the diagonals
 * (a (1,1) sample has length sqrt(2), so renormalising biases toward
 * 45-degree angles). Rejecting samples outside the unit disc — and the
 * near-origin ones that would amplify floating-point noise — gives
 * a true uniform-angle distribution.
 */
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

/*
 * §5.1 Gene — the GA's atomic unit. One 2-D force vector.
 *
 * INTENT
 *   A Gene is what crossover splices and mutation rewrites. It's the
 *   smallest piece of "instruction" the GA can manipulate. Genes
 *   combine into a Genome (the full instruction tape, §5.2), but the
 *   GA's primitive operations — choose a midpoint, copy from parent A
 *   or B, replace this slot with fresh random — all happen at the
 *   GENE granularity.
 *
 * SEMANTICS (physics-side)
 *   Each gene is a force vector applied at one tick of the simulation:
 *      v += (fx, fy)          (subject to MAX_VEL clamp)
 *   Force magnitude is bounded by MAX_FORCE = 0.05 cell-units/tick² —
 *   one gene nudges velocity by at most 5% of a cell. MAX_VEL (0.6
 *   cells/tick) clamps the integrated velocity.
 *
 * WHY 2-D (not just a scalar thrust)
 *   The target lives on a 2-D screen so 2-D actions are the natural
 *   choice. A scalar (just thrust magnitude along a fixed direction)
 *   would force every rocket to commit to one heading; 2-D lets each
 *   gene point anywhere — and the GA discovers heading-changes as
 *   sequences of genes that gradually rotate the velocity.
 *
 * ALGORITHM REFERENCE
 *   Holland (1975), Goldberg (1989) — gene-as-allele abstraction;
 *   standard GA representation. Shiffman ch. 9 uses the same 2-D
 *   force-vector gene form for Smart Rockets.
 */
typedef struct {
    float fx;   /* x component of the force, cell-units / tick².        *
                 * |gene| ≤ MAX_FORCE = 0.05 after init.                */
    float fy;   /* y component, same units.                            */
} Gene;

/*
 * §5.2 Genome — immutable instruction tape for one generation.
 *
 * INTENT
 *   The whole point of the GA. The genome IS the rocket's strategy.
 *   Crossover splices genomes; mutation rewrites individual genes;
 *   selection picks which genomes reproduce. None of that touches
 *   the physical state — the genome lives ONE step removed from
 *   physics.
 *
 * INVARIANT (immutable across one generation)
 *   ga_breed() writes a fresh genome at end-of-generation. rocket_tick
 *   READS genes[age] each step but never writes. So during a
 *   generation the genome is fixed; selection sees the BEHAVIOUR
 *   (encoded by physics + fitness) that resulted from this fixed
 *   program.
 *
 * WHY FIXED-LENGTH (not variable-length)
 *   Variable-length representations (e.g. Genetic Programming) allow
 *   richer behaviour but make crossover harder (how do you splice two
 *   different-length tapes?). Fixed LIFESPAN keeps single-point
 *   crossover trivial: pick a midpoint, copy left from A, right from B.
 *
 * WHY ONE GENE PER TICK (not one gene per "decision")
 *   Each tick consumes exactly genes[age] → the genome is a tape of
 *   length LIFESPAN that maps DIRECTLY to time. No interpreter, no
 *   control flow, no branching. The fitness landscape stays
 *   stationary (each slot has a stable position in the action
 *   sequence), which keeps GA convergence reliable.
 *
 * ALGORITHM REFERENCES
 *   Holland (1975) — chromosome / genome abstraction.
 *   Goldberg (1989) — single-point crossover on fixed-length strings.
 *   Shiffman, "Nature of Code" ch. 9 — the exact tape-of-forces form
 *     used here.
 */
typedef struct {
    Gene genes[LIFESPAN];   /* one force vector applied per tick.        *
                             * Tick i ∈ [0, LIFESPAN) consumes genes[i].*
                             * LIFESPAN = 100 → 100 instructions per    *
                             * lifetime. ga_breed() crossover splices   *
                             * on this array.                            */
} Genome;

/*
 * §5.3 Phenotype — physical state DURING one generation.
 *
 * INTENT
 *   The "expression" of the genome through Newtonian physics. Reset
 *   to zero at every launch; written each tick by rocket_tick();
 *   scored at end-of-generation by rocket_fitness(). The phenotype
 *   carries the OUTCOME of the genome (alive/hit/crashed + fitness)
 *   back into the GA's selection step.
 *
 * GA TERMINOLOGY (memorise — the whole field uses it)
 *   genotype  = Genome    — the recipe                  (immutable)
 *   phenotype = Phenotype — what the recipe produces    (computed)
 *   fitness   = a SCORE attached to the phenotype       (selection input)
 *
 *   The GA never operates on phenotypes directly — it operates on
 *   genotypes BIASED BY phenotype-derived fitness scores. That
 *   separation is what makes evolution work over non-differentiable
 *   fitness functions.
 *
 * LIFECYCLE PER ROCKET (per generation)
 *   1. rocket_launch  — px/py = launch position, all flags = 0
 *   2. rocket_tick (LIFESPAN times) — apply genes[age]:
 *        - integrate position via velocity (clamped to MAX_VEL)
 *        - test target-hit (sets hit_target, alive = 0)
 *        - test off-screen (sets crashed, alive = 0)
 *   3. rocket_fitness — reads final px/py + flags, writes fitness
 *   4. ga_evolve — fitness drives the mating pool
 *
 * WHY hit_target AND crashed ARE SEPARATE FLAGS
 *   The fitness function applies different multipliers (×10 for a
 *   hit, ×0.1 for a crash). Mutually exclusive in practice (rocket
 *   stops on either event), but stored separately so the renderer
 *   can show "successfully landed" rockets (hit) distinctly from
 *   "wandered off-screen" ones (crashed) — important for the user
 *   watching convergence.
 *
 * ALGORITHM REFERENCES
 *   Newton's 2nd law — constant-force integration of (vx, vy, px, py).
 *   Shiffman, "Nature of Code" ch. 9 — exact phenotype state used here.
 *   Goldberg (1989) — the genotype/phenotype distinction in GA theory.
 */
typedef struct {
    /* — position & velocity (the integrator's state) ——————————————— */
    float px;       /* x position, cell units (float). Written by      *
                     * rocket_tick:  v += genes[age]; p += v .         *
                     * Read by fitness + renderer + target-hit test.    */
    float py;       /* y position, same units.                         */
    float vx;       /* x velocity, cells/tick. Reset to 0 at launch;   *
                     * magnitude clamped to MAX_VEL = 0.6.             */
    float vy;       /* y velocity, same units.                         */

    /* — tick + outcome flags ——————————————————————————————————————— */
    int   age;          /* current tick within this generation,        *
                         * [0, LIFESPAN]. Advances each rocket_tick.   *
                         * = LIFESPAN means genome exhausted.          */
    int   alive;        /* 1 → still simulating. Cleared when:        *
                         *   - genome exhausted (age = LIFESPAN), OR  *
                         *   - hit_target = 1, OR                      *
                         *   - crashed = 1                             */
    int   hit_target;   /* 1 → reached the target circle. Fitness     *
                         * gets a ×10 bonus in rocket_fitness().      */
    int   crashed;      /* 1 → flew off-screen. Fitness gets a ×0.1  *
                         * penalty in rocket_fitness().               */

    /* — fitness score (the GA's selection input) ——————————————————— */
    float fitness;      /* set by rocket_fitness() at end-of-gen.     *
                         * = (1/(dist+1))² × bonus × penalty          *
                         * Drives mating-pool slot count in ga_evolve.*/
} Phenotype;

/*
 * §5.4 Trail — visualisation-only history of recent positions.
 *
 * INTENT
 *   Pure cosmetic. Not read by physics, not scored by fitness, not
 *   touched by the GA. Separating it from Phenotype documents AT THE
 *   TYPE LEVEL that this is "what the viewer sees" rather than "what
 *   the algorithm decides" — a learner can confidently ignore Trail
 *   when studying the algorithm.
 *
 * WHY A FIXED-LENGTH BUFFER (not unbounded)
 *   A full LIFESPAN trail per rocket would be 100 floats × 100
 *   rockets = 80 KB of visual data per generation. TRAIL_LEN = 5
 *   reads as a short "comet tail" without clutter; longer trails
 *   actually HURT legibility because the screen fills with dots
 *   and the rockets become hard to distinguish from their own past.
 *
 * WHY NEWEST-FIRST (not oldest-first ring)
 *   The renderer paints all valid trail points (no fade animation),
 *   so the order is purely a tick-shift convention. Newest-first
 *   means a fresh point is just stored at index 0 after a single
 *   linear shift — easy to read in the code.
 *
 * UPDATE EACH TICK (see rocket_tick)
 *   if (count < TRAIL_LEN) count++;
 *   for (i = TRAIL_LEN-1; i > 0; --i) { tx[i] = tx[i-1]; ty[i] = ty[i-1]; }
 *   tx[0] = pheno.px;  ty[0] = pheno.py;
 *
 *   O(TRAIL_LEN) per tick — fine at this size.
 */
typedef struct {
    float tx[TRAIL_LEN];  /* most recent x positions, NEWEST at index 0,*
                           * oldest at index (count-1). Cell-units.    */
    float ty[TRAIL_LEN];  /* most recent y positions, same indexing.   */
    int   count;          /* valid prefix length, [0, TRAIL_LEN].      *
                           * Grows from 0 → TRAIL_LEN, then saturates. *
                           * Reset to 0 on rocket_launch().            */
} Trail;

/*
 * §5.5 Rocket — composition of the three GA layers.
 *
 * INTENT
 *   The composition makes the three conceptual layers EXPLICIT at
 *   every access site:
 *
 *      r->genome.genes[i]   the instruction at tick i (unchanged this gen)
 *      r->pheno.px          physical position (changes every tick)
 *      r->trail.tx[0]       most recent visual breadcrumb (cosmetic)
 *
 *   Same bytes, same algorithm — only the namespace changed. The
 *   payoff is that any line of GA code self-documents whether it's
 *   touching the recipe (genome), the expression (pheno), or just
 *   the viewer's experience (trail).
 *
 * LAYERING (the GA cycle reads in this order)
 *
 *      genome   → pheno     by rocket_tick    (genes drive physics)
 *      pheno    → fitness   by rocket_fitness (outcome → score)
 *      fitness  → genome'   by ga_evolve      (score → next-gen tape)
 *
 *   Trail is OUTSIDE the cycle — it's read only by the renderer.
 *
 * SIZE
 *   Each Rocket is ~900 B:
 *      genome: LIFESPAN × 8 B  = 800 B
 *      pheno : ~9 floats/ints  ≈  40 B
 *      trail : 2·TRAIL_LEN·4 + 4 ≈ 44 B
 *   POP_MAX × ~900 B ≈ 90 KB total — fits comfortably in L2 cache;
 *   no malloc anywhere in the program.
 *
 * ALGORITHM REFERENCE
 *   Shiffman, "Nature of Code" ch. 9 — Smart Rockets — uses the
 *   same three-layer breakdown (DNA / Rocket / trail), framed in
 *   Processing rather than C.
 */
typedef struct {
    Genome    genome;  /* immutable instruction tape this generation.   *
                        * Written only by rocket_init_genome (gen 0)    *
                        * and ga_breed (subsequent gens). Read by       *
                        * rocket_tick.                                  */
    Phenotype pheno;   /* physical state, reset each launch. Written   *
                        * by rocket_tick; read by rocket_fitness +     *
                        * renderer.                                    */
    Trail     trail;   /* visual breadcrumbs (cosmetic only).         *
                        * Written each tick by rocket_tick; read by   *
                        * draw_trails when ui.show_trails is true.    */
} Rocket;

/* Fresh genome — every gene is a random unit force vector x MAX_FORCE. */
static void rocket_init_genome(Rocket *r)
{
    for (int i = 0; i < LIFESPAN; i++) {
        float ux, uy;
        rand_unit(&ux, &uy);
        r->genome.genes[i].fx = ux * MAX_FORCE;
        r->genome.genes[i].fy = uy * MAX_FORCE;
    }
}

/* Reset phenotype + trail to launch state — keeps the genome unchanged.
 * Same rocket, same recipe, fresh expression. */
static void rocket_launch(Rocket *r, int launch_x, int launch_y)
{
    r->pheno.px         = (float)launch_x;
    r->pheno.py         = (float)launch_y;
    r->pheno.vx         = 0.0f;
    r->pheno.vy         = 0.0f;
    r->pheno.age        = 0;
    r->pheno.alive      = 1;
    r->pheno.hit_target = 0;
    r->pheno.crashed    = 0;
    r->pheno.fitness    = 0.0f;
    r->trail.count      = 0;
}

/* Push the current pheno position onto the trail. Newest at index 0;
 * older entries shift down one slot. Count saturates at TRAIL_LEN. */
static void trail_push(Trail *tr, float px, float py)
{
    if (tr->count < TRAIL_LEN) tr->count++;
    for (int i = TRAIL_LEN - 1; i > 0; i--) {
        tr->tx[i] = tr->tx[i - 1];
        tr->ty[i] = tr->ty[i - 1];
    }
    tr->tx[0] = px;
    tr->ty[0] = py;
}

/* Apply one gene's force to velocity, then clamp magnitude to MAX_VEL.
 * Force application + speed cap are the only physics this demo has. */
static void apply_gene_with_velocity_cap(Phenotype *p, const Gene *g)
{
    p->vx += g->fx;
    p->vy += g->fy;
    float v2 = p->vx * p->vx + p->vy * p->vy;
    if (v2 > MAX_VEL * MAX_VEL) {
        float scale = MAX_VEL / sqrtf(v2);
        p->vx *= scale;
        p->vy *= scale;
    }
}

/* Forward-Euler step: advance position by current velocity. */
static void integrate_position(Phenotype *p)
{
    p->px += p->vx;
    p->py += p->vy;
}

/* Hit test against the target's 2:1 cell-space ellipse (which renders
 * as a circle on the terminal). CELL_ASPECT_X compresses x so the
 * test is isotropic in physical screen units. */
static int hit_target_ellipse(const Phenotype *p, float tx, float ty,
                              float trad)
{
    float dx = (p->px - tx) * CELL_ASPECT_X;
    float dy = (p->py - ty);
    return dx * dx + dy * dy <= trad * trad;
}

/* True if the rocket has flown off the playable area. The -1 on rows
 * leaves the bottom hint strip safe. */
static int off_screen(const Phenotype *p, int rows, int cols)
{
    return p->px < 0 || p->px >= cols ||
           p->py < 0 || p->py >= rows - 1;
}

/*
 * rocket_tick — one simulation step. Pseudocode:
 *
 *   (1) early-out if not alive or genome exhausted
 *   (2) push current position onto the trail (cosmetic)
 *   (3) consume genes[age] — apply force, clamp velocity
 *   (4) integrate position by velocity; advance age
 *   (5) target hit?  set hit_target + alive = 0 → done
 *   (6) off-screen?  set crashed    + alive = 0
 */
static void rocket_tick(Rocket *r, float tx, float ty, float trad,
                        int rows, int cols)
{
    Phenotype *p  = &r->pheno;
    Trail     *tr = &r->trail;

    /* (1) early-out */
    if (!p->alive) return;
    if (p->age >= LIFESPAN) { p->alive = 0; return; }

    /* (2) trail (visual) */
    trail_push(tr, p->px, p->py);

    /* (3) consume this tick's gene */
    apply_gene_with_velocity_cap(p, &r->genome.genes[p->age]);

    /* (4) integrate + advance age */
    integrate_position(p);
    p->age++;

    /* (5) target hit ends the flight with a bonus */
    if (hit_target_ellipse(p, tx, ty, trad)) {
        p->hit_target = 1;
        p->alive      = 0;
        return;
    }

    /* (6) off-screen ends the flight with a penalty */
    if (off_screen(p, rows, cols)) {
        p->crashed = 1;
        p->alive   = 0;
    }
}

/*
 * rocket_fitness — see KEY FORMULAS for the exact form.
 *
 * Squaring at the end amplifies selection pressure: a 2x fitness lead
 * becomes a 4x breeding advantage. Without the square the population
 * converges in ~200 gens; with it, ~30 gens. Going further (cubing)
 * collapses diversity too fast and gets stuck in local minima.
 */
static float rocket_fitness(const Rocket *r, float tx, float ty)
{
    const Phenotype *p = &r->pheno;

    /* (1) cell-aspect-corrected distance to target */
    float dx   = (p->px - tx) * CELL_ASPECT_X;
    float dy   = (p->py - ty);
    float dist = sqrtf(dx * dx + dy * dy);

    /* (2) base score: closer is better; +1 keeps the inverse finite */
    float f = 1.0f / (dist + FITNESS_DIST_OFFSET);

    /* (3) outcome multipliers — see §1.1 */
    if (p->hit_target) f *= FITNESS_HIT_BONUS;
    if (p->crashed)    f *= FITNESS_CRASH_PENALTY;

    /* (4) square to sharpen selection pressure (Goldberg 1989) */
    return f * f;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  ga                                                                  */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * §6.1 Population — the set of candidate solutions.
 *
 * INTENT
 *   Holland's GA operates on a POPULATION, not on individuals. The
 *   diversity within the population is what gives the GA its power
 *   — different rockets explore different parts of the genome space,
 *   and the best ones get to breed. n is the active size; rockets[]
 *   is over-provisioned to POP_MAX so we never allocate at runtime.
 *
 * WHY VARIABLE n
 *   The user can grow / shrink the population live via '[' / ']' to
 *   feel the GA's sensitivity to population size:
 *      small n (20)  →  fast generations, prone to losing diversity
 *      large n (100) →  slow generations, better global search
 *   Standard GA literature recommends ~50-200 for problems of this
 *   difficulty (Goldberg 1989 ch. 3, Mitchell 1996).
 *
 * INVARIANT
 *   Mutating n via '[' / ']' triggers a full scene_reseed() because
 *   the active set's identity changes — old rockets at indices ≥
 *   new_n would be ignored but their genomes would still be in
 *   memory. Cleaner to reset everyone.
 *
 * ALGORITHM REFERENCES
 *   Holland (1975) — population concept and why diversity matters.
 *   Goldberg (1989) ch. 3 — population sizing guidance.
 */
typedef struct {
    Rocket rockets[POP_MAX];   /* fixed allocation, POP_MAX = 100.      *
                                * Only rockets[0..n) are live in any    *
                                * generation; rockets[n..POP_MAX) are   *
                                * uninitialised slack.                  */
    int    n;                  /* current population size, clamped to   *
                                * [POP_MIN, POP_MAX] = [20, 100].       *
                                * Default POP_DEFAULT = 50.             *
                                * '[' / ']' keys nudge by POP_STEP = 5; *
                                * each mutation triggers scene_reseed.  */
} Population;

/*
 * §6.2 Generation — counters tracked across one breed cycle.
 *
 * INTENT
 *   Identifies WHERE we are in the GA loop. `number` is the global
 *   generation index (starts at 0, advances on each ga_evolve()).
 *   `tick` and `n_hits` track in-generation progress; best_fitness
 *   carries the previous generation's peak fitness so the HUD shows
 *   something meaningful while the new generation is still flying.
 *
 * LIFECYCLE
 *   At boot:   number = 0,  tick = 0,  n_hits = 0,  best_fitness = 0
 *   Each scene_step → tick += 1, n_hits possibly += 1
 *   When tick reaches LIFESPAN OR all rockets dead → ga_evolve:
 *       best_fitness = max over rockets of pheno.fitness
 *       number      += 1
 *       tick         = 0
 *       n_hits       = 0
 *
 * WHY best_fitness LAGS BY ONE GENERATION
 *   Fitness is only computed at end-of-generation (inside ga_evolve),
 *   so during a generation the only fitness data we have is from
 *   the PREVIOUS generation. Showing that one keeps the HUD reading
 *   meaningful throughout the gen rather than going blank.
 *
 * ALGORITHM REFERENCE
 *   Standard GA generation accounting — Goldberg (1989) ch. 1,
 *   Mitchell (1996) ch. 1.
 */
typedef struct {
    int   number;        /* global generation index, 0 at boot.         *
                          * Advances by 1 on each ga_evolve(). HUD     *
                          * displays this as "gen:NN".                  */
    int   tick;          /* current tick within this generation,        *
                          * [0, LIFESPAN). Wraps to 0 inside ga_evolve.*/
    int   n_hits;        /* rockets that hit the target IN THIS gen.   *
                          * Accumulated by scene_step each tick. Reset *
                          * to 0 in ga_evolve.                         */
    float best_fitness;  /* max fitness from the PREVIOUS generation.  *
                          * Computed in ga_evolve at the end of each   *
                          * generation. Drives no algorithm — pure HUD.*/
} Generation;

/*
 * §6.3 GAParams — user-tunable genetic-algorithm parameters.
 *
 * INTENT
 *   The dials the user can turn at runtime to feel their effect on
 *   GA behaviour. Other GA hyperparameters (population size,
 *   lifespan, selection scheme, crossover style) are either in other
 *   structs or compile-time constants — only mutation_rate has a
 *   live keybinding.
 *
 * WHY MUTATION RATE MATTERS (the demo's pedagogical payload)
 *   - Too LOW  (~0.001):  population converges too fast; gets stuck in
 *                         local optima. Cluster drift around launch
 *                         column without ever reaching target.
 *   - Just right (~0.01): converges in 30-80 gens. Standard textbook
 *                         recommendation (Mitchell 1996 §5.3: ~1/L).
 *   - Too HIGH (~0.10):   mutation dominates over selection; the
 *                         population becomes random noise and never
 *                         converges.
 *
 *   Watching the HUD's "best:" climb across mutation-rate changes
 *   is the most direct way to understand the exploration/exploitation
 *   tradeoff in the GA.
 *
 * ALGORITHM REFERENCES
 *   Mitchell (1996) §5.3 — mutation-rate guidance ≈ 1/L (L = genome
 *                          length). For LIFESPAN = 100, that's 0.01.
 *   Goldberg (1989) ch. 3 — schema-disruption tradeoff with mutation.
 */
typedef struct {
    float mutation_rate;  /* per-gene replace probability during        *
                           * crossover. Clamped to                      *
                           *   [MUTATION_MIN, MUTATION_MAX]            *
                           *   = [0.001, 0.10].                        *
                           * Default MUTATION_DEFAULT = 0.01            *
                           * (1 gene in 100 mutates per child).         *
                           * '+' / '-' keys nudge by MUTATION_STEP =    *
                           * 0.005.                                     */
} GAParams;

/*
 * §6.4 WorldGeom — screen-derived target + launch positions.
 *
 * INTENT
 *   The world has TWO significant points: where the rockets start
 *   (launch) and where they're trying to go (target). Both are
 *   recomputed at startup and on SIGWINCH from the current terminal
 *   size — no pre-baked coordinates anywhere else in the file.
 *
 * WHY CELL-SPACE (not pixel-space)
 *   Rockets live in cell-space — their pheno.px / py are floats but
 *   the integer parts ARE cell coordinates. Storing target and
 *   launch in the same coordinate system means fitness can compute
 *   distance directly without unit conversion.
 *
 * LAYOUT (cf. scene_position)
 *   target at the top centre:    (cols/2, 3)
 *   launch at the bottom centre: (cols/2, rows - LAUNCH_OFFSET)
 *   Both share the centre column, so rockets fly "up" in screen
 *   convention (toward smaller y).
 *
 * WHY THE ASPECT-RATIO FUDGE LIVES IN fitness, not here
 *   Terminal cells are ~2× taller than wide, so a visually-circular
 *   target is a 2:1 ellipse in cell coords. We DON'T store an
 *   ellipse here — fitness compresses dx by 0.5× at distance-test
 *   time, and draw_target renders the matching wide ellipse. Keeps
 *   WorldGeom spec-aligned with cell semantics; aspect correction
 *   stays localised to the two places that need it.
 */
typedef struct {
    int target_x;   /* target column. Centre of screen by default     *
                     * (cols / 2). Cell-space.                         */
    int target_y;   /* target row. Near the top (row 3). Cell-space.  */
    int launch_x;   /* launch column. Centre of screen (cols / 2).    */
    int launch_y;   /* launch row. rows - LAUNCH_OFFSET (≈ 2 from    *
                     * the bottom). Cell-space.                       */
} WorldGeom;

/*
 * §6.5 WorldUI — interactive flags toggled by the keyboard.
 *
 * INTENT
 *   Pure UI state. Mutating these flags NEVER changes population
 *   fitness or genome content — only how the simulation paces and
 *   how the screen looks. The contract is enforced by SEPARATION
 *   from Population / Generation / GAParams: anything affecting GA
 *   outcomes lives in those structs, not here.
 *
 * WHY fast MODE EXISTS (the demo's killer feature)
 *   In normal mode (1 tick / frame), watching the GA converge takes
 *   30-80 generations × 100 ticks × ~33 ms ≈ 100+ seconds — too slow
 *   for an interactive demo.
 *
 *   In fast mode (LIFESPAN ticks / frame = 1 full generation per
 *   render), the same convergence runs in 30-80 frames ≈ 1-3 seconds.
 *   Same algorithm, different visualisation cadence — flipping
 *   between the two is how the user actually feels the GA learning.
 */
typedef struct {
    int paused;       /* 'p' — freeze the simulation entirely.         *
                       * Renderer keeps running so the user can study  *
                       * the current frame.                            */
    int fast;         /* 'f' — 1 full generation per render frame.    *
                       * scene_step runs LIFESPAN times instead of    *
                       * once; breaks early if a generation ends.     */
    int show_trails;  /* 's' — toggle dim breadcrumb dots behind each *
                       * rocket. Default on. Pure cosmetic.            */
    int theme;        /* 't' — cycles 0 .. N_THEMES - 1. Each theme  *
                       * has its own [target, rocket, trail, best]   *
                       * colour tuple in THEME_FG_256 / THEME_FG_8.  *
                       * HUD/HINT stay fixed across themes per       *
                       * CLAUDE.md HUD spec.                          */
} WorldUI;

/*
 * §6.6 Screen — current ncurses dimensions.
 *
 * INTENT
 *   Single point of truth for "how big is the terminal RIGHT NOW".
 *   scene_position() derives target / launch from these; the physics
 *   step uses them as the out-of-bounds crash threshold; the
 *   renderer uses them to clamp each draw.
 *
 * MUTATION RULE
 *   Modified only inside main() (at startup, reading LINES/COLS)
 *   and the SIGWINCH branch (when the terminal resized). Never
 *   changes mid-frame — physics and renderer treat them as
 *   immutable for one tick.
 *
 * RESIZE BEHAVIOUR
 *   Unlike a full reseed, resize KEEPS the in-flight generation
 *   intact (genomes survive, trails survive); only target / launch
 *   refresh to the new geometry. The next ga_evolve gives the new
 *   layout a clean slate.
 */
typedef struct {
    int cols;   /* terminal width in character cells.                  */
    int rows;   /* terminal height in character cells.                 */
} Screen;

/*
 * §6.7 FrameTimer — render-loop pacing + EWMA-smoothed fps.
 *
 * INTENT
 *   Pulls three loose locals out of main() into one named struct.
 *   Single timebase (TARGET_FPS = 30) — no separate sim cadence
 *   here, because the GA is driven by tick COUNTS (LIFESPAN ticks
 *   per generation), not wall-clock seconds.
 *
 * EWMA SMOOTHING (the one bit of math)
 *   fps  =  0.95 · fps  +  0.05 · instantaneous
 *
 *   One-pole IIR low-pass filter (exponentially-weighted moving
 *   average). The 0.95 / 0.05 split gives roughly a 20-frame window
 *   of effective smoothing — fast enough to track real fps changes,
 *   slow enough that the HUD's digits don't jitter every frame.
 *
 * WHY NO ACCUMULATOR / SUB-STEPPING (unlike other demos)
 *   GA progress is per-tick, not per-second. A slow frame doesn't
 *   make the simulation lag; it just means the user waits longer to
 *   see the next tick. There's no Euler integrator that needs a
 *   bounded dt, so a simple "sleep the remainder of frame_ns" cap
 *   is sufficient.
 *
 * ALGORITHM REFERENCE
 *   Fiedler, "Fix Your Timestep!" — frame-pacing pattern. Simpler
 *   variant here because the sim doesn't integrate forces over wall
 *   time. EWMA fps display is a standard one-pole IIR filter
 *   (Oppenheim & Schafer, "Discrete-Time Signal Processing").
 */
typedef struct {
    int64_t frame_ns;  /* one frame budget, ns. Computed at boot as   *
                        * 1e9 / TARGET_FPS = 33.3 ms.                 */
    int64_t fps_prev;  /* clock_ns() at the previous frame. dt is    *
                        * derived as (now - fps_prev) at each render.*/
    double  fps;       /* EWMA-smoothed frames per second.            *
                        * Seeded to TARGET_FPS at boot; updated every*
                        * frame; displayed in the HUD top row.        */
} FrameTimer;

/*
 * §6.8 Scene — the top-level container.
 *
 * INTENT
 *   Single file-static instance (g_scene) owns the whole program
 *   state. Composed of five GA-concept sub-structs (Population,
 *   Generation, GAParams, WorldGeom, WorldUI) plus Screen +
 *   FrameTimer and two volatile sig_atomic_t flags for signal
 *   handlers. Replaces what used to be a global World type plus
 *   separate file-scope g_running / g_need_resize flags.
 *
 * SUB-STRUCT LAYERING (read in this order to understand the program)
 *
 *      1. Population  — the candidates the GA acts on
 *      2. Generation  — where we are in the GA loop right now
 *      3. GAParams    — the user-tunable dials
 *      4. WorldGeom   — where the world's two significant points are
 *      5. WorldUI     — keyboard-driven cosmetic flags
 *      6. Screen      — terminal dimensions
 *      7. FrameTimer  — render-loop pacing
 *      + running, need_resize — signal-handler flags
 *
 * MUTATION CONTRACT (which keys touch which sub-struct)
 *
 *   Population    [ / ]    n            → triggers scene_reseed
 *                 r        full reseed (genomes + launch)
 *   Generation    (per tick)   tick, n_hits        (auto)
 *                 (per evolve) number, best_fitness (auto)
 *   GAParams      + / -    mutation_rate
 *   WorldGeom     SIGWINCH target/launch positions  (auto)
 *   WorldUI       p / f / s / t per the field comments
 *   running       q / ESC, SIGINT, SIGTERM
 *   need_resize   SIGWINCH                          (auto)
 *
 *   Population / Generation / GAParams writes are GA-algorithmic.
 *   WorldGeom / WorldUI / Screen / FrameTimer writes are NOT (no
 *   effect on fitness or genome content).
 *
 * INITIALIZATION ORDER (see main())
 *   1. seed UI defaults (theme, mutation rate, show_trails, pop.n)
 *   2. screen_init    — bring ncurses up
 *   3. read LINES/COLS into screen.cols/rows
 *   4. scene_position — compute target/launch from screen size
 *   5. scene_reseed   — fresh genomes + launch all rockets
 *   6. timer.frame_ns, fps_prev, fps — seed FrameTimer
 *
 * WHY A FILE-STATIC GLOBAL (not passed by pointer)
 *   Population alone is ~90 KB; passing &g_scene through every
 *   helper costs nothing at the call site but the file-static gives
 *   signal handlers a path to running / need_resize without API
 *   ceremony. The fields outside pop.rockets[] are all small scalars,
 *   so the compiler folds the dereference chain efficiently.
 */
typedef struct {
    Population            pop;          /* the candidate set        */
    Generation            gen;          /* GA loop counters         */
    GAParams              ga;           /* user-tunable dials       */
    WorldGeom             geom;         /* screen-derived geometry  */
    WorldUI               ui;           /* keyboard cosmetic flags  */
    Screen                screen;       /* ncurses dimensions       */
    FrameTimer            timer;        /* render-loop pacing       */
    volatile sig_atomic_t running;      /* cleared by 'q' / SIGINT  */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH          */
} Scene;

/*
 * ga_breed — single-point crossover with per-gene mutation. The split
 * point is randomised per child so siblings inherit different
 * combinations of their parents' genomes.
 */
static void ga_breed(Rocket *child, const Rocket *a, const Rocket *b,
                     float mut_rate)
{
    int mid = rand() % LIFESPAN;
    for (int i = 0; i < LIFESPAN; i++) {
        child->genome.genes[i] = (i < mid) ? a->genome.genes[i]
                                           : b->genome.genes[i];
        if (frand() < mut_rate) {
            float ux, uy;
            rand_unit(&ux, &uy);
            child->genome.genes[i].fx = ux * MAX_FORCE;
            child->genome.genes[i].fy = uy * MAX_FORCE;
        }
    }
}

/* File-scope scratch — POP_MAX Rockets is ~100 KB, too large for stack. */
static int    g_pool[POOL_CAPACITY];
static Rocket g_next[POP_MAX];

/* Phase 1 — Score every rocket and return the maximum fitness.
 * The max is needed to normalise mating-pool slot counts. */
static float score_population(Scene *s)
{
    Rocket *pop = s->pop.rockets;
    float   max_f = 0.0f;
    for (int i = 0; i < s->pop.n; i++) {
        pop[i].pheno.fitness = rocket_fitness(&pop[i],
                                              (float)s->geom.target_x,
                                              (float)s->geom.target_y);
        if (pop[i].pheno.fitness > max_f) max_f = pop[i].pheno.fitness;
    }
    return max_f;
}

/* Phase 2 — Build the fitness-proportional mating pool.
 *
 * Each rocket gets k = floor(POOL_SLOTS_MAX · fitness / max_fitness)
 * slots in g_pool[] (minimum 1). Later, uniform random picks from
 * this pool implement fitness-proportional selection (Goldberg 1989).
 * Returns the pool's actual fill size. */
static int build_mating_pool(const Scene *s, float max_f)
{
    if (max_f <= 0.0f) max_f = 1.0f;   /* defensive — should not happen */

    int pool_size = 0;
    for (int i = 0; i < s->pop.n; i++) {
        int slots = (int)((s->pop.rockets[i].pheno.fitness / max_f)
                          * POOL_SLOTS_MAX);
        if (slots < 1) slots = 1;
        for (int j = 0; j < slots && pool_size < POOL_CAPACITY; j++)
            g_pool[pool_size++] = i;
    }
    return pool_size;
}

/* Phase 3 — Breed s->pop.n children into g_next[].
 * For each child: pick two parents uniformly from the pool, splice
 * their genomes via ga_breed (single-point crossover + per-gene
 * mutation), and launch the child from the spawn point. */
static void breed_next_generation(const Scene *s, int pool_size)
{
    const Rocket *pop = s->pop.rockets;
    for (int i = 0; i < s->pop.n; i++) {
        int a = g_pool[rand() % pool_size];
        int b = g_pool[rand() % pool_size];
        ga_breed(&g_next[i], &pop[a], &pop[b], s->ga.mutation_rate);
        rocket_launch(&g_next[i], s->geom.launch_x, s->geom.launch_y);
    }
}

/* Phase 4 — Replace the population with the bred children + reset
 * the per-generation counters. number advances; tick + n_hits zero. */
static void swap_in_children(Scene *s)
{
    memcpy(s->pop.rockets, g_next, sizeof(Rocket) * (size_t)s->pop.n);
    s->gen.number++;
    s->gen.tick   = 0;
    s->gen.n_hits = 0;
}

/*
 * ga_evolve — close out the current generation. Holland's GA cycle:
 *
 *   1. score_population         — compute fitness, track maximum
 *   2. build_mating_pool        — fitness-proportional slot allocation
 *   3. breed_next_generation    — pick parents, splice, mutate, launch
 *   4. swap_in_children         — replace population, advance counters
 *
 * Reads as the four classical GA phases, one helper call per phase.
 */
static void ga_evolve(Scene *s)
{
    float max_f         = score_population(s);
    s->gen.best_fitness = max_f;

    int pool_size = build_mating_pool(s, max_f);
    breed_next_generation(s, pool_size);
    swap_in_children(s);
}

/* scene_position — recompute target / launch in cell coords for the
 * current screen size. Called at startup and after every SIGWINCH. */
static void scene_position(Scene *s, int rows, int cols)
{
    s->geom.target_x = cols / 2;
    s->geom.target_y = 3;
    s->geom.launch_x = cols / 2;
    s->geom.launch_y = rows - LAUNCH_OFFSET;
}

/* scene_reseed — gen-0 seed: brand-new random genomes for every rocket. */
static void scene_reseed(Scene *s)
{
    for (int i = 0; i < s->pop.n; i++) {
        rocket_init_genome(&s->pop.rockets[i]);
        rocket_launch(&s->pop.rockets[i], s->geom.launch_x, s->geom.launch_y);
    }
    s->gen.number       = 0;
    s->gen.tick         = 0;
    s->gen.n_hits       = 0;
    s->gen.best_fitness = 0.0f;
}

/* scene_step — one tick: advance every alive rocket. When the genome
 * runs out (or the whole population is dead), evolve into the next gen. */
static void scene_step(Scene *s, int rows, int cols)
{
    Rocket *pop = s->pop.rockets;
    int     n   = s->pop.n;

    int any_alive = 0;
    for (int i = 0; i < n; i++) {
        if (!pop[i].pheno.alive) continue;
        rocket_tick(&pop[i],
                    (float)s->geom.target_x, (float)s->geom.target_y,
                    TARGET_RADIUS, rows, cols);
        if (pop[i].pheno.hit_target) s->gen.n_hits++;
        if (pop[i].pheno.alive)      any_alive = 1;
    }
    s->gen.tick++;
    if (s->gen.tick >= LIFESPAN || !any_alive)
        ga_evolve(s);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* The alive rocket currently closest to the target. Recomputed per
 * frame so the visual leader updates in real time. -1 when no rocket
 * is still in flight (whole population has hit or crashed). */
static int find_leader(const Scene *s)
{
    int   best   = -1;
    float best_d = 1e30f;
    for (int i = 0; i < s->pop.n; i++) {
        const Rocket    *r = &s->pop.rockets[i];
        const Phenotype *p = &r->pheno;
        if (!p->alive) continue;
        float dx = (p->px - (float)s->geom.target_x) * 0.5f;
        float dy = (p->py - (float)s->geom.target_y);
        float d2 = dx * dx + dy * dy;
        if (d2 < best_d) { best_d = d2; best = i; }
    }
    return best;
}

/* Aspect-corrected ellipse test: is cell (sc, sr) inside the target's
 * 2:1 ellipse around (tx, ty)? Same CELL_ASPECT_X compression as
 * hit_target_ellipse so the visual circle and the hit detector
 * agree exactly. */
static int inside_target_ellipse(int sc, int sr, int tx, int ty)
{
    float dx = (sc - tx) * CELL_ASPECT_X;
    float dy = (sr - ty);
    return dx * dx + dy * dy <= TARGET_RADIUS * TARGET_RADIUS;
}

/*
 * draw_target — paint the target as a filled 2:1 ellipse.
 *
 *   ry = ceil(TARGET_RADIUS)   vertical half-extent (cells)
 *   rx = 2·ry                  horizontal half-extent (2× to undo cell aspect)
 *
 * Iterate the bounding box and stamp '@' wherever
 * inside_target_ellipse() agrees. Reads as the bounding-box scan
 * with the ellipse-test predicate.
 */
static void draw_target(int rows, int cols, int tx, int ty)
{
    int ry = (int)(TARGET_RADIUS + 0.5f);
    int rx = ry * 2;

    attron(COLOR_PAIR(PAIR_TARGET) | A_BOLD);
    for (int sr = ty - ry; sr <= ty + ry; sr++) {
        if (sr < 0 || sr >= rows - 1) continue;
        for (int sc = tx - rx; sc <= tx + rx; sc++) {
            if (sc < 0 || sc >= cols) continue;
            if (inside_target_ellipse(sc, sr, tx, ty))
                mvaddch(sr, sc, (chtype)(unsigned char)'@');
        }
    }
    attroff(COLOR_PAIR(PAIR_TARGET) | A_BOLD);
}

/* Launch pad — three glyphs centred under the spawn column. Drawn with
 * the default pair (just A_BOLD) so it reads as neutral terminal-fg
 * regardless of the active theme. */
static void draw_launch(int lx, int ly, int rows, int cols)
{
    if (ly < 0 || ly >= rows - 1) return;
    attron(A_BOLD);
    if (lx - 1 >= 0)    mvaddch(ly, lx - 1, (chtype)(unsigned char)'=');
    if (lx     <  cols) mvaddch(ly, lx,     (chtype)(unsigned char)'^');
    if (lx + 1 <  cols) mvaddch(ly, lx + 1, (chtype)(unsigned char)'=');
    attroff(A_BOLD);
}

static void draw_trails(const Scene *s, int rows, int cols)
{
    if (!s->ui.show_trails) return;
    attron(COLOR_PAIR(PAIR_TRAIL));
    for (int i = 0; i < s->pop.n; i++) {
        const Trail *tr = &s->pop.rockets[i].trail;
        for (int t = 0; t < tr->count; t++) {
            int sr = (int)tr->ty[t];
            int sc = (int)tr->tx[t];
            if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;
            mvaddch(sr, sc, (chtype)(unsigned char)'.');
        }
    }
    attroff(COLOR_PAIR(PAIR_TRAIL));
}

/* Pack vs leader appearance:  '^' PAIR_ROCKET for everyone, but the
 * rocket currently closest to the target gets '@' PAIR_BEST so the
 * eye has something to track during the flight. */
static void rocket_appearance(int is_lead, int *out_pair, char *out_glyph)
{
    *out_pair  = is_lead ? PAIR_BEST : PAIR_ROCKET;
    *out_glyph = is_lead ? '@'       : '^';
}

/* Render only rockets that are EITHER still flying OR have hit the
 * target (so a successful landing stays visible). Crashed rockets
 * are skipped — only their trail dots survive on screen. */
static int rocket_is_visible(const Phenotype *p)
{
    return p->alive || p->hit_target;
}

static void draw_rockets(const Scene *s, int rows, int cols)
{
    int leader = find_leader(s);
    for (int i = 0; i < s->pop.n; i++) {
        const Phenotype *p = &s->pop.rockets[i].pheno;
        if (!rocket_is_visible(p)) continue;

        int sr = (int)p->py;
        int sc = (int)p->px;
        if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;

        int  pair; char glyph;
        rocket_appearance(i == leader, &pair, &glyph);

        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sr, sc, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* Three-state run-mode label for the HUD's right slot. */
static const char *run_mode_label(const Scene *s)
{
    if (s->ui.paused) return "PAUSED ";
    if (s->ui.fast)   return "FAST   ";
    return "running";
}

/* Build the HUD top-row status string. Order chosen so the eye lands
 * on generation and best-fitness first (the two values the user
 * watches for convergence). */
static void hud_format_status(const Scene *s, double fps,
                              char *buf, size_t n)
{
    snprintf(buf, n,
             " gen:%d  pop:%d  hits:%d/%d  best:%.2f  mut:%.3f  "
             "%5.1f fps  %s ",
             s->gen.number, s->pop.n,
             s->gen.n_hits, s->pop.n,
             s->gen.best_fitness, s->ga.mutation_rate,
             fps, run_mode_label(s));
}

/* Paint the HUD status string right-justified at row 0. */
static void hud_paint_status(const char *buf, int cols)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Bottom-row hint strip — lists every interactive key. */
static void hud_paint_hint(int rows)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  r:reset  f:fast  s:trails  t:theme  "
             "[/]:pop  -/+:mut ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * scene_draw — one frame, painter's order:
 *   1. erase backbuffer
 *   2. target ellipse
 *   3. launch pad
 *   4. trails (dim, behind rockets)
 *   5. rockets (bright, with leader highlighted)
 *   6. HUD status row 0 + hint strip row rows-1
 */
static void scene_draw(int rows, int cols, const Scene *s, double fps)
{
    erase();
    draw_target (rows, cols, s->geom.target_x, s->geom.target_y);
    draw_launch (s->geom.launch_x, s->geom.launch_y, rows, cols);
    draw_trails (s, rows, cols);
    draw_rockets(s, rows, cols);

    char buf[160];
    hud_format_status(s, fps, buf, sizeof buf);
    hud_paint_status (buf, cols);
    hud_paint_hint   (rows);

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
    typeahead(-1);          /* prevent ncurses input-peek tearing */
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

/* The single Scene instance. File-scope so signal handlers can flip
 * the volatile sig_atomic_t flags inside it without ceremony. All
 * other code reaches scene state via this global. */
static Scene g_scene;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_scene.running     = 0;
    if (s == SIGWINCH)               g_scene.need_resize = 1;
}

/* ── §9.1 setup + per-step main-loop helpers ───────────────────────── */

/* One-shot scene setup before the loop: signals, defaults, ncurses,
 * geometry, fresh population, FrameTimer seed. */
static void scene_setup(void)
{
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    /* (1) seed user-facing defaults */
    g_scene.running          = 1;
    g_scene.pop.n            = POP_DEFAULT;
    g_scene.ga.mutation_rate = MUTATION_DEFAULT;
    g_scene.ui.theme         = 0;
    g_scene.ui.show_trails   = 1;

    /* (2) bring subsystems up in dependency order */
    screen_init(g_scene.ui.theme);
    g_scene.screen.cols = COLS;
    g_scene.screen.rows = LINES;
    scene_position(&g_scene, g_scene.screen.rows, g_scene.screen.cols);
    scene_reseed(&g_scene);

    /* (3) seed FrameTimer */
    g_scene.timer.frame_ns = NS_PER_SEC / TARGET_FPS;
    g_scene.timer.fps      = TARGET_FPS;
    g_scene.timer.fps_prev = clock_ns();
}

/* SIGWINCH handler — re-sync ncurses + refresh geometry. Keeps the
 * in-flight generation intact (genomes + trails survive). */
static void scene_handle_resize(void)
{
    g_scene.need_resize = 0;
    endwin();
    refresh();
    g_scene.screen.rows = LINES;
    g_scene.screen.cols = COLS;
    scene_position(&g_scene, g_scene.screen.rows, g_scene.screen.cols);
}

/* Sim cadence — 1 tick/frame in normal mode, up to LIFESPAN ticks/frame
 * in fast mode. When an evolve fires mid-burst (gen.tick wraps to 0)
 * we break so the user sees the gen-0 frame before the next burst. */
static void scene_advance_sim_burst(void)
{
    if (g_scene.ui.paused) return;
    int steps = g_scene.ui.fast ? LIFESPAN : 1;
    for (int i = 0; i < steps; i++) {
        scene_step(&g_scene, g_scene.screen.rows, g_scene.screen.cols);
        if (g_scene.gen.tick == 0) break;
    }
}

/* EWMA one-pole IIR low-pass on instantaneous fps.
 *   fps_new = EWMA_RETAIN · fps_old  +  EWMA_NEW · (1e9 / dt_ns)
 * +1 in the denominator defends against a zero-dt frame. */
static void frame_update_ewma_fps(int64_t now)
{
    int64_t dt_ns = now - g_scene.timer.fps_prev + 1;
    double  instant = 1e9 / (double)dt_ns;
    g_scene.timer.fps = g_scene.timer.fps * EWMA_RETAIN
                      + instant            * EWMA_NEW;
    g_scene.timer.fps_prev = now;
}

/* Sleep the remainder of frame_ns so we hit TARGET_FPS. */
static void frame_cap_to_target_fps(int64_t frame_start)
{
    clock_sleep_ns(g_scene.timer.frame_ns - (clock_ns() - frame_start));
}

/* ── §9.2 keyboard action handlers ─────────────────────────────────── */

/* 'p' / 'f' / 's' — flip a UI bit. */
static void key_toggle_pause(void)  { g_scene.ui.paused      ^= 1; }
static void key_toggle_fast(void)   { g_scene.ui.fast        ^= 1; }
static void key_toggle_trails(void) { g_scene.ui.show_trails ^= 1; }

/* 'r' — fresh random genomes for every rocket, generation back to 0. */
static void key_reset_simulation(void) { scene_reseed(&g_scene); }

/* 't' — cycle theme + rebind colour pairs. */
static void key_cycle_theme(void)
{
    g_scene.ui.theme = (g_scene.ui.theme + 1) % N_THEMES;
    color_init(g_scene.ui.theme);
}

/* '+' / '-' — nudge mutation rate, clamped to [MUTATION_MIN, MAX]. */
static void key_mutation_nudge(float delta)
{
    float v = g_scene.ga.mutation_rate + delta;
    if (v < MUTATION_MIN || v > MUTATION_MAX) return;
    g_scene.ga.mutation_rate = v;
}

/* '[' / ']' — nudge population size, clamped to [POP_MIN, MAX].
 * Population-identity change triggers a full reseed. */
static void key_population_nudge(int delta)
{
    int n = g_scene.pop.n + delta;
    if (n < POP_MIN || n > POP_MAX) return;
    g_scene.pop.n = n;
    scene_reseed(&g_scene);
}

/* Dispatch one keystroke to its named action. Each case is one helper
 * call; the switch reads as the keymap. */
static void scene_handle_one_keystroke(int ch)
{
    switch (ch) {
    case 'q': case 27 /* ESC */:  g_scene.running = 0;             break;
    case 'p':                     key_toggle_pause();              break;
    case 'f':                     key_toggle_fast();               break;
    case 's':                     key_toggle_trails();             break;
    case 'r':                     key_reset_simulation();          break;
    case 't':                     key_cycle_theme();               break;
    case '+': case '=':           key_mutation_nudge(+MUTATION_STEP); break;
    case '-':                     key_mutation_nudge(-MUTATION_STEP); break;
    case '[':                     key_population_nudge(-POP_STEP); break;
    case ']':                     key_population_nudge(+POP_STEP); break;
    default:                                                       break;
    }
}

/* Drain all queued keystrokes through the action dispatcher. */
static void scene_drain_input(void)
{
    int ch;
    while ((ch = getch()) != ERR) scene_handle_one_keystroke(ch);
}

/* ── §9.3 main ─────────────────────────────────────────────────────── *
 *
 * Reads as the program's lifecycle:
 *
 *   SETUP:
 *     scene_setup() — signals, defaults, ncurses, geometry, first reseed.
 *
 *   LOOP (each frame):
 *     1. handle pending resize
 *     2. drain queued keystrokes
 *     3. advance sim burst (no-op if paused)
 *     4. update EWMA fps
 *     5. draw the frame
 *     6. sleep to hit TARGET_FPS
 *
 * ─────────────────────────────────────────────────────────────────── */
int main(void)
{
    scene_setup();

    while (g_scene.running) {
        /* (1) deferred resize */
        if (g_scene.need_resize) scene_handle_resize();

        /* (2) drain keystrokes */
        scene_drain_input();

        /* (3) advance sim (one tick, or one full generation in fast mode) */
        scene_advance_sim_burst();

        /* (4) EWMA fps update */
        int64_t now = clock_ns();
        frame_update_ewma_fps(now);

        /* (5) draw the frame */
        scene_draw(g_scene.screen.rows, g_scene.screen.cols,
                   &g_scene, g_scene.timer.fps);

        /* (6) sleep to target frame rate */
        frame_cap_to_target_fps(now);
    }
    return 0;
}
