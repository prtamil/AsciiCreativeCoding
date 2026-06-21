/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * genetic_rocket.c — a flock of rockets that learn to hit a target by
 * "evolving": each carries a list of nudges, the ones that fly closest get to
 * breed (mixed and slightly mutated), and after a few dozen generations they
 * home in.  Drawn in the terminal with ncurses.
 *
 * This is the classic "Smart Rockets" genetic algorithm (Holland 1975; the
 * rocket framing is from Daniel Shiffman's "Nature of Code", ch. 9).
 * Sister file: flocking/flocking.c (similar moving-agent + force model).
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config — population, lifespan, mutation, target geometry ─────── */

#define TARGET_FPS          30

/* how many rockets fly at once (the array is sized for the largest). */
#define POP_DEFAULT         50
#define POP_MIN             20
#define POP_MAX            100
#define POP_STEP             5

/* how many ticks a rocket flies before the survivors breed. */
#define LIFESPAN           100

/* each tick a rocket gets one nudge of at most MAX_FORCE; its speed is
 * capped at MAX_VEL so it can't run away. */
#define MAX_FORCE         0.05f
#define MAX_VEL            0.6f

/* how often a child's nudge is swapped for a fresh random one. 0.01 means
 * roughly 1 nudge in 100 changes per child — the demo's main dial. */
#define MUTATION_DEFAULT  0.01f
#define MUTATION_MIN     0.001f
#define MUTATION_MAX      0.10f
#define MUTATION_STEP    0.005f

/* visual sizes. */
#define TRAIL_LEN            5
#define TARGET_RADIUS      2.5f      /* vertical radius; horizontal is 2× */
#define LAUNCH_OFFSET        2       /* cells above the bottom edge  */

/* the breeding lottery: each rocket gets up to this many tickets. */
#define POOL_SLOTS_MAX     100
#define POOL_CAPACITY      (POP_MAX * POOL_SLOTS_MAX)

#define N_THEMES           4

/* ── §1.1 fitness (how good a flight was) ──────────────────────── */

/* Score is mostly "how close did it get" — nearer is higher — then boosted
 * 10× for a hit and cut to a tenth for a crash.  The +1 keeps the score from
 * blowing up at distance 0, and rocket_fitness squares it so a small lead in
 * closeness becomes a big lead in breeding chances. */
#define FITNESS_DIST_OFFSET   1.0f
#define FITNESS_HIT_BONUS    10.0f      /* boost for reaching the target */
#define FITNESS_CRASH_PENALTY 0.10f     /* cut for flying off-screen     */

/* ── §1.2 cell shape ──────────────────────────────────────────── */

/* terminal cells are about twice as tall as wide, so we squash horizontal
 * distances by half — that way a target drawn as a wide oval behaves like a
 * round circle for hit-testing. */
#define CELL_ASPECT_X         0.5f

/* ── §1.3 main-loop pacing ─────────────────────────────────────── */

#define NS_PER_SEC      1000000000LL

/* the fps readout is smoothed (mostly the old value, a little of the new) so
 * it doesn't flicker — about a 20-frame rolling average. */
#define EWMA_RETAIN     0.95
#define EWMA_NEW        0.05

/* colour-pair slots. */
#define PAIR_TARGET   1
#define PAIR_ROCKET   2
#define PAIR_TRAIL    3
#define PAIR_BEST     4
#define PAIR_HUD      5
#define PAIR_HINT     6

/* ── §2  clock — read the time and sleep ─────────────────────────────── *
 * The monotonic clock only ever counts forward, so the frame timer can't be
 * thrown off if the system clock is changed. */

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

/* ── §3  color — per-theme palettes + HUD colours ────────────────────── */

/* one colour each for target / rocket / trail / leader, per theme — a
 * 256-colour set and an 8-colour fallback. */
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

/* the HUD's yellow status + cyan hint stay the same in every theme, so they
 * stay readable no matter what's moving behind them. */
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

/* ── §4  random — random floats and random directions ───────────────── */

static float frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/* a random direction with every angle equally likely.  We throw darts at a
 * square and keep only the ones inside the circle (and not right at the
 * centre), then point at the dart — picking x and y separately would secretly
 * favour the diagonal directions. */
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

/* ── §5  rocket — the flight plan, the flight, and the trail ─────────── */

/* Gene — a single nudge: one little push (fx, fy) in 2-D.  It's the smallest
 * thing evolution works with — breeding copies it, mutation swaps it.  Each
 * push is at most MAX_FORCE long. */
typedef struct {
    float fx;   /* sideways part of the push (cell-units per tick²) */
    float fy;   /* up/down part of the push */
} Gene;

/* Genome — a rocket's whole flight plan: one nudge per tick, in order.  It
 * stays fixed for the rocket's whole life; evolution only rewrites it between
 * generations.  Same length for everyone, which keeps breeding simple (just
 * splice two plans at a random point). */
typedef struct {
    Gene genes[LIFESPAN];   /* tick i uses genes[i]; LIFESPAN nudges in all */
} Genome;

/* Phenotype — a rocket's live flight: where it is, how fast, and how it ended.
 * Reset at each launch, updated every tick, then graded.  (In the genetics
 * analogy: the genome is the recipe, this is what the recipe actually does, and
 * fitness is the grade it earns — which is what evolution selects on.) */
typedef struct {
    /* where it is and how it's moving */
    float px;       /* sideways position, in cells */
    float py;       /* up/down position, in cells  */
    float vx;       /* sideways speed, cells/tick (capped at MAX_VEL) */
    float vy;       /* up/down speed, cells/tick   */

    /* how far through the flight, and how it ended */
    int   age;          /* which tick we're on, 0..LIFESPAN */
    int   alive;        /* still flying? (cleared on land, crash, or running out) */
    int   hit_target;   /* reached the target — earns the fitness boost */
    int   crashed;      /* flew off-screen — takes the fitness cut */

    /* the flight's grade, used to decide who breeds */
    float fitness;      /* set at end-of-generation by rocket_fitness */
} Phenotype;

/* Trail — the last few spots a rocket passed through, a little comet tail just
 * for the viewer.  Nothing in the algorithm reads it.  Newest spot is at index
 * 0; keeping it short stops the screen filling with dots. */
typedef struct {
    float tx[TRAIL_LEN];  /* recent sideways positions, newest first */
    float ty[TRAIL_LEN];  /* recent up/down positions, newest first  */
    int   count;          /* how many slots are filled, 0..TRAIL_LEN */
} Trail;

/* Rocket — one candidate, bundling its three layers so each line of code says
 * which it touches.  The plan drives the flight, the flight earns a grade, and
 * the grade shapes the next generation's plans; the trail just rides along. */
typedef struct {
    Genome    genome;  /* the flight plan (fixed for this generation) */
    Phenotype pheno;   /* the live flight (reset and re-run each launch) */
    Trail     trail;   /* cosmetic breadcrumbs for the viewer */
} Rocket;

/* fill a brand-new flight plan with random nudges (used for generation 0,
 * before any breeding has happened). */
static void rocket_init_genome(Rocket *r)
{
    for (int i = 0; i < LIFESPAN; i++) {
        float ux, uy;
        rand_unit(&ux, &uy);
        r->genome.genes[i].fx = ux * MAX_FORCE;
        r->genome.genes[i].fy = uy * MAX_FORCE;
    }
}

/* put a rocket back on the pad for a fresh run — clears its flight state and
 * trail but keeps its flight plan. */
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

/* remember one more spot on the trail — newest goes first, the oldest drops
 * off the end. */
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

/* add this tick's nudge to the rocket's speed, then cap the speed so it can't
 * run away.  (Adding a push and capping the speed is the whole physics.) */
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

/* move the rocket by its current speed (one tick's worth). */
static void integrate_position(Phenotype *p)
{
    p->px += p->vx;
    p->py += p->vy;
}

/* did the rocket reach the target?  We squash sideways distance by half (§1.2)
 * so the wide oval on screen counts as a round circle. */
static int hit_target_ellipse(const Phenotype *p, float tx, float ty,
                              float trad)
{
    float dx = (p->px - tx) * CELL_ASPECT_X;
    float dy = (p->py - ty);
    return dx * dx + dy * dy <= trad * trad;
}

/* did the rocket leave the screen?  (the -1 keeps it clear of the bottom hint
 * row, which isn't playable space.) */
static int off_screen(const Phenotype *p, int rows, int cols)
{
    return p->px < 0 || p->px >= cols ||
           p->py < 0 || p->py >= rows - 1;
}

/* fly one rocket forward by one tick: drop a trail dot, take this tick's nudge,
 * move, then check whether it reached the target or left the screen. */
static void rocket_tick(Rocket *r, float tx, float ty, float trad,
                        int rows, int cols)
{
    Phenotype *p  = &r->pheno;
    Trail     *tr = &r->trail;

    /* nothing to do once it's stopped flying */
    if (!p->alive) return;
    if (p->age >= LIFESPAN) { p->alive = 0; return; }

    trail_push(tr, p->px, p->py);

    apply_gene_with_velocity_cap(p, &r->genome.genes[p->age]);

    integrate_position(p);
    p->age++;

    /* reaching the target ends the flight (and earns the boost) */
    if (hit_target_ellipse(p, tx, ty, trad)) {
        p->hit_target = 1;
        p->alive      = 0;
        return;
    }

    /* leaving the screen ends the flight (and takes the cut) */
    if (off_screen(p, rows, cols)) {
        p->crashed = 1;
        p->alive   = 0;
    }
}

/* grade one finished flight (see §1.1).  We square the score at the end so a
 * small lead in closeness turns into a big lead in breeding chances — that's
 * what makes the population converge in dozens of generations, not hundreds. */
static float rocket_fitness(const Rocket *r, float tx, float ty)
{
    const Phenotype *p = &r->pheno;

    /* distance to the target (sideways squashed, §1.2) */
    float dx   = (p->px - tx) * CELL_ASPECT_X;
    float dy   = (p->py - ty);
    float dist = sqrtf(dx * dx + dy * dy);

    /* closer is better; the +1 keeps it from blowing up at distance 0 */
    float f = 1.0f / (dist + FITNESS_DIST_OFFSET);

    /* reward a hit, punish a crash (§1.1) */
    if (p->hit_target) f *= FITNESS_HIT_BONUS;
    if (p->crashed)    f *= FITNESS_CRASH_PENALTY;

    return f * f;
}

/* ── §6  ga — the population and the breeding cycle ──────────────────── */

/* Population — the whole flock of rockets the algorithm works on.  Evolution
 * needs a crowd: different rockets try different plans, and the best ones breed.
 * The array is sized for the biggest allowed population, so nothing is ever
 * allocated while running; only the first n slots are in use. */
typedef struct {
    Rocket rockets[POP_MAX];   /* room for the largest flock (only [0..n) used) */
    int    n;                  /* how many fly now (20..100; '[' / ']' change it,
                                * which restarts everyone from scratch) */
} Population;

/* Generation — where we are in the breeding cycle, plus two numbers for the HUD.
 * number counts breeding rounds; tick and n_hits track the current round.
 * best_fitness is LAST round's top score — shown while the new round is still
 * flying, because scores aren't known until a round finishes. */
typedef struct {
    int   number;        /* which breeding round we're on (0 at start) */
    int   tick;          /* which tick within this round, 0..LIFESPAN */
    int   n_hits;        /* how many rockets hit the target this round */
    float best_fitness;  /* best score from the PREVIOUS round (HUD only) */
} Generation;

/* GAParams — the one dial the user can turn while it runs: the mutation rate.
 * Too low and the flock converges early and gets stuck near the launch pad; too
 * high and it's all noise and never settles; around 0.01 it homes in over a few
 * dozen rounds.  Watching the HUD's "best:" climb as you change it is the lesson. */
typedef struct {
    float mutation_rate;  /* chance each nudge is randomised when breeding   *
                           * (0.001..0.10; '+' / '-' change it) */
} GAParams;

/* WorldGeom — the two fixed points: where rockets launch (bottom centre) and
 * where they're aiming (top centre).  Recomputed from the terminal size at
 * start and on resize, in the same cell coordinates the rockets fly in, so the
 * distance to the target is a plain subtraction. */
typedef struct {
    int target_x;   /* target column (centre of screen) */
    int target_y;   /* target row (near the top) */
    int launch_x;   /* launch column (centre of screen) */
    int launch_y;   /* launch row (near the bottom) */
} WorldGeom;

/* WorldUI — the on/off switches the keyboard flips.  None of these change the
 * algorithm or the scores — only the pace and the look.  fast mode runs a whole
 * round per frame, so you watch the flock learn in seconds instead of minutes. */
typedef struct {
    int paused;       /* 'p' — freeze the flight (drawing keeps going) */
    int fast;         /* 'f' — run a whole round per frame (watch it learn fast) */
    int show_trails;  /* 's' — show the comet-tail dots (on by default) */
    int theme;        /* 't' — which colour theme, 0..N_THEMES-1 */
} WorldUI;

/* Screen — the terminal's current size in characters.  The single source of
 * truth for how big everything is; set at start and on resize, never mid-frame.
 * A resize keeps the in-flight round going — only the target/launch positions
 * move to fit the new size. */
typedef struct {
    int cols;   /* width in characters  */
    int rows;   /* height in characters */
} Screen;

/* FrameTimer — keeps the loop at a steady ~30 fps and a smoothed fps for the
 * HUD.  There's no separate physics clock: the algorithm runs by tick COUNTS
 * (LIFESPAN ticks per round), not by wall-clock time, so a slow frame just makes
 * you wait a little longer — it can't glitch the flight. */
typedef struct {
    int64_t frame_ns;  /* how long one frame should take (1/TARGET_FPS) */
    int64_t fps_prev;  /* the clock reading at the previous frame */
    double  fps;       /* smoothed frames-per-second, shown in the HUD */
} FrameTimer;

/* Scene — the whole program's state in one place (a single file-scope instance,
 * g_scene).  It gathers the flock and breeding counters, the user's dial and
 * switches, the world geometry, the screen size, the frame timer, and two flags
 * the signal handlers set.  Nothing is allocated on the heap. */
typedef struct {
    Population            pop;          /* the flock of rockets */
    Generation            gen;          /* where we are in the breeding cycle */
    GAParams              ga;           /* the mutation-rate dial */
    WorldGeom             geom;         /* launch + target positions */
    WorldUI               ui;           /* keyboard on/off switches */
    Screen                screen;       /* terminal size */
    FrameTimer            timer;        /* frame pacing + fps */
    volatile sig_atomic_t running;      /* cleared by 'q' / Ctrl-C to quit */
    volatile sig_atomic_t need_resize;  /* set when the terminal is resized */
} Scene;

/* make a child's flight plan: take the first part from parent a and the rest
 * from parent b (the cut point is random, so siblings differ), then randomly
 * replace a few nudges with fresh random ones. */
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

/* scratch space kept at file scope — a whole generation of rockets is ~100 KB,
 * too big to put on the stack. */
static int    g_pool[POOL_CAPACITY];
static Rocket g_next[POP_MAX];

/* Step 1 of breeding — grade every rocket and return the best score (needed so
 * the lottery below can hand out tickets in proportion to it). */
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

/* Step 2 of breeding — fill a lottery bucket with tickets.  A better rocket
 * gets more tickets (everyone gets at least one), so drawing a random ticket
 * later naturally favours the good ones.  Returns how many tickets we filled. */
static int build_mating_pool(const Scene *s, float max_f)
{
    if (max_f <= 0.0f) max_f = 1.0f;   /* guard against divide-by-zero (shouldn't happen) */

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

/* Step 3 of breeding — make the next generation.  For each child: draw two
 * parents from the lottery, splice their plans into the child, and set it on
 * the launch pad. */
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

/* Step 4 of breeding — the children become the new flock; bump the round
 * number and clear the per-round counters. */
static void swap_in_children(Scene *s)
{
    memcpy(s->pop.rockets, g_next, sizeof(Rocket) * (size_t)s->pop.n);
    s->gen.number++;
    s->gen.tick   = 0;
    s->gen.n_hits = 0;
}

/* finish the current round and breed the next flock: grade everyone, fill the
 * lottery, draw parents and make children, then swap them in. */
static void ga_evolve(Scene *s)
{
    float max_f         = score_population(s);
    s->gen.best_fitness = max_f;

    int pool_size = build_mating_pool(s, max_f);
    breed_next_generation(s, pool_size);
    swap_in_children(s);
}

/* place the target (top centre) and launch pad (bottom centre) for the current
 * terminal size.  Called at start and again whenever the window is resized. */
static void scene_position(Scene *s, int rows, int cols)
{
    s->geom.target_x = cols / 2;
    s->geom.target_y = 3;
    s->geom.launch_x = cols / 2;
    s->geom.launch_y = rows - LAUNCH_OFFSET;
}

/* start over from scratch: give every rocket a brand-new random plan and reset
 * the round back to 0. */
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

/* advance the whole flock one tick.  Once the round's time runs out (or every
 * rocket has stopped flying), breed the next generation. */
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

/* ── §7  scene — draw the target, pad, trails, rockets, and HUD ──────── */

/* which still-flying rocket is closest to the target right now.  Recomputed
 * every frame so the highlighted leader keeps up.  -1 if every rocket has
 * already hit or crashed. */
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

/* is screen cell (sc, sr) inside the target?  Sideways distance is squashed by
 * half (§1.2), the same as the hit-test, so the round blob we draw and the spot
 * a rocket can hit are exactly the same shape. */
static int inside_target_ellipse(int sc, int sr, int tx, int ty)
{
    float dx = (sc - tx) * CELL_ASPECT_X;
    float dy = (sr - ty);
    return dx * dx + dy * dy <= TARGET_RADIUS * TARGET_RADIUS;
}

/* paint the target as a filled blob.  We make it twice as wide as tall so it
 * looks round on screen (cells are taller than wide), then walk that little box
 * and stamp '@' on every cell that falls inside. */
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

/* the launch pad: three characters under the spawn column, drawn in plain bold
 * (no theme colour) so it stays neutral whatever theme is active. */
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

/* how to draw a rocket: a plain '^' for the pack, but a bright '@' for the
 * current leader so the eye has something to follow. */
static void rocket_appearance(int is_lead, int *out_pair, char *out_glyph)
{
    *out_pair  = is_lead ? PAIR_BEST : PAIR_ROCKET;
    *out_glyph = is_lead ? '@'       : '^';
}

/* draw a rocket only if it's still flying or has reached the target (so a
 * success stays on screen).  Crashed ones disappear — only their trail dots
 * remain. */
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

/* the word shown at the right of the status line: paused, fast, or running. */
static const char *run_mode_label(const Scene *s)
{
    if (s->ui.paused) return "PAUSED ";
    if (s->ui.fast)   return "FAST   ";
    return "running";
}

/* build the top status line.  Generation and best score come first — those are
 * the two numbers you watch to see the flock improving. */
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

/* paint the status line flush-right on the top row. */
static void hud_paint_status(const char *buf, int cols)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* paint the bottom row that lists every key you can press. */
static void hud_paint_hint(int rows)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  r:reset  f:fast  s:trails  t:theme  "
             "[/]:pop  -/+:mut ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* draw one whole frame, back to front: clear the screen, then the target, the
 * launch pad, the trails (behind), the rockets (on top), and finally the HUD. */
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

/* ── §8  screen — start and stop ncurses ────────────────────────────── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);          /* stop ncurses peeking at input mid-draw (causes tearing) */
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §9  app — globals, signals, keys, and the main loop ─────────────── */

/* the one and only program state.  It's a global so the signal handlers can
 * reach the quit/resize flags; everything else reads it through here too. */
static Scene g_scene;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_scene.running     = 0;
    if (s == SIGWINCH)               g_scene.need_resize = 1;
}

/* ── §9.1 setup + per-step main-loop helpers ───────────────────────── */

/* one-time setup before the loop starts: hook up signals, set defaults, bring
 * ncurses up, place the world, seed the flock, and start the frame clock. */
static void scene_setup(void)
{
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    /* starting values the user can change later */
    g_scene.running          = 1;
    g_scene.pop.n            = POP_DEFAULT;
    g_scene.ga.mutation_rate = MUTATION_DEFAULT;
    g_scene.ui.theme         = 0;
    g_scene.ui.show_trails   = 1;

    /* start ncurses, read the screen size, place things, fill the flock */
    screen_init(g_scene.ui.theme);
    g_scene.screen.cols = COLS;
    g_scene.screen.rows = LINES;
    scene_position(&g_scene, g_scene.screen.rows, g_scene.screen.cols);
    scene_reseed(&g_scene);

    /* start the frame clock */
    g_scene.timer.frame_ns = NS_PER_SEC / TARGET_FPS;
    g_scene.timer.fps      = TARGET_FPS;
    g_scene.timer.fps_prev = clock_ns();
}

/* after the terminal is resized: tell ncurses the new size and re-place the
 * target and pad.  The round in progress keeps flying, untouched. */
static void scene_handle_resize(void)
{
    g_scene.need_resize = 0;
    endwin();
    refresh();
    g_scene.screen.rows = LINES;
    g_scene.screen.cols = COLS;
    scene_position(&g_scene, g_scene.screen.rows, g_scene.screen.cols);
}

/* run the simulation for this frame: one tick normally, or a whole round in
 * fast mode.  We stop early when a round ends (the tick counter resets to 0) so
 * you actually see the fresh start before the next burst. */
static void scene_advance_sim_burst(void)
{
    if (g_scene.ui.paused) return;
    int steps = g_scene.ui.fast ? LIFESPAN : 1;
    for (int i = 0; i < steps; i++) {
        scene_step(&g_scene, g_scene.screen.rows, g_scene.screen.cols);
        if (g_scene.gen.tick == 0) break;
    }
}

/* update the smoothed fps: mostly keep the old value, blend in a little of the
 * latest reading.  The +1 avoids dividing by zero on a frame that took no
 * measurable time. */
static void frame_update_ewma_fps(int64_t now)
{
    int64_t dt_ns = now - g_scene.timer.fps_prev + 1;
    double  instant = 1e9 / (double)dt_ns;
    g_scene.timer.fps = g_scene.timer.fps * EWMA_RETAIN
                      + instant            * EWMA_NEW;
    g_scene.timer.fps_prev = now;
}

/* sleep whatever time is left in this frame's budget, to hold the target rate. */
static void frame_cap_to_target_fps(int64_t frame_start)
{
    clock_sleep_ns(g_scene.timer.frame_ns - (clock_ns() - frame_start));
}

/* ── §9.2 keyboard action handlers ─────────────────────────────────── */

/* 'p' / 'f' / 's' — flip one on/off switch. */
static void key_toggle_pause(void)  { g_scene.ui.paused      ^= 1; }
static void key_toggle_fast(void)   { g_scene.ui.fast        ^= 1; }
static void key_toggle_trails(void) { g_scene.ui.show_trails ^= 1; }

/* 'r' — start over with brand-new random plans. */
static void key_reset_simulation(void) { scene_reseed(&g_scene); }

/* 't' — switch to the next colour theme. */
static void key_cycle_theme(void)
{
    g_scene.ui.theme = (g_scene.ui.theme + 1) % N_THEMES;
    color_init(g_scene.ui.theme);
}

/* '+' / '-' — raise or lower the mutation rate (kept within its allowed range). */
static void key_mutation_nudge(float delta)
{
    float v = g_scene.ga.mutation_rate + delta;
    if (v < MUTATION_MIN || v > MUTATION_MAX) return;
    g_scene.ga.mutation_rate = v;
}

/* '[' / ']' — grow or shrink the flock (kept within range).  Changing the size
 * restarts everyone from scratch. */
static void key_population_nudge(int delta)
{
    int n = g_scene.pop.n + delta;
    if (n < POP_MIN || n > POP_MAX) return;
    g_scene.pop.n = n;
    scene_reseed(&g_scene);
}

/* send one key press to its action — the switch is basically the key map. */
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

/* handle every key press waiting in the queue. */
static void scene_drain_input(void)
{
    int ch;
    while ((ch = getch()) != ERR) scene_handle_one_keystroke(ch);
}

/* ── §9.3 main — set up once, then loop every frame ──────────────────── *
 * Each frame: handle a pending resize, read the keys, advance the simulation
 * (unless paused), refresh the fps, draw, then sleep to hold the frame rate. */
int main(void)
{
    scene_setup();

    while (g_scene.running) {
        if (g_scene.need_resize) scene_handle_resize();

        scene_drain_input();

        /* one tick, or a whole round in fast mode */
        scene_advance_sim_burst();

        int64_t now = clock_ns();
        frame_update_ewma_fps(now);

        scene_draw(g_scene.screen.rows, g_scene.screen.cols,
                   &g_scene, g_scene.timer.fps);

        frame_cap_to_target_fps(now);
    }
    return 0;
}
