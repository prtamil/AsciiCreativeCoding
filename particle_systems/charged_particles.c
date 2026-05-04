/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * charged_particles.c — Coulomb force simulation: like-charges repel,
 *                       opposites attract. Pair-wise inverse-square law.
 *
 * DEMO: A handful of charged particles attract and repel each other
 *       under Coulomb's law `F = k · q₁·q₂ / r²`. Like-charged
 *       particles fly apart; opposite-charged particles orbit, slingshot,
 *       collide softly, and form pairs. Each particle leaves a short
 *       trail showing its recent path so orbits and ejections are
 *       visible against the background. Charge sign + magnitude maps
 *       to a bipolar colour ramp: deep blue at most-negative, white
 *       at neutral, deep red at most-positive.
 *
 *       Patterns:
 *         BINARY        2 particles (+1, −1) — clean orbit (mini "atom")
 *         TRINARY       3 particles (+1, +1, −2) — chaotic 3-body dance
 *         DIPOLE_FIELD  2 fixed heavy charges (+5, −5) + 30 free
 *                       test particles drifting through their field
 *         RANDOM_GAS    40 particles with random charges from
 *                       {−2,−1,+1,+2}; emergent clumping and ejections
 *
 * Study alongside:
 *   physics/barnes_hut.c — same N-body integration but with Newton
 *                           gravity (always attractive). Charged uses
 *                           Coulomb, so signs matter.
 *   vortex.c             — also uses force-field motion (radial
 *                           inflow + tangential), but with a single
 *                           central attractor instead of N pairs.
 *
 * Section map:
 *   §1 config    — constants, themes, per-pattern parameters
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — bipolar 8-step charge ramp + trail pair
 *   §4 particle  — Particle struct (physical coords + trail buffer)
 *   §5 scene     — pool + per-pattern init + N² force tick + draw
 *   §6 screen    — ncurses init / draw / resize
 *   §7 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (re-init pattern with new RNG)
 *   n / N      next pattern   (BINARY → TRINARY → DIPOLE → RANDOM_GAS)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/charged_particles.c \
 *       -o charged_particles -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Naive O(N²) Coulomb-force integration. Each tick:
 *                  for every (non-fixed) particle i, sum the force from
 *                  every other particle j:
 *
 *                    F_ij = k · q_i · q_j / r² · (i − j) / r
 *                         = -k · q_i · q_j · (j - i) / r³
 *
 *                  with `r = |j - i|`. Like charges (q_i·q_j > 0) →
 *                  force points AWAY from j (repulsion). Opposite
 *                  charges (q_i·q_j < 0) → force points TOWARD j
 *                  (attraction).
 *
 *                  After summing forces: explicit Euler:
 *                    v += (F / m) · dt
 *                    v *= damping_factor
 *                    p += v · dt
 *
 *                  Velocities damp slightly each tick (drag) so the
 *                  numerical-integration energy creep doesn't blow the
 *                  particles off the screen. SOFTENING (`r² := max(r², R_SOFT²)`)
 *                  prevents the singularity at r → 0 from producing
 *                  infinite force when two opposite-charge particles
 *                  approach each other closely.
 *
 *                  Boundary: elastic bounce at the screen edges with
 *                  small restitution loss so very-fast particles slow
 *                  on impact rather than amplifying.
 *
 *                  Per-particle TRAIL: a small ring buffer of past
 *                  positions, drawn dimmest-first behind the head.
 *                  Lets you SEE orbits without static visual aids
 *                  like field-line overlays.
 *
 *                  Patterns differ only in initial conditions
 *                  (positions, velocities, charges, fixed-flag) —
 *                  the integrator is identical for all four.
 *
 *                  All physics runs in PHYSICAL coordinates `(px, py)`,
 *                  with `py = cell_y · ASPECT_Y`, so the inverse-square
 *                  law is isotropic in physical space. At render time
 *                  we convert back to cell coords. Without this the
 *                  orbits would be ellipses (terminal cells are 2:1
 *                  taller than wide).
 *
 * Data-structure : Particle[MAX_PARTICLES] with `active` flag.
 *                  Each particle owns a small ring buffer of past
 *                  positions for the trail. Linear scan everywhere —
 *                  N is small so O(N²) is fine.
 *
 * Rendering      : ASCII only. Particle glyph: `+` for q > 0, `-` for
 *                  q < 0, `.` for neutral. Heavy fixed charges use
 *                  `@`. Colour by charge sign + magnitude — bipolar
 *                  8-step ramp where slot 0 = most negative, slot 7
 *                  = most positive, slot 3-4 = neutral white. Trail
 *                  cells dim with age.
 *
 * Performance    : O(N² · TICK_HZ). At N = 40 (RANDOM_GAS) with
 *                  60 fps that's 96k pair forces / sec — trivial.
 *                  Each force is ~10 mul + 1 sqrt. The Barnes-Hut
 *                  trick (`physics/barnes_hut.c`) would buy us
 *                  O(N log N) but isn't needed here.
 *
 * References     :
 *   • Wikipedia — [Coulomb's law](https://en.wikipedia.org/wiki/Coulomb%27s_law).
 *     The exact `k · q₁·q₂ / r²` formula this simulation implements.
 *   • Hockney, R. W. & Eastwood, J. W. (1988) — *Computer Simulation
 *     Using Particles*. Foundational text on N-body force integration.
 *   • Wikipedia — [Three-body problem](https://en.wikipedia.org/wiki/Three-body_problem).
 *     The TRINARY pattern is the electrostatic version — chaotic
 *     for almost all initial conditions.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every charged particle pulls or pushes every other charged
 * particle. The strength of the pull is `k · q₁·q₂ / r²` and the
 * direction is along the line joining them. Sum all those pulls,
 * divide by mass, integrate. That's the entire simulation. Patterns
 * differ ONLY in what particles you start with.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture marbles with magnets stuck to them. North-North repels.
 * South-South repels. North-South attracts. Strength falls off with
 * distance squared, so two marbles right next to each other feel a
 * massive pull, while the same marbles across the room barely
 * notice each other. With two opposite-charge marbles, give them a
 * sideways shove and they orbit each other forever (in a vacuum).
 * With three same-charge marbles, they all fly apart explosively.
 * With many random charges, you get a slow chaotic dance — pairs
 * form, others get ejected, sometimes a triple briefly orbits.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT — set up the particles per the active pattern. Some are
 *     `fixed = true` (they don't move; they only PRODUCE field).
 *
 *  2. EACH TICK, for each non-fixed particle i:
 *       fx = fy = 0
 *       for j ≠ i:
 *         dx = j.px − i.px
 *         dy = j.py − i.py
 *         r²  = dx² + dy²
 *         r²  = max(r², R_SOFT²)               // softening
 *         r   = √r²
 *         f   = -k · q_i · q_j / r³            // signed; negative
 *                                              // sign baked into formula
 *                                              // means attract toward j
 *                                              // when q_i·q_j < 0
 *         fx += f · dx
 *         fy += f · dy
 *       i.vx += fx / m_i · dt
 *       i.vy += fy / m_i · dt
 *
 *  3. DAMP + INTEGRATE:
 *       i.vx *= exp(-DAMP · dt)
 *       i.vy *= exp(-DAMP · dt)
 *       i.px += i.vx · dt
 *       i.py += i.vy · dt
 *
 *  4. BOUNCE off boundaries with restitution:
 *       if px out of range: reflect vx, vx *= BOUNCE_R
 *       if py out of range: reflect vy, vy *= BOUNCE_R
 *
 *  5. PUSH current (px, py) into trail ring buffer.
 *
 *  6. RENDER trail cells then particle head.
 *
 * KEY FORMULAS
 * ────────────
 *  Coulomb force on i from j:
 *    F = k · q_i · q_j / r² · unit(i − j)
 *      = -k · q_i · q_j / r³ · (j − i)
 *
 *  Circular orbit speed for two equal-mass opposite charges at
 *  distance d (each orbits common centre at d/2):
 *    v = √(k / (2 · m · d))
 *
 *  Charge-to-ramp-slot (bipolar, [-2, +2] mapped to [0, 7]):
 *    slot = round(3.5 + q · 1.5)
 *    slot ∈ [0, 7]
 *
 *  Aspect-correct physical coords:
 *    px = cell_x
 *    py = cell_y · ASPECT_Y     (ASPECT_Y = 2)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SINGULARITY AT r → 0. Without softening, two opposite-charge
 *    particles approaching each other get F → ∞ → infinite velocity
 *    → the simulation explodes. We clamp `r²` to `R_SOFT²` (typically
 *    R_SOFT = 2.5 cells) so the closest pair gets a large but finite
 *    force. They appear to "bounce off each other" — physically not
 *    the same as collision but visually OK.
 *
 *  • EULER ENERGY DRIFT. Explicit Euler integration is not
 *    symplectic — it adds energy over time for orbits. A small
 *    velocity damping (`exp(-0.15 · dt)` per tick) compensates.
 *    For long-term orbits, use Verlet or RK4; for visual demos,
 *    Euler + damping is fine.
 *
 *  • LIKE-CHARGES FLY OFF FOREVER. With repulsion only, particles
 *    accelerate toward the screen edges and bounce back. Damping
 *    + bounce restitution prevents runaway energy.
 *
 *  • FIXED PARTICLES. DIPOLE_FIELD has 2 particles with `fixed = true`.
 *    They DO contribute to the force on others but their own
 *    velocity is never updated (effectively infinite mass).
 *
 *  • BOUNCE LOSES ENERGY. Each wall bounce multiplies velocity by
 *    `BOUNCE_R = 0.85`. Combined with damping, particles eventually
 *    settle into the lowest-energy configuration (paired opposite
 *    charges drifting at low speed).
 *
 *  • ASPECT IN FORCE. We work in physical coords so the inverse-
 *    square law is correct in the visual plane. Without this,
 *    particles would orbit as ellipses (cell-y is twice as "long"
 *    as cell-x in visual terms, so a "circular" orbit in cell coords
 *    would render as a vertical ellipse).
 *
 *  • PAUSE. Skip force calc + integration when paused. Trail
 *    buffers don't advance either, so the trail freezes too.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Particles freeze in place. Resume: motion
 *    continues from where it stopped.
 *
 *  • BINARY. Two particles orbit each other. The orbit is roughly
 *    circular (within Euler's energy drift). Trails show clean
 *    ellipses. Each orbit takes a few seconds.
 *
 *  • TRINARY. Three particles dance chaotically — sometimes a pair
 *    forms briefly while the third gets slingshot away to bounce
 *    off a wall. Reseed (`r`) with different initial positions to
 *    see different chaotic outcomes.
 *
 *  • DIPOLE_FIELD. Two large fixed charges at the screen centre-
 *    left and centre-right. Free test particles drift through the
 *    field — repelled by like charges, attracted to opposites.
 *    Watch a + particle accelerate into the − fixed charge and
 *    bounce off (softening prevents collision); a − particle
 *    smoothly orbits between the two fixed charges.
 *
 *  • RANDOM_GAS. 40 particles with random charges. Watch the
 *    emergent behaviour: pairs form, get ejected, fly apart, re-
 *    capture. Reseed for a fresh run.
 *
 *  • Theme cycle (`t`/`T`). The bipolar palette shifts: ICE_FIRE
 *    has cold blue for negatives and hot red for positives; AURORA
 *    has green negatives and pink positives; etc.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    SPEED_MIN        =   1,
    SPEED_DEF        =   8,
    SPEED_MAX        =  64,

    MAX_PARTICLES    =  80,
    TRAIL_LEN        =   5,    /* per-particle position history       */

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_RAMP_BASE   =   3,    /* +0..+7 = bipolar charge palette     */
    PAIR_TRAIL       =  11,
    PAIR_SKY         =  12,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y          2.0f       /* terminal cells 2× taller       */

/* Coulomb / integrator constants. */
#define COULOMB_K         8000.0f
#define R_SOFT_MIN        2.5f       /* softening radius (cells)       */
#define VELOCITY_DAMP     0.15f      /* per-second exp damping         */
#define BOUNCE_REST       0.85f      /* boundary bounce restitution    */
#define MAX_VELOCITY      400.0f     /* cells/sec hard cap (safety)    */

/* Pattern enum. */
typedef enum {
    PATTERN_BINARY       = 0,
    PATTERN_TRINARY      = 1,
    PATTERN_DIPOLE_FIELD = 2,
    PATTERN_RANDOM_GAS   = 3,
    N_PATTERNS           = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_BINARY:       return "BINARY      ";
    case PATTERN_TRINARY:      return "TRINARY     ";
    case PATTERN_DIPOLE_FIELD: return "DIPOLE_FIELD";
    case PATTERN_RANDOM_GAS:   return "RANDOM_GAS  ";
    default:                   return "?           ";
    }
}

/*
 * Themes — bipolar 8-step ramp:
 *   slot 0..3: NEGATIVE charge (deepest → light)
 *   slot 4..7: POSITIVE charge (light → deepest)
 *   slot 3-4 ≈ neutral
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       ramp[8];
    short       trail;
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        ramp[0..7]  (most-neg → neutral → most-pos)         trail sky */

    { "DEFAULT",  {  21,  33,  45, 195, 224, 217, 209, 196 },           246,  234 },
    { "ICE_FIRE", {  17,  19,  39,  87, 224, 209, 202, 196 },           245,  233 },
    { "VOLT",     {  18,  21,  39, 117, 230, 220, 214, 196 },           246,  234 },
    { "AURORA",   {  22,  29,  79, 159, 224, 213, 207, 199 },           245,  234 },
    { "VIOLET",   {  53,  90, 134, 213, 224, 223, 215, 196 },           245,  234 },
    { "CYBER",    {  22,  29, 121, 195, 230, 219, 207, 197 },           245,  234 },
    { "NEON",     {  27,  39,  87, 195, 230, 213, 207, 199 },           245,  234 },
    { "COPPER",   {  24,  31,  67, 195, 230, 222, 215, 130 },           246,  234 },
    { "MONO",     { 240, 243, 245, 247, 249, 251, 253, 255 },           246,  232 },
    { "PASTEL",   { 110, 117, 153, 195, 224, 217, 218, 211 },           246,  234 },
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_TRAIL, t->trail, -1);
        init_pair(PAIR_SKY,   t->sky,   -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,  COLOR_BLUE,   COLOR_CYAN,   COLOR_WHITE,
            COLOR_WHITE, COLOR_YELLOW, COLOR_RED,    COLOR_RED,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
        init_pair(PAIR_TRAIL, COLOR_WHITE, -1);
        init_pair(PAIR_SKY,   COLOR_BLACK, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* Map charge ∈ [-2, +2] → ramp slot [0, 7]. */
static inline int charge_to_slot(float q)
{
    int slot = (int)(3.5f + q * 1.5f + 0.5f);
    if (slot < 0) slot = 0;
    if (slot > 7) slot = 7;
    return slot;
}

/* ===================================================================== */
/* §4  particle                                                           */
/* ===================================================================== */

typedef struct {
    float px, py;       /* position in PHYSICAL coords (py = cell·ASPECT_Y) */
    float vx, vy;       /* velocity in physical coords / second             */
    float charge;       /* signed charge magnitude                          */
    float mass;         /* inertial mass (typically 1)                      */
    bool  fixed;        /* true = does not move (DIPOLE_FIELD anchors)      */
    bool  active;

    /* Per-particle trail ring buffer (physical coords). */
    float trail_px[TRAIL_LEN];
    float trail_py[TRAIL_LEN];
    int   trail_head;
    int   trail_count;
} Particle;

/* Cheap LCG */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}
static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §5  scene — pool, init per pattern, force tick, draw                  */
/* ===================================================================== */

typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    uint32_t  rng;
    int       rows, cols;

    int       n_particles;       /* number of active slots                 */
    Particle  particles[MAX_PARTICLES];
} Scene;

static void scene_clear_particles(Scene *s)
{
    s->n_particles = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        s->particles[i].active     = false;
        s->particles[i].fixed      = false;
        s->particles[i].trail_head = 0;
        s->particles[i].trail_count = 0;
    }
}

/* Append a particle to the pool; returns its index or -1 on full. */
static int scene_add_particle(Scene *s, float px, float py,
                              float vx, float vy,
                              float charge, float mass, bool fixed)
{
    if (s->n_particles >= MAX_PARTICLES) return -1;
    int idx = s->n_particles++;
    Particle *p = &s->particles[idx];
    p->px = px;
    p->py = py;
    p->vx = vx;
    p->vy = vy;
    p->charge = charge;
    p->mass = mass < 1e-3f ? 1e-3f : mass;
    p->fixed = fixed;
    p->active = true;
    p->trail_head = 0;
    p->trail_count = 0;
    /* Pre-fill trail with current position so first frames don't show
     * a long line trailing from (0, 0). */
    for (int t = 0; t < TRAIL_LEN; t++) {
        p->trail_px[t] = px;
        p->trail_py[t] = py;
    }
    return idx;
}

/*
 * Per-pattern initialisation. Each clears the pool, then populates it
 * with the pattern-specific configuration.
 */
static void scene_init_pattern(Scene *s)
{
    scene_clear_particles(s);

    int rows_eff = s->rows - 1;
    float cx = (float)s->cols * 0.5f;
    float cy = (float)rows_eff * 0.5f * ASPECT_Y;

    switch (s->current_pattern) {

    case PATTERN_BINARY: {
        /* Two equal-magnitude opposite charges in stable circular orbit.
         * Orbital speed for circular orbit at distance d (each orbits
         * common centre at d/2):  v = sqrt(k / (2·m·d))               */
        float d = 25.0f;
        float v = sqrtf(COULOMB_K / (2.0f * 1.0f * d));
        scene_add_particle(s, cx - d * 0.5f, cy, 0.0f, -v, +1.0f, 1.0f, false);
        scene_add_particle(s, cx + d * 0.5f, cy, 0.0f, +v, -1.0f, 1.0f, false);
        break;
    }

    case PATTERN_TRINARY: {
        /* Three particles at vertices of an equilateral triangle.
         * Charges (+1, +1, -2) — chaotic 3-body dynamics.            */
        float radius = 18.0f;
        float v = 0.6f * sqrtf(COULOMB_K / (3.0f * radius));
        for (int k = 0; k < 3; k++) {
            float angle = (float)k * 2.0f * (float)M_PI / 3.0f;
            float pcx = cx + radius * cosf(angle);
            float pcy = cy + radius * sinf(angle);
            float vx  = -v * sinf(angle);
            float vy  =  v * cosf(angle);
            float q   = (k == 2) ? -2.0f : +1.0f;
            scene_add_particle(s, pcx, pcy, vx, vy, q, 1.0f, false);
        }
        break;
    }

    case PATTERN_DIPOLE_FIELD: {
        /* Two heavy fixed dipole charges + 30 free test particles. */
        float dipole_d = (float)s->cols * 0.45f;
        scene_add_particle(s, cx - dipole_d * 0.5f, cy, 0, 0, +5.0f, 1e9f, true);
        scene_add_particle(s, cx + dipole_d * 0.5f, cy, 0, 0, -5.0f, 1e9f, true);

        for (int k = 0; k < 30; k++) {
            float r1 = lcg_unit(&s->rng);
            float r2 = lcg_unit(&s->rng);
            float r3 = lcg_unit(&s->rng);
            float r4 = lcg_unit(&s->rng);
            float r5 = lcg_unit(&s->rng);
            float pcx = r1 * (float)s->cols;
            float pcy = r2 * (float)rows_eff * ASPECT_Y;
            float vx  = (r3 - 0.5f) * 8.0f;
            float vy  = (r4 - 0.5f) * 8.0f;
            float q   = (r5 < 0.5f) ? -1.0f : +1.0f;
            scene_add_particle(s, pcx, pcy, vx, vy, q, 1.0f, false);
        }
        break;
    }

    case PATTERN_RANDOM_GAS: {
        /* 40 particles, random positions, charges from {-2, -1, +1, +2}. */
        for (int k = 0; k < 40; k++) {
            float r1 = lcg_unit(&s->rng);
            float r2 = lcg_unit(&s->rng);
            float r3 = lcg_unit(&s->rng);
            float r4 = lcg_unit(&s->rng);
            float r5 = lcg_unit(&s->rng);

            /* Spawn within central area to avoid edge crowding. */
            float pcx = (float)s->cols * (0.10f + r1 * 0.80f);
            float pcy = (float)rows_eff * ASPECT_Y * (0.10f + r2 * 0.80f);
            float vx  = (r3 - 0.5f) * 8.0f;
            float vy  = (r4 - 0.5f) * 8.0f;
            float q;
            if      (r5 < 0.25f) q = -2.0f;
            else if (r5 < 0.50f) q = -1.0f;
            else if (r5 < 0.75f) q = +1.0f;
            else                 q = +2.0f;
            scene_add_particle(s, pcx, pcy, vx, vy, q, 1.0f, false);
        }
        break;
    }

    case N_PATTERNS: break;
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_BINARY;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    scene_init_pattern(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    scene_init_pattern(s);
}

/*
 * scene_tick — N² Coulomb force integration.
 *
 *  1. For each non-fixed particle, sum forces from all others.
 *  2. Update velocities (v += a · dt with damping + cap).
 *  3. Update positions (p += v · dt) with boundary bounce.
 *  4. Push current position into trail ring buffer.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    int rows_eff = s->rows - 1;
    float r_soft2 = R_SOFT_MIN * R_SOFT_MIN;

    /* 1 + 2: forces and velocity update. */
    for (int i = 0; i < s->n_particles; i++) {
        Particle *pi = &s->particles[i];
        if (!pi->active || pi->fixed) continue;

        float fx = 0.0f, fy = 0.0f;
        for (int j = 0; j < s->n_particles; j++) {
            if (i == j) continue;
            const Particle *pj = &s->particles[j];
            if (!pj->active) continue;

            float dx  = pj->px - pi->px;
            float dy  = pj->py - pi->py;
            float r2  = dx * dx + dy * dy;
            if (r2 < r_soft2) r2 = r_soft2;
            float r   = sqrtf(r2);

            /* F_on_i = -k · q_i · q_j / r³ · (j − i)
             * Like charges (q_i·q_j > 0) → factor negative → force
             * points opposite to (j − i) → away from j (repulsion).
             * Opposite charges → factor positive → force toward j. */
            float factor = -COULOMB_K * pi->charge * pj->charge / (r2 * r);
            fx += factor * dx;
            fy += factor * dy;
        }

        pi->vx += fx / pi->mass * dt;
        pi->vy += fy / pi->mass * dt;
    }

    /* 3: damping + position update + boundary bounce. */
    float damp = expf(-VELOCITY_DAMP * dt);
    float py_max = (float)(rows_eff - 1) * ASPECT_Y;

    for (int i = 0; i < s->n_particles; i++) {
        Particle *p = &s->particles[i];
        if (!p->active || p->fixed) continue;

        p->vx *= damp;
        p->vy *= damp;

        /* Velocity safety cap. */
        float vmag2 = p->vx * p->vx + p->vy * p->vy;
        if (vmag2 > MAX_VELOCITY * MAX_VELOCITY) {
            float scale = MAX_VELOCITY / sqrtf(vmag2);
            p->vx *= scale;
            p->vy *= scale;
        }

        p->px += p->vx * dt;
        p->py += p->vy * dt;

        /* Bounce off screen edges (with restitution loss). */
        if (p->px < 1.0f) {
            p->px = 1.0f;
            p->vx = -p->vx * BOUNCE_REST;
        } else if (p->px > (float)(s->cols - 2)) {
            p->px = (float)(s->cols - 2);
            p->vx = -p->vx * BOUNCE_REST;
        }
        if (p->py < ASPECT_Y) {
            p->py = ASPECT_Y;
            p->vy = -p->vy * BOUNCE_REST;
        } else if (p->py > py_max) {
            p->py = py_max;
            p->vy = -p->vy * BOUNCE_REST;
        }
    }

    /* 4: push current position into trail ring buffer. */
    for (int i = 0; i < s->n_particles; i++) {
        Particle *p = &s->particles[i];
        if (!p->active) continue;
        p->trail_head = (p->trail_head + 1) % TRAIL_LEN;
        p->trail_px[p->trail_head] = p->px;
        p->trail_py[p->trail_head] = p->py;
        if (p->trail_count < TRAIL_LEN) p->trail_count++;
    }
}

/*
 * scene_draw — trails first (oldest → newest, dimmest → brightest),
 * then particle heads with charge-coloured glyph.
 */
static void scene_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;

    /* ── Trail cells ─────────────────────────────────────────────── */
    for (int i = 0; i < s->n_particles; i++) {
        const Particle *p = &s->particles[i];
        if (!p->active || p->fixed) continue;     /* fixed: no trail */

        int n = p->trail_count;
        for (int k = 0; k < n - 1; k++) {
            /* k = 0 is the OLDEST; n-1 is most recent (don't draw,
             * head will). */
            int idx = (p->trail_head + 1 + k) % TRAIL_LEN;
            float tpx = p->trail_px[idx];
            float tpy = p->trail_py[idx];
            int ix = (int)(tpx + 0.5f);
            int iy = (int)(tpy / ASPECT_Y + 0.5f);
            if (ix < 0 || ix >= s->cols) continue;
            if (iy < 0 || iy >= rows_eff) continue;

            int attr = (k >= n - 2) ? A_NORMAL : A_DIM;
            char glyph = (k >= n - 2) ? '.' : '`';
            attron(COLOR_PAIR(PAIR_TRAIL) | attr);
            mvaddch(iy, ix, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(PAIR_TRAIL) | attr);
        }
    }

    /* ── Particle heads ──────────────────────────────────────────── */
    for (int i = 0; i < s->n_particles; i++) {
        const Particle *p = &s->particles[i];
        if (!p->active) continue;
        int ix = (int)(p->px + 0.5f);
        int iy = (int)(p->py / ASPECT_Y + 0.5f);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < 0 || iy >= rows_eff) continue;

        char glyph;
        int  attr;
        if (p->fixed) {
            glyph = '@';
            attr  = A_BOLD;
        } else {
            if      (p->charge > 0.5f)  glyph = '+';
            else if (p->charge < -0.5f) glyph = '-';
            else                        glyph = '.';
            attr = (fabsf(p->charge) > 1.5f) ? A_BOLD : A_NORMAL;
        }

        int slot = charge_to_slot(p->charge);
        int pair = PAIR_RAMP_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize_curses(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    const char *state_str = s->paused ? "PAUSED      " : pattern_name(s->current_pattern);

    char buf[200];
    snprintf(buf, sizeof buf,
             " CHARGED   %s   theme:%-8s   N:%2d   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name,
             s->n_particles, fps, sim_fps, s->speed);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                              break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        scene_init_pattern(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_init_pattern(s);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
