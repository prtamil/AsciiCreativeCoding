/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * charged_particles.c — Coulomb force simulation: like-charges repel,
 *                       opposites attract. Pair-wise inverse-square law.
 *
 * DEMO: A handful of charged particles attract and repel each other
 *       under Coulomb's law `F = k · q₁·q₂ / r²`. Like-charged
 *       particles fly apart; opposite-charged particles orbit,
 *       slingshot, collide softly, and form pairs.  Each particle
 *       leaves a 30-frame charge-coloured trail with a 5-tier glyph
 *       fade (* + : . `) so orbits and ejections read as comet-like
 *       streaks.  Free particle heads carry a 4-cell glow halo;
 *       wall bounces produce a brief '*' bold splash; fast or heavy
 *       particles are speed-tinted A_BOLD.  Charge sign + magnitude
 *       maps to a bipolar colour ramp: slot 0 = most-negative,
 *       slot 7 = most-positive, slot 3-4 ≈ neutral.
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
 *   t / T      next / previous theme  (12 bipolar palettes —
 *              every theme is di- or tri-colour so the sign of
 *              charge always reads visually:
 *                VOLT     COPPER    NEON      ICE_FIRE
 *                AURORA   VIOLET    CYBER     PASTEL
 *                TWILIGHT SODIUM    ECLIPSE   MONO
 *              VOLT is the default; MONO is the grayscale
 *              reference.)
 *   + / =      faster
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * HUD: canonical CLAUDE.md two-bar — row 0 right shows live status
 * (pattern, theme, N, speed, paused/running, fps, Hz); row rows-1
 * lists the action keys.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/charged_particles.c \
 *       -o charged_particles -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Naive O(N²) Coulomb-force integration [3].  Each tick:
 *                  for every (non-fixed) particle i, sum the force from
 *                  every other particle j:
 *
 *                    F_ij = k · q_i · q_j / r² · (i − j) / r          [1]
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
 *                  numerical-integration energy creep [4] doesn't blow
 *                  the particles off the screen. SOFTENING (`r² := max(r², R_SOFT²)`)
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
 *                  the integrator is identical for all four.  The
 *                  TRINARY pattern is the electrostatic analogue of
 *                  the classical three-body problem [2] and is chaotic
 *                  for almost all initial conditions.
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
 *                  `@`. Colour by charge sign + magnitude — a BIPOLAR
 *                  8-step diverging palette [5] where slot 0 = most-
 *                  negative, slot 7 = most-positive, slot 3-4 ≈ neutral.
 *                  Trails are charge-coloured with a 5-tier glyph
 *                  fade (* + : . `).  Free particle heads carry a
 *                  4-cell glow halo; wall bounces produce a bright
 *                  '*' splash that fades over BOUNCE_FLASH_FRAMES
 *                  ticks; fast or heavy particles get A_BOLD.
 *
 * Performance    : O(N² · TICK_HZ). At N = 40 (RANDOM_GAS) with
 *                  60 fps that's 96k pair forces / sec — trivial.
 *                  Each force is ~10 mul + 1 sqrt. The Barnes-Hut
 *                  trick (`physics/barnes_hut.c`) would buy us
 *                  O(N log N) but isn't needed here.
 *
 * References (cite inline as [n]):
 *
 *   [1] Griffiths, D. J. — *Introduction to Electrodynamics*, 4th ed.,
 *       Cambridge Univ. Press (2017).  Ch. 2 develops Coulomb's law
 *       F = k · q₁·q₂ / r² in full vector form, plus the
 *       superposition principle this simulation uses to sum pairwise
 *       forces.  Foundational physics for §5 (scene_tick).
 *
 *   [2] Goldstein, H.; Poole, C.; Safko, J. — *Classical Mechanics*,
 *       3rd ed., Addison-Wesley (2002).  §3 (two-body central forces)
 *       gives the circular-orbit speed v = √(k / (2·m·d)) used to seed
 *       BINARY.  §11 (small oscillations / chaotic systems) backs the
 *       qualitative behaviour of TRINARY (sensitive dependence on
 *       initial conditions, no closed-form solution).
 *
 *   [3] Hockney, R. W. & Eastwood, J. W. — *Computer Simulation Using
 *       Particles*, McGraw-Hill (1981, reissued IOP 1988).  The
 *       canonical text on N-body force integration: pair-summed O(N²)
 *       evaluation, softening to avoid the r → 0 singularity, particle
 *       pooling, and energy-diagnostics for sanity-checking the
 *       integrator.  Backs every choice in scene_tick / scene_init_pattern.
 *
 *   [4] Hairer, E.; Lubich, C.; Wanner, G. — *Geometric Numerical
 *       Integration*, 2nd ed., Springer (2006).  Explains why explicit
 *       Euler is NOT symplectic (the integrator we use), why orbits
 *       systematically GAIN energy each step, and why a small velocity
 *       damping is a cheap way to compensate for visual demos.
 *
 *   [5] Ware, C. — *Information Visualization: Perception for Design*,
 *       4th ed., Morgan Kaufmann (2020).  Ch. 4 covers DIVERGING colour
 *       ramps for bipolar signed data — exactly the slot 0..7 ramp
 *       used here, with neutral in the middle and saturated opposing
 *       hues at either end.  Backs the 10 theme palettes in §1.
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
    TRAIL_LEN        =  30,    /* per-particle position history —
                                * longer = more visible orbit/streak  */
    BOUNCE_FLASH_FRAMES = 6,   /* head flashes '*' bold for N frames
                                * after a wall bounce                 */
    HUD_TOP          =   1,    /* canonical two-bar HUD: row 0 status */
    HUD_BOT          =   1,    /*                       row rows-1 hint */

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
#define VBOLD_THRESHOLD    60.0f     /* |v| ≥ this → particle head is
                                      * drawn A_BOLD (speed-tinted)    */

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

#define N_THEMES 12

/*
 * Every theme is a BIPOLAR (di-color or tri-color) gradient: slot 0 =
 * most-negative charge in one hue family, slot 7 = most-positive in
 * the OPPOSITE hue family, slots 3-4 the bright/neutral midpoint.
 * Single-hue (uni-colour) ramps would lose the sign of charge under
 * eye-fatigue — diverging palettes are the canonical choice for
 * bipolar signed data [5].
 *
 * Two flavours of ramp:
 *   DI-COLOUR  cool half → bright midpoint → warm half        (COPPER,
 *              ICE_FIRE, AURORA, VIOLET, PASTEL, ECLIPSE,
 *              TWILIGHT, SODIUM)
 *   TRI-COLOUR cool → distinct mid hue → warm                 (NEON,
 *              VOLT, CYBER)
 *   MONO       grayscale reference                            (MONO)
 *
 * All values respect the CLAUDE.md brightness rule: ≥ 30 everywhere,
 * with 24-29 / 240-243 reserved for the DIMMEST end-tier only.
 */
static const Theme themes[N_THEMES] = {
    /* name        ramp[0..7]  (most-neg → neutral → most-pos)        trail sky */

    { "VOLT",     {  24,  33,  39, 117, 230, 220, 214, 196 },           246,  240 }, /* electric blue → yellow → red (tri)     */
    { "COPPER",   {  25,  32,  67, 110, 230, 222, 214, 202 },           246,  240 }, /* steel blue → cream → copper red (di)   */
    { "NEON",     {  27,  39,  87, 195, 230, 213, 207, 199 },           246,  240 }, /* blue → cyan → pink (tri)               */
    { "ICE_FIRE", {  24,  31,  39,  87, 224, 209, 202, 196 },           246,  240 }, /* ice blue → flame red (di, classic)     */
    { "AURORA",   {  28,  34,  79, 159, 224, 213, 207, 199 },           246,  240 }, /* green → pink (di, aurora-like)         */
    { "VIOLET",   {  53,  90, 134, 213, 224, 223, 215, 196 },           246,  240 }, /* violet → red (di)                      */
    { "CYBER",    {  28,  34, 121, 195, 230, 219, 207, 197 },           246,  240 }, /* green → cyan → red (tri)               */
    { "PASTEL",   { 110, 117, 153, 195, 224, 217, 218, 211 },           246,  240 }, /* soft blue → soft pink (di, low contrast) */
    { "TWILIGHT", {  54,  60,  97, 104, 218, 211, 209, 196 },           246,  240 }, /* indigo → coral (di)                    */
    { "SODIUM",   { 130, 166, 172, 215, 195,  87,  51,  39 },           246,  240 }, /* sodium amber → mercury cyan (di)       */
    { "ECLIPSE",  { 240, 244, 247, 250, 196, 202, 208, 226 },           246,  240 }, /* shadow gray → corona red (di)          */
    { "MONO",     { 240, 243, 245, 247, 249, 251, 253, 255 },           246,  240 }, /* grayscale reference                    */
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

/*
 * Particle — one charged mass-point in the simulation.
 *
 * Coordinate system (px, py):
 *   PHYSICAL coordinates, not cell coordinates.  We use
 *
 *     px = cell_x
 *     py = cell_y · ASPECT_Y          (ASPECT_Y = 2)
 *
 *   so the inverse-square Coulomb law remains ISOTROPIC in the visual
 *   plane.  Terminal cells are ~2× taller than wide; a particle moving
 *   one cell vertically has travelled twice as far visually as one
 *   moving one cell horizontally.  Storing physical coords lets us
 *   write r² = dx² + dy² in the force loop with no aspect correction.
 *   Conversion back to integer cells happens only at draw time:
 *
 *     ix = (int)(px + 0.5f)
 *     iy = (int)(py / ASPECT_Y + 0.5f)
 *
 * Why explicit Euler (not Verlet / RK4):
 *   This is a visual demo, not a long-term orbit simulator.  Euler
 *   is simplest — v += F/m·dt; p += v·dt — and is NOT symplectic [4]:
 *   it gains energy over time on circular orbits.  A small VELOCITY_DAMP
 *   per tick and the BOUNCE_REST loss on every wall hit together
 *   compensate for the drift.  For multi-second visual runs this looks
 *   exactly as physical as Verlet without the extra integrator state.
 *
 * The `fixed` flag:
 *   true → the particle CONTRIBUTES to the force on every other free
 *   particle but its own velocity and position are NEVER integrated
 *   (effectively infinite mass).  Used for the DIPOLE_FIELD pattern's
 *   two heavy ±5 anchors; every other pattern has fixed = false.
 *
 * Trail ring buffer (trail_px / trail_py / trail_head / trail_count):
 *   Each particle owns TRAIL_LEN past positions.  The renderer reads
 *   newest-to-oldest with a 5-tier glyph fade ('*' '+' ':' '.' '`').
 *   Index convention:
 *
 *     trail_head    = next slot to WRITE
 *     trail_count   = number of valid samples (saturates at TRAIL_LEN)
 *     oldest_idx    = (trail_head + 1) mod TRAIL_LEN
 *
 *   Each new sample is pushed at the end of scene_tick; the renderer
 *   never advances `trail_head`.
 *
 * The `bounce_age` counter:
 *   Frames remaining of a wall-bounce visual splash.  Set to
 *   BOUNCE_FLASH_FRAMES on impact in scene_tick, decremented each
 *   tick.  While > 0, the renderer paints the head as '*' bold AND
 *   upgrades the 4-cell halo to a bright '*' splash — the "energy
 *   was just dissipated to the wall" visual cue.
 *
 * Algorithm refs (header REFERENCES):
 *   Coulomb law F = k·q_i·q_j/r²              — Griffiths [1]
 *   Two-body circular orbit speed (BINARY seed) — Goldstein [2] §3
 *   Pairwise O(N²) integration loop            — Hockney & Eastwood [3]
 *   Explicit-Euler energy drift / drag patch   — Hairer et al. [4]
 */
typedef struct {
    /* ── Phase-space state (advanced each tick) ───────────────────── */
    float px, py;       /* position, PHYSICAL coords (py = cell·ASPECT_Y) */
    float vx, vy;       /* velocity, physical units per second           */

    /* ── Identity / static properties ─────────────────────────────── */
    float charge;       /* signed; sign → curl direction, |q| → force
                         * strength AND palette slot (charge_to_slot)    */
    float mass;         /* inertial mass; 1 for free, ≫1 for fixed
                         * anchors so external force can't budge them    */
    bool  fixed;        /* true → contributes to forces but never moves  */
    bool  active;       /* false → slot empty / unused                   */

    /* ── Trail ring buffer (read newest-to-oldest by §5 renderer) ──── */
    float trail_px[TRAIL_LEN];
    float trail_py[TRAIL_LEN];
    int   trail_head;   /* next write index ∈ [0, TRAIL_LEN)             */
    int   trail_count;  /* valid sample count, saturates at TRAIL_LEN    */

    /* ── Visual: wall-bounce flash counter ────────────────────────── */
    int   bounce_age;   /* > 0 → head + halo upgrade to '*' bold;
                         * decremented one frame per tick.  Set to
                         * BOUNCE_FLASH_FRAMES on every wall impact.    */
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
/* §5  scene — Scene struct, particle pool, init / tick / draw drivers   */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level simulation + render state for one run.  Lives inside
 * the file-static App container (g_app.scene); helpers in this file see
 * it through a `Scene *s` pointer.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters — consumed by scene_tick, scene_init_pattern,
 *     scene_reseed.  Anything that affects the PHYSICS (positions,
 *     velocities, force outcomes, which particles spawn where) lives
 *     here.  Mutated by physics-affecting keys: space (pause), n / p
 *     (next/prev pattern), r (reseed), + / − (speed multiplier).
 *
 *   Rendering parameters — consumed by scene_draw and screen_draw only.
 *     Toggling these while paused must leave particle positions and
 *     velocities byte-identical; only colours may differ.  Mutated by
 *     purely cosmetic keys: t / T (theme).
 *
 * Locality rationale (this contract matters, not the bytes):
 *   The split exists for the READER, not the CPU.  A new flag landing
 *   in the rendering group when it actually steers integration or
 *   spawning would silently couple display to physics — exactly the
 *   bug the separation prevents.  When adding a field, ask: does it
 *   change forces, integration, or which particles spawn?  If yes,
 *   simulation; if no, rendering.
 *
 * Single instance (file-scope `g_app.scene`):
 *   particles[] is the bulk of the storage (~20 KB at MAX_PARTICLES =
 *   80, TRAIL_LEN = 30), so Scene lives in BSS as part of the file-
 *   static `g_app` rather than being passed by value.
 *
 * Why cols / rows is in Scene here (unlike blackhole.c):
 *   In this demo the physics chamber IS the screen — wall bounces use
 *   cols / rows directly to know where the boundaries are.  Caching
 *   them on Scene keeps spawn helpers and scene_tick from threading
 *   geometry through every call.  Resize calls scene_resize(), which
 *   updates these fields without rebuilding particles.
 *
 * What stays OUTSIDE Scene (intentionally):
 *
 *   App.running / App.need_resize    sig_atomic_t flags set by signal
 *                                    handlers; must stay at file scope
 *                                    for async-signal safety.  See App
 *                                    (§7) for the wrapper struct.
 *
 *   App.sim_fps                      main-loop pacing (10–120 Hz).
 *                                    Controls how OFTEN scene_tick is
 *                                    called, NOT what each tick does;
 *                                    therefore loop bookkeeping, not
 *                                    simulation state.
 *
 *   App.screen (Screen.cols/rows)    ncurses-side view of geometry.
 *                                    Scene mirrors cols/rows for the
 *                                    chamber-collision use above; the
 *                                    two are kept in sync by
 *                                    scene_resize.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ── Simulation parameters ────────────────────────────────────── */
    Pattern   current_pattern;   /* PATTERN_BINARY..PATTERN_RANDOM_GAS;
                                  * n / p keys cycle.                  */
    int       speed;             /* dt multiplier: actual_dt =
                                  * tick_dt × speed / SPEED_DEF.
                                  * +/- keys double / halve.           */
    bool      paused;            /* space — scene_tick is a no-op.    */
    uint32_t  rng;               /* LCG state for pattern seeding;
                                  * reseeded on 'r' from the clock.   */
    int       cols, rows;        /* chamber size in CELLS = screen
                                  * size; bounce checks read these.   */
    int       n_particles;       /* count of active slots in the pool */
    Particle  particles[MAX_PARTICLES];

    /* ── Rendering parameters ─────────────────────────────────────── */
    int       current_theme;     /* index into themes[] (§1, §3);
                                  * t / T keys cycle.  Pure cosmetic
                                  * — never alters physics.            */
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
 * chamber_centre — geometric centre of the physics chamber in PHYSICAL
 * coords (recall py is cell_y · ASPECT_Y to keep r² isotropic).
 *
 * Used as the spawn anchor for the symmetric patterns (BINARY,
 * TRINARY, DIPOLE).
 */
static void chamber_centre(const Scene *s, float *cx, float *cy)
{
    int rows_eff = s->rows - 1;
    *cx = (float)s->cols  * 0.5f;
    *cy = (float)rows_eff * 0.5f * ASPECT_Y;
}

/*
 * init_pattern_binary — two equal-mass opposite charges in a Keplerian
 * circular orbit about their common centre of mass.
 *
 * Centripetal balance for two ±q charges at separation d, mass m each:
 *
 *     v = √(k · |q₁·q₂| / (2 · m · d))                 — Goldstein [2] §3
 *
 * Each particle orbits at d/2 from the common centre.  With k =
 * COULOMB_K, |q|=1, m=1, d=25 the period is a few seconds — slow
 * enough for the eye to trace the curve.
 */
static void init_pattern_binary(Scene *s)
{
    float cx, cy;
    chamber_centre(s, &cx, &cy);
    float d = 25.0f;
    float v = sqrtf(COULOMB_K / (2.0f * 1.0f * d));
    scene_add_particle(s, cx - d * 0.5f, cy, 0.0f, -v, +1.0f, 1.0f, false);
    scene_add_particle(s, cx + d * 0.5f, cy, 0.0f, +v, -1.0f, 1.0f, false);
}

/*
 * init_pattern_trinary — three particles at the vertices of an
 * equilateral triangle, charges (+1, +1, −2).  The asymmetric −2 charge
 * breaks any closed-form solution; the dynamics are chaotic for
 * almost all initial conditions [2] §11.
 *
 * Velocities are tangent to the inscribing circle (CCW) at speed
 * 0.6 × √(k / (3·r)) — slightly below the symmetric-orbit speed so
 * the configuration breaks down quickly into a 3-body dance.
 */
static void init_pattern_trinary(Scene *s)
{
    float cx, cy;
    chamber_centre(s, &cx, &cy);
    float radius = 18.0f;
    float v = 0.6f * sqrtf(COULOMB_K / (3.0f * radius));
    for (int k = 0; k < 3; k++) {
        float angle = (float)k * 2.0f * (float)M_PI / 3.0f;
        float pcx = cx + radius * cosf(angle);
        float pcy = cy + radius * sinf(angle);
        float vx  = -v * sinf(angle);          /* CCW tangent */
        float vy  =  v * cosf(angle);
        float q   = (k == 2) ? -2.0f : +1.0f;  /* the symmetry-breaker */
        scene_add_particle(s, pcx, pcy, vx, vy, q, 1.0f, false);
    }
}

/*
 * init_pattern_dipole_field — two heavy fixed dipole anchors
 * (±5 charge, mass 10⁹) plus 30 light free test particles drifting
 * through their combined field.  The anchors don't move; the test
 * particles experience the superposed Coulomb field [1] §2 and trace
 * field-line-like trajectories — repelled by like-sign anchors,
 * attracted to opposite-sign anchors.
 */
static void init_pattern_dipole_field(Scene *s)
{
    float cx, cy;
    chamber_centre(s, &cx, &cy);
    int   rows_eff = s->rows - 1;
    float dipole_d = (float)s->cols * 0.45f;

    /* Two ±5 anchors at separation dipole_d (effectively infinite mass). */
    scene_add_particle(s, cx - dipole_d * 0.5f, cy, 0, 0, +5.0f, 1e9f, true);
    scene_add_particle(s, cx + dipole_d * 0.5f, cy, 0, 0, -5.0f, 1e9f, true);

    /* 30 light test particles scattered across the full chamber. */
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
}

/*
 * init_pattern_random_gas — 40 free particles with random positions in
 * the central 80% of the chamber, charges sampled uniformly from
 * {−2, −1, +1, +2}.  Many-body Coulomb dynamics produces emergent
 * pair-formation and ejection [3] — visually the "plasma" run.
 */
static void init_pattern_random_gas(Scene *s)
{
    int rows_eff = s->rows - 1;
    for (int k = 0; k < 40; k++) {
        float r1 = lcg_unit(&s->rng);
        float r2 = lcg_unit(&s->rng);
        float r3 = lcg_unit(&s->rng);
        float r4 = lcg_unit(&s->rng);
        float r5 = lcg_unit(&s->rng);

        /* Spawn in the central 80% so the wall-bounce density isn't
         * already saturated at t = 0. */
        float pcx = (float)s->cols    * (0.10f + r1 * 0.80f);
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
}

/*
 * scene_init_pattern — clear the pool, then dispatch to the active
 * pattern's initialiser.
 *
 * Pseudocode:
 *   scene_clear_particles(s)
 *   dispatch on s->current_pattern → init_pattern_*(s)
 */
static void scene_init_pattern(Scene *s)
{
    scene_clear_particles(s);
    switch (s->current_pattern) {
    case PATTERN_BINARY:       init_pattern_binary       (s); break;
    case PATTERN_TRINARY:      init_pattern_trinary      (s); break;
    case PATTERN_DIPOLE_FIELD: init_pattern_dipole_field (s); break;
    case PATTERN_RANDOM_GAS:   init_pattern_random_gas   (s); break;
    case N_PATTERNS:           break;
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
 * sum_coulomb_force_on — pairwise O(N) Coulomb force [1] on particle i
 * from every other active particle in the pool.
 *
 *   F_on_i = Σ_j  −k · q_i · q_j / r³ · (r_j − r_i)        [j ≠ i]
 *
 * Sign convention: like charges (q_i·q_j > 0) → factor negative →
 * force points OPPOSITE to (r_j − r_i) → AWAY from j → repulsion.
 * Opposite charges → factor positive → TOWARD j → attraction.
 *
 * Softening (r² ← max(r², R_SOFT²)) prevents the singularity at r → 0
 * when an opposite-charge pair approaches closely [3] — without it
 * the force would blow up and the integrator would explode.
 */
static void sum_coulomb_force_on(const Scene *s, int i,
                                 float *fx_out, float *fy_out)
{
    const Particle *pi = &s->particles[i];
    float r_soft2 = R_SOFT_MIN * R_SOFT_MIN;
    float fx = 0.0f, fy = 0.0f;

    for (int j = 0; j < s->n_particles; j++) {
        if (i == j) continue;
        const Particle *pj = &s->particles[j];
        if (!pj->active) continue;

        float dx = pj->px - pi->px;
        float dy = pj->py - pi->py;
        float r2 = dx * dx + dy * dy;
        if (r2 < r_soft2) r2 = r_soft2;           /* softening */
        float r  = sqrtf(r2);

        float factor = -COULOMB_K * pi->charge * pj->charge / (r2 * r);
        fx += factor * dx;
        fy += factor * dy;
    }
    *fx_out = fx;
    *fy_out = fy;
}

/*
 * apply_force_to_velocity — explicit-Euler velocity update.
 *
 *   v ← v + (F / m) · dt
 *
 * Explicit Euler is not symplectic [4] — orbits gain energy over
 * time — but a global apply_velocity_drag() per tick sponges that
 * drift off, and BOUNCE_REST < 1 on wall hits adds more dissipation.
 */
static void apply_force_to_velocity(Particle *p, float fx, float fy, float dt)
{
    p->vx += fx / p->mass * dt;
    p->vy += fy / p->mass * dt;
}

/*
 * apply_velocity_drag — exponential decay of velocity once per tick.
 *
 *   v ← v · damp,         damp = exp(−VELOCITY_DAMP · dt)
 *
 * Compensates for the explicit-Euler energy creep [4] and prevents
 * the pool from blowing apart under repeated Coulomb repulsion.
 * Computed once outside the loop; each particle just multiplies.
 */
static void apply_velocity_drag(Particle *p, float damp)
{
    p->vx *= damp;
    p->vy *= damp;
}

/*
 * clamp_velocity_magnitude — safety cap on |v| at MAX_VELOCITY.
 *
 * If a near-miss through the softening floor produces a very large
 * force impulse, the resulting velocity could teleport the particle
 * across the screen in one step.  Capping |v| to MAX_VELOCITY keeps
 * the position step bounded and defensive against pathological
 * encounters.
 */
static void clamp_velocity_magnitude(Particle *p, float vmax)
{
    float vmag2 = p->vx * p->vx + p->vy * p->vy;
    if (vmag2 > vmax * vmax) {
        float scale = vmax / sqrtf(vmag2);
        p->vx *= scale;
        p->vy *= scale;
    }
}

/*
 * integrate_position — explicit-Euler drift step.
 *
 *   p ← p + v · dt
 *
 * Uses the velocity AFTER drag + cap, so the drift is consistent with
 * the energy-dissipated state.
 */
static void integrate_position(Particle *p, float dt)
{
    p->px += p->vx * dt;
    p->py += p->vy * dt;
}

/*
 * reflect_at_chamber_walls — elastic-with-restitution bounce off the
 * four chamber boundaries.  Each impact:
 *   1. clamps the offending coordinate back inside the chamber
 *   2. negates and scales the matching velocity component by BOUNCE_REST
 *   3. sets bounce_age = BOUNCE_FLASH_FRAMES so the renderer flashes '*'
 *
 * Chamber bounds (PHYSICAL coords):
 *   1.0       ≤ px ≤ cols − 2
 *   ASPECT_Y  ≤ py ≤ py_max          (top + bottom HUD rows excluded)
 */
static void reflect_at_chamber_walls(Particle *p, int cols, float py_max)
{
    if (p->px < 1.0f) {
        p->px = 1.0f;
        p->vx = -p->vx * BOUNCE_REST;
        p->bounce_age = BOUNCE_FLASH_FRAMES;
    } else if (p->px > (float)(cols - 2)) {
        p->px = (float)(cols - 2);
        p->vx = -p->vx * BOUNCE_REST;
        p->bounce_age = BOUNCE_FLASH_FRAMES;
    }
    if (p->py < ASPECT_Y) {
        p->py = ASPECT_Y;
        p->vy = -p->vy * BOUNCE_REST;
        p->bounce_age = BOUNCE_FLASH_FRAMES;
    } else if (p->py > py_max) {
        p->py = py_max;
        p->vy = -p->vy * BOUNCE_REST;
        p->bounce_age = BOUNCE_FLASH_FRAMES;
    }
}

/*
 * push_trail_sample — append the current position to the trail ring
 * buffer and advance the write head.  Saturates trail_count at
 * TRAIL_LEN once the buffer is full.
 */
static void push_trail_sample(Particle *p)
{
    p->trail_head = (p->trail_head + 1) % TRAIL_LEN;
    p->trail_px[p->trail_head] = p->px;
    p->trail_py[p->trail_head] = p->py;
    if (p->trail_count < TRAIL_LEN) p->trail_count++;
}

/*
 * scene_tick — one N² Coulomb integration step.
 *
 * Pseudocode:
 *   if paused: return
 *   dt ← dt · speed / SPEED_DEF
 *
 *   /​* Pass 1: pairwise forces → velocity *​/
 *   for each free particle i:
 *       (fx, fy) ← sum_coulomb_force_on(s, i)
 *       apply_force_to_velocity(p_i, fx, fy, dt)
 *
 *   /​* Pass 2: drag → cap → drift → bounce → cool flash *​/
 *   damp   ← exp(−VELOCITY_DAMP · dt)
 *   py_max ← (rows − 2) · ASPECT_Y
 *   for each free particle p:
 *       apply_velocity_drag(p, damp)
 *       clamp_velocity_magnitude(p, MAX_VELOCITY)
 *       integrate_position(p, dt)
 *       reflect_at_chamber_walls(p, cols, py_max)
 *       if bounce_age > 0: bounce_age--
 *
 *   /​* Pass 3: trail *​/
 *   for each active particle p:
 *       push_trail_sample(p)
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    dt *= (float)s->speed / (float)SPEED_DEF;

    /* Pass 1: pairwise forces → velocity update. */
    for (int i = 0; i < s->n_particles; i++) {
        Particle *pi = &s->particles[i];
        if (!pi->active || pi->fixed) continue;
        float fx, fy;
        sum_coulomb_force_on(s, i, &fx, &fy);
        apply_force_to_velocity(pi, fx, fy, dt);
    }

    /* Pass 2: drag → cap → drift → bounce → cool flash. */
    float damp   = expf(-VELOCITY_DAMP * dt);
    float py_max = (float)(s->rows - 2) * ASPECT_Y;
    for (int i = 0; i < s->n_particles; i++) {
        Particle *p = &s->particles[i];
        if (!p->active || p->fixed) continue;
        apply_velocity_drag(p, damp);
        clamp_velocity_magnitude(p, MAX_VELOCITY);
        integrate_position(p, dt);
        reflect_at_chamber_walls(p, s->cols, py_max);
        if (p->bounce_age > 0) p->bounce_age--;
    }

    /* Pass 3: trail. */
    for (int i = 0; i < s->n_particles; i++) {
        Particle *p = &s->particles[i];
        if (!p->active) continue;
        push_trail_sample(p);
    }
}

/*
 * physical_to_cell — convert PHYSICAL coords (px, py) to integer cell
 * (ix, iy) for ncurses.  Inverse of the convention in the Particle
 * docstring: physical y is stretched by ASPECT_Y to keep r² isotropic
 * in the force loop, so we undo that here for rendering.
 *
 *   ix = round(px)
 *   iy = round(py / ASPECT_Y)
 */
static void physical_to_cell(float px, float py, int *ix, int *iy)
{
    *ix = (int)(px + 0.5f);
    *iy = (int)(py / ASPECT_Y + 0.5f);
}

/*
 * trail_age_glyph — 5-tier glyph + attribute ramp from age fraction
 * `newness` ∈ [0, 1]  (0 = oldest sample, 1 = newest just behind head).
 *
 *     newness > 0.85  →  '*'  A_BOLD
 *     newness > 0.65  →  '+'  A_NORMAL
 *     newness > 0.45  →  ':'  A_NORMAL
 *     newness > 0.25  →  '.'  A_NORMAL
 *     else            →  '`'  A_NORMAL
 *
 * Glyph density alone provides the fade (* densest → ' sparsest).
 *
 * Algorithm ref: perceptual density-based fade — Ware [5].
 */
static void trail_age_glyph(float newness, chtype *glyph, attr_t *attr)
{
    if      (newness > 0.85f) { *glyph = '*'; *attr = A_BOLD;   }
    else if (newness > 0.65f) { *glyph = '+'; *attr = A_NORMAL; }
    else if (newness > 0.45f) { *glyph = ':'; *attr = A_NORMAL; }
    else if (newness > 0.25f) { *glyph = '.'; *attr = A_NORMAL; }
    else                       { *glyph = '`'; *attr = A_NORMAL; }
}

/*
 * paint_particle_trail — render one particle's ring buffer of past
 * positions, oldest-to-newest, with the 5-tier age-fade glyph in the
 * particle's own charge-coloured pair.  Fixed anchors have no trail.
 */
static void paint_particle_trail(const Scene *s, const Particle *p,
                                 int top_clip, int bot_clip)
{
    if (p->fixed) return;
    int n = p->trail_count;
    if (n < 2) return;

    int slot = charge_to_slot(p->charge);
    int pair = PAIR_RAMP_BASE + slot;
    float denom = (float)(n - 2 > 0 ? n - 2 : 1);

    for (int k = 0; k < n - 1; k++) {
        /* k = 0 oldest, k = n-2 newest just behind head (head paints there) */
        int idx = (p->trail_head + 1 + k) % TRAIL_LEN;
        int ix, iy;
        physical_to_cell(p->trail_px[idx], p->trail_py[idx], &ix, &iy);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < top_clip || iy >= bot_clip) continue;

        chtype glyph;
        attr_t attr;
        trail_age_glyph((float)k / denom, &glyph, &attr);
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/*
 * paint_particle_halo — 4-cell glow halo around a free particle's head.
 *
 *   horizontal neighbours (±1, 0)  →  '.'  (closer in physical space —
 *                                          cells are 2× taller than wide)
 *   vertical neighbours   (0, ±1)  →  '`'  (farther physically, softer
 *                                          glyph keeps halo isotropic)
 *
 * While bounce_age > 0 ALL four neighbours upgrade to '*' bold — the
 * wall-impact splash that fades over BOUNCE_FLASH_FRAMES ticks.
 */
static void paint_particle_halo(const Scene *s, const Particle *p,
                                int top_clip, int bot_clip)
{
    if (p->fixed) return;

    int ix, iy;
    physical_to_cell(p->px, p->py, &ix, &iy);
    int slot = charge_to_slot(p->charge);
    int pair = PAIR_RAMP_BASE + slot;

    bool   flash     = (p->bounce_age > 0);
    chtype halo_h    = flash ? '*' : '.';
    chtype halo_v    = flash ? '*' : '`';
    attr_t halo_attr = flash ? A_BOLD : A_NORMAL;

    struct { int dx, dy; chtype gl; } halo[] = {
        { -1,  0, halo_h }, { +1,  0, halo_h },
        {  0, -1, halo_v }, {  0, +1, halo_v },
    };

    attron(COLOR_PAIR(pair) | halo_attr);
    for (int k = 0; k < 4; k++) {
        int hx = ix + halo[k].dx;
        int hy = iy + halo[k].dy;
        if (hx < 0 || hx >= s->cols) continue;
        if (hy < top_clip || hy >= bot_clip) continue;
        mvaddch(hy, hx, halo[k].gl);
    }
    attroff(COLOR_PAIR(pair) | halo_attr);
}

/*
 * paint_particle_head — render the particle head glyph + attributes.
 *
 * Glyph selection:
 *   fixed               →  '@'  A_BOLD       (heavy dipole anchor)
 *   bounce_age > 0      →  '*'  A_BOLD       (wall-impact flash overrides)
 *   q > +0.5            →  '+'
 *   q < −0.5            →  '−'
 *   else                →  '.'                (near-neutral)
 *
 * Brightness when not in a special state:
 *   heavy (|q| > 1.5)  OR  fast (|v|² > VBOLD²)  →  A_BOLD
 *   otherwise                                    →  A_NORMAL
 */
static void paint_particle_head(const Scene *s, const Particle *p,
                                int top_clip, int bot_clip)
{
    int ix, iy;
    physical_to_cell(p->px, p->py, &ix, &iy);
    if (ix < 0 || ix >= s->cols) return;
    if (iy < top_clip || iy >= bot_clip) return;

    chtype glyph;
    attr_t attr;
    if (p->fixed) {
        glyph = '@';
        attr  = A_BOLD;
    } else if (p->bounce_age > 0) {
        glyph = '*';
        attr  = A_BOLD;
    } else {
        if      (p->charge > 0.5f)  glyph = '+';
        else if (p->charge < -0.5f) glyph = '-';
        else                        glyph = '.';
        float vmag2 = p->vx * p->vx + p->vy * p->vy;
        bool  heavy = fabsf(p->charge) > 1.5f;
        bool  fast  = vmag2 > VBOLD_THRESHOLD * VBOLD_THRESHOLD;
        attr = (heavy || fast) ? A_BOLD : A_NORMAL;
    }

    int slot = charge_to_slot(p->charge);
    int pair = PAIR_RAMP_BASE + slot;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * scene_draw — three layered passes, painted back-to-front:
 *
 * Pseudocode:
 *   top_clip ← HUD_TOP
 *   bot_clip ← rows − HUD_BOT
 *   for each active particle: paint_particle_trail  (bottom layer)
 *   for each active particle: paint_particle_halo   (middle layer)
 *   for each active particle: paint_particle_head   (top layer)
 *
 * Separate passes (rather than trail/halo/head per particle in one
 * loop) so that no head ever gets painted over by another particle's
 * trail / halo — the head is always the brightest on-screen pixel.
 */
static void scene_draw(const Scene *s)
{
    int top_clip = HUD_TOP;
    int bot_clip = s->rows - HUD_BOT;

    for (int i = 0; i < s->n_particles; i++) {
        const Particle *p = &s->particles[i];
        if (p->active) paint_particle_trail(s, p, top_clip, bot_clip);
    }
    for (int i = 0; i < s->n_particles; i++) {
        const Particle *p = &s->particles[i];
        if (p->active) paint_particle_halo(s, p, top_clip, bot_clip);
    }
    for (int i = 0; i < s->n_particles; i++) {
        const Particle *p = &s->particles[i];
        if (p->active) paint_particle_head(s, p, top_clip, bot_clip);
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

    /* ── Row 0 right: live status (CP_HUD bright yellow + bold) ─── */
    const char *state_str = s->paused ? "PAUSED " : "running";
    char top[200];
    snprintf(top, sizeof top,
             " %s  theme:%s  N=%d  speed:%d  %s  %.0f fps  %dHz ",
             pattern_name(s->current_pattern),
             themes[s->current_theme].name,
             s->n_particles, s->speed,
             state_str, fps, sim_fps);
    int top_len = (int)strlen(top);
    int top_col = sc->cols - top_len;
    if (top_col < 0) top_col = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvaddnstr(0, top_col, top, sc->cols);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* ── Row rows-1 left: actions (CP_HINT bright cyan + bold) ──── */
    const char *hint_full =
        " q:quit  spc:pause  r:reseed  n/p:pattern  "
        "t/T:theme  +/-:speed  ]/[:fps ";
    const char *hint_short =
        " q:quit  spc:pause  r:reseed  n/p:pat  t/T:theme ";
    const char *hint = hint_full;
    if ((int)strlen(hint_full) >= sc->cols - 1) hint = hint_short;
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvaddnstr(sc->rows - 1, 0, hint, sc->cols);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
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
