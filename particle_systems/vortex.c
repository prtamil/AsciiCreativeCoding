/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * vortex.c — particles spiral inward toward a central drain, accelerating
 *
 * DEMO: A central drain at the screen centre attracts particles
 *       distributed around its outer edge. Each particle has polar
 *       coordinates `(r, θ)` and integrates two forces every tick:
 *       a RADIAL inflow that shrinks r over time, and an ANGULAR
 *       velocity that grows as 1/r (Kepler-style conservation of
 *       angular momentum). The combined motion is a SPIRAL — slow
 *       and lazy at the outer edge, accelerating into a tight
 *       whip near the centre. Each particle leaves a short tangent
 *       trail, so the spiral arms read clearly even at low
 *       terminal resolution.
 *
 *       Patterns:
 *         WHIRLPOOL    log-spiral inflow, smooth water-vortex feel
 *         TORNADO      strong Kepler boost — tight high-speed core
 *         BLACK_HOLE   extreme inflow + spin near centre, with
 *                      central pulse
 *         SINK         constant inflow + constant ω, classic
 *                      bathtub-drain look
 *
 * Study alongside:
 *   physics/blackhole.c     — physically-derived gravity gravity-well
 *                              raymarcher; this is the polar-kinematic
 *                              counterpart.
 *   procedural/worldgen/procedural_galaxy.c — log-spiral arms in
 *                              closed-form (no per-particle integration).
 *   rain.c, snow.c, fountain.c, embers.c — same pool / spawn / tick /
 *                              draw shape; vortex.c is the FORCE-driven
 *                              counterpart to those flow-driven systems.
 *
 * Section map:
 *   §1 config    — constants, themes, per-pattern parameters
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair colour ramp (outer dim → inner bright)
 *   §4 particle  — Particle (polar coord) struct
 *   §5 scene     — pool, polar-physics tick, trail render
 *   §6 screen    — ncurses init / draw / resize
 *   §7 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear pool, re-prewarm)
 *   n / N      next pattern   (WHIRLPOOL → TORNADO → BLACK_HOLE → SINK)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/vortex.c \
 *       -o vortex -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system using POLAR
 *                  coordinates `(r, θ)` instead of Cartesian. Each
 *                  particle carries its radius from centre and its
 *                  angle around centre. Every tick:
 *
 *                    dr/dt = -(α_log · r  +  α_const)
 *                    dθ/dt =   β_const  +  β_kepler / max(r, ε)
 *
 *                  - α_log gives an EXPONENTIAL inflow component:
 *                    `r(t) = r₀ · exp(−α_log · t)`. This is the log-
 *                    spiral signature — equal-angle convergence.
 *                  - α_const adds a CONSTANT inflow per second; useful
 *                    for the bathtub-drain SINK pattern where inflow
 *                    is more about Bernoulli than about gravity.
 *                  - β_const gives a steady angular velocity at all
 *                    radii; combined with α_log alone produces a
 *                    classic log spiral.
 *                  - β_kepler / r mimics conservation of angular
 *                    momentum (`L = m·v·r` constant ⇒ ω ∝ 1/r²` for
 *                    point-particle gravitational fall, but the 1/r
 *                    form gives a visually-readable speed-up without
 *                    blowing up to infinity).
 *
 *                  At each frame we render every particle PLUS a
 *                  short tangent trail. The trail is computed by
 *                  back-stepping the current `dr/dt` and `dθ/dt` for
 *                  TRAIL_LEN cells, so the trail follows the local
 *                  spiral arc. This is what makes the spiral arms
 *                  read at low terminal resolution; without the
 *                  trails you see only the heads and the spiral
 *                  pattern is much harder to recognise.
 *
 *                  Polar coordinates are converted to terminal cells
 *                  with ASPECT_Y compensation so the spiral renders
 *                  ROUND on screen instead of ellipsoid (terminal
 *                  cells are ~2× taller than wide).
 *
 * Data-structure : Particle[MAX_PARTICLES] object pool with `active`
 *                  flag. Linear-scan spawn, no malloc.
 *
 * Rendering      : ASCII only. Glyph + colour driven by the
 *                  particle's normalised radius `(R_OUTER − r) /
 *                  R_OUTER`: outer particles are dim/sparse,
 *                  particles near the drain are bright/dense
 *                  (they're moving fastest and "heating up" toward
 *                  the drain). Trail cells use progressively dimmer
 *                  ramp slots than the head.
 *
 * Performance    : O(MAX_PARTICLES · (1 + TRAIL_LEN)) per tick.
 *                  At MAX_PARTICLES = 700 and TRAIL_LEN = 4, ~3500
 *                  mvaddch per frame. Trivial at 60 fps.
 *
 * References     :
 *   • Wikipedia — [Logarithmic spiral](https://en.wikipedia.org/wiki/Logarithmic_spiral).
 *     The α_log inflow + β_const angular term produces this exact
 *     shape — `r = r₀ · exp(−α/β · θ)`. Found everywhere in nature
 *     (galaxy arms, nautilus shells, hurricane bands).
 *   • Wikipedia — [Specific angular momentum](https://en.wikipedia.org/wiki/Specific_angular_momentum).
 *     The 1/r angular term is a kinematic stand-in for the
 *     conservation-of-L behaviour of gravitational orbits.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Stop thinking in (x, y) — think in (r, θ). Each particle has a
 * radius and an angle around the centre, NOT a position in the
 * plane. Every tick you decrease its radius (inflow) and increase
 * its angle (rotation). When you finally render, you convert
 * (r, θ) to (cell_x, cell_y) — that conversion is a one-liner.
 * The spiral shape EMERGES from the integration; you never compute
 * a spiral curve directly.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a turntable with a marble on the edge. You slowly
 * shorten its leash to the centre while the turntable keeps
 * spinning. The marble traces a spiral inward. Some patterns
 * (Black Hole) make the marble's leash shrink fast AND the
 * turntable spin faster as it gets closer to the middle, so the
 * marble whips around the centre at the end. Others (Sink) keep
 * a steady spin and steady leash-shortening — a smooth lazy
 * spiral.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SPAWN at the OUTER edge: pick a random angle θ ∈ [0, 2π],
 *     pick r ∈ [r_outer · 0.85, r_outer · 1.0]. Set age = 0.
 *
 *  2. INTEGRATE per tick:
 *       dr_dt   = -(α_log · r  +  α_const)
 *       dθ_dt   =   β_const  +  β_kepler / max(r, R_MIN)
 *       r      += dr_dt · dt
 *       θ      += dθ_dt · dt
 *       age    += dt
 *
 *  3. DRAIN: when r < R_DRAIN (small inner radius), deactivate.
 *
 *  4. RENDER. For each particle:
 *       cell_x = cx + r · cos θ
 *       cell_y = cy + r · sin θ / ASPECT_Y    (aspect correction)
 *     Plus tangent trail of TRAIL_LEN cells back-stepped along
 *     the local (dr, dθ).
 *
 *  5. CENTRE GLYPH: a fixed small glyph at (cx, cy) — `O` for
 *     SINK, `@` for BLACK_HOLE pulse, etc.
 *
 *  6. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Polar → cell:
 *    cell_x = cx + r · cos θ
 *    cell_y = cy + r · sin θ / ASPECT_Y         (ASPECT_Y = 2)
 *
 *  Inflow (radial):
 *    dr/dt = -(α_log · r + α_const)
 *
 *  Rotation (angular):
 *    dθ/dt = β_const + β_kepler / max(r, R_MIN)
 *
 *  Log-spiral (α_log alone, β_const alone):
 *    r(θ) = r₀ · exp((-α_log / β_const) · (θ − θ₀))
 *
 *  Trail back-step (visualises spiral arc):
 *    for i = 1..TRAIL_LEN:
 *      r_back  = r + i · |dr/dt| · TRAIL_STEP_DT
 *      θ_back  = θ − i · dθ/dt   · TRAIL_STEP_DT
 *      render at (r_back, θ_back) with dimmer attribute
 *
 *  Brightness fraction (for ramp index):
 *    f = clamp(1 − r / r_outer, 0, 1)         // outer = 0, drain = 1
 *    ramp_slot = ⌊f · 7⌋
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DIVISION BY ZERO at r → 0. The β_kepler / r term blows up at
 *    the centre. We clamp `r` to `R_MIN` (typically 1.0 cell) in
 *    the denominator. Particles are also drained at `R_DRAIN`
 *    (~3 cells) so they vanish before getting close enough to the
 *    singularity to matter.
 *
 *  • ANGULAR WRAP. θ grows monotonically; for cos/sin it's fine to
 *    have huge values. Floating-point precision is plenty even
 *    after thousands of revolutions.
 *
 *  • ASPECT RATIO. Without ASPECT_Y compensation, the spiral
 *    renders as a vertical ellipse — unnatural. Dividing the y
 *    component by ASPECT_Y = 2 makes the spiral look round.
 *
 *  • RESPAWN. When a particle drains, it goes back into the pool.
 *    Next tick the spawn-to-target loop activates it at a fresh
 *    outer-edge position. Steady-state population is maintained.
 *
 *  • TRAIL OFF-SCREEN. Trail cells can fall outside the canvas —
 *    we skip them with a bounds check, no wraparound.
 *
 *  • LARGE TARGET COUNTS at fast spin. With 700 particles all
 *    spinning fast near the drain, the inner ring becomes a
 *    glowing solid mass. Set TARGET reasonably or BLACK_HOLE may
 *    look like a bright disk rather than spiral arms.
 *
 *  • PRE-WARM. On init / reseed / pattern-change we spawn the
 *    full target count at random positions across the FULL valid
 *    radial range (R_DRAIN..R_OUTER) so the spiral fills from
 *    frame 1 instead of growing in from the edge.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Particles freeze on their spiral arc. Resume:
 *    motion continues from where it stopped.
 *
 *  • WHIRLPOOL. Smooth log spiral — equal angles between successive
 *    arm crossings. Particles slowly accelerate as they approach
 *    the centre.
 *
 *  • TORNADO. Tight Kepler-boosted core: outer arms are slow and
 *    visible; inner ring whips around the centre rapidly.
 *
 *  • BLACK_HOLE. Extreme inflow and rotation; the inner ring is
 *    barely a ring — it's a blur. Good "accretion disc" feel.
 *
 *  • SINK. Bathtub drain — constant inflow speed, constant ω.
 *    The spiral has more uniform spacing across radii.
 *
 *  • Theme cycle (`t`/`T`). Each theme produces a recognisably
 *    different vortex character: BLACKHOLE theme is purple/red,
 *    AURORA cycles through rainbow, MONO is greyscale.
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

    MAX_PARTICLES    = 1000,

    TRAIL_LEN        =   4,    /* cells of tangent trail per particle  */

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_RAMP_BASE   =   3,    /* +0..+7 = 8 outer→inner ramp slots    */
    PAIR_CENTER      =  11,    /* central drain glyph                  */
    PAIR_SKY         =  12,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y          2.0f       /* terminal cells 2× taller       */

/* Polar physics safety. */
#define R_MIN_DENOM       1.0f       /* clamp for 1/r in dθ/dt          */
#define R_DRAIN_CELLS     2.5f       /* deactivate particle below this  */

/* Trail back-step time (sec). */
#define TRAIL_STEP_DT     0.04f

/* Pattern enum. */
typedef enum {
    PATTERN_WHIRLPOOL  = 0,
    PATTERN_TORNADO    = 1,
    PATTERN_BLACK_HOLE = 2,
    PATTERN_SINK       = 3,
    N_PATTERNS         = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_WHIRLPOOL:  return "WHIRLPOOL ";
    case PATTERN_TORNADO:    return "TORNADO   ";
    case PATTERN_BLACK_HOLE: return "BLACK_HOLE";
    case PATTERN_SINK:       return "SINK      ";
    default:                 return "?         ";
    }
}

/*
 * PatternParams — physics knobs per pattern.
 *
 *   target_count     : steady-state active particle count
 *   r_outer_frac     : outer radius as fraction of min(cols/2, rows/2·ASPECT_Y)
 *   inflow_log       : α_log — exponential inflow rate (1/s)
 *   inflow_const     : α_const — constant inflow speed (cells/s)
 *   angular_const    : β_const — angular velocity at all radii (rad/s)
 *   angular_kepler   : β_kepler — 1/r angular boost (rad·cell/s)
 *   center_glyph     : single character at the drain centre
 *   center_pulse     : non-zero → centre pulses (BLACK_HOLE)
 */
typedef struct {
    int   target_count;
    float r_outer_frac;
    float inflow_log;
    float inflow_const;
    float angular_const;
    float angular_kepler;
    char  center_glyph;
    int   center_pulse;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /*                  target  rfrac  inLog  inConst  angC   angKep   ctr  pulse */
    /* WHIRLPOOL  */ {  500,   0.85f,  0.40f,  0.30f,  1.40f,  0.0f,  '.',  0 },
    /* TORNADO    */ {  600,   0.85f,  0.20f,  1.20f,  0.80f,  4.0f,  '*',  0 },
    /* BLACK_HOLE */ {  700,   0.95f,  0.55f,  2.00f,  0.50f,  8.0f,  '@',  1 },
    /* SINK       */ {  450,   0.85f,  0.00f,  4.00f,  2.00f,  1.0f,  'O',  0 },
};

/*
 * Themes — each is an 8-step gradient from OUTER (slot 0, dim) to
 * INNER (slot 7, bright). Particles near the drain take the brightest
 * slot; outer arms fade through the lower slots.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       ramp[8];   /* outer dim → inner bright */
    short       center;
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name         ramp[0..7]                                       center sky */

    { "DEFAULT",   {  24,  31,  38,  45,  87, 117, 153, 195 },        231,  234 },
    { "BLACKHOLE", {  53,  91, 134, 165, 207, 213, 219, 231 },        201,  232 },
    { "TORNADO",   { 240, 244, 248, 250, 252, 253, 254, 255 },        226,  234 },
    { "FIRE",      {  88, 124, 130, 166, 196, 208, 214, 226 },        231,  234 },
    { "GREEN",     {  28,  34,  40,  64,  70, 112, 156, 192 },        231,  234 },
    { "VIOLET",    {  53,  54,  91, 134, 135, 176, 213, 219 },        231,  234 },
    { "ICE",       {  24,  31,  67, 110, 117, 153, 195, 231 },        231,  234 },
    { "COPPER",    { 130, 137, 173, 179, 215, 222, 229, 230 },        231,  234 },
    { "AURORA",    {  43,  79, 115, 121, 157, 195, 230, 231 },        231,  234 },
    { "MONO",      { 240, 243, 245, 247, 249, 251, 253, 255 },        255,  232 },
};

/* Glyphs by inner-fraction (outer→inner). */
static const char RAMP_GLYPHS[8] = { '`', '.', ',', ':', ';', '-', '+', '*' };

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
        init_pair(PAIR_CENTER, t->center, -1);
        init_pair(PAIR_SKY,    t->sky,    -1);
    } else {
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), COLOR_CYAN, -1);
        init_pair(PAIR_CENTER, COLOR_WHITE, -1);
        init_pair(PAIR_SKY,    COLOR_BLACK, -1);
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

/* ===================================================================== */
/* §4  particle                                                           */
/* ===================================================================== */

typedef struct {
    float r;        /* radius from centre (in cells, post-aspect) */
    float theta;    /* angle in radians                          */
    float age;
    bool  active;
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
/* §5  scene — pool, polar tick, draw                                    */
/* ===================================================================== */

typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    uint32_t  rng;
    int       rows, cols;

    float     time_accum;       /* for centre pulse animation        */

    Particle  particles[MAX_PARTICLES];
} Scene;

static int particle_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (!s->particles[i].active) return i;
    return -1;
}

static void scene_clear_particles(Scene *s)
{
    for (int i = 0; i < MAX_PARTICLES; i++) s->particles[i].active = false;
}

/* Compute the outer radius (in cells) for the current screen size. */
static float scene_r_outer(const Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    /* The smaller dimension dominates after aspect correction. */
    float half_w = (float)s->cols * 0.5f;
    float half_h = (float)s->rows * 0.5f * ASPECT_Y;
    float min_h  = half_w < half_h ? half_w : half_h;
    return min_h * pp->r_outer_frac;
}

/*
 * scene_spawn_particle — activate one particle at the outer edge.
 *
 *   r_min, r_max : where to draw the spawn radius. Normal use sets
 *                  this to a small band near r_outer; pre-warm uses
 *                  the FULL valid range so the spiral fills from
 *                  frame 1.
 */
static void scene_spawn_particle(Scene *s, float r_min, float r_max)
{
    int idx = particle_pool_find_inactive(s);
    if (idx < 0) return;
    Particle *p = &s->particles[idx];

    float r1 = lcg_unit(&s->rng);
    float r2 = lcg_unit(&s->rng);

    p->r      = r_min + r1 * (r_max - r_min);
    p->theta  = r2 * 2.0f * (float)M_PI;
    p->age    = 0.0f;
    p->active = true;
}

static void scene_prewarm(Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int target = pp->target_count;
    if (target > MAX_PARTICLES) target = MAX_PARTICLES;

    int active = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (s->particles[i].active) active++;

    float r_outer = scene_r_outer(s);
    /* Spawn across the FULL r range so the spiral is fully populated. */
    for (int k = active; k < target; k++)
        scene_spawn_particle(s, R_DRAIN_CELLS + 1.0f, r_outer);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_WHIRLPOOL;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    s->time_accum      = 0.0f;
    scene_clear_particles(s);
    scene_prewarm(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    scene_clear_particles(s);
    scene_prewarm(s);
}

/* Compute (dr/dt, dθ/dt) at the given radius for the active pattern. */
static inline void scene_polar_rates(const PatternParams *pp, float r,
                                     float *out_dr, float *out_dtheta)
{
    float r_safe = r > R_MIN_DENOM ? r : R_MIN_DENOM;
    *out_dr     = -(pp->inflow_log * r + pp->inflow_const);
    *out_dtheta = pp->angular_const + pp->angular_kepler / r_safe;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    s->time_accum += dt;

    const PatternParams *pp = &pattern_params[s->current_pattern];
    float r_outer = scene_r_outer(s);

    /* 1. Top up to target. */
    int active = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (s->particles[i].active) active++;
    int target    = pp->target_count;
    if (target > MAX_PARTICLES) target = MAX_PARTICLES;
    int spawn_cap = (int)((float)pp->target_count * dt * 4.0f) + 4;
    int to_spawn  = target - active;
    if (to_spawn > spawn_cap) to_spawn = spawn_cap;
    /* Re-spawn at the OUTER edge band only — replenishes drained particles. */
    for (int k = 0; k < to_spawn; k++)
        scene_spawn_particle(s, r_outer * 0.85f, r_outer);

    /* 2. Integrate polar kinematics. */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &s->particles[i];
        if (!p->active) continue;

        float dr, dtheta;
        scene_polar_rates(pp, p->r, &dr, &dtheta);

        p->r     += dr * dt;
        p->theta += dtheta * dt;
        p->age   += dt;

        if (p->r < R_DRAIN_CELLS) {
            p->active = false;
        }
    }
}

/*
 * polar_to_cell — convert (r, θ) to terminal cell coords.
 *
 * cx, cy in cells; ASPECT_Y compensates for tall cells so that a
 * polar circle renders as a round circle on screen, not an ellipse.
 */
static inline void polar_to_cell(float r, float theta, float cx, float cy,
                                 float *out_x, float *out_y)
{
    *out_x = cx + r * cosf(theta);
    *out_y = cy + r * sinf(theta) / ASPECT_Y;
}

static void scene_draw(const Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int rows_eff = s->rows - 1;     /* leave bottom row for HUD */
    float cx     = (float)s->cols * 0.5f;
    float cy     = (float)rows_eff * 0.5f;
    float r_outer = scene_r_outer(s);

    /* ── 1. Particles + tangent trails ─────────────────────────────── */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &s->particles[i];
        if (!p->active) continue;

        /* Compute brightness slot from radial position: outer = dim,
         * inner = bright (particle "heats up" approaching drain). */
        float f = 1.0f - p->r / (r_outer + 1.0f);
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        int head_slot = (int)(f * 7.0f + 0.5f);
        if (head_slot < 0) head_slot = 0;
        if (head_slot > 7) head_slot = 7;

        /* Compute back-step deltas for trail (from current dr/dt, dθ/dt). */
        float dr, dtheta;
        scene_polar_rates(pp, p->r, &dr, &dtheta);
        float dr_back     = -dr     * TRAIL_STEP_DT;   /* > 0 — r grew in past   */
        float dtheta_back =  dtheta * TRAIL_STEP_DT;   /* > 0 — θ smaller in past*/

        /* Render trail (tail to head) so head overdraws trail at intersections. */
        for (int t = TRAIL_LEN; t >= 0; t--) {
            float r_at     = p->r     + dr_back     * (float)t;
            float theta_at = p->theta - dtheta_back * (float)t;
            float fx, fy;
            polar_to_cell(r_at, theta_at, cx, cy, &fx, &fy);
            int ix = (int)(fx + 0.5f);
            int iy = (int)(fy + 0.5f);
            if (ix < 0 || ix >= s->cols) continue;
            if (iy < 0 || iy >= rows_eff) continue;

            int slot = head_slot - t;
            if (slot < 0) slot = 0;
            char glyph = RAMP_GLYPHS[slot];
            int  attr  = (slot >= 6) ? A_BOLD
                       : (slot <= 1) ? A_DIM
                       :               A_NORMAL;
            int  pair  = PAIR_RAMP_BASE + slot;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(iy, ix, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* ── 2. Centre drain glyph ─────────────────────────────────────── */
    int icx = (int)(cx + 0.5f);
    int icy = (int)(cy + 0.5f);
    if (icx >= 0 && icx < s->cols && icy >= 0 && icy < rows_eff) {
        char g = pp->center_glyph;
        int  attr = A_BOLD;
        /* BLACK_HOLE pulses: alternate between dim and bold over time. */
        if (pp->center_pulse) {
            float pulse = 0.5f + 0.5f * sinf(s->time_accum * 3.0f);
            if (pulse < 0.4f) attr = A_NORMAL;
        }
        attron(COLOR_PAIR(PAIR_CENTER) | attr);
        mvaddch(icy, icx, (chtype)(unsigned char)g);
        attroff(COLOR_PAIR(PAIR_CENTER) | attr);
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

static int scene_active_count(const Scene *s)
{
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (s->particles[i].active) n++;
    return n;
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int active = scene_active_count(s);

    const char *state_str = s->paused ? "PAUSED    " : pattern_name(s->current_pattern);

    char buf[200];
    snprintf(buf, sizeof buf,
             " VORTEX   %s   theme:%-10s   particles:%4d   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name, active,
             fps, sim_fps, s->speed);

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
        scene_prewarm(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_prewarm(s);
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
