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
 *       Patterns (10 total — same engine, different physics tuning):
 *         WHIRLPOOL    log-spiral inflow, smooth water-vortex feel.
 *         TORNADO      strong Kepler boost — tight high-speed core.
 *         BLACK_HOLE   extreme inflow + spin near centre, with central
 *                      pulse.
 *         SINK         constant inflow + constant ω, classic bathtub-
 *                      drain look.
 *         GALAXY       slow majestic spiral arms — pure log-spiral
 *                      (no Kepler), highest density, fills the canvas.
 *         HURRICANE    wide diameter with moderate Kepler boost —
 *                      visible eye-wall structure, weather-system feel.
 *         NEBULA       sparse slow drift inward — dust cloud, the
 *                      calmest pattern.
 *         CYCLONE      uniform SOLID-BODY rotation — no Kepler boost,
 *                      every particle shares the same ω regardless of
 *                      radius, spiral reads like a rotating disc.
 *         MAELSTROM    everything dialed up + central pulse — chaotic
 *                      intense storm, the most energetic pattern.
 *         PULSAR       tight small-radius spiral with strong Kepler +
 *                      pulsing centre — neutron-star / lighthouse feel.
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
 *   n / N      next pattern   (WHIRLPOOL → TORNADO → BLACK_HOLE → SINK →
 *                              GALAXY → HURRICANE → NEBULA → CYCLONE →
 *                              MAELSTROM → PULSAR)
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
 *                  At MAX_PARTICLES = 1000 and TRAIL_LEN = 4, ~5000
 *                  mvaddch per frame at peak density.  Trivial at 60 fps.
 *
 * References
 * ──────────
 *   PAPERS
 *     Reeves, W. T. (1983)
 *       "Particle Systems — A Technique for Modeling a Class of
 *       Fuzzy Objects"
 *       ACM Transactions on Graphics 2(2): 91-108.
 *       Foundational paper.  The pool / spawn / integrate / cull /
 *       draw skeleton in §5 is straight from Reeves §3.  §4 covers
 *       parameterised natural-phenomena pools — the PatternParams
 *       table here is exactly that design.
 *
 *     Kepler, J. (1609)
 *       "Astronomia Nova" — esp. Ch. 32 (second law: equal areas
 *       in equal times).
 *       Source of the "ω ∝ 1/r" intuition behind `angular_kepler`.
 *       Kepler's second law is a statement of angular-momentum
 *       conservation: the radius vector sweeps equal areas in
 *       equal time, so when r shrinks, ω must grow to compensate.
 *       The β_kepler / r term in dθ/dt is the visual analogue —
 *       it's gentler than the true 1/r² of point-particle gravity
 *       so the spin-up reads cleanly without blowing up.
 *
 *   BOOKS
 *     Goldstein, H., Poole, C. P. & Safko, J. L. (2002)
 *       "Classical Mechanics" (3rd ed, Addison-Wesley).
 *       Ch. 3 — Central-force motion.  Derives the conservation
 *       of angular momentum L = m·r·v_θ that backs the
 *       angular_kepler 1/r approximation.  Ch. 1 §1.2 covers
 *       Lagrangian polar coordinates and the radial / angular
 *       equations of motion (dr/dt and dθ/dt) that scene_tick
 *       integrates with explicit Euler.
 *
 *     Bourg, D. M. & Bywalec, B. (2013)
 *       "Physics for Game Developers" (2nd ed, O'Reilly).
 *       Ch. 8 §1 — polar-coordinate motion and the parametric
 *       circle x(t) = R·cos(ωt), y(t) = R·sin(ωt) used in
 *       polar_to_cell().  Ch. 2-3 — the explicit-Euler
 *       integration step that drives r += dr·dt, θ += dθ·dt.
 *
 *     Acheson, D. J. (1990)
 *       "Elementary Fluid Dynamics" (Oxford University Press).
 *       Ch. 5 — Vortex motion.  Gives the physical context for
 *       the ω(r) profiles available here: SOLID-BODY rotation
 *       (CYCLONE: ω constant across radii), POTENTIAL VORTEX
 *       (Kepler-like, ω ∝ 1/r — TORNADO, BLACK_HOLE), and the
 *       RANKINE COMBINED model (interior solid + exterior
 *       potential, the visual that WHIRLPOOL / HURRICANE evoke).
 *
 *     Akenine-Möller, T., Haines, E. & Hoffman, N. (2018)
 *       "Real-Time Rendering" (4th ed, CRC Press).
 *       §13.7 — point-sprite particle rendering.  The
 *       inner-fraction → ramp-slot mapping used for RAMP_GLYPHS
 *       (outer dim → inner bright) is the ASCII analogue of
 *       size-by-distance point-sprite sizing described there.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — DATA-DRIVEN PATTERN ENGINE ────────────────────────── *
 *
 * Ten visually distinct vortex effects (WHIRLPOOL, TORNADO,
 * BLACK_HOLE, SINK, GALAXY, HURRICANE, NEBULA, CYCLONE, MAELSTROM,
 * PULSAR) are produced by a SINGLE generic polar-physics engine
 * driven by an array of PatternParams structs.  scene_tick and
 * scene_draw never branch on the pattern enum to decide HOW the
 * physics works — they read pattern_params[s->current_pattern] and
 * compute accordingly.  Adding a new pattern is a matter of
 * appending one row to pattern_params[]; no new code paths.
 *
 *
 * THE GENERIC ENGINE (pseudocode)
 * ───────────────────────────────
 *
 *   loop forever (each tick of dt seconds):
 *
 *     pp = pattern_params[scene.current_pattern]   # read inputs
 *     r_outer = compute_outer_radius(scene, pp)
 *
 *     # 1. SPAWN until population reaches target
 *     while count(active particles) < pp.target_count:
 *         p = next_inactive_slot()
 *         p.theta  = uniform(0, 2π)
 *         p.r      = uniform(0.85·r_outer, r_outer)
 *         p.age    = 0
 *         p.active = true
 *
 *     # 2. INTEGRATE every active particle (polar Euler step)
 *     for p in active particles:
 *         dr_dt     = -(pp.inflow_log · p.r + pp.inflow_const)
 *         dtheta_dt =  pp.angular_const + pp.angular_kepler / max(p.r, R_MIN)
 *         p.r      += dr_dt     · dt
 *         p.theta  += dtheta_dt · dt
 *         p.age    += dt
 *
 *     # 3. DRAIN — particles that reach the centre die
 *     for p in active particles:
 *         if p.r < R_DRAIN:  p.active = false
 *
 *     # 4. RENDER — convert (r, θ) → cell, paint heat-ramp glyph
 *     for p in active particles:
 *         (cx_p, cy_p) = polar_to_cell(p.r, p.theta, cx, cy)
 *         slot         = inner_fraction(p.r, r_outer) · 7
 *         paint(cx_p, cy_p, ramp[slot])
 *         paint_tangent_trail(p, dr_dt, dtheta_dt)   # back-step on local arc
 *     paint_centre_glyph(cx, cy, pp.center_glyph,
 *                        pulse=pp.center_pulse and sin(time)-modulated)
 *
 *
 * PATTERNPARAMS FIELD → ENGINE HOOK
 * ─────────────────────────────────
 *
 *   target_count     →  spawn-loop refill cap        (density)
 *   r_outer_frac     →  outer spawn radius           (vortex extent)
 *   inflow_log       →  dr/dt:  −α·r                 (exponential pull)
 *   inflow_const     →  dr/dt:  −α                   (constant pull)
 *   angular_const    →  dθ/dt:  +β                   (uniform spin)
 *   angular_kepler   →  dθ/dt:  +β/r                 (Kepler spin-up)
 *   center_glyph     →  draw step                    (drain marker)
 *   center_pulse     →  draw step                    (heartbeat throb)
 *
 * Every other engine constant — R_DRAIN_CELLS, ASPECT_Y,
 * R_MIN_DENOM, TRAIL_LEN, TRAIL_STEP_DT, MAX_PARTICLES — is a
 * GLOBAL tuning knob shared across all patterns.  Patterns differ
 * ONLY in the eight fields above.
 *
 *
 * ARCHITECTURAL REFERENCES
 * ────────────────────────
 *
 *   Reeves, W. T. (1983)
 *     "Particle Systems — A Technique for Modeling a Class of
 *     Fuzzy Objects", ACM TOG 2(2): 91-108.
 *     §4 makes the explicit argument that ONE engine + a struct
 *     of physical constants per phenomenon is the right
 *     architecture for natural-particle simulations.  Reeves
 *     enumerates fire, fountains, fireworks, smoke, sparks — all
 *     driven by the same engine, varying only their parameter
 *     struct.  This file is a direct application of that idea to
 *     vortices.
 *
 *   Gamma, E., Helm, R., Johnson, R. & Vlissides, J. (1994)
 *     "Design Patterns: Elements of Reusable Object-Oriented
 *     Software" (Addison-Wesley) — STRATEGY pattern (§5.9).
 *     A family of algorithms (here: ten vortex characters)
 *     interchangeable behind a single interface (the engine
 *     reading PatternParams).  In a procedural-C setting the
 *     "interface" is the struct shape; the "concrete strategies"
 *     are the rows of pattern_params[]; "selecting a strategy"
 *     is updating scene.current_pattern.
 *
 *   Acton, M. (2014)
 *     "Data-Oriented Design and C++" (CppCon 2014 keynote).
 *     Argues that variation between behaviours should be
 *     represented as DATA (struct fields) rather than as control
 *     flow (if/switch on type).  pattern_params[] is a compact
 *     data table; the engine has no per-pattern code paths and
 *     the cache footprint stays predictable across pattern
 *     switches.
 *
 *   Nystrom, R. (2014)
 *     "Game Programming Patterns" (Genever Benning).
 *     TYPE OBJECT chapter — PatternParams is a Type Object: one
 *     shared instance per "kind" of vortex, every Particle
 *     implicitly references it via Scene.current_pattern.  DATA
 *     LOCALITY chapter — Particle is a flat-laid-out POD swept
 *     linearly by the integrator each tick, exactly the layout
 *     Nystrom recommends for hot inner loops.
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
 * ALGORITHM IN STEPS
 * ──────────────────
 * (Each step names the helper it dispatches to in §5; the drivers
 * scene_tick and scene_draw are pure pseudocode over these.)
 *
 *  1. SPAWN.
 *       active   = count_active_particles(s)
 *       to_spawn = compute_spawn_count_for_tick(active, pp, dt)
 *       particle_pool_topup_at_outer_band(s, r_outer, to_spawn)
 *     Each new particle drops in at the OUTER edge band:
 *       θ = uniform(0, 2π);  r = uniform(0.85·r_outer, r_outer)
 *
 *  2. INTEGRATE PER PARTICLE.  integrate_particle_polar_euler reads
 *     scene_polar_rates(pp, r) for (dr, dθ) and applies explicit
 *     Euler in (r, θ):
 *       dr/dt = −(α_log · r + α_const)
 *       dθ/dt =  β_const  + β_kepler / max(r, R_MIN_DENOM)
 *       r += dr·dt;   θ += dθ·dt;   age += dt
 *     (Goldstein Ch. 1 §1.2 — polar Lagrangian; Bourg Ch. 2-3 — Euler step.)
 *
 *  3. DRAIN.  particle_drained predicate (r < R_DRAIN_CELLS) — slot
 *     reclaimed for the next spawn.
 *
 *  4. RENDER PARTICLES.  particle_pool_draw → trail_draw_one_particle.
 *     For each active particle:
 *       head_slot = inner_fraction_to_ramp_slot(r, r_outer)   # outer dim → drain bright
 *       (dr_back, dθ_back) = compute_trail_back_step(pp, p)   # local arc
 *       for t in TRAIL_LEN..0:                                # tail → head
 *         (cell_x, cell_y) = polar_to_cell(r + dr_back·t,
 *                                          θ − dθ_back·t,
 *                                          cx, cy)
 *         slot = head_slot − t
 *         paint(cell_x, cell_y, RAMP_GLYPHS[slot],
 *               head_attr_by_brightness_slot(slot))
 *
 *  5. RENDER CENTRE.  center_drain_draw paints pp.center_glyph at
 *     (cx, cy) with center_pulse_attr modulating A_BOLD on a 3 rad/s
 *     sin clock for BLACK_HOLE / MAELSTROM / PULSAR.  Other patterns
 *     stay statically A_BOLD.
 *
 *  6. HUD.  Two-row layout painted by §6 screen — top row (status)
 *     PAIR_HUD + A_BOLD, bottom row (key hints) PAIR_HINT + A_BOLD.
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
    PATTERN_GALAXY     = 4,
    PATTERN_HURRICANE  = 5,
    PATTERN_NEBULA     = 6,
    PATTERN_CYCLONE    = 7,
    PATTERN_MAELSTROM  = 8,
    PATTERN_PULSAR     = 9,
    N_PATTERNS         = 10,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_WHIRLPOOL:  return "WHIRLPOOL ";
    case PATTERN_TORNADO:    return "TORNADO   ";
    case PATTERN_BLACK_HOLE: return "BLACK_HOLE";
    case PATTERN_SINK:       return "SINK      ";
    case PATTERN_GALAXY:     return "GALAXY    ";
    case PATTERN_HURRICANE:  return "HURRICANE ";
    case PATTERN_NEBULA:     return "NEBULA    ";
    case PATTERN_CYCLONE:    return "CYCLONE   ";
    case PATTERN_MAELSTROM:  return "MAELSTROM ";
    case PATTERN_PULSAR:     return "PULSAR    ";
    default:                 return "?         ";
    }
}

/*
 * PatternParams — per-pattern physics + visual signature for one
 * of the ten vortex effects.
 *
 * REFERENCES (each cited at the field it backs):
 *   Reeves, W. T. (1983) "Particle Systems — A Technique for
 *     Modeling a Class of Fuzzy Objects", ACM TOG 2(2): 91-108.
 *     §4 argues that natural-phenomena particle pools should be
 *     parameterised by a small struct of physical constants.
 *     PatternParams IS that struct — one row of pattern_params[]
 *     per effect.  scene_tick never branches on the pattern enum
 *     for the physics; it just reads these fields.
 *
 *   Kepler, J. (1609) "Astronomia Nova" — second law (equal
 *     areas in equal time).  The `angular_kepler` field weights
 *     the ω ∝ 1/r term in dθ/dt — Kepler's L-conservation
 *     visualised as faster spin near the drain.
 *
 *   Acheson, D. J. (1990) "Elementary Fluid Dynamics", Ch. 5
 *     (Vortex motion).  The (angular_const, angular_kepler) pair
 *     selects between three classical vortex profiles:
 *       angular_kepler = 0            → SOLID-BODY rotation
 *                                       (CYCLONE, GALAXY)
 *       angular_kepler ≫ angular_const → POTENTIAL VORTEX
 *                                       (TORNADO, BLACK_HOLE, PULSAR)
 *       both nonzero, balanced       → RANKINE COMBINED model
 *                                       (WHIRLPOOL, HURRICANE,
 *                                        MAELSTROM)
 *
 *   Bourg, D. M. & Bywalec, B. (2013) "Physics for Game
 *     Developers", Ch. 2-3.  inflow_log + inflow_const follow
 *     Bourg's two-term radial-velocity decomposition: an
 *     exponential-decay term proportional to r plus a constant pull.
 *
 * INTENT
 *   Same simulation engine, ten effects — varying these eight
 *   fields produces WHIRLPOOL, TORNADO, BLACK_HOLE, SINK, GALAXY,
 *   HURRICANE, NEBULA, CYCLONE, MAELSTROM, PULSAR.  Tuning a
 *   pattern is a matter of editing one row of pattern_params[];
 *   no scene_tick or scene_draw code change is needed for new
 *   patterns within this parameter space.
 *
 * FIELDS:
 *   target_count    Steady-state active particle count the spawn
 *                   loop refills toward each tick (per-tick cap
 *                   proportional to dt so a long pause doesn't
 *                   dump everyone at once).  Higher = denser
 *                   spiral.  Range: 300 (NEBULA — sparse drift)
 *                   to 800 (GALAXY — dense arms).  Reeves 1983 §3
 *                   — "steady-state pool".
 *
 *   r_outer_frac    Outer spawn radius as a fraction of the
 *                   half-screen size (after ASPECT_Y compensation).
 *                   Smaller = tighter vortex.  Range: 0.65 (PULSAR
 *                   — small dense core) to 0.98 (HURRICANE — wide
 *                   weather-system scale).
 *
 *   inflow_log      α_log — exponential inflow rate, units 1/s.
 *                   dr/dt contribution: −α_log · r.  Solo
 *                   signature is the log-spiral
 *                   r(t) = r₀ · exp(−α_log · t) — equal-angle
 *                   convergence, the GALAXY arm shape.  Range: 0.04
 *                   (NEBULA, GALAXY — slow drift) to 0.55
 *                   (BLACK_HOLE — rapid pull).
 *
 *   inflow_const    α_const — constant inflow speed, cells/s.
 *                   dr/dt contribution: −α_const.  Linear inflow
 *                   that doesn't slow down as r shrinks — the
 *                   bathtub-drain feel of SINK (α_const = 4.0,
 *                   α_log = 0).  Range: 0 (GALAXY — pure log
 *                   inflow) to 4.0 (SINK).
 *
 *   angular_const   β_const — steady angular velocity at every
 *                   radius, rad/s.  dθ/dt contribution: +β_const.
 *                   Combined with α_log alone produces a clean
 *                   log-spiral; combined with α_const gives the
 *                   SINK look.  Range: 0.20 (NEBULA — barely
 *                   turning) to 2.20 (CYCLONE — fast solid-body
 *                   rotation).
 *
 *   angular_kepler  β_kepler — strength of the ω ∝ 1/r angular
 *                   boost, units rad·cell/s.  dθ/dt contribution:
 *                   +β_kepler / max(r, R_MIN_DENOM).  Mimics
 *                   Kepler's second law — when r shrinks, ω spikes.
 *                   Three regimes:
 *                     0       → flat ω profile, particles trace
 *                               concentric circles at the same
 *                               rate (CYCLONE, GALAXY, WHIRLPOOL)
 *                     4 – 5.5 → tight whip near the centre
 *                               (TORNADO, MAELSTROM, PULSAR)
 *                     8       → extreme spin-up — accretion disc
 *                               (BLACK_HOLE)
 *
 *   center_glyph    Single ASCII character drawn at the drain
 *                   centre.  Per-pattern visual signature:
 *                     '.'  WHIRLPOOL, NEBULA   '*'  TORNADO, GALAXY
 *                     '@'  BLACK_HOLE, HURRICANE
 *                     'O'  SINK         'o'  CYCLONE
 *                     '+'  PULSAR       '#'  MAELSTROM
 *
 *   center_pulse    Non-zero → the centre glyph alternates between
 *                   A_BOLD and A_NORMAL on a sin(time_accum · 3)
 *                   clock, giving a heartbeat / accretion-disc
 *                   throb.  Set on BLACK_HOLE, MAELSTROM, PULSAR.
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
    /*                  target  rfrac  inLog  inConst  angC    angKep  ctr  pulse */
    /* WHIRLPOOL  */ {  500,   0.85f,  0.40f,  0.30f,  1.40f,  0.0f,  '.',  0 },
    /* TORNADO    */ {  600,   0.85f,  0.20f,  1.20f,  0.80f,  4.0f,  '*',  0 },
    /* BLACK_HOLE */ {  700,   0.95f,  0.55f,  2.00f,  0.50f,  8.0f,  '@',  1 },
    /* SINK       */ {  450,   0.85f,  0.00f,  4.00f,  2.00f,  1.0f,  'O',  0 },
    /* GALAXY     */ {  800,   0.95f,  0.06f,  0.00f,  0.50f,  0.0f,  '*',  0 },
    /* HURRICANE  */ {  600,   0.98f,  0.15f,  0.10f,  1.00f,  2.5f,  '@',  0 },
    /* NEBULA     */ {  300,   0.85f,  0.04f,  0.05f,  0.20f,  0.5f,  '.',  0 },
    /* CYCLONE    */ {  550,   0.90f,  0.20f,  0.50f,  2.20f,  0.0f,  'o',  0 },
    /* MAELSTROM  */ {  700,   0.85f,  0.45f,  1.80f,  1.60f,  4.0f,  '#',  1 },
    /* PULSAR     */ {  400,   0.65f,  0.30f,  0.60f,  1.40f,  5.5f,  '+',  1 },
};

/*
 * Themes — each is an 8-step gradient from OUTER (slot 0, dim) to
 * INNER (slot 7, bright).  Particles near the drain take the
 * brightest slot; outer arms fade through the lower slots.  Each
 * theme commits to ONE hue family so the t / T cycle is visually
 * obvious — no two themes share a colour story.  NOVA is the
 * standout, the only multi-hue ramp (blue → magenta → orange →
 * yellow → white) so a real supernova spectrum sweeps through the
 * spiral arms.
 *
 *   MATRIX   pure phosphor green        FOREST   olive → leaf → gold
 *   FIRE     coal → flame → white-hot   DESERT   dune brown → cream
 *   OCEANIC  deep blue → teal (no white) NEON    purple → hot pink
 *   ICE      pale frost → pure white    ECLIPSE  void purple → crimson
 *   NOVA     blue → magenta → yellow    MONO     grayscale
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule, so even ramp[0]
 * (the dimmest "outer arm" colour) stays visible against a black
 * terminal with A_DIM applied.  `center` picks the brightest hue
 * that complements the ramp — it paints the drain glyph and pops
 * against the spiral.
 */
typedef struct {
    const char *name;
    short       ramp[8];   /* outer dim → inner bright */
    short       center;
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        ramp[0..7]  (outer dim → inner bright)              center  sky */
    { "MATRIX",   {  28,  34,  40,  46,  82, 118, 154, 190 },           226,  234 },  /* phosphor green       */
    { "FIRE",     {  52,  88, 124, 160, 202, 208, 220, 231 },           231,  233 },  /* coal→flame→white-hot */
    { "OCEANIC",  {  24,  25,  32,  38,  44,  80, 116, 152 },           195,  234 },  /* deep blue → teal     */
    { "NEON",     {  54,  92, 128, 165, 201, 207, 213, 219 },           225,  234 },  /* purple → hot pink    */
    { "MONO",     { 240, 244, 247, 250, 252, 253, 254, 255 },           255,  232 },  /* grayscale            */
    { "ICE",      { 117, 153, 159, 195, 231, 251, 253, 255 },           255,  235 },  /* pale frost → white   */
    { "NOVA",     {  27,  99, 165, 201, 208, 220, 226, 231 },           231,  234 },  /* SPECTRUM (multi-hue) */
    { "FOREST",   {  58,  64,  70, 106, 142, 148, 184, 220 },           226,  234 },  /* olive → leaf → gold  */
    { "DESERT",   {  94, 130, 137, 173, 215, 222, 228, 230 },           230,  234 },  /* dune → sand → cream  */
    { "ECLIPSE",  {  53,  54,  89, 125, 161, 197, 204, 209 },           219,  232 },  /* void purple → crimson*/
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

/*
 * Particle — one swirling speck in the vortex.
 *
 * REFERENCES:
 *   Reeves, W. T. (1983) "Particle Systems — A Technique for
 *     Modeling a Class of Fuzzy Objects", ACM TOG 2(2): 91-108.
 *     The fixed pool + per-particle state + lifetime-via-drain-cull
 *     model is straight from Reeves §3.  §4 specifically discusses
 *     the parameterised natural-phenomena pools that PatternParams
 *     implements above.
 *
 *   Goldstein, H., Poole, C. P. & Safko, J. L. (2002) "Classical
 *     Mechanics" (3rd ed, Addison-Wesley).  Ch. 1 §1.2 — the
 *     polar (r, θ) representation that lets a TWO-component
 *     state describe the same motion that Cartesian would need
 *     four components for (vx, vy, ax, ay).  Ch. 3 — central-
 *     force motion: the conservation of angular momentum that
 *     backs the angular_kepler / r term in dθ/dt.
 *
 *   Kepler, J. (1609) "Astronomia Nova" — second law (equal
 *     areas in equal time).  When r shrinks, ω must grow to
 *     keep L constant — exactly what scene_polar_rates produces
 *     via the β_kepler / r term that this struct's `r` field
 *     feeds.
 *
 *   Bourg, D. M. & Bywalec, B. (2013) "Physics for Game
 *     Developers" (2nd ed, O'Reilly).  Ch. 8 §1 — polar →
 *     Cartesian conversion (cell coordinates) used in
 *     polar_to_cell at render time.
 *
 * INTENT
 *   POLAR coords instead of Cartesian.  Each particle's state is
 *   (r, θ) — radius from drain + angle around drain — so the
 *   physics integrator only ever computes dr/dt and dθ/dt; the
 *   spiral curve EMERGES from those scalar updates without ever
 *   computing a spiral path explicitly.  Cartesian conversion
 *   happens once at render time in polar_to_cell.
 *
 *   Flat layout fits the fixed-size BSS pool — MAX_PARTICLES of
 *   these allocated once at program start, no malloc after init.
 *   The `active` flag is the single source of truth for pool-slot
 *   occupancy; spawn finds the first inactive index via a linear
 *   scan in particle_pool_find_inactive.
 *
 * LIFECYCLE
 *   SPAWN (scene_spawn_particle):
 *     random θ ∈ [0, 2π), r near r_outer (or anywhere in the
 *     valid range during pre-warm), age = 0, active = true.
 *
 *   INTEGRATE (scene_tick):
 *     scene_polar_rates produces (dr, dθ) for the current radius
 *     under the active pattern.  Explicit Euler:
 *       r += dr · dt;   θ += dθ · dt;   age += dt
 *
 *   DRAIN (scene_tick): r drops below R_DRAIN_CELLS → active=false,
 *     slot reclaimed for the next spawn.  No outward escape — every
 *     particle ends at the drain.
 *
 *   RENDER (scene_draw):
 *     polar_to_cell(r, θ) → (ix, iy); colour from inner-fraction
 *     f = 1 − r / r_outer (outer = dim, drain = bright).  Tangent
 *     trail back-stepped from the local (dr, dθ) so the streak
 *     follows the spiral arc.
 *
 * FIELDS:
 *   r            Radius from the drain centre, in cells (after
 *                ASPECT_Y normalisation, so a polar circle renders
 *                round on screen instead of a vertical ellipse).
 *                Always positive; clamped to R_MIN_DENOM in the
 *                denominator of the β_kepler / r term so the
 *                integrator can't divide by zero (Kepler 1609 →
 *                Goldstein Ch. 3 — the 1/r factor for L conservation).
 *
 *   theta        Angle around the drain in radians.  Grows
 *                MONOTONICALLY — no wrap to [0, 2π).  cos/sin
 *                handle huge values cleanly and float precision
 *                is plenty even after thousands of revolutions
 *                (a typical particle dies at r = R_DRAIN before
 *                completing more than a few turns).
 *
 *   age          Seconds since spawn.  Currently unused by the
 *                integrator (death is r-based, not age-based) and
 *                unused by the renderer (brightness comes from r,
 *                not age) — kept available for future per-particle
 *                age effects (e.g. fade-in at spawn, lifetime cap).
 *
 *   active       Pool-slot occupancy flag.  Inactive slots are
 *                skipped by every loop; spawn finds the first
 *                inactive index via a linear scan
 *                (O(MAX_PARTICLES), free at MAX_PARTICLES = 1000
 *                and 60 fps).  Reeves 1983 §3 — pool slot.
 */
typedef struct {
    float r;
    float theta;
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

/*
 * Scene — owns every piece of mutable state for the vortex demo.
 *
 * Two clearly-separated halves:
 *
 *   SIMULATION HALF — read + written by scene_tick.  Owns the
 *                     active pattern, RNG, cached terminal
 *                     dimensions, the global clock for the centre
 *                     pulse, and the particle pool.  Mutated by
 *                     scene_tick and the key handler.
 *
 *   RENDER HALF     — what scene_draw consults to pick colours.
 *                     Purely visual selection index; never read
 *                     inside the physics tick.
 *
 * LOCALITY
 *   Grouping by access pattern matters for two reasons.
 *   Conceptually: a reader scanning scene_tick wants every field
 *   that mutates each frame in one block (not interleaved with
 *   render-only fields), and vice versa for scene_draw.
 *   Mechanically: the hot path is the tick — its small simulation
 *   fields fit in one or two cache lines and stay warm across the
 *   whole tick.  current_theme sits in the render half because the
 *   physics tick never touches it.
 *
 * REFERENCES (cited inline at the relevant member):
 *   Reeves (1983)  — particle pool design (Scene.particles[])
 *   Kepler (1609)  — second law (Scene.particles[].r feeds
 *                    angular_kepler / r in dθ/dt)
 *
 * SEPARATION OF CONCERNS
 *   The Scene knows nothing about ncurses.  Physics writes to the
 *   pool + scalar state; the render layer reads them.  That split
 *   lets the simulation be exercised without a terminal (e.g. for
 *   profiling) and keeps the layering explicit.
 */
typedef struct {
    /* ──────────────────────────────────────────────────────────────
     *  SIMULATION HALF — physics tick reads + writes these
     * ────────────────────────────────────────────────────────────── */

    /* PAUSE — scene_tick early-returns when set.  Render keeps
     * running, so the user sees a frozen frame with particles
     * held mid-arc on their spiral.  Toggled by SPACE. */
    bool      paused;

    /* SPEED — integer multiplier on dt.  SPEED_DEF = 1× wall clock;
     * +/= keys double, − halves; bounded by SPEED_MIN/MAX.  Doesn't
     * change physics constants — just compresses or stretches
     * simulated time so the user can slow-mo / fast-forward the
     * same vortex without altering its character. */
    int       speed;

    /* PATTERN — index into pattern_params[] (WHIRLPOOL … PULSAR).
     * Cycled by n / N.  Switching pattern triggers a scene_prewarm
     * so the new physics has a full population from frame 1.  Read
     * by scene_tick (every physics term comes from
     * pattern_params[s->current_pattern]) AND scene_draw (centre
     * glyph + pulse); the only field used by both halves. */
    Pattern   current_pattern;

    /* RNG — per-scene LCG state, seeded from clock_ns() at init
     * and re-seeded (XOR'd with a sentinel) on r.  Used by every
     * randomness consumer: spawn angle θ ∈ [0, 2π), spawn radius
     * within the outer band.  No globals — full state in this
     * single 32-bit word. */
    uint32_t  rng;

    /* CACHED TERMINAL DIMENSIONS — read every frame by spawn
     * (r_outer needs cols/rows after ASPECT_Y compensation), and
     * the renderer (cell bounds + centre coords).  Cached at init
     * and on SIGWINCH so the hot path never calls getmaxyx().
     * Strictly speaking these are SHARED between sim and render —
     * both halves see the same dimensions — but they're written
     * only by the tick layer so they live with the SIMULATION block. */
    int       rows, cols;

    /* GLOBAL CLOCK — seconds since the scene started, advanced by
     * dt each tick.  Drives the centre-glyph pulse used by
     * BLACK_HOLE / MAELSTROM / PULSAR:
     *   pulse = 0.5 + 0.5 · sin(time_accum · 3.0)
     * Read by render (scene_draw) but written by tick — sits with
     * simulation. */
    float     time_accum;

    /* PARTICLE POOL — fixed-size BSS array, no allocation after
     * init.  See Particle for per-slot detail.
     * particle_pool_find_inactive linear-scans for the first
     * inactive index when spawn needs a slot.  Reeves 1983 §3 —
     * the particle pool. */
    Particle  particles[MAX_PARTICLES];

    /* ──────────────────────────────────────────────────────────────
     *  RENDER HALF — scene_draw reads this; physics tick ignores it
     * ────────────────────────────────────────────────────────────── */

    /* THEME — index into themes[].  Cycled by t / T.  Selects the
     * 8-step outer-→-inner ramp + drain centre + sky background.
     * Pure render concern — vortex physics behaves identically
     * regardless of theme.  theme_apply rewrites pairs
     * PAIR_RAMP_BASE..+7, PAIR_CENTER, and PAIR_SKY on change. */
    int       current_theme;
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

/* ── Tick orchestration helpers ───────────────────────────────────────── */

/* Population census — linear scan of pool's active flags. */
static int count_active_particles(const Scene *s)
{
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (s->particles[i].active) n++;
    return n;
}

/* Spawn budget for this tick — refill toward pattern.target_count,
 * but cap the per-tick burst proportional to dt so a long pause
 * doesn't dump the entire pool in a single frame on resume. */
static int compute_spawn_count_for_tick(int active, const PatternParams *pp,
                                        float dt)
{
    int target = pp->target_count;
    if (target > MAX_PARTICLES) target = MAX_PARTICLES;
    int spawn_cap = (int)((float)pp->target_count * dt * 4.0f) + 4;
    int n = target - active;
    if (n < 0)         n = 0;
    if (n > spawn_cap) n = spawn_cap;
    return n;
}

/* Spawn `n` new particles in the OUTER edge band [0.85·r_outer,
 * r_outer] — replenishes the drained-at-centre population by injecting
 * fresh particles on the periphery, where the pattern's inflow can
 * carry them all the way to the drain. */
static void particle_pool_topup_at_outer_band(Scene *s, float r_outer, int n)
{
    for (int k = 0; k < n; k++)
        scene_spawn_particle(s, r_outer * 0.85f, r_outer);
}

/* Explicit Euler step for one particle in polar coords (Goldstein
 * Ch. 1 §1.2 — polar Lagrangian).  Reads (dr, dθ) from
 * scene_polar_rates — the one place engine touches PatternParams'
 * physics knobs per-particle.  r += dr·dt, θ += dθ·dt, age += dt. */
static inline void integrate_particle_polar_euler(Particle *p,
                                                  const PatternParams *pp,
                                                  float dt)
{
    float dr, dtheta;
    scene_polar_rates(pp, p->r, &dr, &dtheta);
    p->r     += dr     * dt;
    p->theta += dtheta * dt;
    p->age   += dt;
}

/* Particle has drained at the centre — slot is reclaimable.  R_DRAIN_CELLS
 * is set far enough out that particles vanish before getting close to
 * the angular_kepler / r singularity at r = 0. */
static inline bool particle_drained(const Particle *p)
{
    return p->r < R_DRAIN_CELLS;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    dt *= (float)s->speed / (float)SPEED_DEF;
    s->time_accum += dt;

    const PatternParams *pp = &pattern_params[s->current_pattern];
    float r_outer = scene_r_outer(s);

    /* 1. Top up particle pool toward pattern's target density. */
    int active   = count_active_particles(s);
    int to_spawn = compute_spawn_count_for_tick(active, pp, dt);
    particle_pool_topup_at_outer_band(s, r_outer, to_spawn);

    /* 2. Integrate polar kinematics; drain at the centre. */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &s->particles[i];
        if (!p->active) continue;

        integrate_particle_polar_euler(p, pp, dt);
        if (particle_drained(p)) p->active = false;
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

/* ── Render primitives ────────────────────────────────────────────────── */

/* Inner-fraction → ramp-slot mapping.  f = 1 − r/r_outer makes
 * particles brighter as they fall toward the drain (the "heating up"
 * visual that mimics gravitational compression / Akenine-Möller §13.7
 * size-by-distance point-sprite analogue).  Returns slot ∈ [0, 7]. */
static inline int inner_fraction_to_ramp_slot(float r, float r_outer)
{
    float f = 1.0f - r / (r_outer + 1.0f);
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    int slot = (int)(f * 7.0f + 0.5f);
    if (slot < 0) slot = 0;
    if (slot > 7) slot = 7;
    return slot;
}

/* Brightness attribute by ramp slot — top of ramp BOLD, bottom DIM.
 * Shared between particle trail/head and drain centre so the gradient
 * reads consistently across all rendered elements. */
static inline int head_attr_by_brightness_slot(int slot)
{
    if (slot >= 6) return A_BOLD;
    if (slot <= 1) return A_DIM;
    return A_NORMAL;
}

/* Centre-glyph pulse for BLACK_HOLE / MAELSTROM / PULSAR — sinusoidal
 * heartbeat at 3 rad/sec angular frequency.  pulse = 0.5 + 0.5·sin(t·3);
 * below 0.4 we drop to A_NORMAL, otherwise stay A_BOLD — produces a
 * visible accretion-disc throb without fully extinguishing the glyph. */
static inline int center_pulse_attr(float time_accum)
{
    float pulse = 0.5f + 0.5f * sinf(time_accum * 3.0f);
    return (pulse < 0.4f) ? A_NORMAL : A_BOLD;
}

/* Back-step deltas for the tangent trail.  scene_polar_rates gives
 * the INSTANTANEOUS (dr, dθ) at the particle's current radius;
 * back-stepping by TRAIL_STEP_DT seconds traces along the LOCAL ARC
 * of the spiral — so the trail follows the spiral curve, not a
 * straight chord.  dr_back > 0 (r was bigger in the past, spiraling
 * inward); dtheta_back > 0 (θ was smaller in the past).*/
static inline void compute_trail_back_step(const PatternParams *pp,
                                           const Particle *p,
                                           float *dr_back, float *dtheta_back)
{
    float dr, dtheta;
    scene_polar_rates(pp, p->r, &dr, &dtheta);
    *dr_back     = -dr     * TRAIL_STEP_DT;
    *dtheta_back =  dtheta * TRAIL_STEP_DT;
}

/* Draw one particle as a fading tangent trail (TRAIL_LEN steps back
 * along the local arc) plus the bright head at t=0.  Older trail
 * points get cooler ramp slots so the streak fades behind the head.
 * Render order is tail → head so the head overdraws any earlier
 * trail cells at intersections (the bright dot the eye tracks is
 * never shadowed by a passing streak). */
static void trail_draw_one_particle(const Particle *p, const PatternParams *pp,
                                    int head_slot,
                                    float cx, float cy,
                                    int cols, int rows_eff)
{
    float dr_back, dtheta_back;
    compute_trail_back_step(pp, p, &dr_back, &dtheta_back);

    for (int t = TRAIL_LEN; t >= 0; t--) {
        float r_at     = p->r     + dr_back     * (float)t;
        float theta_at = p->theta - dtheta_back * (float)t;
        float fx, fy;
        polar_to_cell(r_at, theta_at, cx, cy, &fx, &fy);
        int ix = (int)(fx + 0.5f);
        int iy = (int)(fy + 0.5f);
        if (ix < 0 || ix >= cols)     continue;
        if (iy < 0 || iy >= rows_eff) continue;

        int slot = head_slot - t;
        if (slot < 0) slot = 0;
        char glyph = RAMP_GLYPHS[slot];
        int  attr  = head_attr_by_brightness_slot(slot);
        int  pair  = PAIR_RAMP_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* Render every active particle as trail + head — the spiral arms. */
static void particle_pool_draw(const Scene *s, float cx, float cy, int rows_eff)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    float r_outer = scene_r_outer(s);
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &s->particles[i];
        if (!p->active) continue;
        int head_slot = inner_fraction_to_ramp_slot(p->r, r_outer);
        trail_draw_one_particle(p, pp, head_slot, cx, cy, s->cols, rows_eff);
    }
}

/* Render the central drain glyph — the focal point at (cx, cy).
 * pulse-enabled patterns (BLACK_HOLE / MAELSTROM / PULSAR) modulate
 * the attribute via center_pulse_attr; static patterns stay A_BOLD. */
static void center_drain_draw(const Scene *s, float cx, float cy, int rows_eff)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int icx = (int)(cx + 0.5f);
    int icy = (int)(cy + 0.5f);
    if (icx < 0 || icx >= s->cols)  return;
    if (icy < 0 || icy >= rows_eff) return;

    int attr = pp->center_pulse ? center_pulse_attr(s->time_accum) : (int)A_BOLD;
    attron(COLOR_PAIR(PAIR_CENTER) | attr);
    mvaddch(icy, icx, (chtype)(unsigned char)pp->center_glyph);
    attroff(COLOR_PAIR(PAIR_CENTER) | attr);
}

static void scene_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;            /* leave bottom row for HUD */
    float cx = (float)s->cols * 0.5f;
    float cy = (float)rows_eff * 0.5f;

    particle_pool_draw(s, cx, cy, rows_eff);   /* spiral arms — trails+heads */
    center_drain_draw (s, cx, cy, rows_eff);   /* drain glyph at the focal pt*/
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

/*
 * screen_draw — render the scene, then paint a two-layer HUD over it:
 *
 *   Row 0        STATUS LINE.  Bright yellow PAIR_HUD + A_BOLD.
 *                Live state: pattern (or PAUSED), theme, particle
 *                count, fps, sim Hz, speed multiplier.
 *   Row rows-1   KEY HINT LINE.  Bright cyan PAIR_HINT + A_BOLD.
 *                Every interactive key the demo accepts.
 *
 * Both rows are pre-filled with their pair colour so the coloured
 * background spans the full width, and drawn AFTER scene_draw so
 * the spiral never bleeds through the bars.
 */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int active = scene_active_count(s);
    const char *state_str = s->paused ? "PAUSED    "
                                      : pattern_name(s->current_pattern);

    /* ── Top row: dynamic status ─────────────────────────────── */
    char status[220];
    snprintf(status, sizeof status,
             " VORTEX   %s   theme:%-8s   particles:%4d   "
             "%5.1f fps  %3d Hz  speed:%-3d ",
             state_str, themes[s->current_theme].name, active,
             fps, sim_fps, s->speed);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(0, x, ' ');
    mvprintw(0, 0, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* ── Bottom row: every interactive key ───────────────────── */
    const char *hints =
        " q:quit  spc:pause  r:reseed  n/p:pattern  t/T:theme  "
        "+/-:speed  ]/[:Hz ";

    int hint_row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(hint_row, x, ' ');
    mvprintw(hint_row, 0, "%s", hints);
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
