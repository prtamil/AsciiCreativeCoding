/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sparks.c — fast electric sparks with motion-blur trails, bouncing
 *            off the floor with elastic restitution.
 *
 * DEMO: Bright, FAST sparks fly out of an emitter on cone-shaped
 *       initial trajectories, arc under gravity, and BOUNCE off the
 *       floor (the row above the HUD) with energy loss given by the
 *       pattern's restitution coefficient. Each spark drags a short
 *       motion-blur trail of dimmer glyphs behind it — the head is
 *       hot/bright, the tail is dark/dim, a learner-readable picture
 *       of the spark's recent path. Distinct from `embers.c`:
 *
 *         embers.c — slow rising particles, no trail, no bounce,
 *                    cool over a long lifetime (fire flicker).
 *         sparks.c — FAST flying particles, motion-blur trails,
 *                    elastic floor bounces, short crackly lifetime.
 *
 *       Patterns (10 effects, each anchored where its physics reads
 *       cleanest):
 *         WELDING        (left-mid)  horizontal jet of orange-yellow
 *                                    sparks arcing rightward across
 *                                    the screen; floor bounces 1–2.
 *         GRINDER        (bottom)    tight up-and-right cone from the
 *                                    floor — very fast, strong gravity,
 *                                    multiple floor bounces.
 *         CAMPFIRE       (bottom)    gentle sparks straight up from
 *                                    the floor; soft gravity, sparse.
 *         TESLA          (centre)    short-lived omnidirectional
 *                                    crackle; low gravity.
 *         SPARKLERS      (centre)    tiny sparks in every direction;
 *                                    each parent pops MID-FLIGHT into
 *                                    3 smaller secondaries — a multi-
 *                                    level crackle like a real sparkler.
 *         SPINNER        (centre)    a rotating wheel at the hub
 *                                    (visible | / - \ axle) throws
 *                                    sparks TANGENTIALLY off two
 *                                    opposing rim points, painting a
 *                                    circular spiral pattern.
 *         FLOWERPOT      (bottom)    tall vertical fountain from the
 *                                    floor — high upward speed, strong
 *                                    gravity, dramatic arcs back down.
 *         CHRYSANTHEMUM  (centre)    large slow radial bloom — sparks
 *                                    hang in the air, fading together.
 *         WILLOW         (bottom)    wide upward fan from the floor
 *                                    that immediately droops — sparks
 *                                    coast outward as they fall,
 *                                    mimicking weeping-willow branches.
 *         WATERFALL      (top)       narrow downward cone from near
 *                                    the ceiling — sparks rain the
 *                                    full height of the screen.
 *
 * Study alongside:
 *   embers.c    — same pool / spawn / tick / draw skeleton, but
 *                 buoyancy + cooling instead of gravity + bounce.
 *                 Read first; sparks.c is the "FAST + BOUNCY"
 *                 counterpart.
 *   fountain.c  — also gravity + bouncing particles. Compare the
 *                 emit cone and trail rendering.
 *   fireworks.c — radial bursts; TESLA pattern is its closest cousin.
 *
 * Section map:
 *   §1 config   — constants, pattern params, theme heat ramps
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 8-pair heat ramp with 8-colour fallback
 *   §4 spark    — Spark struct + motion-blur trail history
 *   §5 scene    — pool, spawn, tick (integrate + bounce), draw (trail+head)
 *   §6 screen   — ncurses init / draw / resize
 *   §7 app      — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear pool)
 *   n / N      next pattern   (WELDING → GRINDER → CAMPFIRE → TESLA →
 *                              SPARKLERS → SPINNER → FLOWERPOT →
 *                              CHRYSANTHEMUM → WILLOW → WATERFALL)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster      (sim speed multiplier)
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *   w / W      shift emitter right / left
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/sparks.c \
 *       -o sparks -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system with one species
 *                  (spark) and a short motion-blur history per particle.
 *                  Each tick:
 *
 *                    0. WHEEL PHASE. Advance SPINNER's wheel rotation
 *                       by SPINNER_OMEGA·dt; wrap to [0, 2π).  Runs
 *                       every tick so the axle keeps turning even when
 *                       SPINNER isn't the active pattern.
 *
 *                    1. SPAWN. Top up the active pool to the pattern's
 *                       target.  Standard patterns sample from a
 *                       (angle, speed) CONE around the emitter; SPINNER
 *                       overrides with a TANGENT emission from a
 *                       rotating wheel rim.  SPARKLERS counts PARENTS
 *                       only — has_split=false sparks — so cascade
 *                       children don't throttle new parent emission.
 *
 *                    2. INTEGRATE. Push each spark's previous (x, y)
 *                       into a short ring of trail history, then update:
 *                          vy  += gravity · dt
 *                          v   *= drag_factor                (per tick)
 *                          x   += vx · dt
 *                          y   += vy · dt
 *                          age += dt
 *
 *                    3. BOUNCE. If the spark crosses the FLOOR while
 *                       moving downward, it bounces ELASTICALLY with
 *                       Hertz restitution e ∈ (0, 1):
 *                          y'  = floor_y − (y − floor_y)         // reflect
 *                          vy' = −vy · e                         // flip + lose
 * energy vx' = vx · floor_friction             // tangential friction If both
 * speed components are below the SETTLE threshold, the spark is killed (it
 * would otherwise micro-bounce forever).
 *
 *                    4. KILL. age >= life, off-screen, or settled.
 *
 *                    5. SPARKLERS CASCADE (this pattern only).  Each
 *                       parent (has_split=false) that has crossed
 *                       SPARKLER_SPLIT_AGE_FRAC of its life pops into
 *                       SPARKLER_CHILDREN_PER_SPLIT smaller secondaries
 *                       at its current (x, y).  has_split latches so
 *                       the cascade caps at 2 generations.
 *
 *                    6. RENDER. Draw each trail-history point as a dim
 *                       glyph from the cool end of the heat ramp; draw
 *                       the head with a hot/bold glyph from the high
 *                       end.  Head ramp slot reads remaining-life
 *                       fraction T = 1 − age/life.  SPINNER also paints
 *                       a rotating axle glyph at the wheel hub.
 *
 *                  Distinct from `embers.c` in three concrete ways:
 *
 *                    – embers have buoyancy (vy negative); sparks have
 *                      gravity (vy grows positive).
 *                    – embers have no trail; sparks store TRAIL_LEN
 *                      previous positions and render them dimly.
 *                    – embers die when they cool/exit; sparks also
 *                      bounce off the floor with restitution and die
 *                      when they settle (low |v|).
 *
 * Data-structure : Spark[MAX_SPARKS] object pool with `active` flag
 *                  and a small ring of `(trail_x, trail_y)` history.
 *                  No malloc, linear-scan spawn (small N — fine).
 *
 * Rendering      : ASCII only. Trail glyphs are sparse (`,` `.` `:`),
 *                  head glyphs are dense (`*` `+` `#`). Colour from a
 *                  theme heat ramp by remaining-life fraction; the
 *                  trail uses a slot 1–3 below the head's slot so it
 *                  fades behind the spark.
 *
 * Performance    : O(MAX_SPARKS · (1 + TRAIL_LEN)) per frame. With
 *                  TRAIL_LEN = 3 and MAX_SPARKS ≈ 800, that is at most
 *                  ~3200 mvaddch per frame. Trivial at 60 fps.
 *
 * References
 * ──────────
 *   PAPERS
 *     Reeves, W. T. (1983)
 *       "Particle Systems — A Technique for Modeling a Class of
 *       Fuzzy Objects"
 *       ACM Transactions on Graphics 2(2): 91-108.
 *       Foundational paper.  The pool-based spawn / integrate /
 *       cull / draw skeleton used in §5 is straight from Reeves §3.
 *       §4 covers natural-phenomena pools parameterised by a
 *       struct of physical constants — exactly the PatternParams
 *       design used for the ten pattern presets here.
 *
 *     Hertz, H. (1882)
 *       "Über die Berührung fester elastischer Körper"
 *       J. reine angew. Math. 92: 156-171.
 *       Classical contact-mechanics derivation of the coefficient
 *       of restitution e ∈ (0, 1).  This file's `pp->restitution`
 *       is exactly Hertz's e; the floor-bounce rule
 *       `vy' = −e · vy` is the macroscopic energy-loss model from
 *       Hertz's elastic-collision analysis applied to a 2-D
 *       particle.
 *
 *   BOOKS
 *     Millington, I. (2010)
 *       "Game Physics Engine Development" (2nd ed, Morgan Kaufmann).
 *       Ch. 6 "Particle Physics" — semi-implicit Euler integration
 *       and frame-rate-independent damping
 *       (drag_factor = exp(−drag_coeff · dt)) used here in
 *       scene_tick.  Ch. 7 "Particle Contacts" — the impulse-based
 *       bounce + settle/sleep criterion that backs SETTLE_VY /
 *       SETTLE_VX so settled sparks don't burn ticks micro-bouncing.
 *
 *     Bourg, D. M. & Bywalec, B. (2013)
 *       "Physics for Game Developers" (2nd ed, O'Reilly).
 *       Ch. 2-3 — ballistic motion under gravity + drag, basis of
 *       the gravity + drag integration step.  Ch. 8 §1 — the
 *       parametric circle x(t) = cx + R·cos(ω·t),
 *       y(t) = cy + R·sin(ω·t) that drives the SPINNER wheel's
 *       rotating emission points.
 *
 *     Akenine-Möller, T., Haines, E. & Hoffman, N. (2018)
 *       "Real-Time Rendering" (4th ed, CRC Press).
 *       §13.7 — point-sprite particle rendering.  The
 *       remaining-life → heat-ramp-slot mapping used for
 *       HEAD_GLYPHS / TRAIL_GLYPHS is the ASCII analogue of
 *       size-by-mass point-sprite sizing described there.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — DATA-DRIVEN PATTERN ENGINE ────────────────────────── *
 *
 * Ten spark effects (WELDING, GRINDER, CAMPFIRE, TESLA, SPARKLERS,
 * SPINNER, FLOWERPOT, CHRYSANTHEMUM, WILLOW, WATERFALL) are
 * produced by a SINGLE generic ballistic-particle engine driven by
 * an array of PatternParams structs.  scene_tick and scene_draw
 * read pattern_params[s->current_pattern] and behave accordingly;
 * adding a new effect is mostly a matter of appending one row to
 * pattern_params[].
 *
 * THE TWO EXCEPTIONS — pattern-aware branches in the engine:
 *
 *   (a) SPINNER overrides the polar (angle, speed) cone with a
 *       TANGENTIAL emission off a rotating wheel rim.  The override
 *       lives in scene_spawn_spark; PatternParams provides the
 *       speed range, but the angle and spawn position come from
 *       wheel_compute_tangential_emission.
 *
 *   (b) SPARKLERS adds a MID-LIFE CASCADE pass: parents pop into
 *       smaller children (cascade_sparkler_split_pass) and the
 *       spawn loop counts PARENTS only (count_active_parent_sparks)
 *       so the cascade's children don't throttle parent emission.
 *
 * Every other variation between the ten effects comes from
 * PatternParams alone.
 *
 *
 * THE GENERIC ENGINE (pseudocode)
 * ───────────────────────────────
 *
 *   loop forever (each tick of dt seconds):
 *
 *     pp = pattern_params[scene.current_pattern]   # read inputs
 *
 *     # 0. WHEEL PHASE — tick the SPINNER axle (always, no-op for others)
 *     scene.spinner_phase += SPINNER_OMEGA · dt    # wrap to [0, 2π)
 *
 *     # 1. SPAWN — top up parent pool toward target density.
 *     #    SPARKLERS counts has_split=false sparks only; others count all.
 *     active   = (pp == SPARKLERS) ? count_parents() : count_all()
 *     to_spawn = compute_spawn_count_for_tick(active, pp, dt)
 *     repeat to_spawn times:
 *         e = next_inactive_slot()
 *         if pp == SPINNER:
 *             # tangent off rotating rim — exception (a)
 *             (e.x, e.y, angle) = wheel_compute_tangential_emission(
 *                                     scene.spinner_phase, cx, cy)
 *         else:
 *             # standard cone — angle in [pp.angle_min, pp.angle_max]
 *             (e.x, e.y, angle) = cone_compute_emission(rng, pp, cx, cy)
 *         speed   = uniform(pp.speed_min, pp.speed_max)
 *         e.life  = uniform(pp.life_min,  pp.life_max)
 *         e.vx, e.vy = velocity_from_polar(speed, angle)
 *         e.has_split = false                       # parent
 *         e.active    = true
 *
 *     # 2. INTEGRATE every active spark — Bourg semi-implicit Euler
 *     drag = exp(-pp.drag_coeff · dt)               # frame-rate independent
 *     for e in active:
 *         spark_shift_trail_history(e)              # ring shift for blur
 *         e.vy  += pp.gravity · dt                  # gravity into vy first
 *         e.vx  *= drag
 *         e.vy  *= drag
 *         e.x   += e.vx · dt;   e.y += e.vy · dt
 *         e.age += dt
 *
 *     # 3. FLOOR BOUNCE — Hertz reflection + settle test
 *     for e in active where crossed_floor_descending(e):
 *         e.y  = reflect_about_floor(e.y, floor)
 *         e.vy = -e.vy · pp.restitution             # Hertz e
 *         e.vx *=         pp.floor_friction         # tangential loss
 *         if |vx|, |vy| both below SETTLE: kill (no more visible bounces)
 *
 *     # 4. KILL — age >= life, off-screen-x, or above TOP_KILL_MARGIN
 *     for e in active:
 *         if spark_should_die(e, scene.cols):  e.active = false
 *
 *     # 5. SPARKLERS CASCADE — exception (b)
 *     if pp == SPARKLERS:
 *         for parents that have crossed pp.life · SPARKLER_SPLIT_AGE_FRAC:
 *             spawn SPARKLER_CHILDREN_PER_SPLIT children at (e.x, e.y)
 *             with reduced speed, reduced life, has_split=true
 *             e.has_split = true                    # latch (no recursion)
 *
 *     # 6. RENDER — trails first (dim, behind), heads on top (bright),
 *     #             SPINNER axle glyph at the wheel hub.
 *     spark_pool_draw_trails(scene)
 *     spark_pool_draw_heads(scene)
 *     if pp == SPINNER: wheel_axle_draw(scene)
 *
 *
 * PATTERNPARAMS FIELD → ENGINE HOOK
 * ─────────────────────────────────
 *
 *   target_sparks      →  spawn-loop refill cap        (parent density)
 *   emitter            →  scene_emitter_xy dispatch    (CENTER_LEFT etc.)
 *   emit_x_jitter      →  spawn-x jitter               (cone emission)
 *   emit_y_jitter      →  spawn-y jitter               (cone emission)
 *   speed_min          →  spawn speed range            (polar cone)
 *   speed_max
 *   angle_min          →  spawn angle range            (polar cone)
 *   angle_max                                          (ignored by SPINNER)
 *   gravity            →  vy update each tick          (Bourg Ch. 2)
 *   drag_coeff         →  drag = exp(-c·dt)            (Millington Ch. 6)
 *   restitution        →  vy reflection scale          (Hertz e)
 *   floor_friction     →  vx scale per bounce          (tangential loss)
 *   life_min           →  spark lifetime range         (heat-ramp clock)
 *   life_max
 *
 * Every other engine constant — MAX_SPARKS, TRAIL_LEN, SETTLE_VY,
 * SETTLE_VX, TOP_KILL_MARGIN, EMITTER_SHIFT_STEP, plus the
 * SPARKLER_* and SPINNER_* tuning knobs — is GLOBAL.  Patterns
 * differ ONLY in the fields above (plus belonging to one of the
 * two exception classes above for SPINNER / SPARKLERS).
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
 *     enumerates sparks specifically as a use case; this file
 *     applies the idea to ten different spark effects with two
 *     small principled exceptions where the data-only model
 *     wasn't expressive enough (SPINNER's tangential geometry,
 *     SPARKLERS' generational cascade).
 *
 *   Gamma, E., Helm, R., Johnson, R. & Vlissides, J. (1994)
 *     "Design Patterns" (Addison-Wesley) — STRATEGY pattern (§5.9).
 *     A family of algorithms (the ten spark effects) interchangeable
 *     behind a single interface (the engine reading PatternParams).
 *     In procedural C the "interface" is the struct shape;
 *     "concrete strategies" are the rows of pattern_params[];
 *     "selecting a strategy" is updating scene.current_pattern.
 *     The two pattern-aware branches in scene_tick / scene_spawn_spark
 *     are TEMPLATE METHOD hooks (§5.10) — one engine skeleton, two
 *     pattern-specific override points.
 *
 *   Acton, M. (2014)
 *     "Data-Oriented Design and C++" (CppCon 2014 keynote).
 *     Argues that variation between behaviours should be
 *     represented as DATA (struct fields) rather than as control
 *     flow (if/switch on type).  pattern_params[] is the data
 *     table; the two if-branches in scene_tick are the explicit
 *     compromises where the data-only design didn't fit.
 *
 *   Nystrom, R. (2014)
 *     "Game Programming Patterns" (Genever Benning).
 *     TYPE OBJECT chapter — PatternParams is a Type Object: one
 *     shared instance per spark effect.  DATA LOCALITY chapter —
 *     Spark is a flat-laid-out POD swept linearly by the integrator
 *     each tick.  OBJECT POOL chapter — fixed-size BSS array
 *     implements Nystrom's pool idiom directly (active flag +
 *     linear scan), with has_split as the cascade-generation tag.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A spark is a fast, short-lived projectile with memory. Each spark
 * remembers its last few positions; the screen shows that memory as
 * a fading streak behind a bright head. Gravity arcs the spark down,
 * the floor bounces it back up with an energy loss, and the heat
 * ramp colours it from white-hot at birth to dark and dead at the
 * end of its life.  Hundreds of these together look like a welder,
 * grinder, Tesla coil, sparkler, ground-spinner, or firework display
 * — same engine driving all ten patterns; the only physics that
 * varies between effects is one row of pattern_params[].
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * (Each step names the helper it dispatches to in §5; the drivers
 * scene_tick / scene_spawn_spark / scene_draw are pure pseudocode
 * over these.)
 *
 *  0. WHEEL PHASE ADVANCE.  wheel_phase_advance_and_wrap(s, dt) —
 *     SPINNER's axle rotates at ω = SPINNER_OMEGA rad/sec, wrapped
 *     to [0, 2π).  Always advances so the wheel keeps spinning
 *     across pattern swaps.
 *
 *  1. EMIT.
 *     active   = SPARKLERS ? count_active_parent_sparks  // cascade-aware
 *                          : count_active_sparks
 *     to_spawn = compute_spawn_count_for_tick(active, pp, dt)
 *     spark_pool_topup_parents(s, to_spawn)
 *
 *     scene_spawn_spark picks (origin, direction) via either:
 *       wheel_compute_tangential_emission   (SPINNER — tangent off
 *                                            the rotating rim)
 *       cone_compute_emission               (every other pattern —
 *                                            random angle in cone,
 *                                            position with jitter)
 *     ...then sample_speed_from_cone, sample_lifetime_from_pattern,
 *     spark_init_kinematics, spark_seed_trail_at_position.
 *
 *  2. INTEGRATE PER SPARK.
 *     spark_shift_trail_history(e)             // drop oldest, push curr.
 *     integrate_spark_semi_implicit_euler(e, g, drag, dt)
 *         → Bourg Ch. 2 semi-implicit Euler:
 *             vy += g·dt;  v *= exp(−drag·dt);  x += vx·dt;  y += vy·dt
 *
 *  3. BOUNCE.
 *     if spark_crossed_floor_descending(e, floor_y) &&
 *        reflect_spark_off_floor_and_test_settle(e, floor_y,
 *                                                restitution, friction):
 *         kill (settled too gently to bounce visibly)
 *     reflect = y reflected, vy *= -e (Hertz), vx *= friction.
 *
 *  4. KILL.  spark_should_die catches lifetime expiry + off-screen
 *     culls (x bounds, y above TOP_KILL_MARGIN).
 *
 *  5. SPARKLERS CASCADE.  cascade_sparkler_split_pass — parents that
 *     have crossed SPARKLER_SPLIT_AGE_FRAC · life pop into
 *     SPARKLER_CHILDREN_PER_SPLIT children at their current (x, y).
 *     has_split latches so each parent only pops once.
 *
 *  6. RENDER.
 *     spark_pool_draw_trails(s, rows_eff)    // dim, behind
 *     spark_pool_draw_heads(s, rows_eff)     // bright, on top
 *     if SPINNER: wheel_axle_draw            // | / - \ at hub
 *
 *     Per-head heat slot:
 *       T          = max(0, 1 − age / life)
 *       head_slot  = ⌊T · 7.999⌋
 *       head_attr  = A_BOLD (slot ≥ 6) | A_DIM (slot ≤ 1) | A_NORMAL
 *
 *  7. HUD.  Two-row layout painted over the scene by §6 screen —
 *     top row (status) PAIR_HUD + A_BOLD, bottom row (key hints)
 *     PAIR_HINT + A_BOLD.
 *
 * KEY FORMULAS
 * ────────────
 *  Initial velocity (cone emission):
 *    angle = angle_min + r · (angle_max − angle_min)
 *    speed = speed_min + r · (speed_max − speed_min)
 *    vx    = speed · cos(angle)
 *    vy    = speed · sin(angle)        // y increases downward
 *
 *  Gravity update (positive = downward):
 *    vy += gravity · dt
 *
 *  Continuous drag (frame-rate-independent damping):
 *    drag_factor = exp(−drag_coeff · dt)
 *    v          *= drag_factor
 *
 *  Floor bounce (about the line y = floor_y):
 *    y'  = 2·floor_y − y                // reflect
 *    vy' = −vy · restitution
 *    vx' = vx · floor_friction
 *
 *  Heat-ramp slot from remaining life:
 *    T          = max(0, 1 − age / life)
 *    head_slot  = ⌊T · 7.999⌋           // 0..7
 *    trail_slot = max(0, head_slot − 1 − k)  // dimmer for older trail points
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
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,

  MAX_SPARKS = 800,
  TRAIL_LEN = 3, /* motion-blur history slots          */

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_HEAT_BASE = 3, /* +0..+7 = 8 heat ramp slots         */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Emitter horizontal-shift step (cells) when the user presses w/W. */
#define EMITTER_SHIFT_STEP 8.0f

/* Bounce settle thresholds (cells/sec). Below these magnitudes after
 * a bounce, the spark is killed — it has too little energy to make a
 * visible bounce and would otherwise keep micro-bouncing forever.
 * Tuned by visual inspection — high enough to cull sliding sparks
 * within ~1 second, low enough that real bounces still register. */
#define SETTLE_VY 6.0f
#define SETTLE_VX 10.0f

/* Off-screen kill margin. A spark whose y goes more than this many
 * cells above the screen never comes back (drag dominates), so kill
 * early to free the pool slot. */
#define TOP_KILL_MARGIN 4.0f

/* SPARKLERS multi-level cascade — each parent spark pops at
 * SPLIT_AGE_FRAC of its lifetime into CHILDREN_PER_SPLIT smaller
 * secondaries.  The children fly off omnidirectionally from the
 * parent's position with reduced speed and shorter life, mimicking
 * the crackle of a handheld sparkler (parent stick → secondary
 * popping sparks). */
#define SPARKLER_SPLIT_AGE_FRAC 0.40f
#define SPARKLER_CHILDREN_PER_SPLIT 3
#define SPARKLER_CHILD_SPEED_MIN 14.0f
#define SPARKLER_CHILD_SPEED_MAX 26.0f
#define SPARKLER_CHILD_LIFE_MIN 0.3f
#define SPARKLER_CHILD_LIFE_MAX 0.7f

/* SPINNER rotating-wheel parameters — emission points sit on a small
 * circle of radius SPINNER_RADIUS rotating at SPINNER_OMEGA rad/sec.
 * Each spawn picks one of two opposing rim points (50/50); the
 * emission angle is TANGENT to the circle (perpendicular to radial),
 * so sparks fly OFF the rim like a real pyrotechnic ground spinner. */
#define SPINNER_OMEGA 14.0f          /* rad/sec ≈ 2.2 rotations/sec */
#define SPINNER_RADIUS 5.0f          /* cells from centre to rim    */
#define SPINNER_TANGENT_JITTER 0.25f /* ± rad around the tangent    */

/* Pattern enum. */
typedef enum {
  PATTERN_WELDING = 0,
  PATTERN_GRINDER = 1,
  PATTERN_CAMPFIRE = 2,
  PATTERN_TESLA = 3,
  PATTERN_SPARKLERS = 4,
  PATTERN_GROUND_SPINNER = 5,
  PATTERN_FLOWER_POT = 6,
  PATTERN_CHRYSANTHEMUM = 7,
  PATTERN_WILLOW = 8,
  PATTERN_WATERFALL = 9,
  N_PATTERNS = 10,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_WELDING:
    return "WELDING  ";
  case PATTERN_GRINDER:
    return "GRINDER  ";
  case PATTERN_CAMPFIRE:
    return "CAMPFIRE ";
  case PATTERN_TESLA:
    return "TESLA    ";
  case PATTERN_SPARKLERS:
    return "SPARKLERS";
  case PATTERN_GROUND_SPINNER:
    return "SPINNER  ";
  case PATTERN_FLOWER_POT:
    return "FLOWERPOT";
  case PATTERN_CHRYSANTHEMUM:
    return "CHRYSANTH";
  case PATTERN_WILLOW:
    return "WILLOW   ";
  case PATTERN_WATERFALL:
    return "WATERFALL";
  default:
    return "?        ";
  }
}

/* Where on the screen the emitter is anchored.  Four positions cover
 * the ten patterns — physics-friendly placement for each effect. */
typedef enum {
  EMIT_CENTER_LEFT,   /* x = 6,        y = rows/2  (WELDING)   */
  EMIT_CENTER_BOTTOM, /* x = cols/2,   y = rows-4  (GRINDER, CAMPFIRE,
                         FLOWERPOT, WILLOW)   */
  EMIT_CENTER,        /* x = cols/2,   y = rows/2  (TESLA, SPARKLERS, SPINNER,
                         CHRYSANTH)   */
  EMIT_CENTER_TOP,    /* x = cols/2,   y = 3       (WATERFALL)    */
} EmitPos;

/*
 * PatternParams — per-pattern physics + emission cone + lifetime.
 *
 * REFERENCES (each cited at the field it backs):
 *   Reeves, W. T. (1983) "Particle Systems — A Technique for
 *     Modeling a Class of Fuzzy Objects", ACM TOG 2(2): 91-108.
 *     §4 argues that natural-phenomena particle pools should be
 *     parameterised by a small struct of physical constants.
 *     PatternParams IS that struct — one row of pattern_params[]
 *     per effect.
 *
 *   Hertz, H. (1882) "Über die Berührung fester elastischer Körper".
 *     Coefficient of restitution e ∈ (0, 1) — the `restitution`
 *     field is literally Hertz's e.
 *
 *   Bourg, D. M. & Bywalec, B. (2013) "Physics for Game Developers".
 *     Ch. 2-3 — gravity + drag conventions.  `gravity` is a
 *     positive scalar applied to vy each tick; `drag_coeff` is a
 *     continuous (frame-rate-independent) coefficient applied as
 *     exp(−drag_coeff · dt) — see Millington (2010) Ch. 6 for the
 *     same form in engine code.
 *
 * INTENT
 *   Same simulation engine, ten different effects.  scene_tick never
 *   branches on the pattern enum for the physics — it just reads
 *   these fields and behaves accordingly.  The only pattern-aware
 *   branches in the engine are:
 *     (a) SPINNER's tangential emission override in scene_spawn_spark
 *     (b) SPARKLERS' mid-life cascade in scene_tick
 *     (c) SPARKLERS' parent-only count for spawn budgeting
 *   Every other variation between effects comes from THIS struct.
 *
 * FIELDS:
 *   target_sparks  Steady-state active "parent" count the spawn loop
 *                  refills toward each tick (per-tick cap proportional
 *                  to dt so a long pause doesn't dump everyone at
 *                  once).  Higher = denser visual.  Range across
 *                  patterns: 200 (SPARKLERS — leaves pool room for
 *                  cascade children) to 400 (FLOWERPOT, dense
 *                  fountain).  Reeves 1983 §3 — "steady-state pool".
 *
 *   emitter        EmitPos enum tag.  scene_emitter_xy dispatches
 *                  on this to compute (cx, cy).  Patterns choose
 *                  the position that makes their physics read
 *                  cleanest:
 *                    CENTER_LEFT     WELDING — room to fly right.
 *                    CENTER_BOTTOM   GRINDER / CAMPFIRE / FLOWERPOT /
 *                                    WILLOW — ground-rising effects.
 *                    CENTER          TESLA / SPARKLERS / SPINNER /
 *                                    CHRYSANTHEMUM — omni bursts use
 *                                    the whole canvas radially.
 *                    CENTER_TOP      WATERFALL — sparks rain the full
 *                                    height of the screen.
 *
 *   emit_x_jitter  ± cells of uniform spawn-position jitter around
 *   emit_y_jitter  the emitter (cx, cy).  Adds a small fuzz so the
 *                  emission point doesn't read as a single pinpoint.
 *                  CAMPFIRE uses 3.0/0.5 — a wide horizontal base
 *                  and thin vertically — for a flame-base feel.
 *                  Most patterns use ~0.5 (subtle).
 *
 *   speed_min      Polar (angle, speed) cone — speed range in
 *   speed_max      cells/sec, sampled uniformly at spawn.
 *                  vx = speed·cos(angle), vy = speed·sin(angle).
 *                  Higher = sparks travel farther before drag +
 *                  gravity catch them.  Range: 28-50 (SPARKLERS,
 *                  gentle fizz) to 100-150 (FLOWERPOT, tall fountain).
 *
 *   angle_min      Cone angle range in radians.  Conventions:
 *   angle_max        0 = +x (right), −π/2 = −y (up),
 *                    +π/2 = +y (down), ±π = left.
 *                  Span ≥ 2π gives full omnidirectional emission
 *                  (TESLA / SPARKLERS / SPINNER / CHRYSANTHEMUM).
 *                  Narrow ranges shape the cone:
 *                    WELDING ±0.55 around 0   (horizontal-right)
 *                    GRINDER  (−1.3, −0.4)    (up-and-right)
 *                    FLOWERPOT ±0.6 around −π/2 (vertical-up)
 *                    WILLOW  ±1.2 around −π/2  (wide upward fan)
 *                    WATERFALL ±0.35 around +π/2 (vertical-down).
 *                  SPINNER's pattern angle is ignored — emission
 *                  angle is computed tangentially from spinner_phase.
 *
 *   gravity        Downward acceleration in cells/sec² (always
 *                  positive — Bourg Ch. 2 convention).  Applied to
 *                  vy each tick BEFORE drag (semi-implicit Euler:
 *                  velocity update, then position update).  Range:
 *                  18 (CHRYSANTHEMUM — sparks hang in air) to 100
 *                  (WILLOW — heavy drooping branches).
 *
 *   drag_coeff     Continuous-damping coefficient (1/sec) used as
 *                  drag_factor = exp(−drag_coeff · dt).  Frame-rate
 *                  independent — commutes correctly across substeps,
 *                  unlike v *= 0.99 which is dt-dependent and breaks
 *                  if the tick rate changes.  Range: 0.15 (WILLOW,
 *                  NOVA — sparks coast a long way) to 0.55
 *                  (CAMPFIRE — sparks settle quickly).
 *                  Millington 2010 Ch. 6 for the exp-form rationale.
 *
 *   restitution    Floor-bounce energy retention, Hertz e ∈ [0, 1].
 *                  vy' = −restitution · vy after a downward floor
 *                  contact.  e = 0 → spark sticks dead on first hit;
 *                  e = 1 → perfectly elastic, bounces forever.
 *                  Range: 0.30 (WILLOW — thuds) to 0.65 (TESLA,
 *                  WATERFALL — bouncy).
 *
 *   floor_friction Tangential vx multiplier per bounce.  Modelled
 *                  as a per-event scalar rather than continuous
 *                  surface friction (simpler, no need to track time
 *                  spent in contact).  Range: 0.60-0.78.
 *
 *   life_min       Lifetime range in seconds; sampled uniformly at
 *   life_max       spawn.  Drives the heat-ramp fade — the head
 *                  colour reads remaining-life fraction T =
 *                  1 − age/life (Akenine-Möller 2018 §13.7 —
 *                  point-sprite life→intensity analogue).  Range:
 *                  0.4-1.0 (SPARKLERS — crackly short bursts) to
 *                  3.0-4.5 (CHRYSANTHEMUM — slow majestic bloom).
 */
typedef struct {
  int target_sparks;
  EmitPos emitter;
  float emit_x_jitter;
  float emit_y_jitter;
  float speed_min, speed_max;
  float angle_min, angle_max;
  float gravity;
  float drag_coeff;
  float restitution;
  float floor_friction;
  float life_min, life_max;
} PatternParams;

/* Angle conventions for the cones below.
 *   +0.0           = pointing right (+x)
 *   -M_PI/2  ≈ -1.57 = pointing straight up (-y)
 *   +M_PI/2  ≈ +1.57 = pointing straight down (+y)
 *   ±M_PI    ≈ ±3.14 = pointing left
 *
 * To get an "up-and-right" cone you want angles in (-π/2, 0). To get
 * "everywhere" you want a 2π-wide range. */
static const PatternParams pattern_params[N_PATTERNS] = {
    /* Field order:
     *  target  emitter             emit_jx emit_jy  spd_min spd_max  ang_min
     * ang_max               grav   drag   rest  fric   lmin  lmax
     */
    /* WELDING       */ {380, EMIT_CENTER_LEFT, 1.0f, 1.0f, 55.0f, 90.0f,
                         -0.55f, 0.55f, 78.0f, 0.40f, 0.55f, 0.78f, 1.0f, 2.2f},
    /* GRINDER       */
    {320, EMIT_CENTER_BOTTOM, 0.6f, 0.6f, 72.0f, 110.0f, -1.30f, -0.40f, 92.0f,
     0.30f, 0.50f, 0.70f, 0.8f, 1.8f},
    /* CAMPFIRE      */
    {220, EMIT_CENTER_BOTTOM, 3.0f, 0.5f, 34.0f, 56.0f, -2.20f, -0.94f, 36.0f,
     0.55f, 0.40f, 0.60f, 1.5f, 3.0f},
    /* TESLA         */
    {260, EMIT_CENTER, 0.4f, 0.4f, 55.0f, 85.0f, -(float)M_PI, (float)M_PI,
     26.0f, 0.20f, 0.65f, 0.78f, 0.5f, 1.2f},
    /* SPARKLERS     */
    {200, EMIT_CENTER, 0.3f, 0.3f, 28.0f, 50.0f, -(float)M_PI, (float)M_PI,
     60.0f, 0.50f, 0.35f, 0.65f, 0.4f, 1.0f},
    /* SPINNER       */
    {380, EMIT_CENTER, 0.5f, 0.5f, 90.0f, 130.0f, -(float)M_PI, (float)M_PI,
     50.0f, 0.30f, 0.55f, 0.75f, 1.0f, 2.0f},
    /* FLOWERPOT     */
    {400, EMIT_CENTER_BOTTOM, 0.5f, 0.5f, 100.0f, 150.0f,
     -(float)(M_PI / 2 + 0.6), -(float)(M_PI / 2 - 0.6), 70.0f, 0.25f, 0.45f,
     0.70f, 2.0f, 3.5f},
    /* CHRYSANTHEMUM */
    {320, EMIT_CENTER, 0.5f, 0.5f, 50.0f, 75.0f, -(float)M_PI, (float)M_PI,
     18.0f, 0.18f, 0.40f, 0.65f, 3.0f, 4.5f},
    /* WILLOW        */
    {280, EMIT_CENTER_BOTTOM, 0.5f, 0.5f, 50.0f, 80.0f,
     -(float)(M_PI / 2 + 1.2), -(float)(M_PI / 2 - 1.2), 100.0f, 0.15f, 0.30f,
     0.60f, 2.8f, 4.5f},
    /* WATERFALL     */
    {380, EMIT_CENTER_TOP, 0.5f, 0.5f, 35.0f, 65.0f, (float)(M_PI / 2 - 0.35),
     (float)(M_PI / 2 + 0.35), 45.0f, 0.25f, 0.65f, 0.78f, 1.8f, 3.0f},
};

/*
 * Themes — each is an 8-step HEAT RAMP from cool/dim (slot 0) to
 * hot/bright (slot 7).  Each theme commits to ONE hue family so the
 * `t` / `T` cycle is visually obvious — no two themes share a colour
 * story.  The standout is NOVA, the only multi-hue ramp (blue →
 * magenta → orange → yellow → white) so a real supernova spectrum
 * passes through the spark trail.
 *
 *   MATRIX   pure phosphor green        FOREST   olive → leaf → gold
 *   FIRE     coal → flame → white-hot   DESERT   dune brown → cream
 *   OCEANIC  deep blue → teal (no white) NEON    purple → hot pink
 *   ICE      pale frost → pure white    ECLIPSE  void purple → crimson
 *   NOVA     blue → magenta → yellow    MONO     grayscale
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule, so even ramp[0]
 * (the dimmest "tail" colour) stays visible against a black terminal
 * with A_DIM applied.
 */
typedef struct {
  const char *name;
  short heat[8]; /* cool → hot */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        heat[0..7]  (cool dim → hot bright)                  */
    {"MATRIX", {28, 34, 40, 46, 82, 118, 154, 190}}, /* phosphor green        */
    {"FIRE", {52, 88, 124, 160, 202, 208, 220, 231}}, /* coal→flame→white-hot */
    {"OCEANIC", {24, 25, 32, 38, 44, 80, 116, 152}}, /* deep blue→teal        */
    {"NEON", {54, 92, 128, 165, 201, 207, 213, 219}},   /* purple→hot pink   */
    {"MONO", {240, 244, 247, 250, 252, 253, 254, 255}}, /* grayscale */
    {"ICE", {117, 153, 159, 195, 231, 251, 253, 255}},  /* pale frost→white  */
    {"NOVA", {27, 99, 165, 201, 208, 220, 226, 231}}, /* SPECTRUM (multi-hue) */
    {"FOREST", {58, 64, 70, 106, 142, 148, 184, 220}}, /* olive→leaf→gold */
    {"DESERT", {94, 130, 137, 173, 215, 222, 228, 230}}, /* dune→sand→cream */
    {"ECLIPSE",
     {53, 54, 89, 125, 161, 197, 204, 209}}, /* void purple→crimson   */
};

/*
 * Glyph ramps for the head and trail. The head uses dense punctuation
 * (`*` `+` `#`) to read as a hot point. The trail uses sparse glyphs
 * (`,` `.` `:`) to read as a fading streak BEHIND the head. Both are
 * indexed cool (slot 0, dim) → hot (slot 7, bright).
 */
static const char HEAD_GLYPHS[8] = {'`', '.', ':', ';', '*', '+', '#', '@'};
static const char TRAIL_GLYPHS[8] = {'`', '.', '.', ',', ':', ';', '+', '*'};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_HEAT_BASE + i), t->heat[i], -1);
  } else {
    /* 8-colour fallback: roughly cool→hot using basic palette. */
    static const short fb[8] = {
        COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
    };
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_HEAT_BASE + i), fb[i], -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

/* ===================================================================== */
/* §4  spark                                                              */
/* ===================================================================== */

/*
 * Spark — one projectile with motion-blur history.
 *
 * REFERENCES:
 *   Reeves, W. T. (1983) "Particle Systems — A Technique for
 *     Modeling a Class of Fuzzy Objects", ACM TOG 2(2): 91-108.
 *     The fixed pool + per-particle state + lifetime-via-age-and-
 *     off-screen-cull model is straight from Reeves §3.  §4 names
 *     sparks specifically as a natural-phenomena instance of his
 *     framework.
 *
 *   Hertz, H. (1882) "Über die Berührung fester elastischer Körper",
 *     J. reine angew. Math. 92: 156-171.
 *     Coefficient of restitution e ∈ (0, 1) governs the floor
 *     bounce — the rule vy' = −e · vy mutates this struct's vy.
 *
 *   Bourg, D. M. & Bywalec, B. (2013) "Physics for Game Developers",
 *     2nd ed (O'Reilly).  Ch. 2-3 — ballistic motion under gravity
 *     + drag.  The semi-implicit Euler step that mutates this
 *     struct each tick:
 *           vy += gravity · dt
 *           v  *= exp(−drag · dt)
 *           x  += vx · dt;   y += vy · dt
 *
 *   Akenine-Möller, T., Haines, E. & Hoffman, N. (2018) "Real-Time
 *     Rendering", 4th ed (CRC Press).  §13.7 — point-sprite particle
 *     rendering.  The remaining-life → heat-ramp-slot mapping in
 *     spark_head_slot() is the ASCII analogue of size-by-mass
 *     point-sprite sizing described there.
 *
 * INTENT
 *   Hold every per-particle value the integrator needs in a flat
 *   layout that fits into a fixed-size BSS pool — MAX_SPARKS of
 *   these are allocated once at program start, no malloc after init.
 *   The `active` flag is the single source of truth for pool-slot
 *   occupancy; spawn finds the first inactive index via a linear
 *   scan (small N, fine).  Per-spark layout is hot-path-friendly:
 *   each tick reads x/y/vx/vy/age/trail consecutively per spark, so
 *   keeping these in a flat struct (not parallel arrays) wins on
 *   cache locality.
 *
 * LIFECYCLE
 *   SPAWN (scene_spawn_spark / scene_spawn_child_spark):
 *     position from emitter + jitter, velocity from a polar
 *     (angle, speed) cone, trail filled with the spawn position,
 *     active=true.  has_split = false for parents, true for cascade
 *     children.
 *   INTEGRATE (scene_tick):
 *     trail shifted by one slot, then semi-implicit Euler advances
 *     vy by gravity·dt, applies exp-drag to v, advances (x, y),
 *     and ages the spark.
 *   BOUNCE (scene_tick):
 *     when y crosses the floor moving down, reflect y, scale vy by
 *     −restitution (Hertz e), scale vx by floor_friction.  Settle
 *     criterion kills sparks with too little energy to bounce
 *     visibly so they don't burn ticks micro-bouncing.
 *   CASCADE (scene_tick, SPARKLERS only):
 *     when age crosses SPARKLER_SPLIT_AGE_FRAC · life, parent pops
 *     into SPARKLER_CHILDREN_PER_SPLIT children at its current
 *     (x, y); has_split latches to true.
 *   KILL: age ≥ life, x off-screen, y above TOP_KILL_MARGIN, or
 *     settled at floor.  Slot is reused by the next spawn.
 *
 * FIELDS:
 *   x, y          Position in cells.  +y is downward (ncurses
 *                 convention), so gravity adds POSITIVE vy.  Sub-cell
 *                 precision is preserved as float; rendering rounds
 *                 to the nearest integer cell.
 *
 *   vx, vy        Velocity in cells/sec.  Composed at spawn from
 *                 polar (angle, speed) via cos/sin, then mutated
 *                 each tick by gravity + drag + bounce (Bourg
 *                 Ch. 2-3).
 *
 *   age, life     Seconds.  `age` advances at dt each tick (scaled
 *                 by user speed multiplier); `life` is sampled once
 *                 at spawn from [pp->life_min, pp->life_max] and
 *                 never changes.  Death at age ≥ life (Reeves 1983
 *                 §3 — lifetime cull) and head colour fades along
 *                 the heat ramp as T = 1 − age/life shrinks
 *                 (Akenine-Möller §13.7).
 *
 *   trail_x[]     Ring of TRAIL_LEN previous positions, oldest at
 *   trail_y[]     index 0, newest (most recent prev) at index
 *                 TRAIL_LEN-1.  Filled with the spawn (x, y) at
 *                 birth so the renderer never sees uninitialised
 *                 history — otherwise fresh sparks would draw a
 *                 streak from a random previous occupant's last
 *                 position.  Shifted by one slot per tick; scene_draw
 *                 renders the ring as a fading streak BEHIND the
 *                 head.
 *
 *   active        Pool-slot occupancy flag.  Inactive slots are
 *                 skipped by every loop; spawn linear-scans for the
 *                 first inactive index (O(MAX_SPARKS), free at 60 fps
 *                 with MAX_SPARKS=800).  Reeves 1983 §3 — pool slot.
 *
 *   has_split     SPARKLERS multi-level cascade flag, two roles:
 *                   (a) gate for the cascade — true means this spark
 *                       has already popped into children (or is
 *                       itself a child), and won't pop again.  Caps
 *                       the cascade at 2 generations.
 *                   (b) generation tag for spawn budgeting — the
 *                       SPARKLERS parent-target refill loop counts
 *                       only has_split=false sparks, so cascade
 *                       children flooding the pool don't throttle
 *                       new parent emission.
 *                 Unused (always false) for every non-SPARKLERS
 *                 pattern.
 */
typedef struct {
  float x, y;
  float vx, vy;
  float age, life;
  float trail_x[TRAIL_LEN];
  float trail_y[TRAIL_LEN];
  bool active;
  bool has_split;
} Spark;

/* Cheap LCG — same constants as embers.c so the visual "noise feel"
 * is consistent across the particle_systems/ family. */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ── Sampling primitives ───────────────────────────────────────────────── */

/* Uniform sample over [lo, hi) — base primitive for every other sampler. */
static inline float sample_uniform_in_range(uint32_t *rng, float lo, float hi) {
  return lo + lcg_unit(rng) * (hi - lo);
}

/* Random angle in [-π, π) — used for omnidirectional emission of
 * SPARKLERS cascade children where direction is fully random. */
static inline float sample_random_phase_2pi(uint32_t *rng) {
  return lcg_unit(rng) * 2.0f * (float)M_PI - (float)M_PI;
}

/* ── Polar emission + initial kinematics ─────────────────────────────── */

/* Polar → Cartesian: split a (speed, angle) cone vector into its (vx, vy)
 * components.  Angle convention matches the rest of the file:
 *   0 = +x (right), -π/2 = -y (up), +π/2 = +y (down).  */
static inline void velocity_from_polar(float speed, float angle, float *vx,
                                       float *vy) {
  *vx = speed * cosf(angle);
  *vy = speed * sinf(angle);
}

/* Fill every trail slot with the spawn (x, y) so a fresh spark draws
 * its trail on TOP of its head — invisible until the spark moves and
 * the ring naturally stretches.  Without this, the renderer would
 * draw a streak from the previous occupant's last position. */
static inline void spark_seed_trail_at_position(Spark *e, float x, float y) {
  for (int k = 0; k < TRAIL_LEN; k++) {
    e->trail_x[k] = x;
    e->trail_y[k] = y;
  }
}

/* Initialise the kinematic state of a spark from polar (speed, angle).
 * Pure write — does not touch active or has_split (caller's responsibility
 * since those depend on whether this is a parent or cascade child). */
static inline void spark_init_kinematics(Spark *e, float x, float y,
                                         float speed, float angle, float life) {
  e->x = x;
  e->y = y;
  velocity_from_polar(speed, angle, &e->vx, &e->vy);
  e->age = 0.0f;
  e->life = life;
}

/* ── Per-tick physics ─────────────────────────────────────────────────── */

/* Continuous-time damping factor applied per tick: drag = exp(−c·dt).
 * Frame-rate independent — commutes correctly across substeps, unlike
 * the multiplicative form v *= 0.99 which depends on tick rate.
 * Millington (2010) Ch. 6 — "exponential drag for stable physics". */
static inline float compute_drag_factor_continuous(float drag_coeff, float dt) {
  return expf(-drag_coeff * dt);
}

/* Drop oldest trail point, push current (x, y) as the newest prev.
 * Called BEFORE integration so the newest trail slot stores where the
 * spark IS RIGHT NOW (the visual head→trail link). */
static inline void spark_shift_trail_history(Spark *e) {
  for (int k = 0; k < TRAIL_LEN - 1; k++) {
    e->trail_x[k] = e->trail_x[k + 1];
    e->trail_y[k] = e->trail_y[k + 1];
  }
  e->trail_x[TRAIL_LEN - 1] = e->x;
  e->trail_y[TRAIL_LEN - 1] = e->y;
}

/* Semi-implicit Euler step (Bourg 2013 Ch. 2 / Millington Ch. 6):
 *   1. Velocity update: gravity into vy first, then drag scales v.
 *   2. Position update with the NEW velocity (this is the "semi-
 *      implicit" part — explicit Euler would use the OLD velocity).
 *   3. Age advances by dt. */
static inline void integrate_spark_semi_implicit_euler(Spark *e, float gravity,
                                                       float drag_factor,
                                                       float dt) {
  e->vy += gravity * dt;
  e->vx *= drag_factor;
  e->vy *= drag_factor;
  e->x += e->vx * dt;
  e->y += e->vy * dt;
  e->age += dt;
}

/* True when the spark has just crossed the floor moving downward —
 * the trigger condition for a Hertz reflection. */
static inline bool spark_crossed_floor_descending(const Spark *e,
                                                  float floor_y) {
  return e->y >= floor_y && e->vy > 0.0f;
}

/* Hertz floor bounce + settle test.  Reflects y about the floor line,
 * scales vy by −restitution (Hertz e ∈ [0, 1]) and vx by floor_friction.
 * Returns true if the post-bounce energy is too small to make a visible
 * bounce — caller should then deactivate the spark to avoid burning
 * ticks micro-bouncing it forever. */
static inline bool reflect_spark_off_floor_and_test_settle(Spark *e,
                                                           float floor_y,
                                                           float restitution,
                                                           float friction) {
  float overshoot = e->y - floor_y;
  e->y = floor_y - overshoot; /* reflect about floor */
  if (e->y > floor_y)
    e->y = floor_y;             /* numeric safety      */
  e->vy = -e->vy * restitution; /* Hertz energy loss   */
  e->vx *= friction;            /* tangential friction */
  return fabsf(e->vy) < SETTLE_VY && fabsf(e->vx) < SETTLE_VX;
}

/* Death conditions: lifetime expired (Reeves 1983 §3 — age cull),
 * x off-screen (sideways drift cull), or y above the kill margin
 * (upward escape cull, e.g. a TESLA spark fired hard upward). */
static inline bool spark_should_die(const Spark *e, int cols) {
  if (e->age >= e->life)
    return true;
  if (e->x < -2.0f || e->x > (float)(cols + 2))
    return true;
  if (e->y < -TOP_KILL_MARGIN)
    return true;
  return false;
}

/* ===================================================================== */
/* §5  scene — pool, tick, draw                                          */
/* ===================================================================== */

/*
 * Scene — owns every piece of mutable state for the sparks demo.
 *
 * Two clearly-separated halves:
 *
 *   SIMULATION HALF — read + written by scene_tick.  Owns the active
 *                     pattern, force overrides, RNG, cached terminal
 *                     dimensions, spinner-wheel phase, and the spark
 *                     pool itself.  Mutated by scene_tick and the
 *                     key handler.
 *
 *   RENDER HALF     — what scene_draw consults to pick colours.
 *                     Purely visual selection index; never read
 *                     inside the physics tick.
 *
 * LOCALITY
 *   Grouping by access pattern matters for two reasons.  Conceptually:
 *   a reader scanning scene_tick wants every field that mutates each
 *   frame in one block (not interleaved with render-only fields), and
 *   vice versa for scene_draw.  Mechanically: the hot path is the
 *   tick — its 9 small simulation fields fit in one or two cache
 *   lines and stay warm across the whole tick.  current_theme sits
 *   in the render half because the physics tick never touches it.
 *
 * REFERENCES (cited inline at the relevant field):
 *   Reeves (1983)        — particle pool design
 *   Hertz (1882)         — coefficient of restitution
 *   Bourg/Bywalec (2013) — gravity + drag + rotating reference frame
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
   * running, so the user sees a frozen frame with sparks held
   * mid-arc and trails frozen behind them.  Toggled by SPACE. */
  bool paused;

  /* SPEED — integer multiplier on dt.  SPEED_DEF = 1× wall clock;
   * +/= keys double, − halves; bounded by SPEED_MIN/MAX.  Doesn't
   * change physics constants — just compresses or stretches
   * simulated time so the user can slow-mo / fast-forward the
   * same physics. */
  int speed;

  /* PATTERN — index into pattern_params[] (WELDING … WATERFALL).
   * Cycled by n / N.  Switching pattern doesn't rebuild the pool —
   * existing sparks finish their lifetimes under the old physics
   * and fade away while new ones spawn under the new pattern.
   * Press r to clear instantly.  Read by scene_tick (spawn loop +
   * cascade gate) AND scene_draw (SPINNER core glyph); the only
   * field used by both halves. */
  Pattern current_pattern;

  /* EMITTER OFFSET — added to the emitter's cx so the user can
   * drag the show sideways with w/W without changing the
   * pattern's identity.  Same physics, displaced origin.  Persists
   * across pattern switches; reset to 0 by r. */
  float emitter_offset_x;

  /* SPINNER WHEEL PHASE — angle in radians, advanced by
   * SPINNER_OMEGA · dt each tick and wrapped to [0, 2π).  Drives:
   *   – two opposing rotating emission points on a circle of
   *     radius SPINNER_RADIUS around the emitter (scene_spawn_spark)
   *   – the rotating axle glyph at the wheel hub (scene_draw)
   * Bourg & Bywalec Ch. 8 §1 — the parametric circle
   *   x(t) = cx + R·cos(ωt),  y(t) = cy + R·sin(ωt).
   * Only meaningful when current_pattern == PATTERN_GROUND_SPINNER,
   * but advances unconditionally so the SPINNER axle picks up
   * smoothly where it left off after a pattern swap. */
  float spinner_phase;

  /* RNG — per-scene LCG state, seeded from clock_ns() at init and
   * re-seeded (XOR'd with a sentinel) on r.  Used by every
   * randomness consumer: spawn jitter (angle, speed, position,
   * life), SPARKLERS child spawn (direction + life), SPINNER rim
   * side + tangent jitter.  No globals — full state in this
   * single 32-bit word. */
  uint32_t rng;

  /* CACHED TERMINAL DIMENSIONS — read every frame by spawn
   * (emitter coords), the integrator (off-screen kill check), and
   * the renderer (cell bounds).  Cached at init and on SIGWINCH
   * so the hot path never calls getmaxyx().  Strictly speaking
   * these are SHARED between sim and render — both halves see the
   * same dimensions — but they're written only by the tick layer
   * so they live with the SIMULATION block. */
  int rows, cols;

  /* SPARK POOL — fixed-size BSS array, no allocation after init.
   * See Spark for per-slot detail.  spark_pool_find_inactive
   * linear-scans for the first inactive index when spawn needs a
   * slot.  Reeves 1983 §3 — the particle pool. */
  Spark sparks[MAX_SPARKS];

  /* ──────────────────────────────────────────────────────────────
   *  RENDER HALF — scene_draw reads this; physics tick ignores it
   * ────────────────────────────────────────────────────────────── */

  /* THEME — index into themes[].  Cycled by t / T.  Selects the
   * 8-step heat ramp (remaining-life → colour) used for both
   * heads and trails.  Pure render concern — spark physics
   * behaves identically regardless of which theme is active.
   * theme_apply rewrites pairs PAIR_HEAT_BASE..+7 on change. */
  int current_theme;
} Scene;

/* The floor — sparks bounce off the row above the HUD. The HUD lives
 * on row (rows - 1); sparks reflect about y = rows - 2. */
static inline float scene_floor_y(const Scene *s) {
  return (float)(s->rows - 2);
}

static int spark_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_SPARKS; i++)
    if (!s->sparks[i].active)
      return i;
  return -1;
}

static void scene_clear_sparks(Scene *s) {
  for (int i = 0; i < MAX_SPARKS; i++)
    s->sparks[i].active = false;
}

/*
 * scene_emitter_xy — compute the emitter's (cx, cy) for the current
 * pattern.  Four anchor positions cover the ten patterns:
 *
 *   CENTER_LEFT    horizontal-jet origin for WELDING — sparks fly
 *                  rightward across the whole screen.
 *   CENTER_BOTTOM  ground origin for GRINDER / CAMPFIRE / FLOWERPOT /
 *                  WILLOW — sparks rise and arc back to the floor.
 *   CENTER         radial origin for omnidirectional bursts —
 *                  TESLA, SPARKLERS, SPINNER, CHRYSANTHEMUM.
 *   CENTER_TOP     ceiling origin for WATERFALL — sparks rain down
 *                  the full height of the screen.
 *
 * The w/W keys shift `emitter_offset_x` so the user can drag any
 * pattern sideways without rebuilding the table.
 */
static void scene_emitter_xy(const Scene *s, float *cx, float *cy) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  switch (pp->emitter) {
  case EMIT_CENTER_LEFT:
    *cx = 6.0f;
    *cy = (float)s->rows * 0.50f;
    break;
  case EMIT_CENTER_BOTTOM:
    *cx = (float)s->cols * 0.50f;
    *cy = (float)(s->rows - 4);
    break;
  case EMIT_CENTER_TOP:
    *cx = (float)s->cols * 0.50f;
    *cy = 3.0f;
    break;
  case EMIT_CENTER:
  default:
    *cx = (float)s->cols * 0.50f;
    *cy = (float)s->rows * 0.50f;
    break;
  }
  *cx += s->emitter_offset_x;
}

/* ── Emission cone primitives ─────────────────────────────────────────── */

/* Sample speed from pattern's (speed_min, speed_max) cone. */
static inline float sample_speed_from_cone(uint32_t *rng,
                                           const PatternParams *pp) {
  return sample_uniform_in_range(rng, pp->speed_min, pp->speed_max);
}

/* Sample lifetime from pattern's (life_min, life_max) range — drives
 * the heat-ramp fade (Akenine-Möller 2018 §13.7). */
static inline float sample_lifetime_from_pattern(uint32_t *rng,
                                                 const PatternParams *pp) {
  return sample_uniform_in_range(rng, pp->life_min, pp->life_max);
}

/* Parametric circle: point on the SPINNER wheel rim at angle `phase`,
 * radius SPINNER_RADIUS, around centre (cx, cy).  Bourg & Bywalec
 * Ch. 8 §1 — x(t) = cx + R·cos(ωt),  y(t) = cy + R·sin(ωt). */
static inline void wheel_rim_position(float cx, float cy, float phase,
                                      float *out_x, float *out_y) {
  *out_x = cx + SPINNER_RADIUS * cosf(phase);
  *out_y = cy + SPINNER_RADIUS * sinf(phase);
}

/* Tangent direction at a rim point — perpendicular to the radial
 * (spoke) at that angle.  Sparks fly OFF the wheel in this direction,
 * not radially outward, which is what makes the SPINNER read as a
 * rotating wheel rather than a centred burst. */
static inline float wheel_tangent_angle(float phase) {
  return phase + (float)M_PI / 2.0f;
}

/* SPINNER emission: pick one of two opposing rim points (50/50), spawn
 * a spark on that rim, and emit tangent to the circle with a small
 * angular jitter so the stream isn't perfectly straight. */
static void wheel_compute_tangential_emission(uint32_t *rng, float phase_now,
                                              float cx, float cy,
                                              float *spawn_x, float *spawn_y,
                                              float *emit_angle) {
  float side = (lcg_unit(rng) < 0.5f) ? 0.0f : (float)M_PI;
  float phase = phase_now + side;
  wheel_rim_position(cx, cy, phase, spawn_x, spawn_y);
  float jitter = (lcg_unit(rng) - 0.5f) * 2.0f * SPINNER_TANGENT_JITTER;
  *emit_angle = wheel_tangent_angle(phase) + jitter;
}

/* Standard cone emission: random angle from the pattern's range,
 * spawn position around the emitter centre with rectangular jitter.
 * Used by every pattern except SPINNER. */
static void cone_compute_emission(uint32_t *rng, const PatternParams *pp,
                                  float cx, float cy, float *spawn_x,
                                  float *spawn_y, float *emit_angle) {
  *emit_angle = sample_uniform_in_range(rng, pp->angle_min, pp->angle_max);
  *spawn_x = cx + (lcg_unit(rng) - 0.5f) * 2.0f * pp->emit_x_jitter;
  *spawn_y = cy + (lcg_unit(rng) - 0.5f) * 2.0f * pp->emit_y_jitter;
}

/*
 * scene_spawn_spark — emit one "parent" spark for the current pattern.
 *
 *   1. Pick origin + direction:
 *        SPINNER → tangent off the rotating rim
 *        else    → standard cone emission with positional jitter
 *   2. Sample speed + life from pattern.
 *   3. Initialise kinematics and seed the trail at the spawn point.
 *   4. Mark active, has_split=false (parents are split-eligible).
 */
static void scene_spawn_spark(Scene *s) {
  int idx = spark_pool_find_inactive(s);
  if (idx < 0)
    return;
  Spark *e = &s->sparks[idx];

  const PatternParams *pp = &pattern_params[s->current_pattern];

  float cx, cy;
  scene_emitter_xy(s, &cx, &cy);

  float spawn_x, spawn_y, emit_angle;
  if (s->current_pattern == PATTERN_GROUND_SPINNER)
    wheel_compute_tangential_emission(&s->rng, s->spinner_phase, cx, cy,
                                      &spawn_x, &spawn_y, &emit_angle);
  else
    cone_compute_emission(&s->rng, pp, cx, cy, &spawn_x, &spawn_y, &emit_angle);

  float speed = sample_speed_from_cone(&s->rng, pp);
  float life = sample_lifetime_from_pattern(&s->rng, pp);

  spark_init_kinematics(e, spawn_x, spawn_y, speed, emit_angle, life);
  spark_seed_trail_at_position(e, spawn_x, spawn_y);
  e->has_split = false;
  e->active = true;
}

/*
 * scene_spawn_child_spark — secondary spark from a SPARKLERS cascade.
 * Reads as: random omni direction, child-scale speed and lifetime,
 * spawn at the parent's current position, mark already-split so the
 * cascade caps at 2 generations.
 */
static void scene_spawn_child_spark(Scene *s, float x, float y) {
  int idx = spark_pool_find_inactive(s);
  if (idx < 0)
    return; /* pool full — accept that some children fail */
  Spark *e = &s->sparks[idx];

  float angle = sample_random_phase_2pi(&s->rng);
  float speed = sample_uniform_in_range(&s->rng, SPARKLER_CHILD_SPEED_MIN,
                                        SPARKLER_CHILD_SPEED_MAX);
  float life = sample_uniform_in_range(&s->rng, SPARKLER_CHILD_LIFE_MIN,
                                       SPARKLER_CHILD_LIFE_MAX);

  spark_init_kinematics(e, x, y, speed, angle, life);
  spark_seed_trail_at_position(e, x, y);
  e->has_split = true; /* no recursive splitting */
  e->active = true;
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_WELDING;
  s->emitter_offset_x = 0.0f;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  scene_clear_sparks(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xDEADBEEFu;
  s->emitter_offset_x = 0.0f;
  s->spinner_phase = 0.0f;
  scene_clear_sparks(s);
}

/* ── Tick orchestration helpers ───────────────────────────────────────── */

/* Advance the SPINNER wheel rotation by ω·dt and wrap to [0, 2π) so
 * the accumulated phase never grows unboundedly.  Runs unconditionally
 * so the wheel picks up smoothly where it left off after a pattern
 * swap; harmless when the current pattern isn't SPINNER. */
static inline void wheel_phase_advance_and_wrap(Scene *s, float dt) {
  s->spinner_phase += SPINNER_OMEGA * dt;
  if (s->spinner_phase > 2.0f * (float)M_PI)
    s->spinner_phase -= 2.0f * (float)M_PI;
}

/* Total population census — linear scan of the pool's active flags. */
static int count_active_sparks(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_SPARKS; i++)
    if (s->sparks[i].active)
      n++;
  return n;
}

/* Population census restricted to PARENTS (has_split=false).  Used by
 * SPARKLERS so the cascade's children don't fill the budget and
 * throttle new parent emission. */
static int count_active_parent_sparks(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_SPARKS; i++)
    if (s->sparks[i].active && !s->sparks[i].has_split)
      n++;
  return n;
}

/* Spawn budget for this tick — refill toward pattern.target_sparks,
 * but cap the per-tick burst proportional to dt so a long pause
 * doesn't dump the entire pool in a single frame on resume. */
static int compute_spawn_count_for_tick(int active, const PatternParams *pp,
                                        float dt) {
  int target = pp->target_sparks;
  if (target > MAX_SPARKS)
    target = MAX_SPARKS;
  int spawn_cap = (int)((float)pp->target_sparks * dt * 6.0f) + 4;
  int n = target - active;
  if (n < 0)
    n = 0;
  if (n > spawn_cap)
    n = spawn_cap;
  return n;
}

/* Spawn `n` new parent sparks via the pattern's emission rules. */
static void spark_pool_topup_parents(Scene *s, int n) {
  for (int k = 0; k < n; k++)
    scene_spawn_spark(s);
}

/* SPARKLERS multi-level cascade pass — every still-eligible parent
 * spark (active && !has_split) that has crossed
 * SPARKLER_SPLIT_AGE_FRAC of its lifetime pops into
 * SPARKLER_CHILDREN_PER_SPLIT secondaries at its current (x, y).  The
 * parent's has_split latches to true so it can't pop again. */
static void cascade_sparkler_split_pass(Scene *s) {
  for (int i = 0; i < MAX_SPARKS; i++) {
    Spark *e = &s->sparks[i];
    if (!e->active || e->has_split)
      continue;
    if (e->age < e->life * SPARKLER_SPLIT_AGE_FRAC)
      continue;
    e->has_split = true;
    for (int c = 0; c < SPARKLER_CHILDREN_PER_SPLIT; c++)
      scene_spawn_child_spark(s, e->x, e->y);
  }
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;

  const PatternParams *pp = &pattern_params[s->current_pattern];

  /* 0. Tick the SPINNER wheel rotation. */
  wheel_phase_advance_and_wrap(s, dt);

  /* 1. Top up the spark pool toward the pattern's target density. */
  int active = (s->current_pattern == PATTERN_SPARKLERS)
                   ? count_active_parent_sparks(s)
                   : count_active_sparks(s);
  int to_spawn = compute_spawn_count_for_tick(active, pp, dt);
  spark_pool_topup_parents(s, to_spawn);

  /* 2. Integrate every active spark; resolve floor bounce + death. */
  float floor_y = scene_floor_y(s);
  float drag_factor = compute_drag_factor_continuous(pp->drag_coeff, dt);

  for (int i = 0; i < MAX_SPARKS; i++) {
    Spark *e = &s->sparks[i];
    if (!e->active)
      continue;

    spark_shift_trail_history(e);
    integrate_spark_semi_implicit_euler(e, pp->gravity, drag_factor, dt);

    if (spark_crossed_floor_descending(e, floor_y) &&
        reflect_spark_off_floor_and_test_settle(e, floor_y, pp->restitution,
                                                pp->floor_friction)) {
      e->active = false; /* settled — kill, skip death checks */
      continue;
    }

    if (spark_should_die(e, s->cols))
      e->active = false;
  }

  /* 3. SPARKLERS multi-level cascade — parents pop at mid-life. */
  if (s->current_pattern == PATTERN_SPARKLERS)
    cascade_sparkler_split_pass(s);
}

/*
 * spark_head_slot — heat-ramp slot for the spark's head from its
 * remaining-life fraction. T = 1 at birth (slot 7, white-hot), T = 0
 * at death (slot 0, dim). Same mapping as ember temperature in
 * embers.c so trained eyes read sparks consistently.
 */
static inline int spark_head_slot(const Spark *e) {
  float T = 1.0f - e->age / e->life;
  if (T < 0.0f)
    T = 0.0f;
  if (T > 1.0f)
    T = 1.0f;
  int slot = (int)(T * 7.999f);
  if (slot < 0)
    slot = 0;
  if (slot > 7)
    slot = 7;
  return slot;
}

/* ── Render primitives ────────────────────────────────────────────────── */

/* Older trail points (lower k) get cooler ramp slots — the streak
 * fades behind the spark as you walk back through the ring buffer.
 * Returns -1 when the offset has cooled below the ramp's bottom. */
static inline int trail_ramp_slot_for_offset(int head_slot, int k) {
  return head_slot - (TRAIL_LEN - k);
}

/* Brightness attribute by ramp slot — top of ramp BOLD, bottom DIM.
 * Shared between trail and head so both layers read the gradient
 * consistently. */
static inline int head_attr_by_brightness_slot(int slot) {
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* Phase ∈ [0, 2π) → one of four rotation glyphs in equal slices.
 * The cycle | / - \ reads as a visibly spinning axle when the SPINNER
 * phase advances at SPINNER_OMEGA rad/sec. */
static inline char wheel_axle_glyph_for_phase(float phase) {
  static const char rot_chars[4] = {'|', '/', '-', '\\'};
  int rot_idx = ((int)(phase * (2.0f / (float)M_PI))) & 3;
  return rot_chars[rot_idx];
}

/* Draw the entire motion-blur trail of one spark — TRAIL_LEN faded
 * glyphs, oldest dimmest, ramping up to one slot below the head. */
static void trail_draw_one_spark(const Spark *e, int head_slot, int cols,
                                 int rows_eff) {
  for (int k = 0; k < TRAIL_LEN; k++) {
    int slot = trail_ramp_slot_for_offset(head_slot, k);
    if (slot < 0)
      continue; /* cooled below the ramp */

    int ix = (int)(e->trail_x[k] + 0.5f);
    int iy = (int)(e->trail_y[k] + 0.5f);
    if (ix < 0 || ix >= cols)
      continue;
    if (iy < 0 || iy >= rows_eff)
      continue;

    char glyph = TRAIL_GLYPHS[slot];
    int attr = (slot <= 1) ? A_DIM : A_NORMAL;
    int pair = PAIR_HEAT_BASE + slot;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
  }
}

/* Draw the bright "head" glyph of one spark at its current (x, y).
 * Head_slot encodes the heat ramp from remaining-life fraction. */
static void head_draw_one_spark(const Spark *e, int cols, int rows_eff) {
  int ix = (int)(e->x + 0.5f);
  int iy = (int)(e->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows_eff)
    return;

  int slot = spark_head_slot(e);
  char glyph = HEAD_GLYPHS[slot];
  int attr = head_attr_by_brightness_slot(slot);
  int pair = PAIR_HEAT_BASE + slot;
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

/* All trails first — they sit BEHIND the heads after the head pass
 * overwrites them where they crossed (the bright dot the eye tracks
 * is never shadowed by a passing streak). */
static void spark_pool_draw_trails(const Scene *s, int rows_eff) {
  for (int i = 0; i < MAX_SPARKS; i++) {
    const Spark *e = &s->sparks[i];
    if (e->active)
      trail_draw_one_spark(e, spark_head_slot(e), s->cols, rows_eff);
  }
}

/* Heads on top — drawn after trails so they sit on the streaks. */
static void spark_pool_draw_heads(const Scene *s, int rows_eff) {
  for (int i = 0; i < MAX_SPARKS; i++) {
    const Spark *e = &s->sparks[i];
    if (e->active)
      head_draw_one_spark(e, s->cols, rows_eff);
  }
}

/* SPINNER core glyph at the wheel hub — visible | / - \ axle that
 * cycles in lockstep with the rim's emission phase. */
static void wheel_axle_draw(const Scene *s, int rows_eff) {
  float cx, cy;
  scene_emitter_xy(s, &cx, &cy);
  int ix = (int)(cx + 0.5f);
  int iy = (int)(cy + 0.5f);
  if (ix < 0 || ix >= s->cols)
    return;
  if (iy < 0 || iy >= rows_eff)
    return;

  char glyph = wheel_axle_glyph_for_phase(s->spinner_phase);
  int pair = PAIR_HEAT_BASE + 7;
  attron(COLOR_PAIR(pair) | A_BOLD);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void scene_draw(const Scene *s) {
  int rows_eff = s->rows - 1; /* leave bottom row for HUD */

  spark_pool_draw_trails(s, rows_eff); /* dim — drawn first       */
  spark_pool_draw_heads(s, rows_eff);  /* bright — sit on top     */

  if (s->current_pattern == PATTERN_GROUND_SPINNER)
    wheel_axle_draw(s, rows_eff); /* spinning axle at hub    */
}
/* ── end §5 — to understand the ncurses I/O wrapper, read §6 screen ── */

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *sc) {
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
static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static int scene_active_count(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_SPARKS; i++)
    if (s->sparks[i].active)
      n++;
  return n;
}

/*
 * screen_draw — render the scene, then paint a two-layer HUD over it:
 *
 *   Row 0        STATUS LINE.  Bright yellow PAIR_HUD + A_BOLD.
 *                Live state: pattern (or PAUSED), theme, spark count,
 *                emitter offset, fps, sim Hz, speed multiplier.
 *   Row rows-1   KEY HINT LINE.  Bright cyan PAIR_HINT + A_BOLD.
 *                Every interactive key the demo accepts.
 *
 * Both rows are pre-filled with their pair colour so the coloured
 * background spans the full width, and drawn AFTER scene_draw so
 * sparks never bleed through the bars.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  int active = scene_active_count(s);
  const char *state_str =
      s->paused ? "PAUSED   " : pattern_name(s->current_pattern);

  /* ── Top row: dynamic status ─────────────────────────────── */
  char status[220];
  snprintf(status, sizeof status,
           " SPARKS   %s   theme:%-8s   sparks:%4d   "
           "emit_dx:%+5.1f   %5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, active,
           (double)s->emitter_offset_x, fps, sim_fps, s->speed);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* ── Bottom row: every interactive key ───────────────────── */
  const char *hints = " q:quit  spc:pause  r:reseed  n/p:pattern  t/T:theme  "
                      "w/W:emitter  +/-:speed  ]/[:Hz ";

  int hint_row = sc->rows - 1;
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(hint_row, x, ' ');
  mvprintw(hint_row, 0, "%s", hints);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize_curses(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reseed(s);
    break;

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
    break;
  case '-':
    s->speed /= 2;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
    break;

  case ']':
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX)
      app->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN)
      app->sim_fps = SIM_FPS_MIN;
    break;

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  case 'n':
  case 'N':
    s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    break;

  case 'w':
    s->emitter_offset_x += EMITTER_SHIFT_STEP;
    break;
  case 'W':
    s->emitter_offset_x -= EMITTER_SHIFT_STEP;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
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
