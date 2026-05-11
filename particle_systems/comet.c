/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * comet.c — moving emitter traces an arc; leaves a fading particle trail
 *
 * DEMO: One or more COMETS spawn from a random screen edge, aimed at
 *       a random point in the opposite quadrant. As each comet
 *       crosses the screen it continuously EMITS trail particles at
 *       its current position. The particles stay (roughly) where
 *       emitted, age, and fade — so as the comet flies on, the
 *       particles it just emitted form a fading streak behind it.
 *       Older particles in the trail are dimmer; the head itself is
 *       a bright glowing glyph plus a small halo.
 *
 *       When a comet's trajectory takes it through the BOTTOM of the
 *       screen heading down, it DETONATES at the floor: a brief
 *       central '*+'-cross flash, then a 32-spark radial fan in 4
 *       staggered waves, fades over ~0.6 s.  The blast algorithm is
 *       ported from particle_systems/burst.c.
 *
 *       Patterns:
 *         SHOOTING_STAR  fast, straight, tight bright trail (meteor)
 *         FIREBALL       slower, puffy outward-drifting trail (warm)
 *         PLASMA_BOLT    fast erratic — random angle kicks, electric
 *                        crackling trail
 *
 * Study alongside:
 *   particle_systems/burst.c — the source of the impact-blast algorithm
 *                              (Burst FSM + radial fan + 4-wave stagger).
 *   embers.c                  — same age-driven cooling palette technique.
 *   rain.c, snow.c, fountain.c, vortex.c — same pool / tick / draw
 *                              framework. Distinct here: the EMITTER moves
 *                              and CRASHES.
 *
 * Section map:
 *   §1 config   — constants, themes, per-pattern parameters
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 8-pair colour ramp
 *   §4 comet    — Comet struct + spawn + tick (motion + emission)
 *   §5 trail    — TrailParticle + BlastParticle + Blast structs
 *   §6 scene    — pools (comet/trail/blast), tick, draw, prewarm, reseed
 *   §7 screen   — ncurses init / draw / resize
 *   §8 app      — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear pools)
 *   n / N      next pattern   (SHOOTING_STAR → FIREBALL → PLASMA_BOLT)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/comet.c \
 *       -o comet -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * READING ORDER
 *   1. CONCEPTS + MENTAL MODEL (below) — algorithm in plain English.
 *   2. GUIDED TUTORIAL (below) — 8 mini-lessons that build the system
 *      from "what is a moving emitter" up to the multi-pool 3-species
 *      architecture (comet + trail + blast).
 *   3. §1 config — every constant you'd tweak when experimenting,
 *      grouped by subsystem (sim / colour / blast).
 *   4. §4 comet — the moving emitter.  Read with §5 trail; together
 *      they are the file's main pedagogy.
 *   5. §5 trail + blast particles — the two follower species.
 *      BlastParticle is the smaller, drag-driven debris cousin.
 *   6. §6 scene — pools + tick + draw orchestration.  blast_ignite +
 *      the ground-impact branch in scene_tick are the new pieces.
 *   7. §7 screen, §8 app — ncurses + fixed-step loop boilerplate.
 *
 * NAMING
 *   Comet              the moving emitter — has pos, vel, age, emit_carry
 *   TrailParticle      one fading dot in the streak behind a comet
 *   Blast              one ground-impact explosion (lives ~0.6 s)
 *   BlastParticle      one spark inside a Blast (max 32 per blast)
 *   pattern_params[]   per-pattern visual+physics tunings (3 entries)
 *   themes[]           per-theme colour palette (10 entries)
 *   lcg_next / unit    fast LCG random (used in hot path; rand() in spawn)
 *   ground_y           rows-2: the last playable row before the HUD strip
 *   emit_carry         fractional accumulator so emit_rate stays accurate
 *                      under any sim_fps without integer-rounding drift
 *
 * BACKGROUND ASSUMED
 *   • Object-pool pattern (fixed array + active flag, no malloc in
 *     hot path).
 *   • Explicit Euler integration (`x += v · dt`).
 *   • Polar emission (radial fan from a centre — see §5 trail spread
 *     and blast_ignite).
 *   • Multiplicative drag and its exp(−rate·dt) per-frame form.
 *   • The ncurses double-buffer pattern (see CLAUDE.md framework docs).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system with THREE species:
 *                  COMETS (the moving emitters), TRAIL particles
 *                  (left behind by the comets), and BLASTS (radial spark
 *                  fans triggered when a comet hits the floor).
 *
 *                  Each tick:
 *                    1. Spawn comets up to `pattern.max_comets` if any
 *                       are inactive. Spawn from a random screen edge,
 *                       aimed at a random target on the OPPOSITE side.
 *                    2. Integrate each comet's position with explicit
 *                       Euler. PLASMA_BOLT pattern adds a random
 *                       per-tick angular kick (electric crackle).
 *                    3. Each comet emits `emit_rate · dt` new trail
 *                       particles at its current position with small
 *                       perpendicular spread; FIREBALL gives trail
 *                       particles outward velocity for puff feel.
 *                    4. Integrate trail particles: position += vel·dt,
 *                       vel *= drag (per second), age += dt; deactivate
 *                       at age >= life.
 *                    5. Comet leaves screen (with margin) → deactivate.
 *
 *                  Render order:
 *                    A. Trail particles (oldest first → newest last).
 *                    B. Comet heads with a small glowing halo.
 *
 *                  Rendering each trail particle: glyph + colour from
 *                  remaining life (fresh = brightest ramp slot, dying
 *                  = dimmest). Comet head always uses ramp[7] +
 *                  A_BOLD plus a 3-cell halo for "glow" feel.
 *
 *                  Distinct from `embers.c` (free particles rising
 *                  from a stationary source) and from `rain.c`/`snow.c`
 *                  (mass particles falling from a stationary line)
 *                  — here the EMITTER moves, so the trail is laid
 *                  along the emitter's PATH rather than emerging
 *                  from a fixed location.
 *
 * Data-structure : THREE fixed-size object pools, all with `active` flags
 *                  + linear-scan inactive search:
 *                    Comet[MAX_COMETS]            typically 1–3 active
 *                    TrailParticle[MAX_TRAIL]     a few hundred active
 *                    Blast[MAX_BLASTS]            0–N at any given moment;
 *                                                  each owns 32 BlastParticles
 *                  No malloc in the hot path; the whole simulation runs
 *                  out of a few statically-sized arrays.
 *
 * Rendering      : ASCII only. Trail glyphs from the airy ramp
 *                  `' .,:;-+*'` indexed by remaining life. Comet head
 *                  glyph by speed: `*` for fast, `O` for slow. Head
 *                  halo: 3-cell radial dim glow (`+` `:`).
 *
 * Performance    : O(MAX_COMETS · emission + MAX_TRAIL +
 *                    MAX_BLASTS · BLAST_PARTICLES) per tick.
 *                  At MAX_TRAIL ≤ 1000 + 1–3 comets + ≤ 8 active blasts
 *                  × 32 sparks = ≤ 1256 spark-updates/frame, well under
 *                  any realistic budget.
 *
 * References     :
 *   • Reeves, W. T. (1983) — "Particle Systems: A Technique for
 *     Modelling a Class of Fuzzy Objects", *ACM TOG* 2(2):91–108.
 *     Reeves' original demo was the genesis-planet fireball — the
 *     direct inspiration for the FIREBALL pattern here.
 *   • Wikipedia — [Meteor](https://en.wikipedia.org/wiki/Meteor).
 *     Real meteors trace nearly-straight paths through the atmosphere
 *     leaving an ionisation trail that decays in 1–10 seconds — the
 *     SHOOTING_STAR pattern's parameter set is calibrated against this.
 *   • particle_systems/burst.c — the source of the ground-impact blast
 *     algorithm (Burst FSM, 4-wave staggered emission, multiplicative
 *     drag).  Ported with two adaptations: angle restricted to the
 *     upper hemisphere, and seconds-based time instead of tick-based.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Picture a sparkler waved through the air. The TIP of the sparkler
 * (the comet head) is bright. As you wave it, it spits sparks (the
 * trail particles) along its path. The sparks stay roughly where
 * they were born and slowly fade. The whole image is the bright
 * head plus the cooling streak it left behind.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The comet is the EMITTER — it has a position and a velocity, and
 * it travels across the screen. Each frame it spits some particles
 * AT ITS CURRENT POSITION. Those particles don't follow the comet
 * — they sit roughly where they were born and fade out over a
 * second or two. So as the comet flies, it pulls a streak of
 * fading particles behind it. Different patterns just change how
 * fast the comet moves, how it curves, how many sparks it sheds,
 * and the colour palette.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SPAWN COMET. Pick a random screen edge (with a bias against
 *     the bottom). Place the comet just outside that edge at a
 *     random offset. Pick a target point in the opposite-side
 *     quadrant. Velocity = unit vector toward target × speed.
 *
 *  2. INTEGRATE COMET each tick:
 *       comet.x += comet.vx · dt
 *       comet.y += comet.vy · dt
 *       (PLASMA_BOLT only: rotate (vx, vy) by random ±KICK rad)
 *     Deactivate when comet leaves the screen with margin.
 *
 *  3. EMIT TRAIL each tick:
 *       n_emit = comet.emit_carry + emit_rate · dt
 *       emit_count = ⌊n_emit⌋
 *       comet.emit_carry = n_emit − emit_count
 *       for k in 1..emit_count:
 *         spawn TrailParticle at comet position
 *         with perpendicular spread ± particle_spread
 *         velocity = perpendicular kick · spread_drift_factor
 *         life = pattern.particle_life · (0.7 + r · 0.6)
 *
 *  4. INTEGRATE TRAIL each tick:
 *       trail.x += trail.vx · dt
 *       trail.y += trail.vy · dt
 *       trail.vx *= exp(−drag · dt)
 *       trail.vy *= exp(−drag · dt)
 *       trail.age += dt
 *       deactivate when age >= life
 *
 *  5. GROUND IMPACT (NEW). For each active comet:
 *       if comet.vy > 0 and comet.y >= rows − 2:
 *         blast_ignite(comet.x, ground_y)
 *         comet.active = false
 *     blast_ignite spawns 32 sparks in 4 staggered waves at angles
 *     [π, 2π] (upper hemisphere — sparks can't fly INTO the floor).
 *     Algorithm ported from particle_systems/burst.c.
 *
 *  6. INTEGRATE BLAST each tick:
 *       for each active Blast:
 *         flash_ttl −= dt
 *         for each BlastParticle:
 *           if delay > 0: delay −= dt; continue
 *           vel *= exp(−BLAST_DRAG_PER_SEC · dt)
 *           offset += vel · dt
 *           life −= dt
 *           kill if life<=0 or off-screen
 *         deactivate Blast when no live sparks AND flash_ttl<=0
 *
 *  7. RENDER:
 *       Trail particles: ramp slot = (1 − age/life) · 7
 *       Blast '*+' flash (during flash_ttl>0) + sparks (ramp by life)
 *       Comet heads: ramp slot 7 + A_BOLD + 3-cell halo
 *
 *  8. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Spawn velocity (toward random target on opposite quadrant):
 *    dx = target_x − spawn_x
 *    dy = target_y − spawn_y
 *    len = √(dx² + dy²)
 *    vx = speed · dx / len     vy = speed · dy / len
 *
 *  Plasma-bolt angular kick (per tick):
 *    α = (r − 0.5) · 2 · ANGULAR_KICK
 *    vx' = vx · cos α − vy · sin α
 *    vy' = vx · sin α + vy · cos α
 *
 *  Perpendicular trail spread:
 *    speed = √(vx² + vy²)
 *    perp_x = −vy / speed       perp_y = vx / speed
 *    kick = (r − 0.5) · 2 · particle_spread
 *    trail.x = comet.x + perp_x · kick
 *    trail.y = comet.y + perp_y · kick
 *    trail.vx = perp_x · kick · spread_drift_factor
 *    trail.vy = perp_y · kick · spread_drift_factor
 *
 *  Trail brightness:
 *    f = 1 − trail.age / trail.life       (1 fresh, 0 dying)
 *    ramp_slot = ⌊f · 7⌋
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • EMITTER MOVES BUT TRAIL DOESN'T. Trail particles are emitted
 *    at the comet's CURRENT position with low/zero velocity. As
 *    the comet flies on, those particles stay behind — the streak
 *    is automatic. If the trail particles inherited the comet's
 *    velocity, they'd FOLLOW the comet (no trail visible). This is
 *    the central trick of the moving-emitter pattern.
 *
 *  • EMIT FRACTIONAL CARRY. With emit_rate = 60/sec at 60 fps that's
 *    1.0 emit/tick; at 30 fps that's 2.0. But emit_rate = 90 at
 *    60 fps gives 1.5 — we lose the fractional half each tick.
 *    `comet.emit_carry` accumulates the fractional remainder so the
 *    long-run emission rate stays accurate.
 *
 *  • COMET POOL EXHAUSTION. With max_comets = 2 and a pattern
 *    spawning a new comet whenever a slot frees up, there's never
 *    overflow. The pool just keeps cycling through 2 active comets.
 *
 *  • TRAIL POOL EXHAUSTION. PLASMA_BOLT emits ~150 particles/sec ×
 *    life 0.8 sec ≈ 120 active. With 2 comets that's 240. Pool sized
 *    600 — plenty. If pool fills, emit silently fails — visual just
 *    shows slightly thinner trails.
 *
 *  • PLASMA_BOLT ANGULAR KICK. The kick rotates (vx, vy) using a
 *    proper rotation matrix. Without this, naive `vx += kick` would
 *    change the speed magnitude — the comet would gradually slow or
 *    speed up. Rotation preserves |v|.
 *
 *  • COMET LEAVES OFFSCREEN. We give a margin (2-5 cells) before
 *    deactivating, so the trail emitted at the edge of the screen
 *    has time to extend visibly. Without a margin, trails get cut
 *    off as soon as the comet head crosses the screen edge.
 *
 *  • PAUSE. Both comet and trail integration skip when paused. New
 *    comets won't spawn either (since spawn is conditional on
 *    inactive slots which won't change without integration).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Comet freezes mid-flight, trail particles freeze
 *    in place. Resume: motion continues from where it stopped.
 *
 *  • SHOOTING_STAR. Single fast comet draws a near-straight line
 *    across the screen with a tight bright trail of about 10–20
 *    cells. Reaches the opposite edge in under a second.
 *
 *  • FIREBALL. Slower, with a wider warmer trail that visibly
 *    PUFFS outward perpendicular to the flight direction. Trail
 *    cells live longer (1+ sec) so the streak hangs in the air.
 *
 *  • PLASMA_BOLT. Erratic path — the comet noticeably zigzags as
 *    the random angular kick changes its direction every frame.
 *    Often two bolts visible at once. Trail looks "electric".
 *
 *  • Theme cycle (`t`/`T`). Each theme produces a distinctive
 *    comet colour: ICE (blue/cyan) for SHOOTING_STAR, FIRE (red/
 *    orange) for FIREBALL, PLASMA (purple/cyan) for PLASMA_BOLT.
 *
 *  • Ground impact (NEW). Wait for a comet heading downward — usually
 *    SHOOTING_STAR launched from the top edge.  As it crosses the
 *    second-to-last row you should see a brief central '*+'-cross
 *    flash, then a fan of sparks rising upward and to the sides.  The
 *    sparks fade through the same theme ramp as the trail, so they
 *    visually belong to the same fireworks.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 * Eight mini-lessons.  Read them in order; each ends with the pseudocode
 * or struct that maps onto a real symbol below.
 *
 * ─── 1.  The moving-emitter trick  ──────────────────────────────────── *
 *   Most particle systems have a STATIONARY source (a fountain on the
 *   ground, a torch on a wall).  Here the source MOVES — and that's
 *   what makes the trail.
 *
 *   The rule is simple: each tick, the comet spits a few particles AT
 *   ITS CURRENT POSITION with zero (or near-zero) velocity.  As the
 *   comet moves on, those particles stay behind and fade.  The streak
 *   IS the history of where the comet has been.
 *
 *       tick 0:  ●               (comet at pos A; emit 2 sparks at A)
 *       tick 1:    ● . .          (comet moved; sparks from tick 0 fade)
 *       tick 2:      ● : ; .      (more sparks; older ones dimmer)
 *
 *   If trail particles INHERITED the comet's velocity, they would fly
 *   along WITH the comet and there would be no streak.  That mistake
 *   is the most common pitfall when porting a moving-emitter design.
 *
 *       Comet  ≡  { pos, vel, emit_carry, age, active }
 *
 * ─── 2.  Why TWO species (Comet + TrailParticle)  ────────────────────── *
 *   Could you put everything in one Particle type with an "is_comet"
 *   flag?  In principle yes.  In practice the two species have
 *   fundamentally different lifecycles:
 *
 *       Comet         lives 1–4 seconds, ONE per slot (max 8 total)
 *                     moves at constant speed, emits trail
 *       TrailParticle lives 0.4–1.2 s, ~1000 simultaneously alive
 *                     barely moves, only fades
 *
 *   Different sizes, different counts, different fields — different
 *   structs.  Splitting them keeps each pool small and each tick loop
 *   simple.  The "is_comet" union would carry overhead for every
 *   particle just to support a handful of special slots.
 *
 *       Comet[MAX_COMETS]            ← few, expensive, move
 *       TrailParticle[MAX_TRAIL]     ← many, cheap, sit and fade
 *
 *   The blast added later follows the same logic:
 *
 *       Blast[MAX_BLASTS]            ← few, owns its sparks
 *       └─ BlastParticle[32]         ← many per Blast, short-lived
 *
 * ─── 3.  Emit fractional carry  ──────────────────────────────────────── *
 *   Suppose emit_rate = 90 emissions/sec and you're running at 60 fps.
 *   Per frame the comet should emit 90 / 60 = 1.5 particles.  But you
 *   can't emit half a particle.  Naive int rounding gives 1 per
 *   frame → 60/sec average, off by 33%.
 *
 *   The fix is a fractional accumulator:
 *
 *       emit_carry += emit_rate · dt
 *       n_emit      = floor(emit_carry)
 *       emit_carry -= n_emit
 *
 *   Over time the leftover fraction adds up and triggers an extra
 *   emission "for free".  Long-run emission rate matches emit_rate
 *   exactly regardless of sim_fps.  Same trick is used by Bresenham's
 *   line algorithm, audio resamplers, and DDA rasterisers.
 *
 * ─── 4.  Pattern parameters as one struct  ───────────────────────────── *
 *   Three patterns (SHOOTING_STAR, FIREBALL, PLASMA_BOLT) — but you
 *   don't see three big switch statements in the tick loop.  Instead,
 *   one struct holds the per-pattern knobs:
 *
 *       PatternParams {
 *           int   max_comets;        // how many on screen at once
 *           float speed;             // base speed, cells/sec
 *           float angular_kick;      // PLASMA_BOLT randomness
 *           float emit_rate;         // trail particles/sec
 *           float particle_life;     // trail particle lifespan
 *           float particle_spread;   // perpendicular emission spread
 *           ...
 *       }
 *
 *   The tick loop reads `pattern_params[scene.current_pattern].emit_rate`
 *   and so on.  Pressing 'n' just bumps an index — no rebuild, no new
 *   tables, no recompile.  This is the cheapest way to ship N variants
 *   of an algorithm: parameterise the differences into a table.
 *
 * ─── 5.  PLASMA_BOLT angular kick — rotation, not addition  ──────────── *
 *   PLASMA_BOLT looks "electric" because its velocity vector wiggles
 *   each frame.  Naive implementation:
 *
 *       v.x += jitter;    v.y += jitter;
 *
 *   But that changes the MAGNITUDE of v, so the comet gradually speeds
 *   up or slows down.  After a few seconds the bolts either crawl or
 *   blink across the screen instantly.
 *
 *   Correct implementation rotates v by a small random angle α:
 *
 *       α     = random(±ANGULAR_KICK)
 *       v.x'  = v.x · cos α − v.y · sin α
 *       v.y'  = v.x · sin α + v.y · cos α
 *
 *   A rotation matrix preserves |v|.  The bolt zigzags freely without
 *   accelerating or decelerating — pure direction change.
 *
 * ─── 6.  Theme ramps + life-to-slot mapping  ─────────────────────────── *
 *   Each theme defines an 8-step colour ramp (cool/dim → hot/bright):
 *
 *       theme.ramp[0..7]:  e.g. ICE = { 24, 31, 67, 110, 117, 153, 195, 231 }
 *
 *   For each trail (or blast) particle, the current ramp index is:
 *
 *       fresh_fraction = 1 − age / life     // ∈ [0, 1]
 *       slot           = floor(fresh_fraction · 7.999)   // ∈ [0, 7]
 *
 *   Slot 7 (the FRESHEST end) is bright; slot 0 (dying) is cool/dim.
 *   The 8 colour pairs are PAIR_RAMP_BASE..+7 (initialised by
 *   theme_apply).  Pressing 't' rebinds the same 8 pair IDs with a
 *   different palette — existing particles pick up the new colour on
 *   the next mvaddch, no needs_clear.
 *
 * ─── 7.  Ground impact blast (ported from burst.c)  ──────────────────── *
 *   The newest piece.  When a comet's trajectory takes it through the
 *   floor (vy > 0, y >= rows-2), it DETONATES instead of dying off-
 *   screen.  The detonation is a port of particle_systems/burst.c's
 *   Burst, with three adaptations:
 *
 *     (1) ANGLE RESTRICTED to [π, 2π].  In screen coords (+y down),
 *         this gives vy ∈ [−1, 0] · speed — sparks rise or fly sideways,
 *         never INTO the floor.  burst.c uses a full circle [0, 2π]
 *         because its bursts can detonate anywhere on screen.
 *
 *     (2) NO FSM.  burst.c has Burst IDLE→FLASH→LIVE→IDLE.  Here a
 *         Blast is born LIVE and dies when all 32 sparks + the central
 *         flash counter are done — once.  The flash_ttl scalar
 *         replaces what was a FSM state.
 *
 *     (3) SECONDS-BASED.  burst.c counts in TICKS; comet.c works in
 *         dt-seconds throughout.  Drag becomes  v *= exp(−rate · dt)
 *         instead of  v *= 0.82 per tick.  Functionally the same
 *         exponential decay, just expressed for variable dt.
 *
 *   Wave delay (the shockwave reading): same 4 waves with delays
 *   0 / 0.033 / 0.066 / 0.10 s.  At 60 fps you see ring 0 expand for
 *   2 frames before ring 1 fires.  Slow the sim with '[' to make
 *   each ring discernible.
 *
 * ─── 8.  Putting it together — one tick  ─────────────────────────────── *
 *
 *       scene_tick(dt):
 *         1. top up comet pool to pattern.max_comets
 *         2. for each comet:
 *            - integrate vel, position
 *            - ground impact?  blast_ignite, deactivate, continue
 *            - off-screen?     deactivate, continue
 *            - emit  floor(emit_carry += emit_rate · dt)  trail particles
 *         3. for each trail particle:
 *            - integrate vel (drag), position, age; kill if past life
 *         4. for each active blast:
 *            - flash_ttl −= dt
 *            - for each blast particle:
 *                if delay > 0: delay -= dt; continue
 *                integrate vel (BLAST_DRAG), position, life; kill if dead
 *            - deactivate blast when nothing left to draw
 *
 *   At 60 fps with all three pools busy this is well under 2000 float
 *   updates per frame — invisible CPU cost.  The whole architecture
 *   fits in ~400 lines of physics + ~100 lines of rendering; the rest
 *   of this file is config, pedagogy, and ncurses glue.
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

    MAX_COMETS       =   8,
    MAX_TRAIL        = 1000,
    MAX_BLASTS       =   8,    /* one per comet possible at impact */

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_RAMP_BASE   =   3,    /* +0..+7 = 8 trail tints (cool→hot)   */
    PAIR_HEAD        =  11,    /* always-bright head colour            */
    PAIR_HALO        =  12,    /* head halo                            */
    PAIR_SKY         =  13,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Comet spawn margin (off-screen distance for spawn/kill). */
#define EDGE_MARGIN       3.0f

/*
 * Blast parameters (ported from particle_systems/burst.c, units adapted
 * from ticks → seconds to match comet.c's dt-driven scene_tick).
 *
 *   BLAST_PARTICLES     count of sparks per blast (32 reads as a "puff";
 *                       burst.c uses 48 but those bursts are the only
 *                       thing on screen — here we share the field with
 *                       the comet trail, so fewer sparks keep the
 *                       impact legible).
 *   BLAST_FLASH_SEC     central '*+'-cross flash lifetime — exactly
 *                       one or two frames at 30–60 Hz; the eye reads
 *                       "BANG, then shrapnel".
 *   BLAST_LIFE_BASE/JITTER  per-spark lifetime: 0.50–0.75 s, so each
 *                       blast fades over the same time as ~10 ticks of
 *                       drag.  After BLAST_LIFE_BASE+JITTER the spark
 *                       is reliably gone.
 *   BLAST_SPEED_MIN/MAX initial spark speed (cells/sec).  Tuned so the
 *                       fan visibly spreads ~6–12 cells before drag dominates.
 *   BLAST_DRAG_PER_SEC  exponential velocity decay rate.  At 3.0 /s the
 *                       spark loses 1−e^(−3) ≈ 95 % of its speed in
 *                       one second, comparable to burst.c's 0.82^tick
 *                       at 30 Hz (≈ exp(−5.9·dt) → 99.7 % loss/sec).
 *   BLAST_WAVE_COUNT    4 — same as burst.c.  Splits a ring into a
 *                       shockwave.
 *   BLAST_MAX_DELAY_SEC last wave fires this much later than wave 0.
 */
#define BLAST_PARTICLES         32
#define BLAST_FLASH_SEC         0.06f
#define BLAST_LIFE_BASE         0.50f
#define BLAST_LIFE_JITTER       0.25f
#define BLAST_SPEED_MIN        18.0f
#define BLAST_SPEED_MAX        42.0f
#define BLAST_DRAG_PER_SEC      3.0f
#define BLAST_WAVE_COUNT        4
#define BLAST_MAX_DELAY_SEC     0.10f

/* Pattern enum. */
typedef enum {
    PATTERN_SHOOTING_STAR = 0,
    PATTERN_FIREBALL      = 1,
    PATTERN_PLASMA_BOLT   = 2,
    N_PATTERNS            = 3,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SHOOTING_STAR: return "SHOOTING_STAR";
    case PATTERN_FIREBALL:      return "FIREBALL     ";
    case PATTERN_PLASMA_BOLT:   return "PLASMA_BOLT  ";
    default:                    return "?            ";
    }
}

/*
 * PatternParams — physics + visuals per pattern.
 *
 *   max_comets         : simultaneous active comets
 *   speed              : comet speed in cells/sec
 *   speed_jitter       : ± fraction of speed
 *   angular_kick       : per-tick random rotation (rad)
 *   emit_rate          : trail particles emitted per second per comet
 *   particle_life      : trail particle lifetime (sec)
 *   particle_spread    : ± perpendicular offset at spawn (cells)
 *   spread_drift_factor: how much initial perpendicular kick × becomes
 *                        velocity (0 = static trail; >0 = puff trail)
 *   trail_drag         : per-second velocity damping for trail
 *   head_glyph         : single character at the head
 */
typedef struct {
    int   max_comets;
    float speed;
    float speed_jitter;
    float angular_kick;
    float emit_rate;
    float particle_life;
    float particle_spread;
    float spread_drift_factor;
    float trail_drag;
    char  head_glyph;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /*                       cnt  spd  jit  ang   emit  life  spread sdrift drag  head */
    /* SHOOTING_STAR    */ { 1, 150.0f, 0.20f, 0.00f,  90.0f, 0.40f, 0.4f,  0.0f, 0.5f, '*' },
    /* FIREBALL         */ { 1,  55.0f, 0.20f, 0.00f, 180.0f, 1.20f, 1.5f,  6.0f, 1.0f, 'O' },
    /* PLASMA_BOLT      */ { 2, 130.0f, 0.30f, 0.10f, 150.0f, 0.80f, 1.0f,  3.0f, 1.5f, '*' },
};

/*
 * Themes — 8-step ramp for the trail (cool/dim → hot/bright). Plus
 * dedicated head and halo colours. All entries sit in the BRIGHT
 * HALF of the 256-colour cube per the CLAUDE.md "Theme Palette
 * Brightness" rule.
 */
typedef struct {
    const char *name;
    short       ramp[8];   /* dying → fresh */
    short       head;      /* always-bright head */
    short       halo;      /* halo around head */
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        ramp[0..7]                                       head halo sky */

    { "DEFAULT",  { 110, 117, 153, 159, 195, 195, 231, 255 },        231, 195, 234 },
    { "ICE",      {  24,  31,  67, 110, 117, 153, 195, 231 },        231, 195, 235 },
    { "FIRE",     {  88, 124, 130, 166, 196, 208, 214, 226 },        226, 220, 234 },
    { "PLASMA",   {  53,  91, 134, 165, 207, 213, 219, 225 },        225, 219, 234 },
    { "GOLD",     { 130, 137, 173, 179, 215, 222, 229, 230 },        230, 222, 234 },
    { "GREEN",    {  28,  34,  40,  64,  70, 112, 156, 192 },        192, 156, 234 },
    { "AURORA",   {  43,  79, 115, 121, 157, 195, 230, 231 },        231, 195, 234 },
    { "ROSE",     {  88, 131, 167, 174, 211, 217, 218, 231 },        231, 218, 234 },
    { "MONO",     { 244, 246, 248, 250, 252, 253, 254, 255 },        255, 252, 232 },
    { "VIOLET",   {  53,  54,  91, 134, 135, 176, 213, 219 },        225, 219, 233 },
};

/* Trail glyph ramp (dying → fresh). */
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
        init_pair(PAIR_HEAD, t->head, -1);
        init_pair(PAIR_HALO, t->halo, -1);
        init_pair(PAIR_SKY,  t->sky,  -1);
    } else {
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), COLOR_WHITE, -1);
        init_pair(PAIR_HEAD, COLOR_WHITE,  -1);
        init_pair(PAIR_HALO, COLOR_WHITE,  -1);
        init_pair(PAIR_SKY,  COLOR_BLACK,  -1);
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
/* §4  comet                                                              */
/* ===================================================================== */

/*
 * §4 PREAMBLE — the moving emitter
 * ─────────────────────────────────
 *
 * A Comet is a small struct that holds enough state to (a) traverse the
 * screen on a straight (or wiggling) path, and (b) sprinkle trail
 * particles AT ITS CURRENT POSITION as it goes.  It owns no glyph and
 * no colour — those come from `pattern_params` and `themes` at draw
 * time.  Position lives in CELL coordinates (not pixels); velocity is
 * cells per second.
 *
 * The two random helpers `lcg_next` / `lcg_unit` are intentionally
 * cheap (one mul, one add per call).  They're used in HOT-PATH choices
 * (per-frame angle jitter, per-particle wave selection); `rand()` is
 * reserved for cold-path startup work (signal handler seeding).
 *
 * COORDINATE SYSTEMS USED HERE
 *   (x, y)   comet centre in screen CELLS (float for sub-cell motion)
 *   (vx, vy) velocity in CELLS / SECOND
 *   age      seconds since spawn; informational only (does not control
 *            death — that's handled by off-screen check + ground impact)
 *   emit_carry  fractional accumulator described in GUIDED TUTORIAL #3
 */
typedef struct {
    float x, y;
    float vx, vy;
    float emit_carry;     /* fractional emission accumulator           */
    float age;
    bool  active;
} Comet;

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
/* §5  trail + blast particles                                            */
/* ===================================================================== */

/*
 * §5 PREAMBLE — the two follower species
 * ───────────────────────────────────────
 *
 * The comet is the actor; the trail and blast particles are everything
 * the comet leaves behind.  They share a pattern (fixed-size pool with
 * `active` flag, per-frame integrate-and-fade) but differ in physics:
 *
 *   TrailParticle  Emitted continuously along the comet's path.
 *                  Lives 0.4–1.2 s.  Drifts slowly.  Fades through the
 *                  theme's 8-step ramp from FRESH (slot 7, bright) to
 *                  DYING (slot 0, dim).
 *
 *   BlastParticle  Emitted in a single burst of 32 when a comet hits
 *                  the floor.  Lives 0.5–0.75 s.  Has strong drag so
 *                  it visibly DECELERATES across its lifetime.  Carries
 *                  a wave-stagger countdown so groups of sparks fire at
 *                  different frame offsets — see GUIDED TUTORIAL #7.
 *
 * Both are pure POSITION + VELOCITY + LIFE + GLYPH structs with no
 * back-pointer to their source.  Once spawned they're owned by the
 * Scene-level pool, not the Comet/Blast that emitted them.
 *
 * COORDINATE SYSTEMS USED HERE
 *   TrailParticle (x, y)        ABSOLUTE cell-space position
 *   BlastParticle (rx, ry)      OFFSET in cells from its parent Blast's
 *                                centre (so a Blast can move? no — but
 *                                this matches burst.c's convention,
 *                                makes the math read like burst.c, and
 *                                centralises clipping in the draw loop)
 */
typedef struct {
    float x, y;
    float vx, vy;
    float age, life;
    bool  active;
} TrailParticle;

/*
 * BlastParticle — one spark in a ground-impact explosion.
 *
 * Mirrors burst.c's Particle struct, adapted to comet.c's cell-space
 * + seconds-time conventions:
 *   - position kept as offset (rx, ry) from the parent Blast's centre,
 *     NOT pixel-space (burst.c uses pixels with an ASPECT compensator
 *     at draw time; comet.c works in cells throughout, so no aspect
 *     correction is needed here).
 *   - life is a remaining-seconds float (burst.c's life is 0..1 with
 *     a per-tick decay; same idea, different units).
 *   - delay is the wave-stagger countdown in seconds.
 */
typedef struct {
    float rx, ry;        /* offset from Blast centre, in cells */
    float vx, vy;        /* velocity, cells/sec */
    float life;          /* remaining seconds */
    float max_life;      /* original life, for ramp-index calculation */
    float delay;         /* wave-stagger countdown */
    char  sym;           /* fixed at spawn */
    bool  alive;
} BlastParticle;

/*
 * Blast — radial spark fan triggered when a comet hits the ground.
 *
 * Algorithm ported from particle_systems/burst.c Burst, with two changes:
 *   1. Angles restricted to [π, 2π] (upper hemisphere in screen coords),
 *      because debris from a floor impact only goes UP and sideways —
 *      sparks aimed into the floor would vanish immediately.
 *   2. No FSM (IDLE→FLASH→LIVE) — a Blast is born LIVE, runs until all
 *      sparks die OR all flash + sparks are dead, then deactivates.
 *      The flash_ttl scalar replaces burst.c's BS_FLASH state.
 */
typedef struct {
    float         cx, cy;       /* impact centre, in cells */
    float         flash_ttl;    /* central '*+'-cross flash remaining (sec) */
    bool          active;
    BlastParticle parts[BLAST_PARTICLES];
} Blast;

/* ===================================================================== */
/* §6  scene — pools, tick, draw                                         */
/* ===================================================================== */

/*
 * §6 PREAMBLE — orchestrator
 * ───────────────────────────
 *
 * The Scene owns the three pools + the running configuration (current
 * pattern, theme, speed, paused state, RNG state, terminal extent).
 * Everything that mutates particles passes through scene_tick();
 * everything that draws passes through scene_draw().
 *
 * The split makes the FRAME pipeline trivial:
 *
 *        scene_tick(dt)                  ← physics, called N times
 *        scene_draw(scene)               ← rendering, called once
 *        screen_draw_hud(...)            ← HUD overlay
 *        screen_present()                ← ncurses flush
 *
 * Helpers (comet_pool_find_inactive, trail_pool_find_inactive,
 * blast_pool_find_inactive) do an O(n) scan for an unused slot.  At
 * pool sizes 8 / 1000 / 8 this is cheap and removes the need for a
 * free-list (which would be 5× more code for zero observable gain).
 *
 * scene_clear_pools / scene_init / scene_resize / scene_reseed —
 * lifecycle housekeeping.  reseed binds 'r': it wipes all three pools
 * and spawns one fresh comet so the screen never goes empty.
 */

typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    uint32_t  rng;
    int       rows, cols;

    Comet         comets[MAX_COMETS];
    TrailParticle trail [MAX_TRAIL];
    Blast         blasts[MAX_BLASTS];
} Scene;

static int comet_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_COMETS; i++)
        if (!s->comets[i].active) return i;
    return -1;
}

static int trail_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_TRAIL; i++)
        if (!s->trail[i].active) return i;
    return -1;
}

static int blast_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_BLASTS; i++)
        if (!s->blasts[i].active) return i;
    return -1;
}

static void scene_clear_pools(Scene *s)
{
    for (int i = 0; i < MAX_COMETS; i++) s->comets[i].active = false;
    for (int i = 0; i < MAX_TRAIL;  i++) s->trail [i].active = false;
    for (int i = 0; i < MAX_BLASTS; i++) s->blasts[i].active = false;
}

/*
 * blast_ignite — spawn one ground-impact blast at (cx, cy).
 *
 * PURPOSE
 *   Called by scene_tick the frame a comet's trajectory crosses the
 *   floor heading down.  Picks an unused Blast slot, sets its centre,
 *   primes the central flash, and spawns BLAST_PARTICLES sparks in 4
 *   staggered waves around the upper hemisphere.
 *
 * PSEUDOCODE  (mirrors burst.c burst_ignite, restricted angle range)
 *   slot       = free Blast in pool                        // skip if full
 *   centre     = (cx, cy)
 *   flash_ttl  = BLAST_FLASH_SEC
 *   for i = 0..N-1:
 *       angle  = π + (i/N)·π + jitter(±0.1)                // [π, 2π] — upward
 *       speed  = uniform(BLAST_SPEED_MIN, BLAST_SPEED_MAX)
 *       wave   = i mod WAVE_COUNT
 *       delay  = wave · MAX_DELAY / (WAVE_COUNT − 1)
 *       vel    = (cos angle, sin angle) · speed             // sin<0 → up
 *       life   = uniform(BLAST_LIFE_BASE .. +JITTER)
 *       glyph  = random from "*+.,oO!#"                     // FIXED at spawn
 *
 * WHY UPPER-HEMISPHERE ONLY
 *   In screen coords, +y is DOWN.  Floor debris that travels in +y
 *   would immediately leave the playable area (which ends at row
 *   rows-2, just above the HUD).  Restricting to [π, 2π] gives
 *   sin(angle) ∈ [−1, 0]  → vy ≤ 0  → all sparks rise or fly sideways.
 */
static void blast_ignite(Scene *s, float cx, float cy)
{
    int idx = blast_pool_find_inactive(s);
    if (idx < 0) return;
    Blast *b = &s->blasts[idx];

    b->cx        = cx;
    b->cy        = cy;
    b->flash_ttl = BLAST_FLASH_SEC;
    b->active    = true;

    static const char k_blast_syms[] = "*+.,oO!#";
    const int n_syms = (int)sizeof k_blast_syms - 1;

    for (int i = 0; i < BLAST_PARTICLES; i++) {
        BlastParticle *p = &b->parts[i];

        float t     = (float)i / (float)BLAST_PARTICLES;
        float angle = (float)M_PI + t * (float)M_PI
                    + (lcg_unit(&s->rng) - 0.5f) * 0.2f;
        float speed = BLAST_SPEED_MIN
                    + lcg_unit(&s->rng) * (BLAST_SPEED_MAX - BLAST_SPEED_MIN);
        int   wave  = i % BLAST_WAVE_COUNT;
        float delay = (float)wave * (BLAST_MAX_DELAY_SEC
                                     / (float)(BLAST_WAVE_COUNT - 1));

        p->rx       = 0.0f;
        p->ry       = 0.0f;
        p->vx       = cosf(angle) * speed;
        p->vy       = sinf(angle) * speed;
        p->max_life = BLAST_LIFE_BASE
                    + lcg_unit(&s->rng) * BLAST_LIFE_JITTER;
        p->life     = p->max_life;
        p->delay    = delay;
        p->sym      = k_blast_syms[lcg_next(&s->rng) % (unsigned)n_syms];
        p->alive    = true;
    }
}

/*
 * scene_spawn_comet — place one new comet in the pool.
 *
 * PURPOSE
 *   Called by scene_tick whenever the pool is below pattern.max_comets.
 *   Picks (a) a screen edge to spawn from, (b) a target point on the
 *   opposite side of the screen, then builds a velocity vector aimed
 *   at the target.  The comet appears just OUTSIDE the chosen edge so
 *   its visible entrance is "from off-screen", not popping into view.
 *
 * PSEUDOCODE
 *   slot     = unused Comet in pool                       // skip if full
 *   edge     = weighted_random({ top:45%, left:25%, right:25%, bottom:5% })
 *   spawn   = point just outside the chosen edge
 *   target  = random in opposite-quadrant band of the same edge
 *   d       = target − spawn;  len = |d|;  unit = d / len
 *   speed   = pp.speed · uniform(1 − jit/2,  1 + jit/2)
 *   velocity = unit · speed
 *
 * MENTAL MODEL
 *   The bottom edge is biased AGAINST (only 5 %) because comets coming
 *   from below look unnatural — meteors don't rise.  Top edge gets the
 *   highest weight (45 %) so most comets dive DOWNWARD, which is also
 *   what lets the ground-impact blast happen frequently.
 *
 *   The target-on-opposite-side trick guarantees diagonal arcs that
 *   visibly cross the screen.  If we picked random direction angles
 *   uniformly, half the time the comet would aim into the same edge
 *   it spawned from and exit invisibly.
 *
 * INPUTS / OUTPUTS
 *   s   ← Scene to draw an inactive slot from (mutated: one comet
 *         becomes active and gets full state)
 *
 * UNITS
 *   spawn_x, spawn_y, target_x, target_y: cells (float for precision)
 *   speed:  cells/second (modulated by pattern jitter)
 *
 * WHY IT EXISTS (vs inlining in scene_tick)
 *   scene_tick stays at orchestrator level — "top up the pool, then
 *   tick each kind of particle".  All the picking-and-aiming logic
 *   lives here as one unit you can read in 50 lines.
 */
static void scene_spawn_comet(Scene *s)
{
    int idx = comet_pool_find_inactive(s);
    if (idx < 0) return;
    Comet *c = &s->comets[idx];

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* Pick spawn edge with bias against the bottom (less common). */
    float r_edge = lcg_unit(&s->rng);
    int edge;     /* 0=top, 1=left, 2=right, 3=bottom */
    if (r_edge < 0.45f)      edge = 0;
    else if (r_edge < 0.70f) edge = 1;
    else if (r_edge < 0.95f) edge = 2;
    else                     edge = 3;

    float spawn_x = 0, spawn_y = 0;
    float r1 = lcg_unit(&s->rng);
    switch (edge) {
    case 0: spawn_x = r1 * (float)s->cols; spawn_y = -EDGE_MARGIN; break;
    case 1: spawn_x = -EDGE_MARGIN; spawn_y = r1 * (float)s->rows; break;
    case 2: spawn_x = (float)s->cols + EDGE_MARGIN;
            spawn_y = r1 * (float)s->rows; break;
    case 3: spawn_x = r1 * (float)s->cols;
            spawn_y = (float)s->rows + EDGE_MARGIN; break;
    }

    /* Pick target on the opposite side (with the edge zone biased to
     * the opposite-quadrant area for visible diagonal arcs). */
    float r2 = lcg_unit(&s->rng);
    float r3 = lcg_unit(&s->rng);
    float target_x = 0, target_y = 0;
    switch (edge) {
    case 0: target_x = r2 * (float)s->cols;
            target_y = (float)s->rows * (0.55f + r3 * 0.45f); break;
    case 1: target_x = (float)s->cols * (0.55f + r2 * 0.45f);
            target_y = r3 * (float)s->rows; break;
    case 2: target_x = (float)s->cols * (0.0f + r2 * 0.45f);
            target_y = r3 * (float)s->rows; break;
    case 3: target_x = r2 * (float)s->cols;
            target_y = (float)s->rows * (0.0f + r3 * 0.45f); break;
    }

    float dx = target_x - spawn_x;
    float dy = target_y - spawn_y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-3f) len = 1.0f;

    /* Speed with jitter. */
    float speed = pp->speed
                * ((1.0f - pp->speed_jitter * 0.5f)
                   + lcg_unit(&s->rng) * pp->speed_jitter);

    c->x          = spawn_x;
    c->y          = spawn_y;
    c->vx         = speed * dx / len;
    c->vy         = speed * dy / len;
    c->emit_carry = 0.0f;
    c->age        = 0.0f;
    c->active     = true;
}

/*
 * scene_emit_trail — drop one trail particle at the comet's current
 * position with a perpendicular spread + drift.
 *
 * PURPOSE
 *   Implements the moving-emitter trick (GUIDED TUTORIAL #1).  Called
 *   `floor(emit_carry + emit_rate · dt)` times per frame per comet.
 *   The particle is BORN at the comet's pos with a small lateral kick;
 *   it doesn't inherit the comet's velocity — that's what lets the
 *   streak FALL BEHIND the comet instead of following it.
 *
 * PSEUDOCODE
 *   slot     = unused TrailParticle in pool                 // skip if full
 *   speed    = |comet.velocity|;  if zero, set to 1 (unit safety)
 *   perp     = (−vy, vx) / speed                            // 90° CCW unit
 *   kick     = uniform(−1, 1) · pp.particle_spread          // lateral offset
 *   pos      = comet.pos + perp · kick                      // sub-cell jitter
 *   vel      = perp · kick · pp.spread_drift_factor         // weak drift
 *   age      = 0
 *   life     = pp.particle_life · uniform(0.7, 1.3)         // ±30% jitter
 *
 * MENTAL MODEL
 *   Imagine the comet trailing a wet paintbrush.  Each tick a drop
 *   falls off — not straight down, but with a small sideways flick
 *   PERPENDICULAR to where the brush is going.  Some flicks have zero
 *   sideways velocity (SHOOTING_STAR: spread_drift_factor=0 → tight
 *   line); some have positive velocity (FIREBALL: factor=6 → drops
 *   drift outward as a "puff").
 *
 *   The perpendicular direction is the comet's velocity rotated 90°.
 *   In 2D the rotation is just (−vy, vx) — a textbook trick.
 *
 * INPUTS / OUTPUTS
 *   s   ← Scene; one inactive trail slot becomes active (mutated)
 *   c   → emitting comet (const — its state is read, never written)
 *
 * UNITS
 *   speed:   cells/second magnitude of comet.velocity
 *   kick:    cells (perpendicular offset distance)
 *   life:    seconds, with per-particle jitter ±30 %
 *
 * WHY IT EXISTS (vs inlining the emission in scene_tick's comet loop)
 *   The emit-fractional-carry logic in scene_tick computes HOW MANY
 *   particles to make this frame; this function computes WHERE each
 *   ONE goes.  Splitting the "count" decision from the "build" code
 *   keeps each at one logical level — counters vs particles.
 */
static void scene_emit_trail(Scene *s, const Comet *c)
{
    int idx = trail_pool_find_inactive(s);
    if (idx < 0) return;
    TrailParticle *p = &s->trail[idx];

    const PatternParams *pp = &pattern_params[s->current_pattern];

    float speed = sqrtf(c->vx * c->vx + c->vy * c->vy);
    if (speed < 1e-3f) speed = 1.0f;
    float perp_x = -c->vy / speed;
    float perp_y =  c->vx / speed;

    float kick = (lcg_unit(&s->rng) - 0.5f) * 2.0f * pp->particle_spread;

    float r_life = lcg_unit(&s->rng);

    p->x      = c->x + perp_x * kick;
    p->y      = c->y + perp_y * kick;
    p->vx     = perp_x * kick * pp->spread_drift_factor;
    p->vy     = perp_y * kick * pp->spread_drift_factor;
    p->age    = 0.0f;
    p->life   = pp->particle_life * (0.7f + r_life * 0.6f);
    p->active = true;
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_SHOOTING_STAR;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    scene_clear_pools(s);
    /* Seed the first comet so the screen is not empty. */
    scene_spawn_comet(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    scene_clear_pools(s);
    scene_spawn_comet(s);
}

/*
 * scene_tick — advance the whole simulation one frame.
 *
 * PURPOSE
 *   The per-frame orchestrator.  Runs five distinct phases in order;
 *   the order matters because later phases observe state set by earlier
 *   ones (e.g. trail emission reads comet.pos, which was just updated
 *   by comet integration in the same tick).
 *
 * PSEUDOCODE
 *   if paused:  return
 *   dt *= speed_mul                            // user '+/-' speed knob
 *
 *   PHASE 1 — top up comet pool to pattern.max_comets
 *   PHASE 2 — integrate each active comet:
 *     - apply plasma angular_kick if pattern requests it
 *     - x += vx·dt;  y += vy·dt;  age += dt
 *     - GROUND IMPACT? blast_ignite(impact_x, ground_y); deactivate
 *     - OFF-SCREEN?    deactivate
 *     - emit trail: floor(emit_carry += emit_rate·dt) particles
 *   PHASE 3 — integrate each active trail particle:
 *     - x += vx·dt; vx *= exp(−drag·dt); age += dt;  die at age≥life
 *   PHASE 4 — tick each active blast:
 *     - flash_ttl −= dt
 *     - for each spark: delay or drag+integrate+fade
 *     - deactivate blast when nothing left to draw
 *
 * MENTAL MODEL
 *   Three pools, three loops, all driven by the same dt.  The whole
 *   physics fits in this one function plus the helpers it calls.  Once
 *   you understand WHICH pool is touched in which phase, every visible
 *   behaviour traces back to a specific line here.
 *
 *   The speed multiplier (speed_mul) is the user's "global time dial":
 *   '+' doubles dt for everything, '-' halves it.  Comets fly faster,
 *   trails fade faster, blasts puff out faster — the whole simulation
 *   runs in slow-motion or fast-forward without changing any constant.
 *
 * INPUTS / OUTPUTS
 *   s   ← Scene (mutated — every pool advances by dt)
 *   dt  → frame time in seconds (typically 1/60)
 *
 * UNITS
 *   dt: seconds (after speed_mul scaling)
 *
 * WHY THE PHASES ARE IN THIS ORDER
 *   1 before 2 — must have comets to integrate
 *   2 before 3 — trail emission reads comet.pos that 2 just updated
 *   3 after 2  — trail particles can be culled in the same frame
 *                they were emitted (rare but possible at high drag)
 *   4 last     — blasts don't interact with anything else, but doing
 *                them last keeps the comet/trail loop tight
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* 1. Top up comets to max. */
    int active_comets = 0;
    for (int i = 0; i < MAX_COMETS; i++) if (s->comets[i].active) active_comets++;
    int target_comets = pp->max_comets;
    if (target_comets > MAX_COMETS) target_comets = MAX_COMETS;
    for (int k = active_comets; k < target_comets; k++) scene_spawn_comet(s);

    /* 2. Integrate comets (motion + emission + plasma kick). */
    for (int i = 0; i < MAX_COMETS; i++) {
        Comet *c = &s->comets[i];
        if (!c->active) continue;

        /* Plasma-bolt: random rotation of velocity vector. */
        if (pp->angular_kick > 0.0f) {
            float ang = (lcg_unit(&s->rng) - 0.5f) * 2.0f * pp->angular_kick;
            float ca = cosf(ang);
            float sa = sinf(ang);
            float nvx = ca * c->vx - sa * c->vy;
            float nvy = sa * c->vx + ca * c->vy;
            c->vx = nvx;
            c->vy = nvy;
        }

        /* Position integration. */
        c->x += c->vx * dt;
        c->y += c->vy * dt;
        c->age += dt;

        /*
         * Ground impact — comet heading DOWN and y crossed the floor.
         * The "floor" is rows-2 (the last playable row; rows-1 is HUD).
         * Trigger a blast at the impact x, then consume the comet.
         * Comets coming from the bottom edge spawn with vy<0 (going up),
         * so the vy>0 guard correctly filters them out.
         */
        float ground_y = (float)s->rows - 2.0f;
        if (c->vy > 0.0f && c->y >= ground_y) {
            float impact_x = c->x;
            if (impact_x < 0.0f)            impact_x = 0.0f;
            if (impact_x >= (float)s->cols) impact_x = (float)s->cols - 1.0f;
            blast_ignite(s, impact_x, ground_y);
            c->active = false;
            continue;
        }

        /* Off-screen with margin → die quietly (any other edge). */
        if (c->x < -EDGE_MARGIN || c->x > (float)s->cols + EDGE_MARGIN ||
            c->y < -EDGE_MARGIN || c->y > (float)s->rows + EDGE_MARGIN) {
            c->active = false;
            continue;
        }

        /* Trail emission with fractional carry. */
        c->emit_carry += pp->emit_rate * dt;
        int n_emit = (int)c->emit_carry;
        c->emit_carry -= (float)n_emit;
        for (int k = 0; k < n_emit; k++) scene_emit_trail(s, c);
    }

    /* 3. Integrate trail particles. */
    float drag_factor = expf(-pp->trail_drag * dt);
    for (int i = 0; i < MAX_TRAIL; i++) {
        TrailParticle *p = &s->trail[i];
        if (!p->active) continue;
        p->x  += p->vx * dt;
        p->y  += p->vy * dt;
        p->vx *= drag_factor;
        p->vy *= drag_factor;
        p->age += dt;
        if (p->age >= p->life) p->active = false;
    }

    /*
     * 4. Tick blast particles (impact debris).
     *    Same explicit-Euler integration as the trail loop, but with
     *    a stronger drag (BLAST_DRAG_PER_SEC ≈ 3 /s vs trail's 0.5–1.5),
     *    plus the wave-delay countdown that defers wave-1..3 sparks for
     *    a few frames so the eye reads "shockwave" not "single ring".
     */
    float blast_drag_factor = expf(-BLAST_DRAG_PER_SEC * dt);
    for (int i = 0; i < MAX_BLASTS; i++) {
        Blast *b = &s->blasts[i];
        if (!b->active) continue;

        if (b->flash_ttl > 0.0f) b->flash_ttl -= dt;

        bool any = false;
        for (int j = 0; j < BLAST_PARTICLES; j++) {
            BlastParticle *p = &b->parts[j];
            if (!p->alive) continue;
            if (p->delay > 0.0f) { p->delay -= dt; any = true; continue; }

            p->vx *= blast_drag_factor;
            p->vy *= blast_drag_factor;
            p->rx += p->vx * dt;
            p->ry += p->vy * dt;
            p->life -= dt;

            float sx = b->cx + p->rx;
            float sy = b->cy + p->ry;
            if (p->life <= 0.0f
                || sx < 0.0f || sx >= (float)s->cols
                || sy < 0.0f || sy >= (float)(s->rows - 1)) {
                p->alive = false;
                continue;
            }
            any = true;
        }
        if (!any && b->flash_ttl <= 0.0f) b->active = false;
    }
}

/*
 * scene_draw — render the whole scene.
 *
 * PURPOSE
 *   Paint trail particles, then blast effects, then comet heads with
 *   halos.  The order matters: each layer can write over the one
 *   below it, so the painter's algorithm puts BACK material first
 *   and FOREGROUND material last.
 *
 * PSEUDOCODE
 *   rows_eff = rows − 1                          // reserve HUD row
 *
 *   PHASE A — trail particles (background streak):
 *     for each active particle:
 *       slot = floor((1 − age/life) · 7.999)     // FRESH=7, DYING=0
 *       attr = (slot≥6) ? BOLD : (slot≤1) ? DIM : NORMAL
 *       mvaddch with RAMP_GLYPHS[slot] in COLOR_PAIR(PAIR_RAMP_BASE+slot)
 *
 *   PHASE B — blasts (mid-ground impact debris):
 *     for each active blast:
 *       draw '*+' cross during flash_ttl > 0
 *       draw each spark with life-based ramp slot (same scheme as trail)
 *
 *   PHASE C — comet heads with halo (foreground):
 *     for each active comet:
 *       paint 3×3 halo of '+' '/' ':' around (hx, hy)
 *       paint head glyph from pattern_params at (hx, hy) in PAIR_HEAD+BOLD
 *
 * MENTAL MODEL
 *   The render is one big z-sort by category, not by depth.  Trails
 *   are "behind" because they were emitted in the past; comets are
 *   "in front" because they're the active actors.  Blasts sit between:
 *   they erupt from the floor, so trail sparks above them stay visible.
 *
 *   The 8-step ramp is the visual heart of the scene.  Slot 7 (fresh)
 *   gets A_BOLD; slot 0 (dying) gets A_DIM.  Everything in between is
 *   COLOR_PAIR-only.  This three-tier brightness curve produces the
 *   characteristic "cool ember" look of every theme.
 *
 * INPUTS / OUTPUTS
 *   s   → Scene (const — never mutated)
 *
 * WHY IT EXISTS (vs scattering draws across the tick functions)
 *   Tick is for STATE; draw is for PIXELS.  Splitting them means you
 *   can change rendering (e.g. add motion blur, swap glyph ramps,
 *   add a debug overlay) without touching the physics.  Same split
 *   that lets burst.c add 4 debug overlay modes without changing
 *   any tick code.
 */
static void scene_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;     /* leave bottom row for HUD */

    /* ── Trail particles ─────────────────────────────────────────── */
    for (int i = 0; i < MAX_TRAIL; i++) {
        const TrailParticle *p = &s->trail[i];
        if (!p->active) continue;
        int ix = (int)(p->x + 0.5f);
        int iy = (int)(p->y + 0.5f);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < 0 || iy >= rows_eff) continue;

        float f = 1.0f - p->age / p->life;
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        int slot = (int)(f * 7.999f);
        if (slot < 0) slot = 0;
        if (slot > 7) slot = 7;

        char glyph = RAMP_GLYPHS[slot];
        int  attr  = (slot >= 6) ? A_BOLD
                   : (slot <= 1) ? A_DIM
                   :               A_NORMAL;
        int  pair  = PAIR_RAMP_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }

    /*
     * ── Blast particles + central flash ──────────────────────────
     *
     * Drawn after the trail (so blast sparks sit ON TOP of the trail)
     * but before the comet heads (which can't coexist with blasts
     * anyway — the comet dies the frame its blast spawns).
     */
    for (int bi = 0; bi < MAX_BLASTS; bi++) {
        const Blast *b = &s->blasts[bi];
        if (!b->active) continue;

        /* Central '*+' cross — burst.c's FLASH state, here a TTL counter. */
        if (b->flash_ttl > 0.0f) {
            int fx = (int)(b->cx + 0.5f);
            int fy = (int)(b->cy + 0.5f);
            if (fx >= 0 && fx < s->cols && fy >= 0 && fy < rows_eff) {
                attron(COLOR_PAIR(PAIR_HEAD) | A_BOLD);
                mvaddch(fy, fx, '*');
                if (fx > 0)            mvaddch(fy,   fx - 1, '+');
                if (fx < s->cols - 1)  mvaddch(fy,   fx + 1, '+');
                if (fy > 0)            mvaddch(fy-1, fx,     '+');
                attroff(COLOR_PAIR(PAIR_HEAD) | A_BOLD);
            }
        }

        /* Per-spark — life/max_life → 0..7 ramp slot, fading hot→cool. */
        for (int j = 0; j < BLAST_PARTICLES; j++) {
            const BlastParticle *p = &b->parts[j];
            if (!p->alive || p->delay > 0.0f) continue;

            int ix = (int)(b->cx + p->rx + 0.5f);
            int iy = (int)(b->cy + p->ry + 0.5f);
            if (ix < 0 || ix >= s->cols)  continue;
            if (iy < 0 || iy >= rows_eff) continue;

            float f = (p->max_life > 0.0f) ? (p->life / p->max_life) : 0.0f;
            if (f < 0.0f) f = 0.0f;
            if (f > 1.0f) f = 1.0f;
            int slot = (int)(f * 7.999f);
            if (slot < 0) slot = 0;
            if (slot > 7) slot = 7;

            int pair = PAIR_RAMP_BASE + slot;
            int attr = (slot >= 6) ? A_BOLD : A_NORMAL;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(iy, ix, (chtype)(unsigned char)p->sym);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* ── Comet heads with halo ───────────────────────────────────── */
    const PatternParams *pp = &pattern_params[s->current_pattern];
    for (int i = 0; i < MAX_COMETS; i++) {
        const Comet *c = &s->comets[i];
        if (!c->active) continue;
        int hx = (int)(c->x + 0.5f);
        int hy = (int)(c->y + 0.5f);

        /* Halo: 3-cell radial dim glow around head. */
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                int hxd = hx + dx;
                int hyd = hy + dy;
                if (hxd < 0 || hxd >= s->cols) continue;
                if (hyd < 0 || hyd >= rows_eff) continue;
                /* Inner ring (4-neighbour) is brighter than corners. */
                int ax = (dx == 0 || dy == 0);
                char hg = ax ? '+' : ':';
                int  hat = ax ? A_BOLD : A_DIM;
                attron(COLOR_PAIR(PAIR_HALO) | hat);
                mvaddch(hyd, hxd, (chtype)(unsigned char)hg);
                attroff(COLOR_PAIR(PAIR_HALO) | hat);
            }
        }

        /* Head itself. */
        if (hx >= 0 && hx < s->cols && hy >= 0 && hy < rows_eff) {
            attron(COLOR_PAIR(PAIR_HEAD) | A_BOLD);
            mvaddch(hy, hx, (chtype)(unsigned char)pp->head_glyph);
            attroff(COLOR_PAIR(PAIR_HEAD) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
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

static void scene_counts(const Scene *s, int *out_comets, int *out_trail)
{
    int c = 0, t = 0;
    for (int i = 0; i < MAX_COMETS; i++) if (s->comets[i].active) c++;
    for (int i = 0; i < MAX_TRAIL;  i++) if (s->trail [i].active) t++;
    *out_comets = c;
    *out_trail  = t;
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int comets, trails;
    scene_counts(s, &comets, &trails);

    const char *state_str = s->paused ? "PAUSED       " : pattern_name(s->current_pattern);

    char buf[200];
    snprintf(buf, sizeof buf,
             " COMET   %s   theme:%-8s   comets:%d  trail:%4d   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name,
             comets, trails, fps, sim_fps, s->speed);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
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
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
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
