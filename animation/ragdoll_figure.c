/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ragdoll_figure.c — Verlet humanoid ragdoll bouncing through slanted shelves
 *
 * DEMO: A 15-particle stick figure falls under gravity and tumbles down five
 *       slanted platforms before settling on the ground. Distance constraints
 *       hold its 17 bones rigid; periodic wind gusts keep the figure lively.
 *
 * Study alongside: snake_inverse_kinematics.c (constraint-projection sibling)
 *
 * Section map:
 *   §1 config   — tunables: physics, gravity, wind, platform layout
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — body-part palette + HUD/hint pairs
 *   §4 coords   — pixel↔cell aspect-ratio bridge
 *   §5 entity   — Ragdoll: Verlet particles + distance constraints
 *       §5a verlet_update             — gravity + wind integration
 *       §5b apply_boundaries          — wall/floor/ceiling bounce
 *       §5b-2 apply_platform_collisions — slanted-shelf reflection
 *       §5c satisfy_constraint        — bone-length projection
 *       §5d ragdoll_tick              — orchestrator (one physics step)
 *       §5e bone_glyph / draw_bone    — ASCII line stamp
 *   §6 scene    — Scene = ragdoll + platforms; tick/draw + render decomposition
 *   §7 screen   — ncurses double-buffer display layer
 *   §8 app      — signals, resize, main game loop
 *
 * Keys:
 *   q / ESC       quit
 *   space         pause
 *   r             reset (figure back to top, new platforms)
 *   w/s, ↑/↓      gravity × 1.3 / ÷ 1.3
 *   a/d, ←/→      wind force ± 20
 *   [ / ]         sim Hz − / +
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       ragdoll_figure.c -o ragdoll_figure -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Position-Verlet integration. Each particle stores its
 *                  current and previous position; velocity is implicit in
 *                  the difference. Gravity and wind accelerate particles;
 *                  8 passes of distance-constraint projection (Jakobsen)
 *                  restore all bone lengths after integration. Boundary
 *                  and slanted-platform collisions are implemented as
 *                  position clamp + old_pos reflection, which naturally
 *                  yields a physical bounce without explicit velocity
 *                  manipulation.
 *
 * Data-structure : Ragdoll holds pos[], old_pos[] (the Verlet pair) and
 *                  prev_pos[] (snapshot for sub-tick alpha lerp).
 *                  Constraint arrays c_a[], c_b[], c_len[] are parallel
 *                  arrays indexed by constraint id. Platforms are kept in
 *                  Scene as Platform[] (cx, y, half_w, slope). All state
 *                  lives in pixel space — square, isotropic — so forces
 *                  and distances are correct regardless of the terminal's
 *                  cell aspect ratio.
 *
 * Rendering      : Each bone is walked in DRAW_STEP_PX increments; the
 *                  direction glyph (-, |, /, \) is stamped at every
 *                  terminal cell the bone crosses. Alpha-interpolated
 *                  prev_pos → pos positions keep motion smooth even when
 *                  render Hz ≠ sim Hz. Platforms render in two passes
 *                  (fill 'o' bead, then '0' nodes at quarter points).
 *
 * Performance    : Fixed-step accumulator decouples physics Hz from render
 *                  Hz. 8 constraint iterations × 17 constraints ≈ 136
 *                  scalar projections per tick — trivial. ncurses
 *                  wnoutrefresh + doupdate emits only changed cells.
 *
 * References     : Jakobsen, "Advanced Character Physics," GDC 2001
 *                    (the canonical Verlet-ragdoll paper).
 *                  Müller et al., "Position Based Dynamics," 2007 — the
 *                    modern generalisation of Jakobsen's projection idea.
 *                  Wikipedia: "Verlet integration" — derivation and the
 *                    velocity-implicit form used here.
 *                  Hecker, "Behind the Screen — Verlet Physics," Game
 *                    Developer Magazine, 2005.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A ragdoll is just a cloud of particles falling under gravity, with rigid
 * bones enforced as a *post-step correction*: integrate first, then project
 * the network back to its rest distances. Repeat the projection enough
 * times and the figure looks rigid; do it too few times and it stretches
 * like rubber. There is no joint solver, no angular velocity, no torque —
 * the rigidity emerges from iterating distance-only constraints.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine 15 marbles connected by 17 inelastic strings, all falling
 * through a stack of tilted glass shelves. Each tick:
 *   • each marble takes one step in the direction it was already moving,
 *     plus a downward kick from gravity (Verlet);
 *   • whenever a string is too long or too short, the two marbles at its
 *     ends move halfway toward each other (or apart) along the string.
 * Run that string-fix step eight times in a row and the figure looks
 * solid. Touch a shelf and the marble *and its previous position* both
 * snap above the surface — the bounce is automatic.
 *
 * DRAWING METHOD / ALGORITHM IN STEPS
 * ───────────────────────────────────
 *   1. Save prev_pos = pos             (anchor for sub-tick interpolation)
 *   2. If a wind period elapsed, kick old_pos sideways (impulse via Verlet
 *      identity: changing old_pos changes implicit velocity).
 *   3. For each particle, Verlet update:
 *        vel     = (pos − old_pos) · DAMPING
 *        old_pos = pos
 *        pos    += vel + accel · dt²
 *   4. Clamp particles to walls/floor/ceiling; reflect old_pos for bounce.
 *   5. Test each particle against each slanted platform; reflect velocity
 *      along the surface normal n = (slope, −1)/||·|| if crossing from above.
 *   6. Repeat N_CONSTRAINT_ITERS times: walk every bone, project both
 *      endpoints by half the length error along the bone direction.
 *      After each iteration re-apply platform collisions so a bone cannot
 *      yank a foot through a shelf.
 *   7. Render: lerp prev_pos → pos by alpha, stamp bone glyphs and joints.
 *
 * KEY FORMULAS
 * ────────────
 *   Verlet integration (position-implicit velocity):
 *     vel     = (pos − old_pos) · DAMPING
 *     old_pos = pos
 *     pos    += vel + accel · dt²
 *
 *   Distance constraint (Jakobsen projection):
 *     v     = p2 − p1
 *     dist  = |v|
 *     error = (dist − rest) / dist
 *     p1   += 0.5 · error · v        (equal mass → half each)
 *     p2   −= 0.5 · error · v
 *
 *   Slanted-surface reflection (screen +y down, upward normal):
 *     n        = (slope, −1) / sqrt(slope² + 1)
 *     v_n      = v · n
 *     v_refl   = v − (1 + BOUNCE) · v_n · n
 *     old_pos  = pos − v_refl
 *
 *   Pixel→cell aspect-ratio fix:
 *     cx = round(px / CELL_W)
 *     cy = round(py / CELL_H)        CELL_W=8, CELL_H=16
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Two coincident particles → distance ≈ 0 → constraint divides by zero.
 *     satisfy_constraint() bails early when dist < 1e-6.
 *   • Order matters: Verlet integration MUST save old_pos before computing
 *     new pos (verlet_update uses a temp). Reverse order destroys velocity.
 *   • Bones can drag a foot through a platform if collisions only run
 *     before constraints — that is why ragdoll_tick re-applies platform
 *     collisions inside the constraint loop.
 *   • Negative slope = right side higher (looks like /), positive = right
 *     side lower (looks like \) — easy to flip mentally.
 *   • Resize: rescaling platforms must preserve relative layout; clamp all
 *     particle positions to new pixel bounds so none get stranded.
 *   • Frame cap: do NOT add dt to elapsed — it cancels the cap. Use
 *     `clock_ns() − frame_start` only.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Drop the figure with wind=0 and gravity=800: it should land in a
 *     limp pile, no limbs longer than they started (constraint solver OK).
 *   • Pause (space): figure freezes mid-fall. Resume: motion continues
 *     without a velocity glitch (because prev_pos is saved every tick).
 *   • Crank gravity to 3200 and bounce off the floor: figure should rebound
 *     to ~30 % of drop height (BOUNCE_COEFF = 0.55, so √0.55² ≈ 0.30).
 *   • Drop on a 0.4-slope platform: ankle slides sideways — proves the
 *     normal-component reflection is working, not just a vertical bounce.
 *   • Reset (r) repeatedly: every reset must produce a clean T-pose at the
 *     top with all 17 bones at rest length (no pop on first frame).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. This file is your introduction to PHYSICS-BASED
 *      animation. Everything before in animation/ was kinematic
 *      (no forces, just geometry). This is forces + integration +
 *      constraint projection.
 *   2. §5 entity — THE HEART. In sub-section order:
 *        §5a verlet_update           ← T2 (the integrator)
 *        §5b apply_boundaries        ← walls / floor / ceiling
 *        §5b-2 platform_collisions   ← T5 (slanted surfaces)
 *        §5c satisfy_constraint      ← T3 (Jakobsen projection)
 *        §5d ragdoll_tick            ← T4 (orchestration order)
 *        §5e bone_glyph / draw_bone  ← rendering
 *      Read AFTER tutorials T1-T6.
 *   3. §6 scene — Ragdoll + platforms.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if you've seen the
 *      framework. Note this file uses fixed-step + alpha lerp.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   pos[i]                  particle i's CURRENT position (Vec2).
 *   old_pos[i]              particle i's PREVIOUS position. Velocity
 *                           is implicit: vel = pos − old_pos.
 *   prev_pos[i]             snapshot at start of sim tick (for
 *                           sub-tick alpha lerp).
 *   c_a[k], c_b[k]          endpoints of constraint k (particle
 *                           indices).
 *   c_len[k]                rest length of constraint k.
 *   N_PARTICLES             15 — head, neck, shoulders, elbows,
 *                           wrists, hips, knees, ankles, ...
 *   N_CONSTRAINTS           17 — the bones connecting them.
 *   N_CONSTRAINT_ITERS      8  — Jakobsen iterations per tick.
 *   DAMPING                 multiplier on implicit velocity each
 *                           tick; <1 = energy loss.
 *
 * Background you need
 * ───────────────────
 *   - Newton's laws — gravity is a constant downward acceleration.
 *   - Numerical integration — converting force into motion.
 *   - Distance: |a − b| = sqrt((a.x − b.x)² + (a.y − b.y)²).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Lagrangian / Hamiltonian mechanics. Verlet is just a
 *     position update.
 *   - Rigid-body dynamics (forces + torques + inertia tensors).
 *     Verlet ragdolls SIMULATE rigid-body behaviour as an
 *     emergent property of constraint iteration.
 *   - Implicit / semi-implicit Euler. Verlet is its own scheme,
 *     simpler than Euler, with better long-term energy behaviour.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Six tutorials that build a Verlet ragdoll from first principles.
 *
 *   T1  The shift from kinematic to physics-based animation
 *   T2  Verlet integration — velocity-implicit position update
 *   T3  Distance constraints — Jakobsen's projection trick
 *   T4  Why iteration order MATTERS for ragdolls
 *   T5  Bouncing off slanted surfaces — reflect, then move old_pos
 *   T6  Why "rigidity" emerges from soft constraints
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE SHIFT FROM KINEMATIC TO PHYSICS-BASED ANIMATION
 * ───────────────────────────────────────────────────────
 * Every previous file in animation/ was KINEMATIC. The animator
 * (you, the user, or a sin/cos formula) commands joint
 * positions; the chain follows by FK or IK. No forces, no mass,
 * no collisions — geometry only.
 *
 * A ragdoll is PHYSICS-BASED. You DON'T command joint positions;
 * you specify FORCES (gravity, wind, contact reactions) and let
 * the simulator compute where the particles go. The pose
 * EMERGES from the integrator — the animator gives up direct
 * control of the figure.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │   kinematic            physics-based             │
 *      │                                                  │
 *      │   command pose      command forces               │
 *      │       ↓                  ↓                       │
 *      │     pose             integrator                  │
 *      │                          ↓                       │
 *      │                      pose emerges                │
 *      └──────────────────────────────────────────────────┘
 *
 * The trade:
 *   ‒ Less control: you can't pose a ragdoll by writing
 *     joint angles.
 *   + More realism: gravity, momentum, and contact "just
 *     work" without per-frame authoring.
 *
 * Ragdolls excel at INVOLUNTARY motion (a knocked-out
 * character collapsing) and fail at GOAL-DIRECTED motion
 * (the same character standing back up). Production engines
 * BLEND animation + ragdoll: keyframe animation when the
 * character is conscious, ragdoll when it's not.
 *
 * T2  VERLET INTEGRATION — VELOCITY-IMPLICIT POSITION UPDATE
 * ──────────────────────────────────────────────────────────
 * The simplest physics integrator is EULER:
 *
 *     vel  += accel · dt
 *     pos  += vel   · dt
 *
 * Two state variables per particle (pos, vel). Drift over time
 * is significant — Euler's energy errors accumulate.
 *
 * VERLET INTEGRATION (Loup Verlet, 1967) replaces velocity with
 * an IMPLICIT velocity stored in the previous position:
 *
 *     vel       = pos − old_pos    (implicit, computed when needed)
 *     old_pos   = pos              (save current as new "old")
 *     pos      += vel + accel · dt²
 *
 * One state variable per particle (pos), with old_pos updated
 * each tick. Equivalent to leapfrog or central-difference
 * integration; better long-term stability than Euler.
 *
 * Why Verlet for a ragdoll? TWO HUGE benefits:
 *
 *   1. CONSTRAINT PROJECTION IS TRIVIAL (T3). When you snap a
 *      particle to a new position to fix a constraint, vel
 *      automatically updates because vel is "pos − old_pos."
 *      With Euler you'd have to recompute vel by hand.
 *
 *   2. NO EXPLICIT VELOCITY MANAGEMENT. Bouncing off a wall is
 *      "set old_pos to where vel should reflect" — no separate
 *      velocity vector to maintain.
 *
 * Damping (energy loss): multiply (pos − old_pos) by DAMPING <
 * 1 each tick. Without damping, Verlet conserves energy and the
 * ragdoll bounces forever; with damping, it eventually settles.
 *
 * T3  DISTANCE CONSTRAINTS — JAKOBSEN'S PROJECTION TRICK
 * ──────────────────────────────────────────────────────
 * Each "bone" of the ragdoll is a fixed-length DISTANCE
 * CONSTRAINT between two particles. After the integration step
 * pushes particles around, bones may have stretched or
 * compressed. Jakobsen's (2001) trick: explicitly snap them
 * back to length.
 *
 *     for each constraint (i, j) with rest length L:
 *       v     = pos[j] − pos[i]
 *       dist  = |v|
 *       error = (dist − L) / dist
 *       pos[i] += 0.5 · error · v        ← move i toward j
 *       pos[j] −= 0.5 · error · v        ← move j toward i
 *
 * "Move both endpoints halfway toward (or away from) each
 * other along their connecting line until the distance is
 * exactly L." For equal masses, move each by half the error.
 *
 * One pass per bone fixes that bone, but typically BREAKS
 * adjacent bones (which shared one endpoint). Repeat the whole
 * pass enough times and the system converges.
 *
 * Convergence rate: for small graphs (15 particles, 17
 * constraints) ~8 iterations per tick gives visually rigid
 * limbs. More iterations = stiffer body (and slower); fewer =
 * stretchy / rubbery.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │  before                  after constraint pass   │
 *      │                                                  │
 *      │  A───────B               A─────B                 │
 *      │   ╲       ╲                                      │
 *      │    ╲       ●  C  →   pulls B toward A,           │
 *      │     ╲                pulls A toward B            │
 *      │      D               until |A−B| = L_AB          │
 *      │                                                  │
 *      │  but now B is too close to C!  Iterate again.    │
 *      └──────────────────────────────────────────────────┘
 *
 * T4  WHY ITERATION ORDER MATTERS FOR RAGDOLLS
 * ────────────────────────────────────────────
 * The ragdoll tick has FOUR stages, run in this order:
 *
 *   1. INTEGRATE       Verlet step on every particle.
 *   2. COLLIDE WORLD   clamp to walls; bounce off platforms.
 *   3. SATISFY (×8)    Jakobsen projection on every bone.
 *   4. COLLIDE WORLD   AGAIN — inside the satisfy loop.
 *
 * Why collide AGAIN inside the satisfy loop? Because step 3
 * (constraint projection) can YANK A FOOT BACK THROUGH A
 * PLATFORM. The bone says "ankle must be exactly L from
 * knee," and pulling the knee out of a slanted shelf would
 * shorten the leg.
 *
 * Solution: every constraint iteration, re-collide all
 * particles with the world. The satisfy step then has to
 * accept the world's hard surface BEFORE the next bone tries
 * to pull through it.
 *
 * If you skip the inner-loop collide, the ragdoll's feet will
 * occasionally pop through platforms — the bone yank wins
 * over a single boundary clamp.
 *
 * T5  BOUNCING OFF SLANTED SURFACES — REFLECT, THEN MOVE old_pos
 * ──────────────────────────────────────────────────────────────
 * A horizontal floor bounce is easy: clamp pos.y above the
 * floor, set old_pos.y = floor (reverses implicit velocity).
 * A slanted platform needs vector reflection.
 *
 * Given a platform with slope k:
 *
 *     surface = (1, k)
 *     normal n = (slope, −1) / sqrt(slope² + 1)
 *
 * (We negate the y-component because screen y points down;
 * the "up" normal is the one with negative y.)
 *
 * For each particle that just crossed the platform:
 *
 *     v       = pos − old_pos
 *     v_n     = v · n                 ← component along normal
 *     v_refl  = v − (1 + BOUNCE) · v_n · n
 *     old_pos = pos − v_refl          ← put old_pos to make
 *                                       implicit vel = v_refl
 *
 * Reflecting v through n removes the component into the
 * surface and reverses it. BOUNCE ∈ [0, 1] is the coefficient
 * of restitution: 1 = perfect bounce, 0 = absorbed completely
 * (no rebound — just slides along). 0.55 is "rubbery."
 *
 * Setting old_pos = pos − v_refl is the Verlet way to assign
 * a new velocity. No explicit vel variable to update.
 *
 * Same recipe works for any surface — circular, polygonal,
 * SDF — as long as you can compute the surface normal.
 *
 * T6  WHY "RIGIDITY" EMERGES FROM SOFT CONSTRAINTS
 * ────────────────────────────────────────────────
 * A real human bone is RIGID — its length CANNOT change. Our
 * constraints are SOFT — each iteration only NUDGES particles
 * toward the rest length. With ∞ iterations, the system
 * converges to exact rigidity; with FINITE iterations, the
 * bones can stretch or compress slightly.
 *
 * For 8 iterations and typical drop velocities (gravity 800,
 * dt 16 ms), bones stretch by < 1 % — visually invisible.
 * For dramatically high gravity or heavy impacts, you might
 * see bones briefly elongate at impact frames; cranking
 * N_CONSTRAINT_ITERS to 16 or 32 fixes that at a small CPU
 * cost.
 *
 * Why use SOFT constraints when hard ones would be more
 * accurate? Because hard constraints require solving a linear
 * system (matrix inversion) that gets ugly fast. The Jakobsen
 * iterative trick produces VISUALLY indistinguishable
 * rigidity at a fraction of the complexity, and it scales
 * trivially to thousands of particles + constraints.
 *
 * Production physics engines (Box2D, Bullet, Havok) use
 * SEQUENTIAL IMPULSES — a velocity-based variant of the same
 * idea: iteratively apply impulses to correct constraint
 * violations. The mathematical core is identical; the
 * representation differs.
 *
 * For this terminal demo (15 particles, 17 bones), Jakobsen's
 * position-based projection is simpler, cleaner, and looks
 * indistinguishable from the impulse approach.
 *
 * Same machinery is reused in ragdoll_ropes.c for cloth-like
 * sheets — same Verlet + same Jakobsen, just thousands of
 * particles instead of fifteen.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/*
 * M_PI is a POSIX extension, not standard C99/C11.
 * _POSIX_C_SOURCE 200809L exposes it on most systems, but if a toolchain
 * omits it we define our own constant so the build never fails.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/*
 * All magic numbers live here.  Change behaviour by editing this block
 * only — never scatter literals through the code.
 */
enum {
    /*
     * SIM_FPS_DEFAULT — target physics tick rate in Hz.
     * The accumulator loop in §8 fires scene_tick() this many times
     * per wall-clock second regardless of render frame rate.
     */
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    /*
     * HUD_COLS — byte budget for the status-bar string.
     * 96 bytes covers the longest possible HUD at max values.
     *
     * FPS_UPDATE_MS — how often the displayed fps is recalculated.
     */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    /*
     * Color-pair IDs.  Pairs 1-7 paint the body; PAIR_HUD/PAIR_HINT paint
     * the two HUD lines per the project HUD spec.
     */
    PAIR_HEAD        =   1,   /* white   — head marker             */
    PAIR_BODY        =   2,   /* white   — non-head joints         */
    PAIR_SPINE       =   3,   /* grey    — spine + collarbones     */
    PAIR_ARM         =   4,   /* orange  — arm bones + wrists      */
    PAIR_LEG         =   5,   /* blue    — leg bones + ankles      */
    PAIR_PLATFORM    =   6,   /* cyan    — slanted shelves         */
    PAIR_STRUT       =   7,   /* dim grey — stabiliser struts/floor*/
    PAIR_HUD         =   8,   /* bright yellow — top status        */
    PAIR_HINT        =   9,   /* bright cyan   — bottom key hints  */

    /*
     * Ragdoll topology constants.
     *
     * N_PARTICLES — 15 joints representing the humanoid skeleton:
     *   0=head, 1=neck, 2=left_shoulder, 3=right_shoulder,
     *   4=left_elbow, 5=right_elbow, 6=left_wrist, 7=right_wrist,
     *   8=hip_center, 9=left_hip, 10=right_hip,
     *   11=left_knee, 12=right_knee, 13=left_ankle, 14=right_ankle
     *
     * N_CONSTRAINTS — 17 distance constraints:
     *   spine(2) + arms(4) + legs(4) + hip-cross(2) + collarbones(2) +
     *   shoulder-width(1) + hip-width(1) + head-shoulder(2) = 17
     *
     * N_CONSTRAINT_ITERS — how many full passes over all constraints per
     *   tick.  8 is sufficient for a 15-particle body at 60 Hz.
     */
    N_PARTICLES        = 15,
    N_CONSTRAINTS      = 17,
    N_CONSTRAINT_ITERS =  8,
    N_PLATFORMS        =  5,   /* staggered platforms the ragdoll falls through */
};

/*
 * Physics constants.
 *
 * GRAVITY      — downward acceleration in px/s² (+y is down in screen space).
 * DAMPING      — per-tick velocity damping (multiply implicit vel each tick).
 *                0.995 ≈ 0.5% energy loss per tick = very slight air drag.
 * BOUNCE_COEFF — fraction of velocity preserved after a floor collision.
 *                0.55 gives a lively but controllable bounce.
 * WIND_PERIOD  — seconds between random lateral impulses.
 * WIND_FORCE   — magnitude of each wind impulse in px/s (Verlet velocity units).
 *
 * Boundary margins in pixel space:
 *   FLOOR_MARGIN — how far above the bottom pixel row the floor is placed.
 *   CEIL_MARGIN  — how far below the top pixel row the ceiling is.
 *   LEFT_MARGIN  — how far inside the left edge the left wall is.
 *   RIGHT_MARGIN — how far inside the right edge the right wall is.
 */
#define GRAVITY       800.0f
#define DAMPING         0.995f
#define BOUNCE_COEFF    0.55f
#define WIND_PERIOD     3.5f
#define WIND_FORCE    120.0f

#define FLOOR_MARGIN    8.0f
#define CEIL_MARGIN    16.0f
#define LEFT_MARGIN    16.0f
#define RIGHT_MARGIN   16.0f

/*
 * DRAW_STEP_PX — pixel step size for dense bone rendering.
 * Must be < CELL_W (8 px) so no terminal cell is skipped along a bone.
 */
#define DRAW_STEP_PX    2.0f

/*
 * Timing primitives — verbatim from framework.c.
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Terminal cell dimensions — the aspect-ratio bridge between physics
 * and display (see §4 for full explanation).
 */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 * CLOCK_MONOTONIC never goes backward; safe for dt measurements.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep for exactly ns nanoseconds.
 * If ns ≤ 0 (frame over budget) returns immediately without sleeping.
 */
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
/* §3  color — body-part palette + dedicated HUD pair                    */
/* ===================================================================== */

/*
 * color_init() — define all ncurses color pairs.
 *
 * Background = -1 (terminal default) so the demo respects the user's
 * theme.  Foreground tints are chosen from the bright half of the
 * 256-colour cube (≥ 24) so every glyph remains legible even under
 * A_DIM (see CLAUDE.md "Theme Palette Brightness").
 *
 *   Pair  256-col  Role
 *   ─────────────────────────────────────────────────
 *     1     231    bright white  — head marker
 *     2     255    near-white    — generic joint dot
 *     3     248    light grey    — spine + collarbones
 *     4     214    orange        — arm bones, wrists
 *     5      75    sky blue      — leg bones, ankles
 *     6      45    cyan          — slanted platform beads
 *     7     244    medium grey   — stabiliser struts + ground line
 *     8     226    bright yellow — HUD top status (PAIR_HUD)
 *     9      51    bright cyan   — HUD bottom key hint (PAIR_HINT)
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(PAIR_HEAD,     231, -1);
        init_pair(PAIR_BODY,     255, -1);
        init_pair(PAIR_SPINE,    248, -1);
        init_pair(PAIR_ARM,      214, -1);
        init_pair(PAIR_LEG,       75, -1);
        init_pair(PAIR_PLATFORM,  45, -1);
        init_pair(PAIR_STRUT,    244, -1);
        init_pair(PAIR_HUD,      226, -1);
        init_pair(PAIR_HINT,      51, -1);
    } else {
        init_pair(PAIR_HEAD,     COLOR_WHITE,   -1);
        init_pair(PAIR_BODY,     COLOR_WHITE,   -1);
        init_pair(PAIR_SPINE,    COLOR_WHITE,   -1);
        init_pair(PAIR_ARM,      COLOR_YELLOW,  -1);
        init_pair(PAIR_LEG,      COLOR_CYAN,    -1);
        init_pair(PAIR_PLATFORM, COLOR_CYAN,    -1);
        init_pair(PAIR_STRUT,    COLOR_WHITE,   -1);
        init_pair(PAIR_HUD,      COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,     COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square (CELL_H=16 vs CELL_W=8).  All particle
 * positions are in pixel space (square, isotropic).  Only at draw time
 * are pixel coordinates converted to cell coordinates.
 *
 * px_to_cell_x / px_to_cell_y — round to nearest cell.
 * Formula: cell = floor(px / CELL_DIM + 0.5)  ("round half up")
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Ragdoll: Verlet particles + distance constraints          */
/* ===================================================================== */

/*
 * Vec2 — 2-D position vector in pixel space.
 * x increases eastward; y increases downward (terminal convention).
 */
typedef struct { float x, y; } Vec2;

/*
 * Ragdoll — complete simulation state for the humanoid figure.
 *
 * VERLET PAIR (pos[], old_pos[]):
 *   pos[]      Current particle positions.
 *   old_pos[]  Positions from the previous tick.
 *              Velocity is implicit: vel = pos - old_pos.
 *              Modifying old_pos without touching pos changes velocity.
 *              This is how Verlet bounce works: reflecting old_pos across
 *              a wall reverses the velocity component perpendicular to
 *              that wall.
 *
 * INTERPOLATION SNAPSHOT (prev_pos[]):
 *   Copied from pos[] at the start of each tick (before physics).
 *   render_ragdoll() lerps between prev_pos and pos using alpha ∈ [0,1)
 *   so motion appears smooth at any combination of sim Hz and render Hz.
 *
 * CONSTRAINT ARRAYS (c_a[], c_b[], c_len[]):
 *   Parallel arrays; constraint i connects particle c_a[i] to c_b[i]
 *   with rest length c_len[i] (computed from the initial T-pose spacing).
 *
 * wind_timer: seconds since last wind gust.
 * wind_x:     current gust x-acceleration component (px/s²). Applied
 *             as an impulse to old_pos at gust time; fades via DAMPING.
 */
typedef struct {
    Vec2  pos[N_PARTICLES];
    Vec2  old_pos[N_PARTICLES];
    Vec2  prev_pos[N_PARTICLES];

    int   c_a[N_CONSTRAINTS];
    int   c_b[N_CONSTRAINTS];
    float c_len[N_CONSTRAINTS];

    float wind_timer;
    float wind_x;

    float gravity;        /* adjustable at runtime via ↑↓ keys        */
    float wind_force;     /* adjustable at runtime via ←→ keys        */

    bool  paused;
} Ragdoll;

/* ── §5a  verlet_update ─────────────────────────────────────────────── */

/*
 * verlet_update() — apply one Verlet integration step to particle i.
 *
 * Classic Verlet (Störmer-Verlet) with per-tick velocity damping:
 *
 *   vel     = (pos - old_pos) * DAMPING    ← implicit velocity, damped
 *   new_pos = pos + vel + accel * dt²
 *   old_pos = pos
 *   pos     = new_pos
 *
 * Expanding:
 *   new_pos = pos + (pos - old_pos)*DAMPING + accel * dt * dt
 *
 * WHY THIS ORDER?
 *   Saving old_pos *before* computing new_pos is essential; otherwise we
 *   overwrite old_pos with the value we need for the velocity calculation.
 *   Using a temporary variable (new_pos) avoids the aliasing issue.
 *
 * accel = (wind_x, gravity_y) — both in px/s²
 *   gravity_y is positive because +y is downward in pixel space.
 */
static void verlet_update(Ragdoll *r, int i, float dt)
{
    float dt2 = dt * dt;

    Vec2 old = r->old_pos[i];
    Vec2 cur = r->pos[i];

    float vel_x = (cur.x - old.x) * DAMPING;
    float vel_y = (cur.y - old.y) * DAMPING;

    r->old_pos[i] = cur;
    r->pos[i].x   = cur.x + vel_x + r->wind_x  * dt2;
    r->pos[i].y   = cur.y + vel_y + r->gravity  * dt2;
}

/* ── §5b  apply_boundaries ──────────────────────────────────────────── */

/*
 * apply_boundaries() — clamp particle i to screen bounds and bounce.
 *
 * Collision in Verlet integration works by clamping pos to the boundary
 * then correcting old_pos to reflect the implicit velocity:
 *
 *   Floor bounce (y too large):
 *     1. Clamp: pos.y = floor_y
 *     2. Reflect: old_pos.y = pos.y + (pos.y - old_pos.y) * BOUNCE_COEFF
 *        • (pos.y - old_pos.y) is the y-velocity component this tick.
 *        • Multiplying by BOUNCE_COEFF scales it (energy loss).
 *        • Adding to the clamped pos.y puts old_pos ABOVE the floor,
 *          so next tick the particle moves upward — the bounce.
 *
 *   Wall clamping (x too large or too small):
 *     Only pos.x is clamped; old_pos.x is also corrected so the x-velocity
 *     reverses direction (elastic bounce off side walls).
 *
 *   Ceiling clamping:
 *     No bounce from the ceiling — just a hard stop.  The figure rarely
 *     reaches the ceiling; if it does, a soft stop prevents tunnelling.
 */
static void apply_boundaries(Ragdoll *r, int i, int cols, int rows)
{
    float floor_y = (float)(rows * CELL_H) - FLOOR_MARGIN;
    float ceil_y  = CEIL_MARGIN;
    float left_x  = LEFT_MARGIN;
    float right_x = (float)(cols * CELL_W) - RIGHT_MARGIN;

    /* Floor — bounce */
    if (r->pos[i].y > floor_y) {
        r->pos[i].y   = floor_y;
        r->old_pos[i].y = r->pos[i].y
                        + (r->pos[i].y - r->old_pos[i].y) * BOUNCE_COEFF;
    }

    /* Ceiling — hard stop */
    if (r->pos[i].y < ceil_y) {
        r->pos[i].y   = ceil_y;
        r->old_pos[i].y = ceil_y;
    }

    /* Left wall — bounce */
    if (r->pos[i].x < left_x) {
        r->pos[i].x   = left_x;
        r->old_pos[i].x = r->pos[i].x
                        + (r->pos[i].x - r->old_pos[i].x) * BOUNCE_COEFF;
    }

    /* Right wall — bounce */
    if (r->pos[i].x > right_x) {
        r->pos[i].x   = right_x;
        r->old_pos[i].x = r->pos[i].x
                        + (r->pos[i].x - r->old_pos[i].x) * BOUNCE_COEFF;
    }
}

/* ── §5b-2  platform collision ─────────────────────────────────────── */

/*
 * Platform — a slanted shelf in pixel space.
 *   cx     : centre x
 *   y      : surface y at cx (midpoint)
 *   half_w : half-width along the x axis
 *   slope  : dy/dx — rise per pixel rightward in screen coords (+y down)
 *            positive slope → right side lower (\), negative → right side higher (/)
 *
 * Surface y at any x:  surf_y(x) = y + (x − cx) * slope
 */
typedef struct { float cx, y, half_w, slope; } Platform;

/*
 * apply_platform_collisions() — bounce particle i off any slanted platform.
 *
 * Detection:
 *   Surface y at particle x:  surf_y = pl.y + (pos.x − pl.cx) * slope
 *   Particle crossed from above: old_pos.y <= old_surf_y  AND  pos.y >= surf_y
 *   X within platform extent:   |pos.x − pl.cx| <= pl.half_w
 *
 * Response — slanted-surface Verlet reflection:
 *   The upward surface normal in screen coords (+y down) is:
 *     n = (slope, -1) / sqrt(slope² + 1)
 *   Decompose velocity v = pos − old_pos along n and tangent t:
 *     v_n = dot(v, n)          (negative = moving into surface)
 *     v_reflected = v − (1 + BOUNCE_COEFF) * v_n * n
 *   Set  old_pos = pos − v_reflected  so next tick sees the reflected vel.
 *   A slanted surface transfers some normal momentum to horizontal,
 *   naturally deflecting the ragdoll sideways as it bounces.
 */
static void apply_platform_collisions(Ragdoll *r, int i,
                                      const Platform *plats, int n)
{
    for (int p = 0; p < n; p++) {
        const Platform *pl = &plats[p];

        /* X range check */
        float xoff = r->pos[i].x - pl->cx;
        if (fabsf(xoff) > pl->half_w) continue;

        /* Surface y at current and previous x */
        float surf_y     = pl->y + xoff * pl->slope;
        float old_xoff   = r->old_pos[i].x - pl->cx;
        float old_surf_y = pl->y + old_xoff * pl->slope;

        /* Crossed from above? */
        if (r->old_pos[i].y > old_surf_y || r->pos[i].y < surf_y) continue;

        /* Save velocity before snap (using original pos.y) */
        float vx = r->pos[i].x - r->old_pos[i].x;
        float vy = r->pos[i].y - r->old_pos[i].y;

        /* Snap to surface */
        r->pos[i].y = surf_y;

        /* Upward normal: (slope, -1) / ||(slope,-1)||
         * (In screen +y-down coords, -1 in y = pointing upward) */
        float s    = pl->slope;
        float nlen = sqrtf(s*s + 1.0f);
        float nx   =  s / nlen;
        float ny   = -1.0f / nlen;

        /* Normal component of velocity (negative = into surface) */
        float vn = vx*nx + vy*ny;
        if (vn >= 0.0f) continue;   /* already moving away — no bounce */

        /* Reflect: old_pos = pos − v_reflected */
        float factor = (1.0f + BOUNCE_COEFF) * vn;
        r->old_pos[i].x = r->pos[i].x - (vx - factor * nx);
        r->old_pos[i].y = r->pos[i].y - (vy - factor * ny);
    }
}

/* ── §5c  satisfy_constraint ────────────────────────────────────────── */

/*
 * satisfy_constraint() — project one bone to its rest length.
 *
 * Given particles at positions p1, p2 and desired distance rest_len:
 *   vec   = p2.pos - p1.pos            ← vector from p1 to p2
 *   dist  = |vec|
 *   error = (dist - rest_len) / dist   ← fractional stretch/compression
 *
 * If dist is nearly zero (two particles on top of each other) the
 * normalisation would produce infinity, so we bail early.
 *
 * Equal-mass correction: each particle moves half the error:
 *   p1.pos += 0.5 * error * vec        ← moves toward p2 if stretched
 *   p2.pos -= 0.5 * error * vec        ← moves toward p1 if stretched
 *
 * Iterating this over all constraints N_CONSTRAINT_ITERS times converges
 * the system: each pass reduces residual stretch by roughly half.
 */
static void satisfy_constraint(Ragdoll *r, int ci)
{
    int a = r->c_a[ci];
    int b = r->c_b[ci];

    float dx   = r->pos[b].x - r->pos[a].x;
    float dy   = r->pos[b].y - r->pos[a].y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < 1e-6f) return;   /* degenerate — particles coincide */

    float error = (dist - r->c_len[ci]) / dist;
    float cx    = 0.5f * error * dx;
    float cy    = 0.5f * error * dy;

    r->pos[a].x += cx;
    r->pos[a].y += cy;
    r->pos[b].x -= cx;
    r->pos[b].y -= cy;
}

/* ── §5d  ragdoll_tick ──────────────────────────────────────────────── */

/*
 * ragdoll_tick() — one complete physics step.
 *
 * ORDER IS CRITICAL:
 *   1. Save prev_pos  — interpolation anchor for render_ragdoll().
 *      Must be saved before any physics modifies pos[].
 *   2. Return if paused — prev_pos is saved so freeze is clean.
 *   3. Wind impulse check — accumulate time; fire a random impulse
 *      by nudging old_pos (equivalent to a velocity kick in Verlet).
 *   4. Verlet update — integrate gravity + wind into all positions.
 *   5. Boundary collision — clamp + bounce (must run after Verlet so
 *      positions are at the new location before correction).
 *   6. Constraint iterations — restore all bone lengths.
 *      Running constraints AFTER collision prevents bones from
 *      tunnelling through the floor: collision moves the ankle to the
 *      floor, and the constraint then pulls the knee to the right height.
 */
static void ragdoll_tick(Ragdoll *r, float dt, int cols, int rows,
                         const Platform *plats)
{
    /* Step 1 — save snapshot for alpha lerp */
    memcpy(r->prev_pos, r->pos, sizeof r->pos);

    /* Step 2 — skip physics if paused */
    if (r->paused) return;

    /* Step 3 — wind gust */
    r->wind_timer += dt;
    if (r->wind_timer >= WIND_PERIOD) {
        r->wind_timer = 0.0f;
        /* Random horizontal impulse: nudge old_pos to inject velocity */
        float dir      = (rand() % 2 == 0) ? 1.0f : -1.0f;
        float strength = r->wind_force * (0.5f + (float)(rand() % 100) / 200.0f);
        float impulse  = dir * strength;
        for (int i = 0; i < N_PARTICLES; i++) {
            r->old_pos[i].x -= impulse * dt;
        }
    }

    /* Step 4 — Verlet integration (gravity + wind) */
    for (int i = 0; i < N_PARTICLES; i++) {
        verlet_update(r, i, dt);
    }

    /* Step 5 — boundary collisions */
    for (int i = 0; i < N_PARTICLES; i++) {
        apply_boundaries(r, i, cols, rows);
    }

    /* Step 5b — platform collisions (after Verlet, before constraints) */
    for (int i = 0; i < N_PARTICLES; i++) {
        apply_platform_collisions(r, i, plats, N_PLATFORMS);
    }

    /* Step 6 — constraint satisfaction (multiple passes for stability) */
    for (int iter = 0; iter < N_CONSTRAINT_ITERS; iter++) {
        for (int ci = 0; ci < N_CONSTRAINTS; ci++) {
            satisfy_constraint(r, ci);
        }
        /* Re-enforce platform collisions inside constraint loop so bones
         * do not push particles through a platform surface */
        for (int i = 0; i < N_PARTICLES; i++) {
            apply_platform_collisions(r, i, plats, N_PLATFORMS);
        }
    }
}

/* ── §5e  ragdoll_draw ──────────────────────────────────────────────── */

/*
 * bone_glyph() — ASCII character that best represents a bone's direction.
 *
 * Same logic as seg_glyph() in snake_forward_kinematics.c.
 * Maps (dx, dy) angle to one of '-', '/', '|', '\'.
 * dy is negated before atan2f to convert terminal-down to math-up.
 */
static chtype bone_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg < 22.5f || deg >= 157.5f) return (chtype)'-';
    if (deg < 67.5f)                   return (chtype)'\\';
    if (deg < 112.5f)                  return (chtype)'|';
    return                             (chtype)'/';
}

/*
 * draw_bone() — stamp glyphs along one bone from pixel point a to b.
 *
 * Walks the bone in DRAW_STEP_PX increments, converting each sample to a
 * terminal cell and stamping a direction glyph.  Deduplication via
 * *prev_cx / *prev_cy avoids double-drawing within one bone and at the
 * joint between adjacent bones (shared endpoint, same cell).
 * Out-of-bounds cells are silently skipped.
 */
static void draw_bone(WINDOW *w,
                      Vec2 a, Vec2 b,
                      int pair, attr_t attr,
                      int cols, int rows,
                      int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    chtype glyph  = bone_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx;
        *prev_cy = cy;

        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/*
 * mark_cell() — stamp one ASCII glyph at terminal cell (cx,cy).
 *
 * Centralises the (chtype)(unsigned char) cast plus bounds-check that would
 * otherwise be repeated at every mvwaddch site.  Glyph is silently dropped
 * if the cell is off-screen.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * lerp_positions() — fill rp[] with alpha-interpolated render positions.
 *
 * rp[i] = prev_pos[i] + (pos[i] − prev_pos[i]) · alpha
 *
 * alpha ∈ [0,1) is the leftover fraction in the fixed-step accumulator.
 * Without this lerp, motion stutters whenever render Hz ≠ sim Hz; with
 * it, the figure glides smoothly at any combination of the two rates.
 */
static void lerp_positions(const Ragdoll *r, float alpha, Vec2 rp[N_PARTICLES])
{
    for (int i = 0; i < N_PARTICLES; i++) {
        rp[i].x = r->prev_pos[i].x + (r->pos[i].x - r->prev_pos[i].x) * alpha;
        rp[i].y = r->prev_pos[i].y + (r->pos[i].y - r->prev_pos[i].y) * alpha;
    }
}

/*
 * draw_platforms() — render the slanted shelves in two passes per platform.
 *
 * Pass 1 fills every cell along the surface with 'o' (the bead).
 * Pass 2 over-stamps quarter-point '0' nodes (A_BOLD on ends + centre),
 * giving each shelf a clear visual anchor without losing the smooth line.
 */
static void draw_platforms(WINDOW *w, const Platform *plats,
                           int cols, int rows)
{
    static const float  node_u   [5] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
    static const char   node_ch  [5] = { '0',  'o',   '0',  'o',   '0'  };
    static const attr_t node_attr[5] = { A_BOLD, A_NORMAL, A_BOLD, A_NORMAL, A_BOLD };

    for (int p = 0; p < N_PLATFORMS; p++) {
        const Platform *pl = &plats[p];
        float x0 = pl->cx - pl->half_w;
        float y0 = pl->y  + (-pl->half_w) * pl->slope;
        float x1 = pl->cx + pl->half_w;
        float y1 = pl->y  + ( pl->half_w) * pl->slope;
        float dx = x1 - x0, dy = y1 - y0;
        float L  = sqrtf(dx*dx + dy*dy);
        if (L < 0.1f) continue;

        /* Pass 1 — bead fill */
        int steps = (int)ceilf(L / 5.0f) + 1;
        int prev_cx = -9999, prev_cy = -9999;
        for (int s = 0; s <= steps; s++) {
            float u  = (float)s / (float)steps;
            int   cx = px_to_cell_x(x0 + dx * u);
            int   cy = px_to_cell_y(y0 + dy * u);
            if (cx == prev_cx && cy == prev_cy) continue;
            prev_cx = cx; prev_cy = cy;
            mark_cell(w, cx, cy, 'o', PAIR_PLATFORM, A_NORMAL, cols, rows);
        }

        /* Pass 2 — quarter-point nodes */
        for (int n = 0; n < 5; n++) {
            int cx = px_to_cell_x(x0 + dx * node_u[n]);
            int cy = px_to_cell_y(y0 + dy * node_u[n]);
            mark_cell(w, cx, cy, node_ch[n],
                      PAIR_PLATFORM, node_attr[n], cols, rows);
        }
    }
}

/*
 * draw_ground() — single dashed row at the floor for visual reference.
 */
static void draw_ground(WINDOW *w, int cols, int rows)
{
    int floor_row = px_to_cell_y((float)(rows * CELL_H) - FLOOR_MARGIN);
    if (floor_row < 0 || floor_row >= rows) return;
    for (int cx = 0; cx < cols; cx++) {
        mark_cell(w, cx, floor_row, '-', PAIR_STRUT, A_DIM, cols, rows);
    }
}

/*
 * draw_bones() — render every distance constraint as a glyph-stamped line.
 *
 * Bone palette is keyed by constraint index ranges:
 *   0–1   spine (head→neck, neck→hip)            PAIR_SPINE A_BOLD
 *   2–3   collarbones (neck→shoulders)           PAIR_SPINE A_NORMAL
 *   4–7   arms (shoulders→elbows→wrists)         PAIR_ARM   A_NORMAL
 *   8–9   hip cross (hip_center→hips)            PAIR_SPINE A_NORMAL
 *   10–13 legs (hips→knees→ankles)               PAIR_LEG   A_NORMAL
 *   14–16 stabilisers (shoulder-W, hip-W, strut) PAIR_STRUT A_DIM
 *
 * The pcx/pcy de-dup state is reset per bone so adjacent bones don't
 * suppress each other's first cell.
 */
static void draw_bones(WINDOW *w, const Ragdoll *r,
                       const Vec2 rp[N_PARTICLES], int cols, int rows)
{
    static const int bone_pair[N_CONSTRAINTS] = {
        PAIR_SPINE, PAIR_SPINE,                                   /*  0–1 */
        PAIR_SPINE, PAIR_SPINE,                                   /*  2–3 */
        PAIR_ARM,   PAIR_ARM,   PAIR_ARM,   PAIR_ARM,             /*  4–7 */
        PAIR_SPINE, PAIR_SPINE,                                   /*  8–9 */
        PAIR_LEG,   PAIR_LEG,   PAIR_LEG,   PAIR_LEG,             /* 10–13 */
        PAIR_STRUT, PAIR_STRUT, PAIR_STRUT,                       /* 14–16 */
    };
    static const attr_t bone_attr[N_CONSTRAINTS] = {
        A_BOLD,   A_BOLD,
        A_NORMAL, A_NORMAL,
        A_NORMAL, A_NORMAL, A_NORMAL, A_NORMAL,
        A_NORMAL, A_NORMAL,
        A_NORMAL, A_NORMAL, A_NORMAL, A_NORMAL,
        A_DIM,    A_DIM,    A_DIM,
    };

    for (int ci = 0; ci < N_CONSTRAINTS; ci++) {
        int pcx = -9999, pcy = -9999;
        draw_bone(w,
                  rp[r->c_a[ci]], rp[r->c_b[ci]],
                  bone_pair[ci], bone_attr[ci],
                  cols, rows,
                  &pcx, &pcy);
    }
}

/*
 * draw_particles() — over-stamp joint markers on top of bones.
 *
 * Glyph distinguishes role: 'O' head (A_BOLD), '*' wrists, 'v' ankles,
 * '.' all other joints (dim).  Drawn last so markers always read above
 * the bone lines that meet at them.
 */
static void draw_particles(WINDOW *w, const Vec2 rp[N_PARTICLES],
                           int cols, int rows)
{
    for (int i = 0; i < N_PARTICLES; i++) {
        int cx = px_to_cell_x(rp[i].x);
        int cy = px_to_cell_y(rp[i].y);

        char   ch    = '.';
        int    pair  = PAIR_BODY;
        attr_t attr  = A_DIM;

        if (i == 0)                  { ch = 'O'; pair = PAIR_HEAD; attr = A_BOLD;   }
        else if (i == 6 || i == 7)   { ch = '*'; pair = PAIR_ARM;  attr = A_NORMAL; }
        else if (i == 13 || i == 14) { ch = 'v'; pair = PAIR_LEG;  attr = A_NORMAL; }

        mark_cell(w, cx, cy, ch, pair, attr, cols, rows);
    }
}

/*
 * render_ragdoll() — orchestrate one frame in painter's order.
 *
 *   1. lerp_positions  — sub-tick interpolation
 *   2. draw_platforms  — slanted shelves first (bottom layer)
 *   3. draw_ground     — floor reference line
 *   4. draw_bones      — every distance constraint stamped as glyph line
 *   5. draw_particles  — joint markers on top
 *
 * Order matters: each later layer over-stamps earlier ones at shared cells,
 * so joints read above bones, bones above ground, etc.
 */
static void render_ragdoll(const Ragdoll *r, WINDOW *w,
                           int cols, int rows, float alpha,
                           const Platform *plats)
{
    Vec2 rp[N_PARTICLES];
    lerp_positions(r, alpha, rp);

    draw_platforms (w, plats,    cols, rows);
    draw_ground    (w,           cols, rows);
    draw_bones     (w, r, rp,    cols, rows);
    draw_particles (w, rp,       cols, rows);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Ragdoll  ragdoll;
    Platform platforms[N_PLATFORMS];
} Scene;

/*
 * init_platforms() — place N_PLATFORMS shelves staggered across the screen.
 *
 * Y positions evenly spaced from ~18% to ~78% of screen height.
 * X positions alternate left-center-right so the ragdoll zig-zags down.
 * Each platform is about 36% of the screen width.
 */
static void init_platforms(Platform *plats, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    float hw   = wpx * 0.18f;   /* half-width: 36% of screen total */

    /* Staggered y, alternating x, alternating slope direction.
     * slope in pixel space: positive = right side lower (\), negative = right higher (/)
     * Using ±0.35 gives visible tilt and meaningful sideways deflection on bounce. */
    static const float yfrac [N_PLATFORMS] = { 0.18f,  0.32f,  0.47f,  0.62f,  0.77f };
    static const float xfrac [N_PLATFORMS] = { 0.50f,  0.25f,  0.72f,  0.35f,  0.65f };
    static const float slopes[N_PLATFORMS] = { -0.35f,  0.38f, -0.40f,  0.32f, -0.36f };

    for (int i = 0; i < N_PLATFORMS; i++) {
        plats[i].y      = hpx * yfrac[i];
        plats[i].cx     = wpx * xfrac[i];
        plats[i].half_w = hw;
        plats[i].slope  = slopes[i];
    }
}

/*
 * add_constraint() — helper to register one distance constraint.
 *
 * The rest length is computed from the current (initial) positions of
 * the two particles so bones are exactly the initial spacing long.
 * This means the rest pose IS the initial T-pose — no separate constant
 * table needed.
 */
static void add_constraint(Ragdoll *r, int *nc, int a, int b)
{
    if (*nc >= N_CONSTRAINTS) return;
    r->c_a[*nc] = a;
    r->c_b[*nc] = b;
    float dx = r->pos[b].x - r->pos[a].x;
    float dy = r->pos[b].y - r->pos[a].y;
    r->c_len[*nc] = sqrtf(dx * dx + dy * dy);
    (*nc)++;
}

/*
 * scene_init() — place the ragdoll in a T-pose at the screen centre.
 *
 * PARTICLE LAYOUT (pixel offsets from screen centre):
 *
 *   Part            Index  dx    dy   Description
 *   ─────────────────────────────────────────────────
 *   head               0    0   -96   32px above neck
 *   neck               1    0   -64   32px above hip_center
 *   left_shoulder      2  -48   -64   same height as neck
 *   right_shoulder     3  +48   -64
 *   left_elbow         4  -96   -64   96px from neck horiz
 *   right_elbow        5  +96   -64
 *   left_wrist         6 -144   -64
 *   right_wrist        7 +144   -64
 *   hip_center         8    0   -32   midpoint of hips
 *   left_hip           9  -32   -32   32px left of hip_center
 *   right_hip         10  +32   -32
 *   left_knee         11  -32    16   below hip
 *   right_knee        12  +32    16
 *   left_ankle        13  -32    64   below knee
 *   right_ankle       14  +32    64
 *
 * All positions start at pixel-space centre of the screen plus these
 * offsets.  The figure hangs in a T-pose so constraints initialise with
 * their natural rest lengths.
 *
 * prev_pos is set equal to pos so the alpha lerp is a no-op on frame 1.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Ragdoll *r = &sc->ragdoll;

    r->gravity    = GRAVITY;
    r->wind_force = WIND_FORCE;
    r->paused     = false;
    r->wind_timer = 0.0f;
    r->wind_x     = 0.0f;

    /* Init platforms first so they exist before figure is placed */
    init_platforms(sc->platforms, cols, rows);

    /* Spawn near top of screen so the figure falls through all platforms.
     * Head is at cy-96; CEIL_MARGIN=16, so cy must be > 112 to clear ceiling. */
    float cx = (float)(cols * CELL_W) * 0.5f;
    float cy = CEIL_MARGIN + 130.0f;

    /*
     * Initial T-pose positions.
     * (x-offset, y-offset) from (cx, cy) for each of the 15 particles.
     */
    static const float off[N_PARTICLES][2] = {
        /*  0 head          */  {  0.0f, -96.0f },
        /*  1 neck          */  {  0.0f, -64.0f },
        /*  2 left_shoulder */  { -48.0f, -64.0f },
        /*  3 right_shoulder*/  { +48.0f, -64.0f },
        /*  4 left_elbow    */  { -96.0f, -64.0f },
        /*  5 right_elbow   */  { +96.0f, -64.0f },
        /*  6 left_wrist    */  {-144.0f, -64.0f },
        /*  7 right_wrist   */  {+144.0f, -64.0f },
        /*  8 hip_center    */  {  0.0f, -32.0f },
        /*  9 left_hip      */  { -32.0f, -32.0f },
        /* 10 right_hip     */  { +32.0f, -32.0f },
        /* 11 left_knee     */  { -32.0f, +16.0f },
        /* 12 right_knee    */  { +32.0f, +16.0f },
        /* 13 left_ankle    */  { -32.0f, +64.0f },
        /* 14 right_ankle   */  { +32.0f, +64.0f },
    };

    for (int i = 0; i < N_PARTICLES; i++) {
        r->pos[i].x     = cx + off[i][0];
        r->pos[i].y     = cy + off[i][1];
        r->old_pos[i]   = r->pos[i];   /* zero initial velocity */
        r->prev_pos[i]  = r->pos[i];
    }

    /* Register all 17 distance constraints */
    int nc = 0;
    add_constraint(r, &nc,  0,  1);   /*  0: head → neck (spine top)          */
    add_constraint(r, &nc,  1,  8);   /*  1: neck → hip_center (spine)        */
    add_constraint(r, &nc,  1,  2);   /*  2: neck → left_shoulder             */
    add_constraint(r, &nc,  1,  3);   /*  3: neck → right_shoulder            */
    add_constraint(r, &nc,  2,  4);   /*  4: left_shoulder → left_elbow       */
    add_constraint(r, &nc,  4,  6);   /*  5: left_elbow → left_wrist          */
    add_constraint(r, &nc,  3,  5);   /*  6: right_shoulder → right_elbow     */
    add_constraint(r, &nc,  5,  7);   /*  7: right_elbow → right_wrist        */
    add_constraint(r, &nc,  8,  9);   /*  8: hip_center → left_hip            */
    add_constraint(r, &nc,  8, 10);   /*  9: hip_center → right_hip           */
    add_constraint(r, &nc,  9, 11);   /* 10: left_hip → left_knee             */
    add_constraint(r, &nc, 11, 13);   /* 11: left_knee → left_ankle           */
    add_constraint(r, &nc, 10, 12);   /* 12: right_hip → right_knee           */
    add_constraint(r, &nc, 12, 14);   /* 13: right_knee → right_ankle         */
    add_constraint(r, &nc,  2,  3);   /* 14: shoulder width stabiliser        */
    add_constraint(r, &nc,  9, 10);   /* 15: hip width stabiliser             */
    add_constraint(r, &nc,  0,  2);   /* 16: head → left_shoulder (strut)     */
    /* Note: (0,3) would be constraint 17, but we cap at N_CONSTRAINTS=17.
     * The head-to-right-shoulder strut is omitted; the left-side strut
     * plus the shoulder-width strut provides sufficient head stability. */
}

/*
 * scene_tick() — one fixed-step physics update.
 * dt is the fixed tick duration in seconds (1.0 / sim_fps).
 * Delegates to ragdoll_tick() which handles prev_pos save internally.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    ragdoll_tick(&sc->ragdoll, dt, cols, rows, sc->platforms);
}

/*
 * scene_draw() — render the scene; called once per render frame.
 * alpha ∈ [0,1) is the sub-tick interpolation factor (see §5e).
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_ragdoll(&sc->ragdoll, w, cols, rows, alpha, sc->platforms);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 * Holds the current terminal dimensions (cols, rows).
 */
typedef struct { int cols, rows; } Screen;

/*
 * screen_init() — configure the terminal for animation.
 * Settings match framework.c exactly (see snake_forward_kinematics.c §7
 * for full rationale).
 */
static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* screen_free() — restore terminal to pre-animation state. */
static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * screen_resize() — handle a SIGWINCH terminal resize event.
 * endwin() + refresh() forces ncurses to re-read LINES and COLS.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — compose a full frame into stdscr.
 *
 * Order:
 *   1. erase()    — clear newscr (no terminal write yet)
 *   2. scene_draw() — bones + particles + ground line
 *   3. HUD top    — status string right-aligned
 *   4. HUD bottom — key hint bar
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Ragdoll *r = &sc->ragdoll;

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  grav:%.0f  wind:%.0f  %s ",
             fps, sim_fps, r->gravity, r->wind_force,
             r->paused ? "PAUSED " : "running");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  w/s:gravity  a/d:wind  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * screen_present() — flush to terminal via ncurses double-buffer.
 * wnoutrefresh() → doupdate() sends only changed cells to the fd.
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state accessible from signal handlers.
 *
 * running and need_resize are volatile sig_atomic_t because POSIX signal
 * handlers can only communicate with the main loop through globals with
 * these qualifiers (see snake_forward_kinematics.c §8 for full rationale).
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* Signal handlers — set flags only; no ncurses or malloc calls */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* cleanup() — atexit safety net: endwin() even on unhandled exit */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize.
 *
 * After resize, clamp all particle positions to the new pixel bounds
 * so no particle is stranded in a now-invisible area.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    int      cols = app->screen.cols;
    int      rows = app->screen.rows;
    Ragdoll *r    = &app->scene.ragdoll;
    float    wpx  = (float)(cols * CELL_W);
    float    hpx  = (float)(rows * CELL_H);
    for (int i = 0; i < N_PARTICLES; i++) {
        if (r->pos[i].x >= wpx) { r->pos[i].x = wpx - 1.0f; r->old_pos[i].x = r->pos[i].x; }
        if (r->pos[i].y >= hpx) { r->pos[i].y = hpx - 1.0f; r->old_pos[i].y = r->pos[i].y; }
        if (r->pos[i].x < 0.0f) { r->pos[i].x = 0.0f;        r->old_pos[i].x = 0.0f; }
        if (r->pos[i].y < 0.0f) { r->pos[i].y = 0.0f;        r->old_pos[i].y = 0.0f; }
    }
    /* Rescale platforms to new terminal dimensions */
    init_platforms(app->scene.platforms, cols, rows);
    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * KEY MAP:
 *   q / Q / ESC   quit
 *   space         toggle pause
 *   r / R         reset simulation (ragdoll back to top + new platforms)
 *   w / ↑         gravity × 1.3  (faster fall)
 *   s / ↓         gravity ÷ 1.3  (slower fall)
 *   d / →         wind_force + 20
 *   a / ←         wind_force − 20 (min 0)
 *   ] / +         sim_fps + SIM_FPS_STEP
 *   [ / -         sim_fps − SIM_FPS_STEP
 */
static bool app_handle_key(App *app, int ch)
{
    Ragdoll *r = &app->scene.ragdoll;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ': r->paused = !r->paused; break;

    case 'r': case 'R':
        /* Reset: preserve sim_fps, restore everything else */
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        break;

    case 'w': case KEY_UP:
        r->gravity *= 1.3f;
        if (r->gravity > 3200.0f) r->gravity = 3200.0f;
        break;
    case 's': case KEY_DOWN:
        r->gravity /= 1.3f;
        if (r->gravity < 100.0f) r->gravity = 100.0f;
        break;

    case 'd': case KEY_RIGHT:
        r->wind_force += 20.0f;
        if (r->wind_force > 600.0f) r->wind_force = 600.0f;
        break;
    case 'a': case KEY_LEFT:
        r->wind_force -= 20.0f;
        if (r->wind_force < 0.0f) r->wind_force = 0.0f;
        break;

    case ']': case '+': case '=':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[': case '-':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

/* ─────────────────────────────────────────────────────────────────────
 * main() — the game loop  (structure identical to framework.c §8)
 *
 * Seven steps per frame (identical to snake_forward_kinematics.c):
 *  ① RESIZE CHECK     — handle pending SIGWINCH before touching ncurses
 *  ② MEASURE dt       — nanoseconds since last frame, capped at 100 ms
 *  ③ ACCUMULATOR      — fire scene_tick() at fixed sim_fps Hz
 *  ④ ALPHA            — sub-tick lerp factor for smooth rendering
 *  ⑤ FPS COUNTER      — 500 ms sliding window average
 *  ⑥ FRAME CAP        — sleep before render to hold ~60 fps
 *  ⑦ DRAW + PRESENT   — erase → draw → wnoutrefresh → doupdate
 *  ⑧ DRAIN INPUT      — getch() loop until ERR
 * ───────────────────────────────────────────────────────────────────── */
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

        int64_t frame_start = clock_ns();

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ─────────────────────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter ───────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ──────────────────── *
         * Budget = 1/60 s.  elapsed is wall time spent on physics +
         * accounting since frame_start; sleep the remainder so the
         * render rate sits at 60 fps regardless of sim Hz.           */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain all pending input ──────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
    }

    screen_free(&app->screen);
    return 0;
}
