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
 * References
 * ──────────
 *   ── Verlet integration (the §5a solver) ──────────────────────────
 *   [1] Verlet, L. (1967), "Computer Experiments on Classical Fluids.
 *       I. Thermodynamical Properties of Lennard-Jones Molecules",
 *       Phys. Rev. 159(1), pp. 98-103 — the original integrator.
 *       The position-only formulation that implicit_velocity()
 *       relies on dates from this paper.
 *   [2] Swope, W. C. et al. (1982), "A computer simulation method
 *       for the calculation of equilibrium constants...", J. Chem.
 *       Phys. 76 — velocity-Verlet variant (we use Verlet's
 *       original position-only form, sometimes called Störmer-Verlet).
 *
 *   ── Ragdoll / character physics (the FULL PIPELINE) ─────────────
 *   [3] Jakobsen, T. (2001), "Advanced Character Physics", GDC —
 *       THE canonical Verlet-ragdoll paper; §2 integrator, §3
 *       distance-constraint relaxation, §4 collisions. This file
 *       implements the entire pipeline described therein.
 *       https://www.cs.cmu.edu/afs/cs/academic/class/15462-s13/www/lec_slides/Jakobsen.pdf
 *   [4] Hecker, C. (2005), "Behind the Screen — Verlet Physics",
 *       Game Developer Magazine — accessible walk-through of the
 *       Jakobsen pipeline with the gotchas a novice will hit.
 *
 *   ── Position Based Dynamics (the modern generalisation) ──────────
 *   [5] Müller, M., Heidelberger, B., Hennix, M. & Ratcliff, J.
 *       (2007), "Position Based Dynamics", J. Vis. Comm. Image Repr.
 *       18(2), pp. 109-118 — generalises Jakobsen's distance-only
 *       projection to arbitrary constraints (bending, volume, etc.).
 *   [6] Bender, J., Müller, M. & Macklin, M. (2017), "A Survey on
 *       Position-Based Simulation Methods in Computer Graphics",
 *       Computer Graphics Forum 36(6) — modern survey including
 *       XPBD and stable extensions.
 *
 *   ── Classical mechanics (the physics of collisions) ─────────────
 *   [7] Goldstein, H. (1980), "Classical Mechanics" (2nd ed.),
 *       Addison-Wesley — §3.6 on collisions: the coefficient of
 *       restitution e and the vector reflection formula
 *       v' = v − (1+e)(v·n)n that drives reflect_velocity_with_
 *       restitution() in §5b-2.
 *
 *   ── Computer graphics (geometric primitives) ─────────────────────
 *   [8] Hughes, J. F., van Dam, A. et al. (2013), "Computer Graphics:
 *       Principles and Practice" (3rd ed.) — §16.4 surface normals;
 *       basis for surface_upward_normal() in §5b-2.
 *
 *   ── Timestep / animation ─────────────────────────────────────────
 *   [9] Fiedler, G. (2004), "Fix Your Timestep!", gafferongames.com —
 *       the fixed-step accumulator pattern this file uses; alpha
 *       interpolation between prev_pos and pos comes from the same
 *       source.
 *
 *   ── Rendering / ncurses ─────────────────────────────────────────
 *  [10] Bresenham, J. E. (1965), "Algorithm for computer control of
 *       a digital plotter", IBM Systems Journal 4(1), pp. 25-30 —
 *       the integer line algorithm; §5e draw_bone uses the simpler
 *       parametric oversample because float math is already paid
 *       for in the physics step.
 *  [11] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — basis for
 *       the high-contrast head + joint glyph choices.
 *  [12] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the diff pipeline §7 relies on.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [13] https://en.wikipedia.org/wiki/Verlet_integration
 *  [14] https://en.wikipedia.org/wiki/Position-based_dynamics
 *  [15] https://en.wikipedia.org/wiki/Coefficient_of_restitution
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

    /*
     * N_THEMES  — entries in §3 THEMES[]. Each theme rebinds pairs 1..7
     *             (body parts) to a distinct multi-colour palette. HUD/HINT
     *             pairs (8, 9) are theme-independent per CLAUDE.md HUD spec.
     * N_PRESETS — entries in §6 PRESETS[]. Each preset bundles (gravity,
     *             wind_force, sim_fps) into a named physics scenario.
     * N_THEME_SLOTS — slot count per theme; matches the 7 body pairs.
     */
    N_THEMES           = 10,
    N_PRESETS          =  5,
    N_THEME_SLOTS      =  7,
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
 * Theme — one named multi-colour palette for the 7 body-rendering pairs.
 *
 * Slot semantics (col[0..6] → pairs PAIR_HEAD..PAIR_STRUT):
 *   [0] head marker            [4] leg bones + ankles
 *   [1] non-head joint dots    [5] slanted platform beads
 *   [2] spine + collarbones    [6] stabiliser struts + ground line (A_DIM)
 *   [3] arm bones + wrists
 *
 * HUD/HINT pairs (PAIR_HUD, PAIR_HINT) are NOT theme-bound — they keep
 * bright yellow / bright cyan across every theme so the status bar
 * remains readable against any palette (CLAUDE.md HUD spec).
 *
 * Brightness rule: every entry sits in the BRIGHT HALF of the 256-colour
 * cube — cube indices ≥ 24, grayscale ≥ 240 — so even the A_DIM strut
 * stays visible. See CLAUDE.md "Theme Palette Brightness".
 */
typedef struct {
    const char *name;                 /* HUD-displayable theme name        */
    int         col[N_THEME_SLOTS];   /* xterm-256 fg indices per slot;    *
                                       * slots [0..6] → PAIR_HEAD..STRUT   */
} Theme;

/*
 * THEMES — ten multi-colour palettes. Each theme uses several distinct
 * hues across the seven body slots; head and leg colours are usually
 * the most contrasted so the eye finds the figure quickly.
 */
static const Theme THEMES[N_THEMES] = {
    /*  name         HEAD  BODY  SPN   ARM   LEG   PLAT  STRUT */
    { "CLASSIC",  { 231,  255,  248,  214,   75,   45,  244 } }, /* default */
    { "MATRIX",   { 231,  195,  119,   46,   40,   34,   34 } }, /* green code */
    { "FIRE",     { 226,  214,  208,  202,  196,  166,  130 } }, /* yellow→red */
    { "ICE",      { 231,  195,  159,  123,   87,   51,   45 } }, /* white→cyan */
    { "TOXIC",    { 227,  220,  190,  154,  118,   82,   34 } }, /* yellow→green */
    { "NEON",     { 213,  207,  201,  165,  129,   93,   57 } }, /* pink→purple */
    { "AUTUMN",   { 228,  220,  214,  208,  166,  130,   94 } }, /* gold→brown */
    { "MONO",     { 255,  250,  245,  240,  244,  248,  244 } }, /* greyscale  */
    { "AURORA",   { 159,  120,   87,   51,   39,   99,  135 } }, /* mint→violet */
    { "CARNIVAL", { 226,  196,  202,   51,   46,  201,  244 } }, /* every primary */
};

/*
 * theme_apply() — rebind pairs PAIR_HEAD..PAIR_STRUT to theme `idx`.
 *
 *   The seven init_pair calls are the only mutation; HUD/HINT pairs
 *   stay untouched. In 8-colour mode (COLORS < 256) the index palette
 *   is not available, so we leave the fallback pairs from color_init()
 *   in place — themes are a 256-colour feature.
 *
 *   Defensive bound: out-of-range idx folds to 0 (CLASSIC).
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS < 256) return;

    const Theme *t = &THEMES[idx];
    init_pair(PAIR_HEAD,     t->col[0], -1);
    init_pair(PAIR_BODY,     t->col[1], -1);
    init_pair(PAIR_SPINE,    t->col[2], -1);
    init_pair(PAIR_ARM,      t->col[3], -1);
    init_pair(PAIR_LEG,      t->col[4], -1);
    init_pair(PAIR_PLATFORM, t->col[5], -1);
    init_pair(PAIR_STRUT,    t->col[6], -1);
}

/*
 * color_init() — start ncurses colour, bind the initial theme,
 * and pin the two theme-independent HUD pairs.
 *
 * Background = -1 (terminal default) so the demo respects the user's
 * terminal background.  Foreground tints from the bright half of the
 * 256-colour cube — see CLAUDE.md "Theme Palette Brightness".
 *
 *   Pair  Role                                    Theme-bound?
 *   ──────────────────────────────────────────────────────────
 *     1   head marker                             yes
 *     2   generic joint dot                       yes
 *     3   spine + collarbones                     yes
 *     4   arm bones, wrists                       yes
 *     5   leg bones, ankles                       yes
 *     6   slanted platform beads                  yes
 *     7   stabiliser struts + ground line         yes
 *     8   HUD top status (PAIR_HUD)               NO  (always 226 yellow)
 *     9   HUD bottom key hint (PAIR_HINT)         NO  (always  51 cyan)
 *
 * For COLORS < 256 we fall back to the 8-colour ANSI palette and
 * ignore theme cycling (no 256-cube available).
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
    } else {
        /* 8-colour fallback — theme-independent, coarser but readable */
        init_pair(PAIR_HEAD,     COLOR_WHITE,   -1);
        init_pair(PAIR_BODY,     COLOR_WHITE,   -1);
        init_pair(PAIR_SPINE,    COLOR_WHITE,   -1);
        init_pair(PAIR_ARM,      COLOR_YELLOW,  -1);
        init_pair(PAIR_LEG,      COLOR_CYAN,    -1);
        init_pair(PAIR_PLATFORM, COLOR_CYAN,    -1);
        init_pair(PAIR_STRUT,    COLOR_WHITE,   -1);
    }

    /* HUD pairs — theme-independent in both 256- and 8-colour modes. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
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
 * Vec2 — a 2-D point in PIXEL space (sub-cell precision).
 *
 * Intent
 *   Every §5 physics quantity — particle positions, velocities (as
 *   pos − old_pos), wind impulses, surface normals — lives in pixel
 *   space. Each character cell is CELL_W × CELL_H sub-pixels
 *   (8 × 16), giving sub-cell precision so the figure moves smoothly
 *   instead of jumping cell-to-cell. The conversion to cell space
 *   happens only inside §5e rendering helpers via px_to_cell_x/y —
 *   the project's "one conversion point" rule.
 *
 * Convention
 *   x : EASTWARD pixel coordinate (positive → right of screen).
 *   y : SOUTHWARD pixel coordinate (positive → DOWN the screen).
 *
 *   The y-down convention matters for the physics:
 *     - GRAVITY is POSITIVE (pulls toward larger y).
 *     - The UPWARD surface normal of a slanted platform has NEGATIVE y.
 *     - "Above" in screen coords is "smaller y".
 *
 * Why a value type
 *   8 bytes; passes in registers. implicit_velocity, apply_damping,
 *   integrate_acceleration, surface_upward_normal, reflect_velocity_
 *   with_restitution all take/return Vec2 by value — -O2 inlines
 *   them into straight-line code with no allocation.
 *
 * Why named (x, y) instead of two loose floats
 *   Type-checking catches (col, row) confusion at compile time
 *   rather than via visual debugging.
 *
 * References [3] Jakobsen §2 for the Verlet position representation;
 *   [8] Hughes & van Dam §16.4 on surface normals in screen space.
 */
typedef struct {
    float x;   /* eastward  pixel coordinate (positive → right) */
    float y;   /* southward pixel coordinate (positive → down)  */
} Vec2;

/*
 * Ragdoll — the full simulation state for the humanoid figure.
 *
 * Intent
 *   A 15-particle skeleton enforced by 17 distance constraints,
 *   integrated with position-only Verlet. Four interlocked subsystems
 *   share one record:
 *
 *     1. VERLET STATE — (pos, old_pos) — the integrator's two-position
 *        memory. Velocity is IMPLICIT: v = pos − old_pos. Modifying
 *        old_pos (without touching pos) injects velocity — this is
 *        how every collision response works in §5b / §5b-2. Ref [1].
 *
 *     2. RENDER SNAPSHOT — prev_pos — copied from pos at the START of
 *        each physics tick. render_ragdoll lerps prev_pos → pos by
 *        alpha ∈ [0, 1) (the leftover fraction in the fixed-step
 *        accumulator) so motion stays smooth at any combination of
 *        sim Hz and render Hz. Ref [9] Fiedler.
 *
 *     3. CONSTRAINT NETWORK — (c_a[], c_b[], c_len[]) — three parallel
 *        arrays describing N_CONSTRAINTS distance constraints. Each
 *        constraint i connects particle c_a[i] to c_b[i] and tries
 *        to keep them at distance c_len[i]. Rest lengths are computed
 *        from the initial T-pose in scene_init, so the natural
 *        spacing IS the rest pose — no separate constant table.
 *        Iteratively projected to convergence by satisfy_constraint
 *        (Jakobsen relaxation). Refs [3] Jakobsen §3, [5] Müller PBD.
 *
 *     4. ENVIRONMENT FORCES — (gravity, wind_force, wind_timer,
 *        wind_x). gravity is a constant downward acceleration; wind
 *        fires periodic random horizontal impulses; wind_timer counts
 *        seconds since the last gust.
 *
 *   The subsystems interact in a fixed order each tick:
 *     env-forces → Verlet integrate → boundary collide → platform
 *     collide → relax constraints (with collision re-pinning).
 *   See ragdoll_tick in §5d for the orchestrator.
 *
 * Why pos AND old_pos AND prev_pos (three position arrays)
 *   - pos     : the CURRENT physics position; mutates every tick.
 *   - old_pos : the PREVIOUS physics position; defines implicit velocity.
 *   - prev_pos: the position at the START of the current tick;
 *               anchor for sub-tick render interpolation.
 *   Why not collapse old_pos and prev_pos? Because they are updated
 *   at DIFFERENT moments: old_pos is shifted inside verlet_update
 *   and tweaked by every collision response throughout the tick;
 *   prev_pos is frozen at tick start. Conflating them would corrupt
 *   either the integrator or the renderer.
 *
 * Why constraint arrays are PARALLEL (SoA), not array-of-structs
 *   The inner loop of relax_constraints_with_collision_repinning
 *   walks all three arrays in lockstep; SoA gives sequential cache
 *   reads. Also the rest of the file already uses parallel arrays
 *   for per-particle state (foot/hip/etc in other files), so the
 *   convention is consistent.
 *
 * Why wind is APPLIED VIA old_pos (not as a force in verlet_update)
 *   Wind is an IMPULSE (an instantaneous velocity change), not a
 *   sustained force. In Verlet form, an impulse is a shift of
 *   old_pos in the direction of the desired velocity change. This
 *   keeps the integrator's force term (just gravity) simple.
 *
 * References [1] Verlet 1967 + [3] Jakobsen 2001 for the integrator
 *   and constraint network; [5] Müller PBD for the projection
 *   generalisation; [9] Fiedler for the prev_pos lerp pattern.
 */
typedef struct {
    /* ── Verlet state (the integrator's two-position memory) ──── *
     * pos    : current positions, mutated every tick.            *
     * old_pos: previous positions; v = pos − old_pos (implicit). *
     *          Collision response writes here to inject velocity. */
    Vec2 pos     [N_PARTICLES];
    Vec2 old_pos [N_PARTICLES];

    /* ── Render snapshot (sub-tick interpolation anchor) ──────── *
     * Frozen at the start of each physics tick; render_ragdoll   *
     * lerps prev_pos → pos by alpha for smooth motion.           */
    Vec2 prev_pos[N_PARTICLES];

    /* ── Constraint network (SoA, length N_CONSTRAINTS) ───────── *
     * Constraint i: keep particles c_a[i] and c_b[i] at distance *
     * c_len[i]. Rest lengths from the initial T-pose.            */
    int   c_a  [N_CONSTRAINTS];
    int   c_b  [N_CONSTRAINTS];
    float c_len[N_CONSTRAINTS];

    /* ── Environment forces ───────────────────────────────────── *
     * gravity     : continuous downward acceleration (px/s²).    *
     * wind_force  : magnitude of each gust impulse (px/s).        *
     * wind_timer  : seconds since last gust; gust fires when      *
     *               ≥ WIND_PERIOD.                                *
     * wind_x      : current gust contribution to integrator       *
     *               acceleration this tick (px/s²). Decays via    *
     *               the natural DAMPING factor.                   */
    float gravity;            /* runtime-tunable via w/s + presets    */
    float wind_force;         /* runtime-tunable via a/d + presets    */
    float wind_timer;
    float wind_x;

    /* ── Control flags ────────────────────────────────────────── */
    bool  paused;             /* gates scene_tick; renderer still runs */
} Ragdoll;

/* ── §5a  verlet_update ─────────────────────────────────────────────── */

/*
 * implicit_velocity — Verlet's defining trick.
 *
 *   In Verlet integration, velocity is NEVER stored explicitly. It's
 *   reconstructed from two consecutive positions:
 *
 *       v(t) ≈ pos(t) − pos(t − dt)
 *
 *   This means the state vector (pos, old_pos) carries BOTH position
 *   AND momentum — any modification to old_pos automatically alters
 *   the implicit velocity. Collision response uses this property
 *   heavily: shift old_pos behind pos to inject velocity.
 *
 *   Reference: Verlet (1967) "Computer Experiments on Classical
 *   Fluids", Phys. Rev. 159 — original integrator. Jakobsen (2001)
 *   "Advanced Character Physics" GDC — cloth/character application.
 */
static inline Vec2 implicit_velocity(Vec2 pos, Vec2 old_pos)
{
    return (Vec2){ pos.x - old_pos.x, pos.y - old_pos.y };
}

/*
 * apply_damping — multiplicative velocity damping per tick.
 *
 *   v' = v · DAMPING
 *
 *   DAMPING = 0.995 corresponds to ~0.5% kinetic-energy loss per
 *   tick — a very mild air drag. Multiplicative damping in Verlet
 *   form is equivalent to a viscous friction term −β·v in the
 *   continuous equations of motion, with β chosen so that
 *   (1 − β·dt) ≈ DAMPING for the simulation's dt.
 */
static inline Vec2 apply_damping(Vec2 v, float damping)
{
    return (Vec2){ v.x * damping, v.y * damping };
}

/*
 * integrate_acceleration — Verlet position step.
 *
 *   pos(t + dt) = pos(t) + v(t)·dt + a·dt²
 *
 *   In our reduced form (with v already multiplied by dt in the
 *   implicit-velocity reconstruction), the equation collapses to:
 *
 *       pos' = pos + v + a · dt²
 *
 *   where v is the position DELTA over the last tick (i.e. already
 *   "·dt") and a · dt² is the standard ½·a·t² term — Verlet's
 *   second-order accuracy folds the ½ into the implicit-velocity
 *   formulation, so the explicit coefficient is 1 not ½.
 */
static inline Vec2 integrate_acceleration(Vec2 pos, Vec2 v, Vec2 a, float dt2)
{
    return (Vec2){
        pos.x + v.x + a.x * dt2,
        pos.y + v.y + a.y * dt2,
    };
}

/*
 * verlet_update — one Störmer-Verlet integration step on particle i.
 *
 *   1. v       = implicit_velocity(pos, old_pos)   (Verlet trick)
 *   2. v      ← apply_damping(v)                   (air drag)
 *   3. a       = (wind_x, gravity)                 (external accel)
 *   4. old_pos ← pos                               (slide state vector)
 *   5. pos    ← integrate_acceleration(pos, v, a, dt²)
 *
 *   Step 4 MUST run before step 5 — otherwise the new pos would
 *   overwrite the cur it depends on. The temporary variable
 *   captured at step 1 prevents the aliasing.
 *
 *   gravity is positive because +y is downward in pixel space.
 *
 *   Reference: Jakobsen (2001) §2 "Verlet Integration".
 */
static void verlet_update(Ragdoll *r, int i, float dt)
{
    float dt2 = dt * dt;

    Vec2 v = implicit_velocity(r->pos[i], r->old_pos[i]);
    v      = apply_damping(v, DAMPING);
    Vec2 a = (Vec2){ r->wind_x, r->gravity };

    Vec2 cur = r->pos[i];
    r->old_pos[i] = cur;
    r->pos[i]     = integrate_acceleration(cur, v, a, dt2);
}

/* ── §5b  apply_boundaries ──────────────────────────────────────────── */

/*
 * bounce_against_wall_1d — one-axis elastic-with-restitution bounce.
 *
 *   For an axis-aligned wall in 1-D, the Verlet bounce is:
 *
 *       1. v_axis = pos − old_pos          (implicit velocity, 1-D)
 *       2. pos    ← wall                    (clamp onto surface)
 *       3. old_pos ← wall + v_axis · e     (encode reflected vel)
 *
 *   Step 3 places old_pos PAST the wall by an amount proportional
 *   to the impact velocity, so the next tick's implicit velocity
 *   (pos − old_pos) points AWAY from the wall — the bounce.
 *
 *   e ∈ (0, 1] is the COEFFICIENT OF RESTITUTION:
 *     e = 1    perfectly elastic (no energy loss)
 *     e → 0    perfectly inelastic (sticks to wall)
 *     e = 0.55 our default — rubbery thump
 *
 *   Reference: any classical mechanics text on 1-D collisions;
 *   e.g. Goldstein "Classical Mechanics" §3.6.
 */
static inline void bounce_against_wall_1d(float *pos, float *old_pos,
                                          float wall)
{
    float v_axis = *pos - *old_pos;
    *pos     = wall;
    *old_pos = wall + v_axis * BOUNCE_COEFF;
}

/*
 * snap_to_wall_1d — one-axis HARD STOP (no bounce).
 *
 *   Both pos and old_pos collapse to `wall`, which zeroes the
 *   implicit velocity on that axis. Used for the ceiling where
 *   bouncing would feel unphysical (gravity pulls particles
 *   down; ceiling impacts are vanishingly rare and the soft stop
 *   prevents tunnelling).
 */
static inline void snap_to_wall_1d(float *pos, float *old_pos, float wall)
{
    *pos     = wall;
    *old_pos = wall;
}

/*
 * apply_boundaries — keep particle i inside the screen rectangle.
 *
 *   Four axis-aligned walls. Floor + side walls BOUNCE; ceiling
 *   HARD-STOPS. Each test is independent so corner impacts bounce
 *   correctly on both axes.
 *
 *     wall          test                action
 *     ────────────────────────────────────────────────────────
 *     floor         pos.y > floor_y     bounce_against_wall_1d
 *     ceiling       pos.y < ceil_y      snap_to_wall_1d
 *     left wall     pos.x < left_x      bounce_against_wall_1d
 *     right wall    pos.x > right_x     bounce_against_wall_1d
 */
static void apply_boundaries(Ragdoll *r, int i, int cols, int rows)
{
    float floor_y = (float)(rows * CELL_H) - FLOOR_MARGIN;
    float ceil_y  = CEIL_MARGIN;
    float left_x  = LEFT_MARGIN;
    float right_x = (float)(cols * CELL_W) - RIGHT_MARGIN;

    if (r->pos[i].y > floor_y)
        bounce_against_wall_1d(&r->pos[i].y, &r->old_pos[i].y, floor_y);
    if (r->pos[i].y < ceil_y)
        snap_to_wall_1d       (&r->pos[i].y, &r->old_pos[i].y, ceil_y);
    if (r->pos[i].x < left_x)
        bounce_against_wall_1d(&r->pos[i].x, &r->old_pos[i].x, left_x);
    if (r->pos[i].x > right_x)
        bounce_against_wall_1d(&r->pos[i].x, &r->old_pos[i].x, right_x);
}

/* ── §5b-2  platform collision ─────────────────────────────────────── */

/*
 * Platform — one slanted shelf the ragdoll bounces off of.
 *
 * Intent
 *   A platform is a LINE SEGMENT in pixel space defined by a centre
 *   point (cx, y), a half-width (half_w), and a slope (dy/dx).
 *   The surface equation is:
 *
 *       y_surface(x) = y + (x − cx) · slope     for x ∈ [cx−hw, cx+hw]
 *
 *   §5b-2 `surface_y_at_x` and `particle_within_platform_extent`
 *   both read directly from this struct.
 *
 * Slope sign convention (screen +y down)
 *   POSITIVE slope ⇒ right side LOWER on screen:  \
 *   NEGATIVE slope ⇒ right side HIGHER on screen: /
 *   ZERO slope     ⇒ flat horizontal:             —
 *
 *   The slanted surface DEFLECTS the ragdoll sideways on impact (a
 *   slanted normal mixes y-momentum into x-momentum) — this is what
 *   makes the figure zig-zag through the platform stack instead of
 *   falling straight down. See `surface_upward_normal` and
 *   `reflect_velocity_with_restitution` in §5b-2.
 *
 * Why a value-only struct (no behaviour)
 *   Platforms are immutable scenery — they sit in `Scene::platforms[]`
 *   and never mutate after `init_platforms()`. All behaviour
 *   (intersection, reflection) lives in §5b-2 helpers that take a
 *   `const Platform *`.
 *
 * References [7] Goldstein §3.6 for the impact mechanics; [8] Hughes
 *   & van Dam §16.4 for the line-segment geometry.
 */
typedef struct {
    float cx;       /* centre x (pixel space)                          */
    float y;        /* surface y at cx (pivot point of the surface line)*/
    float half_w;   /* half-width along x; surface exists on [-hw,+hw]  */
    float slope;    /* dy/dx in screen coords (+y down); see signs above*/
} Platform;

/*
 * surface_y_at_x — height of the platform surface at horizontal x.
 *
 *     y_surface(x) = y_centre + (x − cx) · slope
 *
 *   Linear equation of a line, with cx as the pivot point and slope
 *   as dy/dx (rise per pixel rightward in screen coordinates).
 *   POSITIVE slope ⇒ right side LOWER (screen +y down):  \
 *   NEGATIVE slope ⇒ right side HIGHER:                   /
 *
 *   Called twice per collision check (current x, previous x) so the
 *   swept-segment test handles a slanted surface correctly even
 *   when the particle moves horizontally between ticks.
 */
static inline float surface_y_at_x(const Platform *pl, float x)
{
    return pl->y + (x - pl->cx) * pl->slope;
}

/*
 * particle_within_platform_extent — x-range gate.
 *
 *   A platform exists only on |x − cx| ≤ half_w. A particle outside
 *   the x-range is treated as missing the platform entirely (the
 *   surface line has no support there).
 */
static inline bool particle_within_platform_extent(const Platform *pl,
                                                   float x)
{
    return fabsf(x - pl->cx) <= pl->half_w;
}

/*
 * particle_crossed_surface_from_above — swept-segment collision test.
 *
 *   The particle's trajectory from old_pos to pos is a line segment.
 *   We check whether that segment crossed the platform surface
 *   going DOWNWARD (from above to below):
 *
 *       old_pos.y ≤ surf_y(old_pos.x)  AND  pos.y ≥ surf_y(pos.x)
 *
 *   The double-surface-sample form (different x at each end) is
 *   what makes this work for slanted platforms. Using a single
 *   surf_y would alias under fast horizontal motion.
 */
static inline bool particle_crossed_surface_from_above(const Platform *pl,
                                                       Vec2 old_pos,
                                                       Vec2 pos)
{
    float surf_now = surface_y_at_x(pl, pos.x);
    float surf_old = surface_y_at_x(pl, old_pos.x);
    return (old_pos.y <= surf_old) && (pos.y >= surf_now);
}

/*
 * surface_upward_normal — unit normal of the inclined line,
 * pointing UP in screen coordinates (toward smaller y).
 *
 *   The tangent of a line y = m·x is (1, m).
 *   Rotating by +90° (math-CCW) gives (−m, 1), which points DOWN
 *   on screen because y-axis is flipped.
 *   Rotating by −90° gives (m, −1) — pointing UP on screen. We
 *   want the upward normal; normalising gives:
 *
 *       n = (slope, −1) / √(slope² + 1)
 *
 *   For slope = 0 (flat platform): n = (0, −1) — straight up. ✓
 *   For positive slope (\): n leans rightward — perpendicular to \. ✓
 *
 *   Reference: Hughes, van Dam et al. "Computer Graphics: Principles
 *   and Practice" §16.4 — geometric reflection / surface normals.
 */
static inline Vec2 surface_upward_normal(float slope)
{
    float length = sqrtf(slope * slope + 1.0f);
    return (Vec2){ slope / length, -1.0f / length };
}

/*
 * reflect_velocity_with_restitution — vector reflection of v about
 * surface normal n, scaled by coefficient of restitution e.
 *
 *   Decompose v into normal and tangential components:
 *       v_n = (v · n) · n             (normal component)
 *       v_t = v − v_n                 (tangential component)
 *
 *   Elastic reflection:    v' = v_t − v_n
 *   With restitution e:    v' = v_t − e · v_n
 *                              = v − (1 + e) · (v · n) · n
 *
 *   e = 0 → "stuck to surface" (zero normal velocity after impact).
 *   e = 1 → perfectly elastic (kinetic energy preserved on impact).
 *
 *   Why the slanted surface DEFLECTS the figure sideways: when n
 *   has a non-zero x component, the (v · n) · n term subtracts
 *   from BOTH x and y components of v. Energy from the y direction
 *   leaks into the x direction.
 *
 *   Reference: Goldstein "Classical Mechanics" §3.6 on collisions;
 *   Verlet 1967 + Jakobsen 2001 for the integration framework.
 */
static inline Vec2 reflect_velocity_with_restitution(Vec2 v, Vec2 n, float e)
{
    float dot    = v.x * n.x + v.y * n.y;
    float factor = (1.0f + e) * dot;
    return (Vec2){ v.x - factor * n.x, v.y - factor * n.y };
}

/*
 * write_implicit_velocity — encode a velocity vector back into the
 * Verlet (pos, old_pos) state.
 *
 *   Verlet's implicit-velocity rule is  v = pos − old_pos.
 *   To make the next tick see velocity v, place old_pos at:
 *
 *       old_pos = pos − v
 *
 *   Used after any collision response that computes a new velocity:
 *   pos is already at the collision point; we write old_pos behind
 *   it so the implicit velocity matches the reflected one.
 */
static inline void write_implicit_velocity(Vec2 *old_pos, Vec2 pos, Vec2 v)
{
    old_pos->x = pos.x - v.x;
    old_pos->y = pos.y - v.y;
}

/*
 * resolve_one_platform_collision — full reflection sequence for
 * particle i against platform pl (assumed already INTERSECTED).
 *
 *   1. Capture v = implicit_velocity(pos, old_pos) BEFORE snapping
 *      (so the saved v reflects the pre-collision motion).
 *   2. pos.y ← surface_y_at_x(pl, pos.x)        (snap onto surface)
 *   3. n     = surface_upward_normal(pl.slope)  (Frenet normal)
 *   4. if v · n ≥ 0  : already moving away — return (no-op).
 *   5. v_reflected = reflect_velocity_with_restitution(v, n, e)
 *   6. write_implicit_velocity(old_pos, pos, v_reflected)
 *
 *   Step 4 (the "already moving away" guard) handles the case where
 *   the particle rests on the surface and the constraint pass has
 *   pushed it slightly sideways — without this guard, the reflection
 *   would launch the particle off the platform spuriously.
 */
static void resolve_one_platform_collision(Ragdoll *r, int i,
                                           const Platform *pl)
{
    Vec2 v = implicit_velocity(r->pos[i], r->old_pos[i]);

    r->pos[i].y = surface_y_at_x(pl, r->pos[i].x);
    Vec2 n      = surface_upward_normal(pl->slope);

    float vn = v.x * n.x + v.y * n.y;
    if (vn >= 0.0f) return;            /* moving away — no bounce */

    Vec2 v_reflected = reflect_velocity_with_restitution(v, n, BOUNCE_COEFF);
    write_implicit_velocity(&r->old_pos[i], r->pos[i], v_reflected);
}

/*
 * apply_platform_collisions — bounce particle i off any slanted shelf.
 *
 *   For each platform pl:
 *     1. particle_within_platform_extent (x-range gate)
 *     2. particle_crossed_surface_from_above (swept-segment test)
 *     3. resolve_one_platform_collision (slanted-surface bounce)
 *
 *   A slanted surface naturally transfers some y-momentum into x-
 *   momentum, deflecting the ragdoll sideways on impact — the same
 *   physics that makes a stone skip on water.
 */
static void apply_platform_collisions(Ragdoll *r, int i,
                                      const Platform *plats, int n)
{
    for (int p = 0; p < n; p++) {
        const Platform *pl = &plats[p];

        if (!particle_within_platform_extent(pl, r->pos[i].x)) continue;
        if (!particle_crossed_surface_from_above(pl,
                                                 r->old_pos[i],
                                                 r->pos[i])) continue;

        resolve_one_platform_collision(r, i, pl);
    }
}

/* ── §5c  satisfy_constraint ────────────────────────────────────────── */

/*
 * displacement_and_length — Δ = b − a and its Euclidean length, in one pass.
 *
 *   Returns:
 *     *out_delta  ← b − a            (displacement vector)
 *     return val  ← |b − a|         (Euclidean length)
 *
 *   Saving one sqrt + one struct field-write per call vs computing
 *   them separately. Called N_CONSTRAINT_ITERS × N_CONSTRAINTS times
 *   per tick — the inner loop of the position-based dynamics relax.
 */
static inline float displacement_and_length(Vec2 a, Vec2 b, Vec2 *out_delta)
{
    out_delta->x = b.x - a.x;
    out_delta->y = b.y - a.y;
    return sqrtf(out_delta->x * out_delta->x +
                 out_delta->y * out_delta->y);
}

/*
 * satisfy_constraint — Jakobsen relaxation step for one distance
 * constraint (position-based dynamics).
 *
 *   Given particles p_a, p_b and rest length L, define:
 *
 *       Δ           = p_b − p_a              (current displacement)
 *       d           = |Δ|                    (current length)
 *       fractional  = (d − L) / d            (stretch / length, signed)
 *
 *   POSITIVE fractional → stretched (need to pull together).
 *   NEGATIVE fractional → compressed (need to push apart).
 *
 *   EQUAL-MASS CORRECTION (Jakobsen 2001 §3): split the error
 *   evenly between the two endpoints, each moving HALF the residual:
 *
 *       p_a' = p_a + ½ · fractional · Δ      (toward midpoint if
 *       p_b' = p_b − ½ · fractional · Δ       stretched)
 *
 *   For unequal masses m_a, m_b the splits become m_b/(m_a+m_b) and
 *   m_a/(m_a+m_b). We use uniform mass so the splits are both ½.
 *
 *   Iterating this projection over ALL constraints N_CONSTRAINT_ITERS
 *   times converges the system: each pass typically halves the
 *   residual stretch (geometric convergence — Gauss-Seidel style).
 *
 *   References: Jakobsen (2001) "Advanced Character Physics" GDC §3.
 *     Müller, Heidelberger, Hennix & Ratcliff (2007) "Position Based
 *     Dynamics", J. Vis. Comm. Image Repr. 18(2) — generalisation.
 */
static void satisfy_constraint(Ragdoll *r, int ci)
{
    int a = r->c_a[ci];
    int b = r->c_b[ci];

    Vec2  delta;
    float length = displacement_and_length(r->pos[a], r->pos[b], &delta);
    if (length < 1e-6f) return;        /* degenerate: particles coincide */

    float fractional = (length - r->c_len[ci]) / length;
    float cx         = 0.5f * fractional * delta.x;
    float cy         = 0.5f * fractional * delta.y;

    r->pos[a].x += cx;  r->pos[a].y += cy;
    r->pos[b].x -= cx;  r->pos[b].y -= cy;
}

/* ── §5d  ragdoll_tick ──────────────────────────────────────────────── */

/*
 * snapshot_for_render_interpolation — copy pos into prev_pos.
 *
 *   The renderer lerps prev_pos → pos by `alpha` (the fractional
 *   leftover in the fixed-step accumulator) so motion stays smooth
 *   even when render Hz ≠ sim Hz. MUST run before physics modifies
 *   pos this tick — otherwise the lerp anchor collapses.
 */
static inline void snapshot_for_render_interpolation(Ragdoll *r)
{
    memcpy(r->prev_pos, r->pos, sizeof r->pos);
}

/*
 * apply_wind_gust — periodic random horizontal impulse to every particle.
 *
 *   Wind in Verlet form is a VELOCITY KICK, not a force. Moving
 *   old_pos by Δx shifts the implicit velocity by +Δx for the next
 *   tick — exactly what an instantaneous lateral push would do.
 *
 *   Gust timing  : every WIND_PERIOD seconds.
 *   Magnitude    : wind_force × random ∈ [0.5, 1.0]   (so gusts vary).
 *   Direction    : ±1 coin flip                       (left or right).
 *
 *   `dt` is the per-tick duration so impulse · dt scales the kick
 *   consistently across different sim_fps choices.
 */
static void apply_wind_gust(Ragdoll *r, float dt)
{
    r->wind_timer += dt;
    if (r->wind_timer < WIND_PERIOD) return;
    r->wind_timer = 0.0f;

    float direction = (rand() % 2 == 0) ? 1.0f : -1.0f;
    float magnitude = r->wind_force * (0.5f + (float)(rand() % 100) / 200.0f);
    float impulse   = direction * magnitude;
    for (int i = 0; i < N_PARTICLES; i++)
        r->old_pos[i].x -= impulse * dt;
}

/*
 * relax_constraints_with_collision_repinning — N iterations of
 * Jakobsen distance-constraint relaxation, INTERLEAVED with
 * platform-collision repinning.
 *
 *   For iter = 0 .. N_CONSTRAINT_ITERS:
 *     1. Project every distance constraint to rest length
 *        (satisfy_constraint for ci = 0 .. N_CONSTRAINTS).
 *     2. Re-apply platform collisions to every particle.
 *
 *   WHY INTERLEAVE? Constraint projection moves particles. That
 *   motion can push a particle THROUGH a surface it had bounced off
 *   in the pre-relaxation pass. Re-applying collisions after each
 *   projection step "re-pins" any particle that crossed back through
 *   a platform, preventing bones from tunnelling through shelves.
 *
 *   This is the position-based-dynamics pattern: alternate between
 *   internal (constraint) and external (collision) projections until
 *   they roughly agree. Convergence rate is geometric — each iter
 *   halves the residual violation in practice.
 *
 *   References: Jakobsen (2001) §3-4; Müller et al. (2007) §3.
 */
static void relax_constraints_with_collision_repinning(Ragdoll *r,
                                                       const Platform *plats)
{
    for (int iter = 0; iter < N_CONSTRAINT_ITERS; iter++) {
        for (int ci = 0; ci < N_CONSTRAINTS; ci++)
            satisfy_constraint(r, ci);
        for (int i = 0; i < N_PARTICLES; i++)
            apply_platform_collisions(r, i, plats, N_PLATFORMS);
    }
}

/*
 * ragdoll_tick — one complete physics step.
 *
 *   1. snapshot_for_render_interpolation
 *        prev_pos ← pos                 (anchor for sub-tick lerp)
 *
 *   2. pause gate
 *        if paused return                (prev_pos saved so freeze is clean)
 *
 *   3. apply_wind_gust                   (random horizontal velocity kick)
 *
 *   4. verlet_update per particle        (integrate gravity + wind)
 *
 *   5. apply_boundaries per particle     (walls / floor / ceiling)
 *
 *   6. apply_platform_collisions per particle  (initial platform pass)
 *
 *   7. relax_constraints_with_collision_repinning
 *        N_CONSTRAINT_ITERS × (Jakobsen + collision re-pin)
 *
 *   ORDER MATTERS:
 *     integration (4) → boundary collision (5) → platform collision (6)
 *     → constraint relaxation with re-pinning (7).
 *
 *   The collision-BEFORE-relaxation order is what lets a foot land
 *   on the floor first (5), then have the constraint pull the knee
 *   into position (7). Reversing the order would tunnel bones
 *   through the floor.
 *
 *   References: Jakobsen (2001) §3-4 for the full pipeline;
 *     Verlet (1967) for the integrator.
 */
static void ragdoll_tick(Ragdoll *r, float dt, int cols, int rows,
                         const Platform *plats)
{
    /* (1) snapshot for render interpolation */
    snapshot_for_render_interpolation(r);

    /* (2) pause gate */
    if (r->paused) return;

    /* (3) wind */
    apply_wind_gust(r, dt);

    /* (4) Verlet integration */
    for (int i = 0; i < N_PARTICLES; i++)
        verlet_update(r, i, dt);

    /* (5) walls + floor + ceiling */
    for (int i = 0; i < N_PARTICLES; i++)
        apply_boundaries(r, i, cols, rows);

    /* (6) initial platform pass (before relaxation) */
    for (int i = 0; i < N_PARTICLES; i++)
        apply_platform_collisions(r, i, plats, N_PLATFORMS);

    /* (7) PBD constraint relaxation with collision re-pinning */
    relax_constraints_with_collision_repinning(r, plats);
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
    /*
     * BOLD on every ANATOMICAL bone (0..13) so the figure reads as a
     * "filled in" body rather than a skeletal wire-frame. Stabiliser
     * struts (14..16) stay DIM — they are physics-only constraints,
     * not body parts, and should fade into the background.
     */
    static const attr_t bone_attr[N_CONSTRAINTS] = {
        A_BOLD, A_BOLD,                            /*  0-1  spine        */
        A_BOLD, A_BOLD,                            /*  2-3  collarbones  */
        A_BOLD, A_BOLD, A_BOLD, A_BOLD,            /*  4-7  arms         */
        A_BOLD, A_BOLD,                            /*  8-9  hip cross    */
        A_BOLD, A_BOLD, A_BOLD, A_BOLD,            /* 10-13 legs         */
        A_DIM,  A_DIM,  A_DIM,                     /* 14-16 stabilisers  */
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
 * Visual hierarchy (after brightness pass to make the figure read
 * as a human rather than a wire-frame skeleton):
 *
 *   Particle role        Glyph  Pair         Attr      Why
 *   ─────────────────────────────────────────────────────────────────
 *   HEAD (i=0)           — handled by draw_head() (two-cell head)
 *   neck (1)             '+'    SPINE        A_DIM     hidden by spine
 *   shoulders (2,3)      'o'    SPINE        A_BOLD    torso anchor
 *   elbows (4,5)         'o'    ARM          A_NORMAL  mid-limb hinge
 *   wrists (6,7)         'o'    ARM          A_BOLD    "fists" — limb tip
 *   hip_center (8)       '+'    SPINE        A_DIM     hidden by hip cross
 *   hips (9,10)          'o'    SPINE        A_BOLD    torso anchor
 *   knees (11,12)        'o'    LEG          A_NORMAL  mid-limb hinge
 *   ankles (13,14)       'v'    LEG          A_BOLD    "feet" — limb tip
 *
 * BOLD on the four torso anchors (shoulders + hips) + four limb tips
 * (fists + feet) gives the figure a "human" silhouette: a clear torso
 * with visible body-mass corners, plus four limb extremities. The
 * mid-limb joints (elbows, knees) stay NORMAL so they don't compete
 * with the limb tips for attention.
 */
static void draw_particles(WINDOW *w, const Vec2 rp[N_PARTICLES],
                           int cols, int rows)
{
    for (int i = 1; i < N_PARTICLES; i++) {     /* skip i=0; draw_head handles it */
        int cx = px_to_cell_x(rp[i].x);
        int cy = px_to_cell_y(rp[i].y);

        char   ch;
        int    pair;
        attr_t attr;

        switch (i) {
        case 1:  case 8:                          /* neck, hip_center            */
            ch = '+'; pair = PAIR_SPINE; attr = A_DIM;    break;
        case 2:  case 3:  case 9:  case 10:       /* shoulders + hips            */
            ch = 'o'; pair = PAIR_SPINE; attr = A_BOLD;   break;
        case 4:  case 5:                          /* elbows                      */
            ch = 'o'; pair = PAIR_ARM;   attr = A_NORMAL; break;
        case 6:  case 7:                          /* wrists / fists              */
            ch = 'o'; pair = PAIR_ARM;   attr = A_BOLD;   break;
        case 11: case 12:                         /* knees                       */
            ch = 'o'; pair = PAIR_LEG;   attr = A_NORMAL; break;
        case 13: case 14:                         /* ankles / feet               */
            ch = 'v'; pair = PAIR_LEG;   attr = A_BOLD;   break;
        default:
            ch = '.'; pair = PAIR_BODY;  attr = A_DIM;    break;
        }

        mark_cell(w, cx, cy, ch, pair, attr, cols, rows);
    }
}

/*
 * draw_head() — two-cell head: face + tumble-aware "hair" halo cell.
 *
 *   Cell 1 (face):  'O' BOLD at the head particle position.
 *   Cell 2 (hair):  '\'' BOLD one cell-step along the (head − neck)
 *                   direction — i.e. away from the body, on the
 *                   "outside" of the head. Because the offset is
 *                   recomputed every frame from the live head→neck
 *                   vector, the hair stays on top of the head no
 *                   matter how the ragdoll tumbles (right-side-up,
 *                   upside-down, sideways — the halo always points
 *                   away from the torso).
 *
 *   Step distance — one cell of anisotropy: (CELL_W·u_x, CELL_H·u_y).
 *   This makes the halo land on a NEIGHBOURING cell in any direction
 *   regardless of the screen's 8:16 cell aspect ratio.
 *
 *   Degenerate guard — if the head and neck collide (rare, very
 *   strong compression), the unit vector is undefined; skip the
 *   halo and just draw the face.
 *
 *   Drawn LAST in render_ragdoll() so the head + halo win every
 *   shared cell against bones and other joint markers — the head
 *   stays the focal point of the figure.
 */
static void draw_head(WINDOW *w, const Vec2 rp[N_PARTICLES],
                      int cols, int rows)
{
    Vec2 head = rp[0];
    Vec2 neck = rp[1];

    /* Face cell — 'O' is universally a "head" in ASCII art. */
    int fx = px_to_cell_x(head.x);
    int fy = px_to_cell_y(head.y);
    mark_cell(w, fx, fy, 'O', PAIR_HEAD, A_BOLD, cols, rows);

    /* Hair cell — one cell along (head − neck), normalised. */
    float dx = head.x - neck.x;
    float dy = head.y - neck.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;          /* degenerate: head ≈ neck */

    float ux = dx / len;
    float uy = dy / len;
    int hx = px_to_cell_x(head.x + ux * (float)CELL_W);
    int hy = px_to_cell_y(head.y + uy * (float)CELL_H);

    /* Skip if halo would overdraw the face cell (e.g. very tight neck). */
    if (hx == fx && hy == fy) return;

    mark_cell(w, hx, hy, '\'', PAIR_HEAD, A_BOLD, cols, rows);
}

/*
 * render_ragdoll() — orchestrate one frame in painter's order.
 *
 *   1. lerp_positions  — sub-tick interpolation
 *   2. draw_platforms  — slanted shelves first (bottom layer)
 *   3. draw_ground     — floor reference line
 *   4. draw_bones      — every distance constraint stamped as glyph line
 *   5. draw_particles  — joint markers on top of bones (skips head)
 *   6. draw_head       — two-cell head (face + tumble-aware hair) ON TOP
 *
 * Order matters: each later layer over-stamps earlier ones at shared
 * cells, so joints read above bones, the head reads above joints, etc.
 * draw_head runs LAST so the focal point of the figure always wins.
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
    draw_head      (w, rp,       cols, rows);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — composition root for §6.
 *
 * Intent
 *   The world is one ragdoll plus a fixed stack of N_PLATFORMS
 *   slanted shelves. Scene bundles both into a single record so the
 *   framework loop (scene_init / scene_tick / scene_draw) reads
 *   identically to every other file in the repo.
 *
 * Simulation vs Rendering locality
 *   The Ragdoll struct itself already separates Verlet state /
 *   render snapshot / constraint network / environment forces /
 *   control flags. Within Scene there are only two members:
 *
 *     ── Simulation state ──   things scene_tick READS+WRITES
 *        ragdoll               the 15-particle figure (§5)
 *
 *     ── World geometry (immutable) ──  scenery the figure interacts
 *                                       with; never mutated after
 *                                       init_platforms() in scene_init
 *        platforms[]           N_PLATFORMS slanted shelves (§5b-2)
 *
 *   Platforms count as SIMULATION input (the collision pass reads
 *   them) but never as simulation OUTPUT — they don't carry energy
 *   or position state between ticks. Re-initialised on SIGWINCH so
 *   the shelf layout always fits the current terminal.
 *
 * Things that DO NOT live here
 *   - sim_fps + theme_idx + preset_idx → §8 App (these are SESSION
 *     state, not part of the simulated world).
 *   - fps counter → main() local (display-only).
 *   - terminal extents (cols, rows) → §7 Screen.
 *   - signal flags (running, need_resize) → §8 App.
 *   - Preset / Theme tables → file-scope `static const`; never
 *     mutate, never duplicated per scene.
 *
 * One Scene per program; passed by pointer to every §6 entry point.
 *
 * References [3] Jakobsen for the physics composition; [9] Fiedler
 *   for the scene_tick / scene_draw separation that the framework
 *   loop in §8 uses.
 */
typedef struct {
    /* ── Simulation state ─────────────────────────────────────── */
    Ragdoll  ragdoll;                  /* 15-particle humanoid figure */

    /* ── World geometry (immutable scenery) ───────────────────── */
    Platform platforms[N_PLATFORMS];   /* slanted shelves              */
} Scene;

/*
 * Preset — one named physics scenario.
 *
 * Bundles the three runtime-tunable scalars (gravity, wind_force,
 * sim_fps) into a single "world preset" the user can cycle between
 * with n/p. Each entry is a coherent setting — e.g. MOON is low
 * gravity with light air, JUPITER is heavy gravity with light wind.
 *
 *   gravity    — px/s² along +y (downward). Earth-equivalent ≈ 800.
 *   wind_force — magnitude of each random lateral impulse (px/s).
 *   sim_fps    — fixed-step Hz for the physics accumulator; higher
 *                values give a stiffer feel at the cost of CPU.
 *
 * Reference: any introductory physics text — surface gravity ratios
 * (Moon ≈ 1/6 g, Jupiter ≈ 2.5 g). The values here are SCALED for
 * visual effect, not literal: the demo trades physical accuracy for
 * readable motion at terminal frame rates.
 */
typedef struct {
    const char *name;
    float       gravity;
    float       wind_force;
    int         sim_fps;
} Preset;

/*
 * PRESETS — five named scenarios. PRESETS[0] EARTH is the default
 * matching the original §1 constants GRAVITY + WIND_FORCE.
 *
 *   EARTH    — baseline; the original feel of the demo.
 *   MOON     — low gravity; figure drifts slowly downward.
 *   TORNADO  — Earth gravity + violent wind; rag-doll thrash.
 *   JUPITER  — heavy gravity; collapses fast and slams platforms.
 *   ZERO_G   — gravity off; wind pushes the figure around in a void.
 */
static const Preset PRESETS[N_PRESETS] = {
    /*  name         gravity   wind_force  sim_fps */
    { "EARTH",         800.0f,   120.0f,     60 },
    { "MOON",          130.0f,    20.0f,     60 },
    { "TORNADO",       800.0f,   500.0f,     60 },
    { "JUPITER",      2000.0f,    60.0f,     80 },
    { "ZERO_G",          0.0f,   200.0f,     60 },
};

/*
 * preset_apply() — write the preset's physics scalars into the ragdoll.
 *
 *   Only mutates the runtime-tunable scalars; the chain topology,
 *   particle positions, and constraint network are untouched. Callers
 *   that want a fresh figure (e.g. n/p preset cycling) call scene_init
 *   BEFORE preset_apply.
 *
 *   sim_fps lives in App, not in Ragdoll, so the caller is responsible
 *   for applying PRESETS[idx].sim_fps to app->sim_fps separately.
 *
 *   Defensive bound: out-of-range idx folds to 0 (EARTH).
 */
static void preset_apply(Ragdoll *r, int idx)
{
    if (idx < 0 || idx >= N_PRESETS) idx = 0;
    const Preset *p = &PRESETS[idx];
    r->gravity    = p->gravity;
    r->wind_force = p->wind_force;
}

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
 * Screen — terminal-extent snapshot in CHARACTER CELLS.
 *
 * Intent
 *   Caches the current terminal size so every §6 entry point reads
 *   (cols, rows) as plain ints rather than re-querying ncurses each
 *   frame. Refreshed only when SIGWINCH sets App::need_resize, then
 *   propagated to scene_init via app_do_resize.
 *
 * Why a separate struct (not just two ints in App)
 *   Resize logic (endwin + refresh + getmaxyx) touches NOTHING in App
 *   except this struct. Carving it out makes screen_resize pure and
 *   isolates the ncurses dependency from the simulation layer.
 *
 * Why cells, not pixels
 *   ncurses' coordinate system is cells. Pixel space (CELL_W × CELL_H
 *   sub-pixels per cell) lives only inside §5 — converted at the
 *   draw boundary, per the project's "one conversion point" rule.
 *
 * References [12] Raymond, NCURSES Programming HOWTO.
 */
typedef struct {
    int cols;   /* terminal width  in CHARACTER CELLS */
    int rows;   /* terminal height in CHARACTER CELLS */
} Screen;

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
    color_init(0);                  /* CLASSIC; cycled via 't' at runtime */
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
                        int theme_idx, int preset_idx,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Ragdoll *r = &sc->ragdoll;

    /* Row 0 (top-right): live status — PAIR_HUD bright yellow, A_BOLD */
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

    /* Row 1 (top-left): theme + preset readout — PAIR_HUD, no A_BOLD so
     * row 0 stays the dominant status line per CLAUDE.md HUD spec. */
    char buf2[HUD_COLS + 1];
    snprintf(buf2, sizeof buf2,
             " theme: %-9s  preset: %-8s ",
             THEMES [theme_idx ].name,
             PRESETS[preset_idx].name);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, 0, "%s", buf2);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  w/s:grav  a/d:wind  [/]:Hz  t:theme  n/p:preset ");
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
 * App — top-level container; everything outside the world.
 *
 * Intent
 *   Bundles the simulated world (Scene), the host terminal (Screen),
 *   and the session-level loop-control flags into one record so
 *   main() reads as four-line phases: init / service signals /
 *   step+draw / shutdown. Declared file-scope (g_app) so signal
 *   handlers — which cannot take a user argument — can write
 *   `running` and `need_resize` without globals scattered through
 *   the file.
 *
 * Locality of concern
 *   ── Owned subsystems ── nouns the app composes
 *      scene       — the world being simulated (§6)
 *      screen      — the terminal extent it draws to (§7)
 *
 *   ── Session state ── user-tunable settings that survive a reset
 *      sim_fps     — physics tick rate (cycled with [ / ])
 *      theme_idx   — index into §3 THEMES[]  (cycled with `t`)
 *      preset_idx  — index into §6 PRESETS[] (cycled with `n`/`p`)
 *
 *   ── Loop control ── verbs the loop reads each frame
 *      running     — clear → loop exits; set by SIGINT/SIGTERM
 *      need_resize — set by SIGWINCH; cleared after Screen refresh
 *
 * Why theme_idx / preset_idx live HERE (not in Scene)
 *   Session state, not simulation state. `r` reset rebuilds the
 *   ragdoll from scratch (scene_init wipes Scene memory) but the
 *   user's chosen theme + preset must persist across that reset.
 *   Putting them in App keeps them out of the memset's scope.
 *
 * Why volatile sig_atomic_t (not bool, not int)
 *   `volatile`    : the compiler must not cache the flag across a
 *                   signal-handler write — every loop iteration must
 *                   re-read it from memory.
 *   `sig_atomic_t`: POSIX-guaranteed atomic with respect to async
 *                   signals; a plain `int` could be observed half-
 *                   written on architectures where stores are split.
 *   See [12] Raymond §"Signal handling".
 *
 * Things that DO NOT live here
 *   - Wall-clock timestamps / fps counters — main() locals; no
 *     other code path needs them.
 *   - Ragdoll runtime values (gravity, wind_force) — sim state in
 *     §5 Ragdoll. Presets override these via preset_apply().
 *   - Theme + Preset tables themselves — file-scope `static const`.
 */
typedef struct {
    /* ── Owned subsystems ─────────────────────────────────────── */
    Scene  scene;              /* the world (§6)                       */
    Screen screen;             /* terminal extent (§7)                 */

    /* ── Session state (survives scene_init reset) ────────────── */
    int    sim_fps;            /* physics tick rate (Hz)               */
    int    theme_idx;          /* index into §3 THEMES[]               */
    int    preset_idx;         /* index into §6 PRESETS[]              */

    /* ── Loop control ─────────────────────────────────────────── */
    volatile sig_atomic_t running;      /* main loop predicate            */
    volatile sig_atomic_t need_resize;  /* SIGWINCH pending               */
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
 * cycle_preset() — switch to a different preset and respawn the figure.
 *
 *   Preset cycling resets the scene (so the new physics scenario starts
 *   from a clean T-pose) and applies the preset's sim_fps. Theme is
 *   preserved across the reset so visual identity stays stable.
 *
 *   `dir` is ±1 (next/previous, wraps modulo N_PRESETS).
 */
static void cycle_preset(App *app, int dir)
{
    int next = (app->preset_idx + dir + N_PRESETS) % N_PRESETS;
    app->preset_idx = next;

    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    preset_apply(&app->scene.ragdoll, next);
    app->sim_fps = PRESETS[next].sim_fps;
}

/*
 * cycle_theme() — switch to a different colour theme.
 *
 *   theme_apply rebinds pairs 1..7 via init_pair; the next frame's
 *   draw picks up the new palette automatically. Cheap (7 calls);
 *   no scene reset needed.
 */
static void cycle_theme(App *app, int dir)
{
    int next = (app->theme_idx + dir + N_THEMES) % N_THEMES;
    app->theme_idx = next;
    theme_apply(next);
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * KEY MAP:
 *   q / Q / ESC   quit
 *   space         toggle pause
 *   r / R         reset simulation (preserves theme + preset)
 *   w / ↑         gravity × 1.3  (faster fall)
 *   s / ↓         gravity ÷ 1.3  (slower fall)
 *   d / →         wind_force + 20
 *   a / ←         wind_force − 20 (min 0)
 *   ] / +         sim_fps + SIM_FPS_STEP
 *   [ / -         sim_fps − SIM_FPS_STEP
 *   t / T         cycle THEME (10 multi-colour palettes)
 *   n / N         next PRESET   (5 named physics scenarios)
 *   p / P         previous PRESET
 */
static bool app_handle_key(App *app, int ch)
{
    Ragdoll *r = &app->scene.ragdoll;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ': r->paused = !r->paused; break;

    case 'r': case 'R':
        /* Reset: preserve sim_fps + theme + preset; re-apply preset
         * physics so the freshly-spawned figure inherits current scenario. */
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        preset_apply(&app->scene.ragdoll, app->preset_idx);
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

    case 't': case 'T':
        cycle_theme(app, +1);
        break;

    case 'n': case 'N':
        cycle_preset(app, +1);
        break;
    case 'p': case 'P':
        cycle_preset(app, -1);
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

    App *app        = &g_app;
    app->running    = 1;
    app->sim_fps    = SIM_FPS_DEFAULT;
    app->theme_idx  = 0;                /* CLASSIC palette                */
    app->preset_idx = 0;                /* EARTH physics (matches §1)     */

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    preset_apply(&app->scene.ragdoll, app->preset_idx);

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
                    app->theme_idx, app->preset_idx,
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
