/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * perlin_terrain_bot.c — self-balancing wheel-bot crossing a Perlin landscape
 *
 * DEMO: A two-wheeled robot like a Segway. By itself it would tip over
 *       in milliseconds — gravity is unstable for an inverted
 *       pendulum. Stay-upright is achieved by a PID controller
 *       reading the body's lean angle and pushing the wheels forward
 *       or backward to keep the pendulum balanced. The terrain is
 *       procedurally generated Perlin noise that scrolls past as the
 *       robot drives forward; on slopes the controller adapts its
 *       setpoint so the bot leans into the hill rather than fighting
 *       gravity directly.
 *
 *           ┌────────────────────────────────────────────┐
 *           │     . .  ✦                                 │
 *           │                                            │
 *           │           *      ← top of body (beacon)    │
 *           │            \                               │
 *           │             \                              │
 *           │              \                             │
 *           │             O═O ← wheels (chassis tilts    │
 *           │            /‾‾‾‾\___      with terrain)    │
 *           │       /‾‾‾‾      ‾‾‾\___                   │
 *           │  /‾‾‾‾                ‾‾‾‾\__              │
 *           │##############################              │
 *           └────────────────────────────────────────────┘
 *
 *       Three view modes (toggle with `m`):
 *           TELEMETRY — live values + bar gauges
 *           EQUATIONS — the math with live numbers substituted
 *           PHASE     — phase-space portrait of (θ, ω)
 *
 *       Three real concepts at play:
 *           1. INVERTED PENDULUM — gravity pulls the body away from
 *              vertical. Without control, θ grows exponentially.
 *           2. CART-POLE COUPLING — pushing the wheels horizontally
 *              tilts the body the OTHER way. That's how the bot
 *              corrects itself.
 *           3. PID CONTROL — three-term feedback that produces the
 *              motor force every tick from the body's lean error.
 *
 * Real-world analogues:
 *           Segway PT, Ninebot, hoverboards — same physics
 *           Boston Dynamics Handle (taller pendulum, wheeled base)
 *           Kid balancing a broom on their palm
 *
 * Study alongside:
 *   robots/diff_drive_robot.c             — wheels, no balancing
 *                                            (simpler kinematics).
 *   robots/moving_jump_spring_leg_robot.c — one-legged jumping robot
 *                                            (different actuation,
 *                                             same Perlin terrain).
 *   particle_systems/ragdoll_figure.c     — gravity-driven articulated
 *                                            body without control.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus per-element semantic pairs
 *   §4  noise        — Perlin noise + fBm (terrain primitive)
 *   §5  terrain      — height query, slope, ring buffer
 *   §6  bot          — Bot type, PID, cart-pole physics, init/reset
 *   §7  render       — paint terrain, robot, telemetry, equations, phase
 *   §8  screen       — ncurses init / present
 *   §9  app          — signals, resize, variable-dt main loop with substeps
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   r                reset bot + new terrain
 *   ↑ / ↓            change drive speed
 *   p                toggle PID controller (watch the bot fall)
 *   m                cycle view (TELEMETRY / EQUATIONS / PHASE)
 *   g                cycle gain preset (BALANCED / HIGH-Kp / NO-Kd / NO-Ki)
 *   + / -            tweak Kp manually
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra robots/perlin_terrain_bot.c \
 *       -o perlin_terrain_bot -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Three layers stacked together.
 *
 *                 (A) PERLIN TERRAIN — 5-octave fBm gives a smoothly
 *                     rolling height function h(x). The bot's
 *                     world_x advances at drive_spd; we sample
 *                     h(world_x) for the wheel contact and use a
 *                     central-difference slope α as a feed-forward
 *                     hint to the controller.
 *
 *                 (B) INVERTED PENDULUM — the body is a rigid rod
 *                     of length L, mass m_p, hinged at the wheel
 *                     axle. Its state is (θ, ω) — angle from
 *                     vertical and angular velocity. Gravity tips
 *                     it: τ_gravity = m·g·L·sin(θ). Without
 *                     control, θ runs away exponentially.
 *
 *                 (C) PID CONTROLLER — every tick reads the lean
 *                     error e = θ − θ_ref, computes
 *                          F = Kp·e + Ki·∫e + Kd·de/dt
 *                     and applies F as horizontal motor force on
 *                     the cart (axle). Cart-pole coupling tilts
 *                     the body the other way; the controller's job
 *                     is to keep θ near 0 despite gravity.
 *
 *                 The setpoint θ_ref is shifted by the terrain slope
 *                 (θ_ref = -α · SLOPE_FEED) so the bot leans into the
 *                 hill — the slope pushes the body, and we ask the
 *                 controller to expect that.
 *
 * Data-structure: One Bot struct holds:
 *                   • pendulum state (θ, ω)
 *                   • cached cart-pole derived values (θ_eff,
 *                     M_eff, ẍ, F) for display
 *                   • PID gains (Kp, Ki, Kd) and integrator state
 *                   • motion (world_x, drive_spd)
 *                   • phase-space history ring buffer
 *                   • UI flags (paused, fallen, view mode, preset)
 *
 *                 Terrain is a TBUF-entry ring buffer of heights
 *                 indexed by world column; new columns are filled
 *                 as the bot moves forward.
 *
 * Rendering     : Painter's order — last write wins:
 *                   (1) Sky (with sparse stars)
 *                   (2) Terrain — surface row + texture row + rock
 *                       fill below
 *                   (3) Robot — chassis, wheels, body line, beacon
 *                   (4) Right-side panel — telemetry / equations /
 *                       phase portrait depending on view mode
 *                   (5) HUD row 0 (yellow) + hint row last (cyan)
 *
 * Performance   : Per frame: O(cols) for terrain, O(1) for robot,
 *                 O(HIST_LEN) for phase trail. Sub-stepping in the
 *                 physics integrator keeps the controller stable
 *                 even at low frame rates: each frame splits dt
 *                 into N substeps of size ≤ TICK_DT_TARGET = 1/120
 *                 sec, and each substep runs one integration step.
 *
 * References    :
 *   Wikipedia, "Inverted pendulum" — the canonical control problem.
 *     https://en.wikipedia.org/wiki/Inverted_pendulum
 *   Wikipedia, "PID controller" — three-term feedback in industrial
 *     and robotic systems.  https://en.wikipedia.org/wiki/PID_controller
 *   Åström & Hägglund, "Advanced PID Control" (ISA, 2006) — the
 *     reference text on PID tuning, anti-windup, and feed-forward.
 *   Anderson, "Cart-pole physics" derivation — Lagrangian or Newton-
 *     Euler approach, both give the same equations of motion.
 *   Wikipedia, "Phase space" — the (θ, ω) diagram is a phase
 *     portrait of a 2-D dynamical system.
 *     https://en.wikipedia.org/wiki/Phase_space
 *   Inigo Quilez, "Perlin noise" — the terrain primitive.
 *     https://iquilezles.org/articles/morenoise/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Three independent ideas working together:
 *   1. Gravity wants to tip the bot over — it's an inverted pendulum
 *      and that's fundamentally unstable.
 *   2. Pushing the wheels forward/backward makes the body lean the
 *      OTHER way — that's the cart-pole coupling that makes balancing
 *      possible.
 *   3. A PID controller reads the lean error and computes the right
 *      amount of force to push, every single tick. Slope-aware: the
 *      setpoint shifts so the bot leans into the hill.
 *
 *
 * ANALOGY: BALANCING A BROOM ON YOUR HAND
 * ───────────────────────────────────────
 *
 *   When you balance a broom upright on your palm, you don't push UP
 *   on the broom — you slide your hand SIDEWAYS. If the broom tilts
 *   right, you slide your hand right (faster than the broom is
 *   tipping). That accelerates the broom's base rightward, which
 *   makes the top rotate LEFT (relative to the base), which corrects
 *   the lean.
 *
 *   Your hand = the cart (wheel). The broom = the pendulum. Your
 *   eyes-and-arm = the PID controller, reading the broom's tilt and
 *   computing where to slide your hand.
 *
 *   The bot does exactly this, except instead of "where to slide my
 *   hand" the controller outputs "what motor force to apply to the
 *   wheel". Same physics.
 *
 *
 * GEOMETRY DIAGRAM
 * ────────────────
 *
 *      (gravity ↓)                   * ← top of body (mass m_p)
 *                                   /
 *                                  /  ← rod of length L
 *                                 /    pendulum state: (θ, ω)
 *                                /     θ  = angle from vertical
 *                               /      ω  = θ̇  = angular velocity
 *                              /
 *                       O═════O ← cart (wheels)
 *                       │  ↑
 *                       │  F = motor force from PID (horizontal)
 *                       │
 *                       └──→ x ← cart position
 *                            ẍ = horizontal acceleration
 *
 *      The CRUCIAL coupling:
 *        • Pushing the cart RIGHT (ẍ > 0) tilts the body LEFT.
 *        • Pushing the cart LEFT  (ẍ < 0) tilts the body RIGHT.
 *      That's how the controller corrects lean — by accelerating
 *      the BASE in the same direction as the lean (so the top is
 *      "left behind" and rotates back toward vertical).
 *
 *
 * THE CART-POLE EQUATIONS (Lagrangian)
 * ────────────────────────────────────
 *
 *      M_eff = M_cart + m_pole · sin²θ_eff
 *
 *      ẍ      = (F + m_pole · sin θ_eff · (L · ω² − g · cos θ_eff)) / M_eff
 *
 *      θ̈      = (g · sin θ_eff − ẍ · cos θ_eff) / L
 *
 *      θ_eff = θ + α   (α = terrain slope; gives "lean from gravity",
 *                        not "lean from chassis")
 *
 *      Then forward Euler:
 *          ω ← ω + θ̈ · dt
 *          θ ← θ + ω · dt
 *
 *      Why M_eff = M_cart + m_pole · sin²θ_eff?
 *      Some of the pole's mass effectively belongs to the cart's
 *      inertia (the part being dragged horizontally as the pole
 *      rotates). That fraction is sin²θ — zero at θ = 0 (pole
 *      perfectly upright; cart inertia is just M_cart) and 1 at
 *      θ = 90° (pole horizontal; the entire pole is being shoved
 *      horizontally).
 *
 *
 * PID CONTROLLER — block diagram
 * ──────────────────────────────
 *
 *      θ_ref ──┐                   ┌────────────────┐
 *              │  e = θ − θ_ref    │     Kp · e     │  P term
 *      θ ──────┼─►  error  ─────►  │  + Ki · ∫e     │  I term
 *                                  │  + Kd · de/dt  │  D term
 *                                  └────────┬───────┘
 *                                           ▼  F
 *                                  ┌────────────────┐
 *                                  │  Cart-pole     │
 *                                  │  dynamics      │
 *                                  └────────┬───────┘
 *                                           ▼
 *                              new θ, ω ────┘ (closes the loop)
 *
 *      • P  (Kp · e)    — instant stiffness. "Tilt 0.1 rad? Apply 12 N back."
 *                         Larger Kp → faster correction but more overshoot.
 *      • I  (Ki · ∫e)   — drift correction. On a slope a fixed lean accumulates
 *                         in the integral; Ki sums that error and cancels the
 *                         steady-state offset. Anti-windup clamps the integral
 *                         so it can't grow unbounded.
 *      • D  (Kd · de/dt) — damping. Resists rapid changes in error,
 *                         brakes oscillation. Without Kd, the system
 *                         oscillates forever (underdamped).
 *
 *
 * SLOPE FEED-FORWARD
 * ──────────────────
 *
 *   On a slope of α radians, gravity pulls the body away from
 *   chassis-perpendicular. To stay upright with respect to gravity,
 *   the body must LEAN into the slope. We tell the controller to
 *   target a tilted setpoint:
 *
 *       θ_ref = -SLOPE_FEED · α
 *
 *   With SLOPE_FEED = 0.65, on a 20° slope (α ≈ 0.35 rad), the
 *   setpoint becomes ≈ -13°. The bot's body leans 13° into the
 *   slope, perfectly upright with respect to gravity (modulo the
 *   feed-forward gain).
 *
 *   Without slope feed-forward, only the integral term would
 *   correct slope drift — too slowly to follow rapidly changing
 *   terrain. With feed-forward, slope is mostly handled
 *   pre-emptively and Ki only mops up small residuals.
 *
 *
 * PHASE PORTRAIT
 * ──────────────
 *
 *   Plotting (θ, ω) over time on a 2-D plane shows the controller's
 *   character:
 *
 *      ω ↑
 *        │           UNDERDAMPED (small Kd)
 *        │     ╭─.─╮            wide circles
 *        │    ╱     ╲           that slowly shrink
 *        │   ╱  ●    ╲
 *        │  ╱         ╲    →
 *        │ ╱           ╲
 *        │╱             ╲       STABLE (good Kd)
 *        ┼────────────────────  tight inward spiral
 *        │                ╲     converges to (0, 0)
 *        │                 ╲
 *        │  OVERDAMPED      ╲   →  ────────────●
 *        │  (large Kd)              slow direct
 *        │                          path to origin
 *        │                                          → θ
 *
 *   In the demo's phase view you'll see the actual trajectory.
 *   Press 'g' to cycle gains and watch how the trajectory shape
 *   changes — that's a tuning lesson by direct observation.
 *
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. Measure dt = wall-clock seconds since last frame.
 *  2. Compute n_substeps = ceil(dt / TICK_DT_TARGET), capped at 16.
 *     This keeps the integrator stable when frames take longer
 *     than the physics timestep.
 *  3. For each substep:
 *       a. Advance world_x by drive_spd · sub_dt.
 *       b. Sample terrain slope α at the new x.
 *       c. Compute θ_eff = θ + α and θ_ref = -SLOPE_FEED · α.
 *       d. PID:
 *           e = θ − θ_ref
 *           ∫e += e · sub_dt   (clamped by anti-windup)
 *           de/dt = (e − e_prev) / sub_dt
 *           F = Kp·e + Ki·∫e + Kd·de/dt    (clamped to ±F_MAX)
 *       e. Cart-pole physics:
 *           M_eff = M_cart + m_pole · sin²θ_eff
 *           ẍ     = (F + m_pole·sin·(L·ω² − g·cos)) / M_eff
 *           θ̈     = (g·sin θ_eff − ẍ·cos θ_eff) / L
 *           ω    += θ̈ · sub_dt
 *           θ    += ω · sub_dt
 *       f. Append (θ, ω) to phase-history ring buffer.
 *       g. If |θ_eff| > FALL_ANGLE → mark as fallen; stop ticking.
 *  4. erase()
 *  5. Render terrain → robot → side panel (telemetry / equations /
 *     phase) → HUD row 0 + hint strip.
 *  6. doupdate; sleep to TARGET_FPS.
 *
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Stiff dynamics. At low frame rates (e.g. 30 fps), forward-Euler
 *    integration can become marginally unstable for the cart-pole.
 *    Sub-stepping each frame at TICK_DT_TARGET = 1/120 sec keeps
 *    the integrator inside the stability region.
 *
 *  • Integrator wind-up. On steep slopes the error is sustained;
 *    ∫e grows without bound and the controller overshoots wildly
 *    when the slope ends. Clamp the integrator at WINDUP_MAX so its
 *    contribution can't exceed about ±F_MAX.
 *
 *  • Slope feed-forward gain. SLOPE_FEED = 1.0 would cancel slope
 *    perfectly, but real terrain noise (Perlin) means α changes
 *    rapidly; using 0.65 leaves headroom for the controller to
 *    smooth out the residual.
 *
 *  • Falling threshold. Above ~60° lean, the cart-pole equations
 *    are still mathematically valid but the bot can't recover with
 *    bounded F. We stop ticking and show a "FALLEN" banner.
 *
 *  • Ring buffer wrap-around. Phase history wraps at HIST_LEN; older
 *    samples are silently overwritten. Drawing newest-first keeps the
 *    fresh tip of the trajectory bright.
 *
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press 'r' to reset on flat-ish ground. The bot stands upright
 *    immediately. HUD shows θ ≈ 0, F ≈ 0.
 *
 *  • Press 'p' to disable PID. The bot tips over within ~0.5 sec.
 *    That's how unstable an uncontrolled inverted pendulum is.
 *
 *  • Cycle through gain presets with 'g'. Watch:
 *      BALANCED  — clean inward spiral in the phase view.
 *      HIGH Kp   — bot oscillates more, phase trajectory is wider.
 *      NO Kd     — phase trajectory becomes a circle that doesn't
 *                  shrink. Underdamping = perpetual oscillation.
 *      NO Ki     — on a slope, the bot drifts to a steady non-zero
 *                  lean and stays there. Steady-state error.
 *
 *  • Switch to EQUATIONS view ('m'). Read the live numbers; multiply
 *    Kp by e by hand and verify it matches the displayed P term.
 *
 *  • Push drive speed to max with ↑. The bot still balances — slope
 *    feed-forward + PID handles the constantly changing terrain.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. This is the MOST CONTROL-THEORY-HEAVY file in
 *      robots/. Read robots/diff_drive_robot.c first for pose +
 *      command-ramping basics. Read NO other file for the inverted
 *      pendulum, PID, or cart-pole — this file is the canonical
 *      treatment in the project.
 *   2. §6 bot — THE HEART. Sub-sections in order:
 *        - cart-pole physics    ← inverted pendulum equations (T2)
 *        - PID controller       ← three-term feedback (T3)
 *        - tick                 ← integrator + sub-stepping (T6)
 *      Read AFTER tutorials T1-T6 below.
 *   3. §5 terrain — Perlin noise + slope sampling (T5).
 *   4. §7 render — three view modes (TELEMETRY / EQUATIONS / PHASE).
 *   5. §1-§4, §8-§9 — config / clock / colour / noise / screen /
 *      app loop. Skim if seen.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   theta              body lean angle from vertical (rad). 0 =
 *                      perfectly upright; positive = leaning right.
 *   omega              angular velocity = dθ/dt.
 *   theta_eff          theta + terrain slope α. The "effective"
 *                      lean from gravity's perspective.
 *   theta_ref          PID setpoint. Usually 0 (upright) but
 *                      shifts with terrain slope (T5).
 *   error              theta - theta_ref (signed PID input).
 *   integral           accumulated integral term ∫e dt.
 *   prev_error         previous error (for derivative term).
 *   F                  motor force on the cart, computed by PID.
 *   x_acc              horizontal acceleration of cart.
 *   M_eff              effective inertia (M_cart + m_pole·sin²θ).
 *   Kp, Ki, Kd         PID gains.
 *   drive_spd          forward drive speed (px/sec).
 *   world_x            cumulative position along terrain.
 *   alpha              terrain slope at current world_x (rad).
 *   TICK_DT_TARGET     max physics substep size (1/120 sec).
 *
 * Background you need
 * ───────────────────
 *   - diff_drive_robot T2 (Euler integration of pose).
 *   - Newton's second law, gravity as constant downward
 *     acceleration.
 *   - Trig: sin / cos of small angles.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Lagrangian / Hamiltonian mechanics. The cart-pole
 *     equations are presented; we don't derive them from
 *     first principles.
 *   - Optimal control (LQR, MPC). PID is a much simpler
 *     special case.
 *   - State-space / pole placement. PID is "transfer-function
 *     control" — works without ever writing down a state-space
 *     model.
 *   - Real-time embedded programming. Single-threaded soft
 *     real-time at 60 fps is plenty.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Six tutorials that build a self-balancing inverted pendulum
 * robot from first principles.
 *
 *   T1  The inverted pendulum problem — instability of upright
 *   T2  Cart-pole coupling — push the BASE to correct the TOP
 *   T3  PID control — three terms that keep the body upright
 *   T4  Tuning the gains — Kp, Ki, Kd in isolation
 *   T5  Terrain feed-forward — leaning into the hill
 *   T6  Sub-stepping — keeping the integrator stable
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE INVERTED PENDULUM PROBLEM — INSTABILITY OF UPRIGHT
 * ──────────────────────────────────────────────────────────
 * A pendulum hanging DOWN is naturally STABLE. Disturb it and
 * gravity pulls it back to vertical.
 *
 * A pendulum balanced UPRIGHT is naturally UNSTABLE. Disturb
 * it and gravity AMPLIFIES the disturbance — the angle grows
 * exponentially.
 *
 * Math: for a pendulum of length L, gravity g, the angular
 * acceleration is:
 *
 *     θ̈ = (g / L) · sin θ          ← pendulum
 *
 * For an INVERTED pendulum (gravity tipping it AWAY from
 * vertical):
 *
 *     θ̈ = (g / L) · sin θ          ← inverted pendulum (same form!)
 *
 * Wait, same equation? Yes. The difference is the MEANING of θ:
 *
 *   - Hanging pendulum:  θ = 0 means pointing DOWN; restoring.
 *   - Inverted pendulum: θ = 0 means pointing UP; destabilising.
 *
 * For small θ near 0 (upright):
 *
 *     θ̈ ≈ (g / L) · θ              ← exponential growth
 *
 * Solution: θ(t) = θ_0 · e^(√(g/L) · t)
 *
 * For our defaults (g = 32, L = 1.5), the time constant is
 * √(g/L) = √21.3 ≈ 4.6 rad/sec. A 1° initial lean grows to
 * 90° in less than a second. Without control, the bot falls
 * almost instantly.
 *
 * Real-world examples:
 *   - Broom on your palm (T2 analogy)
 *   - Segway / Ninebot (this file's exact problem)
 *   - SpaceX Falcon 9 booster landing (3-D version)
 *   - Bipedal humanoid robots while standing
 *
 * The interesting question: HOW do you stabilise an unstable
 * system? T2 + T3.
 *
 * T2  CART-POLE COUPLING — PUSH THE BASE TO CORRECT THE TOP
 * ─────────────────────────────────────────────────────────
 * The pendulum is mounted on a CART (the wheel axle here).
 * The cart can be pushed left or right with motor force F.
 *
 * KEY INSIGHT: pushing the CART has an OPPOSITE effect on the
 * POLE's lean. If the pole leans right and you push the cart
 * RIGHT, the pole's TOP gets "left behind" by the
 * acceleration and rotates toward vertical.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   Initial lean       Cart pushed right           │
 *      │                                                  │
 *      │      *  ↘             *  ← top "left behind"     │
 *      │      |                |  → rotates back toward   │
 *      │      |                |    vertical              │
 *      │     [O O]→           [O O] ← cart moved          │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * The cart-pole equations (from Lagrangian mechanics, but you
 * can take them on faith):
 *
 *     M_eff = M_cart + m_pole · sin² θ
 *     ẍ = (F + m_pole · sin θ · (L · ω² − g · cos θ)) / M_eff
 *     θ̈ = (g · sin θ − ẍ · cos θ) / L
 *
 * Note θ̈ depends on ẍ (cart acceleration). That's the
 * COUPLING. By choosing F (motor force), we control ẍ, which
 * controls θ̈.
 *
 * The controller's job: choose F SO THAT θ stays near 0.
 *
 * T3  PID CONTROL — THREE TERMS THAT KEEP THE BODY UPRIGHT
 * ────────────────────────────────────────────────────────
 * PID stands for PROPORTIONAL + INTEGRAL + DERIVATIVE. Three
 * terms summed:
 *
 *     error = theta - theta_ref
 *     integral += error · dt
 *     derivative = (error - prev_error) / dt
 *
 *     F = Kp · error + Ki · integral + Kd · derivative
 *
 *     prev_error = error
 *
 * Each term has a distinct role:
 *
 *   PROPORTIONAL (P)
 *     F responds to current lean. If the bot leans 0.1 rad
 *     right, P pushes the cart with force Kp · 0.1.
 *     Higher Kp → snappier response but tendency to overshoot.
 *
 *   INTEGRAL (I)
 *     F accumulates integrated error over time. If a constant
 *     bias (terrain slope, motor offset) keeps the bot
 *     leaning slightly, P alone reaches a steady-state error;
 *     I notices the persistent error and drives it to zero.
 *
 *   DERIVATIVE (D)
 *     F responds to RATE of error change. If lean is small but
 *     INCREASING fast, D acts as damping — pushes harder
 *     before the lean grows. Critical for stability.
 *
 * The output F is then APPLIED to the cart as motor force,
 * which becomes ẍ via the cart-pole equations (T2), which
 * affects θ̈, which (over time) reduces the error.
 *
 * Together: P pushes proportional to current state, I removes
 * persistent biases, D damps rapid changes. Three knobs;
 * tuning them is an art (T4).
 *
 * PID is the WORKHORSE of industrial control. Cars
 * (cruise control), ovens (temperature), robots (joint
 * angles), HVAC, drones — almost everything that has a
 * setpoint and feedback uses PID or a close variant.
 *
 * T4  TUNING THE GAINS — Kp, Ki, Kd IN ISOLATION
 * ──────────────────────────────────────────────
 * The 'g' key cycles four GAIN PRESETS. Watch each carefully:
 *
 *   BALANCED          all three gains tuned. Bot stays upright
 *                     across slopes. The default.
 *
 *   HIGH-Kp           huge proportional term, no I, no D.
 *                     Bot oscillates wildly — overshoots in
 *                     each direction. Pure P creates a
 *                     pendulum-like ringing.
 *
 *   NO-Kd             P + I but no derivative. No damping.
 *                     Bot oscillates with growing amplitude;
 *                     usually falls within seconds. Why D
 *                     matters.
 *
 *   NO-Ki             P + D but no integral. Bot maintains
 *                     a steady-state lean (especially on
 *                     slopes). Slope feed-forward partly
 *                     compensates but I is still needed for
 *                     residual bias.
 *
 * General tuning recipe (Ziegler-Nichols, simplified):
 *
 *     1. Start with all gains at zero.
 *     2. Slowly increase Kp until the system OSCILLATES
 *        (just barely). Call this Kp_crit.
 *     3. Set Kp = 0.6 · Kp_crit, Kd = Kp · period / 8,
 *        Ki = Kp / period.
 *
 * Real engineers use SIMULATION + TUNING TOOLS (MATLAB,
 * model-predictive optimisation). For this file the gains
 * were hand-tuned by trial and error.
 *
 * 'p' key TOGGLES the entire PID off — watch the bot fall.
 * Confirms it's the controller doing the work, not magic.
 *
 * T5  TERRAIN FEED-FORWARD — LEANING INTO THE HILL
 * ────────────────────────────────────────────────
 * On flat ground, the bot wants to be vertical (θ_ref = 0).
 *
 * On a SLOPE, vertical is the WRONG target. The terrain
 * pushes the bot's wheels along the slope; the body needs to
 * lean INTO the hill to stay aligned with the gravitational
 * vertical, not the local-up-of-the-chassis.
 *
 * Solution: shift the SETPOINT by the terrain slope α:
 *
 *     theta_ref = -SLOPE_FEED · α
 *
 * SLOPE_FEED is a tuning constant (~1.0 means the bot leans
 * exactly with the slope; <1 less responsive; >1 over-leans).
 *
 * Now the PID's error becomes:
 *
 *     error = theta - theta_ref
 *           = theta - (-SLOPE_FEED · α)
 *           = theta + SLOPE_FEED · α
 *
 * On flat ground (α = 0), this reduces to the standard
 * "lean = error". On a slope, the controller's reference
 * shifts — it WANTS the bot to lean with the slope, so the
 * lean is "expected" rather than treated as error.
 *
 * This is FEED-FORWARD CONTROL — using known information
 * (terrain slope from sensor / map) to anticipate the
 * disturbance instead of reacting to it after it shows up
 * in the error. Common in precision robotics: factor out
 * predictable disturbances so the feedback only handles the
 * unpredictable residual.
 *
 * Without slope feed-forward, the integral term eventually
 * compensates for slope-induced lean — but slowly, and with
 * lag. The feed-forward makes slope-handling INSTANTANEOUS.
 *
 * T6  SUB-STEPPING — KEEPING THE INTEGRATOR STABLE
 * ────────────────────────────────────────────────
 * The cart-pole physics is STIFF — the time constant is
 * ~0.2 sec. Forward Euler with a 1/60 sec timestep is OK on
 * flat ground, but at fast drive speed and sharp slopes it
 * can BLOW UP — θ overshoots wildly between integration
 * steps and the controller can't catch up.
 *
 * Solution: SUB-STEPPING. Each frame's dt is divided into
 * smaller substeps:
 *
 *     n_substeps = max(1, ceil(dt / TICK_DT_TARGET))
 *     sub_dt = dt / n_substeps
 *
 *     for _ in n_substeps:
 *       compute_error_and_F(sub_dt)
 *       integrate_cart_pole(sub_dt)
 *
 * TICK_DT_TARGET = 1/120 sec; a 60 fps frame would split
 * into 2 substeps; a 30 fps frame into 4.
 *
 * Each substep is a SHORTER, more accurate Euler step. The
 * controller and physics both run at the smaller timestep,
 * so feedback is tight even when the renderer is slow.
 *
 * Cost: 2-4× more PID + integration calls per frame. At
 * O(20) ops per substep that's nothing.
 *
 * Why TICK_DT_TARGET = 1/120? It's slightly under the time
 * constant of the closed-loop system (controller + physics).
 * Standard rule: SAMPLE AT LEAST 5× FASTER than the system's
 * natural frequency. Our balance ringing is ~3-5 Hz, so we
 * need ≥ 25 Hz sampling — 120 Hz is comfortable margin.
 *
 * Same sub-stepping pattern is used in any STIFF physics
 * simulator: cloth (Verlet with iterations), constraint
 * solvers (Jakobsen), spring-mass chains. Whenever the
 * dynamics are faster than the renderer, split dt.
 *
 * Decision tree for "is sub-stepping needed?":
 *
 *   non-stiff sim (matrix_rain, FK creatures)     → no
 *   stiff but cheap (springs, ragdoll)            → fixed-step
 *                                                    accumulator
 *   STIFF + complex (cart-pole, Verlet cloth)     → sub-stepping
 *                                                    (this file)
 *
 * For balance + procedural terrain, sub-stepping is the
 * cheapest defence.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* ── §1.1 frame rate + sub-step target ────────────────────────────── */
enum {
    TARGET_FPS = 60,
};

/*
 * TICK_DT_TARGET — desired physics step. Each frame is split into
 * sub-steps no larger than this so the cart-pole integrator stays
 * stable even at slow frame rates. 1/120 sec is small enough for
 * the default gains to be well within the stability region.
 *
 * MAX_SUBSTEPS — cap so a long-stalled frame doesn't loop forever.
 */
#define TICK_DT_TARGET   (1.0f / 120.0f)
#define MAX_SUBSTEPS     16

/* ── §1.2 cell pixel dimensions ──────────────────────────────────── */
/* Terminal cells are roughly 2× tall as wide (8 × 16 px). Physics
 * is in pixel space; cell conversion happens at draw time. */
#define CELL_W   8
#define CELL_H  16

/* ── §1.3 cart-pole physics (SI) ─────────────────────────────────── */
/*
 * GRAVITY   — m/s²; standard Earth value.
 * PEND_LEN  — m; rod length from axle to body centre-of-mass.
 *             Longer rod = slower oscillation (τ ∝ √(L/g)).
 * MASS_CART — kg; effective mass of chassis + wheels.
 * MASS_POLE — kg; mass of the body above the axle.
 * MAX_FORCE — N; motor force clamp (saturation).
 *             80 N keeps "everything bounded" for our default
 *             pendulum geometry.
 * FALL_ANGLE — rad; "fallen over" threshold (~60°). At larger
 *              angles even unbounded F couldn't recover quickly,
 *              so we just stop ticking.
 */
#define GRAVITY      9.81f
#define PEND_LEN     1.00f
#define MASS_CART    4.00f
#define MASS_POLE    2.00f
#define MAX_FORCE  200.00f
#define FALL_ANGLE   1.05f

/* ── §1.4 PID defaults + anti-windup ─────────────────────────────── */
/*
 * KP_DEF — proportional gain. Larger = faster correction, more overshoot.
 * KI_DEF — integral gain. Larger = faster drift removal but risk of windup.
 * KD_DEF — derivative gain. Larger = more damping, slower response.
 * WINDUP_MAX — clamp on the integrator's accumulated value (rad·sec).
 *             Prevents unbounded growth on sustained errors.
 */
#define KP_DEF      120.0f
#define KI_DEF        0.20f
#define KD_DEF       18.0f
#define WINDUP_MAX    5.00f

/* ── §1.5 slope feed-forward ─────────────────────────────────────── */
/*
 * SLOPE_FEED — fraction of terrain slope α fed to θ_ref.
 *   1.0 cancels slope perfectly in theory but amplifies high-frequency
 *   slope changes; 0.65 leaves headroom for the PID to smooth residuals.
 */
#define SLOPE_FEED   0.65f

/* ── §1.6 robot geometry (pixels) ────────────────────────────────── */
#define WHEEL_R      18.0f      /* wheel radius (only used for axle height) */
#define AXLE_HW      26.0f      /* half the axle width (wheel separation) */
#define BODY_H       96.0f      /* body length axle → top mass */

/* ── §1.7 drive speed cycle ──────────────────────────────────────── */
#define DRIVE_DEF    55.0f
#define DRIVE_STEP   15.0f
#define DRIVE_MAX   160.0f
#define DRIVE_MIN     0.0f
#define PIX_PER_M   100.0f     /* HUD scale: pixels per metre  */

/* ── §1.8 terrain ─────────────────────────────────────────────────── */
/*
 * TBUF       — power of two; ring-buffer size for terrain heights.
 * T_FREQ     — Perlin frequency per world column. Smaller = bigger hills.
 * T_AMP_F    — amplitude as fraction of screen height.
 * T_MID_F    — vertical placement of the terrain centre line.
 */
#define TBUF       1024
#define TMASK      (TBUF - 1)
#define T_FREQ      0.022f
#define T_AMP_F     0.20f
#define T_MID_F     0.62f

/* ── §1.9 phase-history ring buffer ─────────────────────────────── */
enum { HIST_LEN = 240 };

/* ── §1.10 view modes ────────────────────────────────────────────── */
typedef enum {
    VIEW_TELEMETRY = 0,
    VIEW_EQUATIONS,
    VIEW_PHASE,
    VIEW_COUNT
} ViewMode;

static const char *VIEW_NAMES[VIEW_COUNT] = {
    "TELEMETRY", "EQUATIONS", "PHASE-SPACE"
};

/* ── §1.11 dt cap (spiral-of-death guard) ────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.12 ncurses pair IDs ──────────────────────────────────────── */
enum {
    /* 1..2 — sky & stars */
    CP_SKY      = 1,
    CP_STAR,
    /* 3..4 — terrain */
    CP_SURF,
    CP_ROCK,
    /* 5..7 — robot */
    CP_CHASSIS,
    CP_WHEEL,
    CP_BEACON,
    /* 8..10 — UI */
    CP_DIM,             /* dim text                            */
    CP_GOOD,            /* green positive indicator            */
    CP_WARN,            /* red warning                         */
    /* 11..12 — value text */
    CP_VAL,             /* live numeric values                 */
    CP_EQ,              /* equation labels                     */
    /* 13..14 — bar gauge */
    CP_BAR_POS,
    CP_BAR_NEG,
    /* 15..16 — HUD spec */
    PAIR_HUD,
    PAIR_HINT,
};

/* ── §1.13 timing primitives ─────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.14 HUD layout ────────────────────────────────────────────── */
#define HUD_BUF_LEN  160
#define PANEL_W       32

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

/*
 * Each visual element gets one fixed semantic colour. Layout:
 *
 *   sky background      dark blue
 *   stars               bright yellow (sparse)
 *   terrain surface     bright green
 *   terrain rock fill   dim green
 *   robot chassis       white
 *   robot wheel         cyan
 *   beacon              red (warning-ish; tip of body, easy to spot)
 *   dim text            grey
 *   stable indicator    bright green
 *   warning indicator   bright red
 *   live values         light yellow
 *   equation labels     light cyan
 *   bar pos             cyan
 *   bar neg             magenta
 *
 * HUD pairs (PAIR_HUD, PAIR_HINT) bind separately on default
 * terminal background per CLAUDE.md HUD spec.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_SKY,       25,  -1);    /* dark blue          */
        init_pair(CP_STAR,     226,  -1);    /* yellow             */
        init_pair(CP_SURF,      46,  -1);    /* bright green       */
        init_pair(CP_ROCK,      28,  -1);    /* dim green          */
        init_pair(CP_CHASSIS,  255,  -1);    /* near-white         */
        init_pair(CP_WHEEL,     51,  -1);    /* cyan               */
        init_pair(CP_BEACON,   196,  -1);    /* red                */
        init_pair(CP_DIM,      244,  -1);    /* grey               */
        init_pair(CP_GOOD,      82,  -1);    /* lime               */
        init_pair(CP_WARN,     196,  -1);    /* red                */
        init_pair(CP_VAL,      229,  -1);    /* pale yellow        */
        init_pair(CP_EQ,       159,  -1);    /* light cyan         */
        init_pair(CP_BAR_POS,   51,  -1);    /* cyan               */
        init_pair(CP_BAR_NEG,  201,  -1);    /* magenta            */
        init_pair(PAIR_HUD,    226,  -1);    /* yellow on default  */
        init_pair(PAIR_HINT,    51,  -1);    /* cyan on default    */
    } else {
        init_pair(CP_SKY,      COLOR_BLUE,    -1);
        init_pair(CP_STAR,     COLOR_YELLOW,  -1);
        init_pair(CP_SURF,     COLOR_GREEN,   -1);
        init_pair(CP_ROCK,     COLOR_GREEN,   -1);
        init_pair(CP_CHASSIS,  COLOR_WHITE,   -1);
        init_pair(CP_WHEEL,    COLOR_CYAN,    -1);
        init_pair(CP_BEACON,   COLOR_RED,     -1);
        init_pair(CP_DIM,      COLOR_WHITE,   -1);
        init_pair(CP_GOOD,     COLOR_GREEN,   -1);
        init_pair(CP_WARN,     COLOR_RED,     -1);
        init_pair(CP_VAL,      COLOR_YELLOW,  -1);
        init_pair(CP_EQ,       COLOR_CYAN,    -1);
        init_pair(CP_BAR_POS,  COLOR_CYAN,    -1);
        init_pair(CP_BAR_NEG,  COLOR_MAGENTA, -1);
        init_pair(PAIR_HUD,    COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,   COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §4  noise — Perlin 1-D + fBm                                           */
/* ===================================================================== */

/*
 * Perlin gradient noise over a permutation table. We use a simple
 * 1-D variant with quintic smoothstep:
 *      fade(t) = 6t⁵ − 15t⁴ + 10t³
 * which is C²-continuous, so the resulting surface has continuous
 * curvature (no visible kinks).
 */
static unsigned char g_perm[512];

static void perlin_init(unsigned int seed)
{
    unsigned char p[256];
    srand(seed);
    for (int i = 0; i < 256; i++) p[i] = (unsigned char)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        unsigned char t = p[i]; p[i] = p[j]; p[j] = t;
    }
    for (int i = 0; i < 512; i++) g_perm[i] = p[i & 255];
}

static inline float fade (float t) { return t*t*t*(t*(t*6.0f - 15.0f) + 10.0f); }
static inline float lerp (float a, float b, float t) { return a + t * (b - a); }
static inline float grad1(int h, float x) { return (h & 1) ? x : -x; }

static float perlin1(float x)
{
    int   xi = (int)floorf(x) & 255;
    float xf = x - floorf(x);
    return lerp(grad1(g_perm[xi],     xf       ),
                grad1(g_perm[xi + 1], xf - 1.0f),
                fade(xf));
}

/*
 * Fractional Brownian Motion — sum of 5 perlin octaves at increasing
 * frequency and decreasing amplitude. Result is in [-1, +1] roughly.
 * Each octave doubles frequency and halves amplitude — the classic
 * 1/f spectrum that gives nature-like terrain.
 */
static float fbm(float x)
{
    float v = 0.0f, amp = 0.5f, freq = 1.0f;
    for (int i = 0; i < 5; i++) {
        v   += amp * perlin1(x * freq);
        amp *= 0.5f;
        freq *= 2.0f;
    }
    return v;
}

/* ===================================================================== */
/* §5  terrain — height query, slope, ring buffer                         */
/* ===================================================================== */

/*
 * Terrain heights are stored in a ring buffer indexed by world
 * column. As the bot moves right, new columns are filled lazily
 * by `terrain_ensure`.
 */
typedef struct {
    float h[TBUF];   /* surface height (px from screen top)     */
    int   gen_col;   /* highest world column already populated  */
    int   rows;      /* terminal rows; needed for amplitude     */
} Terrain;

/* World y of the surface at a given world column. */
static float terrain_h_at(int wc, int rows)
{
    float mid = (float)rows * T_MID_F * (float)CELL_H;
    float amp = (float)rows * T_AMP_F * (float)CELL_H;
    return mid + fbm((float)wc * T_FREQ) * amp;
}

/*
 * Terrain slope at a world column, in radians.
 *
 *   slope = atan2(Δh, Δx)   — central or forward difference would
 *                              both work; we use forward difference
 *                              for the cached buffer (h[c+1] − h[c]).
 *
 *   Sign convention in screen-y (which is positive DOWN):
 *     dh > 0 → terrain falls to the right → slope > 0 (descending)
 *     dh < 0 → terrain rises to the right → slope < 0 (ascending)
 *   We negate to match the cart-pole convention (positive = uphill
 *   to the right) so the controller sees an intuitive sign.
 */
static float terrain_slope_at(const Terrain *t, int wc)
{
    float dh = t->h[(wc + 1) & TMASK] - t->h[wc & TMASK];
    return -atanf(dh / (float)CELL_W);
}

static void terrain_ensure(Terrain *t, int upto)
{
    for (int c = t->gen_col + 1; c <= upto; c++)
        t->h[c & TMASK] = terrain_h_at(c, t->rows);
    if (upto > t->gen_col) t->gen_col = upto;
}

static void terrain_init(Terrain *t, int rows, int cols)
{
    t->rows    = rows;
    t->gen_col = -1;
    terrain_ensure(t, cols + 64);
}

/* ===================================================================== */
/* §6  bot — Bot type, PID, cart-pole physics, init/reset                 */
/* ===================================================================== */

/* ── §6.1 GainPreset (for the 'g' key teaching cycle) ─────────────── */

typedef struct {
    const char *name;
    const char *lesson_l1;
    const char *lesson_l2;
    float       kp, ki, kd;
} GainPreset;

static const GainPreset PRESETS[] = {
    { "BALANCED",
      "well-tuned baseline.",
      "fast, damped, slope-aware.",
      120.0f, 0.20f, 18.0f },

    { "HIGH Kp ",
      "stiffer P → more overshoot",
      "& oscillation on slopes.",
      240.0f, 0.20f, 18.0f },

    { "NO Kd   ",
      "no damping → underdamped:",
      "bot oscillates forever.",
      120.0f, 0.20f,  0.0f },

    { "NO Ki   ",
      "no integral → steady-state",
      "drift on every slope.",
      120.0f, 0.00f, 18.0f },
};
#define N_PRESETS  (int)(sizeof PRESETS / sizeof PRESETS[0])

/* ── §6.2 Bot type ───────────────────────────────────────────────── */

/*
 * Bot — the entire simulation state.
 *
 *   Pendulum state (the unknowns):
 *     theta       lean angle from chassis-perpendicular (rad)
 *     omega       angular velocity θ̇ (rad/sec)
 *
 *   Cart-pole derived (computed each substep, exposed for display):
 *     theta_eff   θ + α — lean from gravity
 *     theta_ref   PID setpoint = -SLOPE_FEED · α (lean into slope)
 *     theta_ddot  θ̈ this step
 *     x_ddot      ẍ this step (cart's horizontal accel)
 *     M_eff       effective inertia M_cart + m_pole · sin²θ_eff
 *     F           motor force this step
 *
 *   PID state:
 *     kp, ki, kd   gains
 *     pid_int      integrator accumulator
 *     pid_prev_err previous frame's error (for derivative)
 *     pid_p, pid_i, pid_d, pid_out  per-term outputs (display only)
 *     pid_on       toggle (off → bot tips immediately)
 *
 *   Motion / world:
 *     world_x      pixel-space position along the terrain
 *     drive_spd    forward speed in px/sec
 *     spin_angle   wheel spin (cosmetic; unused now)
 *     alpha        current terrain slope at foot (rad)
 *
 *   Phase ring buffer: last HIST_LEN (θ, ω) samples for the phase
 *     portrait. Newest at idx (head − 1).
 *
 *   UI:
 *     paused       freeze ticks
 *     fallen       above FALL_ANGLE; stop ticking, show banner
 *     dist_m       cumulative travel in metres (HUD)
 *     view         which side panel is visible
 *     preset_idx   which gain preset is active
 */
typedef struct {
    float theta, omega;

    float theta_eff, theta_ref;
    float theta_ddot, x_ddot, M_eff, F;

    float kp, ki, kd;
    float pid_int, pid_prev_err;
    float pid_p, pid_i, pid_d, pid_out;
    bool  pid_on;

    float world_x, drive_spd;
    float spin_angle;
    float alpha;

    float ph_theta[HIST_LEN];
    float ph_omega[HIST_LEN];
    int   ph_head, ph_fill;

    bool     paused, fallen;
    float    dist_m;
    ViewMode view;
    int      preset_idx;
} Bot;

/* ── §6.3 PID controller (single function — easy to read) ──────────── */

/*
 * Compute the motor force from the current lean error.
 *
 *   e          = θ − θ_ref               error
 *   pid_int   += e · dt                  integral (clamped)
 *   de_dt      = (e − e_prev) / dt       derivative
 *   F          = Kp·e + Ki·∫e + Kd·de_dt clamped to ±MAX_FORCE
 *
 * The three term outputs are stored separately so the equations
 * view can show them broken out.
 */
static void pid_step(Bot *b, float dt)
{
    if (!b->pid_on) {
        b->pid_p = b->pid_i = b->pid_d = b->pid_out = 0.0f;
        b->F = 0.0f;
        return;
    }

    float err = b->theta - b->theta_ref;

    b->pid_int += err * dt;
    if (b->pid_int >  WINDUP_MAX) b->pid_int =  WINDUP_MAX;
    if (b->pid_int < -WINDUP_MAX) b->pid_int = -WINDUP_MAX;

    float deriv = (dt > 1e-9f) ? (err - b->pid_prev_err) / dt : 0.0f;
    b->pid_prev_err = err;

    b->pid_p   = b->kp * err;
    b->pid_i   = b->ki * b->pid_int;
    b->pid_d   = b->kd * deriv;
    b->pid_out = b->pid_p + b->pid_i + b->pid_d;

    if (b->pid_out >  MAX_FORCE) b->pid_out =  MAX_FORCE;
    if (b->pid_out < -MAX_FORCE) b->pid_out = -MAX_FORCE;
    b->F = b->pid_out;
}

/* ── §6.4 cart-pole dynamics (Lagrangian) ───────────────────────────── */

/*
 * One forward-Euler step of the cart-pole equations on a sloped
 * surface. See MENTAL MODEL → CART-POLE EQUATIONS for derivation.
 *
 * Inputs:  θ, ω, F      (and the current slope α via theta_eff)
 * Outputs: ẍ, θ̈ stored in b for display; ω, θ updated in place.
 */
static void cart_pole_step(Bot *b, float dt)
{
    float st = sinf(b->theta_eff);
    float ct = cosf(b->theta_eff);

    b->M_eff      =  MASS_CART + MASS_POLE * st * st;
    b->x_ddot     = (b->F + MASS_POLE * st *
                     (PEND_LEN * b->omega * b->omega - GRAVITY * ct))
                    / b->M_eff;
    b->theta_ddot = (GRAVITY * st - b->x_ddot * ct) / PEND_LEN;

    b->omega += b->theta_ddot * dt;
    b->theta += b->omega      * dt;
}

/* ── §6.5 phase history push ──────────────────────────────────────── */

static void phase_push(Bot *b)
{
    b->ph_theta[b->ph_head] = b->theta;
    b->ph_omega[b->ph_head] = b->omega;
    b->ph_head = (b->ph_head + 1) % HIST_LEN;
    if (b->ph_fill < HIST_LEN) b->ph_fill++;
}

/* ── §6.6 bot_substep — one physics step (called N times per frame) ── */

/*
 * Runs the physics pipeline for one timestep `sub_dt`:
 *   1. Move forward along terrain at drive_spd.
 *   2. Sample slope α at the new world position.
 *   3. Compute θ_eff and the slope-aware setpoint θ_ref.
 *   4. Run PID → motor force F.
 *   5. Run cart-pole dynamics → update θ, ω.
 *   6. Push (θ, ω) into the phase history.
 *   7. Detect fall (|θ_eff| > FALL_ANGLE).
 */
static void bot_substep(Bot *b, const Terrain *t, float sub_dt)
{
    if (b->fallen) return;

    /* 1. advance along terrain */
    b->world_x += b->drive_spd * sub_dt;
    b->dist_m  += b->drive_spd * sub_dt / PIX_PER_M;
    int wc = (int)(b->world_x / (float)CELL_W);

    /* 2. sample slope */
    b->alpha = terrain_slope_at(t, wc);

    /* 3. effective angle + slope-aware setpoint */
    b->theta_eff = b->theta + b->alpha;
    b->theta_ref = -b->alpha * SLOPE_FEED;

    /* spin angle (cosmetic; not currently rendered) */
    b->spin_angle += (b->drive_spd / WHEEL_R) * sub_dt;

    /* 4. PID → F */
    pid_step(b, sub_dt);

    /* 5. cart-pole dynamics → updated θ, ω */
    cart_pole_step(b, sub_dt);

    /* 6. record phase trajectory */
    phase_push(b);

    /* 7. fall test */
    if (fabsf(b->theta_eff) > FALL_ANGLE) b->fallen = true;
}

/* ── §6.7 init / reset / preset ──────────────────────────────────── */

static void bot_apply_preset(Bot *b, int idx)
{
    const GainPreset *p = &PRESETS[idx];
    b->kp = p->kp;
    b->ki = p->ki;
    b->kd = p->kd;
    b->pid_int = 0.0f;          /* clear integral on gain change */
}

static void bot_reset(Bot *b)
{
    /* Preserve user-adjustable settings across reset. */
    bool     pid_on = b->pid_on;
    float    drive  = b->drive_spd;
    ViewMode view   = b->view;
    int      preset = b->preset_idx;

    memset(b, 0, sizeof *b);

    b->pid_on      = pid_on;
    b->drive_spd   = drive;
    b->view        = view;
    b->preset_idx  = preset;

    /* Tiny initial lean so the controller has something to work with. */
    b->theta = 0.04f;

    bot_apply_preset(b, preset);
}

static void bot_init(Bot *b)
{
    memset(b, 0, sizeof *b);
    b->pid_on      = true;
    b->drive_spd   = DRIVE_DEF;
    b->view        = VIEW_TELEMETRY;
    b->preset_idx  = 0;
    b->theta       = 0.04f;
    bot_apply_preset(b, 0);
}

/* ===================================================================== */
/* §7  render                                                             */
/* ===================================================================== */

/* ── §7.1 helpers ────────────────────────────────────────────────── */

static inline int   px_to_cx (float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cy (float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline bool  in_screen(int r, int c, int rows, int cols)
{
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

static void put_ch(int r, int c, chtype ch, attr_t a, int cp,
                   int rows, int cols)
{
    if (!in_screen(r, c, rows, cols)) return;
    attron (a | COLOR_PAIR(cp));
    mvaddch(r, c, ch);
    attroff(a | COLOR_PAIR(cp));
}

static void put_str(int r, int c, const char *s, attr_t a, int cp,
                    int rows, int cols)
{
    if (r < 0 || r >= rows) return;
    attron (a | COLOR_PAIR(cp));
    for (int i = 0; s[i] && c + i < cols; i++)
        mvaddch(r, c + i, (unsigned char)s[i]);
    attroff(a | COLOR_PAIR(cp));
}

/*
 * Bresenham line picker — chooses a glyph based on segment slope so
 * a long line "looks line-like" without per-cell trig.
 */
static chtype seg_glyph(int dr, int dc)
{
    if (dr == 0) return '-';
    if (dc == 0) return '|';
    return (dr * dc < 0) ? '/' : '\\';
}

static void draw_line(float x0, float y0, float x1, float y1,
                      attr_t a, int cp, int rows, int cols)
{
    int r0 = px_to_cy(y0), c0 = px_to_cx(x0);
    int r1 = px_to_cy(y1), c1 = px_to_cx(x1);
    int dr = r1 - r0, dc = c1 - c0;
    chtype ch = seg_glyph(dr, dc);
    int sr = (r0 < r1) ? 1 : -1;
    int sc = (c0 < c1) ? 1 : -1;
    int err = abs(dr) - abs(dc);
    int r = r0, c = c0;
    for (;;) {
        put_ch(r, c, ch, a, cp, rows, cols);
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -abs(dc)) { err -= abs(dc); r += sr; }
        if (e2 <  abs(dr)) { err += abs(dr); c += sc; }
    }
}

/* Centred horizontal bar gauge in [-1, +1]. */
static void draw_bar(int r, int c, int W, float v,
                     int cp_pos, int cp_neg, int rows, int cols)
{
    int half = W / 2;
    int mid  = c + half;
    int fill = (int)(fabsf(v) * (float)half);
    if (fill > half) fill = half;

    /* track */
    attron(COLOR_PAIR(CP_DIM));
    for (int i = 0; i < W; i++) put_ch(r, c + i, ' ', 0, CP_DIM, rows, cols);
    put_ch(r, mid, '|', 0, CP_DIM, rows, cols);
    attroff(COLOR_PAIR(CP_DIM));

    /* fill */
    int cp = (v >= 0.0f) ? cp_pos : cp_neg;
    attron(COLOR_PAIR(cp) | A_BOLD);
    if (v >= 0.0f) for (int i = 0; i < fill; i++) put_ch(r, mid + 1 + i, '=', A_BOLD, cp, rows, cols);
    else            for (int i = 0; i < fill; i++) put_ch(r, mid - 1 - i, '=', A_BOLD, cp, rows, cols);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

/* ── §7.2 render_terrain — sky, surface, rock fill ───────────────── */

static bool is_star(int r, int c)
{
    /* Cheap deterministic hash so stars stay in fixed cells. */
    unsigned h = (unsigned)(r * 7919 + c * 6271);
    h ^= h >> 13; h *= 0x45d9f3bU; h ^= h >> 17;
    return (h % 60) == 0;
}

static chtype surface_glyph(float dh)
{
    if (dh > (float)CELL_W * 0.22f)  return '/';
    if (dh < -(float)CELL_W * 0.22f) return '\\';
    return '_';
}

/*
 * For each on-screen column:
 *   - Sky rows render '.' stars at sparse hash-selected cells.
 *   - Surface row gets a slope-shaped glyph ( _ / \ ).
 *   - Texture row directly below alternates ':' '.' for grass feel.
 *   - Rows further below are the rock fill ('#' alternating).
 */
static void render_terrain(const Terrain *t, int bot_wc, int bot_sc,
                           int rows, int cols)
{
    for (int sc = 0; sc < cols; sc++) {
        int   wc  = bot_wc - bot_sc + sc;
        if (wc < 0) wc = 0;
        float h   = t->h[wc & TMASK];
        float dh  = t->h[(wc + 1) & TMASK] - h;
        int   surf = px_to_cy(h);
        if (surf < 1)        surf = 1;
        if (surf > rows - 2) surf = rows - 2;

        chtype sg = surface_glyph(dh);

        for (int r = 0; r < rows; r++) {
            if (r < surf) {
                if (is_star(r, sc))
                    put_ch(r, sc, '.', A_BOLD, CP_STAR, rows, cols);
            } else if (r == surf) {
                int cp = (fabsf(dh) > (float)CELL_W * 0.22f) ? CP_ROCK : CP_SURF;
                put_ch(r, sc, sg, A_BOLD, cp, rows, cols);
            } else if (r == surf + 1) {
                put_ch(r, sc, (sc % 3 == 0) ? ':' : '.', A_DIM, CP_SURF, rows, cols);
            } else {
                put_ch(r, sc, (sc % 2 == 0) ? '#' : ' ', A_DIM, CP_ROCK, rows, cols);
            }
        }
    }
}

/* ── §7.3 render_bot — clean wheels + tilted body line ───────────── */

/*
 * The robot is intentionally minimal:
 *
 *      *           ← beacon (top of body)
 *       \
 *        \         ← body line (tilts with θ_eff = θ + α)
 *         \
 *        O═O       ← chassis '═' between two 'O' wheels
 *
 * Total: 6 cells of body + 1 beacon + 3 chassis = ~10 cells.
 * Lean is shown by the body line's actual angle in cell space.
 */
static void render_bot(const Bot *b, const Terrain *t, int bot_sc,
                       int rows, int cols)
{
    int   wc    = (int)(b->world_x / (float)CELL_W);
    float h_px  = t->h[wc & TMASK];

    /* Axle in pixel space — just above the surface. */
    float ax_px = (float)(bot_sc * CELL_W);
    float ax_py = h_px - WHEEL_R;

    /* Wheels offset along the slope direction so the chassis sits
     * "flat" on the terrain. */
    float ca = cosf(b->alpha), sa = sinf(b->alpha);
    float lx_px = ax_px - AXLE_HW * ca;
    float ly_px = ax_py + AXLE_HW * sa;
    float rx_px = ax_px + AXLE_HW * ca;
    float ry_px = ax_py - AXLE_HW * sa;

    /* Body top in pixel space — tilt by θ_eff. */
    float te    = b->theta_eff;
    float bx_px = ax_px + BODY_H * sinf(te);
    float by_px = ax_py - BODY_H * cosf(te);

    /* (1) chassis line — always between the two wheels. */
    draw_line(lx_px, ly_px, rx_px, ry_px, A_BOLD, CP_CHASSIS, rows, cols);

    /* (2) body line from axle to body top. */
    draw_line(ax_px, ax_py, bx_px, by_px, A_BOLD, CP_CHASSIS, rows, cols);

    /* (3) wheels — single 'O' each. */
    int lcx = px_to_cx(lx_px), lcy = px_to_cy(ly_px);
    int rcx = px_to_cx(rx_px), rcy = px_to_cy(ry_px);
    put_ch(lcy, lcx, 'O', A_BOLD, CP_WHEEL, rows, cols);
    put_ch(rcy, rcx, 'O', A_BOLD, CP_WHEEL, rows, cols);

    /* (4) beacon at body top — '*' that pulses slightly so a learner
     * can easily track "the top of the body" while it leans. */
    int bcx = px_to_cx(bx_px), bcy = px_to_cy(by_px);
    attr_t pulse = (((int)(b->dist_m * 4.0f)) & 1) ? A_BOLD : A_DIM;
    put_ch(bcy, bcx, '*', pulse, CP_BEACON, rows, cols);

    /* (5) lean readout — small floating number near the axle. */
    {
        float deg = b->theta_eff * (180.0f / (float)M_PI);
        int   ar  = px_to_cy(ax_py);
        int   ac  = px_to_cx(ax_px);
        int   cp  = (fabsf(deg) > 20.0f) ? CP_WARN : CP_GOOD;
        char  buf[16];
        snprintf(buf, sizeof buf, "%+5.1f°", deg);
        attron (COLOR_PAIR(cp));
        mvprintw(ar, ac - 6, "%s", buf);
        attroff(COLOR_PAIR(cp));
    }

    /* Fallen banner — overlay middle of screen. */
    if (b->fallen) {
        int mr = rows / 2;
        int mc = cols / 2 - 9;
        if (mc < 0) mc = 0;
        attron (COLOR_PAIR(CP_WARN) | A_BOLD | A_BLINK);
        mvprintw(mr, mc, "  !! FALLEN !!  ");
        attroff(COLOR_PAIR(CP_WARN) | A_BOLD | A_BLINK);
        attron (COLOR_PAIR(CP_DIM));
        mvprintw(mr + 1, mc - 1, "g=preset  r=reset");
        attroff(COLOR_PAIR(CP_DIM));
    }
}

/* ── §7.4 render_panel_telemetry — live numbers + bar gauges ──────── */

static void render_panel_telemetry(const Bot *b, int rows, int cols, int c0)
{
    int r = 0;
    int bw = 14;
    int bc = c0 + 16;
    char buf[64];

    put_str(r++, c0, " TELEMETRY              ", A_BOLD, CP_VAL, rows, cols);

    {
        float deg = b->theta_eff * (180.0f / (float)M_PI);
        int   cp  = fabsf(deg) > 20.0f ? CP_WARN : CP_GOOD;
        snprintf(buf, sizeof buf, " theta_eff %+6.2f° ", deg);
        put_str(r, c0, buf, A_BOLD, cp, rows, cols);
        draw_bar(r, bc, bw, deg / 35.0f, CP_BAR_POS, CP_BAR_NEG, rows, cols);
        r++;
    }
    {
        snprintf(buf, sizeof buf, " omega    %+6.2f  ", b->omega);
        put_str(r, c0, buf, A_NORMAL, CP_VAL, rows, cols);
        draw_bar(r, bc, bw, b->omega / 6.0f, CP_BAR_POS, CP_BAR_NEG, rows, cols);
        r++;
    }
    {
        float sdeg = b->alpha * (180.0f / (float)M_PI);
        int   cp   = fabsf(sdeg) > 15.0f ? CP_WARN : CP_VAL;
        snprintf(buf, sizeof buf, " slope α  %+6.2f° ", sdeg);
        put_str(r, c0, buf, A_NORMAL, cp, rows, cols);
        draw_bar(r, bc, bw, sdeg / 25.0f, CP_BAR_NEG, CP_BAR_POS, rows, cols);
        r++;
    }
    {
        float mps = b->drive_spd / PIX_PER_M;
        snprintf(buf, sizeof buf, " spd  %+5.2f m/s     ", mps);
        put_str(r++, c0, buf, A_BOLD, CP_VAL, rows, cols);
    }
    {
        snprintf(buf, sizeof buf, " dist %7.1f m       ", b->dist_m);
        put_str(r++, c0, buf, A_NORMAL, CP_VAL, rows, cols);
    }

    r++;
    put_str(r++, c0, " PID OUTPUTS            ", A_BOLD, CP_VAL, rows, cols);

    if (b->pid_on) {
        snprintf(buf, sizeof buf, " P  Kp·e   %+8.2f ", b->pid_p);
        put_str(r, c0, buf, A_NORMAL, CP_VAL, rows, cols);
        draw_bar(r, bc, bw, b->pid_p / MAX_FORCE, CP_BAR_POS, CP_BAR_NEG, rows, cols);
        r++;

        snprintf(buf, sizeof buf, " I  Ki·∫e  %+8.2f ", b->pid_i);
        put_str(r++, c0, buf, A_NORMAL, CP_VAL, rows, cols);

        snprintf(buf, sizeof buf, " D  Kd·θ̇   %+8.2f ", b->pid_d);
        put_str(r, c0, buf, A_NORMAL, CP_VAL, rows, cols);
        draw_bar(r, bc, bw, b->pid_d / (MAX_FORCE * 0.4f),
                 CP_BAR_POS, CP_BAR_NEG, rows, cols);
        r++;

        snprintf(buf, sizeof buf, " F total  %+8.2f N", b->pid_out);
        int cp = fabsf(b->pid_out) > MAX_FORCE * 0.85f ? CP_WARN : CP_GOOD;
        put_str(r, c0, buf, A_BOLD, cp, rows, cols);
        draw_bar(r, bc, bw, b->pid_out / MAX_FORCE,
                 CP_BAR_POS, CP_BAR_NEG, rows, cols);
        r++;
    } else {
        put_str(r++, c0, " PID  DISABLED          ", A_BOLD, CP_WARN, rows, cols);
        r += 3;
    }

    r++;
    snprintf(buf, sizeof buf, " Kp:%.0f Ki:%.2f Kd:%.0f  ", b->kp, b->ki, b->kd);
    put_str(r++, c0, buf, A_NORMAL, CP_DIM, rows, cols);

    snprintf(buf, sizeof buf, " preset: %s ", PRESETS[b->preset_idx].name);
    put_str(r++, c0, buf, A_BOLD, CP_VAL, rows, cols);
}

/* ── §7.5 render_panel_equations — math with live values ─────────── */

/*
 * Show the cart-pole and PID equations with their current numerical
 * values substituted. The reader can multiply Kp · e by hand and
 * verify it matches the displayed P term — a "cell-level" debugger.
 */
static void render_panel_equations(const Bot *b, int rows, int cols, int c0)
{
    int r = 0;
    char buf[80];

    put_str(r++, c0, " EQUATIONS               ", A_BOLD, CP_VAL, rows, cols);

    /* Effective lean. */
    put_str(r++, c0 + 1, "Effective lean from grav:", A_DIM, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " θ_eff = θ + α = %+5.3f rad (%+5.1f°)",
             b->theta_eff, b->theta_eff * (180.0f / (float)M_PI));
    put_str(r++, c0, buf, A_BOLD, CP_VAL, rows, cols);

    /* Cart-pole horizontal accel. */
    put_str(r++, c0 + 1, "Horizontal accel of base:", A_DIM, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " ẍ = %+6.3f m/s²", b->x_ddot);
    put_str(r++, c0, buf, A_NORMAL, CP_EQ, rows, cols);

    /* Pendulum angular accel. */
    put_str(r++, c0 + 1, "Pendulum angular accel:", A_DIM, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " θ̈ = %+6.3f rad/s²", b->theta_ddot);
    put_str(r++, c0, buf, A_NORMAL, CP_EQ, rows, cols);

    r++;
    put_str(r++, c0, " PID                    ", A_BOLD, CP_VAL, rows, cols);

    /* Setpoint: lean into slope. */
    put_str(r++, c0 + 1, "Setpoint (slope feed):",  A_DIM, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " θ_ref = -%.2f·α = %+5.3f rad",
             SLOPE_FEED, b->theta_ref);
    put_str(r++, c0, buf, A_NORMAL, CP_EQ, rows, cols);

    /* Error. */
    put_str(r++, c0 + 1, "Error drives PID:",       A_DIM, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " e = θ − θ_ref = %+5.3f",
             b->theta - b->theta_ref);
    put_str(r++, c0, buf, A_NORMAL, CP_EQ, rows, cols);

    /* Three terms. */
    put_str(r++, c0 + 1, "P — instant stiffness:",  A_DIM, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " %.0f · %+.4f = %+.2f",
             b->kp, b->theta - b->theta_ref, b->pid_p);
    put_str(r++, c0, buf, A_NORMAL, CP_EQ, rows, cols);

    put_str(r++, c0 + 1, "I — drift removal:",      A_DIM, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " %.2f · ∫e = %+.2f", b->ki, b->pid_i);
    put_str(r++, c0, buf, A_NORMAL, CP_EQ, rows, cols);

    put_str(r++, c0 + 1, "D — damping:",            A_DIM, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " %.0f · θ̇ = %+.2f", b->kd, b->pid_d);
    put_str(r++, c0, buf, A_NORMAL, CP_EQ, rows, cols);

    /* Total F. */
    int cp = fabsf(b->pid_out) > MAX_FORCE * 0.85f ? CP_WARN : CP_GOOD;
    snprintf(buf, sizeof buf, " F = %+.2f N (max ±%.0f)",
             b->pid_out, MAX_FORCE);
    put_str(r++, c0, buf, A_BOLD, cp, rows, cols);
}

/* ── §7.6 render_panel_phase — phase-space portrait of (θ, ω) ────── */

/*
 * Plot the last HIST_LEN samples of (θ, ω) on a small 2-D grid:
 *
 *      ω axis (vertical) — angular velocity in rad/sec
 *      θ axis (horizontal) — lean angle in radians
 *
 * A stable controller draws an inward spiral converging to (0, 0).
 * Underdamped systems trace wide circles; overdamped systems trace
 * a slow direct path. The shape IS the controller's character.
 */
static void render_panel_phase(const Bot *b, int rows, int cols, int c0)
{
    int r = 0;
    char buf[64];

    put_str(r++, c0, " PHASE PORTRAIT         ", A_BOLD, CP_VAL, rows, cols);
    put_str(r++, c0, " (θ vs ω)               ", A_DIM,  CP_DIM, rows, cols);

    /* Plot rectangle. */
    int plot_r0 = r;
    int plot_h  = 12;
    int plot_w  = 29;
    int mid_r   = plot_r0 + plot_h / 2;
    int mid_c   = c0 + 1 + plot_w / 2;

    /* Axes. */
    for (int i = 0; i < plot_h; i++)
        put_ch(plot_r0 + i, mid_c, '|', A_DIM, CP_DIM, rows, cols);
    for (int i = 0; i < plot_w; i++)
        put_ch(mid_r, c0 + 1 + i, '-', A_DIM, CP_DIM, rows, cols);
    put_ch(mid_r, mid_c, '+', A_DIM, CP_DIM, rows, cols);

    /* Axis labels. */
    put_str(mid_r - 1, c0 + 1,        "-θ", A_DIM, CP_DIM, rows, cols);
    put_str(mid_r - 1, c0 + plot_w,   "+θ", A_DIM, CP_DIM, rows, cols);
    put_str(plot_r0,        mid_c - 2, "+ω", A_DIM, CP_DIM, rows, cols);
    put_str(plot_r0 + plot_h - 1, mid_c - 2, "-ω", A_DIM, CP_DIM, rows, cols);

    /* Pixel-to-cell scale. */
    float th_scale = ((float)plot_w * 0.5f) / 0.6f;   /* full range ±0.6 rad */
    float om_scale = ((float)plot_h * 0.5f) / 5.0f;   /* full range ±5 rad/s */

    /* Trail (oldest → newest, so newest paints on top). */
    int n = b->ph_fill;
    for (int i = 0; i < n; i++) {
        int   idx = (b->ph_head - n + i + HIST_LEN) % HIST_LEN;
        float th  = b->ph_theta[idx];
        float om  = b->ph_omega[idx];
        int   pc  = mid_c + (int)(th * th_scale);
        int   pr  = mid_r - (int)(om * om_scale);
        float age = (float)i / (float)n;
        int   cp  = (age > 0.85f) ? CP_WARN
                  : (age > 0.50f) ? CP_VAL  : CP_DIM;
        put_ch(pr, pc, '.', A_NORMAL, cp, rows, cols);
    }

    /* Current point — bright '@'. */
    int pc = mid_c + (int)(b->theta * th_scale);
    int pr = mid_r - (int)(b->omega * om_scale);
    put_ch(pr, pc, '@', A_BOLD, CP_BEACON, rows, cols);

    r = plot_r0 + plot_h;
    r++;

    /* Regime classification — tells the user what they're seeing. */
    {
        float abs_err = fabsf(b->theta - b->theta_ref);
        const char *regime;
        int regime_cp;
        if (!b->pid_on)              { regime = "PID OFF (free fall)";  regime_cp = CP_WARN; }
        else if (b->kd < 1.0f)       { regime = "UNDERDAMPED (no Kd)";  regime_cp = CP_VAL; }
        else if (b->kd > 50.0f)      { regime = "OVERDAMPED (high Kd)"; regime_cp = CP_VAL; }
        else if (abs_err < 0.03f)    { regime = "STABLE — converged";   regime_cp = CP_GOOD; }
        else                         { regime = "SETTLING — correcting"; regime_cp = CP_VAL; }
        put_str(r++, c0 + 1, regime, A_BOLD, regime_cp, rows, cols);
    }

    snprintf(buf, sizeof buf, " θ=%+.3f ω=%+.3f", b->theta, b->omega);
    put_str(r++, c0, buf, A_NORMAL, CP_VAL, rows, cols);

    r++;
    /* Active preset's mini-lesson. */
    const GainPreset *pp = &PRESETS[b->preset_idx];
    snprintf(buf, sizeof buf, " [%s]", pp->name);
    put_str(r++, c0, buf, A_BOLD, CP_VAL, rows, cols);
    put_str(r++, c0 + 1, pp->lesson_l1, A_DIM, CP_DIM, rows, cols);
    put_str(r++, c0 + 1, pp->lesson_l2, A_DIM, CP_DIM, rows, cols);
}

/* ── §7.7 render_hud — top-row status + bottom-row hints ────────── */

static void render_hud(const Bot *b, double fps, int rows, int cols)
{
    char buf[HUD_BUF_LEN];

    float margin_deg = (FALL_ANGLE - fabsf(b->theta_eff))
                       * (180.0f / (float)M_PI);
    snprintf(buf, sizeof buf,
             " %5.1f fps  %s%s  α=%+5.1f°  margin=%4.1f°  dist=%5.1fm  view=[%s]  preset=%s ",
             fps,
             b->paused ? "PAUSED " : (b->fallen ? "FALLEN " : "running"),
             b->pid_on ? ""        : "  NO-PID ",
             b->alpha * (180.0f / (float)M_PI),
             margin_deg, b->dist_m,
             VIEW_NAMES[b->view], PRESETS[b->preset_idx].name);

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    int used = (int)strlen(buf);
    for (int x = used; x < cols; x++) mvaddch(0, x, ' ');
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  ↑↓:speed  p:PID  m:view  g:preset  +/-:Kp ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §7.8 scene_draw — full frame ─────────────────────────────────── */

static void scene_draw(const Bot *b, const Terrain *t, double fps,
                       int bot_sc, int rows, int cols)
{
    erase();

    int wc = (int)(b->world_x / (float)CELL_W);

    /* (1) terrain (sky + surface + rock) */
    render_terrain(t, wc, bot_sc, rows, cols);

    /* (2) robot */
    render_bot(b, t, bot_sc, rows, cols);

    /* (3) right side panel — view-dependent */
    int c0 = cols - PANEL_W;
    if (c0 < 1) c0 = 1;

    switch (b->view) {
    case VIEW_TELEMETRY: render_panel_telemetry(b, rows, cols, c0); break;
    case VIEW_EQUATIONS: render_panel_equations(b, rows, cols, c0); break;
    case VIEW_PHASE:     render_panel_phase    (b, rows, cols, c0); break;
    default: break;
    }

    /* (4) HUD */
    render_hud(b, fps, rows, cols);
}

/* ===================================================================== */
/* §8  screen — ncurses init / present                                    */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §9  app — signals, resize, variable-dt main loop with substeps         */
/* ===================================================================== */

typedef struct {
    Bot                   bot;
    Terrain               terrain;
    Screen                screen;
    int                   bot_sc;          /* bot stays at this screen col */
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_handle_key(App *app, int ch)
{
    Bot *b = &app->bot;
    switch (ch) {
    case 'q': case 'Q': case 27:
        app->running = 0;
        break;

    case ' ':
        b->paused = !b->paused;
        break;

    case 'p': case 'P':
        b->pid_on  = !b->pid_on;
        b->pid_int = 0.0f;
        break;

    case 'r': case 'R':
        bot_reset(b);
        terrain_init(&app->terrain, app->screen.rows, app->screen.cols);
        break;

    case 'm': case 'M':
        b->view = (ViewMode)((b->view + 1) % VIEW_COUNT);
        break;

    case 'g': case 'G':
        b->preset_idx = (b->preset_idx + 1) % N_PRESETS;
        bot_apply_preset(b, b->preset_idx);
        break;

    case KEY_UP:
        b->drive_spd += DRIVE_STEP;
        if (b->drive_spd > DRIVE_MAX) b->drive_spd = DRIVE_MAX;
        break;

    case KEY_DOWN:
        b->drive_spd -= DRIVE_STEP;
        if (b->drive_spd < DRIVE_MIN) b->drive_spd = DRIVE_MIN;
        break;

    case '+': case '=':
        b->kp += 5.0f; if (b->kp > 400.0f) b->kp = 400.0f;
        break;

    case '-':
        b->kp -= 5.0f; if (b->kp < 0.0f) b->kp = 0.0f;
        break;

    default: break;
    }
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    perlin_init((unsigned int)time(NULL));

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    terrain_init(&app->terrain, app->screen.rows, app->screen.cols);
    bot_init(&app->bot);

    app->bot_sc = app->screen.cols / 3;     /* bot stays at this screen col */

    int64_t last_ns      = clock_ns();
    int64_t fps_accum_ns = 0;
    int     fps_frames   = 0;
    double  fps_display  = 0.0;
    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    while (app->running) {

        /* (1) handle resize */
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->bot_sc = app->screen.cols / 3;
            app->terrain.rows = app->screen.rows;
            app->need_resize = 0;
            last_ns = clock_ns();
        }

        /* (2) measure dt, capped */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3) drain input */
        int ch;
        while ((ch = getch()) != ERR) app_handle_key(app, ch);

        /* (4) ensure terrain is generated ahead of the bot */
        terrain_ensure(&app->terrain,
                       (int)(app->bot.world_x / (float)CELL_W)
                       + app->screen.cols + 32);

        /* (5) advance physics in sub-steps for stability */
        if (!app->bot.paused && !app->bot.fallen) {
            int   n_sub  = (int)ceilf(dt / TICK_DT_TARGET);
            if (n_sub < 1)             n_sub = 1;
            if (n_sub > MAX_SUBSTEPS)  n_sub = MAX_SUBSTEPS;
            float sub_dt = dt / (float)n_sub;
            for (int i = 0; i < n_sub; i++)
                bot_substep(&app->bot, &app->terrain, sub_dt);
        }

        /* (6) rolling fps display (0.5 s window) */
        fps_accum_ns += dt_ns;
        fps_frames++;
        if (fps_accum_ns >= NS_PER_SEC / 2) {
            fps_display = (double)fps_frames * 1e9
                        / (double)fps_accum_ns;
            fps_accum_ns = 0;
            fps_frames   = 0;
        }

        /* (7) draw + present */
        scene_draw(&app->bot, &app->terrain, fps_display,
                   app->bot_sc, app->screen.rows, app->screen.cols);
        screen_present();

        /* (8) frame cap */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
