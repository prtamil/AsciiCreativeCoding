/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diff_drive_robot.c — two-wheeled robot with no steering wheel
 *
 * DEMO: A robot that has TWO INDEPENDENT WHEELS on a single fixed axle
 *       — and that's the whole steering mechanism. There is no
 *       steering wheel. There is no joystick that controls direction.
 *       To turn left, the LEFT wheel goes slower than the RIGHT one.
 *       To turn right, the right wheel goes slower than the left one.
 *       To spin in place, the wheels go in OPPOSITE directions. To
 *       drive straight, the wheels run at the same speed.
 *
 *       Real-world examples:
 *           ─ iRobot Roomba (vacuum)
 *           ─ tank-style RC toys
 *           ─ electric wheelchairs (joystick controls each wheel
 *                                   independently)
 *           ─ many small lab/research robots
 *
 *       This file is a minimal teaching simulator: you press keys,
 *       the keys translate into "left wheel speed" and "right wheel
 *       speed", and the robot moves wherever those two numbers
 *       cause it to go. The screen shows the robot from above, the
 *       wheels marked L and R, a yellow heading arrow, the wheel
 *       velocity arrows (green = forward, red = reverse), and a
 *       trail of recent positions.
 *
 * Study alongside:
 *   robots/walking_robot.c           — bipedal contrast: legs
 *                                      instead of wheels.
 *   animation/hexpod_tripod.c        — 6-leg insect locomotion.
 *   particle_systems/2stroke.c       — slider-crank kinematics
 *                                      (similar idea: gears/wheels
 *                                       drive position).
 *   matrix_rain/matrix_rain.c        — same project conventions
 *                                      (variable-dt loop, HUD spec,
 *                                       theme palettes).
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus per-element semantic pairs
 *   §4  coords       — pixel↔cell aspect-ratio bridge
 *   §5  trail        — ring buffer of recent positions
 *   §6  robot        — the heart: state, kinematics, integration
 *   §7  scene        — input → command translation, tick orchestration
 *   §8  render       — draw the robot, wheels, arrows, trail, HUD
 *   §9  screen       — ncurses init / present / HUD spec
 *   §10 app          — signals, resize, variable-dt main loop
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            full stop (zero both v_cmd and w_cmd)
 *   p                pause / resume physics
 *   r                reset to centre
 *   W / ↑            throttle forward (ramps v_cmd up)
 *   S / ↓            throttle backward (ramps v_cmd down/negative)
 *   A / ←            turn left
 *   D / →            turn right
 *   Z                spin in place LEFT (instant max ω, v=0)
 *   E                spin in place RIGHT (instant max ω, v=0)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra robots/diff_drive_robot.c \
 *       -o diff_drive_robot -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Two-step kinematics, run once per frame.
 *
 *                 (1) FORWARD KINEMATICS — given the two wheel speeds
 *                     vL (left) and vR (right), compute how the
 *                     robot's body position (x, y) and heading θ
 *                     change in the next dt seconds.
 *
 *                         v = (vL + vR) / 2          ← body speed
 *                         ω = (vR − vL) / axle       ← spin rate
 *                         x += v · cos(θ) · dt
 *                         y += v · sin(θ) · dt
 *                         θ += ω · dt
 *
 *                 (2) INVERSE KINEMATICS — the user wants to go
 *                     forward at speed v_cmd and turn at rate w_cmd.
 *                     Compute what wheel speeds achieve that.
 *
 *                         vL = v_cmd − w_cmd · axle / 2
 *                         vR = v_cmd + w_cmd · axle / 2
 *
 *                 The user's keys nudge `v_cmd` and `w_cmd`; (2)
 *                 turns those into `vL`, `vR`; (1) turns those into
 *                 a new pose. That's the entire physics loop.
 *
 * Data-structure: One Robot struct holds the pose (px, py, theta),
 *                 the two wheel speeds (vL, vR), the user's command
 *                 inputs (v_cmd, w_cmd), the constant axle width,
 *                 and a ring-buffer Trail of past positions. No
 *                 heap allocation post-init.
 *
 * Rendering     : Painter's order with the body always on top.
 *                 Trail dots first (oldest → newest), then wheel
 *                 velocity arrows (green/red), then the heading
 *                 arrow (yellow), then 'L' and 'R' wheel labels,
 *                 then the '@' body. HUD on row 0, hint strip on
 *                 the bottom row.
 *
 * Performance   : O(1) per frame. Six trig calls (2 cos, 2 sin,
 *                 plus one cos and one sin per arrow drawn).
 *                 Microseconds. ncurses redraw is the dominant
 *                 cost.
 *
 * References    :
 *   Wikipedia, "Differential wheeled robot" — formulas, geometry,
 *     real-world examples.
 *     https://en.wikipedia.org/wiki/Differential_wheeled_robot
 *   Wikipedia, "Nonholonomic system" — why a diff-drive robot
 *     cannot slide sideways even though its wheels are
 *     independent. (A car parallel-parking is the same constraint.)
 *     https://en.wikipedia.org/wiki/Nonholonomic_system
 *   Siegwart, Nourbakhsh & Scaramuzza, "Introduction to Autonomous
 *     Mobile Robots" (MIT Press, 2nd ed.) — chapter 3 covers
 *     differential drive in depth.
 *   This project, robots/walking_robot.c — the legged contrast:
 *     pose comes from foot positions instead of wheel speeds.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A differential drive robot has two independent driven wheels on a
 * shared axle. Each wheel can be set to any speed (positive, zero,
 * or negative) independently of the other. Steering is purely an
 * emergent property of running them at different speeds.
 *
 * If you understand four motions, you understand the entire concept:
 *
 * THE FOUR MOTIONS
 * ────────────────
 *
 *  ┌──────────────────────────────────────────────────────────────┐
 *  │                                                              │
 *  │    DRIVE STRAIGHT       TURN RIGHT       SPIN IN PLACE       │
 *  │    ────────────────     ───────────      ────────────────    │
 *  │                                                              │
 *  │    vL  =  vR            vL = 2           vL = -5             │
 *  │           ↑   ↑         vR = 8           vR = +5             │
 *  │           │   │             ⤴                                │
 *  │     ┌────L─R────┐       ┌───L─R───┐       ┌────L─R────┐      │
 *  │     │           │       │      ↘  │       │     ↻     │      │
 *  │     │  (body)   │  →    │ (curves)│       │ (rotates  │      │
 *  │     │           │       │   right │       │  in place)│      │
 *  │     └───────────┘       └─────────┘       └───────────┘      │
 *  │                                                              │
 *  │    PIVOT — slow wheel = pivot, fast wheel sweeps the arc:    │
 *  │                                                              │
 *  │    vL = 0                                                    │
 *  │    vR = 5                                                    │
 *  │     ┌────L─R────┐    ← L stuck, R goes forward               │
 *  │     │           │      → robot pivots around L               │
 *  │     │  ↻        │      → ICC (centre of arc) = position of L │
 *  │     │           │                                            │
 *  │     └───────────┘                                            │
 *  │                                                              │
 *  └──────────────────────────────────────────────────────────────┘
 *
 *
 * WHY "DIFFERENTIAL"?
 * ───────────────────
 * Because the steering effect comes from the DIFFERENCE between the
 * two wheel speeds. Not from a steering rod, not from a joint —
 * just (vR − vL).
 *
 *   vR − vL  >  0   →   turn right
 *   vR − vL  <  0   →   turn left
 *   vR − vL  =  0   →   straight (no matter what vR + vL is)
 *
 * The MAGNITUDE of the difference, divided by the axle width, is
 * the angular velocity ω in radians per second:
 *
 *      ω = (vR − vL) / axle
 *
 * The SUM divided by 2 is the body's straight-ahead speed v:
 *
 *      v = (vL + vR) / 2
 *
 * That's the whole forward kinematics. Two scalars in (vL, vR), two
 * scalars out (v, ω).
 *
 * ICC — "INSTANTANEOUS CENTRE OF CURVATURE"
 * ─────────────────────────────────────────
 * When ω ≠ 0, the robot follows a circular arc. The CENTRE of that
 * circle (called the ICC) lies on the axle's extension, on the side
 * of the SLOWER wheel:
 *
 *           ICC × ─ ─ ─ ─ ─ ─ ─ ─ ┐
 *                │                │  R = turn radius (signed)
 *                │                ↓
 *                ┌────L─R────┐
 *                │           │
 *                │  (body)   │
 *                └───────────┘
 *
 *           R = (axle / 2) · (vR + vL) / (vR − vL)
 *             = v / ω
 *
 *  Slow wheel near ICC, fast wheel sweeps wider arc, body's centre
 *  travels at v on a circle of radius |R|. When vL = vR, R = ∞
 *  (straight line). When vL = −vR, R = 0 (spin in place).
 *
 * NONHOLONOMIC CONSTRAINT
 * ───────────────────────
 * The robot can't slide sideways. The wheels resist lateral motion
 * (they roll, they don't slip). In math:
 *
 *      ẋ · sin θ  −  ẏ · cos θ  =  0           (lateral velocity = 0)
 *
 * This constraint is automatically satisfied by `ẋ = v · cos θ` and
 * `ẏ = v · sin θ` — the velocity vector is always along the heading.
 * No projection step or penalty needed.
 *
 * Compare with an OMNIDIRECTIONAL robot (mecanum wheels, omni wheels):
 * it CAN slide sideways and its kinematics have no such restriction.
 * That's the holonomic case. Diff drive is nonholonomic.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. dt = wall-clock seconds since last frame, capped.
 *  2. Read keys, set the boolean flags `fwd`, `rev`, `left`, `right`,
 *     `spin_l`, `spin_r`, `stop`.
 *  3. Translate keys into commands:
 *      - W / S nudges v_cmd up/down at V_RATE per second.
 *      - A / D nudges w_cmd left/right at W_RATE per second.
 *      - When no turn key is held, w_cmd decays exponentially toward 0.
 *      - Z / E sets v_cmd = 0 and w_cmd = ±W_MAX (instant in-place spin).
 *      - Space sets both commands to 0.
 *      - Clamp v_cmd to [-V_MAX, V_MAX], w_cmd to [-W_MAX, W_MAX].
 *  4. INVERSE KINEMATICS — turn (v_cmd, w_cmd) into (vL, vR):
 *           vL = v_cmd − w_cmd · axle / 2
 *           vR = v_cmd + w_cmd · axle / 2
 *  5. FORWARD KINEMATICS + Euler integration — turn (vL, vR) into a
 *     new pose:
 *           v = (vL + vR) / 2
 *           ω = (vR − vL) / axle
 *           x += v · cos(θ) · dt
 *           y += v · sin(θ) · dt
 *           θ += ω · dt
 *           wrap θ into (−π, π]
 *  6. WRAP — if the body left a screen edge, teleport it to the
 *     opposite edge. Robot stays visible forever.
 *  7. PUSH every TRAIL_SAMPLE_STEP frames, push (x, y) into the trail
 *     ring buffer.
 *  8. RENDER.
 *
 * KEY FORMULAS
 * ────────────
 *   Inverse kinematics (commands → wheels):
 *           vL = v_cmd − w_cmd · L/2
 *           vR = v_cmd + w_cmd · L/2
 *
 *   Forward kinematics (wheels → body velocity):
 *           v  = (vL + vR) / 2
 *           ω  = (vR − vL) / L
 *
 *   Pose update (Euler):
 *           x  ← x + v · cos(θ) · dt
 *           y  ← y + v · sin(θ) · dt
 *           θ  ← θ + ω · dt
 *
 *   Turn radius:
 *           R = v / ω         (R > 0 right, < 0 left, ±∞ straight)
 *
 *   Heading wrap:
 *           θ ∈ (−π, π]       (so atan2 / lerp behave well)
 *
 *   Coordinate convention (terminal screens are y-down):
 *           θ = 0      → faces RIGHT  (+x)
 *           θ = π/2    → faces DOWN   (+y)
 *           ω > 0      → CLOCKWISE on screen (because y is down)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Wheel speed clamping. Each wheel is independently capped at
 *    ±V_MAX in `compute_wheels`. If both v_cmd and w_cmd are at
 *    their limits, one wheel would saturate; the clamp ensures
 *    neither goes outside its physical limit even though the
 *    commanded motion may not be exactly achieved.
 *
 *  • Heading wrap-around. After integration, θ may be ±10000° if
 *    you spin for a long time. Wrap into (−π, π] every step so
 *    HUD readout (in degrees) stays sensible and atan2 / lerp
 *    don't drift.
 *
 *  • Toroidal wrap. When the robot leaves the edge it reappears
 *    on the opposite side. Without this it would be hard to
 *    test because long drives go off-screen and you lose it.
 *
 *  • Variable-dt stability. Forward Euler with dt up to 100 ms is
 *    stable here because v and ω are bounded; the worst-case arc
 *    error per step is O((ω·dt)²) — a fraction of a pixel.
 *
 *  • Coordinate convention: y is DOWN. Don't get caught by the
 *    "perpendicular to heading" mistake — in y-down,
 *        left  = ( +sin θ, −cos θ )
 *        right = ( −sin θ, +cos θ )
 *    The math-textbook formulas (which assume y-up) are flipped.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press W and hold. The yellow heading arrow stays straight; the
 *    body slides along it. The trail forms a straight line. HUD
 *    shows v rising to V_MAX, ω = 0, R = INF.
 *
 *  • Press W and D simultaneously. Body curves right. HUD shows
 *    v > 0, ω > 0, R > 0. Trail is a smooth arc.
 *
 *  • Release D, keep W. ω fades back to 0 over ~0.5 sec; the curve
 *    straightens out.
 *
 *  • Press E (spin right). Body rotates clockwise without
 *    translating. HUD: v = 0, ω = +W_MAX, R = 0. Trail is a single
 *    dot.
 *
 *  • Press space. Robot stops mid-motion. HUD: v = 0, ω = 0.
 *
 *  • Stopwatch test: at v = V_MAX = 180 px/s, the robot crosses a
 *    100-cell-wide screen (= 800 px) in 800/180 ≈ 4.4 sec.
 *
 *  • In-place spin: at ω = W_MAX = 3 rad/s, full rotation (2π) takes
 *    2π/3 ≈ 2.1 sec. Time it.
 *
 *  • Wheel arrow test: hold E. Right wheel arrow turns green
 *    pointing forward (vR > 0); left wheel arrow turns red pointing
 *    backward (vL < 0). They have equal magnitude.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. This is the FOUNDATIONAL robotics file in the
 *      project — read it first if robotics kinematics are new.
 *      walking_robot, moving_jump_spring_leg_robot, perlin_terrain_bot
 *      all assume you understand pose (px, py, theta) integration.
 *   2. §6 robot — THE HEART of this file. Forward kinematics +
 *      inverse kinematics + Euler integration. Read AFTER tutorials
 *      T1-T5 below.
 *   3. §7 scene — input → command translation. The keyboard handler
 *      converts presses into commanded (v_cmd, w_cmd) which §6 then
 *      turns into wheel speeds.
 *   4. §8 render — painter's-order draw with trail, wheel arrows,
 *      heading arrow, and labels.
 *   5. §1-§5, §9-§10 — config / clock / colour / coords / trail /
 *      screen / app loop. Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   px, py            body position in pixel space (Vec2 floats).
 *   theta             body heading in radians (0 = +X, π/2 = +Y).
 *   vL, vR            wheel speeds — pixels per second, signed
 *                     (negative = reverse).
 *   v                 body speed = (vL + vR) / 2.
 *   omega             body angular velocity = (vR - vL) / axle.
 *   axle              constant distance between wheel centres.
 *   v_cmd, w_cmd      USER COMMANDS — what the user wants. Differ
 *                     from (v, omega) by an exponential ramp.
 *   V_MAX, W_MAX      command saturation limits.
 *   V_RATE, W_RATE    ramp rates — how fast cmds change per second.
 *   ICC               instantaneous centre of curvature (T3 below).
 *
 * Background you need
 * ───────────────────
 *   - Vec2 / scalar arithmetic.
 *   - Trig: (cos θ, sin θ) is the unit vector at angle θ.
 *   - Variable-timestep main loop (CLAUDE.md §Architecture).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Lagrangian mechanics. Diff-drive kinematics are PURE
 *     KINEMATICS (geometry of motion); no forces / torques /
 *     mass involved.
 *   - Wheel slip, friction modelling. We assume PURE ROLLING
 *     (lateral velocity always zero — T4 below).
 *   - Path planning (A*, RRT). The user drives directly with
 *     keys; no planner.
 *   - Sensor models, SLAM. No sensors here.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a differential-drive robot from
 * first principles.
 *
 *   T1  Two wheels, no steering — what "differential" means
 *   T2  Forward kinematics — wheels in, body motion out
 *   T3  Inverse kinematics — body command in, wheels out
 *   T4  The nonholonomic constraint — why robots can't slide sideways
 *   T5  Command ramping — separating WANT from CAN
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  TWO WHEELS, NO STEERING — WHAT "DIFFERENTIAL" MEANS
 * ───────────────────────────────────────────────────────
 * A car has FOUR wheels and a STEERING WHEEL. The steering
 * wheel turns the front wheels left/right relative to the body;
 * gas controls all four wheels' rotational speed equally.
 *
 * A differential-drive robot has TWO WHEELS, no steering. Each
 * wheel can be controlled INDEPENDENTLY. Steering is purely
 * the result of running the wheels at different speeds.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │  vL = vR        →   straight                     │
 *      │  vL > vR        →   turn right                   │
 *      │  vL < vR        →   turn left                    │
 *      │  vL = -vR       →   spin in place                │
 *      │  vL = 0, vR > 0 →   pivot around left wheel      │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * "Differential" = "the difference between vR and vL is what
 * causes turning." NOT "differential gear" (which is a
 * mechanical part of cars). The naming is an unfortunate
 * collision.
 *
 * Real-world examples:
 *   - Roomba: two driven wheels + caster wheels for stability.
 *   - Tank-style RC toys: same principle scaled up with treads.
 *   - Power wheelchair: joystick input → independent wheel
 *     speeds.
 *   - Many small lab robots: simple, cheap, agile.
 *
 * Two wheels are SIMPLER to control than four (no steering
 * mechanics) and ROOM-EFFICIENT (small turn radius — can spin
 * in place). They lose to cars on rough terrain (wheels can
 * slip on uneven ground) but win in flat indoor environments.
 *
 * T2  FORWARD KINEMATICS — WHEELS IN, BODY MOTION OUT
 * ───────────────────────────────────────────────────
 * Given vL and vR (current wheel speeds), how does the body
 * move?
 *
 * Two scalars come out:
 *
 *     v     = (vL + vR) / 2          ← linear speed of body centre
 *     omega = (vR - vL) / axle        ← angular speed of body
 *
 * Why these formulas? Treat each wheel as a point at distance
 * axle/2 from the body centre. The body's centre moves at the
 * AVERAGE of the two wheel velocities. The body's rotation
 * rate is the DIFFERENCE between the wheel velocities, divided
 * by the perpendicular distance separating them (= axle).
 *
 * Then standard rigid-body integration:
 *
 *     px += v · cos(theta) · dt
 *     py += v · sin(theta) · dt
 *     theta += omega · dt
 *
 * "Walk forward by v·dt in the direction theta points; rotate
 * by omega·dt." Three lines, the entire integrator.
 *
 * Note: this is FORWARD EULER integration. For high speeds or
 * tight turns it can drift slightly; production robotics uses
 * RUNGE-KUTTA or exact arc integration. At our terminal frame
 * rates the drift is invisible.
 *
 * T3  INVERSE KINEMATICS — BODY COMMAND IN, WHEELS OUT
 * ────────────────────────────────────────────────────
 * The user thinks in BODY COMMANDS: "go forward at speed v,
 * turn right at rate omega." NOT in wheel speeds.
 *
 * Inverse kinematics: given (v, omega), what (vL, vR) achieve
 * them? Solve T2's two equations for vL, vR:
 *
 *     v     = (vL + vR) / 2          →   vL + vR = 2v
 *     omega = (vR - vL) / axle       →   vR - vL = omega · axle
 *
 *     vL = v - omega · axle / 2
 *     vR = v + omega · axle / 2
 *
 * This is THE SECOND HALF of the loop. The user's input gives
 * (v_cmd, w_cmd); inverse kinematics turns those into wheel
 * speeds (vL, vR); forward kinematics integrates wheel speeds
 * into the new pose.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   user input        commanded     wheel       new pose │
 *      │   (keys)        →   (v, omega) →  speeds   →   (px, py, θ) │
 *      │                                                  │
 *      │                     INVERSE KIN     FORWARD KIN  │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * Both directions are TRIVIAL ALGEBRA — no iterative solver,
 * no matrix inversion. That's why diff drive is the simplest
 * mobile-robot kinematic model and a good first lesson.
 *
 * ICC — INSTANTANEOUS CENTRE OF CURVATURE
 * ────────────────────────────────────────
 * When omega ≠ 0, the body follows a circular arc. The centre
 * of that circle (the "ICC") lies on the EXTENSION of the axle
 * line, at distance R from the body centre:
 *
 *     R = v / omega
 *
 *     R > 0   →   ICC is on the LEFT of the body
 *     R < 0   →   ICC is on the RIGHT
 *     R = 0   →   ICC is AT THE BODY CENTRE → spin in place
 *     R = ∞   →   no ICC → straight line
 *
 * The CLOSER WHEEL to the ICC moves slower (it's tracing a
 * smaller circle); the FARTHER wheel moves faster. For pivot
 * (vL = 0, vR > 0), ICC sits at the LEFT WHEEL — that wheel
 * is stationary, the right wheel sweeps the arc.
 *
 * T4  THE NONHOLONOMIC CONSTRAINT — WHY ROBOTS CAN'T SLIDE SIDEWAYS
 * ─────────────────────────────────────────────────────────────────
 * Wheels ROLL — they don't slide laterally. The body of a
 * differential-drive robot can ONLY MOVE IN THE DIRECTION ITS
 * WHEELS ARE POINTING. It cannot translate sideways without
 * first rotating to face that direction.
 *
 * Mathematically:
 *
 *     ẋ · sin θ  −  ẏ · cos θ  =  0        (lateral velocity = 0)
 *
 * "The component of velocity perpendicular to the heading is
 * always zero." This is automatic given our forward kinematics:
 *
 *     ẋ = v · cos θ
 *     ẏ = v · sin θ
 *
 * Substituting: v · cos θ · sin θ − v · sin θ · cos θ = 0. ✓
 *
 * This constraint is called NONHOLONOMIC. Practical
 * consequences:
 *
 *   - Parallel parking is hard. To shift a car body sideways
 *     by 1 metre, you need a sequence of forward + reverse +
 *     turn + forward arcs. There's no "side-step" command.
 *
 *   - Path planning is harder than for omnidirectional robots.
 *     The set of reachable poses from a given start grows
 *     more slowly because of the heading constraint.
 *
 *   - In simulation, you DON'T NEED to enforce the constraint
 *     explicitly — it falls out of the forward kinematics.
 *
 * Compare to OMNIDIRECTIONAL robots (mecanum wheels, omni
 * wheels, holonomic platforms): they CAN slide in any
 * direction independent of heading. Their kinematics have an
 * additional vy term not aligned with theta. Easier to plan,
 * harder to build, more expensive.
 *
 * For 99% of indoor robotics, diff drive (nonholonomic) is the
 * right answer.
 *
 * T5  COMMAND RAMPING — SEPARATING WANT FROM CAN
 * ──────────────────────────────────────────────
 * If the user pressed W and we instantly set v = V_MAX, the
 * robot would JUMP to top speed and back to 0 when released.
 * That's both unrealistic and visually jarring.
 *
 * The fix: separate the user's INTENDED COMMAND (`v_cmd`) from
 * the actual current command. Each frame:
 *
 *     while W is held:
 *       v_cmd += V_RATE · dt
 *     while no throttle key:
 *       v_cmd ramps toward 0 with exponential decay
 *     clamp v_cmd to [-V_MAX, V_MAX]
 *
 * Same for w_cmd with the A/D keys. The result: the robot
 * SMOOTHLY ACCELERATES and SMOOTHLY DECELERATES. Press-and-
 * hold ramps up; release coasts down.
 *
 * V_RATE, W_RATE, V_MAX, W_MAX are knobs:
 *
 *     V_MAX     fastest possible body speed
 *     V_RATE    seconds to reach V_MAX from zero (lower = snappier)
 *     W_MAX     fastest spin rate
 *     W_RATE    seconds to reach W_MAX
 *
 * The "instant spin in place" keys (Z, E) bypass the ramp:
 * they set v_cmd = 0 and w_cmd = ±W_MAX directly. Useful for
 * quick rotations.
 *
 * Lesson: real control systems separate WANTED state from
 * ACTUAL state, with rate limits between them. This is the
 * minimum — production controllers add PID, model
 * compensation, and acceleration profiling. But the basic
 * pattern (cmd → ramped command → physics) is the same
 * everywhere from car cruise control to drone autopilots.
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

/* ── §1.1 frame rate ──────────────────────────────────────────────── */
enum { TARGET_FPS = 60 };

/* ── §1.2 cell pixel dimensions (the aspect-ratio bridge) ─────────── */
/*
 * All physics happens in PIXEL SPACE — a uniform grid where 1 px
 * is 1/8 of a cell wide and 1/16 of a cell tall. This decouples
 * the simulation from terminal size and corrects for the 2:1
 * cell aspect ratio that would otherwise stretch circles into
 * vertical ellipses.
 */
#define CELL_W   8
#define CELL_H  16

/* ── §1.3 robot geometry (pixels) ─────────────────────────────────── */
/*
 * AXLE_PX — distance between the two wheels, in pixels.
 *   With CELL_W = 8 px, 36 px ≈ 4.5 cells — a small robot on a
 *   typical terminal. Wider axle → slower turns at the same wheel
 *   speed difference (because ω = (vR-vL)/axle).
 *
 * ARROW_PX — length of the yellow heading arrow at full extension.
 *
 * VEL_ARROW_PX — length of the green/red wheel velocity arrow when
 *   the wheel is at ±V_MAX. At lower speeds the arrow shrinks
 *   linearly with |v_wheel|.
 */
#define AXLE_PX        36.0f
#define ARROW_PX       34.0f
#define VEL_ARROW_PX   28.0f

/* ── §1.4 dynamics (physical units) ───────────────────────────────── */
/*
 * V_MAX, W_MAX — physical limits.
 *   V_MAX = 180 px/sec → robot crosses a 100-col terminal in ~4 sec.
 *   W_MAX = 3 rad/sec  → full rotation in 2π/3 ≈ 2.1 sec.
 *
 * V_RATE, W_RATE — how fast the COMMAND ramps up while the throttle
 *   key is held.
 *   V_RATE = 12 · V_MAX → reaches V_MAX in 1/12 ≈ 0.083 sec (snappy).
 *   W_RATE = 15 · W_MAX → reaches W_MAX in 1/15 ≈ 0.067 sec.
 *
 * V_DECAY_PER_SEC — multiplier on v_cmd per second when no throttle
 *   key is held.
 *   1.0 = no friction (robot keeps coasting indefinitely until braked).
 *
 * W_DECAY_PER_SEC — multiplier on w_cmd per second when no turn key
 *   is held.
 *   0.001 = strong damping → ω drops to 0.1 % of its value over 1 sec
 *   so straight-driving doesn't acquire residual rotation from a
 *   brief turn input.
 */
#define V_MAX            180.0f
#define W_MAX              3.0f
#define V_RATE          (V_MAX * 12.0f)
#define W_RATE          (W_MAX * 15.0f)
#define V_DECAY_PER_SEC    1.000f
#define W_DECAY_PER_SEC    0.001f

/* ── §1.5 trail ring buffer ──────────────────────────────────────── */
enum {
    TRAIL_CAP             = 600,
    /* Push a sample every Nth frame so the trail stretches further
     * for the same memory budget. 2 = sample at 30 Hz on a 60-fps loop. */
    TRAIL_SAMPLE_STEP     = 2,
};

/* ── §1.6 dt cap (spiral-of-death guard) ──────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.7 timing primitives ───────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.8 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..6 — robot semantic colours */
    CP_BODY      = 1,           /* '@'                white          */
    CP_HEAD,                    /* heading arrow      yellow         */
    CP_WHL_L,                   /* 'L' wheel label    green          */
    CP_WHL_R,                   /* 'R' wheel label    magenta        */
    CP_VEL_FWD,                 /* wheel arrow forward  lime         */
    CP_VEL_REV,                 /* wheel arrow reverse  red          */

    /* 7..8 — trail */
    CP_TRAIL_NEW,               /* fresh trail dots   cyan           */
    CP_TRAIL_OLD,               /* aged trail dots    dim blue       */

    /* 9..10 — HUD spec, theme-independent */
    PAIR_HUD,                   /* row 0 status       yellow A_BOLD  */
    PAIR_HINT,                  /* bottom row hint    cyan   A_BOLD  */
};

/* ── §1.9 HUD layout ──────────────────────────────────────────────── */
#define HUD_BUF_LEN  120

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
 * Each visual element gets one fixed semantic colour. The palette is
 * not theme-cycled — this is a teaching simulator, not a screensaver.
 *
 *   body         '@'       white
 *   heading      arrow     yellow
 *   left wheel   'L'       green       (so 'L' for green/Left is mnemonic)
 *   right wheel  'R'       magenta
 *   forward vel  arrow     lime green
 *   reverse vel  arrow     red
 *   fresh trail  '.'       cyan        (recent path)
 *   old trail    ':'       dim blue    (older path, fades visually)
 *
 * HUD pairs (PAIR_HUD, PAIR_HINT) are bound separately on the
 * default terminal background (-1) per CLAUDE.md HUD spec.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_BODY,       255, COLOR_BLACK);    /* near-white     */
        init_pair(CP_HEAD,       226, COLOR_BLACK);    /* yellow         */
        init_pair(CP_WHL_L,       46, COLOR_BLACK);    /* green          */
        init_pair(CP_WHL_R,      201, COLOR_BLACK);    /* magenta        */
        init_pair(CP_VEL_FWD,     82, COLOR_BLACK);    /* lime           */
        init_pair(CP_VEL_REV,    196, COLOR_BLACK);    /* red            */
        init_pair(CP_TRAIL_NEW,   51, COLOR_BLACK);    /* cyan           */
        init_pair(CP_TRAIL_OLD,   25, COLOR_BLACK);    /* dim blue       */
        init_pair(PAIR_HUD,      226, -1);             /* yellow on bg   */
        init_pair(PAIR_HINT,      51, -1);             /* cyan on bg     */
    } else {
        init_pair(CP_BODY,     COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_HEAD,     COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_WHL_L,    COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_WHL_R,    COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_VEL_FWD,  COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_VEL_REV,  COLOR_RED,     COLOR_BLACK);
        init_pair(CP_TRAIL_NEW,COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_TRAIL_OLD,COLOR_BLUE,    COLOR_BLACK);
        init_pair(PAIR_HUD,    COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,   COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio bridge                           */
/* ===================================================================== */

/*
 * Physics lives in PIXEL SPACE. A pixel is 1/8 of a cell wide and
 * 1/16 of a cell tall, so circles drawn in pixel space look round
 * on screen. Conversion to cell coordinates happens ONLY at draw
 * time (px_to_cx, px_to_cy), and the world dimensions in pixels
 * (pw, ph) are derived from the terminal size in cells.
 */
static inline int   pw       (int cols)  { return cols * CELL_W; }
static inline int   ph       (int rows)  { return rows * CELL_H; }
static inline int   px_to_cx (float px)  { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cy (float py)  { return (int)floorf(py / (float)CELL_H + 0.5f); }

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ===================================================================== */
/* §5  trail — ring buffer of recent positions                           */
/* ===================================================================== */

/*
 * Trail keeps the last TRAIL_CAP positions, sampled every
 * TRAIL_SAMPLE_STEP frames so the visible trail stretches further
 * for the same memory budget.
 *
 * `head` is the index of the NEXT write slot. After the buffer
 * fills, `head` wraps and the oldest entry becomes the next slot
 * to be overwritten — a classic circular buffer.
 */
typedef struct {
    float px[TRAIL_CAP];
    float py[TRAIL_CAP];
    int   head;             /* index of next write slot       */
    int   count;            /* valid entries (≤ TRAIL_CAP)    */
    int   skip;             /* frames since last sample        */
} Trail;

static void trail_clear(Trail *t)
{
    t->head = t->count = t->skip = 0;
}

static void trail_push(Trail *t, float px, float py)
{
    if (++t->skip < TRAIL_SAMPLE_STEP) return;
    t->skip          = 0;
    t->px[t->head]   = px;
    t->py[t->head]   = py;
    t->head          = (t->head + 1) % TRAIL_CAP;
    if (t->count < TRAIL_CAP) t->count++;
}

/* ===================================================================== */
/* §6  robot — state, inverse and forward kinematics, integration         */
/* ===================================================================== */

/* ── §6.1 Robot type ──────────────────────────────────────────────── */

/*
 * Robot — the entire simulation state.
 *
 *   Pose:
 *     px, py     position in pixel space (top-left = 0, 0)
 *     theta      heading in radians; 0 = east (+x), π/2 = south
 *
 *   Wheel speeds (pixels/sec, computed every frame from commands):
 *     vL         left wheel
 *     vR         right wheel
 *
 *   User commands (driven by keyboard input):
 *     v_cmd      desired linear speed (pixels/sec)
 *     w_cmd      desired angular speed (radians/sec)
 *
 *   Constants:
 *     axle       wheel separation in pixels (set once at init)
 *
 *   Trail ring buffer of recent positions.
 *   `paused` freezes physics but lets the renderer keep running
 *   so the HUD updates.
 */
typedef struct {
    float px, py, theta;
    float vL, vR;
    float v_cmd, w_cmd;
    float axle;
    Trail trail;
    bool  paused;
} Robot;

/* ── §6.2 compute_wheels — INVERSE KINEMATICS (commands → wheels) ── */

/*
 * The user thinks "I want to go forward at v_cmd while turning at
 * w_cmd". The robot only knows how to set wheel speeds.
 *
 * Given an axle of width L = robot.axle and body centre velocity v,
 * angular ω:
 *
 *     left wheel  travels a smaller arc on a right turn (ω > 0):
 *         vL = v − ω · L/2
 *     right wheel travels a larger arc:
 *         vR = v + ω · L/2
 *
 * That's it — two scalar subtractions/additions. The hard part is
 * remembering it's the DIFFERENCE that causes turning, not a
 * separate steering input.
 *
 * Each wheel is independently clamped to ±V_MAX so neither motor
 * saturates. If the commanded combo would saturate, the actual
 * (v, ω) achieved is less than commanded — but the robot still
 * moves consistently, just slower.
 */
static void compute_wheels(Robot *r)
{
    float half = r->axle * 0.5f;
    r->vL = clampf(r->v_cmd - r->w_cmd * half, -V_MAX, V_MAX);
    r->vR = clampf(r->v_cmd + r->w_cmd * half, -V_MAX, V_MAX);
}

/* ── §6.3 step_pose — FORWARD KINEMATICS + Euler integration ──────── */

/*
 * Given the two wheel speeds, recover (v, ω):
 *
 *     v  = (vL + vR) / 2          (the body's centre speed)
 *     ω  = (vR − vL) / axle       (the body's spin rate)
 *
 * Then advance the pose by one Euler step:
 *
 *     x ← x + v · cos(θ) · dt
 *     y ← y + v · sin(θ) · dt
 *     θ ← θ + ω · dt
 *
 * Forward Euler is more than accurate enough at 60 fps: the worst-
 * case arc error per step is O((ω·dt)²) — a fraction of a pixel.
 * For higher ω or larger dt you'd switch to the exact ICC arc
 * integrator (see CONCEPTS).
 *
 * After the integration, we wrap θ into (−π, π] so the HUD readout
 * (in degrees) doesn't show "thirty-thousand degrees" after a long
 * spin and so atan2 / lerp behave well.
 */
static void step_pose(Robot *r, float dt)
{
    float v     = (r->vL + r->vR) * 0.5f;
    float omega = (r->vR - r->vL) / r->axle;

    r->px    += v     * cosf(r->theta) * dt;
    r->py    += v     * sinf(r->theta) * dt;
    r->theta += omega * dt;

    /* Wrap heading into (−π, π]. */
    while (r->theta >  (float)M_PI) r->theta -= 2.0f * (float)M_PI;
    while (r->theta < -(float)M_PI) r->theta += 2.0f * (float)M_PI;
}

/* ── §6.4 wrap_position — toroidal screen edges ──────────────────── */

/*
 * If the robot drives off the right edge it reappears on the left;
 * off the bottom it reappears at the top, and so on. This is not
 * physically realistic but keeps the robot visible at all times,
 * which is what you want in a teaching simulator.
 */
static void wrap_position(Robot *r, int wpx, int hpx)
{
    if (r->px <  0.0f)         r->px += (float)wpx;
    if (r->px >= (float)wpx)   r->px -= (float)wpx;
    if (r->py <  0.0f)         r->py += (float)hpx;
    if (r->py >= (float)hpx)   r->py -= (float)hpx;
}

/* ── §6.5 robot_init / robot_reset ───────────────────────────────── */

static void robot_init(Robot *r, int wpx, int hpx)
{
    memset(r, 0, sizeof *r);
    r->axle  = AXLE_PX;
    r->px    = (float)wpx * 0.5f;
    r->py    = (float)hpx * 0.5f;
    r->theta = 0.0f;
}

static void robot_reset(Robot *r, int wpx, int hpx)
{
    /* Preserve constant geometry across reset; everything else zero. */
    float axle = r->axle;
    memset(r, 0, sizeof *r);
    r->axle  = axle;
    r->px    = (float)wpx * 0.5f;
    r->py    = (float)hpx * 0.5f;
    r->theta = 0.0f;
}

/* ── §6.6 robot_tick — one frame of physics ──────────────────────── */

/*
 * The orchestrator. Reads the four commands the user has set
 * (v_cmd, w_cmd already updated), runs inverse kinematics to find
 * the wheel speeds, runs forward kinematics + Euler integration
 * to advance the pose, wraps if we left the screen, and pushes
 * the new position into the trail buffer.
 *
 * If paused, do nothing — the robot freezes mid-motion and the
 * renderer keeps showing whatever pose it had. The HUD continues
 * to update so the user can inspect numbers while paused.
 */
static void robot_tick(Robot *r, float dt, int wpx, int hpx)
{
    if (r->paused) return;

    compute_wheels(r);                      /* commands → wheel speeds */
    step_pose     (r, dt);                  /* wheel speeds → new pose */
    wrap_position (r, wpx, hpx);
    trail_push    (&r->trail, r->px, r->py);
}

/* ===================================================================== */
/* §7  scene — input → command translation, tick orchestration            */
/* ===================================================================== */

/*
 * Keys — one bool per action, set fresh each frame from getch().
 *   fwd, rev      throttle (ramps v_cmd up/down)
 *   left, right   turn (ramps w_cmd left/right)
 *   spin_l, spin_r  instant spin in place at ±W_MAX
 *   stop          full stop (zero both commands)
 *
 * Using a struct of bools (rather than a bitmask) keeps the
 * scene_apply_keys logic readable and lets multiple keys be active
 * simultaneously without bitwise ops.
 */
typedef struct {
    bool fwd, rev, left, right, spin_l, spin_r, stop;
} Keys;

typedef struct {
    Robot robot;
    Keys  keys;
    int   wpx, hpx;       /* world size in pixels */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->wpx = pw(cols);
    /* Reserve top row for HUD, bottom row for hint strip. */
    s->hpx = ph(rows - 2);
    memset(&s->keys, 0, sizeof s->keys);
    robot_init(&s->robot, s->wpx, s->hpx);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->wpx = pw(cols);
    s->hpx = ph(rows - 2);
}

/*
 * scene_apply_keys — translate the per-frame Keys flags into
 * updates to the robot's command state (v_cmd, w_cmd).
 *
 * Throttle:
 *   fwd / rev held → v_cmd ramps at V_RATE per second.
 *   No throttle held → v_cmd *= V_DECAY_PER_SEC^dt (≈ no decay
 *     at default V_DECAY_PER_SEC = 1.0).
 *
 * Turn:
 *   left / right held → w_cmd ramps at W_RATE per second.
 *   No turn held → w_cmd *= W_DECAY_PER_SEC^dt (strong damping).
 *
 * Spin override:
 *   spin_l / spin_r set v_cmd = 0 and w_cmd = ±W_MAX directly.
 *   This is "rotate in place", overriding any throttle ramp.
 *
 * Stop:
 *   space sets both commands to 0 immediately.
 *
 * All commands clamped to physical limits at the end.
 */
static void scene_apply_keys(Scene *s, float dt)
{
    Robot *r = &s->robot;
    Keys  *k = &s->keys;

    if (k->stop) { r->v_cmd = 0.0f; r->w_cmd = 0.0f; }

    /* Throttle */
    if (k->fwd) r->v_cmd += V_RATE * dt;
    if (k->rev) r->v_cmd -= V_RATE * dt;
    if (!k->fwd && !k->rev) r->v_cmd *= powf(V_DECAY_PER_SEC, dt);

    /* Turn */
    if (k->right) r->w_cmd += W_RATE * dt;
    if (k->left)  r->w_cmd -= W_RATE * dt;
    if (!k->right && !k->left) r->w_cmd *= powf(W_DECAY_PER_SEC, dt);

    /* Spin in place — direct override */
    if (k->spin_r) { r->v_cmd = 0.0f; r->w_cmd =  W_MAX; }
    if (k->spin_l) { r->v_cmd = 0.0f; r->w_cmd = -W_MAX; }

    r->v_cmd = clampf(r->v_cmd, -V_MAX, V_MAX);
    r->w_cmd = clampf(r->w_cmd, -W_MAX, W_MAX);
}

static void scene_tick(Scene *s, float dt)
{
    scene_apply_keys(s, dt);
    robot_tick(&s->robot, dt, s->wpx, s->hpx);
}

/* ===================================================================== */
/* §8  render                                                             */
/* ===================================================================== */

/* ── §8.1 in_bounds — guard for "draw me only if I'm on screen" ──── */

/*
 * Top row 0 reserved for HUD; bottom row reserved for hint strip.
 * Drawables must live in rows 1..rows-2. cy is allowed up to rows-1
 * exclusive; cy >= rows-1 means we're trying to draw on the hint row.
 */
static inline bool in_bounds(int cx, int cy, int cols, int rows)
{
    return cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1;
}

/* ── §8.2 draw_line_dotted — a directional arrow body ────────────── */

/*
 * Step uniformly in PIXEL space, converting each sample to a cell
 * and skipping duplicates. The character grows brighter along the
 * line so the arrow has a clear "from → to" feel without using
 * angle-dependent diagonal characters.
 *
 *   t < 0.40 → '.'   (faint, near the start)
 *   t < 0.75 → 'o'   (medium)
 *   t ≥ 0.75 → '0'   (boldest, near the tip)
 */
static void draw_line_dotted(float x0, float y0, float x1, float y1,
                             int cp, attr_t extra, int cols, int rows)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.5f) return;

    int   steps   = (int)(len / (float)CELL_W) + 1;
    int   prev_cx = -9999, prev_cy = -9999;

    for (int i = 0; i <= steps; i++) {
        float t  = (float)i / (float)(steps > 0 ? steps : 1);
        int   cx = px_to_cx(x0 + dx * t);
        int   cy = px_to_cy(y0 + dy * t);
        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (!in_bounds(cx, cy, cols, rows)) continue;

        chtype ch = (t < 0.40f) ? '.' : (t < 0.75f) ? 'o' : '0';
        mvaddch(cy, cx, ch | (chtype)COLOR_PAIR(cp) | (chtype)extra);
    }
}

/* ── §8.3 tip_char — arrowhead character chosen by direction ─────── */

/*
 * Cardinal directions get sharp ASCII arrows (>, v, <, ^) that
 * immediately read as directional. Diagonal octants get 'o' — a
 * round, neutral character that doesn't claim a specific angle
 * (which '\' or '/' would inevitably misrepresent on a 2:1 cell
 * grid).
 */
static chtype tip_char(float theta)
{
    float deg = fmodf(theta * (180.0f / (float)M_PI) + 360.0f, 360.0f);
    if (deg < 22.5f  || deg >= 337.5f) return '>';   /* east       */
    if (deg < 67.5f )                  return 'o';   /* south-east */
    if (deg < 112.5f)                  return 'v';   /* south      */
    if (deg < 157.5f)                  return 'o';   /* south-west */
    if (deg < 202.5f)                  return '<';   /* west       */
    if (deg < 247.5f)                  return 'o';   /* north-west */
    if (deg < 292.5f)                  return '^';   /* north      */
    return                                    'o';   /* north-east */
}

/* ── §8.4 draw_wheel_arrow — green/red arrow per wheel ───────────── */

/*
 * Draws a velocity arrow originating at the wheel's pixel position
 * and extending along ±heading by a length proportional to
 * |v_wheel| / V_MAX. Forward = green, reverse = red.
 *
 * If the wheel is essentially stopped (|v| < threshold), no arrow
 * is drawn — the absence is information.
 */
static void draw_wheel_arrow(float wx_px, float wy_px,
                             float v_wheel, float theta,
                             int cols, int rows)
{
    float alen = fabsf(v_wheel / V_MAX) * VEL_ARROW_PX;
    if (alen < 1.0f) return;

    float sign = (v_wheel >= 0.0f) ? 1.0f : -1.0f;
    float ex   = wx_px + cosf(theta) * sign * alen;
    float ey   = wy_px + sinf(theta) * sign * alen;
    int   cp   = (v_wheel >= 0.0f) ? CP_VEL_FWD : CP_VEL_REV;

    draw_line_dotted(wx_px, wy_px, ex, ey, cp, A_BOLD, cols, rows);

    int eax = px_to_cx(ex), eay = px_to_cy(ey);
    if (in_bounds(eax, eay, cols, rows)) {
        float ta = (v_wheel >= 0.0f) ? theta : theta + (float)M_PI;
        mvaddch(eay, eax, tip_char(ta) | (chtype)COLOR_PAIR(cp) | (chtype)A_BOLD);
    }
}

/* ── §8.5 render_trail — paint past positions ────────────────────── */

/*
 * Iterate newest-first (head−1, head−2, …). Age is the fraction of
 * the way from "newest" (k=0) to "oldest" (k=count-1):
 *
 *   age < 0.12  → BOLD '.' bright cyan   (very recent)
 *   age < 0.35  → DIM  '.' cyan          (recent, fading)
 *   age ≥ 0.35  → DIM  ':' dim blue      (old — note the ':' for
 *                                          visual recede)
 */
static void render_trail(const Robot *r, int cols, int rows)
{
    for (int k = 0; k < r->trail.count; k++) {
        int idx = (r->trail.head - 1 - k + TRAIL_CAP) % TRAIL_CAP;
        int tc  = px_to_cx(r->trail.px[idx]);
        int tr  = px_to_cy(r->trail.py[idx]);
        if (!in_bounds(tc, tr, cols, rows)) continue;

        float  age = (float)k / (float)(r->trail.count > 1 ? r->trail.count - 1 : 1);
        int    cp  = (age < 0.35f) ? CP_TRAIL_NEW : CP_TRAIL_OLD;
        attr_t at  = (age < 0.12f) ? A_BOLD       : A_DIM;
        char   ch  = (age < 0.35f) ? '.'          : ':';

        attron (COLOR_PAIR(cp) | at);
        mvaddch(tr, tc, (chtype)ch);
        attroff(COLOR_PAIR(cp) | at);
    }
}

/* ── §8.6 render_robot — body, wheels, arrows ────────────────────── */

/*
 * Painter's order — last write wins:
 *   1. Trail dots
 *   2. Wheel velocity arrows (green/red)
 *   3. Heading arrow (yellow)
 *   4. Wheel labels 'L', 'R'
 *   5. Body '@'   (always on top)
 *
 * The wheels sit perpendicular to the heading at ±axle/2 from the
 * body centre. In y-down screen coordinates:
 *
 *     left  = body + ( +sin θ, −cos θ ) · axle/2
 *     right = body + ( −sin θ, +cos θ ) · axle/2
 *
 * (Common error: math-textbook formulas assume y-up, so they have
 *  the signs flipped. Watch out.)
 */
static void render_robot(const Robot *r, int cols, int rows)
{
    int   cx   = px_to_cx(r->px);
    int   cy   = px_to_cy(r->py);
    float sinT = sinf(r->theta), cosT = cosf(r->theta);
    float half = r->axle * 0.5f;

    /* Wheel pixel positions (y-down perpendicular). */
    float lx_px = r->px + sinT * half,  ly_px = r->py - cosT * half;
    float rx_px = r->px - sinT * half,  ry_px = r->py + cosT * half;

    /* (1) Trail. */
    render_trail(r, cols, rows);

    /* (2) Wheel velocity arrows. */
    draw_wheel_arrow(lx_px, ly_px, r->vL, r->theta, cols, rows);
    draw_wheel_arrow(rx_px, ry_px, r->vR, r->theta, cols, rows);

    /* (3) Heading arrow from body centre. */
    {
        float ex = r->px + cosT * ARROW_PX;
        float ey = r->py + sinT * ARROW_PX;
        draw_line_dotted(r->px, r->py, ex, ey, CP_HEAD, A_BOLD, cols, rows);

        int tx = px_to_cx(ex), ty = px_to_cy(ey);
        if (in_bounds(tx, ty, cols, rows))
            mvaddch(ty, tx, tip_char(r->theta) |
                            (chtype)COLOR_PAIR(CP_HEAD) | (chtype)A_BOLD);
    }

    /* (4) Wheel labels. */
    int lx = px_to_cx(lx_px), ly = px_to_cy(ly_px);
    int rx = px_to_cx(rx_px), ry = px_to_cy(ry_px);
    if (in_bounds(lx, ly, cols, rows)) {
        attron (COLOR_PAIR(CP_WHL_L) | A_BOLD);
        mvaddch(ly, lx, 'L');
        attroff(COLOR_PAIR(CP_WHL_L) | A_BOLD);
    }
    if (in_bounds(rx, ry, cols, rows)) {
        attron (COLOR_PAIR(CP_WHL_R) | A_BOLD);
        mvaddch(ry, rx, 'R');
        attroff(COLOR_PAIR(CP_WHL_R) | A_BOLD);
    }

    /* (5) Body — drawn last, never occluded. */
    if (in_bounds(cx, cy, cols, rows)) {
        attron (COLOR_PAIR(CP_BODY) | A_BOLD);
        mvaddch(cy, cx, '@');
        attroff(COLOR_PAIR(CP_BODY) | A_BOLD);
    }
}

/* ── §8.7 render_hud — yellow status row 0, cyan hint bottom row ─── */

/*
 * Row 0 (PAIR_HUD, BOLD): fps, body pose, body velocity, ω, turn
 *   radius, and the two wheel speeds. Everything in one
 *   left-justified strip.
 *
 * Bottom row (PAIR_HINT, BOLD): the full key list. Stays constant.
 *
 * Both pairs sit on default background (-1) so they remain legible
 * regardless of what the simulation paints behind them.
 */
static void render_hud(const Robot *r, double fps, int cols, int rows)
{
    float v     = (r->vL + r->vR) * 0.5f;
    float omega = (r->vR - r->vL) / r->axle;
    float R     = (fabsf(omega) > 1e-3f) ? (v / omega) : 1e9f;
    float deg   = r->theta * (180.0f / (float)M_PI);

    char buf[HUD_BUF_LEN];
    if (fabsf(R) > 9999.0f) {
        snprintf(buf, sizeof buf,
                 " %5.1f fps  pose:(%.0f,%.0f) %+6.1f°  v:%+6.1fpx/s  "
                 "ω:%+5.2fr/s  R:INF  L:%+6.1f R:%+6.1f  %s ",
                 fps, r->px, r->py, deg, v, omega, r->vL, r->vR,
                 r->paused ? "PAUSED" : "running");
    } else {
        snprintf(buf, sizeof buf,
                 " %5.1f fps  pose:(%.0f,%.0f) %+6.1f°  v:%+6.1fpx/s  "
                 "ω:%+5.2fr/s  R:%+6.0f  L:%+6.1f R:%+6.1f  %s ",
                 fps, r->px, r->py, deg, v, omega, R, r->vL, r->vR,
                 r->paused ? "PAUSED" : "running");
    }

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 0, "%s", buf);
    /* Pad the rest of row 0 so theme bg doesn't show through. */
    int used = (int)strlen(buf);
    for (int x = used; x < cols; x++) mvaddch(0, x, ' ');
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:stop  p:pause  r:reset  "
             "WS:throttle  AD:turn  ZE:spin in place ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §8.8 scene_draw — one frame's worth of rendering ────────────── */

static void scene_draw(const Scene *s, double fps, int cols, int rows)
{
    erase();
    render_robot(&s->robot, cols, rows);
    render_hud  (&s->robot, fps, cols, rows);
}

/* ===================================================================== */
/* §9  screen — ncurses init / present                                    */
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
    typeahead(-1);              /* don't let stdin interrupt frame writes */
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
/* §10  app — signals, resize, variable-dt main loop                      */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * collect_input — drain the ncurses input queue, populate the
 * per-frame Keys flags, and handle "instant-effect" keys (quit,
 * pause, reset). Multiple keys can be active in one frame so the
 * collector ORs the bools together.
 */
static void collect_input(App *app)
{
    Keys *k = &app->scene.keys;
    memset(k, 0, sizeof *k);

    int ch;
    while ((ch = getch()) != ERR) {
        switch (ch) {
        case 'q': case 'Q': case 27:
            app->running = 0;
            break;

        case 'w': case 'W': case KEY_UP:    k->fwd    = true; break;
        case 's': case 'S': case KEY_DOWN:  k->rev    = true; break;
        case 'a': case 'A': case KEY_LEFT:  k->left   = true; break;
        case 'd': case 'D': case KEY_RIGHT: k->right  = true; break;
        case 'e': case 'E':                 k->spin_r = true; break;
        case 'z': case 'Z':                 k->spin_l = true; break;
        case ' ':                           k->stop   = true; break;

        case 'p': case 'P':
            app->scene.robot.paused = !app->scene.robot.paused;
            break;

        case 'r': case 'R':
            robot_reset(&app->scene.robot,
                        app->scene.wpx, app->scene.hpx);
            trail_clear(&app->scene.robot.trail);
            break;

        default: break;
        }
    }
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

    int64_t last_ns      = clock_ns();
    int64_t fps_accum_ns = 0;
    int     fps_frames   = 0;
    double  fps_display  = 0.0;
    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    while (app->running) {

        /* (1) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize (&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            last_ns = clock_ns();
        }

        /* (2) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3) drain input */
        collect_input(app);

        /* (4) advance physics */
        scene_tick(&app->scene, dt);

        /* (5) rolling fps display (0.5 s window) */
        fps_accum_ns += dt_ns;
        fps_frames++;
        if (fps_accum_ns >= NS_PER_SEC / 2) {
            fps_display = (double)fps_frames * 1e9
                        / (double)fps_accum_ns;
            fps_accum_ns = 0;
            fps_frames   = 0;
        }

        /* (6) draw + present */
        scene_draw(&app->scene, fps_display,
                   app->screen.cols, app->screen.rows);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
