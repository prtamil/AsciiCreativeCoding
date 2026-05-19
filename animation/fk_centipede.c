/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_centipede.c — FK centipede with gait-driven legs
 *
 * DEMO: A 24-segment centipede crawls smoothly across the terminal,
 *       carving a sinusoidal S-curve. Ten leg pairs swing in a
 *       contralateral antiphase gait — left and right exactly half a
 *       cycle apart, with a travelling wave from head to tail. Two
 *       antennae sway forward of the head on their own faster oscillator.
 *
 * Study alongside: snake_forward_kinematics.c (same trail-buffer body FK)
 *                  fk_tentacle_forest.c        (same dense-line renderer)
 *
 * Section map:
 *   §1  config     — all tunables in one place
 *   §2  clock      — monotonic clock + sleep (verbatim from framework)
 *   §3  color      — 10 themes + spec HUD/hint pairs
 *   §4  coords     — pixel↔cell aspect-ratio helpers
 *   §5  entity     — Centipede: trail-buffer body + stateless leg FK
 *       §5a  trail helpers
 *       §5b  edge_bias + move_head
 *       §5c  compute_joints
 *       §5d  compute_legs
 *       §5e  rendering helpers
 *       §5f  render_centipede
 *       §5g  render_antennae
 *   §6  scene      — scene_init / scene_tick / scene_draw
 *   §7  screen     — ncurses double-buffer display layer
 *   §8  app        — signals, resize, main game loop
 *
 * Keys:  q / ESC      quit                 space    pause / resume
 *        ↑ / ↓        move speed           ←/→ a/d  undulation freq
 *        w / s        amplitude            t        cycle theme
 *        [ / ]        time scale (0.25× .. 4.0×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/fk_centipede.c \
 *       -o fk_centipede -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Two FK mechanisms running in parallel, plus a steering
 *                 layer that keeps the centipede inside the screen.
 *
 *                 BODY uses path-following (trail-buffer) FK: every frame
 *                 the head's position is pushed into a circular trail;
 *                 each body joint i is placed at arc-length i*SEG_LEN_PX
 *                 backward along the recorded path. The body follows
 *                 the exact path the head carved — no per-segment angle
 *                 math, no constraint solver.
 *
 *                 LEGS and ANTENNAE use stateless sinusoidal FK: their
 *                 positions are pure functions of wave_time and an
 *                 element-specific phase offset. No gait state machine,
 *                 no contact constraints, no IK. The leg phase offset
 *                 i*pi/N_LEGS produces a head-to-tail travelling wave;
 *                 right legs run at phi+pi (contralateral antiphase) —
 *                 the defining trait of alternating wave gait.
 *
 *                 STEERING uses Reynolds-style boundary repulsion: when
 *                 the head enters a margin around a screen edge, a
 *                 quadratic-falloff bias is added to the heading rate,
 *                 turning the centipede back toward open space. There
 *                 is no toroidal wrap — the centipede is always on screen.
 *
 * Data-structure: Circular trail Vec2 trail[TRAIL_CAP=4096]. Body joint
 *                 array joint[BODY_SEGS+1]. Leg arrays leg_left/right
 *                 [N_LEGS][3] holding hip(0)/knee(1)/foot(2). Two phase
 *                 accumulators: wave_time (master clock for legs +
 *                 antennae) and turn_phase (integrates turn_freq*dt so
 *                 the user can adjust frequency without phase jumps).
 *
 * Rendering     : Painter's order (deepest first): legs (dim accents) →
 *                 body lines (tail→head, last write wins on overlap) →
 *                 joint node markers (bold) → antennae → head arrow.
 *                 Body uses a 7-pair gradient indexed by segment number
 *                 with depth-cued attributes (A_BOLD head / A_NORMAL mid
 *                 / A_DIM tail) so the body visibly tapers away.
 *
 * Performance   : Variable timestep at render rate — no fixed-step
 *                 accumulator, no alpha interpolation. The whole sim
 *                 (heading sinusoid + Euler translate) is non-stiff so
 *                 variable dt is safe and gives motion as smooth as the
 *                 terminal can render. O(BODY_SEGS + N_LEGS) per frame
 *                 for simulation; render is O((BODY_SEGS + 4*N_LEGS) *
 *                 cells_per_segment); ncurses doupdate() sends only
 *                 changed cells.
 *
 * References
 * ──────────
 *   ── Forward kinematics + IK contrast ─────────────────────────────
 *   [1] Aristidou, A. & Lasenby, J. (2011), "FABRIK: a fast, iterative
 *       solver for the inverse kinematics problem", Graphical Models
 *       73(5), pp. 243-260 — the iterative-IK paper that the trail-
 *       following body chain is the FK COUNTERPART of.  Read it for
 *       why FK (this file) is so much simpler than the inverse
 *       problem most posing tools have to solve.
 *   [2] Buss, S. R. (2004), "Introduction to Inverse Kinematics with
 *       Jacobian Transpose, Pseudoinverse and Damped Least Squares",
 *       UCSD course notes — the standard IK reference; useful for
 *       readers who want to understand what FK is NOT doing.
 *
 *   ── Procedural locomotion / steering ─────────────────────────────
 *   [3] Reynolds, C. (1999), "Steering Behaviors for Autonomous
 *       Characters", Game Developers Conference — path following
 *       motivates the trail buffer; obstacle avoidance motivates the
 *       boundary-repulsion steering term in §move_head.
 *       https://www.red3d.com/cwr/steer/
 *   [4] Reynolds, C. (1987), "Flocks, Herds, and Schools: A
 *       Distributed Behavioral Model", SIGGRAPH '87 — earlier work
 *       establishing the local-rules → emergent-behaviour pattern
 *       that this file's leg-phase scheduler follows.
 *
 *   ── Biological gait references ───────────────────────────────────
 *   [5] Manton, S. M. (1952), "The evolution of arthropodan
 *       locomotory mechanisms", J. Linn. Soc. London (Zoology) 42,
 *       pp. 93-117 — biological reference for myriapod gaits;
 *       informs the wave-front leg-phase pattern in §compute_legs.
 *   [6] Full, R. J. & Tu, M. S. (1991), "Mechanics of a rapid running
 *       insect: two-, four- and six-legged locomotion", J. Exp. Biol.
 *       156, pp. 215-231 — quantitative reference for leg-pair phase
 *       offsets in segmented arthropods.
 *
 *   ── Timestep / animation ─────────────────────────────────────────
 *   [7] Fiedler, G. (2004), "Fix Your Timestep!", gafferongames.com
 *       — articulates when fixed-step matters and when variable-step
 *       is the better tool; this file uses variable-step because
 *       the FK chain is unconditionally stable.
 *
 *   ── Rendering / ncurses ─────────────────────────────────────────
 *   [8] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the body-segment glyph ramp.
 *   [9] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the newscr/curscr diff pipeline used
 *       in scene_draw().
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [10] https://en.wikipedia.org/wiki/Forward_kinematics
 *  [11] https://en.wikipedia.org/wiki/Inverse_kinematics
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Three cleanly separable simulations share a body. The BODY is a chain
 * that follows wherever the head went — the trail buffer is a recording
 * of where the head has been; each body joint just rewinds the tape by
 * its own segment length. The LEGS (and ANTENNAE) are clocks — at any
 * moment t each appendage knows its own pose by plugging t plus its
 * index into a single formula. STEERING is a soft fence that turns the
 * head away from screen edges before it can leave. No body↔leg
 * coupling, no contact, no ground.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Body  : a magnetic strip behind the head. Each joint is a bead
 *         threaded onto the strip at a fixed distance from its
 *         predecessor. Move the head; the strip extends forward and
 *         the beads slide along it, always exactly SEG_LEN_PX apart.
 *
 * Legs  : wind-up toys keyed to a master clock. Toy i is offset from
 *         toy 0 by i*pi/N_LEGS; the right-side twins are 180° behind
 *         the left ones. They swing without any awareness of the
 *         ground or each other — yet because the master clock is
 *         shared, the collective looks coordinated.
 *
 * Edges : a soft repulsive field around the screen border. Distance >
 *         REPEL_MARGIN: zero force. Distance shrinking toward zero:
 *         quadratically growing inward push on the head's heading.
 *         The centipede curves back well before reaching the edge.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Measure dt = wall-clock since last frame. Multiply by time_scale.
 *  2. wave_time += dt.  turn_phase += turn_freq * dt  (so changing
 *     turn_freq mid-run does not cause a phase jump).
 *  3. heading_rate = turn_amp * sin(turn_phase) + edge_bias(head_pos).
 *  4. heading += heading_rate * dt.
 *  5. Translate the head along heading at move_speed. (No wrap — the
 *     edge bias guarantees the head stays inside.)
 *  6. Push the head position into the circular trail.
 *  7. Body: for each joint i, walk the trail backward summing Euclidean
 *     distance until the running total reaches i*SEG_LEN_PX, then
 *     linearly interpolate within the bracketing trail segment.
 *  8. Legs: for each leg pair i, take body direction at the attachment
 *     joint, derive phi_L = wave_time*GAIT_FREQ + i*pi/N_LEGS and
 *     phi_R = phi_L + pi. Compute upper/lower angles, then forward-
 *     kinematic the chain hip → knee → foot.
 *  9. Render in painter's order: legs → body lines (tail→head) →
 *     joint nodes → antennae → head arrow.
 *
 * KEY FORMULAS
 * ────────────
 *  Edge bias    : let d = distance from head to nearest edge. Per axis:
 *                 push = (d < MARGIN) ? (1 - d/MARGIN)^2 : 0
 *                 desired_dir = atan2(push_y, push_x)
 *                 bias = REPEL_GAIN * angle_diff(desired_dir, heading)
 *
 *  Trail sample : find smallest k with sum_{j=1..k} |Δ_j| >= s
 *                 interpolate inside that bracketing segment
 *
 *  Gait phase   : phi_L = wave_time * GAIT_FREQ + i * (pi / N_LEGS)
 *                 phi_R = phi_L + pi      (contralateral antiphase)
 *
 *  Upper-leg θ  : upper = body_dir ± LEG_SPLAY + SWING_AMP * sin(phi)
 *  Lower-leg θ  : lower = upper + LEG_BEND + LOWER_SWING * sin(phi + pi/4)
 *                 ( pi/4 phase shift makes the foot trace an ellipse,
 *                   reducing foot-into-body clipping vs a circular arc )
 *
 *  FK chain     : hip   = joint + BODY_OFFSET * (cos n,    sin n   )
 *                 knee  = hip   + UPPER_LEN  * (cos upper, sin upper)
 *                 foot  = knee  + LOWER_LEN  * (cos lower, sin lower)
 *
 *  Antenna seg  : ang_s = heading ± ANT_SPLAY
 *                       + ANT_AMP * sin(wave_time * ANT_FREQ + s*0.4)
 *                 chain forward UPPER_LEN/LOWER_LEN style for ANT_SEGS.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - Trail starvation. At scene_init the trail is empty; trail_sample
 *    would return the head for every joint and the body would appear as
 *    a single cell. Pre-fill TRAIL_CAP entries 1 px apart behind the
 *    start heading so the body is fully extended on frame 1.
 *
 *  - Edge bias vs sinusoidal turn. The repel bias adds to the sine
 *    turn_amp wave. When the centipede is in the margin and the sine
 *    is also pushing it outward, the bias must be strong enough to win.
 *    REPEL_GAIN is tuned so the bias dominates near the edge.
 *
 *  - Resize during physics. The head may end up in (or past) the new
 *    margin. app_do_resize clamps it inside; the next tick's edge bias
 *    smoothly steers back toward center.
 *
 *  - Glyph aliasing. The four ASCII line glyphs ('-', '\', '|', '/')
 *    sample a continuous angle; near boundaries (22.5°, 67.5°, ...)
 *    the glyph flickers. Folding to [0°, 180°) halves the boundary
 *    crossings; the residual flicker is barely perceptible.
 *
 *  - Suspend / lid-close. dt can grow huge between frames; capped at
 *    100 ms so the centipede does not teleport.
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Default config (MOVE_SPEED=65, TURN_AMP=0.52, TURN_FREQ=0.95):
 *    body fully extended on frame 1, smooth S-curve in ~6.6 s, all 20
 *    leg tips visibly stepping (10 per side), antennae visibly waving,
 *    no gaps along the body, no glyph doubling at joint nodes.
 *  - Set TURN_AMP=0 (`s` repeatedly) → straight-line walk; the centipede
 *    still curves away from the edges via the boundary repulsion.
 *  - Crank MOVE_SPEED with ↑ → legs step faster, body translation
 *    stays smooth. Expect leg cycle to remain at GAIT_FREQ=2.5 rad/s.
 *  - Press space mid-curve — body must freeze cleanly and resume
 *    identically (no backwards jolt or glyph flash).
 *  - Press [ several times to slow time → motion stays buttery; press ]
 *    to speed up to 4× → still smooth.
 *  - Cycle themes with `t` — every theme must remain readable; the
 *    rear quarter (A_DIM) must not vanish into the terminal background.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read fk_helloworld.c first if FK basics aren't
 *      automatic; this file scales the FK primitive to a 24-segment
 *      body + 20 legs + 2 antennae.
 *   2. §5 entity — THE HEART of this file. In sub-section order:
 *        §5a trail helpers          ← circular buffer of head positions
 *        §5b edge_bias + move_head  ← steering, screen-edge repulsion
 *        §5c compute_joints         ← BODY FK from trail (T1-T2)
 *        §5d compute_legs           ← LEG FK from gait clock (T3-T5)
 *        §5e-§5g rendering helpers  ← painter's order, body taper, antennae
 *      Read AFTER tutorials T1-T6.
 *   3. §6 scene — orchestrator: tick + draw, plus resize handling.
 *   4. §1-§4 + §7-§8 — config / clock / colour / screen / app loop.
 *      Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   trail[]            circular buffer of recent head positions.
 *   trail_head, trail_count
 *                      head = next-write index, count = filled cells.
 *   joint[i]           position of body joint i (0 = head, BODY_SEGS
 *                      = tail).
 *   leg_left/right [i] [3]
 *                      hip / knee / foot of leg i. left and right
 *                      are MIRRORED across the body axis.
 *   wave_time          MASTER CLOCK driving every sinusoid (legs,
 *                      antennae). Increments by dt every frame.
 *   turn_phase         INTEGRATED phase for the body's own
 *                      sinusoidal heading wobble.
 *   heading            head's facing angle (rad).
 *   phi_L, phi_R       per-leg gait phase. phi_R = phi_L + π
 *                      (contralateral antiphase).
 *
 * Background you need
 * ───────────────────
 *   - The FK primitive (fk_helloworld T2). Each leg is FK applied
 *     three times: hip → knee → foot.
 *   - Circular buffers — modular arithmetic to walk a fixed-size
 *     ring of recent samples.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - IK. No solver here; FK only.
 *   - Spline math. The trail buffer is "linear interpolation between
 *     stored points" — no Catmull-Rom, no cubics.
 *   - Locomotion / contact constraints. The legs swing in air; no
 *     ground, no ground reaction, no foot slip detection.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Six tutorials that build a 24-body, 20-leg FK creature from
 * first principles.
 *
 *   T1  The path-following body — a chain that REWINDS the trail
 *   T2  Arc-length sampling — placing joints by distance, not index
 *   T3  Stateless leg FK — each leg is a clock with an offset
 *   T4  Travelling-wave gait — head-to-tail phase shift
 *   T5  Contralateral antiphase — why right legs lag left by π
 *   T6  Steering — soft fence around the screen edge
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE PATH-FOLLOWING BODY — A CHAIN THAT REWINDS THE TRAIL
 * ────────────────────────────────────────────────────────────
 * The naïve way to animate a long body is N joint angles and FK.
 * That works for a 3-link arm; it gets ugly past 8 links because
 * you have to AUTHOR every joint angle each frame.
 *
 * The trick: don't author angles. Record the path the HEAD has
 * walked and place each body joint at a fixed ARC-LENGTH back
 * along that path.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │  trail (newest at left):                         │
 *      │                                                  │
 *      │   head ●─●─●─●─●─●─●─●─●─●─●─●─●─●─●─● tail      │
 *      │   ⌐ where the head was the last 24 segments ⌐    │
 *      │                                                  │
 *      │   joint[0] = head position now                   │
 *      │   joint[1] = head position 1 segment ago         │
 *      │   joint[2] = head position 2 segments ago        │
 *      │   ...                                            │
 *      └──────────────────────────────────────────────────┘
 *
 * Move the head; the body AUTOMATICALLY follows the path the
 * head just carved. No per-segment angle math, no constraint
 * solver. The head is the only thing the simulation truly drives;
 * every other joint is a delayed echo of the head's history.
 *
 * Pros: smooth, no kinks, no IK iterations, body length stays
 * exact regardless of how the head turns.
 *
 * Cons: the body cannot pose INDEPENDENTLY of where the head
 * went. A real centipede can curl into a circle on the spot;
 * this one can't — only places it has been are reachable.
 *
 * T2  ARC-LENGTH SAMPLING — PLACING JOINTS BY DISTANCE, NOT INDEX
 * ───────────────────────────────────────────────────────────────
 * The trail buffer stores discrete samples (one per frame). But
 * the head moves at variable speed, so frame-i and frame-(i-1)
 * are NOT separated by a fixed pixel distance. Indexing the
 * trail by position would give a body whose visible length
 * fluctuates with frame rate.
 *
 * Solution: walk the trail BACKWARDS while keeping a running
 * sum of Euclidean distances, and place joint i where the
 * running sum reaches i · SEG_LEN_PX:
 *
 *     for i in 1..BODY_SEGS:
 *       target_distance = i · SEG_LEN_PX
 *       walk back through trail summing |trail[k+1] − trail[k]|
 *       when sum reaches target_distance:
 *         interpolate inside the bracketing trail segment
 *         joint[i] = that interpolated point
 *
 * This is ARC-LENGTH PARAMETRISATION — sampling along the path
 * by distance rather than by index. The body's visible length
 * is therefore EXACTLY 24 × SEG_LEN_PX every frame, regardless
 * of how slow or fast the head was moving.
 *
 * Implementation in §5c compute_joints uses two helpers from
 * §5a (trail_at, trail_size) so the math reads as plain English.
 *
 * T3  STATELESS LEG FK — EACH LEG IS A CLOCK WITH AN OFFSET
 * ─────────────────────────────────────────────────────────
 * Twenty legs. Twenty independent state machines? No.
 *
 * Every leg's pose is a PURE FUNCTION of (wave_time, leg_index,
 * left/right). No per-leg state. No history. No "which step is
 * this leg in?" — the master clock and the index already encode
 * everything.
 *
 *     phi_i = wave_time · GAIT_FREQ + i · π / N_LEGS
 *
 *     upper_angle = body_dir ± LEG_SPLAY + SWING_AMP · sin(phi_i)
 *     lower_angle = upper_angle + LEG_BEND + LOWER_SWING ·
 *                   sin(phi_i + π/4)
 *
 * Then plain FK gives the leg joint positions:
 *
 *     hip  = joint[hip_segment] + body_offset · body_dir_perp
 *     knee = hip  + UPPER_LEN · (cos upper, sin upper)
 *     foot = knee + LOWER_LEN · (cos lower, sin lower)
 *
 * Three FK applications per leg. Twenty legs. Sixty trig
 * evaluations per frame. Trivial.
 *
 * Why no state? Because there's no contact, no slip detection,
 * no gait change. The legs are STYLED appendages, not biome-
 * chanical actuators. As long as they LOOK like centipede legs,
 * it's enough.
 *
 * T4  TRAVELLING-WAVE GAIT — HEAD-TO-TAIL PHASE SHIFT
 * ───────────────────────────────────────────────────
 * Why does phi_i include "i · π / N_LEGS"? Because that's the
 * PHASE OFFSET that creates a head-to-tail TRAVELLING WAVE in
 * the leg motion.
 *
 * Without the offset (every leg in phase): all legs swing
 * forward together, then back together. Looks like a wind-up
 * toy. Wrong.
 *
 * With the offset i · π / N_LEGS: leg 0 swings forward; a beat
 * later leg 1 starts swinging forward; leg 2 a beat after
 * that; ... leg i is half a cycle behind leg 0 by the time
 * i = N_LEGS. The motion APPEARS as a wave running down the
 * body — just like a real centipede.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │  legs as a function of time:                     │
 *      │                                                  │
 *      │  t=0:    /                                       │
 *      │  t=Δ:    | /                                     │
 *      │  t=2Δ:   \ | /                                   │
 *      │  t=3Δ:    \ \ | /                                │
 *      │                                                  │
 *      │ wave runs head → tail at speed proportional to   │
 *      │ GAIT_FREQ × π/N_LEGS.                            │
 *      └──────────────────────────────────────────────────┘
 *
 * The wave speed and direction can be tuned by changing the
 * coefficient on i (negative coefficient → tail-to-head wave).
 * Real centipedes (Manton 1952) propagate the wave from
 * posterior to anterior; ours runs the cinematically more
 * obvious head → tail.
 *
 * T5  CONTRALATERAL ANTIPHASE — WHY RIGHT LEGS LAG LEFT BY π
 * ──────────────────────────────────────────────────────────
 * Each leg pair (left + right at the same body segment) is
 * EXACTLY HALF A CYCLE OFF. When the left leg is forward, the
 * right is backward; when the left is backward, the right is
 * forward.
 *
 *     phi_L_i = wave_time · GAIT_FREQ + i · π / N_LEGS
 *     phi_R_i = phi_L_i + π
 *
 * That + π is the "contralateral antiphase" rule. It's not a
 * cosmetic choice — it's the defining trait of "alternating
 * wave gait" used by real centipedes and millipedes.
 *
 * Result: at any moment, half the body's legs are pushing back
 * (propulsion), half are swinging forward (recovery). The
 * combination keeps net force balanced and net body roll near
 * zero — even though we don't simulate ground contact, the
 * motion still LOOKS biomechanically grounded.
 *
 * Tip: if you want a "trotting" or "scuttling" gait, change
 * the offset between left and right. + π/2 looks like crab
 * scuttle. + 0 (in phase) looks like a galloping rabbit. The
 * phase offset is the entire gait choreography.
 *
 * T6  STEERING — SOFT FENCE AROUND THE SCREEN EDGE
 * ────────────────────────────────────────────────
 * Without steering, the head wanders off-screen and the body
 * gets stuck in the corners. We need a way to KEEP THE HEAD
 * INSIDE.
 *
 * Hard wall (clamp head inside): the centipede slams into the
 * boundary and slides along it. Looks rigid.
 *
 * Soft fence (used here): a region near each edge applies a
 * CONTINUOUS PUSH on the head's heading. Far from the edge:
 * zero force. Approaching the edge: quadratically increasing
 * push toward the centre.
 *
 *     d = distance from head to nearest edge
 *     if d < REPEL_MARGIN:
 *       push = (1 − d / REPEL_MARGIN)²       ∈ (0, 1]
 *       desired_dir = away_from_nearest_edge
 *       bias = REPEL_GAIN · angle_diff(desired_dir, heading)
 *
 * The bias adds to the body's normal sinusoidal heading wobble.
 * In open space the wobble dominates; near the edge the bias
 * dominates and turns the head away.
 *
 * Quadratic falloff (Reynolds 1999 §3.3): smooth, non-
 * discontinuous, easily TUNED via just two parameters
 * (REPEL_MARGIN sets the ZONE; REPEL_GAIN sets the STRENGTH).
 * The centipede curves gently away well before reaching the
 * edge, so the boundary is never visible as a hard wall.
 *
 * Same trick is reused in fk_tentacle_forest.c (steering swarm
 * tentacles) and is a building block for boids (cohesion +
 * separation + alignment, all "soft forces").
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/* M_PI is a POSIX extension, not standard C99/C11 — provide a fallback
 * so the build never fails on strict-conformance toolchains. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
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

/* All magic numbers live here. Never scatter literals through the code. */
enum {
    /* Render frame-rate target. Variable-timestep simulation so the only
     * thing this controls is the sleep cap at end of frame. */
    TARGET_FPS       =  60,

    /* HUD layout. */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    /* ncurses pair IDs. Pairs 1..N_PAIRS are the body gradient
     * (tail→head); PAIR_HUD/PAIR_HINT are reserved per CLAUDE.md HUD spec. */
    N_PAIRS          =   7,
    PAIR_HUD         =   8,
    PAIR_HINT        =   9,

    /* Body geometry.
     * 24 segments × 20 px = 480 px ≈ 30 terminal rows — long sinuous body. */
    BODY_SEGS        =  24,

    /* Leg pairs. 10 across 24 body segments = 1 pair every ~2.4 segs,
     * matching the dense-leg look of a real centipede. */
    N_LEGS           =  10,

    /* Antenna geometry — short forward-projecting feelers. */
    ANT_SEGS         =   4,

    /* Circular head-position buffer.
     * Body length 480 px. Worst case: MOVE_SPEED_MIN=10, time_scale=4 →
     * 40 px/s; at TARGET_FPS=60 → 0.67 px/frame. Need 480/0.67 ≈ 720
     * frames of trail. 4096 leaves generous head-room. */
    TRAIL_CAP        = 4096,

    N_THEMES         =  10,    /* cycled with the `t` key at runtime */
};

/* SEG_LEN_PX (20 px = 2.5 cells): body segment length — balances arc-
 * length resolution against visual segment separation.
 * DRAW_STEP_PX (5 px < CELL_W): line-stamping step; see draw_line_dense
 * for why it must be smaller than cell width. */
#define SEG_LEN_PX     20.0f
#define DRAW_STEP_PX    5.0f

/*
 * Leg geometry — all distances in pixel space (same coords as body).
 *
 *      body axis ──────── body_joint ──────── (body direction →)
 *                              │
 *                BODY_OFFSET   │ lateral normal
 *                              ↓
 *                            hip ─── UPPER_LEN ──→ knee
 *                                                   │
 *                                          LOWER_LEN│
 *                                                   ↓
 *                                                 foot
 *
 * SWING_AMP=0.5 (was 0.4) and GAIT_FREQ=2.5 (was 2.0): widened so the
 * foot crosses ~1 full terminal cell per swing instead of ~0.7.
 * Sub-cell motion cannot render smoothly at terminal resolution.
 */
#define UPPER_LEN      14.0f   /* hip → knee segment length (px)        */
#define LOWER_LEN      12.0f   /* knee → foot segment length (px)       */
#define LEG_SPLAY       1.2f   /* base splay from body axis (rad)       */
#define SWING_AMP       0.5f   /* upper-leg gait swing amplitude (rad)  */
#define LOWER_SWING     0.35f  /* lower-leg swing amplitude (rad)       */
#define LEG_BEND       -0.5f   /* static knee-bend offset (rad)         */
#define BODY_OFFSET     8.0f   /* hip lateral offset from centerline    */

/*
 * Antenna geometry. Antennae are short forward-projecting feelers that
 * sway at their own (faster) oscillator — visual life around the head.
 *
 * ANT_SEG_LEN=7, ANT_SEGS=4 → 28 px ≈ 3.5 cells of feeler.
 * ANT_FREQ=4.5 (faster than legs) — feelers feel "alert", not lazy.
 * ANT_SPLAY=0.55 → about ±32° from heading: forward-and-out.
 */
#define ANT_SEG_LEN     7.0f
#define ANT_SPLAY       0.55f
#define ANT_AMP         0.45f
#define ANT_FREQ        4.5f

/*
 * Gait and locomotion.
 *
 * TURN_AMP/TURN_FREQ chosen so the body carves a smooth S-curve:
 *   peak heading deflection = TURN_AMP / TURN_FREQ ≈ 0.55 rad ≈ 31°
 *   undulation period       = 2π / TURN_FREQ       ≈ 6.6 s
 * GAIT_FREQ = 2.5 rad/s → leg cycle ≈ 2.5 s per stride.
 * MOVE_SPEED = 65 px/s — slightly slower than the snake (80) because
 *   the centipede body is longer and benefits from screen time.
 */
#define GAIT_FREQ       2.5f
#define MOVE_SPEED     65.0f
#define MOVE_SPEED_MIN 10.0f
#define MOVE_SPEED_MAX 300.0f
#define TURN_AMP        0.52f
#define TURN_AMP_MIN    0.0f
#define TURN_AMP_MAX    3.0f
#define TURN_FREQ       0.95f
#define TURN_FREQ_MIN   0.05f
#define TURN_FREQ_MAX   5.0f

/*
 * Boundary repulsion — a Reynolds-style "soft fence" around the screen.
 *
 * REPEL_MARGIN  = 64 px (8 cells): centipede begins to feel the edge
 *                 this far inside it. Larger margin → gentler, lazier
 *                 turn-back. Smaller → tighter cornering near edges.
 * REPEL_GAIN    = 4.0 rad/s: maximum angular bias when pinned against
 *                 the edge. Tuned to comfortably overpower TURN_AMP_MAX
 *                 (3.0) so the boundary always wins.
 *
 * The push falls off quadratically: full strength only AT the edge,
 * gentle nudge at the margin. Prevents the centipede from oscillating
 * along an edge looking trapped.
 */
#define REPEL_MARGIN    64.0f
#define REPEL_GAIN       4.0f

/*
 * Time scale — user-controlled simulation speed multiplier.
 * 0.25× to 4×, default 1×. Stepped by ×/÷ TIME_SCALE_STEP via [/].
 */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

/* Timing primitives — verbatim from framework.c. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Terminal cell dimensions — the aspect-ratio bridge between physics and
 * display. 1 px represents the same physical distance in x and y. A
 * typical terminal cell is ~2× taller than it is wide. */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);   /* never goes backward */
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;                  /* over-budget frame: skip */
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color — 10 switchable themes + spec-fixed HUD pairs                */
/* ===================================================================== */

/*
 * Theme — palette for ONE of the 10 named looks (Amber, Matrix, …).
 *
 * Intent
 *   The centipede's body is rendered as a GRADIENT from tail to head:
 *   index 0 = tail-most segment, index N_PAIRS-1 = head.  Each theme
 *   names one such gradient — a tail-to-head sweep of N_PAIRS xterm-
 *   256 colour indices.  HUD/HINT pairs are NOT themed; they stay
 *   bright-yellow / bright-cyan forever per CLAUDE.md HUD spec so the
 *   status text reads cleanly against any animation underneath.
 *
 * Why "tail → head" ordering, not "head → tail"
 *   ncurses pair indices count UP from 1; aligning the array with the
 *   pair numbering means pair (i+1) gets body[i] and the body renderer
 *   can do `init_pair(i + 1, ...)` in a single loop.  Reversing the
 *   convention would force a `(N_PAIRS - 1 - i)` index everywhere it
 *   appears.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   Every entry sits in the bright half of the 256-colour space (≥ 24
 *   in the 6×6×6 cube, ≥ 240 in the gray ramp).  Cube indices 16-23
 *   and gray 232-239 render as black against the default background
 *   and would make the tail invisible.
 *
 * Reference [9] Raymond's NCURSES HOWTO §6 — init_pair() semantics
 *   that turn these palette arrays into live colour pairs.
 */
typedef struct {
    const char *name;            /* short ASCII label shown in HUD     */
    int         body[N_PAIRS];   /* xterm-256 fg indices, tail → head  *
                                  * pair (i+1) = body[i] at render time*/
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* 0 Amber  */ { "Amber",  {130, 136, 142, 148, 154, 160, 196} },
    /* 1 Matrix */ { "Matrix", { 24,  28,  34,  40,  46,  82, 118} },
    /* 2 Fire   */ { "Fire",   { 52,  88, 124, 160, 196, 208, 226} },
    /* 3 Ocean  */ { "Ocean",  { 24,  25,  27,  33,  39,  45,  51} },
    /* 4 Nova   */ { "Nova",   { 54,  93, 129, 165, 201, 213, 225} },
    /* 5 Toxic  */ { "Toxic",  { 58,  64,  70,  76,  82, 118, 154} },
    /* 6 Lava   */ { "Lava",   { 52,  58,  94, 130, 166, 202, 208} },
    /* 7 Ghost  */ { "Ghost",  {240, 244, 246, 250, 252, 254, 231} },
    /* 8 Aurora */ { "Aurora", { 24,  29,  35,  71, 107, 143, 221} },
    /* 9 Neon   */ { "Neon",   { 57,  93, 129, 165, 201, 199, 197} },
};

/*
 * theme_apply — (re-)define ncurses pairs for theme idx.
 *
 * ncurses lets pairs be redefined at runtime; the next frame picks up
 * new colors automatically because COLOR_PAIR(n) resolves at draw time,
 * not pair-define time.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *th = &THEMES[idx];

    if (COLORS >= 256) {
        for (int i = 0; i < N_PAIRS; i++)
            init_pair(i + 1, th->body[i], COLOR_BLACK);
    } else {
        int basic[N_PAIRS] = {
            COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
            COLOR_YELLOW, COLOR_GREEN,  COLOR_GREEN,  COLOR_WHITE
        };
        for (int i = 0; i < N_PAIRS; i++)
            init_pair(i + 1, basic[i], COLOR_BLACK);
    }

    /* HUD pairs are theme-independent — see CLAUDE.md HUD spec.
     * Default background (-1) so they overlay any animation cleanly. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * All entity positions live in square pixel space (1 unit = 1 px).
 * Only at draw time do these helpers convert to cell coordinates,
 * undoing the 8:16 cell aspect ratio.
 *   cell = floor(px / CELL_DIM + 0.5)   — nearest-integer rounding
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
/* §5  entity — Centipede                                                 */
/* ===================================================================== */

/*
 * Vec2 — 2-D position / direction vector in pixel space.
 *
 * Intent
 *   Every spatial quantity in this file (joints, legs, trail samples,
 *   antennae) lives in SQUARE PIXEL space — 1 unit = 1 pixel.  The
 *   8:16 cell aspect ratio is applied ONCE per draw in §4 coords,
 *   never inside the simulation.  Keeping pixel space square means
 *   distance, turn rate and leg phase math all read like classical
 *   vector geometry without aspect-correction littering them.
 *
 * Convention
 *   x increases eastward; y increases downward (terminal convention,
 *   inverted from cartesian).  All angles are measured from +x axis
 *   in the math sense (counterclockwise positive), so heading = 0
 *   points east and heading = π/2 points SOUTH (because y is flipped).
 */
typedef struct {
    float x;   /* eastward pixel coordinate     */
    float y;   /* southward pixel coordinate    */
} Vec2;

static inline float vec2_len(Vec2 v)
{
    return sqrtf(v.x * v.x + v.y * v.y);
}
static inline Vec2 vec2_norm(Vec2 v)
{
    float l = vec2_len(v);
    if (l < 1e-6f) return (Vec2){0.0f, 0.0f};
    return (Vec2){v.x / l, v.y / l};
}

/*
 * Centipede — the full simulation state for one segmented creature.
 *
 * Intent
 *   Three subsystems share one body, each with its own time scale:
 *     - HEAD       moves continuously, recording every position.
 *     - BODY       follows the head via FORWARD KINEMATICS (FK):
 *                  sample the trail at evenly-spaced arc lengths so
 *                  each segment "remembers" where the head was N
 *                  cells ago.  Ref [1] Aristidou & Lasenby contrast
 *                  this with inverse kinematics (FABRIK).
 *     - LEGS       are kinematically attached to body segments and
 *                  phase-offset along the body so a travelling wave
 *                  of footsteps moves tail-to-head (the metachronal
 *                  gait of real myriapods).  Refs [5][6].
 *
 *   The trail-following pattern is FK because we KNOW where the head
 *   has been; we just look up the right past position for each
 *   segment.  Inverse kinematics — the alternative — would solve for
 *   joint angles that put the END EFFECTOR at a target, which is
 *   what FABRIK ([1]) does.  FK is roughly 10× simpler in code and
 *   100× faster — appropriate here because we don't need physical
 *   joint constraints.
 *
 * Why a single struct
 *   The three subsystems share state (trail, joints) extensively;
 *   threading separate Body/Legs/Head structs through every helper
 *   would obscure the FK math.  One Centipede instance, all helpers
 *   take `Centipede *`.
 *
 * Why the trail is a CIRCULAR buffer (not a regular array)
 *   Trail samples are pushed at the head and consumed at every
 *   distance along the body.  Circular indexing means no O(N) shift
 *   per frame; head_index walks mod TRAIL_CAP.  See [3] Reynolds for
 *   the path-following pattern this buffer implements.
 *
 * References [1][2] for the FK / IK contrast, [3][4] Reynolds for
 *   path-following + steering, [5][6] for the biological gait that
 *   inspires the leg phasing.
 */
typedef struct {
    /* ── Trail buffer (FK source-of-truth) ───────────────────────── *
     * Circular head-position log.  Each push records the head's     *
     * current pixel position; body segments arc-length-sample this  *
     * buffer to position themselves in compute_joints().            */
    Vec2 trail[TRAIL_CAP];
    int  trail_head;           /* index of newest entry, mod TRAIL_CAP */
    int  trail_count;          /* valid entries, saturates at TRAIL_CAP*/

    /* ── Body joints (FK output) ────────────────────────────────── *
     * joint[0] = head pixel position; joint[1..N] = body segments,  *
     * placed each frame by arc-length sampling along the trail.     *
     * Read by render_centipede + leg solver, not directly mutated   *
     * by the user.                                                  */
    Vec2 joint[BODY_SEGS + 1];

    /* ── Legs (kinematic derivation of body + wave_time) ────────── *
     * [i][0]=hip, [i][1]=knee, [i][2]=foot for each pair i.  Hips   *
     * are attached to body joints; knees and feet derive from a    *
     * 3-link FK chain whose joint angles are functions of phase.   */
    Vec2 leg_left [N_LEGS][3];
    Vec2 leg_right[N_LEGS][3];

    /* ── Motion state (integrated each tick) ────────────────────── *
     * The "verbs" of the simulation.  Heading drives the head's    *
     * direction; wave_time is the master clock that animates legs  *
     * and antennae; turn_phase is a SEPARATE accumulator so the    *
     * user can adjust turn_freq mid-flight without phase jumps.    */
    float heading;             /* travel direction (rad), 0=east       */
    float wave_time;           /* master clock (s)                     */
    float turn_phase;          /* ∫ turn_freq · dt                     */

    /* ── User-tunable parameters (HUD-driven) ───────────────────── *
     * Adjusted via keypress; reset values come from §1 config.     */
    float move_speed;          /* px/s                                 */
    float turn_amp;            /* peak turn rate (rad/s)               */
    float turn_freq;           /* undulation frequency (rad/s)         */

    /* ── UI state (control flags + render-only choice) ──────────── *
     * theme_idx is PURE RENDER (paint colour pair).                *
     * paused is CONTROL (gates scene_tick).                        */
    int  theme_idx;            /* 0..N_THEMES-1                        */
    bool paused;
} Centipede;

/* ── §5a  trail helpers ─────────────────────────────────────────────── */

/* Append the head's new position; advance the write cursor mod TRAIL_CAP. */
static void trail_push(Centipede *c, Vec2 pos)
{
    c->trail_head = (c->trail_head + 1) % TRAIL_CAP;
    c->trail[c->trail_head] = pos;
    if (c->trail_count < TRAIL_CAP) c->trail_count++;
}

/* k=0 → newest (current head); k=1 → one frame older; etc.
 * The +TRAIL_CAP avoids C's negative-modulo trap. */
static inline Vec2 trail_at(const Centipede *c, int k)
{
    return c->trail[(c->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/*
 * trail_sample — interpolated position at arc-length `dist` from head.
 *
 * Walk the trail accumulating Euclidean distance until the running total
 * crosses `dist`, then linearly interpolate inside the bracketing segment.
 * The body literally retraces the path the head carved — joint spacing
 * stays exactly SEG_LEN_PX regardless of the head's instantaneous speed.
 */
static Vec2 trail_sample(const Centipede *c, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(c, 0);   /* current head */

    for (int k = 1; k < c->trail_count; k++) {
        Vec2  b   = trail_at(c, k);
        float dx  = b.x - a.x;
        float dy  = b.y - a.y;
        float seg = sqrtf(dx * dx + dy * dy);

        if (accum + seg >= dist) {
            float t = (dist - accum) / (seg > 1e-4f ? seg : 1e-4f);
            return (Vec2){ a.x + dx * t, a.y + dy * t };
        }
        accum += seg;
        a      = b;
    }
    return trail_at(c, c->trail_count - 1);
}

/* ── §5b  edge_bias + move_head ─────────────────────────────────────── */

/*
 * Quadratic repulsion field — gradient of a soft potential well.
 *
 *   For each wall within REPEL_MARGIN of the head, accumulate a push
 *   along its inward normal with magnitude  u² where u = 1 - d/MARGIN.
 *   The quadratic falloff gives F=0 at the margin and F=1 at the wall,
 *   so the centipede FEELS the boundary gently before COMMITTING to
 *   the turn. Mathematically: -∇Φ for Φ(d) = (MARGIN - d)³ / 3.
 *
 *   Output vector points toward open space, magnitude ∈ [0, ≈√2].
 *   See [3] Reynolds §"Containment".
 */
static Vec2 quadratic_repulsion_from_walls(const Centipede *c,
                                           int cols, int rows)
{
    float wpx = (float)cols * (float)CELL_W;
    float hpx = (float)rows * (float)CELL_H;
    float x = c->joint[0].x, y = c->joint[0].y;

    float d_left   = x;
    float d_right  = wpx - x;
    float d_top    = y;
    float d_bottom = hpx - y;

    Vec2 push = { 0.0f, 0.0f };
    if (d_left   < REPEL_MARGIN) { float u = 1.0f - d_left   / REPEL_MARGIN; push.x += u * u; }
    if (d_right  < REPEL_MARGIN) { float u = 1.0f - d_right  / REPEL_MARGIN; push.x -= u * u; }
    if (d_top    < REPEL_MARGIN) { float u = 1.0f - d_top    / REPEL_MARGIN; push.y += u * u; }
    if (d_bottom < REPEL_MARGIN) { float u = 1.0f - d_bottom / REPEL_MARGIN; push.y -= u * u; }
    return push;
}

/*
 * shortest_signed_angle — wrap a heading difference to [-π, π].
 *
 *   Heading is on a circle; raw subtraction can give a delta of e.g.
 *   +5π/3 when the true short rotation is -π/3. Wrapping ensures we
 *   always rotate the SHORT way round the circle — important when
 *   the desired-vs-current pair straddles the π / -π discontinuity.
 */
static float shortest_signed_angle(float diff)
{
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    return diff;
}

/*
 * edge_bias — Reynolds soft fence, expressed as a heading correction.
 *
 *   1. quadratic_repulsion_from_walls : gradient of soft potential.
 *   2. if push is negligible          : well clear; no correction.
 *   3. atan2f(push)                   : desired heading toward open space.
 *   4. shortest_signed_angle(Δθ)      : short-way rotation needed.
 *   5. clamp ‖push‖ to 1 and scale by REPEL_GAIN.
 *
 * Returns rad/s contribution to dθ/dt. Ref [3] Reynolds 1999.
 */
static float edge_bias(const Centipede *c, int cols, int rows)
{
    Vec2 push = quadratic_repulsion_from_walls(c, cols, rows);

    float mag2 = push.x * push.x + push.y * push.y;
    if (mag2 < 1e-8f) return 0.0f;            /* well clear of every wall */

    float desired = atan2f(push.y, push.x);
    float diff    = shortest_signed_angle(desired - c->heading);

    float strength = sqrtf(mag2);
    if (strength > 1.0f) strength = 1.0f;
    return diff * REPEL_GAIN * strength;
}

/*
 * advance_locomotion_clocks — tick the two phase variables a CPG needs.
 *
 *   wave_time   : master clock; reads from legs, antennae.
 *   turn_phase  : ∫ turn_freq · dt — kept SEPARATE from wave_time so
 *                 the user can adjust turn_freq mid-run without
 *                 producing a phase jump (which would whip the body
 *                 into a sudden curve).
 *
 *   This is the neurobiological "central pattern generator" pattern:
 *   gait timing is driven by an internal oscillator, not by sensory
 *   feedback. Ref [5] Manton 1952, [6] Full & Tu 1991.
 */
static void advance_locomotion_clocks(Centipede *c, float dt)
{
    c->wave_time  += dt;
    c->turn_phase += c->turn_freq * dt;
}

/* CPG output: pure sinusoid in turn_phase, scaled by user amplitude.
 * Units: rad/s contribution to dθ/dt. */
static float cpg_turn_rate(const Centipede *c)
{
    return c->turn_amp * sinf(c->turn_phase);
}

/*
 * integrate_heading_then_position — semi-implicit Euler step.
 *
 *   θ ← θ + ω·dt           (turn first)
 *   x ← x + v·cos(θ)·dt     (then move along the NEW heading)
 *   y ← y + v·sin(θ)·dt
 *
 *   Mathematically equivalent to explicit Euler at first order, but
 *   reads "turn then move", matching the physical intuition.
 */
static void integrate_heading_then_position(Centipede *c,
                                            float omega, float dt)
{
    c->heading    += omega * dt;
    c->joint[0].x += c->move_speed * cosf(c->heading) * dt;
    c->joint[0].y += c->move_speed * sinf(c->heading) * dt;
}

/* Defensive bounds clip — edge_bias keeps the head inside under
 * normal conditions, but SIGWINCH can shrink the screen out from
 * under it between integration and the next bias evaluation. */
static void clamp_head_to_pixel_bounds(Centipede *c, int cols, int rows)
{
    float wpx = (float)cols * (float)CELL_W;
    float hpx = (float)rows * (float)CELL_H;
    if (c->joint[0].x < 0.0f) c->joint[0].x = 0.0f;
    if (c->joint[0].x >= wpx) c->joint[0].x = wpx - 1.0f;
    if (c->joint[0].y < 0.0f) c->joint[0].y = 0.0f;
    if (c->joint[0].y >= hpx) c->joint[0].y = hpx - 1.0f;
}

/*
 * move_head — advance the head one step under CPG + edge avoidance.
 *
 *   1. advance_locomotion_clocks         (wave_time, turn_phase)
 *   2. ω = cpg_turn_rate + edge_bias     (steering composition)
 *   3. integrate_heading_then_position   (semi-implicit Euler)
 *   4. clamp_head_to_pixel_bounds        (defensive)
 *   5. trail_push                        (FK source-of-truth update)
 *
 * See ALGORITHM §2-§6 and Centipede struct doc.
 */
static void move_head(Centipede *c, float dt, int cols, int rows)
{
    advance_locomotion_clocks(c, dt);

    float omega = cpg_turn_rate(c) + edge_bias(c, cols, rows);
    integrate_heading_then_position(c, omega, dt);

    clamp_head_to_pixel_bounds(c, cols, rows);
    trail_push(c, c->joint[0]);
}

/* ── §5c  compute_joints ────────────────────────────────────────────── */

/* Place every body joint at the appropriate arc-length along the trail.
 * joint[0] (head) is already set by move_head(). */
static void compute_joints(Centipede *c)
{
    for (int i = 1; i <= BODY_SEGS; i++)
        c->joint[i] = trail_sample(c, (float)i * SEG_LEN_PX);
}

/* ── §5d  compute_legs ──────────────────────────────────────────────── */

/* leg_fk — forward-kinematic the hip → knee → foot chain given the two
 * joint angles. Pure helper; mirrors the FK chain in KEY FORMULAS. */
static void leg_fk(Vec2 hip, float upper_ang, float lower_ang,
                   Vec2 out[3])
{
    Vec2 knee = { hip.x + UPPER_LEN * cosf(upper_ang),
                  hip.y + UPPER_LEN * sinf(upper_ang) };
    Vec2 foot = { knee.x + LOWER_LEN * cosf(lower_ang),
                  knee.y + LOWER_LEN * sinf(lower_ang) };
    out[0] = hip;
    out[1] = knee;
    out[2] = foot;
}

/* body_dir_at — local body heading at joint index `j`, smoothed using
 * neighbours j-1 and j+1 so it stays well-defined even when the head
 * is moving slowly and consecutive joints are nearly coincident. */
static float body_dir_at(const Centipede *c, int j)
{
    Vec2 fwd = vec2_norm((Vec2){
        c->joint[j - 1].x - c->joint[j + 1].x,
        c->joint[j - 1].y - c->joint[j + 1].y
    });
    return atan2f(fwd.y, fwd.x);
}

/*
 * attachment_joint_for_pair — pick the body joint that leg-pair i hangs
 * from. N_LEGS pairs are spread evenly across joints 1..BODY_SEGS-1;
 * joint 0 (head) and joint BODY_SEGS (tail-tip) are skipped so the
 * silhouette retains a "feeler" head and tapering tail.
 */
static int attachment_joint_for_pair(int i)
{
    return 1 + i * (BODY_SEGS - 2) / (N_LEGS - 1);
}

/*
 * hips_from_body_normal — place port + starboard hips by stepping
 * BODY_OFFSET pixels along the lateral normals of the body spine at
 * `spine`. Lateral normals are body_dir ± π/2 — the rotation by 90°
 * that converts a tangent vector into a side-pointing vector.
 */
static void hips_from_body_normal(Vec2 spine, float body_dir, float offset,
                                  Vec2 *hip_L, Vec2 *hip_R)
{
    const float half_pi = (float)M_PI * 0.5f;
    float n_L = body_dir + half_pi;
    float n_R = body_dir - half_pi;
    hip_L->x = spine.x + offset * cosf(n_L);
    hip_L->y = spine.y + offset * sinf(n_L);
    hip_R->x = spine.x + offset * cosf(n_R);
    hip_R->y = spine.y + offset * sinf(n_R);
}

/*
 * metachronal_gait_phases — phase pair for leg pair i.
 *
 *   φ_L = ω·t + i · π/N    →  travelling wave FRONT TO REAR:
 *                              successive pairs lag by π/N rad,
 *                              producing the characteristic
 *                              "ripple" of myriapod feet. [5][6]
 *   φ_R = φ_L + π          →  contralateral antiphase (when L pushes,
 *                              R lifts).
 */
static void metachronal_gait_phases(int i, float wave_time,
                                    float *phi_L, float *phi_R)
{
    const float pi_f = (float)M_PI;
    *phi_L = wave_time * GAIT_FREQ + (float)i * (pi_f / (float)N_LEGS);
    *phi_R = *phi_L + pi_f;
}

/*
 * swing_angles_from_phase — modulate the 2-DOF leg posture sinusoidally.
 *
 *   upper(φ) = body_dir + side·LEG_SPLAY + A·sin(φ)
 *   lower(φ) = upper(φ) + LEG_BEND       + a·sin(φ + π/4)
 *
 *   `side` is +1 for port, -1 for starboard (mirrors splay).
 *   The π/4 phase shift between upper and lower swings yields the
 *   classic "knee leads, foot trails" walking-leg arc.
 */
static void swing_angles_from_phase(float body_dir, float side, float phi,
                                    float *upper, float *lower)
{
    const float quarter_pi = (float)M_PI * 0.25f;
    *upper = body_dir + side * LEG_SPLAY + SWING_AMP   * sinf(phi);
    *lower = *upper  + LEG_BEND          + LOWER_SWING * sinf(phi + quarter_pi);
}

/*
 * compute_legs — stateless FK over all leg pairs.
 *
 *   For each pair i:
 *     1. attachment_joint_for_pair   (which spine joint hosts this pair)
 *     2. body_dir_at                 (smoothed spine tangent there)
 *     3. hips_from_body_normal       (port + starboard hips ± normal)
 *     4. metachronal_gait_phases     (travelling-wave φ_L, φ_R)
 *     5. swing_angles_from_phase ×2  (port: +splay, starboard: -splay)
 *     6. leg_fk ×2                   (hip → knee → foot chain)
 *
 *   Pure function of (joint[], wave_time): same inputs ⇒ same pose,
 *   no hidden integrator state. See ALGORITHM §8 and Centipede doc.
 */
static void compute_legs(Centipede *c)
{
    for (int i = 0; i < N_LEGS; i++) {
        int   j        = attachment_joint_for_pair(i);
        float body_dir = body_dir_at(c, j);

        Vec2 hip_L, hip_R;
        hips_from_body_normal(c->joint[j], body_dir, BODY_OFFSET,
                              &hip_L, &hip_R);

        float phi_L, phi_R;
        metachronal_gait_phases(i, c->wave_time, &phi_L, &phi_R);

        float upper_L, lower_L, upper_R, lower_R;
        swing_angles_from_phase(body_dir, +1.0f, phi_L, &upper_L, &lower_L);
        swing_angles_from_phase(body_dir, -1.0f, phi_R, &upper_R, &lower_R);

        leg_fk(hip_L, upper_L, lower_L, c->leg_left [i]);
        leg_fk(hip_R, upper_R, lower_R, c->leg_right[i]);
    }
}

/* ── §5e  rendering helpers ─────────────────────────────────────────── */

/*
 * body_seg_pair — ncurses pair for body segment i.
 *   i = 0           → pair N_PAIRS (head color, brightest)
 *   i = BODY_SEGS-1 → pair 1       (tail color, dimmest)
 * Integer linear interpolation; clamped against tail-tip overflow.
 */
static int body_seg_pair(int i)
{
    int p = N_PAIRS - (i * (N_PAIRS - 1)) / (BODY_SEGS - 1);
    if (p < 1)       p = 1;
    if (p > N_PAIRS) p = N_PAIRS;
    return p;
}

/*
 * body_seg_attr — depth-cued attribute for body segment i.
 *   head quarter  : A_BOLD    (brightest, draws the eye)
 *   middle half   : A_NORMAL  (neutral)
 *   tail quarter  : A_DIM     (recedes visually)
 */
static attr_t body_seg_attr(int i)
{
    if (i <     BODY_SEGS / 4) return A_BOLD;
    if (i > 3 * BODY_SEGS / 4) return A_DIM;
    return A_NORMAL;
}

/*
 * seg_glyph — best ASCII direction glyph for vector (dx, dy).
 *
 * Glyphs partition the angular circle, folded to [0°, 180°) since each
 * glyph is symmetric under 180° rotation:
 *   '-' near-horizontal     ( 0°,  22.5° )  ∪  (157.5°, 180°)
 *   '\' down-right diagonal ( 22.5°,  67.5° )
 *   '|' near-vertical       ( 67.5°, 112.5° )
 *   '/' down-left diagonal  (112.5°, 157.5° )
 *
 * dy is negated before atan2f so the angle matches visual direction
 * (terminal y grows downward; math y grows upward).
 */
static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg < 22.5f || deg >= 157.5f) return (chtype)(unsigned char)'-';
    if (deg < 67.5f)                   return (chtype)(unsigned char)'\\';
    if (deg < 112.5f)                  return (chtype)(unsigned char)'|';
    return                             (chtype)(unsigned char)'/';
}

/* head_glyph — directional arrow ('>' '<' '^' 'v') for heading (rad). */
static chtype head_glyph(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (deg <  45.0f || deg >= 315.0f) return (chtype)(unsigned char)'>';
    if (deg < 135.0f)                  return (chtype)(unsigned char)'v';
    if (deg < 225.0f)                  return (chtype)(unsigned char)'<';
    return                             (chtype)(unsigned char)'^';
}

/*
 * sample_count_for_dense_raster — number of parametric samples that
 * guarantees AT LEAST ONE sample per cell along a line of `length` px.
 *
 *   Pixel cells are CELL_W=8 px wide; DRAW_STEP_PX=5 px keeps adjacent
 *   samples strictly < CELL_W apart so no traversed cell is skipped.
 *   The +1 accounts for the inclusive-endpoint final sample.
 *
 *   Alternative: Bresenham [7] yields one cell per step natively; we
 *   stay with parametric oversampling because float arithmetic is
 *   already paid for elsewhere this frame.
 */
static int sample_count_for_dense_raster(float length_px)
{
    return (int)ceilf(length_px / DRAW_STEP_PX) + 1;
}

/*
 * stamp_glyph_at_pixel — place one glyph at the cell containing pixel
 * (px, py), with cell-level dedup against the caller-owned cursor.
 *
 *   1. Convert pixel → cell coords.
 *   2. If same cell as last sample, skip (dedup).
 *   3. If off-screen, skip clip.
 *   4. Stamp.
 *
 *   The caller passes its own (prev_cx, prev_cy) so the cursor persists
 *   across multiple draw_line_dense calls — a centipede leg drawn as
 *   two segments (hip→knee, knee→foot) won't double-stamp at the knee.
 */
static void stamp_glyph_at_pixel(WINDOW *w, float px, float py,
                                 chtype glyph,
                                 int pair, attr_t attr,
                                 int cols, int rows,
                                 int *prev_cx, int *prev_cy)
{
    int cx = px_to_cell_x(px);
    int cy = px_to_cell_y(py);

    if (cx == *prev_cx && cy == *prev_cy) return;     /* same-cell dedup */
    *prev_cx = cx;
    *prev_cy = cy;

    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;

    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, glyph);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * draw_line_dense — direction-glyph line rasteriser with cell dedup.
 *
 *   1. Compute Δ = b - a and length; bail on zero-length lines.
 *   2. seg_glyph(Δ)               : ONE glyph for the whole line based
 *                                   on its angle (-, /, |, \).
 *   3. sample_count_for_dense_raster(length) : samples ≥ cells crossed.
 *   4. For t = 0..n stamp_glyph_at_pixel along the parametric line.
 *      Caller-owned (prev_cx, prev_cy) lives across calls — sentinel
 *      (-9999) before the first call so the opening cell stamps.
 *
 * Used by render_centipede / render_antennae for body and leg lines.
 * See [7] Bresenham 1965 for the bit-twiddling alternative.
 */
static void draw_line_dense(WINDOW *w,
                            Vec2 a, Vec2 b,
                            int pair, attr_t attr,
                            int cols, int rows,
                            int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = sample_count_for_dense_raster(len);

    for (int t = 0; t <= nsteps; t++) {
        float u = (float)t / (float)nsteps;
        stamp_glyph_at_pixel(w, a.x + dx * u, a.y + dy * u,
                             glyph, pair, attr,
                             cols, rows, prev_cx, prev_cy);
    }
}

/* ── §5f  render_centipede ──────────────────────────────────────────── */

/* node_marker — joint glyph by position along body. Marker shape encodes
 * distance from head so the eye can see at a glance which end is which. */
static chtype node_marker(int j)
{
    if (j == BODY_SEGS)        return (chtype)(unsigned char)'*'; /* tail tip   */
    if (j > 3 * BODY_SEGS / 4) return (chtype)(unsigned char)'.'; /* near tail  */
    if (j >     BODY_SEGS / 4) return (chtype)(unsigned char)'o'; /* mid body   */
    if (j > 0)                 return (chtype)(unsigned char)'O'; /* near head  */
    return                            (chtype)(unsigned char)'@'; /* head joint */
}

/* draw_leg — one leg (hip → knee → foot line) plus a bold foot-tip marker. */
static void draw_leg(WINDOW *w, const Vec2 leg[3],
                     int line_pair, chtype foot_glyph,
                     int cols, int rows)
{
    int cx = -9999, cy = -9999;
    draw_line_dense(w, leg[0], leg[1], line_pair, A_DIM, cols, rows, &cx, &cy);
    draw_line_dense(w, leg[1], leg[2], line_pair, A_DIM, cols, rows, &cx, &cy);

    int fx = px_to_cell_x(leg[2].x);
    int fy = px_to_cell_y(leg[2].y);
    if (fx < 0 || fx >= cols || fy < 0 || fy >= rows) return;
    wattron(w, COLOR_PAIR(5) | A_BOLD);
    mvwaddch(w, fy, fx, foot_glyph);
    wattroff(w, COLOR_PAIR(5) | A_BOLD);
}

/* draw_legs — every leg pair (left then right), drawn behind the body.
 * Different line pairs distinguish L/R where they overlap at the hip. */
static void draw_legs(WINDOW *w, const Centipede *c, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        draw_leg(w, c->leg_left [i], 3, (chtype)(unsigned char)'*', cols, rows);
        draw_leg(w, c->leg_right[i], 4, (chtype)(unsigned char)'x', cols, rows);
    }
}

/* draw_body_lines — body segments tail→head sharing one dedup cursor.
 * Tail-to-head order means head-area glyphs win when the body crosses
 * itself in a tight curve. */
static void draw_body_lines(WINDOW *w, const Centipede *c, int cols, int rows)
{
    int prev_cx = -9999, prev_cy = -9999;
    for (int i = BODY_SEGS - 1; i >= 0; i--) {
        draw_line_dense(w, c->joint[i + 1], c->joint[i],
                        body_seg_pair(i), body_seg_attr(i),
                        cols, rows, &prev_cx, &prev_cy);
    }
}

/* draw_body_nodes — bold node glyph at every joint, on top of segment
 * lines. Pair indexing clamped at BODY_SEGS-1 so the tail-tip joint
 * borrows the dimmest body color. */
static void draw_body_nodes(WINDOW *w, const Centipede *c, int cols, int rows)
{
    for (int j = BODY_SEGS; j >= 0; j--) {
        int cx = px_to_cell_x(c->joint[j].x);
        int cy = px_to_cell_y(c->joint[j].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        int    pair  = body_seg_pair(j < BODY_SEGS ? j : BODY_SEGS - 1);
        chtype glyph = node_marker(j);

        wattron(w, COLOR_PAIR(pair) | A_BOLD);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | A_BOLD);
    }
}

/*
 * render_centipede — orchestrator. Painter's order (later draws win on
 * overlaps):  legs (deepest)  →  body lines  →  joint nodes.
 * Antennae and head arrow are drawn by scene_draw on top of all of these.
 */
static void render_centipede(const Centipede *c, WINDOW *w,
                              int cols, int rows)
{
    draw_legs      (w, c, cols, rows);
    draw_body_lines(w, c, cols, rows);
    draw_body_nodes(w, c, cols, rows);
}

/* ── §5g  render_antennae ───────────────────────────────────────────── */

/*
 * Two slim segmented feelers emanate from the head, swaying with their
 * own faster oscillator. See KEY FORMULAS "Antenna seg" for the math.
 *
 * The +side*pi/2 phase offset makes left and right sway out-of-phase
 * with each other, which reads as alert / "feeling around" rather than
 * stiff symmetric flapping.
 */
static void render_antennae(const Centipede *c, WINDOW *w, int cols, int rows)
{
    const float pi_f = (float)M_PI;

    for (int i = 0; i < 2; i++) {
        float side = (i == 0) ? +1.0f : -1.0f;
        float base = c->heading + side * ANT_SPLAY;
        Vec2  prev = c->joint[0];
        int   pcx  = -9999, pcy = -9999;

        for (int s = 1; s <= ANT_SEGS; s++) {
            float wobble = ANT_AMP * sinf(c->wave_time * ANT_FREQ
                                          + (float)s * 0.4f
                                          + side * pi_f * 0.5f);
            float ang = base + wobble;
            Vec2  cur = { prev.x + ANT_SEG_LEN * cosf(ang),
                          prev.y + ANT_SEG_LEN * sinf(ang) };
            draw_line_dense(w, prev, cur, 7, A_DIM, cols, rows, &pcx, &pcy);
            prev = cur;
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — composition root for §6.
 *
 * Intent
 *   In this demo the simulation IS one centipede.  We keep a Scene
 *   wrapper anyway so the framework loop (scene_init / scene_tick /
 *   scene_draw) reads identically to every other file in the repo.
 *   If a future variant adds e.g. obstacles, predators, or pheromone
 *   trails, they slot in here as siblings of `centipede` without
 *   changing the loop.
 *
 * Simulation vs Rendering locality
 *   The Centipede struct itself already separates VERBS (heading,
 *   wave_time, move_speed) from NOUNS (joint, leg_left, leg_right)
 *   from RENDER-ONLY (theme_idx).  Within Scene there is no further
 *   split needed yet — one field, one concern.  When adding a new
 *   member, place it as follows:
 *
 *     ── Simulation state ──   things scene_tick READS+WRITES
 *                              (obstacles[], pheromone_grid, …)
 *     ── Render-only state ──  things scene_draw READS, never the
 *                              physics path (camera, screenshake, …)
 *
 * Things that DO NOT live here
 *   - fps / sim_fps counters → §8 App (frame-timing concern, not
 *     part of the world being simulated).
 *   - terminal extents (cols, rows) → §7 Screen.
 *   - signal flags (running, need_resize) → §8 App.
 *
 * One Scene per program; passed by pointer to every §6 entry point.
 */
typedef struct {
    /* ── Simulation state ─────────────────────────────────────── */
    Centipede centipede;       /* the world: trail, joints, legs, motion */

    /* (no render-only state yet — theme_idx lives inside Centipede
     *  because it's user-toggled via the same keymap as paused) */
} Scene;

/* Apply the user-tunable defaults from §1 config. These are the
 * baseline gait parameters the centipede reverts to on reset. */
static void apply_default_tunables(Centipede *c)
{
    c->move_speed = MOVE_SPEED;
    c->turn_amp   = TURN_AMP;
    c->turn_freq  = TURN_FREQ;
    c->theme_idx  = 0;
    c->paused     = false;
}

/*
 * seed_central_pattern_generator — initial phase + heading of the CPG.
 *
 *   wave_time   = 0           : master clock at start.
 *   turn_phase  = π/2         : sin() integrator at its PEAK, so the
 *                                first sample produces MAX turn rate
 *                                instead of near-zero for ~half a
 *                                period — the centipede curves on
 *                                frame 1, not after a long crawl.
 *   heading     = π/8         : slight south-east tilt so the visible
 *                                arc is across the screen, not along
 *                                an edge.
 *
 *   Ref [5][6] for the CPG model this seeds.
 */
static void seed_central_pattern_generator(Centipede *c)
{
    c->wave_time  = 0.0f;
    c->turn_phase = (float)M_PI * 0.5f;
    c->heading    = (float)M_PI / 8.0f;
}

/* Place the head at 38% of width × 50% of height. The off-centre choice
 * leaves room for the first half-period of curvature to fit on screen
 * without an immediate wall bounce. */
static void place_head_off_centre(Centipede *c, int cols, int rows)
{
    c->joint[0].x = (float)(cols * CELL_W) * 0.38f;
    c->joint[0].y = (float)(rows * CELL_H) * 0.50f;
}

/*
 * prefill_trail_behind_heading — seed the FK trail buffer.
 *
 *   Synthesises TRAIL_CAP virtual past positions, each 1 px BACKWARD
 *   along the starting heading from the previous one. Mathematically
 *   equivalent to saying "the centipede has been walking straight for
 *   the last TRAIL_CAP frames, and arrived HERE now".
 *
 *   Without this pre-fill, compute_joints() would have only one valid
 *   sample, and the body would appear as a single cell that grows
 *   tail-first over ~TRAIL_CAP frames. See Centipede struct doc
 *   "Trail buffer" and EDGE CASES "Trail starvation".
 */
static void prefill_trail_behind_heading(Centipede *c)
{
    float back_x = cosf(c->heading + (float)M_PI);
    float back_y = sinf(c->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        c->trail[k].x = c->joint[0].x + (float)k * back_x;
        c->trail[k].y = c->joint[0].y + (float)k * back_y;
    }
    c->trail_head  = 0;
    c->trail_count = TRAIL_CAP;
}

/*
 * scene_init — clean, immediately-animated start state.
 *
 *   1. memset to zero                  (default everything)
 *   2. apply_default_tunables          (speed, gait amplitudes, theme)
 *   3. seed_central_pattern_generator  (wave/turn phase + heading)
 *   4. place_head_off_centre           (room for first curve)
 *   5. prefill_trail_behind_heading    (avoid tail starvation)
 *   6. compute_joints + compute_legs   (derive renderable pose)
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Centipede *c = &sc->centipede;

    apply_default_tunables(c);
    seed_central_pattern_generator(c);
    place_head_off_centre(c, cols, rows);
    prefill_trail_behind_heading(c);

    compute_joints(c);
    compute_legs(c);
}

/*
 * scene_tick — one variable-timestep simulation step. dt is the wall-
 * clock delta scaled by the caller's time_scale.
 *
 * When paused, do nothing — leg / antennae / body all freeze cleanly
 * because their state at frame N+1 is identical to frame N.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Centipede *c = &sc->centipede;
    if (c->paused) return;
    move_head(c, dt, cols, rows);
    compute_joints(c);
    compute_legs(c);
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    const Centipede *c = &sc->centipede;
    render_centipede(c, w, cols, rows);
    render_antennae (c, w, cols, rows);

    /* Head arrow — drawn last so antennae cannot occlude the head. */
    int hx = px_to_cell_x(c->joint[0].x);
    int hy = px_to_cell_y(c->joint[0].y);
    if (hx >= 0 && hx < cols && hy >= 0 && hy < rows) {
        wattron(w, COLOR_PAIR(7) | A_BOLD);
        mvwaddch(w, hy, hx, head_glyph(c->heading));
        wattroff(w, COLOR_PAIR(7) | A_BOLD);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal-extent snapshot.
 *
 * Intent
 *   Caches the current terminal size in CELLS (not pixels) so every
 *   §6 entry point can take (cols, rows) as plain ints rather than
 *   re-querying ncurses each frame.  Refreshed only when SIGWINCH
 *   sets App::need_resize.
 *
 * Why this is a separate struct (not just two ints in App)
 *   Resize logic touches NOTHING in App except this struct; carving
 *   it out makes screen_resize() pure and testable without faking
 *   the rest of the app.  Also matches Raymond's [8] convention of
 *   isolating the terminal-state concern.
 *
 * Why cells, not pixels
 *   ncurses' coordinate system is cells.  Pixel space (CELL_W×CELL_H
 *   sub-pixels per cell) lives only inside §6 — converting at the
 *   draw boundary, per the project's "one conversion point" rule.
 */
typedef struct {
    int cols;                  /* terminal width  in CHARACTER CELLS   */
    int rows;                  /* terminal height in CHARACTER CELLS   */
} Screen;

/*
 * screen_init — configure terminal for animation.
 * The non-obvious call is typeahead(-1): without it ncurses peeks at
 * stdin during output writes, which can tear frames mid-update.
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
    color_init(0);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* SIGWINCH path: endwin()+refresh() forces ncurses to re-read LINES/COLS
 * from the kernel, then we sample the fresh dimensions. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — compose one full frame:
 *   erase → centipede → status (top right) → key hint (bottom).
 *
 * HUD pairs are spec-fixed (PAIR_HUD = bright yellow, PAIR_HINT = bright
 * cyan, both A_BOLD) so they read against any animation behind them.
 * NEVER use A_DIM on the hint strip — it disappears on bright frames.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Centipede *c = &sc->centipede;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %.1ffps  %.2fx  spd:%.0f  amp:%.2f  freq:%.2f  theme:%s  %s ",
             fps, time_scale,
             c->move_speed, c->turn_amp, c->turn_freq,
             THEMES[c->theme_idx].name,
             c->paused ? "PAUSED" : "crawling");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  ^v:spd  a/d:freq  w/s:amp  t:theme  [/]:time ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ncurses double-buffer flush: stage to newscr, then diff-emit. */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level container; everything outside the world.
 *
 * Intent
 *   Bundles the simulated world (Scene), the host terminal (Screen),
 *   and the loop-control flags into one record so main() reads as
 *   four-line phases: init / per-frame service / per-frame work /
 *   shutdown.  Declared file-scope (g_app) so signal handlers — which
 *   cannot take a user argument — can write `running` and
 *   `need_resize` without globals scattered through the file.
 *
 * Locality of concern
 *   ── Owned subsystems ── nouns the app composes
 *      scene       — the world being simulated  (§6)
 *      screen      — the terminal extent it draws to  (§7)
 *   ── Loop control ───── verbs the loop reads each frame
 *      time_scale  — wall-clock dt multiplier (slow-mo / fast-fwd)
 *      running     — clear → loop exits; set by SIGINT/SIGTERM
 *      need_resize — set by SIGWINCH; cleared after Screen refresh
 *
 * Why volatile sig_atomic_t (not bool, not int)
 *   `volatile`    : the compiler must not cache the flag across a
 *                   signal-handler write — every loop iteration must
 *                   re-read it from memory.
 *   `sig_atomic_t`: POSIX-guaranteed atomic with respect to async
 *                   signals; a plain `int` could be observed half-
 *                   written on architectures where stores are split.
 *   See Raymond [8] §"Signal handling".
 *
 * Things that DO NOT live here
 *   - Wall-clock timestamps / fps counters — local to main(), no
 *     other code path needs them.
 *   - Centipede tuning values — those are user-state in §6.
 */
typedef struct {
    /* ── Owned subsystems ─────────────────────────────────────── */
    Scene  scene;              /* the world (§6)                       */
    Screen screen;             /* terminal extent (§7)                 */

    /* ── Loop control ─────────────────────────────────────────── */
    float                 time_scale;   /* dt multiplier; 1.0 = realtime */
    volatile sig_atomic_t running;      /* main loop predicate           */
    volatile sig_atomic_t need_resize;  /* SIGWINCH pending              */
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* atexit safety net — endwin() called on every exit path. */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize — handle a pending SIGWINCH.
 * Clamp head into the new bounds; the next tick's edge bias smoothly
 * steers it back toward centre.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Centipede *c = &app->scene.centipede;
    float wpx    = (float)(app->screen.cols * CELL_W);
    float hpx    = (float)(app->screen.rows * CELL_H);
    if (c->joint[0].x >= wpx) c->joint[0].x = wpx - 1.0f;
    if (c->joint[0].y >= hpx) c->joint[0].y = hpx - 1.0f;
    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch one keypress; return false to quit.
 * Letter aliases (a/d, w/s) are provided because some terminals swallow
 * arrow-key escape sequences.
 */
static bool app_handle_key(App *app, int ch)
{
    Centipede *c = &app->scene.centipede;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': c->paused = !c->paused; break;

    case 't': case 'T':
        c->theme_idx = (c->theme_idx + 1) % N_THEMES;
        theme_apply(c->theme_idx);
        break;

    case KEY_UP:
        c->move_speed *= 1.25f;
        if (c->move_speed > MOVE_SPEED_MAX) c->move_speed = MOVE_SPEED_MAX;
        break;
    case KEY_DOWN:
        c->move_speed /= 1.25f;
        if (c->move_speed < MOVE_SPEED_MIN) c->move_speed = MOVE_SPEED_MIN;
        break;

    case KEY_LEFT:  case 'a': case 'A':
        c->turn_freq -= 0.15f;
        if (c->turn_freq < TURN_FREQ_MIN) c->turn_freq = TURN_FREQ_MIN;
        break;
    case KEY_RIGHT: case 'd': case 'D':
        c->turn_freq += 0.15f;
        if (c->turn_freq > TURN_FREQ_MAX) c->turn_freq = TURN_FREQ_MAX;
        break;

    case 'w': case 'W':
        c->turn_amp += 0.10f;
        if (c->turn_amp > TURN_AMP_MAX) c->turn_amp = TURN_AMP_MAX;
        break;
    case 's': case 'S':
        c->turn_amp -= 0.10f;
        if (c->turn_amp < TURN_AMP_MIN) c->turn_amp = TURN_AMP_MIN;
        break;

    case ']':
        app->time_scale *= TIME_SCALE_STEP;
        if (app->time_scale > TIME_SCALE_MAX) app->time_scale = TIME_SCALE_MAX;
        break;
    case '[':
        app->time_scale /= TIME_SCALE_STEP;
        if (app->time_scale < TIME_SCALE_MIN) app->time_scale = TIME_SCALE_MIN;
        break;

    default: break;
    }
    return true;
}

/*
 * main — variable-timestep render loop. Per-frame phases:
 *   ① INPUT      drain getch() — a press takes effect on this frame
 *   ② RESIZE     handle pending SIGWINCH before touching ncurses
 *   ③ MEASURE dt wall-clock ns since last frame; capped at 100 ms
 *                (see EDGE CASES "Suspend / lid-close")
 *   ④ TICK       one simulation step at exactly dt * time_scale
 *   ⑤ FPS        smoothed over a 500 ms window
 *   ⑥ RENDER     erase → draw → wnoutrefresh → doupdate
 *   ⑦ FRAME CAP  sleep so total frame ≈ 1/TARGET_FPS
 */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFFU));
    atexit(cleanup);

    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app        = &g_app;
    app->running    = 1;
    app->time_scale = TIME_SCALE_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    const int64_t target_ns = NS_PER_SEC / TARGET_FPS;

    int64_t last_time   = clock_ns();
    int64_t fps_accum   = 0;
    int     fps_frames  = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ① drain input first so a press takes effect on this frame */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
        if (!app->running) break;

        /* ② resize */
        if (app->need_resize) {
            app_do_resize(app);
            last_time = clock_ns();    /* discard the resize stall */
        }

        /* ③ measure dt */
        int64_t dt_ns = frame_start - last_time;
        last_time     = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        /* ④ tick */
        scene_tick(&app->scene, dt * app->time_scale,
                   app->screen.cols, app->screen.rows);

        /* ⑤ fps counter */
        fps_frames++;
        fps_accum += dt_ns;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)fps_frames
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_frames = 0;
            fps_accum  = 0;
        }

        /* ⑥ render */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->time_scale);
        screen_present();

        /* ⑦ frame cap — sleep so total frame ≈ target_ns. The math is
         *    just (target − elapsed); no spurious +dt terms. */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
