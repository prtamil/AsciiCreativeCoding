/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * moving_jump_spring_leg_robot.c — pogo-stick robot hopping across procedural terrain
 *
 * DEMO: A spring-loaded one-legged robot at the left edge of the
 *       screen. It LOADS its spring (body sinks, energy stored
 *       quadratically), RELEASES (energy → upward velocity),
 *       FLIES through the air on a parabolic arc under gravity,
 *       LANDS on the procedural terrain, and starts loading
 *       again. The camera stays still while the robot hops
 *       rightward; once the robot reaches 80 % of the screen the
 *       camera follows so fresh terrain scrolls in. The cycle
 *       repeats forever.
 *
 *           ┌──────────────────────────────────────────────────┐
 *           │                                                  │
 *           │            O                                     │
 *           │           ╱ ╲     ← parabolic arc (FLIGHT)       │
 *           │         ╱     ╲                                  │
 *           │       ╱         ╲                                │
 *           │      .           ↘                               │
 *           │  @                  *                            │
 *           │  |                                               │
 *           │  )    ← spring leg                               │
 *           │  |                                               │
 *           │  v        slope-adapted launch angle             │
 *           │ /‾\___    /‾‾‾\___/‾‾‾‾\__/‾‾‾                   │
 *           │####\_____/#####\_/######\##                      │
 *           └──────────────────────────────────────────────────┘
 *
 *           '@' COMPRESS — body sinks, spring loads
 *           'O' FLIGHT   — projectile motion under gravity
 *           '*' LAND     — impact, body locked, brief flash
 *
 *       Real-world analogues:
 *           ─ pogo stick (the obvious one)
 *           ─ a flea jumping (uses a pre-loaded resilin spring)
 *           ─ MIT Cheetah's leg (real-world springy-legged robot)
 *           ─ catapult / trebuchet (release stored energy)
 *
 * Study alongside:
 *   robots/diff_drive_robot.c       — wheeled contrast: rolling
 *                                      instead of jumping.
 *   physics/stroke_engine.c         — slider-crank kinematics
 *                                      (similar "stored energy
 *                                      released cyclically" theme).
 *   matrix_rain/fireworks_rain.c    — also has a state machine
 *                                      (IDLE → RISING → EXPLODED)
 *                                      and projectile physics.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus per-element semantic pairs
 *   §4  coords       — pixel↔cell aspect-ratio bridge
 *   §5  noise        — 1-D value noise (terrain primitive)
 *   §6  terrain      — height query, slope, surface glyph
 *   §7  trail        — ring buffer of recent body positions
 *   §8  robot/scene  — §8.1 Phase enum + Pose + Velocity + Spring +
 *                       PhaseFSM + LaunchAim sub-structs (Robot);
 *                       §8.1b Camera + World + SimControls sub-structs
 *                       (Scene root); spring physics + per-phase ticks:
 *                       compress_tick decomposed into
 *                       compress_progress_fraction +
 *                       sample_terrain_aim +
 *                       spring_pe_to_launch_speed +
 *                       emit_launch_impulse;
 *                       flight_tick decomposed into
 *                       integrate_projectile_step +
 *                       foot_projection_y + commit_landing;
 *                       scene_tick orchestrator + scene_init / _reset
 *   §9  render       — paint terrain, trail, spring, body, PE bar, HUD;
 *                       render_pe_bar decomposed into PeBarGeometry +
 *                       compute_pe_bar_geometry + pe_ratio_quadratic +
 *                       paint_pe_label + paint_filled_cell +
 *                       paint_empty_cell + paint_pe_bar_column;
 *                       render_hud decomposed into radians_to_degrees +
 *                       hud_paint_text(_padded) + format_hud_status +
 *                       draw_hud_status + draw_hud_hint +
 *                       draw_paused_banner
 *   §10 screen       — ncurses init / present
 *   §11 app          — FpsCounter + App; signals, resize, key dispatch,
 *                       variable-dt main loop (7 numbered phases)
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space / p        pause / resume physics
 *   r                reset to start
 *   f                cycle floor (FLAT ↔ PERLIN)
 *   n                new terrain seed (only matters in PERLIN mode)
 *   a                cycle horizontal speed (1.0× .. 3.0×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra robots/moving_jump_spring_leg_robot.c \
 *       -o moving_jump_spring_leg_robot -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Three-phase state machine driven by simple physics.
 *
 *                 (A) COMPRESS — spring is being loaded. The body
 *                     sinks toward the foot as `spring_compress`
 *                     ramps from 0 to SPRING_COMPRESS_MAX over
 *                     T_COMPRESS seconds. At full compression we
 *                     compute the launch velocity from Hooke's law
 *                     (energy conservation) and the slope-adapted
 *                     launch angle, then transition to FLIGHT.
 *
 *                 (B) FLIGHT — Newtonian projectile motion. Forward
 *                     Euler integrates `body_py += vy · dt` and
 *                     `vy += GRAVITY · dt`. Horizontal velocity vx
 *                     is constant (no air resistance). When the
 *                     foot's projected position reaches the floor
 *                     under the body's current x, transition to LAND.
 *
 *                 (C) LAND — body locked at landing pose for a brief
 *                     T_LAND seconds (impact flash visualised as '*').
 *                     Then transition back to COMPRESS, completing
 *                     the cycle.
 *
 *                 The camera is a slim helper: while the robot stays
 *                 in the left 80 % of the screen, the camera is
 *                 frozen at world origin. Once the robot crosses 80 %,
 *                 the camera exponentially chases a target that
 *                 keeps the robot pinned at the trigger column —
 *                 producing a smooth follow-cam without snapping.
 *
 * Data-structure: Scene = Robot + World + Camera + SimControls,
 *                 each a named sub-struct (see §8.1b for the
 *                 layered-ownership diagram).  Robot itself
 *                 decomposes into Pose (body + foot anchor
 *                 points) + Velocity (vx, vy — meaningful only
 *                 during FLIGHT) + Spring (compression scalar) +
 *                 PhaseFSM (phase + phase_t) + LaunchAim (cached
 *                 slope + launch angle) + Trail (ring buffer of
 *                 past positions).  The five Robot sub-structs
 *                 mirror the SLIP locomotion model: kinematics
 *                 (pose) → dynamics (vel + spring) → control
 *                 (fsm) → targeting (aim).  No heap allocation
 *                 post-init.
 *
 * Rendering     : Painter's order — last write wins:
 *                 (1) trail dots (oldest first so newest paints over)
 *                 (2) terrain (multi-row column sweep)
 *                 (3) spring leg coil (only during COMPRESS / LAND)
 *                 (4) body glyph (always on top)
 *                 (5) energy gauge (only during COMPRESS)
 *                 (6) HUD row 0 + hint row last
 *                 The whole frame is in cell coordinates; conversion
 *                 from pixel-space physics happens at the very end.
 *
 * Performance   : O(cols) per frame for terrain + trail. Single
 *                 floor_y_at() call per terrain column = O(cols)
 *                 trig calls plus the two-octave noise lookup. At
 *                 100 cols × 60 fps that's ~6000 noise samples/sec,
 *                 microseconds. ncurses redraw dominates.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Legged-robot locomotion (the SLIP model) ───────────────────
 *   [1] Raibert, M. H. (1986), "Legged Robots That Balance", MIT
 *       Press — the foundational book on spring-mass robot
 *       locomotion.  §2 introduces the body-foot state pair (our
 *       Pose struct); §3 introduces the spring-loaded inverted
 *       pendulum (SLIP) model (our Spring + PhaseFSM); §4 covers
 *       terrain-adaptive launch angles (our LaunchAim cached
 *       during COMPRESS).
 *   [2] Blickhan, R. (1989), "The spring-mass model for running
 *       and hopping", J. Biomech. 22(11-12), pp. 1217-1227 — the
 *       canonical biomechanics formulation of SLIP.  Compact
 *       reference for the mass-spring-damper analysis of stance
 *       phase.  This file skips the damper for visual simplicity
 *       (see "Why no spring damper" note below).
 *   [3] Geyer, H., Seyfarth, A. & Blickhan, R. (2006), "Compliant
 *       leg behaviour explains basic dynamics of walking and
 *       running", Proc. R. Soc. B 273(1603) — extends SLIP to
 *       running gaits.  Our 3-phase COMPRESS → FLIGHT → LAND
 *       cycle is the simplest running gait realisation.
 *
 *   ── Spring physics + ballistic flight ──────────────────────────
 *   [4] Hooke's law — F = −k·x; PE = ½·k·x²  (the basis of
 *       spring_energy in §8.2).  Standard physics result;
 *       Wikipedia is fine for the formulas.
 *       https://en.wikipedia.org/wiki/Hooke%27s_law
 *   [5] Symon, K. R. (1971), "Mechanics" (3rd ed.), Addison-Wesley
 *       Ch. 3 — classical projectile motion under uniform gravity.
 *       §8.6 flight_tick is plain forward Euler on this ODE.
 *
 *   ── Numerical integration ──────────────────────────────────────
 *   [6] Hairer, E., Lubich, C. & Wanner, G. (2006), "Geometric
 *       Numerical Integration" (2nd ed.), Springer — explicit-Euler
 *       reference.  flight_tick uses forward Euler:
 *           vy += g · dt
 *           y  += vy · dt
 *       Non-symplectic, but each FLIGHT phase is < 1 second and
 *       energy drift is sub-pixel — invisible at the cell scale.
 *
 *   ── Procedural terrain ─────────────────────────────────────────
 *   [7] Quilez, I., "Value noise" — the cosine-interpolated 1-D
 *       noise used in §5/§6 for the PERLIN floor mode.  Concise
 *       online derivation + reference implementation.
 *       https://iquilezles.org/articles/noise/
 *   [8] Perlin, K. (1985), "An image synthesizer", ACM SIGGRAPH
 *       Computer Graphics 19(3) — the original procedural-noise
 *       paper.  Value noise (Quilez [7]) is a simplification of
 *       Perlin's gradient noise; we use value noise because 1-D
 *       gradient noise is overkill for a single height profile.
 *
 *   ── Variable-timestep game loop ────────────────────────────────
 *   [9] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       canonical case for FIXED-step physics.  This file
 *       deliberately uses VARIABLE dt: the ODE is non-stiff (no
 *       coupled springs in the inner loop — spring loading is
 *       linear ramp, FLIGHT is ballistic).  Fiedler's DT_CAP rule
 *       (100 ms hard cap, DT_CAP_SEC here) is the only piece we
 *       adopt.
 *
 *   ── ncurses rendering substrate ────────────────────────────────
 *   [10] Raymond, E. S., "NCURSES Programming HOWTO" — §6 (colour
 *        pairs) for per-element semantic colouring (terrain /
 *        body / spring / trail), §11 (output options) for the
 *        wnoutrefresh + doupdate diff-only write pattern.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     robots/diff_drive_robot.c    — the WHEELED contrast: pose
 *       comes from wheel speeds instead of foot impulses.  Read
 *       this second to see how the kinematic pipeline changes
 *       when the contact model changes.
 *     animation/hexpod_tripod.c    — 6-leg insect locomotion
 *       (gait coordination on top of leg kinematics).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The robot is a spring on a stick. It LOADS the spring (storing
 * energy quadratically: PE = ½kx²), RELEASES (the spring snaps back,
 * converting potential energy to kinetic energy and launching the
 * body upward at an angle), FLIES through the air under gravity (a
 * parabolic arc — vx constant, vy decelerated then accelerated by
 * gravity), LANDS on the terrain (briefly stunned), then LOADS
 * again. Three discrete phases. The terrain is just a height
 * function that changes where the robot lands.
 *
 * THE JUMP CYCLE — three phases visualised
 * ────────────────────────────────────────
 *
 *     COMPRESS               FLIGHT                  LAND
 *     ──────────────         ─────────────────       ────────────
 *
 *      ┌───┐                       O                  ┌───┐
 *      │@  │                      ╱ ╲                 │ * │
 *      │|  │                    ╱     ╲               │   │
 *      │)  │                  ╱         ╲             │   │
 *      │|  │                ╱             ╲           │   │
 *      │v  │              ╱                 ↘         │   │
 *      └───┘            ●                     ●       └───┘
 *      foot                launch              land
 *      planted             instant             impact
 *
 *      • spring_compress     • vx constant         • body locked
 *        ramps 0 → MAX       • vy decelerates      • duration
 *      • PE = ½kx²             then accelerates      T_LAND
 *      • duration              under GRAVITY        • flash '*'
 *        T_COMPRESS          • trail recorded      • → COMPRESS
 *      • PE bar fills        • detect floor
 *      • → FLIGHT              contact → LAND
 *
 *
 * HOOKE'S LAW — first principles
 * ──────────────────────────────
 *
 *   A spring resists being compressed. The harder you push, the
 *   harder it pushes back:
 *
 *       F = -k · x         (force is opposite to displacement,
 *                           proportional to displacement)
 *
 *   The work you do compressing the spring is stored as potential
 *   energy. Integrating force from 0 to x gives the QUADRATIC
 *   potential:
 *
 *       PE = ½ · k · x²
 *
 *       ────────────────────────────────────────────────
 *       Compress 2× as far → store 4× as much energy.
 *       That's why the energy gauge fills SLOWLY at first
 *       and ACCELERATES near the end of compression.
 *       ────────────────────────────────────────────────
 *
 *   When the spring releases, all that PE becomes kinetic energy:
 *
 *       ½ · m · v² = ½ · k · x²
 *       ⟹  v = x · √(k / m)
 *
 *   With our defaults (m = 1, k = SPRING_K = 25, x_max = 48 px):
 *
 *       v_launch = 48 · √25 = 240 px/sec
 *
 *   That's the body's launch speed. We split it into vx and vy by
 *   the launch angle θ:
 *
 *       vx = v_launch · cos θ
 *       vy = -v_launch · sin θ        (negative = upward in y-down)
 *
 *
 * PROJECTILE MOTION — first principles
 * ────────────────────────────────────
 *
 *   Once airborne, only gravity acts on the robot. Two equations:
 *
 *       horizontal:    x(t) = x₀ + vx · t          (vx constant)
 *       vertical:      y(t) = y₀ + vy · t + ½ · g · t²
 *
 *   The trajectory is a PARABOLA opening downward. Useful numbers:
 *
 *       time to peak     :  t_peak = |vy| / g
 *       peak height      :  h_peak = vy² / (2g)
 *       time to land     :  t_land = 2 · t_peak              (flat ground)
 *       horizontal range :  R      = 2 · vx · |vy| / g       (flat ground)
 *
 *   Worked example (default config, flat ground):
 *
 *       v_launch = 240 px/sec, θ = 50°, g = 200 px/sec²
 *       vx       = 240 · cos50° ≈ 154 px/sec
 *       vy       = 240 · sin50° ≈ 184 px/sec
 *       t_peak   = 184 / 200 = 0.92 sec
 *       h_peak   = 184² / (2·200) ≈ 85 px ≈ 5 cells above launch
 *       t_land   = 1.84 sec
 *       R        = 2 · 154 · 184 / 200 ≈ 283 px ≈ 35 cols
 *
 *   So one default jump covers about 35 columns and reaches about
 *   5 cells above the launch height. On an 80-col terminal that's
 *   2-3 jumps before the camera has to pan. Time it with a
 *   stopwatch — the math is verifiable.
 *
 *
 * STATE MACHINE
 * ─────────────
 *
 *      ┌──────────┐  phase_t ≥ T_COMPRESS    ┌────────┐
 *      │ COMPRESS │ ───────────────────────► │ FLIGHT │
 *      │  (LOAD)  │   compute v_launch       │  (FLY) │
 *      │          │   set (vx, vy)            │        │
 *      │   '@'    │                          │   'O'  │
 *      └──────────┘                          └────────┘
 *           ▲                                    │
 *           │ phase_t ≥ T_LAND               foot reaches floor
 *           │                                set vx=vy=0; lock pose
 *           │                                    │
 *           │                                    ▼
 *      ┌──────────┐                         ┌────────┐
 *      │ COMPRESS │ ◄────────────────────── │  LAND  │
 *      └──────────┘                         │  '*'   │
 *                                           └────────┘
 *
 *
 * CAMERA — RIGHT-EDGE FOLLOW
 * ──────────────────────────
 *
 *   While the robot is in the LEFT 80 % of the screen, the camera
 *   doesn't move. Once the robot crosses CAM_TRIGGER × screen
 *   width, the camera target jumps to (body_px − trigger), and the
 *   actual cam_x exponentially chases that target at CAM_SPEED.
 *   This produces a smooth slide-cam (no snap) that pins the robot
 *   at the trigger column once it's "running with the camera".
 *
 *      ┌────────────────────────────────────┐
 *      │                                    │
 *      │  →  →  →  →  →  R                  │
 *      │                ↑                   │
 *      │           80 % trigger             │
 *      │                                    │
 *      └────────────────────────────────────┘
 *      cam_x = 0 (frozen)         past trigger:
 *                                 cam_x → body_px - trigger
 *                                 (exponential chase)
 *
 *   World coordinates: every robot/terrain position is in PIXEL
 *   space and ABSOLUTE (world coords), so old terrain at world_x =
 *   1000 still hashes to the same noise value when you scroll back.
 *   The camera is just a screen offset:  screen_x = world_x − cam_x.
 *
 *
 * SLOPE-ADAPTED LAUNCH ANGLE
 * ──────────────────────────
 *
 *   On flat ground we launch at LAUNCH_ANGLE (default 50°). On a
 *   slope, the robot tilts the angle to better suit the terrain:
 *
 *       slope < 0  (ascending)  → θ increases (steeper jump)
 *       slope > 0  (descending) → θ decreases (shallower jump)
 *       slope = 0  (flat)       → θ = LAUNCH_ANGLE
 *
 *       θ_eff = clamp(LAUNCH_ANGLE - slope · SLOPE_SCALE,
 *                     LAUNCH_MIN, LAUNCH_MAX)
 *
 *   Why? On an ascending slope, a flat-style jump might smack the
 *   wall in front. Steepening the jump clears the rise. On a
 *   descending slope, a flat jump overshoots; a shallower angle
 *   rides the descent.
 *
 *
 * KEY FORMULAS
 * ────────────
 *   Spring stored energy  : PE = ½ · SPRING_K · x²
 *   Spring launch speed   : v  = x · √(SPRING_K / m)     (m = 1)
 *   Launch decomposition  : vx =  v · cos θ
 *                           vy = -v · sin θ              (y-down: up = negative)
 *   Free fall vertical    : y(t) = y₀ + vy·t + ½·g·t²
 *   Free fall horizontal  : x(t) = x₀ + vx·t
 *   Slope (central diff)  : s = atan2(Δy, 2·Δx)          at foot position
 *   Effective angle       : θ_eff = clamp(LAUNCH_ANGLE − s · SLOPE_SCALE,
 *                                          LAUNCH_MIN, LAUNCH_MAX)
 *   Floor contact test    : foot_proj_y ≥ floor_y_at(body_x)
 *
 *
 *
 * Background you need
 * ───────────────────
 *   - Newton's laws: F = ma, vy += g · dt.
 *   - Hooke's law: F = -k · x; PE = ½ k x².
 *   - Trig: launch velocity at angle θ has components
 *     (v · cos θ, -v · sin θ) (negative because screen-y is down).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Real-world spring-mass-damper ODEs (we use COMPRESS as a
 *     time-ramp, not as a real Hooke's-law oscillator). The
 *     simplification is a valid teaching shortcut.
 *   - Air drag, restitution, or real impact dynamics. Landing is
 *     a brief frozen state, not a physical bounce.
 *   - Inverse dynamics. The robot is purely kinematic during
 *     COMPRESS / LAND; only FLIGHT involves Newton's second law.
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

/* ── §1.2 cell pixel dimensions ──────────────────────────────────── */
/*
 * All physics is in PIXEL SPACE — a uniform grid where 1 px is
 * 1/8 cell wide and 1/16 cell tall. Conversion to cells happens
 * only at draw time. Keeps the math isotropic regardless of
 * the terminal's 2:1 cell aspect ratio.
 */
#define CELL_W   8
#define CELL_H  16

/* ── §1.3 spring (Hooke's law) ────────────────────────────────────── */
/*
 * SPRING_REST           — uncompressed leg length in pixels.
 *                         At rest the body sits this far above the foot.
 * SPRING_COMPRESS_MAX   — maximum compression in pixels.
 *                         At full compression the body has sunk this much.
 * SPRING_K              — spring stiffness coefficient.
 *                         Determines launch speed: v = x · √(k/m), m=1.
 *
 *     v_launch = SPRING_COMPRESS_MAX · √SPRING_K
 *              = 48 · √25 = 48 · 5 = 240 px/sec
 *
 * BODY_HALF_H           — pixel half-height of body glyph for ground contact.
 */
#define SPRING_REST          80.0f
#define SPRING_COMPRESS_MAX  48.0f
#define SPRING_K             25.0f
#define BODY_HALF_H           8.0f

/* ── §1.4 launch geometry ────────────────────────────────────────── */
/*
 * LAUNCH_ANGLE — base launch angle from horizontal in radians.
 *   50° gives a near-balanced jump: vx ≈ 154, vy ≈ 184 at v=240.
 *   That balances arc height (≈5 cells) against horizontal range
 *   (≈35 cols), so 2–3 jumps fill an 80-col screen.
 *
 * SLOPE_SCALE — how much terrain slope deflects the launch angle.
 *   0.5 → a 20° slope shifts the launch by 10°.
 *   Higher = more responsive to slope; lower = ignore slope.
 *
 * LAUNCH_MIN, LAUNCH_MAX — clamp the slope-adapted angle to a sane
 *   range so degenerate slopes don't produce vertical or horizontal
 *   launches.
 */
#define LAUNCH_ANGLE  (50.0f * (float)M_PI / 180.0f)
#define SLOPE_SCALE    0.5f
#define LAUNCH_MIN    (25.0f * (float)M_PI / 180.0f)
#define LAUNCH_MAX    (82.0f * (float)M_PI / 180.0f)

/* ── §1.5 gravity ─────────────────────────────────────────────────── */
/*
 * Downward acceleration in pixels per second². 200 px/sec² gives
 * a peak height of ~85 px (≈5 cells) at default launch — visually
 * obvious without dwarfing the screen.
 */
#define GRAVITY  200.0f

/* ── §1.6 phase timing (seconds) ──────────────────────────────────── */
/*
 * T_COMPRESS — duration of the spring-loading phase. The energy
 *   gauge fills over this time. 0.45 s feels deliberate: long enough
 *   to read but not boring.
 *
 * T_LAND — duration of the impact flash. 0.14 s is short — just
 *   long enough to register the '*' visually.
 */
#define T_COMPRESS  0.45f
#define T_LAND      0.14f

/* ── §1.7 terrain ─────────────────────────────────────────────────── */
/*
 * Two-octave value noise — sum of one BIG slow wave and one SMALL
 * fast wave. The big wave gives rolling hills; the small wave adds
 * roughness on top. Three octaves works too but is overkill for
 * this teaching demo.
 *
 *   T_AMP    : amplitude (peak-to-peak / 2) in pixels.
 *              112 px = 7 cells of vertical swing.
 *
 *   T_FREQ_BIG : frequency of the big wave in cycles per pixel.
 *              0.0028 → wavelength ≈ 360 px ≈ 45 cells.
 *
 *   T_FREQ_SMALL : frequency of the small bumps.
 *              0.014 → wavelength ≈ 70 px ≈ 9 cells.
 *
 *   The two amplitudes (60 % big + 40 % small = 1.0) sum to 1.0 so
 *   the combined output is in [0, 1] just like a single noise call.
 */
#define T_AMP            112.0f
#define T_FREQ_BIG       0.0028f
#define T_FREQ_SMALL     0.014f
#define T_WEIGHT_BIG     0.6f
#define T_WEIGHT_SMALL   0.4f

#define NOISE_N          512    /* lattice size; power of 2 for fast & */

/* ── §1.8 camera ──────────────────────────────────────────────────── */
/*
 * CAM_TRIGGER — robot screen fraction at which camera starts to pan.
 *   0.80 → robot must walk across 80 % of screen before camera moves.
 *
 * CAM_SPEED — exponential catch-up rate when triggered, per second.
 *   6.0 → time constant 1/6 ≈ 167 ms. Smooth slide, no snap.
 *
 * CAM_START_COL — column where the robot spawns at reset.
 *   3 cells from left edge. Gives the robot a clear head start.
 */
#define CAM_TRIGGER     0.80f
#define CAM_SPEED       6.0f
#define CAM_START_COL   3

/* ── §1.9 trail ───────────────────────────────────────────────────── */
/* Ring buffer of recent body positions; older entries fade visually. */
enum { TRAIL_CAP = 1500 };

/* ── §1.10 horizontal speed cycle ('a' key) ───────────────────────── */
/*
 * Multiplier on vx at launch. ONLY scales horizontal motion — arc
 * height stays the same so the jump shape stays readable while the
 * robot covers more ground.
 */
enum { SPEED_LEVELS = 5 };
static const float SPEED_MULTS[SPEED_LEVELS] = { 1.0f, 1.5f, 2.0f, 2.5f, 3.0f };

/* ── §1.11 dt cap (spiral-of-death guard) ─────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.12 ncurses pair IDs ───────────────────────────────────────── */
enum {
    /* 1..6 — robot & spring */
    CP_BODY      = 1,        /* '@' grounded                 white       */
    CP_SPRING_LO,            /* spring low compression       yellow      */
    CP_SPRING_MD,            /* spring medium                orange      */
    CP_SPRING_HI,            /* spring high (loaded)         red         */
    CP_FLIGHT,               /* 'O' airborne                 cyan        */
    CP_LAND,                 /* '*' impact                   magenta     */

    /* 7..8 — trail */
    CP_TRAIL,                /* fresh trail '.'              blue        */
    CP_TRAIL_OLD,            /* aged trail ':'               dim blue    */

    /* 9..11 — terrain & PE */
    CP_SURF,                 /* terrain surface              green       */
    CP_ROCK,                 /* terrain sub-surface          dim green   */
    CP_PE,                   /* PE bar fill                  yellow      */

    /* 12..13 — HUD spec, theme-independent */
    PAIR_HUD,                /* row 0 status                 yellow BOLD */
    PAIR_HINT,               /* bottom row hint              cyan BOLD   */
};

/* ── §1.13 timing primitives ──────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.14 HUD layout ─────────────────────────────────────────────── */
#define HUD_BUF_LEN  192

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
 * Palette is fixed (no themes) because this is a teaching demo and
 * one consistent set of semantic colours is easier to memorise:
 *
 *   body              white     (always visible)
 *   spring low        yellow    (just loaded)
 *   spring medium     orange    (mid-load)
 *   spring high       red       (fully loaded — about to fire)
 *   flight body 'O'   cyan      (airborne)
 *   land flash '*'    magenta   (impact)
 *   trail fresh       blue      (recent arc)
 *   trail old         dim blue  (older arc, ':' for visual recede)
 *   terrain surface   green     ('_' '/' '\')
 *   terrain rock      dim green ('#' fill)
 *   PE bar            yellow    (turns red when nearly full)
 *
 * HUD pairs (PAIR_HUD, PAIR_HINT) bind separately on default
 * terminal background per CLAUDE.md HUD spec.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_BODY,        255, COLOR_BLACK);    /* near-white   */
        init_pair(CP_SPRING_LO,   226, COLOR_BLACK);    /* yellow       */
        init_pair(CP_SPRING_MD,   208, COLOR_BLACK);    /* orange       */
        init_pair(CP_SPRING_HI,   196, COLOR_BLACK);    /* red          */
        init_pair(CP_FLIGHT,       51, COLOR_BLACK);    /* cyan         */
        init_pair(CP_LAND,        201, COLOR_BLACK);    /* magenta      */
        init_pair(CP_TRAIL,        27, COLOR_BLACK);    /* blue         */
        init_pair(CP_TRAIL_OLD,    25, COLOR_BLACK);    /* dim blue     */
        init_pair(CP_SURF,         46, COLOR_BLACK);    /* bright green */
        init_pair(CP_ROCK,         28, COLOR_BLACK);    /* dim green    */
        init_pair(CP_PE,          226, COLOR_BLACK);    /* yellow       */
        init_pair(PAIR_HUD,       226, -1);             /* yellow on bg */
        init_pair(PAIR_HINT,       51, -1);             /* cyan on bg   */
    } else {
        init_pair(CP_BODY,      COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_SPRING_LO, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_SPRING_MD, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_SPRING_HI, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_FLIGHT,    COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_LAND,      COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_TRAIL,     COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_TRAIL_OLD, COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_SURF,      COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_ROCK,      COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_PE,        COLOR_YELLOW,  COLOR_BLACK);
        init_pair(PAIR_HUD,     COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,    COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio bridge                           */
/* ===================================================================== */

/*
 * Physics works in PIXEL SPACE; conversion to cell coordinates
 * happens only at draw time. `pw`/`ph` give world dimensions in
 * pixels. `px_to_cx`/`px_to_cy` round to the nearest cell.
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
/* §5  noise — 1-D value noise for terrain                                */
/* ===================================================================== */

/*
 * Value noise: a table of NOISE_N random values in [0, 1] is hashed
 * by integer position; values between integer points are
 * interpolated. We use COSINE interpolation so the resulting curve
 * is C¹-continuous (smooth, no kinks at lattice points):
 *
 *     f(t) = a · (1 - smooth(t)) + b · smooth(t)
 *     where  smooth(t) = (1 - cos(π·t)) / 2
 *
 * The resulting noise(x) ∈ [0, 1] for all real x.
 */
static float g_noise[NOISE_N];

static void noise_init(unsigned seed)
{
    srand(seed);
    for (int i = 0; i < NOISE_N; i++)
        g_noise[i] = (float)rand() / (float)RAND_MAX;
}

static float noise1d(float x)
{
    int   xi = (int)floorf(x) & (NOISE_N - 1);
    float xf = x - floorf(x);
    float a  = g_noise[xi];
    float b  = g_noise[(xi + 1) & (NOISE_N - 1)];
    float t  = (1.0f - cosf(xf * (float)M_PI)) * 0.5f;
    return a * (1.0f - t) + b * t;
}

/*
 * Two-octave fBm — sum of one big slow wave and one small fast wave.
 * Result is in [0, 1] because the weights sum to 1.
 */
static float terrain_noise(float world_x)
{
    return noise1d(world_x * T_FREQ_BIG  ) * T_WEIGHT_BIG
         + noise1d(world_x * T_FREQ_SMALL) * T_WEIGHT_SMALL;
}

/* ===================================================================== */
/* §6  terrain — height query, slope, surface glyph                       */
/* ===================================================================== */

/*
 * FloorMode — selector for the terrain-height function.
 *
 * Intent
 *   Two ways to compute floor_y_at(world_x): a constant baseline
 *   (FLAT) for testing the physics in isolation, or a value-noise
 *   profile (PERLIN) for the visually-interesting case where the
 *   robot has to adapt its launch angle to the local slope.  The
 *   'f' key cycles between them at runtime so the user can compare
 *   both behaviours without restarting.
 *
 * Members
 *   FLOOR_FLAT    = 0   Constant baseline at base_y.  Slope = 0
 *                       everywhere, so the robot always launches at
 *                       LAUNCH_ANGLE_FLAT.  Useful for debugging
 *                       the spring + flight physics in isolation.
 *   FLOOR_PERLIN  = 1   1-D value noise around base_y (see §5/§6).
 *                       Slope varies with x, so each launch picks
 *                       up the local terrain tilt via LaunchAim.
 *   FLOOR_COUNT         Sentinel for modular cycling: `mode = (mode
 *                       + 1) % FLOOR_COUNT`.
 *
 * Why FLOOR_FLAT = 0 deliberately
 *   memset-initialised Worlds default to FLAT without an explicit
 *   initialiser; the main loop bumps the default to PERLIN for
 *   visual interest, but the FLAT default makes scene_reset safe.
 *
 * References
 *   [7] Quilez "Value noise" — the function underlying FLOOR_PERLIN.
 *   [8] Perlin 1985 — the canonical procedural-noise paper.
 */
typedef enum { FLOOR_FLAT = 0, FLOOR_PERLIN, FLOOR_COUNT } FloorMode;
static const char *FLOOR_NAMES[FLOOR_COUNT] = { "FLAT  ", "PERLIN" };

/*
 * floor_y_at — world y of the terrain surface at world x.
 *
 *   FLAT    : returns base_y unchanged.
 *   PERLIN  : centres noise at 0.5, scales to ±T_AMP around base_y.
 */
static float floor_y_at(float world_x, FloorMode mode, float base_y)
{
    if (mode == FLOOR_FLAT) return base_y;
    return base_y + (terrain_noise(world_x) - 0.5f) * 2.0f * T_AMP;
}

/*
 * floor_slope — surface slope at world_x (radians).
 *
 *   slope = atan2(Δy, Δx)
 *
 * with Δy obtained by central difference (sampling 2·CELL_W to
 * each side of the foot). Central difference is unbiased and
 * second-order accurate — the right choice for a smooth slope
 * estimate.
 *
 *   slope > 0  →  ground falls to the right  (descending)
 *   slope < 0  →  ground rises to the right  (ascending)
 */
static float floor_slope(float world_x, FloorMode mode, float base_y)
{
    float dx = (float)(CELL_W * 2);
    float dy = floor_y_at(world_x + dx, mode, base_y)
             - floor_y_at(world_x - dx, mode, base_y);
    return atan2f(dy, 2.0f * dx);
}

/*
 * surface_glyph — char to draw at the surface row, based on the
 * height delta to the next column. Rough approximation of slope:
 *
 *   delta < -threshold  →  '/'   ascending right
 *   delta >  threshold  →  '\\'  descending right
 *   else                →  '_'   flat
 */
static chtype surface_glyph(float dy)
{
    if (dy < -CELL_H * 0.20f) return '/';
    if (dy >  CELL_H * 0.20f) return '\\';
    return '_';
}

/* ===================================================================== */
/* §7  trail — ring buffer of recent body positions                       */
/* ===================================================================== */

/*
 * Trail — circular ring buffer of the body's recent positions in
 * WORLD pixel space.
 *
 * Intent
 *   Renders as the fading-dot arc trail behind the robot.  Only
 *   FLIGHT-phase positions are pushed (flight_tick calls
 *   trail_push); COMPRESS and LAND don't sample because the body
 *   isn't moving meaningfully — the result is that the trail
 *   visualises ONLY the ballistic arcs, making each jump's
 *   parabolic shape readable.
 *
 *   Two parallel float arrays (`wx[]`, `wy[]`) instead of a single
 *   `Vec2[]` because the read path (render_trail) iterates each
 *   axis independently — saves one indirection per cell in the
 *   inner draw loop.
 *
 * Why a ring buffer (not a fixed-length array + memmove)
 *   • O(1) push: just write at `head` and bump.  memmove would be
 *     O(TRAIL_CAP) per FLIGHT frame.
 *   • Constant memory: the buffer never grows; old entries are
 *     silently overwritten by new ones.
 *   • Wrap-aware iteration: render_trail walks
 *     `(head − 1 − k + TRAIL_CAP) mod TRAIL_CAP` to read
 *     newest-first.
 *
 * Why WORLD coordinates (not screen / camera-relative)
 *   The camera scrolls as the robot moves forward.  Storing trail
 *   positions in WORLD space means they stay anchored to the
 *   ground regardless of camera pan — the trail "stays behind" as
 *   the camera follows the robot, exactly as expected for a
 *   trajectory visualisation.  The renderer applies
 *   (world_x − cam_x) → screen_x at draw time.
 *
 * Members
 *   wx[TRAIL_CAP]   Past x positions in WORLD pixel space.
 *   wy[TRAIL_CAP]   Past y positions, parallel to wx[].
 *   head            Index of the NEXT write slot.  After the
 *                   buffer fills, `head` wraps and the oldest
 *                   entry becomes the next slot to overwrite.
 *   count           Number of valid entries (∈ [0, TRAIL_CAP]).
 *                   Grows from 0 to TRAIL_CAP after the first
 *                   TRAIL_CAP FLIGHT frames; stays at TRAIL_CAP
 *                   thereafter.
 *
 * Invariants
 *   0 ≤ head  < TRAIL_CAP.
 *   0 ≤ count ≤ TRAIL_CAP.
 *   count < TRAIL_CAP  ⇒  wx[0..count-1] are valid (pre-wrap).
 *   count == TRAIL_CAP ⇒  every slot is valid; head points at the
 *                          OLDEST entry (next to overwrite).
 *
 * References
 *   Knuth, "The Art of Computer Programming" Vol. 1 §2.2.2 —
 *     circular buffer ("circular queue") as the canonical bounded
 *     FIFO data structure.  Same pattern used in diff_drive_robot.c
 *     for its trail.
 */
typedef struct {
    float wx[TRAIL_CAP];
    float wy[TRAIL_CAP];
    int   head;
    int   count;
} Trail;

static void trail_clear(Trail *t)
{
    t->head = t->count = 0;
}

static void trail_push(Trail *t, float wx, float wy)
{
    t->wx[t->head] = wx;
    t->wy[t->head] = wy;
    t->head        = (t->head + 1) % TRAIL_CAP;
    if (t->count < TRAIL_CAP) t->count++;
}

/* ===================================================================== */
/* §8  robot — Phase, Robot, spring physics, per-state ticks              */
/* ===================================================================== */

/* ── §8.1 Phase + Robot sub-structs ──────────────────────────────── */

/*
 * Phase — the three-phase locomotion FSM driving every jump cycle.
 *
 * State diagram
 *
 *      ┌────────────────────┐  spring_compress hits SPRING_COMPRESS_MAX
 *      │  PHASE_COMPRESS    │ ──────────────────────────────────────►
 *      │  spring loads;     │                                       │
 *      │  body sinks;       │                                       ▼
 *      │  slope sampled     │                          ┌────────────────────┐
 *      └────────────────────┘                          │  PHASE_FLIGHT      │
 *           ▲                                          │  ballistic arc;    │
 *           │ phase_t ≥ LAND_HOLD_SEC                  │  v = launch impulse│
 *           │                                          │  + gravity         │
 *      ┌────────────────────┐  body_py ≥ terrain      └────────────────────┘
 *      │  PHASE_LAND        │ ◄──────────────────────────────────── │
 *      │  impact flash;     │                                       │
 *      │  body locked;      │                                       │
 *      │  count = count+1   │                                       │
 *      └────────────────────┘                                       │
 *
 *   PHASE_COMPRESS — spring loads at SPRING_LOAD_RATE; once it
 *                    reaches SPRING_COMPRESS_MAX, the launch
 *                    impulse fires and we transition to FLIGHT.
 *   PHASE_FLIGHT   — Euler-integrated projectile motion under
 *                    gravity.  Velocity comes from the cached
 *                    LaunchAim direction × spring potential energy.
 *   PHASE_LAND     — landing flash for LAND_HOLD_SEC; body is
 *                    pinned to the new foot position.  When the
 *                    hold expires, return to COMPRESS for next jump.
 *
 * Why PHASE_COMPRESS = 0 deliberately
 *   memset-initialised Robots start in COMPRESS without an
 *   explicit initialiser; scene_reset uses this implicitly.
 */
typedef enum {
    PHASE_COMPRESS = 0,    /* spring loading, body sinking      */
    PHASE_FLIGHT,          /* projectile arc                    */
    PHASE_LAND,            /* impact flash, body locked         */
} Phase;

static const char *PHASE_NAMES[] = { "LOAD", "FLY ", "LAND" };

/*
 * Pose — kinematic position of the robot.
 *
 * Intent
 *   The robot has TWO anchor points:
 *     - foot: ground contact (changes only on LAND)
 *     - body: centre of mass (computed from foot + spring during
 *             COMPRESS/LAND, integrated freely during FLIGHT)
 *
 *   During FLIGHT body_px/body_py float free under Euler-integrated
 *   gravity; foot_px/foot_py stay frozen (the previous touchdown
 *   point).  On LAND, foot is updated to the new touchdown and body
 *   re-anchors via pose_from_spring.
 *
 * Members
 *   body_px, body_py   Centre of body in WORLD pixel space.
 *   foot_px, foot_py   Ground contact point in WORLD pixel space.
 *
 * Invariants
 *   foot_py is on the terrain (terrain_y(foot_px) == foot_py) at
 *   the START of each cycle.
 *   body_py < foot_py − BODY_HALF_H during COMPRESS (body sits
 *   above foot).
 *
 * References
 *   [1] Raibert, "Legged Robots That Balance" (MIT Press 1986) §2 —
 *       the canonical hopping-robot model.  body + foot is the
 *       Raibert state pair.
 */
typedef struct {
    float body_px, body_py;
    float foot_px, foot_py;
} Pose;

/*
 * Velocity — body linear velocity (pixels/sec).
 *
 * Intent
 *   Only meaningful during PHASE_FLIGHT.  Set ONCE at the
 *   COMPRESS → FLIGHT transition (impulse from spring PE × launch
 *   direction); evolves under gravity each frame.  Zero during
 *   COMPRESS and LAND.
 *
 * Members
 *   vx, vy   Linear velocity components, pixels per second.
 *            +x = right, +y = down (screen convention).
 *
 * References
 *   [3] Hairer et al., "Geometric Numerical Integration" — the
 *       Euler integrator in tick_flight uses:  vy += g · dt;
 *       y += vy · dt.
 */
typedef struct {
    float vx, vy;
} Velocity;

/*
 * Spring — the leg spring state.
 *
 * Intent
 *   A single scalar: the current compression of the leg spring
 *   (Hooke's law model with rest length SPRING_REST and stiffness
 *   SPRING_K).  During COMPRESS, this ramps from 0 to
 *   SPRING_COMPRESS_MAX at SPRING_LOAD_RATE.  At launch, the
 *   stored potential energy (½ · K · compress²) becomes the body's
 *   kinetic energy along the launch direction.
 *
 * Members
 *   compress   Current compression in pixels (0 at rest,
 *              SPRING_COMPRESS_MAX at full load).
 *
 * Invariants
 *   0 ≤ compress ≤ SPRING_COMPRESS_MAX.
 *
 * References
 *   [1] Raibert §3 — the spring-loaded inverted pendulum (SLIP)
 *       model; this struct is its compression state.
 *   [2] Hooke's law — F = −K·x; PE = ½·K·x²  (see spring_energy).
 */
typedef struct {
    float compress;
} Spring;

/*
 * PhaseFSM — current phase + phase-timer.
 *
 * Intent
 *   Two pieces of state that always move together: the Phase enum
 *   + the per-phase elapsed timer.  Bundling them prevents the bug
 *   of "phase == LAND but phase_t is stale from FLIGHT".
 *
 * Members
 *   phase       Current Phase (COMPRESS / FLIGHT / LAND).
 *   phase_t     Seconds since the current phase began.  Reset to 0
 *               at every phase transition.
 *
 * Invariants
 *   phase_t ≥ 0.
 *   phase == LAND  ⇒ phase_t counts up toward LAND_HOLD_SEC, then
 *                    the FSM transitions back to COMPRESS.
 */
typedef struct {
    Phase phase;
    float phase_t;
} PhaseFSM;

/*
 * LaunchAim — cached launch-direction angles, sampled during
 * COMPRESS so the launch impulse can re-use them at FLIGHT start.
 *
 * Intent
 *   On flat ground the robot launches at LAUNCH_ANGLE_FLAT
 *   (typically ~45° for maximum range).  On sloped terrain the
 *   launch angle ROTATES with the surface normal so the robot's
 *   "jump direction" feels intuitive.
 *
 *   Both values are computed ONCE when the spring is full (the
 *   COMPRESS → FLIGHT transition) and then frozen until the next
 *   COMPRESS sample.  This avoids recomputing the slope (which
 *   queries the noise function) in the inner integrator.
 *
 * Members
 *   slope_angle   atan(dy/dx) of the terrain at the current foot
 *                 position.  Radians.  Positive = surface tilts
 *                 down-right.
 *   eff_angle     The effective launch angle = LAUNCH_ANGLE_FLAT +
 *                 slope_angle (clamped).  This is what (vx, vy)
 *                 actually point along at FLIGHT start.
 *
 * References
 *   [1] Raibert §4 — terrain-adaptive hopping uses the leg posture
 *       to set the launch direction; this struct caches the
 *       equivalent decision for our simplified planar model.
 */
typedef struct {
    float slope_angle;
    float eff_angle;
} LaunchAim;

/*
 * Robot — the simulated agent.
 *
 * Layered ownership
 *
 *     Robot
 *       ├── pose      : Pose         ← body + foot anchor points
 *       ├── vel       : Velocity     ← (vx, vy) — meaningful during FLIGHT
 *       ├── spring    : Spring       ← leg compression
 *       ├── fsm       : PhaseFSM     ← (phase, phase_t) state machine
 *       ├── aim       : LaunchAim    ← cached slope + launch angle
 *       └── trail     : Trail        ← ring buffer of past positions
 *
 *   Five layered concerns: kinematics (pose), dynamics (vel + spring),
 *   control (fsm), targeting (aim), history (trail).  Each sub-struct
 *   names its responsibility, so a reader can answer "what state does
 *   this helper touch?" from the parameter list alone (e.g.
 *   spring_energy takes only the Spring; pose_from_spring touches
 *   Pose + Spring).
 *
 * Why no World/Camera/UI fields here
 *   Scene owns those — see §9 for the layered diagram.  Robot stays
 *   purely about the agent's own state, not the environment it
 *   moves through or the controls used to interact with it.
 *
 * References
 *   [1] Raibert, "Legged Robots That Balance" (MIT Press 1986) —
 *       the SLIP model + 3-phase jump cycle this file implements.
 */
typedef struct {
    Pose      pose;
    Velocity  vel;
    Spring    spring;
    PhaseFSM  fsm;
    LaunchAim aim;
    Trail     trail;
} Robot;

/* ── §8.1b Scene sub-structs ─────────────────────────────────────── */

/*
 * Camera — scrolling viewport into world space.
 *
 * Intent
 *   The world extends well past the terminal width (the robot jumps
 *   FORWARD continuously).  Camera holds a single scalar — the world
 *   x of the LEFT screen edge — and scrolls smoothly toward the
 *   robot via cam_update (low-pass filter, CAM_SPEED time-constant).
 *
 *   Pixel → screen conversion: screen_x = world_x − cam_x.  Anything
 *   off-screen is silently clipped by the renderer's bounds check.
 *
 * Members
 *   cam_x   World x of the left screen edge (pixels).  Always ≥ 0.
 *           Camera trails the robot at a fixed screen-x offset, so
 *           the robot stays visually centred.
 *
 * Invariants
 *   cam_x ≥ 0 (clamp in cam_update).
 */
typedef struct {
    float cam_x;
} Camera;

/*
 * World — the terrain configuration.
 *
 * Intent
 *   Bundles the two values that describe the ground the robot
 *   bounces on: which floor algorithm to use (FLAT or PERLIN) and
 *   the baseline y coordinate around which terrain undulates.
 *
 *   floor_mode is user-cyclable via 'f'; base_y is recomputed on
 *   SIGWINCH (terrain re-centres if the terminal height changes).
 *
 * Members
 *   base_y       Flat-floor y in pixel space.  PERLIN terrain
 *                undulates around this baseline.  Recomputed from
 *                screen rows in scene_init / app_do_resize.
 *   floor_mode   FLAT or PERLIN.  Cycled by 'f' / 'F' keys.
 *
 * Invariants
 *   base_y > 0 after scene_init.
 *   floor_mode ∈ {FLOOR_FLAT, FLOOR_PERLIN}.
 */
typedef struct {
    float     base_y;
    FloorMode floor_mode;
} World;

/*
 * SimControls — user-facing playback knobs + scene-level counters.
 *
 * Intent
 *   The input handler writes these; scene_tick + render_hud read
 *   them.  Same shape as the SimControls sub-struct on every other
 *   demo's Scene in this project.
 *
 *   `launch_count` lives here because it's a SCENE-level statistic
 *   (how many jumps have happened in this session) — not per-Robot.
 *
 * Members
 *   paused         true → scene_tick early-returns; rendering
 *                  continues.  Toggled by SPACE / p key.
 *   speed_level    Index into SPEED_MULTS — multiplier on the
 *                  launch impulse.  Cycled by 'a' key.
 *   launch_count   Number of jumps since the last reset (HUD
 *                  readout).  Bumped at every COMPRESS → FLIGHT
 *                  transition.
 *
 * Invariants
 *   0 ≤ speed_level < SPEED_LEVELS.
 *   launch_count ≥ 0.
 */
typedef struct {
    bool paused;
    int  speed_level;
    int  launch_count;
} SimControls;

/*
 * Scene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── robot    : Robot        ← the simulated agent (§8.1)
 *       ├── world    : World        ← base_y + floor_mode
 *       ├── camera   : Camera       ← scrolling viewport
 *       └── sim      : SimControls  ← paused + speed_level + launch_count
 *
 *   Data-flow direction:
 *
 *     world      → robot.aim     (slope sampled during COMPRESS)
 *     robot.pose → camera        (cam tracks body_px in cam_update)
 *     sim        → robot.vel     (speed_level scales launch impulse)
 *
 *   Helpers that touch only one sub-domain take just that sub-struct
 *   (e.g. spring_energy takes only the Spring; cam_update touches
 *   Camera + Robot.pose); the "what does this function touch"
 *   question is answerable from the signature.
 */
typedef struct {
    Robot       robot;
    World       world;
    Camera      camera;
    SimControls sim;
} Scene;

/* ── §8.2 spring physics — Hooke's law & energy ──────────────────── */

/* Stored potential energy in the spring (relative units; m=1, k=K).
 * Used by the PE gauge HUD; not by the integrator. */
static inline float spring_energy(float compress)
{
    return 0.5f * SPRING_K * compress * compress;
}

/* Body height above foot for a given compression. */
static inline float leg_length(float compress)
{
    return SPRING_REST - compress;
}

/*
 * pose_from_spring — recompute body position from foot + compression.
 *
 *   body_py = foot_py − leg_length − BODY_HALF_H
 *
 * Called during COMPRESS and LAND, when the robot is grounded and
 * the body's vertical position is determined entirely by spring
 * state. (During FLIGHT body_py is integrated separately.)
 */
static void pose_from_spring(Robot *r)
{
    r->pose.body_px = r->pose.foot_px;
    r->pose.body_py = r->pose.foot_py - leg_length(r->spring.compress) - BODY_HALF_H;
}

/* ── §8.3 effective launch angle (slope-adapted) ─────────────────── */

/*
 * Slope > 0 (descending) → reduce angle (shallower jump, ride down).
 * Slope < 0 (ascending)  → increase angle (steeper jump, clear rise).
 * Clamp to [LAUNCH_MIN, LAUNCH_MAX] so degenerate slopes can't make
 * the launch nearly vertical or nearly horizontal.
 */
static float effective_launch_angle(float slope)
{
    return clampf(LAUNCH_ANGLE - slope * SLOPE_SCALE,
                  LAUNCH_MIN, LAUNCH_MAX);
}

/* ── §8.4 cam_update — right-edge follow camera ──────────────────── */

/*
 * If the robot's screen position has crossed CAM_TRIGGER × screen
 * width, the camera target is set to (body_px − trigger), which
 * keeps the robot pinned at the trigger column. cam_x exponentially
 * chases that target at CAM_SPEED — so the camera slides smoothly
 * toward it instead of snapping.
 *
 * cam_x is clamped to ≥ 0 so we never scroll left of world origin.
 */
static void cam_update(Scene *s, float dt, int cols)
{
    float scr_w   = (float)pw(cols);
    float bot_sx  = s->robot.pose.body_px - s->camera.cam_x;  /* screen x of robot */
    float trigger = scr_w * CAM_TRIGGER;

    if (bot_sx > trigger) {
        float target = s->robot.pose.body_px - trigger;
        s->camera.cam_x += (target - s->camera.cam_x) * CAM_SPEED * dt;
    }

    if (s->camera.cam_x < 0.0f) s->camera.cam_x = 0.0f;
}

/* ── §8.5 compress_tick — load the spring, then launch ───────────── */

/*
 * The body sinks toward the foot as `spring_compress` ramps linearly
 * from 0 to SPRING_COMPRESS_MAX over T_COMPRESS seconds. We sample
 * the terrain slope at the foot to compute the slope-adapted launch
 * angle (used at the moment of release).
 *
 * On reaching full compression, convert stored spring energy into
 * launch velocity using Hooke's-law energy conservation:
 *
 *     v = compress · √(SPRING_K / m)
 *     (m = 1, so v = compress · √K = 48 · 5 = 240 px/sec at full)
 *
 * Decompose v into (vx, vy) using the effective launch angle and the
 * horizontal speed multiplier. Clear compress and transition to FLIGHT.
 */
/* compress_progress_fraction — phase_t / T_COMPRESS clamped to [0,1].
 * Names the linear ramp progress so the inner update reads in
 * English ("compress goes to MAX × progress") rather than naked
 * division. */
static inline float compress_progress_fraction(float phase_t) {
    return clampf(phase_t / T_COMPRESS, 0.0f, 1.0f);
}

/* sample_terrain_aim — query the terrain slope at the current foot
 * position and convert it to the effective launch angle.  Stored
 * back on Robot.aim so flight_tick can read it without re-querying
 * the noise function (which is the expensive part). */
static inline void sample_terrain_aim(Scene *s) {
    Robot *r = &s->robot;
    r->aim.slope_angle = floor_slope(r->pose.foot_px,
                                     s->world.floor_mode, s->world.base_y);
    r->aim.eff_angle   = effective_launch_angle(r->aim.slope_angle);
}

/* spring_pe_to_launch_speed — convert spring compression into the
 * outgoing launch speed via energy conservation (m = 1):
 *
 *     ½ · m · v²   =   ½ · K · x²
 *           ↓
 *     v = x · √(K/m)   =   x · √K     (since m = 1)
 *
 * This is the moment the stored spring PE becomes the body's KE.
 * Reference: [4] Hooke's law + classical energy conservation. */
static inline float spring_pe_to_launch_speed(float compress) {
    return compress * sqrtf(SPRING_K);
}

/* emit_launch_impulse — at the COMPRESS → FLIGHT transition,
 * convert the loaded spring into a velocity vector along the
 * cached aim direction, then transition the FSM.
 *
 *   vx = + v_launch · cos(eff_angle) · speed_mult       (forward)
 *   vy = − v_launch · sin(eff_angle)                     (up; y-down)
 *
 * The speed_mult is the user's "speed level" knob — scales the
 * horizontal component (only) so the user can tune horizontal range
 * without changing the physics. */
static inline void emit_launch_impulse(Scene *s) {
    Robot *r = &s->robot;
    float v_launch  = spring_pe_to_launch_speed(r->spring.compress);
    float speed_mult = SPEED_MULTS[s->sim.speed_level];

    r->vel.vx =  v_launch * cosf(r->aim.eff_angle) * speed_mult;
    r->vel.vy = -v_launch * sinf(r->aim.eff_angle);   /* y-down: up = negative */

    r->spring.compress = 0.0f;          /* leg now fully extended */
    r->fsm.phase       = PHASE_FLIGHT;
    r->fsm.phase_t     = 0.0f;
    s->sim.launch_count++;
}

/*
 * compress_tick — one frame of PHASE_COMPRESS.
 *
 *   (1) Ramp spring.compress linearly from 0 to SPRING_COMPRESS_MAX
 *       over T_COMPRESS seconds (the body sinks toward the foot
 *       under the spring's contraction).
 *   (2) Recompute pose from foot + new compression.
 *   (3) Sample terrain slope continuously so the launch direction
 *       reflects the latest aim (foot doesn't move here, but the
 *       defensive sample is cheap).
 *   (4) When the spring reaches full compression, emit the launch
 *       impulse and transition to PHASE_FLIGHT.
 */
static void compress_tick(Scene *s)
{
    Robot *r = &s->robot;

    /* (1)+(2) ramp compression and re-anchor the body */
    r->spring.compress = SPRING_COMPRESS_MAX
                       * compress_progress_fraction(r->fsm.phase_t);
    pose_from_spring(r);

    /* (3) sample terrain slope into Robot.aim */
    sample_terrain_aim(s);

    /* (4) full compression → launch */
    bool fully_loaded = (r->fsm.phase_t >= T_COMPRESS);
    if (fully_loaded) emit_launch_impulse(s);
}

/* ── §8.6 flight_tick — projectile motion, detect landing ────────── */

/*
 * Forward-Euler integration of horizontal and vertical motion under
 * uniform gravity. Push every flight position into the trail buffer
 * so the parabolic arc is visible. Test for floor contact:
 *
 *     foot_proj_y = body_py + BODY_HALF_H + SPRING_REST
 *
 * is the y the foot WOULD have if the leg were fully extended. When
 * that exceeds the terrain height under the body, the foot has
 * "punched into" the ground — time to land. Snap the foot to the
 * surface, zero the velocity, and transition to LAND.
 */
/* integrate_projectile_step — forward-Euler integration of
 * Newtonian projectile motion under uniform gravity.
 *
 *   vy +=  g · dt        (acceleration → velocity)
 *   x  += vx · dt        (velocity     → position)
 *   y  += vy · dt
 *
 * Non-symplectic (energy drifts upward at long times) but FLIGHT
 * phases last < 1 s and drift is sub-pixel.  Reference: [6] Hairer
 * et al. + the file-header References for full discussion. */
static inline void integrate_projectile_step(Robot *r, float dt) {
    r->vel.vy       += GRAVITY * dt;
    r->pose.body_px += r->vel.vx * dt;
    r->pose.body_py += r->vel.vy * dt;
}

/* foot_projection_y — the y the foot WOULD HAVE if the leg were
 * fully extended below the current body position.  Used by the
 * floor-contact test:
 *
 *   foot_proj_y = body_y + BODY_HALF_H + SPRING_REST
 *
 * If foot_proj_y ≥ terrain(body_x), the leg has "punched into"
 * the ground — time to land. */
static inline float foot_projection_y(const Robot *r) {
    return r->pose.body_py + BODY_HALF_H + SPRING_REST;
}

/* commit_landing — snap the foot to the terrain surface beneath
 * the body, zero everything, and transition the FSM to LAND.
 * Called once at the FLIGHT → LAND transition. */
static inline void commit_landing(Robot *r, float ground_y) {
    r->pose.foot_px    = r->pose.body_px;       /* foot lands directly below body */
    r->pose.foot_py    = ground_y;
    r->spring.compress = 0.0f;
    pose_from_spring(r);
    r->vel.vx = r->vel.vy = 0.0f;

    r->fsm.phase   = PHASE_LAND;
    r->fsm.phase_t = 0.0f;
}

/*
 * flight_tick — one frame of PHASE_FLIGHT.
 *
 *   (1) Integrate projectile motion (Newton + Euler).
 *   (2) Push the new body position into the trail so the parabolic
 *       arc is visible.
 *   (3) Test foot-projection against the terrain under the body.
 *       If the leg has punched in, commit_landing.
 */
static void flight_tick(Scene *s, float dt)
{
    Robot *r = &s->robot;

    /* (1) Newtonian projectile motion via forward Euler. */
    integrate_projectile_step(r, dt);

    /* (2) Visualise the arc. */
    trail_push(&r->trail, r->pose.body_px, r->pose.body_py);

    /* (3) Floor contact test: would the leg punch into the ground? */
    float ground_y = floor_y_at(r->pose.body_px,
                                s->world.floor_mode, s->world.base_y);
    bool leg_punched_ground = (foot_projection_y(r) >= ground_y);
    if (leg_punched_ground) commit_landing(r, ground_y);
}

/* ── §8.7 land_tick — brief stun, then back to COMPRESS ──────────── */

/*
 * The body is locked at the landing pose (set in flight_tick). We
 * just count down T_LAND. The visual difference is the body glyph
 * ('*' instead of '@') and the absence of the spring coil.
 */
static void land_tick(Robot *r)
{
    pose_from_spring(r);
    if (r->fsm.phase_t >= T_LAND) {
        r->fsm.phase   = PHASE_COMPRESS;
        r->fsm.phase_t = 0.0f;
    }
}

/* ── §8.8 scene_tick — orchestrator: dispatch on phase ──────────── */

/*
 * One frame.  If sim.paused, do nothing — rendering keeps running
 * so the HUD stays live.  When unpaused:
 *
 *   (1) Camera follows the robot (smooth low-pass tracker).
 *   (2) phase_t is accumulated CENTRALLY so each per-phase tick
 *       can read it without re-implementing the bookkeeping.
 *   (3) Dispatch on Robot.fsm.phase to the matching per-phase tick.
 */
static void scene_tick(Scene *s, float dt, int cols)
{
    if (s->sim.paused) return;

    cam_update(s, dt, cols);
    s->robot.fsm.phase_t += dt;

    switch (s->robot.fsm.phase) {
    case PHASE_COMPRESS: compress_tick(s);     break;
    case PHASE_FLIGHT:   flight_tick(s, dt);   break;
    case PHASE_LAND:     land_tick(&s->robot); break;
    }
}

/* ── §8.9 scene_init / scene_reset ────────────────────────────────── */

static void scene_init(Scene *s, int rows)
{
    /* Preserve user-tunable settings across init. */
    FloorMode saved_floor = s->world.floor_mode;
    int       saved_speed = s->sim.speed_level;

    memset(s, 0, sizeof *s);
    s->world.floor_mode = saved_floor;
    s->sim.speed_level  = saved_speed;
    s->sim.paused       = false;

    /* Terrain baseline at 72 % of screen height — leaves room above
     * for arcs and energy gauge, room below for terrain-fill. */
    s->world.base_y = (float)ph(rows) * 0.72f;

    Robot *r = &s->robot;

    /* Spawn near the left edge, planted on the surface. */
    r->pose.foot_px    = (float)(CAM_START_COL * CELL_W);
    r->pose.foot_py    = floor_y_at(r->pose.foot_px,
                                    s->world.floor_mode, s->world.base_y);
    r->aim.slope_angle = floor_slope(r->pose.foot_px,
                                     s->world.floor_mode, s->world.base_y);
    r->aim.eff_angle   = effective_launch_angle(r->aim.slope_angle);

    r->fsm.phase   = PHASE_COMPRESS;
    r->fsm.phase_t = 0.0f;
    s->camera.cam_x = 0.0f;

    pose_from_spring(r);
}

/* Light reset for the 'r' key — preserves theme/speed/floor mode. */
static void scene_reset(Scene *s, int rows)
{
    trail_clear(&s->robot.trail);
    scene_init(s, rows);
}

/* ===================================================================== */
/* §9  render                                                             */
/* ===================================================================== */

/* ── §9.1 helpers ────────────────────────────────────────────────── */

/* Convert world x to screen column via camera offset. */
static inline int scr_cx(float world_px, float cam_x)
{
    return px_to_cx(world_px - cam_x);
}

/*
 * Top row 0 reserved for HUD; bottom row reserved for hint strip.
 * Drawables must live in rows 1..rows-2.
 */
static inline bool in_bounds(int cx, int cy, int cols, int rows)
{
    return cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1;
}

/* ── §9.2 render_terrain — per-column sweep ──────────────────────── */

/*
 * For every screen column, compute the terrain row from the noise
 * function (camera offset converts screen column → world x), then
 * paint:
 *
 *   row surf       surface glyph   bright green
 *   row surf+1     texture pattern dim green   (alternating . and :)
 *   row surf+2..   solid fill      dark green  (alternating # and ' ')
 *
 * The texture row gives the surface a fuzzy/grassy edge; the solid
 * fill gives weight to the terrain mass. Both use simple `sc % 2`
 * checkerboards so the pattern is stable regardless of camera scroll.
 */
static void render_terrain(FloorMode mode, float base_y, float cam_x,
                           int cols, int rows)
{
    for (int sc = 0; sc < cols; sc++) {
        float wx      = cam_x + (float)(sc * CELL_W);
        float fy      = floor_y_at(wx, mode, base_y);
        float fy_next = floor_y_at(wx + (float)CELL_W, mode, base_y);
        int   surf    = px_to_cy(fy);
        if (surf < 1)        surf = 1;
        if (surf > rows - 2) surf = rows - 2;

        chtype sg = (mode == FLOOR_PERLIN)
                  ? surface_glyph(fy_next - fy)
                  : '_';

        attron(COLOR_PAIR(CP_SURF) | A_BOLD);
        mvaddch(surf, sc, sg);
        attroff(COLOR_PAIR(CP_SURF) | A_BOLD);

        if (surf + 1 < rows - 1) {
            attron(COLOR_PAIR(CP_SURF) | A_DIM);
            mvaddch(surf + 1, sc, (sc % 2 == 0) ? ':' : '.');
            attroff(COLOR_PAIR(CP_SURF) | A_DIM);
        }

        attron(COLOR_PAIR(CP_ROCK) | A_DIM);
        for (int r = surf + 2; r < rows - 1; r++)
            mvaddch(r, sc, (sc % 2 == 0) ? '#' : ' ');
        attroff(COLOR_PAIR(CP_ROCK) | A_DIM);
    }
}

/* ── §9.3 render_trail — newest brightest, fading leftward ───────── */

/*
 * Iterate newest-first so the freshest dots paint last (top of the
 * stack) and old ones paint first (under any overlaps).
 *
 *   age in [0, 1]  where 0 = newest, 1 = oldest
 *     age < 0.15 → BOLD '.' fresh blue
 *     age < 0.40 → '.' blue
 *     age ≥ 0.40 → DIM ':' dim blue   (':' for visual recede)
 */
static void render_trail(const Robot *r, float cam_x, int cols, int rows)
{
    for (int k = 0; k < r->trail.count; k++) {
        int idx = (r->trail.head - 1 - k + TRAIL_CAP) % TRAIL_CAP;
        int cx  = scr_cx     (r->trail.wx[idx], cam_x);
        int cy  = px_to_cy   (r->trail.wy[idx]);
        if (!in_bounds(cx, cy, cols, rows)) continue;

        float  age = (r->trail.count > 1)
                   ? (float)k / (float)(r->trail.count - 1) : 0.0f;
        int    cp  = (age < 0.40f) ? CP_TRAIL : CP_TRAIL_OLD;
        attr_t at  = (age < 0.15f) ? A_BOLD
                   : (age < 0.40f) ? A_NORMAL : A_DIM;
        chtype ch  = (age < 0.40f) ? '.' : ':';

        attron(COLOR_PAIR(cp) | at);
        mvaddch(cy, cx, ch);
        attroff(COLOR_PAIR(cp) | at);
    }
}

/* ── §9.4 render_spring — coil leg between body bottom and foot ──── */

/*
 * Draw a coil from the body bottom row down to the foot row. The
 * coil pattern { '(', '|', ')', '|' } cycles per row; visually the
 * coil "shrinks" as the body descends (fewer rows between body and
 * foot during full compression).
 *
 * Color shifts yellow → orange → red as compression rises, giving
 * an immediate visual cue of how loaded the spring is.
 */
static void render_spring(const Robot *r, float cam_x, int cols, int rows)
{
    int cx       = scr_cx(r->pose.body_px, cam_x);
    int body_bot = px_to_cy(r->pose.body_py + BODY_HALF_H);
    int foot_cy  = px_to_cy(r->pose.foot_py);

    if (cx < 0 || cx >= cols) return;

    float ratio = r->spring.compress / SPRING_COMPRESS_MAX;
    int    cp   = (ratio < 0.30f) ? CP_SPRING_LO
                : (ratio < 0.70f) ? CP_SPRING_MD : CP_SPRING_HI;
    attr_t at   = (ratio > 0.70f) ? A_BOLD : A_NORMAL;

    static const chtype coil[4] = { '(', '|', ')', '|' };
    for (int row = body_bot; row < foot_cy; row++) {
        if (row < 1 || row >= rows - 1) continue;
        attron(COLOR_PAIR(cp) | at);
        mvaddch(row, cx, coil[(row - body_bot) & 3]);
        attroff(COLOR_PAIR(cp) | at);
    }

    /* Foot marker. */
    if (foot_cy >= 1 && foot_cy < rows - 1) {
        attron(COLOR_PAIR(CP_ROCK));
        mvaddch(foot_cy, cx, 'v');
        attroff(COLOR_PAIR(CP_ROCK));
    }
}

/* ── §9.5 render_body — phase-coded glyph at body position ───────── */

/*
 *   PHASE_COMPRESS  →  '@'  white BOLD     (planted, loading)
 *   PHASE_FLIGHT    →  'O'  cyan BOLD      (airborne)
 *   PHASE_LAND      →  '*'  magenta BOLD   (impact flash)
 */
static void render_body(const Robot *r, float cam_x, int cols, int rows)
{
    int cx = scr_cx(r->pose.body_px, cam_x);
    int cy = px_to_cy(r->pose.body_py);
    if (!in_bounds(cx, cy, cols, rows)) return;

    chtype ch; int cp;
    switch (r->fsm.phase) {
    case PHASE_COMPRESS: ch = '@'; cp = CP_BODY;   break;
    case PHASE_FLIGHT:   ch = 'O'; cp = CP_FLIGHT; break;
    default:             ch = '*'; cp = CP_LAND;   break;
    }

    attron(COLOR_PAIR(cp) | A_BOLD);
    mvaddch(cy, cx, ch);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

/* ── §9.6 render_pe_bar — vertical PE gauge, COMPRESS only ────────── */

/*
 * PE-bar layout constants — placement near the right edge of the
 * sim area + minimum bar height when the terminal is short.
 */
enum {
    PE_BAR_RIGHT_INSET    = 3,   /* cols from right edge to bar left col */
    PE_BAR_BOTTOM_INSET   = 2,   /* rows from bottom edge to bar bottom  */
    PE_BAR_VERTICAL_MARGIN = 10, /* rows reserved above/below the bar    */
    PE_BAR_MIN_HEIGHT     = 4,
    PE_BAR_WIDTH_CELLS    = 2,   /* bar is 2 cells wide ('||' or '!!')   */
};
#define PE_BAR_HIGH_RATIO 0.80f  /* above this, switch to red + '!' tip  */

/* PeBarGeometry — screen-space layout of the PE gauge in one place. */
typedef struct {
    int x;        /* left column of the bar (top-left anchor) */
    int top;      /* topmost simulation row used by the bar   */
    int bottom;   /* bottom simulation row (just above hint)  */
    int height;   /* bar height in rows                       */
} PeBarGeometry;

/* compute_pe_bar_geometry — derive the PE-bar layout from screen
 * extent.  Returns height=0 if the terminal is too small to fit
 * even the minimum 4-row bar; caller skips rendering in that case. */
static inline PeBarGeometry compute_pe_bar_geometry(int cols, int rows) {
    PeBarGeometry g;
    g.x      = cols - PE_BAR_RIGHT_INSET;
    g.bottom = rows - PE_BAR_BOTTOM_INSET;
    g.height = (rows > PE_BAR_VERTICAL_MARGIN + 2)
             ? rows - PE_BAR_VERTICAL_MARGIN
             : PE_BAR_MIN_HEIGHT;
    g.top    = g.bottom - g.height;
    return g;
}

/* pe_ratio_quadratic — PE / PE_max as a quadratic function of
 * compression:
 *
 *     PE = ½·k·x²      →   PE / PE_max = (x / x_max)²
 *
 * The quadratic shape is the WHOLE PEDAGOGICAL POINT — bar fills
 * slowly at first then accelerates near full compression, teaching
 * the quadratic energy law. */
static inline float pe_ratio_quadratic(float compress) {
    return spring_energy(compress) / spring_energy(SPRING_COMPRESS_MAX);
}

/* paint_pe_label — small "PE" tag above the bar, in HUD yellow. */
static inline void paint_pe_label(const PeBarGeometry *g) {
    if (g->top - 1 < 1) return;
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(g->top - 1, g->x, "PE");
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* paint_filled_cell / paint_empty_cell — paint ONE row of the
 * 2-cell-wide bar.  Filled cells use bar_pair (red CP_SPRING_HI at
 * high charge, green CP_PE otherwise); empty cells dim down with
 * the HUD background and a '.' rest pattern. */
static inline void paint_filled_cell(int row, int x, chtype glyph, int pair) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(row, x,     glyph);
    mvaddch(row, x + 1, glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}
static inline void paint_empty_cell(int row, int x) {
    attron(COLOR_PAIR(PAIR_HUD));
    mvaddch(row, x,     '.');
    mvaddch(row, x + 1, '.');
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* paint_pe_bar_column — paint the whole vertical bar.  filled_cells
 * rows from the bottom up are FILLED; the rest are EMPTY rest dots.
 * At high charge (ratio > PE_BAR_HIGH_RATIO) the fill switches to
 * CP_SPRING_HI red and the TOPMOST filled cell becomes '!' instead
 * of '|' — a "ready-to-fire" visual cue. */
static inline void paint_pe_bar_column(const PeBarGeometry *g,
                                        int filled_cells, float ratio,
                                        int rows) {
    bool high_charge = (ratio > PE_BAR_HIGH_RATIO);
    int  fill_pair   = high_charge ? CP_SPRING_HI : CP_PE;

    for (int i = 0; i < g->height; i++) {
        int row = g->bottom - i;
        if (row < 1 || row >= rows - 1) continue;     /* HUD-row guard */

        bool   is_filled  = (i < filled_cells);
        bool   is_top_lit = (i == filled_cells - 1);
        chtype glyph      = (high_charge && is_top_lit) ? '!' : '|';

        if (is_filled) paint_filled_cell(row, g->x, glyph, fill_pair);
        else           paint_empty_cell (row, g->x);
    }
}

/*
 * render_pe_bar — vertical "spring potential energy" gauge.
 *
 *   Visible during PHASE_COMPRESS only (other phases have no
 *   stored energy to show).  Anchored at the right edge of the
 *   simulation area.
 *
 *   Bar height ∝ PE / PE_max = (compress / compress_max)²
 *   — the quadratic shape teaches Hooke's energy law visually.
 */
static void render_pe_bar(const Robot *r, int cols, int rows)
{
    if (r->fsm.phase != PHASE_COMPRESS) return;

    PeBarGeometry geom = compute_pe_bar_geometry(cols, rows);
    if (geom.x < 0 || geom.top < 1) return;

    float ratio        = pe_ratio_quadratic(r->spring.compress);
    int   filled_cells = (int)(ratio * (float)geom.height + 0.5f);

    paint_pe_label    (&geom);
    paint_pe_bar_column(&geom, filled_cells, ratio, rows);
}

/* ── §9.7 render_hud — yellow status row 0 + cyan hint bottom row ── */

/*
 * Status row (row 0, PAIR_HUD, BOLD): floor mode, current phase,
 * compression / max compression, stored PE, slope angle, effective
 * launch angle, speed multiplier, robot screen column, total
 * camera scroll, jump count, fps.
 *
 * Hint strip (bottom row, PAIR_HINT, BOLD): full key list.
 *
 * The 'PAUSED' banner overlays the centre of the screen when the
 * simulation is frozen.
 */
/* radians_to_degrees — unit conversion used only by the HUD. */
static inline float radians_to_degrees(float radians) {
    return radians * (180.0f / (float)M_PI);
}

/* hud_paint_text_padded — paint `text` at (row, col) in pair `pair`
 * + A_BOLD, then pad the rest of the row with spaces so the row's
 * background colour reaches edge-to-edge (otherwise the terminal's
 * default bg shows through past the printed text). */
static inline void hud_paint_text_padded(int row, int col, int pair,
                                          const char *text, int total_cols) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddnstr(row, col, text, total_cols - col);
    int used = (int)strlen(text);
    for (int x = col + used; x < total_cols; x++)
        mvaddch(row, x, ' ');
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* hud_paint_text — same as above but WITHOUT padding (the bottom
 * key-hint strip doesn't need to span the full width). */
static inline void hud_paint_text(int row, int col, int pair,
                                   const char *text) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* format_hud_status — write the top-row status string into `buf`.
 * Reports fps, floor mode, current phase, compression / max, stored
 * PE, slope angle, effective launch angle, speed multiplier, robot
 * screen column, camera scroll, jump count, and paused state. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
    const Robot *r = &s->robot;
    int robot_scr_col = scr_cx(r->pose.body_px, s->camera.cam_x);

    snprintf(buf, buflen,
             " %5.1f fps  [%s] %-4s  cmp:%3.0f/%3.0f  PE:%5.0f  "
             "slope:%+5.1f°  launch:%4.1f°  spd:%.1fx  col:%-3d  "
             "cam:%5.0f  jumps:%-3d  %s ",
             fps, FLOOR_NAMES[s->world.floor_mode], PHASE_NAMES[r->fsm.phase],
             r->spring.compress, SPRING_COMPRESS_MAX,
             spring_energy(r->spring.compress),
             radians_to_degrees(r->aim.slope_angle),
             radians_to_degrees(r->aim.eff_angle),
             SPEED_MULTS[s->sim.speed_level],
             robot_scr_col, s->camera.cam_x, s->sim.launch_count,
             s->sim.paused ? "PAUSED" : "running");
}

/* draw_hud_status — left-justified status row 0 (PAIR_HUD yellow). */
static void draw_hud_status(const Scene *s, double fps, int cols) {
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_BUF_LEN];
    format_hud_status(s, fps, buf, sizeof buf);
    hud_paint_text_padded(HUD_TOP_ROW, 0, PAIR_HUD, buf, cols);
}

/* draw_hud_hint — bottom-row key bindings strip (PAIR_HINT cyan). */
static void draw_hud_hint(int rows) {
    static const char *KEY_HINT =
        " q:quit  spc:pause  r:reset  f:floor  n:new-terrain  a:speed ";
    hud_paint_text(rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* draw_paused_banner — centred " PAUSED " overlay shown when the
 * simulation is frozen.  Uses CP_LAND (the landing-flash colour)
 * to repurpose an already-bright pair without adding another
 * one to color_init. */
static void draw_paused_banner(int cols, int rows) {
    static const char *PAUSED_LABEL = " PAUSED ";
    int label_width = (int)strlen(PAUSED_LABEL);
    int banner_col  = cols / 2 - label_width / 2;
    if (banner_col < 0) return;
    hud_paint_text(rows / 2, banner_col, CP_LAND, PAUSED_LABEL);
}

/*
 * render_hud — required HUD per CLAUDE.md spec.
 *
 *   Row 0 (PAIR_HUD, BOLD): fps + floor + phase + spring state +
 *     slope/launch angles + speed + camera + jumps + paused tag
 *   Bottom row (PAIR_HINT, BOLD): full key list
 *   PAUSED overlay: centred banner when sim.paused is true
 *
 * Both HUD pairs sit on default background (-1) so they stay
 * legible regardless of theme.
 */
static void render_hud(const Scene *s, double fps, int cols, int rows)
{
    draw_hud_status(s, fps, cols);
    draw_hud_hint  (rows);
    if (s->sim.paused) draw_paused_banner(cols, rows);
}

/* ── §9.8 scene_draw — paint one frame ───────────────────────────── */

/*
 * Painter's order — last write wins:
 *   (1) trail  (background; oldest first, newer overwrites older)
 *   (2) terrain
 *   (3) spring leg (only if grounded)
 *   (4) body glyph (always on top of leg)
 *   (5) PE bar (only during COMPRESS)
 *   (6) HUD on row 0 + hint strip on bottom row
 */
static void scene_draw(const Scene *s, double fps, int cols, int rows)
{
    erase();

    const Robot *r = &s->robot;
    float cam_x = s->camera.cam_x;

    render_trail   (r, cam_x, cols, rows);
    render_terrain (s->world.floor_mode, s->world.base_y, cam_x, cols, rows);

    if (r->fsm.phase == PHASE_COMPRESS || r->fsm.phase == PHASE_LAND)
        render_spring(r, cam_x, cols, rows);

    render_body    (r, cam_x, cols, rows);
    render_pe_bar  (r, cols, rows);
    render_hud     (s, fps, cols, rows);
}

/* ===================================================================== */
/* §10  screen — ncurses init / present                                   */
/* ===================================================================== */

/*
 * Screen — terminal cell extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Tracks the TERMINAL side of the demo: cell dimensions for HUD
 *   placement and field clipping.  ncurses owns the back buffer;
 *   this struct is the source-of-truth for CELL-space dimensions,
 *   refreshed via getmaxyx in screen_init / screen_resize.
 *
 *   The simulation lives in WORLD PIXEL space (CELL_W × CELL_H
 *   sub-pixels per cell, see §4 coords).  Scene.world.base_y is
 *   derived from `ph(rows)` and recomputed on every SIGWINCH so
 *   the terrain re-centres when the terminal height changes.
 *
 *   The horizontal direction is UNBOUNDED in world space (the robot
 *   keeps jumping forward) — Scene.camera scrolls the visible
 *   window through this infinite world; Screen.cols controls how
 *   much of it is visible at any time.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_free / screen_present touch ncurses' initscr / endwin /
 *     doupdate.  They all take `Screen *` to make this layer
 *     explicit at the type level.
 *   • Symmetry with every other demo in this project — `{cols, rows}`
 *     is the canonical Screen shape.
 *
 * Members
 *   cols   Terminal width in CELLS (getmaxyx).
 *   rows   Terminal height in CELLS.  Row 0 hosts the HUD status
 *          strip; row (rows - 1) hosts the key-hint strip.  The
 *          simulation paints into rows [1, rows-2].
 *
 * Invariants
 *   cols > 0, rows > 2 (need at least 1 sim row + 2 HUD rows).
 *   Both refreshed on every SIGWINCH via screen_resize + the main
 *   loop's `scene->world.base_y = ph(rows) * 0.72f` recompute.
 */
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
/* §11  app — signals, resize, variable-dt main loop                      */
/* ===================================================================== */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Per-frame fps would jitter; this accumulates frame_count + elapsed
 * nanoseconds over a 500 ms window and emits a smoothed `display`
 * value each time the window fills.  Same shape as the FpsCounter on
 * every other file in this project.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;       /* 500 ms */
    f->frame_count++;
    f->window_ns += dt_ns;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value.
 *
 *   scene         simulation state (Robot + World + Camera + sim)
 *   screen        terminal cell extent + ncurses lifecycle
 *   fps           rolling-window fps estimator for HUD
 *   running       sig_atomic_t flag cleared by SIGINT / SIGTERM + 'q'
 *   need_resize   sig_atomic_t flag set by SIGWINCH; main reacts at
 *                 the top of the next iteration
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_handle_key — process one keystroke.
 *
 *   q / Q / ESC      quit
 *   SPACE / p        toggle paused
 *   r                reset (re-init scene, keep theme + speed + floor)
 *   f                cycle floor mode (FLAT ↔ PERLIN)
 *   n                reseed Perlin noise (new terrain layout)
 *   a                cycle speed level (launch impulse multiplier)
 */
static void app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27:
        app->running = 0;
        break;

    case ' ': case 'p': case 'P':
        s->sim.paused = !s->sim.paused;
        break;

    case 'r': case 'R':
        scene_reset(s, app->screen.rows);
        break;

    case 'f': case 'F':
        s->world.floor_mode =
            (FloorMode)((s->world.floor_mode + 1) % FLOOR_COUNT);
        break;

    case 'n': case 'N':
        noise_init((unsigned)clock_ns());
        break;

    case 'a': case 'A':
        s->sim.speed_level = (s->sim.speed_level + 1) % SPEED_LEVELS;
        break;

    default: break;
    }
}

/*
 * main — variable-dt render loop.
 *
 *   (1) atexit + signal handlers
 *   (2) seed Perlin noise; bring up Screen + Scene
 *   (3) loop:
 *       (a) handle pending SIGWINCH
 *       (b) measure dt (capped to prevent spiral-of-death)
 *       (c) drain non-blocking input
 *       (d) advance Scene (cam + FSM dispatch)
 *       (e) rolling-window fps counter
 *       (f) draw + present
 *       (g) frame cap: sleep so we don't burn the next slot's budget
 */
int main(void)
{
    /* (1) atexit + signal handlers */
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* (2) Perlin seed + Screen + Scene */
    noise_init((unsigned)(clock_ns() & 0xFFFFFFFF));

    App   *app   = &g_app;
    Scene *scene = &app->scene;
    app->running = 1;
    fps_counter_init(&app->fps);
    scene->world.floor_mode = FLOOR_PERLIN;   /* default to interesting terrain */

    screen_init(&app->screen);
    scene_init (scene, app->screen.rows);

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* (3a) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene->world.base_y = (float)ph(app->screen.rows) * 0.72f;
            app->need_resize = 0;
            last_ns = clock_ns();
        }

        /* (3b) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3c) drain input */
        int ch;
        while ((ch = getch()) != ERR) app_handle_key(app, ch);

        /* (3d) advance the Scene */
        scene_tick(scene, dt, app->screen.cols);

        /* (3e) rolling-window fps counter */
        fps_counter_tick(&app->fps, dt_ns);

        /* (3f) draw + present */
        scene_draw(scene, app->fps.display,
                   app->screen.cols, app->screen.rows);
        screen_present();

        /* (3g) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
