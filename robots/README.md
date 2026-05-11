# robots — locomotion and control: wheels, legs, springs, balance

A reference for the **embodied-locomotion backbone** of the project. This
folder contains **4 self-contained C programs** that all answer the same
question:

> **Given the body's current state and the world it sits on, what command
> moves it forward this tick?**

Every robot in this folder is a **body + a controller**. The body is a
rigid or articulated mechanism (two wheels, one spring leg, two legs, an
inverted pendulum). The controller is the **decision rule** that turns
sensor input + a goal into per-tick actuator commands — wheel velocities,
spring release, leg-swing phase, motor torque. The folder ranges from the
simplest possible controller (key-driven wheel speeds) to a full PID loop
stabilising an inverted pendulum on procedural Perlin terrain.

If you read **only one file**, read
[`diff_drive_robot.c`](diff_drive_robot.c) — two wheels, no steering wheel,
and the cleanest possible expression of *forward kinematics drives motion*.
Everything else in the folder is downstream of it.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive — body state + control loop](#the-unifying-primitive--body-state--control-loop)
3. [The four locomotion modes](#the-four-locomotion-modes)
4. [Control-loop anatomy](#control-loop-anatomy)
5. [File index](#file-index)
6. [Building and running](#building-and-running)
7. [Adding a new robot](#adding-a-new-robot)
8. [Cross-folder pointers](#cross-folder-pointers)

---

## How to read this folder

```
   1. diff_drive_robot.c            ← two wheels, FK only, key-driven commands
              │
              ▼
   2. walking_robot.c               ← bipedal legs: FK during swing, IK during stance
              │
              ▼
   3. moving_jump_spring_leg_robot.c ← pogo stick: 3-phase FSM (LOAD / FLIGHT / LAND)
              │                       + Perlin terrain + parabolic projectile arc
              ▼
   4. perlin_terrain_bot.c          ← inverted pendulum + PID stabilisation
                                    + cart-pole coupling + Perlin slopes
```

**Prerequisites.** Each file's header carries *Study alongside* pointers
into the `animation/` folder for the kinematic chains it uses. The
recommended reading order doubles as a **complexity ramp**:

* `diff_drive_robot.c` — open-loop FK. The controller is *you* (the user).
* `walking_robot.c` — closed-loop kinematics. FK swings the airborne leg
  forward; analytical IK keeps the planted foot pinned to the ground while
  the body advances. Foot-lock is the *only* state — no FSM.
* `moving_jump_spring_leg_robot.c` — a finite-state machine over physics.
  Each state has its own integrator: spring (Hookean restoring force),
  projectile (gravity-only ballistic), contact (zero-velocity clamp).
* `perlin_terrain_bot.c` — closed-loop *dynamic* control. The body is
  intrinsically unstable; only a continuous feedback signal (the PID
  controller) keeps it upright.

Each step adds **one new control concept**: kinematics → contact constraint
→ phase machine → continuous feedback.

---

## The unifying primitive — body state + control loop

Every file in this folder factors into the same shape:

```c
typedef struct {
    /* world-space pose */
    float x, y;            /* body centre in pixel space */
    float heading;         /* radians */

    /* body state — what the body IS doing right now */
    float v, omega;        /* linear + angular velocity (diff-drive)        */
    float spring_e;        /* stored potential energy   (pogo)              */
    float theta, dtheta;   /* lean angle + rate         (inverted pendulum) */
    GaitPhase phase;       /* SWING / STANCE             (biped)            */

    /* controller state — what the body WANTS to do */
    float v_cmd, w_cmd;    /* commanded vel + ω         (diff-drive)        */
    float setpoint;        /* desired lean angle        (PID bot)           */
    float pid_i, pid_d;    /* PID integrator + last-error                   */
} Robot;
```

The per-frame loop in every file follows the same outline:

```
# 1. SENSE — read world / state
read terrain height under wheels
measure body's lean angle
check whether foot is in contact

# 2. CONTROL — turn (sensor + goal) into actuator command
v_cmd = throttle_input                  # diff-drive (open loop)
torque = Kp·θ_err + Ki·∫θ_err + Kd·θ̇   # PID bot (closed loop)
phase = (phase_timer > GAIT_DURATION)   # walker / pogo (state machine)
        ? next_phase : phase

# 3. INTEGRATE — apply Newton / kinematics for dt
x += v · cos(θ) · dt
y += v · sin(θ) · dt
θ += omega · dt
... or Verlet, or spring physics, or projectile motion

# 4. CONSTRAIN — enforce contact / joint limits
clamp foot to ground when in STANCE
prevent body from penetrating terrain
re-pin anchor points

# 5. RENDER — draw the body in its new pose
```

**Every file is some version of this five-step loop.** What differs across
the four files is *which step does the heavy lifting*:

* `diff_drive_robot.c` — step 2 is trivial (user keys); step 3 dominates.
* `walking_robot.c` — step 4 dominates (foot-lock IK every tick).
* `moving_jump_spring_leg_robot.c` — step 2 is a state machine; step 3 has
  three different integrators picked by phase.
* `perlin_terrain_bot.c` — step 2 *is* the file (PID); step 3 simulates a
  cart-pole pair via leapfrog at 240 Hz internal sub-stepping.

---

## The four locomotion modes

```
                ┌─────────────────────────────────────────────────────┐
                │              body sits on the world                  │
                └───────┬──────────┬──────────┬──────────┬─────────────┘
                        │          │          │          │
                        ▼          ▼          ▼          ▼
                   WHEELS       LEGS       SPRING     PENDULUM
                  (rolling)   (stepping)  (hopping)   (balancing)
                        │          │          │          │
              ┌─────────┘    ┌─────┘    ┌─────┘    ┌─────┘
              │              │          │          │
         diff_drive     walking      pogo        terrain_bot
         (open-loop     (FK swing +  (3-phase    (PID closed
          FK)            IK stance)   FSM)        loop)
```

Each mode is a **different answer** to *how does the body push against the
ground to move?*:

| Mode      | Push mechanism                       | Energy source              | Failure mode if uncontrolled       |
|-----------|--------------------------------------|----------------------------|-------------------------------------|
| Wheels    | wheel-ground friction                | wheel motors               | nothing — open-loop stable          |
| Legs      | swing foot forward, plant, push off  | hip & knee motors          | foot drift / penetration            |
| Spring    | compress spring, release downward    | stored elastic potential   | rolls / bounces uncontrollably      |
| Pendulum  | tilt body to make wheels accelerate  | wheel motors               | tips over in milliseconds (unstable) |

The progression is **mechanically harder** in that order. A diff-drive
robot rolls on its own — no controller needed for stability, only for
*direction*. A walking robot needs contact constraints. A pogo stick needs
a state machine. An inverted pendulum needs *continuous* feedback or it
falls over instantly.

---

## Control-loop anatomy

The four files form a progression through controller complexity:

**1. Open-loop forward kinematics** (`diff_drive_robot.c`).
   Controller = user's keystrokes. Wheel speeds map to body motion through
   pure FK (`v = (vL + vR)/2`, `ω = (vR − vL)/axle`). No feedback — the
   robot goes exactly where the wheels say it goes.

**2. FK + analytical IK alternation** (`walking_robot.c`).
   Controller = a single `phase` accumulator that drives a sinusoidal gait.
   FK places the swinging foot (angle → position); IK pins the stance foot
   (target → angles, law of cosines) so the body can advance without the
   planted foot sliding. The phase variable IS the controller.

**3. Finite-state machine over physics** (`moving_jump_spring_leg_robot.c`).
   Controller = three phases (COMPRESS, FLIGHT, CONTACT). Each phase runs
   a different integrator:
   ```
   COMPRESS : spring_e += LOAD_RATE · dt        ; body sinks
   FLIGHT   : pos += vel · dt ; vel.y += g · dt ; projectile motion
   CONTACT  : vel = 0 ; body locked             ; brief landing flash
   ```
   Transitions fire on `spring_e == FULL`, ground contact, and timer
   expiry. Real-world analogue: a pogo stick or a flea.

**4. PID closed-loop control** (`perlin_terrain_bot.c`).
   Controller = three-term feedback on the body's lean angle:
   ```
   error  = setpoint − θ
   pid_i += error · dt
   pid_d  = (error − last_error) / dt
   torque = Kp · error + Ki · pid_i + Kd · pid_d
   ```
   Press `p` to disable the PID and watch the bot fall — the file *teaches*
   the controller by letting you turn it off. The setpoint adapts on
   slopes so the bot leans into hills rather than fighting gravity.

The four files together form a **mini-textbook on robot control**: from
"the user IS the controller" to "the controller runs at 240 Hz internally
and the math is on the screen with substituted live values."

---

## File index

| File                              | Description                                                              | Controller       | Body model                  |
|-----------------------------------|--------------------------------------------------------------------------|------------------|------------------------------|
| `diff_drive_robot.c`              | Two-wheeled robot with no steering wheel — left/right wheel speeds steer  | open-loop FK     | rigid body + 2 wheels        |
| `walking_robot.c`                 | Bipedal stick-figure walks forward/backward with FK swing + IK stance     | phase accumulator | 2-link legs × 2 + body       |
| `moving_jump_spring_leg_robot.c`  | Pogo-stick robot hops across Perlin terrain (load / fly / land)           | 3-phase FSM      | body + spring leg            |
| `perlin_terrain_bot.c`            | Self-balancing wheel-bot crossing Perlin slopes (Segway-style)            | PID closed loop  | inverted pendulum + 2 wheels |

---

## Building and running

Every file is self-contained — no shared headers, single `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra <path>.c -o <name> -lncurses -lm
```

**Universal keys** (present in every file):

| Key             | Action                  |
|-----------------|-------------------------|
| `q` / `Q` / `ESC` | quit                  |
| `space`         | pause / resume          |
| `r`             | reset                   |
| arrows          | drive / steer (file-specific) |

**Per-file extras:**

| File                              | Notable keys                                                     |
|-----------------------------------|------------------------------------------------------------------|
| `diff_drive_robot.c`              | `W/S` throttle ± ; `A/D` turn ; `Z/E` spin in place              |
| `walking_robot.c`                 | `+/-` pace ; `r` reverse ; `.` step frame ; `g` toggle ground    |
| `moving_jump_spring_leg_robot.c`  | `f` cycle FLAT/PERLIN floor ; `n` new seed ; `a` speed preset    |
| `perlin_terrain_bot.c`            | `p` toggle PID (watch it fall) ; `m` cycle TELEMETRY/EQUATIONS/PHASE ; `g` gain preset ; `+/-` Kp |

---

## Adding a new robot

1. **Pick a locomotion mode.** Rolling, stepping, hopping, balancing, or
   something genuinely new (slithering? swimming? climbing?).
2. **Pick a controller.** Open-loop (user input), phase accumulator,
   finite-state machine, or closed-loop PID. The choice is dictated by
   stability: if the body falls over without intervention, you need
   closed-loop. If the body just sits there, open-loop is enough.
3. **Copy the closest existing file** as a template:
   * Rolling, single body → `diff_drive_robot.c`
   * Articulated legs → `walking_robot.c`
   * Spring / projectile cycle → `moving_jump_spring_leg_robot.c`
   * Unstable body needing feedback → `perlin_terrain_bot.c`
4. **Replace the §6 body and the §7 control step.** Everything else
   (clock, color, terrain noise, scene, screen, app, signals) carries over
   unchanged. Variable-dt loop with internal sub-stepping for stiff
   physics — copy the pattern from `perlin_terrain_bot.c`.
5. **Add CONCEPTS + MENTAL MODEL blocks** per the project's
   [CLAUDE.md](../CLAUDE.md) template, and explicitly state the controller
   class (open-loop / FSM / PID / etc.) in the *Algorithm* subsection.
6. **Verify**: `gcc -Wall -Wextra` clean, stable 60 fps, `q` / `ESC`
   exits cleanly, `SIGWINCH` doesn't crash, HUD shows fps + body state
   (position, lean, phase — whichever scalars define the robot's pose).

---

## Cross-folder pointers

* **[`animation/`](../animation/)** supplies the kinematic chains these
  robots are built from. `walking_robot.c` is `animation/ik_helloworld.c`
  (2-link IK for the stance leg) paired with `animation/fk_helloworld.c`
  (FK for the swing leg), alternating per gait phase. `animation/hexpod_tripod.c`
  is the 6-leg generalisation of `walking_robot.c`.
* **[`flocking/`](../flocking/)** is the *N-agent* counterpart of this
  folder — many simple bodies running cheap rules. The robots folder is
  *one rich body* running an expensive controller. Same per-tick
  fixed-step accumulator, opposite end of the agent/complexity tradeoff.
* **[`grids/`](../grids/)** is the discrete-coordinate counterpart;
  robots live in continuous pixel space (`float px, py`) so wheel
  kinematics, projectile arcs, and PID error signals stay
  resolution-independent.
* Perlin terrain in `moving_jump_spring_leg_robot.c` and
  `perlin_terrain_bot.c` is the same 1-D value-noise primitive used in
  `particle_systems/flowfield.c` — height-field consumers of the same
  generator.
