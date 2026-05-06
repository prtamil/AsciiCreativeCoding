# Pass 1 — moving_jump_spring_leg_robot.c: Hooke's law, projectile motion, three phases

## Core Idea

A pogo-stick robot crossing a 1D Perlin terrain by repeatedly compressing a spring leg, exploding upward, flying through the air as a projectile, landing, recovering, and compressing again. The whole simulation is a **3-state finite state machine** where each state has its own tick function:

- `compress_tick` — load potential energy into a spring (Hooke's law). When fully loaded, release → enter FLIGHT with launch velocity.
- `flight_tick` — projectile motion under gravity. When the body crosses the terrain, snap to the surface and enter LAND.
- `land_tick` — sit briefly stunned (so the eye can see the contact moment), then re-enter COMPRESS.

The physical units are explicit and verifiable: spring constant in *cells/s²*, gravity in *cells/s²*, fuse times in *seconds*. A learner can plug numbers in by hand and predict where the robot lands.

## The Mental Model

Imagine you're holding a spring in your hand, palm-up. You press down on it (compress) — the spring stores potential energy `½ k x²`. You let go — the spring expands, transferring that potential into kinetic energy of a launched ball: `½ m v²`. The ball flies up, slows, peaks, falls back down (gravity is constant `g`). It lands. You catch it, hold it for a moment, then press again.

Now make the spring a *leg* attached to a robot body, tilt the launch a bit (sloped takeoff matches the terrain slope), and have the robot move horizontally as it lands. That's the entire simulation.

The KEY teaching is energy conservation:
```
   PE_spring  =  KE_launch
   ½·k·x²     =  ½·m·v²              [m=1 in code units]
   v          =  x · sqrt(k/m)
```
Larger compression → more energy stored → faster launch → higher peak. Halve compression → halve launch speed → quarter the peak height (because peak height is `v²/(2g)` — quadratic in v).

## Data Structures

### Phase enum (§8.1)
```
COMPRESS   — leg loading; body sinks into the spring
FLIGHT     — projectile; gravity is the only force
LAND       — recovery; brief pause for visual readability
```

### Robot (§8.1)
```
Phase phase;                — which tick function runs this frame
float compression;          — current spring compression (0..MAX_COMPRESSION)
float fuse_t;               — phase elapsed time in seconds
float bx, by;               — body x, y in pixels
float vx, vy;               — velocity in pixels/sec  (FLIGHT only)
float launch_x, launch_y;   — captured at takeoff for trail base
Trail trail;                — fading dots of past body positions
```

### Trail
A short ring buffer (`TRAIL_CAP = 60`) of past body positions, drawn fading from bright to dim with the newest first.

## Key Formulas

```
COMPRESS                  (load the spring)
  compression += COMPRESS_SPEED · dt
  if compression >= MAX_COMPRESSION:
    angle = effective_launch_angle(slope)
    speed = compression · sqrt(SPRING_K)            ← v = x·√(k/m), m=1
    vx, vy = speed · cos(angle), -speed · sin(angle)  ← y axis points DOWN
    phase = FLIGHT

FLIGHT                    (projectile motion)
  vy += GRAVITY · dt
  bx += vx · dt
  by += vy · dt
  if by intersects terrain:
    snap to terrain surface; phase = LAND

LAND                      (brief pause)
  fuse_t += dt
  if fuse_t >= LAND_FUSE_SEC:
    phase = COMPRESS
    compression = 0

energy check:
  ½ · SPRING_K · MAX_COMPRESSION² ≈ ½ · 1 · launch_speed²
  ⇒ launch_speed = MAX_COMPRESSION · √SPRING_K
```

The "y axis points down" subtlety means UPWARD launch is `vy = -speed·sin(angle)`, NOT `+speed·sin(angle)`. Forgetting that sign flips gravity's role and makes the robot launch into the floor.

## The Main Loop (variable dt)

1. Resize check.
2. `dt` = wall-clock since last frame, capped at `DT_CAP_SEC`.
3. Drain input → `app_handle_key` (changes horizontal speed cycle, theme, pause).
4. `robot_tick(dt)` → dispatches to `compress_tick / flight_tick / land_tick` based on `phase`.
5. `cam_update(dt)` — right-edge follow camera scrolls when robot crosses 70% of screen width.
6. Trail push (every frame).
7. `scene_draw`: terrain → trail → spring/body → phase HUD bar (PE gauge in COMPRESS) → status HUD.
8. Frame cap.

## Non-Obvious Decisions

**Why three named per-phase functions instead of a switch?**
A switch in `robot_tick` would put 50+ lines of physics inline. Splitting them gives each phase its own narrative — `compress_tick` reads like "spring loading instructions," `flight_tick` reads like "projectile motion exam problem," `land_tick` reads like "recovery delay." The `robot_tick` orchestrator is then 12 lines that visibly mirror the FSM diagram.

**Why physical units (cells/sec, cells/sec²) instead of px/tick?**
Units that match a physics textbook let a learner verify by hand. "GRAVITY = 380 cells/s²; if launch_speed_y = 60 cells/s, peak height is `v²/(2g)` ≈ 4.7 cells" — that's checkable against the screen. With px/tick you'd first have to multiply by SIM_FPS to even reason about it.

**Why the brief LAND fuse?**
Without it, the robot transitions from FLIGHT to COMPRESS in one frame and the eye perceives no impact moment. A 0.18 sec stun is enough for the brain to register "thud, then re-load," which makes the gait *readable*. A real pogo stick has the same brief settle.

**Why is launch angle slope-adapted?**
Because launching straight up while standing on a 30° slope means the robot lands on the upslope before reaching forward speed (or, on a downslope, undershoots). Tilting the launch perpendicular to the local terrain mimics what a real spring-leg would do (the spring axis follows the leg axis). The function `effective_launch_angle(slope)` computes the perpendicular to a smoothed local slope.

**Why is `vy = -speed · sin(angle)` not `+`?**
Screen coordinates are y-down. "Up" is `y -= dy`. Calculate `vy` so positive launch angle gives upward motion: `-sin(angle)`. Same reason `dy` in atan2 calls flips sign through the codebase.

**Why is the camera right-edge follow?**
The robot moves rightward. Centring the camera leaves wasted screen on the right; left-anchoring leaves no anticipation room. Right-edge follow keeps the upcoming terrain visible while the robot occupies the trailing two-thirds — the same convention as side-scrolling video games.

**Why is the spring rendered as a coil between body and foot?**
Visualises the simulation state. As `compression` increases from 0 to MAX, the coil's number of visible loops grows; the eye sees energy being loaded. An invisible spring would make the COMPRESS phase look like "the robot pauses for a moment for no reason."

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| SPRING_K | 1200 cells/s² | Stiffness. Bigger = faster launch from same compression. |
| MAX_COMPRESSION | 1.6 cells | Cap on stored compression. |
| GRAVITY | 380 cells/s² | Constant downward accel during FLIGHT. |
| COMPRESS_FUSE_SEC | ~0.55 sec | Time to load (derived from COMPRESS_SPEED). |
| LAND_FUSE_SEC | 0.18 sec | Brief stun before next compress. |
| LAUNCH_ANGLE | 1.05 rad ≈ 60° | Default launch angle from horizontal. |
| TRAIL_CAP | 60 | Ring buffer; ~1 sec of trail. |

## Open Questions

- Energy loss on landing isn't modelled — every jump has the same peak. A `RESTITUTION` factor on the launch speed would mimic damping.
- The "horizontal speed cycle" key (`a`) is a discrete preset switch; a continuous slider would be a better physics-teaching control.
- A second body segment (head bobbing on a soft spring above the body) would visualise inertial separation and add flair.

---

# Pass 2 — Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config | frame rate, cell sizes, spring, launch geometry, gravity, fuses, terrain, camera, trail, ncurses pairs, HUD |
| §2 clock | monotonic timer + sleep |
| §3 color | terrain bands, body palette per phase, HUD pairs |
| §4 coords | pixel ↔ cell |
| §5 perlin | 1D Perlin noise + fBm for terrain heights |
| §6 terrain | per-column height sampler, slope sampler |
| §7 trail | ring-buffer push + iterate |
| §8 robot | (8.1) types (8.2) spring physics (8.3) effective angle (8.4) cam_update (8.5) compress (8.6) flight (8.7) land (8.8) tick orchestrator (8.9) init/reset |
| §9 render | (9.1) helpers (9.2) terrain (9.3) trail (9.4) spring (9.5) body (9.6) PE bar (9.7) HUD (9.8) scene_draw |
| §10 screen | ncurses init / present |
| §11 app | signals, resize, variable-dt main loop |

## Data Flow

```
keys → app_handle_key → robot.{paused, h_speed_idx, theme}
                            │
clock_ns → dt → robot_tick(r, dt)
                     ├── COMPRESS → compress_tick
                     │      compression += rate·dt
                     │      if full: launch (PE → KE), phase = FLIGHT
                     ├── FLIGHT → flight_tick
                     │      vy += g·dt; integrate; if hit terrain → LAND
                     └── LAND → land_tick
                            fuse_t += dt; if expired → COMPRESS
                            │
                            ▼
                       cam_update + trail_push
                            │
                            ▼
                       scene_draw
                          terrain → trail → spring/body → PE bar (COMPRESS) → HUD
```

## Pseudocode

```
setup:
  install signals
  screen_init, color_init
  perlin_init (seed)
  robot_init(cols, rows)

while running:
  if need_resize: re-init geometry preserving phase
  dt = clamp(now - last, 0, DT_CAP_SEC)
  drain input → app_handle_key
  robot_tick(r, dt)
  cam_update(r, dt)
  trail_push(r)
  fps update
  erase
  scene_draw
  wnoutrefresh + doupdate
  sleep to TARGET_FPS
```

## Key Patterns to Internalize

**Per-phase tick function = one concept per function.** Don't switch; split. Each phase becomes its own narrative; the orchestrator becomes a 12-line FSM diagram in code.

**Energy conservation as the launch equation.** `½kx² = ½mv²` is the cleanest possible derivation of `v = x·√(k/m)`. No empirical tuning — physics dictates the launch speed.

**Sign of vy in y-down coordinates.** Launch UP = `vy = -speed·sin(angle)`. Forget the minus and the robot launches into the floor.

**Brief LAND fuse for visual readability.** Frames are the unit of perception. Even a "physically instant" transition needs ≥2 frames (ideally 6+) to be legible.

**Slope-perpendicular launch.** The leg axis follows local terrain — without this, slopes break the gait. `effective_launch_angle(slope)` is the bridge.

**Ring-buffer trail with newest-first iteration.** Same idiom as `diff_drive_robot.c`. Recognise it everywhere.
