# Pass 1 — flocking.c: ASCII terminal simulation of three flocks with five switchable steering algorithms

## Core Idea

Three groups of ASCII characters move around the terminal screen. Each group has a single leader that wanders on its own smooth random path, and a set of followers that steer toward each other and their leader according to one of five selectable algorithms. The physics runs in a high-resolution sub-pixel space so diagonal motion looks proportional on a terminal whose cells are taller than wide.

## The Mental Model

Imagine three herds of birds, each a different color, roaming across a space that wraps around like a globe — if you fly off the right edge you reappear on the left. Each herd has a guide bird that just meanders randomly but smoothly. The rest of the herd watches the guide and each other, adjusting their direction every 60th of a second.

The five herd behaviors you can switch between are:

1. **Classic Boids**: Each bird tries to stay away from birds right next to it, tries to match the average heading of birds nearby, and tries to drift toward the center of the local group — three forces balanced against each other produce realistic murmurating behavior.
2. **Leader Chase**: Every bird ignores the others and just points itself directly at the leader. Produces a comet-tail streaming effect.
3. **Vicsek**: Every bird averages the headings of all nearby birds including the leader, then wobbles by a random angle. Low wobble produces perfect streaming; high wobble produces chaos. This is a famous model of a phase transition.
4. **Orbit Formation**: Each follower is assigned a slot on a rotating ring centered on its leader. The ring spins, so every bird chases a moving point on a circle — produces a spinning halo.
5. **Predator-Prey**: Flock 0 becomes a predator whose leader hunts the nearest bird of any other flock. Flocks 1 and 2 run away when the predator gets close, while still maintaining their own group cohesion.

The entire terminal surface is conceptually much larger than the visible cells — each cell is divided into 8×16 sub-pixels so that a boid moving diagonally at 45° travels the same physical distance as one moving horizontally or vertically.

## Data Structures

### Boid
```
px, py          — position in sub-pixel space (float)
vx, vy          — velocity in pixels per second (float)
cruise_speed    — this individual boid's personal target speed
```
Every boid lives entirely in pixel-space coordinates. Only at draw time is position divided by CELL_W or CELL_H to get the terminal cell coordinate.

### Flock
```
leader          — one Boid that wanders independently
followers[]     — up to FOLLOWERS_MAX Boid slots
n               — how many followers are active
color_phase     — which flock index this is (0, 1, or 2)
wander_angle    — leader's current heading in radians
orbit_phase     — for MODE_ORBIT: current rotation angle of the ring
```
A Flock is self-contained. The Scene holds an array of three Flocks.

### Scene
```
flocks[FLOCKS]  — the three Flock structs
mode            — which of the five algorithms is active
paused          — if true, scene_tick does nothing
vicsek_noise    — the noise level parameter for MODE_VICSEK
```

## The Main Loop

Each iteration of the main while loop:

1. Check if a terminal resize signal arrived. If so, rebuild the pixel-space dimensions.
2. Measure how much real time has passed since the last iteration (dt).
3. Cap dt at 100ms so a long pause (e.g. terminal minimized) does not cause a physics explosion.
4. Add dt to the accumulator. While the accumulator holds enough time for a full physics tick (1/60 second), run scene_tick once and subtract one tick's worth of time.
5. Compute alpha = leftover accumulator time / one tick. This is how far through the next (not yet computed) tick we are. Used to project boid positions forward for smoother drawing.
6. Update the fps display counter.
7. Sleep just long enough to cap the frame rate at 60fps.
8. Draw all boids at their interpolated positions into ncurses' internal buffer.
9. Call doupdate() to push the buffer to the terminal.
10. Poll for a keypress and handle it.

### Inside scene_tick:
For all non-predator modes: call flock_tick on each of the three flocks.

Inside flock_tick:
- Advance the leader one step via its wander_angle random walk.
- For orbit mode, advance orbit_phase by ORBIT_SPEED * dt.
- Stage 1 (read-only): compute each follower's desired new velocity by calling the appropriate steering function. All reads use the old positions.
- Stage 2 (write): apply the new velocities, clamp speed to [MIN_SPEED, MAX_SPEED], integrate position (px += vx * dt), wrap at edges.

## Non-Obvious Decisions

### Sub-pixel space (CELL_W=8, CELL_H=16)
Terminal cells are physically about twice as tall as wide. If physics used cell coordinates directly, a boid moving at 45° would travel twice as fast vertically as horizontally in physical distance. The fix: define a sub-pixel space where each cell spans 8×16 units. Physics uses pixel units. Drawing divides by CELL_W or CELL_H respectively. Now 45° motion is genuinely isotropic.

### Toroidal wrap instead of bouncing
Bouncing walls create corners where boids pile up and get confused — they suddenly have no neighbors on one side. A toroidal (wrap-around) topology means every boid always has potential neighbors in every direction. This keeps flocking behavior uniform everywhere on screen.

### Toroidal distance for neighbor detection
When computing whether boid A can see boid B, naive subtraction of positions fails near the edges. A boid at pixel 5 and one at pixel 995 on a 1000-pixel-wide field are actually 10 pixels apart (via the wrap), not 990. The function `toroidal_delta` always returns the shorter of the two paths, with correct sign.

### Two-stage update (simultaneous, not sequential)
If boid 0 moves first and then boid 1 steers based on boid 0's new position, the update order biases the flock. The fix: read phase (compute all new velocities from old positions) followed by write phase (apply all velocities). Every boid reacts to the same snapshot.

### Why the three Reynolds rules work
- **Separation** (weight 1.8): prevents boids from occupying the same terminal cell. Without it, the whole flock collapses to a single point.
- **Alignment** (weight 1.0): produces the streaming, parallel-flight look that makes flocking recognizable. Without it, birds just cluster randomly.
- **Cohesion** (weight 0.5): weakest rule — keeps the group loosely together so it doesn't permanently scatter. Weak enough not to collapse the group.
- **Leader pull** (weight 0.4): a constant attraction toward the flock's own leader. This is not part of the original Reynolds model but is added here so the three flocks remain distinct over time rather than merging.

### Vicsek phase transition
The Vicsek model (1995) has a single tunable parameter: noise angle. At low noise, the average heading signal dominates and all birds align into a single streaming direction — ordered phase. At high noise, the random perturbation overwhelms the alignment signal — disordered phase. There is a sharp transition between these, which is why 'n' and 'm' produce dramatically different behavior.

### Orbit ring speed check
The ring tangential speed is ORBIT_RADIUS × ORBIT_SPEED = 120 × 1.4 = 168 px/s. Boid speed is 280 px/s. Since followers are faster than the moving slot they chase, they can always catch up, preventing permanent lag behind the ring.

### Rejection-sample unit vector for spawn
To spawn a boid with a uniformly random direction without calling atan2/sin/cos, the code picks random (dx,dy) from [-1,1]², rejects if the point falls outside the unit circle, then normalizes. This uniformly samples the unit circle without the poles-of-latitude bias that would come from picking a random angle.

### Cruise speed variation (±15%)
Each follower's personal cruise_speed is set at spawn to BOID_SPEED × [0.85, 1.15]. Fast boids naturally push toward the front of the group; slow ones drift toward the back. This produces an organic depth gradient visible as a spread-out flock rather than a perfect blob.

### floorf + 0.5f instead of roundf
When converting pixel position to terminal cell, `roundf` uses banker's rounding (round half to even). A boid sitting exactly on a cell boundary would oscillate between two cells every frame, flickering. `floorf(x + 0.5f)` always rounds the half-case in the same direction, eliminating flicker.

## State Machines

The mode selector is a simple enum. No state transitions happen automatically — mode only changes on keypress.

```
MODE_BOIDS ──[key '1']──> same
MODE_CHASE ──[key '2']──> same
VICSEK     ──[key '3']──> same
ORBIT      ──[key '4']──> same
PREDATOR   ──[key '5']──> same

Any mode ──[key '1'..'5']──> corresponding mode
```

Predator-prey has internal leader behavior:

```
Predator leader:
  [no prey in PREDATOR_CHASE_RADIUS] → WANDER (random jitter to wander_angle)
  [prey enters PREDATOR_CHASE_RADIUS] → HUNT (snap wander_angle to bearing of nearest prey)

Prey followers:
  [no predator in PREY_FLEE_RADIUS] → normal boids rules
  [predator enters PREY_FLEE_RADIUS] → boids rules + flee impulse away from predator
```

## From the Source

**Algorithm:** Five switchable modes sharing one code path: Classic Boids (Reynolds 1987), Leader Chase, Vicsek (1995), Orbit Formation, Predator-Prey. Vicsek order parameter φ = |Σv̂_i|/N ∈ [0,1]; 0 = disordered, 1 = perfect alignment. In Vicsek steer, `self` is always included (sum starts with `sum_vx = b->vx, count=1`) before scanning neighbours.

**Math:** Cosine palette: `r,g,b = 0.5 + 0.5·cos(2π(t + phase))` cycles hue slowly while keeping three flocks visually distinct. Orbit tangential speed = `ORBIT_RADIUS × ORBIT_SPEED = 120 × 1.4 = 168 px/s`; boid max speed 280 px/s — followers always fast enough to catch their moving slot. Rejection-sample unit vector at spawn: pick `(dx,dy)` from `[-1,1]²`, reject if outside unit circle, normalize — avoids latitude bias from angle-based sampling.

**Physics:** All boids in isotropic pixel space: `CELL_W=8` sub-pixels wide, `CELL_H=16` sub-pixels tall per terminal cell. Two-stage update: Stage 1 reads all old positions to compute new velocities; Stage 2 writes all velocities simultaneously — prevents update-order bias. Toroidal wrap, not bounce: every boid always has potential neighbours in all directions.

**Performance:** `floorf(x + 0.5f)` instead of `roundf` for pixel→cell conversion — breaks the round-half-even tie to prevent per-frame cell oscillation flicker. Render interpolation: `draw_px = px + vx * alpha * dt_sec` projects each boid forward to its fractional-tick position.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| PERCEPTION_RADIUS | 180 px | Boids see more neighbors; flock becomes more cohesive, can feel "sticky" |
| SEPARATION_RADIUS | 60 px | Wider personal space; flock spreads out more, less dense core |
| W_SEPARATION | 1.8 | Stronger push-apart; flock becomes looser, birds stay further apart |
| W_ALIGNMENT | 1.0 | Stronger direction-matching; flock lines up more rigidly |
| W_COHESION | 0.5 | Stronger center-pull; flock clumps more tightly, can collapse to a point |
| W_LEADER_PULL | 0.4 | Stronger leader gravity; followers trail leader more closely |
| WANDER_JITTER | 0.10 rad | Larger random heading changes; leader path becomes more erratic/zigzaggy |
| VICSEK_NOISE | 0.30 rad | Higher noise; flock transitions from ordered streaming toward chaos |
| ORBIT_RADIUS | 120 px | Larger ring; followers orbit in a wider circle around leader |
| ORBIT_SPEED | 1.4 rad/s | Faster ring rotation; followers need to move faster to keep their slot |
| MAX_STEER | 130 px/s | Higher: boids turn sharper and faster; lower: boids curve more gently |
| PREDATOR_CHASE_RADIUS | 280 px | Larger: predator detects and chases prey from further away |
| PREY_FLEE_RADIUS | 160 px | Larger: prey start running earlier, before the predator is close |
| W_FLEE | 3.0 | Higher: prey break formation faster and scatter more dramatically |
| FOLLOWERS_DEFAULT | 12 | More followers: denser flock, heavier O(n²) computation |
| CELL_W | 8 | Changing breaks aspect ratio correction; do not change independently of CELL_H |

## Open Questions for Pass 3

- How does the two-stage update interact with the predator-prey mode? The predator followers use chase_steer which reads prey positions — but prey have not moved yet in that tick. Is this intentional or an ordering artifact?
- Why does vicsek_steer include self in the average sum (sum_vx = b->vx, count=1 before looping)? What happens to the model if you start count at 0?
- The leader pull force is always on in MODE_BOIDS. What happens if you set W_LEADER_PULL to 0.0? Does the flock still stay near the leader via cohesion alone?
- follower_brightness uses toroidal distance but the brightness threshold (0.35 × PERCEPTION_RADIUS) is fixed. What would it look like to scale brightness continuously with distance rather than binary bold/normal?
- The orbit mode uses each follower's array index (idx) to assign its ring slot. What happens if you add or remove followers at runtime — do the slots redistribute smoothly or do boids jump to new positions?

---

# Pass 2 — flocking.c: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | All #define and enum constants — speeds, radii, weights, timing |
| §2 clock | Monotonic nanosecond timer and nanosleep wrapper |
| §3 color | Initialize 7 ncurses color pairs (3 follower + 3 leader + 1 HUD) |
| §4 coords | Sub-pixel ↔ terminal cell conversion; the aspect ratio fix |
| §5 boid | Boid struct; velocity-to-glyph mapping; spawn; speed clamp; wrap; toroidal delta |
| §6 flock | All 5 steering algorithms; leader wander; flock_tick two-stage update; predator-prey special tick |
| §7 scene | Pool of 3 flocks; scene_tick dispatcher; draw with render interpolation |
| §8 screen | ncurses init, draw (scene + HUD), doupdate |
| §9 app | dt accumulator loop, keypress handling, resize signal, program entry |

## Data Flow Diagram

```
INIT:
  terminal size → pixel dimensions (cols*CELL_W, rows*CELL_H)
  pixel dims → flock_init × 3 → Boid positions/velocities in pixel space

EACH FRAME:
  wall clock ──────────────────────────────────┐
       │                                        │
       ↓                                        ↓
  dt (ns) → sim accumulator             alpha = leftover / tick_ns
       │                                        │
       ↓                                        │
  [while accum >= tick_ns]:                     │
    scene_tick(dt_sec)                          │
    ├─ for each Flock:                          │
    │    leader_tick → wander_angle → vx,vy    │
    │    Stage1: steering_fn reads old pos      │
    │    Stage2: write new vx,vy; pos += v*dt   │
    │    boid_wrap (toroidal)                   │
    accum -= tick_ns                            │
                                                │
  draw_boid(pos + v*alpha*dt_sec) ←────────────┘
       │
       ↓ (pixel pos / CELL_W,CELL_H → col,row)
  ncurses cell → mvwaddch(glyph, color_attr)
       │
       ↓
  doupdate() → terminal display
```

## Function Breakdown

### clock_ns() → int64_t nanoseconds
Purpose: read CLOCK_MONOTONIC as a single 64-bit nanosecond count.
Steps:
  1. Call clock_gettime(CLOCK_MONOTONIC, &t).
  2. Return t.tv_sec * 1e9 + t.tv_nsec.

### clock_sleep_ns(ns)
Purpose: sleep for exactly ns nanoseconds using nanosleep.
Steps:
  1. If ns <= 0, return immediately.
  2. Pack into struct timespec and call nanosleep.

### color_init()
Purpose: register 7 ncurses color pairs for 3 flocks + HUD.
Steps:
  1. start_color().
  2. If 256 colors available: use xterm-256 indices for vivid green/orange/blue followers and yellow/cyan/white leaders.
  3. Else: fall back to 16-color approximations.
  4. Register HUD pair as white-on-black.

### pw(cols) → int, ph(rows) → int
Purpose: convert terminal cell counts to sub-pixel dimensions.
Steps:
  1. Return cols * CELL_W (or rows * CELL_H).
Edge case: none — purely multiplicative.

### px_to_cell_x(px) → int, px_to_cell_y(py) → int
Purpose: convert sub-pixel coordinate to terminal cell index.
Steps:
  1. Return floor(px / CELL_W + 0.5) — the +0.5 breaks ties to prevent per-frame oscillation.

### velocity_dir_char(vx, vy, flock_idx) → char
Purpose: pick an ASCII glyph that visually shows the boid's direction.
Steps:
  1. Compute angle = atan2(vy, vx).
  2. Shift angle to [0, 2π] by adding π/8 to center the sectors.
  3. Divide shifted angle by π/4 to get integer sector 0–7.
  4. Index into flock-specific 8-char table.
Edge case: each flock uses different diagonal chars so mixed flocks stay visually distinguishable.

### boid_spawn_at(b, px, py, speed)
Purpose: place a boid at a position with a random direction.
Steps:
  1. Set px, py, cruise_speed.
  2. Pick (dx,dy) uniformly at random from [-1,1]².
  3. Reject and repick if the point is outside the unit circle or nearly zero.
  4. Normalize to unit vector, scale by speed → vx, vy.

### boid_clamp_speed(b)
Purpose: keep |velocity| in [MIN_SPEED, MAX_SPEED].
Steps:
  1. Compute mag = hypot(vx, vy).
  2. If mag < 0.001: nudge forward (vx=MIN_SPEED, vy=0).
  3. If mag < MIN_SPEED: scale up to MIN_SPEED maintaining direction.
  4. If mag > MAX_SPEED: scale down to MAX_SPEED maintaining direction.

### boid_wrap(b, max_px, max_py)
Purpose: toroidal boundary — teleport boid to opposite edge when it exits.
Steps:
  1. If px < 0: px += max_px.
  2. If px >= max_px: px -= max_px.
  3. Same for py.

### toroidal_delta(a, b, extent) → float
Purpose: shortest signed displacement from a to b on a wrap-around axis.
Steps:
  1. d = b - a.
  2. If d > extent/2: d -= extent.
  3. If d < -extent/2: d += extent.
  4. Return d.
Edge case: ensures boids near opposite edges see each other as close neighbors.

### leader_tick(flock, dt, max_px, max_py)
Purpose: advance the leader one physics step via smooth random heading walk.
Steps:
  1. Add random jitter in [-WANDER_JITTER, +WANDER_JITTER] to wander_angle.
  2. Set vx = LEADER_SPEED * cos(wander_angle), vy = LEADER_SPEED * sin(wander_angle).
  3. px += vx * dt, py += vy * dt.
  4. boid_wrap.

### boids_steer(flock, idx, max_px, max_py) → (new_vx, new_vy)
Purpose: compute Reynolds boids steering for followers[idx].
Steps:
  1. Initialize accumulators: sep_x/y, ali_vx/vy, coh_dx/dy, neighbor_n, sep_n.
  2. For each other follower j:
     a. Compute toroidal (dx, dy) from b to followers[j].
     b. dist = hypot(dx, dy). Skip if dist >= PERCEPTION_RADIUS.
     c. Accumulate ali_vx/vy += followers[j].vx/vy.
     d. Accumulate coh_dx/dy += dx, dy (toroidal offset, not absolute pos).
     e. If dist < SEPARATION_RADIUS: accumulate repulsion scaled by closeness.
     f. Increment neighbor_n (and sep_n if in separation zone).
  3. Compute steer from separation: normalize sep vector, scale by W_SEPARATION * MAX_STEER.
  4. Compute steer from alignment: normalize average velocity, scale by personal cruise_speed, multiply (desired - current) by W_ALIGNMENT.
  5. Compute steer from cohesion: normalize average toroidal offset, scale by cruise_speed, multiply (desired - current) by W_COHESION.
  6. Compute steer from leader pull: toroidal delta to leader, normalize, scale by W_LEADER_PULL * MAX_STEER.
  7. clamp2d total steer to MAX_STEER.
  8. Return b.vx + steer_x, b.vy + steer_y.
Edge case: skip self (j == idx). Handle zero-distance neighbors gracefully.

### chase_steer(flock, idx, ...) → (new_vx, new_vy)
Purpose: home directly on the leader; add only separation from peers.
Steps:
  1. Compute toroidal delta to leader, normalize, scale to cruise_speed → desired velocity.
  2. steer = desired - current.
  3. For each other follower within SEPARATION_RADIUS: add repulsion vector.
  4. Clamp and return.

### vicsek_steer(flock, idx, max_px, max_py, noise_scale) → (new_vx, new_vy)
Purpose: align to average heading of neighbors plus random noise.
Steps:
  1. Start with self: sum_vx = b.vx, sum_vy = b.vy, count = 1.
  2. If leader is within PERCEPTION_RADIUS: add leader velocity, count++.
  3. For each follower within PERCEPTION_RADIUS: add velocity, count++.
  4. avg_angle = atan2(sum_vy/count, sum_vx/count).
  5. noise = random in [-noise_scale, +noise_scale].
  6. new_angle = avg_angle + noise.
  7. Return (cos(new_angle) * cruise_speed, sin(new_angle) * cruise_speed).

### orbit_steer(flock, idx, ...) → (new_vx, new_vy)
Purpose: chase assigned slot on rotating ring around leader.
Steps:
  1. slot_angle = orbit_phase + 2π * idx / n.
  2. target = leader.pos + ORBIT_RADIUS * (cos(slot_angle), sin(slot_angle)).
  3. toroidal delta to target → desired velocity at cruise_speed.
  4. steer = desired - current.
  5. Add separation from other followers within SEPARATION_RADIUS.
  6. Clamp and return.

### flock_tick(flock, mode, vicsek_noise, dt, max_px, max_py)
Purpose: advance one flock one physics step.
Steps:
  1. leader_tick (or predator_leader_tick in predator mode).
  2. If MODE_ORBIT: orbit_phase += ORBIT_SPEED * dt.
  3. Stage 1: for each follower i, call the appropriate steering function, store result in new_vx[i], new_vy[i].
  4. Stage 2: for each follower i, set vx/vy = new_vx/vy[i], clamp speed, integrate position, wrap.

### predator_leader_tick(predator, prey_flocks, n_prey, dt, ...)
Purpose: predator leader hunts nearest prey or wanders if none in range.
Steps:
  1. Search all followers and leaders of all prey flocks for the one with minimum toroidal distance.
  2. If closest prey is within PREDATOR_CHASE_RADIUS: set wander_angle = atan2(dy, dx) toward prey.
  3. Else: add small random jitter to wander_angle (normal wander).
  4. Compute velocity from wander_angle, integrate, wrap.

### prey_boids_steer(flock, idx, predator, ...) → (new_vx, new_vy)
Purpose: normal boids rules plus flee from nearby predator boids.
Steps:
  1. Call boids_steer — writes out_vx, out_vy.
  2. For predator leader and each predator follower:
     a. If within PREY_FLEE_RADIUS: accumulate flee_x/y as direction away from predator.
  3. If flee vector is nonzero: add (flee / |flee|) * W_FLEE * MAX_STEER to out_vx/vy.

### scene_tick(scene, dt, cols, rows)
Purpose: advance all physics one step.
Steps:
  1. If paused: return.
  2. Compute max_px = pw(cols), max_py = ph(rows).
  3. If mode == PREDATOR: call scene_tick_predator and return.
  4. Else: for each flock, call flock_tick.

### follower_brightness(follower, leader, max_px, max_py) → A_BOLD or A_NORMAL
Purpose: boids close to leader render bold (brighter).
Steps:
  1. Compute toroidal distance follower→leader.
  2. Ratio = distance / PERCEPTION_RADIUS.
  3. If ratio < 0.35: A_BOLD. Else: A_NORMAL.

### draw_boid(window, boid, ch, attr, cols, rows, max_px, max_py, alpha, dt_sec)
Purpose: draw one boid at its render-interpolated position.
Steps:
  1. draw_px = boid.px + boid.vx * alpha * dt_sec.
  2. draw_py = boid.py + boid.vy * alpha * dt_sec.
  3. Clamp to pixel bounds.
  4. Convert to cell: cx = px_to_cell_x(draw_px), cy = px_to_cell_y(draw_py).
  5. If within terminal bounds: wattron(attr); mvwaddch(cy, cx, ch); wattroff(attr).

### scene_draw(scene, window, cols, rows, alpha, dt_sec)
Purpose: draw all flocks — followers then leader for each flock.
Steps:
  1. For each flock fi:
     a. For each follower i: pick glyph from velocity_dir_char, pick brightness from follower_brightness, call draw_boid.
     b. Draw leader with leader color pair, always A_BOLD.

## Pseudocode — Core Loop

```
main:
  seed random from current nanosecond time
  register SIGINT, SIGTERM → set running=0
  register SIGWINCH → set need_resize=1
  register atexit cleanup
  screen_init()
  scene_init(screen.cols, screen.rows)

  frame_time = now_ns()
  sim_accum = 0
  fps_accum = 0
  frame_count = 0
  fps_display = 0

  while running:
    if need_resize:
      screen_resize()
      scene_resize() or full reinit
      frame_time = now_ns()
      sim_accum = 0

    now = now_ns()
    dt = now - frame_time
    frame_time = now
    clamp dt to 100ms max

    sim_accum += dt
    tick_ns = NS_PER_SEC / sim_fps

    while sim_accum >= tick_ns:
      scene_tick(tick_ns / NS_PER_SEC as float)
      sim_accum -= tick_ns

    alpha = sim_accum / tick_ns   [0, 1)
    dt_sec = tick_ns / NS_PER_SEC

    frame_count++
    fps_accum += dt
    if fps_accum >= 500ms:
      fps_display = frame_count / (fps_accum / NS_PER_SEC)
      reset frame_count, fps_accum

    sleep until frame cap (1/60 sec from last frame)

    erase screen
    scene_draw(alpha, dt_sec)
    draw HUD
    doupdate()

    ch = getch()
    if ch != ERR: handle_key(ch)

scene_tick(dt_sec):
  if paused: return
  max_px = cols * CELL_W
  max_py = rows * CELL_H
  if mode == PREDATOR:
    predator_leader_tick(flock[0], flock[1..], dt_sec, ...)
    for follower in flock[0]: chase_steer → apply
    for each prey flock[1..]:
      leader_tick(...)
      for follower: prey_boids_steer → apply
    return
  for each flock:
    leader_tick(dt_sec)
    if ORBIT: orbit_phase += ORBIT_SPEED * dt_sec
    for each follower (Stage 1, read-only):
      new_v[i] = steering_fn(follower i)
    for each follower (Stage 2, write):
      follower[i].v = new_v[i]
      clamp_speed(follower[i])
      follower[i].pos += v * dt_sec
      wrap(follower[i])

boids_steer(flock, idx):
  sep = (0,0); ali = (0,0); coh = (0,0)
  neighbor_n = sep_n = 0
  for each other follower j:
    d = toroidal_delta(b, followers[j])
    dist = length(d)
    if dist >= PERCEPTION_RADIUS: continue
    ali += followers[j].velocity
    coh += d           [toroidal offset, not position]
    neighbor_n++
    if dist < SEPARATION_RADIUS:
      closeness = (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS
      sep -= (d/dist) * closeness
      sep_n++
  steer = (0,0)
  if sep_n > 0:  steer += normalize(sep) * W_SEPARATION * MAX_STEER
  if neighbor_n > 0:
    avg_ali = ali / neighbor_n
    desired_v = normalize(avg_ali) * cruise_speed
    steer += (desired_v - b.v) * W_ALIGNMENT
    avg_coh = coh / neighbor_n
    desired_v = normalize(avg_coh) * cruise_speed
    steer += (desired_v - b.v) * W_COHESION
  leader_d = toroidal_delta(b, leader)
  steer += normalize(leader_d) * W_LEADER_PULL * MAX_STEER
  clamp steer magnitude to MAX_STEER
  return b.v + steer
```

## Interactions Between Modules

```
§9 app
  calls: screen_init, scene_init, scene_tick, scene_draw,
         screen_draw (HUD), screen_present, app_handle_key

§7 scene
  calls: flock_tick (§6), flock_init (§6), draw_boid (§7)
  reads: pw/ph from §4 coords

§6 flock
  calls: leader_tick, boids_steer, chase_steer, vicsek_steer,
         orbit_steer, prey_boids_steer, predator_leader_tick
         boid_clamp_speed (§5), boid_wrap (§5), toroidal_delta (§5)

§5 boid
  calls: boid_spawn_at, boid_clamp_speed, boid_wrap, toroidal_delta
  provides: velocity_dir_char → used by draw_boid in §7

§4 coords
  provides: pw, ph, px_to_cell_x, px_to_cell_y → used by §7 and §6

§3 color
  provides: color pairs looked up by follower_color_pair(), leader_color_pair()
  called by: §9 app init, screen_init

Shared state:
  g_app (global App struct)
  g_app.scene.flocks[] — written by §6 flock routines, read by §7 draw
  g_app.scene.mode — written by §9 key handler, read by §6 and §7
  g_app.running, g_app.need_resize — written by signal handlers, read by §9 main loop
```
