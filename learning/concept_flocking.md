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
