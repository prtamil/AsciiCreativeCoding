# Pass 1 — xrayswarm.c: Multi-swarm X-ray pulsating rays with converge-to-origin locomotion

## Core Idea

Multiple swarms of ASCII workers radiate outward from a fixed queen position (DIVERGE), park at the screen edge, pause, then retrace their exact paths back to the origin queen (CONVERGE). Each worker travels in a straight line in its assigned direction at a fixed speed. The queen never moves. The result looks like a pulsating biological organism: rays shoot out in all directions simultaneously, hold at the edge, then collapse back to the source point in perfect symmetry.

## The Mental Model

Picture a sea anemone or a starfish. In the centre sits the queen — a fixed point. Workers radiate outward like spines, travel to the screen boundary, park there, then retract back to the exact same origin. The whole cycle repeats indefinitely.

The key insight is that each worker must return to the exact queen position it left from, not where the queen is when it returns (the queen is stationary, so these are the same — but the implementation locks the queen coordinates at DIVERGE end to guarantee correctness). Workers store their parked screen-edge coordinates (`park_px`, `park_py`) and target the locked queen position during CONVERGE.

Multiple swarms occupy the same terminal simultaneously. Pass-based rendering prevents trail cancellation: all DIM trails are drawn first across all swarms, then MID, then BRIGHT, then HEAD (the leading dots). This ensures bright tips always overwrite dim trails even when swarms overlap.

## Data Structures

### Bug (worker particle)
```
px, py          — current position in pixel space
vx, vy          — velocity (pixels/sec); zero when parked
park_px, park_py — stored screen-edge position (set at DIVERGE→CONVERGE transition)
cp              — color pair index for this bug
```
Workers have no "done" flag. Arrival is detected in tick functions by distance threshold.

### Swarm
```
queen_x, queen_y    — fixed centre position (never moves)
locked_qx, locked_qy — queen position saved at DIVERGE end (used during CONVERGE)
workers[]           — up to MAX_BUGS worker Bug structs
n_workers           — how many workers are active
phase               — current SwarmPhase (DIVERGE / PAUSE / CONVERGE)
phase_timer         — elapsed seconds in current phase
cp                  — color pair for this swarm
```

### SwarmPhase
```
DIVERGE   — workers fly outward from queen to screen edge
PAUSE     — all workers parked; holding for PAUSE_DUR seconds
CONVERGE  — workers fly inward from park position back to locked queen
```

## The Pulse Cycle

**DIVERGE** (until all workers reach screen edge, or DIVERGE_DUR timeout):
- Workers travel at constant velocity from queen outward
- On hitting screen boundary: `vx = vy = 0` (parked), `park_px/park_py` saved
- Transition: when `phase_timer >= DIVERGE_DUR` → lock queen, enter PAUSE

**PAUSE** (PAUSE_DUR seconds):
- All workers stationary at screen edge
- Timer counts; queen position locked into `locked_qx/locked_qy`
- Transition: `phase_timer >= PAUSE_DUR` → launch CONVERGE

**CONVERGE** (until all workers reach queen, or CONVERGE_DUR timeout):
- `worker_launch_converge()`: compute velocity from `park_px/park_py` toward `locked_qx/locked_qy`
- Workers travel inward; on arriving within threshold: `vx = vy = 0` (parked at queen)
- Transition: `phase_timer >= CONVERGE_DUR` → respawn workers, enter DIVERGE

## Why the Queen is Locked

If the queen moved during CONVERGE, workers would target a moving point and miss the exact origin they diverged from. Since the queen IS stationary in this implementation, `locked_qx/locked_qy` equals the fixed queen position — but locking is still the correct pattern because it decouples worker trajectory computation from the queen's current state.

## Pass-Based Multi-Swarm Rendering

With multiple swarms drawn each frame, a worker trail from swarm A can overwrite the bright head dot of swarm B if drawing order is arbitrary. The fix: four render passes across ALL swarms:

```
Pass 1 (DP_DIM):    draw all trail bodies (dim chars) for all swarms
Pass 2 (DP_MID):    draw all mid-brightness segments for all swarms
Pass 3 (DP_BRIGHT): draw all near-head segments for all swarms
Pass 4 (DP_HEAD):   draw all leading head dots for all swarms
```

Because ncurses writes the last value placed in a cell, and pass 4 runs after all other passes, bright heads always win against overlapping trails from any swarm.

## Non-Obvious Decisions

### Workers park (zero velocity) at edge, not respawn
Earlier versions had workers bounce or respawn when hitting the wall. Parking (`vx=vy=0`) gives a clean "ray held at boundary" look, and makes the CONVERGE path exactly retrace the DIVERGE path since the start point is the stored edge coordinate.

### Storing `park_px/park_py` vs computing from queen + direction
Computing the return direction from `(locked_qx, locked_qy)` to `(park_px, park_py)` gives the exact inverse of the outward path. The stored edge position is the ground truth. Recomputing from direction would accumulate floating-point error across long paths.

### Fixed queen position — `queen_tick` removed
Early versions had the queen wander; workers returned to where the queen currently was. This created asymmetric pulses and workers arriving at wrong positions. Removing all queen motion makes the pulse perfectly symmetric and the source/sink point identical every cycle.

## Open Questions to Explore

1. What if workers had slightly randomised speeds — would the DIVERGE front look more biological (spread out) or just messy?
2. Can each swarm have a different timing phase (PAUSE offset) so pulses from different queens are staggered?
3. What if trails faded in brightness with distance from queen — giving a bioluminescent glow that diminishes at range?
4. Can workers curve (constant angular velocity) instead of straight lines — spiral arms?

---

## From the Source

**Algorithm:** Swarm simulation with a 4-phase state machine per swarm: DIVERGE (workers shoot outward), PAUSE, CONVERGE (workers fly inward), second PAUSE. Queen wanders with smooth Ornstein-Uhlenbeck-like bounded random-walk steering (`QUEEN_JITTER=35.0`, `QUEEN_DAMP=0.96`). Worker headings locked at DIVERGE start; held fixed during flight (`WORK_JITTER=4.0`, `WORK_DAMP=0.994`).

**Data-structure:** Per-worker ring buffer of `TRAIL_LEN=48` past positions. Older trail positions drawn at decreasing brightness. Buffer is a circular array with head pointer — O(1) append. N_WORKERS=20 per swarm, N_SWARMS_MAX=5.

**Physics:** Phase durations: DIVERGE_DUR=3.5 s, CONVERGE_DUR=3.5 s, PAUSE_DUR=0.7 s. Arrival threshold: ARRIVE_DIST=40 px. CONVERGE homing correction: `CONVERGE_STEER=5.0` per second (proportional steering toward locked queen). Worker speed: WORK_SPEED=380 px/s; queen speed: QUEEN_SPEED=80 px/s.

---

# Pass 2 — Pseudocode, Module Map, Data Flow

## Module Map

```
§1 config      — MAX_BUGS, N_SWARMS, speeds, durations, color defs
§2 clock       — clock_ns() / clock_sleep_ns()
§3 color       — per-swarm color pairs, DIM/MID/BRIGHT/HEAD brightness
§4 coords      — px_col() / px_row()
§5 entity      — Bug struct, Swarm struct, SwarmPhase enum
               — swarm_init(), worker_tick_diverge(), worker_launch_converge()
               — worker_tick_converge(), swarm_tick()
§6 draw        — scene_draw(): 4-pass rendering across all swarms
§7 screen      — Scene, screen_init/render/resize
§8 app         — App, signal handlers, main loop
```

## Data Flow Diagram

```
swarm_init(s)
    │ assign queen_x/queen_y (fixed, center ± offset)
    │ spawn N workers: direction = k × (360°/N), speed = WORKER_SPEED
    ▼
swarm_tick(s, dt, max_px, max_py)
    │
    ├─ DIVERGE:
    │    worker_tick_diverge(w, max_px, max_py)
    │      │ w.px += w.vx × dt,  w.py += w.vy × dt
    │      │ if hit boundary: vx=vy=0, park_px/py=px/py
    │    if phase_timer >= DIVERGE_DUR:
    │      locked_qx = queen_x, locked_qy = queen_y
    │      phase = PAUSE, timer = 0
    │
    ├─ PAUSE:
    │    if phase_timer >= PAUSE_DUR:
    │      worker_launch_converge(all workers, locked_qx, locked_qy)
    │      phase = CONVERGE, timer = 0
    │
    └─ CONVERGE:
         worker_tick_converge(w, locked_qx, locked_qy)
           │ w.px += w.vx × dt,  w.py += w.vy × dt
           │ if dist(w, queen) < ARRIVE_THRESH: vx=vy=0
         if phase_timer >= CONVERGE_DUR:
           respawn workers at queen, phase = DIVERGE, timer = 0
    ▼
scene_draw (4 passes over all swarms):
    Pass DP_DIM    → trail body chars for every swarm
    Pass DP_MID    → mid-brightness chars
    Pass DP_BRIGHT → near-head chars
    Pass DP_HEAD   → leading dot (queen head char) for every swarm
    ▼
wnoutrefresh → doupdate
```

## Core Loop Pseudocode

```
init N_SWARMS swarms (fixed queens, spread positions)
while running:
    if SIGWINCH: resize
    dt = elapsed since last frame, clamped to 100ms
    for each swarm s:
        swarm_tick(s, dt, max_px, max_py)
    erase()
    for pass in [DIM, MID, BRIGHT, HEAD]:
        for each swarm s:
            draw all worker trails at this brightness pass
    draw HUD
    doupdate()
    sleep to target FPS
    handle keys: q=quit spc=pause r=reset
```

## Key Equations

**Worker initial velocity (DIVERGE):**
```
angle_k = k × (2π / N_WORKERS)
vx_k = WORKER_SPEED × cos(angle_k)
vy_k = WORKER_SPEED × sin(angle_k)
```

**Worker CONVERGE launch:**
```
dx = locked_qx − park_px
dy = locked_qy − park_py
dist = sqrt(dx² + dy²)
vx = (dx / dist) × WORKER_SPEED
vy = (dy / dist) × WORKER_SPEED
```

**Screen-edge parking test:**
```
if px ≤ 0 || px ≥ max_px || py ≤ 0 || py ≥ max_py:
    vx = vy = 0
    park_px = px,  park_py = py
```

**Arrival test (CONVERGE):**
```
if sqrt((px−locked_qx)² + (py−locked_qy)²) < ARRIVE_THRESH:
    vx = vy = 0
```
