# Pass 1 — chain.c: Hanging Chain & Swinging Rope via Position-Based Dynamics

## Core Idea

A chain of N point masses connected by inextensible links. Each link tries to maintain a fixed rest length. The simulation uses **Position-Based Dynamics (PBD)**: instead of computing spring forces and integrating them, it predicts positions with Verlet integration and then directly moves nodes to satisfy distance constraints.

Four presets demonstrate different chain behaviours:
- **Hanging**: top pinned, wind sways the chain
- **Pendulum**: released from 60° angle, waves propagate along the swinging tail
- **Bridge**: both ends pinned, catenary sag, vertical gusts
- **Wave**: top node oscillated sinusoidally, standing waves emerge

## The Mental Model

### Why PBD instead of springs?

A stiff chain (one that resists stretching) requires high spring constants. But explicit numerical integration (Euler, even symplectic Euler) becomes unstable when spring constants are too large: the force overshoots equilibrium, creating oscillations that grow rather than decay. The stability condition for explicit Euler is `k × dt² < 2`, which forces tiny dt for stiff springs.

PBD bypasses force integration entirely. After predicting new positions with Verlet, it corrects the positions directly to satisfy the length constraint: move both endpoints of each link until the link is exactly the right length. This correction is geometrically exact. The chain is as stiff as you want — adding more iterations makes it stiffer, not unstable.

### Verlet integration (implicit velocity)

Instead of storing velocity explicitly, PBD stores two positions: the current `(x, y)` and the old `(ox, oy)`. The velocity is implicit:

```
v ≈ (x - ox) / dt
```

To advance time: predict the new position as if no constraints exist:
```
vx = (x - ox) * DAMP        ← implicit velocity × DAMP
new_x = x + vx + a_x * dt²  ← Verlet: x + v*dt + a*dt²
ox = x;  x = new_x           ← slide window forward
```

The acceleration term here is gravity (and optionally wind). After the constraint projection step, the "velocity" encoded in `(x - ox)` automatically reflects the impulses applied by the constraints — no velocity update needed.

### Distance constraint projection

For a link between nodes a and b with rest length `L`:
```
d = b.pos − a.pos
dist = |d|
correction_vector = (dist − L) / dist × d    ← points along the link, magnitude = error
```

Split equally between the two endpoints (assuming equal mass):
```
a.pos += 0.5 × correction_vector
b.pos -= 0.5 × correction_vector
```

If one endpoint is pinned (infinite mass), the other absorbs the full correction:
```
b.pos -= correction_vector   (a is pinned)
a.pos += correction_vector   (b is pinned)
```

### Why multiple iterations?

One pass of constraint projection only partially satisfies each constraint, because satisfying one link perturbs its neighbours. Multiple iterations (N_ITER = 20 by default) propagate corrections through the chain until all links are approximately at their rest length. More iterations = stiffer chain. Fewer = stretchier, "rubber" chain.

## Data Structures

### ChainNode (§5)
```
float x,  y    — current position (pixel space)
float ox, oy   — previous position (velocity = (x-ox)/dt)
bool  pinned   — position controlled externally; never modified by PBD
```

### Chain (§5)
```
ChainNode nodes[N_NODES_MAX]
int   n_nodes              — number of nodes (24 by default)
float link_rest            — rest length per link (pixels)
float wind_phase           — oscillation phase for sinusoidal wind
float wind_str             — wind force amplitude
bool  wind_on
bool  paused
int   preset
int   n_iter               — constraint iterations per sub-step (5–60)

float wave_phase           — oscillation phase for wave preset
float wave_ax, wave_ay     — fixed anchor position for wave driver

float trail_px[TRAIL_LEN]  — ring buffer of last-node positions
float trail_py[TRAIL_LEN]
int   trail_head, trail_cnt
```

## The Main Loop

1. **Tick** (`chain_tick`): advance `wind_phase` and `wave_phase` by `WIND_FREQ × 2π × dt` and `WAVE_FREQ × 2π × dt`. Run SUB_STEPS sub-steps of PBD. Record last free node into trail ring buffer.
2. **PBD sub-step** (`chain_pbd_step`):
   - For preset 3: set nodes[0].x/ox to driven position before step
   - Verlet predict all free nodes (gravity + wind acceleration)
   - Iterate N_ITER times: project all (i, i+1) distance constraints
3. **Draw** (`chain_draw`): trail dots → link segments → node markers → HUD

## Non-Obvious Decisions

### SUB_STEPS = 8 at 60fps
Each physics tick subdivides into 8 sub-steps of dt/8 each. This gives constraint projection 8 opportunities per frame to distribute corrections. Without sub-stepping, the chain accumulates positional error over a full frame, causing visible jitter. `dt_sub = dt/8 ≈ 2ms` keeps the simulation visually smooth.

### DAMP = 0.997 per sub-step
At 60fps × 8 sub-steps = 480 evaluations/second:
```
0.997^480 ≈ 0.234
```
The velocity decays to ~23% of its initial value per second. This provides realistic energy dissipation (rope in air) without over-damping (which would make the chain look like it's moving through honey).

### link_rest = total_length / (N-1)
The total chain length is set to ~75% of screen height. Dividing by (N-1) gives the rest length of each link. This means the chain always reaches approximately to the bottom of the screen when hanging straight. For the bridge preset, `rest × 1.25 / (N-1)` makes the total arc length 25% longer than the straight-line span, producing a visible catenary sag.

### Wave driving: set both x and ox
In the wave preset, the driver sets not just `nodes[0].x` but also `nodes[0].ox`:
```c
nodes[0].x  = anchor_x + A * sin(phase_now)
nodes[0].ox = anchor_x + A * sin(phase_prev)
```
This gives the driven node a correct velocity `(x - ox)/dt = A·ω·cos(phase)`, which propagates naturally into the constraint projection. If only `x` were set (leaving `ox` unchanged), the driver would appear to "teleport" each tick and the velocity would be wrong.

### Tension coloring
Real chain links under high tension would snap. Visualising tension lets the user see the load distribution:
- Near a fixed pivot, the links carry the weight of everything below — highest tension
- Near the free end, links carry only the weight of a few nodes — lowest tension
- During a pendulum swing, centripetal acceleration adds to tension in the outer links

The stretch ratio `|dist - rest| / rest` is computed per link each draw frame.

### seg_draw: DDA with oriented characters
Link segments between node cell positions are drawn with the most appropriate ASCII character:
- `|dist_x| ≥ 2 * |dist_y|` → `-` (nearly horizontal)
- `|dist_y| ≥ 2 * |dist_x|` → `|` (nearly vertical)
- `dx * dy > 0` → `\` (positive slope in screen coords)
- else → `/` (negative slope)

This avoids the "jagged" appearance of drawing only at the endpoints, and gives the chain a rope-like visual texture.

## State Machines

```
PRESET 0,1,2,3 — no automatic transitions; user advances manually with n/N

TICK:
  wind_phase advances → sinusoidal wind force changes direction each cycle
  wave_phase advances → driven node oscillates; waves travel down chain
  trail ring buffer fills → draws fading tail of bottom node

CONSTRAINT ITERATION:
  repeat N_ITER times:
    for each link i:  project distance constraint (a, b)
  (errors propagate from top to bottom on each pass,
   bottom-to-top on the next — coverage is from both ends)
```

## From the Source

**Algorithm:** Position-Based Dynamics (PBD / XPBD). Constraint projection formula: `correction = (|d| − rest) / |d| · d_vector`. Each free endpoint absorbs half the correction (equal mass assumption); a pinned node absorbs none (infinite mass).

**Math:** Velocity is implicit — no explicit velocity variable. Damping multiplies the positional delta before the predict step: `vx = (x − ox) * DAMP`.

**Performance:** Cost is O(N · I · S) per frame where N = nodes, I = iterations (N_ITER), S = sub-steps (SUB_STEPS = 8). Each sub-step runs the full constraint loop, improving stability for stiff constraints without shrinking the render frame rate or using an implicit (matrix-solving) integrator.

**Physics/References:** PBD is unconditionally stable — no spring constant to blow up. Stiffness is purely iteration count, not a numerical parameter. More iterations → stiffer rope, but each iteration is just a few multiplies.

**Data-structure:** Ring-buffer trail (TRAIL_LEN entries) for the last free node. Oldest entry is `trail_head` wrapped by modulo arithmetic — no shifts needed.

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `N_NODES_DEF` | 24 | More nodes → smoother curves, more computation |
| `N_ITER_DEF` | 20 | More iterations → stiffer chain; fewer → stretchier |
| `SUB_STEPS` | 8 | More sub-steps → less jitter; fewer → more instability |
| `GRAVITY` | 380 px/s² | Higher → faster sag; lower → floaty |
| `DAMP` | 0.997/sub-step | Lower → overdamped; higher → bouncy/oscillating |
| `WAVE_FREQ` | 1.8 Hz | Different frequencies excite different standing-wave modes |
| `WAVE_AMPL` | 22 px | Amplitude of the top-node driver |
| `TRAIL_LEN` | 90 | Length of the bottom-node trail ring buffer |

## Themes

5 themes cycle with `t`/`T`:

| # | Name | Relaxed links | Stretched links | Character |
|---|---|---|---|---|
| 0 | Classic | Cyan | Red | Cold to hot tension gradient |
| 1 | Fire | Orange | Dark red | All warm tones |
| 2 | Ice | Blue | White | Cold, crystal-clear |
| 3 | Neon | Magenta | Violet | Glowing pink |
| 4 | Matrix | Green | Dark green | Digital |

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_chain` | `Chain` | ~5 KB | entire chain state (nodes, trail, wind, preset) |
| `g_chain.nodes[N_NODES_MAX]` | `ChainNode[32]` | ~512 B | current + previous positions + pinned flag per node |
| `g_chain.trail_px/py[TRAIL_LEN]` | `float[90]` × 2 | 720 B | ring-buffer trail of last free node pixel positions |
| `g_rows`, `g_cols` | `int` | 8 B | terminal dimensions, updated on resize |
| `g_sim_fps` | `int` | 4 B | current simulation rate (Hz) |
| `g_quit_flag`, `g_resize_flag` | `volatile sig_atomic_t` | 8 B | signal flags for SIGINT/SIGWINCH |

# Pass 2 — chain: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_NODES=24, N_ITER=20, SUB_STEPS=8, GRAVITY=380, DAMP=0.997 |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5 themes × 7 color pairs, `theme_apply(t)` |
| §4 coords | `px_to_cx()`, `px_to_cy()` |
| §5 physics | `ChainNode`, `Chain`, `seg_draw()`, `chain_pbd_step()`, `chain_tick()`, `chain_draw()` |
| §6 scene | 4 preset builders, `scene_init()` |
| §7 screen | ncurses init, `hud_draw()` |
| §8 app | main loop, input, signal handlers |

## Data Flow

```
chain_tick(dt):
  wind_phase += WIND_FREQ * 2π * dt
  wave_phase += WAVE_FREQ * 2π * dt
  sub_dt = dt / SUB_STEPS
  for s in 0..SUB_STEPS-1:
    if preset == 3:
      nodes[0].x  = wave_ax + WAVE_AMPL * sin(wave_phase - ...)
      nodes[0].ox = wave_ax + WAVE_AMPL * sin(phase_prev)
    chain_pbd_step(sub_dt)
  trail_push(nodes[last].x, nodes[last].y)

chain_pbd_step(dt):
  dt2 = dt * dt
  wind_force = wind_on ? wind_str * sin(wind_phase) : 0

  /* Verlet predict */
  for each free node i:
    vx = (x - ox) * DAMP
    vy = (y - oy) * DAMP
    new_x = x + vx + wind_force * dt2
    new_y = y + vy + GRAVITY * dt2
    ox = x; oy = y
    x = new_x; y = new_y

  /* Constraint projection */
  for iter in 0..N_ITER-1:
    for i in 0..N-2:
      a = nodes[i]; b = nodes[i+1]
      d = b.pos - a.pos
      dist = |d|
      corr_ratio = (dist - link_rest) / dist
      cx = corr_ratio * d.x
      cy = corr_ratio * d.y
      if both free:  a += (cx/2, cy/2); b -= (cx/2, cy/2)
      if a pinned:   b -= (cx, cy)
      if b pinned:   a += (cx, cy)
```

## Open Questions for Pass 3

- How does the wave preset's standing-wave pattern depend on WAVE_FREQ? At what frequencies do clear nodes and antinodes form?
- The constraint projection iterates top-to-bottom on each pass (i=0 to N-2). Would alternating forward/backward sweeps per iteration converge faster?
- Does N_ITER=20 fully satisfy all constraints, or is there residual stretch visible? (Check by printing max stretch per frame.)
- In the bridge preset, does the sag shape converge to a true catenary (hyperbolic cosine) or a parabola? (Answer: a chain under gravity forms a catenary; a cable under uniform horizontal load forms a parabola.)
- How does chain behaviour change if mass is non-uniform (heavier nodes near bottom)?
