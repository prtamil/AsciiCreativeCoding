# Pass 1 — spring_pendulum: Lagrangian Spring-Pendulum Simulation

## Core Idea

A mass hangs from a fixed pivot by a spring that can both rotate (pendulum) and stretch/compress (spring). When the spring frequency is exactly 2× the pendulum frequency, the two modes of motion exchange energy — the bob traces complex rosette paths and the motion looks organic and chaotic. This is the **2:1 resonance** condition.

---

## The Mental Model

Imagine holding a spring with a weight at the end. You can swing it like a pendulum (rotation), or the spring itself can stretch and compress (radial oscillation). Normally these two motions happen independently. But if you tune the spring stiffness so the spring bounces exactly twice as fast as it swings, energy flows back and forth between the two modes — the bob traces big looping rosette shapes, then tightens to a nearly vertical bounce, then spreads out again.

The program simulates this in polar coordinates (angle + length) rather than (x, y) because the physics equations are cleaner in polar form. It then converts to pixel coords to draw a:
- Top bar with a pivot marker
- Spring coil (zigzag computed from the pivot-to-bob axis)
- Iron bob `(@)` at the end

---

## Data Structures

### `Pendulum` struct
| Field | Meaning |
|---|---|
| `r` | Current spring length in pixels |
| `theta` | Current angle from vertical (radians, positive = right) |
| `r_dot` | Radial velocity (how fast r is changing) |
| `th_dot` | Angular velocity (how fast theta is changing) |
| `prev_r` | Spring length at start of previous tick (for lerp) |
| `prev_theta` | Angle at start of previous tick (for lerp) |
| `pivot_px/py` | Fixed anchor point in pixel space (top center) |
| `r0` | Natural rest length of spring (40% of pixel height) |
| `damping` | Friction coefficient — reduces velocities over time |

### `Scene` struct
Owns one `Pendulum` plus terminal dimensions and paused flag.

### `Screen` struct
Terminal columns and rows — just dimensions, no ncurses state.

---

## The Main Loop

1. Check for resize signal → reinit scene to new dimensions
2. Compute `dt` (wall time since last frame, capped at 100ms)
3. Add `dt` to `sim_accum`
4. While `sim_accum >= tick_ns`: run `pendulum_tick(dt_sec)`, subtract `tick_ns`
5. Compute `alpha = sim_accum / tick_ns`
6. Update FPS display every 500ms
7. Sleep to cap render at 60fps
8. `screen_draw(alpha)` → `screen_present()`
9. Handle keyboard input

---

## Non-Obvious Decisions

### Why polar coordinates?
The spring pendulum equations of motion look simple in polar (r, θ):
```
r̈  =  r·θ̇²  +  g·cos θ  −  k·(r − r₀)  −  d·ṙ
θ̈  =  −[g·sin θ  +  2·ṙ·θ̇] / r  −  d·θ̇
```
The `2·ṙ·θ̇` term is the Coriolis effect (energy exchange between modes). In Cartesian (x, y) these equations become coupled in a messier way.

### Why symplectic Euler?
Regular Euler integration adds energy over time — the spring grows larger each step. Symplectic Euler updates velocities first, then positions using the new velocities. This preserves the symplectic structure of Hamiltonian systems — energy stays bounded long-term. No energy drift even over thousands of steps.

Update order is critical:
```
r̈, θ̈ computed from current state
r_dot += r_ddot * dt    ← velocity first
th_dot += th_ddot * dt
r += r_dot * dt         ← position uses NEW velocity
theta += th_dot * dt
```

### Why 2:1 frequency ratio?
- Pendulum frequency: `ω_pend = √(g / r₀)`
- Spring frequency: `ω_spring = √(k / m)`
- At exactly 2:1, energy transfers completely between modes every half-period
- This is where the interesting rosette shapes appear
- The constants are tuned: with `g=2000`, `k=25`, `r₀ = 0.4 × rows × CELL_H`, ratio ≈ 2.0

### Why `N_COILS * 2` nodes for the spring?
The spring is drawn as a zigzag. Each "coil" has two nodes — one offset left, one offset right of the spring axis. The nodes are evenly spaced along the pivot-to-bob line in pixel space. By computing perpendicular unit vectors from the spring axis, the zigzag automatically rotates as the pendulum swings.

### Why draw order: wire → coil lines → coil nodes → bob?
Later draws overwrite earlier ones. Coil nodes (`*`) should overwrite the lines connecting them. The bob `(@)` should overwrite any coil that overlaps it at the end.

---

## State Machines

No state machine — the pendulum is purely continuous physics. There is one boolean flag:
```
paused = false  ←→  paused = true
        space key toggles
```

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| `GRAVITY_PX` | 2000 | Faster swing, higher pendulum frequency |
| `SPRING_K` | 25 | Faster spring oscillation |
| `DAMPING_DEF` | 0.12 | More friction → motion dies faster |
| `REST_LEN_FRAC` | 0.40 | Longer spring → slower pendulum frequency |
| `INIT_THETA_DEG` | 40° | Larger initial swing angle |
| `INIT_R_STRETCH` | 1.15 | Bob starts 15% above rest length |
| `N_COILS` | 8 | More coils in the spring visual |
| `COIL_SPREAD` | 2 cells | Wider spring zigzag |
| `SIM_FPS_DEFAULT` | 120 | Higher physics rate → more accurate ODEs |

**Tuning for resonance:** if `GRAVITY_PX` changes, `SPRING_K` must change to maintain 2:1. Formula: `SPRING_K = 4 * GRAVITY_PX / r₀`. With `r₀ = REST_LEN_FRAC × rows × CELL_H`.

---

## Open Questions for Pass 3

1. What happens visually if you break the 2:1 ratio? Try `SPRING_K = 10` vs `SPRING_K = 100`.
2. What does removing the `2·ṙ·θ̇` Coriolis term do to the motion?
3. What goes wrong if you use regular Euler instead of symplectic — how fast does energy drift?
4. The spring coil zigzag — can you reproduce the exact node positions from scratch?
5. Why does `r` need clamping? What would happen at `r → 0`?
6. Try `DAMPING = 0` (no friction) — does the motion stay bounded?
