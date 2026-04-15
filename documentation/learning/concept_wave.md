# Pass 1 — fluid/wave.c: 2-D wave equation interference patterns

## Core Idea

A grid of floating-point values represents the amplitude of a wave at each terminal cell. Each tick the grid advances one step of the finite-difference time-domain (FDTD) wave equation. Five point sources oscillate at slightly different frequencies, producing interference fringes that shift and beat over time. Amplitude is mapped to characters and colours: troughs are cool/dim, crests are warm/bright, and near-zero cells are blank so the terminal background shows through.

## The Mental Model

Imagine dropping a pebble into a pond — ripples spread outward in circles, and where two sets of ripples meet, they add together (constructive interference) or cancel (destructive interference). This simulation has five such "pebbles" oscillating continuously. Each source drives a sinusoidal disturbance at a slightly different frequency, so the interference pattern slowly rotates and shifts as the phases drift apart. Toggling a source on or off is like lifting a pebble out of the water — the interference pattern immediately simplifies.

## Physics Model

The scalar 2-D wave equation:

    ∂²u/∂t² = c² · ∇²u

Discretised with the explicit FDTD second-order scheme:

    u_new[r][c] = 2·u_cur[r][c] − u_prev[r][c]
                + c² · (u[r+1][c] + u[r-1][c] + u[r][c+1] + u[r][c-1] − 4·u[r][c])

where the last part is the discrete 4-point Laplacian. After each step, all values are multiplied by a damping factor (≈ 0.993) so energy gradually dissipates — without damping the grid would saturate.

**CFL stability condition:** for 2-D, `c·√2 ≤ 1` → `c ≤ 0.707`. The code uses `c = 0.45` for a comfortable margin.

## Data Structures

### Three grid planes
```
float u_prev[ROWS][COLS]   — amplitude two steps ago
float u_cur [ROWS][COLS]   — current amplitude
float u_new [ROWS][COLS]   — scratch for next step
```
After computing `u_new`, the planes rotate: `u_prev ← u_cur`, `u_cur ← u_new`.

### Source
```
int r, c          — grid position
float phase       — current oscillation phase (radians)
float freq        — oscillation frequency (radians per step)
bool active       — toggled by keys 1-5
```

## Animation Loop

Each tick:
1. For each active source: inject `A·sin(phase)` into `u_cur[r][c]` and advance `phase += freq`.
2. Compute `u_new` from `u_cur` and `u_prev` using the FDTD formula.
3. Multiply `u_new` by `DAMP`.
4. Rotate: `u_prev = u_cur`, `u_cur = u_new`.
5. Render: for each cell, map amplitude to character and color.

## Non-Obvious Decisions

### Why three planes instead of two?
The FDTD formula needs both the current and previous step. With two planes, you'd need to read the previous value before overwriting with the new one — this requires either a temporary copy or updating in the wrong order. Three planes make the rotation trivially safe.

### Why the 4-point Laplacian instead of 9-point?
The 4-point stencil is sufficient for visual quality and simpler to code. The 9-point isotropic stencil (used in reaction_diffusion.c) matters more for patterns that need rotational symmetry — wave fronts look correct with 4-point.

### Why different source frequencies?
If all five sources had the same frequency, the interference pattern would be static (a fixed standing-wave node pattern). Slightly different frequencies cause the phase relationships to drift, producing slowly evolving fringes — visually much richer.

### Signed amplitude rendering
The amplitude is real-valued and can be positive or negative. Rendering both signs with different colour schemes makes it immediately clear which cells are crests vs troughs — a purely magnitude-based rendering would lose half the information.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|---|---|---|
| c (wave speed) | 0.45 | Higher → faster propagation, but > 0.707 → instability |
| DAMP | ≈ 0.993 | Lower → faster decay; higher → patterns persist longer |
| Source freq offsets | ~0.01–0.02 | Larger → faster phase drift; smaller → nearly static pattern |
| STEPS_PER_FRAME | 1–8 | Higher → simulation runs faster relative to wall clock |

## From the Source

**Algorithm:** Unlike the wave_2d.c sponge-boundary variant, this file uses multiple independent oscillating sources that drive the field continuously, creating persistent standing/travelling waves. FDTD 2nd-order explicit scheme.

**Math:** CFL stability: C_SPEED=0.45 gives CFL number = 0.45·√2 ≈ 0.636 (not just "comfortable margin" — the exact value). Energy dissipation: DAMPING_DEFAULT=0.993 per tick; without damping, energy accumulates until numerical overflow.

**Performance:** O(W×H) per step. STEPS_DEFAULT=4 sub-steps per render frame advance the simulation faster without changing CFL. Grid uses three flat arrays (prev/cur/new) for cache efficiency; 2D index is y*W + x (row-major).

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `Grid.curr` | `float*` (malloc'd, cols×rows) | ~20 KB at 256×20 | current wave amplitude |
| `Grid.prev` | `float*` (malloc'd, cols×rows) | ~20 KB | previous-step amplitude (FDTD needs t and t−1) |
| `Grid.next` | `float*` (malloc'd, cols×rows) | ~20 KB | scratch for next-step; rotated into curr after each step |
| `g_sources[N_SOURCES]` | `Source[5]` | ~60 B | fractional positions, frequencies, phases, active flags |
