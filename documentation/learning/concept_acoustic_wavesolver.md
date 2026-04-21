# Concept: Acoustic Wave FDTD Solver

## Pass 1 — Understanding

### Core Idea
Simulate pressure wave propagation in 2-D using the finite-difference time-domain (FDTD) method. The scalar wave equation is discretised with centred differences in space and the leapfrog (centred-in-time) scheme. A point source injects sinusoidal pressure waves; boundaries are either reflective (hard wall) or absorptive (sponge layer).

### Mental Model
Imagine a drum skin. Each cell is connected to its four neighbours by springs. When you push one cell (the source), the disturbance ripples outward in a circle. At hard walls it bounces back; at sponge walls it fades away. Two sources in the right positions create interference fringes — bright stripes where waves add up, dark stripes where they cancel.

### Key Equations
```
∂²p/∂t² = c² · ∇²p     (scalar wave equation)

Discretised:
p_new[i,j] = 2·p[i,j] − p_old[i,j]
            + rx²·(p[i+1,j] − 2p[i,j] + p[i-1,j])
            + ry²·(p[i,j+1] − 2p[i,j] + p[i,j-1])

where rx² = (c·dt/dx)²,  ry² = (c·dt/dy)²
```

### Data Structures
- Three float grids: `p_old[ROWS][COLS]`, `p[ROWS][COLS]`, `p_new[ROWS][COLS]`
- Source list: (row, col, frequency, amplitude, phase)
- BC type grid: `bc[ROWS][COLS]` (REFLECT or ABSORB)
- Sponge weights: `sponge[ROWS][COLS]` precomputed quadratic ramp

### Non-Obvious Decisions
- **Triple buffer not double**: Leapfrog needs t−1 and t simultaneously. Can't reuse t as both input and output. Three pointers rotated each step = O(1) overhead.
- **ASPECT_Y=2**: Terminal cells are 2:1 tall:wide. dy = dx/2 makes wavefronts circular on screen.
- **Additive source injection**: Adding to p_new (not clamping it) models a monopole — energy radiates outward without reflecting incoming waves. Clamped Dirichlet sources create impedance discontinuity.
- **CFL=0.90**: 10% safety margin. At CFL≥1.0 the simulation explodes in one step with no warning.
- **Sponge layer**: Quadratic ramp from 1.0→0.0 over SPONGE_WIDTH cells. Cheaper than PML but reflects a few percent of low-frequency energy.

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| C_SPEED | 0.45 | wave speed (cells/tick) |
| CFL | 0.90 | fraction of stability limit |
| ASPECT_Y | 2.0 | dy = dx/ASPECT_Y for circular wavefronts |
| SPONGE_WIDTH | 8 | absorbing boundary thickness (cells) |
| SPONGE_DAMP | 0.85 | sponge attenuation per cell (quadratic ramp) |

### Open Questions
- Why does increasing SPONGE_WIDTH reduce boundary reflection?
- What standing wave modes appear in a 100×40 reflective cavity?
- Why does the zero-crossing frequency estimator divide by 2?

## From the Source

**Algorithm:** 2-D explicit FDTD leapfrog on p_old/p/p_new triple buffer. Point source additive injection. Sponge absorbing BC, Dirichlet (p=0) reflective BC. Zero-crossing frequency estimator.

**Physics/References:** Scalar wave equation models acoustic pressure, surface water waves, and EM fields (scalar approximation). FDTD method: Yee (1966) for EM; Taflove & Hagness "Computational Electrodynamics" for PML and sponge layer details.

**Math:** 2-D CFL: c·dt·√(1/dx²+1/dy²) ≤ 1. With ASPECT_Y=2 and dx=dy_phys: effective constraint c·dt/dx·√(1+1/ASPECT_Y²) ≤ 1. At CFL=0.90 and ASPECT_Y=2: c·dt/dx = 0.90/√(1.25) ≈ 0.805.

**Performance:** O(ROWS·COLS) per step. For 120×80 grid: 9600 cell updates per tick, trivial at 60 fps.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `p_old[R][C]` | `float[R×C]` | ~38 KB (120×80) | pressure at t−1 |
| `p[R][C]` | `float[R×C]` | ~38 KB | pressure at t |
| `p_new[R][C]` | `float[R×C]` | ~38 KB | pressure at t+1 (write target) |
| `sponge[R][C]` | `float[R×C]` | ~38 KB | precomputed damping weights |
| `bc[R][C]` | `uint8_t[R×C]` | ~10 KB | BC type per cell |

---

## Pass 2 — Implementation

### Pseudocode
```
wave_step():
    for i in [1, ROWS-2], j in [1, COLS-2]:
        if bc[i][j] == REFLECT: continue   # skip boundary
        p_new[i][j] = 2*p[i][j] - p_old[i][j]
                    + rx2*(p[i+1][j] - 2*p[i][j] + p[i-1][j])
                    + ry2*(p[i][j+1] - 2*p[i][j] + p[i][j-1])

    # absorbing boundary sponge
    for all i,j:
        p_new[i][j] *= sponge[i][j]   # 1.0 interior, →0 near edges

    # point source injection (monopole — additive)
    for each source s:
        p_new[s.row][s.col] += s.amp * sin(s.phase)
        s.phase += 2π * s.freq * dt

    # rotate triple buffer
    tmp = p_old; p_old = p; p = p_new; p_new = tmp
```

### Module Map
```
§1 config      — C_SPEED, CFL, ASPECT_Y, SPONGE_WIDTH, LAMBDA_CELLS
§2 precompute  — rx2, ry2 from c/dx/dy; sponge weight table
§3 sources     — source list, phase accumulators, injection
§4 BC          — bc[] grid init (reflective walls / sponge / open)
§5 wave step   — leapfrog FDTD update + sponge + source inject
§6 buffer      — triple buffer pointer rotation
§7 render      — signed amplitude → color + char
§8 app         — main loop, BC toggle, source add/remove, keys
```

### Data Flow
```
p_old, p → leapfrog stencil → p_new
p_new × sponge → p_new (absorbing BC)
p_new += source → p_new (injection)
pointer rotate: (p_old←p, p←p_new, p_new←free)
p → signed amplitude → char + color → screen
```
