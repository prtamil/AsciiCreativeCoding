# Concept: SPH Fluid Simulation

## Pass 1 — Understanding

### Core Idea
Smoothed Particle Hydrodynamics (SPH) simulates a fluid as a collection of particles, each carrying mass, position, velocity, and pressure. Instead of a fixed grid, each particle "sees" its neighbours and estimates local density by summing a kernel function W(d) over all nearby particles. Pressure and viscosity forces are then derived from those density estimates and applied to accelerate each particle. The result is a fluid that forms pools, splashes, and flows without any grid.

### Mental Model
Each particle carries a "blob" of fluid mass with it. The kernel W(d) is a bell-shaped function that peaks at d=0 and goes to zero at d=H (the smoothing radius). When you sum W(d_ij) over all neighbours j, you get the local density at particle i. If the density is higher than the rest density, pressure pushes particles apart (repulsion); if lower, they're attracted (cohesion). Viscosity damps the velocity difference between neighbours — it makes the fluid "sticky" rather than inviscid.

### Key Equations

**This implementation's kernel** (endoh1 sign trick):
```
w(d) = (d/H − 1)²    if d < H
w(d) = 0              if d ≥ H
```
w is always non-negative (it's a squared term), and evaluates to 0 at d=H and (−1)²=1 at d=0.

**Density at particle i:**
```
ρᵢ = Σⱼ w(dᵢⱼ)
```

**Pressure force on i from j:**
```
F_pressure = direction_ij · w(dᵢⱼ) · (ρᵢ + ρⱼ − 2·ρ₀) · K_pressure / ρᵢ
```
When ρ > ρ₀: force is repulsive (pushing apart). When ρ < ρ₀: force is attractive (cohesion). The sign comes automatically from (ρᵢ+ρⱼ−2ρ₀).

**Viscosity force on i from j:**
```
F_viscosity = (vⱼ − vᵢ) · K_viscosity · w(dᵢⱼ)
```
Damps velocity differences between neighbours — particles relax toward the same velocity.

**Integration (symplectic Euler):**
```
v += a · dt
x += v · dt
```
Velocity is updated first, then position uses the new velocity — this conserves energy better than standard Euler.

**Wall bounce:** if a particle crosses a boundary, reflect velocity component with damping coefficient.

### Implementation-Specific Notes
- `SMOOTH_RADIUS = 2.2` (in pixel-space units, H in the equations)
- `REST_SUM = 6.0` (target density ρ₀ — calibrated so an interior particle with ~3 neighbours at d≈1 produces density ≈6)
- `PRESSURE_K = 0.04`, `VISCOSITY_K = 0.03`, `GRAVITY_G = 0.08`
- `WALL_DAMPING = 0.6` (60% velocity retained on wall bounce)
- `SPH_DT = 0.12` (fixed physics timestep)

**Spatial grid optimisation (O(N·k) neighbour search):**
- Grid cell size `GCELL = 3 ≥ SMOOTH_RADIUS = 2.2` — all neighbours within H lie in the 3×3 surrounding cells
- Linked list per cell: `ghead[gy][gx]` = head particle index, `gnext[i]` = next particle in same cell
- Rebuild grid each step in O(N); query 9 cells per particle in O(k) where k≈15 average neighbours
- ~50× faster than O(N²) brute-force at N=600+

### What the Original Code Had Wrong
The pasted reference code had three bugs:
1. **Inverted kernel**: returned `w*w` when `d > H` and 0 when `d < H` — the kernel contributed for far particles and ignored close ones
2. **Inverted skip condition**: `if (w > 0) continue` skipped pairs inside the kernel support (the ones that actually matter)
3. **Wrong REST_SUM**: calibrated for the broken kernel; after fixing, REST_SUM needed recalibration to 6.0

Consequence: all particles scattered to the four corners instantly because density was always near-zero (no close-pair contribution), giving infinite pressure repulsion.

### Non-Obvious Design Decisions
- **Why GCELL > SMOOTH_RADIUS?** The 3×3 cell neighbourhood only guarantees all neighbours within H if GCELL ≥ H. Setting GCELL = 3 > 2.2 = H gives a 1-cell safety margin.
- **Why symplectic Euler instead of RK4?** SPH with RK4 needs 4 density+force evaluations per step — 4× the cost. Symplectic Euler is nearly as stable and 4× faster.
- **Why the sign trick instead of separate repulsion/cohesion?** The `w(d) · (ρᵢ+ρⱼ−2ρ₀)` formula handles both in one expression. No if-statement needed for repel-vs-attract; the sign emerges from the physics.

### Comparison to Standard SPH (Müller 2003)
| | This implementation | Standard SPH |
|---|---|---|
| Kernel | One kernel for all forces | 3 kernels (Poly6, Spiky, Viscosity) |
| Density | Σ w(d)² | Σ m·W(d) (mass-weighted) |
| Pressure | Tait EOS implicit | Tait EOS with explicit no-tension clamp |
| Viscosity | Velocity damping | Laplacian viscosity term |
| Cohesion | Yes (negative pressure) | No (no-tension clamped to 0) |

Standard SPH suppresses cohesion (no negative pressure) for stability. This implementation allows it, producing surface tension-like behaviour.

### Open Questions to Explore
1. What happens if GCELL < SMOOTH_RADIUS? (missed neighbours → density too low → scattered particles)
2. How would you add surface tension as an explicit curvature force (Morris 2000)?
3. What is the CFL condition for SPH and how does it constrain SPH_DT given the particle velocities?
4. Why does symplectic Euler conserve energy approximately but regular Euler does not?
5. How would you implement an incompressible SPH variant (PCISPH or DFSPH)?

---

## Pass 2 — Implementation

### Pseudocode

```
// Per step:
grid_build()     — hash every particle into cell, build linked lists
sph_density()    — for each particle i: ρᵢ = Σⱼ∈neighbours w(dᵢⱼ)
sph_forces()     — for each pair (i,j): accumulate pressure + viscosity ax/ay
sph_integrate()  — for each particle: v += a·dt; x += v·dt; wall_bounce()
```

### Module Map
```
§1 config      — all physics constants, MAX_PARTICLES, scenes
§2 clock       — CLOCK_MONOTONIC fixed-timestep accumulator
§3 color       — 8 themes, transparent background
§4 sph         — kernel, density, forces, integrate, spatial grid
§5 scene       — 5 scene loaders (blob, rect, fountain, collision, rain)
§6 draw        — particle → terminal cell rendering
§7 app         — input loop, sim Hz control, pause/reset
```

### Data Flow
```
scene_load()
    │
    ▼
loop {
    grid_build()   O(N)
    sph_density()  O(N·k)
    sph_forces()   O(N·k)
    sph_integrate() O(N)
    draw()         O(N)
}
```

### Core Loop
```c
static void sph_step(void) {
    grid_build();
    sph_density();
    sph_forces();
    sph_integrate();
}

// In main loop (fixed-timestep accumulator):
while (accum >= SPH_DT) {
    if (!paused) sph_step();
    accum -= SPH_DT;
}
draw_particles();
```

### Kernel (the critical function)
```c
static double sph_kernel(double d) {
    double w = d / SMOOTH_RADIUS - 1.0;
    return (w < 0.0) ? w * w : 0.0;  // non-zero only for d < H
}
```
`w` is in range `[−1, 0)` for `d ∈ [0, H)`. `w²` gives a smooth bell peak at d=0.
