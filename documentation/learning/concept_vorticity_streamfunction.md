# Concept: Vorticity-Streamfunction (VSF) Navier-Stokes Solver

## Pass 1 — Understanding

### Core Idea
Solve 2-D incompressible Navier-Stokes by eliminating pressure entirely. Taking the curl of momentum leaves two scalar PDEs: a transport equation for vorticity ω and a Poisson equation for streamfunction ψ. Velocity is recovered as u=∂ψ/∂y, v=−∂ψ/∂x.

### Mental Model
Think of ω as "spin density" — how fast fluid rotates at each point. ψ is a potential whose contour lines are streamlines (particles follow them). The lid of the cavity drags fluid, injecting vorticity at the top wall. That vorticity is transported inward by convection and smoothed out by viscosity. The Poisson solver recomputes ψ from ω at every step, then velocity is read off as derivatives of ψ.

### Key Equations
```
∂ω/∂t + u·∂ω/∂x + v·∂ω/∂y = ν·∇²ω   (vorticity transport)
∇²ψ = −ω                                  (Poisson equation)
u = ∂ψ/∂y,   v = −∂ψ/∂x               (velocity from ψ)
```

### Data Structures
- `omega[N][N]`: vorticity field
- `psi[N][N]`: streamfunction field
- `u[N][N]`, `v[N][N]`: velocity components (derived each step)
- `omega_new[N][N]`: scratch buffer for time advance

### Non-Obvious Decisions
- **SOR not Gauss-Seidel**: Optimal relaxation ω_sor=2/(1+sin(π/(N+1))) gives convergence ~20× faster. Cost: one formula at startup, same inner loop.
- **Upwind for convection, central for diffusion**: Upwind adds just enough numerical diffusion to stabilise the convection term at high Re. Central differences for diffusion are fine because diffusion is linearly stable.
- **Thom (1933) wall vorticity**: Cannot set ω=0 at walls (wrong physics). Thom's formula derives ω_wall from ∇²ψ=−ω evaluated at the boundary via a one-sided finite difference.
- **Adaptive dt**: Both convection CFL and diffusion von Neumann limits tighten as Re increases. Computing both and taking the minimum keeps the simulation stable automatically.
- **Dual-panel display**: Streamfunction contours (particle paths) on one side, vorticity magnitude/sign on the other — together they tell the complete story.

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| N | 64 | grid resolution |
| RE_DEFAULT | 400 | Reynolds number U·L/ν |
| SOR_OMEGA | 2/(1+sin(π/N)) | SOR relaxation factor |
| SOR_ITER | 50 | Poisson solver iterations per step |
| DT_SAFETY | 0.4 | fraction of stability limit |

### Open Questions
- What happens visually when Re exceeds ~1000 in this formulation?
- Why does the SOR_OMEGA formula assume Dirichlet boundaries?
- Why is the corner vorticity at re-entrant corners singular?

## From the Source

**Algorithm:** Vorticity-streamfunction formulation with SOR Poisson solver and Thom wall-BC. Finite-difference time marching: Forward Euler in time, upwind for convection, central for diffusion.

**Physics/References:** 2-D incompressible Navier-Stokes reduced to scalar ω-ψ system. Benchmark: lid-driven cavity (Ghia et al. 1982 — standard validation dataset). SOR optimal ω from Young (1954). Thom wall-vorticity formula from Thom (1933).

**Math:** SOR convergence rate ρ = ω_sor − 1 ≈ 1 − 2π/N. At N=64: ρ≈0.906, need ~25 iterations for 10⁻⁶ residual. Convection CFL: dt ≤ dx/|u_max|. Diffusion stability: dt ≤ dx²/(4ν).

**Performance:** O(N² · SOR_ITER) per step. N=64, SOR_ITER=50: ~200k FLOPs/step, comfortable at 30 fps. Larger grids require more SOR iterations for convergence (SOR_ITER ∝ N).

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `omega[N][N]` | `float[N²]` | ~16 KB | vorticity field |
| `psi[N][N]` | `float[N²]` | ~16 KB | streamfunction |
| `omega_new[N][N]` | `float[N²]` | ~16 KB | scratch for time advance |
| `u[N][N]` | `float[N²]` | ~16 KB | x-velocity (derived) |
| `v[N][N]` | `float[N²]` | ~16 KB | y-velocity (derived) |

---

## Pass 2 — Implementation

### Pseudocode
```
vsf_step():
    # 1. SOR: solve ∇²ψ = −ω
    for SOR_ITER:
        for i,j (interior):
            psi[i,j] = (omega[i,j] + psi[i-1,j] + psi[i+1,j]
                        + psi[i,j-1] + psi[i,j+1]) / 4   (for dx=dy)
            # apply SOR extrapolation
            psi[i,j] = psi_old + SOR_OMEGA * (psi_new - psi_old)
        apply_bc_psi()  # ψ=0 on all walls

    # 2. Recover velocity from ψ
    for i,j: u[i,j] =  (psi[i+1,j] - psi[i-1,j]) / (2*dy)
    for i,j: v[i,j] = -(psi[i,j+1] - psi[i,j-1]) / (2*dx)

    # 3. Advance vorticity (upwind convection + central diffusion)
    for i,j (interior):
        conv = upwind(u[i,j], omega, i, j, 'x') + upwind(v[i,j], omega, i, j, 'y')
        diff = (omega[i+1,j] + omega[i-1,j] + omega[i,j+1] + omega[i,j-1]
                - 4*omega[i,j]) / dx²
        omega_new[i,j] = omega[i,j] + dt*(nu*diff - conv)

    # 4. Thom wall vorticity
    apply_bc_omega()  # Thom formula on all 4 walls

    swap(omega, omega_new)
```

### Module Map
```
§1 config      — N, RE, SOR_OMEGA, SOR_ITER, DT_SAFETY
§2 grid alloc  — omega, psi, u, v, omega_new
§3 BC          — apply_bc_psi(), apply_bc_omega() with Thom formula
§4 Poisson     — solve_poisson() SOR loop
§5 velocity    — compute_velocity() central differences of ψ
§6 transport   — update_vorticity() upwind+central FD
§7 adaptive dt — compute_dt() CFL+von Neumann
§8 render      — dual panel: ψ contours + ω colormap
§9 app         — main loop, Re presets, keys
```

### Data Flow
```
omega → solve_poisson → psi → compute_velocity → u, v
u, v + omega → update_vorticity → omega_new → swap → omega
omega → render (vorticity panel)
psi   → render (streamfunction panel)
```
