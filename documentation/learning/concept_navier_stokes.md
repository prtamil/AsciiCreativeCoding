# Concept: Navier-Stokes (Jos Stam Stable Fluid)

## Pass 1 — Understanding

### Core Idea
Simulate incompressible fluid flow on a 2D grid using Jos Stam's "stable fluids" method. The key insight: split each timestep into diffuse → advect → project. Gauss-Seidel diffusion and semi-Lagrangian advection are unconditionally stable, meaning large timesteps don't explode.

### Mental Model
Imagine ink dropped into water. The ink (density field) spreads by diffusion, then is carried along by the water's velocity. The velocity itself is driven by forces (emitters, buoyancy), diffused (viscosity), advected (fluid carries itself), then projected to remove divergence (making it incompressible — fluid can't be created or destroyed).

### Key Equations
The four steps per frame:
```
1. Add forces:    v += f · dt
2. Diffuse:       (I - ν∇²) v_new = v  → Gauss-Seidel solve
3. Advect:        v(x,t+dt) = v(x - v·dt, t)  → bilinear sample
4. Project:       ∇·v = 0  → solve Poisson equation for pressure
```

Density field follows same diffuse+advect (no projection).

### Data Structures
- `vel_x[N×N]`, `vel_y[N×N]`: velocity components
- `dens[N×N]`: smoke/ink density
- `vel_x0`, `vel_y0`, `dens0`: previous-step buffers
- Boundary: `set_bnd()` mirrors values at edges

### Non-Obvious Decisions
- **Gauss-Seidel not direct solve**: A direct solve is expensive. Gauss-Seidel with 20 iterations gives a good enough approximation and is cache-friendly.
- **Semi-Lagrangian advection**: Trace a particle backwards in time `dt` to find where fluid came from, then bilinear-interpolate. Never unstable because you're sampling, not extrapolating.
- **Projection step**: After advection, velocity has divergence. Solve ∇²p = ∇·v (Poisson), then subtract ∇p from velocity. This enforces incompressibility.
- **Dynamic normalization**: Density values grow over time. Normalize by the maximum density each frame so the display never goes blank or saturates.
- **Pre-warm**: Run 80 steps with emitters before entering the main loop so the display starts with visible fluid.

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| N | 64–128 | grid resolution |
| VISC | 1e-6 | kinematic viscosity |
| DIFF | 1e-5 | density diffusion rate |
| DT | 0.1 | simulation timestep |
| ITER | 20 | Gauss-Seidel iterations |

### Open Questions
- Why does increasing ITER improve smoothness but never reach exact incompressibility?
- What visual effect does increasing VISC have?
- Why does the projection step need boundary handling?

## From the Source

**Algorithm:** Jos Stam's "Stable Fluids" (SIGGRAPH 1999). Operator-splitting approach: separately handles diffusion, projection (divergence removal), and advection. Each sub-step is unconditionally stable — no CFL limit.

**Physics/References:** Incompressible Navier-Stokes: ∂u/∂t = −(u·∇)u + ν∇²u + f (momentum); ∇·u = 0 (incompressibility). The projection step enforces ∇·u=0 via a Poisson solve for the pressure correction field. Kinematic viscosity reference: water ≈ 1e-6 m²/s; air ≈ 1.5e-5 m²/s; VISC_INIT=1e-5 in grid²/step units so vortices persist visually.

**Math:** Gauss-Seidel solves both diffusion and pressure Poisson systems. ITER=16 is enough for N=80: residual decays geometrically per pass. Semi-Lagrangian advection (back-trace + bilinear interp) is first-order accurate but unconditionally stable for large dt.

**Performance:** O(N²·ITER) per frame. At N=80, ITER=16: ~100k operations per step, trivial at 30 fps. Larger ITER → less "leaky" incompressibility, more CPU. Grid size N=80 fits most terminals; total cells = N²=6400.

---

## Pass 2 — Implementation

### Pseudocode
```
fluid_step():
    # velocity
    swap(vel_x, vel_x0)
    diffuse(1, vel_x, vel_x0, VISC, DT)
    swap(vel_y, vel_y0)
    diffuse(2, vel_y, vel_y0, VISC, DT)
    project(vel_x, vel_y, vel_x0, vel_y0)
    swap(vel_x, vel_x0); swap(vel_y, vel_y0)
    advect(1, vel_x, vel_x0, vel_x0, vel_y0, DT)
    advect(2, vel_y, vel_y0, vel_x0, vel_y0, DT)
    project(vel_x, vel_y, vel_x0, vel_y0)
    # density
    swap(dens, dens0)
    diffuse(0, dens, dens0, DIFF, DT)
    swap(dens, dens0)
    advect(0, dens, dens0, vel_x, vel_y, DT)

diffuse(b, x, x0, diff, dt):
    a = dt * diff * N²
    for ITER:
        for i,j: x[i,j] = (x0[i,j] + a*(x[i-1,j]+x[i+1,j]+x[i,j-1]+x[i,j+1])) / (1+4a)
    set_bnd(b, x)

advect(b, d, d0, u, v, dt):
    for i,j:
        x = i - dt*N*u[i,j];  y = j - dt*N*v[i,j]
        clamp x,y to [0.5, N+0.5]
        bilinear_sample d0 at (x,y) → d[i,j]
    set_bnd(b, d)

project(u, v, p, div):
    compute div[i,j] = -0.5*(u[i+1,j]-u[i-1,j]+v[i,j+1]-v[i,j-1])/N
    solve Gauss-Seidel: p[i,j] = (div[i,j]+p[i-1,j]+p[i+1,j]+p[i,j-1]+p[i,j+1])/4
    u[i,j] -= 0.5*N*(p[i+1,j]-p[i-1,j])
    v[i,j] -= 0.5*N*(p[i,j+1]-p[i,j-1])
```

### Module Map
```
§1 config     — N, VISC, DIFF, DT, ITER
§2 fluid core — diffuse(), advect(), project(), set_bnd()
§3 sources    — inject_sources() with rotating emitters
§4 draw       — density → ASCII chars with dynamic normalization
§5 app        — main loop, pre-warm, keys
```

### Data Flow
```
inject_sources → fluid_step (diffuse+advect+project)
→ dens[N×N] → normalize → density chars → screen
```
