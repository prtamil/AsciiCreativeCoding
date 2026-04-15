# Concept: Lorenz Attractor

## Pass 1 — Understanding

### Core Idea
Three coupled ODEs describe convection in a heated fluid layer. Solutions never repeat but stay bounded — a strange attractor. The trajectory traces a butterfly-shaped double-lobe in 3D phase space.

### Mental Model
Imagine stirring two bowls of soup connected by a thin tube. The system almost falls into a rhythm, then unpredictably flips which bowl it circulates. The "butterfly wings" are those two near-cycles.

### Key Equations
```
dx/dt = σ(y − x)
dy/dt = x(ρ − z) − y
dz/dt = xy − βz
```
Classic parameters: σ=10, ρ=28, β=8/3.

### Data Structures
- State vector: (x, y, z) as three floats
- History ring buffer: last N points for trail
- RK4: four slope evaluations k1..k4 per step

### Non-Obvious Decisions
- **RK4 not Euler**: Euler diverges quickly because the Lyapunov exponent is positive. RK4 stays on the attractor far longer.
- **Projection to 2D**: rotate the 3D attractor, then project — pick angles that show both lobes.
- **Normalization**: x,y,z range roughly ±20, must map to screen coords each frame.
- **Trail length**: long trails show the wing shape; short trails show current motion direction.

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| σ (sigma) | 10 | rate of convective mixing |
| ρ (rho) | 28 | temperature difference (chaos threshold) |
| β (beta) | 8/3 | geometric factor |
| dt | 0.005–0.01 | step size (too large → diverge) |

### Open Questions
- What happens when ρ < 24.74? (subcritical — no chaos)
- Try σ=14, ρ=28 — different wing geometry
- What is the Lyapunov exponent for classic parameters?

## From the Source

**Algorithm:** RK4 integration of a 3-variable ODE system. The Lorenz system is autonomous (no explicit time): `dx/dt = f(x,y,z)`, so each RK4 step uses only the current state, not wall-clock time. Step size L_H is fixed in Lorenz-time units regardless of display fps.

**Physics/References:** Originally derived by Edward Lorenz (1963) from a simplified model of atmospheric convection. σ (sigma) = Prandtl number (viscosity/thermal diffusivity). ρ (rho) = Rayleigh number ratio (buoyancy vs. diffusion). β (beta) = geometric factor for the convection cell. At σ=10, ρ=28, β=8/3 the system has a strange attractor: bounded but never repeating, with Lyapunov exponent λ ≈ 0.9.

**Math:** Orthographic projection with azimuth φ and elevation θ. The 3D point (x,y,z) is rotated around the z-axis by φ, then the (ry, z) plane is tilted by θ to produce screen coordinates. ASPECT compensates for non-square cell ratio.

**Data-structure:** Ring-buffer trail (TRAIL_LEN=2500 points) per trajectory. At 60 fps × 8 sub-steps = 480 points/s, 2500 ≈ 5.2 s of trajectory history — enough to show the full butterfly shape.

---

## Pass 2 — Implementation

### Pseudocode
```
lorenz_deriv(x,y,z) → (dx,dy,dz):
    dx = SIGMA*(y-x)
    dy = x*(RHO-z) - y
    dz = x*y - BETA*z

rk4_step(x,y,z, dt):
    k1 = lorenz_deriv(x, y, z)
    k2 = lorenz_deriv(x+dt/2*k1.x, ...)
    k3 = lorenz_deriv(x+dt/2*k2.x, ...)
    k4 = lorenz_deriv(x+dt*k3.x, ...)
    x += dt/6 * (k1.x + 2k2.x + 2k3.x + k4.x)
    (same for y, z)

project(x,y,z) → (col, row):
    # rotate around Y axis by angle θ
    rx = x*cos(θ) + z*sin(θ)
    ry = y
    col = cx + scale*rx
    row = cy - scale*ry   # flip y for screen
```

### Module Map
```
§1 config    — SIGMA, RHO, BETA, DT, TRAIL_LEN
§2 math      — lorenz_deriv(), rk4_step()
§3 project   — world→screen with rotation
§4 draw      — plot trail with density/color
§5 app       — main loop, keys (rotate, speed, reset)
```

### Data Flow
```
initial (x,y,z) → rk4_step × N per frame → ring buffer
ring buffer → project → screen coords → draw chars
```

### Core Loop
```c
for each frame:
    for STEPS_PER_FRAME:
        rk4_step(&state)
        ring_push(trail, state)
    erase()
    for each point in trail:
        (col,row) = project(point, rotation_angle)
        draw dot (older = dimmer)
    draw HUD
    refresh
```
