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
