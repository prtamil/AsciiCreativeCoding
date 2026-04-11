# Concept: Three-Body Problem

## Pass 1 — Understanding

### Core Idea
Three gravitationally interacting bodies. Unlike the two-body problem, no general analytic solution exists. Most initial conditions lead to chaotic trajectories. Special initial conditions (figure-8, Lagrange) give stable periodic orbits.

### Mental Model
Three stars in space pulling each other. Two stars can form a stable binary while the third gets slung around wildly, or all three can orbit in a perfectly choreographed figure-8. The sensitive dependence on initial conditions means tiny changes lead to completely different outcomes.

### Key Equations
Same as N-body with N=3:
```
a_i = Σ_{j≠i} G·m_j·(r_j − r_i) / |r_j − r_i|³
```

Figure-8 initial conditions (Chenciner & Montgomery, 2000):
```
m1=m2=m3=1
x1=−x3=0.9700436, y1=−y3=−0.24308753
x2=0, y2=0
vx1=vx3=0.93240737/2, vy1=vy3=0.86473146/2
vx2=−0.93240737, vy2=−0.86473146
```

### Data Structures
- 3 bodies: {x, y, vx, vy, mass}
- Trail ring buffers (one per body)
- RK4 or Velocity Verlet integrator

### Non-Obvious Decisions
- **RK4 is essential**: Velocity Verlet works for conservative forces but loses the figure-8 orbit quickly. RK4 with small dt preserves the orbit for hundreds of periods.
- **Center of mass correction**: Drift the simulation in the CM frame so it stays on screen.
- **Softening**: Small ε prevents explosion on close approach but distorts the exact figure-8. Use ε≈0.001 for near-exact results.
- **Preset menu**: Show the figure-8, a chaotic preset, and the Lagrange L4/L5 preset so the user can explore.

### Key Constants
| Name | Role |
|------|------|
| G | gravitational constant |
| EPS | softening (near-zero for figure-8) |
| DT | timestep (0.0001–0.001 for accuracy) |
| TRAIL_LEN | history length per body |

### Open Questions
- The figure-8 is unstable — how many periods does your integrator maintain it?
- What are the five Lagrange points and which are stable?
- Try 4 bodies with figure-8 IC — what happens?

---

## Pass 2 — Implementation

### Pseudocode
```
struct Body { double x, y, vx, vy, mass; }

accel(bodies[3], i) → (ax, ay):
    ax=ay=0
    for j != i:
        dx=bodies[j].x-bodies[i].x
        dy=bodies[j].y-bodies[i].y
        r3 = pow(dx²+dy²+EPS², 1.5)
        ax += G*bodies[j].mass*dx/r3
        ay += G*bodies[j].mass*dy/r3

rk4_step(bodies, dt):
    # store k1..k4 for all 3 bodies simultaneously
    k1[i] = {vx[i], vy[i], accel(bodies,i)}
    bodies2 = bodies + dt/2 * k1
    k2[i] = {vx2[i], vy2[i], accel(bodies2,i)}
    ... (k3, k4 similarly)
    bodies[i] += dt/6*(k1+2k2+2k3+k4)
```

### Module Map
```
§1 config    — G, EPS, DT, TRAIL_LEN, presets[]
§2 physics   — accel(), rk4_step()
§3 draw      — trails (colored per body), bodies (bold)
§4 app       — main loop, keys (preset select, speed, pause)
```

### Data Flow
```
preset IC → rk4_step × SUBSTEPS per frame → trails
bodies + trails → world→screen transform → draw
```

### Core Loop
```c
for each frame:
    for SUBSTEPS:
        rk4_step(bodies, dt)
        center_of_mass_correct(bodies)
        for each body: trail_push(trail[i], bodies[i])
    erase()
    for each body: draw_trail(trail[i], color[i])
    for each body: draw_body(bodies[i])
    draw_HUD()
    refresh
```
