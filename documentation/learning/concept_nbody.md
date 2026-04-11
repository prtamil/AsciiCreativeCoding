# Concept: N-Body Gravity Simulation

## Pass 1 — Understanding

### Core Idea
N point masses attract each other via Newton's law F=Gm₁m₂/r². Each body's acceleration is the sum of forces from all others. With 3+ bodies the system is chaotic — no closed-form solution exists.

### Mental Model
Every planet pulls every other planet. A small third body near two massive ones gets flung around unpredictably. The art is in choosing initial conditions that produce beautiful orbits (figure-8, lagrange points) vs. immediate ejection.

### Key Equations
```
F_ij = G·m_i·m_j / (r_ij² + ε²)   ← softening ε prevents 1/0
a_i  = Σ_{j≠i} F_ij / m_i
```

Velocity Verlet integration:
```
v_half = v + a·dt/2
x      = x + v_half·dt
recompute a
v      = v_half + a·dt/2
```

### Data Structures
- Body array: {x, y, vx, vy, mass, trail[]}
- Acceleration buffer: ax[], ay[] recomputed each step
- Trail ring buffer per body

### Non-Obvious Decisions
- **Softening ε**: Without it two bodies can get arbitrarily close → infinite force → explosion. ε≈0.5–2.0 in display units.
- **Velocity Verlet over Euler**: Conserves energy far better; Euler accumulates error and orbits spiral outward.
- **Center-of-mass frame**: Subtract mean velocity so the system stays centered on screen.
- **O(N²)**: Fine for N<100; for larger N use Barnes-Hut tree.

### Key Constants
| Name | Role |
|------|------|
| G | gravitational constant (tune to screen scale) |
| ε (softening) | prevents singularity at close approach |
| dt | integration timestep |
| TRAIL_LEN | how many past positions to show |

### Open Questions
- What are the Lagrange points? Can you visualize L4/L5?
- Try figure-8 initial conditions: three equal masses
- What happens when you add a fourth body to a stable three-body orbit?

---

## Pass 2 — Implementation

### Pseudocode
```
compute_forces():
    zero all ax[], ay[]
    for i in 0..N:
        for j in i+1..N:
            dx = x[j]-x[i], dy = y[j]-y[i]
            r2 = dx²+dy² + EPS²
            r  = sqrt(r2)
            f  = G*m[i]*m[j] / r2
            fx = f*dx/r, fy = f*dy/r
            ax[i] += fx/m[i];  ax[j] -= fx/m[j]
            ay[i] += fy/m[i];  ay[j] -= fy/m[j]

verlet_step():
    vx += ax*dt/2; vy += ay*dt/2   # half kick
    x  += vx*dt;   y  += vy*dt    # drift
    compute_forces()
    vx += ax*dt/2; vy += ay*dt/2   # half kick
```

### Module Map
```
§1 config    — N, G, EPS, DT, TRAIL_LEN
§2 init      — preset initial conditions (solar, figure-8, random)
§3 physics   — compute_forces(), verlet_step()
§4 draw      — trails (dim), bodies (bold+mass-coded char)
§5 app       — main loop, keys (preset, speed, pause)
```

### Data Flow
```
initial conditions → verlet_step × SUBSTEPS → trail append
bodies + trails → world→screen → draw
```

### Core Loop
```c
for each frame:
    for SUBSTEPS:
        verlet_step()
        for each body: trail_push(body)
    center_of_mass_correct()
    erase()
    draw_trails()
    draw_bodies()
    draw_HUD()
    refresh
```
