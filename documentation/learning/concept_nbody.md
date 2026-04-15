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

## From the Source

**Algorithm:** Velocity Verlet (Störmer-Verlet) integration. 2nd-order symplectic integrator — conserves a modified Hamiltonian (energy drifts only by round-off, not by accumulating phase error like explicit Euler). Full formula: `x_new = x + v·dt + ½·a·dt²`, then `a_new = F(x_new)/m`, then `v_new = v + ½·(a + a_new)·dt`.

**Physics:** Softened Newtonian gravity. Real 1/r² force → singularity when r→0. Softening ε (SOFT2 = ε²) limits force magnitude at close approach, modelling extended mass distributions or preventing numerical blow-up. The softened force: `F = G·m₁·m₂·r / (r² + ε²)^(3/2)`.

**Performance:** O(N²) force computation each sub-step (brute force). For N≤32 this is fast; N-body above ~1000 would need Barnes-Hut O(N log N) or FMM O(N). SUB_STEPS=4 improves accuracy without changing display fps.

**Physics/References:** Figure-8 choreography (Chenciner & Montgomery, 2000). An exact periodic solution where 3 equal masses chase each other around a figure-8 curve. Initial conditions in natural units (G=1, m=1) are rescaled to pixel space using dimensional analysis. Per-body ring-buffer trails: 32 bodies × 150 × 8 B = 38 KB — fits easily in L1 cache for small N.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~40 KB | top-level container: scene (NBody) + screen + control flags |
| `g_app.scene.nbody` | `NBody` | ~38 KB | bodies array + state flags + preset |
| `NBody.bodies[MAX_BODIES]` | `Body[32]` | ~38 KB | position, velocity, acceleration, mass, trail ring-buffer per body |
| `Body.tx/ty[TRAIL_LEN]` | `float[150]` × 2 | 1.2 KB per body | per-body trail ring-buffer of pixel positions |

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
