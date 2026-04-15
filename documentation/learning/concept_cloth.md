# Concept: Cloth Simulation (Spring-Mass)

## Pass 1 — Understanding

### Core Idea
A grid of point masses connected by springs. Structural springs resist stretching, shear springs resist diagonal deformation, and bend springs resist folding. The cloth hangs, falls, and drapes under gravity.

### Mental Model
Think of a fishing net. Each knot is a mass. Each line of the net is a spring. When you hold the top row fixed and let gravity pull the rest, it sags and swings like real fabric.

### Key Equations
Hooke's law spring force:
```
F = k · (|r| − rest_length) · r̂
```
where r = displacement between two nodes, r̂ = unit vector.

Symplectic Euler (better energy conservation than explicit Euler):
```
v += (F/m + g) · dt
x += v · dt
```

### Data Structures
- Node grid [ROWS×COLS]: {x, y, vx, vy, pinned}
- Spring list: {node_a, node_b, rest_len, stiffness}
- Spring types: structural (distance 1), shear (diagonal √2), bend (distance 2)

### Non-Obvious Decisions
- **Symplectic Euler**: Update velocity first, then position using the NEW velocity. This conserves energy much better than updating position with the old velocity.
- **Multiple constraint iterations**: Run the spring solve 5–10 times per frame (Gauss-Seidel style) to reduce stretching without needing a tiny timestep.
- **Pinned nodes**: Top row has pinned=true; skip force integration for those.
- **Damping**: Add `v *= (1 - DAMPING)` per step to prevent oscillation from growing.

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| K_STRUCT | 500–2000 | structural stiffness |
| K_SHEAR | 200–500 | shear stiffness |
| K_BEND | 50–200 | bend stiffness |
| DAMPING | 0.98–0.995 | velocity damping |
| ITERATIONS | 5–15 | constraint solve passes |

### Open Questions
- What happens when you cut a spring mid-simulation? (tear simulation)
- How does the cloth change with gravity pointing sideways?
- Why does increasing stiffness without reducing dt cause explosion?

## From the Source

**Algorithm:** Explicit spring-mass simulation with symplectic Euler. "Symplectic" means velocity is updated before position: `vel_new = vel + F/m · dt` (velocity first), then `pos_new = pos + vel_new · dt` (use new velocity). This preserves the Hamiltonian structure — no long-term energy drift unlike pure explicit Euler where energy grows unboundedly without damping.

**Physics:** Hooke's law spring force + relative-velocity damping. Three spring types build in the mechanical response: Structural (resists stretching/compression, strong), Shear (resists diagonal deformation, medium), Bend (resists out-of-plane bending, weak). Without bend springs, cloth wrinkles freely; with strong bend springs it becomes stiff fabric.

**Performance:** Symplectic Euler stability bound: k·dt² < 2. At sub-step dt ≈ 2 ms, max k ≈ 462000 px/s² — all spring constants are far below this limit. SUB_STEPS = 8 divides each frame into 8 mini-steps. Cost: O(N_SPRINGS) per sub-step = O(CLOTH_W × CLOTH_H × 6) per frame. At 30×18 nodes ≈ 3240 spring force evals/frame.

---

## Pass 2 — Implementation

### Pseudocode
```
build_springs():
    for each node (i,j):
        add structural spring to (i+1,j) and (i,j+1)
        add shear spring to (i+1,j+1) and (i+1,j-1)
        add bend spring to (i+2,j) and (i,j+2)

apply_spring_forces():
    for each spring (a,b):
        dx=b.x-a.x, dy=b.y-a.y
        dist = sqrt(dx²+dy²)
        stretch = dist - rest_len
        force = K * stretch
        fx = force * dx/dist
        fy = force * dy/dist
        a.fx+=fx; a.fy+=fy
        b.fx-=fx; b.fy-=fy

integrate():
    for each node (not pinned):
        vx = (vx + (fx/m + gx)*dt) * DAMP
        vy = (vy + (fy/m + gy)*dt) * DAMP
        x += vx*dt
        y += vy*dt
```

### Module Map
```
§1 config    — ROWS, COLS, spacing, K constants, DAMPING
§2 build     — allocate nodes, build spring list
§3 physics   — apply_forces(), integrate(), wind()
§4 draw      — draw springs as lines, nodes as dots
§5 app       — main loop, keys (wind, gravity dir, cut, reset)
```

### Data Flow
```
pinned top row → spring forces → symplectic Euler → new positions
positions → draw springs (interpolated chars) → screen
```

### Core Loop
```c
for each frame:
    zero forces on all nodes
    apply gravity
    apply spring forces
    integrate (skip pinned)
    apply wind force
    erase()
    draw springs (interpolate | / \ between nodes)
    draw HUD
    refresh
```
