# Concept: Elastic Collision (Billiards)

## Pass 1 — Understanding

### Core Idea
Billiard balls collide and exchange momentum. In a perfectly elastic collision both momentum and kinetic energy are conserved. The simulation shows discs bouncing off each other and the walls.

### Mental Model
Pool balls on a table. When two balls hit head-on, the moving ball stops and the stationary one takes off at the same speed. At an angle, each gets a component of the impact. The geometry is all about the collision normal (line through centers).

### Key Equations
Along the collision normal n̂:
```
v1' = v1 - 2m2/(m1+m2) · [(v1-v2)·n̂] · n̂
v2' = v2 + 2m1/(m1+m2) · [(v1-v2)·n̂] · n̂
```
For equal masses this simplifies to exchanging normal components.

Penetration resolution:
```
overlap = (r1 + r2) - distance
separate each ball by overlap/2 along n̂
```

### Data Structures
- Ball array: {px, py, vx, vy, radius, mass, color}
- No spatial grid needed for small N; O(N²) collision check

### Non-Obvious Decisions
- **Separate before resolving velocity**: If you resolve velocity without moving balls apart, they can remain overlapping and "tunnel" through each other next frame.
- **Check relative velocity dot normal**: Only resolve if balls are approaching (dot product < 0). Otherwise you'd reverse balls that are already separating.
- **Wall bounce**: Reverse the perpendicular velocity component. Add slight restitution coefficient for inelastic option.
- **Cell aspect ratio**: Terminal cells are not square. Scale physics radii by CELL_W/CELL_H to get correct circle collision even though glyphs are rectangular.

### Key Constants
| Name | Role |
|------|------|
| RESTITUTION | 1.0 = perfectly elastic, <1 = some energy loss |
| RADIUS | collision radius in physics units |
| CELL_W, CELL_H | pixel dimensions of one terminal cell |

### Open Questions
- At what angle does a ball scatter 90° from the impact direction?
- Add a "heavy ball" with 10× mass — what changes?
- Show the collision normal vector as a drawn line during impact

## From the Source

**Algorithm:** Impulse-based elastic collision resolution. For each overlapping pair, a single impulse along the collision normal simultaneously adjusts both velocities so the constraint (non-penetration + elastic restitution) is satisfied in one step.

**Physics:** Conservation laws for elastic collisions: (1) Conservation of momentum: m₁v₁ + m₂v₂ = const; (2) Conservation of kinetic energy: ½m₁v₁² + ½m₂v₂² = const. Combined, for collision along normal n̂: `impulse J = 2·m₁·m₂/(m₁+m₂) · Δv·n̂`. Velocities updated: `v₁ -= J/m₁·n̂`; `v₂ += J/m₂·n̂`.

**Performance:** O(N²) pair checks per tick — acceptable for N=25. For N>100 broad-phase (spatial hash or sweep-and-prune) would reduce to O(N) average checks.

**Physics:** Mass model: mass = r² (area of 2D disc × uniform density). Heavier discs (larger radius) deflect smaller ones more, matching intuition about billiard balls.

---

## Pass 2 — Implementation

### Pseudocode
```
resolve_collision(a, b):
    dx = b.px - a.px
    dy = b.py - a.py
    dist = sqrt(dx² + dy²)
    if dist >= a.r + b.r: return    # not touching
    if dist < 0.001: dist = 0.001   # guard divide-by-zero

    nx = dx/dist, ny = dy/dist      # collision normal
    rel_vx = a.vx - b.vx
    rel_vy = a.vy - b.vy
    dot = rel_vx*nx + rel_vy*ny

    if dot >= 0: return             # already separating

    j = -2 * dot / (1/a.m + 1/b.m) * RESTITUTION
    a.vx += j/a.m * nx;  a.vy += j/a.m * ny
    b.vx -= j/b.m * nx;  b.vy -= j/b.m * ny

    # push apart
    overlap = (a.r + b.r - dist) / 2
    a.px -= overlap*nx;  a.py -= overlap*ny
    b.px += overlap*nx;  b.py += overlap*ny

wall_bounce(ball):
    if ball.px < ball.r: ball.px = ball.r; ball.vx *= -RESTITUTION
    ...
```

### Module Map
```
§1 config    — N_BALLS, RADIUS, RESTITUTION, CELL_W/H
§2 init      — random or grid positions, random velocities
§3 physics   — move(), wall_bounce(), all-pairs collision
§4 draw      — disc as colored 'O', velocity vector optional
§5 app       — main loop, keys (add/remove balls, reset)
```

### Data Flow
```
positions + velocities → move (Euler) → wall_bounce
→ all-pairs collision → new positions/velocities → draw
```

### Core Loop
```c
for each frame:
    for SUBSTEPS:
        for each ball: move(dt/SUBSTEPS); wall_bounce()
        for i in balls: for j in i+1..balls: resolve_collision(i,j)
    erase()
    draw_balls()
    draw_HUD()
    refresh
```
