# Concept: 3D Boids (Flocking with Perspective Projection)

## Pass 1 — Understanding

### Core Idea
Extend Reynolds' boid flocking algorithm to 3D. Each boid has a 3D position and velocity. The three rules (separation, alignment, cohesion) operate in 3D space. Render to the 2D terminal using perspective projection.

### Mental Model
A flock of starlings (murmuration) moves in three dimensions. From any viewing angle you see complex swirling patterns. The same local rules that make 2D boids flock work in 3D — the math just has a z component.

### Key Equations
Same as 2D boids but with (x,y,z) vectors:
```
separation: F = Σ (pos_i - pos_j) / |pos_i - pos_j|²  for nearby j
alignment:  F = avg(vel_j) for neighbors j
cohesion:   F = avg(pos_j) - pos_i for neighbors j
```

Perspective projection:
```
fov = 90°
screen_x = cx + (x - cam_x) * fov / (z - cam_z)
screen_y = cy + (y - cam_y) * fov / (z - cam_z)
```
Cull boids with z ≤ cam_z + NEAR.

### Data Structures
- Boid array: {x,y,z, vx,vy,vz}
- Camera: {cx,cy,cz, look_at, up}
- Depth sort: render back-to-front for correct overlap

### Non-Obvious Decisions
- **Speed limits in 3D**: Clamp 3D velocity magnitude: `speed = |v|; if speed > MAX: v *= MAX/speed`.
- **Depth-sorted rendering**: Draw farthest boids first. Without this, nearer boids get overwritten by farther ones.
- **Boid size by depth**: Scale the character by inverse depth — closer boids appear larger. Use `'@'` for close, `'o'` for medium, `'.'` for far.
- **Camera orbit**: Slowly rotate the camera around the flock center for a cinematic view.
- **3D boundaries**: Either spherical boundary (boids steered back toward center when too far) or cubic box with bounce.

### Key Constants
| Name | Role |
|------|------|
| N | number of boids |
| W_SEP, W_ALG, W_COH | rule weights |
| SEP_DIST, VIS_DIST | separation and visibility radii |
| FOV | field of view for projection |
| MAX_SPEED | maximum boid speed |

### Open Questions
- Do 3D boids produce the same "split and merge" behavior as 2D?
- Try adding a 3D predator (flee force) — how does the flock evade in 3D?
- Can you project onto a tilted camera plane (not just XY aligned)?

---

## Pass 2 — Implementation

### Pseudocode
```
boid_steer(i, boids):
    sep = (0,0,0); align = (0,0,0); coh = (0,0,0)
    n_sep=0, n_vis=0
    for j != i:
        dist = |pos_i - pos_j|
        if dist < SEP_DIST:
            sep += (pos_i - pos_j) / dist; n_sep++
        if dist < VIS_DIST:
            align += vel_j; coh += pos_j; n_vis++
    if n_sep>0: sep /= n_sep
    if n_vis>0: align /= n_vis; coh = coh/n_vis - pos_i
    vel_i += W_SEP*sep + W_ALG*align + W_COH*coh
    clamp_speed(vel_i)
    boundary_steer(vel_i, pos_i)

project(pos, cam) → (col, row, depth):
    dx=pos.x-cam.x, dy=pos.y-cam.y, dz=pos.z-cam.z
    if dz < NEAR: return OFFSCREEN
    col = cx + dx*FOV/dz
    row = cy - dy*FOV/dz    # y flipped
    return col, row, dz

render(boids, cam):
    sorted = sort_by_depth_descending(boids)
    for boid in sorted:
        col,row,depth = project(boid.pos, cam)
        if offscreen: continue
        size_char = '@' if depth<30 else 'o' if depth<60 else '.'
        direction_char = vel_to_char(boid.vel, cam)
        mvaddch(row, col, direction_char)
```

### Module Map
```
§1 config    — N, weights, radii, FOV, CAM_DIST
§2 physics   — boid_steer(), integrate
§3 project   — project(), depth_sort()
§4 draw      — render() back-to-front with size-by-depth
§5 app       — main loop, camera orbit, keys
```

### Data Flow
```
boids (3D) → steer (3D forces) → new positions
→ depth sort → perspective project → 2D screen coords → draw
```
