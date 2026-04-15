# Concept: Shepherd (User-Controlled Herding)

## Pass 1 — Understanding

### Core Idea
Extend the Reynolds boids flocking model with a user-controlled shepherd (`#`). Sheep flock with the standard separation/alignment/cohesion rules, but also flee from the shepherd. The user steers the shepherd with arrow keys to herd the sheep flock.

### Mental Model
A sheepdog herding sheep. The dog runs around the edge of the flock, and the sheep flee the dog while also staying together. A skilled handler uses the dog to funnel the flock through a gate. The sheep's flock instinct fights the flee instinct — this tension is what makes herding interesting.

### Key Equations
Sheep acceleration (5 forces):
```
F_total = W_SEP·sep + W_ALG·align + W_COH·coh + W_FLEE·flee + W_BOUND·bound
```

Flee force from shepherd:
```
F_flee = (pos_sheep - pos_shepherd) / |pos_sheep - pos_shepherd|²
```
Panic boost: multiply flee weight by 3 if shepherd is within PANIC_RADIUS.

Shepherd movement: direct velocity control via arrow keys (no physics, instant response).

### Data Structures
- `Sheep[N]`: {px, py, vx, vy, cruise_speed, fleeing}
- `Shepherd`: {px, py, speed}
- No spatial grid needed for N<100 (O(N²) neighbor search)

### Non-Obvious Decisions
- **Flee force with inverse-square falloff**: `F ∝ 1/r²` means nearby shepherd causes overwhelming panic while distant shepherd is barely felt. Linear would make sheep flee from across the entire arena.
- **Bounce boundaries not wrap**: Sheep bounce off walls (reflect velocity component). This makes sheep cornerable — essential for herding. Toroidal wrap would let sheep escape through walls.
- **Panic zone**: Double flee weight inside PANIC_RADIUS. Sheep sprint when the shepherd gets close. Creates realistic "flushing" behavior.
- **Flee radius visualization**: Draw a dotted ring (toggle with 'f' key) showing the shepherd's influence range. Helps the user learn the effective herding distance.
- **Sheep character**: Calm=`o`, moving (directional)=`<>^v`, fleeing=`O`. Quick visual feedback on individual sheep state.

### From the Source

**Algorithm:** Sheep use Classic Boids (separation + alignment + cohesion) plus a flee force when shepherd is within `FLEE_RADIUS=180 px`. Flee force uses inverse-distance weighting — `F ∝ 1/r` from the C source (`flee_dir / sdist`), not inverse-square as in the concept. Panic boost at `PANIC_RADIUS=70 px` (not 60 px as in concept). `W_FLEE=7.0` — must dominate cohesion so sheep actually scatter.

**Physics:** Sheep have two speed modes: cruise (`SHEEP_SPEED=200 px/s`, ±25% variation) and flee (`SHEEP_SPEED_FLEE ≈ 1.5× cruise`). Shepherd moves at `SHEPHERD_SPEED=320 px/s` — faster than sheep so the user can catch them. Both run in isotropic pixel space `CELL_W=8, CELL_H=16`. Sheep bounce off walls (`BOUNCE_DAMP=0.6` velocity damping), not wrap — makes sheep cornerable for herding.

**Rendering:** Sheep character selected from 8-direction glyph table based on velocity heading (`o < > ^ v / \`); bold+red when fleeing (`O`). Flee-radius ring: optional dashed circle drawn around shepherd with `'·'` characters at `FLEE_RADIUS/CELL_W` column units, scaled by `CELL_W/CELL_H` in the row direction for correct aspect ratio.

### Key Constants
| Name | Value | Role |
|------|-------|------|
| FLEE_RADIUS | 180 px | shepherd's influence range |
| PANIC_RADIUS | 60 px | panic boost distance |
| W_FLEE | 7.0 | flee force weight (dominates cohesion) |
| W_SEP | 2.5 | separation weight |
| W_COH | 1.0 | cohesion weight |
| SHEPHERD_SPEED | ~80 px/s | shepherd movement speed |

### Open Questions
- What is the optimal herding strategy? (approach from behind, sweep wide)
- Can you add a pen/target area that counts sheep that enter it?
- Add a second shepherd — how does co-operative herding work?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_sheep[N_SHEEP_MAX]` | `Sheep[60]` | ~1.4 KB | sheep boid pool |
| `g_shep` | `Shepherd` | 20 B | player-controlled shepherd position |

## Pass 2 — Implementation

### Pseudocode
```
sheep_steer(sheep_i, all_sheep, shepherd):
    sep = align = coh = (0,0)
    n_sep=0, n_vis=0
    for j != i:
        dist = |pos_i - pos_j|
        if dist < SEP_DIST: sep += (pos_i-pos_j)/dist; n_sep++
        if dist < VIS_DIST: align += vel_j; coh += pos_j; n_vis++
    if n_sep>0: sep /= n_sep
    if n_vis>0: align /= n_vis; coh = coh/n_vis - pos_i

    sdist = |pos_i - shepherd.pos|
    if sdist < FLEE_RADIUS:
        flee_dir = (pos_i - shepherd.pos) / sdist
        flee_w = W_FLEE * (3.0 if sdist < PANIC_RADIUS else 1.0)
        flee = flee_dir / sdist    # inverse distance
    else:
        flee = (0,0); flee_w = 0

    accel = W_SEP*sep + W_ALG*align + W_COH*coh + flee_w*flee + boundary_force()
    vel_i += accel * dt
    clamp_speed(vel_i)
    pos_i += vel_i * dt
    bounce_walls(pos_i, vel_i)

draw_shepherd_ring(shepherd, show_ring):
    mvaddch(shepherd.row, shepherd.col, '#')
    if show_ring:
        r_ring = FLEE_RADIUS / CELL_W
        for angle in 0..64: 
            c = shepherd.col + round(cos(angle*PI/32) * r_ring)
            r = shepherd.row + round(sin(angle*PI/32) * r_ring * CELL_W/CELL_H)
            mvaddch(r, c, '·')

main loop:
    handle arrows → shepherd velocity → clamp to arena
    update all sheep (steer + integrate + bounce)
    erase(); draw sheep; draw shepherd + ring; HUD
    refresh
```

### Module Map
```
§1 config    — N_SHEEP, FLEE_RADIUS, PANIC_RADIUS, weights
§2 boids     — sheep_steer() (5 forces)
§3 shepherd  — arrow key input → shepherd movement
§4 draw      — sheep chars (state-dependent), shepherd '#', ring
§5 app       — main loop, keys (arrows, +/- sheep, f ring, r reset)
```

### Data Flow
```
arrow keys → shepherd position
sheep[N] → boid forces + flee from shepherd → new velocities
bounce walls → new positions → draw
```
