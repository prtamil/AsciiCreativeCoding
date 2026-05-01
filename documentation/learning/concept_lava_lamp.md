# Concept: Magma Chamber / Lava Lamp

## Pass 1 — Understanding

### Core Idea
Slow viscous "lava" blobs drift inside a sealed chamber. Each blob is a **metaball** — a Gaussian-like scalar field centred on its position. The total field is the sum of every blob's contribution; cells where the field exceeds a threshold are rendered as molten material, with intensity (proximity to a blob centre) and average blob temperature jointly choosing the heat-ramp glyph. Blobs heat near the floor, cool near the ceiling — buoyancy drives a slow vertical oscillation. Blobs merge and split organically because the field sums freely.

### Mental Model
The scene is *defined by a scalar field*, not by drawing each blob individually. A blob doesn't have a fixed shape; it's just a centre + radius that contributes to the field. Two close blobs form a smooth dumbbell because their contributions add. The renderer walks every screen cell, evaluates `f(x, y)`, and emits a glyph if `f > threshold`. The cone-aspect-correction (`ASPECT_K2 = 4`) squashes the field vertically so the rendered blobs read as round on a 2:1 character grid.

### Key Equations
```
field at (x, y):
    f(x, y) = Σ_i  rᵢ² / ((x - xᵢ)² + (y - yᵢ)² · k² + ε²)
    where k = 2 (cell aspect) so y-distance is 4× more weighty

t_avg(x, y) = Σ_i wᵢ · tempᵢ / Σ_i wᵢ    where wᵢ = blob i's contribution

buoyancy:
    T_ambient(y) = 1 − y/(rows-2)        /* hot floor, cool ceiling */
    vy += -BUOYANCY · (temp − T_ambient(y)) · dt
    /* hot blob in cool zone → rises; cool blob in hot zone → sinks */
```

### Non-Obvious Decisions
- **Field threshold + interior gradient**: glyph is brighter near a blob's centre (high `f`) and dimmer at the edge (`f` close to threshold). Combined with `t_avg` (avg temperature) it gives both shape and heat colour.
- **Aspect-squashed field**: without `ASPECT_K2 = 4`, blobs render as ellipses (taller than wide). The squash makes them visually round.
- **Heat-ambient profile**: rather than a fixed gravity force, blobs respond to *temperature relative to ambient*. Hot blobs near the floor want to rise; same blob at the top finds T_ambient small and stops climbing.
- **Wall bounce halves velocity**: prevents bouncing chaos but lets blobs still rebound visually from the chamber walls.
- **Slow time scale (30 fps target, low buoyancy)**: a blob's full up-and-down cycle is ~6 seconds — meditative, not frenetic.

### Key Constants
| Name | Role |
|------|------|
| `N_BLOBS_DEFAULT` | 6 metaballs |
| `BLOB_RADIUS_MIN/MAX` | 2.5 / 5.0 cells |
| `THRESHOLD_DEFAULT` | 0.8 — field threshold for visible material |
| `BUOYANCY_DEFAULT` | 8.0 cells/sec² per unit (T - T_ambient) |
| `ASPECT_K2` | 4.0 (= 2² — cell aspect squared) |
| `HEAT_GAIN / HEAT_LOSS` | 0.30 / 0.20 temp units/sec |

### Open Questions
- What if blobs had *charge* and attracted/repelled each other?
- Could the chamber walls be soft and deform when blobs press against them?
- How does the field behave with negative-radius blobs (subtractive)? Hollow lava?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — N_BLOBS, threshold, buoyancy, dynamics
§5 blob      — Blob struct + spawn / tick (buoyancy + walls)
§6 field     — field_eval (inverse-square sum) with t_avg
§7 lamp      — Lamp state + reseed + tick
§8 scene     — chamber walls + field raster + HUD
§10 app      — signals, dt tracking, key handling
```

### Data Flow
```
init: blob_spawn each blob at random position with random radius/temp
tick: for each blob: buoyancy from (T - T_ambient(y)), random horiz walk,
      damping, wall bounce, heat/cool by half-chamber position
draw: erase → walls → for each cell:
        evaluate field f and t_avg
        if f > threshold: bucket from f intensity blended with t_avg
                         emit heat-ramp glyph
      → HUD
```

### References
- Blinn, "A Generalization of Algebraic Surface Drawing" *ACM TOG* 1 (1982) — original metaball paper.
- Wyvill, McPheeters & Wyvill, "Data Structure for Soft Objects" *Visual Computer* 2 (1986).
