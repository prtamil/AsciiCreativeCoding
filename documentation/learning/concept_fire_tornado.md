# Concept: Fire Tornado

## Pass 1 — Understanding

### Core Idea
A swirling cone of fire rendered as a particle pool in cylindrical coordinates around a vertical axis. Each ember has `(y, phase, radius)`; rises, rotates, drifts inward, cools. A side-view 2-D projection translates cylindrical to screen, with depth conveyed by which side of the column an ember is currently on (sin of its phase). Two-pass back/front draw makes the rotation legible. Stage 2 adds a base flame mat (1-D heat strip at ground), outward sparks, and a slow horizontal sway from a wind sinusoid.

### Mental Model
Think of a 3-D tornado lit from the camera. Embers far from the camera (sin(phase) < 0) are dimmed; embers near the camera (sin(phase) > 0) are bright. As phase advances each frame, the same ember alternates between bright and dim — that alternation is what reads as rotation in 2-D. The cone shape comes naturally because inner-radius embers rise faster (hot core) and outer ones cool sooner.

### Key Equations
```
screen_col  = axis_x + wind_tilt(y) + ASPECT_X · radius · cos(phase)
screen_row  = base_y - y
depth       = sin(phase)              +1 front, −1 back

phase  += omega(radius) · omega_mult · dt    # ω ∝ 1/r → centre spins fast
y      += y_vel(radius) · dt                  # v ∝ (1 − r/R_max) → core rises faster
radius *= (1 − RADIUS_DECAY · dt)
temp   -= COOL_RATE · dt
```

### Non-Obvious Decisions
- **Phase determines depth**: A 1-D phase sweep gives the illusion of 3-D rotation when paired with bright/dim dichotomy. No real 3-D math needed.
- **omega ∝ 1/r**: Conservation of angular momentum analogy. Centre spins fast, periphery drifts. Physically motivates the "twist."
- **Two-pass draw**: All back-half embers first, then all front-half embers. Avoids per-frame depth-sort cost.
- **Base flame mat as 1-D heat strip**: A miniature CA fire (decay + diffuse + inject) at the ground row. Replaces the static `_____` scorch line.
- **Sparks spawn from random alive embers**: They inherit a screen position from a real ember, then fly outward with gravity. Ties them to the column geometrically.

### Key Constants
| Name | Role |
|------|------|
| `N_EMBERS_DEFAULT` | 250 cylindrical particles |
| `BASE_RADIUS_CELLS` | 9 — horizontal radius at the base |
| `OMEGA_BASE` | 5.0 rad/sec scale (divided by radius) |
| `Y_VEL_BASE` | 6 cells/sec at the axis |
| `BASE_HEAT_W` | 60 — width of base-flame strip |
| `WIND_AMP_DEFAULT` | 3 cells of horizontal sway at top |

### Open Questions
- Why does sin(phase) give better depth perception than just |x − axis|?
- What changes if you let outer embers escape upward? (Likely loses the cone shape.)
- Could you tilt the whole funnel for a "tornado on the move"?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — pool sizes, speeds, base flame, sparks, wind constants
§3 color     — 5-stop heat ramp per theme, 4 themes
§4 layout    — wind_offset() helper
§5 ember     — Ember struct + respawn / tick / draw (with wind tilt)
§6 tornado   — Spark + base_heat + Tornado pool/lifecycle
§7 scene     — base flame → embers (back) → embers (front) → sparks → HUD
§9 app       — main loop, dt + world_time, key handling
```

### Data Flow
```
init: tornado_reseed → all embers random along lifetime, base_heat zero
tick: ember_tick (advance phase, y, radius, temp; respawn at base on death)
      base_heat_tick (decay + diffuse + inject)
      spark_tick (gravity + cool); on accumulator: tornado_spawn_spark
draw: base flame → embers back-pass → embers front-pass → sparks → HUD
```

### References
- Reeves, "Particle Systems" SIGGRAPH (1983) — foundational fire-particle paper.
- Bourke, "Character representation of greyscale images" — heat ramp.
- Sanglard, "Game Engine Black Book: Doom" (2018) ch. 11 — heat-diffusion CA.
