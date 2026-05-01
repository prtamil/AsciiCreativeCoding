# Concept: Volcano with Lava Bombs and Ash Plume

## Pass 1 — Understanding

### Core Idea
A truncated cone with a glowing crater. Lava bombs erupt from the crater on parabolic trajectories (initial velocity + gravity), arcing outward and dying on impact with the cone slopes or off-screen edges. Above the crater an ash plume rises through horizontal random-walk diffusion. Periodic eruption bursts (every 3 seconds) spawn 12 extra bombs at once for visible rhythm. Two distinct particle pools share the same heat ramp: bombs at the bright end, ash at the cool end.

### Mental Model
Static cone silhouette + two ballistic particle systems. The cone is parameterised by axis position, top row, base row, crater half-width, and base half-width — recomputed on resize. `is_in_mountain(row, col)` is a single predicate that decides whether a bomb has impacted. Bombs are launched in a narrow upward cone of angles, then physics takes over (constant gravity, no drag). Ash particles have constant upward velocity plus horizontal Brownian drift, fading by life rather than temperature.

### Key Equations
```
bomb spawn:  ang   ∈ ±BOMB_LAUNCH_CONE rad about vertical
             speed ∈ [SPEED_MIN, SPEED_MAX]
             vx = sin(ang) · speed · ASPECT_X
             vy = -cos(ang) · speed                /* up = negative */

bomb tick:   vy += GRAVITY · dt
             pos += vel · dt
             temp -= BOMB_COOL · dt
             die if off-screen, in mountain, or temp ≤ 0

cone test:   half_w(row) = crater_half + frac · (base_half - crater_half)
             frac        = (row - top_row) / (base_row - top_row)
             in_mountain = (|col - axis_x| ≤ half_w(row))
```

### Non-Obvious Decisions
- **Linear cone taper**: `half_w` interpolates linearly from crater to base. Real volcanoes are sigmoid but linear reads fine in ASCII.
- **Slope chars only on edges**: `/` and `\` for the left/right slopes; the interior is sparse `.` rocks every ~31st cell — gives texture without visual noise.
- **Burst on `b` key replaces every active slot**: With `bomb_spawn_burst` (1.5× speed, 1.6× cone width), it's unmistakably visible even when the pool is full.
- **Ash spawn rate ≫ bomb spawn rate**: 40 ash/sec vs 20 bombs/sec — the plume needs density to read as smoke.
- **Cool rate kept low**: 0.20/sec lets bombs stay bright through their full arc, not just at launch.

### Key Constants
| Name | Role |
|------|------|
| `N_BOMBS_DEFAULT` | 60 active lava bombs |
| `N_ASH_MAX` | 150 ash particles |
| `BOMB_SPEED_MIN/MAX` | 6/12 cells/sec at launch |
| `GRAVITY` | 14 cells/sec² |
| `BURST_INTERVAL` | 3 seconds — automatic burst cadence |
| `TOP_ROW_FRAC` | 0.35 — crater row as fraction of rows |

### Open Questions
- What if eruption switches phases (REST → RUMBLE → ERUPT → FADE)?
- Could lava flow down the slopes after impact?
- How would underwater eruption look — pillow lava, no ash?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — pool sizes, geometry, eruption rates
§5 bomb      — Bomb struct + spawn (normal + burst) + tick + draw
§6 ash       — Ash struct + spawn / tick / draw
§7 volcano   — Volcano state, cone geometry, eruption lifecycle
§8 scene     — draw_mountain + draw_crater + draw_bombs + draw_ash + HUD
§10 app      — signals, dt tracking, key handling, main loop
```

### Data Flow
```
init: position cone from terminal size; clear pools
tick: bombs (gravity + cool + impact test), ash (drift + fade)
      spawn rate accumulators → spawn into dead slots
      every BURST_INTERVAL → 12 bombs at once
draw: ash → mountain → crater glow → bombs → HUD
```

### References
- Reeves, "Particle Systems" SIGGRAPH (1983) — particle pool pattern.
- Self & Whitehead, "Strombolian eruption mechanics" *Bulletin of Volcanology* 49 (1986) — the ballistic-bomb regime modelled here.
