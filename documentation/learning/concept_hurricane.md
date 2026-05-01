# Concept: Hurricane / Cyclone (Top-Down)

## Pass 1 — Understanding

### Core Idea
Bird's-eye view of a tropical cyclone. A pool of cloud particles orbits a central eye following the **Rankine vortex** velocity profile — solid-body rotation inside the eyewall (`v ∝ r`), free-vortex `1/r` decay outside. Each particle slowly spirals inward (radial inflow) and is recycled when it hits the eye. Three concentric **bands** show different brightness/colour: outer rim dim, middle band brighter, eyewall white-bold. The eye itself is a calm dark spot. The result is a still-frame satellite image of a hurricane, animated.

### Mental Model
Each cloud is in polar `(r, θ)`. Each frame: theta advances by ω(r)·dt; r decreases by inflow rate. The *zone* (outer / band / eyewall / eye) is decided by the cloud's current radius. The wind direction at each cloud is tangential (perpendicular to the radial), so the velocity vector picks one of 8 slope-character glyphs (`-` `\` `|` `/`) — that's how the swirl direction reads even on a still frame.

### Key Equations
```
Rankine ω(r):
    if r ≤ R_eye:    ω = ω_max · r / R_eye        /* solid-body rotation */
    else:            ω = ω_max · R_eye / r        /* free-vortex decay   */

v_max occurs at r = R_eye  (the eyewall — "highest winds")

zone(r):  r < 0.7·R_eye   → eye (skipped — empty centre)
          r < 1.15·R_eye  → eyewall  (A_BOLD white)
          r < 0.6·R_outer → band     (mid-bright)
          else            → outer    (visible mid-tone)

screen projection:
    sx = cx + ASPECT_X · r · cos(θ)
    sy = cy + r · sin(θ)
```

### Non-Obvious Decisions
- **Two-zone Rankine profile, not Holland (1980) curve**: simpler, captures the qualitative shape (peak at eyewall, decay outside).
- **Eye-zone particles SKIPPED in draw**: the centre stays empty (a "hole in the clouds"). A static `draw_eye` paints just three `.` markers.
- **Wind glyph from velocity angle**: 8 sectors → 4 unique chars (`-` `\` `|` `/`), rotation direction is visible on still frames.
- **Cell-aspect correction with `ASPECT_X = 2`**: terminal cells are ~2× tall as wide; without this the storm reads as vertically squashed.
- **Brightness via colour values, not `A_DIM`**: Earlier versions used `A_DIM` on outer band but that combined with mid-saturation hues produced invisibly-dark results on dark terminals.

### Key Constants
| Name | Role |
|------|------|
| `N_CLOUDS_DEFAULT` | 500 orbiting cloud particles |
| `EYE_RADIUS_DEFAULT` | 4 cells |
| `OMEGA_MAX_DEFAULT` | 1.6 rad/sec at the eyewall |
| `OUTER_RADIUS_FRAC` | 0.85 — outer rim as fraction of half-screen |
| `INFLOW_RATE` | 0.6 radius units/sec inward |

### Open Questions
- What does Holland's (1980) modern profile look like in this code?
- Could you simulate a hurricane *passing over* by translating the centre?
- Two interacting cyclones — does the merging look like real storm interaction?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — pool size, eye/outer radii, vortex strength
§5 vortex    — rankine_omega + radial_zone + wind_glyph
§6 cloud     — Cloud struct + respawn / tick / draw
§7 hurricane — Hurricane state + geometry + lifecycle
§8 scene     — eye marker + cloud draw + HUD
§10 app      — signals, dt tracking, key handling
```

### Data Flow
```
init: every cloud at random (r ∈ [outer·0.6, outer], θ ∈ [0, 2π))
tick: θ += ω(r)·dt; r -= inflow·dt; recycle on r < 0.5·R_eye or r > 1.05·R_outer
draw: erase → cloud (zone-coloured) → eye marker dots → HUD
```

### References
- Rankine, "A Manual of Applied Mechanics" (1858) §625 — original two-zone vortex.
- Holland, "An Analytic Model of the Wind and Pressure Profiles in Hurricanes" *Monthly Weather Review* 108 (1980).
