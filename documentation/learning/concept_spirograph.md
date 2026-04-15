# Concept: Spirograph (Hypotrochoid / Epitrochoid)

## Pass 1 — Understanding

### Core Idea
A spirograph traces the path of a point attached to a small circle rolling inside (hypotrochoid) or outside (epitrochoid) a larger circle. The resulting curve is determined by the ratio of radii and the distance of the pen from the rolling circle's center.

### Mental Model
Imagine a gear spinning inside a ring gear. Fix a pen to a point on the spinning gear. As it rolls around, the pen traces a pattern. Depending on the gear sizes, you get flowers, stars, or complex overlapping loops.

### Key Equations
Hypotrochoid (small circle rolls inside large):
```
x(t) = (R-r)·cos(t) + d·cos((R-r)t/r)
y(t) = (R-r)·sin(t) - d·sin((R-r)t/r)
```

Epitrochoid (small circle rolls outside large):
```
x(t) = (R+r)·cos(t) - d·cos((R+r)t/r)
y(t) = (R+r)·sin(t) - d·sin((R+r)t/r)
```

Where: R=outer radius, r=inner radius, d=pen distance from inner center, t∈[0, 2π·lcm(R,r)/R].

### Data Structures
- Sample t from 0 to period in N_STEPS steps
- Draw line segments between consecutive points
- Animate by revealing the curve progressively

### Non-Obvious Decisions
- **Period calculation**: The curve closes when `t = 2π·R/gcd(R,r)`. Using integer values R,r makes this exact.
- **Animated reveal**: Draw one more segment per frame for a "drawing" animation effect.
- **Color along the curve**: Cycle hue as t increases to show the winding structure.
- **Aspect ratio**: Terminal cells are rectangular — scale x by CELL_H/CELL_W so circles look circular.
- **Parameter space**: d > r gives loops (pen outside gear), d = r gives epicycloid/hypocycloid, d < r gives rounded curves.

### Key Constants
| Name | Role |
|------|------|
| R | outer circle radius |
| r | inner circle radius |
| d | pen distance from center of rolling circle |
| N_STEPS | number of sample points for the curve |
| CELL_ASPECT | CELL_W/CELL_H for shape correction |

### Open Questions
- What ratio R/r gives a curve with exactly N petals?
- Try R=5, r=3, d=5 — what shape appears?
- Add a second pen at a different d — do the two curves interfere?

---

## From the Source

**Algorithm:** Parametric curve tracing with a float canvas and fade. Each curve advances `T_STEPS_PER_FRAME` timesteps per tick, writing brightness values to a float canvas. The canvas decays by `FADE_RATE` per tick, creating a fading trail effect.

**Math:** Hypotrochoid parametric equations exactly as implemented:
```
x(t) = (R−r)·cos(t) + d·cos((R−r)/r · t)
y(t) = (R−r)·sin(t) − d·sin((R−r)/r · t)
```
The curve closes when `(R−r)/r = p/q` (rational ratio). With p=3, q=1: deltoid (3-cusp astroid). Irrational ratios → curve never closes (dense in annulus). Pen distance `d` controls petal amplitude: `d < (R−r)` → inside, `d > (R−r)` → loops at each reversal point.

**Rendering:** Three simultaneous curves with different `r` values. `r` drifts sinusoidally (`DRIFT_RATE`), creating continuous morphing between distinct petal patterns without keystrokes.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app.scene.sg.canvas[MAX_ROWS][MAX_COLS]` | `float[80][240]` | ~75 KB | Per-cell brightness [0,1] for the fading trail |
| `g_app.scene.sg.cpair[MAX_ROWS][MAX_COLS]` | `int[80][240]` | ~75 KB | Per-cell colour pair index matching last-written curve |
| `g_app.scene.sg.curves[N_CURVES]` | `Curve[3]` | ~72 B | R, r, d, phase, drift, and colour for each hypotrochoid |

## Pass 2 — Implementation

### Pseudocode
```
compute_curve(R, r, d, n_steps) → points[]:
    # period: curve closes after R/gcd(R,r) full rotations
    period = 2*PI * R / gcd(R,r)
    for i in 0..n_steps:
        t = period * i / n_steps
        x = (R-r)*cos(t) + d*cos((R-r)*t/r)
        y = (R-r)*sin(t) - d*sin((R-r)*t/r)
        points[i] = (x, y)

screen_coords(pt, center, scale):
    col = center_col + pt.x * scale / CELL_W * CELL_H  # aspect correct
    row = center_row + pt.y * scale

draw(points, revealed):
    for i in 0..revealed-1:
        draw_line(points[i], points[i+1], color_at(i))

animate():
    revealed = 0
    each frame: revealed += REVEAL_SPEED; draw(points, revealed)
```

### Module Map
```
§1 config    — R, r, d, N_STEPS, REVEAL_SPEED
§2 math      — gcd(), compute_curve()
§3 draw      — draw_line(), screen_coords()
§4 app       — main loop, keys (R/r/d adjust, reset, speed, static mode)
```

### Data Flow
```
(R, r, d) → compute_curve (parametric) → points[]
→ animate: reveal points[0..k] each frame
→ draw line segments → screen
```
