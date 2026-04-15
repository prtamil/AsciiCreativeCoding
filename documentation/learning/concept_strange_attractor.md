# Concept: Strange Attractor (Clifford / de Jong)

## Pass 1 — Understanding

### Core Idea
Iterated function systems on the plane that converge to a fractal attractor. The Clifford and de Jong maps are discrete: given (x,y) apply a nonlinear map to get (x',y'). After millions of iterations the visited points form an intricate fractal shape. Color by visit density.

### Mental Model
Imagine a chaotic billiard table with curved walls. No matter where you start the ball, it eventually settles into visiting the same strange set of curves. Plot all the bounces over millions of steps and the attractor emerges.

### Key Equations
Clifford attractor:
```
x' = sin(a·y) + c·cos(a·x)
y' = sin(b·x) + d·cos(b·y)
```

de Jong attractor:
```
x' = sin(a·y) - cos(b·x)
y' = sin(c·x) - cos(d·y)
```

Parameters a,b,c,d ∈ [−3,3] each give a completely different shape.

### Data Structures
- Accumulation buffer: `count[H][W]` — how many times each cell was visited
- Parameters: (a,b,c,d) as floats
- Log-normalize count for display

### Non-Obvious Decisions
- **Density accumulation, not single-point drawing**: One iteration = one vote into a bin. After N iterations, normalize and map count to brightness. This reveals the attractor's fine structure.
- **Log normalization**: `brightness = log(1 + count) / log(1 + max_count)`. Without log, high-density regions saturate while faint regions vanish.
- **Discard warm-up iterations**: First 1000 iterations may not be on the attractor. Discard them.
- **Parameter sweep**: The real beauty is in browsing parameters. A good UI lets you increment a,b,c,d and re-accumulate.
- **Flame coloring**: Assign color by the angle `atan2(y,x)` at each point, then blend with density. Creates the "flame" look.

### Key Constants
| Name | Role |
|------|------|
| N_ITER | total iterations (1M–10M) |
| WARMUP | discarded initial iterations (1000) |
| a,b,c,d | attractor parameters |
| GAMMA | tone-mapping exponent |

### Open Questions
- Which parameter values give the most intricate shape?
- What is the Lyapunov exponent of the Clifford map for your chosen parameters?
- Can you color by which quadrant the previous point was in?

---

## Pass 2 — Implementation

### Pseudocode
```
accumulate(a,b,c,d):
    x=0.1, y=0.0
    zero count[H][W]
    for n in 0..N_ITER:
        x2 = sin(a*y) + c*cos(a*x)
        y2 = sin(b*x) + d*cos(b*y)
        x=x2; y=y2
        if n < WARMUP: continue
        # map (x,y) ∈ [-3,3]² to (col,row)
        col = (int)((x+3)/(6) * W)
        row = (int)((y+3)/(6) * H)
        if in bounds: count[row][col]++

normalize_and_draw():
    max_c = max(count)
    for each cell:
        b = log(1+count[row][col]) / log(1+max_c)
        char = density_char(b)
        color = hue_from_density(b)
        mvaddch(row, col, char)
```

### Module Map
```
§1 config    — N_ITER, WARMUP, initial a,b,c,d
§2 attractor — accumulate(), log_normalize()
§3 draw      — count buffer → brightness → char + color
§4 app       — main loop, keys (parameter nudge, reset, type select)
```

### Data Flow
```
(a,b,c,d) → accumulate N_ITER points into count[H][W]
→ log-normalize → map to ASCII density chars → draw once
(redraw only when parameters change)
```

## From the Source

**Algorithm:** Density-map rendering of strange attractor trajectories. A point is iterated for many steps under the attractor map. Rather than drawing each point directly, a density grid accumulates visit counts. Log-normalised density is then mapped to a palette.

**Math:** Several 2D and 3D autonomous ODE systems: Clifford: `x' = sin(a·y)+c·cos(a·x)`, `y' = sin(b·x)+d·cos(b·y)`. Peter de Jong: similar polynomial form. Tinkerbell: complex quadratic map. Lorenz (projected 3D → 2D). All exhibit sensitivity to initial conditions (chaos): nearby trajectories diverge exponentially, producing the intricate filamentary structure of the attractor.

**Performance:** ITERS_PER_FRAME=200,000 iterations per tick, WARMUP_ITERS=5000 discarded transient. Log density coloring: `brightness ∝ log(1 + count)/log(1 + max)`. Log scaling prevents the densest regions from washing out all detail — without it, the "spine" of the attractor would be solid white while outlying filaments remained invisible.
