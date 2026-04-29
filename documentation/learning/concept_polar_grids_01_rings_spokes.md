# Concept: Standard Polar Grid (01_rings_spokes.c)

## Pass 1 — Understanding

### Core Idea
A screen-sweep algorithm where every terminal cell is tested for membership on a concentric ring or radial spoke using modular arithmetic.  The fmod trick detects all rings and all spokes simultaneously — no loop over ring indices needed.

### Mental Model
Think of the screen as a flat disc viewed from above.  Every cell has a distance `r` and angle `θ` from the centre.  A ring is just the set of cells where `r` is a near-integer multiple of `RING_SPACING`.  A spoke is the set where `θ` is a near-multiple of `2π/N_SPOKES`.

### Key Equations

**Pixel-space polar coordinates** (aspect-corrected):
```
dx_px = (col − ox) × CELL_W        CELL_W = 2
dy_px = (row − oy) × CELL_H        CELL_H = 4
r     = sqrt(dx_px² + dy_px²)      → circular in pixel space
θ     = atan2(dy_px, dx_px)        → [−π, π]
```

**Ring test** (all rings at once):
```
ring_phase = fmod(r, RING_SPACING)
on_ring : ring_phase < RING_W || ring_phase > RING_SPACING − RING_W
```

**Spoke test** (all spokes at once):
```
θ_norm     = fmod(θ + 2π, 2π)          → normalise to [0, 2π)
spoke_angle = 2π / N_SPOKES
spoke_phase = fmod(θ_norm, spoke_angle)
on_spoke : spoke_phase < SPOKE_W || spoke_phase > spoke_angle − SPOKE_W
```

**Character selection** (tangent direction at angle `θ`):
```
a = fmod(θ + 2π, π)         → fold to [0, π)  (lines have no direction)
a < π/8 or ≥ 7π/8  → '-'
a < 3π/8            → '\'
a < 5π/8            → '|'
else                → '/'
```

### Non-Obvious Decisions

- **CELL_H/CELL_W = 2**: Terminal rows are roughly twice the height of columns in pixels.  Scaling dy by CELL_H and dx by CELL_W makes the Euclidean distance in pixel space circular, not elliptic.  Without this, rings appear as ellipses stretched vertically.
- **fmod, not integer division**: fmod detects all rings in a single expression.  An alternative would loop over `k = 1, 2, …` and test `|r − k×SPACING| < RING_W`, but that's O(rings) per cell.  fmod is O(1).
- **Normalise θ before fmod**: `atan2` returns [−π, π].  Adding 2π and taking fmod maps negative angles to positive so the spoke test works uniformly.
- **RING_W in pixels vs SPOKE_W in radians**: Ring width is a fixed pixel threshold (line looks equally thick at all radii).  Spoke width is angular (line thickens with radius — physically correct for a radial line).

### Open Questions

- Why does `angle_char` use `fmod(θ, π)` instead of `fmod(θ, 2π)`?  (Answer: a line at angle α is visually identical to a line at α+π — orientation is undirected.)
- What happens at the very centre when r = 0?  (Answer: `SPOKE_MIN_R` suppresses the centre blob.)
- If `RING_W` were a fraction of `RING_SPACING` (not fixed pixels), how would thin vs thick rings change with radius?

---

## From the Source

**Algorithm:** Screen-sweep polar detection.  O(rows × cols) per frame with one sqrt + one atan2 per cell.

**Math:** Pixel coordinates respect terminal aspect ratio via CELL_H/CELL_W = 2 scaling.  Modular fmod detects all rings and spokes in O(1) expressions.

**Rendering:** Intersections of ring and spoke use '+'; otherwise `angle_char(θ)` gives the correct tangent character.

---

## Pass 2 — Implementation

### Pseudocode

```
grid_draw(rows, cols, ox, oy, ring_spacing, n_spokes):
    spoke_angle = 2π / n_spokes
    for each cell (row, col):
        (r, θ) = cell_to_polar(col, row, ox, oy)
        ring_phase = fmod(r, ring_spacing)
        on_ring = ring_phase < RING_W || ring_phase > ring_spacing − RING_W
        θ_norm = fmod(θ + 2π, 2π)
        spoke_phase = fmod(θ_norm, spoke_angle)
        on_spoke = r > SPOKE_MIN_R && (spoke_phase < SPOKE_W || …)
        if on_ring && on_spoke: draw '+'
        elif on_ring or on_spoke: draw angle_char(θ)
```

### Module Map

```
§1 config   — RING_SPACING, N_SPOKES, RING_W, SPOKE_W, CELL_W/H
§2 clock    — clock_ns, clock_sleep_ns
§3 color    — color_init(theme), 5 themes, PAIR_GRID/HUD/LABEL
§4 coords   — cell_to_polar(col,row,ox,oy → r_px,theta)
§5 draw     — angle_char(theta), grid_draw
§6 scene    — scene_draw (erase + grid + HUD + label)
§7 screen   — screen_init, screen_cleanup
§8 app      — signal handlers, main loop
```

### Data Flow

```
getch('+/-/[/]') → update ring_spacing/n_spokes
every frame:
  scene_draw → grid_draw → for each cell:
    cell_to_polar → ring/spoke test → mvaddch(angle_char)
  HUD → wnoutrefresh → doupdate
```
