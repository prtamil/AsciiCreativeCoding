# Pass 1 — constellation: Live Star Constellation Map

## Core Idea

Stars drift slowly across the screen with random wander acceleration. When any two stars come within a threshold distance of each other, a line is drawn between them using slope-matched ASCII characters. The line style (bold/normal/stippled) depends on how close the stars are. The result is a living constellation map that evolves as stars drift.

---

## The Mental Model

Imagine 30 stars floating on a dark sky, each slowly curving around (due to random wander). When two stars get close, a connection line appears between them. Close stars get a bright solid line; far-apart-but-still-connected stars get a faint dotted line.

The trick is that lines are drawn as Bresenham line segments where each cell's character is chosen based on the local step direction — so lines at different angles use `|`, `-`, `/`, `\` appropriately. This makes the star map look like handdrawn star charts.

A `cell_used[][]` boolean grid ensures that when multiple connection lines overlap, only the first writer wins — no messy overprinting.

---

## Data Structures

### `Star` struct
| Field | Meaning |
|---|---|
| `px, py` | Current position in pixel space |
| `prev_px, prev_py` | Position at previous tick (for true lerp interpolation) |
| `vx, vy` | Velocity in pixels per second |
| `color` | Color pair index (1..N_STAR_COLORS) |
| `ch` | Symbol character from `"*+o@."` |

### `Scene` struct
| Field | Meaning |
|---|---|
| `stars[STARS_MAX]` | Pool of all stars |
| `n` | Active star count |
| `paused` | Flag |
| `connect_preset` | Index into k_connect_presets (tight/normal/wide) |

### `cell_used[rows][cols]`
A VLA (variable-length array) allocated on the stack each draw frame. Tracks which cells have already been written by connection lines. First writer wins — no double-writing.

---

## The Main Loop

Standard fixed-timestep with render interpolation:
1. `scene_tick()` — move each star (wander + speed cap + wall bounce)
2. `alpha = sim_accum / tick_ns`
3. `scene_draw(alpha)` — lerp star positions, draw connections, draw stars

---

## Non-Obvious Decisions

### True lerp, not forward extrapolation
`bounce_ball.c` uses forward extrapolation: `draw_px = px + vx * alpha * dt`

`constellation.c` uses true lerp: `draw_px = prev_px + (px - prev_px) * alpha`

Why the difference? Bounce_ball has constant velocity between ticks — extrapolation is exact. But stars have wander acceleration that changes velocity each tick. You cannot predict where the star will be "alpha ticks into the future" from current velocity alone without integrating the acceleration. Lerp between the two known states (prev and current) is always exact.

### Wander acceleration + speed cap
Each tick, stars receive a small random acceleration in any direction. This makes paths gently curve instead of moving in straight lines forever. Without the speed cap, wander would accumulate and stars would eventually move too fast. The cap rescales velocity to SPEED_CAP when exceeded.

### Stippled Bresenham for far connections
Lines are drawn with a `stipple` parameter (every Nth cell):
- `ratio < 0.50` (close): stipple=1 (every cell), A_BOLD
- `ratio < 0.75` (medium): stipple=1 (every cell), normal
- `ratio < 1.00` (far): stipple=2 (every other cell), normal

Far connections appear dotted, giving a visual sense of distance.

### Character selection from local step direction
Standard Bresenham produces runs of the same character — a very flat line is all `-`, a very steep line is all `|`. But between those extremes, the character changes based on whether the next step is diagonal or straight. This is computed from the **Bresenham error value** per step, not the overall angle. Result: smooth-looking lines without extra math.

### `cell_used[][]` VLA deduplication
When stars A-B and stars A-C have overlapping connection lines, without deduplication both would write to the same cells and produce garbled characters. `cell_used` prevents any cell from being written by a second connection line, so the first line drawn "claims" each cell.

Being a VLA allocated on the stack (`bool cell_used[rows][cols]`), it's automatically zeroed via `memset` and freed when `scene_draw()` returns — no malloc needed.

---

## State Machines

No state machine. Purely continuous physics. One boolean: paused.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| `STARS_DEFAULT` | 30 | More stars = more connections = denser map |
| `SPEED_MIN/MAX` | 50–120 px/s | Faster stars = rapidly evolving map |
| `WANDER_ACCEL` | 20 px/s² | Higher = more erratic paths |
| `SPEED_CAP` | 130 px/s | Lower = slower stars even with high wander |
| `k_connect_presets` | 120/200/280 px | Larger = more connections visible |

---

## Open Questions for Pass 3

1. Remove `cell_used` — what does the visual look like when lines overwrite each other?
2. Change the stipple condition to use random sampling instead of fixed-period stepping — how does it look?
3. What happens if you use forward extrapolation instead of lerp? (Start with no wander to see they're equivalent, then add wander to see the difference.)
4. Try connecting stars in order of distance (closest first) vs arbitrary order — does it change what `cell_used` blocks?
5. The VLA `cell_used[rows][cols]` is allocated on the stack every frame — what's the maximum safe terminal size before this risks a stack overflow?
