# Pass 1 — magnetic_field.c: 2D Magnetic Field Lines from Configurable Dipoles

## Core Idea

Each bar magnet is modelled as a dipole — two equal-and-opposite magnetic monopoles separated by a fixed distance. The total magnetic field **B** at any point is the vector superposition of Coulomb-like contributions from every monopole:

```
B(r) = Σ_i  q_i · (r − r_i) / |r − r_i|³
```

`q_i = +1` for a North pole, `q_i = −1` for a South pole. Field lines are the curves everywhere tangent to **B** — they exit North poles and enter South poles. Four preset configurations (Dipole, Quadrupole, Attract, Repel) are traced and revealed incrementally using RK4 streamline integration.

## The Mental Model

Think of field lines as streams of invisible current: they flow out of every North pole, arc through space, and flow back into every South pole. The density of lines at any point indicates field strength. Where field lines converge, the field is stronger; where they spread apart, it is weaker.

The "null point" is a position where contributions from multiple poles cancel exactly — the net **B** is zero. Streamlines can never pass through a null point (no direction to follow), so they terminate there or spiral nearby. In the Quadrupole and Repel presets, a null point sits between the opposing poles and the lines visibly curve away from it.

Terminal cells are ~2× taller than wide (ASPECT_R = 2.0). Without correction, circular field patterns would look elliptical. The fix is applied in `field_at()`: y-differences are scaled by ASPECT_R before computing distance, then the resulting B_y is divided by ASPECT_R before returning. This makes the on-screen geometry match the physical geometry.

## Data Structures

### Monopole (§4)
```
float cx, cy   — cell-space position
float q        — +1 (North) or -1 (South)
```

### Dipole (§4)
```
float nx, ny   — North pole position (cell-space)
float sx, sy   — South pole position (cell-space)
int   color    — color pair for body bar
```

### FieldLine (§5)
```
int  col[MAX_LINE_STEPS]   — cell column of each traced point
int  row[MAX_LINE_STEPS]   — cell row of each traced point
char ch[MAX_LINE_STEPS]    — character: -, |, /, \, >, v, <, ^
int  cp[MAX_LINE_STEPS]    — color pair: BRT near source, DIM far away
int  len                   — number of valid points
bool done                  — always true after trace_line()
```

### Scene (§5)
```
Monopole  mp[MAX_MONOPOLES]   — up to 8 magnetic charges
int       nm                  — number of monopoles
Dipole    dp[MAX_DIPOLES]     — up to 4 bar magnets (for drawing)
int       nd
FieldLine lines[MAX_LINES]    — MAX_DIPOLES × N_SEEDS traced lines
int       n_lines             — total lines seeded
int       lines_traced        — how many to draw this frame
int       lines_per_tick      — animation speed (1–8)
bool      paused
int       preset, theme
int       cols, rows
```

## The Main Loop

1. **Resize / Init**: `scene_init()` → `scene_build_preset()` (place monopoles and dipoles for chosen configuration) → `scene_seed_lines()` (trace all field lines at once, store in `lines[]`).
2. **Tick**: `scene_tick()` increments `lines_traced` by `lines_per_tick` each frame.
3. **Draw**: iterate `lines[0..lines_traced-1]`, draw each stored (col, row, ch) cell with its color pair; then overlay dipole body bars and N/S pole markers.
4. **Auto-cycle**: when all lines are revealed, hold for 3 seconds then advance to next preset.

## Non-Obvious Decisions

### All lines traced at init, not at draw time
RK4 integration is computed once at scene_init — all MAX_LINE_STEPS for all lines. The `lines_traced` counter then controls how many are *rendered* each frame. This separates physics from animation speed: changing `lines_per_tick` only affects reveal rate, not physics accuracy.

### Aspect correction in field_at()
```c
float dy = (py - mp[i].cy) * ASPECT_R;   // stretch y to square space
float r2 = dx*dx + dy*dy + SOFT*SOFT;    // distance in square space
float inv3 = mp[i].q / (r2 * r);
*bx += inv3 * dx;
*by += inv3 * dy / ASPECT_R;             // un-stretch B_y before returning
```
This ensures the physical field topology (circular isolines around a single pole) maps to visually circular patterns on screen despite rectangular terminal cells.

### Character selection from direction vector
`line_char(dx, dy)` maps the normalised field direction to one of four line characters by angle range:
- `-` for near-horizontal (±22.5° from 0°/180°)
- `\` for 22.5°–67.5°
- `|` for 67.5°–112.5°
- `/` for 112.5°–157.5°

`direction_arrow(dx, dy)` maps 8 sectors to `>`, `v`, `<`, `^`. An arrow is placed every ARR_STRIDE=18 steps to show which way the field points.

### Seed radius SEED_R = 1.2 cells
Seeds placed too close to the pole (< 1 cell) can accidentally start inside the softening radius where the field direction is noisy. SEED_R=1.2 gives clean initial directions while keeping seeds visually co-located with the pole marker.

### Softening SOFT = 0.8 cells
Without softening, `|r - r_i| → 0` near a pole causes numerical blow-up. SOFT² is added to the squared distance before computing `r`, capping the field magnitude at the pole centre. The visual effect: streamlines near a pole curve correctly rather than spiralling wildly.

### Color by position along line
Lines are coloured in thirds: BRT (bright) for the first third (near the pole source), MID (medium) for the middle third, DIM (faint) for the far third. This gives a gradient that visually suggests field strength falloff with distance.

## Presets

| # | Name | Config | Visual signature |
|---|---|---|---|
| 0 | Dipole | 1 magnet, N left, S right | Classic closed oval loops curving from N to S |
| 1 | Quadrupole | 2 magnets at 90° | Four-lobe pattern; X-type null point at centre |
| 2 | Attract | N→S   N→S (S poles inward) | Dense convergent lines between the facing poles |
| 3 | Repel | S←N   N→S (N poles inward) | Divergent lines; null point between the facing N poles |

## State Machines

```
INIT: scene_build_preset → scene_seed_lines (all lines traced)
TICK: lines_traced += lines_per_tick   (reveal animation)
DONE: lines_traced >= n_lines → hold 3s → next preset → scene_init
```

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `N_SEEDS` | 16 | More = denser fan of lines from each pole |
| `MAX_LINE_STEPS` | 600 | Longer = lines can wrap further before clipping |
| `RK4_H` | 0.35 | Smaller = smoother curves but more computation |
| `SOFT` | 0.8 | Smaller = sharper near-pole bend; larger = softer |
| `ARR_STRIDE` | 18 | Smaller = more arrow markers on each line |
| `ASPECT_R` | 2.0 | Must match terminal cell aspect ratio |

## Themes

5 themes cycle with `t`/`T`:

| Theme | Field line colors | Character |
|---|---|---|
| Electric | dim cyan → bright white | Cold, scientific |
| Plasma | dim violet → bright pink | Energetic, hot |
| Fire | dim red → bright yellow | Thermal |
| Ocean | dim blue → bright white | Deep water |
| Matrix | dim dark-green → bright green | Digital |

---

# Pass 2 — magnetic_field: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_SEEDS=16, MAX_LINE_STEPS=600, RK4_H=0.35, SOFT=0.8, ARR_STRIDE=18, ASPECT_R=2.0 |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5 themes, 7 color pairs, `theme_apply(t)` |
| §4 physics | `Monopole`, `Dipole`, `field_at()` |
| §5 scene | `FieldLine`, `Scene`, `trace_line()`, `scene_init()`, `scene_tick()`, `scene_draw()` |
| §6 screen | ncurses init, HUD |
| §7 signal | SIGWINCH, SIGTERM |
| §8 app | main loop, input, auto-cycle |

## Data Flow

```
scene_init(preset):
  scene_build_preset()         ← place monopoles and dipoles
  scene_seed_lines():
    for each North pole:
      for k in 0..N_SEEDS-1:
        angle = k/N_SEEDS * 2π
        sx = pole.cx + SEED_R*cos(angle)
        sy = pole.cy + SEED_R*sin(angle)/ASPECT_R
        trace_line(line, mp, nm, sx, sy, cols, rows)
  lines_traced = 0

trace_line(fl, mp, nm, sx, sy, cols, rows):
  px=sx, py=sy
  for step in 0..MAX_LINE_STEPS-1:
    if out of bounds: break
    RK4: k1=field(px,py), k2=field(px+h/2*k1), k3=field(px+h/2*k2), k4=field(px+h*k3)
    (bx,by) = (k1+2k2+2k3+k4)/6
    if |B| < B_MIN: break   ← null point
    normalise (bx,by)
    ch = direction_arrow if step%ARR_STRIDE==0 else line_char(bx,by)
    store (col, row, ch) in fl
    px += h*bx; py += h*by
  assign colors by thirds (BRT/MID/DIM)

scene_tick():
  lines_traced = min(lines_traced + lines_per_tick, n_lines)

scene_draw():
  for li in 0..lines_traced-1:
    for each (col, row, ch, cp) in lines[li]:
      mvaddch(row, col, ch | COLOR_PAIR(cp))
  for each dipole:
    draw body bar with '=' chars (DIM)
    draw 'N' at north pole (CP_NORTH, BOLD)
    draw 'S' at south pole (CP_SOUTH, BOLD)
```

## field_at(mp, nm, px, py) → (bx, by)
```
bx = by = 0
for each monopole i:
  dx = px - mp[i].cx
  dy = (py - mp[i].cy) * ASPECT_R     ← square-space y
  r2 = dx*dx + dy*dy + SOFT*SOFT
  r  = sqrt(r2)
  inv3 = mp[i].q / (r2 * r)
  bx += inv3 * dx
  by += inv3 * dy / ASPECT_R          ← un-square B_y
```

## Open Questions for Pass 3

- Does the null-point termination (`|B| < B_MIN`) visually show as clean symmetric truncation in Quadrupole/Repel presets?
- Are 16 seeds per pole enough to cover the full angular fan, or do diagonal angles produce visible gaps?
- Does the aspect correction hold at very non-square terminal dimensions (e.g., 200×24)?
- Is SEED_R=1.2 always outside the softening zone, or does it clip for large SOFT values?
