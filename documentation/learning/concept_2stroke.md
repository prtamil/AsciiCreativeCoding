# Pass 1 — 2stroke.c: 2-stroke engine cross-section animation

## Core Idea

A 2-stroke internal-combustion engine is drawn in cross-section using ASCII characters.  The crank angle θ advances each tick via slider-crank kinematics.  The piston, connecting rod, and crankshaft are drawn in cell space each frame.  Exhaust and transfer ports open and close automatically as the piston passes them.  A spark fires at TDC.  A phase label updates throughout the cycle: COMPRESSION → IGNITION → POWER → EXHAUST → SCAVENGING.

## The Mental Model

Think of a lawnmower engine cut in half lengthwise.  The piston is a solid block inside the cylinder bore; it is connected by a rod to the crankshaft pin, which orbits the main bearing.  As the crank turns, the piston rises and falls.

Unlike a 4-stroke engine, a 2-stroke completes its power cycle in a single crankshaft revolution:

1. Piston rises → compresses fresh mixture above it.
2. At TDC: spark ignites the mixture → pressure forces piston down (power stroke).
3. Piston descends → uncovers exhaust port first → burned gas escapes left.
4. Piston descends further → uncovers transfer port on right → fresh charge from crankcase enters, scavenges the cylinder.
5. Piston rises again → ports close → compression begins.

All timing is derived purely from the piston position (which comes from the crank angle θ).

## Data Structures

### Engine (§4)
```
float theta       — crank angle (rad), 0=TDC, increases CW
int   rpm         — revolutions per minute (30–600)
bool  paused
```

### Scene (§6)
```
Engine engine
int    sim_fps
```

## Kinematics

### Slider-crank equations
```
crank_pin_row = crank_centre_row - CRANK_R * cos(θ)
crank_pin_col = crank_centre_col + CRANK_R * sin(θ)

rod_vert = sqrt(CONROD_L² - (CRANK_R * sin(θ))²)
wrist_pin_row = crank_pin_row - rod_vert
crown_row = wrist_pin_row - (PISTON_H - 1)
```

All positions are in cell space (floating-point, rounded to int at draw time).

### Port open condition
A port is uncovered when the piston crown has moved below its row:
```
ex_open = (crown_row > engine_top + EX_PORT_OFF)
tr_open = (crown_row > engine_top + TR_PORT_OFF)
```

### Phase detection
```
if tr_open:                                    SCAVENGING
if ex_open:                                    EXHAUST
if theta < IGNITE_WINDOW or theta > 2π - IGNITE_WINDOW: IGNITION
if theta < π:                                  POWER
else:                                          COMPRESSION
```

## Engine Geometry (cell units, from engine_top)

| Element | Position |
|---|---|
| Cylinder head | row 0 (engine_top) |
| Cylinder bore | rows 1–11 |
| Exhaust port (left wall) | row 6 |
| Transfer port (right wall) | row 7 |
| Piston at TDC (crown) | row 1 |
| Piston at BDC (crown) | row 9 |
| Crankcase top | row 12 |
| Crank centre | row 16 |
| Crankcase bottom | row 21 |

CRANK_R=4, CONROD_L=9, PISTON_H=3 → stroke = 2×CRANK_R = 8 cells.

CRANK_CENTER_OFF = HEAD_H + (PISTON_H-1) + CONROD_L + CRANK_R = 1+2+9+4 = 16.

## Drawing Order

Drawing proceeds in layers so later layers overwrite earlier ones:

1. **Cylinder head** — `+--[i]--+` top bar with spark plug `[i]` or `[*]`
2. **Cylinder walls** — `|` at inner and outer wall cols; gap at port row when open
3. **Gas above piston** — IGNITE: `*^` bright white; POWER: `^~` red; EXHAUST: `~~` dim grey; SCAVENGE: `~~` left / `++` right (cyan); COMPRESS: blank
4. **Exhaust pipe** — `~/./` flowing left when port open; dim `.` trail further left
5. **Transfer duct** — `>+` flowing right when port open
6. **Piston** — `[====]` crown, `[####]` body rows; overwrites wall at piston position
7. **Connecting rod** — Bresenham `:` from wrist pin to crank pin; `o` at wrist pin
8. **Crankshaft** — dim `.` ellipse orbit indicator (aspect-corrected); `*` crank arm; `O` main bearing; `o` crank pin
9. **Crankcase** — `+---+` box with opening for bore; `>=>` power-takeoff stub right
10. **Phase label** — orange bold text right of cylinder

## Non-Obvious Decisions

### Cell-space kinematics (no pixel space)
The engine is a mechanical drawing, not a particle simulation requiring isotropic forces.  All geometry is computed directly in cell space.  The crank orbit looks elliptical on screen (cells are ~2:1 tall:wide), but that matches the decorative ellipse drawn as the orbit indicator, so the visual is self-consistent.

### Aspect-corrected orbit ellipse
The crank pin traces a circle of radius CRANK_R in cell space.  On screen this appears as a tall ellipse.  The decorative orbit indicator uses row-radius = CRANK_R × 0.5 (half the col-radius), which compensates for the 2:1 cell aspect ratio and makes the orbit look circular.

### Port timing via crown position
Rather than encoding a timing table in degrees, port open/close is derived directly from kinematics: `ex_open = (crown_row > port_row)`.  This keeps timing numerically consistent with the drawn piston position with no separate lookup.

### Spark on IGNITE phase
The spark is shown whenever the phase is IGNITE (θ within IGNITE_WINDOW = 0.3 rad ≈ 17° of TDC).  This is proportional to rotation angle, not to real time, so the spark window looks the same at any RPM.

### Drawing order handles port occlusion
The port gap in the cylinder wall is drawn unconditionally when ex_open (or tr_open) is true.  The piston is drawn afterward and overwrites wall positions at piston rows.  Since ex_open is only true when crown_row > port_row (piston is entirely below the port), the piston never overwrites the port gap — occlusion is correct without explicit checks.

## From the Source

**Algorithm:** Phase detection by crank angle range comparison. `fmod(theta + 2π, 2π)` keeps angle in [0, 2π). Each phase boundary (port open/close) is a fixed angle derived from the engine geometry defined in the config section.

**Engineering:** Slider-crank mechanism converts rotary crank motion to linear piston motion. Given crank angle θ, crank radius R, and connecting rod length L, the wrist-pin position from crank centre: `y_wrist = R·cos θ + √(L² − R²·sin²θ)`. This is exact geometry — no approximation needed.

**Rendering:** Pure ASCII character art using box-drawing and line characters. No texture or shading — structure conveyed entirely by character choice (`|`, `-`, `+`, `#`, etc.).

---

## Key Constants

| Constant | Value | Effect if changed |
|---|---|---|
| `CRANK_R` | 4 cells | Stroke = 2×CRANK_R; larger = more visible piston travel |
| `CONROD_L` | 9 cells | Larger rod → less side loading; must stay > CRANK_R |
| `CRANK_CENTER_OFF` | 16 rows | Derived from HEAD_H+PISTON_H+CONROD_L+CRANK_R; changing any component shifts engine vertically |
| `EX_PORT_OFF` | 6 rows | Opens at (6-1)/(9-1) = 62% stroke; earlier = more exhaust blowdown |
| `TR_PORT_OFF` | 7 rows | Opens at 75% stroke; gap between EX and TR determines scavenging window |
| `IGNITE_WINDOW` | 0.30 rad | ≈17° either side of TDC for ignition phase |
| `RPM_DEFAULT` | 120 | 2 rev/s at startup; `] [` adjust by RPM_STEP=30 |

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `Engine` (struct) | `typedef struct` | ~40 B | crank angle θ, piston position, phase, RPM state |
| `theta` | `float` | scalar | crank angle in radians; advances each tick |
| `piston_row` | `int` | scalar | current crown row, computed from slider-crank kinematics |
| `phase` | `enum` | scalar | IGNITION / POWER / EXHAUST / SCAVENGING / COMPRESSION |
| `CRANK_R`, `CONROD_L` | `int` constants | N/A | engine geometry: crank radius (4 cells), rod length (9 cells) |
| `CYL_IHW` | `int` constant | N/A | cylinder bore inner half-width (6 cells) |
| `EX_PORT_OFF`, `TR_PORT_OFF` | `int` constants | N/A | row offsets at which exhaust (6) and transfer (7) ports open |
| `scene` (struct) | composite | ~100 B | owns Engine + HUD state; tick advances θ by RPM·dt |

# Pass 2 — 2stroke: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | CRANK_R, CONROD_L, port offsets, RPM range, ENGINE_H |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | CP_WALL/PISTON/CONROD/CRANK/FIRE/EXHAUST/INTAKE/SPARK/HUD/PHASE |
| §4 engine | Engine struct, `engine_kinematics()`, `compute_phase()`, `engine_tick()` |
| §5 draw | `safeaddch()`, `safeaddstr()`, `draw_line_ch()`, `scene_draw()` |
| §6 scene | Scene struct (owns Engine), `scene_init()` |
| §7 screen | ncurses init/cleanup, `draw_hud()` |
| §8 app | Signal handlers, main loop, input |

---

## Data Flow

```
engine_tick(dt):
  omega = 2π × rpm / 60
  theta += omega × dt
  if theta >= 2π: theta -= 2π

scene_draw(engine, engine_top, center_col):
  engine_kinematics(theta, cc_row, cc_col)
    → crown_row, wp_row, cp_row, cp_col

  ex_open = crown_row > engine_top + EX_PORT_OFF
  tr_open = crown_row > engine_top + TR_PORT_OFF
  phase = compute_phase(theta, ex_open, tr_open)
  spark = (phase == PHASE_IGNITE)

  draw_order: head → walls → gas → ports → piston → rod → crank → case → labels
```

---

## Pseudocode — Core Loop

```
setup:
  engine_reset()    ← theta=π (BDC), rpm=120

main loop:
  1. resize → getmaxyx, recompute engine_top = (rows - ENGINE_H - 2) / 2

  2. dt, cap 100ms

  3. sim ticks:
     while sim_accum >= tick_ns:
       engine_tick(dt)

  4. draw:
     erase()
     scene_draw()
     draw_hud(fps, rpm)
     wnoutrefresh + doupdate

  5. input:
     q/ESC → quit
     space/p → pause
     r → engine_reset
     ] → rpm += RPM_STEP
     [ → rpm -= RPM_STEP

  6. frame cap sleep
```
