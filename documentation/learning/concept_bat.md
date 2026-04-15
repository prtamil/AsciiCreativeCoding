# Pass 1 — bat.c: ASCII bat swarms in Pascal-triangle formation

## Core Idea

Three groups of ASCII bats fly outward from the terminal center. Each group is arranged in a filled Pascal-triangle formation: the leader at the apex, two bats in the second row, three in the third, and so on. The `+`/`-` keys resize the formation from 1 to 6 rows live, even while groups are in flight. All bats in a group share a synchronized four-frame wing animation (`/`, `-`, `\`, `-`). Three groups launch with staggered timing (30-tick delay between launches) and fly in different directions. Each group has a distinct color. When a group reaches the edge, it resets and re-launches in a new direction.

## The Mental Model

Imagine a flock of bats exiting a cave in formation — the alpha bat leads, the others follow in a triangular wedge. In flight the whole formation moves together, each bat's wings flapping in sync. When you press `+` a new row of bats materializes at the back of the triangle; when you press `-` the last row disappears.

The Pascal triangle formation means row 0 has 1 bat, row 1 has 2, row 2 has 3, ..., row r has r+1. The total bat count for n_rows rows is the triangular number `(n_rows+1)*(n_rows+2)/2`. At maximum (n_rows=6) that is 28 bats per group, 84 total across three groups.

The wing animation is the same four-frame cycle for every bat in a group, stepped in unison each tick. This gives the impression of a coherent flock flapping together.

## Data Structures

### Bat (§5)
```
float px, py    — position in pixel space (CELL_W=8, CELL_H=16)
float vx, vy    — velocity (pixels/tick)
bool  alive
```
All bats fly at the same velocity (the group velocity). Individual positions are initialized from the formation offset.

### Group (§6)
```
Bat    bats[MAX_BATS]    — MAX_BATS = (ROWS_MAX+1)*(ROWS_MAX+2)/2 = 28
int    n_bats            — active bat count = (n_rows+1)*(n_rows+2)/2
int    n_rows            — current formation rows (1–6)
float  angle             — flight direction in radians
float  leader_px, leader_py  — leader's current pixel position
int    wing_phase        — 0–3 wing animation frame
int    launch_ticks      — stagger countdown before launch
int    color             — xterm color index for this group
```

### Scene (§7)
```
Group  groups[3]
int    n_rows            — shared formation size (all groups)
bool   paused
```

## The Main Loop

1. Resize: recompute center position, reset all group positions.
2. Measure dt.
3. Ticks: for each group, advance leader position; update all bat positions from formation offsets; advance wing_phase every WING_TICKS; if leader goes off-screen, reset group with new angle.
4. Draw: for each bat in each group, compute cell position from pixel position, draw body `o` and wing frames.
5. HUD: fps, n_rows count, speed.
6. Input: `+`/`-`=n_rows, r/n=reset, p=pause, [/]=speed.

## Non-Obvious Decisions

### bat_form_offset(k) — triangular number inverse
The key function maps flat bat index k to a formation offset (along, perp):
```c
int r = 0;
while ((r+1)*(r+2)/2 <= k) r++;        /* find row */
int pos = k - r*(r+1)/2;               /* position within row */
*along = -(float)r * LAG_PX;           /* r rows behind leader */
*perp  = ((float)pos - (float)r * 0.5f) * SPREAD_PX; /* centered spread */
```
- k=0: r=0, pos=0 → along=0, perp=0 (leader)
- k=1: r=1, pos=0 → along=−LAG_PX, perp=−0.5×SPREAD_PX (left wingman)
- k=2: r=1, pos=1 → along=−LAG_PX, perp=+0.5×SPREAD_PX (right wingman)
- k=3: r=2, pos=0 → along=−2×LAG_PX, perp=−SPREAD_PX

### Formation offset → world space rotation
The (along, perp) offsets are in "formation space" — along is the direction behind the leader, perp is perpendicular to flight. To place a bat in world pixel space:
```c
float wx = leader_px + along * cosf(angle) - perp * sinf(angle);
float wy = leader_py + along * sinf(angle) + perp * cosf(angle);
```
This 2-D rotation matrix correctly transforms formation offsets regardless of flight direction.

### Live resize while in flight
When n_rows changes, new bats (for the added row) are immediately placed at their correct formation positions using the current leader position and angle. No group restart is needed. Removed bats (when n_rows decreases) are simply not drawn (n_bats decreases; bats beyond n_bats are ignored).

### Wing animation: synchronized per group
All bats in a group share the same `wing_phase` counter. Every WING_TICKS ticks, wing_phase increments mod 4. The four frames are:
```
frame 0: left='/'  body='o'  right='\'   (wings up)
frame 1: left='-'  body='o'  right='-'   (wings level)
frame 2: left='\'  body='o'  right='/'   (wings down)
frame 3: left='-'  body='o'  right='-'   (wings level)
```
Drawing a bat takes three columns: left wing, body, right wing.

### Staggered group launch
Groups are assigned launch_ticks delays of 0, 30, 60 ticks respectively. This staggers the launches so all three groups are not in the same position at the same time, creating a more dynamic visual.

### Pixel space for physics
Bats use pixel-space positions (CELL_W=8, CELL_H=16) for isotropic motion. A bat flying at 45° traverses the same visual distance as one flying at 0°. Without pixel-space, a bat flying vertically would appear to move twice as fast as one flying horizontally on the non-square terminal grid.

## State Machines

Each group has an implicit two-state lifecycle:
```
LAUNCHING (launch_ticks > 0) ──── countdown ────► FLYING
     ▲                                                 │
     └──── leader exits screen bounds ─────────────────┘ (new angle, reset)
```

## From the Source

**Algorithm:** V-formation particle system with purely kinematic (not physics-integrated) bat positions. Each bat's world position is analytically reconstructed from group heading angle, row index, and column index every frame — no per-bat position storage for the formation shape itself.

**Math:** Row `r`, column `c` bat offset from leader: `dx = c·SPACING_X − r·SPACING_X/2` (fan-out), `dy = r·SPACING_Y` (depth behind leader). A 2D rotation matrix then applies the group heading θ to place bats in world space. Wing cycle: `WING_CYCLE = 36` ticks @ 60 fps ≈ 0.6 s per flap ≈ 1.7 Hz. Group pause at center: `PAUSE_TICKS = 55` ticks before re-launch.

**Data-structure:** Bats within a group are reconstructed each frame from group state. Only the leader position and group heading are stored persistently — per-bat positions are derived values, not stored state.

---

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `ROWS_MAX` | 6 | Maximum formation rows; MAX_BATS = (7*8/2) = 28 |
| `LAG_PX` | e.g. 24 | Gap between rows in pixels; larger = more spread-out formation |
| `SPREAD_PX` | e.g. 16 | Lateral spacing between bats in a row |
| `WING_TICKS` | ~4 | Smaller = faster flapping |
| `STAGGER_TICKS` | 30 | Delay between group launches |
| `N_GROUPS` | 3 | Number of bat groups |
| Group colors | 141, 87, 213 | Light purple, electric cyan, pink-magenta |

## Open Questions for Pass 3

- What is the actual value of LAG_PX and SPREAD_PX — are they defined as CELL_H multiples?
- How many preset angles are there and how are they assigned to groups on each cycle?
- When a group resets after going off-screen, is the angle chosen sequentially from the preset table or randomly?
- Is the wing character drawn using mvwaddch three times (left, body, right) or with a string write?

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_groups[N_GROUPS]` | `Group[3]` | ~840 B | per-group state: heading, position, bats array |
| `g_groups[i].bats[MAX_BATS]` | `Bat[28]` | ~336 B each | pixel position and wing phase per bat in group |
| `g_groups[i].angle` | `float` | 4 B | current flight heading in radians |
| `g_groups[i].vx`, `g_groups[i].vy` | `float` × 2 | 8 B | velocity vector in pixels/second |
| `g_groups[i].n_rows` | `int` | 4 B | number of formation rows (1–6) |

---

# Pass 2 — bat: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | ROWS_MAX, MAX_BATS, LAG_PX, SPREAD_PX, WING_TICKS, STAGGER_TICKS |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | Group colors (xterm 141/87/213), `color_init()` |
| §4 coords | px_to_cell conversion, CELL_W/H, ASPECT_R |
| §5 bat | Bat struct, pixel-space position |
| §6 group | Group struct, `group_launch()`, `group_tick()`, `group_draw()` |
| §7 scene | Three groups, n_rows, `scene_tick()`, `scene_draw()` |
| §8 screen | ncurses init/draw/present/resize, HUD |
| §9 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
scene_tick():
  for each group g:
    if g.launch_ticks > 0: g.launch_ticks--; continue
    group_tick(g)

group_tick(g):
  advance g.leader_px += vx, g.leader_py += vy   ← flight angle
  if off-screen: group_launch(g, next_angle)
  g.wing_ticks++; if g.wing_ticks >= WING_TICKS: g.wing_phase = (g.wing_phase+1)%4
  for k = 0..g.n_bats-1:
    bat_form_offset(k, &along, &perp)
    bats[k].px = g.leader_px + along*cos(angle) - perp*sin(angle)
    bats[k].py = g.leader_py + along*sin(angle) + perp*cos(angle)

group_draw(g, win):
  for k = 0..g.n_bats-1:
    col = px_to_cell_x(bats[k].px)
    row = px_to_cell_y(bats[k].py)
    if out of bounds: continue
    wattron(COLOR_PAIR(g.color))
    mvwaddch(row, col-1, wing_left[g.wing_phase])
    mvwaddch(row, col,   'o')
    mvwaddch(row, col+1, wing_right[g.wing_phase])
    wattroff(...)

scene_set_rows(n):
  n_rows = clamp(n, 1, ROWS_MAX)
  for each group g:
    old_n = g.n_bats
    g.n_rows = n_rows
    g.n_bats = (n_rows+1)*(n_rows+2)/2
    for k = old_n..g.n_bats-1:
      place_bat(g, k)    ← new bats at correct formation position
```

---

## Function Breakdown

### bat_form_offset(k, &along, &perp)
Purpose: map flat index k to formation-space offsets.
Steps:
1. r = 0; while (r+1)*(r+2)/2 <= k: r++
2. pos = k − r*(r+1)/2
3. along = −(float)r × LAG_PX
4. perp = ((float)pos − (float)r × 0.5f) × SPREAD_PX

### group_launch(g, angle)
Purpose: reset group, assign new flight direction, place all bats.
Steps:
1. g.angle = angle
2. g.leader_px = center_px; g.leader_py = center_py
3. g.vx = speed × cos(angle); g.vy = speed × sin(angle)
4. g.wing_phase = 0; g.wing_ticks = 0
5. for k = 0..g.n_bats-1: place_bat(g, k)

---

## Pseudocode — Core Loop

```
setup:
  n_rows = 2   (default)
  scene_init()   ← set center, assign angles, launch groups 0/1/2 with stagger

main loop:
  1. resize → scene_init at new center

  2. dt, cap 100ms

  3. physics ticks:
     while sim_accum >= tick_ns:
       if not paused: scene_tick()

  4. alpha = sim_accum / tick_ns    ← render interpolation
     frame cap sleep

  5. draw:
     erase()
     scene_draw(alpha)   ← each bat drawn at interpolated position
     HUD: fps, n_rows, speed
     wnoutrefresh + doupdate

  6. input:
     q/ESC → quit
     +     → scene_set_rows(n_rows + 1)
     -     → scene_set_rows(n_rows - 1)
     r / n → scene_reset (new launch cycle)
     ] [   → sim_fps
     p/spc → pause
```

---

## Interactions Between Modules

```
App
 └── scene_tick → group_tick → bat positions from formation offsets
               → wing_phase advance
               → off-screen check → group_launch(next_angle)
 └── scene_draw → group_draw → mvwaddch (left wing, body, right wing)

Formation math
 └── bat_form_offset(k) → (along, perp) → rotate by angle → world pixel position
```
