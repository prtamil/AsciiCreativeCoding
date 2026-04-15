# Pass 1 — coral.c: Anisotropic DLA coral / dendrite fractal growth

## Core Idea

Diffusion-Limited Aggregation where the sticking probability depends on the direction of contact. Eight seed cells are placed along the bottom row. Walkers are released from random positions in the top row and drift downward with a gravity bias. When a walker touches a frozen cell, the probability of sticking depends on whether the frozen cell is below (high — 0.90), to the side (medium — 0.40), or above (low — 0.10) the walker. This anisotropy biases growth upward in branching columns that look like coral or dendrite trees. Color is assigned by row position at freeze time: deep brown-red at the base, orange in the middle, bright yellow-white at the tips.

## The Mental Model

Imagine a coral reef. Nutrients drift down from above; the coral grows upward to catch them. If a coral branch tip is below a settling nutrient particle, the particle sticks easily. If it approaches from the side, there is moderate chance of sticking (producing lateral branching). If the particle approaches from above (trying to sneak under a ledge), it almost never sticks. This directional preference creates upward-biased, branching column structures — like real coral or tree branches, not like isotropic DLA blobs or the symmetric snowflake.

Unlike snowflake.c, there is no symmetry enforcement. Each branch grows independently and stochastically, so every reef looks different.

The auto-reset triggers when the tallest branch reaches the top quarter of the screen — preventing the reef from filling the whole terminal and becoming unreadable.

## Data Structures

### Grid (§4)
```
uint8_t frozen[GRID_ROWS_MAX][GRID_COLS_MAX]   — 0 = empty, 1 = frozen
int8_t  color[GRID_ROWS_MAX][GRID_COLS_MAX]    — color ID assigned at freeze
int     rows, cols
int     tallest_row    — tracked to trigger auto-reset
int     n_seeds        — number of bottom seeds (8 by default)
```

### Walker (§5)
```
int   col, row    — cell-space position (coral uses cell-space, not pixel-space)
bool  alive
```
Walkers move in integer cell steps (not pixel space) because the downward bias and direction-dependent sticking are cell-relative concepts — no isotropic physics needed.

### Scene (§6)
```
Grid    grid
Walker  walkers[WALKER_MAX]
int     n_walkers
bool    paused
```

## The Main Loop

1. Resize check: rebuild grid, re-seed bottom row.
2. Measure dt, cap at 100ms.
3. Physics ticks: step all walkers; for each walker contacting a frozen cell, compute direction, apply direction-dependent probability, maybe freeze the walker.
4. Respawn stuck/escaped walkers from random top-row positions.
5. After each freeze, check if new cell is above tallest_row; update tallest_row; if tallest_row < rows/4, trigger auto-reset.
6. Draw: for each frozen cell write `#` with row-based color.
7. HUD.
8. Input.

## Non-Obvious Decisions

### Direction-dependent sticking probabilities
The three probabilities (0.90 / 0.40 / 0.10) are not arbitrary — they encode the physics of upward growth:
- A high probability for "frozen cell is below walker" means the growth advances upward readily.
- A medium probability for lateral contact creates branching: side branches can form, but not as easily as the upward growth.
- A low probability for "frozen cell is above walker" means the aggregate almost never grows downward or creates caves — the morphology stays columnar.

If all three probabilities were equal (isotropic DLA), the result would be the random fractal blobs you see in snowflake.c without the symmetry enforcement.

### Downward walker drift (gravity bias)
Walk probabilities: 50% down, 20% up, 15% left, 15% right. The downward bias makes walkers settle toward the growing tips rather than wandering randomly. Without the bias walkers would spend too long in the upper half of the screen and rarely reach the growing coral tips near the bottom.

### Cell-space (not pixel-space) physics
Unlike snowflake.c, coral walkers move in integer cell steps. The directional sticking probability is a cell-relative concept ("is the frozen neighbor above me, below me, or to the side") that maps naturally to cell coordinates. There is no isotropic-distance physics needed here, so the CELL_W/CELL_H correction is omitted.

### Eight bottom seeds
Starting from multiple seeds means multiple coral columns grow simultaneously and may eventually merge laterally. If you started with one seed the result would be a single central dendrite, not a reef-like carpet.

### Auto-reset at top quarter
The reset threshold (row < rows/4) leaves three-quarters of the screen for coral growth before resetting. At full occupation the coral becomes too dense to read clearly. The quarter threshold provides a natural-looking "reef fills up" animation then recycles.

## State Machines

No explicit state machine. The auto-reset is triggered by a row threshold, not a separate state.

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `PROB_BELOW` | 0.90 | Higher = more vertical columns, less branching |
| `PROB_SIDE` | 0.40 | Higher = denser lateral branching |
| `PROB_ABOVE` | 0.10 | Higher = coral can droop/overhang |
| `WALKER_DEFAULT` | 150 | More = faster growth (no 12-way freeze multiplier) |
| Walk down bias | 50% | Higher = walkers reach tips faster; lower = more uniform random walk |
| Reset threshold | rows/4 | Lower = reset earlier; higher = denser before reset |

## Open Questions for Pass 3

- How exactly are the direction probabilities computed from the walker-to-frozen-cell vector? Does the code check all eight neighbor directions or only four?
- What character is drawn for frozen coral cells — is it always `#` or does it vary by density/neighbors?
- When multiple direction contacts occur simultaneously (a corner), which probability is used?
- Does the auto-reset preserve seed positions or randomize them?

## From the Source

**Algorithm:** Anisotropic Diffusion-Limited Aggregation (DLA). Standard DLA: a random walker sticks to the aggregate with probability 1.0 regardless of contact direction. Anisotropic DLA: the sticking probability depends on the direction of contact — high for upward growth (coral tip absorbing nutrients falling from above), low for downward.

**Math:** In standard DLA, the fractal dimension D ≈ 1.7 in 2D. Anisotropy shifts D — stronger upward bias → more elongated, less branchy aggregate with higher effective D in the vertical. The aggregate is "scale-free": branching density is constant at every magnification (self-similar fractal structure). Sticking probabilities: STICK_P_BELOW=0.90 (frozen cell directly below walker), STICK_P_SIDE=0.40 (to the left or right), STICK_P_ABOVE=0.10 (directly above walker).

**Physics:** The directional probabilities model real coral and crystal growth: nutrients diffuse from above, so upward-facing surfaces have higher capture probability. The gravity-biased walk (50% down, 20% up) models sinking nutrient particles rather than symmetric diffusion.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `scene.grid.cells[80][300]` | `uint8_t[80][300]` | ~24 KB | frozen/empty grid; value encodes depth-based color |
| `scene.walkers[400]` | `Walker[400]` | ~5 KB | active random-walk particles (cx, cy, active flag) |
| `scene.grid.frozen_count` | `int` | 4 B | total cells that have frozen into the coral |
| `scene.grid.tallest_row` | `int` | 4 B | lowest row index reached — used for auto-reset trigger |

# Pass 2 — coral: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | PROB_BELOW/SIDE/ABOVE, walker limits, grid limits, seed count |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | Row-depth color mapping, `color_init()` |
| §4 grid | Frozen array, seed placement, `grid_freeze()`, `grid_draw()`, `grid_reset()` |
| §5 walker | Walker pool, `walker_step()`, direction contact test, probability sticking |
| §6 scene | `scene_tick()`, auto-reset trigger |
| §7 screen | ncurses init/draw/present/resize, HUD |
| §8 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
scene_tick():
  for each alive walker:
    walker_step()       ← random step: 50% down, 20% up, 15% left, 15% right
    for each of 4 orthogonal neighbors of walker:
      if neighbor is frozen:
        dir = direction from walker to frozen neighbor
        p = PROB_BELOW if dir==down, PROB_SIDE if dir==left/right, PROB_ABOVE if dir==up
        if rand() < p:
          freeze(walker.row, walker.col)
          respawn walker at random top position
          break

grid_freeze(row, col):
  frozen[row][col] = 1
  color[row][col]  = row_color(row)   ← deep=brown, mid=orange, tip=yellow
  if row < tallest_row: tallest_row = row
  if tallest_row < rows/4: scene_reset()
```

---

## Function Breakdown

### walker_step(w)
Purpose: move walker one cell in a direction biased downward.
Steps:
1. r = rand() % 100
2. if r < 50: row++  (down)
3. elif r < 70: row--  (up)
4. elif r < 85: col--  (left)
5. else: col++  (right)
6. If walker leaves screen bounds: respawn at top.

### row_color(row) → color_id
Purpose: map row number to a color from the depth gradient.
Steps:
1. t = (float)row / rows  (1.0 = bottom, 0.0 = top)
2. Return color band: t > 0.75 → brown; t > 0.50 → orange; t > 0.25 → yellow; else → white.

---

## Pseudocode — Core Loop

```
setup:
  grid_reset()     ← clear frozen[], place 8 bottom seeds
  spawn walkers at random top positions

main loop:
  1. resize → grid_reset, respawn

  2. dt, cap 100ms

  3. physics ticks:
     while sim_accum >= tick_ns:
       if not paused: scene_tick()
       sim_accum -= tick_ns

  4. frame cap sleep

  5. draw:
     erase()
     grid_draw()    ← all frozen cells with color '#'
     HUD: fps, walker count, speed
     wnoutrefresh + doupdate

  6. input:
     q/ESC → quit
     r     → grid_reset
     + =   → n_walkers++
     -     → n_walkers--
     ] [   → sim_fps ± step
     p/spc → toggle paused
```

---

## Interactions Between Modules

```
App
 └── scene_tick → walker_step → direction contact → freeze → tallest_row check → auto-reset
               → scene_draw → grid_draw → mvwaddch

Grid
 ├── frozen[][] — set once, never cleared within a cycle
 ├── color[][]  — set at freeze time from row_color()
 └── tallest_row — tracked incrementally; triggers auto-reset
```
