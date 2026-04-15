# Pass 1 — sand.c: Cellular automaton falling sand simulation with age-based visuals and wind

## From the Source

**Algorithm:** Cellular Automaton (CA) with probabilistic update order. Rules applied bottom-to-top (row y = rows-1 → 0) so grains that fell this tick don't cascade infinitely in one step. Column order is shuffled each row (Fisher-Yates) to prevent directional bias — without shuffling, sand would always prefer the left diagonal.

**Physics:** Models angle of repose — the maximum slope stable sand can maintain before avalanching. The two-diagonal CA rule approximates this: a grain slides diagonally if the cell directly below is blocked, creating ~45° pile slopes. Real dry sand angle of repose ≈ 30–35°; CA gives ~45° because only one diagonal is checked at a time (no multi-step sliding).

**Wind:** Probabilistic horizontal drift for unsettled grains (age < AGE_SMALL). P(drift) = |wind| / WIND_PROB_DEN. Only young grains drift — settled sand is too heavy to move, matching the physical model of light airborne particles.

**Performance:** O(W×H) per tick. The `moved[]` flag array prevents double-processing; without it a grain could move multiple cells per tick. Per-tick scratch arrays (nxt, nxt_age) avoid in-place mutation during the scan (double-buffering pattern). Per-grain age (stationary ticks) encodes visual compaction; age resets to 0 on any movement.

---

## Core Idea

A 2D grid of cells, each either empty or containing one grain of sand. Every tick, each grain tries to fall downward following simple physics rules. Grains that have been sitting still for a long time change their visual appearance from bright individual specks to dark compressed mass, simulating compaction over time. A wind force can make airborne grains drift sideways.

## The Mental Model

Think of a phone screen held upright, divided into a grid of small squares. You pour sand from a point near the top. Each grain of sand checks: can I fall straight down? If yes, fall. If no, can I slide diagonally down-left or down-right? If yes, slide. If neither works, is there wind? If yes and I'm light enough (young enough), drift sideways. If nothing works, sit still and age.

Sand accumulates from the bottom up in natural pile shapes. The angle of repose emerges naturally from the diagonal-fall rules: once a grain's diagonal path is also blocked, it stays put, and the pile builds at roughly 45 degrees.

The age system makes the simulation look alive: fresh grains falling from the emitter are bright yellow specks. Grains that just landed are dots. Grains that have been sitting still for a while darken through gold to amber to brown-black dense base. The color gradient visually shows depth and compression — the base of a deep pile looks dark and packed, the surface fresh and active.

## Data Structures

### Grid
```
cur[rows*cols]      — current state: 0=empty, 1=sand
nxt[rows*cols]      — next state being computed this tick
age[rows*cols]      — how many consecutive ticks this grain has been stationary
nxt_age[rows*cols]  — age for next state
moved[rows*cols]    — bool: has this cell been processed this tick?
cols, rows          — grid dimensions (match terminal size)
wind                — integer in [-3, +3]; positive = rightward
```

The double-buffer pattern (cur/nxt, age/nxt_age) is crucial: every grain reads from `cur` and writes to `nxt`. After all grains are processed, `cur` and `nxt` are pointer-swapped. This prevents any grain from affecting another grain in the same tick.

The `moved` array solves the "double-move" problem: without it, a grain that falls into a cell could be picked up and moved again in the same sweep. Once a grain is processed (either moved or stayed), its destination cell is marked so it won't be moved again.

### Source
```
col     — horizontal position of the emitter (controllable with arrow keys)
w       — width of the emitter in cells (adjustable with = and -)
on      — bool: is the emitter active?
```

### Scene
```
grid    — the Grid struct
source  — the Source struct
paused  — bool
```

## The Main Loop

Each iteration:

1. Check for terminal resize. If resized, rebuild the grid at the new dimensions (existing sand is cleared, wind is preserved).
2. Measure real time elapsed since last iteration (dt).
3. Cap dt at 100ms to prevent physics explosions after a pause.
4. Add dt to the simulation accumulator. While the accumulator holds enough time for a full tick (1/sim_fps seconds), run scene_tick once.
5. Compute alpha (leftover fraction) — not used for drawing because sand cells snap to integer positions, but computed for structural consistency.
6. Update the fps display counter.
7. Sleep to cap rendering at 60fps.
8. Draw the current grid state.
9. Render the HUD (status bar) and wind indicator.
10. Present to terminal.
11. Poll for a keypress and handle it.

### Inside scene_tick:
1. If paused, return immediately.
2. Call source_emit: place fresh grains at age=0 in the emitter cells.
3. Call grid_tick: advance the cellular automaton one step.

### Inside grid_tick:
1. Clear nxt and nxt_age to zeros, clear moved to false.
2. Allocate a column-order array [0, 1, 2, ..., cols-1].
3. Process rows from bottom to top (y = rows-1 down to 0).
4. For each row, shuffle the column order using Fisher-Yates shuffle.
5. For each column in the shuffled order, call grid_update_cell.
6. After all cells processed, swap cur↔nxt and age↔nxt_age pointers.

### Inside grid_update_cell(x, y):
1. If not sand, return.
2. If already moved this tick, return.
3. Try straight down (x, y+1): if in bounds and empty and not moved, gmove and return.
4. Try diagonal down: pick left or right randomly (prevents left/right bias). Try the first choice; if blocked, try the other. If either works, gmove and return.
5. If wind is nonzero and grain is young (age < AGE_SMALL): with probability abs(wind)/WIND_PROB_DEN, try to drift one cell sideways in the wind direction. If the target is empty and not moved, gmove and return.
6. If none of the above: grain stays. Copy to nxt, increment age (cap at AGE_MAX), mark as moved.

## Non-Obvious Decisions

### Bottom-to-top row processing
Rows are processed from bottom to top. This is essential: if you processed top-to-bottom, a grain at the top would fall down, and then when you processed the row below, the grain you just moved there might try to fall down again — one grain could fall multiple rows in a single tick, making the sand fall infinitely fast. Processing bottom-up means grains can only fall into rows that have already been committed for this tick.

### Column shuffle per row (the anti-bias shuffle)
If columns are always processed left-to-right, diagonal falling has a systematic left-right asymmetry: grains always prefer left. By shuffling the column order randomly each row, left and right diagonals are chosen with equal frequency over time, producing natural symmetric pile shapes. This is the core trick that makes sand simulations look correct.

### Diagonal priority randomization
Within grid_update_cell, diagonal directions are also randomized: `a = rand()&1 ? -1 : 1; b = -a`. This randomizes whether left or right diagonal is tried first for this specific grain, adding another layer of symmetry.

### The moved[] array prevents double-counting
Without `moved`, grain A at (5,3) falls to (5,4). Then when the sweep reaches (5,4), it finds a grain and tries to move it again. With `moved`, after A is placed at (5,4), that cell is marked — the sweep skips it.

### Wind only affects young grains (age < AGE_SMALL = 12)
Settled grains should not be blown sideways — that would cause unrealistic avalanches from the base of a pile. Only grains that have been stationary for fewer than 12 ticks (just landed or recently disturbed) can drift with wind. Deep-packed grains are immune.

### Wind probability is fractional (abs(wind)/WIND_PROB_DEN)
Wind strength 1 = 25% chance of drift per tick. Wind strength 3 = 75% chance. This makes wind feel gradual rather than binary, and allows intermediate wind speeds.

### Source emitter leans with wind (wind_offset)
When wind is active, the emitter spawns grains offset by `g->wind` cells in the wind direction. This makes the stream visibly lean, matching physical intuition — a stream of sand poured in wind deflects before falling.

### Age capped at AGE_MAX=200, not unlimited
Uncapped ages would eventually overflow uint8_t (which maxes at 255). AGE_MAX is set to 200, safely below 255. Once a grain reaches AGE_MAX it stops aging visually — the darkest '#' character is reached and maintained.

### Neighbor count in grain_visual suppresses bright chars in dense piles
A grain freshly dropped inside a tightly packed pile would otherwise show as a bright '`' character (age=0) surrounded by dark '#' characters — visually jarring. The `grain_visual` function checks how many of the 8 neighboring cells contain sand: if nb >= 5, force effective age up to at least AGE_MID; if nb >= 3, force at least AGE_SMALL. This makes grains inside dense piles look dark regardless of their true age.

## State Machines

The simulation has no formal state machine, but each grain's age drives a visual state:

```
[grain falls into empty cell]
  age=0: glyph='`'  color=pale-yellow  (airborne/spawned)
       │
       │ grain stays put tick 1..2
       ▼
  age=3..11: glyph='.'  color=pale-yellow  (just landed)
       │
       │ grain stays put tick 12..29
       ▼
  age=12..29: glyph='o'  color=golden  (settling)
       │
       │ grain stays put tick 30..59
       ▼
  age=30..59: glyph='O'  color=amber  (settled surface)
       │
       │ grain stays put tick 60..119
       ▼
  age=60..119: glyph='0'  color=dark-amber  (packed mid)
       │
       │ grain stays put tick 120+
       ▼
  age>=120: glyph='#'  color=dark-brown  (dense base)

[grain moves] → age resets to 0 → back to '`'
```

Any grain that moves in a tick has its age reset to 0 regardless of how old it was.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| SIM_FPS_DEFAULT | 30 | More ticks per second → sand falls faster |
| SOURCE_W_DEFAULT | 3 | Wider emitter → wider stream |
| SOURCE_W_MAX | 30 | Limits how wide the stream can be |
| WIND_MAX | 3 | Maximum wind strength |
| WIND_PROB_DEN | 4 | Higher denominator → lower drift probability per unit of wind strength |
| AGE_DOT | 3 | Lower → grains switch from '`' to '.' sooner |
| AGE_SMALL | 12 | Lower → grains become immune to wind sooner |
| AGE_MID | 30 | Lower → grains become 'O' amber sooner |
| AGE_PACK | 60 | Lower → grains compress faster |
| AGE_DENSE | 120 | Lower → base turns '#' sooner |
| AGE_MAX | 200 | Cap on age value; doesn't affect appearance (all grains reach '#' at 120) |
| SOURCE_ROW | 1 | Row where grains are emitted (1 = near top) |

## Open Questions for Pass 3

- The grid uses a flat 1D array indexed as `y*cols+x`. What happens at the boundary checks when x or y equals cols or rows exactly? Trace `gin()` for edge cases.
- The Fisher-Yates shuffle allocates a new `order` array every tick with malloc/free. For a large terminal (e.g. 200 columns), is this allocation measurable? Would a stack array be better?
- What determines the natural angle of repose in this CA? Is it always 45 degrees, or can different diagonal-probability settings produce different slopes?
- The `gmove` function marks both source and destination cells as moved. Why mark the destination? Could a grain fall into a marked destination and then be moved again somehow?
- When the emitter width is wide (SOURCE_W_MAX = 30) and wind leans the stream, grains get clamped to column 0 or cols-1. What does this look like visually — does it create a wall effect?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `Grid.cur` | `uint8_t*` (malloc'd, cols×rows) | ~20 KB at 200×100 | current sand grid: 0=empty, 1=grain |
| `Grid.nxt` | `uint8_t*` (malloc'd) | ~20 KB | next-state double-buffer; swapped after each tick |
| `Grid.age` | `uint8_t*` (malloc'd) | ~20 KB | per-grain stationary-tick counter; drives visual character |
| `Grid.nxt_age` | `uint8_t*` (malloc'd) | ~20 KB | next-state age buffer |
| `Grid.moved` | `bool*` (malloc'd) | ~20 KB | per-cell processed flag; prevents double-moving grains |
| `Source` | `struct { int col, w; bool on; }` | 12 B | emitter position, width, and on/off state |

---

# Pass 2 — sand: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | COLS/ROWS defaults, SOURCE_ROW=1, WIND_MAX, color pairs |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | grain age color pairs; dark/light variants |
| §4 grid | `Grid` struct; `grid_init()`, `grid_tick()`, `grid_draw()`, `grid_update_cell()` |
| §5 source | `source_emit()` — fills SOURCE_ROW with new grains |
| §6 screen | ncurses init/resize/present/HUD |
| §7 app | Main dt loop, wind input, signals |

---

## Data Flow Diagram

```
grid_tick():
  source_emit(grid, cols, wind_offset)    ← fill top row SOURCE_ROW

  Fisher-Yates shuffle column order[]:
    for i = cols-1 downto 1:
      j = rand() % (i+1)
      swap order[i], order[j]

  for row = rows-1 downto SOURCE_ROW+1:  ← bottom-to-top
    for each col in order[]:             ← randomized left-right
      grid_update_cell(grid, col, row, cols, rows, wind)

  swap cur ↔ nxt
  swap age ↔ nxt_age
  memset moved[] to false

grid_update_cell(grid, x, y, cols, rows, wind):
  if cur[y][x] == 0: return              ← empty cell, skip
  if moved[y][x]: return                 ← already moved this tick

  grain = cur[y][x]
  new_age = age[y][x] + 1

  // Rule 1: straight down
  if y+1 < rows and cur[y+1][x] == 0:
    nxt[y+1][x] = grain; nxt_age[y+1][x] = new_age
    moved[y+1][x] = true; return

  // Rule 2: diagonal (Fisher-Yates randomizes which diagonal first)
  dirs[] = {-1, +1}
  Fisher-Yates shuffle dirs (2 elements)
  for d in dirs:
    nx = x + d
    if nx in bounds and cur[y+1][nx] == 0:
      nxt[y+1][nx] = grain; nxt_age[y+1][nx] = new_age
      moved[y+1][nx] = true; return

  // Rule 3: wind drift (horizontal)
  wx = x + sign(wind)
  p_drift = abs(wind) / WIND_MAX   ← probability
  if rand()/RAND_MAX < p_drift and cur[y][wx] == 0:
    nxt[y][wx] = grain; nxt_age[y][wx] = new_age
    moved[y][wx] = true; return

  // Rule 4: at rest
  nxt[y][x] = grain; nxt_age[y][x] = new_age

grain_visual(age, nb):    ← nb = neighbor count
  effective_age = max(age, old_threshold_if_nb_high)
  if effective_age < T1:  return ('.', PAIR_YOUNG)
  if effective_age < T2:  return ('o', PAIR_MID)
  if effective_age < T3:  return ('O', PAIR_OLD)
  if effective_age < T4:  return ('0', PAIR_OLDER)
  if effective_age < T5:  return ('#', PAIR_ANCIENT)
  return ('#', PAIR_ROCK)

grid_draw(grid, w, cols, rows):
  for each cell:
    if cur[y][x] != 0:
      nb = count_neighbors(cur, x, y, cols, rows)
      (ch, pair) = grain_visual(age[y][x], nb)
      mvwaddch(y, x, ch) with COLOR_PAIR(pair)
    else:
      mvwaddch(y, x, ' ')   ← erase old grain
```

---

## Function Breakdown

### grid_init(grid, cols, rows)
Purpose: allocate and zero all grid arrays
Steps:
1. Malloc `cur[rows*cols]`, `nxt[rows*cols]`, `age[rows*cols]`, `nxt_age[rows*cols]`, `moved[rows*cols]`
2. Malloc `order[cols]` — column shuffle array
3. Zero all arrays via memset
4. Initialize `order[i] = i` for i in 0..cols-1

---

### source_emit(grid, cols, wind_offset)
Purpose: place new grains at top of screen
Steps:
1. For each x in 0..cols-1:
   - target_x = x + wind_offset (clamped to bounds)
   - If `cur[SOURCE_ROW][target_x] == 0`:
     - Fill with grain character
     - age[SOURCE_ROW][target_x] = 0

---

### grid_update_cell(grid, x, y, cols, rows, wind)
Purpose: apply CA rules to one grain
Steps:
1. Skip if empty or already moved
2. Try straight down — move if empty
3. Shuffle diagonal directions; try each
4. Try wind drift with probability
5. Stay in place (at rest), increment age

---

### grain_visual(age, nb)
Purpose: map grain age + neighbor density to visual char/color
Steps:
1. If nb >= BURY_THRESHOLD: age = max(age, BURIED_MIN_AGE)
   — buried grains look older even if recently placed
2. Compare age against 5 thresholds → return (char, color_pair)

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  grid_init(cols, rows)
  wind = 0
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       grid_free()
       grid_init(new cols, rows)
       reset sim_accum, frame_time

  2. dt (capped at 100ms)

  3. physics ticks:
     sim_accum += dt
     while sim_accum >= tick_ns:
       grid_tick(grid, cols, rows, wind)
       sim_accum -= tick_ns

  4. FPS counter

  5. frame cap: sleep to 60fps

  6. draw:
     erase()
     grid_draw(grid, stdscr, cols, rows)
     HUD: fps, wind value, grain count
     wnoutrefresh + doupdate

  7. input:
     q/ESC     → quit
     LEFT/h    → wind -= 1 (clamped to -WIND_MAX)
     RIGHT/l   → wind += 1 (clamped to +WIND_MAX)
     0         → wind = 0
     r         → grid_free + grid_init (clear all grains)
     ] / [     → sim_fps ± step
```

---

## Interactions Between Modules

```
App
 └── owns Grid (cur/nxt double buffer + age + moved + order)

Grid
 ├── grid_tick() — CA rules per cell
 │    ├── source_emit() fills SOURCE_ROW
 │    ├── Fisher-Yates shuffles order[] (column randomization)
 │    └── grid_update_cell() per (col, row) bottom-to-top
 └── grid_draw() — grain_visual() maps age+neighbors → char+color

§4 grain_visual
 └── called per live cell in grid_draw
     uses age[] and neighbor count from cur[]
```

---

## CA Rule Priority (Why This Order Matters)

```
Rule 1 (down) must be first:
  Gravity wins over diagonal or drift.
  If checked after diagonal, grains pile oddly.

Rule 2 (diagonal) must come before rest:
  Allows grains to slide off piles naturally.
  Fisher-Yates on {-1,+1} prevents left-bias in diagonal preference.

Rule 3 (wind drift) only if can't fall:
  Wind moves resting-surface grains, not airborne ones.
  Probability = abs(wind)/WIND_MAX gives smooth control.

Rule 4 (at rest):
  Grain stays, age increments.
  This drives the visual aging from young ('.') to ancient ('#').
```

---

## Double-Buffer Swap Pattern

```
Each tick:
  grid_update_cell() writes to nxt[], nxt_age[]
  moved[] prevents a grain from being moved twice (once written to nxt, skip original)

After all cells processed:
  swap pointer: cur ↔ nxt, age ↔ nxt_age
  memset moved[] = 0

Why memset moved[]:
  Without it, grains moved early in the row order could be moved again
  when the scanning cursor reaches their new position.
  moved[] is the "already handled" flag for this tick.
```

---

## Fisher-Yates in Two Places

```
1. Column order shuffle (grid_tick):
   Ensures no left-to-right or right-to-left bias in which grain
   moves first when two compete for the same target cell.
   Without it: sand always piles with a slight lean.

2. Diagonal direction shuffle (grid_update_cell):
   Ensures no left-bias or right-bias when a grain can fall
   either way. Without it: piles always slope to one side.
```
