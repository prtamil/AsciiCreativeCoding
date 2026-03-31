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
