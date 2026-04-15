# Pass 1 — aafire_port: aalib aafire Algorithm Port

## Core Idea

A faithful port of the aafire algorithm from the aalib library, but rendered through ncurses instead of aalib's ASCII art engine. The simulation uses a uint8 heat buffer (0–255) instead of fire.c's float grid, and the propagation rule uses 5 neighbours (including two rows below) instead of Doom's 3. The key visual difference: aafire produces rounded dome-shaped flames, while Doom fire produces sharp upward spires.

---

## The Mental Model

Think of the terminal as a heat map with values 0–255. The bottom two rows are "fuel" — they get seeded each tick with a heat pattern shaped like an arch (tall in the middle, zero at the edges). All other rows are updated by averaging their 5 neighbours from below and subtracting a cooling amount.

The seeding arch is not computed mathematically — instead, two counters (`i1` counting up, `i2` counting down) produce values that form an arch when you take `min(i1, i2)`. A warm-up counter (`height`) clamps the arch small at startup and lets it grow over many frames, so the fire builds from a cold start rather than appearing instantly.

Heat flows upward. At each cell the sum of 5 neighbours is fed into a precomputed decay table (`table[]`) that performs the division-by-5 averaging and subtracts a small cooling value in one lookup. This makes the CA extremely fast.

---

## Data Structures

### `Bitmap` struct
| Field | Meaning |
|---|---|
| `bmap[cols × (rows+2)]` | uint8 heat map; visible area is [0..rows-1], fuel rows are [rows] and [rows+1] |
| `prev[cols × rows]` | Previous frame's values; used for diff-based clearing (only cells that went from hot to cold get a space drawn) |
| `dither[cols × rows]` | Float working buffer for Floyd-Steinberg per frame |
| `table[MAXTABLE]` | Precomputed decay LUT: table[sum_of_5_neighbours] = cooled output |
| `height` | Frame counter for warm-up; increases each tick so fire grows from cold |
| `loop` | Countdown for fuel seeding bursts; occasionally introduces gaps |
| `sloop` | Counts how many seeding loops have run |
| `fuel` | User-adjustable intensity [0.1, 1.0] |
| `theme`, `cycle_tick` | Theme state |

### Why 2 extra rows?
`bmap` is allocated as `cols × (rows+2)`. The visible heat is rows 0 to rows-1. The fuel seeding writes to rows `rows` and `rows+1`. When `firemain()` computes `y+2` for the 5th neighbour, it reads from row `rows+1` without ever going out of bounds.

---

## The Main Loop

Same fixed-timestep pattern as all other files:
1. Compute `dt`
2. Add `dt` to `sim_accum`
3. While accumulator >= tick duration:
   - `drawfire()` — seed the two fuel rows
   - `firemain()` — propagate heat upward through all rows
   - Advance theme counter
   - Subtract tick duration
4. `bitmap_draw()` — gamma-correct, Floyd-Steinberg dither, draw to terminal with diff-based clearing
5. HUD, present

---

## Non-Obvious Decisions

### Why a precomputed decay table?
Instead of computing `(sum - minus) / 5` for every cell in every frame, the table precomputes all 1280 possible sum values. `firemain()` becomes a table lookup — very fast with CPU cache.

### Why `minus = 800 / rows`?
This makes cooling scale with screen height. On a tall terminal (large `rows`), `minus` is small → less cooling per row → fire burns higher. On a short terminal, more cooling → fire stays low. The fire automatically adapts to terminal size.

### Why 5 neighbours instead of Doom's 3?
Doom: reads 3 cells from one row below (x-1, x, x+1), picks one randomly.
aafire: reads 5 cells from two rows below (x-1, x, x+1 from y+1; x-1, x+1 from y+2).
The two-row lookahead gives flames more "vertical inertia" — they carry heat further upward, producing the rounded blob shapes vs Doom's spires.

### Why the i1/i2 arch construction?
`i1` starts at 1 and increases by 4 each step (left edge → center). `i2` starts at `4*cols+1` and decreases by 4 (right edge → center). `min(i1, i2)` is small at the edges and large in the middle — naturally producing an arch shape without any `sin()` or `sqrt()` call. This is the exact aafire algorithm.

### Why `height` as warm-up?
Each call to `drawfire()` increments `height`. The arch value is clamped to `min(arch_value, height)`, so at startup (height=0) everything seeds to 0. Over many frames, `height` grows and the arch reaches full amplitude. The fire "warms up" from nothing.

### Diff-based selective clearing (`prev[]`)
The `prev[]` buffer holds the uint8 heat values from the previous drawn frame. For cells that were hot (prev > 0) but are now cold (bmap = 0), a space character is written to erase the stale drawing. Cold cells that were already cold get no write at all — this avoids unnecessary terminal I/O and prevents flicker.

---

## State Machines

No state machine. Continuous CA. One boolean flag for paused.

---

## From the Source

**Physics/References:** Original aafire by Jakub Olszak, 1999, part of aalib 1.4 — the ASCII art rendering library. The CA and decay table design are faithful to the original aalib source.
**Algorithm:** 5-neighbour CA: `(x-1,y+1), (x,y+1), (x+1,y+1)` (one row below) plus `(x-1,y+2), (x+1,y+2)` (two rows below, no centre). The two-row lookahead gives vertical inertia — rounder blob flames vs Doom's 3-neighbour sharp spires. Edge cells clamp to nearest valid column rather than wrapping.

**Math:** Decay table: `table[i] = (i > minus) ? (i - minus) / 5 : 0` where `minus = 800 / rows` (min 1). Dividing by 5 averages the 5 neighbours back to [0,255]; subtracting `minus` before the division is what provides per-row cooling. MAXTABLE = 1280 = 5 × 256 covers the maximum possible neighbour sum.

**Physics:** Arch seeding uses two linear counters: `i1` starts at 1 and counts up by 4; `i2` starts at `4*cols+1` and counts down by 4. `min(i1, i2)` forms an arch — tall in the middle, zero at edges — with no `sin()` call. Within each burst of up to 6 cells, `last1` does a random walk of ±2, writing to both fuel rows. `height` (frame counter) clamps the arch cap to `min(arch, height)`, producing a cold-start warm-up.

**Performance:** All 1280 possible neighbour sums are precomputed once into `table[]` at init and again on every resize. `firemain()` inner loop is a single array lookup — zero arithmetic per cell. `gentable()` must be called whenever `rows` changes because `minus = 800 / rows`.

**Data-structure:** `bmap` is `cols × (rows+2)` uint8 — the 2 extra rows are the fuel rows. `prev[]` uint8 holds the previous drawn frame for diff-based clearing (only cells that went hot→cold get a space written). `dither[]` float is a scratch buffer rebuilt each render frame for Floyd-Steinberg.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect |
|---|---|---|
| `MAXTABLE` | 1280 | Must be >= 5 × 255; never change |
| `FUEL_DEFAULT` | 1.0 | Full intensity; lower = shorter dimmer flames |
| `800 / rows` (minus) | varies | Cooling per row; hardcoded in `gentable()` |
| `SIM_FPS_DEFAULT` | 30 | Steps/second; higher = faster burning flame |
| `CYCLE_TICKS` | 300 | Frames before automatic theme change |

---

## Comparison: fire.c vs aafire_port.c

| Aspect | fire.c | aafire_port.c |
|---|---|---|
| Heat type | float [0, 1.0] | uint8 [0, 255] |
| CA neighbours | 3 (one row below) | 5 (two rows below) |
| Fuel shape | Arch via math (t, edge_dist) | Arch via i1/i2 counters |
| Cooling | DECAY_BASE + DECAY_RAND per cell | `table[sum]` lookup |
| Flame shape | Sharp spires (Doom-style) | Rounded domes (aafire-style) |
| Warm-up | `warmup` counter, scale 0→1 | `height` counter, arch clamp |

---

## Open Questions for Pass 3

1. Try building the decay table yourself: compute `table[i]` for i=0..1279 with `minus=5`.
2. What happens if you change the arch from `min(i1,i2)` to a flat constant? How does the flame shape change?
3. Why does aafire use `rand()%6 - 2` (±2) for the fuel walk instead of a larger range?
4. What would happen if you used the fuel seeding pattern from aafire with the 3-neighbour propagation from fire.c?
5. The diff clearing uses `prev[]` as uint8 — what would break if you compared floats instead?

---

# Pass 2 — aafire_port: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | MAXTABLE=1280, FUEL_DEFAULT/STEP/MIN/MAX, SIM_FPS, CYCLE_TICKS |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 theme | Same as fire.c: 9-level ramp, LUT, FireTheme, 6 themes, theme_apply, ramp_attr |
| §4 bitmap | `Bitmap` struct; `gentable()` — decay LUT; `firemain()` — 5-neighbour CA; `drawfire()` — arch seeding; `bitmap_draw()` — render |
| §5 scene | Thin wrapper: owns Bitmap |
| §6 screen | ncurses init/resize/present/HUD |
| §7 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
gentable() at init:
  minus = 800 / rows  (at least 1)
  table[i] = (i > minus) ? (i - minus) / 5 : 0
      for i in 0..MAXTABLE-1

Each tick:

drawfire():  ← seed fuel rows
  height++
  i1 = 1, i2 = 4*cols+1
  x = 0
  while x < cols:
    cap = min(i1, i2, height)
    last1 = rand() % cap × fuel
    burst = rand() % 6
    for j in 0..burst:
      bmap[rows * cols + x]       = clamp(last1, 0, 255)
      last1 += rand() % 6 − 2
      bmap[(rows+1) * cols + x]   = clamp(last1, 0, 255)
      last1 += rand() % 6 − 2
      x++; i1 += 4; i2 -= 4
    x++; i1 += 4; i2 -= 4  ← skip gap cell

firemain():  ← propagate upward
  for y in 0..rows-1:
    for x in 0..cols-1:
      xl = clamp(x-1, 0, cols-1)
      xr = clamp(x+1, 0, cols-1)
      sum = bmap[(y+1)*cols+xl] + bmap[(y+1)*cols+x] + bmap[(y+1)*cols+xr]
            + bmap[(y+2)*cols+xl] + bmap[(y+2)*cols+xr]
      bmap[y*cols+x] = table[min(sum, MAXTABLE-1)]

bitmap_draw():  ← render frame
  for each cell (x, y):
    raw = bmap[y*cols+x]
    if raw == 0:
      if prev[y*cols+x] > 0:    ← was hot last frame
        mvaddch(y, x, ' ')       ← erase stale char
      prev[y*cols+x] = 0
      continue
    v = pow(raw / 255.0, 1/2.2) ← gamma correction
    dither[y*cols+x] = v

  Floyd-Steinberg pass (same as fire.c):
    for each warm cell:
      idx = lut_index(dither[i])
      err = dither[i] − lut_midpoint(idx)
      distribute err * (7/16, 3/16, 5/16, 1/16) to warm neighbours
      draw k_ramp[idx] with ramp_attr(idx, theme)

  copy bmap[0..rows*cols] → prev[]
```

---

## Function Breakdown

### gentable(bitmap)
Purpose: precompute the decay lookup table
Steps:
1. `minus = 800 / rows` (min 1)
2. For i in 0..MAXTABLE-1:
   - If i > minus: `table[i] = (i - minus) / 5`
   - Else: `table[i] = 0`
Why: avoids division + subtraction in the inner loop. All cells mapped in O(1).

---

### drawfire(bitmap)
Purpose: seed fuel rows with arch-shaped heat values
Steps:
1. `height++`, `loop--`
2. If loop < 0: reset `loop = rand()%3`, `sloop++`
3. `i1 = 1`, `i2 = 4*cols+1`, `x = 0`
4. While x < cols:
   a. `cap = min(i1, i2, height)` — arch value at this x
   b. `last1 = (rand() % cap) × fuel`
   c. `burst = rand() % 6`
   d. For j in 0..burst (and x < cols):
      - Clamp last1 to [0, 255]
      - Write `fuel_row[x] = last1`
      - `last1 += rand()%6 − 2` (random walk ±2)
      - Clamp last1 again
      - Write `fuel_row2[x] = last1`
      - `last1 += rand()%6 − 2`
      - Advance x, i1 += 4, i2 -= 4
   e. After burst: x++, i1 += 4, i2 -= 4 (skip one gap cell)
Edge cases: cap always >= 1 to avoid modulo-zero crash

---

### firemain(bitmap)
Purpose: propagate heat upward through all visible rows
Steps:
1. For y in 0..rows-1:
   For x in 0..cols-1:
   a. xl = clamp(x-1, 0, cols-1)
   b. xr = clamp(x+1, 0, cols-1)
   c. sum = 5 neighbours: (y+1)×[xl, x, xr] + (y+2)×[xl, xr]
   d. `bmap[y*cols+x] = table[min(sum, MAXTABLE-1)]`
Note: reading from rows `y+1` and `y+2` — the extra 2 allocated rows ensure row `rows+1` is always valid

---

### bitmap_draw(bitmap, tcols, trows)
Purpose: render the heat buffer to terminal with dithering and diff clearing
Steps:
1. Gamma correct: for each cell, if raw > 0: `dither[i] = pow(raw/255, 1/2.2)`
2. Floyd-Steinberg dither + draw (same algorithm as fire.c)
3. After drawing: `memcpy(prev, bmap_visible_region, cols*rows)`
Edge cases: cells outside terminal bounds (tcols/trows) skipped

---

## Pseudocode — Core Loop

```
setup:
  screen_init(theme=0)
  bitmap_alloc(cols, rows)
  gentable()   ← build table once per size
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       bitmap_free(); bitmap_alloc(new cols, rows)
       gentable()   ← rebuild table for new rows
       reset sim_accum, frame_time

  2. dt (capped at 100ms)

  3. physics ticks:
     sim_accum += dt
     while sim_accum >= tick_ns:
       if not paused:
         drawfire()   ← seed fuel rows
         firemain()   ← propagate
         cycle_tick++ → auto-theme every CYCLE_TICKS
       sim_accum -= tick_ns

  4. FPS counter (every 500ms)

  5. frame cap: sleep to 60fps

  6. draw + present:
     bitmap_draw(tcols, trows)   ← dither + diff clear
     HUD: mvprintw (fps, theme, fuel)
     wnoutrefresh + doupdate

  7. input:
     q/ESC → quit
     space → toggle paused
     ] / [ → sim_fps ± 5
     t     → advance theme manually
     g / G → fuel −/+ 0.05
```

---

## Interactions Between Modules

```
App
 └── main loop → drawfire + firemain → bitmap_draw → doupdate

Bitmap
 ├── bmap[cols*(rows+2)]  — heat (rows 0..rows-1 visible, rows/rows+1 fuel)
 ├── prev[cols*rows]      — previous frame for diff clearing
 ├── dither[cols*rows]    — Floyd-Steinberg working buffer
 └── table[1280]          — decay LUT built once per terminal size

§3 theme (shared code with fire.c)
 └── theme_apply, ramp_attr, lut_index, lut_midpoint
```

---

## The Decay Table Trick

```
Normal per-cell computation:
  output = max(0, (neighbour_sum - cooling) / num_neighbours)
  = (sum - minus) / 5

With precomputed table:
  table[i] = (i > minus) ? (i - minus) / 5 : 0
  output = table[sum]   ← one array lookup, zero arithmetic

For a 200×50 terminal: 200×50 = 10,000 cells/tick.
Without table: 10,000 × (2 ops) = 20,000 ops/tick.
With table: 10,000 × (1 lookup) — faster, cache-friendly.
```

---

## Why the Fire Looks Different from fire.c

```
fire.c (Doom):             aafire_port.c:

bottom: flat line           bottom: arch dome
rule:   3 neighbours        rule:   5 neighbours (incl. 2 rows below)
result: sharp tall spires   result: rounded blob flames

The arch seeding + 5-neighbour rule together produce the distinctive
aafire silhouette: tall rounded middle, tapering to zero at both edges.
```
