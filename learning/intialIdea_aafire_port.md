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
