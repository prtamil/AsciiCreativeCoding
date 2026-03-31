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
