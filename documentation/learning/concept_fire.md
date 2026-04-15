# Pass 1 — fire.c: Doom-style cellular automaton fire with Floyd-Steinberg dithering

## Core Idea

A 2-D grid of floating-point heat values simulates flame using a simple physics rule lifted from the original Doom game engine: heat rises from a "fuel row" at the bottom, spreads slightly left or right as it propagates upward, and cools as it travels. The heat values are converted to ASCII characters and terminal colors through a perceptual rendering pipeline involving gamma correction and Floyd-Steinberg error-diffusion dithering. The result is a flame that flickers realistically with natural spire shapes, gradients, and color. Six color themes (fire, ice, plasma, nova, poison, gold) cycle automatically every 300 ticks.

## The Mental Model

Picture a campfire from above, rotated so you look at it side-on. The ground is the bottom row of the terminal. It is constantly "hot" — that is the fuel. Each frame, every particle of heat above the ground copies itself one row up while losing some energy. It also randomly drifts one cell left or right as it rises, which makes the flames spread and sway. Flames that have lost all their energy go cold and become empty space.

The display converts the raw heat number (0.0 to 1.0) into one of 9 ASCII characters (space, dot, colon, plus, x, star, X, hash, @) ordered by visual density. Characters are matched to the theme's gradient of colors. The clever part is the dithering: rather than simply rounding each heat value to the nearest character level, the rounding error is propagated to neighboring cells before they are drawn. This makes the gradient look much smoother — you see subtle transitions instead of hard boundaries.

The fuel row is shaped like an arch — hot in the center, fading at the edges, with a 10% margin of no-fuel on each side. This creates the characteristic bonfire silhouette. A wind control shifts this arch left or right each tick, making the whole flame lean.

## Data Structures

### Grid (§4)
```
float *heat;      — [rows * cols] current heat values, 0.0 to MAX_HEAT (1.0)
float *prev_heat; — [rows * cols] heat values from the previous drawn frame
float *dither;    — [rows * cols] working buffer for Floyd-Steinberg errors
int    cols, rows;
float  fuel;      — user-controlled flame intensity [0.1, 1.0]
int    wind;      — wind velocity in cells/tick, range -WIND_MAX to +WIND_MAX
int    wind_acc;  — accumulated wind offset for the fuel row shift
int    theme;     — index into k_themes[] (0 = fire, 1 = ice, ...)
int    cycle_tick; — counts down to next auto-theme-cycle
int    warmup;    — frame counter 0..80, controls startup intensity
```
The simulation and rendering state are both in this one struct. `heat` is the active grid being read/written this tick. After drawing, `heat` and `prev_heat` are swapped — `prev_heat` becomes what was just shown, `heat` is cleared for the next tick. `dither` is a scratch buffer that is filled once per frame and discarded.

### FireTheme (§3)
```
const char *name;
int fg256[RAMP_N]; — 9 xterm-256 color indices, one per ramp level (cold to hot)
int fg8[RAMP_N];   — 9 standard-8 color fallbacks
attr_t attr8[RAMP_N]; — A_DIM / A_BOLD modifiers for 8-color mode
```
One theme = one color gradient. 9 levels from coldest (index 0, near-black) to hottest (index 8, bright white). The fire theme goes dark-gray → red → orange → yellow → white. The ice theme goes dark-blue → cyan → white.

### Scene (§5)
```
Grid  grid;
bool  paused;
bool  needs_clear; — true after resize or theme change — triggers one full erase()
```
Thin wrapper that owns the Grid and tracks two control flags.

## The Main Loop

Each iteration:

1. **Resize check**: If terminal changed size, rebuild the grid at new dimensions (preserving fuel and wind settings), set `needs_clear = true`.

2. **Measure dt**: Nanosecond delta-time; clamp to 100ms.

3. **Drain sim accumulator**: Call `scene_tick()` (which calls `grid_tick()`) as many times as needed to catch up to real time at sim_fps.

4. **FPS counter update**: Every 500ms, recalculate displayed FPS.

5. **Frame cap sleep**: Sleep to stay at ~60fps render rate. Sleep before draw.

6. **Draw**: `screen_draw()` calls `grid_draw()`. If `needs_clear` is set, call `erase()` once, then clear the flag. Normally `erase()` is never called — the fire manages its own cell clearing via the `prev_heat` diff.

7. **HUD**: Write FPS, theme name, fuel level, wind indicator, sim speed.

8. **Present**: `wnoutrefresh()` + `doupdate()`.

9. **Input**: Handle keys for pause, theme, fuel, wind, speed.

## Non-Obvious Decisions

### Why not call erase() every frame?
Calling `erase()` every frame would fill every terminal cell with a space character. The fire only occupies the bottom portion of the screen (the upper area is black background). If you erased everything, the area above the flames would appear as a solid black rectangle with a visible bounding box — it would look like the fire is inside a box rather than rising freely into infinite darkness. Instead, `grid_draw()` only writes a space character for cells that were hot last frame but are cold this frame. All other cold cells are simply left alone by ncurses' diff engine, keeping the terminal background seamless.

### The prev_heat swap pattern
After drawing, the code does `float *tmp = prev_heat; prev_heat = heat; heat = tmp; memcpy(heat, prev_heat, ...)`. This gives you: old heat (what was drawn) in `prev_heat`, and a fresh copy of it in `heat` for next tick's starting point. The swap avoids a memcpy of `prev_heat` — you just swap pointers. The subsequent memcpy re-seeds `heat` so the simulation continues from the current state rather than garbage.

### Floyd-Steinberg dithering: why?
The heat grid is a continuous float field. The display only has 9 levels of ASCII characters. Without dithering, you'd see hard staircase boundaries between levels — the flame would look like it has distinct bands rather than a smooth gradient. Floyd-Steinberg propagates the quantization error of each cell to its right and below-neighbors before they are drawn. This means locally you always display the nearest level, but the errors average out over a region, producing an apparent gradient that is much smoother than the 9-level character set would suggest.

### Gamma correction before dithering
`v = powf(v / MAX_HEAT, 1.0f / 2.2f)`. Human perception of brightness is nonlinear — we are much more sensitive to dark values than bright ones. The raw linear heat value in [0,1] would allocate too many ramp levels at the bright end and not enough at the dark end, making the fire look flat. Raising to power 1/2.2 (inverting the sRGB gamma curve) redistributes the range so that darker flame intensities get more character levels, matching human perceptual sensitivity. The LUT break-points are then clustered in the mid-range [0.3–0.75] after gamma, which covers the visually richest part of the flame body.

### The fuel arch shape with margins
The fuel row is not uniformly hot across its full width. Instead, it has an arch profile: zero at the left and right edges, maximum at the center, with the squared arch formula making the rise steeper (more like a bonfire base than a gentle hill). The 10% margin on each side keeps the flame away from terminal edges, making it look like it is floating in space rather than clamped to the sides. The squaring (`arch = arch * arch`) sharpens the edge of the fuel zone: the flame drops off quickly outside the center rather than having a gradual taper.

### Warmup multiplier
On startup or after a theme change, `warmup` starts at 0 and counts to 80. The fuel intensity is multiplied by `warmup / 80.0`. This means the flame starts from cold and builds to full intensity over 80 ticks rather than instantly appearing at full brightness. It creates a visually satisfying "flame igniting" effect.

### Doom fire propagation: upward-only with random horizontal drift
The update loop iterates y from 0 (top) to rows-2 (one above the fuel row). For each cell, it samples from ONE cell below at a random horizontal offset of -1, 0, or +1. The randomness comes before the lookup, not after — this is not jitter applied to the result, it is truly sampling a different source cell. This is why flames sway and wander rather than rising straight up.

### Decay formula: DECAY_BASE + rand * DECAY_RAND
The fixed `DECAY_BASE = 0.040f` ensures heat always decreases as it rises (the flame must cool). The random `DECAY_RAND = 0.060f` component adds up to 50% additional random cooling on any given cell. The wide random range creates the ragged tops of spires — some cells cool quickly and terminate early, while others survive longer and rise higher.

## State Machines

The Grid simulation has no state machine — it is a pure functional update. However, the Scene has a weak two-state lifecycle:

```
RUNNING ──── space key ────► PAUSED
   ▲                              │
   └───────── space key ──────────┘
```

And the theme has an implicit cycle:
```
theme 0 (fire) ──── 300 ticks ────► theme 1 (ice) ──── 300 ticks ────► ... ──► theme 0
               or 't' key at any time
```

## From the Source

**Algorithm:** Doom 3-neighbour rule: for each cell `(x, y)`, a random horizontal offset `rx = x + rand()%3 - 1` is applied, then `src = heat[(y+1)*cols + clamp(rx)]`. Only one cell is sampled (not averaged), and the randomness is in the source address, not the result — this is why flames sway rather than rising straight.

**Math:** Decay is computed per-tick from screen height: `target = rows × 0.75; avg_d = MAX_HEAT / target`. Split: `d_base = avg_d × 0.55; d_rand = avg_d × 0.90`. This ensures the average flame peak always reaches ~75% of screen height regardless of terminal size. Floor-bounded by `DECAY_BASE_MIN=0.010` and `DECAY_RAND_MIN=0.015`.

**Physics:** Arch fuel seeding: `t = (x - margin - wind_acc) / span` where margin=4%, span=92% of cols. `arch = (min(t, 1-t) × 2)²` — squared to steepen the edge rise. Wind shifts the arch sampling position by `wind_acc` each tick (cyclic, resets if `>= cols`). `warmup_scale = min(tick / 80, 1.0)` ramps the fuel from 0→1 over first 80 ticks.

**Performance:** Diff-based clearing: `prev_heat` stores what was drawn last frame; cold cells that were also cold last frame get no write at all. After `grid_draw()`, `heat ↔ prev_heat` pointers are swapped, then `memcpy(heat, prev_heat)` re-seeds heat for next tick — one memcpy instead of two.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect if changed |
|---|---|---|
| `DECAY_BASE` | 0.040 | Higher = shorter flames (more aggressive cooling); lower = taller flames |
| `DECAY_RAND` | 0.060 | Higher = more variation in flame height (jagged tops); lower = more uniform |
| `MAX_HEAT` | 1.0 | Scale factor only; raising it while keeping decay the same has no effect |
| `CYCLE_TICKS` | 300 | More = longer time on each theme; fewer = rapid theme cycling |
| `WIND_MAX` | 3 | Maximum wind offset in cells/tick; higher = stronger lean |
| `RAMP_N` | 9 | Number of distinct display levels; adding levels would require matching theme color entries |
| warmup limit (80) | 80 ticks | Faster = quicker ignition; slower = longer cold startup |
| fuel margin (0.10) | 10% each side | Higher = narrower base flame (taller but thinner); lower = fuel extends to edges |

## Open Questions for Pass 3

- Floyd-Steinberg normally distributes error right, lower-left, lower, lower-right (7/16, 3/16, 5/16, 1/16). The code only does this for "warm neighbours" (`if (d[i+1] >= 0.f)`). What visual effect does skipping cold neighbours have? Would including cold-to-cold error propagation matter?
- Why does `grid_draw` iterate top-to-bottom (y=0 first) for Floyd-Steinberg? Heat rises from the bottom — does the scan direction matter for flame aesthetics?
- The code swaps `heat` and `prev_heat` pointers but then immediately memcpy's back. Why not just keep `prev_heat` as a separate snapshot? What exactly would break if you removed the memcpy?
- What happens at the left and right edges of the grid during the Doom propagation step? The code clamps `rx` to [0, cols-1]. Does this mean edge columns accumulate slightly more heat than interior columns (the clamped offset samples the same neighbour more often)?
- The `needs_clear` flag triggers one `erase()` call. What visual artifact appears if you resize and don't erase — why isn't the normal diff-based clearing sufficient after a resize?

---

# Pass 2 — fire: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | MAX_HEAT, DECAY_BASE/RAND, WIND_MAX, CYCLE_TICKS, SIM_FPS |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 theme | 9-level ASCII ramp, gamma-corrected LUT breaks, FireTheme struct, 6 themes, `theme_apply()`, `ramp_attr()` |
| §4 grid | `Grid` struct; `grid_tick()` — Doom CA + arch fuel; `grid_draw()` — Floyd-Steinberg + diff clear |
| §5 scene | Owns Grid; thin wrappers for tick/draw/resize |
| §6 screen | ncurses init/draw/present/resize; HUD |
| §7 app | Signal handlers, main dt loop, key handling |

---

## Data Flow Diagram

```
grid_tick():
  warmup_scale = min(1.0, tick_count / 80)

  Arch fuel seeding (bottom row):
    for each x:
      sx = x − margin − wind_acc
      t = sx / span            [0..1 within flame zone]
      edge_dist = min(t, 1−t)
      arch = (edge_dist * 2)²  [squared = steep edges]
      jitter = rand in [0.82, 1.0]
      heat[bottom][x] = MAX_HEAT * fuel * arch * jitter * warmup_scale

  Propagation (all rows except bottom, top-to-bottom):
    for each (x, y) from y=0 to rows-2:
      rx = x + rand(-1, 0, +1)   [random horizontal spread]
      src = heat[y+1][rx]
      decay = DECAY_BASE + rand() * DECAY_RAND
      heat[y][x] = max(0, src − decay)

  wind_acc += wind (cyclic shift)
  cycle_tick++ → auto-cycle theme every CYCLE_TICKS

grid_draw():
  for each cell (x, y):
    v = heat[y][x]
    if v <= 0:
      if prev_heat[y][x] > 0:   ← was hot last frame
        mvaddch(y, x, ' ')      ← erase stale char
      continue                  ← skip cold cells otherwise
    v = pow(v / MAX_HEAT, 1/2.2)  ← gamma correction
    dither[y][x] = v

  Floyd-Steinberg pass (left-to-right, top-to-bottom):
    for each cell (x, y) with dither[y][x] >= 0:
      idx = lut_index(dither[y][x])  ← find ramp level
      qv  = lut_midpoint(idx)
      err = dither[y][x] − qv        ← quantization error
      distribute error to warm neighbours only:
        right:       += err * 7/16
        below-left:  += err * 3/16
        below:       += err * 5/16
        below-right: += err * 1/16
      attr = COLOR_PAIR(CP_BASE + idx) [+ A_BOLD for top 2 levels]
      mvaddch(y, x, k_ramp[idx])

  swap heat ↔ prev_heat
  copy new heat from prev_heat (so next tick writes into cleared buffer)
```

---

## Function Breakdown

### lut_index(v) → int
Purpose: map gamma-corrected float [0..1] to ramp level 0..8
Steps:
1. Walk k_lut_breaks from highest to lowest
2. Return first index where v >= break value
Ramp: `" .:+x*X#@"` — 9 levels, space = cold, @ = hottest

---

### lut_midpoint(idx) → float
Purpose: return quantized value for error diffusion
Steps:
1. If idx == 0: return 0.0
2. If idx == RAMP_N-1: return 1.0
3. Return (breaks[idx] + breaks[idx+1]) / 2

---

### theme_apply(t)
Purpose: register ncurses color pairs for theme t
Steps:
1. For i in 0..RAMP_N-1:
   - If COLORS >= 256: `init_pair(CP_BASE+i, theme.fg256[i], COLOR_BLACK)`
   - Else: `init_pair(CP_BASE+i, theme.fg8[i], COLOR_BLACK)`
Note: safe to call mid-loop — takes effect next frame

---

### ramp_attr(i, theme) → attr_t
Purpose: build ncurses attr for ramp level i
Steps:
1. Start with `COLOR_PAIR(CP_BASE + i)`
2. If 256-color and i >= RAMP_N-2: add A_BOLD
3. If 8-color: add theme.attr8[i] (A_DIM/A_NORMAL/A_BOLD from theme table)

---

### grid_tick(grid) → bool (theme changed?)
Purpose: one CA step — seed fuel, propagate heat, auto-cycle theme
Steps:
1. warmup_scale = clamp(warmup / 80, 0, 1); warmup++
2. wind_acc += wind (cyclic, reset if >= cols)
3. Arch fuel seeding (bottom row):
   - 10% margin each side
   - Normalize position along span to t ∈ [0,1]
   - arch = (min(t, 1-t) * 2)² — tall dome profile
   - jitter = rand [0.82, 1.0] for organic edge
   - heat[bottom][x] = MAX_HEAT × fuel × arch × jitter × warmup_scale
4. Propagation (rows 0 to rows-2, top to bottom):
   - rx = x + rand(-1..+1) — random lateral spread
   - src = heat[y+1][clamped rx]
   - decay = DECAY_BASE + rand × DECAY_RAND (0.04 to 0.10)
   - heat[y][x] = max(0, src - decay)
5. cycle_tick++; if >= CYCLE_TICKS: advance theme, reset warmup, return true

---

### grid_draw(grid, tcols, trows)
Purpose: render heat grid with dithering and diff-based clearing
Steps:
1. Gamma correction pass:
   - For each cell: if v <= 0, set dither=-1 (cold marker)
   - Else: dither[i] = pow(v / MAX_HEAT, 1/2.2)
2. Floyd-Steinberg + draw pass (left-to-right, top-to-bottom):
   a. If dither[i] < 0 (cold):
      - If prev_heat[i] > 0: mvaddch(y, x, ' ') — erase stale hot char
      - Else: skip (no write at all — no flicker, no solid box effect)
   b. If warm:
      - idx = lut_index(dither[i])
      - qv = lut_midpoint(idx)
      - err = dither[i] - qv
      - Add weighted error to right, below-left, below, below-right neighbours
        (only if they are also warm — cold cells don't receive error)
      - Draw k_ramp[idx] with ramp_attr(idx, theme)
3. Swap heat ↔ prev_heat pointers
4. Copy new current heat from prev_heat into heat for next tick

---

## Pseudocode — Core Loop

```
setup:
  theme = 0  (fire)
  screen_init(theme)    — initscr + color pairs
  scene_init(cols, rows, theme)
  grid_alloc()
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       scene_resize(new cols, rows)  ← preserves fuel/wind/theme, resets warmup
       erase()                       ← one explicit full clear
       needs_clear = false
       reset sim_accum, frame_time

  2. dt:
     now = clock_ns()
     dt = now − frame_time
     frame_time = now
     cap at 100ms

  3. physics ticks:
     tick_ns = 1_000_000_000 / sim_fps
     sim_accum += dt
     while sim_accum >= tick_ns:
       if not paused:
         theme_changed = grid_tick()
         if theme_changed: erase()  ← clear old theme chars
       sim_accum -= tick_ns

  4. FPS counter (every 500ms)

  5. frame cap: sleep to 60fps

  6. draw:
     if needs_clear: erase(); needs_clear = false
     scene_draw(cols, rows)  ← grid_draw()
     HUD: top-right mvprintw (fps, theme name, fuel, wind)
     wnoutrefresh + doupdate

  7. input:
     q/ESC   → quit
     space   → toggle paused
     ]       → sim_fps += 5
     [       → sim_fps -= 5
     t       → advance theme, reset warmup, needs_clear = true
     g       → fuel -= 0.1
     G       → fuel += 0.1
     w       → wind++ (right)
     W       → wind-- (left)
     0       → wind = 0
```

---

## Interactions Between Modules

```
App
 └── main loop: sim_accum → scene_tick → grid_tick → CA + fuel seeding
                render → grid_draw → Floyd-Steinberg → mvaddch
                input → fuel/wind/theme adjustments

Grid
 ├── heat[]     — current frame physics
 ├── prev_heat[] — previous drawn frame (for diff clearing)
 ├── dither[]   — working float buffer for Floyd-Steinberg
 └── table (aafire only) — precomputed decay LUT

§3 theme
 ├── k_ramp[], k_lut_breaks[] — shared between fire.c and aafire_port.c
 ├── k_themes[6] — FireTheme structs
 ├── theme_apply() — updates ncurses init_pair calls
 └── ramp_attr() — builds attr_t for each ramp level
```

---

## Floyd-Steinberg Dithering Explained

```
Goal: map continuous float [0,1] to discrete ramp indices 0..8
      while minimizing visible banding.

For each cell (left→right, top→bottom):
  1. Quantize: idx = lut_index(v)   (snap to nearest ramp level)
  2. Error: err = v − lut_midpoint(idx)  (how far off we are)
  3. Spread error to neighbors:
        [  this  | +7/16 ]
        [+3/16 | +5/16 | +1/16]

  → Neighbors receive fractional correction so the next cells
    are quantized more accurately.
  → Over a region, the average quantized value equals the original float.
  → Result: smooth gradients without needing more colors.

Key detail: only warm cells (dither >= 0) receive error.
Cold cells are excluded to prevent error propagating across the flame boundary.
```
