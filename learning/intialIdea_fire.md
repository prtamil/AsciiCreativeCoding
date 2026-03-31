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
