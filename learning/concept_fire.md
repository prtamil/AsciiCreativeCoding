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
