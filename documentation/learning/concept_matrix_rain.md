# Pass 1 — matrix_rain.c: Classic vertical Matrix digital rain

## Core Idea

One independent stream per terminal column. Each stream owns a float head row that advances by `speed · dt` every frame, a fixed-length trail of cached glyphs that reroll at `SHIMMER_HZ`, and a constant per-stream speed in rows/sec so different streams visibly fall at different rates. When a stream's tail clears the bottom edge, the column becomes inactive and may be respawned above the screen with fresh randoms (random head_y, length, speed, glyphs).

The whole simulation is variable-dt: `head_y += speed * dt * speed_scale` per frame. There is no fixed-step accumulator and no alpha-interpolation scaffolding — the float head_y gives smooth sub-cell motion at any frame rate because the simulation is non-stiff (no springs, no fast oscillators).

## The Mental Model

Picture the screen as a vertical comb. Each tooth of the comb is a terminal column. Each column has at most one bright stream sliding down it like a Roman candle pointing earthward. The bright tip is the HEAD; behind it, a tail that fades from "hot white" through theme colour all the way down to a barely-visible tinted glyph. The streams slide independently — different lengths, different speeds, different start times — so the screen looks like dense rainfall rather than a synchronised fall.

The shimmer effect comes from rerolling the cached glyphs at `SHIMMER_HZ` (default 20). At 60 fps render, that's a glyph swap every 3 frames per cell — visible "blink" rather than 60 Hz blur.

## Data Structures

### Column (§4)
```
int    col;              — terminal column index, set once at spawn
float  head_y;           — head row, FLOAT for sub-row smoothness
float  speed;            — rows/sec; per-stream constant
int    trail_len;        — visible tail length (TRAIL_MIN..TRAIL_MAX)
char   glyphs[TRAIL_MAX];— per-stream cache, rerolled at SHIMMER_HZ
bool   active;           — true while at least part of the trail is on screen
```

### Rain (§5)
```
Column *columns;         — flat array indexed by terminal column x
int     ncols, nrows;
int     density;         — every density-th column starts active
float   speed_scale;     — global multiplier on stream speeds (], [ keys)
float   shimmer_accum;   — seconds since last cache reroll
bool    paused;
```

### Brightness bands (§4 col_band)
```
dist 0          → SHADE_HEAD    (white,    BOLD)
dist 1          → SHADE_HOT     (theme[4], BOLD)
dist 2          → SHADE_BRIGHT  (theme[3], BOLD)
dist 3..len/2   → SHADE_MID     (theme[2], NORMAL)
dist len/2+1..  → SHADE_DARK    (theme[1], NORMAL)
dist len-1      → SHADE_FADE    (theme[0], DIM)
```

## The Main Loop

Variable-dt. Each iteration:

1. **Resize check**. SIGWINCH triggers `rain_init` with the new (cols, rows). All in-flight streams reset.
2. **Measure dt**. Wall-clock seconds since the previous frame, capped at `DT_CAP_SEC = 0.10` to prevent spiral-of-death.
3. **Drain input**. Arrows / `[` / `]` / `+` / `-` / `t` / `r` / `space` / `q`.
4. **`rain_tick(dt)`**. For each column:
   - Active: `head_y += speed * dt * speed_scale`. If pile_top above bottom edge, deactivate.
   - Inactive: roll a die scaled by `RESPAWN_RATE_PER_SEC * dt / density`; respawn if hit.
   - Shimmer: every cell with prob 1/SHIMMER_HZ per frame swaps its glyph.
5. **fps rolling average**.
6. **Draw**: `erase()`, `rain_draw()` (per-active-column trail render), HUD on row 0 (yellow, BOLD), hint strip on bottom row (cyan, BOLD), `wnoutrefresh + doupdate`.
7. **Frame cap**. Sleep until 1/60 s elapsed since frame start.

## Non-Obvious Decisions

**Why drop the alpha accumulator?**
The simulation is non-stiff. The float `head_y` advancing by `speed * dt` per frame is unconditionally stable and gives smooth sub-cell motion automatically. The fixed-step + alpha pattern is a 30-line apparatus to solve a problem that doesn't exist here. Removing it makes the file ~30 lines shorter and removes the SIM_FPS / RENDER_FPS distinction entirely.

**Why per-stream cache and not a 2-D `g_rain_ch[][]` grid?**
The 2-D grid pattern was used in earlier sister files (matrix_snowflake.c had it). It enables ambient flicker across the whole screen, but it adds a parallel data path and makes the algorithm harder to follow. With per-stream cache, each stream owns its glyphs and there's only one data structure to reason about. The visual is "discrete falling streams on black" instead of "field of flickering chars" — both are valid Matrix-rain aesthetics.

**Why round-half-up instead of `roundf`?**
`roundf(0.5)` returns 0 but `roundf(1.5)` returns 2 (banker's rounding). At `head_y = N + 0.5` the head would oscillate between rows N and N+1 across frames — visible flicker. `floor(x + 0.5)` always rounds up at .5 — deterministic, no flicker.

**Why `SHIMMER_HZ` (not per-frame reroll)?**
Per-frame reroll at 60 fps would make every glyph blur into noise. The 20 Hz shimmer rate decouples the visual character from the frame rate — you see discrete glyph swaps at 20 Hz on top of smooth 60 Hz motion. Tweak `SHIMMER_HZ` to taste.

**Why is `head_y` initial value negative?**
A fresh stream spawns at `head_y = uniform(-rows/2, 0)` so it enters the visible area gradually — no synchronised "they all started at row 0 at t=0" effect.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|---|---|---|
| TARGET_FPS | 60 | Render cap. Lower = chunkier animation. |
| TRAIL_MIN, TRAIL_MAX | 6, 24 | Stream length range. Longer = more theme-colour visible. |
| SPEED_MIN_RPS, SPEED_MAX_RPS | 8, 24 | Rows/sec range per stream. Wider = more speed variation. |
| DENSITY_DEFAULT | 2 | Every density-th column starts active. Smaller = denser. |
| SHIMMER_HZ | 20 | Glyph reroll frequency. 60+ blurs into noise; 5– freezes. |
| RESPAWN_RATE_PER_SEC | 0.6 | Probability an inactive column wakes up per second. |
| SPAWN_OFFSCREEN_FRAC | 0.5 | Fraction of screen height a stream may pre-emerge above. |

## Open Questions for Pass 3
- Is there any visual win from re-introducing a separate ambient-flicker layer (random cells across the screen flickering), or does the per-stream cache suffice?
- The per-stream `shimmer_accum` is shared across all streams (one global). Per-stream timers would let different streams shimmer at different rates — would that look more organic or just chaotic?
- At `density = 1` and small terminals, every column is active and the shimmer dominates. Is there a "max active streams" cap that would keep dense terminals readable?

---

# Pass 2 — matrix_rain: Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config | TARGET_FPS, TRAIL/SPEED/DENSITY ranges, SHIMMER_HZ, color pair IDs |
| §2 clock | `clock_ns`, `clock_sleep_ns` |
| §3 color | 4 themes, `theme_apply`, `hud_pairs_init` |
| §4 column | `Column`, `col_spawn`, `col_advance`, `col_shimmer`, `col_band`, `col_draw` |
| §5 rain | `Rain`, `rain_init`, `rain_tick`, `rain_draw` |
| §6 screen | `screen_init`, `screen_resize`, `screen_present`, `screen_draw_hud` |
| §7 app | signals, `main` (variable-dt loop) |

## Data Flow

```
arrow keys / +/- / t / r / space ──► app_handle_key ──► Rain.{speed_scale, density, paused}, theme_apply
                                                         │
clock_ns ──► dt ─────────► rain_tick(dt)
                                  │
                                  ├─► col_advance(c, dt, scale)  → head_y += speed * dt
                                  ├─► col_shimmer(c)             → reroll cache at SHIMMER_HZ
                                  └─► (if inactive) respawn check
                                  │
                                  ▼
                              rain_draw ──► col_draw(c) per active column
                                                 │
                                                 ▼
                                        ncurses mvaddch grid + HUD + hint strip
                                                 │
                                                 ▼
                                          wnoutrefresh + doupdate
```

## Function Breakdown

### `col_spawn(c, x, rows)`
1. `col = x`
2. `head_y = uniform(-SPAWN_OFFSCREEN_FRAC · rows, 0)`
3. `trail_len = uniform(TRAIL_MIN, TRAIL_MAX)`
4. `speed = uniform(SPEED_MIN_RPS, SPEED_MAX_RPS)`
5. fill `glyphs[]` with random ASCII

### `col_advance(c, dt, scale, rows)`
1. `head_y += speed · dt · scale`
2. return `(head_y - trail_len) < rows` (false → off-screen, deactivate)

### `col_band(dist, trail_len)`
- Returns the ncurses pair-and-attribute combo for the trail slot.

### `col_draw(c, rows)`
For `dist = 0..trail_len-1`:
- `row = floor(head_y - dist + 0.5)`
- skip if out of bounds
- paint `glyphs[dist]` in `col_band(dist, trail_len)`

### `rain_tick(rain, dt)`
- shimmer_accum += dt; if exceeds 1/SHIMMER_HZ, reroll all active caches and reset
- per column: advance active, respawn dead columns with probability scaled by dt/density

### `rain_draw(rain)`
- per active column: `col_draw`

## Pseudocode — Main Loop

```
setup:
  install_signals
  screen_init, color_init, hud_pairs_init
  rain_init(rows, cols, density)

while running:
  if need_resize:
      rain_init(new rows, cols, density)
  dt = clock - last
  if dt > DT_CAP_SEC: dt = DT_CAP_SEC
  drain getch into app_handle_key
  rain_tick(rain, dt)
  fps update
  erase
  rain_draw(rain)
  screen_draw_hud
  wnoutrefresh + doupdate
  sleep to TARGET_FPS

cleanup:
  endwin
```

## Key Patterns to Internalize

**Variable dt is fine for non-stiff sims.** No accumulator needed.

**Float positions = sub-cell smoothness.** `head_y` as `float` rounded with `floor(x + 0.5)` gives smooth scrolling for free.

**Decouple shimmer from frame rate.** SHIMMER_HZ is a frequency in real time, not a per-frame probability. Same visual character at 30 fps and 144 fps.

**Per-stream cache > shared 2D grid.** Each stream owning its glyphs is a single data path; the grid pattern adds parallel state and complicates the mental model.

**Brightness bands are universal.** The HEAD/HOT/BRIGHT/MID/DARK/FADE ramp is identical across `matrix_rain.c`, `pulsar_rain.c`, `sun_rain.c`, `fireworks_rain.c`. Learn it once.
