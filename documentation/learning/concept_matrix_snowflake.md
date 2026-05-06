# Pass 1 — matrix_snowflake.c: Matrix rain falls, glyphs freeze, snow piles up

## Core Idea

Two simulations sharing one screen, with a moving boundary between them. In each terminal column, a stream of matrix-rain glyphs falls from the top. When the stream's head reaches the top of the snow pile in that column, the head glyph freezes there — its colour switches from rain hue to snow hue, the pile's top in that column rises by one row, and the stream respawns above the screen after a short random delay. When every column is full (pile reaches the top across the whole screen) the entire pile flashes bright for ~1 s and the world resets.

The previous version of this file ran a DLA aggregate with D6 hexagonal symmetry — clever fractal math but a weird match for "matrix rain hits something and snow grows." The new design is straight gravity + per-column accumulation, which is both intuitively obvious from a 30-second watch and dramatically simpler to read in code.

## The Mental Model

A row of N hourglasses, side by side. In each hourglass the upper chamber is the rain region (where a glyph stream is currently falling) and the lower chamber is the snow pile. The "neck" between chambers — the boundary where a falling glyph turns into a frozen one — is the row index `pile_top[c] − 1`. As snow piles up, that boundary moves UP the screen (smaller row indices). When the boundary in every column reaches the top edge, the whole setup glares bright and refills empty for another round.

```
   t = 0 (empty pile)        t = mid                     t = full
   ┌──┐                      ┌──┐                        ┌──┐
   │  │                      │  │                        │##│
   │X │ ← head               │  │                        │##│
   │k │   trail              │X │                        │##│
   │7 │                      │k │ ← head                 │##│
   │  │                      │##│ ← pile_top moved up    │##│ ← pile_top = 0
   │##│ ← pile_top           │##│                        │##│   COLUMN FULL
   ├──┤ ← HUD                ├──┤                        ├──┤

   pile_top[c]:  rows-1      pile_top[c]: ~rows/2        pile_top[c]: 0
```

Different columns fill at different rates because per-stream speeds and respawn delays are randomised. The pile profile (pile_top vs col) is therefore never flat — always wavy.

## Data Structures

### RainStream (§4)
```
int    col;            — column index, set once at spawn
float  head;           — head row (float for sub-row smoothness)
int    trail_len;      — visible tail length (4..14)
float  speed;          — rows/sec; per-stream constant
float  restart_delay;  — seconds remaining until respawn (only valid when !active)
char   glyphs[14];     — per-stream glyph cache, top 3 reroll every frame
bool   active;
```

### Snow (§5)
```
char  pile_chars[rows][cols];  — glyph at each frozen cell, 0 if empty
int   pile_top[cols];          — row index of TOPMOST FROZEN row in column c.
                                 Rows pile_top[c]..rows-2 are frozen.
                                 Rows 0..pile_top[c]-1 are empty.
                                 DECREASES (moves UP) as snow grows.
                                 Initial: rows-1 (no usable frozen row yet).
                                 Reaches 0 when column is full.
int   cols, rows;
int   frozen_count;             — total frozen cells; HUD readout
```

### Scene (§6)
```
RainStream  streams[cols];
Snow        snow;
int         cols, rows;
SceneState  state;     — STATE_FALL or STATE_FLASH
int         flash_tick;— frames remaining in FLASH state
int         theme_idx;
bool        paused;
float       rain_speed_scale;
```

## Brightness bands

Rain (3 bands):
```
dist 0      → CP_RAIN_HEAD  (BOLD)
dist 1..3   → CP_RAIN_MID
dist 4+     → CP_RAIN_FADE  (DIM)
```

Snow (2 bands):
```
depth = r - pile_top[c]
depth < SNOW_FRESH_DEPTH    → CP_SNOW_FRESH   (BOLD)  — recently fallen
depth ≥ SNOW_FRESH_DEPTH    → CP_SNOW_PACKED  (NORMAL) — older
```

## The Main Loop (variable dt)

1. Resize check — SIGWINCH triggers `scene_init` with the new (cols, rows).
2. `dt` = wall-clock seconds since last frame, capped at `DT_CAP_SEC`.
3. Drain input.
4. `scene_tick(dt)`:
   - **STATE_FLASH**: `flash_tick--`; if zero, `scene_reset` (clears pile, respawns streams).
   - **STATE_FALL**: per column:
     - Inactive stream: `restart_delay -= dt`; respawn when ≤ 0 AND column not yet full.
     - Active stream's column already full: deactivate forever (until next reset).
     - Active stream: `head += speed · dt · rain_speed_scale`; reroll top 3 glyphs at 67% chance each. If `floor(head + 0.5) ≥ pile_top[c] - 1`: `snow_freeze(c, glyphs[0])`, deactivate, schedule respawn.
   - If `snow_is_full()` (every `pile_top[c] == 0`), enter STATE_FLASH.
5. fps rolling average.
6. Draw: `erase`, `snow_draw` (pile, fresh-on-top), `rain_draw` (streams above pile_top), HUD, hint, `wnoutrefresh + doupdate`.
7. Frame cap.

## State Machine

```
      ┌───────┐  every column full   ┌───────┐
      │ FALL  │ ──────────────────►  │ FLASH │
      │       │   (snow_is_full)      │       │
      │ rain  │                       │ pile  │
      │ feeds │                       │ glares│
      │ pile  │                       │ white │
      │       │  ← FLASH_FRAMES = 0 ──│       │
      └───────┘  (scene_reset clears  └───────┘
                  pile, respawns streams)
```

## Non-Obvious Decisions

**Why drop the DLA + D6 symmetry version?**
DLA + D6 produces a fractal snowflake — clever, but the user has to know what DLA is and what dihedral groups are to read the code. The new "rain → snow pile" version requires only "things fall and stack up", which anyone watching the screen recognises in seconds.

**Why is `pile_top[c]` initialised to `rows-1` (not `rows-2`)?**
`rows-1` is the HUD hint strip — never frozen. `pile_top = rows-1` means "no actual snow yet, but the floor is conceptually at rows-1." The first freeze in column c places a glyph at `pile_top[c] - 1 = rows - 2` (the lowest usable row) and decrements pile_top to rows-2. Off-by-one trap if you initialise to rows-2.

**Why does each stream own its own glyph cache?**
A 2-D `g_rain_ch[][]` grid was used in the old DLA version for ambient flicker. It adds a parallel data path and complicates the mental model. Per-stream cache is one structure per stream, no shared state — simpler to teach.

**Why does the trail vanish when the head freezes?**
When a stream freezes its head, the rest of the trail (which extends ABOVE the head row) is just thrown away — `active = false` and the stream restarts later from above the screen. This is the cleanest design: the trail is purely visual, not stored as separate cells. Animating the trail dissipating would mean carrying state through the freeze event, which we don't need for the visual.

**Why no horizontal sand-pile dynamics?**
Real falling-sand simulations cascade laterally: when a pile is much taller in column c than c+1, glyphs slide diagonally down. The matrix_snowflake design deliberately does NOT do that — each column accumulates independently. The result is a wavy but vertically-stratified pile, which reads as "snow accumulating" rather than "sand piling." Adding lateral cascades would be a separate file (sandpile.c).

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| RAIN_TRAIL_MIN, MAX | 4, 14 | Stream length range. |
| RAIN_SPEED_MIN, MAX | 6, 20 (cells/sec) | Per-stream speed range. |
| RAIN_HEAD_FLICKER | 3 | Top 3 cells reroll every frame. |
| RAIN_HEAD_REROLL_PROB | 0.67 | Per-cell reroll chance per frame. |
| STREAM_RESTART_MIN, VAR | 0.30, 1.20 sec | Respawn delay range. |
| SNOW_FRESH_DEPTH | 3 | Top 3 rows of pile render BOLD. |
| FLASH_FRAMES | 28 | At 30 fps ≈ 0.93 s flash duration. |

## Open Questions
- Should the pile "melt" between flashes? Currently it clears instantly. A melt animation could reuse the same per-column state.
- Wind drift on freeze (small lateral chance) would produce uneven pile profiles. Worth adding for visual interest, or does it muddy the simple "vertical stack" model?
- Could SNOW_FRESH be a gradient (3-4 rows fade from BOLD to NORMAL) rather than a sharp boundary?

---

# Pass 2 — Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config | grid bounds, frame rate, stream geometry/speed, snow params, color pairs |
| §2 clock | monotonic timer + sleep |
| §3 color | 5 themes (rain hue paired with snow hue), HUD pairs |
| §4 stream | RainStream type, spawn, advance, draw helpers |
| §5 snow | Snow type, pile state, freeze, full check, draw |
| §6 scene | Scene state + FALL/FLASH state machine |
| §7 screen | ncurses init / present / HUD |
| §8 app | signals, resize, variable-dt main loop |

## Data Flow

```
keys ──► app_handle_key ──► scene.{paused, theme_idx, rain_speed_scale}
                                      │
clock_ns ──► dt ──► scene_tick(dt)
                          │
                          ├── STATE_FALL: per-column
                          │     ├── if !active: restart_delay -= dt; respawn if ready
                          │     ├── stream_advance(s, dt, scale, land_row)
                          │     │     └── return true → snow_freeze(c, glyphs[0]); deactivate
                          │     └── if all pile_top == 0 → STATE_FLASH
                          │
                          └── STATE_FLASH: flash_tick--; if 0 → scene_reset
                          │
                          ▼
                      scene_draw
                          ├── snow_draw   (rows pile_top..rows-2 per column)
                          └── rain_draw   (per active stream, rows < pile_top[c])
                                  │
                                  ▼
                         ncurses paint + HUD + hint
```

## Pseudocode

```
setup:
  install signals
  screen_init, color (theme_apply + hud_pairs_init)
  scene_init(cols, rows)

while running:
  if need_resize: scene_init(new cols, rows) preserving theme & speed
  dt = clamp(now - last, 0, DT_CAP_SEC)
  drain input → app_handle_key
  scene_tick(scene, dt)
  fps update
  erase
  scene_draw(scene)              # snow first, rain on top above pile_top
  screen_draw_hud(screen, fps, scene)
  wnoutrefresh + doupdate
  sleep to TARGET_FPS

cleanup: endwin
```

## Key Patterns to Internalize

**Per-column accumulators.** Each column is its own little hourglass. No global "snow level" — only per-column `pile_top[c]`.

**The boundary is just a number.** `pile_top[c]` defines what's snow vs what's air. The physics changes nothing else.

**Variable dt + float head.** Same as matrix_rain.c. Smooth motion at any frame rate, no accumulator.

**State machines belong in the orchestrator.** `scene_tick` dispatches by `state`; the per-column logic doesn't know about FLASH at all.

**Two simulations on one screen, one boundary.** This is the meta-pattern: when you want two things sharing a grid, define a clean boundary (pile_top, freeze line, etc.) and let each simulation operate above/below it independently.
