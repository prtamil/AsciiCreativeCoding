# Pass 1 — matrix_rain/matrix_snowflake.c: Matrix rain + live DLA snowflake

## Core Idea

Two independent real-time simulations run on the same terminal simultaneously. In the background, classic matrix digital rain falls — streams of random ASCII characters in theme-colored columns, flickering as they descend. In the foreground, a DLA (Diffusion-Limited Aggregation) ice crystal grows from the screen center using the same D6 hexagonal symmetry as snowflake.c. Frozen crystal cells are drawn on top of the rain. When the crystal fills the screen it flashes white and resets — new crystal, same rain — repeating indefinitely.

## The Mental Model

Think of two transparent overlays stacked on one screen. The bottom overlay is a video of rain (matrix-style characters falling). The top overlay is a growing ice crystal. The crystal is clear glass where it hasn't grown yet (rain shows through) and opaque white-to-blue ice where it has frozen. The rain always falls everywhere; it is only visually occluded by the crystal in frozen regions.

The DLA physics: random walkers (invisible) spawn in a ring just outside the current crystal boundary, take random steps, and when they touch the crystal they freeze to it. The D6 symmetry law means every stick event is applied to all 12 positions simultaneously — 6 rotations × 2 reflections — so all six arms grow together without any explicit arm-counting.

The key insight of this simulation: rain and crystal share the same terminal grid but are logically independent. The rain does not affect the crystal, and the crystal does not affect the rain. The only interaction is visual: the rain draw skips frozen cells, and the crystal draw overwrites whatever rain was there.

## Data Structures

### Crystal (§4)
```
uint8_t cells[ROWS_MAX][COLS_MAX]  — 0 = empty, n = color pair ID (CP_XTAL_1..6)
int     frozen_count               — total frozen cells across all arms
int     cols, rows, cx0, cy0       — grid size and center position
float   max_dist                   — normalisation radius (90% of half-diagonal)
float   trigger_dist               — reset threshold (88% of max_dist)
float   max_frozen_dist            — current farthest frozen cell from center
```
Color pair assigned at freeze time by Euclidean distance: outer tips get CP_XTAL_1 (bright), core gets CP_XTAL_6 (dim). Never changes once assigned.

### RainStream (§5)
```
float head    — head row as float for fractional speed
int   trail   — trail length in rows (6–21)
float speed   — rows per frame (0.35–1.25)
bool  active
```
One slot per column — g_streams[COLS_MAX]. Independent per-column.

### g_rain_ch[ROWS_MAX][COLS_MAX]
Global char buffer. Each cell holds its current character. Randomised near stream heads every tick (~67% chance per head cell) and globally flickered at ~4% of all cells per tick. Shared between all streams; a cell's char is updated by whichever stream is above it.

### Walker (§6)
```
int  col, row
bool active
```
Spawned in a ring around the crystal. Random-walk one cardinal step per frame. Tested for adjacency to frozen cells each step.

### AppState
```
typedef enum { STATE_GROW, STATE_FLASH } AppState;
```
STATE_GROW: normal DLA + rain. STATE_FLASH: crystal complete, 28-frame flash, then scene_reset().

## The Main Loop

1. Resize → scene_init() at new dimensions.
2. Input: q/r/p/+/-/]/[/t keys.
3. scene_tick():
   - If STATE_FLASH: rain_tick(), decrement flash_timer, reset when zero.
   - If STATE_GROW: rain_tick(), walkers_tick(), check trigger_dist.
4. erase().
5. rain_draw() — skip frozen cells (crystal draws on top).
6. xtal_draw() — two passes: glow halo, then frozen cells.
7. HUD: walker count, speed, theme, frozen count.
8. wnoutrefresh + doupdate.
9. Sleep to maintain 30fps.

## Non-Obvious Decisions

### Walker spawn radius — the critical optimisation
In snowflake.c, walkers spawn at screen edges and random-walk to the center. On a 220-column terminal the expected steps for a random walk from edge to center is O((cols/2)²) ≈ 10,000+ steps. With 60 walkers at 30fps the crystal would take minutes to start. The fix: spawn walkers at `max_frozen_dist + 8` (the current crystal boundary + 8 cells buffer). Walkers need only a few steps to find the crystal. Growth begins immediately and stays fast throughout.

The spawn code uses polar coordinates to place walkers uniformly on the circular spawn ring:
```c
float angle = rand_float * 2 * PI;
int nc = cx0 + roundf(cosf(angle) * spawn_r);
int nr = cy0 + roundf(sinf(angle) * spawn_r / ASPECT_R);
```
ASPECT_R=2.0 is divided in the row direction to compensate for terminal cells being 2× taller than wide, making the spawn ring look circular rather than elliptical.

### Rain char buffer vs on-the-fly generation
g_rain_ch[ROWS_MAX][COLS_MAX] stores one char per cell (80×300 = 24,000 bytes). Alternative: generate chars on the fly using a hash of (row, col, frame). The buffer wins because: (1) different regions can be updated at different rates — head cells flicker fast, bulk cells slow; (2) chars persist across frames so streams leave a visible "shimmer trail"; (3) no per-frame hash computation.

### Two draw layers, one z-order rule
draw order: rain_draw → xtal_draw. Because ncurses uses "last write wins", crystal cells are always on top. The rain draw skips frozen cells (`if (x->cells[r][c] != 0) continue`) so rain never flickers through frozen positions. The crystal glow halo (dim `:` on empty neighbors) is drawn in xtal_draw pass 1 — it appears on top of rain, which is acceptable and looks like the ice is glowing through the rain.

### Crystal color: black background not transparent
Crystal color pairs use `COLOR_BLACK` background while rain uses `-1` (transparent terminal background). This is intentional: frozen cells need an opaque background so rain chars beneath them are fully occluded. Without black background, the crystal chars would be drawn over the rain chars in the same cell, producing visual noise instead of clean crystal structure.

### Flash on completion
When `max_frozen_dist >= trigger_dist` (88% of max radius): switch to STATE_FLASH. All frozen cells rendered as bold white `*` for 28 frames (~0.9s). Then scene_reset(): clear crystal, re-seed, respawn walkers. Rain continues uninterrupted through the flash and reset — only the crystal resets, not the rain.

## D6 Symmetry — Same as snowflake.c

Identical math to snowflake.c: 12 = 6 rotations × 2 reflections. The 6 rotation angles are multiples of 60°; reflection is applied by negating dy before rotation. Euclidean space (with ASPECT_R correction) is used for the rotation math; converted back to terminal (col, row) after.

```c
for (int refl = 0; refl < 2; refl++) {
    float dx_e =  dx;
    float dy_e = (refl == 0 ? dy : -dy) / ASPECT_R;  /* Euclidean y */
    for (int k = 0; k < 6; k++) {
        float rx_e = dx_e * CA6[k] - dy_e * SA6[k];
        float ry_e = dx_e * SA6[k] + dy_e * CA6[k];
        int nc = cx0 + roundf(rx_e);
        int nr = cy0 + roundf(ry_e * ASPECT_R);
        xtal_freeze_one(x, nc, nr);
    }
}
```

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `STICK_PROB` | 0.35 | Lower = thicker arms, slower growth; higher = spiky, faster |
| `WALKER_DEFAULT` | 12 | Fewer = slow dramatic growth; more = rapid fill (use + key) |
| `FLASH_FRAMES` | 28 | Frames of white flash before reset (~0.9s at 30fps) |
| `RENDER_FPS` | 30 | Lower = more CPU time for DLA; higher = smoother rain but less DLA |
| `ASPECT_R` | 2.0 | Must match terminal font aspect ratio; wrong value = lopsided arms |

## Themes

5 themes, each pairing a rain hue family with a contrasting crystal palette:
- **Classic**: green rain + ice blue crystal (teal→white gradient)
- **Inferno**: red rain + gold/orange crystal
- **Nebula**: purple rain + cyan crystal
- **Toxic**: cyan rain + pink/magenta crystal
- **Gold**: yellow rain + violet/purple crystal

Contrast between rain and crystal hue is intentional: the crystal must be visually distinct from the rain color so the eye can separate the two layers.

## State Machine

```
                     r key or trigger_dist reached
STATE_GROW ──────────────────────────────────────► STATE_FLASH
    ▲                                                    │
    │              flash_tick == 0                       │
    └────────────────── scene_reset() ──────────────────-┘
         (crystal cleared, walkers respawned, rain continues)
```

## Open Questions for Pass 3

- What happens if two walkers try to freeze the same cell simultaneously in the same tick? (The freeze_one function silently skips already-frozen cells, so the second walker's symmetric freeze just skips some positions — no double-count, no race condition.)
- How does the color gradient behave when the crystal resets? (It recalculates from scratch — center = CP_XTAL_6, tips = CP_XTAL_1 — based on distance from the new center, same every cycle.)
- What would happen if walkers had a drift toward center instead of pure random walk? (Arms would grow outward faster and be spikier — same effect as raising STICK_PROB.)
- Could the rain and DLA run at different frame rates? (Yes: run DLA every N frames while rain runs every frame. Would decouple their visual rhythms.)

---

# Pass 2 — matrix_snowflake: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| [1] config | Constants: FPS, STICK_PROB, ASPECT_R, FLASH_FRAMES, rain chars, color pair IDs, Theme struct, k_themes[5] |
| [2] clock | clock_ns(), clock_sleep_ns() |
| [3] color | theme_apply(), color_init() |
| [4] crystal | Crystal struct, xtal_freeze_one/symmetric, xtal_char, xtal_init, xtal_draw (2-pass) |
| [5] rain | RainStream struct, g_rain_ch[][], rain_init, rain_tick, rain_draw |
| [6] walkers | Walker struct, walker_spawn (proximity ring), walkers_init, walkers_tick |
| [7] scene | AppState, g_xtal, scene_init/reset/tick/draw |
| [8] app | Signal handlers, main loop, key input |

## Data Flow Diagram

```
scene_tick():
  if STATE_FLASH:
    rain_tick()
    flash_tick--
    if 0: scene_reset()
  else:
    rain_tick():
      advance stream heads (+=speed*mult)
      randomise chars near heads (67% chance/cell)
      ambient flicker (4% of all cells)
      spawn new streams at random columns to hit g_n_streams target

    walkers_tick(crystal):
      for each walker:
        random cardinal step
        if out-of-bounds: walker_spawn(w, crystal)  ← on proximity ring
        if new cell frozen: try stick at current pos
        if new cell adj-frozen: try stick at new pos
        on stick: xtal_freeze_symmetric(12 positions), respawn walker

    if crystal.max_frozen_dist >= trigger_dist:
      state = STATE_FLASH, flash_tick = 28

scene_draw():
  rain_draw(crystal):
    for each active stream s:
      for trail depth d=0..trail:
        r = head - d
        if crystal.cells[r][col] != 0: skip  (crystal on top)
        draw g_rain_ch[r][col] with depth-based color

  xtal_draw(crystal, flashing):
    Pass 1: for each frozen cell:
      paint dim ':' on all empty 8-neighbors (glow halo)
    Pass 2: for each frozen cell:
      if flashing: bold white '*'
      else: xtal_char() with distance-based color pair + bold for tips/core

  HUD: walkers, speed, theme, frozen_count
```

## Pseudocode — Core Loop

```
srand(time); atexit(cleanup); signals(SIGINT/SIGTERM/SIGWINCH)
initscr / cbreak / noecho / keypad / nodelay / curs_set(0)
color_init()
getmaxyx → g_rows, g_cols
scene_init()   ← rain_init + xtal_init + walkers_init(crystal)

loop while !g_quit:
  if g_resize:
    endwin; refresh; getmaxyx
    scene_init()

  getch() → switch:
    q/ESC → quit
    r     → scene_reset()
    p/spc → g_paused toggle
    + =   → g_n_walkers += 5; walkers_init(crystal)
    -     → g_n_walkers -= 5
    ] [   → g_speed_mult ± 0.2
    t     → g_theme = (g_theme+1)%5; theme_apply()

  now = clock_ns()
  scene_tick()
  erase()
  scene_draw()
  wnoutrefresh; doupdate
  clock_sleep_ns(RENDER_NS - elapsed)
```
