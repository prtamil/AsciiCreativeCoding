# Pass 1 — matrix_rain.c: Interpolated ncurses Matrix digital rain

## Core Idea

A collection of vertical columns of characters fall down the terminal screen. Each column is an independent stream with its own speed, length, and character set. The head of the stream glows bright white; the tail behind it fades through progressively darker shades of the theme color until it dissolves into black. The defining technical feature is render interpolation: the simulation physics run at a lower rate (default 20 ticks/sec) while rendering runs at 60 fps, and the sub-tick fractional position of each column head is projected forward so motion appears continuously smooth rather than jumpy.

## The Mental Model

Imagine a rain of neon letters falling down a black screen. Each "drop" is a glowing streak — bright at the tip, fading behind it. The streaks appear at random columns, fall at different speeds, and vanish when they leave the bottom of the screen. New ones spawn randomly.

The trick that makes it look good: the computer's physics engine runs slowly (20 times per second) but the screen redraws 60 times per second. Without any fix, you'd see the drops frozen for 2 out of every 3 frames and then jump — visible judder. The fix is to look at how far into the next physics step we are (a fraction between 0 and 1, called alpha) and draw each drop slightly ahead of where physics last placed it. At 33% through the next tick, draw the head 0.33 rows lower. At 66%, draw it 0.66 rows lower. This makes the motion look completely fluid even though the physics only computed two actual positions.

Additionally, each column "writes" into a persistent background grid as it falls. After a column has passed, the grid retains the faded characters. A random scatter-erase function dissolves these lingering characters away over time, creating the "characters dissolving into black" effect from the original film.

## Data Structures

### Cell (§4)
```
char  ch;    — the ASCII character to display at this position
Shade shade; — which brightness level (FADE, DARK, MID, BRIGHT, HOT, HEAD)
```
A single grid position. `ch == 0` means empty. This is the unit stored in the dissolve grid.

### Grid (§5)
```
Cell *cells; — flat array, indexed as [y * cols + x]
int   cols, rows;
```
The off-screen framebuffer that holds the persistent dissolve texture. It is cleared each tick and repainted by active columns, then scatter-erased. It is NOT what gets drawn for live column positions — that happens via interpolation directly to stdscr.

### Column (§6)
```
int  col;              — which terminal column (x position, fixed for this stream's life)
int  head_y;           — integer row of the stream's leading character (physics position)
int  length;           — number of characters in the trail (6 to 24)
int  speed;            — rows advanced per sim tick (1 or 2)
bool active;           — whether this stream is currently falling
char ch_cache[TRAIL_MAX]; — one randomly chosen character per trail slot
```
The core moving entity. `head_y` is the integer physics position. The actual drawn position adds `speed * alpha` to get a fractional row. `ch_cache` is refreshed every tick so characters shimmer as the stream falls. The cache exists so the interpolated draw can access the current character for any trail position without consulting the grid.

### Rain (§7)
```
Column *columns; — one slot per terminal column (x = 0..ncols-1)
int     ncols, nrows;
int     density_divisor; — only columns at x % divisor == 0 spawn streams
Grid    grid;    — the dissolve texture
```
The simulation owner. Column slot at index `x` always corresponds to terminal column `x`. Inactive slots wait to randomly respawn.

### Theme (§3)
```
const char *name;
int fg[5]; — xterm-256 color indices for the 5 brightness shades (fade..hot)
```
Shade HEAD is always white (bright tip) regardless of theme. The five theme foregrounds map to Shade values FADE through HOT.

### App (§9)
The top-level struct owning Rain, Screen, sim_fps, density, theme_idx, and the two signal flags (`running`, `need_resize`). It is a global so signal handlers can reach it.

## The Main Loop

Each iteration of the while loop in `main()`:

1. **Resize check**: If SIGWINCH fired, tear down the rain, ask ncurses for new dimensions, rebuild the rain at the new size, reset the timing accumulators.

2. **Measure delta-time (dt)**: Get the current nanosecond timestamp. Subtract the last frame's timestamp. Clamp at 100ms to prevent a huge catch-up burst after the program was suspended or stalled.

3. **Drain the sim accumulator**: Add dt to `sim_accum`. While `sim_accum` is larger than one tick duration, call `rain_tick()` once and subtract one tick duration. This means the simulation always runs at exactly sim_fps logical steps per real second regardless of how fast the render loop runs.

4. **Compute alpha**: Divide the remaining `sim_accum` (the leftover nanoseconds) by the tick duration. This is a float in [0, 1) representing how far through the next unfired tick we are.

5. **Update FPS display**: Count frames and elapsed time; recalculate displayed FPS every 500ms.

6. **Frame rate cap sleep**: Calculate how much time the sim+overhead took. Sleep enough to keep renders at approximately 60fps. The sleep happens BEFORE rendering, not after, to avoid I/O drift.

7. **Draw**: Call `screen_draw_rain()` which calls `rain_draw()` with the current alpha. This does two passes: first draw the grid (dissolve texture at integer positions), then draw each active column at its interpolated float position.

8. **Draw HUD**: Write FPS, speed, density, and theme name to the top-right corner of stdscr.

9. **Present**: `wnoutrefresh()` + `doupdate()` atomically push all changes to the terminal in one diff.

10. **Input**: `getch()` in non-blocking mode. Handle key presses for speed/density/theme/quit.

## Non-Obvious Decisions

### Why forward extrapolation instead of lerp?
For lerp (interpolating between previous and current position), you'd need to store `prev_head_y` in each Column. The code avoids this by observing that columns move at constant speed with no acceleration between ticks. The position at time `alpha` ticks into the future is exactly `head_y + speed * alpha`, which is identical to lerping between `head_y - speed` (previous position) and `head_y` (current position). No extra storage needed. If you ever add variable speed or acceleration, you'd need to switch to true lerp with stored previous positions.

### Why `floorf(x + 0.5f)` instead of `roundf(x)`?
`roundf()` uses "round half to even" (banker's rounding) as its tie-breaking rule. When the fractional part is exactly 0.5, it rounds to the nearest even integer. This means a character sitting right at a half-cell boundary would alternate between two rows on consecutive frames — a one-pixel flicker. `floorf(x + 0.5f)` always breaks ties by rounding up, which is deterministic and flicker-free.

### Two-pass drawing in rain_draw()
Pass 1 draws the integer-positioned grid (dissolve texture). Pass 2 draws live column heads at fractional positions directly to stdscr, bypassing the grid. Why not just draw everything from the grid? The grid only stores integer row positions — it can't hold the sub-row fractional data needed for smooth interpolation. The grid is still maintained because it serves a different purpose: it holds the fading remnants of columns that have already passed that position, creating the character-dissolve effect.

### grid_scatter_erase()
Each tick, one random row has `cols / DISSOLVE_FRAC` cells randomly zeroed. This is not a systematic fade — it is chaotic, random deletion. This creates the organic, non-uniform dissolve where some characters linger and others vanish quickly, matching the film aesthetic much better than a clean uniform fade would.

### density_divisor controls column spacing
With `density_divisor = 2`, only even-numbered x columns spawn streams. With divisor = 1, every column can have a stream. The `chance = 15 / density_divisor` formula in the spawn check means denser settings also increase the spawn rate, so the screen fills faster.

### Column characters shimmer every tick
`col_tick()` refreshes the entire `ch_cache` on every tick, not just the head position. This means every character in every active trail changes randomly each tick. Visually this creates the "digital rain" shimmer effect where trailing characters are not static.

### ch_cache[] size is TRAIL_MAX (24), but length is variable
The cache is always allocated for the maximum trail length. Only the first `c->length` entries are used. This avoids a variable-length allocation while keeping the struct size bounded.

## State Machines

Each Column has an implicit binary state:

```
         rand() % 100 < spawn_chance
INACTIVE ─────────────────────────────► ACTIVE
    ▲                                      │
    │   head_y - length >= rows            │
    └──────────────────────────────────────┘
       (tail has left the screen)
```

There is no explicit enum — `active` bool + the tick logic handles it. An inactive column with no rain just does a random spawn check each tick. Active columns advance until their tail exits the bottom.

The Rain simulation itself has no state machine — it is a pure tick function.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect if changed |
|---|---|---|
| `SIM_FPS_DEFAULT` | 20 | Lower = slower rain but more judder without alpha; higher = smoother but less Matrix-like feel |
| `TRAIL_MIN` / `TRAIL_MAX` | 6 / 24 | Shorter trails = sparse dots; longer trails = long continuous streams |
| `SPEED_MIN` / `SPEED_MAX` | 1 / 2 | Higher SPEED_MAX allows 3+ row jumps per tick; would need higher sim_fps to look smooth |
| `DENSITY_DEFAULT` | 2 | Lower = more columns active; 1 = maximum density (every column) |
| `DISSOLVE_FRAC` | 4 | Lower = faster dissolve (more cells erased per tick); higher = ghosting lingers longer |
| `CYCLE_TICKS` | (not present — per rain_tick) | N/A — no auto-cycle here; theme is user-controlled only |

## Open Questions for Pass 3

- How exactly does ncurses' internal double buffer (curscr / newscr) work, and why does `doupdate()` after `wnoutrefresh()` produce less flicker than a direct `refresh()`?
- If you set SPEED_MAX to 3 or 4, does the interpolation still look smooth, or does `floorf(draw_head_y + 0.5f)` create visible quantization?
- The grid is cleared (`grid_clear`) and re-painted every tick. Why not just diff it? What would break if you removed the clear?
- `grid_scatter_erase` picks a random row but then picks random columns within that row. Would picking fully random (x, y) pairs per erase call produce a visually different dissolve?
- What happens to the `ch_cache` for the interpolated draw when `alpha = 0.99` and the head is near the edge? Does the character assignment for "out-of-screen" trail slots matter?

---

# Pass 2 — matrix_rain.c: Pseudocode

## Module Map

| Section | Name | Purpose |
|---|---|---|
| §1 | config | All tunable enums and macros in one place |
| §2 | clock | Monotonic nanosecond timer + sleep |
| §3 | theme | 4 named color themes; maps Shade levels to ncurses color pairs |
| §4 | cell | One virtual pixel: a character + a brightness level |
| §5 | grid | 2-D flat array of cells; the persistent dissolve framebuffer |
| §6 | column | One falling stream: position, trail, cache, paint to grid or stdscr |
| §7 | rain | Owns all columns + the grid; tick = advance physics; draw = interpolated render |
| §8 | screen | Wraps ncurses init/teardown/resize; calls rain_draw |
| §9 | app | Main dt loop, signal handling, key input, resize dispatch |

## Data Flow Diagram

```
clock_ns() → dt measurement
    │
    ▼
sim_accum += dt
    │
    ├─ [while sim_accum >= tick_ns]──► rain_tick()
    │                                        │
    │                                        ├─► grid_clear(grid)
    │                                        ├─► grid_scatter_erase(grid)  ← random dissolve
    │                                        └─► for each column:
    │                                               col_tick()  → head_y += speed
    │                                                           → ch_cache[] refreshed
    │                                               col_paint() → writes to grid
    │
    └─ sim_accum remainder → alpha = sim_accum / tick_ns
                                  │
                                  ▼
                           rain_draw(alpha)
                                  │
                           Pass 1: grid.cells[]
                                  │  char + shade → shade_attr() → attron + mvaddch
                                  ▼
                           Pass 2: for each active column:
                                  draw_head_y = head_y + speed * alpha
                                  col_paint_interpolated(draw_head_y)
                                       │
                                       for dist 0..length:
                                         row = floorf(draw_head_y - dist + 0.5f)
                                         ch_cache[dist] + shade_attr → mvaddch(row, col)
                                  │
                                  ▼
                           screen_draw_hud() → mvprintw() top-right
                                  │
                                  ▼
                           wnoutrefresh() + doupdate() → terminal
```

## Function Breakdown

### clock_ns() → int64_t nanoseconds
Purpose: Read CLOCK_MONOTONIC, return as a single int64 nanosecond count.
Steps:
  1. Call clock_gettime(CLOCK_MONOTONIC, &t)
  2. Return t.tv_sec * 1_000_000_000 + t.tv_nsec
Edge cases: Cannot go backward (monotonic guarantee). Safe to call from main loop.

### clock_sleep_ns(int64_t ns)
Purpose: Sleep exactly ns nanoseconds via nanosleep.
Steps:
  1. If ns <= 0, return immediately.
  2. Split ns into seconds and remainder nanoseconds.
  3. Call nanosleep().
Edge cases: Does not retry on EINTR. Acceptable for a soft frame cap.

### theme_apply(int theme_idx)
Purpose: Register ncurses color pairs for all 6 shade levels of the chosen theme.
Steps:
  1. If COLORS >= 256, pick fg from k_themes[theme_idx].fg.
  2. Else pick fg from k_themes_8color[theme_idx].
  3. For Shade FADE..HOT: init_pair(shade_id, fg[shade-1], COLOR_BLACK).
  4. init_pair(SHADE_HEAD, COLOR_WHITE, COLOR_BLACK) — always white, always.
Edge cases: Safe to call at runtime; takes effect on next rendered frame.

### shade_attr(Shade s) → attr_t
Purpose: Map a shade level to the ncurses attribute bitmask.
Steps:
  1. Switch on shade:
     - FADE: COLOR_PAIR + A_DIM
     - DARK: COLOR_PAIR only
     - MID: COLOR_PAIR only
     - BRIGHT: COLOR_PAIR + A_BOLD
     - HOT: COLOR_PAIR + A_BOLD
     - HEAD: COLOR_PAIR(HEAD) + A_BOLD
Edge cases: Default returns A_NORMAL for safety.

### grid_alloc(Grid *g, int cols, int rows)
Purpose: Allocate the cell array for the given dimensions.
Steps:
  1. Store cols, rows.
  2. calloc(cols * rows, sizeof(Cell)) — zero-initialized = all cells empty.

### grid_scatter_erase(Grid *g)
Purpose: Randomly dissolve a fraction of one row per call.
Steps:
  1. Pick a random row y.
  2. Compute count = cols / DISSOLVE_FRAC.
  3. For count iterations: pick random x in [0, cols), zero the cell at (x, y).
Edge cases: Multiple picks may hit the same cell. That is acceptable — dissolve is probabilistic.

### col_spawn(Column *c, int x, int rows)
Purpose: Initialise a column for a new falling stream.
Steps:
  1. Set col = x.
  2. Set head_y = -(random value in [0, rows/2)) — starts above the visible screen.
  3. Set length = random in [TRAIL_MIN, TRAIL_MAX].
  4. Set speed = random in [SPEED_MIN, SPEED_MAX].
  5. Set active = true.
  6. Fill ch_cache[0..length] with random ASCII characters.
Edge cases: Head starts negative so the stream "falls in" from above rather than popping on mid-screen.

### col_tick(Column *c, int rows) → bool
Purpose: Advance column physics by one sim tick.
Steps:
  1. head_y += speed.
  2. Refresh all ch_cache[0..length] with new random characters.
  3. Return true if (head_y - length) < rows — the tail is still on screen.
Edge cases: Returns false when tail exits bottom. Caller sets active = false.

### col_shade_at(int dist, int length) → Shade
Purpose: Map trail distance from head to a shade level.
Steps:
  1. dist == 0 → HEAD (bright white)
  2. dist == 1 → HOT
  3. dist == 2 → BRIGHT
  4. dist <= length/2 → MID
  5. dist <= length-2 → DARK
  6. else → FADE (dimmest, tail end)

### col_paint(Column *c, Grid *g)
Purpose: Write this column's current state into the grid at integer positions.
Steps:
  1. For dist = 0 to length-1:
     - y = head_y - dist
     - Skip if y < 0 or y >= rows.
     - Get cell pointer at (col, y).
     - Assign ch_cache[dist] and shade_at(dist, length).

### col_paint_interpolated(Column *c, float draw_head_y, int cols, int rows)
Purpose: Draw column directly to stdscr using fractional head position.
Steps:
  1. If col >= cols, return (column out of bounds after resize).
  2. For dist = 0 to length-1:
     - row = int(floorf(draw_head_y - dist + 0.5f))
     - Skip if row < 0 or row >= rows.
     - Look up shade and attr for this dist.
     - attron(attr); mvaddch(row, col, ch_cache[dist]); attroff(attr).

### rain_init(Rain *r, int cols, int rows, int density_divisor)
Purpose: Allocate and spawn the initial column set.
Steps:
  1. Store ncols, nrows, density_divisor.
  2. calloc(cols) Column structs.
  3. Alloc grid.
  4. For x = 0..cols-1: if x % density_divisor == 0, call col_spawn.

### rain_tick(Rain *r)
Purpose: One simulation step — clear grid, scatter-erase, advance all columns.
Steps:
  1. grid_clear — wipe all cells.
  2. grid_scatter_erase — randomly dissolve some cells.
  3. For each column x:
     - If not active:
       - chance = 15 / density_divisor
       - if rand() % 100 < chance, spawn it.
     - If active:
       - call col_tick. If returns false, mark inactive.
       - Otherwise call col_paint into grid.

### rain_draw(Rain *r, float alpha, int cols, int rows)
Purpose: Render one frame — grid texture then interpolated live columns.
Steps:
  1. Pass 1 — Grid texture:
     - For each cell in grid.cells[]:
       - Skip if ch == 0 (empty).
       - Clamp x, y to current terminal bounds.
       - Draw with shade_attr.
  2. Pass 2 — Live columns:
     - For each active column:
       - draw_head_y = head_y + speed * alpha
       - col_paint_interpolated(draw_head_y).

### app_do_resize(App *app)
Purpose: Rebuild everything to new terminal dimensions.
Steps:
  1. rain_free — free column and grid arrays.
  2. screen_resize — endwin + refresh + getmaxyx.
  3. rain_init — allocate fresh at new size.
  4. Reset need_resize flag.

### app_handle_key(App *app, int ch) → bool
Purpose: Process one keypress. Returns false only on quit.
Steps:
  1. q / Q / ESC → return false.
  2. ] → sim_fps += STEP, cap at MAX.
  3. [ → sim_fps -= STEP, floor at MIN.
  4. = or + → density-- (more columns), clamp at MIN.
  5. - → density++ (fewer columns), clamp at MAX.
  6. t → theme_idx = (theme_idx + 1) % THEME_COUNT; theme_apply.

## Pseudocode — Core Loop

```
initialize random seed from clock_ns()
register atexit(cleanup), signal handlers for SIGINT/SIGTERM/SIGWINCH

screen_init()           — ncurses setup, get cols/rows
theme_apply(0)          — load green theme
rain_init(cols, rows, density=2)

frame_time = clock_ns()
sim_accum = 0

loop while running:

    if need_resize:
        free rain
        get new cols/rows
        rain_init(new cols, new rows, density)
        reset frame_time, sim_accum

    now = clock_ns()
    dt = now - frame_time
    frame_time = now
    clamp dt to 100ms max

    tick_ns = 1_000_000_000 / sim_fps
    sim_accum += dt
    while sim_accum >= tick_ns:
        rain_tick()     — physics step
        sim_accum -= tick_ns

    alpha = sim_accum / tick_ns    // [0.0, 1.0)

    update FPS display if 500ms elapsed

    elapsed = clock_ns() - frame_time + dt
    sleep(16_666_666 - elapsed)    // cap at ~60 fps

    erase()
    rain_draw(alpha)               // two-pass: grid + interpolated
    screen_draw_hud(fps, sim_fps, density, theme)
    wnoutrefresh(); doupdate()

    ch = getch()  // non-blocking
    if ch != ERR and not handle_key(ch):
        break

rain_free()
endwin()
```

## Interactions Between Modules

```
main() [§9]
  ├─ clock_ns()           [§2] — dt measurement
  ├─ screen_init()        [§8] — ncurses init
  ├─ theme_apply()        [§3] — color pair registration
  ├─ rain_init()          [§7]
  │     └─ col_spawn()    [§6] — per active column
  │     └─ grid_alloc()   [§5]
  │
  ├─ rain_tick()          [§7] — physics
  │     ├─ grid_clear()   [§5]
  │     ├─ grid_scatter_erase() [§5]
  │     ├─ col_tick()     [§6]
  │     └─ col_paint()    [§6] → writes to grid [§5]
  │
  ├─ screen_draw_rain()   [§8]
  │     └─ rain_draw()    [§7]
  │           ├─ Pass 1: reads grid [§5] → shade_attr [§3] → mvaddch
  │           └─ Pass 2: col_paint_interpolated [§6] → shade_attr [§3] → mvaddch
  │
  ├─ screen_draw_hud()    [§8] — reads k_themes[].name [§3]
  └─ screen_present()     [§8] — doupdate

Shared state:
  - Rain struct owns: Column[] and Grid (the only mutable simulation data)
  - g_app global: signal handlers write running=0 and need_resize=1
  - ncurses stdscr: all drawing functions write into this global window
```
