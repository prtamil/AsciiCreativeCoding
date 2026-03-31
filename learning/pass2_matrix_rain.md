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
