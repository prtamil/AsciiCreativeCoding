# Pass 1 — lightning.c: Fractal branching lightning in a dark terminal sky

## Core Idea

A single bolt tip starts at the top-center of the screen. Each tick it advances one cell downward, leaning left or right according to a persistent integer lean bias (−2 to +2). After at least MIN_FORK_STEPS steps, the tip may fork into two child tips that inherit lean ± 1. This produces a fractal binary tree that spreads as it descends. Characters encode direction: `|` for straight, `/` for left-lean, `\` for right-lean. A depth-color gradient (light blue → teal → white) runs top to bottom. A Manhattan-radius-2 glow halo is drawn around each frozen cell at draw time. The state machine runs GROWING → ST_STRIKING (full bolt blazes) → ST_DARK (screen goes black) → new bolt.

## The Mental Model

Imagine static electricity discharging from a cloud to the ground. A single leader channel advances downward, occasionally splitting into two branches, each of which can split again. Each branch has a persistent "lean" — if it started moving slightly left it continues to move left, giving branches a characteristic directional bias. This is why real lightning has recognizable arm-like branches rather than random zigzags.

The fractal structure emerges from the binary tree: at level 0 you have one branch, at level 1 two, at level 2 four, etc. The lean inheritance (child lean = parent lean ± 1) means branches spread apart by 1 cell per fork — exactly the right rate to prevent overlap.

The glow halo is a visual effect that makes the bolt look luminous rather than just a thin line. The inner corona (distance 1) uses dim teal `|`; the outer halo (distance 2) uses dim deep-blue `.`. These are computed at draw time from the frozen cell grid — no separate data structure is needed.

## Data Structures

### Grid (§4)
```
uint8_t frozen[GRID_ROWS_MAX][GRID_COLS_MAX]   — 0=empty; stores color+direction encoding
uint8_t glow[GRID_ROWS_MAX][GRID_COLS_MAX]     — glow halo cells (set at draw time)
int     rows, cols
int     deepest_row     — row of the deepest frozen tip (for shockwave)
```
The frozen cell encoding packs both the direction character and color band into a single byte — this avoids a separate color array.

### Tip (§5)
```
int   row, col
int   lean          — −2 to +2; column delta per step = lean / 2 (rounded)
int   steps_since_fork
bool  alive
```
Tips are managed as a fixed-size pool (TIPS_MAX). Dead tips are recycled.

### Scene (§6)
```
Tip      tips[TIPS_MAX]
int      n_tips
Grid     grid
State    state          — ST_GROWING / ST_STRIKING / ST_DARK
int      state_ticks    — how long in current state
int      shockwave_r    — expanding ring radius during ST_STRIKING
```

## The Main Loop

1. Resize: rebuild grid, reset scene to ST_DARK.
2. Measure dt.
3. Ticks: advance all alive tips one step (move down, check fork, freeze cell); when all tips are done or ground is reached, advance state machine.
4. Draw: glow halo overlay, then frozen bolt cells, then active tip `!` characters.
5. HUD: fps, fork probability, speed.
6. Input: r=new bolt, +/-=fork probability, [/]=speed, p=pause.

## Non-Obvious Decisions

### Lean bias in integers, not floats
`lean` is an integer in {−2, −1, 0, 1, 2}. The column delta is `lean / 2` (integer division). This means:
- lean 0: straight (delta=0)
- lean ±1: still straight (1/2 = 0 in integer division) — slight lean but no visible drift per step
- lean ±2: diagonal (delta=±1) — moves one cell left or right every step

This discrete encoding ensures the bolt always occupies exactly one cell per row — no two cells at the same row for one branch. Without this constraint, sub-pixel lean would need special handling.

### Persistent lean bias (not per-step random)
If the lean were re-randomized every step, the bolt would look like a random walk (rough zigzag). Persistent lean means a branch "commits" to a direction for multiple steps, producing the smooth diagonal arms you see in real lightning.

### Fork condition: MIN_FORK_STEPS + probability
A tip only forks if it has taken at least MIN_FORK_STEPS since its last fork. Without this, early forks would produce a dense blob near the top rather than a spreading tree. The probability is user-adjustable (+/- keys) so you can get everything from a single straight bolt (0%) to a dense fractal tree (50%+).

### Lean inheritance: child lean = parent lean ± 1
When a tip forks, one child gets `lean - 1` and the other gets `lean + 1`. This means branches diverge at ±1 lean from each other. If the parent has lean 0, children get -1 and +1 (both still straight, but ready to diverge further). If the parent has lean 2, children get 1 and 3 — but lean is clamped to [−2, +2], so one child stays at 2 and the other goes to 3→clamped 2. This prevents extreme lean accumulation.

### Glow halo computed at draw time
Rather than storing glow intensity in a separate buffer during simulation, the draw function iterates every frozen cell's Manhattan neighbors and writes dim characters to empty cells. This is O(frozen_cells × halo_area) but fast enough in practice and avoids maintaining a separate buffer through state changes.

### ST_DARK state for dramatic effect
After the bolt strikes, the screen goes completely black for approximately 1 second (ST_DARK_TICKS ticks). This simulates the moment after a lightning flash when your eyes are adjusting — and dramatically resets the screen so the next bolt appears from nothing.

## State Machines

```
ST_DARK ──── dark_ticks >= DARK_TICKS ────► ST_GROWING (new bolt, random x)
    ▲                                              │
    │                                    all tips done/grounded
    │                                              │
ST_STRIKING ──── strike_ticks >= STRIKE_TICKS ────┘
    ▲
    └──── all tips grounded (during ST_GROWING) → ST_STRIKING
```

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `MIN_FORK_STEPS` | ~8 | Lower = more forks near top (denser blob); higher = longer arms before branching |
| `FORK_PROB` (default) | ~30% | Higher = denser branching tree; 0 = single straight bolt |
| `TIPS_MAX` | ~128 | Maximum concurrent active tips; 2^FORK_DEPTH |
| `DARK_TICKS` | ~30 | ~1 s blank screen between bolts |
| `STRIKE_TICKS` | ~15 | ~0.5 s full-bolt flash |

## Open Questions for Pass 3

- What exactly is the frozen cell encoding? Is it a bitmask of (color_band << 2 | direction_index)?
- How is the shockwave ring drawn during ST_STRIKING? Is it a circle or a ring of cells at radius shockwave_r?
- If lean is clamped to [−2, +2] and a child would exceed this, is the child clamped silently or is the fork suppressed?
- How does the code determine that "all tips are done" — does it scan all tip slots or maintain an active count?

---

## From the Source

**Algorithm:** Recursive binary-tree branching with persistent lean bias. Each active tip moves one cell downward per tick, leaning ±1 column per `MIN_FORK_STEPS` steps. After `MIN_FORK_STEPS`, a tip may fork — producing two child tips with bias ±1 from parent's bias. This is NOT a DLA simulation; there are no random walkers — only the branching probability is stochastic.

**Math:** A tip with lean bias `b` traces a path offset by ±b cells per row descended. After `d` rows, the tip is at column `c₀ ± b·d`. With uniform branching, the horizontal spread of tips grows as O(d) — creating a roughly triangular shape. The path width (cells visited) is a fractal between 1 and 2D: single-cell-wide paths with unlimited branching have D ≈ 1.5.

**Visual:** The shockwave (striking phase) expands as a Manhattan-radius ring from the lowest strike point, fading over time. Characters encode direction: `|` straight, `/` `\` leaning. Color depth (row / total_rows) maps top-to-bottom: blue → teal → white.

**Note:** `lightning.c` lives in `fractal_random/lightning.c`, not in `geometry/`.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app.scene.grid.cells[GRID_ROWS_MAX][GRID_COLS_MAX]` | `uint8_t[80][300]` | ~24 KB | Frozen bolt channel: encodes direction and depth colour per cell |
| `g_app.scene.grid.glow[GRID_ROWS_MAX][GRID_COLS_MAX]` | `uint8_t[80][300]` | ~24 KB | Ambient halo level (0=none, 1=outer, 2=inner) per cell |
| `g_app.scene.tips[MAX_TIPS]` | `Tip[64]` | ~1 KB | Pool of active and inactive growing branch tips |

# Pass 2 — lightning: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | MIN_FORK_STEPS, FORK_PROB, TIPS_MAX, DARK/STRIKE ticks |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | Bolt colors by depth (xterm 45/51/231), glow colors |
| §4 grid | frozen[][], `grid_reset()`, `grid_draw()`, glow overlay |
| §5 tip | Tip struct, `tip_step()`, `tip_fork()` |
| §6 scene | Tip pool, state machine, shockwave, `scene_tick()`, `scene_draw()` |
| §7 screen | ncurses init/draw/present/resize, HUD |
| §8 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
scene_tick() [ST_GROWING]:
  for each alive tip:
    col_delta = tip.lean / 2
    tip.row++; tip.col += col_delta
    char = (col_delta==0) ? '|' : (col_delta<0) ? '/' : '\\'
    color = depth_color(tip.row)
    frozen[tip.row][tip.col] = encode(char, color)
    tip.steps_since_fork++
    if tip.row >= rows: tip.alive = false; track deepest
    elif tip.steps_since_fork >= MIN_FORK_STEPS:
      if rand()%100 < fork_prob:
        fork tip into two children (lean±1)
        tip.alive = false
  if all tips dead: state = ST_STRIKING

scene_draw():
  [glow pass]
  for each frozen (r, c):
    for (dr, dc) in Manhattan(radius 1): draw inner glow if empty
    for (dr, dc) in Manhattan(radius 2): draw outer glow if empty
  [bolt pass]
  for each frozen (r, c):
    (ch, color) = decode(frozen[r][c])
    mvwaddch(ch with color)
  [active tips]
  for each alive tip:
    mvwaddch('!' bright-white at tip.row, tip.col)
```

---

## Function Breakdown

### depth_color(row) → color_id
```
if row < rows/3:   return COL_BOLT_TOP   (xterm 45 light blue)
if row < 2*rows/3: return COL_BOLT_MID   (xterm 51 teal)
else:              return COL_BOLT_BOT   (xterm 231 white)
```

### tip_fork(tip) → (child1, child2)
```
child1.row = tip.row; child1.col = tip.col
child1.lean = clamp(tip.lean - 1, -2, +2)
child1.steps_since_fork = 0; child1.alive = true

child2 = same but lean = clamp(tip.lean + 1, -2, +2)
tip.alive = false
```

---

## Pseudocode — Core Loop

```
setup:
  state = ST_DARK
  state_ticks = 0

main loop:
  1. resize → grid_reset, reset to ST_DARK

  2. dt, cap 100ms

  3. physics ticks:
     while sim_accum >= tick_ns:
       if not paused:
         if state == ST_GROWING: scene_tick_growing()
         if state == ST_STRIKING: scene_tick_striking()  ← expand shockwave
         if state == ST_DARK:
           state_ticks++
           if state_ticks >= DARK_TICKS:
             grid_reset(); spawn_bolt(rand_x); state = ST_GROWING

  4. frame cap sleep

  5. draw:
     erase()
     scene_draw()
     HUD: fps, fork_prob, speed
     wnoutrefresh + doupdate

  6. input:
     q/ESC → quit
     r     → force new bolt
     + =   → fork_prob++
     -     → fork_prob--
     ] [   → sim_fps
     p/spc → pause
```

---

## Interactions Between Modules

```
App
 └── scene_tick → tip_step → tip_fork → pool management
               → state transitions: GROWING→STRIKING→DARK→GROWING

Grid
 ├── frozen[][] — set during GROWING, cleared at start of DARK
 └── read in scene_draw for glow+bolt rendering

Tip pool
 └── fixed array; alive flag used for pool management
```
