# Pass 1 — snowflake.c: DLA crystal with D6 six-fold symmetry

## Core Idea

Random walkers drift inward from the terminal edges. When a walker makes contact with the frozen aggregate it sticks in place, and simultaneously all 12 positions in the dihedral-6 symmetry group are frozen too — six rotations times two reflections. This forces the classic six-armed hexagonal snowflake morphology without explicitly coding six arms. Color is distance-based: cells near the center are deep navy and cells near the tips are bright white. The displayed character at each frozen cell is chosen from the local neighborhood topology to look like ice crystal structure.

## The Mental Model

Picture an ice crystal forming in a cloud. Water molecules (walkers) drift randomly until they touch the growing crystal and freeze. In a real snowflake all six arms grow simultaneously because they experience the same temperature and humidity — here that is enforced mathematically by the D6 symmetry operation. Every time one arm grows, the other five arms and all six mirror positions grow by the exact same amount in the exact same direction.

The D6 group on a terminal grid is approximate because a terminal is rectangular and characters are taller than wide (ASPECT_R ≈ 2). To make the six arms look equiangular, positions are computed in isotropic pixel space and converted back to (col, row). Without this correction the snowflake would be squashed and the arms would not appear 60° apart.

Characters are chosen from the frozen-neighbor topology: `*` for isolated tips, `-` for horizontal segments, `|` for vertical segments, `+` for junctions, and `/` `\` for diagonal struts. These are determined after each freeze based on which of the eight neighbors are also frozen.

## Data Structures

### Grid (§4)
```
uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX]  — frozen state per cell (0 = empty)
int8_t  color[GRID_ROWS_MAX][GRID_COLS_MAX]  — color ID per frozen cell
int     rows, cols
int     cx, cy                               — center pixel coordinates
float   scale                                — Euclidean units per terminal row
```
The grid is a static 2-D array. Frozen cells are never unfrozen within a cycle.

### Walker (§5)
```
float px, py    — position in pixel space (isotropic, CELL_W=8, CELL_H=16)
bool  alive     — false if out-of-bounds or just stuck
```
Walkers are allocated from a fixed pool. When a walker sticks, the pool slot is reused.

### Scene (§6)
```
Grid    grid
Walker  walkers[WALKER_MAX]
int     n_walkers          — active walker count
bool    paused
int     done_ticks         — counts hold period after crystal fills the screen
```

## The Main Loop

1. Check resize signal; rebuild grid at new dimensions, re-center, reset.
2. Measure dt, cap at 100ms.
3. Physics tick(s): advance all walkers one step; test each walker for contact with frozen cells; on contact, maybe stick (STICK_PROB); on stick, freeze all 12 D6 positions.
4. Respawn dead walkers from random edge positions.
5. If crystal reaches center, hold for done_ticks, then reset.
6. Draw: for each frozen cell write its topology character with distance-based color.
7. HUD: fps, walker count, sim speed.
8. Input: r=reset, +/-=walkers, [/]=speed, p=pause.

## Non-Obvious Decisions

### Why freeze all 12 positions simultaneously?
DLA normally grows asymmetrically — random chance determines which direction each arm grows first. Without symmetry enforcement, real DLA produces irregular blobs and dendrites, not snowflakes. The forced D6 group means every stochastic event (a walker sticking to one arm) produces a perfectly symmetric event in the other 11 positions. The result looks identical to a real snowflake's growth.

### Why pixel-space coordinates for walkers?
The terminal has non-square cells (CELL_H=16, CELL_W=8 — roughly 2:1 aspect). A walker moving in cell-space at 45° would travel a diagonal that looks far steeper than 45° visually. Physics in pixel space with `CELL_W=8, CELL_H=16` makes the Euclidean distances isotropic: a diagonal walker really does travel 45° visually.

### STICK_PROB = 0.55 — why not 1.0?
At 100% sticking probability, DLA produces very sparse, thin fractal arms with no branching — every walker that even touches the aggregate immediately freezes, and no walkers ever reach inner regions. At 0.55, nearly half of all contacts are rejected, so walkers slide along arm surfaces and penetrate deeper before freezing. This produces significantly thicker, denser arms with rounded edges — closer to the appearance of a real ice crystal than the sparse fractal produced by high stick probability.

### Glow halo — two-pass draw
The crystal arms are 1 cell wide structurally, but visually appear thin on a terminal. A two-pass draw solves this: Pass 1 paints a dim `:` on every empty 8-neighbor of every frozen cell (using the frozen cell's own color band), creating a soft halo. Pass 2 draws the frozen cells themselves on top. The result makes each arm appear ~3 cells wide visually without changing the DLA aggregate at all — purely a rendering trick.

### Distance-based color gradient
Color is assigned once at freeze time using Euclidean distance from center. Deep navy (xterm 117) at the core transitions through ocean blue (38), teal (44), pale ice (195), and bright white (231) at the tips. This means old frozen cells (near center) are always dark and recently frozen cells (tips) are always bright — the gradient reads naturally as depth.

### Context-sensitive characters
After freezing, each cell examines its eight neighbors. A horizontal neighbor but no vertical neighbors → `-`. Vertical but no horizontal → `|`. Both → `+`. Both diagonals but nothing orthogonal → `/` or `\`. No neighbors at all → `*` (tip). This topology-derived labeling gives the crystal a realistic structural appearance.

## State Machines

```
GROWING ──── crystal fills center or r key ────► RESET (via grid_init)
    ↑                                                   │
    └───────────────────── immediately ─────────────────┘
```

There is no DONE hold state; on auto-fill the scene resets immediately.

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `STICK_PROB` | 0.55 | Lower = denser, thicker arms (walkers bounce more); higher = sparser, spikier fractal |
| `WALKER_DEFAULT` | 80 | More = faster growth; fewer needed than basic DLA due to 12-way freeze |
| `WALKER_MAX` | 200 | Upper bound for `+` key |
| `N_ICE_COLORS` | 6 | Number of distance color bands |
| `SIM_FPS_DEFAULT` | 30 | Ticks per second; lower = slower visible growth |

## Open Questions for Pass 3

- How exactly are the 12 D6 positions computed from a single (row, col) contact? Which of the six rotations is applied first and how is reflection handled?
- When two of the 12 mirrored positions coincide (e.g., on the symmetry axes), is the freeze skipped or applied twice?
- Walker respawn: are walkers spawned uniformly along all four edges, or biased toward certain edges?
- How does the topology character selection handle the case where a newly frozen cell is deep inside the aggregate (surrounded on all sides)?

## From the Source

**Algorithm:** DLA with 6-fold (D6) symmetry constraint. Each walker's trajectory is computed only for one of 6 symmetric sectors. When it sticks, 5 mirror copies are placed simultaneously. This enforces hexagonal symmetry throughout the growth, unlike diffusion_map.c which grows freely in any direction.

**Math:** D6 symmetry group: 6 rotations × (π/3 each) + 6 reflections. The aggregate remains invariant under all 12 symmetry operations. Fractal dimension of DLA in 2D ≈ 1.71; with D6 symmetry the 6 arms grow independently but share the same statistical properties. STICK_PROB=0.55 → thicker, denser arms with rounded edges. 0.90 → sparse, spiky fractal arms (classic DLA morphology).

**Physics:** Real snow crystal formation: water vapour condenses on a dust nucleation site and diffuses outward. The hexagonal symmetry comes from the molecular geometry of ice (H₂O ice Ih). DLA with D6 symmetry captures this growth morphology — the branching and tip-screening effects produce the characteristic dendritic snowflake arms.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `scene.grid.cells[80][300]` | `uint8_t[80][300]` | ~24 KB | crystal lattice; value encodes distance-based color band |
| `scene.walkers[200]` | `Walker[200]` | ~2 KB | active DLA random-walk particles (cx, cy, active flag) |
| `scene.grid.frozen_count` | `int` | 4 B | total frozen cells in crystal |
| `scene.grid.cx0`, `scene.grid.cy0` | `int` | 8 B | grid center for D6 symmetry mapping |

# Pass 2 — snowflake: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | STICK_PROB, walker counts, grid limits, color count |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 6 ice palette colors, `color_init()` |
| §4 grid | 2-D frozen array, `grid_freeze()`, `grid_draw()`, context-char selection |
| §5 walker | Walker pool, `walker_step()`, contact test, D6 symmetry freeze |
| §6 scene | Owns grid + walker pool, `scene_tick()`, `scene_draw()` |
| §7 screen | ncurses init/draw/present/resize, HUD |
| §8 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
scene_tick():
  for each alive walker:
    walker_step()          ← random ±1 in pixel space
    contact = check_neighbors_frozen(px, py)
    if contact:
      if rand() < STICK_PROB:
        freeze_d6(px, py)  ← freeze all 12 D6 positions
        respawn walker from random edge
      else:
        continue walking   ← rejection: 10% chance

freeze_d6(px, py):
  for angle in {0°, 60°, 120°, 180°, 240°, 300°}:
    for reflection in {identity, mirror}:
      (r, c) = transform(px, py, angle, reflection)
      if in bounds and not already frozen:
        grid.cells[r][c] = frozen
        grid.color[r][c] = distance_color(r, c)

scene_draw():
  for each frozen (r, c):
    ch = topology_char(r, c)   ← examine 8 neighbors
    color = grid.color[r][c]
    mvwaddch(ch with color pair)
```

---

## Function Breakdown

### freeze_d6(px, py)
Purpose: freeze a single contact point and all 11 D6 symmetric partners.
Steps:
1. Compute Euclidean coords (ex, ey) from pixel coords relative to center.
2. For each of 6 rotation angles: rotate (ex, ey) by angle, convert to (col, row).
3. For each rotated point, also apply the horizontal reflection: (−ex, ey).
4. For each of the 12 resulting positions: bounds-check, skip if already frozen, set `cells[r][c]=1`, assign `color[r][c]` from distance.
5. Update topology chars for the newly frozen cell and its immediate neighbors.

### topology_char(r, c) → char
Purpose: select display character based on frozen neighborhood.
Steps:
1. Test if (r, c±1) frozen → h (horizontal neighbors present).
2. Test if (r±1, c) frozen → v (vertical neighbors present).
3. Test if (r±1, c±1) frozen → d (diagonal neighbors present).
4. Return: `*` if none; `-` if h only; `|` if v only; `+` if both h and v; `\` or `/` if d pattern.

---

## Pseudocode — Core Loop

```
setup:
  grid_init(cols, rows)    ← compute cx, cy, scale
  walker_pool_spawn(WALKER_DEFAULT)
  frame_time = clock_ns()

main loop:
  1. resize → grid_init at new size, respawn walkers

  2. dt = now − frame_time; cap 100ms

  3. physics ticks:
     sim_accum += dt
     while sim_accum >= tick_ns:
       if not paused: scene_tick()
       sim_accum -= tick_ns

  4. frame cap sleep

  5. draw:
     erase()
     scene_draw()     ← frozen cells with topology chars + colors
     HUD: fps, walker count, speed
     wnoutrefresh + doupdate

  6. input:
     q/ESC → quit
     r     → grid_init (reset)
     + =   → n_walkers++, spawn new walker
     -     → n_walkers--, kill one walker
     ] [   → sim_fps ± step
     p/spc → toggle paused
```

---

## Interactions Between Modules

```
App
 └── main loop → scene_tick → walker_step → contact check → freeze_d6
                            → scene_draw → topology_char → mvwaddch

Grid
 ├── cells[][] — frozen state
 ├── color[][] — assigned at freeze time, never changed
 └── cx, cy, scale — for D6 transform and distance coloring

Walker pool
 ├── alive[] — pool slots
 └── px, py  — pixel-space position; converted to cell only for grid lookup
```
