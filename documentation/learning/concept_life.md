# Pass 1 — artistic/life.c: Conway's Game of Life + Variants

## Core Idea

A grid of cells, each either alive or dead. Each generation, every cell counts its 8 neighbours and applies a birth/survival rule. Conway's rule says: a dead cell with exactly 3 neighbours is born; a live cell with 2 or 3 neighbours survives; otherwise it dies. From these simple rules, complex structures emerge: gliders that move, oscillators that pulse, and the Gosper gun that creates gliders indefinitely. Five rule variants produce different emergent behaviours.

## The Mental Model

Imagine a colony of bacteria on a Petri dish. Each bacterium dies if isolated (too few neighbours) or overcrowded (too many neighbours). A dead spot can spontaneously grow a new bacterium if exactly the right number of neighbours surround it. The specific threshold numbers determine whether the colony produces stable patterns, gliders, or explosive chaos.

## B/S Bitmask Rule Encoding

Each rule is stored as two 16-bit integers where bit N is set if neighbor count N triggers the event:

```c
uint16_t bit = (uint16_t)(1u << n);   // n = live neighbor count (0-8)
uint8_t  nxt = cur ? ((survive & bit) ? 1 : 0)
                   : ((birth   & bit) ? 1 : 0);
```

Conway B3/S23:
- `birth   = 0b000001000` — bit 3 set → born with 3 neighbors
- `survive = 0b000001100` — bits 2,3 set → survives with 2 or 3 neighbors

This encoding makes any B/S rule trivial to add: just specify which neighbor counts matter.

## Six Rules

| Rule | Birth | Survive | Character |
|---|---|---|---|
| Conway B3/S23 | 3 | 2,3 | The classic; gliders, guns, oscillators |
| HighLife B36/S23 | 3,6 | 2,3 | Like Conway but gliders can self-replicate |
| Day&Night B3678/S34678 | 3,6,7,8 | 3,4,6,7,8 | Symmetric: dead and alive regions follow same rule |
| Seeds B2/S | 2 | none | Every live cell dies; explosive growth from pairs |
| Morley B368/S245 | 3,6,8 | 2,4,5 | Chaotic; moving structures |
| 2×2 B36/S125 | 3,6 | 1,2,5 | Tiling patterns of 2×2 blocks |

## Double-Buffered Grid

`g_grid[2][MAX_ROWS][MAX_COLS]` — `g_buf` holds the current generation index. The step function reads from `g_grid[g_buf]` and writes into `g_grid[1 - g_buf]`. After the step, `g_buf` flips. This ensures every cell reads the same generation — no cell sees its neighbours' already-updated values.

## Seed Patterns

**Glider:** 5 cells in a specific L-shape; moves diagonally across the grid every 4 generations.

**Gosper Glider Gun:** 36 cells that produce a new glider every 30 generations indefinitely. Hardcoded as `static const int GG[][2]` coordinate array. Placed at offset `(g_ca_rows/4, 5)` to leave room for gliders to travel.

**R-Pentomino:** 5 cells that evolve chaotically for 1,103 generations before stabilising. Classic chaos demonstration.

**Acorn:** 7 cells that evolve for 5,206 generations, producing 633 cells and 13 gliders.

## Population Histogram

A `HIST_LEN = 512` ring buffer stores the live cell count per generation. The bottom 3 rows of the screen render as a bar chart:

```c
int level = (int)((float)pop / (float)max_pop * HIST_ROWS * 2.0f);
```

Bars grow upward. This shows oscillations (oscillators produce periodic spikes), exponential growth (Seeds rule exploding), and extinction events (all cells die).

## Non-Obvious Decisions

### Why `g_steps` (multiple generations per frame)?
Some patterns take thousands of generations to develop interesting structure. Running 3–30 steps per frame lets the user fast-forward to the interesting part.

### Why `p` is prev-rule rather than R-pentomino?
`p` was the natural choice for "previous", and the R-pentomino seed was assigned to `e` (for "pEntomino") to avoid the conflict. The HUD notes both.

### Why toroidal (wraparound) boundary?
Absorbing boundaries (cells outside the grid are always dead) create artificial edge effects — gliders die at edges, patterns near edges behave differently. Toroidal topology means every cell has exactly 8 neighbours and the grid is homogeneous.

## From the Source

**Algorithm:** Conway's Game of Life and variants using B/S rule bitmasks. Birth/survival encoded as bit N = 1 in a uint16 mask where N is the neighbour count (0–8). This allows checking any rule with a single bitshift: born = (birth>>n)&1. Double-buffered toroidal grid; update is O(rows × cols).

**Physics:** Emergent complexity from local rules (Conway, 1970): despite only two states and 9 neighbour values, Life supports stable structures (still-lifes), oscillators, spaceships (gliders), and Turing-complete computation. The Gosper gun produces an infinite glider stream with period 30.

**Data-structure:** Population histogram ring buffer — last HIST_LEN generation counts stored; drawn as a 3-row bar chart using a 4-level block character ramp (▁▂▄█).

**Performance:** Per-step cost: O(rows × cols) neighbour-count evaluation. Steps per frame adjustable (1–16) to allow fast-forward to interesting evolved states.

## Key Constants

| Constant | Effect |
|---|---|
| MAX_ROWS × MAX_COLS | Grid size cap; larger → more room for structures |
| HIST_LEN | Ring buffer length for population history |
| HIST_ROWS | Screen rows used by histogram bar chart |
| STEPS_MAX | Maximum generations per frame (speed ceiling) |
