# Pass 1 — artistic/leaf_fall.c: ASCII Tree with Leaf Fall

## Core Idea

A procedural ASCII tree is grown on the terminal grid and displayed for 2.5 seconds. Then the leaves detach and fall downward in matrix-rain style — each leaf column has a bright white head trailing a green smear. When all leaves have settled, the screen blanks briefly and a new randomly varied tree appears.

## The Mental Model

Think of autumn: the tree stands still, then a gust of wind loosens the leaves and they rain down in streams. Each stream moves independently — some start immediately, some wait a moment — giving the fall a natural, staggered look. Once the ground is covered, the scene resets with a fresh tree.

## State Machine

```
ST_DISPLAY (2.5 s)     — static tree is shown
     ↓ (timeout or spacebar)
ST_FALLING             — leaf columns rain downward
     ↓ (all heads reached bottom)
ST_RESET (0.7 s)       — brief blank before next tree
     ↓
new tree → ST_DISPLAY
```

`spc` immediately skips to the next state.

## Tree Generation

The trunk is a vertical column of `|` characters from the bottom of the screen to about 50–65% of the height (randomised each tree). From the trunk tip, branches are grown recursively:

- Each branch has a starting position, heading angle, length, and depth level.
- At each branch tip, it either terminates (placing a leaf cluster) or splits into two sub-branches with slightly randomised angles and lengths.
- Branch depth is limited to `MAX_DEPTH = 7`. Characters used: `|` for near-vertical, `/` and `\` for diagonal, `-` for near-horizontal.
- Leaves are placed at terminal tips as `*` characters (or similar dense chars).

The branching stack (`BSTACK`) holds up to 512 pending branch segments, avoiding deep recursion.

## Leaf Fall

Each leaf column tracks:
```
int   head_r     — current row of the bright white head
int   start_r    — starting row (where the leaf was in the tree)
int   delay      — ticks before this column starts falling
bool  active     — still falling
```

The head moves downward one row every `FALL_NS = 55 ms`. Behind the head, a trail of `TRAIL_LEN = 7` characters is drawn in green, fading from the head backward. Once the head reaches `g_rows - 1` (bottom of screen), the column deactivates. When all columns are inactive, the state transitions to `ST_RESET`.

The `MAX_START_DELAY = 80` ticks stagger the fall start times — columns near the centre of the tree may start immediately while outer branches start up to 80 ticks later, giving an organic staggered appearance.

## Non-Obvious Decisions

### Why a branching stack instead of recursion?
Deep recursion for 128+ branch segments risks stack overflow. A manual stack (`BSTACK`) processes segments iteratively, avoiding any stack depth issues and making the generation order explicit.

### Why randomise trunk height per tree?
A fixed tree height would make every tree look like a scaled copy of the previous one. Varying trunk height (45–65%) combined with randomised branch angles makes each generation visually distinct.

### Why TRAIL_LEN = 7?
Long enough to see the trailing smear (making each column look like a falling stream rather than a single moving dot), short enough that the tail disappears quickly after the head passes. 7 is empirically good for 30 fps.

### Why stagger start delays?
Without staggering, all columns would start falling at the same moment, looking like an animation artifact. The stagger makes the fall look like leaves loosening and drifting one by one.

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_grid[GRID_ROWS][GRID_COLS]` | `char[]` | ~40 KB | virtual render grid (128 × 320) holding tree characters |
| `g_leaves[MAX_LEAVES]` | `struct{int col,row,delay}[4096]` | ~48 KB | leaf pool — column, start row, remaining delay ticks |
| `g_bstack[BSTACK_MAX]` | branch stack | ~10 KB | iterative DFS stack for tree generation (avoids deep recursion) |
| `MAX_LEAVES` | `int` constant | N/A | leaf pool capacity (4096) |
| `TRAIL_LEN` | `int` constant | N/A | green trail length behind white fall head (7 rows) |
| `MAX_START_DELAY` | `int` constant | N/A | max stagger delay in ticks (80) for wave-like leaf fall |

## From the Source

**Algorithm:** Two-phase animation: stochastic recursive tree growth using a branching stack (`BSTACK_MAX=512`) for iterative DFS — avoids stack overflow from deep recursion. Then matrix-rain-style leaf fall where each leaf column streams downward with a white head and green trail. Branch characters chosen by angle: `|` near-vertical, `/\` diagonal, `-` near-horizontal.

**Physics:** Leaf fall runs at `FALL_NS=55ms` per step (~18 fps) — slower than the 30 fps render tick. `MAX_START_DELAY=80` ticks stagger the fall so leaves drop in waves rather than simultaneously. Trail drawn by remembering last `TRAIL_LEN=7` row positions per leaf column.

**Data-structure:** Leaf pool: `MAX_LEAVES=4096` structs of `(col, row, delay)`. Tree drawn into a virtual grid `GRID_ROWS=128 × GRID_COLS=320` before being projected to the actual terminal. `TRUNK_H_MIN=0.45 / TRUNK_H_MAX=0.65` randomises trunk height fraction each tree for visual variety. `BRANCH_SPREAD=0.50 rad` is the angle jitter range at each branch fork.

**State machine:** DISPLAY (2.5 s, `DISPLAY_NS`) → FALLING (all heads reach bottom) → RESET (0.7 s, `RESET_NS`) → new tree. `spc` immediately skips to the next state.

## Key Constants

| Constant | Effect |
|---|---|
| DISPLAY_NS | How long the tree is shown before fall starts |
| FALL_NS | Speed of leaf fall (lower → faster) |
| TRAIL_LEN | Length of green smear behind white head |
| MAX_START_DELAY | Maximum stagger between column fall starts |
| MAX_DEPTH | Maximum branch recursion depth |
| TRUNK_H_MIN/MAX | Range for random trunk height as fraction of screen height |
