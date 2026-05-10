# grids — drawing grids and placing objects on them

A reference for the discrete-grid backbone of every demo in this project.
This folder contains **76 self-contained C programs** organised as four grid
families (rect, hex, tri, polar) crossed with four placement modes (direct,
patterns, path, scatter). Together they answer two questions every creative-
coding sketch eventually faces:

1. **How do I draw a grid in the terminal?** — given a chosen geometry,
   convert grid addresses to screen cells and back, frame after frame.
2. **How do I put objects on it?** — store, query, and render an arbitrary
   collection of objects whose positions live in *grid space*, not pixel
   space, so they survive resize and grid-mode switching.

Every file in this folder is built around the **same two primitives**:
`GridCtx` (the grid ↔ screen mapping) and `Pool` (a placement-mode-agnostic
object store). The whole rest of this README is the long answer to *what
those two primitives are and how each grid family realises them*.

If you read **only one file**, read
[`rect_grids/01_uniform_rect.c`](rect_grids/01_uniform_rect.c) — it is the
canonical exemplar and the entire taxonomy is downstream of it.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The two primitives](#the-two-primitives)
   * [GridCtx — grid ↔ screen](#1-gridctx--grid--screen)
   * [Pool — placement-mode-agnostic storage](#2-pool--placement-mode-agnostic-storage)
   * [Why this split is universal](#why-this-split-is-universal)
3. [Drawing the grid — four families](#drawing-the-grid--four-families)
   * [`rect_grids/`](#rect_grids--14-files--modify-one-thing-pedagogy)
   * [`hex_grids/`](#hex_grids--7-files--cube-coordinates-and-archimedean-tilings)
   * [`tri_grids/`](#tri_grids--12-files--three-family-stripes-and-substitution)
   * [`polar_grids/`](#polar_grids--7-files--radial-coordinate-systems)
4. [Placing objects — four modes](#placing-objects--four-modes)
   * [Mode 1 — direct (cursor + toggle)](#mode-1--direct-cursor--toggle)
   * [Mode 2 — patterns (parametric stamps)](#mode-2--patterns-parametric-stamps)
   * [Mode 3 — path (two-point traversals)](#mode-3--path-two-point-traversals)
   * [Mode 4 — scatter (procedural sampling)](#mode-4--scatter-procedural-sampling)
5. [Why `tri_grids_placement/` has 24 files](#why-tri_grids_placement-has-24-files)
6. [Building and running](#building-and-running)
7. [Adding a new grid](#adding-a-new-grid)
8. [Quick-reference file index](#quick-reference-file-index)

---

## How to read this folder

The recommended path through the 76 files goes from the simplest possible
mapping (rect) to the richest (polar / tri tilings), then layers placement
modes on top. Each step changes one thing at a time.

```
        DRAWING                                 PLACEMENT
        ───────                                 ─────────
   1.  rect_grids/01_uniform_rect.c   ──▶  rect_grids_placement/01_direct.c
        the base formula                      cursor + toggle on rect
                │                                       │
                ▼                                       ▼
   2.  rect_grids/02..14                   rect_grids_placement/02_patterns
        modify one knob each                 03_path, 04_scatter
                │                                       │
                ▼                                       ▼
   3.  hex_grids/01_flat_top.c        ──▶  hex_grids_placement/01..04
        cube coordinates Q+R+S=0
                │
                ▼
   4.  tri_grids/01_equilateral.c     ──▶  tri_grids_placement/01..06_×_4
        three-family stripe classifier        per-tri-type × per-mode
                │
                ▼
   5.  polar_grids/01_rings_spokes.c  ──▶  polar_grids_placement/01..04
        polar (r, θ) coordinates
```

**Prerequisites graph.** Every file states *Study alongside:* in its header.
Follow those pointers in either direction:

* `rect_grids/01_uniform_rect.c` is the root; every other rect file is a
  one-knob variation of it.
* `hex_grids/01_flat_top.c` is the root for hex; `02_pointy_top` rotates
  the matrix; `03_axial`, `04_ring_distance` reuse the cursor;
  `05_triangular` introduces the dual coord system; `06`, `07` are
  Archimedean tilings.
* `tri_grids/01_equilateral.c` introduces the three-family stripe
  classifier; `02..06` swap out the lattice; `07..12` move from regular
  tilings to substitution / aperiodic / random tilings.
* `polar_grids/01_rings_spokes.c` is the polar root; `02..07` change the
  radial law (linear → log → spiral → sunflower → equal-area → elliptic).

For placement files, **always read `01_direct.c` of the matching family
first.** It defines the GridCtx and Pool that the other three modes reuse
verbatim.

---

## The two primitives

Every file in `grids/` is built from exactly two structs and a small handful
of operations on them. Master these and the 76 files become 76 small
variations on the same theme.

### 1. `GridCtx` — grid ↔ screen

`GridCtx` answers the question **"where on the screen does grid cell
(r, c) live?"** — the only place in the codebase that knows. Every other
function takes positions in grid space (`(r, c)` integers, sometimes with
a sub-cell tag like `up`, `wedge`, `sector`, or `Q,R,S`) and routes its
screen-side work through `GridCtx`.

The canonical struct (from `rect_grids_placement/01_direct.c:264`):

```c
typedef struct {
    GridMode mode;                          /* which grid is active */
    int rows, cols;                         /* terminal size in cells */
    int cw, ch;                             /* cell width, cell height */
    int ox, oy;                             /* screen origin */
    int range;                              /* ±range for centred grids */
    int min_r, max_r, min_c, max_c;         /* cursor bounds */
} GridCtx;
```

It exposes three operations:

```
ctx_init   (g, mode, rows, cols)        — set cell size + cursor bounds
                                          for the chosen grid
ctx_to_screen (g, r, c) -> (sr, sc)     — forward map: address → screen
ctx_draw_bg   (g)                       — paint the background lines
```

**Forward map (rect uniform):**

```
sr = r * ch        sc = c * cw
```

**Forward map (hex flat-top):**

```
sx_px = size * (3/2 * Q)
sy_px = size * (sqrt(3)/2 * Q + sqrt(3) * R)
sr = ROUND(sy_px / CELL_H)         sc = ROUND(sx_px / CELL_W)
```

**Forward map (polar ring/spoke):**

```
r_px = ring_index * RING_SPACING
theta = spoke_index * (2*PI / N_SPOKES)
sx_px = r_px * cos(theta)          sy_px = r_px * sin(theta)
sr = oy + ROUND(sy_px / CELL_H)    sc = ox + ROUND(sx_px / CELL_W)
```

**Inverse map** (screen pixel → grid address) is sometimes needed too — for
hit-testing, hover effects, drawing the grid by sweeping every screen cell.
The inverse is the *harder* of the two; this is where `cube_round`,
stripe classifiers, log-polar inverses, etc. live. Each grid family has its
own inverse and they are the principal subject of the drawing files.

Drawing the grid background is a per-cell sweep:

```
for sr in 0 .. rows-1:
    for sc in 0 .. cols-1:
        (r, c) = inverse(sr, sc)
        if pixel is on a grid line:
            draw line character with appropriate orientation
        else:
            skip
```

This is O(rows × cols) per frame — invisible at 80×24 even on slow
terminals because terminals are tiny compared to GPUs.

### 2. `Pool` — placement-mode-agnostic storage

`Pool` answers **"what objects exist, and where do they live in grid
space?"** It is identical across all 16 placement files in the rect / hex /
polar families (and across all 24 in the tri family). It is the single
unifying piece of state in placement code.

```c
typedef struct {
    int  r, c;                      /* grid-space address */
    char glyph;                     /* which character to render */
    bool alive;                     /* tombstone for removal */
} Obj;

typedef struct {
    Obj items[MAX_OBJ];             /* flat array, no allocation */
    int count;                      /* number currently in use */
} Pool;
```

Five operations, all O(n) on `count`:

```
pool_place  (p, r, c, glyph)        — insert; ignore if (r,c) already occupied
pool_remove (p, r, c)               — swap-with-last delete
pool_toggle (p, r, c, glyph)        — place if absent, remove if present
pool_clear  (p)                     — set count to 0
pool_find   (p, r, c) -> index|-1   — linear scan for membership
```

Why O(n) instead of a hash map? Because `MAX_OBJ` is a few hundred and the
hash-map prelude (insertion bookkeeping, hash function, collision policy)
buys nothing at this scale. Three nested calls fit on one screen of code.

Rendering is one pass — for each alive object, ask `GridCtx` where it goes
and `mvaddch` it:

```
for each obj in pool:
    if obj.alive:
        (sr, sc) = ctx_to_screen(g, obj.r, obj.c)
        mvaddch(sr, sc, obj.glyph)
```

### Why this split is universal

The split — `GridCtx` for geometry, `Pool` for state — is the same across
rect / hex / polar **because the placement abstraction does not know what
the grid looks like.** It knows only:

* a cursor lives at integer `(r, c)` (or `(Q, R)`, or `(si, sj)`, etc.);
* `cursor_move` clamps to `min_r..max_r`, `min_c..max_c`;
* `pool_place` records an address;
* `ctx_to_screen` can render an address as a screen pixel.

Substitute axial coordinates, stripe coordinates, ring/spoke coordinates,
or wedge coordinates and the four operations above keep working. **The grid
family changes the address type and the inverse formula. Everything else
stays still.** That invariance is what makes grids re-usable across every
demo in the project.

---

## Drawing the grid — four families

| Family  | Files | Address type         | Inverse trick                |
|---------|-------|----------------------|------------------------------|
| `rect`  | 14    | `(int r, int c)`     | `r = sr/CH`, `c = sc/CW`     |
| `hex`   | 7     | `(int Q, int R)` axial; cube `S = -Q-R` derived | inverse 2×2 matrix + `cube_round` |
| `tri`   | 12    | varies per file (skew lattice, axis-aligned, double-diagonal, hex-axial-with-sector) | three-family stripe classifier or barycentric |
| `polar` | 7     | `(int ring, int spoke)` or continuous `(r, θ)` | `r = sqrt(dx²+dy²)`, `θ = atan2(dy,dx)`, then per-radial-law step |

### `rect_grids/` — 14 files — modify-one-thing pedagogy

`rect_grids/` is the **ladder of variations** on the cartesian grid. Every
file modifies exactly **one knob** of `01_uniform_rect.c`:

| #   | File                       | What it changes vs. previous |
|-----|----------------------------|------------------------------|
| 01  | `01_uniform_rect.c`        | base formula: `sr = r*CH`, `sc = c*CW` |
| 02  | `02_square.c`              | aspect correction (`CW = 2*CH`) so cells *look* square |
| 03  | `03_fine_dense.c`          | shrink `CW`/`CH` → many small cells |
| 04  | `04_coarse_sparse.c`       | grow `CW`/`CH` → few large cells, with labels |
| 05  | `05_hierarchical.c`        | two nested grids (minor cells inside major) |
| 06  | `06_brick_stagger.c`       | offset every other row by `CW/2` |
| 07  | `07_half_brick_vert.c`     | offset every other column by `CH/2` |
| 08  | `08_diamond.c`             | rotate the lattice 45° |
| 09  | `09_isometric.c`           | replace squares with parallelograms (2:1 isometric) |
| 10  | `10_crosshatch.c`          | overlay two diagonal grids |
| 11  | `11_checkerboard.c`        | colour cells in a 2-colouring (no line drawing) |
| 12  | `12_ruled.c`               | drop vertical lines entirely (1-D grid in Y) |
| 13  | `13_dot.c`                 | drop lines, keep only intersection points |
| 14  | `14_origin.c`              | Cartesian quadrants with labelled axes |

The pedagogy: **read 01 in full, then read each subsequent file by diffing
its `§4 formula` and `§6 scene` sections against 01.** This is by design —
no other directory in the project follows this strict modify-one-knob
discipline so consistently.

### `hex_grids/` — 7 files — cube coordinates and Archimedean tilings

Hex grids do not admit a clean 2-D row/column index because the row stride
depends on parity. The standard fix is **cube coordinates**: embed the hex
plane in 3-D as the slice `Q + R + S = 0`. Now every hex has a unique
`(Q, R, S)` and the formulas for distance, rounding, neighbours all become
clean.

The full cube-rounding pipeline (the centrepiece of `01_flat_top.c`):

```
                 pixel (px, py)
                       │
         flat-top inverse 2x2 matrix
                       ▼
              fractional cube (fq, fr, fs = -fq - fr)
                       │
              cube_round (fix-largest-error)
                       ▼
              integer hex (Q, R, S = -Q - R)
                       │
        cube_dist = max(|fq-Q|, |fr-R|, |fs-S|)
                       ▼
                border / interior
```

| #   | File                       | What it adds                  |
|-----|----------------------------|-------------------------------|
| 01  | `01_flat_top.c`            | base hex with cube rounding + cursor |
| 02  | `02_pointy_top.c`          | same code, rotated inverse matrix |
| 03  | `03_axial.c`               | label every hex with its `(Q, R)` |
| 04  | `04_ring_distance.c`       | colour every hex by ring distance from cursor |
| 05  | `05_triangular.c`          | the *dual* tiling — three-family stripe classifier |
| 06  | `06_rhombille.c`           | hex + 3 spokes per hex (isometric-cube illusion) |
| 07  | `07_trihexagonal.c`        | hex grid + triangular grid overlaid (3.6.3.6) |

`05_triangular.c` is in `hex_grids/` because it is the dual of the hex
tiling, but its coordinate system is *different* — see
[`tri_grids/`](#tri_grids--12-files--three-family-stripes-and-substitution).

### `tri_grids/` — 12 files — three-family stripes and substitution

Triangular tilings are the richest family because the lattice does not have
the regular `(Q, R)` structure of hex. Different tri tilings need
**different coordinate systems**, and that is the entire reason
[`tri_grids_placement/` has 24 files instead of 4](#why-tri_grids_placement-has-24-files).

The first six files are all *regular* tilings — every cell is the same
shape — but with five distinct coord systems among them:

| #   | File                       | Coord system                                  |
|-----|----------------------------|-----------------------------------------------|
| 01  | `01_equilateral.c`         | skew-lattice `(col, row, up ∈ {0,1})`         |
| 02  | `02_right_isosceles.c`     | axis-aligned `(col, row, up ∈ {LL, UR})`      |
| 03  | `03_double_diagonal.c`     | `(col, row, wedge ∈ {N,E,S,W})` — 4 wedges per square |
| 04  | `04_30_60_90.c`            | reuses 01's coords (medians overlay)          |
| 05  | `05_isometric.c`           | reuses 01's coords (solid-fill render)        |
| 06  | `06_hex_subdivision.c`     | hex-axial `(Q, R, sector ∈ 0..5)`             |

Files 07–12 leave the regular-tiling world entirely:

| #   | File                       | What's new                                    |
|-----|----------------------------|-----------------------------------------------|
| 07  | `07_barycentric.c`         | recursive barycentric subdivision             |
| 08  | `08_triforce.c`            | recursive 4-way midpoint subdivision          |
| 09  | `09_sierpinski.c`          | Sierpinski triangle (3-way midpoint)          |
| 10  | `10_pinwheel.c`            | 5-way substitution of a 1:2:√5 triangle       |
| 11  | `11_delaunay.c`            | Delaunay triangulation of random points       |
| 12  | `12_penrose.c`             | Penrose-style aperiodic substitution          |

Files 07–12 build a **mesh** (lists of vertices + triangles) instead of
sweeping every screen cell — the lattice is no longer regular enough for a
per-pixel inverse formula.

### `polar_grids/` — 7 files — radial coordinate systems

Polar grids are organised around a centre. The forward map is
`(r, θ) → (sx, sy) = (r·cos θ, r·sin θ)`. Variations come from how `r`
spaces its rings and what curves replace the radial spokes.

| #   | File                       | Radial law                                    |
|-----|----------------------------|-----------------------------------------------|
| 01  | `01_rings_spokes.c`        | `r_k = k * RING_SPACING` (linear)             |
| 02  | `02_log_polar.c`           | `r_k = R_MIN * RATIO^k` (geometric)           |
| 03  | `03_archimedean_spiral.c`  | `r = a + b·θ` (constant pitch)                |
| 04  | `04_log_spiral.c`          | `r = a · e^(b·θ)` (equiangular)               |
| 05  | `05_sunflower.c`           | Vogel `(r_n, θ_n) = (sqrt(n), n·137.5°)`      |
| 06  | `06_sector.c`              | `r_k = sqrt(k) * c` (equal-area rings)        |
| 07  | `07_elliptic.c`            | confocal ellipses (axis-scaled radius)        |

`02_log_polar.c` is the coordinate system of conformal optics, SIFT
descriptors, and the human fovea; `05_sunflower.c` is phyllotaxis;
`07_elliptic.c` is the equipotential grid of an elliptic conducting
cylinder. Each file lists 2–5 references in its CONCEPTS block.

---

## Placing objects — four modes

Every placement folder contains the same four files (the tri folder
multiplies by 6, see below). The pattern is identical across grid
families; only the underlying `ctx_to_screen` changes.

```
                ┌─────────────────────────────────────────┐
   USER  ───►   │  cursor_move(g, dr, dc)                 │
                │      ▼                                  │
                │  pool_place / pool_toggle (r, c, glyph) │
                │      ▼                                  │
                │  pool_draw(g)  ── uses ctx_to_screen    │
                └─────────────────────────────────────────┘
                                   │
                                   ▼
                            scene_draw(g)
                                   │
                                   ▼
                            ncurses screen
```

### Mode 1 — direct (cursor + toggle)

A single cursor moves with arrows; SPACE toggles an object at the cursor
cell.

```
loop forever:
    key = read_input()
    case key:
        arrows: cursor_move(g, dr, dc)
        SPACE:  pool_toggle(p, cursor.r, cursor.c, 'O')
        C:      pool_clear(p)
    scene_draw(g, cursor, pool)
```

Direct mode is the **identity** placement: nothing computed about the
arrangement, just one cell at a time. It's the trivial case of the other
three modes.

### Mode 2 — patterns (parametric stamps)

Press a key, stamp a pattern centred on the cursor. Key insight: a pattern
is **a predicate over offsets**, not a bitmap.

```
border (dr, dc, N)   :=   |dr| == N  ||  |dc| == N
fill   (dr, dc, N)   :=   |dr| <= N  &&  |dc| <= N
hollow (dr, dc, N)   :=   fill(N)    && !fill(N-1)
row    (dr, dc, N)   :=   dr == 0    &&  |dc| <= N
col    (dr, dc, N)   :=   dc == 0    &&  |dr| <= N
```

Stamping iterates a bounding box and applies the predicate:

```
for dr in -N..N:
    for dc in -N..N:
        if predicate(dr, dc, N):
            pool_place(p, cursor.r + dr, cursor.c + dc, 'O')
```

Three things follow from the predicate representation:

1. **Patterns scale** with a single integer `N` — no per-size bitmap.
2. **Patterns compose** — `hollow = fill(N) ∧ ¬fill(N-1)`.
3. **Patterns are grid-agnostic** — the same `border(dr,dc,N)` works on
   rect, hex, tri, polar because the offsets are in *address space*, not
   screen space.

### Mode 3 — path (two-point traversals)

A small state machine: ENTER once to set point A, ENTER again to set point
B, then a key chooses the path algorithm.

```
state := IDLE
loop:
    key = read_input()
    if state == IDLE       and key == ENTER:  A := cursor; state := ONE
    if state == ONE        and key == ENTER:  B := cursor; state := TWO
    if state == TWO        and key in {L,P,O,X}:
        cells := path_algo(A, B)              # Bresenham / L-path / ring / staircase
        for (r,c) in cells: pool_place(p, r, c, 'O')
        state := IDLE
```

Algorithms:

* **L (Bresenham line)** — closest integer approximation of the straight
  segment, error-accumulation per step. (Bresenham 1965.)
* **P (L-path)** — move all the way in one axis, then the other.
  Two variants exist (h-first vs v-first); pick the shorter Manhattan
  distance.
* **O (ring)** — axis-aligned rectangle border with corners A, B.
* **X (diagonal staircase)** — alternate row and column steps until target
  is reached.

### Mode 4 — scatter (procedural sampling)

All four scatter algorithms are **sampling strategies** — different
distributions over the grid:

| Key | Algorithm    | Distribution                           |
|-----|--------------|----------------------------------------|
| R   | random       | uniform over valid cells               |
| M   | min-distance | uniform with rejection (Poisson-disk)  |
| F   | BFS flood    | breadth-first from cursor              |
| G   | gradient     | denser near grid centre                |

Min-distance is a simplified Bridson Poisson-disk — O(n²) per attempt
without spatial hashing, but correct for `n ≤ 256`. BFS uses a small ring
buffer sized to the max grid area — no allocation in the hot path.

```
# Min-distance (Poisson-ish)
for attempt in 1..MAX_TRIES:
    candidate := random cell
    if all (r,c) in pool have chebyshev_dist(candidate, (r,c)) >= MIN_DIST:
        pool_place(p, candidate.r, candidate.c, 'O')
```

---

## Why `tri_grids_placement/` has 24 files

Every other placement folder has exactly **4 files** (one per mode) that
internally `switch` on grid-type. `tri_grids_placement/` has **24 files**
(6 grid types × 4 modes), and that asymmetry is **intentional**.

The reason is that triangular grids do not share a single cell-address
system. Compare the cursor type across the six tri drawing files:

| Tri grid        | Cell address          | Pixel→cell inverse                                              |
|-----------------|-----------------------|------------------------------------------------------------------|
| equilateral     | `(col, row, up∈{0,1})`         | skew lattice (60° basis), check which sub-triangle      |
| right-isosceles | `(col, row, up∈{LL,UR})`       | axis-aligned, diagonal test `fa ≥ fb`                   |
| double-diagonal | `(col, row, wedge∈{N,E,S,W})`  | both diagonals split each square, 4 wedges              |
| 30-60-90        | reuses equilateral coords      | medians overlaid on 01's lattice                        |
| isometric       | reuses equilateral coords      | render only — solid colour blocks                       |
| hex-subdivision | `(Q, R, sector∈0..5)`          | cube-rounding **then** wedge angle inside the hex       |

A unified placement file would need a tagged-union cursor type, a
function-pointer table for `cursor_move` and `ctx_to_screen`, and runtime
dispatch on every pool operation. The complexity cost is real and would
hide the per-tri-type *math* — which is the whole point of the file.

By contrast, `rect_grids_placement/` can unify 14 grid types in one file
because every rect grid uses the same `(int r, int c)` address: only the
forward `ctx_to_screen` differs, dispatched by a single switch on
`mode`. `hex_grids_placement/` covers only flat-top, so there is no
multiplexing to do. `polar_grids_placement/` has 7 grid types but they all
share `(int ring, int spoke)` addressing and per-mode arrow-key dispatch.

**The general rule:** *the placement file consolidates over grid types
that share an address system*. Tri files don't, so they fan out.

---

## Building and running

Every file is self-contained — no shared headers, single `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra <path>.c -o <name> -lncurses -lm
```

Drop `-lm` for the few cell-space sims that don't use `<math.h>`. The
project is strict about `-Wall -Wextra` clean — every file in this folder
compiles with zero warnings.

**Universal keys** (always present):

| Key             | Action                  |
|-----------------|-------------------------|
| `q` / `ESC`     | quit                    |
| `p`             | pause                   |
| `r`             | reset                   |
| `t` / `T`       | next / previous theme   |
| arrows          | move cursor (if any)    |

**Placement keys** (in placement files only):

| Key         | Action                                       |
|-------------|----------------------------------------------|
| `space`     | toggle object at cursor (direct mode)        |
| `B/F/H/R/V` | stamp pattern (patterns mode)                |
| `+` / `-`   | grow / shrink pattern radius                 |
| `C`         | clear all objects                            |
| `a` / `e`   | switch to previous / next grid type (rect, polar) |

---

## Adding a new grid

If you want to add (say) a Voronoi grid or a custom Penrose variant:

1. **Pick a coordinate system.** Cell-space (`int r, int c`) if every cell
   IS one terminal character. Pixel-space (`float px, py` in sub-pixel
   units) if you need continuous motion.
2. **Choose the inverse.** The hard problem is `pixel → address`. If you
   can write that as one closed-form formula, you have a regular tiling
   (rect, hex, equilateral, polar). If not, build a mesh (tri 07–12).
3. **Copy the `01` of the closest existing family** as a template:
   * rect-like → copy `rect_grids/01_uniform_rect.c`
   * hex-like → copy `hex_grids/01_flat_top.c`
   * tri-like → copy `tri_grids/01_equilateral.c`
   * polar-like → copy `polar_grids/01_rings_spokes.c`
4. **Replace `§4 formula` and `§5 draw`.** Everything else (clock, color,
   scene, screen, app, signals) carries over unchanged.
5. **Add CONCEPTS + MENTAL MODEL** per the project's
   [CLAUDE.md](../CLAUDE.md) template. Both blocks are mandatory.
6. **Verify**: `gcc -Wall -Wextra` clean, stable 60 fps, `q`/`ESC` exits
   cleanly, `SIGWINCH` doesn't crash, HUD shows fps + state.

Adding a placement mode for the new grid? Copy
`rect_grids_placement/01_direct.c` (or the matching family's), keep `Pool`
identical, swap in your new `GridCtx`. The four modes — direct, patterns,
path, scatter — should all carry across with only the address-space
constants changing.

---

## Quick-reference file index

### Drawing — `rect_grids/`

| File                          | Description                                       |
|-------------------------------|---------------------------------------------------|
| `01_uniform_rect.c`           | standard rectangular grid, the base formula       |
| `02_square.c`                 | rectangular grid with visually square cells       |
| `03_fine_dense.c`             | tight small-cell rectangular grid                 |
| `04_coarse_sparse.c`          | large-cell rectangular grid with coordinate labels |
| `05_hierarchical.c`           | two-level grid (minor cells inside major cells)   |
| `06_brick_stagger.c`          | horizontal brick (staggered row) grid             |
| `07_half_brick_vert.c`        | vertical brick (staggered column) grid            |
| `08_diamond.c`                | 45°-rotated rectangular grid (diamond / iso cells) |
| `09_isometric.c`              | flat-parallelogram isometric grid                 |
| `10_crosshatch.c`             | two diagonal grids overlaid (cross-hatch pattern) |
| `11_checkerboard.c`           | checkerboard pattern (alternating cell fill)      |
| `12_ruled.c`                  | ruled lines only (horizontal stripes, no vertical) |
| `13_dot.c`                    | dot grid (intersection points only, no lines)     |
| `14_origin.c`                 | coordinate-system grid with labelled axes         |

### Drawing — `hex_grids/`

| File                          | Description                                       |
|-------------------------------|---------------------------------------------------|
| `01_flat_top.c`               | flat-top hexagonal grid with keyboard cursor      |
| `02_pointy_top.c`             | pointy-top hexagonal grid with keyboard cursor    |
| `03_axial.c`                  | flat-top hex grid with axial labels               |
| `04_ring_distance.c`          | hex ring-distance coloring with movable cursor    |
| `05_triangular.c`             | triangular grid with stripe-index cursor          |
| `06_rhombille.c`              | rhombille tiling: hexagons divided into 3 rhombuses |
| `07_trihexagonal.c`           | trihexagonal tiling: hex + triangular overlaid    |

### Drawing — `tri_grids/`

| File                          | Description                                       |
|-------------------------------|---------------------------------------------------|
| `01_equilateral.c`            | equilateral triangular grid, the base formula     |
| `02_right_isosceles.c`        | square grid bisected by one diagonal              |
| `03_double_diagonal.c`        | tetrakis square tiling (4 triangles per cell)     |
| `04_30_60_90.c`               | kisrhombille (equilaterals subdivided into 6 right tris) |
| `05_isometric.c`              | equilateral grid rendered as solid colored blocks |
| `06_hex_subdivision.c`        | flat-top hexagons split into 6 equilateral tris   |
| `07_barycentric.c`            | recursive barycentric subdivision of one triangle |
| `08_triforce.c`               | recursive midpoint (4-way) subdivision            |
| `09_sierpinski.c`             | Sierpinski triangle (3-way midpoint subdivision)  |
| `10_pinwheel.c`               | pinwheel-inspired 5-way substitution              |
| `11_delaunay.c`               | Delaunay triangulation of random points (Bowyer-Watson) |
| `12_penrose.c`                | Penrose-style aperiodic substitution              |

### Drawing — `polar_grids/`

| File                          | Description                                       |
|-------------------------------|---------------------------------------------------|
| `01_rings_spokes.c`           | standard polar grid: concentric rings + spokes    |
| `02_log_polar.c`              | logarithmic polar grid: exponentially-spaced rings |
| `03_archimedean_spiral.c`     | Archimedean spiral grid (constant pitch)          |
| `04_log_spiral.c`             | logarithmic (equiangular) spiral grid             |
| `05_sunflower.c`              | phyllotaxis sunflower pattern (Vogel model)       |
| `06_sector.c`                 | equal-area polar sector grid                      |
| `07_elliptic.c`               | elliptic polar grid (confocal ellipses + hyperbolae) |

### Placement — `rect_grids_placement/` (universal: switches over 14 grid types)

| File              | Mode      | What it shows                              |
|-------------------|-----------|--------------------------------------------|
| `01_direct.c`     | direct    | cursor + toggle on all 14 grids            |
| `02_patterns.c`   | patterns  | predicate-based stamps (border/fill/hollow/row/col) |
| `03_path.c`       | path      | Bresenham line, L-path, ring, staircase    |
| `04_scatter.c`    | scatter   | random / Poisson-disk / BFS flood / gradient |

### Placement — `hex_grids_placement/` (flat-top hex only)

| File                | Mode      | What it shows                              |
|---------------------|-----------|--------------------------------------------|
| `01_hex_direct.c`   | direct    | axial-cursor toggle on flat-top hex        |
| `02_hex_pattern.c`  | patterns  | disc / ring / row / col stamps             |
| `03_hex_path.c`     | path      | hex line, ring walk, L-path                |
| `04_hex_scatter.c`  | scatter   | four scatter strategies                    |

### Placement — `polar_grids_placement/` (per-mode arrow dispatch)

| File                  | Mode      | What it shows                              |
|-----------------------|-----------|--------------------------------------------|
| `01_polar_direct.c`   | direct    | cursor on 7 polar grid backgrounds         |
| `02_polar_arc.c`      | path      | two-anchor arc and spoke drawing           |
| `03_polar_spiral.c`   | patterns  | parametric spiral path placement           |
| `04_polar_scatter.c`  | scatter   | four polar scatter strategies              |

### Placement — `tri_grids_placement/` (per tri-type × per mode)

The 24 files form a 6 × 4 matrix. Each row is one tri grid type; each
column is one placement mode.

|                       | direct                              | patterns                              | path                              | scatter                              |
|-----------------------|-------------------------------------|---------------------------------------|-----------------------------------|--------------------------------------|
| `01_equilateral_*`    | direct placement                    | preset pattern stamps                 | line-of-sight path                | distance-coloured scatter            |
| `02_right_isosceles_*`| direct placement                    | preset pattern stamps                 | line-of-sight path                | distance-coloured scatter            |
| `03_double_diagonal_*`| direct placement (tetrakis)         | preset stamps (tetrakis)              | line-of-sight path (tetrakis)     | distance-coloured scatter (tetrakis) |
| `04_30_60_90_*`       | direct on kisrhombille              | preset stamps                         | line-of-sight path                | distance-coloured scatter            |
| `05_isometric_*`      | direct on iso (solid-fill)          | preset stamps                         | line-of-sight path                | distance-coloured scatter            |
| `06_hex_subdivision_*`| direct on hex-with-radii            | preset stamps                         | line-of-sight path                | distance-coloured scatter            |

See [Why `tri_grids_placement/` has 24 files](#why-tri_grids_placement-has-24-files) for the rationale.
