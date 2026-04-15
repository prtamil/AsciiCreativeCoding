# Concept: Marching Squares

## Pass 1 — Understanding

### Core Idea
Given a scalar field f(x,y) — a function that assigns a number to every point in the plane — find and draw the contour line f(x,y) = threshold. Marching Squares does this by walking ("marching") across a grid of sample points, looking at each 2×2 cell of corners, classifying each corner as inside or outside the threshold, and looking up in a 16-entry table which edges the contour must cross through that cell.

### Mental Model
Imagine the scalar field as a height map. The threshold is the water level. "Inside" means above water, "outside" means below. For each square cell of the grid, look at its four corners — some are above water, some below. The shoreline (contour) must cross the edges between inside/outside corners. There are only 16 possible combinations of 4 on/off corners, so you precompute all 16 cases and look them up.

### Key Equations

**4-bit case index:**
```
case = (TL << 3) | (TR << 2) | (BR << 1) | (BL << 0)
```
where each bit is 1 if that corner's value exceeds the threshold, 0 otherwise.
- Bit 3 = Top-Left
- Bit 2 = Top-Right
- Bit 1 = Bottom-Right
- Bit 0 = Bottom-Left

**Linear edge interpolation** (find exact crossing point):
```
t = (threshold − f_a) / (f_b − f_a)
crossing = a + t · (b − a)
```
where a and b are two adjacent corners straddling the threshold.

**Metaball scalar field** (common test signal):
```
f(x,y) = Σᵢ  rᵢ² / ((x − xᵢ)² + (y − yᵢ)²)
```
Each ball i contributes 1.0 at its centre and falls off with distance squared. Setting threshold = 1.0 gives the classic "blobby" merged-sphere contour.

### The 16 Cases

Cases 0 and 15: no contour (all outside / all inside).  
Cases 1–14: the contour crosses 1 or 2 cell edges. Each lookup entry stores which pair(s) of edges are crossed.

Two ambiguous cases exist (case 5 and case 10 — checkerboard pattern) where two diagonally opposite corners are inside. Different disambiguation choices produce different topologies. The standard approach chooses the "saddle point" rule based on the centre value.

### Implementation-Specific Notes
- Grid sampled at every terminal cell; each cell covers 1 character
- Scalar field built from N animated metaballs moving under simple gravity + bounce
- Threshold = 1.0 (standard metaball isovalue)
- Only the edge crossing character is drawn; background transparent
- Characters chosen from `|`, `−`, `/`, `\`, `+`, `X`, `.` based on which edges cross and at what angle — gives a directional line appearance instead of a uniform dot
- Contour character can optionally use the interpolated position to pick a sub-cell character (half-step precision with `▌`, `▐`, `▀`, `▄`)

## From the Source

**Algorithm:** Marching Squares — the 2-D analogue of Marching Cubes (Lorensen & Cline, 1987). Classifies each 2×2 cell by a 4-bit index (inside/outside per corner), looks up which of the 16 possible edge-crossing patterns applies, then draws an ASCII character at each crossing position.

**Math:** Metaball potential field: f(x,y) = Σ A_i / r_i² where r_i = distance from point (x,y) to source i. This is the gravitational potential of multiple point masses. When f(x,y) = threshold, the iso-contour is the locus of equal potential — it encircles sources and merges blobs when sources are close enough (classic "organic" metaball look).

**Rendering:** Terminal cells are ~2× taller than wide (ASPECT=0.5). The field is sampled at (col × ASPECT, row) in world space to correct for this and produce circular blobs on screen.

**Performance:** O(W×H) per frame. The scalar field is re-evaluated at every grid corner each frame (N_BLOBS × W × H evaluations). No caching because blob positions change every frame. Multi-level mode draws 5 iso-contours with one pass over the same pre-evaluated corner values.

### Non-Obvious Design Decisions
- **Why terminal cells instead of sub-pixel grid?** Sub-cell grids (2×2 or 4×4 per character) give finer resolution but require more computation. Character-aligned grid is the simplest approach and fast enough for real-time.
- **Why linear interpolation?** It's the standard and gives C0-continuous contours (no kinks at cell boundaries). Cubic interpolation would give C1 (smooth normals) but isn't needed for ASCII output.
- **Why metaballs as the scalar field?** They produce smooth closed contours naturally. A single metaball gives a circle; two nearby balls merge into a peanut, then a blob — the same behavior as implicit surface modelling in 3D.

### Open Questions to Explore
1. Marching Squares is the 2D analogue of Marching Cubes (used for 3D isosurface extraction). What changes in 3D? (256 cases vs 16, triangles instead of line segments)
2. How would you handle the ambiguous cases (5 and 10) using the asymptotic decider?
3. What scalar field would produce a Mandelbrot set contour? (hint: escape time as field value)
4. How does Dual Contouring differ from Marching Squares? (it places vertices at QEF minimisers, not edge midpoints)
5. Can you use Marching Squares to detect zero-crossings of a wave PDE to find nodal lines (Chladni figures)?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_blobs[N_BLOBS]` | `Blob[4]` | ~80 B | metaball positions, velocities, strengths |
| `fld[256][128]` | `float[256][128]` (local in `app_draw`) | ~128 KB | corner field values sampled once per frame before marching |

---

## Pass 2 — Implementation

### Pseudocode

```
// Per frame:
update_balls()   — move metaballs under gravity, bounce walls

// Per cell (cx, cy):
sample corners: TL=f(cx,cy), TR=f(cx+1,cy), BR=f(cx+1,cy+1), BL=f(cx,cy+1)
case = (TL>T)<<3 | (TR>T)<<2 | (BR>T)<<1 | (BL>T)<<0
if case == 0 or case == 15: skip (no contour)
edges = lookup_table[case]   // list of edge pairs that the contour crosses
for each edge pair in edges:
    p1 = lerp(a1, b1, t1)   // interpolate position on edge 1
    p2 = lerp(a2, b2, t2)   // interpolate position on edge 2
    ch = direction_char(p1, p2)  // pick |/-\+ based on slope
    draw(cx, cy, ch)
```

### Module Map
```
§1 config      — grid resolution, threshold, N_BALLS, physics constants
§2 clock       — fixed-timestep loop
§3 field       — scalar field evaluation: f(x,y) = Σ r²/d²
§4 metaballs   — ball state (pos, vel, radius), gravity + bounce update
§5 march       — 16-entry lookup table, per-cell case classify + draw
§6 draw        — terminal rendering, colour by field value
§7 app         — input, preset switching, main loop
```

### Data Flow
```
balls[N] (pos, vel, r)
      │
      ▼
  update_physics()         ← gravity, wall bounce
      │
      ▼
  sample_field(cx, cy)     ← metaball sum at each grid corner
      │
      ▼
  classify_case()          ← 4-bit index
      │
      ▼
  lookup_table[case]       ← which edges cross
      │
      ▼
  lerp_crossing()          ← exact crossing position on each edge
      │
      ▼
  draw_char()              ← direction-matched ASCII glyph
```

### 16-Case Lookup Table (edge indices: 0=top, 1=right, 2=bottom, 3=left)
```
case 0:  {}           (none outside)
case 1:  {2,3}        (BL inside)
case 2:  {1,2}        (BR inside)
case 3:  {1,3}        (BR+BL inside)
case 4:  {0,1}        (TR inside)
case 5:  {0,3},{1,2}  (AMBIGUOUS: TR+BL)
case 6:  {0,2}        (TR+BR inside)
case 7:  {0,3}        (TR+BR+BL inside)
case 8:  {0,3}        (TL inside)
case 9:  {0,2}        (TL+BL inside)
case 10: {0,1},{2,3}  (AMBIGUOUS: TL+BR)
case 11: {0,1}        (TL+BR+BL inside)
case 12: {1,3}        (TL+TR inside)
case 13: {1,2}        (TL+TR+BL inside)
case 14: {2,3}        (TL+TR+BR inside)
case 15: {}           (all inside)
```
