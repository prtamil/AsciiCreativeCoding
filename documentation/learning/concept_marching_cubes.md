# Concept: Marching Cubes — Animated Metaballs

## Pass 1 — Understanding

### Core Idea
The Mandelbulb is implicit; metaballs are implicit; any scalar field defined by `f(x, y, z) = T` has no triangles. Marching cubes (Lorensen & Cline, 1987) MAKES triangles: walk a regular voxel grid, classify each cube's 8 corners as inside/outside the level set, and look up — from a precomputed 256-case table — which of the 12 cube edges to interpolate and how to stitch them into triangles. After that the rasteriser is the same as any other.

### Mental Model
Don't try to mesh a curved surface directly. Slice space into tiny cubes and ASK each one: "of my eight corners, which are inside the shape?" There are 2^8 = 256 possible answers. For every one we precomputed where to draw the triangles. Visit every cube, look up its case, emit triangles. The continuous surface emerges from a discrete lookup.

### Key Equations
```
Metaball field    : f(p) = Σ s_i / (|p − c_i|² + ε)     // soft-blob
Iso-surface       : { p | f(p) = THRESHOLD }
Cube case         : 8-bit index, bit n set iff f(corner n) > THRESHOLD
Edge interpolate  : t = (T − f_a) / (f_b − f_a)
                    p = lerp(p_a, p_b, t)
Outward normal    : N = normalize( Σ s_i · (p − c_i) / (|p − c_i|² + ε)² )
Colour blend      : c = Σ w_i · color_i / Σ w_i,  w_i = s_i / (|p − c_i|² + ε)
```

### Data Structures
- **Field grid** — `g_field[GRID+1][GRID+1][GRID+1]` linear scalar field at every cube corner, recomputed each frame from the moving metaballs (≈ 16k corner evals / frame at default GRID=24).
- **Lookup tables** — `EDGE_TABLE[256]` (12-bit mask of crossed edges per case) and `TRI_TABLE[256][16]` (sequence of edge indices, terminated by −1). Both standard Bourke tables; startup verifier confirms they're mutually consistent.
- **Vertex pool** — `g_mc[MC_MAX_VERTS]` flat triangle storage. Cleared and refilled every frame; up to 5 triangles per cube, typically 1-3k tris on the surface.

### Non-Obvious Decisions
- **256-case table, not the 15-case minimal set**: the 256-entry table handles every corner pattern directly without runtime symmetry expansion. Trades data size (~4 KB) for code simplicity.
- **`mc_verify_tables()` at boot**: for each case, the OR of every edge mentioned in TRI_TABLE must equal EDGE_TABLE. Typo in the embedded data → boot fails loudly with the offending case index. Without this check a typo would silently corrupt topology and you'd be debugging mystery holes.
- **Per-vertex colour blend, not per-object**: each MC vertex evaluates `metaball_color` (influence-weighted palette blend) at the interpolated surface point. The rasteriser then barycentric-blends three vertex colours per triangle. The seam between two merging metaballs shows their two colours bleeding into each other — a hard per-object albedo would draw a sharp seam.
- **Gradient normals, not face normals**: the outward normal is the analytic gradient of the field at the interpolated vertex position. Smooth across the surface; bends correctly with the implicit shape. Face normals would show the cube grid as faceted ridges.
- **Theme rim lighting**: each of the 7 themes carries a `rim_strength`. The rim term `(1 − N·V)^2.5 · rim_strength` brightens silhouette edges so each blob reads as a clearly rounded shape against a dark ambient.

### Key Constants
| Name | Default | Effect |
|------|---------|--------|
| `GRID_DIM` | 24 | cubes per axis (smoother but slower at higher) |
| `WORLD_HALF` | 1.5 | volume spans `[−H, +H]` on each axis |
| `MC_THRESHOLD_DEF` | 1.0 | iso level; `t`/`g` raise / lower it |
| `N_METABALLS` | 4 | number of orbiting blobs |
| `BALL_STRENGTH` | 0.18 | per-ball field amplitude |
| `MC_MAX_VERTS` | 18000 | vertex pool cap (drops excess on overflow) |

### 7 Themes (`n` cycles)
PRIMARY / LAVA / PLASMA / MATRIX / OCEAN / SUNSET / NEON. Each bundles 4 ball colours + ambient + sun colour + rim strength. Lower ambient = stronger 3-D contrast; higher rim = brighter silhouette glow.

### Open Questions
- Why does marching tetrahedra (16-case table) avoid the topology ambiguities of marching cubes?
- What's the cost of computing the gradient analytically vs sampling the field at 4 nearby points?
- How does dual contouring resolve the cube ambiguity better?

## From the Source

**Algorithm:** Lorensen-Cline marching cubes with 256-case table. Sum-of-blobs scalar field for the metaballs. Per-frame mesh rebuild (no caching). Standard rasteriser with per-vertex colour barycentric blend.

**Physics/References:** Lorensen & Cline "Marching Cubes" SIGGRAPH '87 (the original); Paul Bourke "Polygonising a scalar field" (canonical 256-case tables); Inigo Quilez "Distance functions" + "Smin." The "smooth threshold via float bias" trick avoids the ambiguous-cube case.

**Math:** Hubbard-Douady-style level-set extraction. Outward normal is the negated gradient `−∇f / |∇f|` simplified to `+(p − c) / d⁴` weighted sum. Colour blend is influence-weighted convex combination.

**Performance:** O(GRID³) field evals per frame (≈ 16k at default), O(GRID³) cube classifications, O(active_cubes) triangle emissions. Surface ≈ GRID² active cubes (~600); each emits 1-4 triangles. ~2k triangles / frame on the default grid.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `Metaball` | `struct` | 32 B | pos, color, orbit_r, orbit_speed, orbit_y, phase |
| `g_field[K+1][J+1][I+1]` | `float[]` | ~62 KB | scalar field at corners |
| `EDGE_TABLE[256]` | `unsigned short` | 512 B | which edges are crossed per case |
| `TRI_TABLE[256][16]` | `int[]` | 16 KB | triangulation sequence per case |
| `MCVertex` | `struct` | 36 B | pos, normal, color (interp at hit) |
| `g_mc[MC_MAX_VERTS]` | `MCVertex[]` | ~648 KB | per-frame triangle pool |
| `Theme` | `struct` | ~64 B | name + 4 ball colours + ambient + sun + rim |
| `THEMES[7]` | `Theme[]` | ~448 B | preset palette table |

---

## Pass 2 — Implementation

### Pseudocode
```
init:
    ssao_init_kernel — n/a here
    mc_verify_tables → exit if EDGE_TABLE[c] != OR-of-tritable-edges[c]

per frame:
    1. update metaball positions (orbits)
    2. mc_extract:
        for k, j, i in [0, GRID):
            read 8 corner values from g_field
            cube_case = 8-bit bitset
            emask = EDGE_TABLE[cube_case]
            if emask == 0: continue          // entirely in or out

            edge_pos[12]  ← interpolate iso-crossings on the crossed edges
            for tri in TRI_TABLE[cube_case]:
                emit 3 vertices (pos, gradient_normal, palette_blend)

    3. render_gbuffer over the dynamic mesh
    4. render_lightpass: Blinn-Phong + rim per theme
    5. paint each cell

mc_edge_lerp(pa, fa, pb, fb, T):
    t = (T − fa) / (fb − fa)
    return lerp(pa, pb, clamp01(t))

metaball_outward_normal(p):
    return normalize( Σ s_i · (p − c_i) / (|p − c_i|² + ε)² )
```

### Module Map
```
§1 config       — frame, cam, GRID_DIM, MC_THRESHOLD, ramp, pairs
§2 clock        — monotonic timer + sleep
§3 math         — Vec3 / Mat4 + perspective / lookat
§4 paint        — 216 RGB cube + Bourke ramp + paint_cell
§5 metaballs    — Metaball struct, field, gradient, color blend
§6 marching cubes
   §6.1 cube corner / edge geometry (Bourke convention)
   §6.2 EDGE_TABLE (256 entries)
   §6.3 TRI_TABLE  (256 × 16 entries)
   §6.4 vertex pool MCVertex + g_mc[]
   §6.5 mc_extract — the 4-step algorithm
§7 gbuffer      — per-vertex colour rasteriser
§8 lightpass    — Blinn-Phong + emissive 0 + rim
§9 scene        — Scene struct, theme_apply, init, tick
§10 screen      — render_scene + HUD
§11 app         — signals, resize, main loop (verifies tables)
```

### Data Flow
```
metaballs → field eval → g_field
g_field → for-each cube → case index → EDGE_TABLE → TRI_TABLE → triangle pool g_mc
g_mc → render_gbuffer (per-vertex colour interp) → G-buffer
G-buffer → render_lightpass (Blinn-Phong + rim) → g_light
g_light → render_scene → paint_cell → ncurses
```
