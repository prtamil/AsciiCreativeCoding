/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * marching_cubes.c — animated metaballs + marching cubes isosurface
 *
 * DEMO: Four colored metaballs (red, green, blue, yellow) orbit at
 *       different speeds and heights inside a cube of empty space.
 *       Wherever their combined "density" exceeds a threshold, a
 *       smooth surface appears — and that surface is RE-EXTRACTED
 *       from scratch every frame. As the metaballs drift apart you
 *       see four separate blobs; as they pass close they MERGE into
 *       a single smooth shape, with their colours blending in the
 *       seam. The geometry itself is animated — most demos shade
 *       static meshes; this one rebuilds the mesh 60 times a second.
 *
 *       Watch the topology change: a hole opens between two blobs as
 *       they pull apart, then closes as they slide back together.
 *       Genus changes in real time. That's marching cubes — extract
 *       the level set f(p) = THRESHOLD by visiting each cube of a
 *       voxel grid and stitching together the triangles that
 *       approximate the surface inside that cube.
 *
 *       Camera orbits slowly so you see the surface from every side
 *       and confirm the gradient-derived normals shade correctly.
 *
 * Study alongside:
 *   raster/ssao_pipeline.c              — same G-buffer + raster path
 *   raster/deferred_rendering_pipeline.c — same MVP / lighting setup
 *
 * Section map:
 *   §1  config     — frame, view, MC grid, metaballs, lighting, ramp
 *   §2  clock      — monotonic timer + sleep
 *   §3  math       — V3, V4, Mat4 helpers
 *   §4  paint      — 216-pair RGB cube + Bourke ramp + paint_cell
 *   §5  metaballs  — Metaball type, scalar field eval, gradient, color
 *   §6  mc         — marching cubes
 *                    §6.1  cube corner / edge geometry
 *                    §6.2  EDGE_TABLE  — 256-case edge mask
 *                    §6.3  TRI_TABLE   — 256-case triangulation
 *                    §6.4  vertex pool — dynamic per-frame mesh
 *                    §6.5  mc_extract  — the algorithm
 *   §7  gbuffer    — geometry pass (per-vertex color barycentric blend)
 *   §8  lightpass  — Blinn-Phong, sun + ambient
 *   §9  scene      — Scene struct, init / view / tick
 *   §10 screen     — render_scene + HUD (CLAUDE.md spec)
 *   §11 app        — signals, resize, fixed-step main loop
 *
 * Keys:
 *   space     pause / resume metaball orbits + camera
 *   n / N     cycle colour theme (PRIMARY → LAVA → PLASMA → MATRIX
 *                                 → OCEAN  → SUNSET → NEON → wrap)
 *   + / =     zoom in
 *   - / _     zoom out
 *   t / T     raise threshold (smaller, tighter blobs)
 *   g / G     lower threshold (bigger, fatter blobs)
 *   r / R     reset
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/marching_cubes.c -o mc -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Marching cubes (Lorensen & Cline, 1987). Walk a
 *                  voxel grid; for each cube, classify its 8 corners
 *                  as inside/outside the level set; look up which
 *                  edges are crossed and how to triangulate them
 *                  from a 256-case table. Linear-interpolate the
 *                  crossings, emit triangles. The scalar field here
 *                  is a sum-of-blobs:
 *                      f(p) = Σ s_i / (|p − c_i|² + ε)
 *                  Each term peaks at a metaball centre and falls
 *                  off as 1/r²; the sum smoothly merges blobs whose
 *                  level sets intersect.
 *
 * Data-structure : 24×24×24 voxel grid → 25³ corner field samples.
 *                  Recomputed every frame from the moving metaballs.
 *                  Output is a flat triangle pool (no shared verts) of
 *                  up to MC_MAX_VERTS positions/normals/colours.
 *
 * Rendering      : Same G-buffer + Blinn-Phong path as ssao_pipeline,
 *                  but PER-VERTEX colour: each MC vertex blends the
 *                  metaball palette by influence weights at that
 *                  point, and the rasterizer barycentric-interpolates
 *                  the colour across each triangle. That's why the
 *                  seam between two merging blobs shows their two
 *                  colours bleeding into each other.
 *
 * Performance    : 24³ = 13 824 cubes × 8 corner reads per frame.
 *                  Most cubes are entirely inside or entirely outside
 *                  → case 0 or 255 → emit 0 triangles. Only surface-
 *                  adjacent cubes (≈ 24² ≈ 600 of them) emit any
 *                  geometry. Typical output: 1–3 k triangles/frame.
 *
 * References     : Lorensen & Cline, "Marching Cubes: A High
 *                    Resolution 3D Surface Construction Algorithm,"
 *                    SIGGRAPH '87.
 *                  Paul Bourke, "Polygonising a scalar field":
 *                    https://paulbourke.net/geometry/polygonise/
 *                  Inigo Quilez, "Distance functions" + "Smin":
 *                    https://iquilezles.org/articles/distfunctions/
 *                  Möller, "Fast Triangle Rasterization by
 *                    Interpolating Edge Functions," GPG (2000).
 *                  Reinhard et al., "Photographic Tone Reproduction
 *                    for Digital Images," SIGGRAPH '02.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Don't try to mesh a curved surface directly. Instead, slice space
 * into tiny cubes and ASK each one: "of my eight corners, which are
 * inside the shape?" There are 2^8 = 256 possible answers, and for
 * every one of them we precomputed where to draw the triangles.
 * Visit every cube, look up its case, emit triangles. The continuous
 * surface emerges from a discrete lookup.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a 3-D heightmap, but instead of "above sea level vs below"
 * the question is "above density threshold vs below". A cube whose
 * eight corners are all OUT contributes nothing. A cube whose eight
 * corners are all IN is buried inside the shape and contributes
 * nothing either. Only cubes with a MIX of in/out corners straddle
 * the surface — and for each such mix, the table tells you exactly
 * which edges to slice and how to weave triangles between them.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │         one cube, case 1 (corner 0 in)       │
 *      │                                              │
 *      │           7────────6                         │
 *      │          /│       /│                         │
 *      │         4─┼──────5 │                         │
 *      │         │ 3──────┼─2     ← three edges       │
 *      │         │/       │/         touch corner 0   │
 *      │         0────────1                           │
 *      │                                              │
 *      │   In corners: {0}      Edges crossed: 0,3,8  │
 *      │   Triangle:   (e0, e8, e3)                   │
 *      └──────────────────────────────────────────────┘
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 * (per frame)
 *
 *   1. Move metaballs along their orbits.
 *   2. For every corner (i, j, k) of the voxel grid:
 *        g_field[k][j][i] = Σ s_n / (|p − c_n|² + ε)
 *   3. For every cube (i, j, k) with i, j, k in [0, GRID-1]:
 *        a. Read the 8 corner values f[0..7].
 *        b. case = bitset where bit n is set iff f[n] > threshold.
 *        c. emask = EDGE_TABLE[case]  (which of the 12 edges cross)
 *        d. For each set bit e in emask:
 *             interpolate t = (T − f[a]) / (f[b] − f[a])
 *             vert_pos[e]    = lerp(p_a, p_b, t)        ← world pos
 *             vert_normal[e] = outward_gradient(vert_pos[e])
 *             vert_color[e]  = palette_blend(vert_pos[e])
 *        e. Walk TRI_TABLE[case] in groups of 3 until you hit −1:
 *             emit a triangle from vert_pos[group[0..2]],
 *                 with their normals and colours.
 *   4. Hand the triangle pool to the standard G-buffer + Blinn-Phong
 *      pipeline; barycentric-blend the per-vertex colours.
 *
 * KEY FORMULAS
 * ────────────
 *   Field:        f(p) = Σ s_i / (|p − c_i|² + ε)
 *   Cube case:    case = Σ (f[n] > T) << n      , n ∈ [0, 8)
 *   Edge interp:  t = (T − f[a]) / (f[b] − f[a]),  p = lerp(p_a, p_b, t)
 *   Outward N:    N = normalize( Σ s_i (p − c_i) / (|p − c_i|² + ε)² )
 *   Colour blend: c = Σ w_i c_i / Σ w_i,  w_i = s_i / (|p − c_i|² + ε)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Ambiguous cases — when the surface enters and leaves the same
 *     face the topology of TRI_TABLE matters; we use Bourke's table
 *     which picks one consistent resolution.
 *   • Threshold-touching corners — if f[n] equals T exactly, tiny
 *     float jitter can flip the case bit. The implicit ε in the
 *     metaball formula prevents the field from going singular at the
 *     centres but not on the iso-line itself; in practice it's fine.
 *   • Vertex pool overflow — we cap at MC_MAX_VERTS triangles. If a
 *     metaball sits perfectly at a corner the surface area can spike;
 *     we silently drop excess triangles.
 *   • Normals from gradient — at saddle points the gradient magnitude
 *     can dip near zero. v3_norm's safety fallback (return up) keeps
 *     the rasterizer from dividing by zero.
 *   • EDGE_TABLE / TRI_TABLE consistency — at startup we verify that
 *     the bitwise OR of every edge mentioned in TRI_TABLE for case c
 *     equals EDGE_TABLE[c]. A typo is fatal at boot, not silent at
 *     runtime.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Pause (space): the surface should be perfectly smooth — no
 *     visible cube grid. Smoothness comes from gradient normals
 *     evaluated at the actual interpolated vertex position, NOT from
 *     the cube corners.
 *   • Watch two metaballs approach: a thin "neck" forms first, then
 *     the topology changes (the gap closes) — you should see one
 *     fused blob with TWO colours visibly blending in the seam.
 *   • Press 't' to raise the threshold: blobs shrink and may split
 *     apart. Press 'g' to lower: they grow and merge faster. The HUD
 *     shows the live threshold value.
 *   • Triangle count in the HUD oscillates as the surface area grows
 *     and shrinks with merging — it should never be zero (the
 *     metaballs always meet the threshold somewhere) and never
 *     exceed MC_MAX_VERTS / 3.
 *   • If the program prints "MC table mismatch" at startup and
 *     exits, the embedded TRI_TABLE has a typo for the printed case;
 *     check that case against Bourke's reference table.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read raster/deferred_rendering_pipeline.c first if
 *      you don't yet know the G-buffer pattern; the rendering half
 *      of this file is the same. The NEW half is §6 mc.
 *   2. §1 config — every constant has a unit-bearing comment.
 *      Note especially MC_GRID (resolution), MC_DEFAULT_THRESHOLD,
 *      and MC_MAX_VERTS (triangle pool cap).
 *   3. §6 mc — THE HEART of this file. Read AFTER tutorials T1-T6.
 *      In order:
 *        §6.1 cube corner / edge geometry  ← labels
 *        §6.2 EDGE_TABLE  (256 entries)    ← which edges crossed
 *        §6.3 TRI_TABLE   (256 × ≤16)      ← how to triangulate
 *        §6.4 vertex pool                  ← per-frame allocator
 *        §6.5 mc_extract                   ← the algorithm
 *      Bourke's reference page has the same labelling — keep it open
 *      next to the file.
 *   4. §5 metaballs — the scalar field. Independent of MC; you could
 *      swap in any f: ℝ³ → ℝ and the extractor wouldn't change.
 *   5. §7 gbuffer + §8 lightpass — same as deferred_rendering.
 *   6. §3 math + §4 paint + §9-§11 — infrastructure; skip on first
 *      read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   GRID, GRID+1            cube count vs. corner count along each
 *                           axis. 24 cubes in x = 25 corners in x.
 *   g_field[k][j][i]        scalar value at corner (i, j, k).
 *   case_idx                8-bit cube index — bit n set iff
 *                           corner n is INSIDE the level set.
 *   emask                   12-bit mask — bit e set iff edge e
 *                           crosses the iso-surface in this case.
 *   t                       lerp parameter ∈ [0, 1] for the edge's
 *                           crossing position.
 *   T (or threshold)        the iso-value: f(p) = T defines the
 *                           surface.
 *   blob colour             palette colour of an individual metaball;
 *                           per-vertex colour is the blended average
 *                           weighted by metaball influence.
 *
 * Background you need
 * ───────────────────
 *   - The 7-stage forward rasteriser (cube_raster.c).
 *   - G-buffer / deferred shading (deferred_rendering_pipeline.c).
 *   - Linear interpolation: lerp(a, b, t) = a + t·(b − a).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Dual contouring or extended marching cubes. Plain MC has well-
 *     known ambiguity-case wobbles; we accept them at this scale.
 *   - SDF / level-set theory at depth — we just need "field value
 *     above threshold = inside" and "below = outside."
 *   - Octree spatial decomposition / sparse voxels. Our 24³ grid is
 *     small enough to scan exhaustively every frame.
 *   - GPU compute / parallel MC. The CPU version at 24³ is plenty
 *     fast for terminal output.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Eight tutorials that build marching cubes + animated metaballs
 * from first principles.
 *
 *   T1  The level-set problem — what's an isosurface?
 *   T2  Why CUBES instead of a smooth equation?
 *   T3  The 256 cases — how an 8-bit index encodes corner state
 *   T4  EDGE_TABLE and TRI_TABLE — the precomputed lookup
 *   T5  Linear edge interpolation — placing a vertex on a crossed edge
 *   T6  Gradient normals — smooth shading without per-vertex authoring
 *   T7  The metaball field — animated topology for free
 *   T8  Per-vertex colour blending — why merging blobs show their seam
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE LEVEL-SET PROBLEM — WHAT'S AN ISOSURFACE?
 * ─────────────────────────────────────────────────
 * Take any function f: ℝ³ → ℝ that returns a real number for every
 * 3-D point. Pick a threshold T. The set of points where f(p) = T
 * is a 2-D surface (almost everywhere) called the LEVEL SET or
 * ISOSURFACE of f at T.
 *
 *   f(p) = x² + y² + z² − 1   →   level T = 0 is a unit sphere.
 *   f(p) = density of clouds  →   level T = 0.5 is the cloud's
 *                                 visible boundary.
 *   f(p) = signed distance    →   level T = 0 is the surface itself.
 *
 * The CHALLENGE: there's no closed-form list of triangles for an
 * arbitrary level set. Even for the unit sphere, the formula
 * doesn't TELL you where to put the triangles — only which points
 * are on the sphere.
 *
 * Marching cubes solves this for ANY scalar field: it produces a
 * triangle approximation of f(p) = T from a discrete grid of
 * sample values, no analytic surface-finding required.
 *
 * T2  WHY CUBES INSTEAD OF A SMOOTH EQUATION?
 * ───────────────────────────────────────────
 * The trick: don't try to mesh the surface DIRECTLY. Instead,
 * partition all of space into tiny cubes (a regular voxel grid).
 * Inside each cube, ask a much simpler question:
 *
 *     "Of my 8 corners, which are INSIDE the surface
 *      (f(corner) > T) and which are OUTSIDE?"
 *
 * Eight corners, each binary → 2⁸ = 256 possible patterns. The
 * surface inside that one cube is approximated by triangles
 * connecting the EDGES that cross from inside-to-outside.
 *
 * For each of the 256 cases, ONE triangle layout is enough (with
 * a couple of ambiguity tweaks). Lorensen & Cline (1987) tabulated
 * all 256 — that's EDGE_TABLE and TRI_TABLE in §6.2 / §6.3.
 *
 * The whole continuous surface emerges from doing this lookup at
 * EVERY cube of the grid. No global meshing — every cube acts
 * independently and the triangles automatically meet at shared
 * edges because the interpolation is consistent.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │  surface threading through a cube grid:      │
 *      │                                              │
 *      │  ┌───┬───┬───┬───┐                           │
 *      │  │   │ ╱ │ ╲ │   │   each cube emits         │
 *      │  ├───┼─/─┼─\─┼───┤   triangles based on      │
 *      │  │ ╱ │   │   │ ╲ │   which corners are in    │
 *      │  ├─/─┼───┼───┼─\─┤                           │
 *      │  │   │   │   │   │                           │
 *      │  └───┴───┴───┴───┘                           │
 *      └──────────────────────────────────────────────┘
 *
 * T3  THE 256 CASES — HOW AN 8-BIT INDEX ENCODES CORNER STATE
 * ───────────────────────────────────────────────────────────
 * Number the cube's 8 corners 0..7 in a fixed labelling
 * convention. Bourke's labelling (which we use):
 *
 *     0: (0, 0, 0)        4: (0, 0, 1)
 *     1: (1, 0, 0)        5: (1, 0, 1)
 *     2: (1, 1, 0)        6: (1, 1, 1)
 *     3: (0, 1, 0)        7: (0, 1, 1)
 *
 * Build an 8-bit index by setting bit n if corner n is INSIDE:
 *
 *     case_idx = 0
 *     for n in 0..7:
 *       if f[n] > T: case_idx |= (1 << n)
 *
 * Examples:
 *     case 0   = 0000 0000   all 8 corners outside  → no triangles
 *     case 1   = 0000 0001   only corner 0 inside   → one triangle
 *     case 255 = 1111 1111   all 8 corners inside   → no triangles
 *     case 11  = 0000 1011   corners 0,1,3 inside   → small wedge
 *
 * 256 cases sounds large but most are uninteresting: cases 0 and
 * 255 emit nothing (cube is entirely out or entirely in). Most
 * surface-adjacent cubes hit the SAME small handful of cases
 * (cases 1, 2, 4, 8, 16, 32, 64, 128 are the "single corner
 * inside" cases — symmetric rotations of one another).
 *
 * Lorensen & Cline noted that of 256 cases, only 15 are unique
 * up to rotation/reflection. Some MC implementations exploit
 * that with smaller tables; we use the full 256-entry table for
 * clarity.
 *
 * T4  EDGE_TABLE AND TRI_TABLE — THE PRECOMPUTED LOOKUP
 * ─────────────────────────────────────────────────────
 * Two tables drive the algorithm:
 *
 *   EDGE_TABLE[256]
 *     12-bit value per case. Bit e set iff edge e is crossed by
 *     the surface in this case. A "crossed edge" has one corner
 *     inside and one outside, so the surface must pass through
 *     somewhere along that edge.
 *
 *   TRI_TABLE[256][16]
 *     For each case, a list of edge indices in groups of three,
 *     terminated by -1. Each triple says "make a triangle from
 *     the vertices placed on these three edges, in this winding
 *     order." Up to 5 triangles per case = 15 indices + sentinel.
 *
 * The algorithm becomes:
 *
 *     case = (8-bit corner-inside bitset)
 *     emask = EDGE_TABLE[case]
 *     for each set bit e in emask:
 *       compute vert_pos[e] by linear interpolation along edge e
 *     for each triangle indices in TRI_TABLE[case] until -1:
 *       emit triangle (vert_pos[i], vert_pos[j], vert_pos[k])
 *
 * Two table reads, one short loop per cube. The complexity is in
 * the OFFLINE construction of the tables; the online algorithm
 * is trivial.
 *
 * Sanity check (we do this at startup): EDGE_TABLE[c] must equal
 * the bitwise-OR of every edge mentioned in TRI_TABLE[c]. A typo
 * in either table produces a fatal "MC table mismatch" exit.
 *
 * T5  LINEAR EDGE INTERPOLATION — PLACING A VERTEX ON A CROSSED EDGE
 * ──────────────────────────────────────────────────────────────────
 * Suppose edge e connects corner a (value f[a]) and corner b
 * (value f[b]) with f[a] < T < f[b]. The surface passes through
 * SOMEWHERE along that edge. WHERE exactly?
 *
 * Approximation: assume f varies LINEARLY along the edge. Then
 * the surface crosses where the linear interpolant equals T:
 *
 *     f[a] + t · (f[b] − f[a]) = T
 *     t = (T − f[a]) / (f[b] − f[a])
 *     vert_pos = lerp(p_a, p_b, t)
 *
 * This is the "magic" smoothness step. Without it, you'd snap
 * vertices to the centre of every crossed edge (t = 0.5
 * always), and the resulting mesh would be cube-shaped — you
 * could see every voxel boundary.
 *
 * With t computed properly, vertices slide along their edges to
 * track the actual crossing position, and adjacent cubes' shared
 * edges produce IDENTICAL t-values (since both see the same
 * f[a] and f[b]). Result: triangles meet seamlessly at edges
 * and the cube grid is invisible in the output.
 *
 * Caveat: the interpolation is linear, not quadratic — fine for
 * smooth fields but CAN produce visible kinks where the field
 * curves sharply within a single cube. Increase MC_GRID to
 * shrink each cube and the kinks shrink too.
 *
 * T6  GRADIENT NORMALS — SMOOTH SHADING WITHOUT PER-VERTEX AUTHORING
 * ──────────────────────────────────────────────────────────────────
 * Each triangle vertex needs a normal for lighting. Two options:
 *
 *   PER-FACE  Compute one normal per triangle (cross-product of
 *             two edges). Cheap, but produces faceted shading —
 *             you'd see every triangle.
 *
 *   PER-VERTEX  Each vertex gets its own normal, the rasteriser
 *             interpolates across the triangle, lighting smooths
 *             over face boundaries. The result LOOKS curved.
 *
 * Per-vertex needs a normal at each vertex. For an isosurface
 * the answer is built in: the GRADIENT of the field f is
 * perpendicular to the surface at every point.
 *
 *     N(p) = ∇f / |∇f|
 *
 * For our metaball field f(p) = Σ s_i / (|p − c_i|² + ε), each
 * blob contributes a radial gradient outward from its centre,
 * weighted by the blob's strength. We have an analytic
 * expression — no need for finite differences:
 *
 *     ∇f = − Σ s_i · 2(p − c_i) / (|p − c_i|² + ε)²
 *
 * (negative sign because as p moves AWAY from c_i, f decreases).
 * Negate again to get the OUTWARD normal — points where f exceeds
 * T are "inside", so outward means in the direction f decreases.
 *
 * §5 metaball_gradient computes this; mc_extract evaluates it at
 * every emitted vertex.
 *
 * T7  THE METABALL FIELD — ANIMATED TOPOLOGY FOR FREE
 * ───────────────────────────────────────────────────
 * The scalar field is a sum of "metaballs" (Blinn 1982):
 *
 *     f(p) = Σ s_i / (|p − c_i|² + ε)
 *
 * Each term:
 *   - peaks at the centre c_i (≈ s_i / ε near c_i)
 *   - falls off as 1/r² with distance
 *   - blends ADDITIVELY with other metaballs
 *
 * Because the contributions add, two nearby metaballs raise the
 * field in the gap between them. If the gap-raised value crosses
 * T, the level set MERGES — one connected component instead of
 * two.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │  far apart            close together         │
 *      │                                              │
 *      │   ⌐⌐⌐    ⌐⌐⌐         ⌐⌐⌐⌐⌐⌐⌐⌐                │
 *      │  / 1 \  / 2 \         /         \            │
 *      │  \___/  \___/        \_1______2_/            │
 *      │                                              │
 *      │  two blobs           one merged shape        │
 *      │  genus 0 + genus 0   genus 0                 │
 *      └──────────────────────────────────────────────┘
 *
 * As the metaballs orbit, the surface's TOPOLOGY changes — holes
 * open and close, components merge and separate. Marching cubes
 * handles all of this without special-casing: each frame extracts
 * the level set of the CURRENT field, whatever its topology.
 *
 * That's the demo's value proposition: most rendering files
 * shade a STATIC mesh; this one rebuilds the mesh 60 times a
 * second from a moving field, and topology evolves naturally.
 *
 * T8  PER-VERTEX COLOUR BLENDING — WHY MERGING BLOBS SHOW THEIR SEAM
 * ──────────────────────────────────────────────────────────────────
 * Each metaball has a colour. At every emitted vertex p, blend
 * the metaball colours by their INFLUENCE WEIGHTS at that point:
 *
 *     w_i  = s_i / (|p − c_i|² + ε)         ← same as the field
 *     vert_colour(p) = Σ w_i · colour_i / Σ w_i
 *
 * Properties:
 *   - Right next to ball 1: w_1 ≫ all others → vertex looks
 *     ball-1's colour.
 *   - In the middle of a "neck" between ball 1 and ball 2: w_1
 *     and w_2 are comparable → blended colour.
 *   - Far from any ball (shouldn't happen on the surface): all
 *     w_i small but the normalisation by Σ w_i keeps the result
 *     well-defined.
 *
 * The fragment shader (in §8 lightpass) reads the barycentric-
 * interpolated vertex colour and uses it as the surface albedo.
 * Result: the SEAM between two merging blobs visibly mixes their
 * two source colours, while the bulks of each blob retain their
 * own colour. Merging is BOTH topological (one mesh component
 * instead of two) AND chromatic (colour mixing in the seam).
 *
 * This is the cheap way to fake "material blending" without any
 * texturing: per-vertex attribute + barycentric interpolation in
 * the rasteriser. Same machinery as Gouraud shading, just used
 * for colour instead of intensity.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <float.h>
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI */
enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500,
  HUD_ROWS = 5,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define DT_CAP_NS (100 * NS_PER_MS)

/* §1.2 G-buffer dimensions (static; sized for a large terminal). */
#define GBUF_MAX_W 300
#define GBUF_MAX_H 80

/* §1.3 view geometry — eye orbits the metaball volume. */
#define CAM_FOV (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 50.0f

#define CAM_DIST 3.4f
#define CAM_DIST_MIN 1.6f
#define CAM_DIST_MAX 8.0f
#define CAM_ZOOM_STEP 0.2f
#define CAM_EYE_Y 0.6f
#define CAM_LOOK_Y 0.0f
#define CAM_ORBIT_RAD_PER_SEC 0.18f

#define CELL_W 8
#define CELL_H 16

/* §1.4 marching cubes grid
 *
 * GRID_DIM cubes per axis → (GRID_DIM+1)³ corner samples.
 * Bigger = smoother but slower. 24 is a sweet spot at terminal res.
 *
 * The volume spans WORLD_HALF on each side (so x, y, z ∈ [−H, +H]).
 * Cube edge length = 2·H / GRID_DIM. */
#define GRID_DIM 24
#define WORLD_HALF 1.50f

#define MC_THRESHOLD_DEF 1.00f
#define MC_THRESHOLD_MIN 0.30f
#define MC_THRESHOLD_MAX 3.00f
#define MC_THRESHOLD_STEP 0.05f

/* Triangle pool capacity. Each emitted triangle stores 3 verts.
 * Surface area for 4 metaballs in the volume is well under this. */
#define MC_MAX_VERTS 18000

/* §1.5 metaballs — count, strength, orbit, palette */
#define N_METABALLS 4
#define BALL_STRENGTH 0.18f /* s_i in the field formula  */
#define BALL_EPSILON 0.05f  /* ε prevents division spike  */

/* §1.6 lighting — sun direction is universal; sun colour, ambient,
 * rim strength, and ball palette all come from the active theme below. */
/* key light in FRONT (negative z) so camera-facing faces are lit; was +z
 * (behind), which only looked right under the old flipped m4_lookat. */
static const float SUN_DIR[3] = {-0.55f, -0.85f, -0.30f};
#define SHININESS 18.0f
#define SPEC_GAIN 0.28f
#define RIM_POWER 2.5f /* falloff exponent for rim/Fresnel */

/* §1.7 themes — preset palettes + atmosphere.
 *
 * Each theme bundles four metaball colours, an ambient colour, a sun
 * colour, and a rim-light strength. Two of these knobs are what give
 * a theme its 3-D feel:
 *
 *   - LOW ambient → strong contrast between lit and shadow sides
 *     (the blob's "back" goes dark, the silhouette pops).
 *   - HIGH rim_strength → bright Fresnel-like edge along the
 *     silhouette where the surface curves away from view; turns
 *     each blob into a clearly-rounded shape instead of a flat
 *     coloured disc.
 *
 * Press 'n' to cycle. The scene re-applies metaball albedos and the
 * lightpass swaps ambient / sun / rim live. Per CLAUDE.md theme rules
 * every entry sits in the bright half of the 256-colour cube so the
 * darkest tone is still visible against a black terminal background. */
typedef struct {
  const char *name;
  float ball_colors[N_METABALLS][3];
  float ambient[3];
  float sun_col[3];
  float rim_strength;
} Theme;

static const Theme THEMES[] = {
    /* PRIMARY — high-contrast RGB+yellow, the classic graphics-paper look. */
    {"PRIMARY",
     {{0.95f, 0.30f, 0.30f},
      {0.30f, 0.95f, 0.40f},
      {0.40f, 0.50f, 0.95f},
      {1.00f, 0.85f, 0.25f}},
     {0.20f, 0.22f, 0.28f},
     {0.95f, 0.85f, 0.65f},
     0.45f},

    /* LAVA — molten reds and oranges in a hot, dim ambient. */
    {"LAVA",
     {{0.95f, 0.30f, 0.10f},
      {1.00f, 0.65f, 0.15f},
      {0.85f, 0.20f, 0.05f},
      {1.00f, 0.85f, 0.20f}},
     {0.18f, 0.10f, 0.06f},
     {1.00f, 0.78f, 0.45f},
     0.65f},

    /* PLASMA — electric blue / purple / cyan, cool atmosphere. */
    {"PLASMA",
     {{0.30f, 0.50f, 1.00f},
      {0.60f, 0.30f, 0.95f},
      {0.20f, 0.85f, 0.95f},
      {0.85f, 0.30f, 0.85f}},
     {0.10f, 0.10f, 0.22f},
     {0.65f, 0.80f, 1.00f},
     0.75f},

    /* MATRIX — green-on-near-black, cyber feel, strong rim. */
    {"MATRIX",
     {{0.20f, 0.95f, 0.30f},
      {0.40f, 0.90f, 0.20f},
      {0.10f, 0.75f, 0.40f},
      {0.55f, 1.00f, 0.40f}},
     {0.08f, 0.18f, 0.10f},
     {0.55f, 0.95f, 0.55f},
     0.75f},

    /* OCEAN — blues and teals with a cool sun. */
    {"OCEAN",
     {{0.20f, 0.55f, 0.95f},
      {0.30f, 0.75f, 0.95f},
      {0.40f, 0.90f, 0.80f},
      {0.20f, 0.45f, 0.95f}},
     {0.10f, 0.20f, 0.30f},
     {0.70f, 0.85f, 1.00f},
     0.55f},

    /* SUNSET — warm pinks / oranges / purples on a violet ambient. */
    {"SUNSET",
     {{0.95f, 0.50f, 0.40f},
      {1.00f, 0.70f, 0.30f},
      {0.85f, 0.30f, 0.55f},
      {0.70f, 0.30f, 0.75f}},
     {0.28f, 0.16f, 0.26f},
     {1.00f, 0.75f, 0.55f},
     0.55f},

    /* NEON — saturated magenta / cyan / yellow, near-black ambient. */
    {"NEON",
     {{1.00f, 0.20f, 0.85f},
      {0.20f, 1.00f, 0.85f},
      {0.85f, 1.00f, 0.20f},
      {0.20f, 0.50f, 1.00f}},
     {0.08f, 0.08f, 0.16f},
     {1.00f, 1.00f, 1.00f},
     0.85f},
};
#define N_THEMES ((int)(sizeof(THEMES) / sizeof(THEMES[0])))

/* §1.8 character ramp — Paul Bourke 92-char density ladder. */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.9 Bayer 4×4 dither. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.10f

/* §1.10 ncurses pair IDs — 216 cube + yellow HUD + cyan hint. */
#define PAIR_CUBE_BASE 1
#define PAIR_HUD 217
#define PAIR_HINT 218

/* ── §2 clock ────────────────────────────────────────────────────────── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {.tv_sec = (time_t)(ns / NS_PER_SEC),
                       .tv_nsec = (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ── §3 math (V3, V4, Mat4) ──────────────────────────────────────────── */

typedef struct {
  float x, y, z;
} Vec3;
typedef struct {
  float x, y, z, w;
} Vec4;
typedef struct {
  float m[4][4];
} Mat4;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec4 v4(float x, float y, float z, float w) {
  return (Vec4){x, y, z, w};
}

static inline Vec3 v3_add(Vec3 a, Vec3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline Vec3 v3_sub(Vec3 a, Vec3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline Vec3 v3_scale(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}
static inline Vec3 v3_neg(Vec3 a) { return v3(-a.x, -a.y, -a.z); }
static inline Vec3 v3_lerp(Vec3 a, Vec3 b, float t) {
  return v3_add(a, v3_scale(v3_sub(b, a), t));
}
static inline float v3_dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3_len(Vec3 a) { return sqrtf(v3_dot(a, a)); }
static inline Vec3 v3_norm(Vec3 a) {
  float l = v3_len(a);
  return l > 1e-7f ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}
static inline Vec3 v3_bary(Vec3 p0, Vec3 p1, Vec3 p2, float b0, float b1,
                           float b2) {
  return v3(b0 * p0.x + b1 * p1.x + b2 * p2.x,
            b0 * p0.y + b1 * p1.y + b2 * p2.y,
            b0 * p0.z + b1 * p1.z + b2 * p2.z);
}

static inline Mat4 m4_identity(void) {
  Mat4 m = {{{0}}};
  m.m[0][0] = m.m[1][1] = m.m[2][2] = m.m[3][3] = 1.f;
  return m;
}

static inline Vec4 m4_mul_v4(Mat4 m, Vec4 v) {
  return v4(
      m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z + m.m[0][3] * v.w,
      m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z + m.m[1][3] * v.w,
      m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z + m.m[2][3] * v.w,
      m.m[3][0] * v.x + m.m[3][1] * v.y + m.m[3][2] * v.z + m.m[3][3] * v.w);
}

static inline Mat4 m4_mul(Mat4 a, Mat4 b) {
  Mat4 r = {{{0}}};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        r.m[i][j] += a.m[i][k] * b.m[k][j];
  return r;
}

static Mat4 m4_perspective(float fovy, float aspect, float near, float far) {
  Mat4 m = {{{0}}};
  float f = 1.f / tanf(fovy * 0.5f);
  m.m[0][0] = f / aspect;
  m.m[1][1] = f;
  m.m[2][2] = (far + near) / (near - far);
  m.m[2][3] = (2.f * far * near) / (near - far);
  m.m[3][2] = -1.f;
  return m;
}

static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up) {
  Vec3 f = v3_norm(v3_sub(at, eye));
  Vec3 r = v3_norm(v3(f.y * up.z - f.z * up.y, f.z * up.x - f.x * up.z,
                      f.x * up.y - f.y * up.x));
  Vec3 u =
      v3(r.y * f.z - r.z * f.y, r.z * f.x - r.x * f.z, r.x * f.y - r.y * f.x);
  Mat4 m = m4_identity();
  m.m[0][0] = r.x;
  m.m[0][1] = r.y;
  m.m[0][2] = r.z;
  m.m[0][3] = -v3_dot(r, eye);
  m.m[1][0] = u.x;
  m.m[1][1] = u.y;
  m.m[1][2] = u.z;
  m.m[1][3] = -v3_dot(u, eye);
  m.m[2][0] = -f.x;
  m.m[2][1] = -f.y;
  m.m[2][2] = -f.z;
  m.m[2][3] = v3_dot(f, eye);
  return m;
}

/* ── §4 paint (216 RGB cube + Bourke ramp) ───────────────────────────── */

static int g_256;

static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);

  if (g_256) {
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_CUBE_BASE, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/*
 * paint_cell — RGB → terminal pipeline (same as every other raster
 * file): Reinhard tone-map → gamma → Bayer dither → 6×6×6 cube + Bourke
 * ramp char + A_BOLD/A_DIM by luma.
 */
static void paint_cell(int sx, int sy, Vec3 col) {
  float r = gamma_enc(reinhard(col.x));
  float g = gamma_enc(reinhard(col.y));
  float b = gamma_enc(reinhard(col.z));

  float luma = 0.2126f * r + 0.7152f * g + 0.0722f * b;
  float dith = (k_bayer[sy & 3][sx & 3] - 0.5f) * DITHER_AMP;
  float lum_d = clamp01(luma + dith);

  int pair;
  if (g_256) {
    int r5 = (int)(r * 5.f + 0.5f);
    if (r5 > 5)
      r5 = 5;
    if (r5 < 0)
      r5 = 0;
    int g5 = (int)(g * 5.f + 0.5f);
    if (g5 > 5)
      g5 = 5;
    if (g5 < 0)
      g5 = 0;
    int b5 = (int)(b * 5.f + 0.5f);
    if (b5 > 5)
      b5 = 5;
    if (b5 < 0)
      b5 = 0;
    pair = PAIR_CUBE_BASE + r5 * 36 + g5 * 6 + b5;
  } else {
    pair = PAIR_CUBE_BASE;
  }

  int idx = (int)(lum_d * (BOURKE_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= BOURKE_LEN)
    idx = BOURKE_LEN - 1;

  int attr = (luma > 0.85f) ? A_BOLD : (luma < 0.15f) ? A_DIM : A_NORMAL;

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[idx]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 metaballs ────────────────────────────────────────────────────── */

typedef struct {
  Vec3 pos;          /* current world position (set by scene_tick) */
  Vec3 color;        /* palette entry                              */
  float orbit_r;     /* horizontal orbit radius                    */
  float orbit_speed; /* radians per second                         */
  float orbit_y;     /* fixed y of this orbit                      */
  float phase;       /* current angle, advanced each tick          */
} Metaball;

/* Eval scalar field Σ s / (|p − c|² + ε) at world point p. */
static float metaball_field(Vec3 p, const Metaball *balls, int n) {
  float sum = 0.f;
  for (int i = 0; i < n; i++) {
    Vec3 d = v3_sub(p, balls[i].pos);
    float d2 = v3_dot(d, d) + BALL_EPSILON;
    sum += BALL_STRENGTH / d2;
  }
  return sum;
}

/*
 * metaball_outward_normal — outward iso-surface normal at p.
 *   ∇f      = Σ −2s(p−c)/(d²+ε)²       points TOWARD metaballs
 *   N ∝ −∇f = Σ  s(p−c)/(d²+ε)²       points AWAY from metaballs
 */
static Vec3 metaball_outward_normal(Vec3 p, const Metaball *balls, int n) {
  Vec3 g = v3(0, 0, 0);
  for (int i = 0; i < n; i++) {
    Vec3 d = v3_sub(p, balls[i].pos);
    float d2 = v3_dot(d, d) + BALL_EPSILON;
    float k = BALL_STRENGTH / (d2 * d2);
    g = v3_add(g, v3_scale(d, k));
  }
  return v3_norm(g);
}

/*
 * metaball_color — influence-weighted palette blend at p.
 *   c = Σ w_i · color_i / Σ w_i,  w_i = s_i / (|p − c_i|² + ε)
 * Smooth mix through the merge zone where two blobs overlap.
 */
static Vec3 metaball_color(Vec3 p, const Metaball *balls, int n) {
  Vec3 c = v3(0, 0, 0);
  float total_w = 0.f;
  for (int i = 0; i < n; i++) {
    Vec3 d = v3_sub(p, balls[i].pos);
    float d2 = v3_dot(d, d) + BALL_EPSILON;
    float w = BALL_STRENGTH / d2;
    c = v3_add(c, v3_scale(balls[i].color, w));
    total_w += w;
  }
  return total_w > 1e-6f ? v3_scale(c, 1.f / total_w) : v3(1, 1, 1);
}

/* ── §6 marching cubes ───────────────────────────────────────────────── */

/* §6.1 ── cube corner / edge geometry ──────────────────────────────── *
 *
 * Standard Paul Bourke convention. Corner offsets (in unit cube
 * coords) and the 12 edges connecting them — these *must* match the
 * encoding of EDGE_TABLE / TRI_TABLE.
 *
 *           7────────6              corners 0..3 = bottom face (y = 0)
 *          /│       /│              corners 4..7 = top    face (y = 1)
 *         4─┼──────5 │
 *         │ 3──────┼─2              edges 0..3  = bottom  ring
 *         │/       │/               edges 4..7  = top     ring
 *         0────────1                edges 8..11 = vertical pillars
 */
static const int k_corner[8][3] = {
    {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
    {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
};
static const int k_edge[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, /* bottom ring */
    {4, 5}, {5, 6}, {6, 7}, {7, 4}, /* top    ring */
    {0, 4}, {1, 5}, {2, 6}, {3, 7}, /* vertical pillars */
};

/* §6.2 ── EDGE_TABLE ───────────────────────────────────────────────── *
 *
 * For each of the 256 in/out combinations, a 12-bit mask telling
 * which of the 12 cube edges are crossed by the iso-surface. Bit e
 * is set iff edge e has one endpoint inside (f > T) and one outside.
 * Used to know which edges need an interpolated vertex.            */
static const unsigned short EDGE_TABLE[256] = {
    0x000, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c, 0x80c, 0x905, 0xa0f,
    0xb06, 0xc0a, 0xd03, 0xe09, 0xf00, 0x190, 0x099, 0x393, 0x29a, 0x596, 0x49f,
    0x795, 0x69c, 0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90, 0x230,
    0x339, 0x033, 0x13a, 0x636, 0x73f, 0x435, 0x53c, 0xa3c, 0xb35, 0x83f, 0x936,
    0xe3a, 0xf33, 0xc39, 0xd30, 0x3a0, 0x2a9, 0x1a3, 0x0aa, 0x7a6, 0x6af, 0x5a5,
    0x4ac, 0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0, 0x460, 0x569,
    0x663, 0x76a, 0x066, 0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a,
    0x963, 0xa69, 0xb60, 0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0x0ff, 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0, 0x650, 0x759, 0x453,
    0x55a, 0x256, 0x35f, 0x055, 0x15c, 0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53,
    0x859, 0x950, 0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0x0cc, 0xfcc,
    0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0, 0x8c0, 0x9c9, 0xac3, 0xbca,
    0xcc6, 0xdcf, 0xec5, 0xfcc, 0x0cc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9,
    0x7c0, 0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x055,
    0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650, 0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6,
    0xfff, 0xcf5, 0xdfc, 0x2fc, 0x3f5, 0x0ff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x36c, 0x265, 0x16f,
    0x066, 0x76a, 0x663, 0x569, 0x460, 0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af,
    0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0x0aa, 0x1a3, 0x2a9, 0x3a0, 0xd30,
    0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 0x53c, 0x435, 0x73f, 0x636,
    0x13a, 0x033, 0x339, 0x230, 0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895,
    0x99c, 0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x099, 0x190, 0xf00, 0xe09,
    0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c, 0x70c, 0x605, 0x50f, 0x406, 0x30a,
    0x203, 0x109, 0x000,
};

/* §6.3 ── TRI_TABLE ───────────────────────────────────────────────── *
 *
 * For each case, a sequence of edge indices (0..11) read in groups
 * of 3 to form CCW triangles. Sequence is terminated by −1. Up to 5
 * triangles (15 indices) per case. From Bourke's reference table.    */
static const int TRI_TABLE[256][16] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
};

/*
 * mc_verify_tables — at startup, check that EDGE_TABLE is internally
 * consistent with TRI_TABLE: for each case, the bitwise OR of every
 * edge index appearing in TRI_TABLE for that case must equal the
 * EDGE_TABLE entry. Catches typos in the embedded tables before the
 * algorithm runs against silently-wrong data.
 *
 * Returns -1 on success, or the offending case index on mismatch.
 */
static int mc_verify_tables(void) {
  for (int c = 0; c < 256; c++) {
    unsigned derived = 0;
    for (int i = 0; i < 16 && TRI_TABLE[c][i] != -1; i++)
      derived |= (1u << TRI_TABLE[c][i]);
    if (derived != EDGE_TABLE[c])
      return c;
  }
  return -1;
}

/* §6.4 ── vertex pool ───────────────────────────────────────────────── *
 *
 * Each frame mc_extract clears g_mc and appends triangles. Every
 * triangle is three independent vertices (no index buffer) — wastes
 * memory but keeps the mesh format identical to the rasterizer's
 * Vertex layout and trivially debuggable.                             */
typedef struct {
  Vec3 pos;
  Vec3 normal;
  Vec3 color;
} MCVertex;

static MCVertex g_mc[MC_MAX_VERTS];
static int g_mc_count; /* number of vertices in use */

/* §6.5 ── mc_extract — the algorithm ────────────────────────────────── */

/*
 * Sample the metaball field at every grid corner once, then walk the
 * cubes. The corner field array is reused for adjacent cubes — each
 * corner is shared by up to 8 cubes and we evaluate it once per
 * frame, not eight times.
 */
static float g_field[GRID_DIM + 1][GRID_DIM + 1][GRID_DIM + 1];

/* Map integer corner index in [0, GRID_DIM] to world-space coordinate
 * in [-WORLD_HALF, +WORLD_HALF].  step = 2·H / GRID. */
static inline float grid_to_world(int i) {
  const float step = 2.f * WORLD_HALF / (float)GRID_DIM;
  return -WORLD_HALF + (float)i * step;
}

static void mc_compute_field(const Metaball *balls, int n) {
  for (int k = 0; k <= GRID_DIM; k++) {
    float wz = grid_to_world(k);
    for (int j = 0; j <= GRID_DIM; j++) {
      float wy = grid_to_world(j);
      for (int i = 0; i <= GRID_DIM; i++) {
        float wx = grid_to_world(i);
        g_field[k][j][i] = metaball_field(v3(wx, wy, wz), balls, n);
      }
    }
  }
}

/* Append one vertex (pos interpolated; normal+colour evaluated at
 * pos). Returns false if the pool is full. */
static bool mc_push_vertex(Vec3 pos, const Metaball *balls, int n) {
  if (g_mc_count >= MC_MAX_VERTS)
    return false;
  MCVertex *v = &g_mc[g_mc_count++];
  v->pos = pos;
  v->normal = metaball_outward_normal(pos, balls, n);
  v->color = metaball_color(pos, balls, n);
  return true;
}

/* Iso-crossing on the edge a→b:
 *   t = (T − fa) / (fb − fa),  p = lerp(pa, pb, t)
 * t is clamped to [0, 1] against numerical noise. */
static Vec3 mc_edge_lerp(Vec3 pa, float fa, Vec3 pb, float fb,
                         float threshold) {
  float denom = fb - fa;
  float t = (fabsf(denom) > 1e-6f) ? (threshold - fa) / denom : 0.5f;
  if (t < 0.f)
    t = 0.f;
  if (t > 1.f)
    t = 1.f;
  return v3_lerp(pa, pb, t);
}

/*
 * mc_extract — for each cube in the grid:
 *   1. read 8 corner values, build case bitset
 *   2. look up EDGE_TABLE[case] → which edges to interpolate
 *   3. interpolate one vertex per crossed edge
 *   4. walk TRI_TABLE[case] in groups of 3 → emit triangles
 */
static void mc_extract(const Metaball *balls, int n, float threshold) {
  g_mc_count = 0;
  mc_compute_field(balls, n);

  for (int k = 0; k < GRID_DIM; k++) {
    for (int j = 0; j < GRID_DIM; j++) {
      for (int i = 0; i < GRID_DIM; i++) {

        /* (1) read the 8 corner values + world positions. */
        float fv[8];
        Vec3 pv[8];
        int cube_case = 0;
        for (int c = 0; c < 8; c++) {
          int ci = i + k_corner[c][0];
          int cj = j + k_corner[c][1];
          int ck = k + k_corner[c][2];
          fv[c] = g_field[ck][cj][ci];
          pv[c] = v3(grid_to_world(ci), grid_to_world(cj), grid_to_world(ck));
          if (fv[c] > threshold)
            cube_case |= (1 << c);
        }

        /* (2) cubes entirely in or out have no crossings. */
        unsigned emask = EDGE_TABLE[cube_case];
        if (emask == 0)
          continue;

        /* (3) one vertex per crossed edge. */
        Vec3 edge_pos[12];
        for (int e = 0; e < 12; e++) {
          if (!(emask & (1u << e)))
            continue;
          int a = k_edge[e][0];
          int b = k_edge[e][1];
          edge_pos[e] = mc_edge_lerp(pv[a], fv[a], pv[b], fv[b], threshold);
        }

        /* (4) emit triangles from TRI_TABLE[cube_case]. */
        const int *tri = TRI_TABLE[cube_case];
        for (int t = 0; tri[t] != -1; t += 3) {
          if (!mc_push_vertex(edge_pos[tri[t]], balls, n))
            return;
          if (!mc_push_vertex(edge_pos[tri[t + 1]], balls, n))
            return;
          if (!mc_push_vertex(edge_pos[tri[t + 2]], balls, n))
            return;
        }
      }
    }
  }
}

/* ── §7 G-buffer — geometry pass ─────────────────────────────────────── */

static Vec3 g_pos[GBUF_MAX_H][GBUF_MAX_W];
static Vec3 g_normal[GBUF_MAX_H][GBUF_MAX_W];
static Vec3 g_albedo[GBUF_MAX_H][GBUF_MAX_W];
static float g_zbuf[GBUF_MAX_H][GBUF_MAX_W];
static uint8_t g_valid[GBUF_MAX_H][GBUF_MAX_W];

static void gbuffer_clear(int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      g_zbuf[r][c] = 1.0f;
      g_valid[r][c] = 0;
    }
  }
}

static void barycentric(const float sx[3], const float sy[3], float px,
                        float py, float b[3]) {
  float d =
      (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2]);
  if (fabsf(d) < 1e-6f) {
    b[0] = b[1] = b[2] = -1.f;
    return;
  }
  b[0] = ((sy[1] - sy[2]) * (px - sx[2]) + (sx[2] - sx[1]) * (py - sy[2])) / d;
  b[1] = ((sy[2] - sy[0]) * (px - sx[2]) + (sx[0] - sx[2]) * (py - sy[2])) / d;
  b[2] = 1.f - b[0] - b[1];
}

/*
 * rasterize_mc_pool — the camera raster, but with PER-VERTEX colour.
 * Barycentric-blends each triangle's three colours into g_albedo, so
 * the merge seam between two metaballs reads as a smooth blend
 * instead of a hard edge.
 *
 * Geometry is g_mc[] (3 verts per triangle, no index buffer); the
 * model matrix is identity (mc_extract emits in world space).
 */
static void rasterize_mc_pool(Mat4 mvp, int cols, int rows) {
  int n_tri = g_mc_count / 3;
  for (int ti = 0; ti < n_tri; ti++) {
    const MCVertex *v0 = &g_mc[ti * 3 + 0];
    const MCVertex *v1 = &g_mc[ti * 3 + 1];
    const MCVertex *v2 = &g_mc[ti * 3 + 2];

    Vec4 clip[3];
    clip[0] = m4_mul_v4(mvp, v4(v0->pos.x, v0->pos.y, v0->pos.z, 1.f));
    clip[1] = m4_mul_v4(mvp, v4(v1->pos.x, v1->pos.y, v1->pos.z, 1.f));
    clip[2] = m4_mul_v4(mvp, v4(v2->pos.x, v2->pos.y, v2->pos.z, 1.f));

    if (clip[0].w < 0.001f && clip[1].w < 0.001f && clip[2].w < 0.001f)
      continue;

    float sx[3], sy[3], sz[3];
    for (int vi = 0; vi < 3; vi++) {
      float w = clip[vi].w;
      if (fabsf(w) < 1e-6f)
        w = 1e-6f;
      sx[vi] = (clip[vi].x / w + 1.f) * 0.5f * (float)cols;
      sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows;
      sz[vi] = clip[vi].z / w;
    }

    float area =
        (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
    if (area <= 0.f)
      continue;

    int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

    for (int py = y0; py <= y1 && py < GBUF_MAX_H; py++) {
      for (int px = x0; px <= x1 && px < GBUF_MAX_W; px++) {
        float b[3];
        barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
        if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
          continue;

        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        if (z >= g_zbuf[py][px])
          continue;

        g_zbuf[py][px] = z;
        g_pos[py][px] = v3_bary(v0->pos, v1->pos, v2->pos, b[0], b[1], b[2]);
        g_normal[py][px] = v3_norm(
            v3_bary(v0->normal, v1->normal, v2->normal, b[0], b[1], b[2]));
        g_albedo[py][px] =
            v3_bary(v0->color, v1->color, v2->color, b[0], b[1], b[2]);
        g_valid[py][px] = 1;
      }
    }
  }
}

static void render_gbuffer(Mat4 view, Mat4 proj, int cols, int rows) {
  gbuffer_clear(cols, rows);
  Mat4 mvp = m4_mul(proj, view); /* model = identity */
  rasterize_mc_pool(mvp, cols, rows);
}

/* ── §8 lightpass — Blinn-Phong + rim, themed ────────────────────────── *
 *
 *   ambient = theme.ambient · albedo
 *   diffuse = albedo · theme.sun_col · max(0, N · L)
 *   spec    = theme.sun_col · max(0, N · H)^SHININESS · SPEC_GAIN
 *   rim     = theme.sun_col · (1 − max(0, N · V))^RIM_POWER · rim_strength
 *   out     = clamp01( ambient + diffuse + spec + rim )
 *
 * The rim term ramps up at the silhouette (N·V → 0), giving each
 * blob a Fresnel halo that reads as 3-D curvature even on the unlit
 * side. */
static Vec3 g_light[GBUF_MAX_H][GBUF_MAX_W];

static void render_lightpass(Vec3 cam_pos, const Theme *theme, int cols,
                             int rows) {
  Vec3 sun_dir = v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2]);
  Vec3 sun_col = v3(theme->sun_col[0], theme->sun_col[1], theme->sun_col[2]);
  Vec3 ambient = v3(theme->ambient[0], theme->ambient[1], theme->ambient[2]);
  float rim_str = theme->rim_strength;
  Vec3 L = v3_norm(v3_neg(sun_dir));

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g_valid[r][c]) {
        g_light[r][c] = v3(0, 0, 0);
        continue;
      }

      Vec3 P = g_pos[r][c];
      Vec3 N = g_normal[r][c];
      Vec3 albedo = g_albedo[r][c];

      Vec3 amb =
          v3(ambient.x * albedo.x, ambient.y * albedo.y, ambient.z * albedo.z);

      float diff = fmaxf(0.f, v3_dot(N, L));
      Vec3 dif = v3(albedo.x * sun_col.x * diff, albedo.y * sun_col.y * diff,
                    albedo.z * sun_col.z * diff);

      Vec3 V = v3_norm(v3_sub(cam_pos, P));
      Vec3 H = v3_norm(v3_add(L, V));
      float spec = powf(fmaxf(0.f, v3_dot(N, H)), SHININESS) * SPEC_GAIN;
      Vec3 sp = v3(sun_col.x * spec, sun_col.y * spec, sun_col.z * spec);

      float facing = fmaxf(0.f, v3_dot(N, V));
      float rim = powf(1.f - facing, RIM_POWER) * rim_str;
      Vec3 rm = v3(sun_col.x * rim, sun_col.y * rim, sun_col.z * rim);

      Vec3 sum = v3_add(v3_add(v3_add(amb, dif), sp), rm);
      g_light[r][c] =
          v3(fminf(1.f, sum.x), fminf(1.f, sum.y), fminf(1.f, sum.z));
    }
  }
}

/* ── §9 scene ────────────────────────────────────────────────────────── */

typedef struct {
  Metaball balls[N_METABALLS];
  Mat4 view, proj;
  Vec3 cam_pos;
  float cam_dist;
  float cam_yaw;
  float threshold;
  int theme_idx; /* index into THEMES[]               */
  bool paused;
  int scene_cols;
  int scene_rows;
} Scene;

/* theme_apply — copy the active theme's palette onto each ball.
 * Lighting (ambient/sun/rim) isn't cached — render_lightpass reads
 * THEMES[s->theme_idx] live every frame. */
static void theme_apply(Scene *s) {
  const Theme *t = &THEMES[s->theme_idx];
  for (int i = 0; i < N_METABALLS; i++) {
    s->balls[i].color =
        v3(t->ball_colors[i][0], t->ball_colors[i][1], t->ball_colors[i][2]);
  }
}

static void scene_rebuild_proj(Scene *s, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void scene_rebuild_view(Scene *s) {
  float r = s->cam_dist;
  s->cam_pos = v3(sinf(s->cam_yaw) * r, CAM_EYE_Y, cosf(s->cam_yaw) * r);
  s->view = m4_lookat(s->cam_pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
}

/* scene_init — four metaballs on non-resonant orbits (ratios picked
 * so the configuration drifts continuously and never visibly repeats). */
static void scene_init(Scene *s, int total_cols, int total_rows) {
  memset(s, 0, sizeof *s);
  s->scene_cols = total_cols;
  s->scene_rows = total_rows - HUD_ROWS;
  s->threshold = MC_THRESHOLD_DEF;
  s->cam_dist = CAM_DIST;
  s->cam_yaw = 0.f;
  s->theme_idx = 0;

  static const struct {
    float r, speed, y, phase;
  } k_orbit[N_METABALLS] = {
      {0.55f, 0.40f, 0.30f, 0.f},
      {0.65f, 0.55f, -0.25f, (float)M_PI * 0.5f},
      {0.50f, -0.65f, 0.05f, (float)M_PI}, /* reversed */
      {0.70f, 0.85f, -0.10f, (float)M_PI * 1.5f},
  };
  for (int i = 0; i < N_METABALLS; i++) {
    Metaball *b = &s->balls[i];
    b->orbit_r = k_orbit[i].r;
    b->orbit_speed = k_orbit[i].speed;
    b->orbit_y = k_orbit[i].y;
    b->phase = k_orbit[i].phase;
    b->pos = v3(b->orbit_r * cosf(b->phase), b->orbit_y,
                b->orbit_r * sinf(b->phase));
  }

  theme_apply(s);
  scene_rebuild_proj(s, total_cols, s->scene_rows);
  scene_rebuild_view(s);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;

  for (int i = 0; i < N_METABALLS; i++) {
    Metaball *b = &s->balls[i];
    b->phase += b->orbit_speed * dt;
    b->pos = v3(b->orbit_r * cosf(b->phase), b->orbit_y,
                b->orbit_r * sinf(b->phase));
  }

  s->cam_yaw += CAM_ORBIT_RAD_PER_SEC * dt;
  scene_rebuild_view(s);
}

/* ── §10 screen — render_scene + HUD ─────────────────────────────────── */

static void render_scene(const Scene *s) {
  int cols = s->scene_cols;
  int rows = s->scene_rows;

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g_valid[r][c])
        continue;
      paint_cell(c, r, g_light[r][c]);
    }
  }
}

static void hud_draw(const Scene *s, double fps) {
  int hr = s->scene_rows;
  int cols = s->scene_cols;

  int n_tri = g_mc_count / 3;

  /* Row 0: title + status. */
  char status[160];
  snprintf(
      status, sizeof status,
      " %5.1f fps  theme:%s  grid:%dx%dx%d  T=%.2f  tris:%d  zoom:%.1f  %s ",
      fps, THEMES[s->theme_idx].name, GRID_DIM, GRID_DIM, GRID_DIM,
      (double)s->threshold, n_tri, (double)s->cam_dist,
      s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > cols)
    slen = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - slen, "%s", status);
  mvprintw(0, 0, " MARCHING CUBES · METABALLS ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(
      hr + 0, 1,
      "field f(p) = %d * %.2f / (|p-c|^2 + %.2f)   surface = level set f = T",
      N_METABALLS, (double)BALL_STRENGTH, (double)BALL_EPSILON);
  mvprintw(hr + 1, 1,
           "Per cube: 8 corner reads -> 8-bit case -> EDGE_TABLE -> "
           "TRI_TABLE -> emit triangles.");
  mvprintw(hr + 2, 1,
           "Watch the seam where blobs merge: per-vertex colour blend "
           "shows the influence mix.");
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hr + HUD_ROWS - 1, 0,
           " q:quit  spc:pause  n:theme  t/g:threshold  +/-:zoom  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §11 app ─────────────────────────────────────────────────────────── */

typedef struct {
  Scene scene;
  int total_cols;
  int total_rows;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void screen_init(void) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
}

static void app_do_resize(App *app) {
  endwin();
  refresh();
  getmaxyx(stdscr, app->total_rows, app->total_cols);
  scene_init(&app->scene, app->total_cols, app->total_rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_init(s, app->total_cols, app->total_rows);
    break;
  case 't':
  case 'T':
    s->threshold += MC_THRESHOLD_STEP;
    if (s->threshold > MC_THRESHOLD_MAX)
      s->threshold = MC_THRESHOLD_MAX;
    break;
  case 'g':
  case 'G':
    s->threshold -= MC_THRESHOLD_STEP;
    if (s->threshold < MC_THRESHOLD_MIN)
      s->threshold = MC_THRESHOLD_MIN;
    break;
  case 'n':
  case 'N':
    s->theme_idx = (s->theme_idx + 1) % N_THEMES;
    theme_apply(s);
    break;
  case '=':
  case '+':
    s->cam_dist -= CAM_ZOOM_STEP;
    if (s->cam_dist < CAM_DIST_MIN)
      s->cam_dist = CAM_DIST_MIN;
    scene_rebuild_view(s);
    break;
  case '-':
  case '_':
    s->cam_dist += CAM_ZOOM_STEP;
    if (s->cam_dist > CAM_DIST_MAX)
      s->cam_dist = CAM_DIST_MAX;
    scene_rebuild_view(s);
    break;
  default:
    break;
  }
  return true;
}

/* Sum the edge bits TRI_TABLE actually uses for `cube_case`, so the
 * boot-time error message can show what the derived edge mask was. */
static unsigned tri_table_derived_edges(int cube_case) {
  unsigned d = 0;
  for (int i = 0; i < 16 && TRI_TABLE[cube_case][i] != -1; i++)
    d |= (1u << TRI_TABLE[cube_case][i]);
  return d;
}

int main(void) {
  /* Fail loudly at boot if the embedded MC tables disagree —
   * a typo there would silently break surface topology. */
  int bad = mc_verify_tables();
  if (bad >= 0) {
    fprintf(stderr,
            "MC table mismatch at case %d: edge=0x%03x derived=0x%03x\n"
            "Fix TRI_TABLE[%d] or EDGE_TABLE[%d] (Bourke reference).\n",
            bad, EDGE_TABLE[bad], tri_table_derived_edges(bad), bad, bad);
    return 1;
  }

  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;

  screen_init();
  getmaxyx(stdscr, app->total_rows, app->total_cols);
  scene_init(&app->scene, app->total_cols, app->total_rows);

  int64_t frame_time = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    scene_tick(&app->scene, dt_sec);

    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);

    Scene *s = &app->scene;
    erase();
    mc_extract(s->balls, N_METABALLS, s->threshold);
    render_gbuffer(s->view, s->proj, s->scene_cols, s->scene_rows);
    render_lightpass(s->cam_pos, &THEMES[s->theme_idx], s->scene_cols,
                     s->scene_rows);
    render_scene(s);
    hud_draw(s, fps_display);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  endwin();
  return 0;
}
