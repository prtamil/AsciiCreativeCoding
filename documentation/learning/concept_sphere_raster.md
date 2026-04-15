# Pass 1 — sphere_raster: Software Rasterizer (Sphere)

## Core Idea

Identical rasterization pipeline as torus_raster.c and cube_raster.c. Only the tessellation changes: a UV sphere (36 longitude × 24 latitude = 1728 triangles, smooth normals). The sphere is the best showcase for the pipeline: smooth Phong produces a continuous highlight, Toon forms clean latitudinal rings, Normals gives a full RGB hue map, Wireframe shows the latitude/longitude grid.

---

## The Mental Model

Same GPU pipeline. The sphere has smooth normals (normal = normalize(position) for a unit sphere) so:
- Phong: a smooth circular highlight moves across the surface as the sphere rotates
- Toon: 4 discrete bands form clean concentric rings around the light direction
- Normals: the normal map produces a smooth hue gradient — the classic "normal ball" used in 3D tools as a shading reference
- Wireframe: clean UV grid of latitude lines and longitude lines

---

## Data Structures

Same as torus_raster.c. Tessellation:

### Sphere Tessellation
- UV grid: `theta` (longitude, around equator), `phi` (latitude, pole to pole)
- Position: `(sin(phi)*cos(theta), cos(phi), sin(phi)*sin(theta))`  (standard spherical)
- Normal: `normalize(position)` (trivial for a unit sphere at origin)
- TESS_U=36 longitude slices, TESS_V=24 latitude stacks

---

## Non-Obvious Decisions vs Torus

### Normal = position (smooth, trivial)
For a unit sphere, the outward normal at any surface point is simply `normalize(point)`. Unlike the torus (which needs `normalize(pos - ring_centre)`), the sphere normal is the simplest case.

### Pole vertex handling
The two pole vertices (top: phi=0, bottom: phi=��) are on the Z axis. Different longitude slices share the same position at the poles but have different normal directions in the UV grid. Each pole is a "fan" of triangles, each with a slightly different normal.

### TESS_U=36, TESS_V=24 — higher than torus
The sphere needs more slices for a smooth silhouette. 36 slices = 10° per slice, enough that the equator looks circular at terminal resolution.

---

## State Machine

No state machine. Continuous rotation. One boolean: `paused`.

---

## From the Source

**Algorithm:** Same software rasterization pipeline as torus_raster.c. The sphere is the canonical smooth-shading demo: per-vertex normals = normalised vertex positions (unit sphere). Interpolated across the face using barycentric coordinates → continuous Phong highlight across triangle boundaries.

**Math:** UV sphere tessellation: for `u ∈ [0, 2π]` and `v ∈ [0, π]`: `x = sin(v)·cos(u), y = cos(v), z = sin(v)·sin(u)`. `TESS_U` rings × `TESS_V` stacks gives `2 × TESS_U × TESS_V` triangles. Poles require triangle fans (degenerate quads).

**Performance:** The sphere tessellation is fixed at init; no re-tessellation per frame (unlike displace_raster.c which recomputes vertices every frame). Smooth normals reduce visual artifacts when triangle count is low — a UV sphere looks smooth with just 16×16 tessellation.

---

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| `SPHERE_R` | 1.0 | Radius — fixed (no interactive resize) |
| `TESS_U` | 36 | Longitude slices (more = smoother equator) |
| `TESS_V` | 24 | Latitude stacks (more = smoother poles) |
| `CAM_DIST` | 2.6 | Camera distance (close = larger sphere) |
| `ROT_Y/X` | 0.6/0.4 | Rotation speeds |
| `WIRE_THRESH` | 0.08 | Wireframe threshold (same as torus) |

---

## Open Questions for Pass 3

1. Visualize the normal map: does it look like a rainbow ball? Does the hue match the surface orientation?
2. Try TESS_U=6, TESS_V=4 — what does the "sphere" look like? (A faceted polyhedron)
3. The sphere normals shader doesn't use the normal matrix — why? (Sphere has uniform scale, rotation of a unit normal is still unit)
4. How does the wireframe compare visually to wireframe.c's sphere? (same latitude/longitude grid, different line drawing method)
5. Add a second sphere translated to the side: `scene_sdf = min(sphere_a, sphere_b)`. Can the rasterizer handle two separate meshes?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `VSIn` / `VSOut` / `FSIn` / `FSOut` | `typedef struct` | ~40–64 B each | shader I/O types for the vertex → fragment pipeline |
| `Uniforms` (struct) | `typedef struct` | ~200 B | MVP matrices, normal matrix, light pos/col, camera pos |
| `g_zbuf[rows×cols]` | `float[]` | ~80 KB | per-cell depth buffer for hidden-surface removal |
| `g_cbuf[rows×cols]` | `Vec3[]` | ~240 KB | per-cell colour accumulator; blitted to ncurses each frame |
| `TESS_U`, `TESS_V` | `int` constants | N/A | UV sphere tessellation (36 longitude × 24 latitude = 1728 triangles) |
| `SPHERE_R` | `float` constant | N/A | sphere radius (1.0); fits well at CAM_DIST=2.6 with FOV=55° |
| `k_bayer[4][4]` | `float[16]` | 64 B | Bayer ordered-dither matrix for ASCII density ramp |

# Pass 2 — sphere_raster: Pseudocode

## Module Map

Same sections as torus_raster.c (§1–§9). Only §1 config and §4 tessellation differ.

| Section | Difference from torus_raster |
|---|---|
| §1 config | SPHERE_R=1.0, TESS_U=36, TESS_V=24, CAM_DIST=2.6, ROT_Y=0.6, ROT_X=0.4, WIRE_THRESH=0.08 |
| §4 mesh | `tessellate_sphere()` — UV sphere, smooth normals |
| All others | Identical to torus_raster.c |

---

## Sphere Tessellation

```
tessellate_sphere() → Mesh:
  R = SPHERE_R
  nu = TESS_U, nv = TESS_V
  nvert = (nu+1)*(nv+1)
  ntri  = nu*nv*2

  for i in 0..nu:
    u = i/nu; theta = u * 2π
    ct = cos(theta); st = sin(theta)

    for j in 0..nv:
      v = j/nv; phi = v * π     ← latitude: 0=north pole, π=south pole
      cp = cos(phi); sp = sin(phi)

      pos = (R*sp*ct, R*cp, R*sp*st)
           //  ← note: phi from 0 (top) to π (bottom), standard spherical

      normal = normalize(pos) = (sp*ct, cp, sp*st)  ← same as pos for unit sphere
      verts[i*(nv+1)+j] = {pos, normal, u=i/nu, v=j/nv}

  // Build quad triangles: same UV grid pattern as torus
  for i in 0..nu-1:
    for j in 0..nv-1:
      r0 = i*(nv+1)+j; r1 = r0+1
      r2 = (i+1)*(nv+1)+j; r3 = r2+1
      tris[...] = {r0,r2,r1}
      tris[...] = {r1,r2,r3}

Key difference from torus:
  Torus normal = normalize(pos - ring_centre)
  Sphere normal = normalize(pos)    ← trivial since it's a unit sphere
```

---

## Core Loop (Differences from torus_raster)

```
input additions:
  c / C   → toggle back-face culling (same as cube_raster)
  + / =   → cam_dist -= ZOOM_STEP
  -       → cam_dist += ZOOM_STEP
```

All other loop logic identical to torus_raster.c.

---

## Smooth vs Flat Normals

```
Torus:  smooth (each vertex has unique tube-outward normal)
Sphere: smooth (each vertex has unique normalize(position) normal)
Cube:   flat (all 4 vertices per face share one normal)

In pipeline_draw_mesh:
  fsin.world_nrm = normalize(bary_blend(vo[0..2].world_nrm, b))
  ↑ Barycentric interpolation of vertex normals
  For smooth normals: this interpolates across the face → Gouraud shading
  For flat normals: all 3 normals are the same → no visible interpolation
```
