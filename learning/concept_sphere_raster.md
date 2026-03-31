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
