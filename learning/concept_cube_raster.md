# Pass 1 — cube_raster: Software Rasterizer (Cube)

## Core Idea

Identical rasterization pipeline as torus_raster.c. Only the tessellation changes: instead of a smooth torus, we have a unit cube (6 faces × 2 triangles = 12 triangles with flat normals). Because the cube has large flat faces and sharp edges, the four shaders look dramatically different: Phong shows hard shadow steps per face, Toon is dramatic, Normals shows exactly 6 colors, Wireframe lines are clean and perfectly straight.

Additional feature: `c/C` toggles back-face culling, which lets you see the inside of the cube.

---

## The Mental Model

Same GPU pipeline as torus_raster.c. The cube is a much simpler mesh (12 triangles vs 1536) so all pipeline effects are very visible:
- Back-face culling: exactly 6 triangles visible at any time (the 3 front-facing faces)
- Flat normals: each face has constant normal direction → uniform shading per face
- Toon: 4 diffuse bands × 6 face orientations → at most 6×4=24 possible bands, usually 2-3 visible bands at once
- Wireframe: all 12 edges perfectly straight

---

## Data Structures

Same as torus_raster.c. The only difference is tessellation:

### Cube Tessellation
- 8 vertices at ±CUBE_S on each axis
- Each face: 4 vertices, 1 normal, 2 triangles
- **Flat normals**: all vertices of a face share the same outward-facing normal

| Face | Normal |
|---|---|
| +X right | (1, 0, 0) |
| -X left | (-1, 0, 0) |
| +Y top | (0, 1, 0) |
| -Y bottom | (0, -1, 0) |
| +Z front | (0, 0, 1) |
| -Z back | (0, 0, -1) |

---

## Non-Obvious Decisions vs Torus

### Flat normals (not smooth)
The cube's faces have flat normals — the same normal for all vertices of a face. This means within a face, the diffuse term is constant (no interpolation-based gradient). This is intentional: flat normals make the face look like a flat plane, not a smooth curved surface.

### WIRE_THRESH=0.06 (smaller than torus's 0.08)
Cube triangles cover larger screen area than torus triangles. A smaller threshold keeps the wireframe lines thin and sharp rather than thick bands.

### Toggleable back-face culling
The `c/C` key toggles culling. With culling off, you can see through the cube to the back faces — a good way to verify the winding order and normal direction of all 12 triangles.

### Different rotation rates
ROT_Y=0.55, ROT_X=0.37 — irrational ratio (0.37 ≈ 1/e) ensures you'll eventually see all 6 faces without any face being perfectly axis-aligned for too long.

---

## State Machine

No state machine. Continuous rotation. One boolean: `paused`.

---

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| `CUBE_S` | 0.75 | Half-extent — vertices at ±0.75 on each axis |
| `ROT_Y` | 0.55 | Y-axis spin speed |
| `ROT_X` | 0.37 | X-axis tilt speed |
| `WIRE_THRESH` | 0.06 | Wireframe line width |
| `CAM_DIST` | 2.8 | Camera Z (close = more perspective distortion) |
| `CAM_DIST_MIN/MAX` | 1.0/8.0 | Zoom range |

---

## Open Questions for Pass 3

1. What does the cube look like with TESS_U=TESS_V=1 (just 12 triangles, flat, no subdivision)? (Same — it's already flat)
2. Remove flat normals — interpolate across the face. Does it look like a sphere?
3. What happens to the wireframe shader when back-face culling is off? Do back edges overlap front edges?
4. Try changing cube_s for non-uniform scale (different x/y/z half-extents). Does the normal_mat still produce correct lighting?
5. Add vertex colors: pass a color per vertex through VSOut.custom[]. Interpolate → color the cube's faces.
