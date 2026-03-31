# Pass 2 — cube_raster: Pseudocode

## Module Map

Same sections as torus_raster.c (§1–§9). Only §1 config and §4 tessellation differ.

| Section | Difference from torus_raster |
|---|---|
| §1 config | CUBE_S=0.75, ROT_Y=0.55, ROT_X=0.37, WIRE_THRESH=0.06, CAM_DIST=2.8, CAM_ZOOM_STEP=0.2, CAM_DIST_MIN/MAX, + culling toggle |
| §4 mesh | `tessellate_cube()` — 6 faces × 2 triangles = 12, flat normals |
| All others | Identical to torus_raster.c |

---

## Cube Tessellation

```
tessellate_cube() → Mesh:
  // 6 faces, each as 4 vertices + 1 normal + 2 triangles

  faces[6] = {
    // normal, and 4 CCW vertex positions
    { n=(+1,0,0), verts=[(+s,-s,+s),(+s,+s,+s),(+s,+s,-s),(+s,-s,-s)] },  // +X
    { n=(-1,0,0), verts=[(-s,-s,-s),(-s,+s,-s),(-s,+s,+s),(-s,-s,+s)] },  // -X
    { n=(0,+1,0), verts=[(-s,+s,-s),(+s,+s,-s),(+s,+s,+s),(-s,+s,+s)] },  // +Y
    { n=(0,-1,0), verts=[(-s,-s,+s),(+s,-s,+s),(+s,-s,-s),(-s,-s,-s)] },  // -Y
    { n=(0,0,+1), verts=[(-s,-s,+s),(-s,+s,+s),(+s,+s,+s),(+s,-s,+s)] },  // +Z
    { n=(0,0,-1), verts=[(+s,-s,-s),(+s,+s,-s),(-s,+s,-s),(-s,-s,-s)] },  // -Z
  }

  for each face:
    add 4 vertices with face normal (flat shading)
    add 2 triangles: [v0,v1,v2] and [v0,v2,v3]

Note: flat normals — all 4 vertices of a face share the same outward normal.
This makes diffuse lighting constant within each face (no interpolated gradient).
```

---

## Key Differences from torus_raster

```
1. Tessellation:
   Torus:  (TESS_U+1)*(TESS_V+1) verts = 33*25 = 825, 2*32*24 = 1536 tris
   Cube:   6*4 = 24 verts, 6*2 = 12 tris

2. Normals:
   Torus:  smooth (interpolated from tube geometry)
   Cube:   flat (all 4 vertices per face share one normal)

3. back_face_culling toggle:
   scene.cull_back = true by default
   if !cull_back: skip the `area <= 0` check in pipeline_draw_mesh

4. Zoom via CAM_DIST:
   + / = : cam_dist -= CAM_ZOOM_STEP (move closer)
   -     : cam_dist += CAM_ZOOM_STEP (move further)
   Rebuild view matrix: m4_lookat((0,0,cam_dist), origin, up)
```

---

## Core Loop (Differences from torus_raster)

```
input additions:
  c / C   → cull_back = !cull_back
  + / =   → cam_dist -= ZOOM_STEP (clamp to MIN)
  -       → cam_dist += ZOOM_STEP (clamp to MAX)
  // when cam_dist changes: rebuild view matrix
```

All other loop logic identical to torus_raster.c.

---

## Interactions

Same as torus_raster.c. pipeline_draw_mesh receives `cull_back` flag and conditionally skips the back-face cull step.
