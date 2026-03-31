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
