# Pass 1 — sphere_raytrace.c: Analytic ray-traced sphere

## Core Idea

Fires one ray per terminal cell and solves the exact quadratic equation for where
that ray intersects a sphere.  No mesh, no marching — the hit point is computed
directly from algebra.  Three coloured lights (key · fill · rim) shade the surface
with Phong, and four modes expose different mathematical properties of the surface.

The camera orbits the sphere slowly, revealing all sides without the object ever
needing to "rotate."

---

## The Ray-Sphere Intersection

Ray parametrically: `P(t) = ro + t · rd`  where `|rd| = 1`.

Sphere at origin, radius r: `|P|² = r²`.

Substitute:
```
|ro + t·rd|² = r²
t²(rd·rd) + 2t(rd·ro) + (ro·ro − r²) = 0
```

Since `|rd| = 1`, this simplifies to:
```
t² + 2bt + c = 0       where b = rd·ro,  c = |ro|² − r²
discriminant  = b² − c
t = −b ± √discriminant
```

- `discriminant < 0`  → miss (ray passes outside sphere)
- Take the smaller positive root: `t₀ = −b − √disc` if `t₀ > 0`, else `t₁ = −b + √disc`

This is a closed-form solution — no iteration.

---

## 3-Point Lighting

Three lights in fixed world space create depth without any shadow rays:

| Light | Position | Colour | Role |
|-------|----------|--------|------|
| Key | (3, 4, −2) | warm white | primary diffuse + sharp specular |
| Fill | (−4, 1, −1) | cool blue | soft diffuse; lifts shadow side |
| Rim | (0.5, −1, 5) | warm colour | backlight; separates silhouette |

**Phong per light:**
```
L = normalize(light.pos − P)
d = max(0, dot(N, L))             ← diffuse
R = reflect(−L, N)
s = pow(max(0, dot(R, V)), k)     ← specular

contribution = d·intensity·(obj·light_color) + s·spec_color
```

The rim light uses a low shininess exponent (10 vs 52 for key) — it spreads into a
wide soft highlight on the back silhouette rather than a tight glint.

---

## Four Shade Modes

### Phong
Standard 3-light illumination.  Combined ambient + diffuse (key + fill) + specular
(key + rim).

### Normals
```
color = (N + 1) / 2     ← maps [−1,1]³ → [0,1]³
```
X → red channel, Y → green, Z → blue.  Useful for verifying normals are correct.

### Fresnel (glass / crystal)
Schlick approximation: brightness at a surface point depends on the viewing angle.
Grazing angles → bright edge; head-on → dark core.
```
cosA    = |dot(N, V)|
fresnel = (1 − cosA)⁵           ← Schlick
color   = lerp(dark_core, bright_edge, fresnel)
```
Makes the sphere look like glass or a crystal ball — completely dark in the centre,
glowing where the surface curves away.

### Depth
Brightness proportional to `1 − (t / t_max)²`.  No colour information —
pure shape from shading by distance.

---

## Camera Orbit

Instead of rotating the sphere (which changes nothing for a uniform sphere),
the camera moves in a horizontal circle:
```
cam.x = cam_dist × sin(orbit_angle)
cam.z = −cam_dist × cos(orbit_angle)
cam.y = CAM_HEIGHT = 0.55          ← slight elevation
```

The camera always looks at the origin.  Camera basis is rebuilt each frame:
```
fwd = normalize(origin − cam)
rgt = normalize(cross(fwd, world_up))
up  = cross(rgt, fwd)
```

This reveals all sides of the sphere as the lights stay fixed — the Doppler-like
left/right brightness asymmetry sweeps across the surface.

---

## 256-Color Output

Pre-initialise pairs 1–216 at startup:
```c
for (int i = 0; i < 216; i++)
    init_pair(i + 1, 16 + i, -1);   /* maps to 6×6×6 color cube */
```

Per pixel: compute shaded RGB (0–1), map to nearest cube entry:
```
r5 = round(r × 5),  g5 = round(g × 5),  b5 = round(b × 5)
pair = r5×36 + g5×6 + b5 + 1
```

Luminance maps to Paul Bourke character density ramp (92 levels):
```
lum = 0.299r + 0.587g + 0.114b
ch  = ramp[round(lum × 91)]
```

Both the **character** (density) and the **colour** (hue/saturation) carry
information — the terminal cell encodes two independent dimensions of the image.

---

## Non-Obvious Decisions

### Why orbit the camera rather than rotate the sphere?
A uniform sphere is rotationally symmetric — rotating it changes nothing unless it
has surface markings.  Orbiting the camera achieves the same animation with less
indirection, and leaves the world-space light positions fixed so lighting is
predictable.

### Why divide ray v-component by ASPECT?
Terminal cells are taller than wide (~2:1 pixel ratio).  Without correction, a
sphere would appear as a tall oval.  Dividing the vertical NDC by ASPECT
(`cell_w / cell_h ≈ 0.47`) compresses the vertical ray angle to match the physical
cell proportions, giving circular silhouettes.

### Why `b = dot(rd, ro)` not `b = 2·dot(rd, ro)`?
The standard quadratic form `at² + 2bt + c = 0` uses `2b` in the formula so that
`t = −b ± √(b²−ac)`.  With `a = 1`, this becomes `t = −b ± √(b²−c)` — one fewer
multiplication.  The convention here absorbs the factor of 2 into the variable
name.

### Why three lights instead of one?
A single light leaves the shadow hemisphere completely black — half the sphere
disappears.  The fill light lifts shadow darkness to ~15% visible brightness,
maintaining shape cues.  The rim light adds a bright outline that separates the
sphere silhouette from the black background — the single most effective trick for
making 3D objects pop on a dark terminal.

---

## From the Source

**Algorithm:** The full quadratic form uses `b = 2·(rd · oc)` with `oc = ro − c`, giving discriminant `b²−4ac`. The concept file uses the optimized half-b form (`b' = rd·ro`, discriminant `b'²−c`) which absorbs the factor of 2 — both are equivalent but the source form is the canonical textbook version.

**Rendering:** Each terminal cell fires exactly **one** ray (no anti-aliasing). Luminance is mapped to ASCII ramp `".+*#@"` — the specific 5-character density set used. The Phong model: `I = ka + kd·max(N·L,0) + ks·max(R_v·V,0)^shininess` where `R_v = 2(N·L)N − L`.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect |
|----------|---------|--------|
| `SPHERE_R` | 1.0 | sphere radius in world units |
| `ORBIT_SPEED` | 0.32 rad/s | how fast camera orbits |
| `CAM_HEIGHT` | 0.55 | elevation; 0 = equatorial view |
| `CAM_DIST_DEF` | 3.6 | camera distance; `+/-` at runtime |
| `SHININESS` | 52 | specular exponent; higher = tighter glint |
| `AMBIENT` | 0.04 | minimum brightness in full shadow |
| `FOV_DEG` | 58° | wider = more distortion at edges |

---

## Open Questions

1. The Fresnel mode uses Schlick `(1−cosθ)⁵`.  Derive the full Fresnel equations
   for a dielectric with refractive index n.  How does changing n from 1.0 to 1.5
   (glass) change the reflectance curve?

2. Add a secondary reflected ray: when a Phong hit occurs, shoot a new ray in the
   reflection direction `R = reflect(rd, N)` and trace it.  Mix the reflected
   colour with the surface colour at some reflectivity k.  What does the sphere
   look like when it reflects itself?

3. The current implementation shades `P` in world space with world-space light
   positions.  Mathematically, you could equally shade in camera space.  What
   would need to change?

4. The rim light uses exponent 10 (vs 52 for key).  Derive the visual half-angle
   of a Phong specular lobe as a function of the exponent.  At what exponent does
   the rim light look "diffuse" vs "specular"?

---

# Pass 2 — sphere_raytrace.c: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | SPHERE_R, ORBIT_SPEED, CAM_*, FOV_DEG, SHININESS, AMBIENT |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 V3 math | add, sub, scale, mul, dot, len, norm, cross, reflect, clamp1 |
| §4 color | 6 themes (obj, spec, key_col, fill_col, rim_col); 216 pre-init pairs |
| §5 intersection | `ray_sphere(ro, rd, r)` → t |
| §6 shading | `shade_phong`, `shade_normal`, `shade_fresnel`, `shade_depth` |
| §7 render | camera orbit → ray per cell → shade → `draw_color` |
| §8 screen | `screen_hud()` |
| §9 main | fixed-timestep loop, input |

## Data Flow

```
each frame:
  orbit_ang += ORBIT_SPEED × dt

  render(cols, rows, orbit_ang, cam_dist, theme, mode):
    cam = { cam_dist×sin(orbit_ang), CAM_HEIGHT, −cam_dist×cos(orbit_ang) }
    build (fwd, rgt, up) from cam toward origin

    for each (row, col):
      rd = normalize(fwd + u×rgt + v×up)    ← u,v from FOV and ASPECT
      t  = ray_sphere(cam, rd, SPHERE_R)
      if miss: continue

      P = cam + t×rd
      N = normalize(P)                       ← sphere at origin
      V = normalize(cam − P)

      color = shade_*(P, N, V, theme)
      lum   = luminance(color)
      draw_color(row, col, color, lum)
```

## ray_sphere

```
b    = dot(rd, ro)
c    = dot(ro,ro) − r×r
disc = b×b − c
if disc < 0: miss
sq   = sqrt(disc)
t0   = −b − sq
t1   = −b + sq
if t1 < ε: miss
return t0 > ε ? t0 : t1
```

## shade_phong (per light)

```
for light in [key, fill, rim]:
  L = normalize(light.pos − P)
  d = max(0, dot(N, L))
  R = reflect(−L, N)
  s = pow(max(0, dot(R, V)), shininess_for_light)
  col += d × intensity × (obj × light_color)
  col += s × spec_intensity × spec_color
return clamp(col, 0, 1)
```

## shade_fresnel

```
cosA    = |dot(N, V)|
fresnel = (1 − cosA)^5
return lerp(0.06×obj, clamp(0.7×spec + 0.5×rim_col), fresnel)
```
