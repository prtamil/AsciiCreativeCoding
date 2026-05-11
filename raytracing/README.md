# raytracing — analytic ray ↔ primitive intersection + path tracing

A reference for the **raytracing** paradigm of 3-D rendering. This
folder contains **10 self-contained C programs** that build a ray
tracer from a 130-line single-sphere quadratic up to a volumetric
path tracer with next-event estimation — every file re-using the
same per-pixel ray-generation skeleton and the same RGB → 6×6×6 cube
+ 92-char Bourke ramp paint pipeline.

Raytracing answers the question *"for each terminal cell, where does
this ray hit the scene — solved analytically?"*. Unlike the
raymarcher (which **iteratively** marches by an SDF) and unlike the
rasteriser (which **walks the triangles**, asking "which pixels does
this triangle cover?"), the analytic ray tracer **solves an equation**
for the closest intersection. A ray hits a sphere by solving a
quadratic; a box by intersecting three slabs; a torus by solving a
quartic; a cylinder by another quadratic. Every primitive has its own
closed-form solver. The opposite poles of this paradigm live in
[`../raster/`](../raster/) (forward triangle rasterisation on an
explicit mesh) and [`../raymarcher/`](../raymarcher/) (sphere-tracing
on signed distance fields). The same scene — a sphere lit by one
light — materialises as `sphere_raytrace.c` here,
[`raster/sphere_raster.c`](../raster/sphere_raster.c) there, and
[`raymarcher/raymarcher.c`](../raymarcher/raymarcher.c) in the third
folder. Reading the trio side-by-side is the cleanest way to feel the
three paradigms.

If you read **only one file**, read
[`sphere_raytrace.c`](sphere_raytrace.c) — it is the canonical
exemplar and every other file in the folder is downstream of its
`ray_sphere` + `shade_phong` + `render` skeleton.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive](#the-unifying-primitive)
   * [`ray_X(ro, rd, ...) → bool`](#ray_xro-rd----bool)
   * [The render loop](#the-render-loop)
   * [Why every file looks the same](#why-every-file-looks-the-same)
3. [File index](#file-index)
   * [Single-primitive demos — 4 files](#single-primitive-demos)
   * [Surface raytracing scenes — 2 files](#surface-raytracing-scenes)
   * [Path tracer + volumetric rendering — 4 files](#path-tracer--volumetric-rendering)
4. [Building and running](#building-and-running)
5. [Adding a new raytracing demo](#adding-a-new-raytracing-demo)

---

## How to read this folder

The recommended path goes from the simplest single-primitive analytic
intersection up through progressively harder algebra (quadratic →
slab → decomposed → quartic), then to multi-primitive scenes
(starfield + planet + rings, sun + moon + corona), then to **path
tracing** where many rays per pixel + bounces resolve global
illumination, and finally to **volumetric path tracing** with
next-event estimation for god rays.

```
        ONE PRIMITIVE                        SCENES                   PATH TRACING + VOLUMETRICS
        ─────────────                        ──────                   ──────────────────────────
   1.  sphere_raytrace.c           ──▶  5. saturn_with_rings.c   ──▶ 7. path_tracer.c
        quadratic                         sphere + flat ring +         Cornell Box, cosine-weighted
                │                         starfield + soft shadows     bounces, Russian roulette
                ▼
   2.  cube_raytrace.c             ──▶  6. solar_eclipse.c            8. forest_god_rays.c
        slab method                       sphere + sphere + corona +   volumetric NEE through height-
        (3 axis-aligned slabs)            chromosphere + Bailey beads  decaying mist; trees as cylinders
                │
                ▼                                                       9. god_rays_window.c
   3.  capsule_raytrace.c                                                slit-aware NEE through 10
        decomposed (cyl + 2 spheres)                                     pointed-arch openings
                │
                ▼                                                       10. tunnel.c
   4.  torus_raytrace.c                                                  inside-the-cylinder fly-through
        quartic                                                          analytic ray-cylinder
        (numerical scan + bisect)
```

**Prerequisites graph.** Every file states *Study alongside:* in its
header:

* `sphere_raytrace.c` is the **root** — read first. The ray-sphere
  quadratic is the canonical analytic raytrace; if you understand it
  you can read every other file in the folder.
* `cube_raytrace.c` introduces the **slab method** — ray vs
  axis-aligned box = intersect three pairs of parallel planes, keep
  the largest near-hit and the smallest far-hit. Geometric, not
  algebraic.
* `capsule_raytrace.c` introduces **decomposed analytic** intersection
  — capsule = cylinder body + two hemispherical caps, test each piece
  separately and pick the closest hit. The general pattern for
  composite primitives.
* `torus_raytrace.c` is the **algebraic ceiling** — the only primitive
  in the folder that needs a degree-4 polynomial solved numerically
  (Horner-form coefficient build, sign-change scan, then bisection).
* `saturn_with_rings.c` and `solar_eclipse.c` are **scene files**:
  multi-primitive analytic raytrace with hand-authored shading that
  layers limb darkening + atmospheric rim + soft shadow + Cassini
  forward-scatter (Saturn), or corona σ(r) ∝ (R/r)³ + Hα chromosphere
  + procedural Bailey's beads + diamond ring (eclipse).
* `path_tracer.c` is the **paradigm shift** — Monte Carlo: stop
  layering hand-authored effects, let randomness + bounce-sampling
  resolve global illumination on its own. Cornell Box, cosine-weighted
  bounces, Russian roulette, progressive accumulation.
* `forest_god_rays.c` and `god_rays_window.c` are the **volumetric
  pair** — ray-march through a participating medium and use
  next-event estimation (NEE) at each sample to ask "can I see the
  sun from here?". Henyey-Greenstein phase function. The god rays
  emerge naturally from the unobstructed regions of the mist.
* `tunnel.c` is a **first-person showcase** — analytic ray-cylinder
  with cylindrical UV, procedural patterns, distance fog. Read it as
  a quick study of cylindrical surface parameterisation.

---

## The unifying primitive

Every file in this folder is built around exactly **one function
signature** and **one per-pixel render loop**. Master those and the
10 files become 10 variations on the same theme.

### `ray_X(ro, rd, ...) → bool`

Every primitive in this folder exposes the same intersection contract:

```c
/* returns true if the ray (ro + t·rd, t ≥ 0) hits this primitive
 * before any previous near-hit; writes t at the first hit point.
 */
bool ray_sphere (V3 ro, V3 rd, float r,                  float *t_hit);
bool ray_aabb   (V3 ro, V3 rd, V3 box_min, V3 box_max,   float *t_hit);
bool ray_capsule(V3 ro, V3 rd, V3 a, V3 b, float r,      float *t_hit);
bool ray_torus  (V3 ro, V3 rd, float R, float r,         float *t_hit);
bool ray_cyl    (V3 ro, V3 rd, V3 axis, float r,         float *t_hit);
bool ray_plane  (V3 ro, V3 rd, float plane_y,            float *t_hit);
bool ray_ring   (V3 ro, V3 rd, float r_in, float r_out,  float *t_hit);
```

The implementations are all **closed-form** (sphere quadratic, slab
intervals, cylinder quadratic, ring plane-intersect-then-radius-test)
except `ray_torus`, which builds a quartic in `t` and finds the
smallest positive root by scanning for sign changes then bisecting.

The five solver styles across the folder, in increasing complexity:

| Style | Files | Math |
|---|---|---|
| Quadratic | `sphere_raytrace`, `tunnel`, both god-rays files | discriminant + `b ± √Δ / 2a` |
| Slab method (linear per axis) | `cube_raytrace`, `god_rays_window` (wall AABB) | per-axis `t_min`, `t_max`; intersect 3 intervals |
| Decomposed (segment + caps) | `capsule_raytrace` | cylinder body + 2 spheres, take closest |
| Quartic (numerical) | `torus_raytrace` | Horner-form coefficient build + sign-change scan + bisection |
| Procedural (path tracer's job) | `path_tracer`, both god-rays | analytic per-primitive + nearest-hit loop over the scene |

### The render loop

The per-frame body of every file in this folder is:

```
   for each terminal cell (row, col):
       1. generate primary ray (ro, rd) from camera through pixel
       2. scene_hit(ro, rd) → nearest Hit  (t, point, normal, material)
       3. if no hit:    paint sky / background
          if hit:       shade(Hit, lights) → RGB
       4. (path tracer / volumetric only:)
              loop: sample bounce direction, accumulate throughput,
                    Russian-roulette terminate;   OR
              loop: ray-march samples through medium, NEE at each
       5. tone map RGB → 6×6×6 cube + glyph from 92-char Bourke ramp
       6. write cell  (char, color pair, attribute)
```

The **direct** raytrace files (sphere, cube, capsule, torus, Saturn,
eclipse, tunnel) finish at step 3 — one primary ray, one analytic
hit, one shader call, done. The **path tracer** (`path_tracer.c`)
loops bounces in step 4, accumulating throughput over up to 8
bounces, then averages across many samples per pixel. The
**volumetric** files (`forest_god_rays.c`, `god_rays_window.c`)
loop ray-march samples through participating media in step 4, calling
NEE at each sample to add in-scattered sunlight that reaches that
sample without being blocked.

The **paint pipeline** for step 5 is byte-for-byte identical to the
one used by every file in [`../raster/`](../raster/) — continuous RGB
→ Reinhard tone map → gamma → 6×6×6 RGB cube quantisation → 92-char
Bourke ramp glyph + Bayer 4×4 dither. Pixel quality is decoupled from
the rendering paradigm.

### Why every file looks the same

The split — `ray_X` for intersection, `shade_*` for material, the
ncurses paint stack for output — is the same across every file in
the folder **because the render loop doesn't know what the primitive
is**. It knows only:

* a ray is `(ro, rd)` with `rd` unit;
* `ray_X(ro, rd, ..., &t_hit)` returns true and writes `t_hit` if it
  intersects;
* the closest of several `ray_X` calls is the **nearest hit**;
* a `Hit` carries a normal and a material, both of which feed `shade_*`.

Substitute a sphere, a slab, a torus, a planet + rings, a Cornell Box,
ten arch-shaped wall openings — the render loop keeps working. **The
primitive set changes and the shading layers on top. Everything else
stays still.** That invariance is what makes raytracing re-usable
across every demo in this folder.

---

## File index

### Single-primitive demos

| File | Description | What it adds vs. predecessor |
|---|---|---|
| `sphere_raytrace.c` | analytic ray-sphere quadratic, 3 fixed white lights, 4 shaders (phong / normals / fresnel / depth), 20 PBR themes | the canonical exemplar — `ray_sphere` quadratic, paint pipeline, three-point lighting, theme system. Read first. |
| `cube_raytrace.c` | spinning cube, slab method | replaces the quadratic with **3 slab intervals**: per-axis `t_min`, `t_max`, intersect them. Debug overlays show axis / interval / face-uv to visualise the algorithm |
| `capsule_raytrace.c` | rotating pill, decomposed analytic | replaces the single solver with **3 sub-tests** (cylinder body + 2 spheres) and a dispatcher that picks the nearest valid hit |
| `torus_raytrace.c` | tumbling donut, quartic | the algebraic ceiling — builds a degree-4 polynomial in `t` (Horner form), scans for sign changes, bisects to refine. Same shader scaffold as `sphere_raytrace.c` |

### Surface raytracing scenes

| File | Description | What it adds vs. predecessor |
|---|---|---|
| `saturn_with_rings.c` | banded planet + flat ring + starfield + sun orbit | **multi-primitive scene**: `ray_sphere` (planet, depth sort), `ray_ring` (annular plane), star field RNG; hand-authored shading layers limb darkening + atmospheric rim + Cassini forward-scatter + soft ring-on-planet shadow. 4 patterns (SATURN / URANUS / EARTH / EXO) |
| `solar_eclipse.c` | path-traced sun + moon with corona, Hα chromosphere, prominences, Bailey's beads, diamond ring | **layered volumetric scene**: corona σ(r) ∝ (R/r)³ + closed-form circle-circle disc-overlap visibility, procedural moon-limb valleys (44 micro + 14 macro) so Bailey beads + diamond ring emerge from real geometry, eye-adaptation gate. 5 stellar classes, 4 eclipse patterns |

### Path tracer + volumetric rendering

| File | Description | What it adds vs. predecessor |
|---|---|---|
| `path_tracer.c` | progressive Cornell Box, gold + indigo spheres | the **paradigm shift**: 7 passes (camera ray → intersection → shading decision → cosine-weighted bounce sampling → throughput chain → Russian-roulette terminate → accumulate + tone map). Color bleeding visibly materialises as samples accumulate |
| `forest_god_rays.c` | bare forest with low sun, height-decaying mist | **volumetric NEE** — per pixel ray-march through `σ_e(y) = σ₀ · exp(−y/h)` mist; at each sample, next-event estimation tests if the chord-to-sun is unblocked by a trunk (`ray_cyl`). Henyey-Greenstein phase function, blackbody sun colour |
| `god_rays_window.c` | mosque-interior with 10 pointed-arch openings, uniform mist | same NEE kernel as `forest_god_rays.c`, but the **slit-aware wall** is what defines visibility — the chord-to-sun passes through one of the 10 lancet arches or hits solid wall. Two crossing planes of shafts from a circular sun orbit |
| `tunnel.c` | first-person fly-through, analytic ray-cylinder, procedural patterns | **inside-out raytracing** — camera sits at the origin inside an infinite cylinder, every cell fires one ray outward, intersects analytically (quadratic again), recovers cylindrical UV at the hit, samples one of 4 procedural patterns + distance fog. The v-coord offsets by speed·time so the pattern flows toward you |

---

## Building and running

Every file is self-contained — no shared headers, one `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra raytracing/<file>.c -o <name> -lncurses -lm
```

Every file in this folder needs `-lm` (the solver, the shader, the
tone map all use libm).

**Universal keys** (every file):

| Key | Action |
|---|---|
| `q` / `ESC` | quit |
| `p` / `space` | pause / resume |
| `r` / `R`   | reset |
| `+` / `=`   | zoom in / faster |
| `-`         | zoom out / slower |
| `t` / `T`   | next / previous theme |

**Common per-file keys:**

| Key | Action | Files |
|---|---|---|
| `s` | cycle shade mode (phong / normals / fresnel / depth) | `sphere_raytrace`, `cube_raytrace`, `capsule_raytrace`, `torus_raytrace` |
| `d` / `D` | cycle debug overlay | `cube_raytrace` (axis / interval / face-uv), `capsule_raytrace` (hit-type / axial / discrim), `path_tracer` (NORMAL / ALBEDO / DEPTH), god-rays (SCATTER / SURFACE / TR) |
| `n` / `N` | next / previous pattern or preset | `saturn`, `solar_eclipse`, `tunnel`, god-rays |
| `1` .. `5` | family / preset select | various |
| `[` / `]` | sim Hz down / up | most files |

See each file's header for the full key map.

The 20-PBR-theme system in the four single-primitive files
(`sphere_raytrace`, `cube_raytrace`, `capsule_raytrace`,
`torus_raytrace`) is identical: 12 metals (gold, silver, copper,
bronze, brass, platinum, titanium, iron, steel, chrome, mercury,
aluminium) + 4 gems (ruby, emerald, sapphire, amethyst) +
3 dielectrics (plastic, glass, ceramic) + 1 emissive (neon). The
theme defines `albedo / specular / emissive / diffuse_weight`; the
shader reads those fields and produces RGB.

---

## Adding a new raytracing demo

To add (say) a new analytic primitive or a multi-object scene:

1. **Pick the closest existing file as the template.**
   * Single primitive with a closed-form solver → copy
     `sphere_raytrace.c` and replace `§5` with your `ray_X`.
   * Composite primitive (your shape = N analytic pieces) → copy
     `capsule_raytrace.c` and write N sub-tests + a dispatcher.
   * Multi-object scene with hand-authored shading → copy
     `saturn_with_rings.c`.
   * Global illumination / many bounces → copy `path_tracer.c` and
     replace the Cornell-Box scene definition in `§5`.
   * Volumetric scene → copy `forest_god_rays.c` or
     `god_rays_window.c`; replace the visibility predicate
     (`scene_blocked_to_sun`) with your own occluder geometry.
2. **Write `ray_YOUR_PRIMITIVE`** following the contract above —
   take `(ro, rd, params, *t_hit)` and return `true` + write `t_hit`
   on the first valid intersection. If your shape doesn't admit a
   closed-form intersection, it belongs in
   [`../raymarcher/`](../raymarcher/) instead.
3. **Add CONCEPTS + MENTAL MODEL** per [`../CLAUDE.md`](../CLAUDE.md).
   Both blocks are mandatory.
4. **Verify**: `-Wall -Wextra` clean, stable 60 fps, `q` / `ESC` exits
   cleanly, `SIGWINCH` doesn't crash, HUD shows fps + state.

If your new primitive has a closed-form normal (sphere, plane, AABB,
torus), use it directly — no finite-difference needed. That's the
analytic raytracer's headline advantage over the raymarcher.

See [`../documentation/Architecture.md`](../documentation/Architecture.md)
for the framework-wide loop / clock / signal conventions every file shares,
and [`../documentation/Master.md`](../documentation/Master.md) for the
deep-dive essays on ray-primitive math and the path-tracing rendering
equation.
