# raymarcher — sphere tracing on signed distance fields

A reference for the **raymarcher** paradigm of 3-D rendering. This folder
contains **8 self-contained C programs** that build a sphere-tracing
renderer from a 130-line single-sphere demo up to a full fractal
encyclopedia — every file re-using the same trace loop, the same
finite-difference normal trick, and the same Phong shader scaffold.

Raymarching answers the question *"for each terminal cell, how far
along this ray do I have to march before I hit the surface?"*. The
whole pipeline runs **field-first**: a Signed Distance Function (SDF)
`f(p) → R` tells you the distance to the nearest surface from any
point `p`, the trace loop steps forward by exactly that safe distance,
and where the SDF crosses zero you have your hit. Surface normals are
the **gradient** of the SDF, recovered by a 4-tap central difference.
The opposite poles of this paradigm live in
[`../raster/`](../raster/) (forward triangle rasterisation on an
explicit mesh) and [`../raytracing/`](../raytracing/) (analytic
ray ↔ primitive intersection). The same scene — a sphere lit by one
light — materialises as `raymarcher.c` here,
[`raster/sphere_raster.c`](../raster/sphere_raster.c) there, and
[`raytracing/sphere_raytrace.c`](../raytracing/sphere_raytrace.c) in the
third folder. Reading the trio side-by-side is the cleanest way to feel
the three paradigms — see also
[`raster/mandelbulb_raster.c`](../raster/mandelbulb_raster.c) vs
`mandelbulb.c` here for the most explicit raster/raymarcher contrast.

If you read **only one file**, read
[`raymarcher.c`](raymarcher.c) — it is the canonical exemplar and
every other file in the folder is downstream of its
`trace` + `normal` + `shade` skeleton.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive](#the-unifying-primitive)
   * [`f(p) → R` — the SDF](#fp--r--the-sdf)
   * [The sphere-tracing loop](#the-sphere-tracing-loop)
   * [Normal as gradient](#normal-as-gradient)
   * [Why every file looks the same](#why-every-file-looks-the-same)
3. [File index](#file-index)
4. [Building and running](#building-and-running)
5. [Adding a new raymarcher demo](#adding-a-new-raymarcher-demo)

---

## How to read this folder

The recommended path goes from the simplest possible SDF (one sphere)
through more sophisticated leaves (box, gallery of 17 primitives) to
composition (CSG operators, smooth blends) and finally to fractal SDFs
that are merely conservative lower bounds on the true distance.

```
        SINGLE PRIMITIVE              CATALOGUE              COMPOSITION                FRACTAL SDF
        ────────────────              ─────────              ───────────                ───────────
   1.  raymarcher.c           ──▶ 3. raymarcher_primitives.c ──▶ 5. sdf_gallery.c    ──▶ 7. mandelbulb.c
        sphere SDF                 17 primitives,            blend/twist/repeat/        non-linear iterated
        closed-form normal         function-pointer table    sculpt presets             distance estimator
                │                                                       │
                ▼                                                       ▼
   2.  raymarcher_cube.c          4. metaballs.c               6. csg_atlas.c       ──▶ 8. kifs_fractal.c
        box SDF                       6 spheres + smin            13-operator                Kaleidoscopic IFS
        tetra finite-diff normal      curvature colouring         catalogue                  (Sierpinski tetra /
                                                                                              Menger / rot crystal)
```

**Prerequisites graph.** Every file states *Study alongside:* in its
header:

* `raymarcher.c` is the **root** of the whole folder — single sphere,
  closed-form normal, no recursion, no composition. Read it before
  anything else.
* `raymarcher_cube.c` introduces the **tetrahedral 4-tap normal**
  (`∇f ≈ (f(p+ε)·e₀ + f(p−ε)·e₁ + ...)` over 4 carefully chosen ε
  directions) — needed once the surface no longer has a closed-form
  gradient.
* `raymarcher_primitives.c` shows that 17 different SDFs share **one
  marcher** — switching primitive changes only the function pointer
  in `prim_table[active]`.
* `sdf_gallery.c` varies the **branches** of the SDF tree (smooth-min
  blending, twist warping, domain repetition) instead of the leaves.
* `csg_atlas.c` is the **encyclopedia** — holds two operand primitives
  constant and sweeps a 13-entry catalogue of boolean operators
  organised as a 2-D table (4 families × 3-4 op kinds).
* `metaballs.c` is the **composition exemplar**: many sphere SDFs
  combined by a polynomial smooth-min, plus soft shadows and curvature
  colouring.
* `mandelbulb.c` and `kifs_fractal.c` are the **fractal pair**:
  what happens when the SDF stops being a true distance function and
  becomes a *conservative lower bound*. Both use the same marcher with
  a smaller step factor and extra orbit-trap colouring.

---

## The unifying primitive

Every file in this folder is built around exactly **one math object**
and **one trace loop**. Master those and the 8 files become 8
variations on the same theme.

### `f(p) → R` — the SDF

A Signed Distance Function maps a 3-D point to *the signed shortest
distance to the surface*: positive outside, negative inside, zero on
the surface. The sphere SDF is one line:

```c
static float sdf_sphere(V3 p, float r) {
    return v3_len(p) - r;
}
```

`p = (4, 0, 0)` on a unit sphere → `f = 3` (outside, 3 units away).
`p = (0.5, 0, 0)` → `f = -0.5` (inside, 0.5 units in). `p = (1, 0, 0)`
→ `f = 0` (on the surface). The SDF is the **entire** description of
the scene — no triangles, no vertices, no parametric equations. Every
file in this folder adds a different `f(p)`.

The most useful SDFs in this folder:

| SDF leaf | File | Closed form |
|---|---|---|
| sphere | every file | `|p| − r` |
| box / rounded box | `raymarcher_cube`, `raymarcher_primitives`, `sdf_gallery` | `max(|p| − h, 0) + min(max comp, 0)` |
| torus | `raymarcher_primitives`, `sdf_gallery` | `length((length(p.xz) − R, p.y)) − r` |
| capsule | `raymarcher_primitives`, `metaballs` | distance to a line segment − r |
| Mandelbulb DE | `mandelbulb.c` | iterated triplex-power escape, `0.5·log(r)·r / dr` |
| KIFS DE | `kifs_fractal.c` | iterated fold + contract, primitive at the end |

### The sphere-tracing loop

The trace loop is the **central abstraction** — every file calls a
variant of it. From `raymarcher.c`:

```
   march along the ray starting at t = 0:
   ─────────────────────────────────────
   t := 0
   for step in 1 .. MAX_STEPS:
       p := ro + t * rd
       d := sdf(p)
       if d < EPS:  return HIT  (t, p)         # close enough = on surface
       if t > T_MAX: return MISS                # walked off scene
       t := t + d                                # take the safe step
   return MISS                                   # ran out of steps
```

Why does stepping by `d` work? Because `d = sdf(p)` is **the distance
to the nearest surface from p**. Stepping any less is conservative
(no surface within `d` units in any direction). Stepping any more
could skip past a surface. Stepping exactly `d` is the largest
provably-safe step you can make — that is what "sphere tracing"
means (Hart 1996, *Sphere Tracing*).

For fractal SDFs (`mandelbulb.c`, `kifs_fractal.c`) the SDF is a
**conservative lower bound** on true distance, not the true distance
itself. The fix is a fudge factor: step by `0.5 · d` or `0.7 · d`
instead of `d`. Slower convergence, but no overshoot.

### Normal as gradient

Once `trace` returns a hit point `p` on the surface, the outward
normal is the **gradient of the SDF** at `p`, normalised:

```
n = normalise( ∇f(p) )
   = normalise( ( f(p+ε·x̂) − f(p−ε·x̂),
                  f(p+ε·ŷ) − f(p−ε·ŷ),
                  f(p+ε·ẑ) − f(p−ε·ẑ) ) )
```

This is the **6-tap central difference**. The cheaper **4-tap
tetrahedral** form (used by every file after `raymarcher.c`) probes
along the 4 vertices of a regular tetrahedron and reconstructs the
gradient algebraically — 4 SDF evaluations instead of 6, with the same
accuracy.

For the sphere alone the gradient has a closed form
(`n = p / |p|`) — that's why `raymarcher.c` doesn't bother with the
finite-difference. Every other file does.

### Why every file looks the same

The split — `sdf(p)` for geometry, `trace + normal + shade` for
optics — is the same across every file in the folder **because the
trace loop doesn't know what the surface is**. It knows only:

* `sdf(p)` returns a float for any point `p`;
* `sdf` is signed (positive outside, negative inside);
* the gradient of `sdf` exists almost everywhere and gives the normal.

Substitute a sphere, a box, a torus, a smoothly-blended set of spheres,
a Mandelbulb, a Sierpinski tetrahedron — the trace loop keeps working.
**The SDF changes and the shading layers on top. Everything else stays
still.** That invariance is what makes raymarching re-usable across
every demo in this folder.

The output pipeline — Phong (or N·V or Lambert) → gamma → 8-char or
92-char glyph ramp + theme palette — is shared across every file.
Pixel quality is decoupled from the SDF. Several files
(`sdf_gallery.c`, `metaballs.c`, `mandelbulb.c`) layer on **soft
shadows** (Quílez's running-min penumbra), **ambient occlusion** (an
inverse function of trace step count, or a separate AO march), and
**curvature** (Laplacian of the SDF) as additional shading channels.

---

## File index

| File | Description | What it adds vs. predecessor |
|---|---|---|
| `raymarcher.c` | one sphere, one light, Phong shading | the canonical sphere-tracing skeleton — closed-form sphere SDF, closed-form normal, Phong shade, debug overlays for normal / depth / step-count |
| `raymarcher_cube.c` | spinning cube, same Phong skeleton | swaps sphere SDF for the box SDF; introduces the **tetrahedral 4-tap finite-difference normal** (cube has no closed-form gradient); rotation-via-inverse-rotate-point trick |
| `raymarcher_primitives.c` | gallery of 17 primitives cycled by Tab | one trace loop, **function-pointer dispatch** into 17 SDFs (sphere, box, torus, cylinder, capsule, cone, octahedron, pyramid, hex prism, …); Floyd-Steinberg dither on the 92-char ramp |
| `metaballs.c` | 6 sphere-SDFs blended by smooth-min | **composition** — `smin(a, b, k)` polynomial blends multiple sphere SDFs into one organic surface; soft shadows, curvature-based colour bands, optional 2×2 SSAA |
| `sdf_gallery.c` | 5 scenes showing 5 composition operators | **branch operators** — animated blend (smin), boolean (union / intersect / subtract), twist (rotate p by p.y · k), domain repetition (mod-fold), sculpt (humanoid from 7 smin'd parts) |
| `csg_atlas.c` | 13-operator catalogue, 2 fixed operands | the **encyclopedia** of CSG seams — 2-D table of {HARD, SMOOTH, ROUND, CHAMFER} × {UNION, INTERSECT, SUBTRACT, XOR}, live parameter dialing of each family |
| `mandelbulb.c` | canonical p = 8 Mandelbulb | **fractal SDF** as conservative distance estimator (iterated triplex-power escape, `0.5·log(r)·r / dr`); smaller step factor, smooth-iteration colouring, AO from step count |
| `kifs_fractal.c` | Kaleidoscopic IFS — Sierpinski tetra / Menger sponge / rotating crystal | **fold + contract** iteration — non-iterating-point fractal SDFs built from reflections and scales; orbit-trap colouring (running min `|p|²`), per-preset fold dispatch |

---

## Building and running

Every file is self-contained — no shared headers, one `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra raymarcher/<file>.c -o <name> -lncurses -lm
```

Every file in this folder needs `-lm` (the SDF, the normal, the
shader, the tone map all use libm).

**Universal keys** (every file):

| Key | Action |
|---|---|
| `q` / `ESC` | quit |
| `space`     | pause / resume |
| `r` / `R`   | reset |
| `z` / `Z`   | zoom in / out |
| `t` / `T`   | next / previous theme |
| `d` / `D`   | cycle debug overlay (NORMAL / NORMALS / DEPTH / STEPS) |

**Common per-file keys:**

| Key | Action | Files |
|---|---|---|
| `Tab` / `n` | next primitive / preset | `raymarcher_primitives`, `kifs_fractal` |
| `1` .. `5`  | select scene / family | `sdf_gallery`, `csg_atlas` |
| `j` / `k`   | smoothing factor down / up | `metaballs` |
| `i` / `I`   | iteration depth − / + | `mandelbulb`, `kifs_fractal` |
| `s`         | toggle soft shadows | `sdf_gallery`, `metaballs` |
| `o`         | toggle ambient occlusion | `sdf_gallery` |
| `a`         | toggle 2×2 anti-aliasing | `metaballs` |
| arrows      | manual orbit (yaw / pitch) | fractal files |
| `+` / `-`   | parameter or speed | every file |
| `]` / `[`   | sim Hz or light orbit speed | most files |

See each file's header for the full key map. The four debug overlays
(`d` / `D`) are present in every file and visualise the four
intermediate signals of the trace: surface normal as RGB, normal lit
in grayscale, depth as inverse-grey, march step count as heat.

---

## Adding a new raymarcher demo

To add (say) a new fractal or a new CSG composition:

1. **Pick the closest existing file as the template.**
   * Single closed-form primitive → copy `raymarcher.c` or
     `raymarcher_cube.c`.
   * Multiple primitives → copy `raymarcher_primitives.c` and add an
     entry to `prim_table[]`.
   * Composition / boolean → copy `sdf_gallery.c` or `csg_atlas.c`.
   * Fractal / iterated DE → copy `mandelbulb.c` (single non-linear
     iteration) or `kifs_fractal.c` (linear fold + contract).
2. **Replace `§5 SDF`** with your distance function. Every other
   section (trace, normal, shade, scene, screen, app) carries over
   unchanged. If your SDF is a conservative lower bound, drop the
   step factor in `trace` from `1.0` to `0.5` (see `mandelbulb.c`).
3. **Add CONCEPTS + MENTAL MODEL** per [`../CLAUDE.md`](../CLAUDE.md).
   Both blocks are mandatory.
4. **Verify**: `-Wall -Wextra` clean, stable 60 fps, `q` / `ESC` exits
   cleanly, `SIGWINCH` doesn't crash, HUD shows fps + state.

If your new SDF has a closed-form gradient (sphere, plane, hyperplane),
use it — skip the 4-tap normal for that primitive only. The general
trace loop won't care.

See [`../documentation/Architecture.md`](../documentation/Architecture.md)
for the framework-wide loop / clock / signal conventions every file shares,
and [`../documentation/Master.md`](../documentation/Master.md) for the
deep-dive essays on SDF math.
