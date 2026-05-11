# raster — software rasterisation in C, painted to ASCII

A reference for the **rasteriser** paradigm of 3-D rendering. This folder
contains **14 self-contained C programs** that build a forward triangle
rasteriser from a 130-line wireframe up to a deferred renderer with
shadow mapping, SSAO, screen-space edge detection and bloom — every file
re-using the same vertex-shader / fragment-shader / framebuffer skeleton
under the hood.

Rasterisation answers the question *"for each triangle in my mesh, which
terminal cells does it cover, and what colour are they?"*. The whole
pipeline runs **mesh-first**: tessellate once, then per frame walk each
triangle, find its pixel footprint with a bounding-box + barycentric
test, depth-test against a z-buffer, and run a fragment shader to pick
a glyph from the 92-character Bourke ramp. No rays. No SDFs. The
opposite poles of this paradigm live in [`../raymarcher/`](../raymarcher/)
(sphere-tracing on signed distance fields) and
[`../raytracing/`](../raytracing/) (analytic ray ↔ primitive
intersection). The same scene — a sphere lit by one light —
materialises as `sphere_raster.c` here,
[`raymarcher/raymarcher.c`](../raymarcher/raymarcher.c) there, and
[`raytracing/sphere_raytrace.c`](../raytracing/sphere_raytrace.c) in the
third folder. Reading the trio side-by-side is the cleanest way to feel
the three paradigms.

If you read **only one file**, read
[`cube_raster.c`](cube_raster.c) — it is the canonical exemplar and
every other file in the folder is downstream of its
`ShaderProgram` + `Framebuffer` + `pipeline_draw_mesh` skeleton.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive](#the-unifying-primitive)
   * [`ShaderProgram` + `Framebuffer`](#shaderprogram--framebuffer)
   * [The pipeline in five passes](#the-pipeline-in-five-passes)
   * [Why every file looks the same](#why-every-file-looks-the-same)
3. [File index](#file-index)
   * [Core skeleton — 5 files](#core-skeleton)
   * [G-buffer extensions — 5 files](#g-buffer-extensions)
   * [Geometry-extraction files — 3 files](#geometry-extraction-files)
   * [2-D screen-space outlier — 1 file](#2-d-screen-space-outlier)
4. [Building and running](#building-and-running)
5. [Adding a new raster demo](#adding-a-new-raster-demo)

---

## How to read this folder

The recommended path goes from the simplest possible mesh+line pipeline
(wireframe) up through forward Phong rasterisation, then deferred
shading, then image-space post passes. Each step adds **one new buffer
or one new pass** to the previous file.

```
        FORWARD                              DEFERRED + IMAGE PASSES
        ───────                              ───────────────────────
   1.  wireframe.c              ──▶   6.  deferred_rendering_pipeline.c
        Bresenham edges                    G-buffer split: geometry once,
        no z-buffer, no shading            light per-light
                │                                       │
                ▼                                       ▼
   2.  cube_raster.c            ──▶   7.  shadow_mapping.c
        ShaderProgram + zbuf               + light-view depth pass
        Bourke ramp + dither
                │                                       │
                ▼                                       ▼
   3.  sphere_raster.c                      8.  ssao_pipeline.c
        smooth normals on UV sphere           + screen-space AO
                │                                       │
                ▼                                       ▼
   4.  torus_raster.c                       9.  neon_edges.c
        parametric mesh                       + Sobel(depth, normal)
                │                                       │
                ▼                                       ▼
   5.  displace_raster.c                    10. bloom_finale.c
        animated vertex displacement          + emissive HDR + Gaussian
                                                bloom (capstone)

   GEOMETRY EXTRACTION (mesh built from a 3-D scalar field)
                ┌───────────────────┬─────────────────────────┐
                ▼                   ▼                         ▼
        11. marching_cubes.c   12. mandelbulb_raster.c   13. donut.c
        animated metaballs     sphere-projected fractal  Sloane analytic point-cloud
        re-meshed every frame   meshed once at startup    pedagogical rebuild

   OUTLIER (no 3-D pipeline at all)
                ▼
        14. sun_solar.c — pure 2-D screen-space layered noise + arcs
```

**Prerequisites graph.** Every file states *Study alongside:* in its
header. Follow those pointers:

* `cube_raster.c` is the root for **every shaded raster file**. It
  introduces `ShaderProgram` (function-pointer VS + FS with split
  uniforms), the cell buffer + z-buffer + Bourke ramp + Bayer dither
  paint stack, and the barycentric rasteriser. Everything after just
  swaps in a different mesh or adds a pass.
* `sphere_raster.c`, `torus_raster.c`, `displace_raster.c` are
  **mesh-only variations** of `cube_raster.c`. The pipeline is
  byte-for-byte identical; only `§4 mesh` (tessellator) and a few
  config constants change.
* `deferred_rendering_pipeline.c` introduces the **G-buffer split**
  (geometry pass writes pos / normal / albedo; lighting pass reads
  them). `shadow_mapping.c`, `ssao_pipeline.c`, `neon_edges.c`,
  `bloom_finale.c` all sit on top of that split and add **one extra
  pass each**.
* `bloom_finale.c` is the **capstone** that runs SSAO + bloom + Blinn-
  Phong together — read it last.
* `marching_cubes.c`, `mandelbulb_raster.c`, `donut.c` are
  **geometry-extraction** files: they show three different ways to
  turn a 3-D scalar field (metaballs / Mandelbulb / torus parametric)
  into renderable points or triangles, then feed them into the same
  raster pipeline.
* `wireframe.c` is the **introductory file** — small enough to read
  cold, no z-buffer, no fragment shader. Read first if rasterisation
  is unfamiliar.
* `sun_solar.c` is the **outlier**: a purely 2-D screen-space layered
  noise renderer that happens to live in this folder. Don't expect
  the pipeline to match.

---

## The unifying primitive

Every file (except `sun_solar.c` and the simple `wireframe.c`) is built
around exactly **one struct family** and **one ordered sequence of
passes**. Master those and the 14 files become 14 variations on the
same theme.

### `ShaderProgram` + `Framebuffer`

The shader interface (from `cube_raster.c`):

```c
typedef struct { Vec3 pos; Vec3 normal; float u, v; } VSIn;
typedef struct { Vec4 clip; Vec3 world_pos; Vec3 normal; ... } VSOut;
typedef struct { Vec3 world_pos; Vec3 normal; Vec3 bary; ... } FSIn;
typedef struct { Vec3 color; bool discard; } FSOut;

typedef struct {
    void  (*vert)(const VSIn  *in, VSOut *out, const void *uni);
    FSOut (*frag)(const FSIn  *in,             const void *uni);
    const void *vert_uni;   /* opaque uniforms passed to vert */
    const void *frag_uni;   /* opaque uniforms passed to frag */
} ShaderProgram;
```

Splitting `vert_uni` / `frag_uni` lets each shader see only the
uniforms it needs (the toon shader carries extra band-count fields
the phong shader doesn't). The cost of using the wrong uniform type
on the wrong side would be a segfault; the split prevents it.

The framebuffer is two arrays, sized to terminal `rows × cols`:

```c
typedef struct { char ch; int pair; int attr; } Cell;
typedef struct {
    float *zbuf;   /* one float per cell, cleared to +FLT_MAX */
    Cell  *cbuf;   /* one Cell per cell, cleared to ' ' */
    int    cols, rows;
} Framebuffer;
```

The whole rendering act between `erase()` and `doupdate()` is *fill
`cbuf` correctly, ignoring anything you couldn't see because `zbuf`
already had something closer*.

### The pipeline in five passes

```
   per frame:
   ──────────
   1. erase()
   2. fb_clear(fb)                   ── zbuf := +inf, cbuf := ' '
   3. for each Triangle T in Mesh:
        a. T.v[0..2] -> vert_shader -> VSOut[3]
        b. discard if any clip-w <= 0   (near-plane reject)
        c. perspective divide  -> NDC -> screen pixels
        d. signed-area back-face cull (sign chosen by §1 winding)
        e. for each pixel (px, py) in T.bounding_box:
             barycentric weights (w0, w1, w2)
             if any < 0: skip (outside triangle)
             interpolate world_pos / normal / uv with (w0, w1, w2)
             z_test against zbuf[py * cols + px]
             frag_shader(in) -> FSOut
             tone-map -> 6×6×6 RGB cube + Bourke ramp glyph + Bayer dither
             write Cell into cbuf
   4. blit cbuf to ncurses via mvaddch / colour pair
   5. HUD + hint strip
   6. wnoutrefresh(stdscr); doupdate();
```

This is **the** pipeline for every shaded file in the folder. The
deferred files split step 3 into two: a **geometry pass** that fills
parallel arrays (`g_pos`, `g_normal`, `g_albedo`, `g_zbuf`) instead of
writing colour directly, and a **lighting pass** that loops over those
arrays once per light. Image-space passes (SSAO, Sobel, bloom) read
the G-buffer arrays and write into auxiliary buffers (`g_ao`,
`g_edge`, `g_bloom`) that the final composite step combines.

### Why every file looks the same

The split — `ShaderProgram` for shading logic, `Framebuffer` for pixel
output — is the same across every shaded file in the folder **because
the pipeline doesn't know what the mesh is**. It knows only:

* a mesh is a flat array of `Vertex` (pos + normal + uv) and
  `Triangle` (3 indices);
* a vertex shader produces clip-space coordinates from a `VSIn`;
* a fragment shader returns RGB from an interpolated `FSIn`;
* a framebuffer holds depth and a `Cell` per pixel.

Substitute a sphere mesh, a torus mesh, a deformed sphere, an
extracted iso-surface — the pipeline keeps working. **The mesh changes
and the lighting passes layer on top. Everything else stays still.**
That invariance is what makes rasterisation re-usable across every
demo in this folder.

The output paint stack — continuous RGB → Reinhard tone map → gamma
→ 6×6×6 RGB cube quantisation → 92-char Bourke ramp glyph + Bayer
4×4 dither — is **identical to** the one used by every file in
[`../raytracing/`](../raytracing/). Pixel quality is decoupled from
the rendering paradigm.

---

## File index

### Core skeleton

| File | Description | What it adds vs. predecessor |
|---|---|---|
| `wireframe.c` | 4 shapes cycled, vertex + edge tables, Bresenham line draw | introductory — no z-buffer, no shading, no fragment shader |
| `cube_raster.c` | 12-tri cube with 4 shaders (phong / toon / normals / wire) | `ShaderProgram`, `Framebuffer` (zbuf + cbuf), barycentric raster, Bourke ramp + Bayer dither — the canonical exemplar |
| `sphere_raster.c` | UV sphere with smooth normals, 4 shaders | swaps the mesh for a UV-tessellated sphere; smooth per-vertex normals expose Phong's specular highlight |
| `torus_raster.c` | parametric torus, 4 shaders | mesh has closed-form normals via `n = normalise(p − ring_centre)` — no derivatives needed |
| `displace_raster.c` | UV sphere deformed by a time-varying scalar field | vertex shader pushes verts along normals; central-difference re-derives surface normal so lighting follows the deformation |

### G-buffer extensions

| File | Description | What it adds vs. predecessor |
|---|---|---|
| `deferred_rendering_pipeline.c` | RGB-light sphere — geometry pass writes pos / normal / albedo, lighting pass loops per-light | splits forward shading into **geometry-once + lighting-per-light**; up to 8 lights at no geometry cost |
| `shadow_mapping.c` | cube + sphere + floor with hard PCF shadows from a directional light | adds a **light-view depth pass** (Williams 1978); fragment shader compares its own light-space depth against the shadow map |
| `ssao_pipeline.c` | ziggurat + sphere + floor with screen-space ambient occlusion | adds an **image-space SSAO pass**: per-pixel hemispherical sampling + 3×3 blur; ambient is scaled by AO before Blinn-Phong |
| `neon_edges.c` | tron-style glowing edges on platonic shapes | adds a **Sobel filter** on the G-buffer's depth + normal channels; flagged edges drive HDR neon into the bloom pipeline |
| `bloom_finale.c` | emissive orb + cubes on a slate floor, SSAO + bloom + Blinn-Phong | capstone — every pass from earlier files runs in one frame: G-buffer → SSAO → lit + emissive → bloom extract → Gaussian H+V → composite |

### Geometry-extraction files

| File | Description | What it adds vs. predecessor |
|---|---|---|
| `marching_cubes.c` | 4 metaballs orbit; iso-surface re-extracted every frame | mesh is **rebuilt at 60 Hz** from a 3-D scalar field — genus can change in real time; gradient-derived normals; per-vertex colour barycentric blend |
| `mandelbulb_raster.c` | Mandelbulb fractal meshed once at startup, then rasterised | uses a **sphere-projection of distance-estimator rays** to extract the outer skin as triangles — captures only the convex hull, by design (compare with `../raymarcher/mandelbulb.c`) |
| `donut.c` | Andy Sloane's spinning donut, pedagogical rebuild | **analytic point-cloud rasterisation**: walks the torus parameterisation (θ, φ) and writes per-point z + glyph directly into the framebuffer — no triangles, no shaders. The pedagogical-refactor exemplar |

### 2-D screen-space outlier

| File | Description | What it adds vs. predecessor |
|---|---|---|
| `sun_solar.c` | sun with limb darkening, granulation (fBm), sunspots, corona, arc flares | **not a 3-D rasteriser** — purely 2-D layered screen-space noise + parabolic flare arcs; filed here because the shape (a circle on the screen) is trivial and all the work is in the radial layers, just like `donut.c` |

---

## Building and running

Every file is self-contained — no shared headers, one `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra raster/<file>.c -o <name> -lncurses -lm
```

Every file in this folder needs `-lm` (math throughout the pipeline).

**Universal keys** (every file):

| Key | Action |
|---|---|
| `q` / `ESC` | quit |
| `space`     | pause / resume |
| `+` / `=`   | zoom in |
| `-`         | zoom out |
| `r`         | reset (most files) |

**Common per-file keys:**

| Key | Action | Files |
|---|---|---|
| `s`         | cycle shader (phong → toon → normals → wire) | `cube_raster`, `sphere_raster`, `torus_raster`, `displace_raster`, `mandelbulb_raster` |
| `c`         | toggle back-face culling | every shaded mesh file |
| `t` / `T`   | next / previous theme | `wireframe`, `donut`, `sun_solar`, fractal files |
| `a`         | cycle SSAO / output mode | `ssao_pipeline`, `bloom_finale` |
| `b`         | toggle bloom | `bloom_finale`, `neon_edges` |
| `e`         | toggle edge detection | `neon_edges` |
| `l`         | extra lights | `deferred_rendering_pipeline` |
| `d` / `D`   | cycle debug overlay | `donut` (pedagogical-refactor file) |

See each file's header for the full key map.

---

## Adding a new raster demo

To add (say) a fractal terrain rasteriser or a fluid-surface mesh demo:

1. **Pick the closest existing file as the template.**
   * Single-mesh forward shading → copy `cube_raster.c`.
   * Multi-object scene → copy `deferred_rendering_pipeline.c`.
   * Image-space post effect on the G-buffer → copy `ssao_pipeline.c`
     and replace `§7 ssao` with your pass.
   * Mesh extracted from a 3-D scalar field → copy `marching_cubes.c`
     and replace the metaball field with yours.
2. **Replace `§4 mesh`** with your tessellator. Every other section
   (paint, gbuffer, lightpass, scene, screen, app) carries over unchanged.
3. **Add CONCEPTS + MENTAL MODEL** per [`../CLAUDE.md`](../CLAUDE.md).
   Both blocks are mandatory.
4. **Verify**: `-Wall -Wextra` clean, stable 60 fps, `q` / `ESC` exits
   cleanly, `SIGWINCH` doesn't crash, HUD shows fps + state.

Need a new fragment shader instead of new geometry? Add an entry to the
shader-program table in `§9 scene` and a `case` to the `s` key in
`§11 app`. The pipeline is shader-agnostic.

See [`../documentation/Architecture.md`](../documentation/Architecture.md)
for the framework-wide loop / clock / signal conventions every file shares.
