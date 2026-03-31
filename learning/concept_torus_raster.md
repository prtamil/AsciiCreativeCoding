# Pass 1 — torus_raster: Software Rasterizer (Torus)

## Core Idea

A complete software rasterization pipeline in ~1000 lines of C, rendering a spinning torus in the terminal. The pipeline mirrors a real GPU pipeline: vertex shader → clip/NDC/screen → back-face cull → rasterization (barycentric) → z-test → fragment shader → framebuffer → display. Four fragment shaders are cycled with Tab: Phong (diffuse+specular), Toon (cel shading), Normals (debug), Wireframe (edge detection via barycentric coordinates).

---

## The Mental Model

The GPU pipeline is implemented by hand in C:
1. **Tessellate** the torus into triangles once at startup (32×24 UV grid = 1536 triangles)
2. **Each frame**: rotate the model matrix, then for every triangle:
   - **Vertex shader** — transform vertices from model space to clip space (MVP matrix)
   - **Clip** — discard triangles behind the near plane
   - **Perspective divide** — clip → NDC → screen coordinates
   - **Back-face cull** — skip triangles facing away from camera (signed area < 0)
   - **Rasterize** — scan the bounding box, test barycentric coverage
   - **Z-test** — only draw if this triangle is closer than what's already in the zbuffer
   - **Fragment shader** — compute color (depends on shader mode)
   - **luma_to_cell** — convert luminance to Bayer-dithered Paul Bourke character
3. **Blit** the character buffer to the ncurses window

The design deliberately mirrors GLSL: separate `VSIn`/`VSOut`/`FSIn`/`FSOut` structs, `VertShaderFn`/`FragShaderFn` function pointers, a `Uniforms` struct with model/view/proj/MVP matrices.

---

## Data Structures

### `VSIn` / `VSOut` / `FSIn` / `FSOut`
Shader interface structs. `custom[4]` in VSOut/FSIn is a shader-specific interpolated payload:
- Phong/Toon: unused
- Normals: custom[0..2] = world normal X/Y/Z
- Wireframe: custom[0..2] = per-vertex barycentric coordinates

### `Uniforms`
| Field | Meaning |
|---|---|
| `model` | Model rotation matrix |
| `view` | Camera lookat matrix |
| `proj` | Perspective projection matrix |
| `mvp` | proj × view × model (precomputed per frame) |
| `norm_mat` | Cofactor of model's 3×3 block (for correct normal transform under non-uniform scale) |
| `light_pos/col, ambient` | Lighting params |
| `cam_pos, obj_color, shininess` | Phong params |

### `Framebuffer`
| Field | Meaning |
|---|---|
| `zbuf[cols*rows]` | Depth buffer (float, initialized to FLT_MAX) |
| `cbuf[cols*rows]` | Character buffer (Cell{ch, color_pair, bold}) |

---

## The Four Shaders

| Shader | vert | frag | Effect |
|---|---|---|---|
| Phong | vert_default | frag_phong | Blinn-Phong shading + gamma correction |
| Toon | vert_default | frag_toon | 4 discrete brightness bands + binary spec highlight |
| Normals | vert_normals | frag_normals | Maps world normal [-1,1] → RGB [0,1] |
| Wireframe | vert_wire | frag_wire | Barycentric edge detection — discard interior |

---

## Non-Obvious Decisions

### Normal matrix = cofactor of model 3×3
When the model matrix has non-uniform scale, naively transforming normals with the model matrix distorts them. The correct transform is `transpose(inverse(upper-left 3×3))`. The code computes the cofactor matrix (= adjugate) which equals inverse × determinant. Since we normalize afterwards, the determinant doesn't matter.

### Barycentric wireframe via custom[] payload
The wireframe shader doesn't trace lines — instead, each vertex gets a barycentric identity coordinate: vertex 0 = (1,0,0), vertex 1 = (0,1,0), vertex 2 = (0,0,1). These are interpolated across the triangle. At each fragment, `min(custom[0], custom[1], custom[2])` = distance to nearest edge. If this is below WIRE_THRESH, draw a line character; otherwise discard.

### luma_to_cell: Bayer dither + Paul Bourke ramp
Instead of simple quantization, luminance goes through:
1. Bayer 4×4 ordered dithering (position-dependent threshold)
2. Linear map into the 94-character Paul Bourke density ramp
3. Warm/cool color pair assignment by brightness

This produces smooth gradients that look good in the ASCII character density system.

### Aspect ratio: `(cols * CELL_W) / (rows * CELL_H)` for perspective
The projection matrix uses physical pixel aspect ratio, not cell count ratio. Without this, the torus ring appears as an ellipse instead of a circle.

---

## State Machine

No state machine. Continuous rotation. One boolean: `paused`.

---

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| `TESS_U` | 32 | Ring slices — more = smoother torus |
| `TESS_V` | 24 | Tube slices — more = rounder tube |
| `TORUS_R` | 0.65 | Major radius (ring centre to tube centre) |
| `TORUS_r` | 0.28 | Minor radius (tube radius) |
| `WIRE_THRESH` | 0.08 | Wireframe edge width in barycentric space |
| `ROT_Y` | 0.70 | Y-axis rotation speed |
| `ROT_X` | 0.28 | X-axis tilt speed |

---

## Open Questions for Pass 3

1. Remove back-face culling — what happens? (inside faces visible, torus looks hollow)
2. Set TESS_U=4, TESS_V=3 — does the torus still look like a torus or just a blocky polyhedron?
3. Replace Bayer dither with Floyd-Steinberg error diffusion — does quality improve?
4. Add ambient occlusion: compute average visibility in a cone around each normal — how to approximate it with the data available?
5. What happens if you use the wrong normal matrix (just model instead of norm_mat)?
