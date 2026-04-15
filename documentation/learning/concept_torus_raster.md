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

## From the Source

**Algorithm:** Software rasterization pipeline (GPU-pipeline emulation in C). The 7-stage pipeline runs every frame: (1) tessellate mesh once at init; (2) vertex shader: object→world→clip space; (3) perspective divide: clip → NDC → screen; (4) back-face culling; (5) rasterize bounding box with barycentric test; (6) Z-test against float z-buffer; (7) fragment shader: Phong/toon/normal/wireframe.

**Math:** Barycentric coordinates `(λ₀, λ₁, λ₂)`: `λᵢ = signed_area(edge_i) / total_area`. All λ ∈ [0,1] and sum to 1 iff inside the triangle. **Perspective-correct interpolation** requires interpolating `z⁻¹` and then dividing — linear interpolation in screen space is not perspective-correct.

**Performance:** Z-buffer resolves occlusion without sorting triangles. Back-face culling halves triangle count for closed meshes. **Function pointers** (`vert_shader`, `frag_shader`) allow swapping all 4 shader pairs without changing the pipeline structure.

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

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app.scene.mesh.verts` | `Vertex*` (heap, TESS_U×TESS_V vertices) | ~30 KB | Per-vertex position, normal, and UV for the tessellated torus |
| `g_app.scene.mesh.tris` | `Triangle*` (heap, TESS_U×TESS_V×2 triangles) | ~12 KB | Triangle index triples into the vertex array |
| `g_app.fb.zbuf` | `float*` (heap, cols×rows) | ~varies | Per-fragment depth for z-test (FLT_MAX = empty) |
| `g_app.fb.cbuf` | `Cell*` (heap, cols×rows) | ~varies | Per-cell character and colour pair ready for blitting |
| `k_bayer[4][4]` | `const float[4][4]` | 64 B | 4×4 ordered dither matrix for softening ramp banding |

# Pass 2 — torus_raster: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | TESS_U=32, TESS_V=24, TORUS_R=0.65, TORUS_r=0.28, CAM_FOV=60°, CAM_DIST=3.2, WIRE_THRESH=0.08, CELL_W=8, CELL_H=16, Paul Bourke ramp, Bayer matrix |
| §2 math | Vec3/Vec4/Mat4; m4_perspective, m4_lookat, m4_normal_mat, m4_rotate_x/y |
| §3 shaders | VSIn/VSOut/FSIn/FSOut; 4 shader pairs (phong/toon/normals/wire); Uniforms; ToonUniforms |
| §4 mesh | `Vertex` + `Triangle` + `Mesh`; `tessellate_torus()` |
| §5 framebuffer | `Framebuffer` zbuf+cbuf; `luma_to_cell()`; `fb_blit()` |
| §6 pipeline | `barycentric()`; `pipeline_draw_mesh()` |
| §7 scene | `Scene` owns Mesh + ShaderProgram + Uniforms; tick/draw/next_shader |
| §8 screen | ncurses init/resize/HUD/present |
| §9 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
tessellate_torus() — called once at init:
  for i in 0..TESS_U, j in 0..TESS_V:
    theta = 2π * i/U; phi = 2π * j/V
    pos = ((R + r*cos(phi))*cos(theta),
            r*sin(phi),
           (R + r*cos(phi))*sin(theta))
    ring_centre = (R*cos(theta), 0, R*sin(theta))
    normal = normalize(pos - ring_centre)   ← outward tube normal
  Build triangles: quad grid → 2 triangles per cell

scene_tick(dt_sec):
  angle_y += ROT_Y * dt_sec
  angle_x += ROT_X * dt_sec
  uni.model    = m4_rotate_y(angle_y) * m4_rotate_x(angle_x)
  uni.mvp      = uni.proj * uni.view * uni.model
  uni.norm_mat = m4_normal_mat(uni.model)   ← cofactor for correct normal transform
  toon_uni.base = uni   ← keep in sync

scene_draw(fb):
  fb_clear(fb)   ← zbuf = FLT_MAX, cbuf = all zero
  pipeline_draw_mesh(fb, mesh, shader, is_wire)
  fb_blit(fb)

pipeline_draw_mesh(fb, mesh, sh, is_wire):
  for each triangle:
    // 1. Vertex shader (×3)
    for vi in 0..2:
      in.pos = mesh.verts[tri.v[vi]].pos
      in.normal = mesh.verts[tri.v[vi]].normal
      if is_wire: in.u = wire_u[vi]; in.v = wire_v[vi]  ← bary injection
      else:       in.u = vtx.u; in.v = vtx.v
      sh.vert(&in, &vo[vi], sh.vert_uni)
      if is_wire:
        vo[vi].custom[0] = wire_u[vi]   ← (1,0,0)/(0,1,0)/(0,0,1)
        vo[vi].custom[1] = wire_v[vi]
        vo[vi].custom[2] = 1 - wire_u[vi] - wire_v[vi]

    // 2. Clip reject
    if all 3 vertices behind near plane: continue

    // 3. Perspective divide → screen
    for vi in 0..2:
      w = vo[vi].clip_pos.w
      sx[vi] = ( vo[vi].clip_pos.x/w + 1) * 0.5 * cols
      sy[vi] = (-vo[vi].clip_pos.y/w + 1) * 0.5 * rows    ← Y flip
      sz[vi] = vo[vi].clip_pos.z / w

    // 4. Back-face cull (CCW → positive area = front-facing)
    area = (sx[1]-sx[0])*(sy[2]-sy[0]) - (sx[2]-sx[0])*(sy[1]-sy[0])
    if area <= 0: continue

    // 5. Bounding box
    x0..x1, y0..y1 = clamped bbox of projected triangle

    // 6. Rasterize
    for py in y0..y1:
      for px in x0..x1:
        b[3] = barycentric(sx, sy, px+0.5, py+0.5)
        if any b[i] < 0: continue    ← outside triangle

        z = b[0]*sz[0] + b[1]*sz[1] + b[2]*sz[2]
        if z >= zbuf[py*cols+px]: continue   ← z-test
        zbuf[py*cols+px] = z

        // Interpolate VSOut → FSIn
        fsin.world_pos = bary_blend(vo[0..2].world_pos, b)
        fsin.world_nrm = normalize(bary_blend(vo[0..2].world_nrm, b))
        fsin.u/v = bary_blend
        fsin.custom[c] = bary_blend(vo[0..2].custom[c], b)
        fsin.px = px; fsin.py = py

        // Fragment shader
        sh.frag(&fsin, &fsout, sh.frag_uni)
        if fsout.discard: continue

        // luma → cell
        luma = 0.2126*r + 0.7152*g + 0.0722*b   ← Rec.709
        cbuf[py*cols+px] = luma_to_cell(luma, px, py)

luma_to_cell(luma, px, py):
  thr = k_bayer[py&3][px&3]   ← 4×4 Bayer threshold [0,1]
  d = clamp(luma + (thr-0.5)*0.15, 0, 1)
  idx = d * (BOURKE_LEN-1)
  ch = k_bourke[idx]
  cp = 1 + d*6  (warm at bright, cool at dark)
  bold = d > 0.6
  return Cell{ch, cp, bold}
```

---

## Function Breakdown

### tessellate_torus()
Purpose: build vertex and triangle arrays from torus parametric equations
Steps:
1. For i in 0..TESS_U, j in 0..TESS_V:
   - `theta = 2π*i/TESS_U` (around ring), `phi = 2π*j/TESS_V` (around tube)
   - `pos = ((R+r*cos(phi))*cos(theta), r*sin(phi), (R+r*cos(phi))*sin(theta))`
   - `ring_centre = (R*cos(theta), 0, R*sin(theta))`
   - `normal = normalize(pos - ring_centre)` (outward from tube axis)
2. Build quad grid triangles: (i,j)→(i+1,j)→(i,j+1) and (i+1,j)→(i+1,j+1)→(i,j+1)

---

### frag_phong(in, out, uni)
Purpose: Blinn-Phong shading with gamma correction
Steps:
1. N = normalize(in.world_nrm)
2. L = normalize(uni.light_pos - in.world_pos)
3. V = normalize(uni.cam_pos - in.world_pos)
4. H = normalize(L + V)  ← Blinn half-vector
5. diff = max(0, N·L)
6. spec = max(0, N·H)^shininess
7. color = ambient + obj_color * light_col * diff + spec * 0.5
8. Gamma: `output = clamp(color, 0, 1) ^ (1/2.2)`

---

### frag_toon(in, out, toon_uni)
Purpose: quantize diffuse into discrete bands
Steps:
1. diff = max(0, N·L)
2. banded = floor(diff * bands) / bands  ← step to N bands
3. spec = (N·H > 0.94) ? 0.7 : 0.0  ← binary specular
4. color = obj_color * (banded + 0.12) + spec

---

### frag_wire(in, out)
Purpose: discard interior fragments, keep only edges
Steps:
1. b0=custom[0], b1=custom[1], b2=custom[2]
2. edge = min(b0, b1, b2)
3. If edge > WIRE_THRESH: discard=true; return
4. t = edge / WIRE_THRESH  ← 0 at edge centre, 1 at threshold
5. color = (0.9-t*0.3, 0.9-t*0.3, 0.9-t*0.3)  ← slight fade from edge

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  fb_alloc(fb, cols, rows)
  scene_init(scene, cols, rows)   ← tessellate_torus + shader setup + matrices
  frame_time = clock_ns()

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       fb_free + fb_alloc(new size)
       scene_rebuild_proj(new cols, rows)   ← recompute proj matrix aspect
       need_resize = 0

  2. dt (capped at 100ms)

  3. physics ticks:
     while sim_accum >= tick_ns:
       scene_tick(dt_sec)   ← update angle, model, mvp, norm_mat
       sim_accum -= tick_ns

  4. draw:
     erase()
     scene_draw(fb)         ← fb_clear → pipeline_draw_mesh → fb_blit
     screen_draw_hud()
     wnoutrefresh + doupdate

  5. input:
     q/ESC   → quit
     space   → toggle paused
     s / S   → scene_next_shader() → cycle phong→toon→normals→wire
```

---

## Interactions Between Modules

```
App
 └── owns Scene + Framebuffer (allocated separately from scene)

Scene
 ├── mesh: (nu+1)*(nv+1) vertices, nu*nv*2 triangles — computed once
 ├── shader: current vert/frag function pointers + uniforms pointer
 └── scene_tick():
      update model = rotate_y * rotate_x
      update mvp = proj * view * model
      update norm_mat = cofactor(model 3x3)

pipeline_draw_mesh():
  For each tri: vert shader × 3 → clip → screen → cull → rasterize → frag → luma_to_cell

luma_to_cell():
  Bayer dither → Paul Bourke ramp → warm/cool color pair
  Called once per drawn fragment
```
