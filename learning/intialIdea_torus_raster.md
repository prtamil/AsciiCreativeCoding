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
