# Pass 2 — wireframe: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | SIM_FPS_DEFAULT=30, MAX_VERTS=400, MAX_EDGES=800, SHAPE_COUNT=4, CAM_DIST=5, FILL=0.82, CELL_AR=2.0, ROT_X/Y_DEF, ROT/ZOOM limits |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 4 color pairs: COL_CUBE(cyan) COL_SPHERE(green) COL_PYRAMID(yellow) COL_TORUS(magenta) |
| §4 vec3 | `Vec3` + `v3()`, `v3mul()` — value types, no heap |
| §5 project | `rot_yx()`, `project_to_screen()`, `fov_from_screen()` |
| §6 canvas | `Canvas` heap framebuffer; `canvas_line()` Bresenham with slope chars; `canvas_draw()` |
| §7 shapes | `Shape` struct; vertex/edge tables for cube/sphere/pyramid/torus |
| §8 scene | `Scene` owns 4 shapes + canvas; `scene_tick/render/draw` |
| §9 screen | ncurses init/resize/present/HUD |
| §10 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
scene_render(scene):
  canvas_clear(canvas)

  fov = fov_from_screen(cols, rows, zoom)
  ox = cols/2; oy = rows/2

  sh = shapes[active]

  // project all vertices
  for each vertex i in sh.verts:
    v = rot_yx(verts[i], rx, ry)
    proj[i] = project_to_screen(v, fov, ox, oy)
      → clip_pos.col = ox + v.x * fov / (CAM_DIST - v.z)
      → clip_pos.row = oy - v.y * fov / (CAM_DIST - v.z) / CELL_AR
      → z = v.z (for clipping check)

  ch_override = curved ? 'o' : 0

  // draw each edge
  for each edge (a, b) in sh.edges:
    pa = proj[a]; pb = proj[b]
    if pa.z < -CAM_DIST+0.1 or pb.z < -CAM_DIST+0.1: skip
    canvas_line(canvas, pa.col, pa.row, pb.col, pb.row, ch_override, sh.color)

canvas_line(c, x0, y0, x1, y1, ch_override, col):
  // Slope-based char selection (when ch_override == 0):
  slope = |dy| / |dx|
  if dx == 0:           slope_ch = '|'
  elif dy == 0:          slope_ch = '-'
  elif slope < 0.5:      slope_ch = '-'
  elif slope < 2.0:      slope_ch = (sx==sy) ? '\\' : '/'
  else:                  slope_ch = '|'

  // Standard Bresenham, all 8 octants
  while not done:
    canvas_set(x0, y0, slope_ch, col)
    advance x0/y0 by Bresenham error step
```

---

## Function Breakdown

### rot_yx(p, rx, ry) → Vec3
Purpose: rotate 3D point — first Y-axis, then X-axis
Steps:
1. Y rotation: x1 = x*cos(ry) + z*sin(ry); z1 = -x*sin(ry) + z*cos(ry)
2. X rotation: y2 = y*cos(rx) - z*sin(rx); z2 = y*sin(rx) + z*cos(rx)

---

### project_to_screen(p, fov_px, ox, oy) → P2{col, row, z}
Purpose: perspective project 3D world point → terminal cell coordinates
Steps:
1. denom = CAM_DIST - p.z  (depth from camera)
2. If denom < 0.01: return invalid (-1, -1, -9999)  ← behind camera
3. scale = fov_px / denom
4. col = ox + p.x * scale
5. row = oy - p.y * scale / CELL_AR  ← /CELL_AR compensates for tall cells

---

### fov_from_screen(cols, rows, zoom) → float
Purpose: compute fov so shape fills FILL fraction of screen
Steps:
1. fov_rows = FILL * (rows/2) * CAM_DIST  ← from row height
2. fov_cols = FILL * (cols/2) * CAM_DIST / CELL_AR  ← from col width adjusted
3. fov = min(fov_rows, fov_cols)  ← use smaller so shape fits both dims
4. return fov * zoom

---

### shape_build_cube(shape)
Purpose: 8 vertices at ±1 on all axes, 12 edges
Steps:
1. verts[i] = (i&1 ? 1 : -1, i&2 ? 1 : -1, i&4 ? 1 : -1) for i in 0..7
2. edges: 4 front face + 4 back face + 4 pillars connecting them

---

### shape_build_sphere(shape)
Purpose: latitude/longitude grid tessellation
Steps:
1. For st in 0..SPHERE_STACKS, sl in 0..SPHERE_SLICES-1:
   - phi = π * st / STACKS; theta = 2π * sl / SLICES
   - vert = (sin(phi)*cos(theta), cos(phi), sin(phi)*sin(theta))
2. Longitude lines: edges connecting (st, sl) → (st+1, sl)
3. Latitude rings: edges connecting (st, sl) → (st, (sl+1)%SLICES)
4. curved = true → renders with 'o' dots (not slope chars)

---

### shape_build_torus(shape)
Purpose: parametric torus mesh
Steps:
1. For i in 0..TORUS_MAJOR-1, j in 0..TORUS_MINOR-1:
   - phi = 2π*i/MAJOR; theta = 2π*j/MINOR
   - vert = ((R + r*cos(theta))*cos(phi), r*sin(theta), (R + r*cos(theta))*sin(phi))
2. Tube edges: (i,j) → (i, (j+1)%MINOR)
3. Ring edges: (i,j) → ((i+1)%MAJOR, j)
4. curved = true → renders with 'o' dots

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  scene_init(cols, rows)   ← builds all 4 shapes + allocs canvas
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       scene_resize(new cols, rows)   ← realloc canvas
       reset sim_accum, frame_time

  2. dt (capped at 100ms)

  3. physics ticks:
     sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec)   ← rx += rot_x*dt; ry += rot_y*dt
       sim_accum -= tick_ns

  4. alpha = sim_accum / tick_ns   ← computed but unused (wireframe has no lerp)

  5. scene_render(scene)   ← project + draw edges into canvas

  6. FPS counter

  7. frame cap: sleep to 60fps

  8. draw:
     screen_draw():
       erase()
       canvas_draw(canvas)   ← write non-empty cells with COLOR_PAIR + A_BOLD
       HUD: fps, shape name, rot_y speed, zoom
     wnoutrefresh + doupdate

  9. input:
     q/ESC       → quit
     Tab         → active = (active+1) % 4; reset rx=0.4, ry=0.6
     space       → toggle paused
     ] / [       → rot_x, rot_y *= / /= ROT_STEP (clamped to ROT_MIN/MAX)
     = or + / -  → zoom *= / /= ZOOM_STEP (clamped to ZOOM_MIN/MAX)
```

---

## Interactions Between Modules

```
App
 └── owns Scene (4 shapes + canvas + rotation state)

Scene
 ├── shapes[4]: vertices + edges precomputed at init
 ├── canvas: heap-allocated [rows*cols] char+color grid, cleared each render
 └── scene_render():
      rot_yx() per vertex
      project_to_screen() per vertex
      canvas_line() per edge (Bresenham into canvas)

§5 project
 └── fov_from_screen() accounts for CELL_AR (2.0) twice:
     once in fov_cols (corrects horizontal vs vertical scale)
     once in project_to_screen (divides row by CELL_AR)

§6 canvas
 └── canvas_line() uses fixed slope_ch for entire edge:
     computed once from (adx, ady, sx, sy) at line start
     NOT recomputed per step — so all cells in one edge share one char
```

---

## Slope Character Selection

```
canvas_line slope decision (when ch_override == 0):
  adx = |x1-x0|, ady = |y1-y0|
  slope = ady / adx

  slope < 0.5   → '-'       (nearly horizontal)
  0.5 ≤ slope < 2.0 and sx==sy → '\\'  (going down-right or up-left)
  0.5 ≤ slope < 2.0 and sx!=sy → '/'  (going down-left or up-right)
  slope ≥ 2.0   → '|'       (nearly vertical)

  dx==0: '|'
  dy==0: '-'

For curved shapes (sphere, torus):
  ch_override = 'o' → all cells use 'o' regardless of slope
  Reason: arc slope changes along each curved edge — using slope chars
  would produce visual noise as characters change mid-arc
```

---

## Aspect Ratio Correction

```
Without correction:
  Screen y spans rows cells physically
  Screen x spans cols cells physically
  Terminal cells are ~2× taller than wide
  → A unit sphere projects as a vertical ellipse

With CELL_AR = 2.0:
  fov_from_screen takes min(fov_rows, fov_cols/CELL_AR):
    → ensures shape fits both dimensions in physical pixels

  project_to_screen divides row by CELL_AR:
    row = oy - p.y * scale / CELL_AR
    → compresses y by 2× so world-space circles appear round on screen
```
