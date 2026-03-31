# Pass 2 — raymarcher_cube: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | SIM_FPS_DEFAULT=24, CELL_W=1, CELL_H=1, CELL_ASPECT=2.0, RM_MAX_STEPS=80, RM_HIT_EPS=0.003, RM_NORM_EPS=0.001, CAM_Z=4.5, FOV_HALF_TAN=0.65, CUBE_H_DEFAULT=0.9, ROT_X/Y_DEFAULT, LIGHT_SPD_DEFAULT, KA=0.08/KD=0.72/KS=0.65/SHIN=50 |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 8 luminance grey pairs (235..255); `lumi_attr()` |
| §4 vec3 | `Vec3` + v3add/sub/mul/dot/len/norm/abs/max0 |
| §5 raymarch | `rm_rotate_yx()`, `sdf_box()`, `sdf_scene()`, `rm_normal()`, `rm_march()`, `rm_shade()`, `rm_cast_pixel()` |
| §6 canvas | same as raymarcher.c (CELL_W×CELL_H block upscale) |
| §7 scene | `Scene` owns canvas + rx,ry angles + rot_x,rot_y speeds + cube_h + light |
| §8 screen | ncurses init/resize/present/HUD |
| §9 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
scene_tick(dt_sec):
  if paused: return
  rx += rot_x * dt_sec
  ry += rot_y * dt_sec
  t  += dt_sec
  light = (cos(t * light_spd) * 3.5,
           sin(t * light_spd * 0.6) * 1.0 + 1.5, 3.0)

canvas_render(canvas, rx, ry, cube_h, light):
  for each pixel (px, py):
    intensity = rm_cast_pixel(px, py, w, h, rx, ry, cube_h, light)
    pixels[py*w+px] = (intensity >= 0) ? round(intensity*(RAMP_N-1)) : MISS

rm_cast_pixel(px, py, cw, ch, rx, ry, cube_h, light):
  u =  (px+0.5)/cw * 2 - 1
  v = -(py+0.5)/ch * 2 + 1
  phys_aspect = (ch * CELL_ASPECT) / cw
  ro = (0, 0, CAM_Z)
  rd = normalize(u*FOV, v*FOV*phys_aspect, -1)
  t = rm_march(ro, rd, rx, ry, cube_h)
  if t < 0: return -1
  hit = ro + rd*t
  N = rm_normal(hit, rx, ry, cube_h)   ← tetrahedron method
  return rm_shade(N, hit, ro, light)

sdf_box(p, h) → float:
  q = |p| - (h, h, h)        ← component-wise
  outside = |max(q, 0)|       ← to nearest face/edge/corner
  inside  = min(max(q.x, q.y, q.z), 0)   ← inside = negative
  return outside + inside

sdf_scene(p, rx, ry, cube_h) → float:
  lp = rm_rotate_yx(p, rx, ry)   ← rotate query point by inverse orientation
  return sdf_box(lp, cube_h)

rm_rotate_yx(p, rx, ry) → Vec3:
  // Y rotation
  x1 = p.x*cos(ry) + p.z*sin(ry)
  z1 = -p.x*sin(ry) + p.z*cos(ry)
  // X rotation
  y2 = p.y*cos(rx) - z1*sin(rx)
  z2 = p.y*sin(rx) + z1*cos(rx)
  return (x1, y2, z2)

rm_normal(p, rx, ry, cube_h) → Vec3 (tetrahedron method):
  e = RM_NORM_EPS
  k0=(+e,-e,-e), k1=(-e,-e,+e), k2=(-e,+e,-e), k3=(+e,+e,+e)
  d0=sdf_scene(p+k0,...), d1=sdf_scene(p+k1,...), ...
  n = k0*d0 + k1*d1 + k2*d2 + k3*d3
  return normalize(n)

rm_shade(N, hit, cam, light) → float:
  L = normalize(light - hit)
  V = normalize(cam - hit)
  ndl = max(0, N·L)
  R = 2*ndl*N - L
  spec = max(0, R·V) ^ SHIN
  return min(KA + KD*ndl + KS*spec, 1.0)
```

---

## Function Breakdown

### sdf_box(p, h)
Purpose: exact signed distance from p to axis-aligned cube half-extent h
Key insight:
- `q = |p| - h` gives signed excess in each dimension
- `max(q, 0)` clips to 0 inside → length = distance to nearest face/edge/corner if outside
- `min(max(q.x,q.y,q.z), 0)` = 0 if outside, negative distance to nearest face if inside
- Sum of these two terms = exact SDF

---

### rm_normal (tetrahedron method)
Purpose: estimate gradient of SDF at hit point p
Why tetrahedron instead of central differences:
- Central diff: 6 samples, all on axes → can straddle sharp edges of cube, averaging across them
- Tetrahedron: 4 samples at diagonal offsets → better at edges, fewer evaluations
Formula produces: n ≈ gradient(sdf) = surface normal direction

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  scene_init(cols, rows)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize → realloc canvas, reset accum

  2. dt (capped at 100ms)

  3. physics ticks:
     while sim_accum >= tick_ns:
       scene_tick(dt_sec)  ← rx += rot_x*dt, ry += rot_y*dt, t += dt
       sim_accum -= tick_ns

  4. canvas_render(canvas, rx, ry, cube_h, light)

  5. FPS counter, frame cap

  6. draw:
     erase()
     canvas_draw(canvas, cols, rows)
     HUD: fps, cube_h, rot_y, light_spd
     wnoutrefresh + doupdate

  7. input:
     q/ESC   → quit
     space   → toggle paused
     ] / [   → rot_x, rot_y *= / /= ROT_STEP (clamped)
     = or +  → cube_h *= SIZE_STEP (clamped)
     -       → cube_h /= SIZE_STEP (clamped)
     l / L   → light_spd *= / /= LIGHT_SPD_STEP (clamped)
```

---

## Interactions Between Modules

```
App
 └── owns Scene

Scene
 ├── canvas: CANVAS_MISS or ramp index per pixel
 └── canvas_render():
      rm_cast_pixel() per pixel
       → sdf_scene(p, rx, ry, cube_h) = sdf_box(rm_rotate_yx(p,...), cube_h)
       → rm_march() sphere traces
       → rm_normal() tetrahedron gradient estimate
       → rm_shade() Phong

Key difference from sphere raymarcher:
  Sphere: N = normalize(hit)   ← analytical
  Cube:   N = rm_normal(hit)   ← numerical (tetrahedron)
  This is because the box SDF gradient is discontinuous at edges/corners
```

---

## sdf_box Cases Illustrated

```
Point above a face (only z > h):
  q = (-, -, +)   (two negative, one positive)
  max(q,0) = (0, 0, q.z)
  outside = q.z             ← distance straight up to face
  inside  = 0               ← not inside
  Result: q.z ✓

Point at an edge (x > h, z > h):
  q = (-, +, +, ) or (+ ,-, +) etc.
  max(q,0) = (0, q.y, q.z)
  outside = sqrt(q.y² + q.z²)  ← distance to edge corner
  inside = 0
  Result: distance to edge ✓

Point at a corner (all > h):
  max(q,0) = q
  outside = |q|   ← 3D distance to corner
  Result: 3D Euclidean to corner ✓

Point inside:
  all q < 0
  outside = 0 (max(q,0) = 0)
  inside = min(max(q.x,q.y,q.z), 0) = max(q.x,q.y,q.z) < 0
  Result: negative = closest face distance from inside ✓
```
