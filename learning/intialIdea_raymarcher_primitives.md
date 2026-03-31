# Pass 2 — raymarcher_primitives: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_PRIMS=17, SIM_FPS_DEFAULT=24, CELL_ASPECT=2.0, RM_MAX_STEPS=100, RM_HIT_EPS=0.002, CAM_Z=4.5, FOV_HALF_TAN=0.65, LIGHT=(-3,3.5,2.5), KA/KD/KS/SHIN, ROT_Y_SPD_DEFAULT=0.60, ROT_X_RATIO=0.37 |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 8 luminance grey pairs; `lumi_attr()` |
| §4 vec2/vec3 | `V2`, `V3` + full vector math; `rot_y()`, `rot_x()`, `tumble()` |
| §5 sdf | 17 IQ SDF functions (pure math) |
| §6 wrappers | `Prim` table with name + function pointer; wrapper scales `s` → SDF params |
| §7 raymarch | `rm_cast_pixel()`, `rm_march()`, `rm_normal()`, `rm_shade()` |
| §8 canvas | same as raymarcher.c |
| §9 scene | `Scene` owns canvas + prim_idx + rot_y + rot_spd + prim_size |
| §10 screen | ncurses init/resize/present/HUD |
| §11 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
scene_tick(dt_sec):
  if paused: return
  rot_y += rot_spd * dt_sec
  rot_x  = rot_y * ROT_X_RATIO

canvas_render():
  for each pixel (px, py):
    intensity = rm_cast_pixel(px, py, prim_idx, prim_size, rot_y, rot_x)
    pixels[py*w+px] = (intensity >= 0) ? round(intensity*(RAMP_N-1)) : MISS

rm_cast_pixel(px, py, prim_idx, s, rot_y, rot_x):
  u, v = NDC coordinates
  phys_aspect = (ch * CELL_ASPECT) / cw
  ro = (0, 0, CAM_Z)
  rd = normalize(u*FOV, v*FOV*phys_aspect, -1)

  // SDF evaluator with rotation applied to query point
  sdf = lambda(p): k_prims[prim_idx].sdf(tumble(p, rot_y, rot_x), s)

  t = rm_march(ro, rd, sdf)
  if t < 0: return -1
  hit = ro + rd*t
  N = rm_normal(hit, sdf)   ← tetrahedron method
  return rm_shade(N, hit, ro, LIGHT)

tumble(p, ry, rx) → V3:
  return rot_x(rot_y(p, ry), rx)
  // Apply Y rotation first (fast spin), then X rotation (slow tilt)

rm_march(ro, rd, sdf) → float t:
  t = 0
  for i in 0..RM_MAX_STEPS-1:
    p = ro + rd*t
    d = sdf(p)
    if d < RM_HIT_EPS: return t
    if t > RM_MAX_DIST: return -1
    t += d
  return -1

rm_normal(p, sdf) → V3 (tetrahedron):
  e = RM_NORM_EPS
  k0=(+e,-e,-e), k1=(-e,-e,+e), k2=(-e,+e,-e), k3=(+e,+e,+e)
  n = k0*sdf(p+k0) + k1*sdf(p+k1) + k2*sdf(p+k2) + k3*sdf(p+k3)
  return normalize(n)
```

---

## Key SDF Implementations

```
sdf_sphere(p, r):
  return v3len(p) - r

sdf_box(p, b):      ← b is half-extents vector (V3)
  q = v3abs(p) - b
  return v3len(v3max0(q)) + min(max(q.x,q.y,q.z), 0)

sdf_round_box(p, b, r):
  return sdf_box(p, b) - r

sdf_torus(p, R, r):
  return v2len(v2(v2len(v2(p.x,p.z))-R, p.y)) - r
  // = sqrt( (sqrt(x²+z²) - R)² + y² ) - r

sdf_cylinder(p, h, r):
  d = v2abs( v2(v2len(v2(p.x,p.z)), p.y) ) - v2(r, h)
  return min(max(d.x,d.y), 0) + v2len(v2max0(d))
  // = 2D rounded-rect SDF in (radial, height) space

sdf_capsule(p, a, b, r):
  pa = p - a; ba = b - a
  h = clamp(dot(pa,ba)/dot(ba,ba), 0, 1)
  return len(pa - ba*h) - r
  // = distance to line segment, offset by r
```

---

## Prim Table Structure

```
Prim k_prims[17] = {
  {"sphere",       w_sphere},
  {"box",          w_box},
  {"round_box",    w_round_box},
  {"box_frame",    w_box_frame},
  {"torus",        w_torus},
  {"capped_torus", w_cap_torus},
  {"link",         w_link},
  {"cone",         w_cone},
  {"plane",        w_plane},
  {"hex_prism",    w_hex_prism},
  {"capsule",      w_capsule},
  {"cylinder",     w_cylinder},
  {"round_cone",   w_round_cone},
  {"octahedron",   w_octahedron},
  {"pyramid",      w_pyramid},
  {"triangle",     w_triangle},
  {"quad",         w_quad},
}

Each wrapper (e.g. w_box):
  static float w_box(V3 p, float s) {
    return sdf_box(p, v3(s*0.70, s*0.70, s*0.70));
  }
```

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
       scene_tick(dt_sec)   ← rot_y += rot_spd*dt, rot_x = rot_y*RATIO
       sim_accum -= tick_ns

  4. canvas_render()

  5. FPS counter, frame cap

  6. draw:
     erase()
     canvas_draw(canvas, cols, rows)
     HUD: fps, prim name, size, rot_spd
     wnoutrefresh + doupdate

  7. input:
     q/ESC       → quit
     space       → toggle paused
     Tab / t     → prim_idx = (prim_idx+1) % N_PRIMS
     T           → prim_idx = (prim_idx-1+N_PRIMS) % N_PRIMS
     ] / [       → rot_spd *= / /= ROT_SPD_STEP (clamped)
     = or + / -  → prim_size *= / /= PRIM_SIZE_STEP (clamped)
     r / R       → rot_spd *= / /= ROT_SPD_STEP (same as ] / [)
```

---

## Interactions Between Modules

```
App
 └── owns Scene

Scene
 ├── canvas: ramp index or MISS per pixel
 └── canvas_render():
      for each pixel:
        rm_cast_pixel() passes sdf = lambda(p): k_prims[idx].sdf(tumble(p,...), s)
        rm_march() sphere traces with this sdf
        rm_normal() tetrahedron gradient
        rm_shade() fixed Phong (LIGHT is constant)

§6 prim table
 └── 17 function pointers, each wraps one §5 SDF
     wrapper scales s → good-looking proportions for that primitive
```

---

## V2 Dimension Reduction Pattern

```
Many 3D SDFs reduce to 2D for revolving or extruded shapes:

Cylinder (revolved around Y):
  → measure (radial_dist, y) instead of (x, y, z)
  → sdf_2d_rounded_rect(v2(len(v2(p.x,p.z)), p.y), v2(r, h))

Torus (revolved loop):
  → measure (radial_dist - major_R, y)
  → sdf_2d_circle(v2(len_xz - R, y), r)

Capsule (extruded endpoint-to-endpoint):
  → project onto segment
  → subtract tube radius

This pattern: "project to 2D, apply 2D SDF, subtract tube radius"
is how most curved 3D primitives are built.
```
