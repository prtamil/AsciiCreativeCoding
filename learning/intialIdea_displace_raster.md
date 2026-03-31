# Pass 2 — displace_raster: Pseudocode

## Module Map

Same sections as sphere_raster.c, with two key additions:

| Section | Difference from sphere_raster |
|---|---|
| §1 config | TESS_U=48, TESS_V=32, ROT_Y=0.30, ROT_X=0.12, WIRE_THRESH=0.09, CD_EPS=0.03 |
| §3 displacement | NEW: 4 displacement functions + tangent basis + normal recompute |
| §4 shaders | vert_displace replaces vert_default; same 3 frag shaders (phong/toon/normals/wire) |
| §8 scene | `disp_mode` added; scene_tick advances time; disp uniforms |
| All others | Same as sphere_raster.c |

---

## Data Flow Diagram

```
scene_tick(dt_sec):
  time += dt_sec
  angle_y += ROT_Y * dt_sec
  angle_x += ROT_X * dt_sec
  uni.model    = m4_rotate_y * m4_rotate_x
  uni.mvp      = proj * view * model
  uni.norm_mat = m4_normal_mat(model)
  uni.time = time

vert_displace(in, out, uni):
  // 1. Standard MVP transform
  out.clip_pos  = uni.mvp * in.pos
  out.world_pos = uni.model * in.pos
  N = normalize(uni.norm_mat * in.normal)   ← sphere normal in world space

  // 2. Evaluate displacement at model-space position
  pos_ms = in.pos   ← model space (unit sphere)
  d = k_disp_fn[disp_mode](pos_ms, uni.time, uni.amp, uni.freq)

  // 3. Offset position along world-space normal
  out.world_pos = out.world_pos + N * d

  // 4. Update clip_pos for displaced position
  out.clip_pos = uni.proj * uni.view * v4(out.world_pos, 1)

  // 5. Recompute normal via central difference
  out.world_nrm = recompute_normal(pos_ms, N, uni)
```

---

## Normal Recomputation

```
recompute_normal(pos_ms, N, uni) → Vec3:

  // Build tangent basis perpendicular to N
  (Tu, Tv) = make_tangent_basis(N)
  // make_tangent_basis:
  //   pick candidate = (0,1,0) if |N.x| > 0.9 else (1,0,0)
  //   Tu = normalize(cross(candidate, N))
  //   Tv = cross(N, Tu)

  eps = CD_EPS

  // Sample displacement at 4 nearby points along tangents
  // (in model space, on the sphere surface)
  du_pos = normalize(pos_ms + Tu*eps) * SPHERE_R   ← stay on sphere
  du_neg = normalize(pos_ms - Tu*eps) * SPHERE_R
  dv_pos = normalize(pos_ms + Tv*eps) * SPHERE_R
  dv_neg = normalize(pos_ms - Tv*eps) * SPHERE_R

  d_u = displace(du_pos, t, amp, freq) - displace(du_neg, t, amp, freq)
  d_v = displace(dv_pos, t, amp, freq) - displace(dv_neg, t, amp, freq)

  // Reconstruct displaced tangent vectors in world space
  Tu_w = uni.norm_mat * Tu    ← transform tangents to world space
  Tv_w = uni.norm_mat * Tv
  T_prime = Tu_w * (2*eps) + N * d_u
  B_prime = Tv_w * (2*eps) + N * d_v

  return normalize(cross(T_prime, B_prime))
```

---

## Displacement Functions

```
displace_ripple(pos, time, amp, freq):
  r = sqrt(pos.x² + pos.z²)          ← distance from Y axis
  taper = 1 - |pos.y| * 0.6          ← fade at poles
  return sin(time*2.5 + r*freq) * amp * taper

displace_wave(pos, time, amp, freq):
  phase = pos.x*freq + pos.y*freq*0.8 + pos.z*freq*0.5
  return sin(time*2.0 + phase) * amp

displace_pulse(pos, time, amp, freq):
  r = sqrt(pos.x² + pos.z²)
  breathe = sin(time*1.5)*0.85 + sin(time*4.5)*0.15
  falloff = exp(-r * freq * 0.4)
  return breathe * amp * falloff

displace_spiky(pos, time, amp, freq):
  f = freq * 1.4; t = time * 0.8
  val = |sin(pos.x*f + t) * sin(pos.y*f + t*0.7) * sin(pos.z*f + t*1.3)|
  return pow(val, 0.6) * amp
```

---

## Function Breakdown

### make_tangent_basis(N) → (Tu, Tv)
Purpose: construct two orthogonal tangent vectors perpendicular to N
Steps:
1. If |N.x| > 0.9: candidate = (0, 1, 0)  (avoid near-parallel)
   Else: candidate = (1, 0, 0)
2. Tu = normalize(cross(candidate, N))
3. Tv = cross(N, Tu)  ← already unit since N and Tu are orthonormal

---

### vert_displace vs vert_default
Purpose: displace vertex along normal + recompute normal
Additional steps beyond vert_default:
1. Sample displacement at current vertex
2. Offset world_pos by `N * d`
3. Recompute clip_pos from displaced world_pos
4. Replace world_nrm with recomputed_normal from central differences

---

## Pseudocode — Core Loop

```
Same as sphere_raster.c, with:

  physics ticks:
    while sim_accum >= tick_ns:
      scene_tick(dt_sec)   ← time += dt, update model, mvp, norm_mat, uni.time
      sim_accum -= tick_ns

  input additions:
    d / D   → disp_mode = (disp_mode + 1) % DM_COUNT
    c / C   → toggle back-face culling
    + / =   → cam_dist -= ZOOM_STEP
    -       → cam_dist += ZOOM_STEP
    s / S   → next shader (phong/toon/normals/wire)
```

---

## Interactions Between Modules

```
App
 └── owns Scene + Framebuffer

Scene
 ├── mesh: tessellated sphere (48×32, smooth normals) — computed once
 ├── disp_mode: which of k_disp_fn[4] to call
 ├── time: advances each tick
 └── scene_tick():
      advance time
      update model/mvp/norm_mat
      uni.time = time  ← passed to vert_displace via Uniforms

vert_displace (vertex shader):
  called once per vertex per frame
  evaluates k_disp_fn[disp_mode](pos, time, amp, freq)
  calls recompute_normal() which calls k_disp_fn 4 more times
  → 5 displacement evaluations per vertex total

Central difference requires displacement to be PURE:
  k_disp_fn[mode](same pos, same time) = same result always
  → displace_ripple/wave/pulse/spiky are all pure functions
  → cross-call consistency for central differences ✓
```
