# Pass 1 — blackhole.c: Gargantua 3D (Schwarzschild geodesic ray tracer)

## Core Idea

Renders a physically accurate 3D Schwarzschild black hole — the same physics used
by Kip Thorne's DNGR (Double Negative Gravitational Renderer) for Interstellar.

For each terminal cell, a null geodesic (photon path) is integrated backward in
time from the camera through curved Schwarzschild spacetime using RK4.  The ray
terminates when it:
- **Hits the accretion disk** (crosses the y = 0 equatorial plane at DISK_IN ≤ r ≤ DISK_OUT)
- **Falls into the event horizon** (r < r_s)
- **Escapes to the background** (r > ESCAPE_R = 130 r_s, fixed constant)

The lensing table is built once at startup (~0.4 s).  Each animation frame is then
a fast table-lookup + relativistic Doppler colour computation as the disk rotates.

During integration, every ray also records its **closest approach radius** `min_r`.
Escaped rays with small `min_r` (grazed the photon sphere) draw the photon ring.

---

## Why 3D Changes Everything

The 2D version classified pixels by their screen-space distance from the center —
essentially drawing circles and bands.  It had no light bending at all.  The 3D
version traces actual curved photon paths.  The difference:

| Feature | 2D (fake) | 3D (geodesic) |
|---------|-----------|---------------|
| Shadow shape | Circle drawn by hand | Emerges from r < r_s cutoff |
| Upper arc | Hardcoded Gaussian formula | True secondary image of back disk |
| Disk curvature | Flat horizontal band | Wraps correctly; disk visible above AND below shadow |
| Photon ring | Approximate band | Emerges from min_r of escaped rays near critical impact |
| Doppler | Fake | Exact Keplerian β, relativistic beaming D = [(1+β)/(1−β)]^1.5 |

---

## Units and Geometry

All distances in **Schwarzschild units** with r_s = 1 (= 2GM/c²):

| Feature | Radius in r_s | Physical meaning |
|---------|---------------|-----------------|
| Event horizon | r = 1.0 | r_s — no light escapes inside |
| Photon sphere | r = 1.5 | Unstable circular photon orbit |
| ISCO | r = 3.0 | Innermost stable circular orbit (inner disk edge) |
| Disk outer | r = 12.0 | Outer disk edge (arbitrary) |
| Camera default | r = 38.0 | Default viewing distance; adjustable at runtime |
| Escape boundary | r = 130.0 | Fixed far-field; large enough to cover all cam_dist values |

---

## The Geodesic Equation

For null geodesics (light paths) in Schwarzschild spacetime, the acceleration of
a photon in 3D Cartesian-like embedding is:

```
d²pos/dλ² = −(3/2) · |pos × vel|² · pos / |pos|^5
```

where λ is the affine parameter, vel = dpos/dλ, and h = pos × vel is the specific
angular momentum vector.

**Derivation sketch:**

The Binet equation for Schwarzschild (with u = 1/r, M = r_s/2 = 0.5):
```
d²u/dφ² + u = 3M u²   →   d²u/dφ² = 3M u² − u
```

Converting from orbit-plane polar (u, φ) to 3D Cartesian with h = r² dφ/dλ:
```
d²r/dλ² = h²/r³ − 3Mh²/r⁴  =  h²/r³ (1 − 1.5/r)   (with r_s = 1)
```

In 3D vector form with h = |pos × vel|:
```
acc = (h²/r³ − 3h²/(2r⁴)) r̂  =  −(3h²/2r^5) pos
```

This gives the photon sphere at `acc = 0 → r = 1.5` ✓ (= 1.5 r_s).

**Why this formula works in 3D:**  Schwarzschild orbits are always planar (h is
conserved and defines the orbital plane).  The formula is therefore exact for all
orbital planes simultaneously — no need to project to 2D.

---

## RK4 Integration

State vector: (pos, vel) — 6 floats.

```
k1 = f(pos, vel)
k2 = f(pos + ds/2 · k1.dpos, vel + ds/2 · k1.dvel)
k3 = f(pos + ds/2 · k2.dpos, vel + ds/2 · k2.dvel)
k4 = f(pos + ds   · k3.dpos, vel + ds   · k3.dvel)

pos_new = pos + ds/6 · (k1 + 2k2 + 2k3 + k4)
vel_new = vel + ds/6 · (...)
```

**Adaptive step size:**
```
ds = clamp(r × 0.05, 0.003, DS_BASE=0.10)
```
Smaller steps when the ray is close to the black hole (high curvature), larger
when far away.  This keeps accuracy without wasting steps in flat-space regions.

**Why not normalize vel each step?**  Normalizing introduces numerical errors in
the direction derivative.  The RK4 error in |vel| is fourth-order in ds and stays
below 1e-6 for our step sizes.

---

## Closest Approach Tracking (min_r)

Every ray now tracks the minimum radius reached during integration:

```c
float mr = v3len(origin);
for each step:
    r = v3len(pos);
    if (r < mr) mr = r;
```

`mr` is stored in the `Cell` struct alongside the usual ray outcome.  It serves
one purpose: identifying rays that grazed the photon sphere and escaped — these
produce the photon ring.

---

## Photon Ring

The photon ring is the thin bright arc just outside the black shadow.  In
Schwarzschild geometry, rays with impact parameter slightly above the critical
value (b_crit = 3√3 × M ≈ 2.6 r_s) spiral many times around the photon sphere
before escaping, carrying heavily lensed light.  These rays have `min_r` close
to the photon sphere radius (r = 1.5).

Brightness formula:
```
ring_b = exp(−(min_r − PHOTON_R) × 2.4)
```

| min_r | ring_b | visual |
|-------|--------|--------|
| 1.5 (photon sphere) | 1.00 | `*` bold CP_RING |
| 1.8 | 0.49 | `+` CP_HOT |
| 2.2 | 0.23 | `.` CP_HOT |
| 3.2 (threshold) | 0.05 | not drawn |

The ring is not explicitly constructed — it emerges from the physics.  Rays in
a thin annulus of screen pixels happen to have small `min_r`; they all render
bright, producing the arc.

**Why the ring is asymmetric:**  The disk rotates (disk_angle advances each
tick).  The ring itself is symmetric (it depends only on geodesic geometry and
`min_r`), but the underlying disk brightness creates a brighter arc where the
disk approaches the observer.

---

## Equatorial Plane Crossing Detection

The accretion disk is in the y = 0 plane.  Sign-change detection:

```c
if (prev.y * pos.y < 0) {               /* sign changed → crossed y = 0 */
    t   = |prev.y| / (|prev.y| + |pos.y|)    /* fraction of step at crossing */
    hit = lerp(prev, pos, t)                  /* interpolated crossing point  */
    cr  = sqrt(hit.x² + hit.z²)              /* cylindrical radius           */
    if (DISK_IN ≤ cr ≤ DISK_OUT):
        phi = atan2(hit.z, hit.x)
        → DISK_HIT at (cr, phi)
}
```

**Why this catches both primary and secondary images:**

- **Primary image** (near disk): Ray starts at cam (y > 0), travels toward BH,
  crosses y = 0 from above (y: + → −).  Detected immediately.

- **Secondary image** (back disk → upper arc): Ray starts at cam (y > 0), curves
  around the photon sphere without crossing y = 0 on the near side, emerges from
  the far side and crosses y = 0 from below (y: − → +).  Also detected.

The same `prev.y * pos.y < 0` condition catches both.  The upper arc you see just
above the shadow is the secondary image automatically rendered by the geodesic
integrator — no special-case code needed.

---

## Doppler Beaming and Relativistic Color

For a disk element at (r, φ) rotating counterclockwise in Keplerian orbit:

```
v_disk = v_orb · (−sinφ, 0, cosφ)       where v_orb = √(M/r) = √(1/(2r))
```

The radial velocity component toward the observer (camera approximately at -z):
```
β = v_disk · n̂_cam = −v_orb · cosφ · cos(tilt)
```

- β > 0 at φ = π (left side): disk approaching → blue-shifted → **brighter**
- β < 0 at φ = 0 (right side): disk receding → red-shifted → **dimmer**

**Relativistic Doppler beaming factor** (intensity):
```
D = [(1+β)/(1−β)]^(3/2)
```
At v_orb = 0.41 (ISCO, r = 3 r_s):
- Bright side: D_max = [(1.41)/(0.59)]^1.5 ≈ **6.8×**
- Dim side:   D_min = [(0.59)/(1.41)]^1.5 ≈ **0.15×**
- Ratio: ~45×  (Interstellar used ~20-50× depending on r)

**Gravitational redshift** (additional dimming from climbing out of the potential):
```
g = √(1 − r_s/r) = √(1 − 1/r)
```
At r = 3 (ISCO): g = √(1 − 1/3) = √(2/3) ≈ 0.82

Combined: `bright = D × g × radial_profile × texture`

---

## ISCO Brightness Spike and Radial Profile

```c
float dr   = disk_r - DISK_IN;
float isco = exp(−dr² × 0.65);
float rad  = (1 − 0.86 × r_norm)^2.2 + 0.65 × isco;
```

The exponential spike models the plasma temperature maximum at the ISCO.  In
thin-disk models, T ∝ (r/r_ISCO)^{-3/4} with a peak just outside r_ISCO.
The spike makes the inner edge look distinct rather than blending into the band.

**Why exponent changed from 2.5 to 2.2:**  The steeper 2.5 exponent made the
outer disk (r_norm → 1) nearly invisible — the outer edge contribution fell to
(0.14)^2.5 ≈ 0.001.  With 2.2 the falloff is gentler: (0.14)^2.2 ≈ 0.003;
combined with the 0.86 prefactor this keeps the outer disk visibly populated.

---

## Camera Setup and Size Control

```
cam = (0, cam_dist × sin(tilt), −cam_dist × cos(tilt))
fwd = normalize(−cam)                  ← looking at origin
rgt = normalize(fwd × (0,1,0))        ← right
up  = rgt × fwd                       ← up (camera-space)
```

For each cell (col, row):
```
u = (col − cx)/cx × tan(FOV/2)
v = −(row − cy)/cx × tan(FOV/2) / ASPECT   ← divide by ASPECT for isotropic space
dir = normalize(fwd + u·rgt + v·up)
```

**Why divide by ASPECT instead of multiply:**  Terminal cells are ~2.1× taller
than wide (ASPECT = 0.47 = cell_w/cell_h).  In screen space, row differences are
physically larger.  Dividing v by ASPECT compresses the vertical ray direction
accordingly — making circular objects appear circular instead of oval.

**Tilt is fixed at 5°** — the slight inclination above the equatorial plane shows
both the near and far disk halves without going so high that the secondary arc
disappears.  It is not a runtime control (no lensing rebuild cost at every frame).

**cam_dist is runtime-adjustable** via `[+]` (closer → bigger) and `[-]` (farther
→ smaller).  Each change triggers a full lensing table rebuild (~0.4 s).  The HUD
shows `dist` — the number decreases when the black hole grows on screen.

---

## Dynamic Clip Radius

The screen-space clip circle prevents stray characters outside the disk boundary.
The clip radius scales with cam_dist so zooming in never cuts the disk edge:

```c
fov_h_tan = tan(FOV_DEG / 2)
clip_frac  = min((DISK_OUT / cam_dist) / fov_h_tan × 1.24, 0.96)
clip_r     = cx × clip_frac
```

This is derived from the angular fraction of the screen the disk edge occupies:
- `atan(DISK_OUT / cam_dist)` = the angle subtended by the outer disk edge
- Dividing by `tan(half-FOV)` gives the NDC fraction
- Factor 1.24 adds margin so the clip sits just outside the disk

| cam_dist | clip_frac | visual |
|----------|-----------|--------|
| 38 (default) | 54% | disk fits, no stray chars |
| 23 | 88% | zoomed in, disk still fits fully |
| 14 (closest) | 96% | capped at screen edge |

---

## Character and Colour Mapping

**9-level brightness gradient:**
```
bright ≥ 0.92 → '@'    (white-hot photon ring / ISCO spike)
bright ≥ 0.82 → '#'
bright ≥ 0.70 → '8'
bright ≥ 0.57 → '0'
bright ≥ 0.45 → 'O'
bright ≥ 0.33 → 'o'
bright ≥ 0.21 → '+'
bright ≥ 0.12 → ':'
otherwise     → '.'
```

**6 colour pairs per frame:**

| Pair | Name | Brightness range | Attribute |
|------|------|-----------------|-----------|
| CP_RING | photon ring / white-hot | > 0.85 | A_BOLD |
| CP_HOT | inner disk | 0.67–0.85 | A_BOLD if r_norm < 0.25 |
| CP_WARM | mid-inner | 0.50–0.67 | A_NORMAL |
| CP_MID | mid | 0.33–0.50 | A_NORMAL |
| CP_COOL | outer | 0.17–0.33 | A_NORMAL |
| CP_DIM | far outer | < 0.17 | A_NORMAL (not A_DIM) |

**Why not A_DIM on CP_DIM:**  A_DIM halves the terminal foreground brightness
on top of an already-dark palette color.  This made the outer disk invisible on
most terminals.  Removing A_DIM lets the dim palette color show through visibly.

---

## Themes (11 total)

| # | Name | Core colors | Character |
|---|------|-------------|-----------|
| 0 | interstellar | white → gold → orange → tan | Kip Thorne DNGR palette |
| 1 | matrix | bright lime → pure green → olive | cyber green |
| 2 | nova | white → ice-blue → blue-violet | stellar detonation |
| 3 | ocean | bright cyan → teal → navy | bioluminescent |
| 4 | poison | bright yellow → lime → olive | acid chartreuse |
| 5 | fire | white → yellow → orange → brown | white-hot core |
| 6 | plasma | white → pink → magenta → purple | neon hot pink |
| 7 | gold | bright yellow → gold → amber → bronze | molten metal |
| 8 | arctic | white → ice → light-blue → steel | polar ice |
| 9 | lava | bright red → orange-red → dark magenta → purple | volcanic |
| 10 | mono | white → light grey → mid grey → dark grey | clean greyscale |

All themes are designed so the DIM (outer disk) color is visibly above the
terminal's black background — minimum color index ≈ 64–130, never 17–22.

---

## Non-Obvious Decisions

### Why precompute instead of integrate every frame?

Each integration takes ~200–600 RK4 steps × 6 floats × 4 stages.  At 8,000 cells
× 60 fps: 480,000 integrations/second.  Even at 200 steps each: 96M RK4
evaluations/second.  The precompute costs ~0.4 s once; then rendering is ~50,000
multiplications per frame.

### Why `prev.y * pos.y < 0` instead of tracking sign?

The condition is branchless and catches both + → − and − → + crossings in one
comparison.  No state variable needed.

### Why no vel normalization?

Normalizing vel at each step would force |vel| = 1 artificially, interfering with
the ODE solver's error estimate.  The geodesic equation preserves the null
condition `g_μν vel^μ vel^ν = 0` to RK4 accuracy — normalizing only helps if you
have accumulated floating-point drift over thousands of steps, which we don't.

### Why adaptive step size `r × 0.05`?

The curvature term scales as h²/r^5.  For fixed angular accuracy, the step in φ
must be proportional to Δφ = Δs / r.  Setting Δs ∝ r gives constant angular
resolution per step regardless of orbit size.

### Why ESCAPE_R = 130 (fixed constant)?

Previously ESCAPE_R was `cam_dist × 4.5`, which worked when cam_dist was a
compile-time constant.  Once cam_dist became a runtime variable, ESCAPE_R needed
to remain valid for all cam_dist values (14–72 r_s).  130 r_s covers all of them
with wide margin.

### Why does the photon ring use min_r instead of checking ray steps?

Storing min_r (one float per cell) at precompute time costs nothing at render
time.  The alternative — re-integrating each ray during rendering — would defeat
the purpose of precomputing.  min_r is a sufficient statistic: it tells us
exactly how close to the photon sphere each ray came.

### Why tilt is fixed at 5° (not 0° or 90°)?

- 0° (edge-on): the camera is exactly in the disk plane; the disk appears as a
  thin line and the secondary arc is invisible.
- 90° (face-on): the disk fills the screen symmetrically but the Doppler
  asymmetry vanishes (both sides move perpendicular to the line of sight).
- 5°: shows both disk halves, the secondary arc above the shadow, and the full
  Doppler brightness asymmetry — closest to the Interstellar aesthetic.  Tilt
  rebuilds the entire lensing table, so making it runtime-adjustable is costly
  with no artistic benefit at small angles.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|--------------------|
| `BH_R` | 1.0 | Event horizon; smaller → larger shadow (must stay < 1.5) |
| `DISK_IN` | 3.0 | ISCO; moving inward → brighter/hotter inner edge |
| `DISK_OUT` | 12.0 | Outer disk; larger → wider band on screen |
| `CAM_DIST_DEF` | 38.0 | Default camera distance |
| `CAM_DIST_MIN` | 14.0 | Closest (largest on screen) |
| `CAM_DIST_MAX` | 72.0 | Farthest (smallest on screen) |
| `TILT_DEG` | 5.0 | Fixed inclination above equatorial plane |
| `MAX_STEPS` | 900 | More → slower build, catches higher-order images |
| `DS_BASE` | 0.10 | Smaller → more accurate geodesics, slower build |
| `ESCAPE_R` | 130.0 | Must exceed all cam_dist values with margin |
| `PHOTON_R` | 1.5 | Photon sphere radius (= 1.5 r_s for Schwarzschild) |

---

## Open Questions for Pass 3

1. Add Kerr metric support (spinning black hole).  The geodesic equation gets
   an extra Carter constant term.  Kerr has a smaller ISCO (down to r = 0.5 r_s
   for maximal spin) and a different photon sphere.  How does the spin parameter
   `a` change the shadow shape?

2. The photon ring brightness formula `exp(−(min_r − 1.5) × 2.4)` is empirical.
   Derive the exact relationship between impact parameter `b` and `min_r` for
   Schwarzschild, then map `b → deflection angle → ring intensity` properly.

3. Implement gravitational colour shift: map `D × g` to a blackbody temperature
   shift (inner disk blue-white, outer red), not just brightness.

4. The precompute is single-threaded.  Parallelize with pthreads (one thread per
   row).  What locking is needed?  Answer: none — each row writes to independent
   cells.  Benchmark the speedup.

5. Currently only the first disk crossing is recorded.  Modify `ray_trace` to
   record the second crossing (tertiary image).  Store it in a separate table.
   How dim is it relative to the primary?

6. At tilt = 0°, the upper and lower secondary arcs should be symmetric.  Verify
   this.  If not, trace which code path breaks the symmetry.

---

# Pass 2 — blackhole.c: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | BH_R, PHOTON_R, DISK_IN/OUT, CAM_DIST_DEF/MIN/MAX, TILT_DEG, MAX_STEPS |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 11 themes × 8 pairs; `theme_apply()` |
| §4 V3 math | dot, cross, len, norm, add, sub, scale — all inline |
| §5 geodesic | `geo_deriv()` (acc = −3h²pos/2r^5), `geo_step()` (RK4) |
| §6 ray table | `Cell` struct (kind, disk_r, disk_phi, esc_th, esc_ph, **min_r**); `ray_trace()` |
| §7 precompute | Builds `g_table[MAX_ROWS][MAX_COLS]`; progress bar; takes cam_dist |
| §8 render | Table lookup + Doppler + photon ring + disk_pair + disk_char; takes cam_dist |
| §9 screen | `screen_init()`, `screen_hud()` (shows dist, closer/farther labels) |
| §10 main | Precompute on start + resize + size-change; fixed-timestep loop |

---

## Data Flow

```
Startup:
  precompute(cols, rows, cam_dist=38)
    │
    └─ for each (row, col):
         dir = camera_ray(row, col, fwd, rgt, up)
         g_table[row][col] = ray_trace(cam, dir)
           │
           ├─ tracks min_r = minimum radius reached
           └─ terminates when:
                r < 0.92 × BH_R  → HORIZON  (min_r stored)
                r > ESCAPE_R     → ESCAPED  (esc_th, esc_ph, min_r stored)
                y sign changes   → DISK     (disk_r, disk_phi, min_r stored)

Frame N:
  disk_ang += spin × dt

  render(disk_ang, cols, rows, cam_dist):
    clip_r = cx × min((DISK_OUT/cam_dist) / tan(half-FOV) × 1.24, 0.96)

    for each (row, col):
      if isotropic_dist(row,col) > clip_r: continue   ← outside clip circle

      cell = g_table[row][col]
      switch cell.kind:
        HORIZON  → skip (pure black from erase())
        DISK     → phi = cell.disk_phi + disk_ang
                   beta = −v_orb(r) × cos(phi) × cos_tilt
                   D = [(1+β)/(1−β)]^1.5
                   g = √(1 − 1/r)
                   bright = D × g × rad(r) × tex(phi, disk_ang)
                   → disk_pair(bright, r_norm), disk_char(bright) → mvaddch
        ESCAPED  → if cell.min_r < 3.2:
                     ring_b = exp(−(min_r − PHOTON_R) × 2.4)
                     if ring_b > 0.09: draw photon ring char with CP_RING/CP_HOT
```

---

## geo_deriv (acceleration formula)

```
Input:  pos, vel
Output: dpos = vel,  dvel = −(3h²/2r^5) × pos

h  = cross(pos, vel)
h2 = dot(h, h)
r2 = dot(pos, pos)
r  = sqrt(r2)
c  = −1.5 × h2 / (r2 × r2 × r)
dvel = c × pos
```

## geo_step (RK4)

```
(dp1,dv1) = deriv(pos,          vel         )
(dp2,dv2) = deriv(pos+½ds×dp1, vel+½ds×dv1)
(dp3,dv3) = deriv(pos+½ds×dp2, vel+½ds×dv2)
(dp4,dv4) = deriv(pos+  ds×dp3, vel+  ds×dv3)

pos += ds/6 × (dp1 + 2dp2 + 2dp3 + dp4)
vel += ds/6 × (dv1 + 2dv2 + 2dv3 + dv4)
```

## ray_trace (with min_r)

```
mr   = len(origin)        ← initialise closest approach to starting radius
prev = origin
for step in 0..MAX_STEPS:
  r = len(pos)
  if r < mr: mr = r       ← update closest approach
  if r < 0.92 × BH_R: return HORIZON(min_r=mr)
  if r > ESCAPE_R:    return ESCAPED(normalize(vel), min_r=mr)
  ds = clamp(r × 0.05, 0.003, DS_BASE)
  prev = pos
  geo_step(&pos, &vel, ds)
  if prev.y × pos.y < 0:           ← disk crossing
    t   = |prev.y| / (|prev.y| + |pos.y|)
    hit = lerp(prev, pos, t)
    cr  = sqrt(hit.x² + hit.z²)
    if DISK_IN ≤ cr ≤ DISK_OUT:
      return DISK(cr, atan2(hit.z, hit.x), min_r=mr)
return ESCAPED(normalize(vel), min_r=mr)    ← max steps
```

## render (per cell)

```
DISK cell:
  phi_rot  = disk_phi + disk_ang
  r_norm   = clamp((disk_r − DISK_IN) / (DISK_OUT − DISK_IN), 0, 1)
  v_orb    = sqrt(0.5 / disk_r)
  beta     = −v_orb × cos(phi_rot) × cos_tilt
  D        = pow((1+beta)/(1−beta), 1.5)
  g        = sqrt(max(0.01, 1 − 1/disk_r))
  dr       = disk_r − DISK_IN
  isco     = exp(−dr² × 0.65)
  rad      = (1 − 0.86×r_norm)^2.2 + 0.65×isco
  tex      = 1 + 0.18 × sin(disk_phi×5 − disk_ang×4)
  bright   = clamp(D × g × rad × tex, 0, 1)
  if bright < 0.07: skip
  → disk_pair(bright, r_norm) → attron; mvaddch(disk_char(bright)); attroff

ESCAPED cell:
  mr = cell.min_r
  if mr < 3.2:
    ring_b = exp(−(mr − 1.5) × 2.4)
    if ring_b > 0.09:
      cp = ring_b > 0.55 ? CP_RING : CP_HOT
      a  = ring_b > 0.55 ? A_BOLD  : A_NORMAL
      ch = ring_b > 0.65 ? '*' : ring_b > 0.35 ? '+' : '.'
      → attron(COLOR_PAIR(cp)|a); mvaddch(ch); attroff

HORIZON cell:
  → nothing (erase() already cleared to black)
```
