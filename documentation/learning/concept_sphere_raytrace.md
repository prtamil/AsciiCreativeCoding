# Pass 1 — sphere_raytrace.c: Analytic ray-traced sphere (the foundational demo)

## Core Idea

The "Hello World" of analytic ray tracing. One ray per terminal cell, one
sphere, three coloured lights, four shading modes. The intersection test is
a single quadratic in `t` — solving it gives the hit point exactly, with no
marching and no mesh.

The geometry is the simplest possible setup: sphere fixed at the origin,
camera orbiting around it at fixed elevation. Read this file before any of
its siblings (cube / capsule / torus / path_tracer / saturn / etc.) — it
establishes the skeleton they all extend.

## Ray-Sphere Quadratic

Ray: `P(t) = ro + t·rd` with `|rd| = 1`.
Sphere at origin, radius R: `|P|² = R²`.

Substituting and simplifying (since `|rd| = 1` removes the leading `t²`
coefficient):

```
t² + 2(rd·ro)·t + (|ro|² − R²) = 0
half-b form (saves divisions):
  b    = rd·ro
  c    = |ro|² − R²
  disc = b² − c
  t    = -b - √disc    (front face; back is -b + √disc)
```

If `disc < 0`: miss. Else `t = -b - √disc` is the front-face hit. The
T_EPS guard rejects effectively-zero t-values that arise from
self-intersection or grazing tangents.

Surface normal at hit P: `N = P / R` (sphere centred at origin).

## Three-Point Lighting

Three world-space POINT lights (positions, not directions). Each pixel:

| Light | Position | Role | Contribution |
|---|---|---|---|
| KEY  | (3, 4, −2)  | warm primary | diffuse + sharp specular (shininess 52) |
| FILL | (−4, 1, −1) | cool ambient | diffuse only — lifts shadow side |
| RIM  | (0.5, −1, 5) | accent | wide specular at silhouette (shininess 10) |

For each light per pixel:
```
L = normalize(light_pos − P)
d = max(0, N · L)                       Lambertian
R = reflect(−L, N) = 2(N·L)·N − L       reflection of incoming light
s = max(0, R · V_dir)^shininess         Phong specular
```

`V_dir = normalize(cam − P)`. The total is `ambient + KEY + FILL + RIM`,
clamped to [0, 1].

## §6 split into named sub-functions

Old code was one monolithic `shade_phong`. The current rewrite splits
contributions:

```
§6.1 light_key      KEY light (warm diffuse + specular)
§6.2 light_fill     FILL light (cool diffuse, no specular by design)
§6.3 light_rim      RIM light (silhouette accent specular)
§6.4 shade_phong    orchestrator: ambient + key + fill + rim
§6.5 shade_normal   diagnostic — N encoded as RGB
§6.6 shade_fresnel  Schlick (1−cosθ)^5 — glass-marble look
     shade_depth    inverse-square hit-distance encoding
```

Splitting per-light makes the lighting model readable: each function is one
concept; the orchestrator is 5 lines of glue.

## Shade modes (cycled with `s`)

| Mode | Visual |
|---|---|
| PHONG    | Full 3-point Phong — the default cinematic look |
| NORMAL   | RGB-encoded surface normal (diagnostic). +Z silhouette → blue, +Y → green, +X → red |
| FRESNEL  | Schlick — dark head-on, bright at silhouette (glass-marble) |
| DEPTH    | Closer = brighter, falloff is `(1 − t/t_max)²` |

## Paint pipeline (RGB cube + Bourke ramp)

Same template as the rest of the raytracing folder:
- 216 ncurses pairs allocated as a 6×6×6 RGB cube (PAIR_CUBE_BASE..+215)
- For each pixel: tone-mapped RGB → quantise to cube pair + density char
  from 92-char Bourke ramp
- Effective resolution per cell ≈ 216 × 92 ≈ 20 000 distinct visual states

## HUD spec

- Yellow status row 0 (top-right): fps, dist, theme, paused/running
- Yellow mode label row 0 (top-left): "mode: phong" etc.
- Cyan hint bottom row: key reference

## Worked Example (verify by hand)

Camera at (0, 0.55, −3.6) (orbit_ang=0, cam_dist=3.6). Sphere at origin, R=1.
Centre cell of screen, so `pu = pv = 0`, `rd ≈ fwd`:

```
fwd  = normalize((0,0,0) − (0, 0.55, -3.6)) = (0, -0.151, 0.989)
oc   = ro = (0, 0.55, -3.6)
b    = rd·oc = -0.151·0.55 + 0.989·(-3.6) = -3.643
c    = oc·oc − R² = 12.262
disc = b² − c = 1.010
t    = -b − √disc = 3.643 − 1.005 = 2.638
P    = ro + t·rd = (0, 0.152, -0.991)
|P|  ≈ 1.003 ✓ (≈ unit sphere surface)
N    = P / 1 ≈ (0, 0.151, -0.987)
```

For KEY light at (3, 4, -2):
```
L_dir = (3, 3.848, -1.009)
|L_dir| ≈ 4.983
L     = (0.602, 0.772, -0.202)
N·L   = 0 + 0.117 + 0.199 = 0.316
diffuse = 0.316
```

With gold theme (obj=(0.90, 0.72, 0.18), key_col=(1.00, 0.92, 0.70)):
```
key_diff = 0.316 · 0.65 · obj·key_col
         = 0.205 · (0.90, 0.662, 0.126)
         ≈ (0.185, 0.136, 0.026)
```

Plus ambient + FILL + RIM contributions, clamp, gamma-encode, quantise to
cube → bright warm gold pixel at the centre.

## Edge cases

- **Discriminant near zero** — tangent ray. T_EPS guard rejects t<1e-4 to
  avoid self-intersection/numerical instability.
- **Camera inside sphere** — both quadratic roots have opposite signs
  (back face accepts). Code falls back from t0 (front) to t1 (back) when
  t0 < T_EPS.
- **Specular on dark side artifact** — KEY's specular is gated only by R·V
  (not by N·L). On the unlit hemisphere this can produce a "ghost"
  highlight. Acceptable simplification for the simplest Phong model.
- **Tone-map at paint time** — quantising linear HDR puts every pixel in
  one cube cell. Reinhard inside the paint function opens dynamic range.

## How to verify

- Default gold sphere: KEY highlight upper-right, FILL fills shadow side
  cool, RIM kisses the silhouette from behind.
- MODE_NORMAL: rainbow ball — +Z silhouette mostly blue (0.5, 0.5, 1.0)
  fading to red/green at silhouette.
- MODE_FRESNEL: dark middle (cosθ=1 → F=0), bright silhouette (cosθ=0
  → F=1). Classic glass-marble.
- MODE_DEPTH: silhouette dimmest (largest t).
- Worked example: at orbit_ang=0, cam_dist=3.6, centre-cell ray hits at
  t ≈ 2.638. Inspect source if your sphere appears in the wrong place.

---

# Pass 2 — Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config        | frame rate, FOV, sphere radius, camera, ramp, HUD pairs |
| §2 clock         | monotonic timer + sleep |
| §3 math          | V3 helpers |
| §4 color         | themes (6) + 256-colour cube + paint_cell |
| §5 sphere        | THE CORE — analytic quadratic ray-sphere |
| §6 shading       | 6.1 KEY, 6.2 FILL, 6.3 RIM, 6.4 phong, 6.5 normal, 6.6 fresnel/depth |
| §7 render        | one frame: ray-per-cell, three-point lighting |
| §8 screen + HUD  | yellow row 0 status + cyan bottom hint |
| §9 app           | signals, resize, fixed-step main loop |

## Data flow

```
keys → main → orbit_ang, cam_dist, theme_idx, mode, paused
                              │
clock_ns → dt → orbit_ang += ROT_SPEED · dt
                              │
                       render(cols, rows, ...)
                          per cell:
                            rd = camera_ray
                            ray_sphere(cam, rd, R) → t_hit
                            P = ro + t·rd; N = P/R; V_dir = -rd
                            switch (mode):
                              PHONG   → ambient + KEY + FILL + RIM
                              NORMAL  → (N+1)/2
                              FRESNEL → mix(core, edge, (1-cosθ)⁵)
                              DEPTH   → (1 - t/t_max)²
                            draw_color(row, col, color, lum)
                              │
                              ▼
                       hud_draw → wnoutrefresh + doupdate
```

## Pseudocode

```
setup:
  install signals (SIGINT/SIGTERM/SIGWINCH)
  initscr + colour_init (216 cube pairs + HUD/HINT)
  atexit(cleanup)

loop:
  if need_resize: re-init geometry
  dt = clock_ns - last
  if !paused: orbit_ang += ROT_SPEED · dt
  fps update
  erase
  render(cols, rows, orbit_ang, cam_dist, theme, mode)
  hud_draw
  wnoutrefresh + doupdate
  drain input → handle key
  sleep to TARGET_FPS
```

## Key patterns to internalise

**Half-b form of the quadratic.** Using `b = rd·ro` (not `2·rd·ro`) and
`disc = b² − c` (not `b²/4 − c`) saves two divisions per pixel without
changing the result. Standard simplification when `|rd| = 1`.

**Three-point lighting per-function split.** Each light is one named
function that returns its contribution. The orchestrator is glue. Easier
to read, easier to disable a light for debugging.

**Inverse-rotation trick.** Sphere stays at origin; camera orbits.
Equivalent to rotating the sphere, but keeps the intersection math at its
simplest.

**Same paint pipeline as the entire folder.** RGB → Reinhard → gamma →
6×6×6 cube + 92-char ramp. Read once across all the raytracers.
