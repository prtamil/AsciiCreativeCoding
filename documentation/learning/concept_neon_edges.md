# Concept: Neon Edges (Sobel Post-Process + Bloom)

## Pass 1 — Understanding

### Core Idea
Tron-bike-style edge glow on 3-D shapes. Don't model edge geometry. Detect edges in the IMAGE after the scene is rendered. Two things define a visual edge: a place where DEPTH jumps suddenly (silhouette) or where the NORMAL suddenly flips (crease between two faces). Both show up as a high spatial gradient in the G-buffer. Sobel measures that gradient in one cheap convolution; threshold it; colour the result; let the bloom pipeline (§185 / `bloom_finale.c`) bleed it into a soft glow.

### Mental Model
Run your finger across the depth buffer. On a smooth surface, depth slides under your finger smoothly — small derivative. At a silhouette, depth abruptly drops to the void — large derivative. At a crease, the normal flips abruptly — large derivative on the normal field. Sobel is just "compute that derivative magnitude per pixel." Bright where the derivative is large = exactly the edges. Then bloom turns the edges into glowing halos.

### Key Equations
```
Sobel kernels (3×3):
  Sx = | -1  0  1 |       Sy = | -1 -2 -1 |
       | -2  0  2 |            |  0  0  0 |
       | -1  0  1 |            |  1  2  1 |

Per-pixel gradient magnitude:
  gx       = (f[2] + 2·f[5] + f[8]) − (f[0] + 2·f[3] + f[6])
  gy       = (f[6] + 2·f[7] + f[8]) − (f[0] + 2·f[1] + f[2])
  |∇field| = √(gx² + gy²)

Edge magnitude:
  depth_grad   = |∇ z_view|
  normal_grad  = Σ |∇ Nx, Ny, Nz|
  edge         = max(depth_grad · DEPTH_SCALE, normal_grad · NORMAL_SCALE)

Smooth threshold (smoothstep):
  t = clamp01((edge − THRESHOLD) / KNEE)
  t = t² · (3 − 2t)

Edge buffer:
  g_edge = NEON_COLOR · t · INTENSITY    // HDR cyan when t = 1
```

### Data Structures
- **G-buffer** — same as ssao_pipeline (`g_pos`, `g_normal`, `g_albedo`, `g_zbuf`, `g_z_view`, `g_valid`)
- **Edge buffer** — `g_edge[H][W]` Vec3, HDR neon per pixel
- **Bloom buffers** — `g_bloom`, `g_bloom_tmp` (same as bloom_finale)
- **Sobel kernels** — implicit (3×3 indices in `sobel_magnitude`)

### Non-Obvious Decisions
- **LINEAR depth (g_z_view), not NDC z**: NDC z is non-linear after perspective and would falsely report depth gradients on receding flat surfaces. Sobel on `g_z_view` only fires at real geometry boundaries.
- **Invalid neighbours sampled as `−CAM_FAR`**: at the silhouette, a 3×3 neighbourhood straddles the object boundary; some samples fall on `g_valid = 0` (the void). Substituting `−CAM_FAR` creates a HUGE depth jump → silhouette correctly detected. Without this, silhouettes would have a small gradient and not trigger.
- **Two signal sources with `max`, not `+`**: silhouettes have huge depth gradient and ~0 normal gradient. Creases have ~0 depth gradient and big normal gradient. Maxing the two correctly catches both kinds. Adding would dilute each.
- **Normal Sobel is sum-of-magnitudes across components**: each of `Nx`, `Ny`, `Nz` is Sobel'd independently and summed. A crease where the +X face meets the +Y face has a big jump in two components but not the third; summing captures the total normal change.
- **Lit faces are intentionally DIM**: ambient ~0.04, sun ~0.12 per channel. The shape interiors are barely visible without edges. This is what creates the iconic Tron canvas where edges dominate the visual energy.
- **Smoothstep, not binary threshold**: a hard `if edge > T` would produce 1-pixel-wide, aliased lines. Smoothstep gives a 2-3 pixel anti-aliased edge transition which looks much better at terminal resolution.

### Key Constants
| Name | Default | Effect |
|------|---------|--------|
| `EDGE_THRESHOLD` | 0.75 | smoothstep low knee — below: no glow |
| `EDGE_KNEE` | 0.45 | smoothstep range — full glow at threshold + knee |
| `DEPTH_SCALE` | 0.05 | weight on the depth gradient |
| `NORMAL_SCALE` | 0.40 | weight on the normal gradient |
| `EDGE_INTENSITY` | 1.50 | multiplier on the neon HDR colour |
| `NEON_COLOR` | (0.5, 2.0, 2.5) | cyan (channels exceed 1.0 = HDR) |
| `BLOOM_THRESHOLD` | 0.90 | bloom captures partial edges |
| `BLOOM_INTENSITY` | 1.50 | strong glow halo |

### Open Questions
- What changes if you crank NORMAL_SCALE high enough that low-poly tessellation creases trigger? (sphere becomes a wire mesh)
- Can you replace Sobel with a simple `max(neighbour) − min(neighbour)` and get the same look? (yes for binary, no for smoothstep)
- What does an axis-aligned Laplacian-of-Gaussian do here vs Sobel?

## From the Source

**Algorithm:** Sobel filter on G-buffer depth + each normal component → edge magnitude → smoothstep → HDR neon colour → bloom extract → separable Gaussian → composite → tone-map.

**Physics/References:** Sobel & Feldman 1968 SAIL talk (the original 3×3 isotropic gradient operator); Mitchell et al. "Real-Time Rendering Tricks for Ambient Occlusion and Edge Detection" GDC '07; James & O'Rorke "Real-Time Glow" GPU Gems 2004; LearnOpenGL "Bloom" tutorial.

**Math:** Discrete approximation of `∂f/∂x`, `∂f/∂y` via centred differences with weighted neighbours. Smoothstep: cubic Hermite interpolation `t² · (3 − 2t)` between two endpoints.

**Performance:** Edge pass is `O(pixels × 9 reads × 4 channels)` ≈ trivial at terminal resolution. Bloom is the same cost as bloom_finale.c. Total per frame ≈ 1 ms at 80×40.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_pos / g_normal / g_albedo` | `Vec3[H][W]` | ~115 KB each | gbuffer attribute channels |
| `g_zbuf / g_z_view` | `float[]` | ~38 KB each | NDC + linear depths |
| `g_valid[H][W]` | `uint8_t[]` | ~10 KB | covered mask |
| `g_edge[H][W]` | `Vec3[]` | ~115 KB | HDR neon per pixel |
| `g_light[H][W]` | `Vec3[]` | ~115 KB | dim lit + edge HDR |
| `g_bloom / g_bloom_tmp` | `Vec3[]` | ~115 KB each | bloom extract + blur scratch |

---

## Pass 2 — Implementation

### Pseudocode
```
per frame:
    render_gbuffer  — pos / normal / albedo / NDC z / view-z / valid

    if edges_on:
        edge_pass:
            for r, c in valid:
                // sample 3×3 of g_z_view (invalid → −CAM_FAR)
                field_z[9] = sample_zview_3x3(r, c)
                depth_grad = sobel_magnitude(field_z)

                // sample 3×3 of each normal component
                normal_grad = 0
                for axis in 0..2:
                    field_n[9] = sample_normal_3x3(r, c, axis)
                    normal_grad += sobel_magnitude(field_n)

                edge = max(depth_grad · DEPTH_SCALE,
                           normal_grad · NORMAL_SCALE)
                t    = smoothstep(THRESHOLD, THRESHOLD + KNEE, edge)
                g_edge[r][c] = NEON · t · INTENSITY

    render_lightpass(edges_on):
        for r, c in valid:
            ambient   = AMBIENT · albedo                  // intentionally dim
            diffuse   = albedo · sun · max(0, N·L)
            specular  = sun · max(0, N·H)^SHININESS · SPEC_GAIN
            edge_term = edges_on ? g_edge[r][c] : 0
            g_light[r][c] = ambient + diffuse + specular + edge_term   // HDR

    if bloom_on:
        bloom_extract → bloom_blur_h → bloom_blur_v → bloom_composite

    render_scene → paint_cell (Reinhard tone-map → ncurses)

sobel_magnitude(field[9]):
    gx = (field[2] + 2·field[5] + field[8])
       − (field[0] + 2·field[3] + field[6])
    gy = (field[6] + 2·field[7] + field[8])
       − (field[0] + 2·field[1] + field[2])
    return √(gx² + gy²)
```

### Module Map
```
§1  config      — frame, cam, scene, EDGE params, BLOOM params, lighting (DIM!)
§2  clock
§3  math        — Vec3 / Mat4 + v3_luma
§4  paint       — Reinhard + dither + Bourke ramp
§5  mesh        — box / quad + tessellate_polyhedron + tetra + octa
§6  gbuffer     — geometry pass (writes z_view for Sobel)
§7  edge        — sobel_magnitude + sample helpers + edge_pass
§8  lightpass   — DIM Blinn-Phong + g_edge → HDR g_light
§9  bloom       — extract + separable blur + composite (same as bloom_finale)
§10 scene       — 3 platonic shapes + per-object spin
§11 screen      — render_scene + HUD
§12 app         — render_frame (conditional dispatch), main loop
```

### Data Flow
```
mesh (cube + tetra + octa + floor) → render_gbuffer → G-buffer (pos / normal / depths)

G-buffer ─┬─→ edge_pass (Sobel on z_view + normals) → g_edge (HDR neon)
          │
          └─→ render_lightpass (DIM Blinn-Phong + g_edge) → g_light (HDR)

g_light → bloom (extract + blur + composite) → bloom-augmented g_light → paint_cell
```
