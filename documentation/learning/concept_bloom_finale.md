# Concept: Bloom Finale (Deferred + SSAO + HDR Bloom)

## Pass 1 — Understanding

### Core Idea
The capstone of the raster folder. A single frame composes three techniques into one pipeline: deferred shading writes per-pixel surface data into a G-buffer; SSAO modulates the ambient term; the lightpass produces an HDR colour buffer; bloom extracts pixels above a luma threshold, blurs them with a separable Gaussian, and adds the blur back in. The pipeline ends with a Reinhard tone-map at paint time. Demonstrates that the previous techniques compose cleanly and produces the iconic "glowing emissive surface" look every modern engine ships with.

### Mental Model
Render in HDR (don't clamp to white), then pull just the OVER-WHITE pixels into a separate buffer, blur them so they spread, and add them back. That extra glow only appears around things that were already too bright for the display — exactly what your eye sees staring at a candle in a dark room. Direct lighting gives shape; SSAO gives crevices; bloom gives the felt sense of "this surface is emitting light."

### Key Equations
```
Lightpass (HDR):
  ambient   = AMBIENT · albedo · AO_factor          // SSAO modulates ambient
  diffuse   = albedo · sun_col · max(0, N·L)
  specular  = sun_col · max(0, N·H)^SHININESS · SPEC_GAIN
  emissive  = g_emissive[r][c]                       // 0 except for the orb
  g_light   = ambient + diffuse + specular + emissive
              ←  NOT clamped to [0, 1]  ←
              the orb's emissive (3.2, 1.8, 0.8) pushes channels past white

Bloom extract:
  bright = (luma(g_light) > THRESHOLD) ? g_light : 0

Separable Gaussian (1-D kernel applied along x then y):
  g(x) = exp(−x²/(2σ²)) / (σ·√(2π))
  blur_h(r, c) = Σ kernel[k] · in[r][c + k − RADIUS]
  blur_v(r, c) = Σ kernel[k] · in[r + k − RADIUS][c]

Composite:
  g_light += BLOOM_INTENSITY · g_bloom

Reinhard tone-map (in paint_cell):
  out = x / (1 + x)        // collapses [0, ∞) to [0, 1)
```

### Data Structures
- **G-buffer (5 channels)** — `g_pos`, `g_normal`, `g_albedo`, **`g_emissive`** (NEW HDR per-surface channel), `g_zbuf`, `g_z_view`, `g_valid`
- **AO buffers (2)** — same as ssao_pipeline.c
- **Bloom buffers (2)** — `g_bloom` (current bright pixels / blurred result), `g_bloom_tmp` (horizontal-pass scratch)

### Non-Obvious Decisions
- **Lightpass writes HDR, NOT [0, 1]**: bloom needs OVER-1.0 pixels to extract. If the lightpass clamped, only the orb's surface would peak at 1.0 (= ordinary white) and the threshold test would never fire. Reinhard tone-mapping is moved to paint time so HDR survives all the way through bloom.
- **`g_emissive` is its own G-buffer channel, not folded into albedo**: emissive bypasses ambient × AO and diffuse × N·L. A surface with emissive `(3, 1.5, 0.5)` glows at full intensity even on its dark side; mixing it into albedo would let SSAO and N·L gate it down.
- **Separable Gaussian, not 2-D**: a 7×7 2-D Gaussian costs 49 reads per pixel. The same blur factored into a 1-D horizontal pass + 1-D vertical pass is mathematically identical and costs 7 + 7 = 14 reads. Every modern bloom does this.
- **Boundary handling clamps coordinates**: when the kernel reaches off-buffer at the screen edge, sampled coords are clamped to `[0, W-1]` so edge values are duplicated outward. Without this, bloom halos would cut off at the screen boundary.
- **Threshold sits just above 1.0**: ordinary lit surfaces never exceed white, so they don't trigger bloom. Only the HDR emissive orb does. Lower threshold = more "fog"; higher = bloom only from the brightest points.

### Key Constants
| Name | Default | Effect |
|------|---------|--------|
| `BLOOM_THRESHOLD` | 1.00 | luma above this enters the bright buffer |
| `BLOOM_INTENSITY` | 1.00 | multiplier on the blur added back to g_light |
| `BLOOM_RADIUS` / `BLOOM_TAPS` | 3 / 7 | 7-tap 1-D Gaussian (σ ≈ 1.5) |
| `ORB_EMISSIVE` | (3.2, 1.8, 0.8) | HDR emissive of the orb (the only emitter) |
| `SSAO_RADIUS` | 0.45 | hemisphere radius for the AO pass |

### Open Questions
- What does FXAA do that bloom doesn't, and why don't we need it here?
- Bloom with multiple downsampled levels (Mipmap bloom): when do you need it?
- Why does Reinhard `x/(1+x)` go to 1.0 asymptotically without ever clipping?

## From the Source

**Algorithm:** Five image-space passes composed: (1) gbuffer, (2) ssao + blur (optional), (3) lightpass HDR, (4) bloom extract + horizontal Gaussian + vertical Gaussian + composite (optional), (5) Reinhard tone-map at paint. Each toggle is a clean conditional dispatch.

**Physics/References:** James & O'Rorke "Real-Time Glow" GPU Gems (2004); LearnOpenGL "Bloom" tutorial; Reinhard et al. "Photographic Tone Reproduction for Digital Images" SIGGRAPH '02. Saito & Takahashi 1990 (G-buffer).

**Math:** Separable Gaussian kernel; Reinhard local tone operator; Blinn-Phong + emissive composition (additive HDR).

**Performance:** Bloom is `pixels × kernel_taps × 2 passes` = trivial at terminal res. SSAO is `pixels × K samples`. Total per frame ≈ `1 ms` at 80×40.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_pos / g_normal / g_albedo` | `Vec3[H][W]` | ~115 KB each | gbuffer attribute channels |
| `g_emissive[H][W]` | `Vec3[]` | ~115 KB | HDR per-surface emission |
| `g_zbuf / g_z_view` | `float[]` | ~38 KB each | NDC + linear depths |
| `g_valid[H][W]` | `uint8_t[]` | ~10 KB | covered mask |
| `g_ao / g_ao_blur` | `float[]` | ~38 KB each | SSAO outputs |
| `g_light[H][W]` | `Vec3[]` | ~115 KB | HDR lit colour (NOT clamped) |
| `g_bloom / g_bloom_tmp` | `Vec3[]` | ~115 KB each | bright extract + blur scratch |

---

## Pass 2 — Implementation

### Pseudocode
```
per frame:
    render_gbuffer        — pos / normal / albedo / emissive / depths

    if ssao_enabled:
        ssao_pass(VP, radius)
        ssao_blur()

    render_lightpass(use_ao):
        for r, c in valid:
            ao = use_ao ? g_ao_blur[r][c] : 1
            ambient   = AMBIENT · albedo · ao
            diffuse   = albedo · sun · max(0, N·L)
            specular  = sun · max(0, N·H)^SHININESS · SPEC_GAIN
            emissive  = g_emissive[r][c]
            g_light[r][c] = ambient + diffuse + specular + emissive   // HDR, NOT clamped

    if bloom_enabled:
        bloom_extract(THRESHOLD):
            for r, c:
                g_bloom[r][c] = (luma(g_light[r][c]) > T) ? g_light[r][c] : 0

        bloom_blur_h():
            for r, c:
                sum = 0
                for k in 0..TAPS-1:
                    sc = clamp(c + k − RADIUS, 0, cols-1)
                    sum += KERNEL[k] · g_bloom[r][sc]
                g_bloom_tmp[r][c] = sum

        bloom_blur_v():
            for r, c:
                sum = 0
                for k in 0..TAPS-1:
                    sr = clamp(r + k − RADIUS, 0, rows-1)
                    sum += KERNEL[k] · g_bloom_tmp[sr][c]
                g_bloom[r][c] = sum

        bloom_composite(intensity):
            for r, c: g_light[r][c] += intensity · g_bloom[r][c]

    render_scene → paint_cell (Reinhard tone-map collapses HDR to display)
```

### Module Map
```
§1  config      — frame, cam, SSAO, BLOOM, sun + ambient, scene geometry
§2  clock
§3  math        — Vec3 / Mat4 + v3_luma helper
§4  paint       — Reinhard + gamma + dither + Bourke ramp
§5  mesh        — box / quad / sphere tessellators
§6  gbuffer     — geometry pass (writes EMISSIVE channel too)
§7  ssao        — kernel + ssao_pass + ssao_blur
§8  lightpass   — HDR Blinn-Phong + emissive (no clamp)
§9  bloom       — extract + blur_h + blur_v + composite
§10 scene       — Scene struct, init, tick (camera orbit)
§11 screen      — render_scene + HUD
§12 app         — render_frame (conditional dispatch by toggle), main loop
```

### Data Flow
```
mesh → render_gbuffer (with emissive) → G-buffer

G-buffer ─┬─→ ssao_pass + ssao_blur (if on) → g_ao_blur
          │
          └─→ render_lightpass (HDR, ambient × ao + diffuse + spec + emissive) → g_light

g_light ─┬─→ bloom_extract → g_bloom
         │       ↓
         │   bloom_blur_h → g_bloom_tmp
         │       ↓
         │   bloom_blur_v → g_bloom
         │       ↓
         │   bloom_composite (g_light += intensity · g_bloom)
         │
         └─→ render_scene → paint_cell (Reinhard tone-map → ncurses)
```
