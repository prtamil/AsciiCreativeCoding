# Concept: Screen-Space Ambient Occlusion (SSAO)

## Pass 1 — Understanding

### Core Idea
SSAO darkens the AMBIENT term of lit pixels in proportion to how "crowded" their local neighbourhood is in 3-D. For each visible pixel, take K random samples in the upper hemisphere along the surface normal, project them back to the screen, and ask the depth buffer "is something in front of this sample?" The fraction of YES answers is the occlusion factor; `(1 − occluded)` scales the ambient illumination so corners darken without affecting direct light.

### Mental Model
Each pixel sticks K little antennas into the air, each pointing in a random direction in the upper hemisphere along the surface normal. Each antenna asks its tip: "is there a closer surface between you and the camera?" Open-sky antennas say NO — pixel stays bright. Antennas pointing into a wall say YES — pixel darkens. The output AO factor is `(1 − occluded / total)`, multiplied into the ambient term only.

### Key Equations
```
Hemisphere flip:    if dir · N < 0:   dir ← −dir
Sample point:       S = P + dir · radius
Project to screen:  clip = VP · (S, 1);  ndc = clip / clip.w
                    sx = ( ndc.x + 1)/2 · cols
                    sy = (−ndc.y + 1)/2 · rows
Range falloff:      attn = max(0, 1 − |Δz_view| / radius)
Depth test:         g_zbuf[sy][sx] < ndc_z(S) − BIAS  → occluded
AO factor:          AO = 1 − Σ(occl · attn) / max(Σ attn, ε)
3×3 box blur:       AO′[r][c] = mean(AO[r±1][c±1])  with valid mask
Composite:          col = AMBIENT · albedo · AO′ + diffuse + specular
```

### Data Structures
- **G-buffer (5 channels)** — `g_pos`, `g_normal`, `g_albedo`, `g_zbuf` (NDC z), `g_z_view` (LINEAR view-space z, for the range falloff), `g_valid` (mask)
- **AO buffers (2)** — `g_ao` (raw, noisy), `g_ao_blur` (after 3×3 box blur)
- **Sample kernel** — `k_ssao[VARIANTS][SAMPLES]` precomputed unit-ish vectors. 4 variants × 12 samples = 48 distinct directions across each 2×2 pixel tile

### Non-Obvious Decisions
- **View-space z, not NDC z, for the range check**: NDC z is non-linear after perspective and produces a skewed range falloff. The G-buffer carries a parallel `g_z_view` channel — pre-divide, linear — just for SSAO's range check.
- **Per-pixel kernel variant**: each pixel picks one of 4 variants by `(c & 1) | ((r & 1) << 1)`. This means every 2×2 pixel tile uses every variant once. The 3×3 box blur then averages over a 9-cell neighbourhood that contains every variant several times — smooths the per-pixel checker noise without bleeding silhouettes.
- **Off-screen samples are SKIPPED, not counted**: when the projected `(sx, sy)` lands outside the G-buffer or on `g_valid = 0`, the sample is dropped (doesn't count for or against occlusion). Counting them as occluded would paint a dark halo around every silhouette.
- **Self-occlusion bias**: a flat surface's samples sit just above itself; without `SSAO_BIAS` the depth test teeters on the edge and float jitter darkens flat regions into noisy gray. BIAS pushes the comparison plane slightly forward.

### Key Constants
| Name | Default | Effect |
|------|---------|--------|
| `SSAO_SAMPLES` | 12 | Samples per pixel per variant |
| `SSAO_KERNEL_VARIANTS` | 4 | Sample sets cycled across the 2×2 tile |
| `SSAO_RADIUS_DEF` | 0.45 | World-units the sampler reaches; live `[`/`]` adjusts |
| `SSAO_BIAS` | 0.0008 | NDC offset against acne |

### Modes (`a` cycles)
| Mode | What's painted | Useful for |
|------|---------------|------------|
| `AO_ONLY` | grayscale `g_ao_blur` | counting AO features |
| `LIT_NO_AO` | `g_light` from lightpass with `use_ao = false` | what would look like if SSAO wasn't there |
| `LIT_WITH_AO` | `g_light` with `use_ao = true` | the final composite |

### Open Questions
- Why does `cosine-weighted` hemisphere sampling produce smoother AO than uniform? (importance-samples Lambertian)
- What changes if SSAO multiplies the diffuse term, not just ambient? (physically wrong but visually striking)
- Half-resolution SSAO + bilateral upsample — what's the cost / quality tradeoff?

## From the Source

**Algorithm:** Hemisphere occlusion estimation in image space. `ssao_pass` walks every visible pixel, picks K hemisphere directions from a precomputed kernel, flips into the upper hemisphere along N, projects each sample point to the screen, reads `g_zbuf` and `g_z_view` at the projected position, and combines an in/out depth test with a range-falloff weight.

**Physics/References:** Mittring "Finding Next Gen — CryEngine 2" SIGGRAPH '07 (the original screen-space AO); Bavoil & Sainz "Multi-Layer Dual-Resolution SSAO" SIGGRAPH '09; LearnOpenGL "SSAO" tutorial. The "ambient lighting only" rule for AO is Hayden Landis's seminal 2002 result.

**Math:** Hubbard-Douady-style smoothstep at the threshold; valid-mask 3×3 box blur (no bleed across silhouettes). Range falloff in linear view-z, not NDC.

**Performance:** SSAO is O(pixels × K). At 80×40 cells × 12 samples ≈ 38k ops per frame — trivial. Real engines run at half-res + bilateral upsample to cut cost ~4×.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_pos[H][W]` | `Vec3[]` | ~115 KB | world position from gbuffer |
| `g_normal[H][W]` | `Vec3[]` | ~115 KB | world normal |
| `g_albedo[H][W]` | `Vec3[]` | ~115 KB | flat surface colour |
| `g_zbuf[H][W]` | `float[]` | ~38 KB | NDC depth |
| `g_z_view[H][W]` | `float[]` | ~38 KB | LINEAR view-space depth (for SSAO range check) |
| `g_valid[H][W]` | `uint8_t[]` | ~10 KB | covered-by-geometry mask |
| `g_ao[H][W]` | `float[]` | ~38 KB | raw AO per pixel |
| `g_ao_blur[H][W]` | `float[]` | ~38 KB | 3×3 box-averaged AO |
| `k_ssao[V][S]` | `Vec3[48]` | 576 B | precomputed sample kernel |

---

## Pass 2 — Implementation

### Pseudocode
```
init:
    ssao_init_kernel()                  // deterministic LCG, 4 variants × 12 samples

per frame:
    render_gbuffer()                    // pos, normal, albedo, zbuf, z_view, valid
    if ssao_enabled:
        ssao_pass(VP, radius)
        ssao_blur()
    render_lightpass(use_ao = ssao_enabled)
    paint_each_cell

ssao_pass(VP, radius):
    for r, c in valid pixels:
        P = g_pos[r][c]
        N = g_normal[r][c]
        variant = (c & 1) | ((r & 1) << 1)

        occlude_w, total_w = 0, 0
        for dir in k_ssao[variant]:
            if dot(dir, N) < 0:  dir = −dir
            S = P + dir · radius
            clip = VP · (S, 1)
            if clip.w < ε: continue

            ix, iy = ndc_to_screen(clip)
            if out_of_bounds or !g_valid[iy][ix]: continue

            dz = |g_z_view[r][c] − g_z_view[iy][ix]|
            attn = max(0, 1 − dz / radius)
            if attn ≤ 0: continue
            total_w += attn

            ndc_z_S = clip.z / clip.w
            if g_zbuf[iy][ix] + BIAS < ndc_z_S:
                occlude_w += attn

        g_ao[r][c] = (total_w > ε) ? (1 − occlude_w / total_w) : 1

ssao_blur():
    for r, c in valid pixels:
        sum, count = 0
        for dr, dc in 3×3:
            if neighbour valid: sum += g_ao[r+dr][c+dc]; count++
        g_ao_blur[r][c] = sum / count

render_lightpass(use_ao):
    for r, c in valid pixels:
        ao = use_ao ? g_ao_blur[r][c] : 1
        ambient_lit = AMBIENT · albedo · ao
        direct_lit  = blinn_phong(P, N, albedo, sun, cam)
        g_light[r][c] = clamp01(ambient_lit + direct_lit)
```

### Module Map
```
§1 config       — SSAO_SAMPLES, KERNEL_VARIANTS, RADIUS, BIAS, sun + ambient
§2 clock        — monotonic timer + sleep
§3 math         — Vec3, Vec4, Mat4 + perspective / lookat
§4 paint        — 216 RGB cube + Bourke ramp + paint_cell
§5 mesh         — box + quad + sphere tessellators
§6 gbuffer      — geometry pass, writes pos/normal/albedo/depths
§7 ssao         — kernel init + ssao_pass + ssao_blur
§8 lightpass    — Blinn-Phong; ambient × g_ao_blur (or 1)
§9 scene        — Mode enum, init/tick (camera orbit + ssao_radius)
§10 screen      — render_scene + HUD
§11 app         — signals, resize, dispatched main loop
```

### Data Flow
```
mesh → vertex shader → clip + screen
clip → cull → barycentric → fragment → G-buffer write

G-buffer ─┬─→ ssao_pass → g_ao
          │       ↓
          │   ssao_blur → g_ao_blur
          │       ↓
          ├─→ lightpass: ambient × g_ao_blur + diffuse + spec → g_light
          │       ↓
          └─→ render_scene → paint_cell → ncurses
```
