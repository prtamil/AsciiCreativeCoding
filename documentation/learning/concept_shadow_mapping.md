# Concept: Shadow Mapping (Directional Light)

## Pass 1 — Understanding

### Core Idea
"Is point P in shadow of light L?" is the same as "is some surface closer to L than P along the ray P → L?" Tracing that ray is expensive. But the LIGHT sees the closest surface in every direction whenever you render the scene from the light's camera — that's literally what a depth buffer records. Render the scene from L once, store the depth buffer, then every shading pixel just LOOKS UP the answer instead of tracing.

### Mental Model
The sun has a camera and snaps a depth photo of the scene. Every pixel of the photo records "this is how far the closest thing was from me." Now any point in 3-D can be projected through the sun's lens. If the photo's depth at that pixel is SMALLER than the point's actual distance to the sun, then something was IN THE WAY between the sun and the point — that point is in shadow. Direct lighting is multiplied by `(1 − shadow)`; ambient is unaffected (corners in real shadow still receive bounce light).

### Key Equations
```
Light eye         : eye = scene_centre − sun_dir · LIGHT_DISTANCE
Light view        : V_light = lookAt(eye, origin, up)
Light proj        : P_light = ortho(±H, ±H, near, far)         // directional → ortho
Light MVP         : L_mvp = P_light · V_light · M_model
Light NDC         : ndc = (L_mvp · pos) / clip.w               // = clip for ortho

Shadow UV         : u = ( ndc.x + 1)/2 · SHADOW_W
                    v = (−ndc.y + 1)/2 · SHADOW_H               // Y-flip
Hard shadow test  : in_shadow ⇔ shadow_zbuf[v][u] + BIAS < ndc.z
PCF (3×3)         : shadow = (1/9) · Σ test(v±1, u±1)
Composite         : col = ambient + (1 − shadow) · (diffuse + specular)
```

### Data Structures
- **Shadow buffer** — `g_shadow_zbuf[SHADOW_H][SHADOW_W]`, float NDC z (range `[−1, +1]`). Reset to `+1.0` (NDC far) each frame before pass 1.
- **Light matrices** — `s->light_view` and `s->light_proj` cached on the Scene; the camera orbit doesn't change them, so they're built once at scene_init.
- **Main G-buffer** — same `g_pos / g_normal / g_albedo / g_zbuf / g_valid` as the deferred pipeline.

### Non-Obvious Decisions
- **Orthographic projection for the light**: a directional sun has parallel rays. Ortho preserves linear depth across the scene and produces uniform shadow texel density. Perspective from the light would over-resolve nearby objects and starve the back of the scene.
- **Bias calibrated to per-texel depth gradient**: per-texel jitter on a flat surface is `texel_world_size · |∇light_z| · (2 / (far − near))` ≈ 0.005 NDC. SHADOW_BIAS = 0.008 sits comfortably above that. Too small → grid acne; too large → "peter-panning" (shadows visibly detach from casters).
- **Outside-frustum returns 0 shadow, not 1**: when the projected NDC isn't in `[−1, +1]`, the fragment isn't visible to the light at all. Treat as fully lit. Treating as shadowed would paint a giant black halo wherever the ortho frustum doesn't cover.
- **Cull / lookAt convention flipped**: this file uses standard glm `cross(forward, up)` in `m4_lookat` AND the **flipped** back-face cull (`area >= 0 → skip`). The flip is needed because the screen Y-flip on its own makes OpenGL CCW front faces produce NEGATIVE signed area. Without the flip the floor's top face — the only side ever visible from above — gets silently culled and there's nothing to receive shadows.
- **PCF only smooths edges**: the centre of every shadow region is pixel-identical between hard and soft modes; only the boundary differs. A 3×3 average gives a 1-cell-wide soft transition.

### Key Constants
| Name | Default | Effect |
|------|---------|--------|
| `SHADOW_W / SHADOW_H` | 256 | shadow-map resolution |
| `LIGHT_ORTHO_HALF` | 6.0 | half-extent of light's frustum (must enclose all receivers) |
| `LIGHT_DISTANCE` | 6.0 | how far back the light eye is along −sun_dir |
| `LIGHT_NEAR / LIGHT_FAR` | 0.5 / 14.0 | depth slab range |
| `SHADOW_BIAS` | 0.008 | NDC offset against acne |

### Modes
- `s` toggles shadows on/off — shadows OFF is the visual reference (no algorithm contribution); ON shows the dark patches.
- `f` toggles HARD ↔ PCF — hard has crisp 1-pixel boundaries, PCF feathers them.

### Open Questions
- What changes if you cull FRONT faces during the shadow pass instead of back faces? (writes farther-from-light depth — alternative acne fix)
- Cascaded shadow maps: how do you split one ortho frustum into multiple resolution slabs?
- Variance shadow maps: store moments instead of depth — what does that buy you?

## From the Source

**Algorithm:** Williams 1978 depth-buffer shadow mapping. Rasterise scene from light POV with ortho projection into a 256² depth-only buffer. Lookup per fragment in the lightpass: project world position into light clip space, compare NDC z against shadow buffer at the projected uv. PCF averages 3×3 binary tests for soft shadow edges.

**Physics/References:** Williams "Casting Curved Shadows on Curved Surfaces" SIGGRAPH '78 (the original); Reeves, Salesin & Cook "Rendering Antialiased Shadows with Depth Maps" SIGGRAPH '87 (PCF); LearnOpenGL "Shadow Mapping" tutorial; Akenine-Möller et al. *Real-Time Rendering* 4ed §7.

**Math:** Linear NDC z in light space (ortho projection). Bias as a constant offset in NDC. PCF as binary average; could be replaced by depth comparison filtering on hardware-PCF-capable GPUs.

**Performance:** Shadow pass cost ≈ tris × pixel-area in the shadow map (~ 1 ms at 256²). PCF adds 9 reads per visible fragment. Both trivial at terminal resolution (~3000 cells).

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_pos[H][W]` | `Vec3[]` | ~115 KB | world position from gbuffer |
| `g_normal[H][W]` | `Vec3[]` | ~115 KB | world normal |
| `g_albedo[H][W]` | `Vec3[]` | ~115 KB | flat surface colour |
| `g_zbuf[H][W]` | `float[]` | ~38 KB | NDC depth (camera) |
| `g_valid[H][W]` | `uint8_t[]` | ~10 KB | covered mask |
| `g_shadow_zbuf[256][256]` | `float[]` | 256 KB | depth from light's POV |
| `s->light_view` / `s->light_proj` | `Mat4` | 128 B | cached lookAt + ortho |

---

## Pass 2 — Implementation

### Pseudocode
```
init:
    light_view = lookAt(−sun_dir · LIGHT_DISTANCE, origin, up)
    light_proj = ortho(±LIGHT_ORTHO_HALF, near, far)

per frame:
    if shadows_on:
        shadow_clear()                          // g_shadow_zbuf := 1.0
        for each object:
            light_mvp = light_proj · light_view · model
            rasterize_object_shadow → write NDC z to g_shadow_zbuf

    render_gbuffer(camera_view, camera_proj)    // standard

    render_lightpass(shadows_on, soft_pcf):
        for r, c in valid pixels:
            P = g_pos[r][c]; N = g_normal[r][c]
            ambient = AMBIENT · albedo
            diff_vec = albedo · sun · max(0, N·L)
            spec_vec = sun · max(0, N·H)^SHININESS · SPEC_GAIN

            shadow = shadow_sample(P, light_view, light_proj, soft_pcf) if shadows_on else 0
            lit = 1 − shadow
            g_light[r][c] = clamp01(ambient + lit · (diff_vec + spec_vec))

    render_scene → paint each cell

shadow_sample(P, V_light, P_light, soft_pcf):
    clip = P_light · V_light · (P, 1)
    ndc  = clip / clip.w
    if any ndc out of [−1, 1]: return 0          // outside frustum → lit
    ix, iy = ndc_to_shadow_texel(ndc)
    z_frag = ndc.z

    if !soft_pcf:
        return shadow_occluded(ix, iy, z_frag) ? 1 : 0

    sum = 0; count = 0
    for dy, dx in 3×3:
        if neighbour valid: sum += shadow_occluded(ix+dx, iy+dy, z_frag)
        count++
    return sum / count

shadow_occluded(ix, iy, z_frag):
    return g_shadow_zbuf[iy][ix] + SHADOW_BIAS < z_frag
```

### Module Map
```
§1  config      — sun, light_distance, ortho_half, shadow size, bias
§2  clock       — monotonic timer + sleep
§3  math        — Vec3 / Mat4 + perspective / orthographic / lookat
§4  paint       — 216 RGB cube + Bourke ramp + paint_cell
§5  mesh        — box / quad / sphere tessellators
§6  gbuffer     — camera-view rasteriser
§7  shadow      — light-view depth-only rasteriser + shadow_sample
§8  lightpass   — Blinn-Phong + (1 − shadow) gating of direct light
§9  scene       — Scene struct, light_view + light_proj cached
§10 screen      — render_scene + HUD
§11 app         — main loop, signals, resize
```

### Data Flow
```
mesh ─┬─→ shadow_pass (light POV, depth only) → g_shadow_zbuf
      │
      └─→ render_gbuffer (camera POV) → g_pos / g_normal / g_albedo

g_pos[r][c] → project to light NDC → shadow_sample → 0..1 shadow factor
shadow + ambient + (1 − shadow) · (diff + spec) → g_light → paint_cell
```
