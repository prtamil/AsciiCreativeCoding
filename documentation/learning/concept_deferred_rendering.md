# Concept: Deferred Rendering Pipeline

## Pass 1 — Understanding

### Core Idea
Deferred rendering splits shading into two passes. Pass 1 (geometry): rasterize all geometry into a G-buffer (position, normal, albedo, material). Pass 2 (lighting): for each screen pixel, read G-buffer data and evaluate lighting. Lights are computed once per visible pixel, not once per geometry-light pair. This decouples scene complexity from lighting cost.

### Mental Model
Forward rendering is like a painter who computes the final color of every pixel of every object, then paints over most of them. Deferred rendering is like first sketching the scene (geometry pass: who is visible where), then coloring it in one pass from the sketch (lighting pass). The G-buffer is the "sketch" — position, normal, color per pixel.

### Key Equations
```
Perspective projection:
  clip.xy = M_proj · M_view · M_model · pos
  NDC = clip / clip.w   (perspective divide)

Blinn-Phong lighting (per G-buffer pixel):
  L̂ = normalize(light_pos − world_pos)
  V̂ = normalize(cam_pos − world_pos)
  Ĥ = normalize(L̂ + V̂)
  diffuse  = albedo · max(N̂·L̂, 0)
  specular = specular_color · max(N̂·Ĥ, 0)^shininess
  L = ambient + Σ_lights [diffuse + specular]

Normal matrix (inverse-transpose):
  N_mat = (M_model⁻¹)ᵀ   (for pure rotation: N_mat = M_model)
```

### Data Structures
- G-buffer: 4 arrays of size ROWS×COLS
  - `gbuf_pos[R][C]` — world position (Vec3)
  - `gbuf_norm[R][C]` — world normal (Vec3)
  - `gbuf_albedo[R][C]` — albedo color index
  - `gbuf_mat[R][C]` — shininess / material flags
- Depth buffer `zbuf[R][C]` — float, initialised to FLT_MAX
- Intermediate framebuffer `cbuf[R][C]` — char + color pair

### Non-Obvious Decisions
- **G-buffer world-space not view-space**: World-space positions and normals are easier to reason about for multiple lights with arbitrary positions. View-space would require transforming every light into view space.
- **Barycentric rasterization**: For each triangle compute barycentric weights (λ0, λ1, λ2) per pixel. Only rasterize if all λ ≥ 0 (inside triangle). Interpolate attributes (position, normal, UV) by λ-weighted average.
- **Bayer dithering not Floyd-Steinberg**: Bayer is O(1) per pixel with no state — correct for parallel/rasterizer evaluation. Floyd-Steinberg needs neighbouring pixels computed first — sequential only.
- **Flat vs smooth normals**: Cube: 24 vertices (4 per face) with replicated normals → hard edges. Sphere: normals = position/radius → smooth gradient. Both from the same rasterizer, just different mesh data.
- **Back-face cull before rasterize**: Discard triangles where (N̂·view_dir) ≥ 0 in clip space. Halves the rasterization work for closed convex objects.

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| SHININESS | 32 | specular exponent (8=matte, 128=metal) |
| AMBIENT_STR | 0.08 | prevents completely black shadows |
| CAM_NEAR | 0.1 | near clip plane (too small → z-fighting) |
| CAM_FAR | 100.0 | far clip plane |
| CELL_W / CELL_H | 8 / 16 | aspect correction for terminal cells |

### Open Questions
- Why does m[3][2]=−1 in the projection matrix enable perspective divide?
- Why is the normal matrix the inverse-transpose (not just the model matrix)?
- What does "deferred" mean in game engine terminology vs this implementation?

## From the Source

**Algorithm:** Two-pass software rasterizer: geometry pass writes G-buffer per fragment; lighting pass reads G-buffer and evaluates Blinn-Phong for all lights. Barycentric rasterization with back-face cull. Bayer 4×4 dithering + Paul Bourke ASCII ramp.

**Physics/References:** Blinn-Phong shading: Phong (1975), Blinn half-vector (1977). Deferred shading: Deering et al. (1988), popularised by Killzone 2 (2009). G-buffer layout: Unreal Engine 5 GBufferA/B/C, Unity HDRP RT0–RT3. Normal matrix: Shirley & Marschner "Fundamentals of Computer Graphics."

**Math:** Barycentric coordinates: λi = signed_area(opposite_triangle) / total_area. Back-face cull: dot(face_normal_clip, (0,0,1)) < 0 → back facing. Perspective divide: NDC = clip.xyz / clip.w, where clip.w = −z_view from m[3][2]=−1.

**Performance:** Pass 1: O(triangles × pixels_covered) for rasterization. Pass 2: O(ROWS × COLS × lights) for lighting. For 4 lights, 120×50 grid: 24k lighting evaluations per frame.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `gbuf_pos[R][C]` | `Vec3[R×C]` | ~115 KB | world-space position |
| `gbuf_norm[R][C]` | `Vec3[R×C]` | ~115 KB | world-space normal |
| `gbuf_albedo[R][C]` | `uint8_t[R×C]` | ~10 KB | albedo color index |
| `gbuf_mat[R][C]` | `float[R×C]` | ~38 KB | material shininess |
| `zbuf[R][C]` | `float[R×C]` | ~38 KB | depth (FLT_MAX=empty) |
| `cbuf[R][C]` | `Cell {ch, cp}[R×C]` | ~19 KB | intermediate framebuffer |

---

## Pass 2 — Implementation

### Pseudocode
```
geometry_pass(mesh_list):
    clear(gbuf_pos, gbuf_norm, gbuf_albedo, gbuf_mat)
    fill(zbuf, FLT_MAX)

    for each mesh:
        for each triangle (v0, v1, v2):
            # vertex shader: model → clip space
            c0 = M_proj * M_view * M_model * v0
            c1 = ...; c2 = ...

            # back-face cull in clip space
            if cross(c1-c0, c2-c0).z >= 0: skip

            # perspective divide → NDC → screen
            screen0 = NDC_to_screen(c0 / c0.w)
            ...

            # rasterize: bounding box + barycentric test
            for px,py in bounding_box(screen0..screen2):
                λ = barycentric(px, py, screen0..screen2)
                if any(λ < 0): continue

                z = interpolate(c0.w, c1.w, c2.w, λ)
                if z >= zbuf[py][px]: continue
                zbuf[py][px] = z

                # fragment shader: write G-buffer
                gbuf_pos[py][px]    = interpolate(wp0, wp1, wp2, λ)
                gbuf_norm[py][px]   = normalize(interp normals)
                gbuf_albedo[py][px] = mesh.albedo
                gbuf_mat[py][px]    = mesh.shininess

lighting_pass(lights, cam_pos):
    for py, px in all pixels:
        if gbuf_albedo[py][px] == EMPTY: clear(cbuf); continue

        P = gbuf_pos[py][px]
        N = normalize(gbuf_norm[py][px])
        V = normalize(cam_pos − P)
        color = AMBIENT_STR * albedo

        for each light:
            L = normalize(light.pos − P)
            H = normalize(L + V)
            atten = 1 / (dist(light.pos, P)²)
            color += albedo * max(dot(N,L), 0) * atten
            color += max(dot(N,H), 0)^shininess * atten

        luma = luminance(color)
        thresh = bayer[py%4][px%4]
        ri = clamp(int((luma + thresh) * RAMP_LEN), 0, RAMP_LEN-1)
        cbuf[py][px] = {ramp[ri], color_pair(albedo)}
```

### Module Map
```
§1 config        — SHININESS, AMBIENT_STR, CAM_*, CELL_W/H, Bayer matrix
§2 math          — Vec3, Mat4, perspective, lookat, normal_mat
§3 mesh          — Vertex, Triangle, tessellate_cube/sphere/plane
§4 G-buffer      — gbuf_pos/norm/albedo/mat arrays, zbuf, cbuf
§5 geometry pass — transform, cull, rasterize, fragment write
§6 lighting pass — Blinn-Phong per G-buffer pixel
§7 blitting      — cbuf → ncurses mvaddch
§8 scene         — object list, light presets, animation
§9 app           — main loop, keys, SIGWINCH
```

### Data Flow
```
mesh vertices → vertex shader → clip space
clip space → cull → NDC → screen coords
screen coords → barycentric rasterize → fragment
fragment → z-test → G-buffer write

G-buffer → Blinn-Phong per pixel → luminance
luminance → Bayer dither → ASCII ramp → cbuf
cbuf → fb_blit → ncurses → screen
```
