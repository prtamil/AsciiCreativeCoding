/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shadow_mapping.c — software directional-light shadow mapping
 *
 * DEMO: A small open-air scene — a slate floor, one tall sandstone
 *       pillar, one squat terracotta cube, and a cream sphere — lit
 *       by a single warm directional sun coming from the upper right.
 *       Each object casts a hard, sharply attached shadow onto the
 *       floor and onto its neighbours. The camera orbits slowly so
 *       you watch the long pillar shadow swing across the floor and
 *       the round sphere shadow track the sphere as it moves through
 *       the frame.
 *
 *       Press 's' to toggle shadows on/off — the difference is the
 *       entire algorithm's contribution. With shadows OFF every
 *       face that's geometrically lit gets full direct light, even
 *       when something is in the way. With shadows ON, fragments
 *       behind a closer occluder lose their direct light and drop
 *       to ambient only — which is exactly what shadow mapping
 *       reproduces from a single depth photo of the scene taken
 *       from the sun's POV.
 *
 *       Press 'f' to toggle hard ↔ soft (PCF) shadows. PCF averages
 *       a 3×3 neighbourhood of the shadow map per fragment, so the
 *       shadow EDGE feathers from sharp pixel-boundary to a soft
 *       gradient. That's the cheapest "soft shadow" trick; the
 *       interior darkness is unchanged.
 *
 * Study alongside:
 *   raster/ssao_pipeline.c               — same G-buffer + raster path
 *   raster/deferred_rendering_pipeline.c — same MVP + Blinn-Phong
 *   raster/marching_cubes.c              — same lightpass shape + rim
 *
 * Section map:
 *   §1  config     — frame, view, sun, shadow map, scene, ramp
 *   §2  clock      — monotonic timer + sleep
 *   §3  math       — V3, V4, Mat4 + perspective / orthographic / lookat
 *   §4  paint      — 216-pair RGB cube + Bourke ramp + paint_cell
 *   §5  mesh       — Vertex / Triangle types + box / quad / sphere
 *   §6  gbuffer    — camera-view geometry pass (pos, normal, albedo, z)
 *   §7  shadow     — light-view DEPTH-ONLY pass into a 2-D shadow map
 *   §8  lightpass  — Blinn-Phong, with shadow lookup (hard or PCF)
 *   §9  scene      — Scene struct, init / view / tick
 *   §10 screen     — render_scene + HUD (CLAUDE.md spec)
 *   §11 app        — signals, resize, fixed-step main loop
 *
 * Keys:
 *   s / S     toggle shadows on/off (compare with vs without)
 *   f / F     toggle filter: hard ↔ soft (3×3 PCF)
 *   + / =     zoom in
 *   - / _     zoom out
 *   space     pause / resume camera orbit
 *   r / R     reset
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/shadow_mapping.c -o shadow -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Shadow mapping (Lance Williams, 1978). Take a depth
 *                  photograph of the scene FROM THE LIGHT'S POV — that
 *                  is, render every triangle with the light as the
 *                  camera and write only NDC depth into a 2-D buffer
 *                  (the shadow map). Then in the main render, for each
 *                  fragment project its world position back into the
 *                  light's clip space, look up the shadow map at that
 *                  (u,v), and compare the fragment's light-space depth
 *                  against the stored value. If something closer to
 *                  the light was already recorded at that texel, the
 *                  fragment is occluded — drop its direct lighting,
 *                  keep ambient.
 *
 *                  Two passes, two cameras, one comparison. No rays.
 *                  The same algorithm OpenGL / Vulkan / D3D engines
 *                  use for sun shadows in every modern game.
 *
 * Data-structure : One float buffer of size SHADOW_W × SHADOW_H storing
 *                  NDC z (range [-1, +1]). Reset to +1.0 each frame
 *                  before pass 1. Plus one orthographic light_proj
 *                  matrix and a light_view matrix that puts the
 *                  light's eye behind the scene along sun_dir.
 *
 *                  The main G-buffer (g_pos, g_normal, g_albedo,
 *                  g_zbuf, g_valid) is the same as in deferred /
 *                  SSAO files.
 *
 * Rendering      : Three passes per frame:
 *                    1. shadow_pass  — rasterise scene from light POV
 *                                      into the shadow map (depth only)
 *                    2. render_gbuffer — main camera, full G-buffer
 *                    3. lightpass    — Blinn-Phong + shadow lookup
 *                                      against the shadow map
 *                  Then paint each cell through the standard RGB →
 *                  Bourke ramp pipeline.
 *
 * Performance    : Shadow pass cost ≈ tris × pixel-area in the shadow
 *                  map. With ~30 triangles and a 256² shadow map
 *                  that's well under a millisecond. PCF adds 9 shadow
 *                  reads per visible fragment — still trivial at
 *                  terminal resolution (≈ 3 000 cells).
 *
 * References     : Williams, "Casting Curved Shadows on Curved
 *                    Surfaces," SIGGRAPH '78. The original.
 *                  LearnOpenGL, "Shadow Mapping" tutorial:
 *                    https://learnopengl.com/Advanced-Lighting/Shadows/Shadow-Mapping
 *                  Reeves, Salesin & Cook, "Rendering Antialiased
 *                    Shadows with Depth Maps," SIGGRAPH '87 (PCF).
 *                  Möller, "Fast Triangle Rasterization by
 *                    Interpolating Edge Functions," GPG (2000).
 *                  Reinhard et al., "Photographic Tone Reproduction
 *                    for Digital Images," SIGGRAPH '02.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * "Is point P in shadow of light L?" is the same question as "is some
 * surface closer to L than P along the ray P → L?" Tracing that ray
 * is expensive. But the LIGHT sees the closest surface in every
 * direction whenever you render the scene from the light's camera —
 * that's literally what a depth buffer records. So render the scene
 * from L once, store the depth buffer, and afterwards every shading
 * pixel just LOOKS UP the answer instead of tracing.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the sun has a camera and snaps a depth photo of the scene
 * — at every pixel of the photo it writes "this is how far the
 * closest thing was from me in this direction". Now any point in
 * 3-D can be placed into the sun's photo by projecting through the
 * sun's lens. If the photo's depth at that pixel is SMALLER than
 * the point's actual distance to the sun, then something was IN
 * THE WAY between the sun and the point. That point is in shadow.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │             LIGHT POV (orthographic)         │
 *      │                                              │
 *      │   sun ──── ▶                                 │
 *      │            ┌──┐    ▼                         │
 *      │            │██│   shadow_map[u,v] = depth    │
 *      │       ─────┴──┴──── floor                    │
 *      │                                              │
 *      │   At fragment F on floor:                    │
 *      │     project F into light space → (u,v,z_F)   │
 *      │     if shadow_map[u,v] + bias < z_F          │
 *      │         → something closer than F is there   │
 *      │         → F is in shadow                     │
 *      └──────────────────────────────────────────────┘
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 *
 *   1. Build light_view = lookAt(light_eye, scene_center, up)
 *      and  light_proj = ortho(left, right, bottom, top, near, far)
 *      Light_eye is placed along −sun_dir at LIGHT_DISTANCE.
 *
 *   2. SHADOW PASS — rasterise EVERY object using light_proj·light_view
 *      as the MVP. At each cube of the shadow map, write the smallest
 *      NDC z seen. NO colour, NO normal — just depth.
 *
 *   3. G-BUFFER PASS — rasterise EVERY object using camera_proj·view
 *      as the MVP. Write world pos, world normal, albedo, NDC z into
 *      the main G-buffer.
 *
 *   4. LIGHT PASS — for each visible camera pixel:
 *        P = g_pos[r][c],  N = g_normal[r][c],  albedo = g_albedo[r][c]
 *        ambient   = AMBIENT · albedo
 *        light_clip = light_proj · light_view · (P, 1)
 *        light_ndc  = light_clip / light_clip.w        (= clip for ortho)
 *        if light_ndc outside [−1, +1]³:
 *            shadow = 0          (P is outside the light's frustum →
 *                                 not visible to the light → not in
 *                                 shadow either; let direct light through)
 *        else:
 *            (u, v) = NDC → shadow-map cell with Y-flip
 *            if HARD:
 *                shadow = (shadow_map[u,v] + BIAS < light_ndc.z) ? 1 : 0
 *            else PCF:
 *                shadow = mean over 3×3 of the same comparison
 *        diffuse  = albedo · sun_col · max(0, N·L)
 *        specular = sun_col · max(0, N·H)^SHININESS · SPEC_GAIN
 *        direct   = (diffuse + specular) · (1 − shadow)
 *        out      = clamp01(ambient + direct)
 *
 * KEY FORMULAS
 * ────────────
 *   Light eye:        light_eye = scene_center − sun_dir · LIGHT_DISTANCE
 *   Light MVP:        L_mvp     = light_proj · light_view · model
 *   Light clip:       light_clip = L_mvp · (P, 1)
 *   Light NDC:        ndc       = light_clip / light_clip.w
 *   Shadow UV:        u = ( ndc.x + 1)/2 · SHADOW_W
 *                     v = (−ndc.y + 1)/2 · SHADOW_H        (Y-flip)
 *   Shadow test:      in_shadow ⇔ shadow_map[v][u] + BIAS < ndc.z
 *   PCF (3×3):        shadow = (1/9) · Σ[dy∈{−1,0,1}, dx∈{−1,0,1}] test(v+dy,u+dx)
 *   Composite:        out = ambient + (1 − shadow) · (diffuse + specular)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Outside the light's frustum — fragments whose NDC isn't in
 *     [-1, +1] aren't visible to the light at all. Returning 0
 *     (NOT shadowed) is the safe default; returning 1 would create
 *     an enormous black halo around any small ortho frustum.
 *
 *   • Shadow acne — without bias, a flat surface's own depth
 *     samples in the shadow map are essentially equal to its
 *     fragment's light-space z. Tiny float jitter then causes
 *     "every other pixel is in shadow" striping. SHADOW_BIAS pushes
 *     the comparison threshold slightly away so flat regions
 *     reliably read NOT shadowed.
 *
 *   • Peter-Panning — too much bias makes shadows visibly DETACH
 *     from their casters (sphere sits above its own shadow). Pick
 *     the smallest bias that hides the acne. SHADOW_BIAS in §1.4
 *     is calibrated for this scene's worst-case depth gradient.
 *
 *   • Shadow map resolution — SHADOW_W × SHADOW_H sets the smallest
 *     shadow feature you can resolve. Too low → blocky shadows.
 *     Too high → memory + slower rasterise. 256² is fine here.
 *
 *   • Y-flip consistency — both the rasterise pass AND the lookup
 *     pass must use the same NDC.y → row mapping. We use the same
 *     "(−ndc.y + 1)/2 · H" formula in both places.
 *
 *   • Backface caster — culling back faces during the shadow pass
 *     prevents the BACK side of an object from writing the shadow
 *     map and self-shadowing the front. We use the same cull as the
 *     main pass for consistency; bias handles the rest.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Toggle 's' off: every face that's geometrically lit gets full
 *     direct light, even when blocked. The pillar's shadow on the
 *     floor disappears; the scene looks "wrong" — that's how you
 *     know shadow mapping is doing real work.
 *
 *   • Toggle 'f': hard shadows have crisp pixel-boundary edges; PCF
 *     gives a 1-cell-wide soft transition along the same edge.
 *     Centres of large shadow regions are pixel-identical between
 *     the two modes — only edges differ.
 *
 *   • Watch the long pillar shadow slide across the floor as the
 *     camera orbits. The shadow length and direction depend on
 *     SUN_DIR, NOT on camera position — verify by pausing (space)
 *     mid-orbit and confirming the shadow stays put.
 *
 *   • The sphere casts an oval shadow on the floor; if the shadow
 *     looks faceted (visible polygons), the sphere mesh is too low-
 *     poly OR the shadow map resolution is too high relative to
 *     the screen. Default settings give a smooth oval.
 *
 *   • If you see striped / checker pattern on lit flat surfaces,
 *     SHADOW_BIAS is too small (acne). If shadows visibly float
 *     above their casters, SHADOW_BIAS is too large (peter-panning).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read raster/cube_raster.c first if you don't yet
 *      know the 7-stage forward pipeline; this file runs that
 *      pipeline TWICE per frame, once from the sun and once from
 *      the camera.
 *   2. §1 config — every constant has a unit-bearing comment.
 *   3. §7 shadow_pass + §8 lightpass are THE TWO HEART functions.
 *      Read AFTER tutorials T1-T4. The shadow pass is a stripped-
 *      down rasteriser (depth only, no normal/colour); the light
 *      pass does the per-fragment shadow lookup.
 *   4. §3 math — projections (perspective + orthographic + lookAt).
 *      Orthographic is unique to this file; T2 explains why.
 *   5. §6 gbuffer — same pattern as the other deferred files.
 *      Read AFTER cube_raster.c if you haven't.
 *   6. §4 paint + §9-§11 — infrastructure; skip on first read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   light_view / light_proj / light_mvp   matrices for the SUN's
 *                                         camera (pass 1)
 *   shadow_map[v][u]                      flat 2-D float buffer of
 *                                         depths from the sun's POV
 *   light_ndc / light_clip                fragment's position projected
 *                                         into the sun's clip / NDC space
 *   shadow                                ∈ [0, 1]; 0 = unshadowed,
 *                                         1 = fully shadowed
 *   SHADOW_BIAS                           tiny offset added to the
 *                                         comparison threshold to
 *                                         avoid self-shadowing acne
 *   PCF                                   "Percentage-Closer Filtering"
 *                                         — the 3×3 average soft-shadow
 *                                         trick
 *
 * Background you need
 * ───────────────────
 *   - The 7-stage rasteriser (cube_raster.c).
 *   - Orthographic projection: parallel-ray analogue of perspective.
 *     Same final NDC mapping; no perspective divide effect.
 *   - Why a depth buffer (z-buffer) records the closest surface per
 *     pixel.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Variance shadow maps, exponential shadow maps — we use plain
 *     hard-test + PCF, the simplest variant.
 *   - Cascaded shadow maps (sun across an entire game world) —
 *     we have one ortho frustum, scene fits in it.
 *   - Shadow volumes (a different algorithm using stencil buffers).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Seven short tutorials that build shadow mapping from first
 * principles. Read in order; each builds on the previous.
 *
 *   T1  The shadow-mapping principle — render twice, compare depth
 *   T2  Orthographic projection — why a directional sun uses ortho
 *   T3  Pass 1: depth-only rasterisation from the light's POV
 *   T4  Pass 2: per-fragment shadow lookup
 *   T5  Shadow acne and the bias workaround (Peter-Panning trade)
 *   T6  PCF — 3×3 average for soft shadow edges
 *   T7  Outside-the-frustum and other defensible defaults
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  THE SHADOW-MAPPING PRINCIPLE — RENDER TWICE, COMPARE DEPTH
 * ───────────────────────────────────────────────────────────────
 * "Is point P in shadow of light L?" is exactly the same question
 * as "is some surface closer to L than P along the ray P → L?"
 * Tracing that ray every frame from every pixel is expensive.
 *
 * Williams (1978) realised: when you render the scene FROM L's
 * point of view, the depth buffer at each light-pixel ALREADY
 * stores the distance to the closest surface in that direction.
 * That's a precomputed answer to the shadow question for every
 * direction the light sees.
 *
 * Algorithm:
 *
 *   1. Render the scene from L's POV. Output: a depth buffer
 *      called the SHADOW MAP.
 *   2. Render the scene from the camera's POV. For each fragment
 *      F:
 *      a. Project F's world position INTO the light's clip space.
 *      b. Look up the shadow map at F's (u, v) light-space pixel.
 *      c. Compare the stored depth against F's light-space depth.
 *      d. If the stored depth is SMALLER, something closer to L
 *         was already there → F is in shadow.
 *
 * One render of the scene from L's POV. One depth comparison per
 * fragment in the main render. No rays, no bounce, no recursion.
 *
 * Two passes, two cameras, one comparison. That's the entire
 * algorithm. Everything below is implementation details.
 *
 * T2  ORTHOGRAPHIC PROJECTION — WHY A DIRECTIONAL SUN USES ORTHO
 * ───────────────────────────────────────────────────────────────
 * The camera uses PERSPECTIVE projection (T3 in cube_raster.c) —
 * a frustum that narrows toward the eye. Rays from a perspective
 * camera DIVERGE outward.
 *
 * The SUN is treated as DIRECTIONAL — infinitely far away, all rays
 * parallel, no convergence point. The right projection for a
 * directional light is ORTHOGRAPHIC:
 *
 *   PERSPECTIVE   frustum, narrows at eye, divides by w
 *   ORTHOGRAPHIC  rectangular box (no narrowing), no perspective
 *                 divide effect (after the divide, x/w = x because
 *                 w = 1 throughout)
 *
 * Mathematically, the ortho matrix maps a 3-D box [l, r] × [b, t] ×
 * [n, f] into NDC [-1, +1]³ via translate-and-scale only:
 *
 *     m[0][0] =  2 / (r - l)         m[0][3] = -(r + l) / (r - l)
 *     m[1][1] =  2 / (t - b)         m[1][3] = -(t + b) / (t - b)
 *     m[2][2] = -2 / (f - n)         m[2][3] = -(f + n) / (f - n)
 *     m[3][3] =  1
 *
 * Apply it to (x, y, z, 1) and you get clip coords with w = 1
 * (no divide ever does anything). The "depth" stored in the shadow
 * map is just the z component of the result.
 *
 * Geometrically: the light's frustum is a BOX, big enough to
 * contain every shadow-casting object in the scene. That box's
 * cross-section maps to the shadow map's (u, v) grid; the box's
 * length-along-light-direction is mapped to z and stored.
 *
 * Read §1 LIGHT_PROJ_HALF + §3 m4_orthographic for the constants
 * and matrix builder.
 *
 * T3  PASS 1: DEPTH-ONLY RASTERISATION FROM THE LIGHT'S POV
 * ─────────────────────────────────────────────────────────
 * The shadow pass is the standard rasteriser STRIPPED DOWN — we
 * write only depth, no colour, no normal:
 *
 *   shadow_pass(scene, light_view, light_proj):
 *     shadow_map[*][*] = +1.0                  (clear to far plane)
 *     for each triangle:
 *       v0, v1, v2 = light_proj · light_view · vertex_world_pos
 *       if all three behind near plane: skip
 *       (u0, v0, z0) = NDC mapping (T3 in cube_raster)
 *       ... same screen mapping ...
 *       for each cell in triangle's bbox:
 *         compute barycentric (b0, b1, b2)
 *         if any < 0: outside, skip
 *         z = b0·z0 + b1·z1 + b2·z2          (interpolated NDC z)
 *         if z < shadow_map[v][u]:           (closer than current)
 *           shadow_map[v][u] = z
 *
 * No fragment shader, no surface attributes carried, no
 * back-face cull strictly required (back-facing shadow casters
 * write a slightly-larger depth that gets overwritten by the
 * front-facing surface anyway). The output is a 2-D buffer of
 * depth values from the light's POV.
 *
 * For our scene with ~30 triangles and a 256² shadow map, this
 * pass is sub-millisecond.
 *
 * Read §7 shadow_pass for the implementation.
 *
 * T4  PASS 2: PER-FRAGMENT SHADOW LOOKUP
 * ───────────────────────────────────────
 * The main render goes through the standard pipeline (G-buffer +
 * Blinn-Phong from cube_raster.c et al). The new step is in the
 * lightpass: per fragment, compute "am I in shadow?" and use that
 * to gate the direct lighting.
 *
 * lightpass per pixel:
 *
 *   P     = g_pos[r][c]                       fragment world pos
 *   N     = g_normal[r][c]                    fragment normal
 *   albedo = g_albedo[r][c]
 *
 *   ambient = AMBIENT · albedo                always present
 *
 *   light_clip = light_proj · light_view · (P, 1)
 *   light_ndc  = light_clip / light_clip.w        (≈ light_clip — ortho)
 *
 *   if light_ndc outside [-1, +1]³:
 *     shadow = 0                              not in light's frustum
 *   else:
 *     u = ( light_ndc.x + 1)/2 · SHADOW_W
 *     v = (-light_ndc.y + 1)/2 · SHADOW_H     Y-flip (T4 cube_raster)
 *     stored = shadow_map[v][u]
 *     if stored + BIAS < light_ndc.z:         something closer first
 *       shadow = 1                            (fully in shadow)
 *     else:
 *       shadow = 0                            (not shadowed)
 *
 *   diffuse  = albedo · sun_col · max(0, N · L)
 *   specular = sun_col · max(0, N · H)^SHININESS · SPEC_GAIN
 *
 *   direct = (diffuse + specular) · (1 - shadow)
 *
 *   out = ambient + direct
 *
 * Note that ambient is NEVER multiplied by (1 - shadow) — even
 * shadowed fragments still pick up indirect/skylight illumination,
 * just not the sun's direct rays.
 *
 * Read §8 render_lightpass.
 *
 * T5  SHADOW ACNE AND THE BIAS WORKAROUND
 * ────────────────────────────────────────
 * Without bias, a flat surface like a floor casts a shadow ON
 * ITSELF. Why?
 *
 * The shadow map records depth from the light's POV. When we test
 * a floor fragment F, we project F into light space and compare
 * its depth against shadow_map[u][v]. But the shadow map at (u, v)
 * recorded a DIFFERENT FLOOR PIXEL — the one that the light's ray
 * happened to land on at the centre of that texel.
 *
 *   shadow_map[u][v] ≈ floor depth at the texel centre
 *   F's depth        ≈ floor depth at F's exact light-space position
 *
 * For a flat floor these should be identical. With float jitter,
 * SOMETIMES the recorded value is slightly less than F's; SOMETIMES
 * slightly more. The result is striped "shadow acne" — alternating
 * lit/shadow pixels across the floor.
 *
 * The fix: add a SMALL POSITIVE BIAS to the recorded value before
 * comparing:
 *
 *   if stored + BIAS < light_ndc.z:  shadow
 *
 * This shifts the comparison so flat surfaces reliably read as
 * NOT shadowed. Bias = 0.001 is enough for our scene.
 *
 * Trade-off — PETER-PANNING:
 *   too much bias → shadows DETACH from their casters. A sphere
 *   appears to FLOAT above its own shadow. Looks wrong.
 *
 * The right bias is the smallest one that eliminates acne for THIS
 * scene's worst-case depth gradient. SHADOW_BIAS in §1 is calibrated
 * for our pillar / cube / sphere mix.
 *
 * Sophisticated alternatives (slope-scaled bias, depth gradient
 * bias) compute bias per-fragment based on local surface slope.
 * Worth knowing exists; not implemented here.
 *
 * T6  PCF — 3×3 AVERAGE FOR SOFT SHADOW EDGES
 * ────────────────────────────────────────────
 * Hard shadow mapping returns 0 or 1 — every fragment is either
 * fully lit or fully shadowed. The shadow EDGE is therefore a
 * stair-step at the shadow-map's resolution.
 *
 * Percentage-Closer Filtering (Reeves, Salesin & Cook 1987) softens
 * the edge by averaging multiple shadow tests:
 *
 *   pcf:
 *     hits = 0
 *     for dy ∈ {-1, 0, +1}:
 *       for dx ∈ {-1, 0, +1}:
 *         u' = u + dx;  v' = v + dy
 *         if shadow_map[v'][u'] + BIAS < light_ndc.z: hits++
 *     shadow = hits / 9.0                       ∈ [0, 1] in steps of 1/9
 *
 * Now the edge feathers from 0/9 (lit) through 1/9, 2/9, ..., 8/9
 * to 9/9 (fully shadowed). The transition is 3 shadow-map texels
 * wide — soft enough to read as a real penumbra at terminal scale.
 *
 * INTERIOR shadow darkness is unchanged (every cell of the 3×3
 * block agrees → 9/9). Only the EDGE softens. That's exactly what
 * you want.
 *
 * Cost: 9 shadow-map reads per fragment instead of 1. Trivial at
 * terminal resolution; can matter on a real GPU at 4K.
 *
 * Press 'f' to toggle hard ↔ PCF and watch the edges change.
 *
 * T7  OUTSIDE-THE-FRUSTUM AND OTHER DEFENSIBLE DEFAULTS
 * ──────────────────────────────────────────────────────
 * What if a fragment projects to (u, v) OUTSIDE the shadow map?
 *
 *   light_ndc.x or .y outside [-1, +1]   →   off the shadow texture
 *   light_ndc.z outside [-1, +1]         →   beyond the light's near
 *                                            or far plane
 *
 * The wrong default: clamp (u, v) to the edge and use the edge
 * texel. Border texels would create a HUGE BLACK HALO around any
 * fragment behind the light's frustum.
 *
 * The right default: shadow = 0. "If the fragment isn't even in
 * the light's view, the light can't possibly know whether
 * something occludes it — assume not in shadow."
 *
 * This is the usual convention in real shadow-mapping implementations
 * (often via a "border texture" or "outside-clamp = white" mode in
 * the texture sampler). For our software version it's a simple early
 * return:
 *
 *   if light_ndc.x or .y outside [-1, +1]: return 0
 *   if light_ndc.z outside [-1, +1]:       return 0
 *
 * The safest thing the algorithm can do when it doesn't have data.
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <float.h>
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI */
enum {
    FPS_TARGET    = 60,
    FPS_UPDATE_MS = 500,
    HUD_ROWS      = 5,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define DT_CAP_NS       (100 * NS_PER_MS)

/* §1.2 G-buffer dimensions (static; sized for a large terminal). */
#define GBUF_MAX_W   300
#define GBUF_MAX_H    80

/* §1.3 view geometry — eye orbits the scene at fixed elevation. */
#define CAM_FOV       (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR      0.1f
#define CAM_FAR       50.0f

/* Camera elevated to roughly 25° above the horizon so the FLOOR
 * (where every shadow lives) fills the lower half of the screen
 * instead of being a pencil-thin strip. */
#define CAM_DIST       6.5f
#define CAM_DIST_MIN   3.5f
#define CAM_DIST_MAX  12.0f
#define CAM_ZOOM_STEP  0.4f
#define CAM_EYE_Y      3.2f
#define CAM_LOOK_Y     0.3f
#define CAM_ORBIT_RAD_PER_SEC  0.18f

#define CELL_W        8
#define CELL_H       16

/* §1.4 shadow mapping
 *
 * SUN_DIR is the direction the light TRAVELS (from sun toward scene).
 * The light's "eye" sits at -SUN_DIR · LIGHT_DISTANCE looking back at
 * the origin. ~40° elevation gives shadows that stretch ~2× the
 * caster's height — much more visible than steep noon shadows.
 *
 * LIGHT_ORTHO_HALF must enclose every shadow receiver in the light's
 * view. Anything outside is "not visible to the light" and never
 * shadows. 6.0 gives margin around the 8×8 floor at this sun angle.
 *
 * SHADOW_BIAS is the smallest NDC-z gap that counts as occlusion.
 * Without it, flat surfaces stripe themselves on float jitter
 * ("acne"); too much and shadows visibly detach from casters
 * ("peter-panning"). Calibrate against the worst-case per-texel
 * depth gradient:
 *   texel_world_size = 2·ORTHO_HALF / SHADOW_W   ≈ 0.047
 *   |∇light_z| on floor ≈ |sun_dir.xz|           ≈ 0.75
 *   NDC factor          = 2/(LIGHT_FAR-NEAR)     ≈ 0.148
 *   per-texel NDC jitter ≈ texel × ∇ × NDC factor ≈ 0.005
 * 0.008 is comfortably above that on every surface here. */
static const float SUN_DIR[3] = { -0.55f, -0.55f, 0.30f };
#define LIGHT_DISTANCE     6.0f
#define LIGHT_ORTHO_HALF   6.0f
#define LIGHT_NEAR         0.5f
#define LIGHT_FAR         14.0f

#define SHADOW_W          256
#define SHADOW_H          256
#define SHADOW_BIAS    0.008f

/* §1.5 lighting — single warm sun + cool ambient.
 *
 * AMBIENT is kept LOW so shadowed regions visibly drop in luma vs
 * lit regions; raising it would wash out the demo's whole point.
 * Shadow scales the direct terms (diffuse + specular) only — ambient
 * is the "lift" that keeps shadowed pixels visible. */
static const float SUN_COL[3]    = { 0.98f, 0.88f, 0.68f };
static const float AMBIENT_COL[3]= { 0.18f, 0.20f, 0.26f };
#define SHININESS    24.0f
#define SPEC_GAIN     0.30f

/* §1.6 scene geometry — floor + tall pillar + cube + sphere.
 *
 *                       sun ─── ▶ ▼
 *                                │
 *                       │█│      │      ┌──┐
 *                       │█│              │ ●  ← sphere
 *                       │█│ pillar       └──┘  ← cube
 *                  ─────┴──┴──────────────  floor
 *
 * Each object casts a clearly-shaped shadow onto the floor:
 *   • the pillar throws a long thin rectangle
 *   • the cube throws a short square patch
 *   • the sphere throws a smooth oval
 *
 * Camera orbits, so the shadows visibly track the geometry from
 * every viewing angle — they belong to the WORLD, not the screen. */
#define FLOOR_HALF_X     4.0f
#define FLOOR_HALF_Z     4.0f

#define PILLAR_HX  0.30f
#define PILLAR_HY  1.20f       /* tall — long shadow                  */
#define PILLAR_HZ  0.30f
#define PILLAR_CX  -1.10f
#define PILLAR_CY  (PILLAR_HY) /* sit on floor                         */
#define PILLAR_CZ  0.40f

#define CUBE_HALF  0.55f
#define CUBE_CX    0.80f
#define CUBE_CY    (CUBE_HALF)
#define CUBE_CZ   -0.50f

#define SPHERE_R       0.55f
#define SPHERE_RINGS   12
#define SPHERE_SEGS    18
#define SPHERE_CX      0.10f
#define SPHERE_CY      (SPHERE_R)
#define SPHERE_CZ     -1.40f

enum {
    OBJ_FLOOR = 0,
    OBJ_PILLAR,
    OBJ_CUBE,
    OBJ_SPHERE,
    N_OBJECTS,
};

/* §1.7 character ramp — Paul Bourke 92-char density ladder. */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.8 Bayer 4×4 dither. */
static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};
#define DITHER_AMP   0.10f

/* §1.9 ncurses pair IDs — 216 cube + yellow HUD + cyan hint. */
#define PAIR_CUBE_BASE   1
#define PAIR_HUD       217
#define PAIR_HINT      218

/* ── §2 clock ────────────────────────────────────────────────────────── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                          .tv_nsec = (long)  (ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ── §3 math (V3, V4, Mat4) ──────────────────────────────────────────── */

typedef struct { float x, y, z;    } Vec3;
typedef struct { float x, y, z, w; } Vec4;
typedef struct { float m[4][4];    } Mat4;

static inline Vec3 v3(float x, float y, float z)         { return (Vec3){x,y,z}; }
static inline Vec4 v4(float x, float y, float z, float w){ return (Vec4){x,y,z,w}; }

static inline Vec3  v3_add  (Vec3 a, Vec3 b)  { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vec3  v3_sub  (Vec3 a, Vec3 b)  { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3  v3_scale(Vec3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }
static inline Vec3  v3_neg  (Vec3 a)          { return v3(-a.x, -a.y, -a.z); }
static inline float v3_dot  (Vec3 a, Vec3 b)  { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3_len  (Vec3 a)          { return sqrtf(v3_dot(a, a)); }
static inline Vec3  v3_norm (Vec3 a)
{
    float l = v3_len(a);
    return l > 1e-7f ? v3_scale(a, 1.f/l) : v3(0, 1, 0);
}
static inline Vec3 v3_bary(Vec3 p0, Vec3 p1, Vec3 p2,
                           float b0, float b1, float b2)
{
    return v3(b0*p0.x + b1*p1.x + b2*p2.x,
              b0*p0.y + b1*p1.y + b2*p2.y,
              b0*p0.z + b1*p1.z + b2*p2.z);
}

static inline Mat4 m4_identity(void)
{
    Mat4 m = {{{0}}};
    m.m[0][0] = m.m[1][1] = m.m[2][2] = m.m[3][3] = 1.f;
    return m;
}

static inline Vec4 m4_mul_v4(Mat4 m, Vec4 v)
{
    return v4(m.m[0][0]*v.x + m.m[0][1]*v.y + m.m[0][2]*v.z + m.m[0][3]*v.w,
              m.m[1][0]*v.x + m.m[1][1]*v.y + m.m[1][2]*v.z + m.m[1][3]*v.w,
              m.m[2][0]*v.x + m.m[2][1]*v.y + m.m[2][2]*v.z + m.m[2][3]*v.w,
              m.m[3][0]*v.x + m.m[3][1]*v.y + m.m[3][2]*v.z + m.m[3][3]*v.w);
}

static inline Mat4 m4_mul(Mat4 a, Mat4 b)
{
    Mat4 r = {{{0}}};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

static inline Vec3 m4_pt(Mat4 m, Vec3 p)
{
    Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
    return v3(r.x, r.y, r.z);
}

static inline Vec3 m4_dir(Mat4 m, Vec3 d)
{
    Vec4 r = m4_mul_v4(m, v4(d.x, d.y, d.z, 0.f));
    return v3(r.x, r.y, r.z);
}

/* OpenGL-style perspective. clip.w = -z_view. */
static Mat4 m4_perspective(float fovy, float aspect, float near, float far)
{
    Mat4 m = {{{0}}};
    float f = 1.f / tanf(fovy * 0.5f);
    m.m[0][0] = f / aspect;
    m.m[1][1] = f;
    m.m[2][2] = (far + near) / (near - far);
    m.m[2][3] = (2.f * far * near) / (near - far);
    m.m[3][2] = -1.f;
    return m;
}

/*
 * m4_orthographic — parallel projection (no perspective divide).
 *
 * Maps view-space cuboid [left,right] × [bottom,top] × [-far,-near]
 * to NDC cube [-1,+1]³. clip.w stays 1, so the perspective divide
 * is identity. Used for directional-light shadow mapping where
 * "depth from the light" must be linear and uniform across the scene.
 */
static Mat4 m4_orthographic(float left, float right,
                            float bottom, float top,
                            float near, float far)
{
    Mat4 m = {{{0}}};
    m.m[0][0] =  2.f / (right - left);
    m.m[1][1] =  2.f / (top   - bottom);
    m.m[2][2] = -2.f / (far   - near);
    m.m[0][3] = -(right + left)   / (right - left);
    m.m[1][3] = -(top   + bottom) / (top   - bottom);
    m.m[2][3] = -(far   + near)   / (far   - near);
    m.m[3][3] =  1.f;
    return m;
}

/* m4_lookat — standard glm convention.
 *   forward = normalize(at - eye)
 *   right   = normalize(cross(forward, up))
 *   up'     = cross(right, forward)
 * Combined with the back-face cull below, this puts world +X on
 * screen right and world +Y at screen top. */
static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up)
{
    Vec3 f = v3_norm(v3_sub(at, eye));
    Vec3 r = v3_norm(v3(f.y*up.z - f.z*up.y,
                        f.z*up.x - f.x*up.z,
                        f.x*up.y - f.y*up.x));
    Vec3 u = v3(r.y*f.z - r.z*f.y,
                r.z*f.x - r.x*f.z,
                r.x*f.y - r.y*f.x);
    Mat4 m = m4_identity();
    m.m[0][0] = r.x; m.m[0][1] = r.y; m.m[0][2] = r.z; m.m[0][3] = -v3_dot(r, eye);
    m.m[1][0] = u.x; m.m[1][1] = u.y; m.m[1][2] = u.z; m.m[1][3] = -v3_dot(u, eye);
    m.m[2][0] = -f.x; m.m[2][1] = -f.y; m.m[2][2] = -f.z; m.m[2][3] = v3_dot(f, eye);
    return m;
}

/* Cofactor of upper-left 3×3 — correct normal transform under
 * non-uniform scale; equals the rotation for pure rotation. */
static Mat4 m4_normal_mat(Mat4 m)
{
    Mat4 n = m4_identity();
    n.m[0][0] = m.m[1][1]*m.m[2][2] - m.m[1][2]*m.m[2][1];
    n.m[0][1] = m.m[1][2]*m.m[2][0] - m.m[1][0]*m.m[2][2];
    n.m[0][2] = m.m[1][0]*m.m[2][1] - m.m[1][1]*m.m[2][0];
    n.m[1][0] = m.m[0][2]*m.m[2][1] - m.m[0][1]*m.m[2][2];
    n.m[1][1] = m.m[0][0]*m.m[2][2] - m.m[0][2]*m.m[2][0];
    n.m[1][2] = m.m[0][1]*m.m[2][0] - m.m[0][0]*m.m[2][1];
    n.m[2][0] = m.m[0][1]*m.m[1][2] - m.m[0][2]*m.m[1][1];
    n.m[2][1] = m.m[0][2]*m.m[1][0] - m.m[0][0]*m.m[1][2];
    n.m[2][2] = m.m[0][0]*m.m[1][1] - m.m[0][1]*m.m[1][0];
    return n;
}

static Mat4 m4_translate(float x, float y, float z)
{
    Mat4 m = m4_identity();
    m.m[0][3] = x; m.m[1][3] = y; m.m[2][3] = z;
    return m;
}

/* ── §4 paint (216 RGB cube + Bourke ramp) ───────────────────────────── */

static int g_256;

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);

    if (g_256) {
        for (int i = 0; i < 216; i++)
            init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_CUBE_BASE, COLOR_WHITE,  -1);
        init_pair(PAIR_HUD,       COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,   -1);
    }
}

static inline float clamp01  (float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float reinhard (float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

static void paint_cell(int sx, int sy, Vec3 col)
{
    float r = gamma_enc(reinhard(col.x));
    float g = gamma_enc(reinhard(col.y));
    float b = gamma_enc(reinhard(col.z));

    float luma  = 0.2126f*r + 0.7152f*g + 0.0722f*b;
    float dith  = (k_bayer[sy & 3][sx & 3] - 0.5f) * DITHER_AMP;
    float lum_d = clamp01(luma + dith);

    int pair;
    if (g_256) {
        int r5 = (int)(r * 5.f + 0.5f); if (r5 > 5) r5 = 5; if (r5 < 0) r5 = 0;
        int g5 = (int)(g * 5.f + 0.5f); if (g5 > 5) g5 = 5; if (g5 < 0) g5 = 0;
        int b5 = (int)(b * 5.f + 0.5f); if (b5 > 5) b5 = 5; if (b5 < 0) b5 = 0;
        pair = PAIR_CUBE_BASE + r5*36 + g5*6 + b5;
    } else {
        pair = PAIR_CUBE_BASE;
    }

    int idx = (int)(lum_d * (BOURKE_LEN - 1) + 0.5f);
    if (idx < 0)            idx = 0;
    if (idx >= BOURKE_LEN)  idx = BOURKE_LEN - 1;

    int attr = (luma > 0.85f) ? A_BOLD
             : (luma < 0.15f) ? A_DIM
             :                  A_NORMAL;

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[idx]);
    attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 mesh ─────────────────────────────────────────────────────────── */

typedef struct { Vec3 pos; Vec3 normal; float u, v; } Vertex;
typedef struct { int v[3]; } Triangle;
typedef struct { Vertex *verts; Triangle *tris; int nvert, ntri; } Mesh;

static void mesh_free(Mesh *m) { free(m->verts); free(m->tris); *m = (Mesh){0}; }

/*
 * mesh_add_quad — append one flat quad (4 verts, 2 CCW tris) to a Mesh.
 * Walks origin → +e1 → +e1+e2 → +e2 in CCW order viewed from outside.
 * Caller must arrange e1 × e2 to have the same sign as nrm.
 */
static void mesh_add_quad(Mesh *m, Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm)
{
    int v0 = m->nvert;
    Vec3 p0 = origin;
    Vec3 p1 = v3_add(origin, e1);
    Vec3 p2 = v3_add(origin, v3_add(e1, e2));
    Vec3 p3 = v3_add(origin, e2);
    m->verts[m->nvert++] = (Vertex){ p0, nrm, 0.f, 0.f };
    m->verts[m->nvert++] = (Vertex){ p1, nrm, 1.f, 0.f };
    m->verts[m->nvert++] = (Vertex){ p2, nrm, 1.f, 1.f };
    m->verts[m->nvert++] = (Vertex){ p3, nrm, 0.f, 1.f };
    m->tris [m->ntri++ ] = (Triangle){{ v0, v0+1, v0+2 }};
    m->tris [m->ntri++ ] = (Triangle){{ v0, v0+2, v0+3 }};
}

/* tessellate_box — axis-aligned cuboid centred at origin, 24 verts
 * (4 per face × 6 faces) so each face has its own flat normal. */
static Mesh tessellate_box(float hx, float hy, float hz)
{
    Mesh m;
    m.verts = malloc(24 * sizeof(Vertex));
    m.tris  = malloc(12 * sizeof(Triangle));
    m.nvert = 0; m.ntri = 0;

    /* +X (right):   e1×e2 = (+, 0, 0) */
    mesh_add_quad(&m, v3( hx,-hy,-hz), v3(0, 2*hy, 0), v3(0, 0, 2*hz),  v3( 1, 0, 0));
    /* -X (left):    e1×e2 = (-, 0, 0) */
    mesh_add_quad(&m, v3(-hx,-hy, hz), v3(0, 2*hy, 0), v3(0, 0,-2*hz),  v3(-1, 0, 0));
    /* +Y (top):     e1×e2 = (0, +, 0) */
    mesh_add_quad(&m, v3(-hx, hy, hz), v3(2*hx, 0, 0), v3(0, 0,-2*hz),  v3( 0, 1, 0));
    /* -Y (bottom):  e1×e2 = (0, -, 0) */
    mesh_add_quad(&m, v3(-hx,-hy,-hz), v3(2*hx, 0, 0), v3(0, 0, 2*hz),  v3( 0,-1, 0));
    /* +Z (front):   e1×e2 = (0, 0, +) */
    mesh_add_quad(&m, v3(-hx,-hy, hz), v3(2*hx, 0, 0), v3(0, 2*hy, 0),  v3( 0, 0, 1));
    /* -Z (back):    e1×e2 = (0, 0, -) */
    mesh_add_quad(&m, v3( hx,-hy,-hz), v3(-2*hx, 0, 0), v3(0, 2*hy, 0), v3( 0, 0,-1));
    return m;
}

static Mesh tessellate_quad(Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm)
{
    Mesh m;
    m.verts = malloc(4 * sizeof(Vertex));
    m.tris  = malloc(2 * sizeof(Triangle));
    m.nvert = 0; m.ntri = 0;
    mesh_add_quad(&m, origin, e1, e2, nrm);
    return m;
}

/* UV sphere with smooth normals (radial outward). */
static Mesh tessellate_sphere(float radius, int rings, int segs)
{
    int n_verts = (rings + 1) * (segs + 1);
    int n_tris  = rings * segs * 2;
    Mesh m;
    m.verts = malloc((size_t)n_verts * sizeof(Vertex));
    m.tris  = malloc((size_t)n_tris  * sizeof(Triangle));
    m.nvert = 0; m.ntri = 0;

    for (int i = 0; i <= rings; i++) {
        float theta = (float)M_PI * (float)i / (float)rings;
        float st = sinf(theta), ct = cosf(theta);
        for (int j = 0; j <= segs; j++) {
            float phi = 2.f * (float)M_PI * (float)j / (float)segs;
            float x = radius * st * cosf(phi);
            float y = radius * ct;
            float z = radius * st * sinf(phi);
            Vec3 pos = v3(x, y, z);
            Vec3 nrm = v3(x/radius, y/radius, z/radius);
            float u  = (float)j / (float)segs;
            float vv = (float)i / (float)rings;
            m.verts[m.nvert++] = (Vertex){ pos, nrm, u, vv };
        }
    }

    for (int i = 0; i < rings; i++) {
        for (int j = 0; j < segs; j++) {
            int v00 =  i      * (segs + 1) + j;
            int v10 = (i + 1) * (segs + 1) + j;
            int v11 = (i + 1) * (segs + 1) + (j + 1);
            int v01 =  i      * (segs + 1) + (j + 1);
            m.tris[m.ntri++] = (Triangle){{ v00, v10, v01 }};
            m.tris[m.ntri++] = (Triangle){{ v10, v11, v01 }};
        }
    }
    return m;
}

/* ── §6 G-buffer — camera-view geometry pass ─────────────────────────── */

static Vec3    g_pos    [GBUF_MAX_H][GBUF_MAX_W];
static Vec3    g_normal [GBUF_MAX_H][GBUF_MAX_W];
static Vec3    g_albedo [GBUF_MAX_H][GBUF_MAX_W];
static float   g_zbuf   [GBUF_MAX_H][GBUF_MAX_W];
static uint8_t g_valid  [GBUF_MAX_H][GBUF_MAX_W];

static void gbuffer_clear(int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            g_zbuf [r][c] = 1.0f;
            g_valid[r][c] = 0;
        }
    }
}

static void barycentric(const float sx[3], const float sy[3],
                        float px, float py, float b[3])
{
    float d = (sy[1] - sy[2]) * (sx[0] - sx[2])
            + (sx[2] - sx[1]) * (sy[0] - sy[2]);
    if (fabsf(d) < 1e-6f) { b[0] = b[1] = b[2] = -1.f; return; }
    b[0] = ((sy[1] - sy[2]) * (px - sx[2]) + (sx[2] - sx[1]) * (py - sy[2])) / d;
    b[1] = ((sy[2] - sy[0]) * (px - sx[2]) + (sx[0] - sx[2]) * (py - sy[2])) / d;
    b[2] = 1.f - b[0] - b[1];
}

/*
 * rasterize_object — vertex transform → perspective divide + cull →
 * barycentric raster + z-test → write full G-buffer (pos, normal,
 * albedo, ndc-z). Three-stage GPU-style pipeline.
 */
static void rasterize_object(const Mesh *mesh, Vec3 albedo,
                             Mat4 mvp, Mat4 model, Mat4 norm_mat,
                             int cols, int rows)
{
    for (int ti = 0; ti < mesh->ntri; ti++) {
        const Triangle *tri = &mesh->tris[ti];

        Vec4 clip[3];
        Vec3 wpos[3], wnrm[3];
        for (int vi = 0; vi < 3; vi++) {
            const Vertex *v = &mesh->verts[tri->v[vi]];
            clip[vi] = m4_mul_v4(mvp,  v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
            wpos[vi] = m4_pt   (model, v->pos);
            wnrm[vi] = v3_norm (m4_dir(norm_mat, v->normal));
        }

        if (clip[0].w < 0.001f && clip[1].w < 0.001f && clip[2].w < 0.001f)
            continue;

        float sx[3], sy[3], sz[3];
        for (int vi = 0; vi < 3; vi++) {
            float w = clip[vi].w; if (fabsf(w) < 1e-6f) w = 1e-6f;
            sx[vi] = ( clip[vi].x / w + 1.f) * 0.5f * (float)cols;
            sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows;
            sz[vi] =   clip[vi].z / w;
        }

        /* Back-face cull. Screen Y-flip turns OpenGL CCW front-faces
         * into NEGATIVE signed area, so we keep negative and reject
         * non-negative. Without this the floor's +Y top — the only
         * side visible from above — would be culled. */
        float area = (sx[1] - sx[0]) * (sy[2] - sy[0])
                   - (sx[2] - sx[0]) * (sy[1] - sy[0]);
        if (area >= 0.f) continue;

        int x0 = (int)fmaxf(0.f,        floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
        int x1 = (int)fminf(cols - 1.f,  ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
        int y0 = (int)fmaxf(0.f,        floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
        int y1 = (int)fminf(rows - 1.f,  ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

        for (int py = y0; py <= y1 && py < GBUF_MAX_H; py++) {
            for (int px = x0; px <= x1 && px < GBUF_MAX_W; px++) {
                float b[3];
                barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
                if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f) continue;

                float z = b[0]*sz[0] + b[1]*sz[1] + b[2]*sz[2];
                if (z >= g_zbuf[py][px]) continue;

                g_zbuf  [py][px] = z;
                g_pos   [py][px] = v3_bary(wpos[0], wpos[1], wpos[2],
                                           b[0], b[1], b[2]);
                g_normal[py][px] = v3_norm(v3_bary(wnrm[0], wnrm[1], wnrm[2],
                                                   b[0], b[1], b[2]));
                g_albedo[py][px] = albedo;
                g_valid [py][px] = 1;
            }
        }
    }
}

static void render_gbuffer(const Mesh *meshes, const Vec3 *albedos,
                           const Mat4 *models, int n_objects,
                           Mat4 view, Mat4 proj, int cols, int rows)
{
    gbuffer_clear(cols, rows);
    for (int oi = 0; oi < n_objects; oi++) {
        Mat4 mv   = m4_mul(view, models[oi]);
        Mat4 mvp  = m4_mul(proj, mv);
        Mat4 nmat = m4_normal_mat(models[oi]);
        rasterize_object(&meshes[oi], albedos[oi], mvp, models[oi], nmat,
                         cols, rows);
    }
}

/* ── §7 shadow — light-view depth-only pass ──────────────────────────── *
 *
 * Stripped-down version of the camera rasteriser: same control flow,
 * but writes only NDC depth (no pos/normal/albedo) into the SHADOW
 * MAP. The light's projection is orthographic so clip.w = 1; we keep
 * the divide anyway so this code reads identically to the main pass. */

static float g_shadow_zbuf[SHADOW_H][SHADOW_W];

static void shadow_clear(void)
{
    for (int r = 0; r < SHADOW_H; r++)
        for (int c = 0; c < SHADOW_W; c++)
            g_shadow_zbuf[r][c] = 1.0f;     /* NDC far */
}

static void rasterize_object_shadow(const Mesh *mesh, Mat4 light_mvp)
{
    for (int ti = 0; ti < mesh->ntri; ti++) {
        const Triangle *tri = &mesh->tris[ti];

        Vec4 clip[3];
        for (int vi = 0; vi < 3; vi++) {
            const Vertex *v = &mesh->verts[tri->v[vi]];
            clip[vi] = m4_mul_v4(light_mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
        }

        float sx[3], sy[3], sz[3];
        for (int vi = 0; vi < 3; vi++) {
            float w = clip[vi].w; if (fabsf(w) < 1e-6f) w = 1e-6f;
            sx[vi] = ( clip[vi].x / w + 1.f) * 0.5f * (float)SHADOW_W;
            sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)SHADOW_H;
            sz[vi] =   clip[vi].z / w;
        }

        float area = (sx[1] - sx[0]) * (sy[2] - sy[0])
                   - (sx[2] - sx[0]) * (sy[1] - sy[0]);
        if (area >= 0.f) continue;

        int x0 = (int)fmaxf(0.f,             floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
        int x1 = (int)fminf(SHADOW_W - 1.f,   ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
        int y0 = (int)fmaxf(0.f,             floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
        int y1 = (int)fminf(SHADOW_H - 1.f,   ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

        for (int py = y0; py <= y1; py++) {
            for (int px = x0; px <= x1; px++) {
                float b[3];
                barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
                if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f) continue;

                float z = b[0]*sz[0] + b[1]*sz[1] + b[2]*sz[2];
                if (z < g_shadow_zbuf[py][px])
                    g_shadow_zbuf[py][px] = z;
            }
        }
    }
}

static void shadow_pass(const Mesh *meshes, const Mat4 *models, int n_objects,
                        Mat4 light_view, Mat4 light_proj)
{
    shadow_clear();
    for (int oi = 0; oi < n_objects; oi++) {
        Mat4 mv  = m4_mul(light_view, models[oi]);
        Mat4 mvp = m4_mul(light_proj, mv);
        rasterize_object_shadow(&meshes[oi], mvp);
    }
}

/* True iff the shadow map at (ix, iy) records something closer to
 * the light than fragment depth z_frag (i.e. the fragment is in shadow). */
static inline bool shadow_occluded(int ix, int iy, float z_frag)
{
    return g_shadow_zbuf[iy][ix] + SHADOW_BIAS < z_frag;
}

/*
 * shadow_sample — project a world-space fragment into the light's
 * clip space, then look up the shadow map. Returns:
 *   0.0  fully lit       (no occluder, or outside the light frustum)
 *   1.0  fully shadowed
 *   in between (PCF only) along a shadow edge
 */
static float shadow_sample(Vec3 world_pos, Mat4 light_view, Mat4 light_proj,
                           bool soft_pcf)
{
    /* World → light clip → light NDC. */
    Vec4 clip = m4_mul_v4(m4_mul(light_proj, light_view),
                          v4(world_pos.x, world_pos.y, world_pos.z, 1.f));
    if (clip.w < 1e-6f) return 0.f;
    Vec3 ndc = v3(clip.x / clip.w, clip.y / clip.w, clip.z / clip.w);

    /* Outside the light's clip cube → fragment isn't visible to the
     * light at all. Treat as fully lit (treating as shadowed would
     * paint a black halo everywhere the ortho frustum doesn't cover). */
    if (ndc.x < -1.f || ndc.x > 1.f
     || ndc.y < -1.f || ndc.y > 1.f
     || ndc.z < -1.f || ndc.z > 1.f) return 0.f;

    /* NDC → shadow map cell (Y-flip matches the rasteriser). */
    int ix = (int)(( ndc.x + 1.f) * 0.5f * (float)SHADOW_W);
    int iy = (int)((-ndc.y + 1.f) * 0.5f * (float)SHADOW_H);
    if (ix < 0 || ix >= SHADOW_W || iy < 0 || iy >= SHADOW_H) return 0.f;

    float z_frag = ndc.z;

    if (!soft_pcf)
        return shadow_occluded(ix, iy, z_frag) ? 1.f : 0.f;

    /* PCF 3×3: average the binary test over a 9-cell neighbourhood. */
    float sum   = 0.f;
    int   count = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int rr = iy + dy, cc = ix + dx;
            if (rr < 0 || rr >= SHADOW_H || cc < 0 || cc >= SHADOW_W) continue;
            sum += shadow_occluded(cc, rr, z_frag) ? 1.f : 0.f;
            count++;
        }
    }
    return (count > 0) ? (sum / (float)count) : 0.f;
}

/* ── §8 lightpass — Blinn-Phong + shadow ─────────────────────────────── */

static Vec3 g_light[GBUF_MAX_H][GBUF_MAX_W];

static void render_lightpass(Vec3 cam_pos, Mat4 light_view, Mat4 light_proj,
                             bool shadows_on, bool soft_pcf,
                             int cols, int rows)
{
    Vec3 sun_dir = v3(SUN_DIR[0],     SUN_DIR[1],     SUN_DIR[2]);
    Vec3 sun_col = v3(SUN_COL[0],     SUN_COL[1],     SUN_COL[2]);
    Vec3 ambient = v3(AMBIENT_COL[0], AMBIENT_COL[1], AMBIENT_COL[2]);
    Vec3 L       = v3_norm(v3_neg(sun_dir));

    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) { g_light[r][c] = v3(0,0,0); continue; }

            Vec3 P      = g_pos   [r][c];
            Vec3 N      = g_normal[r][c];
            Vec3 albedo = g_albedo[r][c];

            /* ambient — unaffected by shadow */
            Vec3 amb = v3(ambient.x * albedo.x,
                          ambient.y * albedo.y,
                          ambient.z * albedo.z);

            /* diffuse + specular — these get multiplied by (1 − shadow) */
            float diff = fmaxf(0.f, v3_dot(N, L));
            Vec3  dif  = v3(albedo.x * sun_col.x * diff,
                            albedo.y * sun_col.y * diff,
                            albedo.z * sun_col.z * diff);

            Vec3  V    = v3_norm(v3_sub(cam_pos, P));
            Vec3  H    = v3_norm(v3_add(L, V));
            float spec = powf(fmaxf(0.f, v3_dot(N, H)), SHININESS) * SPEC_GAIN;
            Vec3  sp   = v3(sun_col.x * spec,
                            sun_col.y * spec,
                            sun_col.z * spec);

            float shadow = shadows_on
                         ? shadow_sample(P, light_view, light_proj, soft_pcf)
                         : 0.f;
            float lit    = 1.f - shadow;

            Vec3 sum = v3(amb.x + lit * (dif.x + sp.x),
                          amb.y + lit * (dif.y + sp.y),
                          amb.z + lit * (dif.z + sp.z));
            g_light[r][c] = v3(fminf(1.f, sum.x),
                               fminf(1.f, sum.y),
                               fminf(1.f, sum.z));
        }
    }
}

/* ── §9 scene ────────────────────────────────────────────────────────── */

typedef struct {
    Mesh        meshes [N_OBJECTS];
    Vec3        albedos[N_OBJECTS];
    Mat4        models [N_OBJECTS];

    /* camera (orbits the scene) */
    Mat4        view, proj;
    Vec3        cam_pos;
    float       cam_dist;
    float       cam_yaw;

    /* light (fixed) */
    Mat4        light_view, light_proj;

    bool        shadows_on;
    bool        soft_pcf;
    bool        paused;

    int         scene_cols;
    int         scene_rows;
} Scene;

static void scene_rebuild_proj(Scene *s, int cols, int rows)
{
    float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
    s->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void scene_rebuild_view(Scene *s)
{
    float r = s->cam_dist;
    s->cam_pos = v3(sinf(s->cam_yaw) * r,
                    CAM_EYE_Y,
                    cosf(s->cam_yaw) * r);
    s->view = m4_lookat(s->cam_pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
}

/*
 * scene_build_light — light view + ortho projection. Built once.
 *   eye  = scene_centre − sun_dir · LIGHT_DISTANCE
 *   look = origin
 *   proj = ortho(±ORTHO_HALF, ±ORTHO_HALF, NEAR, FAR)
 */
static void scene_build_light(Scene *s)
{
    Vec3 sun_dir   = v3_norm(v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2]));
    Vec3 light_eye = v3_scale(sun_dir, -LIGHT_DISTANCE);
    s->light_view  = m4_lookat(light_eye, v3(0, 0, 0), v3(0, 1, 0));
    s->light_proj  = m4_orthographic(-LIGHT_ORTHO_HALF,  LIGHT_ORTHO_HALF,
                                     -LIGHT_ORTHO_HALF,  LIGHT_ORTHO_HALF,
                                      LIGHT_NEAR,        LIGHT_FAR);
}

/* scene_init — floor + pillar + cube + sphere + camera + light. */
static void scene_init(Scene *s, int total_cols, int total_rows)
{
    for (int i = 0; i < N_OBJECTS; i++) mesh_free(&s->meshes[i]);

    memset(s, 0, sizeof *s);
    s->scene_cols  = total_cols;
    s->scene_rows  = total_rows - HUD_ROWS;
    s->shadows_on  = true;
    s->soft_pcf    = false;
    s->cam_dist    = CAM_DIST;
    s->cam_yaw     = 0.f;

    s->meshes [OBJ_FLOOR]  = tessellate_quad(
        v3(-FLOOR_HALF_X, 0.f,  FLOOR_HALF_Z),
        v3( 2*FLOOR_HALF_X, 0.f,  0.f),
        v3( 0.f,            0.f, -2*FLOOR_HALF_Z),
        v3( 0.f, 1.f, 0.f));
    s->albedos[OBJ_FLOOR]  = v3(0.42f, 0.46f, 0.50f);    /* slate     */
    s->models [OBJ_FLOOR]  = m4_identity();

    s->meshes [OBJ_PILLAR] = tessellate_box(PILLAR_HX, PILLAR_HY, PILLAR_HZ);
    s->albedos[OBJ_PILLAR] = v3(0.78f, 0.62f, 0.42f);    /* sandstone */
    s->models [OBJ_PILLAR] = m4_translate(PILLAR_CX, PILLAR_CY, PILLAR_CZ);

    s->meshes [OBJ_CUBE]   = tessellate_box(CUBE_HALF, CUBE_HALF, CUBE_HALF);
    s->albedos[OBJ_CUBE]   = v3(0.82f, 0.48f, 0.32f);    /* terracotta */
    s->models [OBJ_CUBE]   = m4_translate(CUBE_CX, CUBE_CY, CUBE_CZ);

    s->meshes [OBJ_SPHERE] = tessellate_sphere(SPHERE_R, SPHERE_RINGS, SPHERE_SEGS);
    s->albedos[OBJ_SPHERE] = v3(0.90f, 0.84f, 0.74f);    /* cream     */
    s->models [OBJ_SPHERE] = m4_translate(SPHERE_CX, SPHERE_CY, SPHERE_CZ);

    scene_rebuild_proj(s, total_cols, s->scene_rows);
    scene_rebuild_view(s);
    scene_build_light(s);
}

/* Only the camera orbits; the light and geometry are static. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->cam_yaw += CAM_ORBIT_RAD_PER_SEC * dt;
    scene_rebuild_view(s);
}

/* ── §10 screen — render_scene + HUD ─────────────────────────────────── */

static void render_scene(const Scene *s)
{
    int cols = s->scene_cols;
    int rows = s->scene_rows;

    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) continue;
            paint_cell(c, r, g_light[r][c]);
        }
    }
}

static void hud_draw(const Scene *s, double fps)
{
    int hr   = s->scene_rows;
    int cols = s->scene_cols;

    int total_tris = 0;
    for (int i = 0; i < N_OBJECTS; i++) total_tris += s->meshes[i].ntri;

    /* Row 0: title + status. */
    char status[140];
    snprintf(status, sizeof status,
             " %5.1f fps  shadow:%s  filter:%s  zoom:%.1f  tris:%d  %s ",
             fps,
             s->shadows_on ? "ON " : "OFF",
             s->soft_pcf   ? "PCF" : "HARD",
             (double)s->cam_dist, total_tris,
             s->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > cols) slen = cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - slen, "%s", status);
    mvprintw(0, 0, " SHADOW MAPPING ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(hr + 0, 1,
             "shadow map: %dx%d float NDC z   bias=%.4f   "
             "ortho frustum: %.1f x %.1f x %.1f",
             SHADOW_W, SHADOW_H, (double)SHADOW_BIAS,
             2.0*(double)LIGHT_ORTHO_HALF, 2.0*(double)LIGHT_ORTHO_HALF,
             (double)(LIGHT_FAR - LIGHT_NEAR));

    const char *explain = "";
    if (!s->shadows_on)
        explain = "Shadows OFF: every lit face gets full direct light. The cubes 'float'.";
    else if (!s->soft_pcf)
        explain = "Shadows ON, HARD: each fragment compares its depth to the shadow map.";
    else
        explain = "Shadows ON, PCF: 3x3 average around the lookup softens shadow edges.";
    mvprintw(hr + 1, 1, "%s", explain);

    mvprintw(hr + 2, 1,
             "Pass 1 light-view depth -> shadow map. "
             "Pass 2 camera G-buffer. Pass 3 lookup + light.");

    attroff(COLOR_PAIR(PAIR_HUD));

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(hr + HUD_ROWS - 1, 0,
             " q:quit  spc:pause  s:shadows  f:filter  +/-:zoom  r:reset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §11 app ─────────────────────────────────────────────────────────── */

typedef struct {
    Scene                 scene;
    int                   total_cols;
    int                   total_rows;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup         (void)    { endwin(); }

static void screen_init(void)
{
    initscr();
    noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
}

static void app_do_resize(App *app)
{
    endwin(); refresh();
    getmaxyx(stdscr, app->total_rows, app->total_cols);
    scene_init(&app->scene, app->total_cols, app->total_rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused; break;
    case 'r': case 'R': scene_init(s, app->total_cols, app->total_rows); break;
    case 's': case 'S': s->shadows_on = !s->shadows_on; break;
    case 'f': case 'F': s->soft_pcf   = !s->soft_pcf;   break;
    case '=': case '+':
        s->cam_dist -= CAM_ZOOM_STEP;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        scene_rebuild_view(s);
        break;
    case '-': case '_':
        s->cam_dist += CAM_ZOOM_STEP;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        scene_rebuild_view(s);
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app = &g_app;
    app->running = 1;

    screen_init();
    getmaxyx(stdscr, app->total_rows, app->total_cols);
    scene_init(&app->scene, app->total_cols, app->total_rows);

    int64_t frame_time  = clock_ns();
    int64_t fps_acc     = 0;
    int     fps_cnt     = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;
        float dt_sec = (float)dt / (float)NS_PER_SEC;

        scene_tick(&app->scene, dt_sec);

        fps_cnt++;
        fps_acc += dt;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
            fps_cnt = 0; fps_acc = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);

        Scene *s = &app->scene;
        erase();

        /* Pass 1: depth-from-light. */
        if (s->shadows_on)
            shadow_pass(s->meshes, s->models, N_OBJECTS, s->light_view, s->light_proj);

        /* Pass 2: camera G-buffer. */
        render_gbuffer(s->meshes, s->albedos, s->models, N_OBJECTS,
                       s->view, s->proj, s->scene_cols, s->scene_rows);

        /* Pass 3: lightpass with optional shadow lookup. */
        render_lightpass(s->cam_pos, s->light_view, s->light_proj,
                         s->shadows_on, s->soft_pcf,
                         s->scene_cols, s->scene_rows);

        render_scene(s);
        hud_draw(s, fps_display);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    for (int i = 0; i < N_OBJECTS; i++) mesh_free(&app->scene.meshes[i]);

    endwin();
    return 0;
}
