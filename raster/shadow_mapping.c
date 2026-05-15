/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shadow_mapping.c — software directional-light shadow mapping
 *
 * DEMO: A square slate floor with one warm sandstone cube and one
 *       cool grey sphere. A directional sun lights from the upper
 *       right. Each object casts a hard shadow (PCF-softened) onto
 *       the floor; where their light-projections overlap the shadows
 *       blend on each other. Camera orbits slowly. Press 's' to
 *       toggle the shadows — without them every lit face gets full
 *       sun even where another object is in the way.
 *
 * Section map:
 *   §1  config   — frame, view, sun, shadow map, scene, ramp
 *   §2  clock    — monotonic timer + sleep
 *   §3  math     — V3, V4, Mat4, perspective / ortho / lookAt
 *   §4  paint    — 216-pair RGB cube + Bourke ramp + paint_cell
 *   §5  mesh     — Vertex/Triangle/Mesh + quad / box builders
 *   §6  gbuffer  — camera-view geometry pass (pos, normal, albedo, z)
 *   §7  shadow   — light-view depth-only pass (the shadow map)
 *   §8  lightpass — Blinn-Phong + shadow lookup
 *   §9  scene    — Scene struct, init, tick
 *   §10 screen   — render_scene + HUD
 *   §11 app      — signals, resize, main loop
 *
 * Keys:
 *   s / S     toggle shadow on/off
 *   space     pause / resume camera orbit
 *   + / =     zoom in
 *   - / _     zoom out
 *   r / R     reset camera
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/shadow_mapping.c \
 *       -o shadow -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────── *
 *
 * Algorithm     : Shadow mapping (Williams 1978). Two passes per frame:
 *                 (1) render the scene FROM THE LIGHT's POV, writing only
 *                 NDC depth into a 2-D buffer (the shadow map).
 *                 (2) render normally from the camera; for each fragment
 *                 project its world position into the light's clip space,
 *                 look up the shadow map, and compare. If the stored
 *                 depth is smaller than the fragment's, something closer
 *                 to the light was already there → fragment is in shadow.
 *
 * Data          : One 256×256 float buffer = the shadow map.
 *                 Standard G-buffer (pos / normal / albedo / zbuf / valid)
 *                 sized for the terminal.
 *                 Three meshes: floor quad + cube + UV-sphere.
 *
 * Rendering     : Continuous RGB → Reinhard tone-map → 6×6×6 cube + 92-
 *                 char Bourke ramp + Bayer 4×4 dither. Same paint
 *                 pipeline as every other raster file.
 *
 * Performance   : Trivial — a few dozen triangles, sub-millisecond per
 *                 pass.
 *
 * References    : Williams, "Casting Curved Shadows on Curved Surfaces,"
 *                   SIGGRAPH '78.
 *                 Möller, "Fast Triangle Rasterization by Interpolating
 *                   Edge Functions," GPG (2000).
 *                 Reinhard et al., "Photographic Tone Reproduction for
 *                   Digital Images," SIGGRAPH '02.
 *                 LearnOpenGL, "Shadow Mapping" tutorial.
 *
 * ──────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * "Is point P in shadow of light L?" = "is some surface closer to L
 * than P along the ray P → L?". A depth buffer rendered FROM L records
 * exactly that — the closest surface in every direction the light
 * sees. Render the scene from L once, then every shading fragment
 * just LOOKS UP the answer instead of tracing.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 *
 *      sun ──── ▶
 *                 ┌──┐                    shadow_map[u,v] = depth
 *                 │██│                     of the closest surface
 *           ──────┴──┴─────────  floor     the light sees
 *
 *      For floor fragment F under the cube:
 *        project F into light space → (u, v, z_F)
 *        if shadow_map[u,v] + bias < z_F:
 *          something closer than F is between F and the sun
 *          → F is in shadow
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────────────────
 *  1. Build  light_view = lookAt(light_eye, origin, +Y)
 *           light_proj = orthographic(±OH, ±OH, near, far)
 *     where light_eye = -SUN_DIR · LIGHT_DISTANCE.
 *  2. SHADOW PASS — for each object, rasterise into the shadow map
 *     using light_proj · light_view · model. Write only NDC depth.
 *  3. G-BUFFER PASS — for each object, rasterise into the main
 *     G-buffer using camera_proj · camera_view · model. Write
 *     world pos, world normal, albedo, NDC depth.
 *  4. LIGHT PASS — per visible pixel:
 *        ambient = AMBIENT · albedo                 always present
 *        project P into light NDC; sample shadow map
 *        shadow = (shadow_map[u,v] + BIAS < light_ndc.z) ? 1 : 0
 *        diffuse  = albedo · sun_col · max(0, N·L)
 *        specular = sun_col · max(0, N·H)^SHININESS · SPEC_GAIN
 *        out      = ambient + (1 − shadow) · (diffuse + specular)
 *
 * KEY FORMULAS
 * ────────────
 *  Light eye:    light_eye = -SUN_DIR · LIGHT_DISTANCE
 *  Light MVP:    L = light_proj · light_view · model
 *  Light NDC:    ndc = (L · v).xyz / (L · v).w
 *  Shadow UV:    u = ( ndc.x + 1)/2 · SHADOW_W
 *                v = (-ndc.y + 1)/2 · SHADOW_H        (Y-flip)
 *  Test:         shadow_map[v][u] + SHADOW_BIAS < ndc.z   → in shadow
 *  Composite:    out = ambient + (1 − shadow) · (diffuse + specular)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Outside the light's frustum → shadow = 0 (lit). Otherwise a
 *    huge halo would form anywhere the ortho frustum doesn't cover.
 *  • Shadow acne — bias too small → flat lit surfaces self-shadow
 *    in a striping pattern. Bias too large → shadows DETACH from
 *    casters (peter-panning). 0.002 NDC works for this scene.
 *  • PCF (3×3 average) softens both shadow EDGES (stair-step → 3-cell
 *    gradient) and per-texel popping during camera movement. The
 *    interior darkness is unchanged — only edges feather.
 *  • Back-face culling is DISABLED on both passes for this scene.
 *    On a closed convex mesh the z-buffer / shadow-map depth-test
 *    already keep the right surface; turning cull off lets reverse-
 *    wound triangles still appear instead of vanishing silently.
 *  • Bayer dither amp 0.04 (not the default 0.10) keeps the floor's
 *    flat luma from forming visible 4-cell wallpaper bands.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Toggle 's': cube + sphere shadows on the floor appear/disappear.
 *  • As the camera orbits, both shadows stay ATTACHED at their casters'
 *    bases (no peter-panning) and have softly feathered edges (PCF).
 *  • The cube's right face (away from sun) and the sphere's lower-
 *    right hemisphere drop to ambient only.
 *  • Where the cube's and sphere's light-projections overlap at certain
 *    sun angles, you should see the same shadow region — no extra
 *    darkening (a single shadow can't go below "fully shadowed" = 1).
 *
 * ──────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ─────────────────────────────────────────── *
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
 *      pass does the per-fragment shadow lookup with PCF.
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
 *   - Cascaded shadow maps (sun across an entire game world) — we
 *     have one ortho frustum, scene fits in it.
 *   - Shadow volumes (a different algorithm using stencil buffers).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ───────────────────────────────────────────────── *
 *
 * Eight short tutorials that build shadow mapping from first
 * principles. Read in order; each builds on the previous.
 *
 *   T1  The shadow-mapping principle — render twice, compare depth
 *   T2  Orthographic projection — why a directional sun uses ortho
 *   T3  Pass 1: depth-only rasterisation from the light's POV
 *   T4  Pass 2: per-fragment shadow lookup
 *   T5  Shadow acne and the bias workaround (Peter-Panning trade)
 *   T6  PCF — 3×3 average for soft shadow edges
 *   T7  Outside-the-frustum and other defensible defaults
 *   T8  Why this scene disables back-face culling
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
 *   2. Render the scene from the camera's POV. For each fragment F:
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
 * The camera uses PERSPECTIVE projection — a frustum that narrows
 * toward the eye. Rays from a perspective camera DIVERGE outward.
 *
 * The SUN is treated as DIRECTIONAL — infinitely far away, all rays
 * parallel, no convergence point. The right projection for a
 * directional light is ORTHOGRAPHIC:
 *
 *   PERSPECTIVE   frustum, narrows at eye, divides by w
 *   ORTHOGRAPHIC  rectangular box (no narrowing); after the divide,
 *                 x/w = x because w = 1 throughout
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
 * contain every shadow-casting object in the scene. The box's
 * cross-section maps to the shadow map's (u, v) grid; the box's
 * length along the light direction is mapped to z and stored.
 *
 * Read §1 LIGHT_ORTHO_HALF + §3 m4_orthographic for the constants
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
 *       (sx, sy, sz) = NDC mapping (with Y-flip)
 *       for each cell in triangle's bbox:
 *         compute barycentric (b0, b1, b2)
 *         if any < 0: outside, skip
 *         z = b0·sz0 + b1·sz1 + b2·sz2          (interpolated NDC z)
 *         if z < shadow_map[v][u]:              (closer than current)
 *           shadow_map[v][u] = z
 *
 * No fragment shader, no surface attributes, no lighting. The
 * output is a 2-D buffer of depth values from the light's POV.
 *
 * For our 256² shadow map and a few hundred triangles this pass is
 * sub-millisecond.
 *
 * Read §7 shadow_pass + rasterize_object_shadow.
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
 *     shadow = PCF_3x3 average over neighbourhood (T6)
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
 * Without bias, a flat surface like the floor casts a shadow ON
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
 * This shifts the comparison so flat surfaces reliably read NOT
 * shadowed. SHADOW_BIAS = 0.002 is enough for our scene.
 *
 * Trade-off — PETER-PANNING:
 *   too much bias → shadows DETACH from their casters. The cube
 *   appears to FLOAT above its own shadow. Looks wrong. We saw
 *   this at SHADOW_BIAS = 0.008 during phase-2 iteration; 0.002
 *   is the smallest value that hides acne in this scene.
 *
 * T6  PCF — 3×3 AVERAGE FOR SOFT SHADOW EDGES
 * ────────────────────────────────────────────
 * Hard shadow mapping returns 0 or 1 — every fragment is either
 * fully lit or fully shadowed. The shadow EDGE is therefore a
 * stair-step at the shadow-map's resolution.
 *
 * Worse: when the CAMERA moves (orbit, zoom) but the SUN stays
 * fixed, fragments project to slightly different shadow-map cells
 * each frame. The boundary "pops" between texels — a wobble that
 * makes the shadow look detached from its caster even when the
 * geometry is right.
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
 *     shadow = hits / 9.0                       ∈ [0, 1] in 1/9 steps
 *
 * Now the edge feathers from 0/9 (lit) through 1/9, 2/9, ..., 8/9
 * to 9/9 (fully shadowed). The transition is 3 shadow-map texels
 * wide — soft enough to read as a real penumbra at terminal scale.
 *
 * INTERIOR shadow darkness is unchanged (every cell of the 3×3
 * block agrees → 9/9). Only the EDGE softens. PCF also smooths
 * the camera-motion popping into a gentler shimmer.
 *
 * Cost: 9 shadow-map reads per fragment instead of 1. Trivial at
 * terminal resolution.
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
 * T8  WHY THIS SCENE DISABLES BACK-FACE CULLING
 * ──────────────────────────────────────────────
 * The classic forward rasteriser (cube_raster.c) culls back-facing
 * triangles via signed area test in screen space. After Y-flip, the
 * convention is "keep negative-area triangles, reject non-negative."
 *
 * This file DISABLES that cull on both passes. Why?
 *
 * For a CLOSED CONVEX MESH (cube, sphere) and a properly-wound
 * triangle list, back-face culling is purely an optimisation —
 * back-facing triangles are always occluded by their front-facing
 * counterparts via the z-buffer / shadow-map depth test. Removing
 * the cull does not change the visible image.
 *
 * What removing the cull DOES is make the renderer ROBUST to
 * inverted winding. If a face's triangles are wound the "wrong"
 * way (e.g. by an artist authoring error or a buggy mesh
 * generator), the cull would silently skip them and produce a
 * missing face. With cull off, the depth test still keeps the
 * correct surface — and any winding errors become visually obvious
 * via incorrect lighting (normal pointing into the surface
 * instead of away) rather than as silent gaps.
 *
 * For a teaching / experimentation file this is worth the slight
 * extra rasterisation cost. The shadow pass also has cull
 * disabled for the same reason — if the camera pass and the
 * shadow pass disagreed on which faces to keep, the shadow could
 * be cast from a different surface than the visible one.
 *
 * Production renderers normally KEEP cull on for performance.
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
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

/* ── §1 config ─────────────────────────────────────────────────────── */

enum {
  FPS_TARGET = 60,
  HUD_ROWS = 2,
};

/* §1.1 view + camera. */
#define CAM_FOV (50.0f * 3.14159265f / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
#define CAM_DIST_DEF 5.0f
#define CAM_DIST_MIN 2.0f
#define CAM_DIST_MAX 9.0f
#define CAM_ZOOM_STEP 0.25f
#define CAM_HEIGHT 1.6f
#define CAM_LOOK_Y 0.5f
#define CAM_ORBIT_RAD_PER_SEC 0.30f
#define CELL_W 8
#define CELL_H 16

/* §1.2 sun (directional) + ambient. */
static const float SUN_DIR[3] = {-0.55f, -0.55f, 0.30f}; /* light's
                                                            travel direction */
static const float SUN_COL[3] = {0.98f, 0.88f, 0.68f};
static const float AMBIENT_COL[3] = {0.18f, 0.20f, 0.26f};
#define SHININESS 24.0f
#define SPEC_GAIN 0.30f

/* §1.3 shadow map.
 *
 * The light is treated as orthographic (parallel rays). LIGHT_EYE
 * sits at -SUN_DIR · LIGHT_DISTANCE looking back at the origin.
 * The orthographic frustum is a box of half-extent OH in light's
 * right & up axes, with depth from NEAR to FAR along light's forward.
 *
 * SHADOW_BIAS in NDC z. With FAR-NEAR ≈ 10, NDC step 0.002 ≈ 0.01
 * world units — enough to defeat acne while keeping shadows attached
 * to the cube's base (no peter-panning). */
#define LIGHT_DISTANCE 6.0f
#define LIGHT_ORTHO_HALF 3.0f
#define LIGHT_NEAR 0.5f
#define LIGHT_FAR 11.0f
#define SHADOW_W 256
#define SHADOW_H 256
#define SHADOW_BIAS 0.002f

/* §1.4 scene geometry — floor + cube + sphere.
 *
 *                  sun ─── ▶ ▼
 *                            │
 *                ┌─┐                ●     ← sphere
 *                │█│  cube
 *                └─┘
 *           ─────────────────────────  floor
 *
 * Both objects sit on the floor (y = 0); their shadows fall away
 * from the sun onto the floor and onto each other where their
 * light-projections overlap. */
#define FLOOR_HALF_X 2.5f
#define FLOOR_HALF_Z 2.5f

#define CUBE_HALF 0.55f
#define CUBE_CX -0.75f
#define CUBE_CY (CUBE_HALF)
#define CUBE_CZ -0.20f

#define SPHERE_R 0.55f
#define SPHERE_RINGS 12
#define SPHERE_SEGS 18
#define SPHERE_CX 0.85f
#define SPHERE_CY (SPHERE_R)
#define SPHERE_CZ 0.30f

enum { OBJ_FLOOR = 0, OBJ_CUBE, OBJ_SPHERE, N_OBJECTS };

/* §1.5 G-buffer dims. Static; sized for the largest terminal we expect. */
#define GBUF_MAX_W 400
#define GBUF_MAX_H 200

/* §1.6 ASCII glyph ramp (Paul Bourke 92-char). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.7 Bayer 4×4 dither.
 *
 * DITHER_AMP scales the per-cell luma perturbation. With the 92-char
 * Bourke ramp each step is ~0.011 luma; AMP=0.10 produces a ±5-step
 * swing per cell that becomes visible on large FLAT regions (the
 * floor) as a 4×4 wallpaper pattern. AMP=0.04 keeps the dither
 * subtle on flat areas while still breaking up banding on smooth
 * gradients (cube silhouette + lit-to-shadow transition). */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.04f

/* §1.8 ncurses pair IDs. */
#define PAIR_CUBE_BASE 1 /* + 0..215 = 6×6×6 cube     */
#define PAIR_HUD 217
#define PAIR_HINT 218

/* ── §2 clock ──────────────────────────────────────────────────────── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {.tv_sec = (time_t)(ns / 1000000000LL),
                         .tv_nsec = (long)(ns % 1000000000LL)};
  nanosleep(&req, NULL);
}

/* ── §3 math (V3, V4, Mat4) ────────────────────────────────────────── */

typedef struct {
  float x, y, z;
} Vec3;
typedef struct {
  float x, y, z, w;
} Vec4;
typedef struct {
  float m[4][4];
} Mat4;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec4 v4(float x, float y, float z, float w) {
  return (Vec4){x, y, z, w};
}
static inline Vec3 v3_add(Vec3 a, Vec3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline Vec3 v3_sub(Vec3 a, Vec3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline Vec3 v3_scale(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}
static inline float v3_dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline Vec3 v3_neg(Vec3 a) { return v3(-a.x, -a.y, -a.z); }
static inline Vec3 v3_cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline Vec3 v3_norm(Vec3 a) {
  float l = sqrtf(v3_dot(a, a));
  return (l > 1e-9f) ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}
static inline Vec3 v3_bary(Vec3 a, Vec3 b, Vec3 c, float u, float v, float w) {
  return v3(a.x * u + b.x * v + c.x * w, a.y * u + b.y * v + c.y * w,
            a.z * u + b.z * v + c.z * w);
}

static Mat4 m4_identity(void) {
  Mat4 m = {0};
  for (int i = 0; i < 4; i++)
    m.m[i][i] = 1.f;
  return m;
}

static Mat4 m4_mul(Mat4 a, Mat4 b) {
  Mat4 r = {0};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        r.m[i][j] += a.m[i][k] * b.m[k][j];
  return r;
}

static Vec4 m4_mul_v4(Mat4 m, Vec4 v) {
  return v4(
      m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z + m.m[0][3] * v.w,
      m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z + m.m[1][3] * v.w,
      m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z + m.m[2][3] * v.w,
      m.m[3][0] * v.x + m.m[3][1] * v.y + m.m[3][2] * v.z + m.m[3][3] * v.w);
}

static Vec3 m4_pt(Mat4 m, Vec3 p) {
  Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
  return v3(r.x, r.y, r.z);
}

/* Transform a direction (e.g. surface normal) by the upper 3×3 of m.
 * Correct for orthonormal rotations + uniform scale; for non-uniform
 * scale you'd want the inverse-transpose. We use rotation only. */
static Vec3 m4_dir(Mat4 m, Vec3 d) {
  return v3(m.m[0][0] * d.x + m.m[0][1] * d.y + m.m[0][2] * d.z,
            m.m[1][0] * d.x + m.m[1][1] * d.y + m.m[1][2] * d.z,
            m.m[2][0] * d.x + m.m[2][1] * d.y + m.m[2][2] * d.z);
}

static Mat4 m4_translate(float x, float y, float z) {
  Mat4 m = m4_identity();
  m.m[0][3] = x;
  m.m[1][3] = y;
  m.m[2][3] = z;
  return m;
}

static Mat4 m4_perspective(float fovy, float aspect, float near, float far) {
  float f = 1.f / tanf(fovy * 0.5f);
  Mat4 m = {0};
  m.m[0][0] = f / aspect;
  m.m[1][1] = f;
  m.m[2][2] = (far + near) / (near - far);
  m.m[2][3] = (2.f * far * near) / (near - far);
  m.m[3][2] = -1.f;
  return m;
}

static Mat4 m4_orthographic(float l, float r, float b, float t, float n,
                            float f) {
  Mat4 m = m4_identity();
  m.m[0][0] = 2.f / (r - l);
  m.m[1][1] = 2.f / (t - b);
  m.m[2][2] = -2.f / (f - n);
  m.m[0][3] = -(r + l) / (r - l);
  m.m[1][3] = -(t + b) / (t - b);
  m.m[2][3] = -(f + n) / (f - n);
  return m;
}

static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up) {
  Vec3 f = v3_norm(v3_sub(at, eye));
  Vec3 s = v3_norm(v3_cross(f, up));
  Vec3 u = v3_cross(s, f);
  Mat4 m = m4_identity();
  m.m[0][0] = s.x;
  m.m[0][1] = s.y;
  m.m[0][2] = s.z;
  m.m[1][0] = u.x;
  m.m[1][1] = u.y;
  m.m[1][2] = u.z;
  m.m[2][0] = -f.x;
  m.m[2][1] = -f.y;
  m.m[2][2] = -f.z;
  m.m[0][3] = -v3_dot(s, eye);
  m.m[1][3] = -v3_dot(u, eye);
  m.m[2][3] = v3_dot(f, eye);
  return m;
}

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* ── §4 paint (216 RGB cube + Bourke ramp) ─────────────────────────── */

static int g_256;

static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  if (g_256) {
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_CUBE_BASE, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static void paint_cell(int sx, int sy, Vec3 col) {
  float r = gamma_enc(reinhard(col.x));
  float g = gamma_enc(reinhard(col.y));
  float b = gamma_enc(reinhard(col.z));

  float luma = 0.2126f * r + 0.7152f * g + 0.0722f * b;
  float dith = (k_bayer[sy & 3][sx & 3] - 0.5f) * DITHER_AMP;
  float lum_d = clamp01(luma + dith);

  int pair;
  if (g_256) {
    int r5 = (int)(r * 5.f + 0.5f);
    if (r5 > 5)
      r5 = 5;
    if (r5 < 0)
      r5 = 0;
    int g5 = (int)(g * 5.f + 0.5f);
    if (g5 > 5)
      g5 = 5;
    if (g5 < 0)
      g5 = 0;
    int b5 = (int)(b * 5.f + 0.5f);
    if (b5 > 5)
      b5 = 5;
    if (b5 < 0)
      b5 = 0;
    pair = PAIR_CUBE_BASE + r5 * 36 + g5 * 6 + b5;
  } else {
    pair = PAIR_CUBE_BASE;
  }

  int idx = (int)(lum_d * (BOURKE_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= BOURKE_LEN)
    idx = BOURKE_LEN - 1;

  int attr = (luma > 0.85f) ? A_BOLD : A_NORMAL;
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[idx]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 mesh ───────────────────────────────────────────────────────── */

typedef struct {
  Vec3 pos;
  Vec3 normal;
} Vertex;
typedef struct {
  int v[3];
} Triangle;
typedef struct {
  Vertex *verts;
  Triangle *tris;
  int nvert, ntri;
} Mesh;

static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

static void mesh_add_quad(Mesh *m, Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm) {
  int v0 = m->nvert;
  m->verts[v0 + 0] = (Vertex){origin, nrm};
  m->verts[v0 + 1] = (Vertex){v3_add(origin, e1), nrm};
  m->verts[v0 + 2] = (Vertex){v3_add(v3_add(origin, e1), e2), nrm};
  m->verts[v0 + 3] = (Vertex){v3_add(origin, e2), nrm};
  m->nvert += 4;

  m->tris[m->ntri++] = (Triangle){{v0, v0 + 1, v0 + 2}};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 2, v0 + 3}};
}

static Mesh mesh_floor(float hx, float hz) {
  Mesh m = {0};
  m.verts = malloc(4 * sizeof *m.verts);
  m.tris = malloc(2 * sizeof *m.tris);
  mesh_add_quad(&m, v3(-hx, 0, hz), /* origin (back-left) */
                v3(2 * hx, 0, 0),   /* +x edge            */
                v3(0, 0, -2 * hz),  /* -z edge            */
                v3(0, 1, 0));
  return m;
}

static Mesh mesh_box(float hx, float hy, float hz) {
  Mesh m = {0};
  m.verts = malloc(24 * sizeof *m.verts);
  m.tris = malloc(12 * sizeof *m.tris);
  /* +X face */ mesh_add_quad(&m, v3(hx, -hy, -hz), v3(0, 0, 2 * hz),
                              v3(0, 2 * hy, 0), v3(1, 0, 0));
  /* -X face */ mesh_add_quad(&m, v3(-hx, -hy, hz), v3(0, 0, -2 * hz),
                              v3(0, 2 * hy, 0), v3(-1, 0, 0));
  /* +Y face */ mesh_add_quad(&m, v3(-hx, hy, -hz), v3(2 * hx, 0, 0),
                              v3(0, 0, 2 * hz), v3(0, 1, 0));
  /* -Y face */ mesh_add_quad(&m, v3(-hx, -hy, hz), v3(2 * hx, 0, 0),
                              v3(0, 0, -2 * hz), v3(0, -1, 0));
  /* +Z face */ mesh_add_quad(&m, v3(-hx, -hy, hz), v3(2 * hx, 0, 0),
                              v3(0, 2 * hy, 0), v3(0, 0, 1));
  /* -Z face */ mesh_add_quad(&m, v3(hx, -hy, -hz), v3(-2 * hx, 0, 0),
                              v3(0, 2 * hy, 0), v3(0, 0, -1));
  return m;
}

/* UV-sphere tessellation: rings × segs grid + 2 triangles per cell.
 * Smooth per-vertex normals = position / radius (= position for unit
 * sphere). At the poles sin(phi) = 0 collapses many vertices to one
 * point; we override the normal to (0, ±1, 0) explicitly to avoid a
 * divide-by-near-zero. */
static Mesh mesh_sphere(float radius, int rings, int segs) {
  int nv = (rings + 1) * (segs + 1);
  int nt = rings * segs * 2;
  Mesh m = {0};
  m.verts = malloc(nv * sizeof *m.verts);
  m.tris = malloc(nt * sizeof *m.tris);

  for (int r = 0; r <= rings; r++) {
    float phi = (float)M_PI * (float)r / (float)rings; /* 0..π */
    float sp = sinf(phi), cp = cosf(phi);
    for (int s = 0; s <= segs; s++) {
      float th = 2.f * (float)M_PI * (float)s / (float)segs;
      float st = sinf(th), ct = cosf(th);
      Vec3 p = v3(radius * sp * ct, radius * cp, radius * sp * st);
      Vec3 n = (sp < 1e-6f) ? v3(0, (cp > 0 ? 1.f : -1.f), 0) : v3_norm(p);
      m.verts[m.nvert++] = (Vertex){p, n};
    }
  }
  for (int r = 0; r < rings; r++) {
    for (int s = 0; s < segs; s++) {
      int a = r * (segs + 1) + s;
      int b = r * (segs + 1) + (s + 1);
      int c = (r + 1) * (segs + 1) + s;
      int d = (r + 1) * (segs + 1) + (s + 1);
      m.tris[m.ntri++] = (Triangle){{a, b, d}};
      m.tris[m.ntri++] = (Triangle){{a, d, c}};
    }
  }
  return m;
}

/* ── Barycentric helper (Möller signed-area form) ──────────────────── */

static inline void barycentric(float sx[3], float sy[3], float px, float py,
                               float b[3]) {
  float d =
      (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2]);
  if (fabsf(d) < 1e-9f) {
    b[0] = b[1] = b[2] = -1.f;
    return;
  }
  b[0] = ((sy[1] - sy[2]) * (px - sx[2]) + (sx[2] - sx[1]) * (py - sy[2])) / d;
  b[1] = ((sy[2] - sy[0]) * (px - sx[2]) + (sx[0] - sx[2]) * (py - sy[2])) / d;
  b[2] = 1.f - b[0] - b[1];
}

/* ── §6 G-buffer (camera-view geometry pass) ───────────────────────── */

static Vec3 g_pos[GBUF_MAX_H][GBUF_MAX_W];
static Vec3 g_normal[GBUF_MAX_H][GBUF_MAX_W];
static Vec3 g_albedo[GBUF_MAX_H][GBUF_MAX_W];
static float g_zbuf[GBUF_MAX_H][GBUF_MAX_W];
static int g_valid[GBUF_MAX_H][GBUF_MAX_W];

static void gbuffer_clear(int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      g_zbuf[r][c] = 1.f;
      g_valid[r][c] = 0;
    }
  }
}

static void rasterize_object(const Mesh *mesh, Vec3 albedo, Mat4 mvp,
                             Mat4 model, int cols, int rows) {
  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];
    Vec4 clip[3];
    Vec3 wpos[3], wnrm[3];
    for (int vi = 0; vi < 3; vi++) {
      const Vertex *v = &mesh->verts[tri->v[vi]];
      clip[vi] = m4_mul_v4(mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
      wpos[vi] = m4_pt(model, v->pos);
      wnrm[vi] = v3_norm(m4_dir(model, v->normal));
    }
    if (clip[0].w < 0.001f && clip[1].w < 0.001f && clip[2].w < 0.001f)
      continue;

    float sx[3], sy[3], sz[3];
    for (int vi = 0; vi < 3; vi++) {
      float w = clip[vi].w;
      if (fabsf(w) < 1e-6f)
        w = 1e-6f;
      sx[vi] = (clip[vi].x / w + 1.f) * 0.5f * (float)cols;
      sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows;
      sz[vi] = clip[vi].z / w;
    }

    /* Back-face cull DISABLED on the camera pass — every triangle
     * (including the cube's far faces) gets rasterised. The
     * z-buffer still hides occluded fragments, so a closed cube
     * looks identical from outside; if a triangle is wound the
     * "wrong" way, leaving culling off lets it still appear.
     * The shadow pass (§7) keeps culling because shadow casters
     * write only their light-facing surfaces. */
    int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

    for (int py = y0; py <= y1 && py < GBUF_MAX_H; py++) {
      for (int px = x0; px <= x1 && px < GBUF_MAX_W; px++) {
        float b[3];
        barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
        if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
          continue;

        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        if (z >= g_zbuf[py][px])
          continue;

        g_zbuf[py][px] = z;
        g_pos[py][px] = v3_bary(wpos[0], wpos[1], wpos[2], b[0], b[1], b[2]);
        g_normal[py][px] =
            v3_norm(v3_bary(wnrm[0], wnrm[1], wnrm[2], b[0], b[1], b[2]));
        g_albedo[py][px] = albedo;
        g_valid[py][px] = 1;
      }
    }
  }
}

static void render_gbuffer(const Mesh *meshes, const Vec3 *albedos,
                           const Mat4 *models, int n_objects, Mat4 view,
                           Mat4 proj, int cols, int rows) {
  gbuffer_clear(cols, rows);
  for (int oi = 0; oi < n_objects; oi++) {
    Mat4 mvp = m4_mul(m4_mul(proj, view), models[oi]);
    rasterize_object(&meshes[oi], albedos[oi], mvp, models[oi], cols, rows);
  }
}

/* ── §7 shadow (light-view depth-only pass) ────────────────────────── */

static float g_shadow_zbuf[SHADOW_H][SHADOW_W];

static void shadow_clear(void) {
  for (int r = 0; r < SHADOW_H; r++)
    for (int c = 0; c < SHADOW_W; c++)
      g_shadow_zbuf[r][c] = 1.f;
}

static void rasterize_object_shadow(const Mesh *mesh, Mat4 light_mvp) {
  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];
    Vec4 clip[3];
    for (int vi = 0; vi < 3; vi++) {
      const Vertex *v = &mesh->verts[tri->v[vi]];
      clip[vi] = m4_mul_v4(light_mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
    }
    float sx[3], sy[3], sz[3];
    for (int vi = 0; vi < 3; vi++) {
      float w = clip[vi].w;
      if (fabsf(w) < 1e-6f)
        w = 1e-6f;
      sx[vi] = (clip[vi].x / w + 1.f) * 0.5f * (float)SHADOW_W;
      sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)SHADOW_H;
      sz[vi] = clip[vi].z / w;
    }
    /* Back-face cull DISABLED on the shadow pass too (matching the
     * camera pass). All cube triangles, front and back from the
     * light's POV, write their depth to the shadow map. The
     * `z < g_shadow_zbuf[py][px]` test still keeps the smallest
     * depth per texel, so for a closed convex cube the shadow is
     * unchanged — only difference is uncullled triangles also
     * cast shadow (matters for non-closed or reverse-wound meshes). */
    int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 =
        (int)fminf(SHADOW_W - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 =
        (int)fminf(SHADOW_H - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

    for (int py = y0; py <= y1; py++) {
      for (int px = x0; px <= x1; px++) {
        float b[3];
        barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
        if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
          continue;
        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        if (z < g_shadow_zbuf[py][px])
          g_shadow_zbuf[py][px] = z;
      }
    }
  }
}

static void shadow_pass(const Mesh *meshes, const Mat4 *models, int n_objects,
                        Mat4 light_view, Mat4 light_proj) {
  shadow_clear();
  for (int oi = 0; oi < n_objects; oi++) {
    Mat4 mvp = m4_mul(m4_mul(light_proj, light_view), models[oi]);
    rasterize_object_shadow(&meshes[oi], mvp);
  }
}

/* shadow_sample — project a world point into light NDC, then look up
 * the shadow map with 3×3 PCF averaging.
 *
 * Why PCF: a single binary depth test gives stair-stepped shadow
 * edges (one shadow-map texel per fragment = pixel resolution at
 * the texel level). Averaging the binary test over a 3×3 cell
 * neighbourhood softens the edge to a 3-cell gradient — looks like
 * a real penumbra and hides per-texel popping when the camera
 * moves between frames. Cost: 9 shadow-map reads per fragment.
 */
static float shadow_sample(Vec3 world_pos, Mat4 light_view, Mat4 light_proj) {
  Vec4 clip = m4_mul_v4(m4_mul(light_proj, light_view),
                        v4(world_pos.x, world_pos.y, world_pos.z, 1.f));
  if (clip.w < 1e-6f)
    return 0.f;
  Vec3 ndc = v3(clip.x / clip.w, clip.y / clip.w, clip.z / clip.w);

  /* Outside the light's frustum → not visible to the light → not in shadow. */
  if (ndc.x < -1.f || ndc.x > 1.f || ndc.y < -1.f || ndc.y > 1.f ||
      ndc.z < -1.f || ndc.z > 1.f)
    return 0.f;

  int ix = (int)((ndc.x + 1.f) * 0.5f * (float)SHADOW_W);
  int iy = (int)((-ndc.y + 1.f) * 0.5f * (float)SHADOW_H);
  if (ix < 0 || ix >= SHADOW_W || iy < 0 || iy >= SHADOW_H)
    return 0.f;

  /* 3×3 PCF: average the binary occluder test over a 9-cell
   * neighbourhood around the lookup centre. Result ∈ {0, 1/9,
   * 2/9, ..., 9/9}: a soft transition instead of a stair-step. */
  float z_frag = ndc.z;
  int hits = 0;
  int count = 0;
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      int rr = iy + dy, cc = ix + dx;
      if (rr < 0 || rr >= SHADOW_H || cc < 0 || cc >= SHADOW_W)
        continue;
      if (g_shadow_zbuf[rr][cc] + SHADOW_BIAS < z_frag)
        hits++;
      count++;
    }
  }
  return (count > 0) ? ((float)hits / (float)count) : 0.f;
}

/* ── §8 lightpass (Blinn-Phong + shadow lookup) ────────────────────── */

static Vec3 g_light[GBUF_MAX_H][GBUF_MAX_W];

static void render_lightpass(Vec3 cam_pos, Mat4 light_view, Mat4 light_proj,
                             bool shadows_on, int cols, int rows) {
  Vec3 sun_dir = v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2]);
  Vec3 sun_col = v3(SUN_COL[0], SUN_COL[1], SUN_COL[2]);
  Vec3 ambient = v3(AMBIENT_COL[0], AMBIENT_COL[1], AMBIENT_COL[2]);
  Vec3 L = v3_norm(v3_neg(sun_dir));

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g_valid[r][c]) {
        g_light[r][c] = v3(0, 0, 0);
        continue;
      }

      Vec3 P = g_pos[r][c];
      Vec3 N = g_normal[r][c];
      Vec3 albedo = g_albedo[r][c];

      Vec3 amb =
          v3(ambient.x * albedo.x, ambient.y * albedo.y, ambient.z * albedo.z);

      float diff = fmaxf(0.f, v3_dot(N, L));
      Vec3 dif = v3(albedo.x * sun_col.x * diff, albedo.y * sun_col.y * diff,
                    albedo.z * sun_col.z * diff);

      Vec3 V = v3_norm(v3_sub(cam_pos, P));
      Vec3 H = v3_norm(v3_add(L, V));
      float spec = powf(fmaxf(0.f, v3_dot(N, H)), SHININESS) * SPEC_GAIN;
      Vec3 sp = v3(sun_col.x * spec, sun_col.y * spec, sun_col.z * spec);

      float shadow =
          shadows_on ? shadow_sample(P, light_view, light_proj) : 0.f;
      float lit = 1.f - shadow;

      g_light[r][c] =
          v3(amb.x + lit * (dif.x + sp.x), amb.y + lit * (dif.y + sp.y),
             amb.z + lit * (dif.z + sp.z));
    }
  }
}

/* ── §9 scene ──────────────────────────────────────────────────────── */

typedef struct {
  Mesh meshes[N_OBJECTS];
  Vec3 albedos[N_OBJECTS];
  Mat4 models[N_OBJECTS];

  Mat4 view, proj;
  Vec3 cam_pos;
  float cam_dist;
  float cam_yaw;

  Mat4 light_view, light_proj;

  bool shadows_on;
  bool paused;
  int scene_cols;
  int scene_rows;
} Scene;

static void scene_rebuild_proj(Scene *s, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void scene_rebuild_view(Scene *s) {
  float r = s->cam_dist;
  s->cam_pos = v3(sinf(s->cam_yaw) * r, CAM_HEIGHT, cosf(s->cam_yaw) * r);
  s->view = m4_lookat(s->cam_pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
}

static void scene_build_light(Scene *s) {
  Vec3 sun_dir = v3_norm(v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2]));
  Vec3 light_eye = v3_scale(sun_dir, -LIGHT_DISTANCE);
  s->light_view = m4_lookat(light_eye, v3(0, 0, 0), v3(0, 1, 0));
  s->light_proj =
      m4_orthographic(-LIGHT_ORTHO_HALF, LIGHT_ORTHO_HALF, -LIGHT_ORTHO_HALF,
                      LIGHT_ORTHO_HALF, LIGHT_NEAR, LIGHT_FAR);
}

static void scene_init(Scene *s, int total_cols, int total_rows) {
  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&s->meshes[i]);

  memset(s, 0, sizeof *s);
  s->scene_cols = total_cols;
  s->scene_rows = total_rows - HUD_ROWS;
  s->shadows_on = true;
  s->cam_dist = CAM_DIST_DEF;
  s->cam_yaw = 0.f;

  s->meshes[OBJ_FLOOR] = mesh_floor(FLOOR_HALF_X, FLOOR_HALF_Z);
  s->albedos[OBJ_FLOOR] = v3(0.32f, 0.36f, 0.42f); /* dark slate */
  s->models[OBJ_FLOOR] = m4_identity();

  s->meshes[OBJ_CUBE] = mesh_box(CUBE_HALF, CUBE_HALF, CUBE_HALF);
  s->albedos[OBJ_CUBE] = v3(0.78f, 0.62f, 0.42f); /* sandstone */
  s->models[OBJ_CUBE] = m4_translate(CUBE_CX, CUBE_CY, CUBE_CZ);

  s->meshes[OBJ_SPHERE] = mesh_sphere(SPHERE_R, SPHERE_RINGS, SPHERE_SEGS);
  s->albedos[OBJ_SPHERE] = v3(0.78f, 0.78f, 0.82f); /* cool grey */
  s->models[OBJ_SPHERE] = m4_translate(SPHERE_CX, SPHERE_CY, SPHERE_CZ);

  scene_rebuild_proj(s, total_cols, s->scene_rows);
  scene_rebuild_view(s);
  scene_build_light(s);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->cam_yaw += CAM_ORBIT_RAD_PER_SEC * dt;
  scene_rebuild_view(s);
}

/* ── §10 screen ────────────────────────────────────────────────────── */

static void render_scene(const Scene *s) {
  int cols = s->scene_cols;
  int rows = s->scene_rows;
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++)
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++)
      if (g_valid[r][c])
        paint_cell(c, r, g_light[r][c]);
}

static void hud_draw(const Scene *s, double fps) {
  int hr = s->scene_rows;
  int cols = s->scene_cols;

  char status[120];
  snprintf(status, sizeof status, " %5.1f fps  shadow=%s  zoom=%.1f ", fps,
           s->shadows_on ? "on " : "off", (double)s->cam_dist);
  int slen = (int)strlen(status);
  if (slen > cols)
    slen = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - slen, "%s", status);
  mvprintw(0, 0, " SHADOW MAPPING ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(hr + 0, 1,
           "shadow map: %dx%d float NDC z   bias=%.4f   "
           "ortho frustum: %.1f x %.1f x %.1f",
           SHADOW_W, SHADOW_H, (double)SHADOW_BIAS,
           2.0 * (double)LIGHT_ORTHO_HALF, 2.0 * (double)LIGHT_ORTHO_HALF,
           (double)(LIGHT_FAR - LIGHT_NEAR));
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hr + 1, 0, " q:quit  s:shadow  +/-:zoom  spc:pause  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §11 app ───────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint(int s) {
  (void)s;
  g_run = 0;
}
static void on_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}
static void cleanup(void) { endwin(); }

int main(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);
  signal(SIGWINCH, on_sigwinch);
  atexit(cleanup);

  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1);
  color_init();

  int cols, rows;
  getmaxyx(stdscr, rows, cols);

  Scene s;
  memset(&s, 0, sizeof s);
  scene_init(&s, cols, rows);

  int64_t prev = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps = 0.0;
  int64_t frame_ns = 1000000000LL / FPS_TARGET;

  while (g_run) {
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      scene_init(&s, cols, rows);
    }

    int64_t now = clock_ns();
    int64_t dt = now - prev;
    if (dt > 100000000LL)
      dt = 100000000LL;
    prev = now;

    fps_acc += dt;
    fps_cnt++;
    if (fps_acc >= 500000000LL) {
      fps = (double)fps_cnt * 1e9 / (double)fps_acc;
      fps_acc = 0;
      fps_cnt = 0;
    }

    scene_tick(&s, (float)dt * 1e-9f);

    if (s.shadows_on)
      shadow_pass(s.meshes, s.models, N_OBJECTS, s.light_view, s.light_proj);
    render_gbuffer(s.meshes, s.albedos, s.models, N_OBJECTS, s.view, s.proj,
                   s.scene_cols, s.scene_rows);
    render_lightpass(s.cam_pos, s.light_view, s.light_proj, s.shadows_on,
                     s.scene_cols, s.scene_rows);

    erase();
    render_scene(&s);
    hud_draw(&s, fps);
    wnoutrefresh(stdscr);
    doupdate();

    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 'Q':
      case 27:
        g_run = 0;
        break;
      case 's':
      case 'S':
        s.shadows_on = !s.shadows_on;
        break;
      case ' ':
        s.paused = !s.paused;
        break;
      case 'r':
      case 'R':
        s.cam_dist = CAM_DIST_DEF;
        s.cam_yaw = 0.f;
        scene_rebuild_view(&s);
        break;
      case '+':
      case '=':
        s.cam_dist -= CAM_ZOOM_STEP;
        if (s.cam_dist < CAM_DIST_MIN)
          s.cam_dist = CAM_DIST_MIN;
        scene_rebuild_view(&s);
        break;
      case '-':
      case '_':
        s.cam_dist += CAM_ZOOM_STEP;
        if (s.cam_dist > CAM_DIST_MAX)
          s.cam_dist = CAM_DIST_MAX;
        scene_rebuild_view(&s);
        break;
      }
    }

    int64_t target = clock_ns();
    int64_t left = frame_ns - (target - now);
    if (left > 0)
      clock_sleep_ns(left);
  }

  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&s.meshes[i]);
  return 0;
}
