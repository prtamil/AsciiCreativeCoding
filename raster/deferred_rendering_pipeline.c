/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * deferred_rendering_pipeline.c — software deferred rendering
 *
 * DEMO: A WHITE SPHERE lit by RED, GREEN, and BLUE point lights
 *       orbiting at 120° apart on a horizontal ring. You watch each
 *       pure-RGB pool slide across the sphere; where two pools overlap
 *       the secondary colour appears (yellow / cyan / magenta); where
 *       all three meet you see white. That's additive light made
 *       visible — and it's exactly what deferred lighting computes,
 *       one light pass at a time.
 *
 *       Each light is one DEFERRED LIGHTING PASS. The sphere's
 *       geometry buffers (g_pos, g_normal, g_albedo) are computed
 *       ONCE per frame in the geometry pass; then each light reads
 *       them and accumulates its contribution into g_light. Three
 *       lights → three accumulations → final colour = sum.
 *
 *       Press 'l' to enable extra lights (up to 8). The geometry
 *       pass cost stays fixed; only g_light's accumulation grows.
 *       That's the deferred-shading guarantee — geometry once,
 *       lighting per-light, never per-(object × light).
 *
 *       Cycle 'g' through 4 G-buffer layers (POSITION / NORMAL /
 *       ALBEDO / LIGHTING). Only LIGHTING re-shades when you add
 *       lights — POSITION / NORMAL / ALBEDO are light-agnostic.
 *
 *       This is a software port of the technique used by Unity HDRP,
 *       Unreal Engine 5 (default deferred path), and OpenGL/Vulkan
 *       Multiple Render Targets — but written in C for ASCII output.
 *
 * Study alongside:
 *   raster/cube_raster.c          — single-mesh forward rasterisation
 *   raster/sphere_raster.c        — same skeleton, smooth normals
 *   raster/torus_raster.c         — same skeleton, parametric tessellation
 *   raster/displace_raster.c      — same skeleton + vertex displacement
 *   raytracing/path_tracer.c      — same RGB cube + Bourke ramp paint
 *
 * Section map:
 *   §1 config     — frame rate, FOV, geometry, ramp, ncurses pairs
 *   §2 clock      — monotonic timer + sleep
 *   §3 math       — V3, V4, Mat4 + perspective / lookat / normal mat
 *   §4 paint      — 216-pair RGB cube + Bourke ramp + paint_cell
 *                   (same as cube_raster / raytracers)
 *   §5 mesh       — Vertex / Triangle types + UV-sphere tessellator
 *   §6 gbuffer    — geometry pass: rasterise meshes into per-pixel arrays
 *                   §6.1 buffers (g_pos, g_normal, g_albedo, g_zbuf, ...)
 *                   §6.2 barycentric (Möller signed-area form)
 *                   §6.3 rasterize_object (vert → cull → raster → write)
 *                   §6.4 render_gbuffer (loop over scene objects)
 *   §7 lightpass  — Blinn-Phong shading per pixel × per light
 *                   §7.1 PointLight type
 *                   §7.2 blinn_phong (one light's contribution)
 *                   §7.3 render_lightpass (accumulate over all lights)
 *   §8 scene      — Scene state, init/tick, mode_to_rgb
 *                   §8.1 GBufMode + Scene type   (no Object struct;
 *                                                 parallel arrays only)
 *                   §8.2 LIGHT_PRESETS table
 *                   §8.3 scene_init / scene_tick
 *                   §8.4 mode_to_rgb (G-buffer layer → RGB)
 *   §9 screen     — render_scene (paint each pixel) + HUD (spec compliant)
 *   §10 app       — signals, resize, fixed-step main loop
 *
 * Keys:
 *   g / G     cycle G-buffer layer (POSITION → NORMAL → ALBEDO → LIGHTING)
 *   l / L     add a point light (wraps to 1 after MAX_LIGHTS)
 *   + / =     zoom in   (decrease camera distance)
 *   - / _     zoom out  (increase camera distance)
 *   space     pause / resume animation
 *   r / R     reset scene
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/deferred_rendering_pipeline.c \
 *       -o deferred -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two-pass deferred rendering. Pass 1 (geometry/G-buffer):
 *                  rasterise every mesh into per-pixel arrays of position,
 *                  normal, and albedo — NO lighting at all. Pass 2 (lighting):
 *                  loop over pixels and run Blinn-Phong against every light,
 *                  using the buffered surface data. Cost goes from
 *                  O(objects × lights) (forward rendering) to
 *                  O(objects) + O(pixels × lights) (deferred).
 *
 *                  The second pass NEVER touches mesh data — geometry has
 *                  been "flattened" into per-pixel surface properties.
 *                  That's why adding lights doesn't change POSITION,
 *                  NORMAL, or ALBEDO views: those layers are produced
 *                  ONCE in Pass 1, before any light is considered.
 *
 *                  Output paint pipeline matches the rest of the folder:
 *                  per-pixel V3 RGB → Reinhard tone-map → gamma → 6×6×6
 *                  cube quantise + 92-char Bourke density ramp.
 *
 * Data-structure : Six parallel grids of size [GBUF_MAX_ROWS][GBUF_MAX_COLS]:
 *                  g_pos, g_normal, g_albedo, g_zbuf, g_valid, g_light.
 *                  Same scheme as Unity HDRP and Unreal's GBufferA/B/C
 *                  layout — different packing details, same idea.
 *                  Statically allocated; no malloc in the hot render path.
 *
 * Rendering      : Pass 1 (render_gbuffer) walks each object's triangles
 *                  through vertex transform → perspective divide → back-face
 *                  cull → barycentric raster → G-buffer write. Pass 2
 *                  (render_lightpass) walks the screen and accumulates
 *                  Blinn-Phong from every active light into g_light.
 *                  render_scene picks ONE G-buffer layer per pixel
 *                  (POSITION / NORMAL / ALBEDO / LIGHTING), produces a
 *                  V3 RGB, and routes through the unified paint_cell.
 *
 * Performance    : Geometry pass touches each surface pixel ONCE.
 *                  Adding lights only grows the lighting pass —
 *                  geometry pass is invariant. The HUD shows the
 *                  forward (obj × lights) vs deferred (obj + lights)
 *                  draw-call comparison so the difference is visible.
 *
 * References     : Saito & Takahashi, "Comprehensible Rendering of 3-D
 *                    Shapes," SIGGRAPH '90 (G-buffer concept).
 *                  LearnOpenGL, "Deferred Shading" tutorial:
 *                    https://learnopengl.com/Advanced-Lighting/Deferred-Shading
 *                  Unreal Engine 5 GBuffer documentation.
 *                  Unity HDRP "Forward and Deferred Rendering" docs.
 *                  Möller, "Fast Triangle Rasterization by Interpolating
 *                    Edge Functions," Game Programming Gems (2000).
 *                  Reinhard et al., "Photographic Tone Reproduction for
 *                    Digital Images," SIGGRAPH '02.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Stop trying to shade a pixel "while you draw it". Instead, draw every
 * surface FIRST and dump its position / normal / colour into per-pixel
 * arrays. THEN, only after the geometry phase is over, walk the screen
 * pixel-by-pixel and run lighting against those arrays. Lights can no
 * longer see triangles — only the flattened surface data — so adding
 * a light costs another pass over PIXELS, never over geometry.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine taking a Polaroid photograph of the scene where every pixel
 * of the photo carries three pieces of metadata in invisible ink:
 *   - WHERE the surface was in 3-D (g_pos)
 *   - WHICH WAY it faced              (g_normal)
 *   - WHAT COLOUR the paint was       (g_albedo)
 *
 * A second person can later look at the Polaroid, read the metadata,
 * and compute lighting — never seeing the scene itself. The 'g' key
 * swaps the visible "ink" so you can see each metadata layer; pressing
 * 'l' adds another lightbulb, which only the second person notices.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │       FORWARD                  DEFERRED      │
 *      ├──────────────────────────────────────────────┤
 *      │   for each obj:                              │
 *      │     for each light:        for each obj:     │
 *      │       shade(obj, lite)       write_gbuffer   │
 *      │                                              │
 *      │                            for each pixel:   │
 *      │                              for each lite:  │
 *      │                                shade_gbuf    │
 *      ├──────────────────────────────────────────────┤
 *      │ O(objects × lights)   O(objects + pix×lits)  │
 *      └──────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────────────────
 *  1. gbuffer_clear: g_zbuf = +1.0 (NDC far), g_valid = 0 everywhere.
 *  2. PASS 1 — render_gbuffer:
 *       For each object:
 *         build mvp = proj · view · model; norm_mat = cofactor(model).
 *         For each triangle:
 *           - Vertex stage: clip[i] = mvp·pos; world_pos = model·pos;
 *             world_nrm  = normalize(norm_mat · normal).
 *           - Perspective divide: ndc = clip.xyz / clip.w.
 *           - Screen-space + back-face cull (signed area > 0 = front).
 *           - For each pixel in the screen-space bounding box:
 *               barycentric weights → in/out test
 *               z-test (< g_zbuf)
 *               write g_pos, g_normal, g_albedo, g_zbuf, g_valid.
 *  3. PASS 2 — render_lightpass:
 *       For each pixel where g_valid = 1:
 *         result = ambient · albedo
 *         For each active light:
 *           result += blinn_phong(P, N, albedo, light, cam)
 *         clamp to [0,1]; store in g_light.
 *  4. PASS 3 — render_scene:
 *       For each pixel: produce a V3 RGB based on the active mode
 *       (POSITION / NORMAL / ALBEDO / LIGHTING) and route through
 *       the unified paint_cell (Reinhard → gamma → 6×6×6 cube + ramp).
 *
 * KEY FORMULAS
 * ────────────
 *  mvp = P · V · M                          clip-space transform
 *  norm = cofactor(M_3×3) · n                correct under non-uniform scale
 *  sx, sy = ((±ndc + 1)/2) · dim             NDC → pixel
 *  area = (x1-x0)(y2-y0) - (x2-x0)(y1-y0)    back-face cull (CCW: area>0)
 *  bary: b·v0 + b·v1 + b·v2, Σb = 1          per-pixel attribute interp
 *  Blinn-Phong:
 *    L  = normalize(light_pos - P)
 *    V  = normalize(cam_pos   - P)
 *    H  = normalize(L + V)
 *    diff = max(0, N · L)
 *    spec = max(0, N · H)^shininess
 *    col  = ambient·albedo + albedo·light_col·diff + light_col·spec·0.35
 *
 *  forward cost ∝ obj·lights;  deferred ∝ obj + pix·lights
 *
 *  Mode → RGB mapping (mode_to_rgb):
 *    POSITION:  warm-near → cool-far gradient based on z
 *    NORMAL:    (N + 1) / 2  (each component remapped to [0,1])
 *    ALBEDO:    g_albedo[r][c]    raw flat colour
 *    LIGHTING:  g_light[r][c]     final shaded result
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Triangles with all w < 0.001 are entirely behind the near plane.
 *    Skip BEFORE perspective divide or you divide by ~0.
 *  • Back-face cull uses screen-space signed area, NOT view-space dot.
 *    CCW triangles produce positive area after Y-flip; cull negative.
 *  • g_zbuf MUST be reset to +1.0 each frame. The test is `z < g_zbuf`,
 *    so forgetting reset locks the buffer at last frame's depths.
 *  • g_valid gates ALL reads in lighting + display. Without it,
 *    lighting accumulates onto bare clear pixels.
 *  • Adding a light updates ONLY g_light. POSITION / NORMAL / ALBEDO
 *    must be pixel-perfect identical — that's the whole point.
 *  • Static G-buffer arrays are GBUF_MAX_W × GBUF_MAX_H. Resizing past
 *    those silently clips the render area.
 *  • Tone-map at paint time. The lighting pass clamps to [0,1] after
 *    summing all lights (poor man's tone-map); paint_cell additionally
 *    Reinhard-tone-maps. Without that, sums of bright lights produce
 *    one cube cell after quantising.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Switch to ALBEDO ('g'): cube is solid orange, sphere solid blue,
 *    floor grey. Pressing 'l' must NOT change this view.
 *  • In NORMAL view, cube faces show 6 distinct constant colours
 *    (flat normals); sphere shows a smooth gradient (smooth normals).
 *  • In LIGHTING view, adding a 2nd light should immediately re-shade
 *    every visible pixel without requiring scene rotation.
 *  • The HUD's "fwd vs def" counters should diverge as you add lights:
 *    fwd grows by N_objects per light, def by 1 per light.
 *  • POSITION view: warm tones near the camera, cooler tones at the
 *    back of the scene (depth gradient).
 *  • +/- zoom: sphere grows / shrinks, but the three coloured pools
 *    stay in the same triangular arrangement on its surface. The HUD's
 *    "zoom" readout matches s->cam_dist, clamped to [MIN, MAX].
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read raster/cube_raster.c first if you don't yet
 *      know forward rendering — deferred is a REORGANISATION of the
 *      same pipeline. The point is best appreciated by contrast.
 *   2. §1 config — every constant has a unit-bearing comment.
 *   3. §6 gbuffer + §7 lightpass — the TWO HEART sections. Read
 *      AFTER tutorials T1-T4. §6 is the geometry pass (writes per-
 *      pixel attributes); §7 is the lighting pass (reads them).
 *   4. §8 scene + LIGHT_PRESETS — three orbiting RGB lights.
 *   5. §3 math, §4 paint, §9-§10 — same as cube_raster; skim.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   g_pos[r][c]            world-space surface position per pixel
 *   g_normal[r][c]          world-space surface normal per pixel
 *   g_albedo[r][c]          surface base colour per pixel
 *   g_zbuf[r][c]            NDC depth per pixel
 *   g_valid[r][c]           1 if pixel has surface data, 0 = sky
 *   g_light[r][c]           accumulated lit colour from PASS 2
 *   GBufMode                which g_* buffer is being shown
 *
 * Background you need
 * ───────────────────
 *   - cube_raster.c's 7-stage forward pipeline.
 *   - The pain of "shade objects per light" in forward — N objects
 *     × M lights = N·M shading invocations.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - GPU memory bandwidth tradeoffs (relevant for real GPUs;
 *     this is software, no MRT cost).
 *   - Tile-based deferred / clustered shading (advanced variants).
 *   - Anti-aliasing in the deferred path (TAA, MSAA-with-deferred);
 *     we just write last-wins per pixel.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Six short tutorials. Read in order; each builds on the previous.
 *
 *   T1  Forward vs deferred — the cost equation that motivates everything
 *   T2  The G-buffer concept — flatten geometry into per-pixel arrays
 *   T3  Pass 1: writing to the G-buffer
 *   T4  Pass 2: per-pixel lighting accumulation
 *   T5  Many lights cheaply — what changes when you press 'l'
 *   T6  G-buffer visualisation — POSITION / NORMAL / ALBEDO / LIGHTING
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  FORWARD VS DEFERRED — THE COST EQUATION
 * ────────────────────────────────────────────
 * In a FORWARD renderer (cube_raster.c, sphere_raster.c, etc), the
 * loop structure is:
 *
 *     for each object O in scene:
 *       for each pixel P covered by O:
 *         for each light L:
 *           accumulate shading(O, P, L)
 *
 * Total shading invocations ≈ (covered pixels) × (lights). For a
 * scene with M objects and N lights you do M·N shader calls per
 * pixel — even though most light/object combinations contribute
 * nothing (a light far away from one object barely lights it).
 *
 * In a DEFERRED renderer, the loops are SWAPPED and DECOUPLED:
 *
 *     PASS 1 (geometry):
 *       for each object O:
 *         for each pixel P covered by O:
 *           write surface data (pos, normal, albedo) at P     ← ONCE
 *
 *     PASS 2 (lighting):
 *       for each pixel P with valid surface data:
 *         for each light L:
 *           accumulate shading(P_data, L)                      ← N×pixels
 *
 * Total: M (geometry) + pixels × N (lighting). For dense scenes
 * with many lights, deferred wins because:
 *   - Geometry pass cost is INVARIANT to N (number of lights)
 *   - Adding a 10th light only costs `pixels` more shader invocations,
 *     not `M × pixels` more
 *
 * That guarantee — "geometry once, lighting per-light" — is the
 * entire point of deferred rendering and what real-time engines
 * (Unity HDRP, Unreal default deferred) exploit.
 *
 * Trade-offs (good to know):
 *   - Memory: 4-6 buffers worth of per-pixel data (g_pos, g_normal,
 *     g_albedo, g_zbuf, g_valid, g_light here) instead of just one
 *     framebuffer.
 *   - Transparency: deferred handles transparency POORLY (per-pixel
 *     g_pos overwrites). Real engines fall back to forward for glass.
 *   - Anti-aliasing: harder in deferred (need TAA or MSAA-with-
 *     resolve). We don't AA here.
 *
 * For TEACHING purposes the cost-equation flip is the WHOLE point.
 *
 * T2  THE G-BUFFER — FLATTEN GEOMETRY INTO PER-PIXEL ARRAYS
 * ──────────────────────────────────────────────────────────
 * The G-BUFFER is a set of parallel 2-D arrays, one per surface
 * attribute, storing the CLOSEST surface's data at each pixel
 * position. Standard layout (varies between engines):
 *
 *   g_pos[r][c]      Vec3   world-space hit position
 *   g_normal[r][c]   Vec3   world-space surface normal
 *   g_albedo[r][c]   Vec3   base material colour
 *   g_zbuf[r][c]     float  NDC depth (for hidden-surface removal)
 *   g_valid[r][c]   bool    1 = pixel has surface, 0 = sky
 *
 * After PASS 1, every pixel that hits a surface has its full surface
 * properties recorded — and the geometry data is no longer needed.
 *
 * The G-buffer is the INTERFACE between the two passes. Pass 1
 * writes; Pass 2 only reads. This is what enables independent
 * scaling.
 *
 * The "G" stands for GEOMETRY (Saito & Takahashi 1990). Modern
 * engines call the same thing GBufferA, GBufferB, GBufferC...
 *
 * In code we declare them as static arrays sized for the maximum
 * resolution we expect:
 *
 *     static Vec3  g_pos    [GBUF_MAX_ROWS][GBUF_MAX_COLS];
 *     static Vec3  g_normal [GBUF_MAX_ROWS][GBUF_MAX_COLS];
 *     static Vec3  g_albedo [GBUF_MAX_ROWS][GBUF_MAX_COLS];
 *     static float g_zbuf   [GBUF_MAX_ROWS][GBUF_MAX_COLS];
 *     static int   g_valid  [GBUF_MAX_ROWS][GBUF_MAX_COLS];
 *     static Vec3  g_light  [GBUF_MAX_ROWS][GBUF_MAX_COLS];   pass-2 output
 *
 * Read §6.1 for the full declaration.
 *
 * T3  PASS 1: WRITING TO THE G-BUFFER
 * ────────────────────────────────────
 * Pass 1 is the standard rasteriser (cube_raster.c) BUT with the
 * fragment shader replaced by "write surface data, don't compute
 * lighting":
 *
 *   render_gbuffer(scene, view, proj):
 *     gbuffer_clear()
 *     for each object O:
 *       mvp     = proj · view · O.model
 *       nrm_mat = cofactor(O.model)
 *       for each triangle:
 *         clip[i] = mvp · vertex[i].pos
 *         if all behind near plane: skip
 *         (sx, sy, sz) = perspective divide + screen mapping
 *         if back-facing (signed area ≤ 0): skip
 *         for each pixel in screen bbox:
 *           (b0, b1, b2) = barycentric
 *           if any < 0: skip                        outside triangle
 *           z = b0·sz0 + b1·sz1 + b2·sz2            interpolated NDC z
 *           if z >= g_zbuf[r][c]: skip              z-test
 *           g_zbuf  [r][c] = z
 *           g_pos   [r][c] = b0·world_pos0 + b1·world_pos1 + b2·world_pos2
 *           g_normal[r][c] = b0·world_nrm0 + ... (then normalise)
 *           g_albedo[r][c] = O.albedo
 *           g_valid [r][c] = 1
 *
 * Same machinery as the forward pipeline, just with different output.
 * No lighting — that's pass 2's job. Read §6.3 + §6.4.
 *
 * T4  PASS 2: PER-PIXEL LIGHTING ACCUMULATION
 * ────────────────────────────────────────────
 * Pass 2 walks every VALID pixel of the G-buffer and accumulates
 * lighting:
 *
 *   render_lightpass(scene, cam_pos, lights):
 *     for r in 0..rows:
 *       for c in 0..cols:
 *         if not g_valid[r][c]: continue        sky pixel
 *         P      = g_pos    [r][c]
 *         N      = g_normal [r][c]
 *         albedo = g_albedo [r][c]
 *         result = AMBIENT · albedo               base illumination
 *         for each light L:
 *           result += blinn_phong(P, N, albedo, L, cam_pos)
 *         g_light[r][c] = clamp01(result)
 *
 * blinn_phong is the standard sum: ambient + diffuse·max(N·L) +
 * specular·max(N·H)^shininess. The pass NEVER touches mesh data —
 * just per-pixel surface attributes from the G-buffer.
 *
 * Read §7 for the full implementation.
 *
 * T5  MANY LIGHTS CHEAPLY — WHAT CHANGES WHEN YOU PRESS 'L'
 * ──────────────────────────────────────────────────────────
 * Pressing `l' adds another light to the scene. What's the cost
 * impact?
 *
 *   FORWARD:  M·N → M·(N+1)
 *             extra cost = M · pixels covered by ALL objects
 *
 *   DEFERRED: M + pixels·N → M + pixels·(N+1)
 *             extra cost = pixels (only the lighting loop grows)
 *
 * For this scene (M ≈ 1 sphere, N starts at 3, pixels ≈ 3000):
 *   forward extra: 3000 (per object, per added light)
 *   deferred extra: 3000 (just per added light)
 *
 * They look similar with M=1. With M=20 cube_raster-style scenes:
 *   forward extra: 20 · 3000 = 60,000
 *   deferred extra: 3000
 *
 * That's the deferred advantage — INDEPENDENT of geometry complexity
 * once you're in the lighting pass.
 *
 * The HUD shows the math live: pressing `l' adds light, the cost
 * line shows current forward (M·N) vs deferred (M+pixels·N). At
 * MAX_LIGHTS=8 deferred is significantly cheaper.
 *
 * T6  G-BUFFER VISUALISATION — POSITION / NORMAL / ALBEDO / LIGHTING
 * ───────────────────────────────────────────────────────────────────
 * Cycling `g' switches between four DEBUG VIEWS of the G-buffer:
 *
 *   POSITION   warm-near, cool-far gradient based on z. Identifies
 *              "where in 3-D is each pixel?" Useful for verifying
 *              the perspective divide and screen mapping.
 *
 *   NORMAL     (N + 1) / 2 visualised as RGB. +X red, +Y green,
 *              +Z blue. Smooth gradients over a sphere → smooth
 *              normals working; faceted → flat normals.
 *
 *   ALBEDO     raw material colour, no lighting at all. The
 *              sphere appears as a flat untextured disc. Sets a
 *              baseline for what the lighting is adding.
 *
 *   LIGHTING   the actual lit output (g_light). RED + GREEN +
 *              BLUE point lights painting their colours onto the
 *              sphere; overlaps make secondary colours; full
 *              overlap = white.
 *
 * KEY OBSERVATION: only LIGHTING re-shades when you press `l'. The
 * other three layers are computed BEFORE any light is applied, so
 * they're invariant to light count — proof that the geometry pass
 * doesn't care about lighting.
 *
 * This is the deferred-rendering DEMO point. Press `l' multiple
 * times in NORMAL mode — nothing changes. Switch to LIGHTING — the
 * sphere lights up differently with each new light.
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
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + scene capacity */
enum {
    FPS_TARGET    = 60,
    FPS_UPDATE_MS = 500,
    HUD_ROWS      = 5,           /* yellow row 0 + 3 educational rows + cyan hint */
    MAX_OBJECTS   = 4,           /* one sphere — extras reserved for later        */
    MAX_LIGHTS    = 8,           /* 3 RGB primaries + 5 'l'-key extras            */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define DT_CAP_NS       (100 * NS_PER_MS)

/* §1.2 G-buffer dimensions (static, no malloc).
 * Sized for the largest expected terminal. Pixels outside are skipped. */
#define GBUF_MAX_W   300
#define GBUF_MAX_H    80

/* §1.3 view geometry */
#define CAM_FOV       (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR      0.1f
#define CAM_FAR       20.0f

/* CAM_DIST is the DEFAULT eye Z distance; +/- keys slide the eye between
 * CAM_DIST_MIN (close-up — sphere fills the screen) and CAM_DIST_MAX
 * (wide shot — sphere is small, all three coloured pools easy to track
 * at once). MIN is kept above BALL_RADIUS (0.95) so the camera never
 * crosses inside the sphere; MAX is well within CAM_FAR (20.0) so the
 * sphere never falls past the far plane. */
#define CAM_DIST      3.8f       /* default eye Z distance (world units) */
#define CAM_DIST_MIN  1.8f       /* closest zoom — just outside sphere   */
#define CAM_DIST_MAX  8.0f       /* farthest zoom                        */
#define CAM_ZOOM_STEP 0.2f       /* world units moved per +/- press      */

#define CELL_W        8     /* terminal cell width  (pixels)              */
#define CELL_H       16     /* terminal cell height (pixels)              */

/* §1.4 lighting */
#define SHININESS    32.0f
#define AMBIENT_STR   0.06f

/* §1.5 RGB-lights demo geometry (world units) */

/* WHITE SPHERE — the SINGLE object in the scene. Big enough to fill
 * roughly 60% of the screen so the three coloured pools have plenty
 * of room to be visible. Pure-white albedo (1, 1, 1) means coloured
 * lights show as their UNFILTERED hue — red light × white surface =
 * pure red, never pink-tinted-by-the-surface. */
#define BALL_RADIUS      0.95f
#define BALL_RINGS       16
#define BALL_SEGS        24

/* CAMERA — fixed pose, looking straight at the sphere from +Z, slight
 * elevation. When the only things moving are the LIGHTS, the eye
 * locks onto their coloured pools instead of tracking the viewpoint.
 * Pixel changes between frames ⇒ LIGHTING changes, never camera. */
#define CAM_EYE_Y        0.30f      /* slight elevation for some depth    */
#define CAM_LOOK_Y       0.0f       /* aim straight at sphere centre      */

/* §1.6 character ramp — Paul Bourke 92-char density ladder. */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.7 Bayer 4×4 dither (k_bayer[py%4][px%4] in [0,1)). */
static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};
#define DITHER_AMP   0.12f

/* §1.8 ncurses pair IDs.
 * Same layout as cube_raster / raytracers: 216 RGB cube pairs at
 * PAIR_CUBE_BASE, plus yellow PAIR_HUD and cyan PAIR_HINT for the
 * CLAUDE.md HUD spec. */
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
typedef struct { float m[4][4];     } Mat4;

static inline Vec3 v3(float x, float y, float z)         { return (Vec3){x,y,z}; }
static inline Vec4 v4(float x, float y, float z, float w){ return (Vec4){x,y,z,w}; }

static inline Vec3  v3_add  (Vec3 a, Vec3 b)  { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vec3  v3_sub  (Vec3 a, Vec3 b)  { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3  v3_scale(Vec3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }
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

/* m4_pt  — point  transform (w=1, translation applies). */
static inline Vec3 m4_pt(Mat4 m, Vec3 p)
{
    Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
    return v3(r.x, r.y, r.z);
}

/* m4_dir — direction transform (w=0, translation does not apply). */
static inline Vec3 m4_dir(Mat4 m, Vec3 d)
{
    Vec4 r = m4_mul_v4(m, v4(d.x, d.y, d.z, 0.f));
    return v3(r.x, r.y, r.z);
}

/*
 * m4_perspective — OpenGL-style perspective projection.
 *
 *   m[0][0] = (1/tan(fov/2)) / aspect   horizontal scale
 *   m[1][1] =  1/tan(fov/2)              vertical scale
 *   m[2][2] = (far + near) / (near - far) depth remapping
 *   m[2][3] = (2·far·near) / (near - far) depth bias
 *   m[3][2] = -1                          enables perspective divide
 *
 * After m4_mul_v4, clip.w = -z_view; dividing x,y,z by w is the
 * perspective divide that makes far things smaller.
 */
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

static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up)
{
    Vec3 f = v3_norm(v3_sub(at, eye));
    Vec3 r = v3_norm(v3(f.z*up.y - f.y*up.z,
                        f.x*up.z - f.z*up.x,
                        f.y*up.x - f.x*up.y));
    Vec3 u = v3(r.y*f.z - r.z*f.y,
                r.z*f.x - r.x*f.z,
                r.x*f.y - r.y*f.x);
    Mat4 m = m4_identity();
    m.m[0][0] = r.x; m.m[0][1] = r.y; m.m[0][2] = r.z; m.m[0][3] = -v3_dot(r, eye);
    m.m[1][0] = u.x; m.m[1][1] = u.y; m.m[1][2] = u.z; m.m[1][3] = -v3_dot(u, eye);
    m.m[2][0] = -f.x; m.m[2][1] = -f.y; m.m[2][2] = -f.z; m.m[2][3] = v3_dot(f, eye);
    return m;
}

/*
 * m4_normal_mat — cofactor (adjugate) of the upper-left 3×3.
 *
 * Correctly transforms normals under non-uniform scale. For a pure
 * rotation this equals the rotation matrix; for non-uniform scale it
 * differs in a way that preserves perpendicularity to the surface.
 */
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

/* ── §4 paint (216 RGB cube + Bourke ramp) ───────────────────────────── */

static int g_256;

/*
 * color_init — allocate 216 ncurses pairs as a 6×6×6 RGB cube + reserve
 * yellow PAIR_HUD and cyan PAIR_HINT for the HUD spec, plus PAIR_HUD_DIM
 * for the educational mid-rows of this file's verbose HUD.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);

    if (g_256) {
        for (int i = 0; i < 216; i++)
            init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
        init_pair(PAIR_HUD,  226, -1);     /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);     /* bright cyan   */
    } else {
        init_pair(PAIR_CUBE_BASE, COLOR_WHITE,  -1);
        init_pair(PAIR_HUD,       COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,   -1);
    }
}

static inline float clamp01  (float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float reinhard (float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/*
 * paint_cell — full RGB → terminal pipeline (matches cube_raster / raytracers).
 *
 *   1. Reinhard tone-map per channel.
 *   2. Gamma encode 1/2.2.
 *   3. Bayer 4×4 dither on the luma so neighbouring cells with similar
 *      brightness pick different ramp characters → smoother gradients
 *      on the coarse glyph grid.
 *   4. Quantise to 6×6×6 RGB cube → pair id.
 *   5. Pick density glyph from 92-char Bourke ramp by Rec.709 luma.
 *   6. A_BOLD on bright cells, A_DIM on dark cells.
 *
 * Tone-mapping must run BEFORE cube quantisation. Quantising linear
 * HDR puts every bright pixel into one cube cell; Reinhard opens the
 * dynamic range so colours spread.
 */
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

/* §5 ── tessellate_sphere: UV sphere with smooth normals ────────────── */

/*
 * Vertices on a (rings+1) × (segs+1) grid in spherical coords:
 *
 *   theta = π · i / rings        polar angle (0 = north, π = south)
 *   phi   = 2π · j / segs        azimuth angle
 *
 *   pos = R · (sin θ · cos φ,  cos θ,  sin θ · sin φ)
 *
 * Normal at each vertex equals the unit position vector (smooth
 * shading): adjacent vertices have slightly different normals, and
 * barycentric interpolation in the rasteriser blends them per-pixel.
 */
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

    /* Wire vertex grid into quads, each split into 2 CCW triangles. */
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

/* ── §6 G-buffer — geometry pass ─────────────────────────────────────── */

/* §6.1 ── G-buffer arrays ───────────────────────────────────────────── */

/*
 * Six parallel grids storing per-pixel surface data.
 *
 *   g_pos    — world-space position of frontmost surface
 *   g_normal — world-space unit normal
 *   g_albedo — flat surface colour (no lighting)
 *   g_zbuf   — NDC-z depth; reset to +1.0 each frame
 *   g_valid  — 1 if any geometry has touched this pixel
 *   g_light  — output of lighting pass (Pass 2)
 *
 * In a real GPU engine these would be GL_TEXTURE_2D render targets
 * bound as MRT (Multiple Render Targets):
 *   layout(location=0) out vec4 gPosition;
 *   layout(location=1) out vec4 gNormal;
 *   layout(location=2) out vec4 gAlbedo;
 *
 * Unity HDRP packs them as: Albedo+Roughness, Normal+Metallic,
 * Spec+AO, Emissive — same idea, more channels.
 */
static Vec3    g_pos   [GBUF_MAX_H][GBUF_MAX_W];
static Vec3    g_normal[GBUF_MAX_H][GBUF_MAX_W];
static Vec3    g_albedo[GBUF_MAX_H][GBUF_MAX_W];
static float   g_zbuf  [GBUF_MAX_H][GBUF_MAX_W];
static uint8_t g_valid [GBUF_MAX_H][GBUF_MAX_W];
static Vec3    g_light [GBUF_MAX_H][GBUF_MAX_W];

/*
 * gbuffer_clear — reset depth + valid flags before each frame.
 *
 * g_pos / g_normal / g_albedo don't need clearing because g_valid
 * gates ALL reads in render_lightpass and render_scene.
 *
 * OpenGL equivalent: glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
 */
static void gbuffer_clear(int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            g_zbuf [r][c] = 1.0f;     /* NDC far plane */
            g_valid[r][c] = 0;
        }
    }
}

/* §6.2 ── barycentric (Möller signed-area form) ─────────────────────── */

/*
 * Compute barycentric weights of point (px, py) inside triangle.
 *
 *   d  = (y1-y2)(x0-x2) + (x2-x1)(y0-y2)         signed area · 2
 *   b0 = ((y1-y2)(px-x2) + (x2-x1)(py-y2)) / d
 *   b1 = ((y2-y0)(px-x2) + (x0-x2)(py-y2)) / d
 *   b2 = 1 − b0 − b1
 *
 * If d ≈ 0 (degenerate triangle), return -1s so caller skips the pixel.
 */
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

/* §6.3 ── rasterize_object — vertex → cull → raster → G-buffer write ── */

/*
 * Software equivalent of the GPU vertex + rasterisation pipeline.
 * Three conceptual stages:
 *
 *   STAGE 1: VERTEX TRANSFORM       (GPU: Vertex Shader)
 *     Multiply position by MVP; multiply normal by norm_mat.
 *     Save world-space pos and normal for G-buffer writes.
 *
 *   STAGE 2: PERSPECTIVE DIVIDE + RASTERISATION SETUP   (GPU: fixed)
 *     Divide clip x,y,z by w → NDC.
 *     Map NDC to screen pixels (with Y-flip).
 *     Back-face cull via signed area.
 *
 *   STAGE 3: FRAGMENT / G-BUFFER WRITE  (GPU: Fragment Shader → MRT)
 *     For each pixel in the bounding box:
 *       barycentric → in-triangle test
 *       z-test → frontmost surface wins
 *       write g_pos, g_normal, g_albedo, g_zbuf, g_valid
 *
 * NO LIGHTING IS DONE HERE — that's what makes deferred rendering
 * deferred. The mesh is rasterised once; lighting is a separate pass.
 */
static void rasterize_object(const Mesh *mesh, Vec3 albedo,
                             Mat4 mvp, Mat4 model, Mat4 norm_mat,
                             int cols, int rows)
{
    for (int ti = 0; ti < mesh->ntri; ti++) {
        const Triangle *tri = &mesh->tris[ti];

        /* STAGE 1: vertex transform. */
        Vec4 clip[3];
        Vec3 wpos[3], wnrm[3];
        for (int vi = 0; vi < 3; vi++) {
            const Vertex *v = &mesh->verts[tri->v[vi]];
            clip[vi] = m4_mul_v4(mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
            wpos[vi] = m4_pt(model, v->pos);
            wnrm[vi] = v3_norm(m4_dir(norm_mat, v->normal));
        }

        /* Skip triangles entirely behind the near plane. */
        if (clip[0].w < 0.001f && clip[1].w < 0.001f && clip[2].w < 0.001f)
            continue;

        /* STAGE 2: perspective divide → screen. */
        float sx[3], sy[3], sz[3];
        for (int vi = 0; vi < 3; vi++) {
            float w = clip[vi].w; if (fabsf(w) < 1e-6f) w = 1e-6f;
            sx[vi] = ( clip[vi].x / w + 1.f) * 0.5f * (float)cols;
            sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows;   /* Y-flip */
            sz[vi] =   clip[vi].z / w;
        }

        /* Back-face cull: positive area = CCW = front face. */
        float area = (sx[1] - sx[0]) * (sy[2] - sy[0])
                   - (sx[2] - sx[0]) * (sy[1] - sy[0]);
        if (area <= 0.f) continue;

        /* Bounding box clamped to G-buffer. */
        int x0 = (int)fmaxf(0.f,        floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
        int x1 = (int)fminf(cols - 1.f,  ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
        int y0 = (int)fmaxf(0.f,        floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
        int y1 = (int)fminf(rows - 1.f,  ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

        /* STAGE 3: fragment / G-buffer write. */
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

/* §6.4 ── render_gbuffer — geometry pass over all scene objects ────── */

/*
 * After this returns, the G-buffer holds frontmost surface data at
 * every pixel — INDEPENDENT of how many lights exist. Adding lights
 * does not require re-running this function.
 *
 * OpenGL equivalent:
 *   glBindFramebuffer(GL_FRAMEBUFFER, gBuffer);
 *   glDrawBuffers(3, {GL_COLOR_ATTACHMENT0,1,2});
 *   for each object: glDrawElements(GL_TRIANGLES, ...);
 *   glBindFramebuffer(GL_FRAMEBUFFER, 0);
 */
static void render_gbuffer(const Mesh *meshes, const Vec3 *albedos,
                           const Mat4 *models, int n_objects,
                           const Mat4 *view,   const Mat4 *proj,
                           int cols, int rows)
{
    gbuffer_clear(cols, rows);
    for (int oi = 0; oi < n_objects; oi++) {
        Mat4 mv   = m4_mul(*view, models[oi]);
        Mat4 mvp  = m4_mul(*proj, mv);
        Mat4 nmat = m4_normal_mat(models[oi]);
        rasterize_object(&meshes[oi], albedos[oi], mvp, models[oi], nmat,
                         cols, rows);
    }
}

/* ── §7 lightpass — Blinn-Phong shading ─────────────────────────────── */

/* §7.1 ── PointLight type ───────────────────────────────────────────── */

/*
 * Each light orbits the origin in the XZ plane (horizontal ring) at a
 * given radius and height. pos is recomputed each tick by scene_tick:
 *   pos = (radius·cos θ,  height,  radius·sin θ)        with θ = orbit_angle.
 * Light variety comes from differing radius / speed / height / phase —
 * not from different orbit planes. Simpler, and easier to predict.
 */
typedef struct {
    Vec3  color;
    float orbit_radius;
    float orbit_speed;       /* radians / second                   */
    float height;            /* y-coord of the orbit ring          */
    float orbit_angle;       /* current θ, accumulated each tick   */
    Vec3  pos;               /* world-space position (derived)     */
} PointLight;

/* §7.2 ── blinn_phong — one light's contribution at a pixel ────────── */

/*
 * BLINN-PHONG MODEL:
 *
 *   L = normalize(light_pos − P)         surface → light
 *   V = normalize(cam_pos   − P)         surface → camera
 *   H = normalize(L + V)                  halfway vector
 *
 *   diff = max(0, N · L)                  Lambertian
 *   spec = max(0, N · H)^SHININESS        Blinn-Phong specular
 *
 *   contrib = albedo · light_col · diff + light_col · spec · 0.35
 *
 * Albedo modulates diffuse but NOT specular — specular reflects the
 * light's colour, not the surface's. 0.35 is the spec gain.
 *
 * GLSL equivalent (the fragment shader used in the lighting pass):
 *   vec3 L = normalize(light.pos - fragPos);
 *   vec3 V = normalize(camPos    - fragPos);
 *   vec3 H = normalize(L + V);
 *   float diff = max(dot(N, L), 0.0);
 *   float spec = pow(max(dot(N, H), 0.0), shininess);
 *   color += albedo * light.color * diff + light.color * spec * 0.35;
 */
static Vec3 blinn_phong(Vec3 P, Vec3 N, Vec3 albedo,
                        Vec3 light_pos, Vec3 light_col, Vec3 cam_pos)
{
    Vec3 L = v3_norm(v3_sub(light_pos, P));
    Vec3 V = v3_norm(v3_sub(cam_pos,   P));
    Vec3 H = v3_norm(v3_add(L, V));

    float diff = fmaxf(0.f, v3_dot(N, L));
    float spec = powf(fmaxf(0.f, v3_dot(N, H)), SHININESS);

    return v3(albedo.x * light_col.x * diff + light_col.x * spec * 0.35f,
              albedo.y * light_col.y * diff + light_col.y * spec * 0.35f,
              albedo.z * light_col.z * diff + light_col.z * spec * 0.35f);
}

/* §7.3 ── render_lightpass — accumulate Blinn-Phong over G-buffer ──── */

/*
 * The "full-screen quad fragment shader" in software. For each
 * G-buffer pixel where g_valid = 1:
 *   1. ambient = AMBIENT_STR · albedo (small uniform base illumination)
 *   2. for each active light: add blinn_phong contribution
 *   3. clamp to [0, 1] (poor-man's tone-map; paint_cell does Reinhard
 *      proper tone mapping at draw time)
 *
 * This function is INVARIANT to mesh data — the geometry has been
 * reduced to per-pixel surface properties. Adding a light costs ONE
 * more iteration of the inner loop, NOT another mesh rasterisation.
 */
static void render_lightpass(const PointLight *lights, int n_lights,
                             Vec3 cam_pos, int cols, int rows)
{
    Vec3 ambient = v3(AMBIENT_STR, AMBIENT_STR, AMBIENT_STR * 1.1f);

    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) { g_light[r][c] = v3(0, 0, 0); continue; }

            Vec3 P      = g_pos   [r][c];
            Vec3 N      = g_normal[r][c];
            Vec3 albedo = g_albedo[r][c];

            Vec3 lit = v3(ambient.x * albedo.x,
                          ambient.y * albedo.y,
                          ambient.z * albedo.z);

            for (int li = 0; li < n_lights; li++) {
                Vec3 contrib = blinn_phong(P, N, albedo,
                                           lights[li].pos, lights[li].color,
                                           cam_pos);
                lit.x += contrib.x;
                lit.y += contrib.y;
                lit.z += contrib.z;
            }

            g_light[r][c] = v3(fminf(1.f, lit.x),
                               fminf(1.f, lit.y),
                               fminf(1.f, lit.z));
        }
    }
}

/* ── §8 scene ────────────────────────────────────────────────────────── */

/* §8.1 ── GBufMode + Scene ──────────────────────────────────────────── */

typedef enum {
    MODE_POSITION = 0,
    MODE_NORMAL,
    MODE_ALBEDO,
    MODE_LIGHTING,
    MODE_COUNT,
} GBufMode;

static const char *k_mode_names[MODE_COUNT] = {
    "POSITION", "NORMAL", "ALBEDO", "LIGHTING",
};

/* Scene holds three parallel arrays describing every renderable object:
 *   meshes[i]  — the triangle data
 *   albedos[i] — its flat surface colour
 *   models[i]  — its world-space transform
 * That's everything render_gbuffer needs. We never had per-object spin
 * or per-object position state in this demo; the previous Object struct
 * just added indirection. */

/* §8.2 ── LIGHT_PRESETS — 3 RGB primaries + 5 'l'-key progression extras.
 *
 * THE PEDAGOGY:
 *
 *   Lights 0..2 are PURE RED, PURE GREEN, PURE BLUE — the additive-
 *   colour primaries every physics class shows with three flashlights.
 *   They share the same orbit (radius, speed, height) but start at
 *   angles 0°, 120°, 240° → they maintain a perfect equilateral
 *   triangle as they sweep around the sphere.
 *
 *   What you see on the white sphere:
 *     Pure RED patch slides across,  pure GREEN patch slides across,
 *     pure BLUE patch slides across — and where two patches overlap
 *     you get YELLOW (R+G), CYAN (G+B), or MAGENTA (R+B); where
 *     all three overlap you get WHITE.
 *
 *   That's the deferred-shading lighting pass made literally visible:
 *   each pixel's final colour = sum of every light's contribution.
 *
 *   Lights 3..7 are "deferred-cost-story" extras (yellow, cyan,
 *   magenta, white, orange) at varied orbits. Press 'l' to enable
 *   them one at a time. The G-buffer never re-renders; only the
 *   lighting accumulation grows.
 */
static const struct {
    Vec3  color;
    float orbit_radius, orbit_speed, height;
    float angle_start;
} LIGHT_PRESETS[MAX_LIGHTS] = {
    /* ── The three primaries — same orbit, 120° apart on a horizontal
     * ring just above the sphere's equator. The equilateral arrangement
     * is preserved as they sweep, so you always see the three coloured
     * pools in a clear triangular relationship. ── */
    { {1.00f, 0.00f, 0.00f}, 1.55f, 0.45f, 0.40f, 0.0000f },  /* RED   @ 0°  */
    { {0.00f, 1.00f, 0.00f}, 1.55f, 0.45f, 0.40f, 2.0944f },  /* GREEN @ 120° (2π/3) */
    { {0.00f, 0.00f, 1.00f}, 1.55f, 0.45f, 0.40f, 4.1888f },  /* BLUE  @ 240° (4π/3) */

    /* ── Progression extras — 'l'-key adds one at a time. Variety comes
     * from differing radius / speed / height / phase. Press 'l' until
     * 8 lights are active — the geometry pass cost stays fixed; only
     * the lighting accumulation grows. ── */
    { {1.00f, 1.00f, 0.00f}, 1.30f, 0.65f, -0.30f, 1.000f },  /* YELLOW   */
    { {0.00f, 1.00f, 1.00f}, 1.30f, 0.65f, -0.30f, 3.000f },  /* CYAN     */
    { {1.00f, 0.00f, 1.00f}, 1.30f, 0.65f, -0.30f, 5.000f },  /* MAGENTA  */
    { {1.00f, 1.00f, 1.00f}, 1.80f, 0.30f,  1.20f, 0.500f },  /* WHITE    */
    { {1.00f, 0.55f, 0.00f}, 1.20f, 0.80f,  0.00f, 2.500f },  /* ORANGE   */
};

typedef struct {
    /* Renderable objects — three parallel arrays. */
    Mesh        meshes [MAX_OBJECTS];
    Vec3        albedos[MAX_OBJECTS];
    Mat4        models [MAX_OBJECTS];
    int         n_objects;

    /* Lights. */
    PointLight  lights[MAX_LIGHTS];
    int         n_lights;

    /* Camera + projection. Pose is fixed except for +/- zoom, which
     * slides cam_pos along +Z (the eye axis). cam_dist is the live Z
     * distance — modifying it and calling scene_set_zoom() rebuilds
     * cam_pos and the view matrix. */
    Mat4        view, proj;
    Vec3        cam_pos;
    float       cam_dist;

    /* UI / framing. */
    GBufMode    mode;
    bool        paused;
    int         scene_cols;
    int         scene_rows;
} Scene;

/* §8.3 ── scene_init / scene_set_zoom / scene_tick ──────────────────── */

static void scene_rebuild_proj(Scene *s, int cols, int rows)
{
    /* Aspect from PIXEL counts (cols·CELL_W, rows·CELL_H), NOT cell counts.
     * Without this the cube renders as a vertically squashed rectangle. */
    float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
    s->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

/*
 * scene_set_zoom — push cam_dist into cam_pos + view matrix.
 *
 * Zoom slides the eye along +Z only; the sphere stays at the origin
 * and the three coloured pools keep their relative arrangement on its
 * surface. Both the rasteriser (mvp = proj·view·model) and the lighting
 * pass (V = normalize(cam_pos − P) inside blinn_phong) read cam_pos and
 * view, so this single rebuild is enough to make the next frame's
 * geometry pass AND specular highlights track the new viewpoint.
 *
 * Equivalent to scene_set_zoom() in raster/cube_raster.c.
 */
static void scene_set_zoom(Scene *s)
{
    s->cam_pos = v3(0.f, CAM_EYE_Y, s->cam_dist);
    s->view    = m4_lookat(s->cam_pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
}

/*
 * scene_init — build the RGB-LIGHTS DEMO from scratch.
 *
 * Layout:
 *   meshes[0]               :  one white sphere at the origin
 *   models[0]               :  identity (sphere stays at origin)
 *   lights[0..2]            :  RED, GREEN, BLUE primaries (active by default)
 *   lights[3..7]            :  reserved progression extras (press 'l' to enable)
 *
 * Why this minimal layout? A single white sphere + three saturated
 * primaries is the canonical "additive light" demo every graphics
 * textbook draws. Only here it's real-time software-rendered, and you
 * can see the deferred lighting pass accumulate one light at a time
 * by pressing 'l'.
 */
static void scene_init(Scene *s, int total_cols, int total_rows)
{
    /* Free meshes from previous scene (reset / resize). */
    for (int i = 0; i < s->n_objects; i++) mesh_free(&s->meshes[i]);

    memset(s, 0, sizeof *s);
    s->scene_cols = total_cols;
    s->scene_rows = total_rows - HUD_ROWS;
    s->mode       = MODE_LIGHTING;

    /* Camera — pose is fixed except for +/- zoom. cam_dist lives in
     * Scene so the user can move the eye at runtime; scene_set_zoom
     * derives cam_pos and the view matrix from it. */
    s->cam_dist = CAM_DIST;
    scene_set_zoom(s);
    scene_rebuild_proj(s, total_cols, s->scene_rows);

    /* ── The one and only object: a white sphere at the origin. ── */
    s->meshes [0] = tessellate_sphere(BALL_RADIUS, BALL_RINGS, BALL_SEGS);
    s->albedos[0] = v3(1.0f, 1.0f, 1.0f);     /* pure white — no filtering  */
    s->models [0] = m4_identity();             /* at origin, no rotation     */
    s->n_objects  = 1;

    /* ── Three RGB primaries active by default. Press 'l' to enable
     * one more (up to MAX_LIGHTS). ── */
    s->n_lights = 3;

    /* Seed every preset's state — even the inactive extras, so 'l' just
     * bumps n_lights without re-seeding mid-animation. */
    for (int li = 0; li < MAX_LIGHTS; li++) {
        PointLight *l = &s->lights[li];
        l->color        = LIGHT_PRESETS[li].color;
        l->orbit_radius = LIGHT_PRESETS[li].orbit_radius;
        l->orbit_speed  = LIGHT_PRESETS[li].orbit_speed;
        l->height       = LIGHT_PRESETS[li].height;
        l->orbit_angle  = LIGHT_PRESETS[li].angle_start;
    }
}

/*
 * scene_tick — advance simulation by dt seconds.
 *
 * The only moving things are the lights. Sphere is static, camera is
 * static. The whole tick is just: each light's angle += speed·dt, then
 * rebuild its (x, y, z) position from (radius, angle, height).
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    for (int li = 0; li < MAX_LIGHTS; li++) {
        PointLight *l = &s->lights[li];
        l->orbit_angle += l->orbit_speed * dt;
        float r = l->orbit_radius;
        l->pos  = v3(r * cosf(l->orbit_angle),
                     l->height,
                     r * sinf(l->orbit_angle));
    }
}

/* §8.4 ── mode_to_rgb — G-buffer layer → RGB at one pixel ───────────── */

/*
 * Each mode produces a V3 RGB at the cell. The unified paint_cell
 * (§4) handles tone-mapping, gamma, cube quantise, and ramp char.
 *
 *   POSITION  — depth gradient: warm-near (z=-1) → cool-far (z=+1).
 *               R=high near, B=high far, G=middle for visual gradient.
 *   NORMAL    — (N + 1) / 2 mapping. Each face gets a unique RGB
 *               tint; sphere shows a smooth gradient across the
 *               surface. The classic "world-normal-as-color" debug.
 *   ALBEDO    — raw g_albedo, no lighting at all.
 *   LIGHTING  — final shaded result from g_light.
 */
static Vec3 mode_to_rgb(GBufMode mode, int r, int c)
{
    if (!g_valid[r][c]) return v3(0, 0, 0);

    switch (mode) {
    case MODE_POSITION: {
        /* NDC-z ranges from -1 (near) to +1 (far). Remap to t∈[0,1]. */
        float t = (g_zbuf[r][c] + 1.f) * 0.5f;
        if (t < 0.f) t = 0.f;
        if (t > 1.f) t = 1.f;
        /* Warm (red/yellow) near, cool (blue) far. */
        return v3(0.95f - t*0.7f,
                  0.55f - t*0.3f + (1.f-t)*0.2f,
                  0.30f + t*0.65f);
    }
    case MODE_NORMAL: {
        Vec3 N = g_normal[r][c];
        return v3(N.x*0.5f + 0.5f, N.y*0.5f + 0.5f, N.z*0.5f + 0.5f);
    }
    case MODE_ALBEDO:
        return g_albedo[r][c];
    case MODE_LIGHTING:
        return g_light[r][c];
    default:
        return v3(0, 0, 0);
    }
}

/* ── §9 screen — render_scene + HUD spec compliance ──────────────────── */

/*
 * render_scene — paint every pixel via the unified paint_cell pipeline.
 *
 * The active G-buffer mode produces a V3 RGB; paint_cell handles
 * Reinhard tone-map + gamma + 6×6×6 cube quantise + Bourke ramp char
 * + A_BOLD/A_DIM. Same paint pipeline as cube_raster + every raytracer.
 *
 * Pressing 'l' to add lights changes ONLY g_light. The POSITION,
 * NORMAL, and ALBEDO views are pixel-perfect identical to before —
 * that's the deferred-rendering guarantee, plainly visible.
 */
static void render_scene(const Scene *s)
{
    int cols = s->scene_cols;
    int rows = s->scene_rows;

    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) continue;
            Vec3 col = mode_to_rgb(s->mode, r, c);
            paint_cell(c, r, col);
        }
    }
}

/*
 * cube_pair — quantise a Vec3 RGB into one of the 216 cube pairs.
 *
 * Used by the HUD to render light swatches in each light's own colour.
 * Same 6×6×6 cube mapping as paint_cell, factored out so the swatch
 * loop doesn't re-derive (r5, g5, b5) twice (once for attron, once
 * for attroff — the previous version had that dual-derivation bug).
 */
static int cube_pair(Vec3 col)
{
    int r5 = (int)(clamp01(col.x) * 5.f + 0.5f);
    int g5 = (int)(clamp01(col.y) * 5.f + 0.5f);
    int b5 = (int)(clamp01(col.z) * 5.f + 0.5f);
    return PAIR_CUBE_BASE + r5*36 + g5*6 + b5;
}

/*
 * hud_draw — CLAUDE.md HUD spec + 3 educational rows in between.
 *
 * Layout (HUD_ROWS = 5):
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   scene_rows + 0 PAIR_HUD                  — algorithm cost summary
 *   scene_rows + 1 PAIR_HUD                  — mode-specific explanation
 *   scene_rows + 2 PAIR_HUD                  — light colour swatches
 *   scene_rows + 3 PAIR_HUD                  — blank spacer (gives breathing room)
 *   scene_rows + 4 PAIR_HINT (cyan + bold)   — key hint
 */
static void hud_draw(const Scene *s, double fps)
{
    int hr   = s->scene_rows;     /* first HUD row = first row past scene */
    int cols = s->scene_cols;

    int total_tris = 0;
    for (int i = 0; i < s->n_objects; i++) total_tris += s->meshes[i].ntri;

    int fwd_calls   = s->n_objects * s->n_lights;       /* obj × lights    */
    int defer_calls = s->n_objects + s->n_lights;       /* obj + lights    */
    int saved_pct   = fwd_calls > 0
                    ? 100 * (fwd_calls - defer_calls) / fwd_calls : 0;

    /* ── Row 0: title left, status right (yellow + bold per spec). ── */
    char status[120];
    snprintf(status, sizeof status,
             " %5.1f fps  mode:%s  lights:%d  zoom:%.1f  tris:%d  %s ",
             fps, k_mode_names[s->mode],
             s->n_lights, (double)s->cam_dist, total_tris,
             s->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > cols) slen = cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - slen, "%s", status);
    mvprintw(0, 0, " DEFERRED · RGB ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* ── Educational rows (yellow, no bold so they sit under the
     *    primary status row visually). ── */
    attron(COLOR_PAIR(PAIR_HUD));

    mvprintw(hr + 0, 1,
             "fwd = %d (obj × lights)   def = %d (obj + lights)   saved = %d%%",
             fwd_calls, defer_calls, saved_pct);

    const char *explain = "";
    switch (s->mode) {
    case MODE_POSITION:
        explain = "POSITION: warm-near → cool-far depth. 3-D layout BEFORE lighting.";
        break;
    case MODE_NORMAL:
        explain = "NORMAL: (N+1)/2 RGB. Smooth sphere → smooth gradient.";
        break;
    case MODE_ALBEDO:
        explain = "ALBEDO: flat surface colour, NO lighting. 'l' never changes this.";
        break;
    case MODE_LIGHTING:
        explain = "LIGHTING: Blinn-Phong over G-buffer. 'l' adds a light, geometry stays.";
        break;
    default: break;
    }
    mvprintw(hr + 1, 1, "%s", explain);

    /* Light swatch — one '@' per active light, painted in that light's
     * own colour pair so red light shows red, green shows green, etc. */
    mvprintw(hr + 2, 1, "Lights:");
    attroff(COLOR_PAIR(PAIR_HUD));
    int lx = 9;
    for (int li = 0; li < s->n_lights && lx < cols - 2; li++) {
        int pair = cube_pair(s->lights[li].color);
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(hr + 2, lx, (chtype)(unsigned char)'@');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        lx += 2;
    }

    /* ── Cyan hint bottom row (per spec: A_BOLD, never A_DIM). ── */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(hr + HUD_ROWS - 1, 0,
             " q:quit  spc:pause  g:layer  l:add-light  +/-:zoom  r:reset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §10 app ─────────────────────────────────────────────────────────── */

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
    case 'g': case 'G': s->mode = (GBufMode)((s->mode + 1) % MODE_COUNT); break;
    case 'l': case 'L':
        s->n_lights = (s->n_lights >= MAX_LIGHTS) ? 1 : s->n_lights + 1;
        break;
    case '=': case '+':
        s->cam_dist -= CAM_ZOOM_STEP;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        scene_set_zoom(s);
        break;
    case '-': case '_':
        s->cam_dist += CAM_ZOOM_STEP;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        scene_set_zoom(s);
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
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

        /* §10.1 resize. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        /* §10.2 timing. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;
        float dt_sec = (float)dt / (float)NS_PER_SEC;

        /* §10.3 advance scene. */
        scene_tick(&app->scene, dt_sec);

        /* §10.4 fps rolling average. */
        fps_cnt++;
        fps_acc += dt;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
            fps_cnt = 0; fps_acc = 0;
        }

        /* §10.5 frame cap. */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);

        /* §10.6 THE FULL DEFERRED RENDERING PIPELINE.
         *
         *   PASS 1 — render_gbuffer:   geometry → G-buffer (pos/normal/albedo)
         *   PASS 2 — render_lightpass: G-buffer + lights → g_light
         *   PASS 3 — render_scene:     mode_to_rgb per pixel → paint_cell
         *   PASS 4 — hud_draw:         yellow status + cyan hint + edu rows
         */
        Scene *s = &app->scene;
        erase();
        render_gbuffer(s->meshes, s->albedos, s->models, s->n_objects,
                       &s->view, &s->proj, s->scene_cols, s->scene_rows);
        render_lightpass(s->lights, s->n_lights, s->cam_pos,
                         s->scene_cols, s->scene_rows);
        render_scene(s);
        hud_draw(s, fps_display);
        screen_present();

        /* §10.7 input. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    for (int i = 0; i < app->scene.n_objects; i++)
        mesh_free(&app->scene.meshes[i]);

    endwin();
    return 0;
}
