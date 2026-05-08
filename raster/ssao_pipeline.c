/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ssao_pipeline.c — software screen-space ambient occlusion (SSAO)
 *
 * DEMO: A small three-step ziggurat (warm sandstone) with a pale
 *       cream sphere resting on its top, on a cool slate floor. The
 *       camera orbits slowly. Press 'a' to cycle three modes:
 *
 *         AO_ONLY     grayscale "crevice map" — bright everywhere
 *                     except FOUR dark features:
 *                       1. floor halo around the bottom step
 *                       2. step-0 top, around step-1's footprint
 *                       3. step-1 top, around step-2's footprint
 *                       4. circular halo on step-2 under the sphere
 *
 *         LIT_NO_AO   full Blinn-Phong, NO occlusion. Corners stay
 *                     bright, the sphere looks pasted onto the step.
 *
 *         LIT_WITH_AO ambient × AO + direct light. Same four corners
 *                     darken; the sphere "sits" on the step and the
 *                     pyramid steps gain depth.
 *
 *       Toggling LIT_NO_AO ↔ LIT_WITH_AO with the orbit paused is
 *       the cleanest way to see what SSAO contributes — only the
 *       four corner regions change. Flat faces are pixel-identical.
 *
 *       Press '[' / ']' to shrink / grow the SSAO radius — watch
 *       every dark band narrow / widen as the sample neighbourhood
 *       does. The sphere's halo is the smallest feature; the floor
 *       halo is the largest. Both scale together with radius.
 *
 * Study alongside:
 *   raster/deferred_rendering_pipeline.c — same G-buffer, no AO
 *   raster/cube_raster.c                 — single-mesh forward raster
 *
 * Section map:
 *   §1  config     — frame, view, scene, SSAO, lighting, ramp, pairs
 *   §2  clock      — monotonic timer + sleep
 *   §3  math       — V3, V4, Mat4 + perspective / lookat / normal mat
 *   §4  paint      — 216-pair RGB cube + Bourke ramp + paint_cell
 *   §5  mesh       — Vertex / Triangle types + box / quad / sphere
 *   §6  gbuffer    — geometry pass (pos, normal, albedo, NDC + view-z)
 *   §7  ssao       — kernel + per-pixel sample loop + 3×3 blur
 *   §8  lightpass  — Blinn-Phong; ambient is scaled by blurred AO
 *   §9  scene      — Mode enum, Scene struct, init / view / tick
 *   §10 screen     — render_scene + HUD (CLAUDE.md spec)
 *   §11 app        — signals, resize, fixed-step main loop
 *
 * Keys:
 *   a / A     cycle output mode (AO_ONLY → LIT_NO_AO → LIT_WITH_AO)
 *   [         shrink SSAO radius
 *   ]         grow   SSAO radius
 *   + / =     zoom in   (decrease camera distance)
 *   - / _     zoom out  (increase camera distance)
 *   space     pause / resume camera orbit
 *   r / R     reset
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/ssao_pipeline.c -o ssao -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Screen-Space Ambient Occlusion. For each visible
 *                  pixel, take K samples in the upper hemisphere along
 *                  the surface normal, project them back to the screen,
 *                  and ask the depth buffer "is some surface in front
 *                  of this sample?" The fraction of YES answers is the
 *                  occlusion factor; (1 − occluded) scales the ambient
 *                  illumination term so corners darken without affecting
 *                  direct light.
 *
 *                  A pure screen-space heuristic — no rays, no GI. It
 *                  cannot see geometry behind objects or off-screen.
 *                  But it costs O(pixels × K) and produces the iconic
 *                  "dark crevice" look every modern real-time engine
 *                  ships with.
 *
 * Data-structure : The G-buffer arrays from deferred rendering
 *                    g_pos, g_normal, g_albedo, g_zbuf, g_z_view, g_valid
 *                  plus AO arrays
 *                    g_ao, g_ao_blur
 *                  All static; no malloc on the hot path.
 *
 *                  Plus a precomputed kernel:
 *                    k_ssao[VARIANTS][SAMPLES]
 *                  Each pixel picks ONE variant by (c & 1) | ((r & 1) << 1)
 *                  so the 2×2 tile uses every variant once. The 3×3
 *                  blur smooths the resulting checkerboard.
 *
 * Rendering      : Four passes per frame (subset by mode):
 *                    1. render_gbuffer   — pos / normal / albedo / depth
 *                    2. ssao_pass        — K samples per pixel → g_ao
 *                    3. ssao_blur        — 3×3 box average → g_ao_blur
 *                    4. render_lightpass — Blinn-Phong, ambient × AO
 *
 *                  Output paint pipeline matches every other raster
 *                  file: per-pixel V3 RGB → Reinhard tone-map → gamma
 *                  → 6×6×6 cube + 92-char Bourke ramp.
 *
 * Performance    : SSAO is O(pixels × K). At terminal resolution
 *                  (≈80×40 ≈ 3 000 pixels × 12 samples) that's ~36k
 *                  ops per frame — trivial. Real engines run SSAO at
 *                  half resolution + bilateral upsample to amortise.
 *
 * References     : Mittring, "Finding Next Gen — CryEngine 2,"
 *                    SIGGRAPH '07 course notes (the original SSAO).
 *                  LearnOpenGL, "SSAO" tutorial:
 *                    https://learnopengl.com/Advanced-Lighting/SSAO
 *                  Bavoil & Sainz, "Multi-Layer Dual-Resolution
 *                    Screen-Space Ambient Occlusion," SIGGRAPH '09.
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
 * If a surface point has lots of OTHER surface crowding it in 3-D, it
 * must be tucked into a corner — and corners darken in real life
 * because less ambient light reaches them. SSAO doesn't trace any
 * light; it just asks, for each pixel, "what fraction of the
 * hemisphere above this surface is blocked by nearby geometry?" That
 * fraction multiplies the ambient term ONLY. Direct lighting stays
 * untouched.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine each visible pixel sticks K little antennas into the air,
 * each pointing in a random direction in the upper hemisphere along
 * the surface normal. Each antenna asks its tip: "is there a closer
 * surface between you and the camera?" Open-sky antennas say NO and
 * the pixel stays bright. Antennas pointing into a wall say YES and
 * the pixel darkens. The output is (1 − occluded / total).
 *
 *      ┌──────────────────────────────────────────────┐
 *      │       open surface          inside corner    │
 *      │       \   |   /                  ┌──         │
 *      │        \  |  /                   │           │
 *      │   ──────P──────              ────P──         │
 *      │   K rays into sky           K rays into wall │
 *      │   → 0/K occluded            → many/K         │
 *      │   → AO = 1.0  (bright)      → AO ≈ 0.3 (dim) │
 *      └──────────────────────────────────────────────┘
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 * (per frame, after render_gbuffer has filled g_pos / g_normal / g_zbuf)
 *
 *   1. SSAO pass — for each valid pixel (r, c):
 *       a. P = g_pos[r][c],  N = g_normal[r][c]
 *       b. variant = (c & 1) | ((r & 1) << 1)
 *       c. for each direction dir in k_ssao[variant]:
 *             if dot(dir, N) < 0:  dir = −dir         // hemisphere flip
 *             S         = P + dir · radius             // antenna tip
 *             clip      = VP · (S, 1)
 *             if clip.w < ε:  skip                     // behind camera
 *             sx, sy    = NDC → screen(clip)
 *             if (sx, sy) outside or g_valid = 0: skip
 *             dz        = | g_z_view[r][c] − g_z_view[sy][sx] |
 *             attn      = max(0, 1 − dz / radius)      // range falloff
 *             total_w  += attn
 *             if g_zbuf[sy][sx] < ndc_z(S) − BIAS:     // depth test
 *                 occlude_w += attn
 *       d. g_ao[r][c] = 1 − occlude_w / max(total_w, ε)
 *
 *   2. Blur pass — 3×3 box average over g_valid only:
 *        g_ao_blur[r][c] = mean(g_ao[r±1][c±1] where g_valid = 1)
 *
 *   3. Light pass — for each valid pixel:
 *        ambient_lit = AMBIENT · albedo · g_ao_blur[r][c]    ← AO HERE
 *        direct_lit  = blinn_phong(P, N, albedo, sun, cam)
 *        g_light[r][c] = clamp01(ambient_lit + direct_lit)
 *
 * KEY FORMULAS
 * ────────────
 *   Hemisphere flip:   if (dir · N) < 0  then dir ← −dir
 *   Sample point:      S = P + dir · radius
 *   Projection:        clip = VP · (S, 1);  ndc = clip / clip.w
 *                      sx = ( ndc.x + 1) / 2 · cols
 *                      sy = (−ndc.y + 1) / 2 · rows         (Y-flip)
 *   Range falloff:     attn = max(0, 1 − |Δz_view| / radius)
 *   Depth test:        g_zbuf[sy][sx] < ndc_z(S) − BIAS  → occluded
 *   AO factor:         AO = 1 − Σ(occl · attn) / max(Σ attn, ε)
 *   Box blur:          AO′[r][c] = mean(AO[r±1][c±1])
 *   Composite:         col = AMBIENT · albedo · AO′ + diffuse + spec
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Off-screen samples — projected (sx, sy) outside the G-buffer
 *     OR landing on g_valid = 0: SKIP (don't count). Failing to skip
 *     creates a dark halo around every silhouette.
 *
 *   • Self-occlusion bias — a flat surface's samples sit just above
 *     itself; without BIAS the depth test teeters on the edge and
 *     float jitter darkens flat regions into noisy gray. BIAS pushes
 *     the comparison plane slightly in front of the surface.
 *
 *   • Range check — without it, distant background "occludes" through
 *     the air and produces fake AO. attn zeroes out samples whose
 *     view-space Z is more than RADIUS from the centre pixel.
 *
 *   • Per-pixel kernel discontinuity — neighbouring pixels using
 *     different variants form a 2×2 checkerboard of AO values. The
 *     3×3 box blur smooths the checker.
 *
 *   • Normal correctness — if g_normal points the wrong way, every
 *     sample reads BEHIND the surface and the pixel goes 100 %
 *     occluded. Verify normals via a debug view if AO looks wrong.
 *
 *   • RADIUS too large — samples leave the local neighbourhood and
 *     SSAO becomes a soft global darkening, not crevice shadows.
 *     Too small — samples never leave the surface, AO ≈ 1.0.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • In AO_ONLY: bright everywhere except FOUR features:
 *       (1) floor halo around step 0's perimeter
 *       (2) step 0 top, where step 1's footprint sits
 *       (3) step 1 top, where step 2's footprint sits
 *       (4) circular halo on step 2 around the sphere base
 *     Count them — expect four, two of them concentric on top.
 *
 *   • Toggle LIT_NO_AO ↔ LIT_WITH_AO with orbit paused: only the
 *     four corner regions change brightness. Flat top faces of each
 *     step are pixel-identical between the two modes.
 *
 *   • Press '[' a few times: every dark band narrows (the sphere's
 *     halo first, since it's the smallest feature). Press ']': all
 *     bands widen together.
 *
 *   • Pause the orbit (space): the AO pattern is rock-stable per
 *     frame — no shimmer. Shimmer means the kernel variants are
 *     cycling instead of being deterministic.
 *
 *   • The HUD's "K=… R=…" readouts always match SSAO_SAMPLES and
 *     s->ssao_radius.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read raster/deferred_rendering_pipeline.c FIRST —
 *      SSAO is built on top of the deferred G-buffer, and you need
 *      to understand g_pos / g_normal / g_zbuf before you can read
 *      the AO loop.
 *   2. §1 config — every constant has a unit-bearing comment.
 *   3. §7 ssao — THE HEART. Read AFTER tutorials T1-T5. The
 *      ~30-line per-pixel sample loop is the entire algorithm.
 *   4. §8 lightpass — minimal change from deferred: one extra
 *      multiplier on the ambient term.
 *   5. §6 gbuffer — same pattern as deferred. Skim if familiar.
 *      Note the addition of g_z_view (view-space Z) used by the
 *      range-falloff calculation.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   g_pos / g_normal / g_albedo / g_zbuf / g_valid / g_light — same
 *     G-buffer layout as deferred_rendering_pipeline.c
 *   g_z_view[r][c]      view-space depth (linear distance from camera)
 *                       added for the range-falloff math
 *   g_ao[r][c]          raw AO factor ∈ [0, 1] from §7 ssao_pass
 *   g_ao_blur[r][c]     blurred AO ∈ [0, 1] from §7 ssao_blur
 *   k_ssao[V][K]        precomputed kernel: V variants × K samples
 *   variant             pixel chooses one variant by 2×2 tile pattern
 *   SSAO_RADIUS         physical radius (world units) of the
 *                       hemisphere we sample
 *   SSAO_BIAS           tiny depth offset to prevent self-occlusion
 *
 * Background you need
 * ───────────────────
 *   - Deferred rendering's G-buffer (deferred_rendering_pipeline.c).
 *   - Hemisphere sampling: K random unit vectors clustered around
 *     the surface normal.
 *   - Why AO multiplies AMBIENT and not direct light (T6).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - HBAO, GTAO, voxel AO — variants and refinements; we use
 *     vanilla Crytek-style SSAO (the original).
 *   - Cosine-weighted vs uniform hemisphere sampling — both work;
 *     we use uniform-ish.
 *   - Bilateral upsample — relevant when SSAO runs at half res; we
 *     run at full terminal res so plain box blur suffices.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Eight short tutorials. Read in order; each builds on the previous.
 *
 *   T1  What ambient occlusion IS — and why it darkens corners
 *   T2  Why "screen-space" — the cost trade vs ray-traced AO
 *   T3  Hemisphere sampling — K antennas pointed at the sky
 *   T4  The depth test — "is this antenna blocked?"
 *   T5  Range falloff — keeping AO local
 *   T6  Modulating AMBIENT only — never direct light
 *   T7  The 3×3 blur — denoising the per-pixel checkerboard
 *   T8  Three modes — AO_ONLY / LIT_NO_AO / LIT_WITH_AO
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT AMBIENT OCCLUSION IS — AND WHY IT DARKENS CORNERS
 * ───────────────────────────────────────────────────────────
 * Real-world surfaces in CORNERS look DARKER than identical surfaces
 * in the OPEN. Why? Ambient light (sky, indirect bounce) reaches an
 * open surface from a full hemisphere of directions. A surface
 * tucked into a corner has much of that hemisphere BLOCKED by
 * neighbouring geometry — less ambient light arrives, so the corner
 * darkens.
 *
 *      open: full sky access     corner: half sky blocked by wall
 *           ╲ │ ╱                            ┌──
 *            ╲│╱                             │
 *      ───────P───              ─────────────P──
 *                                              │ all sky from this
 *                                              │ side BLOCKED
 *
 * Ambient occlusion (AO) is the FRACTION OF THE HEMISPHERE that
 * remains UNblocked. AO = 1.0 → fully open, normal ambient. AO = 0.0
 * → fully enclosed, no ambient. The result multiplies the ambient
 * lighting term.
 *
 * Computing AO exactly requires casting many rays per pixel into the
 * hemisphere and asking each "does it hit something?" That's
 * expensive. SSAO is a CHEAP HEURISTIC that approximates the answer
 * using only the depth buffer of the current view (T2).
 *
 * T2  WHY "SCREEN-SPACE" — THE COST TRADE VS RAY-TRACED AO
 * ─────────────────────────────────────────────────────────
 * RAY-TRACED AO: cast K rays per pixel, intersect each with the full
 * scene geometry. Cost: O(pixels × K × scene_complexity). Slow.
 *
 * SCREEN-SPACE AO: cast K rays per pixel BUT only test against the
 * existing depth buffer — the per-pixel "depth from camera" data
 * computed during the geometry pass. Cost: O(pixels × K × 1). Fast.
 *
 * What you give up:
 *   - Off-screen geometry doesn't occlude (the depth buffer doesn't
 *     have it).
 *   - Geometry behind objects doesn't occlude either (the depth buffer
 *     stores only the closest surface per pixel).
 *
 * What you gain:
 *   - O(K) per pixel, completely independent of scene complexity.
 *   - Trivial to integrate into a deferred pipeline (just sample the
 *     depth buffer that's already there).
 *
 * For most scenes the cheating is invisible — corners and crevices
 * darken correctly because their occluders are visible to the camera
 * and therefore in the depth buffer. SSAO is now standard in every
 * real-time engine since CryEngine 2 shipped it (Mittring 2007).
 *
 * T3  HEMISPHERE SAMPLING — K ANTENNAS POINTED AT THE SKY
 * ────────────────────────────────────────────────────────
 * For each visible pixel with surface point P and normal N, generate
 * K sample directions clustered in the upper hemisphere along N:
 *
 *     for k in 0 .. K-1:
 *       dir = pre-computed random unit vector
 *       if dot(dir, N) < 0: dir = -dir       hemisphere flip
 *       sample_point = P + dir · SSAO_RADIUS
 *
 * `dir` is from a precomputed kernel of K unit vectors with random
 * orientations — uploaded once at startup (§1 + §7). The "if dot < 0,
 * flip" trick is cheaper than generating cosine-weighted samples
 * directly and works fine for AO.
 *
 * Each sample_point is "an antenna sticking up from P into the
 * hemisphere above the surface." We're going to ask each antenna
 * "is there a closer surface between you and the camera?" (T4).
 *
 * Per-pixel kernel variation: each pixel picks a kernel from
 * VARIANTS = 4 different precomputed sets, indexed by the 2×2 tile
 * pattern `(c & 1) | ((r & 1) << 1)`. This breaks the directional
 * banding that would result from EVERY pixel using the SAME K
 * samples. The result is a per-pixel checkerboard of AO values —
 * cleaned up by the 3×3 blur (T7).
 *
 * Read §1 k_ssao kernel + §7 ssao_pass.
 *
 * T4  THE DEPTH TEST — "IS THIS ANTENNA BLOCKED?"
 * ────────────────────────────────────────────────
 * For each sample_point S, project it back to the screen and read
 * the depth buffer at that pixel:
 *
 *     clip       = VP · (S, 1)
 *     ndc.z      = clip.z / clip.w
 *     (sx, sy)   = ( ndc.x + 1)/2 · cols, (-ndc.y + 1)/2 · rows
 *
 *     if g_zbuf[sy][sx] < ndc.z - BIAS:
 *       OCCLUDED        (something closer than S is at this pixel)
 *     else:
 *       NOT OCCLUDED    (S is the closest, or close enough)
 *
 * The intuition: if the depth buffer at S's projected pixel
 * recorded a CLOSER surface than where S actually sits, then
 * something occludes the path from camera to S.
 *
 * BIAS prevents self-occlusion. Without it, samples just above a
 * flat surface read the surface's own depth — float jitter then
 * randomly says "occluded" half the time, producing noisy gray
 * across flat regions. BIAS pushes the comparison threshold
 * slightly in front of the surface to fix this.
 *
 * Trade with bias: too small → acne, too large → AO disappears
 * inside thin features. SSAO_BIAS = 0.0008 in §1 is calibrated
 * for this scene's worst-case depth gradient.
 *
 * T5  RANGE FALLOFF — KEEPING AO LOCAL
 * ─────────────────────────────────────
 * Without falloff, a distant background object occludes a foreground
 * pixel — producing fake AO darkening that bears no relation to the
 * real corner geometry.
 *
 * Range falloff: weight each sample by HOW CLOSE the occluder is in
 * VIEW-SPACE Z:
 *
 *     dz_view = | g_z_view[r][c] - g_z_view[sy][sx] |
 *     attn    = max(0, 1 - dz_view / SSAO_RADIUS)
 *
 *     if SAMPLE_OCCLUDED:  occluded_weight += attn
 *     total_weight += attn
 *
 * AO = 1 - occluded_weight / total_weight
 *
 * Now an occluder more than SSAO_RADIUS from the surface contributes
 * 0 (attn = 0). Only LOCAL geometry can cast an AO darkening.
 *
 * SSAO_RADIUS becomes the "size of the AO halo" — small radius
 * gives tight crevice shadows; large radius gives fluffy soft
 * darkening that extends across object boundaries. Pressing `[' /
 * `]' adjusts it live.
 *
 * Note we use g_z_view (linear view-space Z) not g_zbuf (NDC z).
 * View-space Z varies linearly with distance; NDC z compresses
 * heavily near the far plane. Range-falloff math is much cleaner
 * with linear Z.
 *
 * T6  MODULATING AMBIENT ONLY — NEVER DIRECT LIGHT
 * ─────────────────────────────────────────────────
 * Crucial design choice: AO multiplies the AMBIENT lighting term
 * ONLY. Direct lighting (sun, point lights) is UNTOUCHED.
 *
 *     lit = AMBIENT · albedo · g_ao_blur     ← AO modulates this
 *         + diffuse · sun_col · max(0, N·L)  ← unchanged
 *         + specular · ...                   ← unchanged
 *
 * Why? Ambient light is the diffuse SKY/INDIRECT contribution from
 * the full hemisphere. That's exactly what "fraction of hemisphere
 * occluded" should affect — less hemisphere visible to the surface,
 * less ambient reaches it.
 *
 * Direct light (sun) reaches the surface from a SINGLE specific
 * direction. If the ray from the surface to the sun is BLOCKED, that's
 * a SHADOW (handled by shadow_mapping.c, not AO). If the ray is
 * UNBLOCKED, the sun fully reaches the surface regardless of how
 * many other directions are blocked — AO has nothing to say about it.
 *
 * Multiplying everything by AO would over-darken corners because
 * sun light would get blocked by ambient occlusion that doesn't
 * actually obstruct the sun's specific direction. Wrong physics.
 *
 * Read §8 render_lightpass — only the ambient line picks up AO.
 *
 * T7  THE 3×3 BLUR — DENOISING THE PER-PIXEL CHECKERBOARD
 * ────────────────────────────────────────────────────────
 * Per-pixel kernel variation (T3) means neighbouring pixels use
 * different K-sample sets. Each AO value is therefore a stochastic
 * estimate; neighbouring pixels can have noticeably different AO
 * values even on a flat surface.
 *
 * 3×3 box blur averages g_ao over the 3×3 neighbourhood:
 *
 *     g_ao_blur[r][c] = mean(g_ao[r±1][c±1] for valid pixels)
 *
 * Every pixel ends up sharing samples with its 8 neighbours →
 * checkerboard collapses into a smooth gradient. A 3×3 average is
 * sufficient because the variation is small and short-range.
 *
 * Edge handling: skip pixels where g_valid = 0 (sky pixels).
 * Otherwise the blur would pull AO toward the sky's default value
 * across silhouettes.
 *
 * Read §7 ssao_blur for the implementation.
 *
 * T8  THREE MODES — AO_ONLY / LIT_NO_AO / LIT_WITH_AO
 * ────────────────────────────────────────────────────
 * Cycling `a' switches between three OUTPUT MODES, each emphasising
 * a different aspect of the algorithm:
 *
 *   AO_ONLY      Grayscale visualisation of g_ao_blur. Bright
 *                everywhere except crevices and corners. Diagnostic:
 *                "is the AO map producing correct dark features?"
 *
 *   LIT_NO_AO    Full Blinn-Phong, AMBIENT term unmultiplied. The
 *                sphere "floats" above the step (no contact halo);
 *                the pyramid corners are bright; the floor under
 *                the bottom step is plain.
 *
 *   LIT_WITH_AO  Full Blinn-Phong, AMBIENT × AO. The sphere SITS on
 *                the step (visible contact halo); pyramid corners
 *                darken; floor under the bottom step has a faint
 *                surrounding shadow ring. The PROOF that AO adds
 *                grounding/depth.
 *
 * Toggling LIT_NO_AO ↔ LIT_WITH_AO with the orbit paused is the
 * cleanest way to see what SSAO contributes. Only the four corner
 * regions change. Flat faces are pixel-identical. The lighting
 * itself doesn't change — only the AMBIENT modulation.
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

/* §1.1 frame rate + UI */
enum {
    FPS_TARGET    = 60,
    FPS_UPDATE_MS = 500,
    HUD_ROWS      = 5,           /* row 0 + 3 edu rows + cyan hint */
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

#define CAM_DIST       6.0f
#define CAM_DIST_MIN   3.0f
#define CAM_DIST_MAX  12.0f
#define CAM_ZOOM_STEP  0.4f
#define CAM_EYE_Y      1.9f      /* slight elevation, sees floor + top */
#define CAM_LOOK_Y     1.3f      /* aim at structure mid-height        */
#define CAM_ORBIT_RAD_PER_SEC  0.20f

#define CELL_W        8
#define CELL_H       16

/* §1.4 SSAO parameters
 *
 * RADIUS  — how far in WORLD units the sampler looks. Small radius
 *           catches only tight inside-corners; large radius gives a
 *           soft global darkening. Default sized to ~step height.
 * BIAS    — small NDC-z offset so flat surfaces don't self-occlude.
 * SAMPLES × VARIANTS — 12 samples per pixel × 4 distinct kernels in
 *           a 2×2 tile = 48 directions sampled across each tile. The
 *           blur then averages over a 3×3 = 9-cell neighbourhood, so
 *           every blurred pixel is smoothed by 100+ contributions.
 */
#define SSAO_SAMPLES            12
#define SSAO_KERNEL_VARIANTS     4
#define SSAO_RADIUS_DEF       0.45f
#define SSAO_RADIUS_MIN       0.10f
#define SSAO_RADIUS_MAX       1.40f
#define SSAO_RADIUS_STEP      0.05f
#define SSAO_BIAS            0.0008f

/* §1.5 lighting — single warm sun + cool ambient.
 *
 * AO scales the AMBIENT term only. Make ambient bright enough that
 * the AO darkening is visible against the lit surface. The sun is
 * direct and unaffected by AO. */
static const float SUN_DIR[3]    = { -0.55f, -0.85f,  0.30f };
static const float SUN_COL[3]    = {  0.95f,  0.85f,  0.65f };
static const float AMBIENT_COL[3]= {  0.46f,  0.50f,  0.56f };
#define SHININESS    24.0f
#define SPEC_GAIN     0.30f

/* §1.6 scene geometry — stepped pyramid + sphere on a floor.
 *
 *                    sphere (r=0.35, centre y=2.75)
 *                       ●
 *                    ┌─┴┴─┐    step 2  (1.10 wide × 0.80 tall, y 1.6–2.4)
 *                  ┌─┴────┴─┐  step 1  (2.00 wide × 0.80 tall, y 0.8–1.6)
 *                ┌─┴────────┴─┐  step 0 (3.00 wide × 0.80 tall, y 0.0–0.8)
 *               ─┴────────────┴────────  floor (y = 0)
 *
 * Each step sits ON the previous step's top face — the y boundaries
 * coincide. The sphere's bottom touches the top of step 2 (y = 2.4),
 * giving the iconic round-on-flat contact halo for AO.
 *
 * Four AO features the demo highlights:
 *   1. floor halo around step 0's footprint
 *   2. step 0 top, around step 1's footprint
 *   3. step 1 top, around step 2's footprint
 *   4. step 2 top, circular halo under the sphere
 */
#define FLOOR_HALF_X    4.0f
#define FLOOR_HALF_Z    4.0f

#define STEP0_HX 1.50f
#define STEP1_HX 1.00f
#define STEP2_HX 0.55f
#define STEP_HY  0.40f      /* every step is 0.8 tall                 */
#define STEP0_CY (STEP_HY)                         /* 0.4  → y 0..0.8 */
#define STEP1_CY (STEP_HY * 3.f)                   /* 1.2  → y 0.8..1.6 */
#define STEP2_CY (STEP_HY * 5.f)                   /* 2.0  → y 1.6..2.4 */

#define SPHERE_RADIUS  0.35f
#define SPHERE_CY      (STEP_HY * 6.f + SPHERE_RADIUS)  /* 2.4 + 0.35 */
#define SPHERE_RINGS    12
#define SPHERE_SEGS     18

/* Object indices — used to assign meshes/colours in scene_init. */
enum {
    OBJ_FLOOR = 0,
    OBJ_STEP0,
    OBJ_STEP1,
    OBJ_STEP2,
    OBJ_SPHERE,
    N_OBJECTS,
};

/* §1.7 character ramp — Paul Bourke 92-char density ladder. */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.8 Bayer 4×4 dither — added to luma at paint time so neighbouring
 * cells with similar brightness pick different ramp characters. */
static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};
#define DITHER_AMP   0.10f

/* §1.9 ncurses pair IDs — 216 RGB cube + yellow HUD + cyan hint. */
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

/* OpenGL-style perspective. After m4_mul_v4, clip.w = -z_view; the
 * perspective divide x/w, y/w shrinks far things on screen. */
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
 * paint_cell — RGB → terminal pipeline used for every visible cell:
 *   1. Reinhard tone-map per channel
 *   2. gamma encode 1/2.2
 *   3. Bayer 4×4 dither on luma
 *   4. quantise to 6×6×6 RGB cube → pair id
 *   5. pick density glyph from 92-char Bourke ramp by Rec.709 luma
 *   6. A_BOLD on bright cells, A_DIM on dark
 *
 * Tone-map BEFORE quantisation — quantising linear HDR puts every
 * bright pixel into one cube cell; Reinhard opens the dynamic range.
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

/*
 * mesh_add_quad — append one flat quad (4 verts, 2 CCW tris) to a Mesh.
 *
 * Walk origin → +e1 → +e1+e2 → +e2 in CCW order viewed from outside
 * the surface (i.e. looking down the +nrm direction). Caller must
 * arrange e1 and e2 so e1 × e2 has the same sign as nrm — otherwise
 * the face is back-face-culled. tessellate_box verifies the sign per
 * face in the comments.
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

/*
 * tessellate_box — axis-aligned cuboid centred at the origin. 24
 * vertices (4 per face × 6 faces) so each face has its own flat
 * normal. Sharing 8 corner verts would average normals across faces
 * and round the cube into a smoothed lump — useless for SSAO where
 * flat faces and hard edges are the whole point.
 */
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

/* tessellate_quad — one rectangle (4 verts, 2 tris). Used for the floor. */
static Mesh tessellate_quad(Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm)
{
    Mesh m;
    m.verts = malloc(4 * sizeof(Vertex));
    m.tris  = malloc(2 * sizeof(Triangle));
    m.nvert = 0; m.ntri = 0;
    mesh_add_quad(&m, origin, e1, e2, nrm);
    return m;
}

/*
 * tessellate_sphere — UV sphere, smooth normals.
 *
 * Vertices on a (rings+1) × (segs+1) grid in spherical coords:
 *   theta ∈ [0, π] polar angle,  phi ∈ [0, 2π] azimuth.
 *   pos = R · (sin θ · cos φ, cos θ, sin θ · sin φ)
 *   nrm = pos / R   (unit normal = unit position vector)
 *
 * Smooth shading: each vertex's normal points outward radially. The
 * rasteriser barycentric-blends them per pixel, so the AO pass sees
 * a smooth normal field on the sphere — and the contact halo bends
 * smoothly around the sphere's curve.
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

static Vec3    g_pos    [GBUF_MAX_H][GBUF_MAX_W];
static Vec3    g_normal [GBUF_MAX_H][GBUF_MAX_W];
static Vec3    g_albedo [GBUF_MAX_H][GBUF_MAX_W];
static float   g_zbuf   [GBUF_MAX_H][GBUF_MAX_W];   /* NDC z (-1 near, +1 far) */
static float   g_z_view [GBUF_MAX_H][GBUF_MAX_W];   /* view-space z (linear)   */
static uint8_t g_valid  [GBUF_MAX_H][GBUF_MAX_W];

static void gbuffer_clear(int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            g_zbuf  [r][c] = 1.0f;
            g_z_view[r][c] = -CAM_FAR;
            g_valid [r][c] = 0;
        }
    }
}

/*
 * Möller signed-area barycentrics. Returns -1s for degenerate
 * triangles so the caller skips the pixel.
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

/*
 * rasterize_object — three-stage GPU-style pipeline:
 *   1. vertex transform: clip = mvp·pos, world = model·pos,
 *      world_nrm = norm_mat·nrm, view_z = (view·model·pos).z
 *   2. perspective divide → screen + back-face cull
 *   3. barycentric interpolate, z-test, write G-buffer
 *
 * SSAO needs view-space Z (linear) for its range check, NOT NDC-z
 * (non-linear after perspective). g_z_view is the parallel buffer
 * that records linear depth.
 */
static void rasterize_object(const Mesh *mesh, Vec3 albedo,
                             Mat4 mvp, Mat4 model, Mat4 modelview,
                             Mat4 norm_mat, int cols, int rows)
{
    for (int ti = 0; ti < mesh->ntri; ti++) {
        const Triangle *tri = &mesh->tris[ti];

        /* STAGE 1: vertex transform. */
        Vec4 clip[3];
        Vec3 wpos[3], wnrm[3];
        float vz[3];
        for (int vi = 0; vi < 3; vi++) {
            const Vertex *v = &mesh->verts[tri->v[vi]];
            clip[vi] = m4_mul_v4(mvp,  v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
            wpos[vi] = m4_pt   (model, v->pos);
            wnrm[vi] = v3_norm (m4_dir(norm_mat, v->normal));
            vz [vi]  = m4_pt   (modelview, v->pos).z;
        }

        if (clip[0].w < 0.001f && clip[1].w < 0.001f && clip[2].w < 0.001f)
            continue;

        /* STAGE 2: perspective divide → screen. */
        float sx[3], sy[3], sz[3];
        for (int vi = 0; vi < 3; vi++) {
            float w = clip[vi].w; if (fabsf(w) < 1e-6f) w = 1e-6f;
            sx[vi] = ( clip[vi].x / w + 1.f) * 0.5f * (float)cols;
            sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows;
            sz[vi] =   clip[vi].z / w;
        }

        /* Back-face cull: positive screen-space area = CCW = front. */
        float area = (sx[1] - sx[0]) * (sy[2] - sy[0])
                   - (sx[2] - sx[0]) * (sy[1] - sy[0]);
        if (area <= 0.f) continue;

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
                g_z_view[py][px] = b[0]*vz[0] + b[1]*vz[1] + b[2]*vz[2];
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
        rasterize_object(&meshes[oi], albedos[oi], mvp, models[oi], mv, nmat,
                         cols, rows);
    }
}

/* ── §7 ssao — per-pixel hemisphere sampling ─────────────────────────── */

/*
 * §7.1 ── kernel ───────────────────────────────────────────────────────
 *
 * VARIANTS sets of SAMPLES unit-ish vectors. Each pixel picks one
 * variant by (c & 1) | ((r & 1) << 1) — every 2×2 tile uses every
 * variant once. The 3×3 blur in §7.4 averages over a 9-cell
 * neighbourhood, so the final blurred AO is smooth.
 *
 * Vector LENGTHS are biased toward the surface: scale = 0.1 + 0.9·t²
 * with t = i/SAMPLES. Early samples sit very close (catching tight
 * crevices); late samples reach toward RADIUS. That's why nearby
 * corners darken more than distant flat surfaces.
 *
 * Generated with a deterministic LCG so the kernel is identical
 * every run, regardless of when scene_init / main are called.
 */
static Vec3 k_ssao[SSAO_KERNEL_VARIANTS][SSAO_SAMPLES];

static unsigned lcg_step(unsigned *s) { *s = *s * 1664525u + 1013904223u; return *s; }
static float    lcg_unit(unsigned *s) { return (lcg_step(s) >> 8) / (float)0x01000000; }

static void ssao_init_kernel(void)
{
    unsigned seed = 0xC0FFEE5Au;

    for (int v = 0; v < SSAO_KERNEL_VARIANTS; v++) {
        for (int i = 0; i < SSAO_SAMPLES; i++) {

            /* (1) rejection-sample a unit vector inside the unit sphere. */
            float dx, dy, dz, len2;
            do {
                dx = 2.f * lcg_unit(&seed) - 1.f;
                dy = 2.f * lcg_unit(&seed) - 1.f;
                dz = 2.f * lcg_unit(&seed) - 1.f;
                len2 = dx*dx + dy*dy + dz*dz;
            } while (len2 > 1.f || len2 < 1e-6f);

            float inv = 1.f / sqrtf(len2);
            Vec3  dir = v3(dx*inv, dy*inv, dz*inv);

            /* (2) bias length toward the surface (quadratic curve). */
            float t     = (float)i / (float)SSAO_SAMPLES;
            float scale = 0.1f + 0.9f * t * t;

            k_ssao[v][i] = v3_scale(dir, scale);
        }
    }
}

/* §7.2 ── AO buffers ────────────────────────────────────────────────── */

static float g_ao      [GBUF_MAX_H][GBUF_MAX_W];   /* raw, noisy   */
static float g_ao_blur [GBUF_MAX_H][GBUF_MAX_W];   /* 3×3 averaged */

/*
 * §7.3 ── ssao_pass — the per-pixel sample loop ──────────────────────
 *
 * Mirrors MENTAL MODEL → ALGORITHM IN STEPS exactly. Each step in
 * the prose has one labelled block of code below — read either side,
 * the lines should match 1:1.
 */
static void ssao_pass(Mat4 vp, float radius, int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) { g_ao[r][c] = 1.f; continue; }

            /* (a) read surface point and normal */
            Vec3 P = g_pos   [r][c];
            Vec3 N = g_normal[r][c];

            /* (b) pick this pixel's kernel variant from the 2×2 tile */
            int variant = (c & 1) | ((r & 1) << 1);

            /* (c) accumulate weighted occlusion over K samples */
            float occlude_w = 0.f;
            float total_w   = 0.f;

            for (int i = 0; i < SSAO_SAMPLES; i++) {

                /* hemisphere flip — sample must point ABOVE N */
                Vec3 dir = k_ssao[variant][i];
                if (v3_dot(dir, N) < 0.f) dir = v3_neg(dir);

                /* antenna tip in world space */
                Vec3 S = v3_add(P, v3_scale(dir, radius));

                /* project S to screen via VP */
                Vec4 clip = m4_mul_v4(vp, v4(S.x, S.y, S.z, 1.f));
                if (clip.w < 0.001f) continue;
                float sx = ( clip.x / clip.w + 1.f) * 0.5f * (float)cols;
                float sy = (-clip.y / clip.w + 1.f) * 0.5f * (float)rows;
                int   ix = (int)sx;
                int   iy = (int)sy;
                if (ix < 0 || ix >= cols || iy < 0 || iy >= rows) continue;
                if (!g_valid[iy][ix]) continue;

                /* range falloff — view-space z difference, linear */
                float dz   = fabsf(g_z_view[r][c] - g_z_view[iy][ix]);
                float attn = 1.f - dz / radius;
                if (attn <= 0.f) continue;
                total_w += attn;

                /* depth test — is some surface in front of S? */
                float ndc_z_S = clip.z / clip.w;
                if (g_zbuf[iy][ix] < ndc_z_S - SSAO_BIAS)
                    occlude_w += attn;
            }

            /* (d) AO factor; fully lit if no usable samples */
            float ao = (total_w > 1e-6f) ? (1.f - occlude_w / total_w) : 1.f;
            g_ao[r][c] = clamp01(ao);
        }
    }
}

/*
 * §7.4 ── ssao_blur — 3×3 box average ────────────────────────────────
 *
 * Smooths the per-pixel kernel-variant noise. Skips invalid cells in
 * both source and target (a "valid mask" blur), so silhouettes don't
 * bleed AO from off-pixel onto the object.
 *
 * Box blur is good enough at terminal res; a real engine uses a
 * bilateral blur (depth/normal-aware) to preserve edges between
 * adjacent objects.
 */
static void ssao_blur(int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) { g_ao_blur[r][c] = 1.f; continue; }

            float sum   = 0.f;
            int   count = 0;
            for (int dr = -1; dr <= 1; dr++) {
                for (int dc = -1; dc <= 1; dc++) {
                    int rr = r + dr, cc = c + dc;
                    if (rr < 0 || rr >= rows || cc < 0 || cc >= cols) continue;
                    if (!g_valid[rr][cc]) continue;
                    sum += g_ao[rr][cc];
                    count++;
                }
            }
            g_ao_blur[r][c] = (count > 0) ? (sum / (float)count) : 1.f;
        }
    }
}

/* ── §8 lightpass — Blinn-Phong with AO-scaled ambient ───────────────── *
 *
 *   ambient = AMBIENT_COL · albedo · AO        ← AO HERE
 *   diffuse = albedo · sun_col · max(0, N · L)
 *   spec    = sun_col · max(0, N · H)^SHININESS · SPEC_GAIN
 *   out     = clamp01(ambient + diffuse + spec)
 *
 * AO scales ONLY the ambient term — direct sun light is NOT darkened
 * in crevices. Multiplying direct light by AO would be physically
 * wrong (a corner can still receive direct sunlight) and would mute
 * the LIT_NO_AO ↔ LIT_WITH_AO toggle.                                   */
static Vec3 g_light[GBUF_MAX_H][GBUF_MAX_W];

static void render_lightpass(Vec3 cam_pos, bool use_ao, int cols, int rows)
{
    Vec3 sun_dir = v3(SUN_DIR[0],     SUN_DIR[1],     SUN_DIR[2]);
    Vec3 sun_col = v3(SUN_COL[0],     SUN_COL[1],     SUN_COL[2]);
    Vec3 ambient = v3(AMBIENT_COL[0], AMBIENT_COL[1], AMBIENT_COL[2]);
    Vec3 L       = v3_norm(v3_neg(sun_dir));

    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) { g_light[r][c] = v3(0,0,0); continue; }

            Vec3  P      = g_pos   [r][c];
            Vec3  N      = g_normal[r][c];
            Vec3  albedo = g_albedo[r][c];
            float ao     = use_ao ? g_ao_blur[r][c] : 1.f;

            Vec3 amb = v3(ambient.x * albedo.x * ao,
                          ambient.y * albedo.y * ao,
                          ambient.z * albedo.z * ao);

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

            Vec3 sum = v3_add(v3_add(amb, dif), sp);
            g_light[r][c] = v3(fminf(1.f, sum.x),
                               fminf(1.f, sum.y),
                               fminf(1.f, sum.z));
        }
    }
}

/* ── §9 scene ────────────────────────────────────────────────────────── */

typedef enum {
    MODE_AO_ONLY = 0,
    MODE_LIT_NO_AO,
    MODE_LIT_WITH_AO,
    MODE_COUNT,
} Mode;

static const char *k_mode_names[MODE_COUNT] = {
    "AO_ONLY", "LIT_NO_AO", "LIT_WITH_AO",
};

typedef struct {
    /* renderable objects — three parallel arrays indexed by OBJ_* */
    Mesh        meshes [N_OBJECTS];
    Vec3        albedos[N_OBJECTS];
    Mat4        models [N_OBJECTS];

    /* camera + projection (vp = proj·view, rebuilt each tick) */
    Mat4        view, proj, vp;
    Vec3        cam_pos;
    float       cam_dist;
    float       cam_yaw;

    /* SSAO + UI */
    float       ssao_radius;
    Mode        mode;
    bool        paused;

    int         scene_cols;
    int         scene_rows;
} Scene;

static void scene_rebuild_proj(Scene *s, int cols, int rows)
{
    /* Aspect from PIXEL counts — cell sizes 8×16 are non-square. */
    float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
    s->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

/*
 * scene_rebuild_view — eye orbits scene centre at fixed elevation.
 *   cam_pos = ( sin(yaw)·dist,  CAM_EYE_Y,  cos(yaw)·dist )
 *
 * Called every tick (orbit) AND on +/- zoom. Rebuilds vp = proj·view
 * so the SSAO and lighting passes use a consistent camera.
 */
static void scene_rebuild_view(Scene *s)
{
    float r = s->cam_dist;
    s->cam_pos = v3(sinf(s->cam_yaw) * r,
                    CAM_EYE_Y,
                    cosf(s->cam_yaw) * r);
    s->view = m4_lookat(s->cam_pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
    s->vp   = m4_mul(s->proj, s->view);
}

/*
 * scene_init — build floor + 3 stacked steps + sphere from scratch.
 *
 * Each step's centre y is chosen so its bottom touches the previous
 * step's top exactly (no float overlap); the sphere's centre y puts
 * its south pole on step 2's top face.
 */
static void scene_init(Scene *s, int total_cols, int total_rows)
{
    for (int i = 0; i < N_OBJECTS; i++) mesh_free(&s->meshes[i]);

    memset(s, 0, sizeof *s);
    s->scene_cols  = total_cols;
    s->scene_rows  = total_rows - HUD_ROWS;
    s->mode        = MODE_LIT_WITH_AO;
    s->ssao_radius = SSAO_RADIUS_DEF;
    s->cam_dist    = CAM_DIST;
    s->cam_yaw     = 0.f;

    /* OBJ_FLOOR — large quad at y = 0, normal up. */
    s->meshes [OBJ_FLOOR] = tessellate_quad(
        v3(-FLOOR_HALF_X, 0.f,  FLOOR_HALF_Z),
        v3( 2*FLOOR_HALF_X, 0.f,  0.f),
        v3( 0.f,            0.f, -2*FLOOR_HALF_Z),
        v3( 0.f, 1.f, 0.f));
    s->albedos[OBJ_FLOOR] = v3(0.42f, 0.46f, 0.50f);    /* slate */
    s->models [OBJ_FLOOR] = m4_identity();

    /* OBJ_STEP0..2 — same warm sandstone, decreasing footprints. */
    Vec3 stone = v3(0.78f, 0.62f, 0.42f);

    s->meshes [OBJ_STEP0] = tessellate_box(STEP0_HX, STEP_HY, STEP0_HX);
    s->albedos[OBJ_STEP0] = stone;
    s->models [OBJ_STEP0] = m4_translate(0.f, STEP0_CY, 0.f);

    s->meshes [OBJ_STEP1] = tessellate_box(STEP1_HX, STEP_HY, STEP1_HX);
    s->albedos[OBJ_STEP1] = stone;
    s->models [OBJ_STEP1] = m4_translate(0.f, STEP1_CY, 0.f);

    s->meshes [OBJ_STEP2] = tessellate_box(STEP2_HX, STEP_HY, STEP2_HX);
    s->albedos[OBJ_STEP2] = stone;
    s->models [OBJ_STEP2] = m4_translate(0.f, STEP2_CY, 0.f);

    /* OBJ_SPHERE — cream, sits on step 2's top. */
    s->meshes [OBJ_SPHERE] = tessellate_sphere(SPHERE_RADIUS,
                                               SPHERE_RINGS, SPHERE_SEGS);
    s->albedos[OBJ_SPHERE] = v3(0.90f, 0.84f, 0.74f);
    s->models [OBJ_SPHERE] = m4_translate(0.f, SPHERE_CY, 0.f);

    scene_rebuild_proj(s, total_cols, s->scene_rows);
    scene_rebuild_view(s);
}

/* The geometry is static — only the camera orbits. AO recomputes
 * every frame against the current view; if you pause the orbit the
 * AO map should be perfectly stable. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->cam_yaw += CAM_ORBIT_RAD_PER_SEC * dt;
    scene_rebuild_view(s);
}

/* mode_to_rgb — what to paint per pixel. The main loop runs only the
 * passes the active mode needs (see dispatch_passes), so g_light
 * already holds the right thing here. */
static Vec3 mode_to_rgb(Mode mode, int r, int c)
{
    if (!g_valid[r][c]) return v3(0, 0, 0);

    switch (mode) {
    case MODE_AO_ONLY: {
        float a = g_ao_blur[r][c];
        return v3(a, a, a);
    }
    case MODE_LIT_NO_AO:
    case MODE_LIT_WITH_AO:
        return g_light[r][c];
    default:
        return v3(0, 0, 0);
    }
}

/* ── §10 screen — render_scene + HUD ─────────────────────────────────── */

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
 * hud_draw — CLAUDE.md HUD spec layout (row 0 yellow+bold status,
 * bottom row cyan+bold hint) plus three educational rows in PAIR_HUD
 * (no bold) for: pipeline cost, mode explanation, what to look for.
 */
static void hud_draw(const Scene *s, double fps)
{
    int hr   = s->scene_rows;
    int cols = s->scene_cols;

    int total_tris = 0;
    for (int i = 0; i < N_OBJECTS; i++) total_tris += s->meshes[i].ntri;

    /* Row 0: title + status. */
    char status[140];
    snprintf(status, sizeof status,
             " %5.1f fps  mode:%s  K=%d  R=%.2f  zoom:%.1f  tris:%d  %s ",
             fps, k_mode_names[s->mode],
             SSAO_SAMPLES, (double)s->ssao_radius,
             (double)s->cam_dist, total_tris,
             s->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > cols) slen = cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - slen, "%s", status);
    mvprintw(0, 0, " SSAO · STEPPED PYRAMID ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Educational rows. */
    attron(COLOR_PAIR(PAIR_HUD));

    int total_dirs = SSAO_SAMPLES * SSAO_KERNEL_VARIANTS;
    mvprintw(hr + 0, 1,
             "samples=%d (across %d variants = %d distinct dirs)   blur=3x3   bias=%.4f",
             SSAO_SAMPLES, SSAO_KERNEL_VARIANTS, total_dirs, (double)SSAO_BIAS);

    const char *explain = "";
    switch (s->mode) {
    case MODE_AO_ONLY:
        explain = "AO_ONLY: grayscale crevice map. White=open. Dark=corners (4 features).";
        break;
    case MODE_LIT_NO_AO:
        explain = "LIT_NO_AO: full Blinn-Phong, no AO. Sphere looks pasted on the step.";
        break;
    case MODE_LIT_WITH_AO:
        explain = "LIT_WITH_AO: ambient * AO + direct. Pyramid + sphere settle into floor.";
        break;
    default: break;
    }
    mvprintw(hr + 1, 1, "%s", explain);

    mvprintw(hr + 2, 1,
             "Look for: floor halo, two horizontal step bands, "
             "circular halo under the sphere");

    attroff(COLOR_PAIR(PAIR_HUD));

    /* Cyan hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(hr + HUD_ROWS - 1, 0,
             " q:quit  spc:pause  a:mode  [/]:radius  +/-:zoom  r:reset ");
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
    case 'a': case 'A': s->mode = (Mode)((s->mode + 1) % MODE_COUNT); break;
    case '[':
        s->ssao_radius -= SSAO_RADIUS_STEP;
        if (s->ssao_radius < SSAO_RADIUS_MIN) s->ssao_radius = SSAO_RADIUS_MIN;
        break;
    case ']':
        s->ssao_radius += SSAO_RADIUS_STEP;
        if (s->ssao_radius > SSAO_RADIUS_MAX) s->ssao_radius = SSAO_RADIUS_MAX;
        break;
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

/*
 * dispatch_passes — run only the passes the active mode actually needs.
 *   AO_ONLY      → gbuffer + ssao + blur
 *   LIT_NO_AO    → gbuffer + lightpass(use_ao = false)
 *   LIT_WITH_AO  → gbuffer + ssao + blur + lightpass(use_ao = true)
 *
 * Skipping unused passes makes the toggle keys feel snappy and proves
 * the teaching point: SSAO and lighting are independent passes that
 * can each be enabled or disabled at will.
 */
static void dispatch_passes(Scene *s)
{
    render_gbuffer(s->meshes, s->albedos, s->models, N_OBJECTS,
                   s->view, s->proj, s->scene_cols, s->scene_rows);

    if (s->mode != MODE_LIT_NO_AO) {
        ssao_pass(s->vp, s->ssao_radius, s->scene_cols, s->scene_rows);
        ssao_blur(s->scene_cols, s->scene_rows);
    }

    if (s->mode != MODE_AO_ONLY) {
        bool use_ao = (s->mode == MODE_LIT_WITH_AO);
        render_lightpass(s->cam_pos, use_ao, s->scene_cols, s->scene_rows);
    }
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    ssao_init_kernel();

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
        dispatch_passes(s);
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
