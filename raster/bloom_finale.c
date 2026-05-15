/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bloom_finale.c — deferred + SSAO + bloom capstone
 *
 * DEMO: A dark slate floor with two warm sandstone cubes on it, and
 *       a small glowing orb floating between them. The orb is the
 *       only emissive object — its surface "outputs" more light than
 *       a normal albedo can reflect — and that excess HDR brightness
 *       is what bloom feeds on. Around the orb you see a soft warm
 *       halo bleeding several cells past its silhouette; that halo
 *       is the bloom contribution. The cube corners and floor halos
 *       are darkened by SSAO. Direct sunlight from the upper-left
 *       picks out the cube faces and the orb's own surface.
 *
 *       Press 'b' to toggle bloom — the halo around the orb appears
 *       and disappears. Without bloom the orb is just a bright disc
 *       with a hard silhouette; with bloom it reads as a real light
 *       source. Press 'a' to toggle SSAO — the dark contact halos
 *       under the cubes vanish and the scene flattens.
 *
 *       Together: this file runs every technique developed in the
 *       earlier files of the folder back-to-back in one frame.
 *
 * Study alongside:
 *   raster/deferred_rendering_pipeline.c — the deferred / G-buffer base
 *   raster/ssao_pipeline.c               — the SSAO sampling + blur
 *   raster/shadow_mapping.c              — same lookAt + cull conventions
 *
 * Section map:
 *   §1  config     — frame, view, scene, SSAO, bloom, lighting, ramp
 *   §2  clock      — monotonic timer + sleep
 *   §3  math       — V3, V4, Mat4 + perspective / lookat
 *   §4  paint      — 216-pair RGB cube + Bourke ramp + paint_cell
 *   §5  mesh       — Vertex / Triangle types + box / quad / sphere
 *   §6  gbuffer    — geometry pass (pos, normal, albedo, emissive, depths)
 *   §7  ssao       — kernel + per-pixel sample loop + 3×3 blur
 *   §8  lightpass  — Blinn-Phong + emissive (HDR output)
 *   §9  bloom      — extract → separable Gaussian (H + V) → composite
 *   §10 scene      — Scene struct, init / view / tick
 *   §11 screen     — render_scene + HUD (CLAUDE.md spec)
 *   §12 app        — signals, resize, fixed-step main loop
 *
 * Keys:
 *   b / B     toggle bloom on/off
 *   a / A     toggle SSAO  on/off
 *   + / =     zoom in
 *   - / _     zoom out
 *   space     pause / resume camera orbit
 *   r / R     reset
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/bloom_finale.c -o bloom -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three image-space techniques composed into one
 *                  frame. (1) Deferred shading: rasterise once into a
 *                  G-buffer of per-pixel surface properties, then run
 *                  Blinn-Phong over the G-buffer in a separate pass.
 *                  (2) SSAO: per-pixel hemisphere sampling against the
 *                  same G-buffer, modulating only the ambient term.
 *                  (3) Bloom: copy the BRIGHT pixels from the lit
 *                  buffer into a separate buffer, blur it with a
 *                  separable Gaussian, then add the blurred result
 *                  back on top. Bright sources gain a soft halo —
 *                  the visual signature of a real camera or eye
 *                  responding to over-exposed light.
 *
 * Data-structure : G-buffer parallels (pos, normal, albedo, emissive,
 *                  zbuf, z_view, valid) + AO buffers (raw + blurred)
 *                  + bloom buffers (bright + blurred). All static at
 *                  GBUF_MAX_W × GBUF_MAX_H. Lightpass output g_light
 *                  is HDR (channels can exceed 1.0 thanks to emissive
 *                  surfaces) — bloom WANTS the HDR; if lightpass
 *                  clamped to [0,1] there'd be no excess to bleed.
 *
 * Rendering      : Per frame:
 *                    1. render_gbuffer  — pos / normal / albedo /
 *                                         emissive / depth
 *                    2. ssao_pass + ssao_blur (if enabled)
 *                    3. render_lightpass — HDR Blinn-Phong + emissive
 *                    4. bloom_extract → bloom_blur_h → bloom_blur_v
 *                       → bloom_composite (if enabled)
 *                    5. render_scene → paint_cell (Reinhard tone-map
 *                       finally collapses HDR to terminal-friendly
 *                       brightness)
 *
 * Performance    : SSAO ≈ pixels × K samples; bloom ≈ pixels × kernel
 *                  taps × 2 passes. Both trivial at terminal resolution.
 *
 * References     : Saito & Takahashi, "Comprehensible Rendering of
 *                    3-D Shapes," SIGGRAPH '90 (G-buffer concept).
 *                  Mittring, "Finding Next Gen — CryEngine 2,"
 *                    SIGGRAPH '07 course notes (SSAO).
 *                  James & O'Rorke, "Real-Time Glow," GPU Gems (2004).
 *                  LearnOpenGL, "Bloom" tutorial:
 *                    https://learnopengl.com/Advanced-Lighting/Bloom
 *                  Reinhard et al., "Photographic Tone Reproduction
 *                    for Digital Images," SIGGRAPH '02.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Render in HDR (don't clamp to white), then pull just the OVER-WHITE
 * pixels into a separate image, blur them so they spread, and add
 * them back. That extra glow only appears around things that were
 * already too bright for the display — exactly what your eye sees
 * staring at a candle in a dark room.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The lit buffer can hold values like (3.0, 1.6, 0.7) for an
 * emissive surface even though the screen can only show up to
 * (1.0, 1.0, 1.0). That extra "1.0 + 0.6 + 1.0 + 0.7" of light
 * energy isn't lost — it gets siphoned into a side buffer, blurred,
 * and added on top. After tone-mapping, the original surface still
 * peaks at white, but its blurred ghost now bleeds into the
 * neighbouring pixels.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │   HDR lit:        bright extract:            │
 *      │   ░░░░░░░         ░░░░░░░                    │
 *      │   ░░███░░    →    ░░███░░     blur →   ░██░░ │
 *      │   ░██▒██░         ░██▒██░              ████░ │
 *      │   ░░███░░         ░░███░░              ░██░░ │
 *      │                                              │
 *      │   composite = lit + blurred_extract          │
 *      │   tone-map → terminal cells                  │
 *      └──────────────────────────────────────────────┘
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 *  1. G-BUFFER PASS — write pos / normal / albedo / EMISSIVE / depth.
 *     Emissive is the new bit: most surfaces have (0, 0, 0); the orb
 *     has a HDR colour like (3, 1.6, 0.7).
 *
 *  2. SSAO PASS (optional) — same as the SSAO file: hemisphere
 *     samples against the G-buffer, output AO ∈ [0, 1] per pixel.
 *
 *  3. LIGHT PASS — for each valid pixel:
 *       lit = AMBIENT · albedo · AO         (modulated by SSAO)
 *           + albedo · sun · max(0, N·L)
 *           + sun · max(0, N·H)^SHININESS · SPEC_GAIN
 *           + EMISSIVE                       (pure energy out)
 *     NOT clamped to 1.0 — emissive can push channels past white.
 *
 *  4. BLOOM (optional) — four cheap passes:
 *       a. EXTRACT     g_bloom[r][c] = lit if luma > THRESHOLD else 0
 *       b. BLUR_H      horizontal Gaussian into g_bloom_temp
 *       c. BLUR_V      vertical   Gaussian back into g_bloom
 *       d. COMPOSITE   g_light += BLOOM_INTENSITY · g_bloom
 *
 *  5. PAINT — Reinhard tone-map collapses HDR to [0,1] inside
 *     paint_cell, then 6×6×6 cube quantise + Bourke ramp.
 *
 * KEY FORMULAS
 * ────────────
 *   Emissive:       lit += emissive   (no diffuse/spec, just emit)
 *   Bright extract: bright = lit · step(THRESHOLD, luma(lit))
 *   Gaussian:       g(x) = exp(-x²/(2σ²)) / (σ√(2π))
 *   Sep blur (H):   out[r][c] = Σ kernel[k] · in[r][c+k-RADIUS]
 *   Sep blur (V):   out[r][c] = Σ kernel[k] · in[r+k-RADIUS][c]
 *   Composite:      lit += BLOOM_INTENSITY · bloom_blurred
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Clamping in lightpass — the moment you write `min(1, sum)` the
 *     bloom pass has nothing to extract. Lightpass MUST write HDR.
 *
 *   • Bloom threshold — too low and dimly-lit surfaces start
 *     glowing (everything looks foggy); too high and only the
 *     emissive core itself triggers, halo barely visible.
 *
 *   • Separable validity — Gaussian is separable: a 2-D blur equals
 *     H pass then V pass with the SAME 1-D kernel. This is what
 *     makes the cost O(taps) instead of O(taps²).
 *
 *   • Boundary handling — at the edges of the screen the kernel
 *     reaches off-buffer. We clamp the read coordinate to [0, W-1]
 *     so the edge values get duplicated outward (no halo gets
 *     cut off at the screen boundary).
 *
 *   • SSAO + bloom interaction — SSAO modulates only the AMBIENT
 *     term; emissive contributes directly. So an emissive surface
 *     in a deep corner stays bright (correct — it emits its own
 *     light, doesn't reflect ambient).
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Toggle 'b' off: orb becomes a hard-edged bright disc, no
 *     halo. Toggle on: soft warm bleed extends 3-4 cells past the
 *     silhouette. The diff is exactly the bloom contribution.
 *
 *   • Toggle 'a' off: cube corners go uniformly bright; toggle on
 *     and the contact halos under each cube + the corner where
 *     they sit on the floor visibly darken.
 *
 *   • The HUD's "bloom" / "ssao" tags show whether each pass ran.
 *
 *   • Pause (space) and toggle 'b' repeatedly — only the pixels
 *     near the orb change, the rest of the scene is pixel-identical.
 *     That's how you know bloom only affects bright neighbourhoods.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read raster/deferred_rendering_pipeline.c first if
 *      you don't yet know the G-buffer pattern, and ssao_pipeline.c
 *      if you don't yet know the AO pass; this file COMPOSES both
 *      of those plus a new bloom stage.
 *   2. §1 config — every constant has a unit-bearing comment. The
 *      bloom block (THRESHOLD, INTENSITY, RADIUS, σ) is what tunes
 *      the halo's strength and width.
 *   3. §9 bloom — THE NEW HEART of this file. Read AFTER tutorials
 *      T1-T6. The four functions (extract / blur_h / blur_v /
 *      composite) line up 1:1 with the algorithm steps.
 *   4. §6 gbuffer + §7 ssao + §8 lightpass — copies of the earlier
 *      files. Skim if you've read those. Notice §8 outputs HDR
 *      (no clamp before bloom).
 *   5. §3 math + §4 paint + §10-§12 — infrastructure; skip on
 *      first read. paint_cell does the Reinhard tone-map at the
 *      very end — that's the line that finally collapses HDR into
 *      terminal-friendly brightness.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   g_light[r][c]       HDR lit colour, components may exceed 1.0.
 *   g_bloom[r][c]       buffer that holds the bright-extract,
 *                       then the blurred halo. Reused across
 *                       passes.
 *   g_bloom_temp[r][c]  scratch buffer for separable-blur
 *                       intermediate (after H pass, before V).
 *   luma                Rec.709 luminance: 0.2126 R + 0.7152 G
 *                       + 0.0722 B.
 *   THRESHOLD           luminance above which a pixel counts as
 *                       "bright enough to bloom."
 *   RADIUS / σ          Gaussian kernel radius (in pixels) and
 *                       standard deviation. Controls halo size.
 *   INTENSITY           composite scale factor on the blurred
 *                       bloom buffer.
 *
 * Background you need
 * ───────────────────
 *   - Deferred shading (deferred_rendering_pipeline.c) — the
 *     G-buffer pattern.
 *   - SSAO (ssao_pipeline.c) — hemisphere sampling + AO blur.
 *   - Convolution: 1-D and 2-D image kernels.
 *   - HDR vs. LDR: HDR keeps brightness > 1.0 around so it can
 *     be processed; LDR clamps at 1.0 (white).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Lens-flare or anamorphic-streak bloom variants. We do
 *     plain isotropic Gaussian bloom.
 *   - Multi-scale bloom (mip-chain pyramid). Real engines blur at
 *     several resolutions and add them; we do one scale, which is
 *     plenty for the orb-sized bloom in this scene.
 *   - Filmic tone-mapping (ACES, Hable). We use Reinhard
 *     (x / (1 + x)), the simplest curve that does the job.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Eight tutorials that build bloom + tone-mapping from first
 * principles.
 *
 *   T1  Why bloom even exists — over-exposed light bleeding
 *   T2  HDR vs. LDR — the absolute prerequisite
 *   T3  Emissive surfaces — the source of the excess light
 *   T4  Bright extract — luminance threshold + smooth knee
 *   T5  The Gaussian kernel — what σ and RADIUS mean
 *   T6  Separable blur — O(taps) instead of O(taps²)
 *   T7  Composite + tone-map — the final HDR-to-LDR collapse
 *   T8  Why DITHER_AMP was tuned to 0.04 (phase-2 finding)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHY BLOOM EVEN EXISTS — OVER-EXPOSED LIGHT BLEEDING
 * ───────────────────────────────────────────────────────
 * Look at a candle in a dark room. The flame doesn't end at a
 * crisp silhouette — there's a soft halo bleeding outward,
 * fading over a few millimetres of dark surroundings. That halo
 * is real. Two physical sources:
 *
 *   - The eye (or camera lens) imperfectly focuses bright light;
 *     a fraction scatters across nearby retinal cells / sensor
 *     photosites.
 *   - Tiny dust on the lens, micro-scratches in your eye's
 *     vitreous humour, etc., diffract the brightest light over
 *     a wider area than its source.
 *
 * The brain reads "bleed around brightness" as "this thing is
 * very bright." Without the bleed, a digital image of a candle
 * looks like a sticker. WITH bloom, even a 256-colour terminal
 * cell can read as a real light source.
 *
 * Bloom in rendering is ONE technique that fakes both effects in
 * one pass: blur the bright parts of the image and add the blur
 * back on top.
 *
 * T2  HDR VS. LDR — THE ABSOLUTE PREREQUISITE
 * ───────────────────────────────────────────
 * "Bright" is meaningless without a notion of brightness ABOVE
 * full white. A standard 8-bit-per-channel image saturates at
 * (1.0, 1.0, 1.0). That's LOW DYNAMIC RANGE.
 *
 * In LDR, an emissive flame and a perfectly white piece of paper
 * read as the same pixel value: (1, 1, 1). They look identical.
 * No bloom can fix that — the over-bright information was
 * destroyed when the renderer clamped to 1.0.
 *
 * In HIGH DYNAMIC RANGE, lit values are stored as floats with no
 * upper limit. The flame might be (3.0, 1.6, 0.7); the paper
 * (0.95, 0.95, 0.95). The bloom pass can now distinguish them by
 * looking at luma > 1.0.
 *
 * The lightpass in this file emits HDR — no clamp:
 *
 *     lit = ambient·albedo·AO + diffuse + specular + emissive
 *     g_light[r][c] = lit          (NOT clamp01(lit))
 *
 * Tone mapping (T7) collapses HDR to LDR ONLY at the very last
 * step, after bloom has had its chance to operate.
 *
 * T3  EMISSIVE SURFACES — THE SOURCE OF THE EXCESS LIGHT
 * ──────────────────────────────────────────────────────
 * The G-buffer in this file has an extra channel: g_emissive.
 * For most surfaces it's (0, 0, 0). For the orb it's a HDR
 * colour like (3.0, 1.6, 0.7) — meaning "this surface emits
 * light at 3 units of red, 1.6 of green, 0.7 of blue."
 *
 * "Emit" here means: contribute to the lit buffer WITHOUT
 * needing diffuse / specular reflection. Emissive surfaces
 * appear bright even with the sun off, even in a deep corner
 * (SSAO doesn't darken them — they make their OWN light, they
 * don't reflect ambient).
 *
 * In the lightpass:
 *
 *     lit = ambient·albedo·AO              ← reflected ambient
 *         + albedo · sun · max(0, N·L)     ← diffuse
 *         + spec_term                      ← specular
 *         + emissive                       ← direct emission
 *
 * The orb's emissive (3, 1.6, 0.7) puts its lit value well above
 * 1.0 in red. That's the over-bright signal bloom will detect.
 *
 * Without an emissive channel you can still bloom, but only the
 * brightest specular highlights would trigger — a much subtler
 * effect.
 *
 * T4  BRIGHT EXTRACT — LUMINANCE THRESHOLD + SMOOTH KNEE
 * ──────────────────────────────────────────────────────
 * The first bloom step copies only the BRIGHT pixels of g_light
 * into g_bloom, leaving everything else at zero:
 *
 *     for each pixel:
 *       Y = luma(g_light[r][c])
 *       w = smoothstep(THRESHOLD - KNEE, THRESHOLD + KNEE, Y)
 *       g_bloom[r][c] = g_light[r][c] · w
 *
 * Why luminance instead of channel-wise brightness? Because the
 * eye perceives green as much brighter than blue at the same
 * value. Rec.709 weights:
 *
 *     Y = 0.2126·R + 0.7152·G + 0.0722·B
 *
 * Why smoothstep instead of a hard threshold? Same reason as
 * neon_edges T7: a hard cut-off makes single-pixel aliasing.
 * smoothstep ramps over a small KNEE-width band so dim halos
 * blend in.
 *
 * After this step, g_bloom looks like the original lit buffer
 * but with all the dim regions blacked out — a sparse map of
 * bright spots.
 *
 * T5  THE GAUSSIAN KERNEL — WHAT σ AND RADIUS MEAN
 * ────────────────────────────────────────────────
 * To "spread" the bright spots, convolve g_bloom with a
 * Gaussian kernel. The 1-D Gaussian is:
 *
 *     g(x) = exp(-x² / (2σ²)) / (σ · √(2π))
 *
 * Two parameters:
 *
 *   σ (sigma)    standard deviation, controls the WIDTH of the
 *                bell curve. Bigger σ = wider, softer halo.
 *
 *   RADIUS       how many pixels of the kernel to actually use.
 *                The Gaussian extends to ±∞, but past about ±3σ
 *                the values are < 1% of the peak; we truncate.
 *                RULE: RADIUS ≥ 3σ.
 *
 * For this file: σ ≈ 3.0, RADIUS = 6, so the kernel has 13 taps
 * (from -6 to +6 inclusive). The taps are precomputed once in
 * §9 bloom_kernel_init and stored in a static array.
 *
 * The kernel is normalised so its taps sum to 1.0. Convolving by
 * a normalised kernel preserves the average brightness — we move
 * energy around, we don't add or remove it.
 *
 * T6  SEPARABLE BLUR — O(TAPS) INSTEAD OF O(TAPS²)
 * ────────────────────────────────────────────────
 * A naïve 2-D blur with a (2R+1)² kernel costs (2R+1)² reads per
 * pixel. For our R = 6 that's 169 reads per pixel — wasteful.
 *
 * Crucial property of Gaussians: they are SEPARABLE.
 *
 *     G_2D(x, y) = G_1D(x) · G_1D(y)
 *
 * That means a 2-D Gaussian blur EQUALS:
 *
 *     blur_horizontal(image, kernel_1D)   →  intermediate
 *     blur_vertical (intermediate, kernel_1D)  →  output
 *
 * Two 1-D passes, each with (2R+1) reads per pixel. For R = 6
 * that's 13 + 13 = 26 reads — 6× fewer than the naïve
 * implementation, and it gets MORE valuable as R grows.
 *
 * §9 bloom_blur_h and bloom_blur_v are the two 1-D passes. They
 * use the same precomputed kernel; they differ only in whether
 * they walk pixels horizontally or vertically.
 *
 * Boundary handling: when the kernel reaches off-screen, we
 * CLAMP the read coordinate to [0, W-1]. This is "edge
 * extension" — pretends the pixel at the boundary repeats. The
 * alternative ("zero outside the image") would create a dark
 * fringe at the screen edges where bright objects approached
 * the boundary. Edge extension hides that.
 *
 * T7  COMPOSITE + TONE-MAP — THE FINAL HDR-TO-LDR COLLAPSE
 * ────────────────────────────────────────────────────────
 * After both blur passes, g_bloom holds a soft halo around every
 * bright source. Composite is just adding it back into g_light:
 *
 *     g_light[r][c] += BLOOM_INTENSITY · g_bloom[r][c]
 *
 * BLOOM_INTENSITY < 1 keeps the halo subtle; > 1 makes everything
 * super-shiny.
 *
 * g_light is STILL HDR — adding the bloom can only make it
 * brighter. So we cannot send g_light directly to a 256-colour
 * terminal. The final step is TONE MAPPING — collapsing HDR
 * floats into LDR [0, 1] in a way that preserves visual
 * relationships:
 *
 *     ldr = hdr / (1 + hdr)        ← Reinhard, applied per channel
 *
 * Properties:
 *   - hdr = 0     → ldr = 0
 *   - hdr = 1     → ldr = 0.5
 *   - hdr = ∞     → ldr = 1
 *
 * Continuous, monotone, never clips. The dim parts of the scene
 * stay roughly linear (small hdr → ldr ≈ hdr); the very bright
 * parts get smoothly compressed toward 1.0 instead of clamped.
 *
 * Reinhard happens per channel inside paint_cell, JUST before the
 * 6×6×6 cube quantise + Bourke ramp. That's the FIRST and ONLY
 * place HDR gets crushed. By that point bloom has already done
 * its work in HDR space.
 *
 * T8  WHY DITHER_AMP WAS TUNED TO 0.04 (PHASE-2 FINDING)
 * ──────────────────────────────────────────────────────
 * The Bourke ramp gives ~92 distinct brightness glyphs. The
 * 6×6×6 RGB cube gives 6 levels per channel. Together that's
 * still discrete — you'll see banding on smooth dim ramps like
 * the floor unless you DITHER.
 *
 * Bayer dithering adds a deterministic 4×4 noise pattern to the
 * brightness BEFORE quantising:
 *
 *     y_dithered = y + (BAYER[r%4][c%4] - 0.5) · DITHER_AMP
 *
 * DITHER_AMP controls how strong the dither is. Phase-2
 * iteration on this file:
 *
 *   0.10  → "wallpaper bands" — the 4×4 Bayer pattern is too
 *            visible against the dim floor.
 *
 *   0.04  → enough dither to break the banding on the floor
 *            without making the Bayer texture itself visible.
 *
 * The constant lives in §1 config with that history noted. Any
 * scene where the bloom-darkened ambient occupies a large area
 * of the screen will benefit from low DITHER_AMP for the same
 * reason; scenes with brighter overall illumination tolerate
 * higher dither without visible patterning.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI */
enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500,
  HUD_ROWS = 5,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define DT_CAP_NS (100 * NS_PER_MS)

/* §1.2 G-buffer dimensions (static; sized for a large terminal). */
#define GBUF_MAX_W 300
#define GBUF_MAX_H 80

/* §1.3 view geometry — eye orbits the scene at fixed elevation. */
#define CAM_FOV (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 50.0f

#define CAM_DIST 5.5f
#define CAM_DIST_MIN 3.0f
#define CAM_DIST_MAX 10.0f
#define CAM_ZOOM_STEP 0.4f
#define CAM_EYE_Y 2.4f
#define CAM_LOOK_Y 0.5f
#define CAM_ORBIT_RAD_PER_SEC 0.18f

#define CELL_W 8
#define CELL_H 16

/* §1.4 lighting — single warm sun + cool ambient. */
static const float SUN_DIR[3] = {-0.55f, -0.65f, 0.30f};
static const float SUN_COL[3] = {0.95f, 0.85f, 0.65f};
static const float AMBIENT_COL[3] = {0.18f, 0.20f, 0.25f};
#define SHININESS 24.0f
#define SPEC_GAIN 0.30f

/* §1.5 SSAO parameters
 *
 * Same kernel design as ssao_pipeline.c: 12 unit-ish hemisphere
 * samples × 4 variants picked per-pixel by (c&1 | (r&1)<<1), then a
 * 3×3 box blur to denoise. Modulates the ambient term only. */
#define SSAO_SAMPLES 12
#define SSAO_KERNEL_VARIANTS 4
#define SSAO_RADIUS 0.45f
#define SSAO_BIAS 0.0008f

/* §1.6 bloom parameters
 *
 * THRESHOLD — luma above this counts as "over-bright" and gets
 *   extracted into the bloom buffer. Set just above 1.0 so only
 *   HDR (emissive) pixels trigger; ordinary lit surfaces stay calm.
 * INTENSITY — multiplier on the blurred bright buffer when adding
 *   it back into g_light. 1.0 is a strong glow.
 * KERNEL    — a 1-D Gaussian; applied separably (H then V).
 *   13-tap with σ ≈ 3.0 spreads the halo ~6 cells out from the
 *   bright source. The previous 7-tap (σ ≈ 1.5, ~3 cells) was too
 *   tight to read as a glow at terminal resolution — the halo
 *   barely cleared the orb's silhouette. Wider σ + more taps gives
 *   an actual halo with a visible falloff. */
#define BLOOM_THRESHOLD 1.00f
#define BLOOM_INTENSITY 1.00f
#define BLOOM_RADIUS 6                    /* taps each side  */
#define BLOOM_TAPS (2 * BLOOM_RADIUS + 1) /* = 13            */
static const float BLOOM_KERNEL[BLOOM_TAPS] = {
    /* Gaussian weights for σ = 3.0, normalised to sum 1.0. */
    0.0185f, 0.0342f, 0.0563f, 0.0831f, 0.1097f, 0.1296f, 0.1370f,
    0.1296f, 0.1097f, 0.0831f, 0.0563f, 0.0342f, 0.0185f};

/* §1.7 scene geometry — floor + two cubes + one glowing orb.
 *
 *                     ●  ← orb (emissive, the bloom showcase)
 *                  ┌──┐         ┌──┐
 *                  │AA│         │BB│   cubes (sandstone)
 *                  └──┘         └──┘
 *               ───────────────────  floor (slate)                 */
#define FLOOR_HALF_X 3.0f
#define FLOOR_HALF_Z 3.0f

#define CUBE_HALF 0.45f
#define CUBE_A_CX -1.10f
#define CUBE_A_CY (CUBE_HALF)
#define CUBE_A_CZ 0.40f
#define CUBE_B_CX 1.10f
#define CUBE_B_CY (CUBE_HALF)
#define CUBE_B_CZ -0.30f

#define ORB_RADIUS 0.40f
#define ORB_RINGS 12
#define ORB_SEGS 18
#define ORB_CX 0.0f
#define ORB_CY 0.95f
#define ORB_CZ 0.05f
/* HDR emissive — channels exceed 1.0 so the orb glows past white
 * and bloom has something to extract. (1.0 wouldn't trigger.) */
static const float ORB_EMISSIVE[3] = {3.20f, 1.80f, 0.80f};

enum {
  OBJ_FLOOR = 0,
  OBJ_CUBE_A,
  OBJ_CUBE_B,
  OBJ_ORB,
  N_OBJECTS,
};

/* §1.8 character ramp — 18-char dense-low-end ladder.
 *
 * Replaces the 92-char Bourke ramp because Bourke's bottom ~25 chars
 * (` `, '`', '.', ''', '`', '-', '_', ',', ':', etc) are visually
 * thin/sparse on terminal fonts — at the luma values bloom halos
 * actually produce (~0.05-0.15), they read as scattered DOTS rather
 * than a smooth glow.
 *
 * This ramp skips the sparse single-pixel glyphs entirely. The lowest
 * non-space character is `:` (two visible marks), so even very dim
 * halo cells render as a clearly-visible mark instead of a phantom
 * dot. Top end keeps the dense '%@#$' for over-bright cells.
 *
 * Trade: less fine gradation (18 levels vs 92) — but the 6×6×6 cube
 * already limits us to ~6 effective brightness levels per channel,
 * so 18 ramp entries is plenty.                                        */
static const char k_bourke[] = " .:;~-+*coxOQ0%&@#$";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.9 Bayer 4×4 dither. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.10f

/* §1.10 ncurses pair IDs — 216 cube + yellow HUD + cyan hint. */
#define PAIR_CUBE_BASE 1
#define PAIR_HUD 217
#define PAIR_HINT 218

/* ── §2 clock ────────────────────────────────────────────────────────── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {.tv_sec = (time_t)(ns / NS_PER_SEC),
                       .tv_nsec = (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ── §3 math (V3, V4, Mat4) ──────────────────────────────────────────── */

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
static inline Vec3 v3_neg(Vec3 a) { return v3(-a.x, -a.y, -a.z); }
static inline float v3_dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3_len(Vec3 a) { return sqrtf(v3_dot(a, a)); }
static inline Vec3 v3_norm(Vec3 a) {
  float l = v3_len(a);
  return l > 1e-7f ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}
static inline Vec3 v3_bary(Vec3 p0, Vec3 p1, Vec3 p2, float b0, float b1,
                           float b2) {
  return v3(b0 * p0.x + b1 * p1.x + b2 * p2.x,
            b0 * p0.y + b1 * p1.y + b2 * p2.y,
            b0 * p0.z + b1 * p1.z + b2 * p2.z);
}
static inline float v3_luma(Vec3 c) {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

static inline Mat4 m4_identity(void) {
  Mat4 m = {{{0}}};
  m.m[0][0] = m.m[1][1] = m.m[2][2] = m.m[3][3] = 1.f;
  return m;
}

static inline Vec4 m4_mul_v4(Mat4 m, Vec4 v) {
  return v4(
      m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z + m.m[0][3] * v.w,
      m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z + m.m[1][3] * v.w,
      m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z + m.m[2][3] * v.w,
      m.m[3][0] * v.x + m.m[3][1] * v.y + m.m[3][2] * v.z + m.m[3][3] * v.w);
}

static inline Mat4 m4_mul(Mat4 a, Mat4 b) {
  Mat4 r = {{{0}}};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        r.m[i][j] += a.m[i][k] * b.m[k][j];
  return r;
}

static inline Vec3 m4_pt(Mat4 m, Vec3 p) {
  Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
  return v3(r.x, r.y, r.z);
}

static inline Vec3 m4_dir(Mat4 m, Vec3 d) {
  Vec4 r = m4_mul_v4(m, v4(d.x, d.y, d.z, 0.f));
  return v3(r.x, r.y, r.z);
}

static Mat4 m4_perspective(float fovy, float aspect, float near, float far) {
  Mat4 m = {{{0}}};
  float f = 1.f / tanf(fovy * 0.5f);
  m.m[0][0] = f / aspect;
  m.m[1][1] = f;
  m.m[2][2] = (far + near) / (near - far);
  m.m[2][3] = (2.f * far * near) / (near - far);
  m.m[3][2] = -1.f;
  return m;
}

/* m4_lookat — standard glm convention (cross(forward, up)). */
static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up) {
  Vec3 f = v3_norm(v3_sub(at, eye));
  Vec3 r = v3_norm(v3(f.y * up.z - f.z * up.y, f.z * up.x - f.x * up.z,
                      f.x * up.y - f.y * up.x));
  Vec3 u =
      v3(r.y * f.z - r.z * f.y, r.z * f.x - r.x * f.z, r.x * f.y - r.y * f.x);
  Mat4 m = m4_identity();
  m.m[0][0] = r.x;
  m.m[0][1] = r.y;
  m.m[0][2] = r.z;
  m.m[0][3] = -v3_dot(r, eye);
  m.m[1][0] = u.x;
  m.m[1][1] = u.y;
  m.m[1][2] = u.z;
  m.m[1][3] = -v3_dot(u, eye);
  m.m[2][0] = -f.x;
  m.m[2][1] = -f.y;
  m.m[2][2] = -f.z;
  m.m[2][3] = v3_dot(f, eye);
  return m;
}

static Mat4 m4_normal_mat(Mat4 m) {
  Mat4 n = m4_identity();
  n.m[0][0] = m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1];
  n.m[0][1] = m.m[1][2] * m.m[2][0] - m.m[1][0] * m.m[2][2];
  n.m[0][2] = m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0];
  n.m[1][0] = m.m[0][2] * m.m[2][1] - m.m[0][1] * m.m[2][2];
  n.m[1][1] = m.m[0][0] * m.m[2][2] - m.m[0][2] * m.m[2][0];
  n.m[1][2] = m.m[0][1] * m.m[2][0] - m.m[0][0] * m.m[2][1];
  n.m[2][0] = m.m[0][1] * m.m[1][2] - m.m[0][2] * m.m[1][1];
  n.m[2][1] = m.m[0][2] * m.m[1][0] - m.m[0][0] * m.m[1][2];
  n.m[2][2] = m.m[0][0] * m.m[1][1] - m.m[0][1] * m.m[1][0];
  return n;
}

static Mat4 m4_translate(float x, float y, float z) {
  Mat4 m = m4_identity();
  m.m[0][3] = x;
  m.m[1][3] = y;
  m.m[2][3] = z;
  return m;
}

/* ── §4 paint (216 RGB cube + Bourke ramp) ───────────────────────────── */

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

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* paint_cell — the only place that collapses HDR to display range.
 * Reinhard tone-map → gamma → Bayer dither → 6×6×6 cube + Bourke
 * ramp + A_BOLD/A_DIM. Emissive HDR (channels > 1.0) is fine; Reinhard
 * compresses it to [0, 1) before quantisation. */
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

  /* Bloom halo cells land in luma ≈ 0.05-0.15. A_DIM previously
   * kicked in there and HALVED their brightness on most terminals
   * — making the halo nearly invisible despite the bloom math
   * producing a clean Gaussian. Drop A_DIM so low-luma cells paint
   * at their actual computed brightness. A_BOLD on the bright orb
   * stays so it still pops over the halo. */
  int attr = (luma > 0.85f) ? A_BOLD : A_NORMAL;

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[idx]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 mesh ─────────────────────────────────────────────────────────── */

typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
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
  Vec3 p0 = origin;
  Vec3 p1 = v3_add(origin, e1);
  Vec3 p2 = v3_add(origin, v3_add(e1, e2));
  Vec3 p3 = v3_add(origin, e2);
  m->verts[m->nvert++] = (Vertex){p0, nrm, 0.f, 0.f};
  m->verts[m->nvert++] = (Vertex){p1, nrm, 1.f, 0.f};
  m->verts[m->nvert++] = (Vertex){p2, nrm, 1.f, 1.f};
  m->verts[m->nvert++] = (Vertex){p3, nrm, 0.f, 1.f};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 1, v0 + 2}};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 2, v0 + 3}};
}

/* tessellate_box — axis-aligned cuboid centred at origin, 24 verts
 * (4 per face × 6 faces) so each face has its own flat normal. */
static Mesh tessellate_box(float hx, float hy, float hz) {
  Mesh m;
  m.verts = malloc(24 * sizeof(Vertex));
  m.tris = malloc(12 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  mesh_add_quad(&m, v3(hx, -hy, -hz), v3(0, 2 * hy, 0), v3(0, 0, 2 * hz),
                v3(1, 0, 0));
  mesh_add_quad(&m, v3(-hx, -hy, hz), v3(0, 2 * hy, 0), v3(0, 0, -2 * hz),
                v3(-1, 0, 0));
  mesh_add_quad(&m, v3(-hx, hy, hz), v3(2 * hx, 0, 0), v3(0, 0, -2 * hz),
                v3(0, 1, 0));
  mesh_add_quad(&m, v3(-hx, -hy, -hz), v3(2 * hx, 0, 0), v3(0, 0, 2 * hz),
                v3(0, -1, 0));
  mesh_add_quad(&m, v3(-hx, -hy, hz), v3(2 * hx, 0, 0), v3(0, 2 * hy, 0),
                v3(0, 0, 1));
  mesh_add_quad(&m, v3(hx, -hy, -hz), v3(-2 * hx, 0, 0), v3(0, 2 * hy, 0),
                v3(0, 0, -1));
  return m;
}

static Mesh tessellate_quad(Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm) {
  Mesh m;
  m.verts = malloc(4 * sizeof(Vertex));
  m.tris = malloc(2 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;
  mesh_add_quad(&m, origin, e1, e2, nrm);
  return m;
}

/* UV sphere with smooth normals (radial outward). */
static Mesh tessellate_sphere(float radius, int rings, int segs) {
  int n_verts = (rings + 1) * (segs + 1);
  int n_tris = rings * segs * 2;
  Mesh m;
  m.verts = malloc((size_t)n_verts * sizeof(Vertex));
  m.tris = malloc((size_t)n_tris * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  for (int i = 0; i <= rings; i++) {
    float theta = (float)M_PI * (float)i / (float)rings;
    float st = sinf(theta), ct = cosf(theta);
    for (int j = 0; j <= segs; j++) {
      float phi = 2.f * (float)M_PI * (float)j / (float)segs;
      float x = radius * st * cosf(phi);
      float y = radius * ct;
      float z = radius * st * sinf(phi);
      Vec3 pos = v3(x, y, z);
      Vec3 nrm = v3(x / radius, y / radius, z / radius);
      float u = (float)j / (float)segs;
      float vv = (float)i / (float)rings;
      m.verts[m.nvert++] = (Vertex){pos, nrm, u, vv};
    }
  }

  for (int i = 0; i < rings; i++) {
    for (int j = 0; j < segs; j++) {
      int v00 = i * (segs + 1) + j;
      int v10 = (i + 1) * (segs + 1) + j;
      int v11 = (i + 1) * (segs + 1) + (j + 1);
      int v01 = i * (segs + 1) + (j + 1);
      m.tris[m.ntri++] = (Triangle){{v00, v10, v01}};
      m.tris[m.ntri++] = (Triangle){{v10, v11, v01}};
    }
  }
  return m;
}

/* ── §6 G-buffer — geometry pass ─────────────────────────────────────── */

static Vec3 g_pos[GBUF_MAX_H][GBUF_MAX_W];
static Vec3 g_normal[GBUF_MAX_H][GBUF_MAX_W];
static Vec3 g_albedo[GBUF_MAX_H][GBUF_MAX_W];
static Vec3 g_emissive[GBUF_MAX_H][GBUF_MAX_W]; /* HDR per surface  */
static float g_zbuf[GBUF_MAX_H][GBUF_MAX_W];    /* NDC z            */
static float g_z_view[GBUF_MAX_H][GBUF_MAX_W];  /* linear view-z    */
static uint8_t g_valid[GBUF_MAX_H][GBUF_MAX_W];

static void gbuffer_clear(int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      g_zbuf[r][c] = 1.0f;
      g_z_view[r][c] = -CAM_FAR;
      g_valid[r][c] = 0;
    }
  }
}

static void barycentric(const float sx[3], const float sy[3], float px,
                        float py, float b[3]) {
  float d =
      (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2]);
  if (fabsf(d) < 1e-6f) {
    b[0] = b[1] = b[2] = -1.f;
    return;
  }
  b[0] = ((sy[1] - sy[2]) * (px - sx[2]) + (sx[2] - sx[1]) * (py - sy[2])) / d;
  b[1] = ((sy[2] - sy[0]) * (px - sx[2]) + (sx[0] - sx[2]) * (py - sy[2])) / d;
  b[2] = 1.f - b[0] - b[1];
}

/*
 * rasterize_object — vertex transform → perspective divide + cull →
 * barycentric raster + z-test → write full G-buffer including the
 * EMISSIVE channel. Most surfaces have emissive=(0,0,0); the orb's
 * emissive carries the HDR colour that bloom will pick up.
 */
static void rasterize_object(const Mesh *mesh, Vec3 albedo, Vec3 emissive,
                             Mat4 mvp, Mat4 model, Mat4 modelview,
                             Mat4 norm_mat, int cols, int rows) {
  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];

    Vec4 clip[3];
    Vec3 wpos[3], wnrm[3];
    float vz[3];
    for (int vi = 0; vi < 3; vi++) {
      const Vertex *v = &mesh->verts[tri->v[vi]];
      clip[vi] = m4_mul_v4(mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
      wpos[vi] = m4_pt(model, v->pos);
      wnrm[vi] = v3_norm(m4_dir(norm_mat, v->normal));
      vz[vi] = m4_pt(modelview, v->pos).z;
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

    /* Back-face cull. Screen Y-flip turns OpenGL CCW front faces
     * into NEGATIVE signed area; keep negative, reject the rest. */
    float area =
        (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
    if (area >= 0.f)
      continue;

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
        g_z_view[py][px] = b[0] * vz[0] + b[1] * vz[1] + b[2] * vz[2];
        g_pos[py][px] = v3_bary(wpos[0], wpos[1], wpos[2], b[0], b[1], b[2]);
        g_normal[py][px] =
            v3_norm(v3_bary(wnrm[0], wnrm[1], wnrm[2], b[0], b[1], b[2]));
        g_albedo[py][px] = albedo;
        g_emissive[py][px] = emissive;
        g_valid[py][px] = 1;
      }
    }
  }
}

static void render_gbuffer(const Mesh *meshes, const Vec3 *albedos,
                           const Vec3 *emissives, const Mat4 *models,
                           int n_objects, Mat4 view, Mat4 proj, int cols,
                           int rows) {
  gbuffer_clear(cols, rows);
  for (int oi = 0; oi < n_objects; oi++) {
    Mat4 mv = m4_mul(view, models[oi]);
    Mat4 mvp = m4_mul(proj, mv);
    Mat4 nmat = m4_normal_mat(models[oi]);
    rasterize_object(&meshes[oi], albedos[oi], emissives[oi], mvp, models[oi],
                     mv, nmat, cols, rows);
  }
}

/* ── §7 ssao ─────────────────────────────────────────────────────────── *
 *
 * Same algorithm as ssao_pipeline.c. For each visible pixel sample K
 * directions in the upper hemisphere along N; project each sample
 * back to the screen and ask the depth buffer "is something closer
 * to the camera than my sample?" Average the YES answers → AO ∈ [0,1]
 * darkening factor. 3×3 box blur smooths the per-pixel kernel noise. */

static Vec3 k_ssao[SSAO_KERNEL_VARIANTS][SSAO_SAMPLES];

static unsigned lcg_step(unsigned *s) {
  *s = *s * 1664525u + 1013904223u;
  return *s;
}
static float lcg_unit(unsigned *s) {
  return (lcg_step(s) >> 8) / (float)0x01000000;
}

static void ssao_init_kernel(void) {
  unsigned seed = 0xC0FFEE5Au;
  for (int v = 0; v < SSAO_KERNEL_VARIANTS; v++) {
    for (int i = 0; i < SSAO_SAMPLES; i++) {
      float dx, dy, dz, len2;
      do {
        dx = 2.f * lcg_unit(&seed) - 1.f;
        dy = 2.f * lcg_unit(&seed) - 1.f;
        dz = 2.f * lcg_unit(&seed) - 1.f;
        len2 = dx * dx + dy * dy + dz * dz;
      } while (len2 > 1.f || len2 < 1e-6f);
      float inv = 1.f / sqrtf(len2);
      float t = (float)i / (float)SSAO_SAMPLES;
      float scale = 0.1f + 0.9f * t * t; /* bias toward surface */
      k_ssao[v][i] = v3_scale(v3(dx * inv, dy * inv, dz * inv), scale);
    }
  }
}

static float g_ao[GBUF_MAX_H][GBUF_MAX_W];
static float g_ao_blur[GBUF_MAX_H][GBUF_MAX_W];

static void ssao_pass(Mat4 vp, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g_valid[r][c]) {
        g_ao[r][c] = 1.f;
        continue;
      }

      Vec3 P = g_pos[r][c];
      Vec3 N = g_normal[r][c];
      int variant = (c & 1) | ((r & 1) << 1);

      float occlude_w = 0.f, total_w = 0.f;
      for (int i = 0; i < SSAO_SAMPLES; i++) {
        Vec3 dir = k_ssao[variant][i];
        if (v3_dot(dir, N) < 0.f)
          dir = v3_neg(dir); /* hemisphere */
        Vec3 S = v3_add(P, v3_scale(dir, SSAO_RADIUS));

        Vec4 clip = m4_mul_v4(vp, v4(S.x, S.y, S.z, 1.f));
        if (clip.w < 0.001f)
          continue;
        int ix = (int)((clip.x / clip.w + 1.f) * 0.5f * (float)cols);
        int iy = (int)((-clip.y / clip.w + 1.f) * 0.5f * (float)rows);
        if (ix < 0 || ix >= cols || iy < 0 || iy >= rows)
          continue;
        if (!g_valid[iy][ix])
          continue;

        float dz = fabsf(g_z_view[r][c] - g_z_view[iy][ix]);
        float attn = 1.f - dz / SSAO_RADIUS;
        if (attn <= 0.f)
          continue;
        total_w += attn;

        float ndc_z_S = clip.z / clip.w;
        if (g_zbuf[iy][ix] < ndc_z_S - SSAO_BIAS)
          occlude_w += attn;
      }
      float ao = (total_w > 1e-6f) ? (1.f - occlude_w / total_w) : 1.f;
      g_ao[r][c] = clamp01(ao);
    }
  }
}

static void ssao_blur(int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g_valid[r][c]) {
        g_ao_blur[r][c] = 1.f;
        continue;
      }
      float sum = 0.f;
      int count = 0;
      for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
          int rr = r + dr, cc = c + dc;
          if (rr < 0 || rr >= rows || cc < 0 || cc >= cols)
            continue;
          if (!g_valid[rr][cc])
            continue;
          sum += g_ao[rr][cc];
          count++;
        }
      }
      g_ao_blur[r][c] = (count > 0) ? (sum / (float)count) : 1.f;
    }
  }
}

/* ── §8 lightpass — Blinn-Phong + emissive (HDR output) ──────────────── *
 *
 *   ambient  = AMBIENT · albedo · AO       (AO modulates ambient only)
 *   diffuse  = albedo · sun · max(0, N · L)
 *   spec     = sun · max(0, N · H)^SHININESS · SPEC_GAIN
 *   lit      = ambient + diffuse + spec + EMISSIVE
 *
 * NOT clamped to [0,1] — emissive can push channels past white, and
 * bloom needs that excess to extract. Tone-mapping happens at paint
 * time only. */
static Vec3 g_light[GBUF_MAX_H][GBUF_MAX_W];

static void render_lightpass(Vec3 cam_pos, bool ssao_enabled, int cols,
                             int rows) {
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
      Vec3 emit = g_emissive[r][c];
      float ao = ssao_enabled ? g_ao_blur[r][c] : 1.f;

      Vec3 amb = v3(ambient.x * albedo.x * ao, ambient.y * albedo.y * ao,
                    ambient.z * albedo.z * ao);

      float diff = fmaxf(0.f, v3_dot(N, L));
      Vec3 dif = v3(albedo.x * sun_col.x * diff, albedo.y * sun_col.y * diff,
                    albedo.z * sun_col.z * diff);

      Vec3 V = v3_norm(v3_sub(cam_pos, P));
      Vec3 H = v3_norm(v3_add(L, V));
      float spec = powf(fmaxf(0.f, v3_dot(N, H)), SHININESS) * SPEC_GAIN;
      Vec3 sp = v3(sun_col.x * spec, sun_col.y * spec, sun_col.z * spec);

      /* HDR output — emissive can push past 1.0 per channel. */
      g_light[r][c] =
          v3(amb.x + dif.x + sp.x + emit.x, amb.y + dif.y + sp.y + emit.y,
             amb.z + dif.z + sp.z + emit.z);
    }
  }
}

/* ── §9 bloom — extract → blur → composite ───────────────────────────── *
 *
 * Four small passes, each one statement-per-step:
 *
 *   bloom_extract(threshold) — copy bright pixels of g_light into
 *     g_bloom; everything below threshold becomes (0, 0, 0).
 *
 *   bloom_blur_h() / bloom_blur_v() — 1-D Gaussian along x then y.
 *     Separable: 2-D blur via two 1-D passes is mathematically
 *     equivalent to a 2-D blur with a square kernel, but cheap.
 *
 *   bloom_composite(intensity) — add INTENSITY · g_bloom back into
 *     g_light. Now g_light contains the original lit pixels plus a
 *     soft halo around each bright source. */

static Vec3 g_bloom[GBUF_MAX_H][GBUF_MAX_W];     /* current bloom buffer */
static Vec3 g_bloom_tmp[GBUF_MAX_H][GBUF_MAX_W]; /* horizontal-pass scratch */

static void bloom_extract(float threshold, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      Vec3 lit = g_light[r][c];
      g_bloom[r][c] = (v3_luma(lit) > threshold) ? lit : v3(0, 0, 0);
    }
  }
}

/* Clamp a coordinate to [0, max-1] so the kernel sampling at the
 * screen edge duplicates the boundary value (no halo cut-off). */
static inline int clamp_ix(int v, int max) {
  return v < 0 ? 0 : (v >= max ? max - 1 : v);
}

static void bloom_blur_h(int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      Vec3 sum = v3(0, 0, 0);
      for (int k = 0; k < BLOOM_TAPS; k++) {
        int sc = clamp_ix(c + k - BLOOM_RADIUS, cols);
        float w = BLOOM_KERNEL[k];
        Vec3 s = g_bloom[r][sc];
        sum = v3_add(sum, v3_scale(s, w));
      }
      g_bloom_tmp[r][c] = sum;
    }
  }
}

static void bloom_blur_v(int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      Vec3 sum = v3(0, 0, 0);
      for (int k = 0; k < BLOOM_TAPS; k++) {
        int sr = clamp_ix(r + k - BLOOM_RADIUS, rows);
        float w = BLOOM_KERNEL[k];
        Vec3 s = g_bloom_tmp[sr][c];
        sum = v3_add(sum, v3_scale(s, w));
      }
      g_bloom[r][c] = sum;
    }
  }
}

static void bloom_composite(float intensity, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      g_light[r][c] = v3_add(g_light[r][c], v3_scale(g_bloom[r][c], intensity));
    }
  }
}

/* ── §10 scene ───────────────────────────────────────────────────────── */

typedef struct {
  Mesh meshes[N_OBJECTS];
  Vec3 albedos[N_OBJECTS];
  Vec3 emissives[N_OBJECTS];
  Mat4 models[N_OBJECTS];

  Mat4 view, proj, vp;
  Vec3 cam_pos;
  float cam_dist;
  float cam_yaw;

  bool bloom_on;
  bool ssao_on;
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
  s->cam_pos = v3(sinf(s->cam_yaw) * r, CAM_EYE_Y, cosf(s->cam_yaw) * r);
  s->view = m4_lookat(s->cam_pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
  s->vp = m4_mul(s->proj, s->view);
}

/* scene_init — floor + 2 cubes + 1 emissive orb. The orb is the
 * only object with non-zero emissive; bloom feeds on its HDR pixels. */
static void scene_init(Scene *s, int total_cols, int total_rows) {
  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&s->meshes[i]);

  memset(s, 0, sizeof *s);
  s->scene_cols = total_cols;
  s->scene_rows = total_rows - HUD_ROWS;
  s->bloom_on = true;
  s->ssao_on = true;
  s->cam_dist = CAM_DIST;
  s->cam_yaw = 0.f;

  s->meshes[OBJ_FLOOR] = tessellate_quad(
      v3(-FLOOR_HALF_X, 0.f, FLOOR_HALF_Z), v3(2 * FLOOR_HALF_X, 0.f, 0.f),
      v3(0.f, 0.f, -2 * FLOOR_HALF_Z), v3(0.f, 1.f, 0.f));
  s->albedos[OBJ_FLOOR] = v3(0.32f, 0.36f, 0.42f); /* dark slate */
  s->emissives[OBJ_FLOOR] = v3(0, 0, 0);
  s->models[OBJ_FLOOR] = m4_identity();

  s->meshes[OBJ_CUBE_A] = tessellate_box(CUBE_HALF, CUBE_HALF, CUBE_HALF);
  s->albedos[OBJ_CUBE_A] = v3(0.78f, 0.62f, 0.42f); /* sandstone  */
  s->emissives[OBJ_CUBE_A] = v3(0, 0, 0);
  s->models[OBJ_CUBE_A] = m4_translate(CUBE_A_CX, CUBE_A_CY, CUBE_A_CZ);

  s->meshes[OBJ_CUBE_B] = tessellate_box(CUBE_HALF, CUBE_HALF, CUBE_HALF);
  s->albedos[OBJ_CUBE_B] = v3(0.78f, 0.62f, 0.42f);
  s->emissives[OBJ_CUBE_B] = v3(0, 0, 0);
  s->models[OBJ_CUBE_B] = m4_translate(CUBE_B_CX, CUBE_B_CY, CUBE_B_CZ);

  s->meshes[OBJ_ORB] = tessellate_sphere(ORB_RADIUS, ORB_RINGS, ORB_SEGS);
  s->albedos[OBJ_ORB] = v3(0.20f, 0.20f, 0.20f); /* mostly emit */
  s->emissives[OBJ_ORB] = v3(ORB_EMISSIVE[0], ORB_EMISSIVE[1], ORB_EMISSIVE[2]);
  s->models[OBJ_ORB] = m4_translate(ORB_CX, ORB_CY, ORB_CZ);

  scene_rebuild_proj(s, total_cols, s->scene_rows);
  scene_rebuild_view(s);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->cam_yaw += CAM_ORBIT_RAD_PER_SEC * dt;
  scene_rebuild_view(s);
}

/* ── §11 screen — render_scene + HUD ─────────────────────────────────── */

static void render_scene(const Scene *s) {
  int cols = s->scene_cols;
  int rows = s->scene_rows;

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g_valid[r][c])
        continue;
      paint_cell(c, r, g_light[r][c]);
    }
  }
}

static void hud_draw(const Scene *s, double fps) {
  int hr = s->scene_rows;
  int cols = s->scene_cols;

  int total_tris = 0;
  for (int i = 0; i < N_OBJECTS; i++)
    total_tris += s->meshes[i].ntri;

  char status[160];
  snprintf(status, sizeof status,
           " %5.1f fps  bloom:%s  ssao:%s  zoom:%.1f  tris:%d  %s ", fps,
           s->bloom_on ? "ON " : "OFF", s->ssao_on ? "ON " : "OFF",
           (double)s->cam_dist, total_tris, s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > cols)
    slen = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - slen, "%s", status);
  mvprintw(0, 0, " BLOOM · DEFERRED · SSAO ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(hr + 0, 1,
           "passes: gbuffer -> ssao+blur -> lightpass(HDR) -> "
           "bloom(extract+blurH+blurV+composite) -> paint");
  mvprintw(hr + 1, 1,
           "bloom: threshold=%.2f  intensity=%.2f  kernel=%d-tap Gaussian "
           "(separable)",
           (double)BLOOM_THRESHOLD, (double)BLOOM_INTENSITY, BLOOM_TAPS);
  mvprintw(hr + 2, 1,
           "Toggle 'b' off: orb is a hard-edged disc.   "
           "Toggle 'a' off: cube corners stop darkening.");
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hr + HUD_ROWS - 1, 0,
           " q:quit  spc:pause  b:bloom  a:ssao  +/-:zoom  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §12 app ─────────────────────────────────────────────────────────── */

typedef struct {
  Scene scene;
  int total_cols;
  int total_rows;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void screen_init(void) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
}

static void app_do_resize(App *app) {
  endwin();
  refresh();
  getmaxyx(stdscr, app->total_rows, app->total_cols);
  scene_init(&app->scene, app->total_cols, app->total_rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_init(s, app->total_cols, app->total_rows);
    break;
  case 'b':
  case 'B':
    s->bloom_on = !s->bloom_on;
    break;
  case 'a':
  case 'A':
    s->ssao_on = !s->ssao_on;
    break;
  case '=':
  case '+':
    s->cam_dist -= CAM_ZOOM_STEP;
    if (s->cam_dist < CAM_DIST_MIN)
      s->cam_dist = CAM_DIST_MIN;
    scene_rebuild_view(s);
    break;
  case '-':
  case '_':
    s->cam_dist += CAM_ZOOM_STEP;
    if (s->cam_dist > CAM_DIST_MAX)
      s->cam_dist = CAM_DIST_MAX;
    scene_rebuild_view(s);
    break;
  default:
    break;
  }
  return true;
}

/*
 * One-frame pipeline. Reads as plain pseudocode — each named pass
 * does exactly what its name says, conditionally, in dependency
 * order. SSAO blur depends on SSAO pass; lightpass depends on the
 * G-buffer (and optionally on AO); bloom depends on lightpass.
 */
static void render_frame(const Scene *s) {
  render_gbuffer(s->meshes, s->albedos, s->emissives, s->models, N_OBJECTS,
                 s->view, s->proj, s->scene_cols, s->scene_rows);

  if (s->ssao_on) {
    ssao_pass(s->vp, s->scene_cols, s->scene_rows);
    ssao_blur(s->scene_cols, s->scene_rows);
  }

  render_lightpass(s->cam_pos, s->ssao_on, s->scene_cols, s->scene_rows);

  if (s->bloom_on) {
    bloom_extract(BLOOM_THRESHOLD, s->scene_cols, s->scene_rows);
    bloom_blur_h(s->scene_cols, s->scene_rows);
    bloom_blur_v(s->scene_cols, s->scene_rows);
    bloom_composite(BLOOM_INTENSITY, s->scene_cols, s->scene_rows);
  }
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  ssao_init_kernel();

  App *app = &g_app;
  app->running = 1;

  screen_init();
  getmaxyx(stdscr, app->total_rows, app->total_cols);
  scene_init(&app->scene, app->total_cols, app->total_rows);

  int64_t frame_time = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    scene_tick(&app->scene, dt_sec);

    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);

    Scene *s = &app->scene;
    erase();
    render_frame(s);
    render_scene(s);
    hud_draw(s, fps_display);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&app->scene.meshes[i]);

  endwin();
  return 0;
}
