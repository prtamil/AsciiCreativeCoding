/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * displace_raster.c — software vertex displacement on a UV sphere
 *
 * DEMO: A UV sphere whose vertices are pushed along their normals every
 *       frame by a time-varying scalar field. The geometry actually
 *       changes — this isn't a normal-map fake, the surface really
 *       moves. Cycle 'd' through four modes:
 *
 *         RIPPLE  concentric rings sweep from the equator outward
 *         WAVE    a diagonal travelling wave deforms the whole ball
 *         PULSE   the sphere breathes, biggest at the equator
 *         SPIKY   a porcupine of moving spikes
 *
 *       Cycle 's' through four shaders (phong → toon → normals → wire)
 *       to see HOW the deformed surface is being lit. The fragment
 *       shaders are unchanged from any other file in the folder —
 *       what makes the demo "react" to the deformation is the VERTEX
 *       pass recomputing each surface normal numerically (central
 *       difference) at every frame. Without that, the highlights
 *       would still hug the original sphere while the visible surface
 *       heaved underneath.
 *
 * Study alongside:
 *   raster/sphere_raster.c  — same UV sphere, no displacement
 *   raster/cube_raster.c    — same shader-program scaffolding
 *
 * Section map:
 *   §1  config       — frame, view, sphere, displacement, ramp, dither
 *   §2  math         — Vec3 / Vec4 / Mat4 helpers
 *   §3  displace     — four mode functions + tangent basis + central-diff
 *   §4  shaders      — VSIn / VSOut / FSIn / FSOut + 1 vert × 4 frag
 *   §5  mesh         — UV sphere tessellation (poles handled explicitly)
 *   §6  framebuffer  — zbuf + cbuf, Bayer dither, Bourke ramp, blit
 *   §7  pipeline     — vertex transform → cull → barycentric raster → FS
 *   §8  scene        — uniforms wiring, tick, draw, mode/shader swap
 *   §9  screen       — ncurses init / resize / HUD / present
 *   §10 app          — dt loop, input, resize, cleanup
 *
 * Keys:
 *   d / D     cycle displacement mode  (ripple → wave → pulse → spiky)
 *   s / S     cycle shader             (phong → toon → normals → wire)
 *   c / C     toggle back-face culling
 *   + / =     zoom in
 *   -         zoom out
 *   space     pause / resume animation
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/displace_raster.c -o displace \
 *       -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Vertex displacement. Push every mesh vertex along its
 *                  surface normal by a scalar function f(p, t) — the
 *                  "displacement field". Then recompute the surface
 *                  normal numerically so lighting follows the deformed
 *                  shape, not the original. The fragment shaders never
 *                  see the displacement; they read whatever world
 *                  position + normal arrived from the vertex stage.
 *
 *                  Field formulas (all return a scalar offset along N):
 *                    RIPPLE   f = A·sin(ω·t + k·r) · taper(y),  r = √(x²+z²)
 *                    WAVE     f = A·sin(ω·t + k·(x + 0.8y + 0.5z))
 *                    PULSE    f = A·sin(ω·t)·exp(−γ·r)
 *                    SPIKY    f = A·|sin(kx)·sin(ky)·sin(kz)|^0.6
 *
 *                  Normal recomputation (central difference): pick two
 *                  tangent directions T, B perpendicular to N; sample
 *                  f at p ± εT and p ± εB; reconstruct the displaced
 *                  tangent vectors and cross-product them.
 *
 * Data-structure : One static UV sphere mesh (TESS_U × TESS_V quads,
 *                  pole vertices handled explicitly), tessellated once
 *                  at init. Per-frame the vertex shader re-derives
 *                  position + normal from the base sphere; the mesh
 *                  buffer never changes. Fragment outputs land in a
 *                  zbuf + cbuf framebuffer the same shape as cube_raster.
 *
 * Rendering      : Standard forward pipeline. Per triangle: vertex
 *                  shader runs (displace + normal recompute) → clip →
 *                  perspective divide → screen → back-face cull →
 *                  bounding-box raster with barycentric interp →
 *                  fragment shader → luma → Bayer dither → Bourke ramp.
 *
 * Performance    : O(N_verts) displacement evaluations per frame. The
 *                  normal pass adds 4 extra f() calls per vertex (2
 *                  tangent dirs × ±ε). With TESS_U=48, TESS_V=32 →
 *                  ~3000 vertices × 5 = 15 k field evals per frame —
 *                  trivial at 60 fps in a terminal.
 *
 * References     : Cook, "Shade Trees," SIGGRAPH '84 (the original
 *                    "displacement shader" concept).
 *                  RenderMan Companion §13 — central difference is
 *                    the canonical RenderMan technique for derived
 *                    normals.
 *                  Akenine-Möller et al., Real-Time Rendering 4ed,
 *                    §16.2.4 (modern displacement on the GPU).
 *                  Möller, "Fast Triangle Rasterization by
 *                    Interpolating Edge Functions," GPG (2000).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Tessellate the sphere ONCE. Every frame the vertex shader pushes
 * each vertex along its (original) normal by f(p, t). Moving the
 * vertex invalidates its normal — the surface has bent — so we
 * recompute the normal numerically by sampling f at four nearby
 * points and crossing the resulting tangent vectors. That single
 * trick is what makes the four fragment shaders react to the
 * moving geometry without knowing it moved.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Pushing thumbtacks into and out of a balloon under a torch:
 * displacement = "move the thumbtacks"; normal recompute = "tell
 * the torch where the new face is." If you only move thumbtacks,
 * the highlight stays where it was while the surface heaves
 * underneath — wrong. Recomputing the normal puts the highlight
 * where the new face actually points.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │   undisplaced              displaced         │
 *      │                                              │
 *      │      ↑ N (sphere)            ↑ N′ (recomputed)│
 *      │       \                       \              │
 *      │    ────●────             ─────●──╲           │
 *      │      sphere                deformed surface  │
 *      └──────────────────────────────────────────────┘
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 *  1. Tessellate once at init: TESS_U × TESS_V quads, normals = unit
 *     positions, poles handled explicitly.
 *
 *  2. Per frame, vert_displace runs for every vertex:
 *       N    = normalize(pos)                    // sphere normal
 *       d    = disp_fn(pos, time, amp, freq)     // scalar offset
 *       p′   = pos + N · d                       // displaced pos
 *       N′   = displaced_normal(pos, N, ...)     // central diff
 *       clip = MVP · (p′, 1)                     // for rasteriser
 *       wpos = model    · p′                     // for lighting
 *       wnrm = norm_mat · N′                     // for lighting
 *
 *  3. displaced_normal builds a tangent frame (T, B) at pos and
 *     samples f at pos ± εT and pos ± εB. The displaced tangents are
 *       T′ = 2εT + N · (f(+εT) − f(−εT))
 *       B′ = 2εB + N · (f(+εB) − f(−εB))
 *     and N′ = normalize(T′ × B′).
 *
 *  4. Rasteriser barycentric-interpolates world_pos, world_nrm,
 *     u/v, and custom[] across each triangle; z-test → fragment
 *     shader → Vec3 colour.
 *
 *  5. luma_to_cell quantises the Vec3 to a Bayer-dithered Bourke
 *     glyph + colour pair.
 *
 * KEY FORMULAS
 * ────────────
 *   Displacement    p′ = p + N · f(p, t)
 *   Tangent basis   T = norm(cross(up, N)),  B = cross(N, T)
 *                   up = (0, 1, 0) if |N.y| < 0.9 else (1, 0, 0)
 *   Central diff    ∂f/∂T ≈ (f(p+εT) − f(p−εT)) / (2ε)
 *   Displaced T′    T′ = 2εT + N · ΔfT
 *   New normal      N′ = normalize(T′ × B′)
 *   CD epsilon      ε = 0.03 · R    (3 % of radius — sweet spot)
 *   Bayer dither    d ← luma + (B[py%4][px%4] − 0.5) · 0.15
 *   Luma            Y = 0.2126·R + 0.7152·G + 0.0722·B
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • CD_EPS too small → float noise dominates the difference,
 *     normals jitter, lighting flickers. Too large → normal lags
 *     curvature, high-freq spikes get softened. 0.03 · R is the
 *     tuned sweet spot.
 *
 *   • Tangent basis at the poles: if N ≈ ±Y, cross(Y, N) is
 *     degenerate. make_tangent_basis falls back to up = (1, 0, 0).
 *
 *   • DisplaceUniforms leads with `Uniforms base` so &disp_uni casts
 *     cleanly to const Uniforms* inside the fragment shaders. The
 *     vert_uni / frag_uni split in ShaderProgram is the alternative
 *     for shaders that need a different uniform struct (toon).
 *
 *   • Pole vertex normals are hard-coded to (0, ±1, 0) in
 *     tessellate_sphere — sin(phi) = 0 makes v3_norm degenerate.
 *
 *   • Back-face cull is by signed-area sign in screen space. A deep
 *     concave displacement (big negative d) can flip a triangle's
 *     winding; press 'c' to disable culling if it happens.
 *
 *   • Wireframe shader reads barycentric coords from custom[0..2];
 *     the pipeline writes those AFTER vert_displace_wire runs.
 *
 *   • Time advances only while !paused; rotation does too.
 *     Resize does NOT reset time, so animation continues smoothly.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Switch to NORMALS shader: the RGB hue at each pixel is the
 *     world normal at that fragment. Wave crests rotate the hue;
 *     spikes show needle-tip swirls. If hue stayed static while
 *     the silhouette heaved, normal recompute would be broken.
 *
 *   • Pause + cycle 'd': at t = 0, ripple/wave/pulse are zero
 *     (sin(0) = 0) so the sphere is undeformed. SPIKY has permanent
 *     peaks because |sin · sin · sin| > 0 for most positions.
 *
 *   • Toon shader: the band boundaries follow the wave crests, not
 *     the original sphere outline — confirms the displaced normal
 *     is what reaches the fragment.
 *
 *   • Wireframe: lat/long grid ripples with the surface. SPIKY mode
 *     makes porcupine-quill distortions of the grid.
 *
 *   • Press 'c' to disable culling — on big PULSE inflations you
 *     should see the interior surface poke through.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. Read CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL as prose. Read
 *      raster/sphere_raster.c first if you don't yet know the
 *      forward 7-stage pipeline; this file changes ONLY the vertex
 *      shader, everything downstream is the same.
 *   2. §1 config — every constant has a unit-bearing comment.
 *   3. §3 displace — the four displacement fields + tangent basis +
 *      central-difference normal recompute. THIS IS THE HEART of
 *      the file. Every other section exists to feed §3 and consume
 *      its output.
 *   4. §4 shaders — one vertex shader (vert_displace) + four fragment
 *      shaders (phong, toon, normals, wire). Notice the fragment
 *      shaders are unmodified copies from cube_raster.c et al —
 *      they have no idea the geometry is moving.
 *   5. §5 mesh — UV-sphere tessellation, identical to sphere_raster.c.
 *   6. §6 framebuffer + §7 pipeline + §8-§10 — infrastructure;
 *      skim on first read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   pos                base sphere position (unit sphere · SPHERE_R)
 *   N                  base sphere normal = pos / |pos|
 *   T, B               tangent and bi-tangent at pos (perpendicular
 *                      to N, perpendicular to each other)
 *   eps / CD_EPS       central-difference step length (along T or B)
 *   d, ΔfT, ΔfB        displacement scalar at the sample points
 *   p_displaced        pos + N · d
 *   N_displaced        normalize(T_displaced × B_displaced)
 *   amp / freq         per-mode amplitude and spatial frequency
 *
 * Background you need
 * ───────────────────
 *   - The 7-stage rasteriser (cube_raster.c).
 *   - Why a unit sphere's surface normal at p is just p̂.
 *   - Cross product as "perpendicular to both" — this is how T × B
 *     reconstructs a normal from two displaced tangents.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Tessellation shaders / hardware tessellation. We do CPU vertex
 *     displacement on a fixed mesh; modern GPUs subdivide adaptively.
 *   - Analytic normal derivation. We use the universal numeric trick
 *     (central difference); the analytic version requires a separate
 *     gradient function per displacement field.
 *   - Bump / normal maps. Those FAKE the lighting; we genuinely move
 *     the geometry.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Eight tutorials that build vertex displacement from first principles.
 *
 *   T1  Genuine displacement vs. fake displacement
 *   T2  Why moving a vertex BREAKS its normal
 *   T3  Building a tangent frame (T, B, N)
 *   T4  Central difference: a numerical derivative without algebra
 *   T5  Reconstructing the displaced normal from displaced tangents
 *   T6  The four displacement fields — anatomy
 *   T7  Choosing ε — the noise-vs-lag trade
 *   T8  Why fragment shaders never see the displacement
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  GENUINE DISPLACEMENT VS. FAKE DISPLACEMENT
 * ──────────────────────────────────────────────
 * There are two ways to make a sphere LOOK like it has bumps:
 *
 *   FAKE  (normal mapping / bump mapping):
 *     The geometry stays a perfect sphere. The fragment shader
 *     reads a texture of fake normals and lights AS IF the surface
 *     were bumpy. The silhouette is still smooth — peek at the
 *     edge and you'll see a circle, not bumps.
 *
 *   REAL (vertex displacement):
 *     The vertices actually move outward (or inward). The
 *     silhouette changes — bumps poke into the outline, valleys
 *     dent it. You can see displacement; you can only INFER bump.
 *
 * This file does the REAL kind. Press 'd' until SPIKY: the
 * silhouette is genuinely jagged, not just shaded jagged.
 *
 *      ┌─────────────────────────────────────────────────────┐
 *      │   FAKE (bump map)            REAL (displacement)    │
 *      │                                                     │
 *      │       _______                    _M_M_M_            │
 *      │      /       \                  / W   W \           │
 *      │     |  bumpy  |                |  bumpy  |          │
 *      │     |  shading|                |  shaded |          │
 *      │      \_______/                  \_W_M_W_/           │
 *      │                                                     │
 *      │ silhouette: smooth circle    silhouette: jagged     │
 *      └─────────────────────────────────────────────────────┘
 *
 * T2  WHY MOVING A VERTEX BREAKS ITS NORMAL
 * ─────────────────────────────────────────
 * A surface normal is "the direction perpendicular to the
 * surface at this point." If you move the surface, the
 * perpendicular changes too.
 *
 * Concretely: a sphere's normal at p is just p̂ (the position
 * is the normal because the centre is at origin). After we
 * push p outward by f(p), the surface no longer matches the
 * sphere — it bulges where f is big and dimples where f is
 * small. The new perpendicular DEPENDS on how f varies in the
 * neighbourhood of p.
 *
 *   At a peak       N′ still points outward (peak is locally a
 *                   small dome → almost the original N).
 *   On a slope      N′ tilts in the direction f is increasing.
 *   At a valley     N′ also points outward (valley is locally a
 *                   small dome from above).
 *   On a ridge edge N′ tilts sharply where f changes fast.
 *
 * The slope case is what kills naïve renderers: if you forget
 * to update N, the highlight stays where it was on the original
 * sphere and the ridges look painted-on rather than physical.
 *
 * The job of §3 displaced_normal is to compute N′ correctly.
 *
 * T3  BUILDING A TANGENT FRAME (T, B, N)
 * ──────────────────────────────────────
 * To detect "how does f vary near p?" we need TWO directions
 * along the surface (perpendicular to N) so we can sample both.
 *
 * The standard trick:
 *
 *     pick an "up-ish" reference vector U (normally world-Y)
 *     T = normalize(cross(U, N))   ← perpendicular to both U and N
 *     B = cross(N, T)              ← perpendicular to T and N
 *
 * (T, B, N) is a right-handed orthonormal frame: three mutually
 * perpendicular unit vectors anchored at p. T and B span the
 * tangent plane — the local "ground" at p; N is the local "up."
 *
 * Pole degeneracy: if N happens to BE Y (north pole), cross(Y, N)
 * is zero — Y and N are parallel, no perpendicular direction
 * exists. We detect that with |N.y| > 0.9 and fall back to
 * U = (1, 0, 0).
 *
 *      ┌──────────────────────────────────────┐
 *      │           N                          │
 *      │           │                          │
 *      │           │                          │
 *      │     ──────●──────  T                 │
 *      │          /                           │
 *      │         /                            │
 *      │        B  (out of page)              │
 *      └──────────────────────────────────────┘
 *
 * T4  CENTRAL DIFFERENCE: NUMERICAL DERIVATIVE WITHOUT ALGEBRA
 * ────────────────────────────────────────────────────────────
 * To know how fast f changes along T, the analytic answer is
 * "compute ∂f/∂T symbolically." That requires a separate
 * derivative function per displacement field — annoying, and
 * brittle if you swap fields.
 *
 * The numeric answer is CENTRAL DIFFERENCE:
 *
 *       ∂f       f(p + εT) − f(p − εT)
 *      ──── ≈   ─────────────────────────
 *       ∂T              2ε
 *
 * Sample f at TWO points slightly along ±T, take the difference,
 * divide by the step. Forward difference (sample at p and p+εT)
 * works too but is less accurate; central differences cancel
 * the leading O(ε) error term, leaving O(ε²).
 *
 * Cost: 2 extra f() evaluations per direction, 4 total per
 * vertex. We don't actually do the divide here — we'll fold it
 * into the cross product in T5.
 *
 * Same trick along B gives ∂f/∂B. Two field samples per
 * direction, four samples per vertex normal.
 *
 * T5  RECONSTRUCTING THE DISPLACED NORMAL FROM DISPLACED TANGENTS
 * ───────────────────────────────────────────────────────────────
 * The point p moves to p′ = p + N·f(p). What about a neighbour
 * p + εT? It moves to (p + εT) + N′·f(p+εT) where N′ is its
 * own normal — but for small ε, N′ ≈ N, so:
 *
 *     (p + εT) + N · f(p + εT)
 *
 * The vector from p′ to that displaced neighbour is therefore:
 *
 *     T_displaced = εT + N·(f(p+εT) − f(p))      forward diff
 *
 * For central difference with the symmetric pair:
 *
 *     T_displaced = 2εT + N·(f(p+εT) − f(p−εT))
 *
 * This is the tangent vector AFTER displacement — pointing along
 * the displaced surface in the original T direction. Same recipe
 * for B_displaced.
 *
 * Two vectors lying in the new tangent plane → cross-product →
 * vector perpendicular to that plane → the new normal:
 *
 *     N_displaced = normalize(T_displaced × B_displaced)
 *
 * Notice the "÷ 2ε" we postponed in T4 cancels: the cross product
 * is bilinear, scaling both inputs by the same factor scales the
 * result by that factor squared, and we normalise anyway. So the
 * code drops the divide entirely.
 *
 * T6  THE FOUR DISPLACEMENT FIELDS — ANATOMY
 * ──────────────────────────────────────────
 * Each mode picks a scalar function f(p, t). That's it — change
 * f, get a different deformation. The renderer doesn't change.
 *
 *   RIPPLE   f = A · sin(ω·t + k·r) · taper(y)
 *            r = √(x² + z²) is distance from the polar axis;
 *            taper(y) = 1 − y² damps near the poles. Reads as
 *            concentric rings travelling outward from the
 *            equator.
 *
 *   WAVE     f = A · sin(ω·t + k·(x + 0.8y + 0.5z))
 *            One global plane wave, travelling along a fixed
 *            diagonal direction. Whole sphere undulates as one.
 *
 *   PULSE    f = A · sin(ω·t) · exp(−γ·r)
 *            No spatial term other than a radial decay; the whole
 *            sphere breathes in/out, biggest near the equator,
 *            smallest near the poles.
 *
 *   SPIKY    f = A · |sin(kx)·sin(ky)·sin(kz)|^0.6
 *            Three sine waves multiplied — each axis contributes
 *            zeros where its sin vanishes, so f stays at zero
 *            along entire sin-gridlines and peaks at the cell
 *            centres. The 0.6 power compresses the dynamic range
 *            so peaks aren't too sharp. Result: a porcupine.
 *
 * The four fields cost O(1) each. The vertex pass is dominated by
 * the FOUR f() calls for normal recompute, not the one for the
 * displacement.
 *
 * T7  CHOOSING ε — THE NOISE-VS-LAG TRADE
 * ───────────────────────────────────────
 * Central difference's error is O(ε²) for smooth f. But with
 * floating-point arithmetic, very small ε hits a different wall:
 *
 *   ε too LARGE  → step crosses real curvature; the difference
 *                  reports the AVERAGE slope, not the local one.
 *                  Sharp peaks get rounded off in the lighting.
 *
 *   ε too SMALL  → f(p+εT) ≈ f(p−εT) to within float precision.
 *                  Their difference is dominated by float noise;
 *                  N′ jitters frame to frame even when t doesn't
 *                  move. Lighting flickers.
 *
 * "Sweet spot" depends on the field's frequency. For our four
 * fields with their default amplitudes/frequencies, ε = 0.03·R
 * is small enough to track the highest-frequency variation
 * (SPIKY) and large enough to dominate float noise. Drop to
 * 0.01·R if you cut amplitudes; raise to 0.05·R if you push k
 * higher.
 *
 * T8  WHY FRAGMENT SHADERS NEVER SEE THE DISPLACEMENT
 * ───────────────────────────────────────────────────
 * The fragment shader receives:
 *
 *   world_pos    interpolated p′ (the DISPLACED position)
 *   world_nrm    interpolated N′ (the DISPLACED normal)
 *   uv, custom   barycentric-interpolated whatever the vert wrote
 *
 * It does NOT receive:
 *   - the base sphere position
 *   - the displacement amplitude
 *   - the time
 *   - which mode is active
 *
 * From the fragment shader's perspective, it's just shading a
 * surface with the position and normal it was handed. That's
 * why phong / toon / normals / wire are unmodified copies from
 * cube_raster.c — none of them know about displacement, and yet
 * they all visibly REACT to it.
 *
 * This is the cleanest way to add per-vertex effects to a forward
 * pipeline: own the vertex shader, leave fragment shaders alone.
 * The vert/frag uniform split (DisplaceUniforms leads with a base
 * Uniforms struct) is what makes &disp_uni cast cleanly to const
 * Uniforms* inside the fragment shaders.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

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

enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500,
  HUD_COLS = 80,

  /*
   * Tessellation resolution.
   * Higher = smoother base sphere, better displacement detail.
   * 48×32 gives 3072 triangles — fast enough at 60fps in a terminal,
   * detailed enough that the displacement waves look smooth.
   * Drop to 36×24 if fps is low on your terminal.
   */
  TESS_U = 48,
  TESS_V = 32,
};

#define CAM_FOV (55.0f * 3.14159265f / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
#define CAM_DIST 3.2f
#define CAM_DIST_MIN 1.2f
#define CAM_DIST_MAX 8.0f
#define CAM_ZOOM_STEP 0.2f

#define SPHERE_R 1.0f

/* Rotation — slow tumble so displacement detail is visible */
#define ROT_Y 0.30f
#define ROT_X 0.12f

/* Wireframe threshold — sphere triangles are small, needs 0.09+ */
#define WIRE_THRESH 0.09f

/*
 * Central difference epsilon for normal recomputation.
 * Too small → floating point noise in the normal.
 * Too large → normal lags the actual surface curvature.
 * 0.03 * SPHERE_R is a good balance for all four displacement modes.
 */
#define CD_EPS (0.03f * SPHERE_R)

/* Paul Bourke ramp */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};

#define CELL_W 8
#define CELL_H 16

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define PI 3.14159265f

/* ── §2 math (V3, V4, Mat4) ──────────────────────────────────────────── */

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
static inline float v3_len(Vec3 a) { return sqrtf(v3_dot(a, a)); }
static inline Vec3 v3_norm(Vec3 a) {
  float l = v3_len(a);
  return l > 1e-7f ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}
static inline Vec3 v3_cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline Vec3 v3_bary(Vec3 a, Vec3 b, Vec3 c, float u, float v, float w) {
  return v3(u * a.x + v * b.x + w * c.x, u * a.y + v * b.y + w * c.y,
            u * a.z + v * b.z + w * c.z);
}

static inline Vec4 v4(float x, float y, float z, float w) {
  return (Vec4){x, y, z, w};
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
static Mat4 m4_rotate_y(float a) {
  Mat4 m = m4_identity();
  m.m[0][0] = cosf(a);
  m.m[0][2] = sinf(a);
  m.m[2][0] = -sinf(a);
  m.m[2][2] = cosf(a);
  return m;
}
static Mat4 m4_rotate_x(float a) {
  Mat4 m = m4_identity();
  m.m[1][1] = cosf(a);
  m.m[1][2] = -sinf(a);
  m.m[2][1] = sinf(a);
  m.m[2][2] = cosf(a);
  return m;
}
static Mat4 m4_perspective(float fovy, float aspect, float near, float far) {
  Mat4 m = {{{0}}};
  float f = 1.f / tanf(fovy * .5f);
  m.m[0][0] = f / aspect;
  m.m[1][1] = f;
  m.m[2][2] = (far + near) / (near - far);
  m.m[2][3] = (2.f * far * near) / (near - far);
  m.m[3][2] = -1.f;
  return m;
}
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

/* ── §3 displacement — the four mode functions + normal recompute ────── */

/*
 * Displacement modes — each returns a scalar offset to apply along
 * the surface normal.  Positive = push outward, negative = push inward.
 *
 * All functions receive:
 *   pos   — point on unit sphere in model space
 *   time  — seconds since start
 *   amp   — amplitude scale (from uniforms)
 *   freq  — frequency scale (from uniforms)
 *
 * They are pure functions — same inputs always produce same output.
 * This is required for the central difference normal recomputation
 * to work correctly: we call them at pos±eps and the delta must be
 * a genuine derivative of the function, not a stateful value.
 */

typedef enum { DM_RIPPLE = 0, DM_WAVE, DM_PULSE, DM_SPIKY, DM_COUNT } DispMode;

static const char *k_disp_names[] = {"ripple", "wave", "pulse", "spiky"};

/*
 * displace_ripple — concentric rings radiating from the equator.
 *
 * r = distance from Y axis in XZ plane.
 * sin(time + r*freq) produces rings that travel inward over time.
 * Multiplied by (1 - |y|) to taper off at the poles — prevents
 * the poles from having large displacements that look broken.
 */
static float displace_ripple(Vec3 pos, float time, float amp, float freq) {
  float r = sqrtf(pos.x * pos.x + pos.z * pos.z);
  float taper = 1.f - fabsf(pos.y) * 0.6f;
  return sinf(time * 2.5f + r * freq) * amp * taper;
}

/*
 * displace_wave — diagonal travelling wave across the surface.
 *
 * Uses a combination of X and Y position as the wave phase, producing
 * a wave that travels diagonally across the sphere.
 * Adding 0.7*pos.z gives the wave a slight depth twist so it does
 * not look flat from any viewing angle.
 */
static float displace_wave(Vec3 pos, float time, float amp, float freq) {
  float phase = pos.x * freq + pos.y * freq * 0.8f + pos.z * freq * 0.5f;
  return sinf(time * 2.0f + phase) * amp;
}

/*
 * displace_pulse — whole sphere breathes in and out.
 *
 * sin(time) gives a smooth oscillation at ~0.5 Hz.
 * Adding a secondary sin at 3x frequency gives the breath a slight
 * "catch" — it does not feel perfectly mechanical.
 * exp(-r * falloff) concentrates the pulse at the equator, making
 * the poles stable anchors that contrast the heaving equatorial band.
 */
static float displace_pulse(Vec3 pos, float time, float amp, float freq) {
  float r = sqrtf(pos.x * pos.x + pos.z * pos.z);
  float breathe = sinf(time * 1.5f) * 0.85f + sinf(time * 4.5f) * 0.15f;
  float falloff = expf(-r * freq * 0.4f);
  return breathe * amp * falloff;
}

/*
 * displace_spiky — spiky ball driven by product of three sine waves.
 *
 * |sin(x*f) * sin(y*f) * sin(z*f)| produces spikes at positions where
 * all three waves are simultaneously at their peaks.
 * The time term slowly rotates the spike pattern so it animates.
 * powf(..., 0.6) softens the product so spikes have a smooth base
 * rather than a pinched needle tip.
 */
static float displace_spiky(Vec3 pos, float time, float amp, float freq) {
  float f = freq * 1.4f;
  float t = time * 0.8f;
  float val = fabsf(sinf(pos.x * f + t) * sinf(pos.y * f + t * 0.7f) *
                    sinf(pos.z * f + t * 1.3f));
  return powf(val, 0.6f) * amp;
}

/* Dispatch table — indexed by DispMode */
typedef float (*DispFn)(Vec3, float, float, float);
static const DispFn k_disp_fn[DM_COUNT] = {
    displace_ripple,
    displace_wave,
    displace_pulse,
    displace_spiky,
};

/*
 * make_tangent_basis — compute two orthogonal tangent vectors for a
 * point on the sphere given its outward normal.
 *
 * We need two vectors tangent to the sphere surface so we can step
 * along them for the central difference normal computation.
 *
 * Method: pick an arbitrary "up" vector that is not parallel to N,
 * then use cross products to build an orthonormal frame.
 *
 *   T = normalize(cross(N, up))     ← tangent
 *   B = cross(N, T)                 ← bitangent (already unit length)
 *
 * The choice of "up" does not matter for correctness — it only affects
 * the orientation of the tangent frame, not the resulting normal.
 * We avoid (0,1,0) when N is nearly vertical to prevent degenerate cross.
 */
static void make_tangent_basis(Vec3 N, Vec3 *T, Vec3 *B) {
  Vec3 up = (fabsf(N.y) < 0.9f) ? v3(0, 1, 0) : v3(1, 0, 0);
  *T = v3_norm(v3_cross(up, N));
  *B = v3_cross(N, *T); /* already unit since N and T are */
}

/*
 * displaced_normal — recompute the surface normal at `pos` after
 * the displacement field f has deformed the surface, using central
 * differences in the surface's tangent plane.
 *
 *   1. Build a tangent frame (T, B) at pos.
 *   2. Sample f at four neighbours: pos ± εT and pos ± εB.
 *   3. Reconstruct the displaced tangent vectors:
 *        T′ = 2εT + N · (f(p+εT) − f(p−εT))
 *        B′ = 2εB + N · (f(p+εB) − f(p−εB))
 *   4. N′ = normalize(T′ × B′).
 *
 * Works for ANY scalar displacement function — no analytic gradient
 * required. ε = CD_EPS (~3 % of radius) is the empirical sweet spot
 * for all four modes here.
 */
static Vec3 displaced_normal(Vec3 pos, Vec3 N, DispFn fn, float time, float amp,
                             float freq) {
  Vec3 T, B;
  make_tangent_basis(N, &T, &B);

  float eps = CD_EPS;

  /* Sample the displacement field at four tangent-plane neighbours. */
  float f_Tp = fn(v3_add(pos, v3_scale(T, eps)), time, amp, freq);
  float f_Tm = fn(v3_add(pos, v3_scale(T, -eps)), time, amp, freq);
  float f_Bp = fn(v3_add(pos, v3_scale(B, eps)), time, amp, freq);
  float f_Bm = fn(v3_add(pos, v3_scale(B, -eps)), time, amp, freq);

  /* Central differences — how much the surface rises/falls along each. */
  float df_T = f_Tp - f_Tm;
  float df_B = f_Bp - f_Bm;

  /* Reconstruct the deformed tangent vectors. */
  Vec3 T_disp = v3_add(v3_scale(T, 2.f * eps), v3_scale(N, df_T));
  Vec3 B_disp = v3_add(v3_scale(B, 2.f * eps), v3_scale(N, df_B));

  return v3_norm(v3_cross(T_disp, B_disp));
}

/* ── §4 shaders — VS/FS types, uniforms, vertex pass, four frag passes ─ */

typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
} VSIn;

typedef struct {
  Vec4 clip_pos;
  Vec3 world_pos;
  Vec3 world_nrm;
  float u, v;
  float custom[4];
} VSOut;

typedef struct {
  Vec3 world_pos;
  Vec3 world_nrm;
  float u, v;
  float custom[4];
  int px, py;
} FSIn;

typedef struct {
  Vec3 color;
  bool discard;
} FSOut;

typedef void (*VertShaderFn)(const VSIn *, VSOut *, const void *);
typedef void (*FragShaderFn)(const FSIn *, FSOut *, const void *);

/*
 * ShaderProgram — separate uniform pointers for vertex and fragment.
 *
 * WHY SPLIT:
 *   vert_displace always needs DisplaceUniforms (disp_fn, time, amp, freq).
 *   frag_toon needs ToonUniforms (bands).  These are different structs.
 *   A single void* uniforms cannot satisfy both simultaneously without
 *   casting to the wrong type — which is exactly what caused the crash.
 *   Separate pointers cost one extra pointer and fix the entire problem.
 */
typedef struct {
  VertShaderFn vert;
  FragShaderFn frag;
  const void *vert_uni; /* passed to vert() — always &scene.disp_uni */
  const void *frag_uni; /* passed to frag() — &disp_uni or &toon_uni */
} ShaderProgram;

/* ── uniforms ─────────────────────────────────────────────────────── */

typedef struct {
  Mat4 model, view, proj, mvp, norm_mat;
  Vec3 light_pos, light_col, ambient, cam_pos, obj_color;
  float shininess;
} Uniforms;

typedef struct {
  Uniforms base;
  int bands;
} ToonUniforms;

/*
 * DisplaceUniforms — extends Uniforms with displacement parameters.
 *
 * Leading with Uniforms base so &disp_uni casts cleanly to
 * const Uniforms* inside vert_default and all fragment shaders.
 *
 * disp_fn    — pointer to active displacement function
 * time       — seconds since start, updated every frame
 * amplitude  — displacement magnitude (fraction of sphere radius)
 * frequency  — spatial frequency of the wave pattern
 * mode       — active DispMode (for display only)
 */
typedef struct {
  Uniforms base;
  DispFn disp_fn;
  float time;
  float amplitude;
  float frequency;
  DispMode mode;
} DisplaceUniforms;

/* ── vertex shaders ──────────────────────────────────────────────── */

/*
 * vert_displace — the heart of the demo. Reads like its formula:
 *
 *   N    = normalize(pos)                         // sphere normal
 *   d    = disp_fn(pos, time, amp, freq)          // scalar offset
 *   p′   = pos + N · d                            // displaced pos
 *   N′   = displaced_normal(pos, N, ...)          // central diff
 *   clip = MVP · (p′, 1)                          // for raster
 *   wpos = model    · p′                          // for FS lighting
 *   wnrm = norm_mat · N′                          // for FS lighting
 *
 * The fragment shaders never learn the displacement happened —
 * they only see whatever world_pos / world_nrm arrived. That's why
 * all four FSes (phong / toon / normals / wire) work unchanged.
 */
static void vert_displace(const VSIn *in, VSOut *out, const void *u_) {
  const DisplaceUniforms *du = (const DisplaceUniforms *)u_;
  const Uniforms *u = &du->base;

  Vec3 N = v3_norm(in->pos); /* sphere normal     */
  float d = du->disp_fn(in->pos, du->time, du->amplitude, du->frequency);
  Vec3 dpos = v3_add(in->pos, v3_scale(N, d)); /* displaced pos     */
  Vec3 dnrm = displaced_normal(in->pos, N, du->disp_fn, du->time, du->amplitude,
                               du->frequency);

  out->clip_pos = m4_mul_v4(u->mvp, v4(dpos.x, dpos.y, dpos.z, 1.f));
  out->world_pos = m4_pt(u->model, dpos);
  out->world_nrm = v3_norm(m4_dir(u->norm_mat, dnrm));

  out->u = in->u;
  out->v = in->v;
  out->custom[0] = out->custom[1] = out->custom[2] = out->custom[3] = 0.f;
}

/* Variant that also packs the world normal into custom[0..2] so
 * frag_normals can read it directly without barycentric loss. */
static void vert_displace_normals(const VSIn *in, VSOut *out, const void *u_) {
  vert_displace(in, out, u_);
  out->custom[0] = out->world_nrm.x;
  out->custom[1] = out->world_nrm.y;
  out->custom[2] = out->world_nrm.z;
}

/* Variant that leaves custom[0..2] free; the pipeline writes
 * barycentric coordinates there for wireframe edge detection. */
static void vert_displace_wire(const VSIn *in, VSOut *out, const void *u_) {
  vert_displace(in, out, u_);
}

/* ── fragment shaders ────────────────────────────────────────────── */

/*
 * frag_phong — Blinn-Phong + gamma.
 * On the displaced sphere the highlight dances across the waves
 * and spikes, making the deformation clearly visible even on the
 * lit side of the sphere.
 */
static void frag_phong(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));
  float diff = fmaxf(0.f, v3_dot(N, L));
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess);
  Vec3 c = u->obj_color;
  float r = u->ambient.x + c.x * u->light_col.x * diff + spec * 0.5f;
  float g = u->ambient.y + c.y * u->light_col.y * diff + spec * 0.5f;
  float b = u->ambient.z + c.z * u->light_col.z * diff + spec * 0.5f;
  out->color.x = powf(fminf(r, 1.f), 1.f / 2.2f);
  out->color.y = powf(fminf(g, 1.f), 1.f / 2.2f);
  out->color.z = powf(fminf(b, 1.f), 1.f / 2.2f);
  out->discard = false;
}

/*
 * frag_toon — 4-band quantised Phong.
 * On the displaced sphere the band boundaries follow the wave
 * crests and troughs — the toon shading reacts to the geometry,
 * not just the overall sphere curvature.
 */
static void frag_toon(const FSIn *in, FSOut *out, const void *u_) {
  const ToonUniforms *tu = (const ToonUniforms *)u_;
  const Uniforms *u = &tu->base;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));
  float diff = fmaxf(0.f, v3_dot(N, L));
  float banded = floorf(diff * (float)tu->bands) / (float)tu->bands;
  float spec = (v3_dot(N, H) > 0.94f) ? 0.7f : 0.f;
  Vec3 c = u->obj_color;
  out->color.x = fminf(c.x * (banded + 0.12f) + spec, 1.f);
  out->color.y = fminf(c.y * (banded + 0.12f) + spec, 1.f);
  out->color.z = fminf(c.z * (banded + 0.12f) + spec, 1.f);
  out->discard = false;
}

/*
 * frag_normals — world normal → RGB.
 * On the displaced sphere this shows the recomputed deformed normals
 * directly — the wave crests appear as rotating hue bands, making
 * the central difference calculation visually verifiable.
 */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  Vec3 N = v3_norm(in->world_nrm);
  out->color = v3(N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f);
  out->discard = false;
}

/*
 * frag_wire — barycentric edge detection.
 * On the displaced sphere the wireframe shows the UV grid deformed
 * by the displacement — latitude lines ripple in and out, the spiky
 * mode makes the grid look like a porcupine.
 */
static void frag_wire(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  float edge = fminf(in->custom[0], fminf(in->custom[1], in->custom[2]));
  if (edge > WIRE_THRESH) {
    out->discard = true;
    return;
  }
  float t = edge / WIRE_THRESH;
  out->color = v3(0.9f - t * 0.3f, 0.9f - t * 0.3f, 0.9f - t * 0.3f);
  out->discard = false;
}

typedef enum { SH_PHONG = 0, SH_TOON, SH_NORMALS, SH_WIRE, SH_COUNT } ShaderIdx;
static const char *k_shader_names[] = {"phong", "toon", "normals", "wire"};

/* ── §5 mesh — UV sphere tessellation ────────────────────────────────── */

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
  int nvert;
  Triangle *tris;
  int ntri;
} Mesh;

static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

/*
 * tessellate_sphere — identical to sphere_raster.c.
 * Normal = normalised position (unit sphere).
 * Pole normals set explicitly to avoid sin(phi)=0 degenerate case.
 * Winding: (r0,r2,r1) and (r1,r2,r3) — CCW from outside.
 */
static Mesh tessellate_sphere(void) {
  int nu = TESS_U, nv = TESS_V;
  float R = SPHERE_R, PI2 = 2.f * PI;
  Mesh m;
  m.verts = malloc((size_t)(nu + 1) * (nv + 1) * sizeof(Vertex));
  m.tris = malloc((size_t)nu * nv * 2 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  for (int j = 0; j <= nv; j++) {
    float v = (float)j / nv;
    float phi = v * PI;
    float sp = sinf(phi), cp = cosf(phi);
    for (int i = 0; i <= nu; i++) {
      float u = (float)i / nu;
      float theta = u * PI2;
      Vec3 pos = v3(R * sp * cosf(theta), R * cp, R * sp * sinf(theta));
      Vec3 nrm =
          (sp < 1e-6f) ? ((j == 0) ? v3(0, 1, 0) : v3(0, -1, 0)) : v3_norm(pos);
      m.verts[m.nvert++] = (Vertex){pos, nrm, u, v};
    }
  }
  for (int j = 0; j < nv; j++) {
    for (int i = 0; i < nu; i++) {
      int r0 = j * (nu + 1) + i, r1 = r0 + 1;
      int r2 = r0 + (nu + 1), r3 = r2 + 1;
      m.tris[m.ntri++] = (Triangle){{r0, r2, r1}};
      m.tris[m.ntri++] = (Triangle){{r1, r2, r3}};
    }
  }
  return m;
}

/* ── §6 framebuffer — zbuf + cbuf + Bourke ramp + dither + blit ──────── */

typedef struct {
  char ch;
  int color_pair;
  bool bold;
} Cell;
typedef struct {
  float *zbuf;
  Cell *cbuf;
  int cols, rows;
} Framebuffer;

static void fb_alloc(Framebuffer *fb, int c, int r) {
  fb->cols = c;
  fb->rows = r;
  fb->zbuf = malloc((size_t)(c * r) * sizeof(float));
  fb->cbuf = malloc((size_t)(c * r) * sizeof(Cell));
}
static void fb_free(Framebuffer *fb) {
  free(fb->zbuf);
  free(fb->cbuf);
  *fb = (Framebuffer){0};
}
static void fb_clear(Framebuffer *fb) {
  for (int i = 0; i < fb->cols * fb->rows; i++)
    fb->zbuf[i] = FLT_MAX;
  memset(fb->cbuf, 0, (size_t)(fb->cols * fb->rows) * sizeof(Cell));
}

static void color_init(void) {
  start_color();
  if (COLORS >= 256) {
    init_pair(1, 196, COLOR_BLACK);
    init_pair(2, 208, COLOR_BLACK);
    init_pair(3, 226, COLOR_BLACK);
    init_pair(4, 46, COLOR_BLACK);
    init_pair(5, 51, COLOR_BLACK);
    init_pair(6, 33, COLOR_BLACK);
    init_pair(7, 201, COLOR_BLACK);
  } else {
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    init_pair(4, COLOR_GREEN, COLOR_BLACK);
    init_pair(5, COLOR_CYAN, COLOR_BLACK);
    init_pair(6, COLOR_BLUE, COLOR_BLACK);
    init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
  }
}

static int hue_to_pair(Vec3 c) {
  float mx = fmaxf(c.x, fmaxf(c.y, c.z));
  float mn = fminf(c.x, fminf(c.y, c.z));
  float chroma = mx - mn;
  if (chroma < 0.08f)
    return -1;
  float h;
  if (mx == c.x)
    h = 60.f * fmodf((c.y - c.z) / chroma, 6.f);
  else if (mx == c.y)
    h = 60.f * ((c.z - c.x) / chroma + 2.f);
  else
    h = 60.f * ((c.x - c.y) / chroma + 4.f);
  if (h < 0.f)
    h += 360.f;
  static const float pal[7] = {0.f, 30.f, 60.f, 120.f, 180.f, 240.f, 300.f};
  int best = 0;
  float bd = 1e9f;
  for (int i = 0; i < 7; i++) {
    float d = fabsf(h - pal[i]);
    if (d > 180.f)
      d = 360.f - d;
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return best + 1;
}

/* rgb_to_cell — GLYPH from luminance (Bayer-dithered Bourke ramp); COLOUR PAIR
 * from the fragment HUE so the normals shader reads as a rainbow, distinct from
 * the material-coloured phong shader. Desaturated fragments (wireframe) fall
 * back to the luma ramp. (Was luma-only, which made normals look like phong.) */
static Cell rgb_to_cell(Vec3 col, int px, int py) {
  float luma = 0.2126f * col.x + 0.7152f * col.y + 0.0722f * col.z;
  float d = luma + (k_bayer[py & 3][px & 3] - 0.5f) * 0.15f;
  d = d < 0.f ? 0.f : d > 1.f ? 1.f : d;
  int idx = (int)(d * (BOURKE_LEN - 1));
  int cp = hue_to_pair(col);
  if (cp < 0) {
    cp = 1 + (int)(d * 6.f);
    if (cp > 7)
      cp = 7;
  }
  return (Cell){k_bourke[idx], cp, d > 0.6f};
}

static void fb_blit(const Framebuffer *fb) {
  for (int y = 0; y < fb->rows; y++) {
    for (int x = 0; x < fb->cols; x++) {
      Cell c = fb->cbuf[y * fb->cols + x];
      if (!c.ch)
        continue;
      attr_t a = COLOR_PAIR(c.color_pair) | (c.bold ? A_BOLD : 0);
      attron(a);
      mvaddch(y, x, (chtype)(unsigned char)c.ch);
      attroff(a);
    }
  }
}

/* ── §7 pipeline — vertex transform → cull → barycentric raster → FS ── */

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

static void pipeline_draw_mesh(Framebuffer *fb, const Mesh *mesh,
                               ShaderProgram *sh, bool is_wire,
                               bool cull_backface) {
  int cols = fb->cols, rows = fb->rows;
  static const float wu[3] = {1.f, 0.f, 0.f};
  static const float wv[3] = {0.f, 1.f, 0.f};

  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];
    VSOut vo[3];
    for (int vi = 0; vi < 3; vi++) {
      const Vertex *vtx = &mesh->verts[tri->v[vi]];
      VSIn in;
      in.pos = vtx->pos;
      in.normal = vtx->normal;
      in.u = is_wire ? wu[vi] : vtx->u;
      in.v = is_wire ? wv[vi] : vtx->v;
      memset(&vo[vi], 0, sizeof vo[vi]);
      sh->vert(&in, &vo[vi], sh->vert_uni);
      if (is_wire) {
        vo[vi].custom[0] = wu[vi];
        vo[vi].custom[1] = wv[vi];
        vo[vi].custom[2] = 1.f - wu[vi] - wv[vi];
      }
    }

    if (vo[0].clip_pos.w < 0.001f && vo[1].clip_pos.w < 0.001f &&
        vo[2].clip_pos.w < 0.001f)
      continue;

    float sx[3], sy[3], sz[3];
    for (int vi = 0; vi < 3; vi++) {
      float w = vo[vi].clip_pos.w;
      if (fabsf(w) < 1e-6f)
        w = 1e-6f;
      sx[vi] = (vo[vi].clip_pos.x / w + 1.f) * 0.5f * (float)cols;
      sy[vi] = (-vo[vi].clip_pos.y / w + 1.f) * 0.5f * (float)rows;
      sz[vi] = vo[vi].clip_pos.z / w;
    }

    float area =
        (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
    if (cull_backface && area <= 0.f)
      continue;

    int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

    for (int py = y0; py <= y1; py++) {
      for (int px = x0; px <= x1; px++) {
        float b[3];
        barycentric(sx, sy, px + .5f, py + .5f, b);
        if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
          continue;

        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        int idx = py * cols + px;
        if (z >= fb->zbuf[idx])
          continue;
        fb->zbuf[idx] = z;

        FSIn fsin;
        fsin.world_pos = v3_bary(vo[0].world_pos, vo[1].world_pos,
                                 vo[2].world_pos, b[0], b[1], b[2]);
        fsin.world_nrm = v3_norm(v3_bary(vo[0].world_nrm, vo[1].world_nrm,
                                         vo[2].world_nrm, b[0], b[1], b[2]));
        fsin.u = b[0] * vo[0].u + b[1] * vo[1].u + b[2] * vo[2].u;
        fsin.v = b[0] * vo[0].v + b[1] * vo[1].v + b[2] * vo[2].v;
        fsin.px = px;
        fsin.py = py;
        for (int c = 0; c < 4; c++)
          fsin.custom[c] = b[0] * vo[0].custom[c] + b[1] * vo[1].custom[c] +
                           b[2] * vo[2].custom[c];

        FSOut fsout;
        fsout.discard = false;
        sh->frag(&fsin, &fsout, sh->frag_uni);
        if (fsout.discard)
          continue;

        fb->cbuf[idx] = rgb_to_cell(fsout.color, px, py);
      }
    }
  }
}

/* ── §8 scene — uniforms wiring, tick, draw, mode/shader swap ────────── */

/*
 * Per-mode amplitude and frequency tuning.
 * Each mode looks best at different scales — these are the values
 * that make each mode look visually distinct and dramatic.
 *
 * amp  — fraction of sphere radius to displace
 * freq — spatial frequency of the wave pattern
 */
static const float k_amp[DM_COUNT] = {0.22f, 0.18f, 0.30f, 0.35f};
static const float k_freq[DM_COUNT] = {8.0f, 5.0f, 4.0f, 4.5f};

typedef struct {
  Mesh mesh;
  float angle_x, angle_y;
  float cam_dist;
  bool paused;
  bool cull_backface;
  ShaderIdx shade_idx;
  DispMode disp_idx;
  ShaderProgram shader;
  Uniforms uni;
  ToonUniforms toon_uni;
  DisplaceUniforms disp_uni;
  float time;
} Scene;

static void scene_build_shader(Scene *s) {
  /*
   * vert_uni is ALWAYS &disp_uni — every vertex shader variant is a
   * form of vert_displace and needs DisplaceUniforms to call disp_fn.
   *
   * frag_uni points to the struct the fragment shader actually needs:
   *   frag_phong   → Uniforms*    — cast from &disp_uni (base is first member)
   *   frag_toon    → ToonUniforms* — needs bands field
   *   frag_normals → unused       — (void)u_ so either pointer is fine
   *   frag_wire    → unused       — (void)u_ so either pointer is fine
   *
   * DisplaceUniforms leads with Uniforms base so &disp_uni casts cleanly
   * to const Uniforms* inside frag_phong — same zero-offset rule as before.
   */
  switch (s->shade_idx) {
  case SH_PHONG:
    s->shader.vert = vert_displace;
    s->shader.frag = frag_phong;
    s->shader.vert_uni = &s->disp_uni;
    s->shader.frag_uni = &s->disp_uni; /* Uniforms* cast from Disp* */
    break;
  case SH_TOON:
    s->toon_uni.base = s->disp_uni.base;
    s->toon_uni.bands = 4;
    s->shader.vert = vert_displace;
    s->shader.frag = frag_toon;
    s->shader.vert_uni = &s->disp_uni; /* vert needs disp params    */
    s->shader.frag_uni = &s->toon_uni; /* frag needs bands          */
    break;
  case SH_NORMALS:
    s->shader.vert = vert_displace_normals;
    s->shader.frag = frag_normals;
    s->shader.vert_uni = &s->disp_uni;
    s->shader.frag_uni = &s->disp_uni;
    break;
  case SH_WIRE:
    s->shader.vert = vert_displace_wire;
    s->shader.frag = frag_wire;
    s->shader.vert_uni = &s->disp_uni;
    s->shader.frag_uni = &s->disp_uni;
    break;
  default:
    break;
  }
}

static void scene_sync_disp(Scene *s) {
  /*
   * Copy current base uniforms into disp_uni and update displacement
   * params.  Called every frame from scene_tick so time, mvp, and
   * norm_mat stay current.
   *
   * toon_uni.base is updated in scene_build_shader when the toon
   * shader is selected, and again here every frame so the lighting
   * matrices stay current when the toon shader is active.
   */
  s->disp_uni.base = s->uni;
  s->disp_uni.disp_fn = k_disp_fn[s->disp_idx];
  s->disp_uni.time = s->time;
  s->disp_uni.amplitude = k_amp[s->disp_idx];
  s->disp_uni.frequency = k_freq[s->disp_idx];
  s->disp_uni.mode = s->disp_idx;

  /* keep toon base matrices current if toon shader is active */
  if (s->shade_idx == SH_TOON)
    s->toon_uni.base = s->uni;
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->mesh = tessellate_sphere();
  s->shade_idx = SH_PHONG;
  s->disp_idx = DM_RIPPLE;
  s->cam_dist = CAM_DIST;
  s->cull_backface = true;
  s->time = 0.f;

  s->uni.light_pos = v3(4.f, 5.f, 3.f);
  s->uni.light_col = v3(1.f, 1.f, 1.f);
  s->uni.ambient = v3(0.06f, 0.06f, 0.06f);
  s->uni.shininess = 60.f;
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.obj_color = v3(0.2f, 0.7f, 0.95f); /* ocean blue */

  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);

  scene_sync_disp(s);
  scene_build_shader(s);
}

static void scene_set_zoom(Scene *s) {
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
}

static void scene_rebuild_proj(Scene *s, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void scene_tick(Scene *s, float dt) {
  if (!s->paused) {
    s->time += dt;
    s->angle_y += ROT_Y * dt;
    s->angle_x += ROT_X * dt;
  }
  Mat4 ry = m4_rotate_y(s->angle_y);
  Mat4 rx = m4_rotate_x(s->angle_x);
  s->uni.model = m4_mul(ry, rx);
  s->uni.mvp = m4_mul(s->uni.proj, m4_mul(s->uni.view, s->uni.model));
  s->uni.norm_mat = m4_normal_mat(s->uni.model);
  scene_sync_disp(s);
  /*
   * No shader.uniforms fixup here — vert_uni/frag_uni are set once in
   * scene_build_shader and remain valid for the lifetime of that shader
   * selection.  scene_sync_disp keeps the pointed-to structs current.
   */
}

static void scene_draw(Scene *s, Framebuffer *fb) {
  fb_clear(fb);
  pipeline_draw_mesh(fb, &s->mesh, &s->shader, (s->shade_idx == SH_WIRE),
                     s->cull_backface);
  fb_blit(fb);
}

static void scene_next_shader(Scene *s) {
  s->shade_idx = (ShaderIdx)((s->shade_idx + 1) % SH_COUNT);
  scene_build_shader(s);
}

static void scene_next_disp(Scene *s) {
  s->disp_idx = (DispMode)((s->disp_idx + 1) % DM_COUNT);
  scene_sync_disp(s);
  scene_build_shader(s);
}

/* ── §9 screen — ncurses init / resize / HUD / present ───────────────── */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s) {
  (void)s;
  endwin();
}
static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* HUD layout (CLAUDE.md spec):
 *   row 0          PAIR_HUD (yellow + bold)  — title left, status right
 *   row rows-1     PAIR_HINT (cyan + bold)   — key hint */
#define PAIR_HUD 3  /* yellow — see color_init() palette        */
#define PAIR_HINT 5 /* cyan                                     */

static void screen_draw_hud(const Screen *s, const Scene *sc, double fps) {
  /* Right-aligned status. */
  char status[HUD_COLS + 1];
  snprintf(status, sizeof status,
           " %5.1f fps  disp:%s  shader:%s  zoom:%.1f  cull:%s%s ", fps,
           k_disp_names[sc->disp_idx], k_shader_names[sc->shade_idx],
           sc->cam_dist, sc->cull_backface ? "on " : "off",
           sc->paused ? " PAUSED" : "");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " DISPLACE · UV SPHERE ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  d:disp  s:shader  c:cull  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §10 app — main loop, signals, resize, cleanup ───────────────────── */

typedef struct {
  Scene scene;
  Screen screen;
  Framebuffer fb;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;
static void on_exit(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  fb_free(&app->fb);
  fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
  scene_rebuild_proj(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 's':
  case 'S':
    scene_next_shader(s);
    break;
  case 'd':
  case 'D':
    scene_next_disp(s);
    break;
  case 'c':
  case 'C':
    s->cull_backface = !s->cull_backface;
    break;
  case '=':
  case '+':
    s->cam_dist -= CAM_ZOOM_STEP;
    if (s->cam_dist < CAM_DIST_MIN)
      s->cam_dist = CAM_DIST_MIN;
    scene_set_zoom(s);
    break;
  case '-':
    s->cam_dist += CAM_ZOOM_STEP;
    if (s->cam_dist > CAM_DIST_MAX)
      s->cam_dist = CAM_DIST_MAX;
    scene_set_zoom(s);
    break;
  default:
    break;
  }
  return true;
}

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

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit);
  signal(SIGTERM, on_exit);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;

  screen_init(&app->screen);
  fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps_disp = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    scene_tick(&app->scene, dt_sec);

    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    erase();
    scene_draw(&app->scene, &app->fb);
    screen_draw_hud(&app->screen, &app->scene, fps_disp);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);
  }

  mesh_free(&app->scene.mesh);
  fb_free(&app->fb);
  screen_free(&app->screen);
  return 0;
}
