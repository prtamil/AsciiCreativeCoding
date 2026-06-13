/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sphere_raster.c — software rasteriser of a smooth-shaded UV sphere
 *
 * DEMO: A blue UV-tessellated sphere rotates slowly under a single
 *       point light, demonstrating four classic real-time shaders on
 *       the canonical smooth-surface primitive. Cycle 's' through:
 *
 *         phong      Blinn-Phong with a sharp specular highlight.
 *                    The highlight glides across the sphere as it
 *                    spins — proof that smooth normals are working.
 *         toon       4-band quantised diffuse + hard specular.
 *                    Concentric latitudinal rings track the light.
 *         normals    World normal RGB visualisation. The full RGB
 *                    cube wraps once around the sphere — every
 *                    surface direction gets a unique hue.
 *         wireframe  Barycentric edge detection. Reveals the UV grid:
 *                    TESS_U meridians × TESS_V parallels, with the
 *                    iconic pole-fan singularity.
 *
 *       Toggle 'c' to disable back-face culling — the inner surface
 *       becomes visible and you can see triangle winding from below.
 *
 *       Pipeline is identical to cube_raster.c — only §1 config and
 *       §4 tessellation differ. The sphere is the best showcase for
 *       the pipeline because every shader has something distinct to
 *       say about a continuously curved surface.
 *
 * Study alongside:
 *   raster/cube_raster.c     — same pipeline, single mesh primitive
 *   raster/torus_raster.c    — same pipeline, parametric torus
 *   raster/displace_raster.c — same UV sphere, animated displacement
 *
 * Section map:
 *   §1  config       — frame, view, sphere, tessellation, ramp, dither
 *   §2  math         — V3 / V4 / Mat4 helpers
 *   §3  shaders      — VS / FS types + uniforms + 1 base VS + 4 FS
 *   §4  mesh         — UV sphere tessellation (smooth normals + poles)
 *   §5  framebuffer  — zbuf + cbuf + Bourke ramp + Bayer dither + blit
 *   §6  pipeline     — vertex transform → cull → barycentric raster → FS
 *   §7  scene        — Scene struct, uniforms wiring, tick, draw, swap
 *   §8  screen       — ncurses init / resize / HUD / present
 *   §9  app          — main loop, signals, resize, cleanup
 *
 * Keys:
 *   s / S     cycle shader  (phong → toon → normals → wireframe)
 *   c / C     toggle back-face culling
 *   + / =     zoom in
 *   -         zoom out
 *   space     pause / resume rotation
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/sphere_raster.c -o sphere \
 *       -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Standard forward rasterisation. Tessellate the
 *                  sphere once at init into a static triangle mesh
 *                  with smooth per-vertex normals (= position / R for
 *                  the unit sphere). Each frame transforms vertices
 *                  through MVP, runs back-face culling, walks each
 *                  triangle's bounding box with barycentric weights,
 *                  z-tests, and runs a fragment shader. The sphere's
 *                  smooth normals + barycentric interpolation are
 *                  what produce the continuous Phong highlight.
 *
 *                  UV parameterisation:
 *                    pos = (R·sin φ·cos θ, R·cos φ, R·sin φ·sin θ)
 *                    nrm = pos / R
 *                  φ ∈ [0, π] is latitude (0 = north pole),
 *                  θ ∈ [0, 2π) is longitude.
 *
 * Data-structure : One Mesh = flat arrays of vertices and triangles,
 *                  sized at compile time:
 *                    verts = (TESS_U+1) × (TESS_V+1)
 *                    tris  = TESS_U × TESS_V × 2 (two per quad cell)
 *                  Plus a Framebuffer (zbuf + cbuf) re-allocated on
 *                  each resize. No dynamic state changes per frame
 *                  except rotation matrices.
 *
 * Rendering      : Standard forward pipeline matching cube_raster.c:
 *                    vert shader → near-clip reject → perspective
 *                    divide → screen mapping (with cell-aspect fix)
 *                    → back-face cull → bounding-box raster +
 *                    barycentric interp → frag shader → luma →
 *                    Bayer dither → Bourke ramp glyph + colour pair.
 *
 * Performance    : Tessellation is one-shot at init. Per-frame cost
 *                  is O(N_tris × pixels_per_tri). Smooth normals let
 *                  TESS_U × TESS_V stay modest (36 × 24 = 1728 tris)
 *                  while the surface still looks round — coarse
 *                  tessellation is "hidden" by the smooth shading.
 *
 * References     : Phong, "Illumination for Computer Generated
 *                    Pictures," CACM '75 (the original Phong model).
 *                  Blinn, "Models of Light Reflection for Computer
 *                    Synthesised Pictures," SIGGRAPH '77 (Blinn-Phong).
 *                  Möller, "Fast Triangle Rasterization by
 *                    Interpolating Edge Functions," GPG (2000).
 *                  RTRender 4ed, §16.1 (sphere tessellation
 *                    schemes — UV vs icosphere comparison).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A sphere in the terminal is a CHAIN of transforms ending in luma.
 * Tessellate once into a UV grid, then per frame: every vertex →
 * MVP transform → screen-space coordinates; every triangle → bbox
 * raster with barycentric weights; every covered cell → a fragment
 * shader → an RGB colour → a luma → a Bayer-dithered Bourke glyph.
 * Four shaders demonstrate the pipeline isn't sphere-specific —
 * only §4 changes if you swap the mesh.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Peel a globe into a flat rectangular grid (latitude × longitude),
 * then re-paste it in 3-D using (sin φ cos θ, cos φ, sin φ sin θ).
 * That's the mesh. The camera takes a 3-D photograph, but instead
 * of film the negative is a Z-buffered grid of cells: nearer
 * fragments win, brighter fragments pick a denser glyph, with a
 * bit of ordered noise (Bayer) so smooth gradients don't band.
 * The four shaders are four "exposure recipes" on the same negative.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │  vertex shader        rasteriser             │
 *      │      ▾                     ▾                 │
 *      │  pos → MVP → clip → /w → screen              │
 *      │                            ▾                 │
 *      │  tri  → bbox → bary → z-test → fragment      │
 *      │                                  ▾           │
 *      │  RGB → luma → bayer → bourke glyph + pair    │
 *      └──────────────────────────────────────────────┘
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 *  1. scene_tick — angle_x, angle_y += ROT · dt; build
 *       model = Ry · Rx; mvp = proj · view · model;
 *       norm_mat = cofactor(model 3×3).
 *
 *  2. fb_clear — zbuf := FLT_MAX, cbuf := all-zero cells.
 *
 *  3. pipeline_draw_mesh — per triangle:
 *        a. Run vert shader on each of the 3 vertices → VSOut
 *           (clip_pos + world_pos + world_nrm + custom[]).
 *        b. Reject if all 3 clip.w < ε (behind near plane).
 *        c. Perspective divide: sx = (clip.x/w + 1) · cols/2,
 *                              sy = (−clip.y/w + 1) · rows/2,
 *                              sz = clip.z / w.
 *        d. Back-face cull if signed area ≤ 0 (CCW front).
 *        e. Walk integer bbox; per cell compute barycentric (b0,b1,b2);
 *           skip if any negative; z-test against zbuf; on win write z.
 *        f. Build FSIn by interpolating world_pos / world_nrm /
 *           u / v / custom[]; run frag shader → FSOut.
 *        g. luma = 0.2126·R + 0.7152·G + 0.0722·B (Rec. 709).
 *        h. luma_to_cell: bayer dither → Bourke glyph + colour pair.
 *
 *  4. fb_blit — walk cbuf, mvaddch each non-empty cell.
 *
 * KEY FORMULAS
 * ────────────
 *   UV parameterisation
 *     pos = (R · sin φ · cos θ,  R · cos φ,  R · sin φ · sin θ)
 *     φ ∈ [0, π], θ ∈ [0, 2π)
 *   Vertex normal       N = pos / R          (unit sphere shortcut)
 *   Pole degeneracy     if sin φ < 1e-6,  N = (0, ±1, 0) explicitly
 *   Mesh sizes          V = (TESS_U+1) · (TESS_V+1)        = 925
 *                       T = TESS_U · TESS_V · 2            = 1728
 *   Perspective divide  sx = ( ndc.x + 1)/2 · cols
 *                       sy = (−ndc.y + 1)/2 · rows         (Y-flip)
 *   Aspect              cols · CELL_W / (rows · CELL_H)
 *   Phong               diff = max(0, N · L)
 *                       spec = max(0, N · H)^shininess
 *                       out  = ambient + obj·light·diff + 0.5·spec
 *   Toon                banded = ⌊diff · bands⌋ / bands     (bands = 4)
 *   Normal viz          RGB = N · 0.5 + 0.5
 *   Wire edge           edge = min(b0, b1, b2);  discard if > WIRE_THRESH
 *   Luma → glyph        d = clamp01(luma + (bayer − 0.5) · 0.15)
 *                       glyph = k_bourke[(int)(d · (BOURKE_LEN−1))]
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Pole singularity. sin φ = 0 makes the normalised position
 *     degenerate. tessellate_sphere overrides the normal to
 *     (0, ±1, 0) at the poles to avoid divide-by-zero.
 *
 *   • Wireframe relies on custom[0..2] = barycentric coords being
 *     written by the pipeline AFTER the vertex shader runs. If you
 *     swap the order, the wire shader silently sees zeros and
 *     discards everything.
 *
 *   • Cell aspect. CELL_W / CELL_H = 8/16 = 0.5 in the projection
 *     aspect ratio — without it the sphere stretches into an egg.
 *
 *   • Sub-pixel triangles. When |signed area| < 1e-6 the
 *     barycentric helper returns -1s; the triangle silently
 *     disappears. Acceptable at the poles where many tiny tris meet.
 *
 *   • Back-face cull works because no negative scale is applied.
 *     Press 'c' to disable and study the inner-surface winding.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Pause (space). Phong: a single bright highlight over the
 *     upper-right hemisphere (light at (3, 4, 3)).
 *   • Toon: four concentric latitudinal bands tracking the light.
 *   • Normals: smooth RGB across the sphere — +X red, +Y green,
 *     +Z blue. The full RGB cube wraps once.
 *   • Wireframe: ~1728 triangle outlines forming a clean lat/long
 *     grid with the iconic pole-fan singularity.
 *   • Toggle 'c': inner surface becomes visible at the silhouette;
 *     watch how triangles wind from the back side.
 *   • Zoom in to CAM_DIST_MIN = 1.0 — the sphere fills (and exceeds)
 *     the screen; the wireframe is still legible up close.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read raster/cube_raster.c FIRST if you don't yet
 *      know the 7-stage raster pipeline; this file inherits it
 *      directly. Sphere only adds tessellation + smooth normals.
 *   2. §1 config — every constant has a unit-bearing comment.
 *   3. §4 mesh — UV sphere tessellation. Read AFTER tutorials
 *      T2-T4. The pole-singularity workaround in §4 is a classic
 *      gotcha worth understanding.
 *   4. §6 pipeline — same as cube_raster.c. Skim if already familiar.
 *   5. §3 shaders — reuses cube_raster.c's four FS pairs, adapted
 *      for smooth-normal interpolation. Read AFTER T6.
 *   6. §5 / §7 / §8 / §9 — infrastructure; skip on first read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Same as cube_raster.c (V3 / V4 / Mat4 / mvp / norm_mat / sx / sy /
 *   b0/b1/b2 / zbuf / cbuf / VSIn / VSOut / FSIn).
 *   Sphere-specific:
 *     TESS_U                   number of longitude segments (meridians)
 *     TESS_V                   number of latitude segments (parallels)
 *     phi  (φ)                 latitude angle ∈ [0, π]; 0 = north pole
 *     theta (θ)                longitude angle ∈ [0, 2π)
 *     ring_idx, slice_idx      grid coordinates in the UV tessellation
 *
 * Background you need
 * ───────────────────
 *   - cube_raster.c's pipeline (read it first if unfamiliar).
 *   - Spherical coordinates: (radius, latitude, longitude) → (x, y, z).
 *   - Why a unit-sphere normal equals its position vector.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Icosphere subdivision (we use simpler UV grid; T5 explains).
 *   - Geodesic / projection distortion (only matters for textures;
 *     this file uses no texture).
 *   - Quaternions for rotation — Euler angles suffice here.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Six short tutorials. Read in order; each builds on the previous.
 *
 *   T1  The shared raster pipeline — reference cube_raster.c
 *   T2  UV-sphere parameterisation — lat/lon → 3-D point
 *   T3  Smooth normals — why N = pos / R for a unit sphere
 *   T4  The pole singularity — sin(0) = 0 problem and the workaround
 *   T5  UV-sphere vs icosphere — why the simpler grid wins here
 *   T6  Same four shaders as cube — what changes with smooth normals
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  THE SHARED RASTER PIPELINE
 * ───────────────────────────────
 * This file implements the SAME 7-stage pipeline as cube_raster.c:
 * vertex shader → perspective divide → screen mapping → back-face
 * cull → bounding-box raster + barycentric → z-test → fragment
 * shader. Read cube_raster.c tutorials T1-T6 for the pipeline; here
 * we focus only on what's SPHERE-specific.
 *
 * What changes between cube and sphere:
 *
 *   §4 mesh tessellation            cube: 24 verts / 12 tris (hand-listed)
 *                                   sphere: 925 verts / 1728 tris
 *                                           (procedurally tessellated)
 *
 *   §3 vertex normals               cube: face-normal duplicated to 4
 *                                         vertices per face → flat shading
 *                                   sphere: each vertex gets its own
 *                                           outward-pointing normal →
 *                                           smooth shading via interpolation
 *
 * Everything else (pipeline, shaders, framebuffer, paint, scene,
 * screen, app) is identical. The same FS function pointers (phong /
 * toon / normals / wire) plug into either mesh without modification.
 *
 * T2  UV-SPHERE PARAMETERISATION — LAT/LON → 3-D POINT
 * ─────────────────────────────────────────────────────
 * A UV sphere is a SPHERICAL-COORDINATES grid:
 *
 *     for ring (latitude)  in 0 .. TESS_V:
 *       φ = π · ring / TESS_V             ∈ [0, π]
 *       for slice (longitude) in 0 .. TESS_U:
 *         θ = 2π · slice / TESS_U          ∈ [0, 2π)
 *         pos.x = R · sin(φ) · cos(θ)
 *         pos.y = R · cos(φ)
 *         pos.z = R · sin(φ) · sin(θ)
 *
 * φ (phi) is LATITUDE measured from the +Y axis (north pole). At
 * φ = 0 we're at the north pole (pos = (0, R, 0)); at φ = π we're
 * at the south pole. cos(φ) sweeps from +1 to -1, giving the y
 * coordinate; sin(φ) is the radius of the latitude circle in the
 * XZ plane.
 *
 * θ (theta) is LONGITUDE around the Y axis. Parameterises the
 * latitude circle: cos(θ) for x, sin(θ) for z.
 *
 * Each (ring, slice) cell of the grid corresponds to a 2-triangle
 * QUAD on the sphere's surface. For TESS_U=36, TESS_V=24:
 *
 *     vertices = (TESS_U+1) · (TESS_V+1) = 37 · 25 = 925
 *     triangles = TESS_U · TESS_V · 2     = 36 · 24 · 2 = 1728
 *
 * The +1 on each grid axis is the SEAM repetition: the slice at
 * θ = 0 is duplicated at θ = 2π so triangles can index it without
 * a modular wrap. Same for the poles.
 *
 * Read §4 tessellate_sphere — the implementation is ~30 lines.
 *
 * T3  SMOOTH NORMALS — WHY N = POS / R FOR A UNIT SPHERE
 * ───────────────────────────────────────────────────────
 * The OUTWARD NORMAL at any point on a sphere centred at origin
 * with radius R is simply the position vector divided by R:
 *
 *     N = pos / R     (which is just `pos` if R = 1)
 *
 * Why: a sphere is the locus of points equidistant from the centre.
 * The surface tangent at any point is perpendicular to the radius
 * vector (which is `pos - centre = pos` for centre at origin).
 * Therefore the outward normal IS the radius direction.
 *
 * For our unit sphere (R = 1):
 *
 *     N = pos                              (unit-length already)
 *
 * Per-vertex smooth normals are what produce the continuous Phong
 * highlight that GLIDES across the sphere as it rotates. Compare to
 * cube_raster.c where each face has 4 vertices with the SAME flat
 * normal — there the highlight has hard edges at every face boundary.
 *
 * Smoothness is INTERPOLATED by the rasteriser. At each pixel inside
 * a triangle:
 *
 *     N_pixel = b0 · N_v0 + b1 · N_v1 + b2 · N_v2
 *
 * (linear interpolation via barycentric weights — see cube_raster.c
 * T5). Because the three vertex normals point slightly differently,
 * the interpolated normal varies smoothly across the triangle —
 * letting the FS produce a continuous gradient.
 *
 * Subtle but important: the interpolated normal isn't unit-length
 * after interpolation. Most FS code re-normalises before lighting:
 *
 *     N = normalize(N_pixel)
 *
 * T4  THE POLE SINGULARITY — SIN(0) = 0 AND THE WORKAROUND
 * ─────────────────────────────────────────────────────────
 * At the poles (φ = 0 or φ = π), sin(φ) = 0. The position formula
 * collapses:
 *
 *     pos.x = R · 0 · cos(θ) = 0
 *     pos.y = R · cos(φ)     = ±R
 *     pos.z = R · 0 · sin(θ) = 0
 *
 * So all TESS_U+1 vertices around the pole map to the same
 * (0, ±R, 0). Geometrically that's correct (the pole is one point).
 * BUT the formula N = pos / R becomes degenerate at the pole if you
 * try to compute it from a non-pole position approaching zero —
 * floating-point rounding can produce undefined directions.
 *
 * Workaround in §4 tessellate_sphere:
 *
 *     if sin(φ) < 1e-6:
 *       N = (0, +1, 0)  if φ ≈ 0  (north pole)
 *       N = (0, -1, 0)  if φ ≈ π  (south pole)
 *     else:
 *       N = pos / R     (the normal formula, well-defined)
 *
 * This explicit override avoids divide-by-near-zero NaN that would
 * propagate through the FS and produce a black hole at the pole.
 *
 * Visually, the wireframe shader makes the singularity OBVIOUS:
 * many tiny triangles fan out from the pole, all sharing the same
 * pole vertex. That's the famous "pole pinch" of UV spheres.
 *
 * T5  UV-SPHERE VS ICOSPHERE — WHY THE SIMPLER GRID WINS HERE
 * ────────────────────────────────────────────────────────────
 * Two ways to tessellate a sphere:
 *
 *   UV-SPHERE        Lat/lon grid. Parametric, easy to compute,
 *                    O(TESS_U · TESS_V) triangles. Suffers from pole
 *                    singularity and uneven triangle sizes (huge
 *                    near equator, tiny near poles).
 *
 *   ICOSPHERE        Subdivide an icosahedron. Uniform triangle
 *                    sizes, no pole singularity, no UV seam. But
 *                    requires recursive subdivision logic and
 *                    indirect indexing.
 *
 * For TEACHING purposes UV-sphere wins:
 *   1. Parametric formula is two lines of code (sin·cos·sin·cos).
 *   2. The pole singularity IS pedagogically interesting — it
 *      teaches about coordinate-system degeneracies.
 *   3. The grid structure makes the wireframe shader produce a
 *      clean lat/lon grid that obviously reads as a sphere.
 *
 * For PRODUCTION (game engines, GPU compute) icosphere is usually
 * preferred because uniform triangles play nicer with most lighting
 * + LOD systems. We're not production; we're learners. UV wins.
 *
 * T6  SAME FOUR SHADERS AS CUBE — WHAT CHANGES WITH SMOOTH NORMALS
 * ────────────────────────────────────────────────────────────────
 * The four FS pairs (phong / toon / normals / wire) are LITERALLY
 * the same code as cube_raster.c. The pipeline is the same too. So
 * what's different on a sphere?
 *
 *   PHONG    Smooth interpolated normals → smooth diffuse gradient
 *            and a SLIDING specular highlight. The highlight glides
 *            across the surface as the sphere rotates — the most
 *            obvious payoff of smooth normals.
 *
 *   TOON     Diffuse banded into 4 levels. Because normals smooth-
 *            interpolate, the bands form CONCENTRIC RINGS centred on
 *            the light direction. (On a cube each face was a single
 *            band; on a sphere each ring corresponds to one band.)
 *
 *   NORMALS  RGB = (N + 1) / 2 visualised. The full RGB cube wraps
 *            once around the sphere — every direction gets a unique
 *            colour. (On a cube, only 6 distinct face colours.)
 *
 *   WIRE     Barycentric edge detection. Reveals the UV grid:
 *            TESS_U meridians × TESS_V parallels, with the iconic
 *            pole fan singularity (T4).
 *
 * Cycle `s` to compare. Press `c` to disable back-face culling and
 * see the inner-surface winding direction.
 *
 * ─────────────────────────────────────────────────────────────────── */

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
};

#define CAM_FOV (55.0f * 3.14159265f / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
#define CAM_DIST 2.6f      /* default camera Z distance */
#define CAM_DIST_MIN 1.0f  /* closest zoom              */
#define CAM_DIST_MAX 8.0f  /* furthest zoom             */
#define CAM_ZOOM_STEP 0.2f /* distance change per keypress */

/*
 * Sphere radius.
 * 1.0 fills the terminal well at CAM_DIST=2.6 with FOV=55°.
 */
#define SPHERE_R 1.0f

/*
 * Tessellation resolution.
 * TESS_U: longitude slices (around the equator).
 * TESS_V: latitude  stacks (pole to pole).
 *
 * Higher = rounder sphere, more triangles, slower fill.
 * At terminal resolution 36×24 is a good balance:
 *   - 36 slices → ~10° per slice, smooth silhouette
 *   - 24 stacks → ~7.5° per stack, smooth top/bottom caps
 *   - Total triangles: 36*24*2 = 1728
 *
 * Wireframe at this resolution shows clear latitude/longitude lines
 * without being too dense to read.
 */
#define TESS_U 36
#define TESS_V 24

/*
 * Rotation speeds.
 * Slow X tilt so the poles are visible but the sphere mostly
 * shows its equatorial band — best view for Phong highlight.
 */
#define ROT_Y 0.50f /* radians / second */
#define ROT_X 0.20f

/*
 * Wireframe edge threshold.
 * Sphere triangles are smaller than cube faces in screen space,
 * so a slightly larger threshold (0.09) keeps lines visible.
 */
#define WIRE_THRESH 0.09f

/* Paul Bourke ASCII density ramp — darkest → brightest */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

/* Bayer 4×4 ordered dither matrix */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};

/*
 * CELL_W / CELL_H — terminal cell aspect ratio correction.
 * Passed to m4_perspective as (cols*CELL_W)/(rows*CELL_H).
 * Without this the sphere appears vertically stretched.
 */
#define CELL_W 8
#define CELL_H 16

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

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
static inline Vec3 v3_neg(Vec3 a) { return v3(-a.x, -a.y, -a.z); }
static inline float v3_dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3_len(Vec3 a) { return sqrtf(v3_dot(a, a)); }
static inline Vec3 v3_norm(Vec3 a) {
  float l = v3_len(a);
  return l > 1e-7f ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}
static inline Vec3 v3_reflect(Vec3 d, Vec3 n) {
  return v3_sub(d, v3_scale(n, 2.f * v3_dot(d, n)));
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

/*
 * m4_normal_mat — cofactor of upper-left 3×3.
 * Correctly transforms normals under non-uniform scale.
 * For a sphere with uniform scale this equals the rotation block,
 * but computing it properly means adding squash/stretch later just works.
 */
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

/* ── §3 shaders — VS/FS types + uniforms + 1 base VS + 4 FS ──────────── */

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

typedef void (*VertShaderFn)(const VSIn *in, VSOut *out, const void *uni);
typedef void (*FragShaderFn)(const FSIn *in, FSOut *out, const void *uni);
typedef struct {
  VertShaderFn vert;
  FragShaderFn frag;
  const void *vert_uni;
  const void *frag_uni;
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

/* ── vertex shaders ──────────────────────────────────────────────── *
 *
 * All three vertex shaders share the same MVP transform — they only
 * differ in what they pack into out->custom[]. vert_base does the
 * common work; vert_normals + vert_wire wrap it and add their per-
 * shader payload. (vert_wire leaves custom[] at zero; the pipeline
 * overwrites it with barycentric coords AFTER this runs.)
 */

/* Common transform: MVP for the rasteriser, world pos + world normal
 * for the fragment stage. Reads like its formula:
 *   clip = MVP · (pos, 1)
 *   wpos = model    · pos
 *   wnrm = norm_mat · nrm,   then normalised
 */
static void vert_base(const VSIn *in, VSOut *out, const Uniforms *u) {
  out->clip_pos = m4_mul_v4(u->mvp, v4(in->pos.x, in->pos.y, in->pos.z, 1.f));
  out->world_pos = m4_pt(u->model, in->pos);
  out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
  out->u = in->u;
  out->v = in->v;
  out->custom[0] = out->custom[1] = out->custom[2] = out->custom[3] = 0.f;
}

static void vert_default(const VSIn *in, VSOut *out, const void *u_) {
  vert_base(in, out, (const Uniforms *)u_);
}

/* Same as default, but additionally packs world normal into
 * custom[0..2] so frag_normals can read it without barycentric
 * loss. */
static void vert_normals(const VSIn *in, VSOut *out, const void *u_) {
  vert_base(in, out, (const Uniforms *)u_);
  out->custom[0] = out->world_nrm.x;
  out->custom[1] = out->world_nrm.y;
  out->custom[2] = out->world_nrm.z;
}

/* Same as default, custom[] left at zero — the pipeline writes
 * barycentric coordinates into custom[0..2] after this runs. */
static void vert_wire(const VSIn *in, VSOut *out, const void *u_) {
  vert_base(in, out, (const Uniforms *)u_);
}

/* ── fragment shaders ────────────────────────────────────────────── */

/*
 * frag_phong — Blinn-Phong shading with gamma correction.
 *
 * The sphere's smooth normals produce a continuous specular lobe —
 * the highlight glides across the surface as it rotates, which is
 * the classic "shiny ball" look.  At terminal resolution the dither
 * LUT gives the highlight a pleasing grain rather than a hard step.
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
 * frag_toon — 4-band quantised diffuse + hard specular.
 *
 * On the sphere the bands form horizontal rings that track the
 * light direction — as the sphere rotates the rings sweep across
 * the surface.  More bands = finer steps; fewer = more graphic.
 * 4 bands at terminal resolution looks clean without being too coarse.
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
 * frag_normals — world normal [-1,1] → RGB [0,1].
 *
 * The sphere is the ideal showcase for this shader: every surface
 * direction is represented, so the full RGB colour cube appears
 * mapped continuously across the surface.  As the sphere rotates
 * the colours shift smoothly — every orientation has a unique hue.
 */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  Vec3 N = v3_norm(in->world_nrm);
  out->color = v3(N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f);
  out->discard = false;
}

/*
 * frag_wire — barycentric edge detection.
 *
 * On the sphere wireframe shows the UV latitude/longitude grid —
 * TESS_V horizontal rings and TESS_U vertical meridians.
 * The poles converge to a fan of triangles rather than a clean ring;
 * this is inherent to UV tessellation and is part of the aesthetic.
 * An icosphere would give uniform triangles but no clean lat/lon lines.
 */
static void frag_wire(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  float b0 = in->custom[0], b1 = in->custom[1], b2 = in->custom[2];
  float edge = fminf(b0, fminf(b1, b2));
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

/* ── §4 mesh — UV sphere tessellation ────────────────────────────────── */

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
 * tessellate_sphere — generate the UV-sphere mesh.
 *
 * Walks an (nu+1) × (nv+1) UV grid:
 *   φ = (j / nv) · π            // latitude:  0 = north pole, π = south
 *   θ = (i / nu) · 2π           // longitude: 0 .. 2π
 *   pos = (R · sin φ · cos θ,  R · cos φ,  R · sin φ · sin θ)
 *   nrm = pos / R               // unit-sphere shortcut
 *
 * Pole handling: when sin φ ≈ 0 the normalised position is degenerate;
 * we override the normal to (0, ±1, 0) so the poles don't get black
 * bands from divide-by-zero.
 *
 * Triangulation: each quad cell (i, j)..(i+1, j+1) emits two CCW
 * triangles (r0, r2, r1) and (r1, r2, r3). The winding is what
 * "outward = front" depends on for back-face culling.
 */
static Mesh tessellate_sphere(void) {
  int nu = TESS_U;
  int nv = TESS_V;
  float R = SPHERE_R;
  float PI = 3.14159265f;
  float PI2 = 2.f * PI;

  int nvert = (nu + 1) * (nv + 1);
  int ntri = nu * nv * 2;

  Mesh m;
  m.verts = malloc((size_t)nvert * sizeof(Vertex));
  m.tris = malloc((size_t)ntri * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  /* Step 1 — emit (nu+1) × (nv+1) vertices on the parameterised sphere. */
  for (int j = 0; j <= nv; j++) {
    float v = (float)j / (float)nv;
    float phi = v * PI; /* latitude  */
    float sp = sinf(phi), cp = cosf(phi);

    for (int i = 0; i <= nu; i++) {
      float u = (float)i / (float)nu;
      float theta = u * PI2; /* longitude */
      float st = sinf(theta), ct = cosf(theta);

      Vec3 pos = v3(R * sp * ct, R * cp, R * sp * st);
      Vec3 nrm = (sp < 1e-6f) /* pole guard */
                     ? ((j == 0) ? v3(0, 1, 0) : v3(0, -1, 0))
                     : v3_norm(pos);

      m.verts[m.nvert++] = (Vertex){pos, nrm, u, v};
    }
  }

  /* Step 2 — stitch each quad into two CCW triangles.
   *   r0 = top-left      r1 = top-right
   *   r2 = bot-left      r3 = bot-right                            */
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

/* ── §5 framebuffer — zbuf + cbuf + Bourke ramp + dither + blit ──────── */

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

static void fb_alloc(Framebuffer *fb, int cols, int rows) {
  fb->cols = cols;
  fb->rows = rows;
  fb->zbuf = malloc((size_t)(cols * rows) * sizeof(float));
  fb->cbuf = malloc((size_t)(cols * rows) * sizeof(Cell));
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

/* ── §6 pipeline — vertex transform → cull → barycentric raster → FS ─── */

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

    /* near clip reject */
    if (vo[0].clip_pos.w < 0.001f && vo[1].clip_pos.w < 0.001f &&
        vo[2].clip_pos.w < 0.001f)
      continue;

    /* perspective divide → screen */
    float sx[3], sy[3], sz[3];
    for (int vi = 0; vi < 3; vi++) {
      float w = vo[vi].clip_pos.w;
      if (fabsf(w) < 1e-6f)
        w = 1e-6f;
      sx[vi] = (vo[vi].clip_pos.x / w + 1.f) * 0.5f * (float)cols;
      sy[vi] = (-vo[vi].clip_pos.y / w + 1.f) * 0.5f * (float)rows;
      sz[vi] = vo[vi].clip_pos.z / w;
    }

    /* back-face cull — skipped when cull_backface is false */
    float area =
        (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
    if (cull_backface && area <= 0.f)
      continue;

    /* bounding box */
    int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

    for (int py = y0; py <= y1; py++) {
      for (int px = x0; px <= x1; px++) {
        float b[3];
        barycentric(sx, sy, px + 0.5f, py + 0.5f, b);
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

/* ── §7 scene — Scene struct, uniforms wiring, tick, draw, swap ──────── */

typedef struct {
  Mesh mesh;
  float angle_x, angle_y;
  float cam_dist; /* current zoom distance — changed by +/- */
  bool paused;
  bool cull_backface; /* c toggles — false shows inner surface  */
  ShaderIdx shade_idx;
  ShaderProgram shader;
  Uniforms uni;
  ToonUniforms toon_uni;
} Scene;

static void scene_build_shader(Scene *s) {
  switch (s->shade_idx) {
  case SH_PHONG:
    s->shader = (ShaderProgram){vert_default, frag_phong, &s->uni, &s->uni};
    break;
  case SH_TOON:
    s->toon_uni.base = s->uni;
    s->toon_uni.bands = 4;
    s->shader = (ShaderProgram){vert_default, frag_toon, &s->uni, &s->toon_uni};
    break;
  case SH_NORMALS:
    s->shader = (ShaderProgram){vert_normals, frag_normals, &s->uni, &s->uni};
    break;
  case SH_WIRE:
    s->shader = (ShaderProgram){vert_wire, frag_wire, &s->uni, &s->uni};
    break;
  default:
    break;
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->mesh = tessellate_sphere();
  s->shade_idx = SH_PHONG;
  s->cam_dist = CAM_DIST;
  s->cull_backface = true;

  s->uni.light_pos = v3(3.f, 4.f, 3.f);
  s->uni.light_col = v3(1.f, 1.f, 1.f);
  s->uni.ambient = v3(0.07f, 0.07f, 0.07f);
  s->uni.shininess = 80.f; /* higher shininess = tighter highlight */
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.obj_color = v3(0.25f, 0.55f, 0.95f); /* cool blue */

  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);

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
  if (s->paused)
    return;
  s->angle_y += ROT_Y * dt;
  s->angle_x += ROT_X * dt;
  Mat4 ry = m4_rotate_y(s->angle_y);
  Mat4 rx = m4_rotate_x(s->angle_x);
  s->uni.model = m4_mul(ry, rx);
  s->uni.mvp = m4_mul(s->uni.proj, m4_mul(s->uni.view, s->uni.model));
  s->uni.norm_mat = m4_normal_mat(s->uni.model);
  s->toon_uni.base = s->uni;
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

/* ── §8 screen — ncurses init / resize / HUD / present ────────────────── */

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
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan + bold)   — key hint
 *
 * The 7-pair luma palette uses pair 3 (yellow) and pair 5 (cyan) at
 * the right brightness for HUD text — we alias them as named constants
 * so the HUD code reads "yellow / cyan" instead of "magic 3 / 5". */
#define PAIR_HUD 3  /* yellow */
#define PAIR_HINT 5 /* cyan   */

static void screen_draw_hud(const Screen *s, const Scene *sc, double fps) {
  char status[HUD_COLS + 1];
  snprintf(status, sizeof status,
           " %5.1f fps  shader:%s  zoom:%.1f  cull:%s%s ", fps,
           k_shader_names[sc->shade_idx], sc->cam_dist,
           sc->cull_backface ? "on " : "off", sc->paused ? " PAUSED" : "");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " SPHERE · RASTER ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0, " q:quit  spc:pause  s:shader  c:cull  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §9 app — main loop, signals, resize, cleanup ─────────────────────── */

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
