/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sphere_raytrace.c — analytic ray-traced sphere (the foundational demo)
 *
 * DEMO: A glossy sphere (gold by default) orbits in space, lit by
 *       three FIXED WHITE LIGHTS. Each material's distinctive look
 *       comes from its own properties — gold reflects yellow at the
 *       highlight, glass shows a near-white specular peak, sapphire
 *       reads as deep blue. Cycle phong → normals → fresnel → depth
 *       to see how the same ray-sphere intersection feeds different
 *       shading models. Single primitive in the simplest possible
 *       setup — this is the "Hello World" of analytic ray tracing.
 *
 * Study alongside:
 *   raytracing/cube_raytrace.c     — same skeleton, slab method
 *   raytracing/capsule_raytrace.c  — same skeleton, decomposed analytic
 *   raytracing/torus_raytrace.c    — same skeleton, QUARTIC (much harder)
 *   raytracing/path_tracer.c       — same RGB-cube paint pipeline,
 *                                    multi-bounce Monte Carlo on top
 *
 * Section map:
 *   §1 config     — frame rate, FOV, sphere geometry, camera, ramp, HUD
 *   §2 clock      — monotonic timer + sleep
 *   §3 math       — V3 helpers
 *   §4 color      — themes + 256-colour cube + ASCII-ramp painter
 *   §5 sphere     — THE CORE: analytic ray-sphere quadratic
 *   §6 shading    — phong / normals / fresnel / depth
 *                   §6.1 KEY light  §6.2 FILL light  §6.3 RIM light
 *                   §6.4 phong glue §6.5 normal mode §6.6 fresnel/depth
 *   §7 render     — one frame: ray-per-cell, three-point lighting
 *   §8 screen     — ncurses init + HUD
 *   §9 app        — signals, input, main loop
 *
 * Keys:
 *   s         cycle shade mode (phong → normals → fresnel → depth)
 *   t / T     next / previous theme (20 PBR materials: 12 metals,
 *             4 gems, 3 dielectrics, 1 emissive — see §4)
 *   p / SPC   pause / resume orbit
 *   + / =     zoom in (orbit closer)
 *   -         zoom out
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/sphere_raytrace.c \
 *       -o sphere_rt -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Ray-sphere is the canonical analytic raytrace; if
 *      you understand it you can read every other file in this
 *      folder.
 *   2. §1 config — every constant has a unit-bearing comment.
 *   3. §5 ray_sphere — THE CORE. Read AFTER tutorial T2.
 *   4. §6 shading — five small shaders (phong / normal / fresnel /
 *      depth / luma helper). Read AFTER tutorials T3-T5.
 *   5. §4 themes — pure data, skim. The interesting bit is the
 *      `albedo / specular / emissive / diffuse_weight' field
 *      semantics (T3 explains).
 *   6. §7 render + §8 hud + §9 app — infrastructure, skip on first
 *      read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   ro / rd            primary ray as (origin, unit direction)
 *   th                 pointer to the active Theme
 *   P                  surface hit point (world space)
 *   N                  surface normal at P (unit vector)
 *   V_dir              vector from P toward the camera
 *   L                  vector from P toward a light
 *   R                  reflection of L across N (Phong specular axis)
 *   NdotL / NdotV      dot products used by every shader
 *
 *   Single-letter names (i, c, r, t) appear inside tight loops or as
 *   ad-hoc symbols whose meaning is on the line above.
 *
 * Background you need
 * ───────────────────
 *   - 3-D vectors (dot, cross, normalise).
 *   - The quadratic formula and what its discriminant means
 *     geometrically (positive → two real roots → ray hits sphere).
 *   - Lambertian diffuse: max(0, N · L) is the "fraction of incoming
 *     light at this surface point."
 *   - 6×6×6 RGB colour cube on xterm-256 (16 + 36·R5 + 6·G5 + B5).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Recursive raytracing — every ray here is one bounce only.
 *   - PDFs, importance sampling — see path_tracer.c for that.
 *   - Acceleration structures (BVH, KD-tree). One sphere = no need.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-sphere intersection — exact, no marching
 *                  steps, no mesh, no acceleration structure. For a ray
 *                  ro + t·rd and a sphere at centre c with radius R, the
 *                  hit condition |ro + t·rd − c|² = R² expands to a
 *                  quadratic in t:
 *                    a·t² + b·t + c = 0
 *                  where (with rd unit-length and oc = ro − c):
 *                    a = 1
 *                    b = 2 (rd · oc)
 *                    c = |oc|² − R²
 *                  Discriminant = b² − 4ac. If < 0: ray misses entirely.
 *                  Else t = (−b ± √disc) / 2. Pick the nearest positive
 *                  root for the front-face hit.
 *
 *                  Surface normal at hit point P: N = (P − c) / R.
 *                  Three white lights at fixed positions drive the
 *                  PBR-flavored Phong shading model. The lights are
 *                  pure white; each MATERIAL is described by:
 *                    albedo          body colour (diffuse tint)
 *                    specular  (F0)  Fresnel reflectance at normal
 *                                    incidence — METALS tint to
 *                                    match albedo, DIELECTRICS ≈ white
 *                    emissive        self-glow added after lighting
 *                    diffuse_weight  scales body diffuse — metals
 *                                    ≈ 0.15 (low), plastics ≈ 0.85
 *                  This is the same Theme structure used by
 *                  cube_raytrace.c, torus_raytrace.c — one material
 *                  vocabulary across the folder.
 *
 * Data-structure : NONE persistent. Each frame is a pure function of
 *                  (cols, rows, orbit_angle, cam_dist, theme, mode).
 *                  Per-frame: one camera basis. 20 themes are static
 *                  PBR descriptors. 216 ncurses pairs are pre-
 *                  allocated as a 6×6×6 RGB cube; computed RGB
 *                  ∈ [0,1]³ quantises to one of those at draw time.
 *
 * Rendering      : One ray per terminal cell (no AA). RGB shaded
 *                  output → quantised to xterm 6×6×6 cube + Bourke
 *                  92-char density ramp. Both colour and glyph
 *                  density carry shading information.
 *
 * Performance    : Per pixel: one ray-sphere test (~10 multiplications),
 *                  three Phong light contributions (~50 multiplications
 *                  total), one cube quantise + ramp lookup. Trivially
 *                  fast — sphere is the cheapest analytic primitive.
 *                  240×80 cells × 60 fps ≈ 1 ms shading per frame.
 *
 * References     : Whitted, T. "An Improved Illumination Model for
 *                    Shaded Display," CACM 23(6), 1980. (Foundational
 *                    paper on analytic ray-tracing of spheres + Phong
 *                    lighting.)
 *                  Phong, B.T. "Illumination for Computer Generated
 *                    Pictures," CACM 18(6), 1975. (The shading model.)
 *                  Schlick, C. "An Inexpensive BRDF Model for
 *                    Physically-based Rendering," Comp. Graph. Forum
 *                    13(3), 1994. (The Fresnel approximation we use.)
 *                  Shirley, P. "Ray Tracing in One Weekend"
 *                    https://raytracing.github.io/books/RayTracingInOneWeekend.html
 *                  Inigo Quílez, "Sphere — intersection"
 *                    https://iquilezles.org/articles/intersectors/
 *                  Real-Time Rendering 4e §22.7 (ray-sphere variants).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To test if a ray hits a sphere, ask: "for what value of t along the
 * ray does the ray's distance to the sphere centre equal the sphere
 * radius?" Square both sides (so we don't deal with the square root)
 * and you get a QUADRATIC in t. Solve the quadratic. If real solutions
 * exist, the ray hits — pick the smallest positive one. That's it.
 * No marching, no iteration, no special cases (other than discriminant
 * < 0 = miss). The geometry of "where does a line meet a sphere" maps
 * exactly onto "solve a quadratic equation."
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the ray as a moving point. At time t = 0 it's at the camera;
 * at time t = ∞ it's gone forever. As t advances, the point traces a
 * straight line. We're asking: "between t = 0 and t = ∞, when does
 * this point sit exactly on the sphere's surface?"
 *
 *     ●─────────────────────●─── ray (at parameter t)
 *     ↑                     ↑
 *     ro                    P(t) = ro + t·rd
 *
 *           ╭────────╮
 *          ╱  sphere  ╲
 *         │            │
 *         │     ●c    │      ← centre c, radius R
 *          ╲          ╱
 *           ╰────────╯
 *
 * Distance from P(t) to c is |ro + t·rd − c|. Setting that equal to R
 * (and squaring to remove the sqrt) gives a quadratic. Geometric
 * interpretation:
 *
 *   t² + 2(rd · oc)·t + (|oc|² − R²) = 0     where oc = ro − c
 *      ╲___╱     ╲_________╱     ╲____________╱
 *       a            b/2                c
 *
 *   discriminant = b² − 4ac
 *      < 0  →  ray passes WIDE of the sphere — never hits.
 *      = 0  →  ray is exactly tangent to the sphere.
 *      > 0  →  two real roots: the entry and exit times.
 *
 * Once we have the hit point, the SURFACE NORMAL is just the unit
 * vector from the centre to the hit:
 *
 *   N = (P − c) / R
 *
 * Three WHITE lights illuminate the sphere via PBR-flavored Phong:
 * each light contributes a soft Lambertian (cosine of N·L) scaled by
 * the material's diffuse_weight, plus a sharp specular ((R·V)^n)
 * scaled by the material's specular F0. METALS get a tinted spec
 * (gold reflects warm yellow, copper reflects orange-red) because
 * their F0 IS the albedo; DIELECTRICS get near-white spec because
 * their F0 ≈ (1, 1, 1) regardless of body colour. The KEY light is
 * the dominant source from upper-right; FILL lifts the shadow side
 * from upper-left; RIM kisses the silhouette from behind.
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS  (per pixel)
 * ──────────────────────────────────────────────────
 *  1. Build a primary ray from the orbiting camera into world space:
 *        rd = normalize( fwd + pu·right + pv·up )
 *     where (pu, pv) are screen coordinates scaled by tan(FOV/2)
 *     and aspect.
 *  2. Solve the ray-sphere quadratic (§5):
 *        b = rd · (ro − c)
 *        c = |ro − c|² − R²
 *        disc = b² − c
 *        if disc < 0: MISS, continue.
 *        t = -b - √disc   (front face)
 *  3. Compute the hit point P = ro + t·rd and surface normal
 *        N = (P − c) / R
 *  4. SHADE in continuous RGB:
 *        col = ambient + KEY + FILL + RIM
 *     Each light contributes diffuse (and KEY/RIM also specular):
 *        L = normalize(light_pos − P)
 *        d = max(0, N · L)                         ← Lambertian
 *        R = reflect(−L, N) = 2(N·L)N − L          ← reflection of L
 *        s = max(0, R · V_dir)^shininess           ← Phong specular
 *  5. Quantise → 6×6×6 cube pair + 92-char density ramp + A_BOLD/A_DIM.
 *
 * KEY FORMULAS
 * ────────────
 *   Ray-sphere quadratic (rd unit, oc = ro − c):
 *     b    = rd · oc
 *     c    = oc · oc − R²
 *     disc = b² − c
 *     t    = -b − √disc      (front face; back face is -b + √disc)
 *
 *   Surface normal (sphere centred at c):
 *     N = (P − c) / R         and |N| = 1 by construction
 *
 *   Phong shading per light:
 *     L    = normalize(light_pos − P)
 *     R    = 2(N·L)·N − L
 *     I    = albedo · max(0, N·L) · light_col      ← diffuse
 *          + spec_col · max(0, R·V_dir)^shininess  ← specular
 *
 *   Schlick Fresnel (used by MODE_FRESNEL):
 *     cosθ = |N · V_dir|
 *     F    = (1 − cosθ)^5     ← assumes F₀ = 0 (full dielectric)
 *
 * WORKED EXAMPLE  (verify by hand)
 * ────────────────────────────────
 *   Sphere at origin, R = 1. Camera at (0, 0.55, -3.6) (orbit_ang = 0,
 *   cam_dist = 3.6). Centre cell of screen, so pu = pv = 0, rd ≈ fwd.
 *
 *   fwd = normalize((0,0,0) − (0, 0.55, -3.6))
 *       = normalize((0, -0.55, 3.6))
 *       ≈ (0, -0.151, 0.989)
 *
 *   Ray-sphere:
 *     oc   = ro − 0 = (0, 0.55, -3.6)
 *     b    = rd · oc = 0·0 + (-0.151)·0.55 + 0.989·(-3.6)
 *          = -0.0830 − 3.560 ≈ -3.643
 *     c    = oc·oc − R² = (0 + 0.3025 + 12.960) − 1 = 12.262
 *     disc = b² − c = 13.272 − 12.262 = 1.010
 *     t    = -b − √disc = 3.643 − 1.005 = 2.638
 *
 *   Hit point:
 *     P = ro + t·rd = (0, 0.55 − 0.398, -3.6 + 2.609)
 *       = (0, 0.152, -0.991)
 *     |P| ≈ 1.003 (≈1.0; small rounding ok)
 *     N = P / 1 ≈ (0, 0.151, -0.987)        ← faces toward camera
 *
 *   KEY light at (3, 4, -2):
 *     L_dir = (3, 4-0.152, -2-(-0.991)) = (3, 3.848, -1.009)
 *     |L_dir| = √(9 + 14.81 + 1.02) ≈ 4.983
 *     L = (0.602, 0.772, -0.202)
 *     N·L = 0 + 0.117 + 0.199 = 0.316
 *     diffuse = 0.316     ← hit is partially lit by KEY
 *
 *   For the GOLD theme (albedo = 1.00, 0.77, 0.34, diffuse_weight = 0.15):
 *     key_diffuse  = NdotL · diffuse_weight · 1.00 · albedo
 *                  = 0.316 · 0.15 · 1.00 · (1.00, 0.77, 0.34)
 *                  ≈ (0.0474, 0.0365, 0.0161)
 *
 *   FILL contributes ≈ 0.55× weight; RIM ≈ 0.40× weight (both
 *   scaled by their own NdotL). Plus specular at ≈ 1.30× the
 *   tinted-yellow F0 = (1.00, 0.77, 0.34) when the eye looks near
 *   the reflection direction.
 *
 *   With AMBIENT_K = 0.20 the ambient term alone is (0.20, 0.154,
 *   0.068). Final RGB at this cell sums to roughly (0.45, 0.34,
 *   0.10) — a warm gold body with a yellow specular peak that hits
 *   harder where the camera looks straight at the highlight.
 *   Reinhard tone-map → gamma → 6×6×6 cube → warm-yellow region;
 *   density ≈ 0.30 → mid ramp glyph.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DISCRIMINANT NEAR ZERO. The ray is nearly tangent. t = -b ± √disc
 *    are very close to each other; rounding can flip the sign. The
 *    1e-4 epsilon in `t1 < 1e-4f` check rejects effectively-zero
 *    distances and avoids self-intersection.
 *  • CAMERA INSIDE SPHERE. Then c = |oc|² − R² < 0, and disc = b² − c
 *    > 0 always, so we get a hit. -b ± √disc gives one negative and
 *    one positive root; we want the positive one (the exit face).
 *    The code handles this by trying t0 first, falling back to t1.
 *  • RAY DIRECTION NOT UNIT-LENGTH. The quadratic relies on |rd| = 1.
 *    If you pass a non-unit vector the formula needs `a = rd·rd` term.
 *    We always normalise rd in render(), so this is safe.
 *  • PURE DIFFUSE LIMIT. As shininess → 0, (R·V)^n → 1 everywhere
 *    and the sphere looks flat-shaded. As shininess → ∞ the highlight
 *    becomes a single bright pixel. SHININESS = 52 gives a tight
 *    plastic-like specular.
 *  • SPECULAR ON DARK SIDE. We multiply specular by max(0, N·L) — no,
 *    we don't always; KEY's specular is s alone (not gated by d). On
 *    the back of the sphere (N·L < 0), the reflection vector still
 *    has a positive R·V at certain angles, producing a "ghost" highlight.
 *    Look at the unlit hemisphere with very shiny shaders to see this
 *    artifact. For more physically correct shading, gate specular by
 *    (N·L > 0). We don't here — it's the simplest Phong model on
 *    purpose, to keep the file small.
 *  • TONE-MAP AT PAINT. Quantising linear HDR puts everything in one
 *    cube cell because the cube is uniform [0,1]³. Tone-mapping
 *    inside paint_cell opens the dynamic range first.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Static sphere (paused, orbit_ang = 0): gold sphere, warm KEY
 *    light from the upper-right, soft cool FILL on the left, bright
 *    RIM from behind catching the silhouette. The brightest specular
 *    spot should be on the upper-right surface.
 *  • MODE_NORMAL: each surface direction gets a unique RGB colour.
 *    The +Z hemisphere (toward camera at orbit_ang=0) is mostly blue
 *    (0.5, 0.5, 1.0) at centre fading to red/green at silhouette.
 *  • MODE_FRESNEL: dark in the middle (head-on, cosθ ≈ 1 → F ≈ 0),
 *    bright at the silhouette (grazing, cosθ ≈ 0 → F ≈ 1). The
 *    classic glass-marble look.
 *  • MODE_DEPTH: closer cells brighter, farther cells darker. The
 *    silhouette should be the dimmest part (largest t).
 *  • Worked-example check: at orbit_ang=0, cam_dist=3.6, the centre-
 *    cell ray hits the sphere at t ≈ 2.638, normal ≈ (0, 0.15, -0.99).
 *    Inspect the source if your sphere appears in the wrong place.
 *  • Cycle THEME (t): the obj/specular/light tints all change while
 *    the geometry stays fixed.
 *  • Zoom (+/-): smaller cam_dist → bigger sphere on screen, brighter
 *    edges (RIM gets closer). Larger → smaller sphere, dimmer overall.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Seven short tutorials that build the renderer from first principles.
 * Read in order; each builds on the previous.
 *
 *   T1  The pinhole camera — pixel → world-space ray
 *   T2  Ray-sphere intersection — the textbook quadratic
 *   T3  PBR-flavored Phong — white lights + tinted F0
 *   T4  Why metals look like metals — diffuse_weight + tinted spec
 *   T5  Schlick Fresnel — the glass-marble look (MODE_FRESNEL)
 *   T6  Three debug overlays — NORMAL / FRESNEL / DEPTH
 *   T7  Continuous-RGB pipeline → 6×6×6 cube + 92-char ramp
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  THE PINHOLE CAMERA — PIXEL → WORLD-SPACE RAY
 * ─────────────────────────────────────────────────
 * The screen is a rectangle of cols × rows cells. The camera orbits
 * the sphere on a circle of radius cam_dist at height CAM_HEIGHT;
 * its forward vector points at the world origin (where the sphere
 * sits).
 *
 * For each cell (col, row):
 *
 *   1. Build screen-space coords (pu, pv). Centred at (0,0) in the
 *      middle of the frame, scaled by tan(FOV/2) so a unit step in
 *      pu / pv = a unit step on the image plane at z = 1:
 *
 *        pu =  (col − cx) / cx · fov_tan
 *        pv = -(row − cy) / cx · fov_tan / ASPECT
 *
 *      Why divide pv by cx (not cy)? We want SQUARE pixels in world
 *      space — equal angular extent per cell on both axes. Then we
 *      correct for the terminal cell's aspect ratio (cells are
 *      taller than wide) by /ASPECT. Negation flips screen-y-down
 *      to world-y-up.
 *
 *   2. Build the world-space ray direction from the camera basis:
 *
 *        rd = norm(forward + pu · right + pv · up)
 *
 *      `forward = norm(origin − cam)`, `right = norm(forward × up)`,
 *      `up' = right × forward` rebuilds an orthonormal basis at the
 *      orbiting camera each frame.
 *
 *   3. The primary ray is (cam_pos, rd). Pass it to ray_sphere (T2).
 *
 * Read §7 render frame for the exact code.
 *
 * T2  RAY-SPHERE INTERSECTION — THE TEXTBOOK QUADRATIC
 * ─────────────────────────────────────────────────────
 * To test if a ray (ro, rd) hits a sphere of radius R at the origin,
 * ask "for what t along the ray is |ro + t·rd| = R?"  Squaring both
 * sides removes the radical; rearranging yields:
 *
 *     t² (rd · rd) + 2t (rd · ro) + (ro · ro − R²) = 0
 *
 * With unit-length rd, rd · rd = 1 — the leading t² coefficient is 1.
 * Define b = rd · ro, c = ro · ro − R². Then:
 *
 *     t² + 2b·t + c = 0
 *     disc = b² − c             (the discriminant divided by 4)
 *     if disc < 0: ray MISSES (no real roots)
 *     t = -b - √disc            ← front face
 *
 * Three geometric cases:
 *
 *   disc < 0:   ray's closest approach > R, sphere never crosses
 *   disc = 0:   ray is exactly tangent (one repeated root)
 *   disc > 0:   ray enters at t = -b - √, exits at t = -b + √
 *
 * Once we have t, the hit point is P = ro + t·rd, and the unit
 * normal at P is (P − centre) / R = P (since centre = origin and
 * radius = 1). Read §5 ray_sphere.
 *
 * Worked example (camera at (0, 0.55, -3.6), centre cell of frame):
 *   ro    = (0, 0.55, -3.6)
 *   rd    ≈ (0, -0.151, 0.989)         (looking back at origin)
 *   b     = rd · ro = 0 - 0.083 - 3.560 = -3.643
 *   c     = ro · ro − 1 = (0 + 0.302 + 12.96) − 1 = 12.262
 *   disc  = b² − c = 13.272 − 12.262 = 1.010
 *   √disc = 1.005
 *   t0    = -b − √disc = 3.643 − 1.005 = 2.638
 *   P     = ro + 2.638·rd = (0, 0.152, -0.991)
 *   |P|   ≈ 1.003 ≈ 1.0   ✓ (on the unit sphere, mod rounding)
 *   N     = P/1 ≈ (0, 0.152, -0.991)   (faces toward camera)
 *
 * T3  PBR-FLAVORED PHONG — WHITE LIGHTS + TINTED F0
 * ──────────────────────────────────────────────────
 * Old-school three-point Phong used TINTED LIGHTS (warm key, cool
 * fill, accent rim) to give the rendered object character. That
 * works but the colour comes from the LIGHT, not the material — a
 * gold sphere under a cool light reads as cool-yellow, which is
 * physically wrong.
 *
 * PBR-flavored Phong (this file) uses three WHITE lights at fixed
 * positions, and lets the material decide the colour. Per light:
 *
 *     diffuse  = max(0, N·L) · diffuse_weight · weight · albedo
 *     specular = max(0, R·V_dir)^SHININESS  · weight · F0
 *
 * Where R = 2(N·L)·N − L is the reflection of the light direction
 * across the normal. The three lights are:
 *
 *     KEY  upper-right, weight = 1.00 (dominant)
 *     FILL upper-left,  weight = 0.55 (lifts shadow)
 *     RIM  behind,      weight = 0.40 (back-light silhouette)
 *
 * Plus a 0.20 ambient term: ambient = AMBIENT_K · albedo.
 * Plus emissive (0 except for neon).
 *
 * That's the entire shader (§6). Read shade_phong after §1 config to
 * see the full code — it's about 40 lines.
 *
 * Worked example (gold theme at the hit point from T2):
 *   P     = (0, 0.152, -0.991),  N ≈ (0, 0.152, -0.991)
 *   th->albedo         = (1.00, 0.77, 0.34)
 *   th->specular  (F0) = (1.00, 0.77, 0.34)             gold metal
 *   th->diffuse_weight = 0.15                           low (metal)
 *
 *   ambient  = AMBIENT · albedo = 0.20 · (1.00, 0.77, 0.34)
 *            = (0.200, 0.154, 0.068)
 *
 *   KEY light:
 *     L      = norm(LIGHT_KEY − P)
 *            = norm((3.0, 3.848, -1.009)) = (0.602, 0.772, -0.202)
 *     N·L    = 0 + 0.117 + 0.200 = 0.317
 *     diffuse_KEY = 0.317 · 0.15 · 1.00 · (1.00, 0.77, 0.34)
 *               ≈ (0.0476, 0.0366, 0.0162)
 *   FILL light:
 *     N·L ≈ 0.13     (cool fill from upper-left)
 *     diffuse_FILL = 0.13 · 0.15 · 0.55 · (1.00, 0.77, 0.34)
 *               ≈ (0.011, 0.008, 0.004)
 *   RIM light:
 *     N·L ≈ 0.18     (behind, kissing silhouette)
 *     diffuse_RIM = 0.18 · 0.15 · 0.40 · (1.00, 0.77, 0.34)
 *               ≈ (0.011, 0.008, 0.004)
 *
 *   Sum so far (no specular yet):
 *     col ≈ (0.27, 0.21, 0.080)        warm dim gold body
 *
 *   If V_dir aligns near the KEY's reflection direction, specular
 *   adds (R·V)^75 · 1.30 · F0 — a thin yellow peak that can take
 *   col over 1.0 in the red channel. Reinhard tone-map (T7) then
 *   compresses it back to display range.
 *
 * T4  WHY METALS LOOK LIKE METALS — DIFFUSE_WEIGHT + TINTED SPEC
 * ──────────────────────────────────────────────────────────────
 * Two material properties make METALS look distinctively metallic:
 *
 *   1. TINTED SPECULAR (F0 matches albedo).
 *      Real metals are CONDUCTORS — light cannot enter the surface,
 *      everything reflects. The reflectance at normal incidence is
 *      colour-tinted to the metal's chemistry: gold reflects yellow
 *      because gold's electrons resonate at yellow wavelengths.
 *      In our Theme: specular F0 == albedo for all 12 metals.
 *
 *   2. LOW DIFFUSE_WEIGHT (~0.15).
 *      Because nearly all light reflects via specular, the diffuse
 *      Lambertian term is much smaller than for dielectrics. A metal
 *      sphere's body colour is dim; the visible warmth comes from
 *      the highlight, which carries the tint.
 *
 * Compare to GLASS (a dielectric):
 *     albedo         = (0.10, 0.12, 0.16)   dark base
 *     specular F0    = (1.00, 1.00, 1.00)   ACHROMATIC ~4% reflection
 *                                            scaled up so it pops
 *     diffuse_weight = 0.10                 very low; body is dark
 *
 * Glass sphere reads as "dark, with a CRISP WHITE highlight" — and
 * that's exactly the look of glass under terminal-grade rendering
 * (real glass also refracts; we don't simulate that).
 *
 * Compare to RUBY (a gem, dielectric):
 *     albedo         = (0.85, 0.10, 0.18)   saturated red
 *     specular F0    = (1.00, 0.95, 0.95)   near-white
 *     diffuse_weight = 0.70                 mid; saturated body
 *
 * Ruby sphere reads as "deep red with a white highlight" — body
 * colour from absorption inside the crystal, white highlight from
 * dielectric F0.
 *
 * Cycling `t' through the 20 themes shows all four material
 * families: 12 metals (gold..aluminum), 4 gems (ruby..amethyst),
 * 3 dielectrics (plastic / glass / ceramic), 1 emissive (neon —
 * has nonzero `emissive' added after lighting).
 *
 * T5  SCHLICK FRESNEL — THE GLASS-MARBLE LOOK
 * ────────────────────────────────────────────
 * The Fresnel equations describe how reflectance varies with
 * incidence angle: head-on rays reflect a small fraction (F₀ ≈ 4%
 * for dielectrics, higher for metals); grazing rays reflect ALMOST
 * EVERYTHING. That's why a glass marble seen from the side reads
 * as bright at the silhouette and dark in the centre.
 *
 * Schlick's approximation (1994):
 *
 *     F(θ) = F₀ + (1 − F₀) · (1 − cosθ)^5
 *
 * For F₀ = 0 this collapses to (1 − cosθ)^5 — dark head-on, bright
 * at grazing. We use this in MODE_FRESNEL (§6) as an EFFECT, not
 * an actual Fresnel-weighted reflection. It blends a dim core
 * colour (0.06 · albedo) toward a bright edge colour
 * (1.10 · specular F0) using the Schlick weight.
 *
 * The result: a "glass marble" diagnostic mode that lets you see
 * how grazing the silhouette feels on the eye — useful for
 * confirming that normals are correct around the sphere.
 *
 * T6  THREE DEBUG OVERLAYS — NORMAL / FRESNEL / DEPTH
 * ────────────────────────────────────────────────────
 * Cycling `s' switches between four shading modes. Each is a
 * different pedagogical lens on the same scene:
 *
 *   PHONG     The full pipeline — PBR Phong with three lights.
 *             Use this to admire the result.
 *
 *   NORMAL    RGB-encoded surface normal: N → (N+1)/2. Each
 *             component remapped from [-1,+1] → [0,1] so the three
 *             RGB channels visualise N.x, N.y, N.z. The +Z
 *             hemisphere reads mostly blue (0.5, 0.5, 1.0); +X
 *             red-ish; +Y green-ish. A correct sphere intersection
 *             produces a smooth radial rainbow gradient.
 *
 *   FRESNEL   See T5. Dark head-on, bright at silhouette. The
 *             glass-marble look.
 *
 *   DEPTH     Brightness ∝ 1 − t/t_max. Closer surfaces brighter,
 *             distant ones darker. Sanity check that the camera
 *             produces sane distances.
 *
 * The three debug modes converge in ONE FRAME (no randomness).
 * Switching INTO them is instant; switching back to PHONG looks
 * the same.
 *
 * T7  CONTINUOUS-RGB PIPELINE → 6×6×6 CUBE + 92-CHAR RAMP
 * ────────────────────────────────────────────────────────
 * The shader produces a LINEAR HDR RGB triplet — values can exceed
 * 1.0 (specular peaks especially). To display we compress and
 * quantise:
 *
 *   1. Per-pixel shading produces an HDR linear RGB triplet.
 *   2. Reinhard tone-map per channel:  L' = L / (1 + L)
 *      Compresses [0, ∞) into [0, 1) without saturation.
 *   3. Gamma encode 1/2.2: brightens midtones perceptually for
 *      sRGB displays.
 *   4. Quantise each channel to {0..5}: 6 levels per axis = 216
 *      cube cells.
 *   5. Pair index = 16 + 36·R5 + 6·G5 + B5  (xterm cube convention).
 *   6. Density character = Rec.601 luma → ramp[index] in 92-char
 *      ramp.
 *   7. A_BOLD on luma > 0.85, A_DIM on luma < 0.15.
 *
 * Both the COLOUR (cube cell) and the GLYPH DENSITY (ramp index)
 * carry shading information. Together they encode ~20,000
 * effective shades per cell — more than enough for smooth
 * specular gradients without visible banding.
 *
 * Worked example (HDR shader output (1.40, 0.85, 0.20) from a
 * specular peak on the gold sphere):
 *
 *   1. Reinhard per channel:
 *        L' = (1.40/2.40, 0.85/1.85, 0.20/1.20)
 *           = (0.583, 0.459, 0.167)
 *
 *   2. Gamma encode (1/2.2):
 *        out = (0.583^0.455, 0.459^0.455, 0.167^0.455)
 *            ≈ (0.776, 0.685, 0.435)
 *
 *   3. Quantise to {0..5}:
 *        r5 = round(0.776 · 5) = round(3.88) = 4
 *        g5 = round(0.685 · 5) = round(3.42) = 3
 *        b5 = round(0.435 · 5) = round(2.17) = 2
 *
 *   4. Pair index = 16 + 36·4 + 6·3 + 2 = 180
 *      → xterm 256-colour cube cell #180 ≈ #FFD787 (warm yellow)
 *
 *   5. Rec.601 luma:
 *        Y = 0.299·0.776 + 0.587·0.685 + 0.114·0.435
 *          = 0.232 + 0.402 + 0.050
 *          = 0.684
 *
 *   6. ASCII glyph: ramp[round(0.684 · 91)] = ramp[62]
 *      ≈ 'p' or 'P' (mid-density)
 *      Bright? 0.684 > 0.85 → no → A_NORMAL.
 *
 *   Cell paints `P` in warm-yellow on the terminal — a faint
 *   highlight pixel.
 *
 * Read §6 paint_cell and the matching tutorials in path_tracer.c
 * for the full discussion.
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 199309L
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate */
#define TARGET_FPS    60
#define DT_CAP_NS     100000000LL          /* 0.1 sec — spiral-of-death cap */

/* §1.2 view geometry */
#define ASPECT        0.47f                /* terminal cell W/H ratio       */
#define FOV_DEG       58.0f                /* full vertical-equivalent FOV  */

/* §1.3 sphere (object space, centred at origin) */
#define SPHERE_R      1.0f                 /* unit-radius sphere            */

/* §1.4 orbit (camera) — orbits the sphere at fixed distance + height */
#define ORBIT_SPEED   0.32f                /* radians / second              */
#define CAM_HEIGHT    0.55f                /* camera elevation above equator*/
#define CAM_DIST_DEF  3.6f
#define CAM_DIST_MIN  1.9f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f

/* §1.5 shading */
#define AMBIENT       0.20f                /* dim copy of albedo as ambient */
#define SHININESS     75.0f                /* phong exponent — high = metal */

/* §1.6 character ramp — Paul Bourke 92-char density ladder.
 * Index 0 (space) is invisible; index N−1 ('@') is densest. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN  ((int)(sizeof k_ramp - 1))

/* §1.7 ncurses pair IDs (256-colour cube + reserved HUD/HINT) */
#define PAIR_CUBE_BASE   1                 /* + 0..215 = 6×6×6 cube       */
#define PAIR_HUD       217
#define PAIR_HINT      218

/* §1.8 epsilon for ray distances */
#define T_EPS         1e-4f                /* reject t < this (self-hit)  */

/* ── §2 clock ────────────────────────────────────────────────────────── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 math (V3) ────────────────────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3add   (V3 a, V3 b)    { return (V3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline V3    v3sub   (V3 a, V3 b)    { return (V3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline V3    v3scale (float s, V3 a) { return (V3){s*a.x, s*a.y, s*a.z};      }
static inline float v3dot   (V3 a, V3 b)    { return a.x*b.x + a.y*b.y + a.z*b.z;     }
static inline float v3len   (V3 a)          { return sqrtf(v3dot(a, a));              }
static inline V3    v3norm  (V3 a)          { float l=v3len(a); return l>1e-9f ? v3scale(1.f/l, a) : (V3){0,1,0}; }
static inline V3    v3cross (V3 a, V3 b)    { return (V3){a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x}; }
static inline V3    v3reflect(V3 v, V3 n)   { return v3sub(v, v3scale(2.f*v3dot(v,n), n)); }
static inline V3    v3clamp1(V3 v)
{
    return (V3){ v.x<0?0:v.x>1?1:v.x, v.y<0?0:v.y>1?1:v.y, v.z<0?0:v.z>1?1:v.z };
}

/* ── §4 color / themes ───────────────────────────────────────────────── */

/*
 * Theme — PBR-flavoured material descriptor (matches cube_raytrace.c
 * so all three primitives share one material vocabulary).
 *
 * First-principles redesign: the LIGHTS are pure WHITE, and each
 * material's distinctive look comes entirely from its own properties.
 * Under white light, gold looks like gold and blue plastic looks like
 * blue plastic — the colour is intrinsic to the material, not painted
 * on by a tinted light source.
 *
 * Fields:
 *   albedo          body diffuse colour (what the material LOOKS LIKE
 *                   under uniform white illumination)
 *   specular        F0 — Fresnel reflectance at normal incidence:
 *                     METALS:      tinted to match albedo (gold spec
 *                                  is warm yellow because gold reflects
 *                                  yellow at the highlight)
 *                     DIELECTRICS: near-white (~4% achromatic
 *                                  reflectance for non-conductors)
 *   emissive        self-luminance, added AFTER lighting — visible
 *                   even in shadow. Mostly (0,0,0); used for neon.
 *   diffuse_weight  scales the diffuse contribution from albedo:
 *                     metals ≈ 0.15, gems ≈ 0.70, plastic/ceramic ≈ 0.85,
 *                     glass ≈ 0.10, neon ≈ 0.20.
 *
 * The 20 themes below are organised into 4 families. Switching themes
 * with `t / T` cycles through all 20 in order.
 */
typedef struct {
    V3          albedo;         /* body diffuse colour                      */
    V3          specular;       /* F0 — metal: matches albedo; die: white   */
    V3          emissive;       /* self-glow (added after lighting)         */
    float       diffuse_weight; /* 0.10..0.90 — metal/dielectric scale      */
    const char *name;
} Theme;

static const Theme g_themes[] = {
    /* === METALS (12) — spec hue MATCHES albedo, low diffuse_weight ===
     * Real metals reflect nearly all incident light specularly. Their
     * F0 (Fresnel at normal incidence) is what tints the highlight. */

    /* gold     — warm yellow precious metal                            */
    {{1.00f,0.77f,0.34f}, {1.00f,0.77f,0.34f}, {0.f,0.f,0.f}, 0.15f, "gold"},
    /* silver   — bright cool precious metal, near-pure white           */
    {{0.97f,0.96f,0.92f}, {0.97f,0.96f,0.92f}, {0.f,0.f,0.f}, 0.15f, "silver"},
    /* copper   — warm orange-red metal                                 */
    {{0.96f,0.64f,0.54f}, {0.96f,0.64f,0.54f}, {0.f,0.f,0.f}, 0.15f, "copper"},
    /* bronze   — warm brown alloy (Cu+Sn)                              */
    {{0.78f,0.55f,0.30f}, {0.78f,0.55f,0.30f}, {0.f,0.f,0.f}, 0.15f, "bronze"},
    /* brass    — yellow-green alloy (Cu+Zn)                            */
    {{0.85f,0.70f,0.25f}, {0.85f,0.70f,0.25f}, {0.f,0.f,0.f}, 0.15f, "brass"},
    /* platinum — cool greyish-white precious metal                     */
    {{0.83f,0.81f,0.78f}, {0.83f,0.81f,0.78f}, {0.f,0.f,0.f}, 0.15f, "platinum"},
    /* titanium — dark silvery metal                                    */
    {{0.62f,0.60f,0.55f}, {0.62f,0.60f,0.55f}, {0.f,0.f,0.f}, 0.15f, "titanium"},
    /* iron     — neutral grey base metal                               */
    {{0.56f,0.57f,0.58f}, {0.56f,0.57f,0.58f}, {0.f,0.f,0.f}, 0.15f, "iron"},
    /* steel    — cool blue-grey alloy                                  */
    {{0.65f,0.70f,0.78f}, {0.65f,0.70f,0.78f}, {0.f,0.f,0.f}, 0.15f, "steel"},
    /* chrome   — mirror-bright cool metal                              */
    {{0.92f,0.94f,0.96f}, {0.92f,0.94f,0.96f}, {0.f,0.f,0.f}, 0.15f, "chrome"},
    /* mercury  — liquid silver                                         */
    {{0.85f,0.85f,0.88f}, {1.00f,1.00f,1.00f}, {0.f,0.f,0.f}, 0.15f, "mercury"},
    /* aluminum — pale neutral metal                                    */
    {{0.91f,0.92f,0.92f}, {0.91f,0.92f,0.92f}, {0.f,0.f,0.f}, 0.15f, "aluminum"},

    /* === GEMS (4) — saturated body + WHITE spec, mid diffuse_weight =
     * Gems are dielectrics; their Fresnel reflectance is achromatic.
     * Body colour comes from absorption inside the crystal. */

    /* ruby     — red corundum (Cr-doped)                               */
    {{0.85f,0.10f,0.18f}, {1.00f,0.95f,0.95f}, {0.f,0.f,0.f}, 0.70f, "ruby"},
    /* emerald  — green beryl (Cr-doped)                                */
    {{0.10f,0.70f,0.30f}, {0.95f,1.00f,0.95f}, {0.f,0.f,0.f}, 0.70f, "emerald"},
    /* sapphire — blue corundum (Fe/Ti-doped). Green channel tuned to
     * 0.22 (not 0.30) so the diffuse gradient crosses only ONE cube-
     * quantization boundary instead of two — eliminates the worst
     * patchy banding on the sphere's smooth normal gradient while
     * preserving brightness. Blue lifted to 0.95 for crisper saturation. */
    {{0.10f,0.22f,0.95f}, {0.95f,0.95f,1.00f}, {0.f,0.f,0.f}, 0.70f, "sapphire"},
    /* amethyst — purple quartz                                         */
    {{0.55f,0.30f,0.85f}, {1.00f,0.95f,1.00f}, {0.f,0.f,0.f}, 0.70f, "amethyst"},

    /* === DIELECTRICS (3) — body colour + WHITE spec ===================
     * Plastics, ceramics, and glass. F0 is achromatic (~4%); body
     * colour comes from sub-surface absorption. */

    /* plastic  — saturated blue plastic, full body colour              */
    {{0.20f,0.40f,0.92f}, {1.00f,1.00f,1.00f}, {0.f,0.f,0.f}, 0.85f, "plastic"},
    /* glass    — dark base + bright spec fakes transparency            */
    {{0.10f,0.12f,0.16f}, {1.00f,1.00f,1.00f}, {0.f,0.f,0.f}, 0.10f, "glass"},
    /* ceramic  — soft warm-cream porcelain                             */
    {{0.92f,0.90f,0.85f}, {1.00f,0.98f,0.95f}, {0.f,0.f,0.f}, 0.85f, "ceramic"},

    /* === EMISSIVE (1) — neon glow ====================================
     * Neon plasma is self-emissive. The albedo is the dim "off" tube
     * colour; the emissive value glows hot pink even in shadow because
     * emissive is added AFTER lighting. */

    /* neon     — hot pink/magenta self-glow                            */
    {{0.05f,0.02f,0.10f}, {0.80f,0.80f,1.00f}, {1.00f,0.20f,0.85f}, 0.20f, "neon"},
};
#define THEME_N ((int)(sizeof g_themes / sizeof g_themes[0]))

static int g_256;     /* 1 if 256-colour cube available, 0 = mono fallback */

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);
    if (g_256) {
        /* Pairs 1..216 ↔ 6×6×6 RGB cube (xterm 16..231).
         * Index = r·36 + g·6 + b + 1, all in {0..5}. */
        for (int i = 0; i < 216; i++)
            init_pair(PAIR_CUBE_BASE + i, 16 + i, -1);
    }
    init_pair(PAIR_HUD,  226, -1);          /* bright yellow                 */
    init_pair(PAIR_HINT,  51, -1);          /* bright cyan                   */
}

/*
 * draw_color — paint one cell with a colour and a luminance.
 *
 * Hue → 6×6×6 cube pair (≈ RGB resolution 1/5).
 * Luminance → ASCII-density character. Two channels of the same pixel:
 *   - colour gives "what it is" (warm gold vs cool ice)
 *   - density gives "how bright it is" (dim '. ' vs solid '@')
 * Both together convey shading on a monochrome glyph grid better than
 * either alone.
 */
static void draw_color(int row, int col, V3 c, float lum)
{
    if (lum < 0.f) lum = 0.f;
    if (lum > 1.f) lum = 1.f;
    char ch = k_ramp[(int)(lum * (RAMP_LEN - 1))];

    if (g_256) {
        int r5 = (int)(c.x * 5.f + .5f); if (r5 > 5) r5 = 5;
        int g5 = (int)(c.y * 5.f + .5f); if (g5 > 5) g5 = 5;
        int b5 = (int)(c.z * 5.f + .5f); if (b5 > 5) b5 = 5;
        int pair = PAIR_CUBE_BASE + r5*36 + g5*6 + b5;
        attron(COLOR_PAIR(pair));
        mvaddch(row, col, (chtype)(unsigned char)ch);
        attroff(COLOR_PAIR(pair));
    } else {
        mvaddch(row, col, (chtype)(unsigned char)ch);
    }
}

/* ── §5 ray-sphere intersection (THE CORE) ───────────────────────────── */

/*
 * ray_sphere — analytic ray vs sphere intersection (T2).
 *
 * Inputs:
 *   ro     ray origin (world space)
 *   rd     ray direction — MUST be unit-length (|rd| = 1)
 *   r      sphere radius (sphere is centred at origin)
 * Output:
 *   *t_hit nearest valid t such that ro + t·rd is on the sphere
 * Returns: 1 on hit, 0 on miss.
 *
 * Pseudocode (mirrors body 1:1):
 *   b    = rd · ro              half-coefficient of the linear term
 *   c    = (ro · ro) − r²       constant term
 *   disc = b² − c               (b/2)²-form discriminant
 *   if disc < 0:           ray misses sphere → return 0
 *   sq   = √disc
 *   t0   = −b − sq              near root (front face)
 *   t1   = −b + sq              far  root (back  face)
 *   if t1 < ε:             whole sphere behind us → return 0
 *   *t_hit = (t0 > ε) ? t0 : t1
 *   return 1
 *
 * Mental model: ray as a point sweeping forward in t; we ask "for
 * what t is the sweeping point exactly r away from origin?" Squaring
 * "distance equals radius" gives a quadratic; b² − c is the
 * discriminant divided by 4 (the "half-b" form, which saves us a
 * /2 divide in the root formula).
 *
 * Why use the half-b form: with full b' = 2·rd·ro, the standard
 * quadratic disc' = b'² − 4ac would equal 4·(b² − c). Computing the
 * roots costs an extra factor of 2 and a divide. The half-b form
 * eliminates both.
 *
 * Why prefer t0 over t1 normally: t0 is the entry point (front
 * face), t1 the exit (back face). For a primary ray hitting an
 * outside sphere, t0 > 0 < t1 and we want t0. Camera INSIDE the
 * sphere produces t0 < 0 < t1, and we fall back to t1 (the exit).
 *
 * The T_EPS guard rejects effectively-zero t values, preventing
 * self-intersection on bounce rays (not used here — single bounce —
 * but harmless).
 */
static int ray_sphere(V3 ro, V3 rd, float r, float *t_hit)
{
    float b    = v3dot(rd, ro);
    float c    = v3dot(ro, ro) - r * r;
    float disc = b * b - c;
    if (disc < 0.f) return 0;                /* ray misses sphere       */

    float sq = sqrtf(disc);
    float t0 = -b - sq;                      /* near root (front face) */
    float t1 = -b + sq;                      /* far root  (back face)  */

    /* Back face beyond camera = whole sphere is behind us → miss. */
    if (t1 < T_EPS) return 0;

    /* Prefer front face; fall back to back face if camera is INSIDE
     * the sphere (then t0 < 0 < t1 — exit through the far wall). */
    *t_hit = (t0 > T_EPS) ? t0 : t1;
    return 1;
}

/* ── §6 shading ──────────────────────────────────────────────────────── */

typedef enum { MODE_PHONG=0, MODE_NORMAL, MODE_FRESNEL, MODE_DEPTH, MODE_N } ShadeMode;
static const char *const k_mode_names[] = { "phong","normals","fresnel","depth" };

/* Three fixed world-space lights — POSITIONS, not directions, all
 * PURE WHITE. Per pixel we compute L = normalize(light_pos − P) so
 * each light direction depends on the hit point (point lights, not
 * directional). The lights have NO tint of their own — every visible
 * colour comes from the material (Theme.albedo + Theme.specular). */
static const V3 LIGHT_KEY  = { 3.0f, 4.0f, -2.0f };   /* upper-right       */
static const V3 LIGHT_FILL = {-4.0f, 1.0f, -1.0f };   /* upper-left        */
static const V3 LIGHT_RIM  = { 0.5f,-1.0f,  5.0f };   /* behind            */

/* §6.1 ── shade_phong: PBR-flavoured Phong (T3, T4) ──────────────────
 *
 * Inputs:
 *   P       hit point on sphere surface (world space)
 *   N       unit surface normal at P (= P/r since sphere at origin)
 *   V_dir   unit vector from P toward camera
 *   th      pointer to active Theme (albedo, F0, emissive, weight)
 * Returns: clamped RGB ∈ [0, 1]³.
 *
 * Pseudocode (mirrors body 1:1):
 *   col = AMBIENT · albedo                              ambient
 *   for (LIGHT, weight, has_specular) in {
 *        (KEY,  1.00, true),
 *        (FILL, 0.55, false),
 *        (RIM,  0.40, true with low shininess) }:
 *     L = norm(LIGHT − P)                               light direction
 *     d = max(0, N·L)                                   Lambertian
 *     col += d · diffuse_weight · weight · albedo
 *     if has_specular:
 *       R = reflect(−L, N)                              reflection of L
 *       s = max(0, R·V_dir)^shininess                   Phong cone
 *       col += s · spec_weight · F0
 *   col += emissive                                     neon adds here
 *   return clamp01(col)
 *
 * Mental model: three white spotlights at fixed positions. Each one
 * contributes diffuse (light scattering off the surface in all
 * directions, brightest where N faces L) and specular (mirror-like
 * reflection of L visible only when V_dir ≈ R). The MATERIAL decides
 * the colour:
 *   metals      diffuse_weight ≈ 0.15 (low body), F0 = albedo (tinted spec)
 *   dielectrics diffuse_weight ≈ 0.85 (full body), F0 ≈ white
 *   gems        diffuse_weight ≈ 0.70 (saturated body), F0 ≈ white
 *
 * Why split into three lights with different weights:
 *   KEY  upper-right, full-strength — primary illumination, hard spec
 *   FILL upper-left,  half-strength — lifts shadow side without
 *                                     adding a competing highlight
 *   RIM  behind,      0.4×        — silhouette glow with a wide spec
 *                                     cone (shininess = 10 not 75) so
 *                                     the back edge "kisses" the form
 *
 * Why ambient: even with three lights, points facing AWAY from all
 * of them go pitch black. AMBIENT_K = 0.20 sets a floor brightness
 * so the sphere never has fully-black cells (terminal cells with
 * pure-black RGB lose all glyph density and look like holes).
 */

static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th)
{
    /* Ambient: dim version of body albedo. */
    V3 col = v3scale(AMBIENT, th->albedo);

    /* §6.1.1 KEY light — primary diffuse + sharp specular. */
    {
        V3    L = v3norm(v3sub(LIGHT_KEY, P));
        float d = fmaxf(0.f, v3dot(N, L));
        V3    R = v3reflect(v3scale(-1.f, L), N);
        float s = powf(fmaxf(0.f, v3dot(R, V_dir)), SHININESS);
        col = v3add(col, v3scale(d * th->diffuse_weight * 1.00f, th->albedo));
        col = v3add(col, v3scale(s * 1.30f, th->specular));
    }
    /* §6.1.2 FILL light — soft diffuse, no specular. Lifts shadow side. */
    {
        V3    L = v3norm(v3sub(LIGHT_FILL, P));
        float d = fmaxf(0.f, v3dot(N, L));
        col = v3add(col, v3scale(d * th->diffuse_weight * 0.55f, th->albedo));
    }
    /* §6.1.3 RIM light — wide specular kissing the back silhouette. */
    {
        V3    L = v3norm(v3sub(LIGHT_RIM, P));
        float d = fmaxf(0.f, v3dot(N, L));
        V3    R = v3reflect(v3scale(-1.f, L), N);
        float s = powf(fmaxf(0.f, v3dot(R, V_dir)), 10.f);
        col = v3add(col, v3scale(d * th->diffuse_weight * 0.40f, th->albedo));
        col = v3add(col, v3scale(s * 1.20f, th->specular));
    }
    /* §6.1.4 Emissive — added before clamp. Lets neon glow in shadow. */
    col = v3add(col, th->emissive);

    return v3clamp1(col);
}

/* §6.5 ── shade_normal: RGB-encoded surface normal (diagnostic) ─────── */

/*
 * Each component remapped from [-1,+1] → [0,1] so all three channels
 * are valid colours. This is the canonical "normal visualisation"
 * across the entire raytracing folder — useful to verify that the
 * sphere's normals point correctly outward at every pixel.
 *
 * For the sphere centred at origin, N at the +Z silhouette is (0,0,1)
 * → RGB (0.5, 0.5, 1.0) (blue). The +X side → red-ish. The +Y top →
 * green-ish. A rotating sphere in this mode shows a familiar
 * "rainbow ball" identifying every direction by its colour.
 */
static V3 shade_normal(V3 N)
{
    return (V3){ N.x*.5f + .5f, N.y*.5f + .5f, N.z*.5f + .5f };
}

/* §6.6 ── shade_fresnel + shade_depth (alternative diagnostics, T5+T6)  */

/*
 * shade_fresnel — Schlick-Fresnel "glass marble" effect (T5).
 *
 * Inputs:  N (unit normal), V_dir (unit toward camera), th (Theme).
 * Returns: clamped RGB blending between a dim core and a bright edge.
 *
 * Pseudocode:
 *   cosθ    = |N · V_dir|                       angle to camera
 *   inv     = 1 − cosθ
 *   fresnel = inv^5                              Schlick (F₀ = 0)
 *   core    = 0.06 · albedo                      dim head-on tint
 *   edge    = clamp01(1.10 · F0)                 bright grazing tint
 *   return  clamp01( (1−fresnel)·core + fresnel·edge )
 *
 * Schlick (1994): F(θ) = F₀ + (1−F₀)·(1−cosθ)^5. We use F₀ = 0 so
 * the term collapses to (1−cosθ)^5 — pure (1−cosθ)^5 blending.
 *
 *   head-on  (cosθ ≈ 1):  fresnel ≈ 0   → core  (dim albedo tint)
 *   grazing  (cosθ ≈ 0):  fresnel ≈ 1   → edge  (bright F0 tint)
 *
 * Worked example for cosθ at various angles:
 *   cosθ = 1.00 (head-on)    fresnel = 0
 *   cosθ = 0.90 (slight tilt)  fresnel = 0.1^5 = 1·10⁻⁵ (negligible)
 *   cosθ = 0.50               fresnel = 0.5^5 = 0.03 (3%)
 *   cosθ = 0.20 (near-edge)  fresnel = 0.8^5 = 0.33 (33%)
 *   cosθ = 0.05 (silhouette) fresnel = 0.95^5 = 0.77 (77%)
 *   cosθ = 0    (limb)       fresnel = 1
 *
 * The 5th-power curve concentrates almost ALL of the brightening
 * very close to the silhouette — exactly the "glass marble" look,
 * dark across the whole interior with a bright outer ring.
 */
static V3 shade_fresnel(V3 N, V3 V_dir, const Theme *th)
{
    float cosA    = fabsf(v3dot(N, V_dir));
    float inv     = 1.f - cosA;
    float fresnel = inv * inv * inv * inv * inv;       /* (1−cosθ)^5 */
    V3 core = v3scale(0.06f, th->albedo);
    V3 edge = v3clamp1(v3scale(1.10f, th->specular));
    return v3clamp1(v3add(v3scale(1.f - fresnel, core), v3scale(fresnel, edge)));
}

/*
 * shade_depth — encode hit distance as brightness (T6).
 *
 * Pseudocode:
 *   d = 1 − min(t / t_max, 1)        normalise to [0, 1]
 *   d = d²                            steeper falloff
 *   return clamp01(d · albedo)
 *
 *   t = 0       → d = 1   → full albedo                (closest hit)
 *   t = t_max/2 → d = 0.25 → 25% albedo                (mid distance)
 *   t = t_max   → d = 0   → black                       (farthest hit)
 *
 * The squared falloff makes distant cells noticeably darker than
 * near cells — useful for "is the silhouette where I expect it"
 * sanity-check.
 */
static V3 shade_depth(float t, float t_max, const Theme *th)
{
    float d = 1.f - fminf(t / t_max, 1.f);
    d = d * d;
    return v3clamp1(v3scale(d, th->albedo));
}

/* Rec. 601 luminance for ramp-index choice. */
static inline float rec601_luma(V3 c)
{
    return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/* ── §7 render frame ─────────────────────────────────────────────────── */

/*
 * render — paint one frame.
 *
 * Inputs:
 *   cols, rows  terminal dimensions
 *   orbit_ang   camera orbit angle (radians)
 *   cam_dist    camera orbit radius from origin (world units)
 *   theme_idx   which Theme is active
 *   mode        shading mode (PHONG / NORMAL / FRESNEL / DEPTH)
 *
 * Pseudocode (mirrors body 1:1):
 *   th       = &themes[theme_idx]
 *   fov_tan  = tan(FOV/2)
 *   cam      = orbit position from cam_dist + orbit_ang
 *   fwd      = norm(origin − cam)             look toward origin
 *   right    = norm(fwd × world_up)
 *   up       = right × fwd                     orthonormal basis
 *   for each cell (col, row) skipping bottom row (HUD):
 *     pu, pv = pixel-to-screen-space coords (centred at 0,0)
 *     rd    = norm(fwd + pu·right + pv·up)
 *     if not ray_sphere(cam, rd, R, &t): paint background; continue
 *     P     = cam + t·rd                      hit point
 *     N     = norm(P)                          normal (sphere at origin)
 *     V_dir = norm(cam − P)
 *     col, lum = SHADE(mode, P, N, V_dir, t, th)
 *     draw_color(row, col, color, lum)
 *
 * THE ORBITING-CAMERA TRICK:
 * Instead of rotating the sphere, we orbit the CAMERA around it at
 * fixed distance. Mathematically equivalent, but keeps the sphere
 * fixed at origin so ray_sphere() needs no centre argument:
 *
 *   cam = (cam_dist · sin(θ),  CAM_HEIGHT,  -cam_dist · cos(θ))
 *
 *   At θ=0 the camera sits at (0, h, -d) — looking toward +Z.
 *   At θ=π/2 the camera is at (d, h, 0) — looking toward -X.
 *   The camera basis (fwd, right, up) is rebuilt each frame from the
 *   look direction toward the origin.
 */
static void render(int cols, int rows,
                   float orbit_ang, float cam_dist,
                   int theme_idx, ShadeMode mode)
{
    const Theme *th  = &g_themes[theme_idx % THEME_N];
    float fov_tan    = tanf(FOV_DEG * (float)M_PI / 360.f);

    /* §7.1 — orbiting camera + look-at basis. */
    V3 cam = { cam_dist * sinf(orbit_ang), CAM_HEIGHT, -cam_dist * cosf(orbit_ang) };
    V3 fwd = v3norm(v3sub((V3){0,0,0}, cam));
    V3 wup = { 0.f, 1.f, 0.f };
    V3 rgt = v3norm(v3cross(fwd, wup));
    V3 up  = v3cross(rgt, fwd);

    float cx = cols * 0.5f, cy = rows * 0.5f;

    /* §7.2 — primary loop: one ray per cell. Skip bottom row for HUD. */
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            /* Normalised screen coords with terminal-cell aspect baked in. */
            float pu =  (col - cx) / cx * fov_tan;
            float pv = -(row - cy) / cx * fov_tan / ASPECT;

            V3 rd = v3norm(v3add(fwd, v3add(v3scale(pu, rgt),
                                            v3scale(pv, up))));

            float t_hit;
            if (!ray_sphere(cam, rd, SPHERE_R, &t_hit)) continue;

            V3 P     = v3add(cam, v3scale(t_hit, rd));
            V3 N     = v3norm(P);                 /* sphere centred at origin */
            V3 V_dir = v3norm(v3sub(cam, P));

            V3    color;
            float lum;

            switch (mode) {
            default:
            case MODE_PHONG:
                color = shade_phong(P, N, V_dir, th);
                lum   = rec601_luma(color);
                break;
            case MODE_NORMAL:
                color = shade_normal(N);
                /* Green-weighted luma in NORMAL mode tracks the
                 * "green is brightest" intuition the eye applies to
                 * RGB normal visualisations. */
                lum   = (N.x*.5f+.5f)*.3f + (N.y*.5f+.5f)*.6f + (N.z*.5f+.5f)*.1f;
                break;
            case MODE_FRESNEL:
                color = shade_fresnel(N, V_dir, th);
                lum   = rec601_luma(color);
                break;
            case MODE_DEPTH:
                color = shade_depth(t_hit, cam_dist * 2.2f, th);
                lum   = rec601_luma(color);
                break;
            }

            draw_color(row, col, color, lum);
        }
    }
}

/* ── §8 screen / HUD ─────────────────────────────────────────────────── */

static void hud_draw(int cols, int rows, float fps,
                     int theme_idx, ShadeMode mode, float cam_dist, int paused)
{
    /* §8.1 top-right status row 0 — yellow, BOLD (HUD spec). */
    char buf[96];
    snprintf(buf, sizeof buf, " %5.1f fps  dist:%.1f  %-9s  %s ",
             (double)fps, (double)cam_dist,
             g_themes[theme_idx % THEME_N].name,
             paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    if (len > cols) len = cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - len, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* §8.2 top-left mode label row 0 — same yellow, no bold (secondary). */
    char buf2[48];
    snprintf(buf2, sizeof buf2, " mode:%-9s ", k_mode_names[mode]);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(0, 0, "%s", buf2);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* §8.3 bottom hint strip — cyan, BOLD. ASCII only. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc/p:pause  s:mode  t:theme  +/-:zoom ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §9 app ──────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run    = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint  (int s) { (void)s; g_run    = 0; }
static void on_sigwinch(int s) { (void)s; g_resize = 1; }

static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   on_sigint);
    signal(SIGTERM,  on_sigint);
    signal(SIGWINCH, on_sigwinch);

    initscr();
    cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    atexit(cleanup);
    color_init();

    int cols, rows;
    getmaxyx(stdscr, rows, cols);

    int       theme_idx = 0;
    ShadeMode mode      = MODE_PHONG;
    float     cam_dist  = CAM_DIST_DEF;
    float     orbit_ang = 0.f;
    int       paused    = 0;

    float     fps       = 0.f;
    long long fps_acc   = 0;
    int       fps_cnt   = 0;
    long long frame_ns  = 1000000000LL / TARGET_FPS;
    long long last      = clock_ns();

    while (g_run) {
        /* §9.1 resize. */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
        }

        /* §9.2 timing. dt is wall-clock; cap to avoid huge jumps after
         * a stall (debugger pause, suspend/resume). */
        long long now = clock_ns();
        long long dt  = now - last;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;
        last = now;

        /* §9.3 advance orbit angle if not paused. */
        if (!paused) orbit_ang += ORBIT_SPEED * (float)dt * 1e-9f;

        /* §9.4 fps rolling average over half-second windows. */
        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps     = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        /* §9.5 paint frame. */
        long long t0 = clock_ns();
        erase();
        render(cols, rows, orbit_ang, cam_dist, theme_idx, mode);
        hud_draw(cols, rows, fps, theme_idx, mode, cam_dist, paused);
        wnoutrefresh(stdscr);
        doupdate();

        /* §9.6 input. */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27 /* ESC */:
            g_run = 0; break;
        case ' ': case 'p': case 'P':
            paused = !paused; break;
        case 's': case 'S':
            mode = (ShadeMode)((mode + 1) % MODE_N); break;
        case 't': case 'T':
            theme_idx = (theme_idx + 1) % THEME_N; break;
        case '+': case '=':
            cam_dist -= CAM_DIST_STEP;
            if (cam_dist < CAM_DIST_MIN) cam_dist = CAM_DIST_MIN;
            break;
        case '-': case '_':
            cam_dist += CAM_DIST_STEP;
            if (cam_dist > CAM_DIST_MAX) cam_dist = CAM_DIST_MAX;
            break;
        default: break;
        }

        /* §9.7 frame cap. */
        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }
    return 0;
}
