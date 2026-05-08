/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * capsule_raytrace.c — analytic ray-trace of a capsule (segment + ball).
 *
 * DEMO: A glossy pill-shaped capsule rotating in space, lit by three
 *       coloured lights. Cycle four shading modes (phong → normals →
 *       fresnel → depth) to see how the same geometry feeds different
 *       visual outputs, and three debug overlays (off → hit-type →
 *       axial → discriminant) to SEE the intersection algorithm work.
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c   — same skeleton, ONE quadratic
 *   raytracing/torus_raytrace.c    — same skeleton, QUARTIC (much harder)
 *   raytracing/cube_raytrace.c     — same skeleton, slab method
 *
 * Section map:
 *   §1  config   — frame rate, FOV, capsule geometry, ramp, palette IDs
 *   §2  clock    — monotonic timer + sleep
 *   §3  math     — V3, Mat3 with explicit object↔world transforms
 *   §4  colour   — themes + 256-colour cube + ASCII-ramp painter
 *   §5  capsule  — THE CORE: ray-vs-capsule decomposition
 *                  §5.1 cylinder body test
 *                  §5.2 cap (sphere) test
 *                  §5.3 dispatcher
 *   §6  shading  — phong, normals, fresnel, depth
 *   §7  debug    — overlays that visualise the §5 intermediate state
 *   §8  render   — one frame, ray per cell
 *   §9  screen   — ncurses HUD (yellow status row, cyan hint strip)
 *   §10 app      — signals, input, main loop
 *
 * Keys:
 *   s         cycle shade mode (phong → normals → fresnel → depth)
 *   d         cycle debug overlay (off → hit-type → axial → discrim)
 *   t / T     cycle theme — 20 materials in 4 families:
 *               metals (12)  gold, silver, copper, bronze, brass, platinum,
 *                            titanium, iron, steel, chrome, mercury, aluminum
 *               gems    (4)  ruby, emerald, sapphire, amethyst
 *               dielectrics  plastic, glass, ceramic
 *               emissive     neon
 *   p / SPC   pause / resume rotation
 *   + / =     zoom in
 *   -         zoom out
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/capsule_raytrace.c \
 *       -o capsule_rt -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL (in this order, as prose).
 *   2. §1 config — the constants you can twiddle.
 *   3. §5 capsule (the core math) — read AFTER tutorials T2-T6.
 *   4. §8 render — see how the math is fed by per-pixel rays.
 *   5. Other sections only if curious.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Long descriptive names everywhere — `axis_dot_dir` not `bard`,
 *   `discriminant` not `h`. Every line becomes a self-explanatory
 *   sentence; no glossary required.
 *
 *   Suffixes name COORDINATE SPACE explicitly:
 *     `_world`  scene-fixed coordinates (camera & lights live here)
 *     `_obj`    capsule-local coordinates (axis is Y)
 *
 *   When a single name lacks a suffix it's space-independent
 *   (e.g. `radius`, `discriminant`).
 *
 * Background you need
 * ───────────────────
 *   - 3-D vector math (dot, scale, normalise).
 *   - The quadratic formula and what its discriminant means
 *     geometrically (positive → two real roots → ray hits the surface).
 *   - That a 3×3 rotation matrix has inverse = transpose.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Any prior raymarching/raytracing knowledge — the file teaches it.
 *   - GPU shading languages — everything is plain CPU-side C.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-vs-capsule. A capsule is the Minkowski
 *                  sum of a finite line segment AB and a ball of radius
 *                  r — every point within distance r of the segment.
 *                  The ray test decomposes into:
 *                    (a) ray vs INFINITE CYLINDER along AB,
 *                    (b) ray vs HEMISPHERE CAP at endpoint A or B.
 *                  Whichever yields the smallest valid t is the hit.
 *                  Closed-form throughout — no marching, no mesh.
 *
 * Data-structure : A capsule is just three numbers' worth of state:
 *                    A : V3      first endpoint
 *                    B : V3      second endpoint
 *                    r : float   tube + cap radius
 *                  Object-space convention: A and B sit on the Y axis;
 *                  world-space orientation is recovered by rotating the
 *                  RAY into object space (much cheaper than rotating
 *                  the capsule every frame — see T7).
 *
 * Rendering      : One ray per terminal cell, no anti-aliasing. The
 *                  surface is shaded by one of four modes (phong /
 *                  normal / fresnel / depth) and optionally re-tinted
 *                  by one of three debug overlays. Hue → 256-colour
 *                  6×6×6 cube, brightness → 92-char Bourke ramp.
 *
 * Performance    : Pure analytic — one quadratic per pixel for the
 *                  body test, optionally a second for the nearer cap.
 *                  No iteration, no spatial structures. Heaviest cost
 *                  is the shading (powf for the specular highlight); a
 *                  capsule is one of the cheapest non-trivial
 *                  primitives to ray-trace.
 *
 * References     : Inigo Quílez, "Capsule — intersection",
 *                    https://iquilezles.org/articles/intersectors/
 *                  Real-Time Rendering 4e §22.7 (ray-cylinder, ray-sphere).
 *                  Shirley & Marschner, "Fundamentals of Computer Graphics"
 *                    4e ch. 4 (raytracing primitives).
 *                  Schlick, "An Inexpensive BRDF Model for Physically-based
 *                    Rendering", Comp. Graph. Forum 13(3), 1994 [fresnel].
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A capsule is a sphere SMEARED along a line segment. So solving "where
 * does my ray hit the capsule?" splits naturally into "where does my
 * ray hit the cylindrical part?" and "where does it hit the rounded
 * ends?". Each sub-problem reduces to a quadratic in t, and we keep
 * whichever answer is the closest valid hit. Three quadratics, no
 * iteration — done.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a soap bubble whose centre is dragged along a finite line
 * segment from A to B. As the centre moves, the bubble traces out a
 * tube of radius r between A and B and leaves a hemispherical "cap" at
 * each endpoint.
 *
 *           ╭────────────────╮          ← cap B (hemisphere)
 *          ╱                  ╲
 *         (         B          )
 *         │                    │
 *         │  ←   tube body  →  │       ← cylindrical band of radius r
 *         │                    │
 *         (         A          )
 *          ╲                  ╱
 *           ╰────────────────╯          ← cap A (hemisphere)
 *
 * For ray-tracing, we treat the band as an infinite cylinder and check
 * AFTERWARD whether the hit lies between A and B along the axis. If it
 * does, body hit. If it doesn't, retest the nearer endpoint as a
 * sphere. Total quadratics solved per pixel: at most two.
 *
 * Coordinate-system pipeline (T8 unpacks each step):
 *
 *       SCREEN ─FOV─→ CAMERA ─cam basis─→ WORLD ─Mᵀ─→ OBJECT
 *                                                        ↓
 *                                                   intersection
 *                                                        ↓
 *       OBJECT ────M────→ WORLD ──────→ shading ──→ pixel
 *
 * Everything left of "intersection" is per-frame ceremony; the
 * intersection happens in OBJECT space where the capsule's axis is
 * fixed at Y. Everything right of it is shading.
 *
 * ALGORITHM IN STEPS  (per pixel)
 * ───────────────────────────────
 *  1. Build a primary ray (origin + direction) in WORLD space from the
 *     screen coordinates and the camera's basis.
 *  2. Apply Mᵀ (the rotation matrix's transpose) to the ray, putting
 *     it in OBJECT space where the capsule lives along the Y axis.
 *  3. Solve ray vs INFINITE CYLINDER:
 *        a = |ba|² − (ba·rd)²
 *        b = |ba|²(rd·oa) − (ba·oa)(ba·rd)
 *        c = |ba|²(|oa|² − r²) − (ba·oa)²
 *        h = b² − a·c                          (discriminant)
 *        t = (−b − √h)/a                       (front face)
 *        y = (ba·oa) + t·(ba·rd)               (axial coord of hit)
 *     If 0 < y < |ba|², it's a BODY hit. Compute the radial normal:
 *        N_obj = normalize( (oa + t·rd) − (y/|ba|²)·ba )
 *  4. Otherwise the ray either missed the cylinder or hit it outside
 *     the segment. Pick the cap to test:
 *        y ≤ 0     → cap at A
 *        y ≥ |ba|² → cap at B
 *  5. Ray-vs-sphere quadratic at the chosen cap centre:
 *        oc = (object-space ray origin) − (cap centre)
 *        b' = rd · oc      c' = oc·oc − r²      h' = b'² − c'
 *        t  = −b' − √h'
 *     If h' < 0 the ray missed — return "no hit".
 *     Otherwise N_obj = normalize(oc + t·rd).
 *  6. Bring the normal back to WORLD space: N_world = M · N_obj.
 *  7. Shade in WORLD space (phong / normals / fresnel / depth).
 *  8. Optionally re-tint by the active debug overlay.
 *
 * KEY FORMULAS
 * ────────────
 *   ba   = B − A                    (segment vector)
 *   oa   = ro − A                   (ray origin relative to A)
 *   baba = |ba|²                    (axis length squared, used to skip
 *                                    explicit normalisation of ba)
 *
 *   CYLINDER QUADRATIC (Quílez, division-free until the very end):
 *     a = baba − (ba·rd)²
 *     b = baba·(rd·oa) − (ba·oa)·(ba·rd)
 *     c = baba·(|oa|² − r²) − (ba·oa)²
 *     h = b² − a·c
 *     t = (−b − √h) / a
 *     y = (ba·oa) + t·(ba·rd)
 *
 *   BODY NORMAL  (radial outward, axial component removed):
 *     N_obj = normalize( (oa + t·rd) − (y/baba)·ba )
 *
 *   CAP SPHERE QUADRATIC:
 *     b' = rd · oc      c' = oc·oc − r²      h' = b'² − c'
 *     t  = −b' − √h'
 *
 *   CAP NORMAL  (away from cap centre):
 *     N_obj = normalize( oc + t·rd )
 *
 *   WORLD/OBJECT TRANSFORMS:
 *     ray_obj    = (Mᵀ·cam_world, Mᵀ·dir_world)   (M orthogonal ⇒ M⁻¹=Mᵀ)
 *     N_world    = M · N_obj
 *
 *   SCHLICK FRESNEL (used in shade mode):
 *     F = F₀ + (1 − F₀)·(1 − cosθ)⁵        with cosθ = |N · V_dir|
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Ray parallel to the axis: (ba·rd)² → baba so a → 0, dividing by
 *    zero. Mathematically the cylinder term degenerates; only the caps
 *    see the ray. In practice axis-parallel rays are zero-measure for
 *    a rotating capsule but a guarded `if (a < 1e-9) skip cylinder`
 *    would harden it.
 *  • t < T_EPS reject. Without this the camera grazing or starting
 *    inside the surface would trigger spurious self-hits at t ≈ 0.
 *  • Body-cap seam (y = 0 or y = baba): the body normal and cap normal
 *    AGREE (both point radially outward from the same circle) so no
 *    crease appears — shading is C¹ across the join.
 *  • Ramp index uses `(int)(luminance * (RAMP_LEN−1))` so luminance=1
 *    maps to the densest character, not to RAMP_LEN (out of bounds).
 *  • The dispatcher reuses `axial_norm` to pick a cap when the
 *    cylinder misses. A pure miss leaves axial_norm uninitialised, but
 *    the same miss implies "perp distance to axis > r", so cap A's
 *    sphere test will return MISS too — the default 0 is safe.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • End-on view (camera looks down the capsule's axis): you should
 *    see a circular DISC, not a pill — only the front cap is visible.
 *    Switch to NORMALS: the disc is overwhelmingly blue (N ≈ −Z).
 *  • Side-on view: classic pill silhouette — central rectangle of
 *    width 2r flanked by two arcs of radius r.
 *  • `d` → HIT-TYPE overlay: at side-on view a cyan band (body) capped
 *    by magenta and yellow blobs (cap A, cap B).
 *  • `d` → AXIAL overlay: at side-on view a smooth bottom-to-top
 *    gradient across the body, constant on each cap.
 *  • `d` → DISCRIM overlay: bright disc, brightest on-axis, fading
 *    to black at the silhouette where the ray grazes (h → 0).
 *  • Worked example (T2 + T3 in tutorials): a head-on ray to a
 *    unit-tall capsule at distance 3.4 should hit at t ≈ 3.05.
 *  • Doubling CAP_R should double the apparent thickness of the body
 *    and leave proportional caps.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Ten short tutorials that build the algorithm from first principles.
 * Read in order; each builds on the previous.
 *
 *   T1  What problem are we solving?
 *   T2  The simplest case: ray vs SPHERE
 *   T3  The next case:    ray vs INFINITE CYLINDER
 *   T4  Why the Quílez form has no division
 *   T5  Bound-checking the cylinder hit (axial coordinate)
 *   T6  Choosing the right cap on a fall-through
 *   T7  The inverse-rotation trick (rotate the RAY)
 *   T8  Coordinate systems at a glance
 *   T9  Four shading modes — same geometry, four pictures
 *   T10 White light is the lab; the material is the specimen
 *   T11 Metals vs dielectrics — F0, Fresnel, and the spec-tint trick
 *   T12 Three debug overlays — making the algorithm visible
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT PROBLEM ARE WE SOLVING?
 * ────────────────────────────────
 * Given:
 *   • a ray  (origin O, unit direction D)
 *   • a capsule  (segment AB, radius r)
 * Find:
 *   • the smallest t > 0 such that O + t·D lies on the capsule's
 *     surface,
 *   • the outward unit normal at that point.
 *
 * Why a capsule and not a more general shape? Because capsules pop up
 * everywhere — limbs of stick figures, wires, swords, animal bones —
 * AND have closed-form intersection AND have rounded ends that hide
 * tessellation artifacts on a low-res terminal.
 *
 * The trick we'll exploit: a capsule is the SET-WISE UNION of a finite
 * cylinder (the "tube body") with two hemispheres (the "caps"), all
 * sharing the same surface. So instead of solving one nasty implicit
 * equation, we solve up to three textbook quadratics and combine.
 *
 * T2  THE SIMPLEST CASE: RAY VS SPHERE
 * ────────────────────────────────────
 * Sphere of centre C, radius r:
 *
 *     |O + t·D − C|² = r²
 *
 * Expand and collect terms in t. With oc = O − C:
 *
 *     (D·D)·t² + 2·(D·oc)·t + (oc·oc − r²) = 0
 *
 * Since D is a unit vector, D·D = 1, so:
 *
 *     t² + 2·b·t + c = 0     where b = D·oc, c = oc·oc − r²
 *     t = −b ± √(b² − c)
 *
 *     b² − c is the DISCRIMINANT. If it's negative, the ray misses.
 *     The smaller root (−b − √…) is the FRONT face hit.
 *
 * Pseudocode:
 *
 *     b = dot(D, O − C)
 *     c = |O − C|² − r²
 *     h = b·b − c
 *     if (h < 0) return MISS
 *     t = −b − sqrt(h)
 *     N = normalize((O + t·D) − C)
 *
 * Read the file's `cap_test` (§5.2) — that IS this exact formula, with
 * `oc` precomputed by the dispatcher.
 *
 * T3  THE NEXT CASE: RAY VS INFINITE CYLINDER
 * ────────────────────────────────────────────
 * A cylinder of axis line (point A, unit direction n̂, radius r) is the
 * locus of points whose perpendicular distance to the axis equals r:
 *
 *     |(P − A) − ((P − A)·n̂)·n̂|  =  r
 *
 * The expression in absolute-value brackets is "(P−A) with its axial
 * component removed". So the equation says: project P onto the plane
 * perpendicular to the axis; that 2-D projection is at distance r from
 * the axis line. In other words, RAY-VS-CYLINDER is morally a 2-D
 * RAY-VS-CIRCLE viewed in the perp-plane.
 *
 * Substitute P = O + t·D and square both sides to drop the absolute
 * value. Define D_perp = D − (D·n̂)·n̂ and oa_perp similarly. The
 * equation collapses to:
 *
 *     |D_perp|² · t²
 *   + 2·(D_perp · oa_perp) · t
 *   + (|oa_perp|² − r²) = 0          where oa = O − A
 *
 * — a textbook 2-D ray-vs-circle quadratic. We've literally reduced
 * the 3-D cylinder problem to a 2-D circle problem in the perp-plane.
 *
 * T4  WHY THE QUÍLEZ FORM HAS NO DIVISION
 * ────────────────────────────────────────
 * Computing |D_perp|² etc. requires us to first NORMALISE n̂, i.e.
 * divide ba by |ba|. Quílez's trick is to rescale the WHOLE quadratic
 * by baba = |ba|² so the divisions cancel:
 *
 *     a = baba − (ba·D)²              (= baba · |D_perp|²)
 *     b = baba·(D·oa) − (ba·oa)·(ba·D) (= baba · (D_perp · oa_perp))
 *     c = baba·(|oa|² − r²) − (ba·oa)² (= baba · (|oa_perp|² − r²))
 *     h = b² − a·c                    (sign matches the original)
 *     t = (−b − √h) / a               (the baba factor cancels)
 *
 * Now there's only ONE division (the final one) and zero divisions
 * involving square roots. Numerically robust and a hair faster than
 * the normalised form. The discriminant h has the right SIGN so the
 * miss/hit decision is unaffected.
 *
 * Read §5.1 `cylinder_test` to see the seven-line implementation.
 *
 * T5  BOUND-CHECKING THE CYLINDER HIT (AXIAL COORDINATE)
 * ───────────────────────────────────────────────────────
 * The cylinder above is INFINITE — the formula doesn't know where A
 * and B are along the axis. So once we have a hit at parameter t we
 * ask: "WHERE along the axis did we hit?"
 *
 *     y = (ba·oa) + t·(ba·D)
 *
 * y is the axial coordinate scaled by baba. So the hit lies between
 * A and B iff:
 *
 *     0 ≤ y ≤ baba           (after scaling, [0, baba] = [A, B])
 *
 * Geometrically:
 *
 *                   y < 0           0 ≤ y ≤ baba       y > baba
 *                      │                  │                │
 *     A ●═══════════════════════ tube body ══════════════════● B
 *     (cap A's domain)        (cylinder body)        (cap B's domain)
 *
 * In-bounds → return body hit. Out-of-bounds → fall through to the
 * cap test (T6). The same y tells us WHICH cap.
 *
 * T6  CHOOSING THE RIGHT CAP ON A FALL-THROUGH
 * ─────────────────────────────────────────────
 * If y ≤ 0 the would-be cylinder hit is on cap A's side of the
 * segment; if y ≥ baba it's on cap B's side. So we test only ONE cap,
 * not both — picking by the sign of y:
 *
 *     y ≤ 0     → test sphere centred at A
 *     y ≥ baba  → test sphere centred at B
 *
 * Why does this work? Because if the ray were going to hit the OTHER
 * cap, the cylinder hit would have been on that cap's side instead.
 * (Geometric reason: a straight ray crosses any half-plane in a
 * single direction.)
 *
 * Edge case: what if the cylinder MISSED entirely (h < 0)? Then we
 * never wrote y. Quirk: a ray that misses the infinite cylinder also
 * cannot hit either cap (its perpendicular distance to the axis is
 * already > r). So testing cap A (or whatever) returns MISS too —
 * the algorithm is still correct, just spends one extra quadratic.
 *
 * T7  THE INVERSE-ROTATION TRICK (ROTATE THE RAY)
 * ────────────────────────────────────────────────
 * Naive way: every frame, rotate the capsule's endpoints A and B by
 * the current orientation matrix M. Cost: 6 floats per frame. Fine
 * for one capsule; generalises poorly to scenes with many primitives.
 *
 * Smarter way: keep A and B FIXED at the canonical Y axis, and rotate
 * the RAY into the capsule's local space:
 *
 *     ray_obj.origin    = Mᵀ · ray_world.origin
 *     ray_obj.direction = Mᵀ · ray_world.direction
 *
 * Why Mᵀ? Because rotation matrices are ORTHOGONAL — their inverse
 * equals their transpose. Computing Mᵀ is free (just access pattern);
 * computing M⁻¹ generally requires a determinant and division.
 *
 * After intersecting in OBJECT space we get a normal N_obj. Bring it
 * back to world space:
 *
 *     N_world = M · N_obj
 *
 * because rotation matrices preserve length and orthogonality (no
 * "inverse-transpose for normals" gymnastics needed for non-uniform
 * scaling — there is no scaling here).
 *
 * Cost: ONE matrix-vector multiply per ray instead of N matrix-vector
 * multiplies per frame for N capsules. Wins big as scenes grow.
 *
 * T8  COORDINATE SYSTEMS AT A GLANCE
 * ───────────────────────────────────
 * Four spaces appear in this file. Keeping them straight avoids hours
 * of confusion:
 *
 *      SCREEN          (col, row)         pixel grid, integer
 *        │
 *        │  pu = (col − cx)/cx · tan(FOV/2)
 *        │  pv = −(row − cy)/cx · tan(FOV/2) / ASPECT
 *        ▼
 *      CAMERA          (pu, pv)           normalised, axis-aligned
 *        │
 *        │  ray_world = camera_origin
 *        │  view_dir  = normalize(forward + pu·right + pv·up)
 *        ▼
 *      WORLD           (x, y, z)          scene-fixed coords; lights live here
 *        │
 *        │  ray_obj.origin = Mᵀ · ray_world.origin
 *        │  ray_obj.dir    = Mᵀ · ray_world.dir
 *        ▼
 *      OBJECT          (x, y, z)          capsule axis fixed at Y
 *        │
 *        │  intersection (§5)
 *        ▼
 *      hit_point_obj, normal_obj
 *        │
 *        │  hit_point_world = camera_origin + t · view_dir   (in world!)
 *        │  normal_world    = M · normal_obj
 *        ▼
 *      WORLD shading (§6) → pixel colour → SCREEN.
 *
 * Note one asymmetry: we transform the RAY into object space but
 * compute the hit point in WORLD space (using the world-space ray +
 * the t we obtained). Either is mathematically correct because t is
 * the same in both spaces (rotation preserves distances). Using
 * world-space hit points lets shading use world-space lights with no
 * extra transforms.
 *
 * T9  FOUR SHADING MODES — SAME GEOMETRY, FOUR PICTURES
 * ──────────────────────────────────────────────────────
 * After we know the hit point and its normal we still have to turn
 * those into a colour. Four orthogonal modes (cycle with `s`):
 *
 *   PHONG    Three pure-WHITE lights (key + fill + rim). Diffuse =
 *            max(N·L, 0); specular = (R·V)^shininess. The material's
 *            distinctive colour comes from its OWN albedo and spec
 *            tint — the lights are white so each material reads as
 *            itself, not as a coloured filter. Diffuse weight per
 *            material: metals ≈ 0.15 (spec-dominated), gems ≈ 0.70,
 *            dielectrics ≈ 0.85, glass ≈ 0.10. Emissive (added after
 *            lighting, before clamp) lets neon glow in shadow.
 *   NORMALS  N → (N+1)/2, channel-by-channel. RGB-encodes the
 *            surface normal as a colour. A correct intersection shows
 *            a smooth radial gradient on the body and hemispherical
 *            colour wheels on the caps. Useful for verifying
 *            intersection correctness.
 *   FRESNEL  Schlick: F = F₀ + (1−F₀)(1−|N·V|)⁵. Looks like glass —
 *            dark facing, glowing edge. Best mode for showing the
 *            silhouette transition between body and caps (which
 *            should be invisible — see EDGE CASES).
 *   DEPTH    Brightness from t: closer = brighter. Useful as a
 *            sanity check (the centre of the silhouette should be the
 *            brightest spot in DEPTH mode — closest to the camera).
 *
 * T10 WHITE LIGHT IS THE LAB; THE MATERIAL IS THE SPECIMEN
 * ─────────────────────────────────────────────────────────
 * A common amateur-lighting trick is to TINT the lights to flatter
 * each material — warm key for gold, cool key for silver, blue rim
 * for plastic. It looks slick at first glance but it CONFLATES two
 * orthogonal things: "what the LIGHT is doing" and "what the
 * MATERIAL is".
 *
 * The first-principles fix: keep all lights pure WHITE. Then any
 * colour the eye sees comes from the MATERIAL itself. This is how
 * PBR (physically-based rendering) sees the world, and the same
 * logic an art gallery uses — neutral white-balanced spotlights so
 * each painting shows its TRUE colours. A warm gallery light would
 * make every Mondrian lean orange.
 *
 * In this file, every per-light colour is unit white:
 *
 *     LIGHT_KEY  = (1, 1, 1)         pure white sun
 *     LIGHT_FILL = (1, 1, 1)         pure white fill
 *     LIGHT_RIM  = (1, 1, 1)         pure white rim
 *
 * The KEY/FILL/RIM weights still differ in INTENSITY (KEY brightest,
 * RIM dimmest, geometric falloff per light position) but they're all
 * achromatic. So:
 *
 *     gold  albedo + gold  spec   →  looks like real gold
 *     blue  albedo + WHITE spec   →  looks like real blue plastic
 *     dark  albedo + WHITE spec   →  looks like real glass
 *     dim   albedo + EMISSIVE     →  looks like real neon
 *
 * Material-as-material, not material-tinted-by-light.
 *
 * Try it:
 *  - Pause (`p`), cycle `t` through all 20 materials. Each material
 *    should be recognisable as ITSELF — gold reads gold, plastic
 *    reads plastic, glass reads glass, neon glows pink.
 *  - If the lights were tinted, every material would lean toward
 *    the dominant light's hue and the recognition would break.
 *
 * T11 METALS VS DIELECTRICS — F0, FRESNEL, AND THE SPEC-TINT TRICK
 * ─────────────────────────────────────────────────────────────────
 * Three numbers describe how light interacts with a surface:
 *
 *   ALBEDO   the body colour from sub-surface absorption — what you
 *            see when the surface is EVENLY illuminated and viewed
 *            head-on (no highlight).
 *   F0       the fraction of light reflected at NORMAL INCIDENCE
 *            (cell facing camera, 0° angle to view).
 *   FRESNEL  the curve raising reflectance to ≈ 1 at GRAZING angles
 *            (cell at 90° to view, like the rim of the silhouette).
 *
 * Real materials split into two regimes along the F0 axis:
 *
 *   METALS       F0 has the SAME TINT as the visible body colour.
 *                  gold's   measured F0 = (1.000, 0.766, 0.336)  ← yellow!
 *                  silver's measured F0 = (0.972, 0.960, 0.915)  ← white-ish
 *                  copper's measured F0 = (0.955, 0.638, 0.538)  ← pink-orange
 *                That tinted F0 is what makes a metal look "metallic"
 *                — its highlights reflect the metal's own colour
 *                because metals are conductors and light can't enter
 *                the bulk to be re-coloured by absorption. Whatever
 *                gets reflected gets reflected AT THE METAL'S TINT.
 *
 *   DIELECTRICS  F0 is achromatic: ~0.04 (4%) across the spectrum.
 *                Plastic, glass, gem, ceramic, wood, water — all of
 *                them. The body colour comes ENTIRELY from absorption
 *                inside the material; the highlight is a small WHITE
 *                fraction reflected off the surface.
 *
 * Why it matters: the DIFFERENCE between gold and yellow plastic is
 * not the body colour (both yellow) but the highlight — gold's
 * highlight is YELLOW, plastic's is WHITE. That single tint
 * difference is what makes "metal" look like metal and "plastic"
 * look like plastic.
 *
 * Our Theme struct encodes this directly:
 *
 *   typedef struct {
 *       V3 albedo;          body colour
 *       V3 specular;        F0:  metal → albedo;  dielectric → white
 *       V3 emissive;        self-glow (mostly 0; neon glows hot pink)
 *       float diffuse_weight;
 *       const char *name;
 *   } Theme;
 *
 * The `diffuse_weight` field captures the OTHER physical truth:
 * metals reflect almost everything specularly, with very little
 * diffuse contribution. So:
 *
 *   metal:        diffuse_weight = 0.15  (low; mostly specular reflects)
 *   gem:          diffuse_weight = 0.70  (saturated body + bright spec)
 *   plastic:      diffuse_weight = 0.85  (full body colour)
 *   glass:        diffuse_weight = 0.10  (dark body, bright white spec)
 *   neon:         diffuse_weight = 0.20  (low diffuse + huge emissive)
 *
 * The §6.1 KEY block of `shade_phong` makes this concrete:
 *
 *     // Diffuse contribution: WHITE light × albedo × diffuse_weight
 *     colour += diffuse · diffuse_weight · albedo
 *
 *     // Specular contribution: WHITE light × F0
 *     //   metal       F0 = tinted albedo  → tinted highlight
 *     //   dielectric  F0 = (1, 1, 1)      → white highlight
 *     colour += specular · 1.30 · spec_F0
 *
 * Try it:
 *  - Cycle to gold (`t` until name says "gold") — watch the highlight
 *    on the bright side of the capsule. WARM YELLOW.
 *  - Cycle to plastic — same geometry, same lighting; highlight is
 *    pure WHITE. The body is blue but the highlight is white. That's
 *    the dielectric signature.
 *  - Cycle to ruby (red gem dielectric) — body deep red, highlight
 *    near-white. Same dielectric signature.
 *  - Cycle to neon — even on the SHADOW side, the surface still glows
 *    hot pink because the emissive value is added AFTER the lighting
 *    sum and BEFORE the clamp.
 *
 * T12 THREE DEBUG OVERLAYS — MAKING THE ALGORITHM VISIBLE
 * ────────────────────────────────────────────────────────
 * Cycling `d` switches the active overlay. Each REPLACES the shaded
 * colour with a visualisation of one piece of intermediate state from
 * §5, teaching one specific mental model:
 *
 *   OFF        normal shade-mode output (T9).
 *   HIT-TYPE   colour the surface by which sub-test fired:
 *                cyan    = body  (cylinder test, T3)
 *                magenta = cap A (sphere at lower endpoint, T6)
 *                yellow  = cap B (sphere at upper endpoint, T6)
 *              You SEE the boundary between cylinder hits and cap
 *              hits. Teaches the dispatcher in §5.3.
 *   AXIAL      colour by y/baba ∈ [0,1] using a cold→hot gradient.
 *              Body shows a smooth bottom-to-top sweep; caps render
 *              flat at their endpoint (0 for A, 1 for B). Teaches
 *              the bound check in §5.1.
 *   DISCRIM    map sqrt(h) (cylinder discriminant) to brightness.
 *              Bright on-axis where the ray plunges deep; black at
 *              the silhouette where the ray grazes (h → 0). Teaches
 *              what a "small h" geometrically means.
 *
 * Each overlay reuses the per-pixel CapsuleHit fields populated by
 * §5, so toggling them costs only a final tint.
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

/* ── §1 config ──────────────────────────────────────────────────────── */

/* §1.1 frame rate. */
#define TARGET_FPS    60
#define DT_CAP_NS     100000000LL          /* 0.1 s — spiral-of-death cap   */

/* §1.2 view geometry — terminal cell aspect (W/H) and full vertical FOV. */
#define ASPECT        0.47f
#define FOV_DEG       55.0f

/* §1.3 capsule (object space, axis along Y, centred on origin). */
#define CAP_HALF_H    0.65f                /* segment endpoints at ±this    */
#define CAP_R         0.35f                /* tube and cap radius           */

/* §1.4 rotation rates (rad/sec). */
#define ROT_Y         0.45f                /* primary spin around Y         */
#define ROT_X         0.22f                /* slow tilt around X            */

/* §1.5 camera distance (orbits along −Z; bigger = capsule looks smaller). */
#define CAM_DIST_DEF  3.4f
#define CAM_DIST_MIN  1.8f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f

/* §1.6 shading constants. */
#define AMBIENT       0.20f
#define SHININESS     75.0f                /* phong exponent — high = metal */

/* §1.7 character ramp — Paul Bourke 92-char density ladder.
 * Index 0 (space) is invisible; index N-1 ('@') is densest. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN  ((int)(sizeof k_ramp - 1))

/* §1.8 ncurses pair IDs (216 cube + reserved HUD/HINT). */
#define PAIR_CUBE_BASE   1                 /* + 0..215 = 6×6×6 cube         */
#define PAIR_HUD       217
#define PAIR_HINT      218

/* §1.9 numerical epsilon for ray distances. */
#define T_EPS         1e-4f                /* reject t < this (self-hit)    */

/* ── §2 clock ───────────────────────────────────────────────────────── */

/*
 * clock_ns — wall-clock time in nanoseconds, monotonic.
 *
 * Why CLOCK_MONOTONIC: we care about ELAPSED real time (for animation
 * dt), not wall date. CLOCK_MONOTONIC never goes backward across NTP
 * adjustments, DST shifts, or system clock changes.
 */
static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/*
 * clock_sleep_ns — best-effort sleep for the requested nanoseconds.
 *
 * Used by the main loop to cap the frame rate without burning the CPU
 * at 100%; we subtract (clock_ns() - frame_start) from the target frame
 * time and sleep the remainder.
 */
static void clock_sleep_ns(long long nanoseconds)
{
    if (nanoseconds <= 0) return;
    struct timespec request = {
        nanoseconds / 1000000000LL,
        nanoseconds % 1000000000LL
    };
    nanosleep(&request, NULL);
}

/* ── §3 math (V3 + Mat3) ────────────────────────────────────────────── *
 *
 * V3 — three floats by value. All vector helpers are inline to avoid
 * call overhead in the per-pixel loop. The ray-tracer operates entirely
 * on V3s; no V4/homogeneous coords appear because we have no
 * perspective-correct interpolation step (no triangles).
 *
 * Mat3 — a 3×3 rotation matrix stored as three V3 ROWS. Storing rows
 * (rather than columns) makes "M · v" a clean trio of v3dot() calls.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3add   (V3 a, V3 b)    { return (V3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline V3    v3sub   (V3 a, V3 b)    { return (V3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline V3    v3scale (float s, V3 a) { return (V3){s*a.x, s*a.y, s*a.z};       }
static inline V3    v3mul   (V3 a, V3 b)    { return (V3){a.x*b.x, a.y*b.y, a.z*b.z}; }
static inline float v3dot   (V3 a, V3 b)    { return a.x*b.x + a.y*b.y + a.z*b.z;     }
static inline float v3len   (V3 a)          { return sqrtf(v3dot(a, a));              }

static inline V3 v3norm(V3 a)
{
    float length = v3len(a);
    return (length > 1e-9f) ? v3scale(1.f/length, a) : (V3){0.f, 1.f, 0.f};
}

/*
 * v3reflect — reflect outgoing vector v across surface normal n.
 *
 * The reflection identity for an outgoing vector v and a unit normal n:
 *     v_reflected = v − 2·(v·n)·n
 *
 * Geometric reading: subtract twice the COMPONENT of v along n. This
 * is symmetric — incoming = outgoing of the same operator.
 *
 * In §6 phong shading we feed v3reflect the NEGATED light vector (−L)
 * so the function's "outgoing" semantics match: light-as-outgoing
 * reflects to viewing-as-outgoing.
 */
static inline V3 v3reflect(V3 v, V3 n)
{
    return v3sub(v, v3scale(2.f * v3dot(v, n), n));
}

static inline V3 v3clamp01(V3 v)
{
    return (V3){
        v.x < 0.f ? 0.f : v.x > 1.f ? 1.f : v.x,
        v.y < 0.f ? 0.f : v.y > 1.f ? 1.f : v.y,
        v.z < 0.f ? 0.f : v.z > 1.f ? 1.f : v.z
    };
}

typedef struct { V3 row[3]; } Mat3;

/*
 * mat3_rotation — build R = Rx(angle_x) · Ry(angle_y).
 *
 * Purpose: build the OBJECT→WORLD rotation matrix.
 * Inputs : angle_x, angle_y in radians.
 * Output : Mat3 with three V3 rows.
 *
 * Composition order: Y first, then X. Rotating Y first (the capsule's
 * own axis of rotational symmetry) doesn't change the silhouette but
 * does change WHICH part of the surface faces the lights — so it
 * still matters. The X tilt then changes the silhouette over time.
 *
 * Why this matrix flavour: T7 explains we rotate the RAY by the
 * INVERSE of this matrix. Since rotations are orthogonal, the inverse
 * is the transpose — see mat3_mulT below. We never compute the
 * inverse explicitly.
 */
static Mat3 mat3_rotation(float angle_x, float angle_y)
{
    float cos_x = cosf(angle_x), sin_x = sinf(angle_x);
    float cos_y = cosf(angle_y), sin_y = sinf(angle_y);
    Mat3 m;
    m.row[0] = (V3){  cos_y,            0.f,    sin_y          };
    m.row[1] = (V3){  sin_x * sin_y,    cos_x, -sin_x * cos_y  };
    m.row[2] = (V3){ -cos_x * sin_y,    sin_x,  cos_x * cos_y  };
    return m;
}

/*
 * mat3_mul — forward transform v_world = M · v_obj.
 *
 * Each output component is a dot of one matrix row with the input.
 * Stored row-by-row, so this is three v3dot() calls — efficient and
 * trivial to read.
 *
 * Used in §8 to bring the OBJECT-space surface normal back to WORLD
 * space for shading.
 */
static V3 mat3_mul(Mat3 m, V3 v)
{
    return (V3){
        v3dot(m.row[0], v),
        v3dot(m.row[1], v),
        v3dot(m.row[2], v)
    };
}

/*
 * mat3_mulT — inverse transform v_obj = Mᵀ · v_world.
 *
 * For an ORTHOGONAL matrix (any pure rotation) Mᵀ = M⁻¹. We get the
 * inverse for free by reading the matrix in the transposed access
 * pattern below — no determinant, no division. Same operation count
 * as mat3_mul; just a different memory pattern.
 *
 * Used in §8 to push the WORLD-space ray into OBJECT space where the
 * capsule's axis is fixed at Y.
 */
static V3 mat3_mulT(Mat3 m, V3 v)
{
    return (V3){
        m.row[0].x * v.x + m.row[1].x * v.y + m.row[2].x * v.z,
        m.row[0].y * v.x + m.row[1].y * v.y + m.row[2].y * v.z,
        m.row[0].z * v.x + m.row[1].z * v.y + m.row[2].z * v.z
    };
}

/* ── §4 colour & themes ─────────────────────────────────────────────── *
 *
 * Two channels make every cell expressive on a monochrome glyph grid:
 *
 *   COLOUR    256-colour 6×6×6 cube → "what" the surface is (gold rim
 *             vs cool fill).
 *   DENSITY   ASCII-ramp character → "how bright" the surface is (dim
 *             '.' vs solid '@').
 *
 * Together they recover ~9 bits of per-cell information from a 1-byte
 * glyph plus a colour pair. Themes vary the obj/spec/light tints; the
 * geometry pipeline doesn't care which theme is active.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── Theme — PBR-flavoured material descriptor ────────────────────── *
 *
 * First-principles redesign: the LIGHTS are pure WHITE, and each
 * material's distinctive look comes entirely from its own properties.
 * Under white light, gold looks like gold (warm yellow metal) and
 * blue plastic looks like blue plastic — the colour is intrinsic to
 * the material, not painted on by a tinted light source.
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
 *                     metals      ≈ 0.15 (real metals diffuse very
 *                                         little — almost everything
 *                                         reflects via specular)
 *                     gems        ≈ 0.70 (saturated body + bright spec)
 *                     plastic /
 *                       ceramic   ≈ 0.85 (full body colour)
 *                     glass       ≈ 0.10 (very dark body, bright spec
 *                                         fakes the transparent look)
 *                     neon        ≈ 0.20 (low diffuse + bright emissive)
 *
 * The 20 themes below are organised into 4 families. Switching themes
 * with `t / T` cycles through all 20 in order.
 *
 * Metal albedos are taken from PBR reference tables (Naty Hoffman,
 * "Physics and Math of Shading", SIGGRAPH 2013) and lightly tweaked
 * for terminal-renderer contrast.
 * ─────────────────────────────────────────────────────────────────── */
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
     * F0 (Fresnel at normal incidence) is what tints the highlight,
     * giving each metal its signature colour. */

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
    /* sapphire — blue corundum (Fe/Ti-doped)                           */
    {{0.10f,0.30f,0.88f}, {0.95f,0.95f,1.00f}, {0.f,0.f,0.f}, 0.70f, "sapphire"},
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
     * Neon plasma is a self-emissive material. The albedo is the dim
     * "off" tube colour; the emissive value glows hot pink even in
     * shadow because emissive is added AFTER lighting. */

    /* neon     — hot pink/magenta self-glow                            */
    {{0.05f,0.02f,0.10f}, {0.80f,0.80f,1.00f}, {1.00f,0.20f,0.85f}, 0.20f, "neon"},
};
#define THEME_N ((int)(sizeof g_themes / sizeof g_themes[0]))

static int g_have_256;     /* 1 if 256-colour cube is available, 0 = mono */

/*
 * color_init — bind ncurses colour pairs.
 *
 * Pair scheme:
 *   1..216    6×6×6 RGB cube (xterm 16..231) — used by draw_color().
 *             Index = red·36 + green·6 + blue + 1, all in {0..5}.
 *   217       PAIR_HUD  — bright yellow on default bg (HUD spec).
 *   218       PAIR_HINT — bright cyan on default bg (HUD spec).
 *
 * All-or-nothing: if the terminal lacks 256 colours we fall back to
 * monochrome (just the density ramp). Themes still cycle for display
 * but produce identical grayscale output.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    g_have_256 = (COLORS >= 256);
    if (g_have_256) {
        for (int i = 0; i < 216; i++)
            init_pair(PAIR_CUBE_BASE + i, 16 + i, -1);
    }
    init_pair(PAIR_HUD,  226, -1);
    init_pair(PAIR_HINT,  51, -1);
}

/*
 * draw_color — paint one cell with a colour and a luminance.
 *
 * Inputs : row, col       terminal cell
 *          colour         0..1 RGB triplet
 *          luminance      0..1 brightness picking the ramp character
 *
 * Pseudocode:
 *   ch       = ramp[ luminance · (RAMP_LEN−1) ]
 *   pair_idx = index_into_6x6x6_cube( colour )
 *   mvaddch(row, col, ch | COLOR_PAIR(pair_idx))
 *
 * Why two channels: see §4 header.
 */
static void draw_color(int row, int col, V3 colour, float luminance)
{
    if (luminance < 0.f) luminance = 0.f;
    if (luminance > 1.f) luminance = 1.f;
    char ch = k_ramp[(int)(luminance * (RAMP_LEN - 1))];

    if (g_have_256) {
        int red_5   = (int)(colour.x * 5.f + 0.5f); if (red_5   > 5) red_5   = 5;
        int green_5 = (int)(colour.y * 5.f + 0.5f); if (green_5 > 5) green_5 = 5;
        int blue_5  = (int)(colour.z * 5.f + 0.5f); if (blue_5  > 5) blue_5  = 5;
        int pair_idx = PAIR_CUBE_BASE + red_5 * 36 + green_5 * 6 + blue_5;
        attron(COLOR_PAIR(pair_idx));
        mvaddch(row, col, (chtype)(unsigned char)ch);
        attroff(COLOR_PAIR(pair_idx));
    } else {
        mvaddch(row, col, (chtype)(unsigned char)ch);
    }
}

/* ── §5 ray-capsule intersection (THE CORE) ─────────────────────────── *
 *
 * The most important section in this file. Three short functions
 * implementing the recipe from MENTAL MODEL → ALGORITHM IN STEPS:
 *
 *   §5.1 cylinder_test  — ray vs INFINITE cylinder + axial bound check
 *   §5.2 cap_test       — ray vs SPHERE at one endpoint
 *   §5.3 ray_capsule    — dispatcher (cylinder first, then nearer cap)
 *
 * All three operate in OBJECT space. The capsule's axis is along Y; A
 * is the lower endpoint, B is the upper.
 *
 * On hit, every test populates a CapsuleHit struct so the renderer can:
 *   - shade the hit (needs `hit_distance` and `normal_obj`),
 *   - run debug overlays (needs `axial_norm`, `discrim`, `part`).
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef enum {
    HIT_NONE  = 0,
    HIT_BODY  = 1,             /* cylinder body                              */
    HIT_CAP_A = 2,             /* sphere at endpoint A (lower)               */
    HIT_CAP_B = 3              /* sphere at endpoint B (upper)               */
} HitPart;

typedef struct {
    HitPart part;
    float   hit_distance;      /* ray parameter t                            */
    V3      normal_obj;        /* object-space outward normal                */
    float   axial_norm;        /* y/baba in [0,1] (interpolation along axis) */
    float   discrim;           /* sqrt(max(h, 0)) of the cylinder quadratic  */
} CapsuleHit;

/*
 * §5.1 cylinder_test — ray vs the INFINITE cylinder along AB,
 *                       with axial bound check inside (0, baba).
 *
 * Purpose:
 *   1. Solve the ray-vs-circle quadratic projected onto the perp-plane
 *      (T3, T4).
 *   2. If a real hit exists, recover its axial coordinate y (T5).
 *   3. If 0 < y < baba, accept it as a BODY hit and compute the radial
 *      outward normal.
 *
 * Inputs:
 *   ray_dir         OBJECT-space ray direction (unit)
 *   axis_seg        ba = B − A
 *   origin_to_A     oa = ray_origin − A
 *   radius          r
 *   axis_len_sq     baba = |ba|² (passed in to skip recomputation)
 *
 * Outputs (only valid on body hit):
 *   *out_distance   ray parameter t > 0
 *   *out_normal_obj radial outward normal in OBJECT space
 *   *out_axial_norm y/baba ∈ (0,1)
 *   *out_discrim    sqrt(max(h, 0)) — set even on miss for the debug
 *                   overlay (lets us SEE where the cylinder is "almost"
 *                   hit even when no body hit was registered).
 *
 * Returns: HIT_BODY on success, HIT_NONE on either:
 *   - cylinder miss (h < 0), or
 *   - cylinder hit outside (0, baba) (caller must try a cap).
 *
 * Pseudocode:
 *   axis_dot_dir       = axis_seg · ray_dir
 *   axis_dot_origin    = axis_seg · origin_to_A
 *   dir_dot_origin     = ray_dir  · origin_to_A
 *   origin_dot_origin  = origin_to_A · origin_to_A
 *
 *   quad_a = axis_len_sq − axis_dot_dir²
 *   quad_b = axis_len_sq · dir_dot_origin
 *          − axis_dot_origin · axis_dot_dir
 *   quad_c = axis_len_sq · (origin_dot_origin − r²)
 *          − axis_dot_origin²
 *   discriminant = quad_b² − quad_a · quad_c
 *
 *   if discriminant < 0:           return MISS  (sets *out_discrim = 0)
 *
 *   t            = (−quad_b − √discriminant) / quad_a
 *   axial_coord  = axis_dot_origin + t · axis_dot_dir
 *
 *   if NOT (T_EPS < t  AND  0 < axial_coord < axis_len_sq):
 *     return MISS but still output axial_coord for the dispatcher
 *
 *   point_minus_A = origin_to_A + t · ray_dir
 *   axial_part    = (axial_coord / axis_len_sq) · axis_seg
 *   normal_obj    = normalize( point_minus_A − axial_part )
 *
 *   return HIT_BODY
 */
static HitPart cylinder_test(V3 ray_dir,
                             V3 axis_seg, V3 origin_to_A,
                             float radius, float axis_len_sq,
                             float *out_distance,
                             V3    *out_normal_obj,
                             float *out_axial_norm,
                             float *out_discrim)
{
    float axis_dot_dir       = v3dot(axis_seg,   ray_dir);
    float axis_dot_origin    = v3dot(axis_seg,   origin_to_A);
    float dir_dot_origin     = v3dot(ray_dir,    origin_to_A);
    float origin_dot_origin  = v3dot(origin_to_A, origin_to_A);

    /* Quílez form (see T4): scaled by axis_len_sq so divisions cancel. */
    float quad_a = axis_len_sq - axis_dot_dir * axis_dot_dir;
    float quad_b = axis_len_sq * dir_dot_origin
                 - axis_dot_origin * axis_dot_dir;
    float quad_c = axis_len_sq * (origin_dot_origin - radius * radius)
                 - axis_dot_origin * axis_dot_origin;

    float discriminant = quad_b * quad_b - quad_a * quad_c;

    if (discriminant < 0.f) {
        *out_discrim = 0.f;
        return HIT_NONE;
    }
    *out_discrim = sqrtf(discriminant);

    float hit_distance = (-quad_b - *out_discrim) / quad_a;
    float axial_coord  = axis_dot_origin + hit_distance * axis_dot_dir;

    /* Tell the dispatcher where on the axis we hit even when the body
     * test fails — it uses this to pick which cap to try next (T6). */
    *out_axial_norm = axial_coord / axis_len_sq;

    if (hit_distance > T_EPS &&
        axial_coord  > 0.f   &&
        axial_coord  < axis_len_sq)
    {
        /* Radial outward normal: take (P − A) and SUBTRACT its axial
         * component. (axial_coord / axis_len_sq) · axis_seg is the
         * axial component, and the remainder is perpendicular to the
         * axis — that's the radial direction we want. */
        V3 point_minus_A = v3add(origin_to_A,
                                 v3scale(hit_distance, ray_dir));
        V3 axial_part    = v3scale(axial_coord / axis_len_sq, axis_seg);
        V3 radial        = v3sub(point_minus_A, axial_part);

        *out_distance   = hit_distance;
        *out_normal_obj = v3norm(radial);
        return HIT_BODY;
    }
    return HIT_NONE;
}

/*
 * §5.2 cap_test — ray vs a SPHERE centred at the supplied cap centre.
 *
 * Purpose: handle the rounded-end case after cylinder_test fell
 *          through (T2). The dispatcher passes us the offset vector
 *          origin_to_cap = ray_origin − cap_centre, so we don't need
 *          to know which cap (A or B) it is.
 *
 * Pseudocode:
 *   quad_b = ray_dir · origin_to_cap
 *   quad_c = origin_to_cap · origin_to_cap − r²
 *   discriminant = quad_b² − quad_c
 *   if discriminant < 0: return MISS
 *
 *   root = sqrt(discriminant)
 *   t = −quad_b − root            (front face)
 *   if t < T_EPS:                 (camera inside? try back face)
 *     t = −quad_b + root
 *     if t < T_EPS: return MISS
 *
 *   normal_obj = normalize( origin_to_cap + t · ray_dir )
 *   return hit
 *
 * We don't enforce a hemisphere bound (e.g. y ≥ 0 for cap A) — if the
 * dispatcher routed us to cap A it's because the cylinder hit was on
 * cap A's side of the segment, so the relevant half of the sphere is
 * automatically the one this ray touches.
 *
 * Returns the HitPart label the caller wants stamped on the result —
 * cap_test is shape-agnostic, the dispatcher tells us what to label.
 */
static HitPart cap_test(V3 ray_dir, V3 origin_to_cap, float radius,
                        HitPart label_on_hit,
                        float *out_distance, V3 *out_normal_obj)
{
    float quad_b = v3dot(ray_dir, origin_to_cap);
    float quad_c = v3dot(origin_to_cap, origin_to_cap) - radius * radius;
    float discriminant = quad_b * quad_b - quad_c;

    if (discriminant < 0.f) return HIT_NONE;

    float root = sqrtf(discriminant);
    float hit_distance = -quad_b - root;
    if (hit_distance < T_EPS) {
        hit_distance = -quad_b + root;
        if (hit_distance < T_EPS) return HIT_NONE;
    }
    V3 point_from_centre = v3add(origin_to_cap,
                                 v3scale(hit_distance, ray_dir));
    *out_distance   = hit_distance;
    *out_normal_obj = v3norm(point_from_centre);
    return label_on_hit;
}

/*
 * §5.3 ray_capsule — top-level dispatcher.
 *
 * Purpose: try the cylinder first (cheap, covers most pixels of a
 *          side-on capsule). If the cylinder produced no in-bounds
 *          hit, pick the nearer cap by the sign of the axial coord
 *          (T6) and test it as a sphere.
 *
 * Inputs:  ray_origin, ray_dir   OBJECT-space ray
 *          A, B                  capsule endpoints (object space)
 *          radius                tube/cap radius
 *
 * Output:  CapsuleHit struct (defined at the top of §5).
 *          On miss, .part = HIT_NONE; other fields meaningless.
 *
 * Pseudocode:
 *   axis_seg     = B − A
 *   origin_to_A  = ray_origin − A
 *   axis_len_sq  = axis_seg · axis_seg
 *
 *   try cylinder_test:
 *     if HIT_BODY:        return body hit
 *     else:               axial coord written as side-effect (used below)
 *
 *   if axial_norm >= 1:   test sphere at B  (origin_to_cap = oa − ba)
 *                         label = HIT_CAP_B
 *   else:                 test sphere at A  (origin_to_cap = oa)
 *                         label = HIT_CAP_A
 *
 *   on cap hit, set axial_norm to 0 (cap A) or 1 (cap B).
 */
static CapsuleHit ray_capsule(V3 ray_origin, V3 ray_dir,
                              V3 A, V3 B, float radius)
{
    V3    axis_seg    = v3sub(B, A);
    V3    origin_to_A = v3sub(ray_origin, A);
    float axis_len_sq = v3dot(axis_seg, axis_seg);

    CapsuleHit hit = { .part = HIT_NONE };
    /* Default 0 routes a cylinder MISS to cap A — cap_test will return
     * HIT_NONE anyway in that case (see EDGE CASES in MENTAL MODEL),
     * so the algorithm stays correct. */
    hit.axial_norm = 0.f;
    hit.discrim    = 0.f;

    HitPart body_result = cylinder_test(ray_dir, axis_seg, origin_to_A,
                                        radius, axis_len_sq,
                                        &hit.hit_distance,
                                        &hit.normal_obj,
                                        &hit.axial_norm,
                                        &hit.discrim);
    if (body_result == HIT_BODY) {
        hit.part = HIT_BODY;
        return hit;
    }

    /* Cylinder either missed entirely (h < 0) or its hit was axially
     * out of bounds. Pick the nearer cap by the sign of axial_norm. */
    int     pick_B        = (hit.axial_norm >= 1.f);
    V3      cap_centre    = pick_B ? B : A;
    HitPart cap_label     = pick_B ? HIT_CAP_B : HIT_CAP_A;
    V3      origin_to_cap = v3sub(ray_origin, cap_centre);

    HitPart cap_result = cap_test(ray_dir, origin_to_cap, radius, cap_label,
                                  &hit.hit_distance, &hit.normal_obj);
    if (cap_result != HIT_NONE) {
        hit.part       = cap_result;
        hit.axial_norm = pick_B ? 1.f : 0.f;
    }
    return hit;
}

/* ── §6 shading ─────────────────────────────────────────────────────── *
 *
 * Once §5 has identified a hit point and its surface normal, §6 turns
 * those into an RGB colour. Four orthogonal modes (T9) are provided.
 * The renderer (§8) feeds whichever the user has selected with `s`.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef enum {
    MODE_PHONG = 0,
    MODE_NORMAL,
    MODE_FRESNEL,
    MODE_DEPTH,
    MODE_N
} ShadeMode;
static const char *const k_mode_names[] = {
    "phong", "normals", "fresnel", "depth"
};

/* Three fixed world-space lights — POSITIONS, not directions.
 * Shading computes (light_position − hit_point), normalises, and uses
 * that as the per-pixel light direction (so closer surfaces get a
 * slight wraparound effect). Numbers chosen for visual contrast — a
 * cinematic three-point setup, not physical accuracy:
 *   KEY  = warm above-front-right     (the bright "sun")
 *   FILL = cool low-front-left        (lifts the shadow side)
 *   RIM  = bright back-low            (separates silhouette from bg)
 */
static const V3 LIGHT_KEY  = { 3.0f,  4.0f, -2.0f };
static const V3 LIGHT_FILL = {-4.0f,  1.0f, -1.0f };
static const V3 LIGHT_RIM  = { 0.5f, -1.0f,  5.0f };

/*
 * shade_phong — three-point Phong with PURE WHITE lights.
 *
 * For each white light at position L_pos:
 *   light_dir   = normalize(L_pos − point_world)
 *   diffuse     = max(0, normal · light_dir)
 *   reflect_dir = reflect(−light_dir, normal)
 *   specular    = max(0, reflect_dir · view_dir)^SHININESS
 *
 * The lights are pure white — every material's distinctive look comes
 * from its OWN albedo and specular tint, not from a coloured key
 * filter. Under white light:
 *   gold ALBEDO + gold SPEC      → looks like gold
 *   blue ALBEDO + WHITE SPEC     → looks like blue plastic
 *   dark ALBEDO + WHITE SPEC     → looks like glass
 * Material-as-material, not material-tinted-by-light.
 *
 * Per-light weighting: KEY > FILL > RIM in diffuse; KEY and RIM
 * contribute specular; FILL is diffuse-only.
 *
 * Diffuse contribution is multiplied by th->diffuse_weight:
 *   metals       ≈ 0.15 — real metals reflect almost everything
 *                         specularly; near-zero diffuse.
 *   gems         ≈ 0.70 — saturated body + bright white spec.
 *   dielectrics  ≈ 0.85 — full body colour.
 *   glass        ≈ 0.10 — very dark body, bright spec fakes
 *                         transparency.
 *
 * Emissive is added AFTER the lighting sum and BEFORE the clamp, so
 * neon glows hot pink even in shadow.
 *
 * Why pre-flip the light vector when calling v3reflect:
 *   v3reflect(v, n) reflects an OUTGOING vector. Light arrives along
 *   −light_dir, so we pass −light_dir to v3reflect; the result is
 *   the reflection direction we compare against view_dir.
 */
static V3 shade_phong(V3 point_world, V3 normal_world, V3 view_dir,
                      const Theme *th)
{
    /* Ambient: dim version of body albedo. */
    V3 colour = v3scale(AMBIENT, th->albedo);

    /* §6.1 KEY light — primary diffuse + sharp specular (white). */
    {
        V3    light_dir   = v3norm(v3sub(LIGHT_KEY, point_world));
        float diffuse     = fmaxf(0.f, v3dot(normal_world, light_dir));
        V3    reflect_dir = v3reflect(v3scale(-1.f, light_dir), normal_world);
        float specular    = powf(fmaxf(0.f, v3dot(reflect_dir, view_dir)),
                                 SHININESS);
        /* Diffuse: white light × albedo × diffuse_weight. */
        colour = v3add(colour,
                       v3scale(diffuse * th->diffuse_weight * 1.00f,
                               th->albedo));
        /* Specular: white light × spec colour. METAL: tinted spec
         * tints the highlight (gold). DIELECTRIC: white spec → white peak. */
        colour = v3add(colour,
                       v3scale(specular * 1.30f, th->specular));
    }
    /* §6.2 FILL light — soft diffuse, no specular. Lifts shadow side. */
    {
        V3    light_dir = v3norm(v3sub(LIGHT_FILL, point_world));
        float diffuse   = fmaxf(0.f, v3dot(normal_world, light_dir));
        colour = v3add(colour,
                       v3scale(diffuse * th->diffuse_weight * 0.55f,
                               th->albedo));
    }
    /* §6.3 RIM light — wide specular kissing the back silhouette. */
    {
        V3    light_dir   = v3norm(v3sub(LIGHT_RIM, point_world));
        float diffuse     = fmaxf(0.f, v3dot(normal_world, light_dir));
        V3    reflect_dir = v3reflect(v3scale(-1.f, light_dir), normal_world);
        float specular    = powf(fmaxf(0.f, v3dot(reflect_dir, view_dir)),
                                 10.f);
        colour = v3add(colour,
                       v3scale(diffuse * th->diffuse_weight * 0.40f,
                               th->albedo));
        colour = v3add(colour,
                       v3scale(specular * 1.20f, th->specular));
    }
    /* §6.4 Emissive — added BEFORE clamp, AFTER lighting. Lets neon
     * glow regardless of light position. (0,0,0) for everything else. */
    colour = v3add(colour, th->emissive);

    return v3clamp01(colour);
}

/*
 * shade_normal — RGB-encode the surface normal as a colour.
 *
 *   N ∈ [−1,1]³   →   (N+1)/2 ∈ [0,1]³
 *
 * Diagnostic mode. A correct intersection should show:
 *   - body: smooth radial gradient as N rotates around the axis;
 *   - caps: hemispherical gradient (red dominates near +X, etc).
 * Aliasing or wrong normals show up as harsh colour discontinuities.
 */
static V3 shade_normal(V3 normal_world)
{
    return (V3){
        normal_world.x * 0.5f + 0.5f,
        normal_world.y * 0.5f + 0.5f,
        normal_world.z * 0.5f + 0.5f
    };
}

/*
 * shade_fresnel — Schlick approximation, F₀ = 0 (clear dielectric).
 *
 *   cos_angle      = |normal · view_dir|
 *   fresnel_factor = (1 − cos_angle)⁵
 *
 * F₀ = 0 simplifies the full Schlick expression to just (1−cosθ)⁵, so
 * the surface appears DARK facing the camera (cos_angle ≈ 1 → factor ≈
 * 0) and bright at the silhouette (cos_angle ≈ 0 → factor ≈ 1). The
 * classic "glass pill" look: dark middle, glowing rim.
 *
 * Useful for inspecting the silhouette transition between body and
 * caps — they SHOULD merge smoothly without a visible crease.
 */
static V3 shade_fresnel(V3 normal_world, V3 view_dir, const Theme *th)
{
    float cos_angle      = fabsf(v3dot(normal_world, view_dir));
    float one_minus_cos  = 1.f - cos_angle;
    float fresnel_factor = one_minus_cos * one_minus_cos * one_minus_cos
                         * one_minus_cos * one_minus_cos;
    V3 core = v3scale(0.06f, th->albedo);
    V3 edge = v3clamp01(v3scale(1.20f, th->specular));
    return v3clamp01(v3add(v3scale(1.f - fresnel_factor, core),
                           v3scale(fresnel_factor,        edge)));
}

/*
 * shade_depth — encode hit distance as brightness (closer = brighter).
 *
 *   depth_norm = 1 − min(t / t_max, 1)         linear depth in [0,1]
 *   bright     = depth_norm²                   steeper falloff
 *
 * Sanity check: in DEPTH mode, the brightest pixel should sit at the
 * nearest point on the silhouette (centre of body for a side-on view).
 */
static V3 shade_depth(float hit_distance, float distance_max, const Theme *th)
{
    float depth_norm = 1.f - fminf(hit_distance / distance_max, 1.f);
    depth_norm = depth_norm * depth_norm;
    return v3clamp01(v3scale(depth_norm, th->albedo));
}

/*
 * rec601_luma — perceptual brightness from RGB (Rec. 601 weighting).
 *
 *   Y = 0.299·R + 0.587·G + 0.114·B
 *
 * Used to choose the ASCII ramp character for a coloured pixel. Green
 * dominates because the human eye is most sensitive to green.
 */
static inline float rec601_luma(V3 c)
{
    return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/* ── §7 debug overlays ──────────────────────────────────────────────── *
 *
 * Three overlays that REPLACE the normal shaded colour with a
 * visualisation of the §5 intersection's intermediate state. Cycled
 * with `d`. Each teaches one specific mental model — see T10.
 *
 * The overlays read the CapsuleHit fields populated by §5; no extra
 * computation is needed beyond a final tint.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef enum {
    DEBUG_OFF = 0,
    DEBUG_HIT_TYPE,            /* colour by which sub-test fired             */
    DEBUG_AXIAL,               /* colour by axial coord y/baba ∈ [0,1]       */
    DEBUG_DISCRIM,             /* colour by cylinder discriminant magnitude  */
    DEBUG_N
} DebugMode;
static const char *const k_debug_names[] = {
    "off", "hit-type", "axial", "discrim"
};

/*
 * gradient_cold_hot — simple two-stop colour gradient driven by t∈[0,1].
 *   t = 0  → deep blue
 *   t = 1  → bright gold
 * Used by the AXIAL and DISCRIM overlays to map a scalar to a colour.
 */
static V3 gradient_cold_hot(float t)
{
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    V3 cold = (V3){ 0.10f, 0.20f, 0.95f };
    V3 hot  = (V3){ 1.00f, 0.85f, 0.20f };
    return (V3){
        cold.x + (hot.x - cold.x) * t,
        cold.y + (hot.y - cold.y) * t,
        cold.z + (hot.z - cold.z) * t
    };
}

/*
 * apply_debug — given a hit and an overlay mode, return the overlay
 *               colour and luminance. If mode == DEBUG_OFF the caller
 *               keeps its original shading.
 *
 * HIT_TYPE      cyan = body, magenta = cap A, yellow = cap B.
 *               Teaches the dispatcher in §5.3.
 * AXIAL         gradient by axial_norm. Body shows a smooth cold→hot
 *               sweep; caps render flat at their endpoint. Teaches
 *               the bound-check logic in §5.1.
 * DISCRIM       brightness from sqrt(h) of the cylinder quadratic.
 *               Teaches what a "small h" geometrically means
 *               (grazing rays at the silhouette).
 *
 * Outputs are written to *out_colour and *out_luminance. The caller
 * passes them to draw_color() unchanged.
 */
static void apply_debug(DebugMode mode, const CapsuleHit *hit,
                        V3 *out_colour, float *out_luminance)
{
    switch (mode) {
    case DEBUG_HIT_TYPE: {
        V3 c;
        switch (hit->part) {
        case HIT_BODY:  c = (V3){ 0.20f, 0.95f, 0.95f }; break;  /* cyan    */
        case HIT_CAP_A: c = (V3){ 0.90f, 0.20f, 0.95f }; break;  /* magenta */
        case HIT_CAP_B: c = (V3){ 1.00f, 0.95f, 0.20f }; break;  /* yellow  */
        default:        c = (V3){ 0.f,   0.f,   0.f   }; break;
        }
        *out_colour    = c;
        *out_luminance = 0.85f;
        break;
    }
    case DEBUG_AXIAL: {
        float t_norm = hit->axial_norm;
        if (t_norm < 0.f) t_norm = 0.f;
        if (t_norm > 1.f) t_norm = 1.f;
        *out_colour    = gradient_cold_hot(t_norm);
        *out_luminance = 0.4f + 0.55f * t_norm;
        break;
    }
    case DEBUG_DISCRIM: {
        /* The discriminant grows roughly linearly with how far INSIDE
         * the cylinder body the ray plunges (zero at silhouette, big
         * in middle). Normalise by a typical max so we get [0,1]. */
        float norm = hit->discrim / (CAP_R * 1.4f);
        if (norm < 0.f) norm = 0.f;
        if (norm > 1.f) norm = 1.f;
        *out_colour    = gradient_cold_hot(norm);
        *out_luminance = norm;
        break;
    }
    default:
        /* DEBUG_OFF — caller keeps its own colour/luminance. */
        break;
    }
}

/* ── §8 render frame ────────────────────────────────────────────────── *
 *
 * One ray per terminal cell. The pipeline:
 *
 *   1. Compute the camera basis (forward / right / up — fixed; the
 *      CAPSULE rotates, not the camera).
 *   2. Pre-build the WORLD↔OBJECT rotation matrix from the current
 *      angles. Used as Mᵀ on the ray (T7) and as M on the normal.
 *   3. For each cell:
 *        a. screen u, v in [-1,1] with FOV expansion + ASPECT correction
 *        b. world ray (origin, direction)
 *        c. object-space ray via mat3_mulT
 *        d. ray_capsule()
 *        e. on hit: world-space hit + normal, shade, optionally apply
 *           debug overlay, draw
 *
 * The bottom row is left blank so the cyan hint strip in §9 has
 * somewhere to live.
 *
 * ─────────────────────────────────────────────────────────────────── */

static void render(int cols, int rows,
                   float angle_x, float angle_y, float cam_dist,
                   int theme_idx, ShadeMode shade_mode,
                   DebugMode debug_mode)
{
    const Theme *th    = &g_themes[theme_idx % THEME_N];
    float fov_half_tan = tanf(FOV_DEG * (float)M_PI / 360.f);

    /* §8.1 — WORLD↔OBJECT rotation. (Mᵀ goes WORLD→OBJECT for the ray;
     *         M goes OBJECT→WORLD for the normal.) */
    Mat3 rotation = mat3_rotation(angle_x, angle_y);

    /* §8.2 — camera basis (fixed). */
    V3 camera_origin = { 0.f, 0.f, -cam_dist };
    V3 view_forward  = { 0.f, 0.f,  1.f };
    V3 view_right    = { 1.f, 0.f,  0.f };
    V3 view_up       = { 0.f, 1.f,  0.f };

    /* §8.3 — capsule endpoints in OBJECT space (NEVER rotated). */
    V3 endpoint_A = { 0.f, -CAP_HALF_H, 0.f };
    V3 endpoint_B = { 0.f, +CAP_HALF_H, 0.f };

    float screen_centre_x = cols * 0.5f;
    float screen_centre_y = rows * 0.5f;

    /* §8.4 — Mᵀ applied to the camera origin once (it doesn't depend
     *         on the pixel). The view direction varies per pixel and
     *         gets transformed inside the loop. */
    V3 ray_origin_obj = mat3_mulT(rotation, camera_origin);

    /* §8.5 — primary loop. */
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            /* Screen → camera. FOV scale applied to both axes; the
             * vertical axis is additionally divided by ASPECT (≈0.47)
             * because terminal cells are taller than wide — without
             * that division a circle would render as a tall ellipse. */
            float screen_u =  (col - screen_centre_x) / screen_centre_x
                              * fov_half_tan;
            float screen_v = -(row - screen_centre_y) / screen_centre_x
                              * fov_half_tan / ASPECT;

            V3 ray_dir_world = v3norm(v3add(view_forward,
                                            v3add(v3scale(screen_u, view_right),
                                                  v3scale(screen_v, view_up))));

            /* Push ray into capsule object space (T7). */
            V3 ray_dir_obj = mat3_mulT(rotation, ray_dir_world);

            CapsuleHit hit = ray_capsule(ray_origin_obj, ray_dir_obj,
                                         endpoint_A, endpoint_B, CAP_R);
            if (hit.part == HIT_NONE) continue;

            /* Hit point in WORLD space: t is the same in both spaces
             * (rotation preserves distance), so we can use the WORLD
             * ray directly. */
            V3 hit_point_world = v3add(camera_origin,
                                       v3scale(hit.hit_distance, ray_dir_world));

            /* Normal: object → world. */
            V3 normal_world = mat3_mul(rotation, hit.normal_obj);

            /* View direction at the hit point (for specular). */
            V3 view_dir = v3norm(v3sub(camera_origin, hit_point_world));

            V3 colour;
            switch (shade_mode) {
            default:
            case MODE_PHONG:
                colour = shade_phong(hit_point_world, normal_world, view_dir, th);
                break;
            case MODE_NORMAL:
                colour = shade_normal(normal_world);
                break;
            case MODE_FRESNEL:
                colour = shade_fresnel(normal_world, view_dir, th);
                break;
            case MODE_DEPTH:
                colour = shade_depth(hit.hit_distance, cam_dist * 2.2f, th);
                break;
            }

            /* MODE_NORMAL's colour comes from the normal vector
             * directly, so its luminance should weight green strongly
             * (matches the eye's intuition that "green is brightest"
             * in RGB-encoded normal visualisations). All other modes
             * use Rec. 601 perceptual luma. */
            float luminance = (shade_mode == MODE_NORMAL)
                ? (normal_world.x * 0.5f + 0.5f) * 0.30f
                + (normal_world.y * 0.5f + 0.5f) * 0.60f
                + (normal_world.z * 0.5f + 0.5f) * 0.10f
                : rec601_luma(colour);

            /* Debug overlay (re-tints the pixel if active). */
            if (debug_mode != DEBUG_OFF) {
                apply_debug(debug_mode, &hit, &colour, &luminance);
            }

            draw_color(row, col, colour, luminance);
        }
    }
}

/* ── §9 screen / HUD ────────────────────────────────────────────────── *
 *
 * Two-row HUD per project spec:
 *   row 0    yellow status (right-aligned) + yellow mode label (left)
 *   rows-1   cyan key-hint strip (BOLD; never DIM, must read against
 *            any animation behind it)
 *
 * ─────────────────────────────────────────────────────────────────── */

static void hud_draw(int cols, int rows, float fps,
                     int theme_idx, ShadeMode shade_mode,
                     DebugMode debug_mode,
                     float cam_dist, int paused)
{
    /* §9.1 right-aligned status. fps lives here so it never gets
     * clipped by a long mode label on narrow terminals. */
    char status[96];
    snprintf(status, sizeof status,
             " %5.1f fps  dist:%.1f  %-8s  %s ",
             (double)fps, (double)cam_dist,
             g_themes[theme_idx % THEME_N].name,
             paused ? "PAUSED " : "running");
    int status_len = (int)strlen(status);
    if (status_len > cols) status_len = cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - status_len, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* §9.2 left-aligned mode + debug labels (yellow without bold). */
    char left_label[80];
    snprintf(left_label, sizeof left_label,
             " mode:%-7s  debug:%-8s ",
             k_mode_names[shade_mode], k_debug_names[debug_mode]);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(0, 0, "%s", left_label);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* §9.3 bottom-left cyan key-hint strip (BOLD, ASCII only). */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc/p:pause  s:mode  d:debug  t:theme  +/-:zoom ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §10 app ────────────────────────────────────────────────────────── */

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
    typeahead(-1);                          /* prevent input tearing of output */
    atexit(cleanup);
    color_init();

    int cols, rows;
    getmaxyx(stdscr, rows, cols);

    int       theme_idx  = 0;
    ShadeMode shade_mode = MODE_PHONG;
    DebugMode debug_mode = DEBUG_OFF;
    float     cam_dist   = CAM_DIST_DEF;
    float     angle_x    = 0.f;
    float     angle_y    = 0.f;
    int       paused     = 0;

    float     fps        = 0.f;
    long long fps_acc    = 0;
    int       fps_cnt    = 0;
    long long frame_ns   = 1000000000LL / TARGET_FPS;
    long long last       = clock_ns();

    while (g_run) {
        /* §10.1 resize. */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
        }

        /* §10.2 wall-clock dt with spiral-of-death cap. */
        long long now = clock_ns();
        long long dt  = now - last;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;
        last = now;

        /* §10.3 advance rotation if not paused. */
        if (!paused) {
            angle_y += ROT_Y * (float)dt * 1e-9f;
            angle_x += ROT_X * (float)dt * 1e-9f;
        }

        /* §10.4 fps rolling average over half-second windows. */
        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps     = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        /* §10.5 paint frame. */
        long long frame_start = clock_ns();
        erase();
        render(cols, rows, angle_x, angle_y, cam_dist,
               theme_idx, shade_mode, debug_mode);
        hud_draw(cols, rows, fps, theme_idx, shade_mode, debug_mode,
                 cam_dist, paused);
        wnoutrefresh(stdscr);
        doupdate();

        /* §10.6 input. */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27 /* ESC */:
            g_run = 0; break;
        case ' ': case 'p': case 'P':
            paused = !paused; break;
        case 's': case 'S':
            shade_mode = (ShadeMode)((shade_mode + 1) % MODE_N); break;
        case 'd': case 'D':
            debug_mode = (DebugMode)((debug_mode + 1) % DEBUG_N); break;
        case 't':
            theme_idx = (theme_idx + 1) % THEME_N; break;
        case 'T':
            theme_idx = (theme_idx + THEME_N - 1) % THEME_N; break;
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

        /* §10.7 frame cap. */
        clock_sleep_ns(frame_ns - (clock_ns() - frame_start));
    }
    return 0;
}
