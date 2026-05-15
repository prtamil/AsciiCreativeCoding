/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * csg_atlas.c — encyclopedia of CSG boolean operators on SDFs
 *
 * DEMO: Two operand primitives are held constant — a sphere at the
 *       origin and a rounded box that swings in and out of it.  The
 *       USER picks a boolean operator from a 13-entry catalogue
 *       organised as a 2-D table:
 *
 *                Union  Intersect  Subtract  XOR
 *         HARD     ●         ●         ●       ●
 *         SMOOTH   ●         ●         ●
 *         ROUND    ●         ●         ●
 *         CHAMFER  ●         ●         ●
 *
 *       Press 1..4 to pick the FAMILY (row); n / N cycles within
 *       the family (column).  The 4 families are 4 DIFFERENT WAYS
 *       OF HANDLING THE SEAM where the two SDFs meet — sharp,
 *       smooth bulge, circular fillet, 45° bevel.  Each smooth /
 *       round / chamfer family takes a parameter (k or r);
 *       +/- adjust it live so you can dial the seam from "hard
 *       limit" to "very soft join".
 *
 *       Camera orbits, box swings, operator stays constant per
 *       frame.  Six colour themes, four debug overlays for
 *       inspecting raw geometry signals.
 *
 * Study alongside: raymarcher/sdf_gallery.c — uses the same boolean
 *       primitives in a SCENE-ORIENTED way (scene 2 is one CSG
 *       column).  This file is the inverse: HOLD the operands
 *       constant and SWEEP every operator.  Together they cover
 *       both axes of CSG learning — what you can build from a few
 *       operators, vs every operator the language gives you.
 *
 * Section map:
 *   §1   config         — every tunable named, no magic numbers later
 *   §2   clock          — monotonic timer + sleep
 *   §3   color          — themes + 2-pair HUD spec
 *   §4   vec3           — value-type 3-D math
 *   §5   sphere SDF
 *   §6   rounded-box SDF
 *   §7   HARD operators       — min / max / max(·,−·) / xor
 *   §8   SMOOTH operators     — Quílez polynomial smin family
 *   §9   ROUND operators      — length-of-positives form
 *   §10  CHAMFER operators    — 45° bevel form
 *   §11  op table + family helpers
 *   §12  scene SDF — animated operands + operator dispatch
 *   §13  sphere trace + step count
 *   §14  surface normal (4-tap tetrahedral)
 *   §15  Phong shade
 *   §16  cast_ray + Hit struct
 *   §17  canvas — production view + 3 debug overlays
 *   §18  scene state + tick + camera basis
 *   §19  screen — ncurses init + 2-row HUD + present
 *   §20  app — main loop, signals, key handling
 *
 * Keys (the encyclopedia is a 2-D table — rows = families, cols = op kind):
 *   1..4        FAMILY (row)   1=HARD  2=SMOOTH  3=ROUND  4=CHAMFER
 *   n / N       OP KIND (col)  cycles UNION/INTERSECT/SUBTRACT[/XOR]
 *                              within current family — never crosses
 *                              into a different family
 *   + / -       parameter up / down (k or r — smooth/round/chamfer)
 *   d / D       cycle debug overlay
 *                 (NORMAL / NORMALS / DEPTH / STEPS)
 *   space       pause / resume orbit + box swing
 *   t / T       next / previous theme
 *                 (CLASSIC / AMBER / MATRIX / NEON / ICE / COPPER)
 *   z / Z       zoom in / out
 *   r           reset
 *   q / ESC     quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/csg_atlas.c \
 *       -o csg_atlas -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is structured as a textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         names the algorithm and lists references.
 *   • MENTAL MODEL     intuition + ASCII diagrams of the seam shapes
 *                      that distinguish the four operator families.
 *   • GUIDED TUTORIAL  ten short answers walking from "why booleans on
 *                      SDFs?" through the four families to the 2-D
 *                      encyclopedia layout.
 *   • §1..§20          the actual code, each section short and focused.
 *
 * Ten-minute version: read the GUIDED TUTORIAL.  By the end the
 * §-sections feel like reviewing notes.
 *
 * Math notation used in code:
 *      a, b       — distance values (signed) from two operand SDFs
 *      k          — smooth-min bandwidth (Quílez polynomial)
 *      r          — round / chamfer fillet radius
 *      p          — query point in 3-D space
 *      h          — auxiliary "how close are a and b?" smoothing scalar
 *      N          — surface normal at the hit
 *      L, V, R    — light, view, reflection unit vectors
 *
 * Background you need:
 *   • basic vector arithmetic (add, dot, length, normalise)
 *   • read raymarcher.c first if sphere tracing is unfamiliar
 *   • familiarity with `min` and `max` on real numbers — that's all
 *     the math the HARD family needs
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : SPHERE TRACING (Hart 1996) of a SCENE SDF that is
 *                ONE OF 13 BOOLEAN OPERATORS applied to two animated
 *                primitive SDFs.  Operators have the signature
 *
 *                    float op(float a, float b [, float param])
 *
 *                where a = sphere SDF value, b = box SDF value at
 *                the query point.  Operators are dispatched through
 *                a switch on the active op_idx.
 *
 *                Per pixel:
 *                  1. ray from camera through cell
 *                  2. sphere-march scene_sdf using active operator
 *                  3. on hit: 4-tap tetrahedral normal, Phong shade
 *                  4. (intensity, theme) → glyph + colour pair
 *
 * Operators    : Drawn from Inigo Quílez's distance-function catalogue.
 *                The 13 here cover every standard family:
 *                  – HARD (4)     min / max / max(·,−·) / xor
 *                  – SMOOTH (3)   polynomial smin and dual / subtract
 *                  – ROUND (3)    length-of-positives form
 *                  – CHAMFER (3)  45° bevel form
 *
 *                Parameter handling per family:
 *                  HARD     none  (sharp seam, no knob)
 *                  SMOOTH   k     polynomial bandwidth, raw
 *                  ROUND    r     fillet radius, scaled = param · 0.5
 *                  CHAMFER  r     bevel width,  scaled = param · 0.5
 *
 * Data         : Stateless math (Vec3 + 13 operators + sphere/box
 *                primitives + scene dispatch).  ONE Scene struct
 *                holds time, camera state, op_idx, theme, param,
 *                debug mode, paused flag.  Per-pixel result is Hit
 *                (hit / intensity / N / hit_t / steps) so the four
 *                debug overlays can read different signals from a
 *                single render pass.
 *
 * Rendering    : One ray per terminal cell.  Glyph from a 13-char
 *                ramp; colour from the active theme's 8-band luma
 *                palette.  Two-row HUD per CLAUDE.md spec.  Four
 *                debug overlays cycled with d / D.
 *
 * Performance  : One scene_sdf call per march step (~30-90 steps);
 *                4 more for the normal estimator.  ~120-360 ops per
 *                hit pixel on the most expensive family (smooth-
 *                intersect).  At 80×24 that's ~600 K ops per frame
 *                — comfortably 60 fps.
 *
 * References   :
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer* 12(10):527-545.
 *   • Quílez, I. — "Distance Functions" + "Smooth Minimum"
 *     https://iquilezles.org/articles/distfunctions/
 *     https://iquilezles.org/articles/smin/
 *     Source for all 13 operators in §7..§10.
 *   • Phong, B. T. (1975) — "Illumination for Computer Generated
 *     Pictures", *CACM* 18(6):311-317.  The lighting model in §15.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Combining two SDFs into one is just a TWO-ARGUMENT FUNCTION on
 * scalars: op(a, b) → c.  The signed-distance convention "negative
 * inside, positive outside" is preserved through the operator, so
 * the result is itself a valid SDF and the marcher doesn't care
 * how complicated the tree of operators is — it sees one number.
 * What VARIES from operator to operator is the SEAM: the shape of
 * the surface along the locus where a = b (or a = ±b for subtract).
 * The HARD family produces a sharp ridge there.  The SMOOTH, ROUND,
 * and CHAMFER families produce three different smoothed shapes.
 * The 13 operators all behave identically AWAY from the seam —
 * the operator's character lives ENTIRELY at the boundary.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of joining two pieces of timber.  You have four ways to do
 * the join:
 *
 *      HARD     pieces meet at a sharp corner              ↓
 *      SMOOTH   pieces meet with a soap-bubble bulge       ↓
 *      ROUND    pieces meet with a circular fillet         ↓
 *      CHAMFER  pieces meet at a 45° angled facet          ↓
 *
 * In each case the BULK of each piece is unchanged — only the join
 * differs.  Same here: away from the seam, all four families
 * produce identical SDFs.  Only along the locus where the two
 * primitives meet do you see what makes each family distinct.
 *
 * The encyclopedia laid out in cross-section through the seam:
 *
 *      HARD              SMOOTH (k=0.2)         ROUND (r=0.2)         CHAMFER
 * (r=0.2) sphere ─────╮     sphere ──────╮         sphere ─────╮         sphere
 * ─────╮ ╲                  ╲                     ╲                     ╲ ╲ ╲
 * ╲                     ╲ ╲              ╲   smooth          ╲    circular ╲
 * 45°              ╲ ╲              ╲  bulge            ╲   fillet ╲   bevel ╲
 *      box ────────╯    box ──────────╯       box ────────╯              box
 * ────────╯ ▲ ridge          ▲ no ridge —          ▲ no ridge —              ▲
 * no ridge — sharp (C⁰         C¹ smooth              C¹ true C⁰ planar crease)
 * polynomial              circular arc            45° cut
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per tick:
 *   1. animate    advance time; cam_yaw + box swing position update
 *
 * Per pixel:
 *   2. ray        primary ray from camera through cell
 *   3. trace      sphere-march scene_sdf(p, time, op_idx, param)
 *                 with under-relaxation (× 0.85) — the round /
 *                 chamfer SDFs aren't perfectly Lipschitz-1 near
 *                 the seam
 *   4. on hit     4-tap tetrahedral normal
 *                 Phong intensity (KA + KD·N·L + KS·spec)
 *                 also store N / hit_t / steps for debug overlays
 *
 * Per cell:
 *   5. quantise   intensity → glyph + theme colour
 *                 OR debug overlay reads N / hit_t / steps directly
 *
 * KEY FORMULAS
 * ────────────
 * Sphere SDF:                f = |p| − r
 * Rounded box SDF (Quílez):  q = |p| − b
 *                            f = |max(q, 0)| + min(max_component(q), 0) − r
 *
 * HARD family (exact set operations):
 *      union          min(a, b)
 *      intersect      max(a, b)
 *      subtract       max(a, −b)
 *      xor            max(min(a, b), −max(a, b))   symmetric difference
 *
 * SMOOTH family (Quílez polynomial smin):
 *      h     = max(k − |a − b|, 0) / k             ∈ [0, 1]
 *      smin  = min(a, b) − h² · k / 4
 *      smax  = −smin(−a, −b)                       (dual)
 *      smooth_sub = smax with second operand inverted
 *
 * ROUND family (length-of-positives, Quílez):
 *      ux = max(r − a, 0)
 *      uy = max(r − b, 0)
 *      round_union = max(r, min(a, b)) − sqrt(ux² + uy²)
 *      (intersect / subtract: similar with sign flips)
 *
 * CHAMFER family (Quílez 45° bevel):
 *      chamfer_union = min(min(a, b), (a − r + b) · √½)
 *      The third term is the distance to a 45° plane offset by r —
 *      the two min()s pick whichever surface is closest.
 *
 * Sphere trace (with under-relaxation):
 *      t ← t + d · 0.85    on each step
 *      hit when d < HIT_EPS, miss when t > MAX_T
 *
 * 4-tap tetrahedral normal:
 *      k = {(+ε,−ε,−ε), (−ε,−ε,+ε), (−ε,+ε,−ε), (+ε,+ε,+ε)}
 *      N = normalise(Σ kᵢ · sdf(p + kᵢ))
 *
 * Phong intensity:
 *      I = KA + KD · max(0, N·L) + KS · max(0, R·V)^SHIN
 *      R = 2(N·L)·N − L
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Smooth/round/chamfer parameters are clamped to [0.05, 1.0].
 *     At param = 0 the formulas divide by zero (smooth) or
 *     collapse to plain min (round/chamfer).  The smooth formula
 *     special-cases k < 1e-6 → fall back to plain min so the
 *     transition through 0 is well-defined.
 *
 *   • SUBTRACTION is non-commutative: max(a, −b) drills b out of
 *     a, NOT vice versa.  The dispatch always passes (sphere, box)
 *     as (a, b), so "subtract" reads as "sphere with box-shaped
 *     notch removed".  Swap operands and you'd get "box with
 *     sphere-shaped notch removed" — different geometry.
 *
 *   • XOR's formula `max(min(a,b), −max(a,b))` produces the
 *     SYMMETRIC DIFFERENCE — regions in EXACTLY ONE of the two
 *     shapes.  At the overlap region, both shapes' "insides"
 *     cancel, and that volume becomes outside.  Visually: the
 *     overlap is HOLLOW.
 *
 *   • Family change with op-kind preservation: pressing 2 from
 *     HARD/INTERSECT lands on SMOOTH/INTERSECT.  But HARD has 4
 *     ops (incl. XOR) while the others have 3.  Pressing 2 from
 *     HARD/XOR clamps to SMOOTH/SUBTRACT (the last available op
 *     in the SMOOTH row).  A small concession to the asymmetry.
 *
 *   • Round / chamfer SDFs are not perfectly Lipschitz-1 near the
 *     seam — the smooth bulge or bevel can slightly exceed unit
 *     gradient.  We under-relax the march by × 0.85 to be safe.
 *     Set step_scale = 1.0 and you'll see speckle on the bevel.
 *
 *   • The sphere trace's tetrahedral normal samples the SDF at four
 *     points around the hit.  At sharp HARD seams the normal can
 *     jump abruptly between the two operand surfaces.  ε = 0.002
 *     blurs that jump over a small region — visible only on close
 *     inspection of the HARD/INTERSECT lens.
 *
 *   • Camera orbit + box swing both stop on pause; the operator
 *     stays static so you can study the seam closely.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Default at startup: HARD/UNION.  Sphere and box visible
 *     simultaneously, sharp circular seam where they meet.
 *
 *   • Press n: HARD/INTERSECT.  Only the lens-shaped overlap
 *     region renders.  Confirms max(a, b) keeps only points where
 *     BOTH a and b are negative.
 *
 *   • Press n: HARD/SUBTRACT.  Sphere with a box-shaped bite
 *     taken out.  Press n again: HARD/XOR — everything visible
 *     EXCEPT the overlap.
 *
 *   • Press 2 (jump to SMOOTH).  Land on SMOOTH/SUBTRACT (clamp
 *     from XOR's column 3 to SMOOTH's max column 2).  Press n to
 *     reach SMOOTH/UNION.  Adjust + / − to grow / shrink the
 *     smooth bulge between the operands.
 *
 *   • Press 3 (ROUND), then n until UNION.  Note: ROUND's seam
 *     is a TRUE CIRCULAR FILLET (constant-curvature arc) where
 *     SMOOTH's was a polynomial bulge.  Visible difference at
 *     large parameter values.
 *
 *   • Press 4 (CHAMFER), then n.  Chamfer's seam is a HARD 45°
 *     PLANAR FACET — distinct visual from both smooth and round.
 *
 *   • Press d to cycle debug overlays.  NORMALS shows raw geometry
 *     hue (no lighting).  DEPTH brightens closer hits.  STEPS makes
 *     silhouettes glow (more steps to converge at grazing angles).
 *
 *   • Press space.  Camera + box freeze; operator stays active —
 *     useful for inspecting the seam closely.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — ten short answers ─────────────────────────────── *
 *
 *
 * T1: Why booleans on SDFs are interesting
 * ────────────────────────────────────────
 * SDFs encode geometry as a continuous function `f(p) → distance`.
 * That continuity makes set operations trivial: the union of two
 * shapes is just the MINIMUM of their two distance functions at
 * every point.  No vertex enumeration, no triangle clipping, no
 * polygon clipping — just `min(a, b)`.  Intersection is `max`.
 * Subtraction is `max(a, −b)`.  All exact, all closed-form.
 *
 * Triangle meshes can't do CSG this cleanly — boolean ops on meshes
 * require Constructive Solid Geometry algorithms that compute
 * triangle-vs-triangle intersections, build classification graphs,
 * and stitch new meshes.  SDFs sidestep all of that.  Cost: you
 * need the iterative sphere-trace renderer to view the result.
 *
 * The 13 operators in this file are the COMPLETE VOCABULARY of
 * boolean / quasi-boolean operators commonly used in SDF modelling.
 * Pick any subset of them, compose them in a tree, and you have a
 * geometry — that's what sdf_gallery.c's scene 5 (the humanoid) is.
 *
 *
 * T2: The atom — `min(a, b)` is set union
 * ───────────────────────────────────────
 * Geometric reasoning: at any point p, `min(d_a, d_b)` is the
 * distance to the union of A and B.
 *
 *      Case 1: p inside A but outside B
 *          d_a < 0  (signed-distance "inside" convention)
 *          d_b > 0
 *          min(d_a, d_b) = d_a < 0   →   p is INSIDE the result ✓
 *
 *      Case 2: p outside both
 *          d_a > 0,  d_b > 0
 *          min(d_a, d_b) = whichever shape is closer
 *                        = distance to nearest surface point of A∪B ✓
 *
 *      Case 3: p inside both (overlap region)
 *          d_a < 0,  d_b < 0
 *          min(d_a, d_b) = the more-negative value
 *                        = "deeper inside" → still negative → INSIDE ✓
 *
 * In all three cases the sign of the result correctly reports
 * inside/outside the UNION, and the magnitude is a valid distance
 * estimate.
 *
 *
 * T3: From min, derive intersection and subtraction
 * ─────────────────────────────────────────────────
 * INTERSECTION (A ∩ B) — set of points inside BOTH:
 *
 *      max(d_a, d_b)
 *
 * Reasoning: p is inside the intersection iff it's inside BOTH A
 * AND B.  That requires both d_a < 0 AND d_b < 0.  `max` picks the
 * less-negative (closer to surface) value.  Sign:
 *
 *      both negative    → max picks the larger of the two negatives
 *                          → still negative → INSIDE ✓
 *      one positive     → max picks the positive
 *                          → positive → OUTSIDE ✓
 *
 * SUBTRACTION (A ∖ B) — set of points inside A but OUTSIDE B:
 *
 *      max(d_a, −d_b)
 *
 * Reasoning: A ∖ B = A ∩ (¬B).  The complement ¬B has the SDF
 * `−d_b` (sign-flipped: the inside of ¬B is the outside of B and
 * vice versa).  So intersect A with ¬B → max(d_a, −d_b).  Drills
 * B out of A.
 *
 * XOR — symmetric difference (in EXACTLY ONE of A, B):
 *
 *      max(min(d_a, d_b), −max(d_a, d_b))
 *
 * The interior is `min(d_a, d_b) < 0` (inside the union) AND
 * NOT `max(d_a, d_b) < 0` (not in the intersection).  Combine
 * those two conditions in SDF form via max → the formula above.
 *
 *
 * T4: HARD family produces sharp seams (with discontinuous derivative)
 * ────────────────────────────────────────────────────────────────────
 * HARD union `min(a, b)` is exact, but the resulting surface has a
 * RIDGE wherever a = b — the locus where the two operand surfaces
 * meet.  The derivative of min at a = b jumps from "follow A" to
 * "follow B" abruptly.  In rendering this shows as:
 *
 *      ●─────────────●
 *       \  sphere  / \  cube
 *        \        /   \
 *         \   sharp ridge here (a = b)
 *          \  ╱
 *           ●
 *
 * For some applications a sharp ridge is what you want (mechanical
 * parts, blocky architecture).  For others (organic shapes,
 * jewellery) you want the seam SMOOTHED.  The next three families
 * are three different ways of doing that.
 *
 *
 * T5: The seam is a knob — three ways of softening it
 * ───────────────────────────────────────────────────
 * Replace the sharp `min` with a function that's continuous near
 * the seam.  Three families of solutions:
 *
 *      SMOOTH   polynomial bulge.  Outside a bandwidth `k` from
 *               the seam, behaves like plain min.  Inside the
 *               band, replaces min with a polynomial that smoothly
 *               interpolates AND adds an outward bulge.
 *
 *      ROUND    circular fillet.  Inside a radius `r` from the
 *               seam, replaces the corner with a true CIRCULAR
 *               cross-section.  Outside the band, behaves like min.
 *
 *      CHAMFER  45° bevel.  Inside a radius `r` from the seam,
 *               cuts the corner at a hard 45° angle.  Outside the
 *               band, behaves like min.
 *
 * All three are PARAMETERISED — you can dial the seam from "very
 * gentle" (param near 1) to "almost a sharp ridge" (param near 0).
 * Each family produces a different visual character for the same
 * parameter value, so trying all three on the same primitives
 * shows you what each one actually means.
 *
 *
 * T6: SMOOTH UNION — Quílez polynomial smin, derived
 * ──────────────────────────────────────────────────
 *      h = max(k − |a − b|, 0) / k                ∈ [0, 1]
 *      smin = min(a, b) − h² · k / 4
 *
 * Walking through h:
 *
 *      |a − b| ≥ k:  k − |a−b| ≤ 0 → max(·, 0) = 0 → h = 0
 *                      → smin = min(a, b)  (plain min, no smoothing)
 *
 *      |a − b| < k:  the two operands are within bandwidth k.
 *                      h = (k − |a−b|) / k  ∈ (0, 1]
 *                      h² · k / 4 is a positive correction to subtract
 *                      from the plain min — pulls the surface OUTWARD
 *                      (more negative = more "inside").
 *
 *      a = b (the seam):
 *                      h = 1
 *                      correction = k / 4 (maximum bulge)
 *                      smin = min(a, b) − k/4 = a − k/4
 *
 * Visual: the surface bulges OUTWARD into the gap between the two
 * primitives by an amount k/4 at the seam.  Set k = 0 → the bulge
 * vanishes and you're back to plain min.
 *
 *
 * T7: ROUND UNION — circular fillet via length-of-positives
 * ─────────────────────────────────────────────────────────
 *      ux = max(r − a, 0)
 *      uy = max(r − b, 0)
 *      round_union = max(r, min(a, b)) − sqrt(ux² + uy²)
 *
 * Walking through:
 *
 *      Both a and b ≥ r (well outside the band):
 *          ux = uy = 0 → sqrt = 0
 *          round_union = max(r, min(a, b)) = min(a, b)
 *          → behaves like plain min
 *
 *      Both a and b ≤ r (inside the band, near the seam):
 *          ux = r − a > 0,  uy = r − b > 0
 *          sqrt(ux² + uy²) is the EUCLIDEAN distance from the
 *          point (a, b) ∈ ℝ² to the corner (r, r)
 *          → the result subtracts that 2-D Euclidean distance
 *
 *      Geometric interpretation: in (a, b)-space, the locus of
 *      points where round_union = 0 is a CIRCULAR ARC of radius
 *      r centred at (r, r).  That's why the fillet is a TRUE
 *      circular cross-section — not a polynomial approximation.
 *
 *      Different from SMOOTH: smooth's seam is a polynomial bulge;
 *      round's seam is a constant-curvature circular arc.  At
 *      large param values the difference is visible — round
 *      produces a perfect quarter-circle fillet, smooth produces
 *      a softer polynomial bulge.
 *
 *
 * T8: CHAMFER UNION — 45° bevel via plane offset
 * ──────────────────────────────────────────────
 *      chamfer_union = min(min(a, b), (a − r + b) · √½)
 *
 * The first inner min is plain HARD union.  The third operand
 * `(a − r + b) · √½` is the distance to a 45° PLANE offset by r.
 *
 * Walking through:
 *
 *      Far from seam:  the third term is large; outer min picks
 *                       plain min(a, b)
 *
 *      Near the seam:  the third term becomes the smallest;
 *                       outer min picks IT — that's the 45° plane
 *
 *      → seam is replaced by a flat 45° angled facet of width r
 *
 * Different from SMOOTH (polynomial bulge) and ROUND (circular
 * arc): chamfer's seam is PLANAR.  Visual: you can SEE the flat
 * 45° face at the join, where smooth and round produce smoothly
 * curved surfaces.
 *
 *
 * T9: Each family has UNION / INTERSECT / SUBTRACT variants
 * ─────────────────────────────────────────────────────────
 * Once you have UNION for a family, the dual (INTERSECTION) follows
 * by sign manipulation: smax(a, b) = −smin(−a, −b).  And SUBTRACTION
 * is just intersection with the second operand inverted: subtract
 * = smax(a, −b).
 *
 * For round and chamfer, the dual / subtract variants follow similar
 * sign-flip patterns (see §9, §10 for the exact formulas).
 *
 * The HARD family additionally has XOR — the symmetric difference
 * — which has no smooth/round/chamfer counterpart in standard SDF
 * vocabulary because there's no well-defined "smooth" version of
 * the XOR seam (it has TWO seams, not one).
 *
 * Net total:
 *      HARD     4 ops (union, intersect, subtract, xor)
 *      SMOOTH   3 ops (union, intersect, subtract)
 *      ROUND    3 ops
 *      CHAMFER  3 ops
 *      = 13 operators in the encyclopedia
 *
 *
 * T10: The encyclopedia laid out as a 2-D table
 * ─────────────────────────────────────────────
 *
 *                        Union  Intersect  Subtract  XOR
 *           1 HARD         ●         ●         ●       ●
 *           2 SMOOTH       ●         ●         ●
 *           3 ROUND        ●         ●         ●
 *           4 CHAMFER      ●         ●         ●
 *
 * The 1-4 number keys pick the FAMILY (which row).  n / N cycles
 * the OP KIND (which column) WITHIN the current family — never
 * crosses the row boundary.  Press 2 from any HARD op to land at
 * the same op-kind in SMOOTH.  Press 1 to come back.  HARD has a
 * 4th column (XOR); other families clamp to column 2 (SUBTRACT)
 * when arriving from XOR.
 *
 * Two independent navigation axes → no surprise jumps.  The
 * encyclopedia structure makes it easy to compare across families
 * (hold op-kind, vary family) or across op-kinds (hold family,
 * vary op-kind).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* End of textbook.  The rest of the file is the worked exercises. */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,

  FPS_UPDATE_MS = 500,
  HUD_ROWS = 2, /* row 0 status + last row hint */

  LUMI_N = 8,
  PAIR_HUD = LUMI_N + 1,
  PAIR_HINT = LUMI_N + 2,

  OP_COUNT = 13,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define CELL_ASPECT 2.0f

/* §1.1 sphere trace tunables.
 *      MARCH_MAX     hard step cap per ray
 *      MARCH_EPS     "we're touching" threshold
 *      MARCH_FAR     ray-length budget before declaring miss
 *      NORMAL_EPS    tetrahedron offset for finite-difference normal
 *      MARCH_RELAX   under-relaxation factor — round/chamfer SDFs
 *                    aren't perfectly Lipschitz-1; α<1 keeps the
 *                    marcher from overshooting near the seam
 */
#define MARCH_MAX 90
#define MARCH_EPS 0.002f
#define MARCH_FAR 20.0f
#define NORMAL_EPS 0.002f
#define MARCH_RELAX 0.85f

/* §1.2 camera + zoom. */
#define CAM_DIST_DEFAULT 3.5f
#define CAM_DIST_MIN 2.0f
#define CAM_DIST_MAX 7.0f
#define CAM_ZOOM_STEP 0.30f
#define CAM_ELEVATION 0.55f  /* radians above horizon                */
#define CAM_ORBIT_RATE 0.30f /* radians / sec                        */
#define FOV_DEG 50.0f

/* §1.3 operand animation — box swings in/out of the sphere. */
#define BOX_SWING_AMP 1.05f  /* peak X offset of the box vs sphere   */
#define BOX_SWING_RATE 0.55f /* radians / sec                        */
#define SPHERE_RADIUS 1.00f
#define BOX_HALF_EXTENT 0.70f
#define BOX_CORNER_R 0.05f

/* §1.4 boolean parameter (k or r) — single user-controlled scalar in
 * [PARAM_MIN, PARAM_MAX].  SMOOTH uses raw k; ROUND/CHAMFER scale by
 * 0.5 so r ∈ [0.025, 0.5]. */
#define PARAM_DEFAULT 0.30f
#define PARAM_MIN 0.05f
#define PARAM_MAX 1.00f
#define PARAM_STEP 0.05f

/* §1.5 Phong + light. */
#define KA 0.10f
#define KD 0.78f
#define KS 0.55f
#define SHIN 40.0f
#define LIGHT_X 3.0f
#define LIGHT_Y 4.5f
#define LIGHT_Z 2.5f

/* §1.6 luma ramp. */
static const char LUMA_RAMP[] = " .,:;+*oxOX#@";
#define RAMP_LEN ((int)(sizeof LUMA_RAMP - 1)) /* = 13 */

/* §1.7 themes — 6 × 8 luma slots.  Same palette family as
 * raymarcher.c, raymarcher_cube.c, etc. */
typedef struct {
  const char *name;
  short ramp[LUMI_N];
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    {"CLASSIC ", {235, 238, 241, 244, 247, 250, 253, 255}},
    {"AMBER   ", {130, 136, 166, 172, 178, 208, 214, 220}},
    {"MATRIX  ", {28, 34, 40, 46, 82, 118, 154, 190}},
    {"NEON    ", {53, 91, 129, 165, 201, 207, 213, 227}},
    {"ICE     ", {25, 31, 38, 45, 51, 87, 123, 159}},
    {"COPPER  ", {94, 130, 136, 166, 172, 208, 214, 220}},
};

/* §1.8 operator catalogue.  Order is contiguous by family — the
 * FAMILY_FIRST array (§11) relies on this.  HARD has 4 entries
 * (incl. XOR); other families have 3 each. */
typedef enum {
  OP_HARD_UNION = 0,
  OP_HARD_INTERSECT,
  OP_HARD_SUBTRACT,
  OP_HARD_XOR,

  OP_SMOOTH_UNION,
  OP_SMOOTH_INTERSECT,
  OP_SMOOTH_SUBTRACT,

  OP_ROUND_UNION,
  OP_ROUND_INTERSECT,
  OP_ROUND_SUBTRACT,

  OP_CHAMFER_UNION,
  OP_CHAMFER_INTERSECT,
  OP_CHAMFER_SUBTRACT,
} OpIndex;

/* §1.9 debug overlays — d / D cycles between them. */
typedef enum {
  DEBUG_NORMAL = 0,  /* full lit production view                */
  DEBUG_NORMALS = 1, /* surface normal direction → colour       */
  DEBUG_DEPTH = 2,   /* hit distance t → brightness             */
  DEBUG_STEPS = 3,   /* march step count → brightness           */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "NORMALS",
    "DEPTH  ",
    "STEPS  ",
};

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ── §3 color — themes + 2-pair HUD spec ─────────────────────────────── */

static void theme_apply(int idx) {
  if (idx < 0 || idx >= THEME_COUNT)
    idx = 0;
  const Theme *t = &THEMES[idx];
  if (COLORS >= 256) {
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), t->ramp[i], COLOR_BLACK);
  } else {
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), COLOR_WHITE, COLOR_BLACK);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

static attr_t lumi_attr(int slot) {
  if (slot < 0)
    slot = 0;
  if (slot >= LUMI_N)
    slot = LUMI_N - 1;
  attr_t a = COLOR_PAIR(slot + 1);
  if (COLORS < 256) {
    if (slot < 3)
      a |= A_DIM;
    else if (slot >= 6)
      a |= A_BOLD;
  }
  return a;
}

/* ── §4 vec3 — value-type 3-D math ───────────────────────────────────── */

typedef struct {
  float x, y, z;
} Vec3;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3 v3add(Vec3 a, Vec3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline Vec3 v3sub(Vec3 a, Vec3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline Vec3 v3mul(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}
static inline float v3dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len(Vec3 a) { return sqrtf(v3dot(a, a)); }
static inline Vec3 v3norm(Vec3 a) {
  float L = v3len(a);
  return (L > 1e-7f) ? v3mul(a, 1.0f / L) : v3(0, 0, 1);
}

/* Component-wise abs / max-with-zero — used in the box SDF. */
static inline Vec3 v3abs(Vec3 a) {
  return v3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
}
static inline Vec3 v3max0(Vec3 a) {
  return v3(fmaxf(a.x, 0.0f), fmaxf(a.y, 0.0f), fmaxf(a.z, 0.0f));
}

/* ── §5 sphere SDF ───────────────────────────────────────────────────── *
 *
 *      f(p, r) = |p| − r
 *
 * Positive outside, negative inside, true Euclidean distance.  See
 * raymarcher.c Tutorial T2 for the derivation.
 */
static float sd_sphere(Vec3 p, float r) { return v3len(p) - r; }

/* ── §6 rounded-box SDF — Quílez exact + offset ──────────────────────── *
 *
 *      q = |p| − b                  (component-wise)
 *      f = |max(q, 0)| + min(max(q.x, q.y, q.z), 0) − r
 *
 * The first two terms are Quílez's exact box SDF (raymarcher_cube.c
 * Tutorial T4 derived this).  Subtracting r at the end offsets the
 * surface outward by r — gives the box rounded corners.
 */
static float sd_round_box(Vec3 p, Vec3 b, float r) {
  Vec3 q = v3sub(v3abs(p), b);
  Vec3 q_pos = v3max0(q);
  float outside = v3len(q_pos);
  float inside = fminf(fmaxf(q.x, fmaxf(q.y, q.z)), 0.0f);
  return outside + inside - r;
}

/* ── §7 HARD operators — exact set operations (T2, T3, T4) ───────────── *
 *
 * Sharp seams (discontinuous derivative at a = b).  Otherwise true
 * Lipschitz-1 distances.  All four formulas in two-line
 * implementations.
 */

/* HARD UNION:  point inside if inside EITHER operand. */
static inline float op_hard_union(float a, float b) { return fminf(a, b); }

/* HARD INTERSECT:  point inside if inside BOTH operands. */
static inline float op_hard_intersect(float a, float b) { return fmaxf(a, b); }

/* HARD SUBTRACT:  point inside A but OUTSIDE B (drill B out of A). */
static inline float op_hard_subtract(float a, float b) { return fmaxf(a, -b); }

/* HARD XOR:  symmetric difference — inside exactly ONE of A, B. */
static inline float op_hard_xor(float a, float b) {
  return fmaxf(fminf(a, b), -fmaxf(a, b));
}

/* ── §8 SMOOTH operators — Quílez polynomial smin family (T6) ────────── *
 *
 *      h = max(k − |a − b|, 0) / k
 *      smin = min(a, b) − h² · k / 4
 *      smax = −smin(−a, −b)            (dual)
 *
 * SMOOTH SUBTRACT applies the dual smax form with the second operand
 * negated.  All three special-case k < 1e-6 → fall back to the
 * corresponding HARD operator (avoids divide-by-zero).
 */

static inline float op_smooth_union(float a, float b, float k) {
  if (k < 1e-6f)
    return fminf(a, b);
  float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
  return fminf(a, b) - h * h * k * 0.25f;
}

static inline float op_smooth_intersect(float a, float b, float k) {
  if (k < 1e-6f)
    return fmaxf(a, b);
  float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
  return fmaxf(a, b) + h * h * k * 0.25f;
}

static inline float op_smooth_subtract(float a, float b, float k) {
  if (k < 1e-6f)
    return fmaxf(a, -b);
  float h = fmaxf(k - fabsf(a + b), 0.0f) / k;
  return fmaxf(a, -b) + h * h * k * 0.25f;
}

/* ── §9 ROUND operators — length-of-positives form (T7) ──────────────── *
 *
 *      ux = max(r − a, 0)
 *      uy = max(r − b, 0)
 *      round_union = max(r, min(a, b)) − sqrt(ux² + uy²)
 *
 * Tutorial T7 derived this.  At points well outside the bandwidth r,
 * ux = uy = 0 and the formula collapses to plain min.  Inside the
 * band, the sqrt(ux² + uy²) term is the 2-D Euclidean distance from
 * (a, b) to the corner (r, r) — that's why the seam is a TRUE
 * circular arc.
 */

static inline float op_round_union(float a, float b, float r) {
  float ux = fmaxf(r - a, 0.0f);
  float uy = fmaxf(r - b, 0.0f);
  return fmaxf(r, fminf(a, b)) - sqrtf(ux * ux + uy * uy);
}

static inline float op_round_intersect(float a, float b, float r) {
  float ux = fmaxf(r + a, 0.0f);
  float uy = fmaxf(r + b, 0.0f);
  return fminf(-r, fmaxf(a, b)) + sqrtf(ux * ux + uy * uy);
}

static inline float op_round_subtract(float a, float b, float r) {
  float ux = fmaxf(r + a, 0.0f);
  float uy = fmaxf(r - b, 0.0f);
  return fminf(-r, fmaxf(a, -b)) + sqrtf(ux * ux + uy * uy);
}

/* ── §10 CHAMFER operators — 45° bevel form (T8) ─────────────────────── *
 *
 *      chamfer_union = min(min(a, b), (a − r + b) · √½)
 *
 * Tutorial T8 derived this.  The third operand is the distance to a
 * 45° plane offset by r.  At the seam, the outer min picks that
 * plane → result is a flat 45° facet of width r.  Far from the seam,
 * the third term is large and the outer min picks plain min(a, b).
 */

#define INV_SQRT2 0.70710678f

static inline float op_chamfer_union(float a, float b, float r) {
  return fminf(fminf(a, b), (a - r + b) * INV_SQRT2);
}

static inline float op_chamfer_intersect(float a, float b, float r) {
  return fmaxf(fmaxf(a, b), (a + r + b) * INV_SQRT2);
}

static inline float op_chamfer_subtract(float a, float b, float r) {
  return fmaxf(fmaxf(a, -b), (a + r - b) * INV_SQRT2);
}

/* ── §11 op table + family helpers (T10) ─────────────────────────────── *
 *
 * The encyclopedia is a 2-D table — family on Y axis, op-kind on X.
 * Two arrays + two helpers expose this 2-D structure to the rest of
 * the code:
 *
 *      FAMILY_FIRST[f]   first op_idx in family f
 *      FAMILY_SIZE[f]    number of ops in family f (3 or 4)
 *      family_of(op)     which family op_idx belongs to (0..3)
 *      op_in_family(op)  position within family (0..size-1)
 */

typedef struct {
  const char *op_kind; /* UNION / INTERSECT / SUBTRACT / XOR     */
  const char *family;  /* HARD / SMOOTH / ROUND / CHAMFER         */
  bool uses_param;
} OpInfo;

static const OpInfo OP_TABLE[OP_COUNT] = {
    /* HARD family — 4 ops (the only one with XOR) */
    {"UNION    ", "HARD   ", false},
    {"INTERSECT", "HARD   ", false},
    {"SUBTRACT ", "HARD   ", false},
    {"XOR      ", "HARD   ", false},

    /* SMOOTH family — 3 ops */
    {"UNION    ", "SMOOTH ", true},
    {"INTERSECT", "SMOOTH ", true},
    {"SUBTRACT ", "SMOOTH ", true},

    /* ROUND family — 3 ops */
    {"UNION    ", "ROUND  ", true},
    {"INTERSECT", "ROUND  ", true},
    {"SUBTRACT ", "ROUND  ", true},

    /* CHAMFER family — 3 ops */
    {"UNION    ", "CHAMFER", true},
    {"INTERSECT", "CHAMFER", true},
    {"SUBTRACT ", "CHAMFER", true},
};

#define FAMILY_COUNT 4
static const int FAMILY_FIRST[FAMILY_COUNT] = {
    OP_HARD_UNION,    /* 1 — HARD,    4 ops (incl. XOR) */
    OP_SMOOTH_UNION,  /* 2 — SMOOTH,  3 ops */
    OP_ROUND_UNION,   /* 3 — ROUND,   3 ops */
    OP_CHAMFER_UNION, /* 4 — CHAMFER, 3 ops */
};
static const int FAMILY_SIZE[FAMILY_COUNT] = {4, 3, 3, 3};

/* family_of — given an op_idx, return its family index (0..3). */
static int family_of(int op_idx) {
  for (int f = FAMILY_COUNT - 1; f >= 0; f--)
    if (op_idx >= FAMILY_FIRST[f])
      return f;
  return 0;
}

/* op_in_family — given an op_idx, return its position within family. */
static int op_in_family(int op_idx) {
  return op_idx - FAMILY_FIRST[family_of(op_idx)];
}

/* ── §12 scene SDF — animated operands + operator dispatch ───────────── *
 *
 * Two operands held constant per frame:
 *      a = sphere at origin
 *      b = rounded box at swung x position
 *
 * Then the active operator is dispatched on op_idx.  Parameter
 * scaling per family: SMOOTH uses raw k; ROUND/CHAMFER scale by 0.5
 * so r ∈ [0.025, 0.5].
 *
 * The marcher sees only the result of this dispatch — it never
 * knows which operator is currently active.  Same march loop, same
 * normal estimator, regardless of which family the user picked.
 */

static float scene_sdf(Vec3 p, float time, int op_idx, float param) {
  /* Operand A: sphere at origin. */
  float a = sd_sphere(p, SPHERE_RADIUS);

  /* Operand B: rounded box at swung x position. */
  float swing = BOX_SWING_AMP * sinf(time * BOX_SWING_RATE);
  Vec3 box_pos = v3(swing, 0.0f, 0.0f);
  Vec3 bp = v3sub(p, box_pos);
  Vec3 b_ext = v3(BOX_HALF_EXTENT, BOX_HALF_EXTENT, BOX_HALF_EXTENT);
  float b = sd_round_box(bp, b_ext, BOX_CORNER_R);

  /* SMOOTH uses raw k; ROUND/CHAMFER scale param by 0.5 → r. */
  float r = param * 0.5f;

  switch (op_idx) {
  case OP_HARD_UNION:
    return op_hard_union(a, b);
  case OP_HARD_INTERSECT:
    return op_hard_intersect(a, b);
  case OP_HARD_SUBTRACT:
    return op_hard_subtract(a, b);
  case OP_HARD_XOR:
    return op_hard_xor(a, b);

  case OP_SMOOTH_UNION:
    return op_smooth_union(a, b, param);
  case OP_SMOOTH_INTERSECT:
    return op_smooth_intersect(a, b, param);
  case OP_SMOOTH_SUBTRACT:
    return op_smooth_subtract(a, b, param);

  case OP_ROUND_UNION:
    return op_round_union(a, b, r);
  case OP_ROUND_INTERSECT:
    return op_round_intersect(a, b, r);
  case OP_ROUND_SUBTRACT:
    return op_round_subtract(a, b, r);

  case OP_CHAMFER_UNION:
    return op_chamfer_union(a, b, r);
  case OP_CHAMFER_INTERSECT:
    return op_chamfer_intersect(a, b, r);
  case OP_CHAMFER_SUBTRACT:
    return op_chamfer_subtract(a, b, r);
  }
  return op_hard_union(a, b); /* unreachable; satisfy compiler */
}

/* ── §13 sphere trace — Hart 1996 with under-relaxation ──────────────── *
 *
 * Pseudocode:
 *      t = 0
 *      repeat MARCH_MAX:
 *          d = scene_sdf(ro + t·rd)
 *          if d < HIT_EPS:    HIT (return t, steps)
 *          if t > MAX_T:      MISS
 *          t += d · MARCH_RELAX
 *          steps++
 *
 * MARCH_RELAX = 0.85 under-relaxes the step.  Round/chamfer SDFs
 * aren't perfectly Lipschitz-1 near the seam — full step would
 * occasionally overshoot.  Cost: ~15% more march iterations on
 * average.
 */
static float sphere_trace(Vec3 ro, Vec3 rd, float time, int op_idx, float param,
                          int *out_steps) {
  float t = 0.0f;
  int step;
  for (step = 0; step < MARCH_MAX; step++) {
    Vec3 p = v3add(ro, v3mul(rd, t));
    float d = scene_sdf(p, time, op_idx, param);
    if (d < MARCH_EPS) {
      if (out_steps)
        *out_steps = step + 1;
      return t;
    }
    if (t > MARCH_FAR)
      break;
    t += d * MARCH_RELAX;
  }
  if (out_steps)
    *out_steps = step;
  return -1.0f;
}

/* ── §14 surface normal — 4-tap tetrahedral (Quílez) ─────────────────── *
 *
 *      k = {(+ε,−ε,−ε), (−ε,−ε,+ε), (−ε,+ε,−ε), (+ε,+ε,+ε)}
 *      N = normalise(Σ kᵢ · scene_sdf(p + kᵢ))
 *
 * 4 SDF evals per hit pixel.  No two offsets share an axis →
 * cleaner normals at sharp seams (HARD/INTERSECT's lens edge,
 * CHAMFER's bevel facet) than 6-tap central differences.
 */
static Vec3 sdf_normal(Vec3 p, float time, int op_idx, float param) {
  const float e = NORMAL_EPS;
  Vec3 k0 = v3(e, -e, -e);
  Vec3 k1 = v3(-e, -e, e);
  Vec3 k2 = v3(-e, e, -e);
  Vec3 k3 = v3(e, e, e);

  float d0 = scene_sdf(v3add(p, k0), time, op_idx, param);
  float d1 = scene_sdf(v3add(p, k1), time, op_idx, param);
  float d2 = scene_sdf(v3add(p, k2), time, op_idx, param);
  float d3 = scene_sdf(v3add(p, k3), time, op_idx, param);

  Vec3 n = v3add(v3add(v3mul(k0, d0), v3mul(k1, d1)),
                 v3add(v3mul(k2, d2), v3mul(k3, d3)));
  return v3norm(n);
}

/* ── §15 Phong shade — ambient + diffuse + specular ──────────────────── */

static float phong_shade(Vec3 N, Vec3 hit, Vec3 cam, Vec3 light) {
  Vec3 L_dir = v3norm(v3sub(light, hit));
  Vec3 V_dir = v3norm(v3sub(cam, hit));
  float ndl = fmaxf(0.0f, v3dot(N, L_dir));
  Vec3 R_dir = v3sub(v3mul(N, 2.0f * ndl), L_dir);
  float spec = powf(fmaxf(0.0f, v3dot(R_dir, V_dir)), SHIN);

  float I = KA + KD * ndl + KS * spec;
  if (I < 0.0f)
    I = 0.0f;
  if (I > 1.0f)
    I = 1.0f;
  return I;
}

/* ── §16 cast_ray — full per-pixel pipeline → Hit ────────────────────── *
 *
 * Hit carries everything any debug overlay might need:
 *
 *   hit       did the ray reach the surface?
 *   intensity Phong result in [0, 1]                (DEBUG_NORMAL)
 *   N         surface normal at hit                  (DEBUG_NORMALS)
 *   hit_t     ray-parameter at hit                   (DEBUG_DEPTH)
 *   steps     march iterations                       (DEBUG_STEPS)
 *
 * Production view reads .intensity; the three debug overlays read
 * .N / .hit_t / .steps directly without going through the Phong
 * + glyph quantisation path.
 */
typedef struct {
  bool hit;
  float intensity;
  Vec3 N;
  float hit_t;
  int steps;
} Hit;

static Hit cast_ray(int col, int row, int cw, int ch, Vec3 ro, Vec3 fwd,
                    Vec3 right, Vec3 up, float fov_t, Vec3 light, float time,
                    int op_idx, float param) {
  Hit h = {false, 0.0f, {0, 0, 1}, 0.0f, 0};

  float u = ((float)col + 0.5f) / (float)cw * 2.0f - 1.0f;
  float v = -((float)row + 0.5f) / (float)ch * 2.0f + 1.0f;
  float phys_aspect = ((float)ch * CELL_ASPECT) / (float)cw;

  Vec3 rd = v3norm(v3add(
      fwd, v3add(v3mul(right, u * fov_t), v3mul(up, v * fov_t * phys_aspect))));

  int steps = 0;
  float t = sphere_trace(ro, rd, time, op_idx, param, &steps);
  h.steps = steps;
  if (t < 0.0f)
    return h;

  h.hit = true;
  h.hit_t = t;
  Vec3 hit_p = v3add(ro, v3mul(rd, t));
  h.N = sdf_normal(hit_p, time, op_idx, param);
  h.intensity = phong_shade(h.N, hit_p, ro, light);
  return h;
}

/* ── §17 canvas — production view + 3 debug overlays ─────────────────── */

static char intensity_to_glyph(float intensity) {
  int idx = (int)(intensity * (float)(RAMP_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= RAMP_LEN)
    idx = RAMP_LEN - 1;
  return LUMA_RAMP[idx];
}

static attr_t intensity_to_attr(float intensity) {
  int idx = (int)(intensity * (float)(RAMP_LEN - 1) + 0.5f);
  int slot = (idx * LUMI_N) / RAMP_LEN;
  return lumi_attr(slot);
}

/* ── §18 scene state + tick + camera basis ───────────────────────────── */

typedef struct {
  int cols, rows;
  float time;
  int op_idx;
  int theme_index;
  float cam_dist;
  float cam_yaw; /* auto-orbit angle                  */
  float param;
  DebugMode debug_mode;
  bool paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->cols = cols;
  s->rows = rows;
  s->time = 0.0f;
  s->op_idx = OP_HARD_UNION;
  s->theme_index = 0;
  s->cam_dist = CAM_DIST_DEFAULT;
  s->cam_yaw = 0.5f;
  s->param = PARAM_DEFAULT;
  s->debug_mode = DEBUG_NORMAL;
  s->paused = false;
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reset(Scene *s) {
  s->time = 0.0f;
  s->op_idx = OP_HARD_UNION;
  s->cam_dist = CAM_DIST_DEFAULT;
  s->cam_yaw = 0.5f;
  s->param = PARAM_DEFAULT;
  s->debug_mode = DEBUG_NORMAL;
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->time += dt;
  s->cam_yaw += dt * CAM_ORBIT_RATE;
  if (s->cam_yaw > (float)(2.0 * M_PI))
    s->cam_yaw -= (float)(2.0 * M_PI);
}

/* Camera basis at the orbiting position, looking at origin. */
typedef struct {
  Vec3 origin, fwd, right, up;
  float fov_t;
} Camera;

static Camera camera_basis(const Scene *s) {
  float yaw = s->cam_yaw;
  float ele = CAM_ELEVATION;

  Camera c;
  c.origin = v3(s->cam_dist * cosf(ele) * cosf(yaw), s->cam_dist * sinf(ele),
                s->cam_dist * cosf(ele) * sinf(yaw));
  c.fwd = v3norm(v3sub(v3(0, 0, 0), c.origin));
  Vec3 wup = v3(0, 1, 0);
  /* right = normalise(fwd × world_up); up = right × fwd. */
  Vec3 r =
      v3(c.fwd.y * wup.z - c.fwd.z * wup.y, c.fwd.z * wup.x - c.fwd.x * wup.z,
         c.fwd.x * wup.y - c.fwd.y * wup.x);
  c.right = v3norm(r);
  c.up = v3(c.right.y * c.fwd.z - c.right.z * c.fwd.y,
            c.right.z * c.fwd.x - c.right.x * c.fwd.z,
            c.right.x * c.fwd.y - c.right.y * c.fwd.x);
  c.fov_t = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
  return c;
}

/* ── §19 screen — ncurses init + 2-row HUD + present ─────────────────── */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}

static void screen_resize(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/*
 * paint_cell — emit one cell with attron/attroff batched on (pair,
 * attr) change.  Adjacent cells in the same luma slot share one
 * attron call.
 */
static void paint_cell(int ty, int tx, char glyph, attr_t attr,
                       int *last_pair_attr) {
  int pa = (int)attr;
  if (pa != *last_pair_attr) {
    if (*last_pair_attr != -1)
      attroff((attr_t)*last_pair_attr);
    attron(attr);
    *last_pair_attr = pa;
  }
  mvaddch(ty, tx, (chtype)(unsigned char)glyph);
}

/*
 * scene_render — one ray per cell into rows 1..rows-2 (top + bottom
 * rows reserved for HUD).  Dispatches to one of four overlays.
 */
static void scene_render(const Scene *s) {
  int rows_eff = s->rows - HUD_ROWS;
  int y_offset = 1;
  if (rows_eff < 1)
    return;

  Camera cam = camera_basis(s);
  Vec3 light = v3(LIGHT_X, LIGHT_Y, LIGHT_Z);

  int last_pair_attr = -1;

  for (int row = 0; row < rows_eff; row++) {
    for (int col = 0; col < s->cols; col++) {
      Hit h =
          cast_ray(col, row, s->cols, rows_eff, cam.origin, cam.fwd, cam.right,
                   cam.up, cam.fov_t, light, s->time, s->op_idx, s->param);
      if (!h.hit) {
        if (last_pair_attr != -1) {
          attroff((attr_t)last_pair_attr);
          last_pair_attr = -1;
        }
        continue;
      }

      /* Pick the value that drives the chosen overlay. */
      float v;
      switch (s->debug_mode) {
      case DEBUG_NORMALS: {
        /* hue from azimuth atan2(N.x, N.z); brightness from N.y */
        float az = atan2f(h.N.x, h.N.z) / (2.0f * (float)M_PI) + 0.5f;
        float yl = h.N.y * 0.5f + 0.5f;
        if (yl < 0.0f)
          yl = 0.0f;
        if (yl > 1.0f)
          yl = 1.0f;
        /* glyph from y_lit, colour from azimuth */
        attr_t attr = intensity_to_attr(az);
        char glyph = intensity_to_glyph(yl);
        paint_cell(row + y_offset, col, glyph, attr, &last_pair_attr);
        continue;
      }
      case DEBUG_DEPTH: {
        /* closer hits get higher brightness */
        float t_min = s->cam_dist - 1.5f;
        float t_max = s->cam_dist + 1.5f;
        if (t_min < 0.0f)
          t_min = 0.0f;
        v = (t_max - h.hit_t) / (t_max - t_min);
        if (v < 0.0f)
          v = 0.0f;
        if (v > 1.0f)
          v = 1.0f;
        break;
      }
      case DEBUG_STEPS: {
        /* normalise step count → silhouette glow */
        v = (float)h.steps / (float)MARCH_MAX;
        if (v > 1.0f)
          v = 1.0f;
        break;
      }
      case DEBUG_NORMAL:
      default:
        v = h.intensity; /* production view */
        break;
      }

      char glyph = intensity_to_glyph(v);
      attr_t attr = intensity_to_attr(v);
      paint_cell(row + y_offset, col, glyph, attr, &last_pair_attr);
    }
  }
  if (last_pair_attr != -1)
    attroff((attr_t)last_pair_attr);
}

/*
 * hud_draw — CLAUDE.md HUD spec.
 *   row 0          PAIR_HUD  (yellow + bold) — title + fps left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 *
 * FPS lives in the LEFT label so it stays visible regardless of how
 * narrow the terminal is.
 */
static void hud_draw(const Screen *sc, const Scene *s, double fps) {
  /* Top row — title + fps left, status right (truncated to fit). */
  char left[48];
  snprintf(left, sizeof left, " CSG ATLAS  %5.1f fps ", fps);
  int llen = (int)strlen(left);

  const OpInfo *op = &OP_TABLE[s->op_idx];
  char param_field[16];
  if (op->uses_param)
    snprintf(param_field, sizeof param_field, "%4.2f", (double)s->param);
  else
    snprintf(param_field, sizeof param_field, "—");

  char status[200];
  snprintf(status, sizeof status,
           " family:%s  op:%s  param:%s  debug:%s  theme:%s  zoom:%4.2f  %s ",
           op->family, op->op_kind, param_field,
           DEBUG_MODE_NAMES[s->debug_mode], THEMES[s->theme_index].name,
           (double)s->cam_dist, s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  int max_slen = sc->cols - llen;
  if (max_slen < 0)
    max_slen = 0;
  if (slen > max_slen)
    slen = max_slen;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, 0, "%s", left);
  if (slen > 0)
    mvprintw(0, sc->cols - slen, "%.*s", slen, status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Bottom row — cyan key hint. */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  1-4:family(row)  n/N:op(col)  "
           "+/-:param  d/D:debug  t/T:theme  z/Z:zoom  r:reset ");
  clrtoeol();
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps) {
  erase();
  scene_render(s);
  hud_draw(sc, s, fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §20 app — main loop, signals, key handling ──────────────────────── */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
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

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
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
  case 'r':
  case 'R':
    scene_reset(s);
    break;

  case 'n': {
    /* Cycle op WITHIN the current family — never crosses to a
     * different family.  Press 1..4 to switch family. */
    int f = family_of(s->op_idx);
    int kind = (op_in_family(s->op_idx) + 1) % FAMILY_SIZE[f];
    s->op_idx = FAMILY_FIRST[f] + kind;
    break;
  }
  case 'N': {
    int f = family_of(s->op_idx);
    int kind = (op_in_family(s->op_idx) + FAMILY_SIZE[f] - 1) % FAMILY_SIZE[f];
    s->op_idx = FAMILY_FIRST[f] + kind;
    break;
  }

  case '1':
  case '2':
  case '3':
  case '4': {
    /* Switch family while preserving op-kind (UNION/INTERSECT/...).
     * SMOOTH / ROUND / CHAMFER lack XOR (kind = 3) — clamp. */
    int new_f = ch - '1';
    int kind = op_in_family(s->op_idx);
    if (kind >= FAMILY_SIZE[new_f])
      kind = FAMILY_SIZE[new_f] - 1;
    s->op_idx = FAMILY_FIRST[new_f] + kind;
    break;
  }

  case '=':
  case '+':
    s->param += PARAM_STEP;
    if (s->param > PARAM_MAX)
      s->param = PARAM_MAX;
    break;
  case '-':
    s->param -= PARAM_STEP;
    if (s->param < PARAM_MIN)
      s->param = PARAM_MIN;
    break;

  case 'd':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    s->debug_mode =
        (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
    break;

  case 't':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;
  case 'T':
    s->theme_index = (s->theme_index + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;

  case 'z':
    s->cam_dist -= CAM_ZOOM_STEP;
    if (s->cam_dist < CAM_DIST_MIN)
      s->cam_dist = CAM_DIST_MIN;
    break;
  case 'Z':
    s->cam_dist += CAM_ZOOM_STEP;
    if (s->cam_dist > CAM_DIST_MAX)
      s->cam_dist = CAM_DIST_MAX;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

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

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t target_ns = TICK_NS(app->sim_fps);
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(target_ns - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
