/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * raymarcher_primitives.c — a gallery of 17 SDF primitives
 *
 * DEMO: Cycle through 17 distinct geometric primitives — sphere, box,
 *       torus, hex prism, capsule, cone, octahedron, pyramid, and
 *       more — each rendered with the same sphere-tracing skeleton.
 *       What changes per primitive is ONE line: the SDF function
 *       pointer in the active table entry.  Two-axis "tumble"
 *       rotation shows every face/edge/vertex over time.  92-char
 *       Bourke ramp + Floyd-Steinberg dithering for smooth shading.
 *       Six themes, four debug overlays.
 *
 * Study alongside: raymarcher/raymarcher.c (sphere — same march loop
 *       with one primitive) and raymarcher/raymarcher_cube.c (box
 *       SDF deeply explained).  This file is what happens when you
 *       want a CATALOGUE of primitives sharing one renderer.
 *
 * Section map:
 *   §1   config          — every tunable named, no magic numbers later
 *   §2   clock           — monotonic timer + sleep
 *   §3   color           — themes + HUD/hint pairs
 *   §4   vec2 / vec3     — 2-D and 3-D math, value types
 *   §5   rotation        — rot_y, rot_x, tumble (two-axis spin)
 *   §6   SDFs A          — sphere + box family (|p|−h primitives)
 *   §7   SDFs B          — torus family (radial reduction)
 *   §8   SDFs C          — cone + capsule + cylinder (segment-based)
 *   §9   SDFs D          — polyhedra + 2-D extruded slabs
 *   §10  wrappers + prim table (function-pointer dispatch)
 *   §11  raymarch        — trace + tetrahedral normal + Phong + cast_ray → Hit
 *   §12  canvas state    — Hit + intensity + pixels arrays
 *   §13  Bourke ramp + LUT
 *   §14  shade_to_terminal — gamma + Floyd-Steinberg dithering
 *   §15  canvas render   + production canvas_draw
 *   §16  debug overlays  — see the rendering signals raw
 *   §17  scene + screen + app
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume the tumble
 *   Tab / t / T  next primitive (T = previous)
 *   = / -        primitive size larger / smaller
 *   r / R        tumble faster / slower
 *   z / Z        zoom in / out (camera closer / farther)
 *   c / C        next / previous colour theme
 *                  (CLASSIC / AMBER / MATRIX / NEON / ICE / COPPER)
 *   d / D        cycle debug overlay
 *                  (NORMAL / NORMALS / DEPTH / STEPS)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/raymarcher_primitives.c \
 *       -o primitives -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is its own textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         names the algorithm and lists references.
 *   • MENTAL MODEL     intuition + an ASCII diagram of the dispatch.
 *   • GUIDED TUTORIAL  nine short answers walking from "why a function-
 *                      pointer table?" through each shape family,
 *                      tetrahedron normals, gamma correction, and
 *                      Floyd-Steinberg dithering.
 *   • §1..§17          the actual code, each section short and focused.
 *
 * Ten-minute version: read the GUIDED TUTORIAL.  By the end the
 * §-sections feel like reviewing notes.
 *
 * Math notation used in code:
 *      p          — a 3-D point (the SDF input)
 *      s          — primitive size scalar (1.0 = default)
 *      ry, rx     — current Y-spin and X-tilt angles (radians)
 *      ro, rd     — ray origin, ray direction (unit)
 *      t          — distance the ray has marched
 *      N          — surface normal at hit
 *      L, V, R    — light, view, reflection unit vectors
 *
 * Background you need:
 *   • basic vector arithmetic (add, dot, length, normalise)
 *   • read raymarcher.c first if sphere tracing is unfamiliar
 *   • optional: Inigo Quílez's "Distance Functions" page is the
 *     reference for the 17 primitive formulas in §6..§9
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : SPHERE TRACING (Hart 1996) of a SWAPPABLE SDF chosen
 *                from a 17-entry function-pointer table.  Each entry
 *                is a tiny wrapper that maps a single user-controlled
 *                size scalar to that primitive's intrinsic parameters
 *                (half-extents, radii, heights).  The renderer never
 *                knows WHICH primitive is active — it just calls
 *                k_prims[active].sdf(p, size).  Tab cycles the active
 *                index; nothing else has to change.
 *
 *                Beneath the dispatch: each primitive is one of Inigo
 *                Quílez's catalogued distance functions, expressed in
 *                7-25 lines of pure math.
 *
 *                Per pixel:
 *                  1. ray from camera through cell, aspect-corrected
 *                  2. tumble query point by (ry, rx)        (rotation)
 *                  3. sphere-trace using k_prims[i].sdf
 *                  4. on hit: 4-tap tetrahedral normal, Phong shade
 *                  5. (intensity) → glyph + colour pair via Bourke
 *                                  ramp + gamma + Floyd-Steinberg
 *
 * Data         : Stateless math (V2 + V3 + 17 SDFs + 17 wrappers + 1
 *                Prim table).  Per-pixel result is Hit (hit / p /
 *                normal / intensity / t / steps); Canvas stores Hit[]
 *                + intensity[] + pixels[].  Production view applies
 *                gamma + Floyd-Steinberg from intensity[] into
 *                pixels[]; debug overlays read Hit[] directly.
 *
 * Rendering    : One ray per terminal cell.  GLYPH from Paul Bourke's
 *                92-character ASCII ramp (from " " to "@", measured
 *                by ink density).  COLOUR from the active theme's
 *                8-band luma palette (the 92 ramp slots are grouped
 *                into 8 colour bands).  Floyd-Steinberg dithering
 *                hides the ~1% quantisation step between adjacent
 *                ramp slots — smooth gradients stay smooth.
 *
 * Performance  : ~5 SDF evals per hit pixel (1 trace step at hit + 4
 *                tetrahedral normal samples).  Per-march-step: ~20-100
 *                evals depending on grazing.  At 80×24 terminal, ~50K
 *                SDF evals per frame.  Holds 24 fps.  Floyd-Steinberg
 *                pass is one extra full-canvas scan.
 *
 * References   :
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer* 12(10):527-545.  The march loop.
 *   • Quílez, I. — "Distance Functions" (the catalogue):
 *     https://iquilezles.org/articles/distfunctions/
 *     Source for all 17 primitives in §6..§9.
 *   • Floyd & Steinberg (1976) — "An Adaptive Algorithm for Spatial
 *     Greyscale", *Proceedings of the SID* 17(2):75-77.  The
 *     dithering pattern (§14).
 *   • Bourke, P. — "ASCII art" character ramp:
 *     https://paulbourke.net/dataformats/asciiart/
 *     The 92-char ink-density-ordered ramp used in §13.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * One renderer, seventeen geometries.  The renderer doesn't know the
 * difference between a sphere and a hex prism — it asks a question
 * ("how far am I from the surface, given size s?") through a function
 * pointer in a table.  The table entry knows.  Cycling primitives
 * with Tab swaps which row of the table is active; the renderer's
 * machine code is identical from one frame to the next.  The 17
 * entries cover the standard library of analytical SDFs: spheres,
 * boxes, tori, capsules, cones, polyhedra, extruded 2-D shapes.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a museum gallery: one rotating turntable, seventeen
 * sculptures.  An attendant stands in the centre and answers the
 * single question "how far is your sculpture from this point in
 * space?" — they swap which sculpture they describe each time the
 * curator (Tab key) walks by.  The visitor (the ray) doesn't care
 * about the swap; they just keep asking and walking the safe distance
 * the answer gives them.  When the answer is essentially zero, the
 * visitor has touched the surface and the lighting / colour pipeline
 * decides which character + theme colour to paint.
 *
 * Function-pointer dispatch in cross-section:
 *
 *      Tab / t / T
 *           │
 *           ▼
 *      active prim index ─────►  k_prims[i].sdf  ──────►  pure math
 *           │                          │                    (one of
 *           │                          │                     17 SDFs)
 *           │                          │
 *           ▼                          ▼
 *      sphere_trace loop:       returns distance estimate
 *        p = ro + t·rd                    │
 *        d = k_prims[i].sdf(p, s) ◄───────┘
 *        if d < ε:  HIT              (whole inner loop unchanged
 *        if t > MAX: MISS             whether i = SPHERE or HEX_PRISM)
 *        t += d
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per frame:
 *   1. tick           advance Y-spin angle ry by rot_spd · dt
 *                     X-tilt is implicit: rx = ry · ROT_X_RATIO (0.37)
 *                     The irrational ratio gives a quasi-periodic
 *                     orbit that visits every face/edge/vertex.
 *
 * Once per pixel:
 *   2. ray            primary ray from camera through cell, aspect-corr
 *   3. tumble         rotate the SAMPLE POINT (not the cube) — see
 *                     raymarcher_cube.c Tutorial T5 for the trick
 *   4. trace          sphere-march using k_prims[active].sdf
 *   5. on hit:        4-tap tetrahedral normal (see raymarcher_cube T6)
 *                     Phong intensity (KA + KD·N·L + KS·(R·V)^SHIN)
 *
 * Once per cell (production view):
 *   6. gamma          v ← v^(1/2.2)        (linear → perceptual)
 *   7. quantise       v → 1 of 92 ramp indices via LUT
 *   8. dither         distribute quantisation error to 4 neighbours
 *                     (Floyd-Steinberg)
 *   9. emit           glyph = bourke_ramp[idx]
 *                     colour = lumi_attr(idx · LUMI_N / 92)
 *
 * KEY FORMULAS
 * ────────────
 * Sphere SDF (canonical):
 *      f(p) = |p| − r
 *
 * Box SDF (Quílez exact):
 *      q = |p| − b              (component-wise abs and subtract)
 *      d = |max(q, 0)| + min(max(q.x, q.y, q.z), 0)
 *
 * Torus SDF (radial reduction):
 *      r_xz = |(p.x, p.z)| − R    (distance to centre circle)
 *      d    = |(r_xz, p.y)| − r   (distance to tube around it)
 *
 * Capsule SDF (line-segment distance):
 *      h = clamp((p−a)·(b−a) / |b−a|², 0, 1)
 *      d = |(p − a) − (b − a)·h| − r
 *
 * Tetrahedron normal (4-tap finite difference):
 *      k = {(+,−,−), (−,+,−), (−,−,+), (+,+,+)}
 *      N = normalise( Σ kᵢ · sdf(p + ε · kᵢ) )
 *
 * Phong intensity:
 *      I = KA + KD · max(0, N·L) + KS · max(0, R·V)^SHIN
 *      R = 2 · (N·L) · N − L
 *
 * Pixel → ray (cell at column px, row py, canvas cw × ch):
 *      u =  (px + 0.5) / cw · 2 − 1
 *      v = −(py + 0.5) / ch · 2 + 1
 *      rd = normalise(u · F, v · F · aspect, −1)
 *      F  = FOV_HALF_TAN, aspect = (ch · CELL_ASPECT) / cw
 *
 * Tumble (rotation of the query point):
 *      p ← rot_x( rot_y(p, ry), rx )       rx = ry · ROT_X_RATIO
 *
 * Gamma correction:
 *      v_perceptual = v_linear ^ (1 / 2.2)
 *
 * Floyd-Steinberg dithering (4-neighbour error diffusion):
 *      err = v − quantised_value(v)
 *      buf[x+1, y    ] += err · 7/16
 *      buf[x−1, y + 1] += err · 3/16
 *      buf[x  , y + 1] += err · 5/16
 *      buf[x+1, y + 1] += err · 1/16
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Some primitives (cone, pyramid) are NOT Lipschitz-1 globally —
 *     their SDF is a true distance only inside a bounding region,
 *     and far-away rays can step past the surface.  RM_MAX_DIST = 20
 *     catches escaped rays.
 *
 *   • The triangle SDF requires CCW vertex winding: at interior
 *     points the cross products (edge × q-vertex) are POSITIVE.  We
 *     test `> 0` for the inside flag (the original `< 0` test was a
 *     winding bug — fixed earlier — that made the triangle render
 *     hollow).  See sdf_triangle in §9.
 *
 *   • RM_HIT_EPS = 0.002 is in world units.  Tighter ε gives sharper
 *     edges but more steps; RM_MAX_STEPS = 100 caps the budget.  At
 *     grazing angles the marcher can stall — visible "haloing" along
 *     silhouettes.
 *
 *   • Bourke ramp has 92 chars (~1% intensity per slot).  Without
 *     dithering you'd see distinct bands; with FS dithering the bands
 *     dissolve into noise the eye reads as a continuous gradient.
 *
 *   • Plane / Triangle / Quad pre-tilt their query point by ~0.52
 *     rad (~30°) so the tumble shows the FACE rather than just the
 *     edge.  At rot=0 a flat shape would be edge-on and invisible.
 *
 *   • shade_to_terminal allocates a malloc PER FRAME.  On alloc
 *     failure it falls back to direct linear mapping (no dither, no
 *     gamma).  Visually noticeable but the program keeps running.
 *
 *   • The ROT_X_RATIO = 0.37 is irrational on purpose — a rational
 *     ratio (e.g. 0.5) makes the tumble close back on itself
 *     periodically and you'd never see some faces.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press Tab through all 17 primitives without resizing.  Every
 *     shape should re-centre and re-orient automatically; if a shape
 *     drifts off-centre, its wrapper's translation is wrong.
 *
 *   • Pause (space) and watch the Phong highlight: a single bright
 *     spot should sit on the side facing LIGHT = (−3, 3.5, 2.5).
 *     Its position is independent of rotation — only the OBJECT
 *     turns under it.
 *
 *   • +/- doubles/halves apparent size by PRIM_SIZE_STEP = 1.15.
 *     Verify the silhouette grows/shrinks but the centre stays put.
 *
 *   • Set rot_spd to 0 (R repeatedly): image freezes.  No jitter
 *     means rm_normal is stable.  Jitter ⇒ RM_NORM_EPS too small.
 *
 *   • Sphere is the canonical test: at default size the silhouette
 *     should be a perfect ellipse (terminal aspect 2:1) with smooth
 *     Phong fall-off.  Visible banding ⇒ Floyd-Steinberg broken;
 *     flat shading ⇒ rm_normal returning a constant.
 *
 *   • Press d to cycle debug overlays.  NORMALS shows raw geometry
 *     hue (no lighting); DEPTH shows hit-distance brightness; STEPS
 *     shows where the marcher took many iterations (silhouette).
 *
 *   • Press t to cycle themes.  Geometry stays put; only colour
 *     ramp and HUD bg change.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — nine short answers ────────────────────────────── *
 *
 *
 * T1: Why a function-pointer table?
 * ─────────────────────────────────
 * Naïve approach: one big switch in the renderer:
 *
 *      float scene_sdf(V3 p, int prim, float s) {
 *          switch (prim) {
 *          case 0: return sdf_sphere(p, s);
 *          case 1: return sdf_box(p, ...);
 *          ...
 *          }
 *      }
 *
 * Two problems: every renderer touchpoint has the switch, and adding
 * a new primitive means editing the renderer.  The function-pointer
 * approach moves dispatch into a table:
 *
 *      typedef struct { const char *name;
 *                       float (*sdf)(V3 p, float s); } Prim;
 *      static const Prim k_prims[N_PRIMS] = {
 *          { "Sphere", w_sphere },
 *          { "Box",    w_box    },
 *          ...
 *      };
 *
 * Now the renderer just calls `k_prims[active].sdf(p, s)`.  Adding
 * a new primitive: write the SDF, write a wrapper, add ONE row to
 * k_prims.  No renderer change.
 *
 * Cost: one indirect call per SDF evaluation.  At -O2 the compiler
 * can sometimes inline through a constant-table lookup, but here
 * `active` is runtime-variable, so we pay the indirect-call cost.
 * Negligible — the call cost is dwarfed by the SDF body.
 *
 *
 * T2: The two-axis tumble — irrational ratio for full coverage
 * ────────────────────────────────────────────────────────────
 * One axis of rotation reveals only a single ring of orientations.
 * Two perpendicular axes reveal a 2-D family of orientations — but
 * if the speeds are RATIONALLY related (e.g. 1:2), the orbit on the
 * 2-torus closes back on itself periodically and you never visit
 * many points.
 *
 * Solution: make the speed ratio IRRATIONAL.  We use:
 *
 *      ry = scene_time · rot_spd
 *      rx = ry · 0.37       ← irrational ratio (= ROT_X_RATIO)
 *
 * Now the (ry mod 2π, rx mod 2π) trajectory on the 2-torus is a
 * QUASI-PERIODIC ORBIT — never exactly closing — and over time it
 * fills the surface densely.  Wait long enough and every face,
 * edge, and vertex of the primitive faces the camera.
 *
 * The constant 0.37 is arbitrary; any irrational ratio works.  We
 * just want NOT a small-integer ratio.
 *
 *
 * T3: The simplest SDF — sphere
 * ─────────────────────────────
 * For a sphere of radius r centred at origin:
 *
 *      f(p, r) = |p| − r          (one line of math)
 *
 *      |p| > r:  outside, distance to surface = |p| − r
 *      |p| = r:  on the surface
 *      |p| < r:  inside, negative distance
 *
 * That's it.  Lipschitz-1 (true Euclidean distance), so sphere
 * tracing converges in two steps for a head-on ray.  The sphere
 * is the "hello world" of distance functions — every other SDF
 * in this file is more involved.
 *
 *
 * T4: Box SDF — outside + inside terms
 * ────────────────────────────────────
 * Quílez's exact box SDF for a box of half-extents b:
 *
 *      q = |p| − b              (component-wise)
 *      d = |max(q, 0)|      +   min(max(q.x, q.y, q.z), 0)
 *          └ outside ─┘         └ inside (negative inside) ──┘
 *
 * Two regimes:
 *   q has at least one positive component → p is OUTSIDE the box
 *      max(q,0) keeps only the positive parts; |max(q,0)| is the
 *      Euclidean distance to the nearest face/edge/corner.
 *      The inside term is min(positive, 0) = 0 → no contribution.
 *
 *   all q < 0 → p is INSIDE the box
 *      max(q,0) is the zero vector → outside term = 0.
 *      max(q.x, q.y, q.z) is the LEAST-NEGATIVE q (the closest face
 *      from inside); min(negative, 0) = negative → returns a negative
 *      distance.
 *
 * Same code handles both regimes — that's the elegance.  Round box
 * is sdf_box(p, b−r) − r (offset surface).  Box frame is three
 * thin slabs Booleaned together.  Hex prism is a 6-fold symmetric
 * variant.  All in §6.
 *
 *
 * T5: Torus SDF — two-step radial reduction
 * ─────────────────────────────────────────
 * A torus is the surface of revolution of a circle of radius r
 * around a circle of radius R (the centre line of the tube).
 *
 *      r_xz   = |(p.x, p.z)| − R       (distance from p to centre circle)
 *      d      = |(r_xz, p.y)| − r       (distance from p to tube)
 *
 * Step 1: project p onto the XZ plane and measure how far it is
 *         from the centre circle of radius R.  This is the "ring
 *         distance" — positive outside, negative inside the donut
 *         hole region.
 *
 * Step 2: combine that ring distance with the height p.y to get the
 *         distance to the tube of radius r centred on the ring.
 *
 * Capped torus and link extend the same idea with extra clamping
 * (to make the torus an open arc) or extrusion (to make a chain
 * link).  All in §7.
 *
 *
 * T6: Cone, capsule, cylinder — segment-based distances
 * ─────────────────────────────────────────────────────
 * For shapes built around a LINE SEGMENT (capsule), a CIRCULAR FACE
 * (cylinder), or a TILTED LINE (cone), the SDF reduces to "distance
 * from p to the closest point on the segment/cap/edge, minus a
 * radius".
 *
 * Capsule (segment from a to b, radius r):
 *      h = clamp((p − a) · (b − a) / |b − a|², 0, 1)
 *      closest_on_segment = a + (b − a) · h
 *      d = |p − closest_on_segment| − r
 *
 * The clamp ensures the closest point sits within [a, b]; outside
 * that range the closest point is one of the endpoints.  The result
 * is a stadium-like swept-sphere shape.
 *
 * Cylinder = capsule with flat caps (extra max with cap distance).
 * Cone = tilted line capsule with a varying radius along its length.
 * Round cone = cone whose two endpoints have different radii (drips,
 * bullets, etc.).  All in §8.
 *
 *
 * T7: Polyhedra and 2-D extruded slabs
 * ────────────────────────────────────
 * Octahedron, pyramid: SDFs derived from the geometry of the
 * specific polyhedron — a few dot products to project p onto the
 * appropriate face, then a clamped distance.  These are the
 * trickiest closed-form SDFs in the file (octahedron exploits the
 * |x|+|y|+|z| = const symmetry; pyramid uses several halfspace
 * tests with explicit case analysis).
 *
 * Plane (rendered as a finite disc), triangle, quad: take the 2-D
 * shape's SDF in the XZ plane, then extrude it along Y by a
 * thickness:
 *
 *      d_xz = sdf_2d(p.x, p.z)            (distance in the plane)
 *      d_y  = |p.y| − thickness/2
 *      d    = min(max(d_xz, d_y), 0) + |max((d_xz, d_y), 0)|
 *
 * That last formula is the standard extrusion combining 2-D and
 * height SDFs.  All in §9.
 *
 * The triangle SDF has one subtlety: a winding-direction test for
 * "is this point inside the triangle".  Our vertices are wound CCW
 * (top, bottom-left, bottom-right), so the cross-product sign at
 * interior points is POSITIVE — the inside test uses `> 0`, not
 * `< 0`.  Earlier code had the wrong sign and the triangle rendered
 * HOLLOW (only the perimeter edges hit).  Fixed.
 *
 *
 * T8: Tetrahedron normal — 4-tap finite difference
 * ────────────────────────────────────────────────
 * For ANY SDF, the gradient ∇f points OUTWARD from the surface, so
 * N = ∇f / |∇f|.  We approximate ∇f via finite differences.  The
 * cube file (raymarcher_cube.c, Tutorial T6) explained why 4-tap
 * tetrahedral beats 6-tap central differences for sharp edges:
 *
 *      k₀ = ( ε, −ε, −ε)
 *      k₁ = (−ε,  ε, −ε)        (no two share an axis)
 *      k₂ = (−ε, −ε,  ε)
 *      k₃ = ( ε,  ε,  ε)
 *      N  = normalise( Σ kᵢ · sdf(p + kᵢ) )
 *
 * 4 SDF evals per hit pixel.  Normal is sharp at edges (no blur
 * from straddling adjacent faces).
 *
 *
 * T9: Bourke ramp + gamma + Floyd-Steinberg dithering
 * ───────────────────────────────────────────────────
 * Three tricks combine to produce smooth ASCII gradients.
 *
 *   PAUL BOURKE'S 92-CHAR RAMP.  Most SDF demos use 8-13 characters
 *   for shading.  Bourke measured the ACTUAL ink density of every
 *   printable ASCII character and ranked them from sparsest (' ') to
 *   densest ('@'), giving a 92-element ramp.  See the literal ramp
 *   in the k_ramp[] declaration at §13.
 *
 *   With 92 levels we have ~1% intensity quantisation per slot,
 *   instead of ~12.5% with 8 slots.  Much finer gradient.
 *
 *   GAMMA CORRECTION.  Terminal displays are NON-LINEAR.  An RGB
 *   value of 0.5 doesn't render at half brightness — it renders at
 *   roughly 0.5^2.2 ≈ 0.22 brightness (gamma 2.2).  Without
 *   correction, Phong's linear-light intensity values look too DARK
 *   on screen.  Apply v ← v^(1/2.2) before quantising.
 *
 *   FLOYD-STEINBERG DITHERING.  Even at 92 slots, smooth gradients
 *   show subtle banding.  Floyd & Steinberg (1976): when you
 *   QUANTISE a pixel, distribute the ROUNDING ERROR to four
 *   neighbours that haven't been quantised yet:
 *
 *       quantised pixel       raw error e = v_true − v_quant
 *            ●─────────────────►  +e · 7/16
 *           ╱│╲
 *          ╱ │ ╲
 *      e·3/16 e·5/16 e·1/16
 *         ↙   ↓   ↘
 *
 *   The error is "pushed forward" so the LOCAL AVERAGE of any
 *   region's brightness stays correct, even though individual
 *   pixels are quantised.  The eye sees a smooth gradient because
 *   the high-frequency quantisation noise averages out across a
 *   small visual neighbourhood.
 *
 *   All three live in shade_to_terminal (§14).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* End of textbook.  The rest of the file is the worked exercises. */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

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

/* §1.1 frame rate + UI layout. */
enum {
    N_PRIMS         = 17,
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 24,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,
    HUD_COLS        = 56,
    FPS_UPDATE_MS   = 500,
};

/* §1.2 cell aspect (terminal cells are ~2× as tall as wide). */
#define CELL_ASPECT       2.0f

/* §1.3 sphere-trace tunables. */
#define RM_MAX_STEPS      100
#define RM_HIT_EPS        0.002f
#define RM_MAX_DIST       20.0f
#define RM_NORM_EPS       0.001f

/* §1.4 camera (zoom). */
#define CAM_Z_DEFAULT     4.5f
#define CAM_Z_MIN         3.0f       /* keep camera outside SIZE_MAX prim */
#define CAM_Z_MAX        12.0f
#define CAM_ZOOM_STEP     0.30f
#define FOV_HALF_TAN      0.65f

/* §1.5 primitive size. */
#define PRIM_SIZE_DEFAULT 1.0f
#define PRIM_SIZE_STEP    1.15f
#define PRIM_SIZE_MIN     0.2f
#define PRIM_SIZE_MAX     3.0f

/* §1.6 two-axis tumble (T2).
 *      ROT_Y_SPD_DEFAULT  fast Y spin rate (rad/sec)
 *      ROT_X_RATIO        irrational ratio for X tilt (rx = ry · ratio)
 */
#define ROT_Y_SPD_DEFAULT 0.60f
#define ROT_X_RATIO       0.37f
#define ROT_SPD_STEP      1.3f
#define ROT_SPD_MIN       0.0f
#define ROT_SPD_MAX       5.0f

/* §1.7 fixed light position — three-point side key.  Object rotates;
 * the light never moves. */
#define LIGHT_X   -3.0f
#define LIGHT_Y    3.5f
#define LIGHT_Z    2.5f

/* §1.8 Phong shading coefficients. */
#define KA    0.10f
#define KD    0.78f
#define KS    0.55f
#define SHIN  40.0f

/* §1.9 time helpers. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC/(f))

/* §1.10 luma slot count + colour pair IDs.
 *
 * The 92-slot Bourke ramp's output index is grouped down to LUMI_N
 * = 8 colour bands (idx · LUMI_N / 92).  HUD/HINT pairs are theme-
 * independent yellow + cyan per the CLAUDE.md HUD spec.
 */
enum {
    LUMI_N    = 8,
    PAIR_HUD  = LUMI_N + 1,
    PAIR_HINT = LUMI_N + 2,
};

/* §1.11 themes — six 8-band 256-colour ramps.  Per CLAUDE.md every
 * entry sits in the bright half of the 256-cube. */
typedef struct {
    const char *display_name;
    short       ramp_256[LUMI_N];
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    { "CLASSIC ", { 235, 238, 241, 244, 247, 250, 253, 255 } },
    { "AMBER   ", { 130, 136, 166, 172, 178, 208, 214, 220 } },
    { "MATRIX  ", {  28,  34,  40,  46,  82, 118, 154, 190 } },
    { "NEON    ", {  53,  91, 129, 165, 201, 207, 213, 227 } },
    { "ICE     ", {  25,  31,  38,  45,  51,  87, 123, 159 } },
    { "COPPER  ", {  94, 130, 136, 166, 172, 208, 214, 220 } },
};

/* §1.12 debug overlays — d / D cycles between them. */
typedef enum {
    DEBUG_NORMAL    = 0,    /* full lit primitive (production view)   */
    DEBUG_NORMALS   = 1,    /* surface normal direction → colour band */
    DEBUG_DEPTH     = 2,    /* hit distance t → brightness            */
    DEBUG_STEPS     = 3,    /* march iteration count → brightness     */
    DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ", "NORMALS", "DEPTH  ", "STEPS  ",
};

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { (time_t)(ns/NS_PER_SEC), (long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ── §3 color — themes + HUD/hint pairs ──────────────────────────────── */

static void theme_apply(int theme_index)
{
    if (theme_index < 0 || theme_index >= THEME_COUNT) theme_index = 0;
    const Theme *t = &THEMES[theme_index];
    if (COLORS >= 256) {
        for (int i = 0; i < LUMI_N; i++)
            init_pair((short)(i+1), t->ramp_256[i], COLOR_BLACK);
    } else {
        for (int i = 0; i < LUMI_N; i++)
            init_pair((short)(i+1), COLOR_WHITE, COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

static attr_t lumi_attr(int l)
{
    if (l<0) l=0;
    if (l>LUMI_N-1) l=LUMI_N-1;
    attr_t a = COLOR_PAIR(l+1);
    if (COLORS < 256) { if (l<3) a|=A_DIM; else if (l>=6) a|=A_BOLD; }
    return a;
}

/* ── §4 vec2 / vec3 — value-type math ────────────────────────────────── */

typedef struct { float x, y; }    V2;
typedef struct { float x, y, z; } V3;

static inline V2  v2(float x,float y)         { return (V2){x,y}; }
static inline V3  v3(float x,float y,float z) { return (V3){x,y,z}; }

static inline V2  v2add(V2 a,V2 b)   { return v2(a.x+b.x,a.y+b.y); }
static inline V2  v2sub(V2 a,V2 b)   { return v2(a.x-b.x,a.y-b.y); }
static inline V2  v2mul(V2 a,float s){ return v2(a.x*s,a.y*s); }
static inline V2  v2abs(V2 a)        { return v2(fabsf(a.x),fabsf(a.y)); }
static inline V2  v2max0(V2 a)       { return v2(fmaxf(a.x,0),fmaxf(a.y,0)); }
static inline float v2dot(V2 a,V2 b) { return a.x*b.x+a.y*b.y; }
static inline float v2dot2(V2 a)     { return a.x*a.x+a.y*a.y; }
static inline float v2len(V2 a)      { return sqrtf(v2dot2(a)); }

static inline V3  v3add(V3 a,V3 b)   { return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline V3  v3sub(V3 a,V3 b)   { return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline V3  v3mul(V3 a,float s){ return v3(a.x*s,a.y*s,a.z*s); }
static inline V3  v3abs(V3 a)        { return v3(fabsf(a.x),fabsf(a.y),fabsf(a.z)); }
static inline V3  v3max0(V3 a)       { return v3(fmaxf(a.x,0),fmaxf(a.y,0),fmaxf(a.z,0)); }
static inline float v3dot(V3 a,V3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float v3len(V3 a)      { return sqrtf(v3dot(a,a)); }
static inline V3  v3norm(V3 a)       { float l=v3len(a); return l>1e-7f?v3mul(a,1.f/l):v3(0,0,1); }

static inline float clmpf(float v,float lo,float hi){ return fmaxf(lo,fminf(hi,v)); }

/* ── §5 rotation — rot_y, rot_x, tumble (T2) ─────────────────────────── *
 *
 * rot_y / rot_x rotate a 3-D point around Y or X.  tumble applies
 * Y first, then X — the order matters: applying X-then-Y produces
 * different behaviour at large angles.
 */

static inline V3 rot_y(V3 p, float a)
{
    float c=cosf(a), s=sinf(a);
    return v3(c*p.x+s*p.z, p.y, -s*p.x+c*p.z);
}
static inline V3 rot_x(V3 p, float a)
{
    float c=cosf(a), s=sinf(a);
    return v3(p.x, c*p.y-s*p.z, s*p.y+c*p.z);
}

/* tumble — apply Y rotation then X rotation to the SAMPLE POINT.
 * Equivalent to spinning the primitive in the OPPOSITE direction
 * (see raymarcher_cube.c Tutorial T5 for the "rotate the question,
 * not the answer" framing). */
static inline V3 tumble(V3 p, float ry, float rx)
{
    return rot_x(rot_y(p, ry), rx);
}

/* ── §6 SDFs A — sphere + box family (T3, T4) ────────────────────────── *
 *
 * Five primitives built around |p| − something:
 *   sphere       canonical |p| − r
 *   box          Quílez exact box (outside + inside terms)
 *   round box    box with offset surface (subtract r at the end)
 *   box frame    three thin slabs Booleaned together
 *   hex prism    six-fold symmetric cylinder
 */

/* 1. Sphere of radius r at the origin. */
static float sdf_sphere(V3 p, float r)
{
    return v3len(p) - r;
}

/* 2. Box of half-extents b.  T4 derived this. */
static float sdf_box(V3 p, V3 b)
{
    V3 q = v3sub(v3abs(p), b);
    return v3len(v3max0(q)) + fminf(fmaxf(q.x,fmaxf(q.y,q.z)), 0.f);
}

/* 3. Round Box — box(p, b) − r offsets the surface outward. */
static float sdf_round_box(V3 p, V3 b, float r)
{
    V3 q = v3sub(v3abs(p), b);
    return v3len(v3max0(q)) + fminf(fmaxf(q.x,fmaxf(q.y,q.z)), 0.f) - r;
}

/* 4. Box Frame — three thin slabs (one per principal axis pair). */
static float sdf_box_frame(V3 p, V3 b, float e)
{
    V3 pa = v3sub(v3abs(p), b);
    V3 q  = v3sub(v3abs(v3add(pa,v3(e,e,e))), v3(e,e,e));
    float d0=v3len(v3max0(v3(pa.x,q.y,q.z)))+fminf(fmaxf(pa.x,fmaxf(q.y,q.z)),0.f);
    float d1=v3len(v3max0(v3(q.x,pa.y,q.z)))+fminf(fmaxf(q.x,fmaxf(pa.y,q.z)),0.f);
    float d2=v3len(v3max0(v3(q.x,q.y,pa.z)))+fminf(fmaxf(q.x,fmaxf(q.y,pa.z)),0.f);
    return fminf(fminf(d0,d1),d2);
}

/* 10. Hex Prism — hexagonal cross-section, height ry. */
static float sdf_hex_prism(V3 p, float rx, float ry)
{
    const float kx=-0.8660254f, ky=0.5f, kz=0.57735f;
    V3 q=v3abs(p);
    float d=kx*q.x+ky*q.y; if(d<0.f){q.x-=2.f*d*kx;q.y-=2.f*d*ky;}
    float cx=clmpf(q.x,-kz*rx,kz*rx);
    V2 dv=v2(v2len(v2sub(v2(q.x,q.y),v2(cx,rx)))*(q.y<rx?-1.f:1.f), q.z-ry);
    return fminf(fmaxf(dv.x,dv.y),0.f)+v2len(v2max0(dv));
}

/* ── §7 SDFs B — torus family (T5, radial reduction) ─────────────────── *
 *
 * Three primitives built by reducing 3-D distance to 2-D distance
 * via the radial-projection trick:
 *   torus            full revolution of a circle
 *   capped torus     partial arc (controlled by sx, cx of half-angle)
 *   link             torus extruded into a chain link
 */

/* 5. Torus in XZ plane, ring radius R, tube radius r.  T5 derived this. */
static float sdf_torus(V3 p, float R, float r)
{
    return v2len(v2(v2len(v2(p.x,p.z))-R, p.y)) - r;
}

/*
 * 6. Capped Torus — partial arc.  The (sx, cx) pair encodes
 * (sin, cos) of the arc's half-angle.  Folds p.x into +X so the
 * arc is symmetric, then uses a piecewise distance.
 */
static float sdf_capped_torus(V3 p, float ra, float rb, float sx, float cx)
{
    p.x = fabsf(p.x);
    float k = (cx*p.x > sx*p.y) ? v2dot(v2(p.x,p.y),v2(cx,sx))
                                  : sqrtf(p.x*p.x+p.y*p.y);
    return sqrtf(v3dot(p,p)+ra*ra-2.f*ra*k) - rb;
}

/*
 * 7. Link — chain link.  le = half-length of straight section;
 * r1 = link radius (the loop part); r2 = tube radius.
 */
static float sdf_link(V3 p, float le, float r1, float r2)
{
    V3 q = v3(p.x, fmaxf(fabsf(p.y)-le, 0.f), p.z);
    return v2len(v2(v2len(v2(q.x,q.y))-r1, q.z)) - r2;
}

/* ── §8 SDFs C — cone + capsule + cylinder (T6, segment-based) ───────── *
 *
 * Four primitives whose distance reduces to "closest point on a line
 * segment" plus a radius:
 *   cone           tilted line with varying radius along length
 *   plane (disc)   thin extruded disc
 *   capsule        swept sphere along a segment
 *   cylinder       capsule with flat caps
 *   round cone     cone with two different endpoint radii
 */

/*
 * 8. Cone — apex at y=0, base at y=−h.  (si, co) = (sin, cos) of
 * the half-angle.  IQ exact formula (verbatim from the GLSL source).
 */
static float sdf_cone(V3 p, float si, float co, float h)
{
    V2 q = v2(h*si/co, -h);
    V2 w = v2(v2len(v2(p.x,p.z)), p.y);
    V2 a = v2sub(w, v2mul(q, clmpf(v2dot(w,q)/v2dot(q,q), 0.f, 1.f)));
    V2 b = v2sub(w, v2(clmpf(w.x/q.x, 0.f, 1.f)*q.x, q.y));
    float k = (q.y < 0.f) ? -1.f : 1.f;
    float d = fminf(v2dot(a,a), v2dot(b,b));
    float s = fmaxf(k*(w.x*q.y - w.y*q.x), k*(w.y - q.y));
    return sqrtf(d) * (s >= 0.f ? 1.f : -1.f);
}

/*
 * 9. Plane (rendered as a thin disc so it has visible silhouette).
 * Disc of radius R, thickness t, in the XZ plane.
 */
static float sdf_plane_disc(V3 p, float R, float t)
{
    float r2d = v2len(v2(p.x,p.z)) - R;
    float ry  = fabsf(p.y) - t;
    return fminf(fmaxf(r2d,ry), 0.f) + v2len(v2max0(v2(r2d,ry)));
}

/* 11. Capsule — swept sphere from a to b, radius r.  T6 derived. */
static float sdf_capsule(V3 p, V3 a, V3 b, float r)
{
    V3 pa=v3sub(p,a), ba=v3sub(b,a);
    float h=clmpf(v3dot(pa,ba)/v3dot(ba,ba),0.f,1.f);
    return v3len(v3sub(pa,v3mul(ba,h))) - r;
}

/* 12. Cylinder — vertical, capped.  Half-height h, radius r. */
static float sdf_cylinder(V3 p, float h, float r)
{
    V2 d=v2sub(v2abs(v2(v2len(v2(p.x,p.z)),p.y)), v2(r,h));
    return fminf(fmaxf(d.x,d.y),0.f) + v2len(v2max0(d));
}

/*
 * 13. Round Cone — bottom radius r1, top radius r2, height h.
 * Three regions: spherical bottom cap, conical sides, spherical top.
 */
static float sdf_round_cone(V3 p, float r1, float r2, float h)
{
    V2 q=v2(v2len(v2(p.x,p.z)),p.y);
    float b=(r1-r2)/h, a=sqrtf(1.f-b*b), k=v2dot(q,v2(-b,a));
    if(k<0.f) return v2len(q)-r1;
    if(k>a*h) return v2len(v2sub(q,v2(0.f,h)))-r2;
    return v2dot(q,v2(a,b))-r1;
}

/* ── §9 SDFs D — polyhedra + 2-D extruded slabs (T7) ─────────────────── *
 *
 * Five primitives built from explicit polyhedron geometry or from
 * 2-D shapes extruded along Y:
 *   octahedron     |x|+|y|+|z| symmetry
 *   pyramid        explicit halfspace tests + face projection
 *   triangle       2-D equilateral triangle extruded (slab)
 *   quad           2-D rectangle extruded (slab)
 */

/*
 * 14. Octahedron — exploits the L¹ norm |x|+|y|+|z| = const.
 * Three cases by which face is closest, plus a degenerate centre case.
 */
static float sdf_octahedron(V3 p, float s)
{
    V3 q=v3abs(p);
    float m=q.x+q.y+q.z-s;
    V3 r;
    if      (3.f*q.x<m) r=q;
    else if (3.f*q.y<m) r=v3(q.y,q.z,q.x);
    else if (3.f*q.z<m) r=v3(q.z,q.x,q.y);
    else                return m*0.57735027f;
    float k=clmpf(0.5f*(r.z-r.y+s),0.f,s);
    return v3len(v3(r.x,r.y-s+k,r.z-k));
}

/* 15. Pyramid — apex at top, square base.  Several halfspace tests. */
static float sdf_pyramid(V3 p, float h)
{
    float m2=h*h+0.25f;
    p.x=fabsf(p.x); p.z=fabsf(p.z);
    if(p.z>p.x){float t=p.x;p.x=p.z;p.z=t;}
    p.x-=0.5f; p.z-=0.5f;
    V3 q=v3(p.z,h*p.y-0.5f*p.x,h*p.x+0.5f*p.y);
    float ss=fmaxf(-q.x,0.f);
    float t=clmpf((q.y-0.5f*p.z)/(m2+0.25f),0.f,1.f);
    float a=m2*(q.x+ss)*(q.x+ss)+q.y*q.y;
    float b=m2*(q.x+0.5f*t)*(q.x+0.5f*t)+(q.y-m2*t)*(q.y-m2*t);
    float d2=(fminf(q.y,-q.x*m2-q.y*0.5f)>0.f)?0.f:fminf(a,b);
    return sqrtf((d2+q.z*q.z)/m2)*(fmaxf(q.z,-p.y)>=0.f?1.f:-1.f);
}

/*
 * 16. Triangle slab — equilateral 2-D triangle extruded along Y.
 *
 * Vertices are wound CCW in (x, z): top, bottom-left, bottom-right.
 * For CCW winding the cross product (edge × q-vertex) is POSITIVE
 * at interior points — the inside test below uses `> 0`.  An older
 * version used `< 0`, which made the triangle render hollow (only
 * the perimeter edges hit because the interior was reported as
 * positive distance).  T7 explained.
 */
static float sdf_triangle(V3 p, float sz, float thick)
{
    V2 a=v2( 0.f,         sz);
    V2 b=v2(-sz*0.866f,  -sz*0.5f);
    V2 c=v2( sz*0.866f,  -sz*0.5f);
    V2 q=v2(p.x,p.z);
    V2 ab=v2sub(b,a),bc=v2sub(c,b),ca=v2sub(a,c);
    V2 qa=v2sub(q,a),qb=v2sub(q,b),qc=v2sub(q,c);
    float d2=fminf(fminf(
        v2dot2(v2sub(qa,v2mul(ab,clmpf(v2dot(qa,ab)/v2dot2(ab),0.f,1.f)))),
        v2dot2(v2sub(qb,v2mul(bc,clmpf(v2dot(qb,bc)/v2dot2(bc),0.f,1.f))))),
        v2dot2(v2sub(qc,v2mul(ca,clmpf(v2dot(qc,ca)/v2dot2(ca),0.f,1.f)))));
    int inside = (ab.x*qa.y-ab.y*qa.x>0.f &&
                  bc.x*qb.y-bc.y*qb.x>0.f &&
                  ca.x*qc.y-ca.y*qc.x>0.f);
    float d_xz = (inside ? -1.f : 1.f) * sqrtf(d2);
    float d_y  = fabsf(p.y) - thick;
    return fminf(fmaxf(d_xz,d_y),0.f) + v2len(v2max0(v2(d_xz,d_y)));
}

/* 17. Quad slab — axis-aligned rectangle extruded along Y. */
static float sdf_quad(V3 p, float wx, float wz, float thick)
{
    V2 q2=v2sub(v2abs(v2(p.x,p.z)), v2(wx,wz));
    float d_xz=v2len(v2max0(q2))+fminf(fmaxf(q2.x,q2.y),0.f);
    float d_y =fabsf(p.y)-thick;
    return fminf(fmaxf(d_xz,d_y),0.f)+v2len(v2max0(v2(d_xz,d_y)));
}

/* ── §10 wrappers + prim table (T1 — function-pointer dispatch) ──────── *
 *
 * Each wrapper maps the user-controlled `s` (size) to that
 * primitive's intrinsic parameters.  Some wrappers also pre-tilt
 * the query point so flat shapes (plane, triangle, quad) face the
 * camera at rotation 0 — otherwise they'd be edge-on and invisible.
 *
 * The Prim struct + k_prims[] table is the "switch statement" of
 * the renderer, but as data instead of code.  Tab cycles the
 * active index.
 */

typedef struct {
    const char *name;
    float (*sdf)(V3 p, float s);
} Prim;

static float w_sphere    (V3 p,float s){ return sdf_sphere(p,s); }
static float w_box       (V3 p,float s){ return sdf_box(p,v3(s*.70f,s*.70f,s*.70f)); }
static float w_round_box (V3 p,float s){ return sdf_round_box(p,v3(s*.55f,s*.55f,s*.55f),s*.15f); }
static float w_box_frame (V3 p,float s){ return sdf_box_frame(p,v3(s*.65f,s*.65f,s*.65f),s*.07f); }

/* Torus: rotate 90° around X so the ring faces the camera. */
static float w_torus(V3 p,float s)
{
    return sdf_torus(rot_x(p,1.5708f), s*.62f, s*.21f);
}

/* Capped torus: same tilt, ~270° arc. */
static float w_cap_torus(V3 p,float s)
{
    return sdf_capped_torus(rot_x(p,1.5708f), s*.62f, s*.17f, sinf(2.4f), cosf(2.4f));
}

/* Link: stands vertical; two rounded ends visible from front. */
static float w_link(V3 p,float s)
{
    return sdf_link(p, s*.32f, s*.26f, s*.11f);
}

/* Cone: apex at top, base at bottom, centred at origin.
 * sdf_cone returns apex-at-y=0; we shift up by h/2 so midpoint = origin. */
static float w_cone(V3 p, float s)
{
    float h = s * 0.7f;
    V3 pp = v3(p.x, p.y + h*0.5f, p.z);
    return sdf_cone(pp, sinf(0.48f), cosf(0.48f), h);
}

/* Plane: thin horizontal disc, tilted ~30° forward to show the face. */
static float w_plane(V3 p,float s)
{
    V3 q=rot_x(p, 0.52f);
    return sdf_plane_disc(q, s*.85f, s*.04f);
}

/* Hex prism: rotate so a hexagonal face points at the camera. */
static float w_hex_prism(V3 p,float s)
{
    return sdf_hex_prism(rot_x(p,1.5708f), s*.55f, s*.35f);
}

/* Capsule: vertical, endpoints above and below origin. */
static float w_capsule(V3 p,float s)
{
    return sdf_capsule(p, v3(0,-s*.52f,0), v3(0,s*.52f,0), s*.27f);
}

static float w_cylinder(V3 p,float s)
{
    return sdf_cylinder(p, s*.58f, s*.36f);
}

/* Round cone: wider base at bottom, narrower top (drop / drip shape). */
static float w_round_cone(V3 p,float s)
{
    V3 pp=v3(p.x, p.y+s*.38f, p.z);
    return sdf_round_cone(pp, s*.34f, s*.11f, s*.76f);
}

static float w_octahedron(V3 p,float s){ return sdf_octahedron(p, s*.82f); }

/* Pyramid: apex up, base shifted down so centred at origin. */
static float w_pyramid(V3 p,float s)
{
    float h=s*1.0f;
    V3 pp=v3(p.x, p.y+h*.33f, p.z);
    return sdf_pyramid(pp, h);
}

/* Triangle slab: tilt ~30° so face AND edge visible during tumble. */
static float w_triangle(V3 p,float s)
{
    V3 q=rot_x(p, 0.52f);
    return sdf_triangle(q, s*.80f, s*.055f);
}

/* Quad slab: same tilt as triangle. */
static float w_quad(V3 p,float s)
{
    V3 q=rot_x(p, 0.52f);
    return sdf_quad(q, s*.72f, s*.52f, s*.05f);
}

static const Prim k_prims[N_PRIMS] = {
    { "Sphere",       w_sphere     },   /*  1 */
    { "Box",          w_box        },   /*  2 */
    { "Round Box",    w_round_box  },   /*  3 */
    { "Box Frame",    w_box_frame  },   /*  4 */
    { "Torus",        w_torus      },   /*  5 */
    { "Capped Torus", w_cap_torus  },   /*  6 */
    { "Link",         w_link       },   /*  7 */
    { "Cone",         w_cone       },   /*  8 */
    { "Plane",        w_plane      },   /*  9 */
    { "Hex Prism",    w_hex_prism  },   /* 10 */
    { "Capsule",      w_capsule    },   /* 11 */
    { "Cylinder",     w_cylinder   },   /* 12 */
    { "Round Cone",   w_round_cone },   /* 13 */
    { "Octahedron",   w_octahedron },   /* 14 */
    { "Pyramid",      w_pyramid    },   /* 15 */
    { "Triangle",     w_triangle   },   /* 16 */
    { "Quad",         w_quad       },   /* 17 */
};

/* ── §11 raymarch — trace + tetrahedral normal + Phong + cast_ray ────── *
 *
 * Same march loop as raymarcher.c but with k_prims[prim].sdf instead
 * of a hardcoded SDF.  The Hit struct collects per-pixel result so
 * production view + 3 debug overlays can share one render pass.
 */

typedef struct {
    bool  hit;
    V3    p;            /* hit position (world coords)    */
    V3    normal;       /* unit surface normal at hit     */
    float intensity;    /* Phong result in [0, 1]         */
    float t;            /* ray parameter at hit           */
    int   steps;        /* march iterations (AO proxy)    */
} Hit;

/*
 * rm_march — Hart 1996 march.  The query point gets tumbled by the
 * current rotation before each SDF evaluation (T2).  Returns t at
 * hit, -1 on miss; *out_steps gets the iteration count.
 */
static float rm_march(V3 ro, V3 rd, int prim, float s, float ry, int *out_steps)
{
    float rx = ry * ROT_X_RATIO;
    float t  = 0.05f;
    int   step;
    for (step = 0; step < RM_MAX_STEPS; step++) {
        V3    p = tumble(v3add(ro, v3mul(rd, t)), ry, rx);
        float d = k_prims[prim].sdf(p, s);
        if (d < RM_HIT_EPS) {
            if (out_steps) *out_steps = step + 1;
            return t;
        }
        if (t > RM_MAX_DIST) break;
        t += d;
    }
    if (out_steps) *out_steps = step;
    return -1.f;
}

/*
 * rm_normal — 4-tap tetrahedral normal estimation (T8).  Each of
 * the 4 sample directions has no axis shared with another, so the
 * normal stays sharp at edges.  Note: pos is in WORLD space; we
 * tumble each sample point before evaluating the SDF.
 */
static V3 rm_normal(V3 pos, int prim, float s, float ry)
{
    float rx = ry * ROT_X_RATIO;
    const float e = RM_NORM_EPS;
    V3 n = v3(0,0,0);
    for (int i = 0; i < 4; i++) {
        float bx=(float)(((i+3)>>1)&1),by=(float)((i>>1)&1),bz=(float)(i&1);
        V3 ev = v3mul(v3(2.f*bx-1.f,2.f*by-1.f,2.f*bz-1.f), 0.5773f);
        V3 sp = tumble(v3add(pos, v3mul(ev, e)), ry, rx);
        n = v3add(n, v3mul(ev, k_prims[prim].sdf(sp, s)));
    }
    return v3norm(n);
}

/*
 * rm_shade — Phong: ambient + diffuse + specular.  Same formula as
 * the other raymarcher files.
 */
static float rm_shade(V3 N, V3 hit, V3 cam, V3 light)
{
    V3    L   = v3norm(v3sub(light, hit));
    V3    V   = v3norm(v3sub(cam,   hit));
    float ndl = fmaxf(0.f, v3dot(N, L));
    V3    R   = v3sub(v3mul(N, 2.f * ndl), L);
    float sp  = powf(fmaxf(0.f, v3dot(R, V)), SHIN);
    float I   = KA + KD * ndl + KS * sp;
    return I > 1.f ? 1.f : I;
}

/*
 * cast_ray — full per-pixel pipeline returning Hit.
 *
 *   u = (px + 0.5) / cw · 2 − 1         in [−1, +1]
 *   v = −(py + 0.5) / ch · 2 + 1        Y-flip
 *   rd = normalise(u·F, v·F·aspect, −1) F = FOV_HALF_TAN
 *   t  = rm_march(ro, rd, prim, s, ry, &steps)
 *   if t < 0:  return Hit{ hit=false, steps }
 *   hit_p = ro + t · rd
 *   N     = rm_normal(hit_p, prim, s, ry)
 *   I     = rm_shade(N, hit_p, ro, light)
 *   return Hit{ true, hit_p, N, I, t, steps }
 *
 * The aspect-correction factor on v compensates for terminal cells
 * being ~2× as tall as they are wide; without it primitives would
 * render as vertical ellipses.
 */
static Hit cast_ray(int px, int py, int cw, int ch,
                    int prim, float s, float ry, V3 light, float cam_z)
{
    Hit h = { false, {0,0,0}, {0,0,1}, 0.f, 0.f, 0 };

    float u =  ((float)px + 0.5f) / (float)cw * 2.f - 1.f;
    float v = -((float)py + 0.5f) / (float)ch * 2.f + 1.f;
    float pa = ((float)ch * CELL_ASPECT) / (float)cw;

    V3 ro = v3(0.f, 0.f, cam_z);
    V3 rd = v3norm(v3(u * FOV_HALF_TAN, v * FOV_HALF_TAN * pa, -1.f));

    int   steps = 0;
    float t = rm_march(ro, rd, prim, s, ry, &steps);
    h.steps = steps;
    if (t < 0.f) return h;

    h.hit       = true;
    h.t         = t;
    h.p         = v3add(ro, v3mul(rd, t));
    h.normal    = rm_normal(h.p, prim, s, ry);
    h.intensity = rm_shade(h.normal, h.p, ro, light);
    return h;
}

/* ── §12 canvas state — Hit + intensity + pixels arrays ──────────────── *
 *
 * Three parallel arrays:
 *
 *   hits[]       Hit per pixel.  Production view reads .intensity to
 *                fill intensity[]; debug overlays read everything else.
 *
 *   intensity[]  Phong float per pixel (or INTENSITY_MISS).  Input
 *                to shade_to_terminal (gamma + LUT + Floyd-Steinberg).
 *
 *   pixels[]     Bourke ramp index per pixel (or CANVAS_MISS).  Output
 *                of shade_to_terminal; canvas_draw paints from this.
 */

#define CANVAS_MISS     -1
#define INTENSITY_MISS  -1.0f

typedef struct {
    int    w, h;
    Hit   *hits;        /* [h*w] per-pixel cast_ray result        */
    float *intensity;   /* [h*w] raw Phong or INTENSITY_MISS      */
    int   *pixels;      /* [h*w] ramp index or CANVAS_MISS        */
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows)
{
    c->w = cols; c->h = rows;
    c->hits      = calloc((size_t)(cols*rows), sizeof(Hit));
    c->intensity = calloc((size_t)(cols*rows), sizeof(float));
    c->pixels    = calloc((size_t)(cols*rows), sizeof(int));
}

static void canvas_free(Canvas *c)
{
    free(c->hits);      free(c->intensity);  free(c->pixels);
    c->hits = NULL;     c->intensity = NULL; c->pixels = NULL;
    c->w = c->h = 0;
}

/* ── §13 Bourke ramp + LUT ───────────────────────────────────────────── *
 *
 * Paul Bourke's 92-character ASCII ramp, ordered from least to most
 * ink density.  The LUT (`k_lut_breaks[]`) is a 92-entry table of
 * thresholds in gamma-corrected space; lut_index() finds which slot
 * a value falls into; lut_value() returns the slot's midpoint (used
 * by Floyd-Steinberg for error calculation).
 *
 * Because the Bourke ramp was already MEASURED to have approximately
 * uniform density steps, our LUT is just evenly-spaced thresholds.
 * The architecture preserves the option to use a non-uniform LUT
 * later (e.g. perceptually-tuned) without touching the dithering.
 */

static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_N (int)(sizeof k_ramp - 1)   /* 92 */

static float k_lut_breaks[RAMP_N];        /* filled once at startup */

static void lut_init(void)
{
    for (int i = 0; i < RAMP_N; i++)
        k_lut_breaks[i] = (float)i / (float)(RAMP_N - 1);
}

/* lut_index — gamma-corrected float [0, 1] → ramp slot 0..RAMP_N−1. */
static inline int lut_index(float v)
{
    int idx = RAMP_N - 1;
    for (int i = 0; i < RAMP_N - 1; i++) {
        if (v < k_lut_breaks[i+1]) { idx = i; break; }
    }
    return idx;
}

/* lut_value — ramp slot → midpoint float (for Floyd-Steinberg error). */
static inline float lut_value(int idx)
{
    if (idx <= 0)         return 0.f;
    if (idx >= RAMP_N-1)  return 1.f;
    return (k_lut_breaks[idx] + k_lut_breaks[idx+1]) * 0.5f;
}

/* ── §14 shade_to_terminal — gamma + Floyd-Steinberg dithering (T9) ──── *
 *
 * Three corrections in sequence, fixing one class of artefact each:
 *
 *   STEP 1 — GAMMA.  Phong returns LINEAR light; the terminal
 *            applies gamma ~2.2.  Apply v ← v^(1/2.2) so the
 *            displayed brightness matches the mathematical intent.
 *
 *   STEP 2 — LUT.  Map gamma-corrected float to one of 92 ramp
 *            slots via the threshold table from §13.
 *
 *   STEP 3 — FLOYD-STEINBERG.  At each pixel, compute the
 *            quantisation error (true value minus quantised
 *            midpoint) and distribute it to 4 neighbours that
 *            haven't been quantised yet.  T9 explained the pattern:
 *
 *               quantised pixel ●
 *                              ╱│╲
 *                             7/16 (right)
 *                          3/16  5/16  1/16
 *                          ↙     ↓     ↘
 *
 * On malloc failure the function falls back to direct linear
 * mapping (no gamma, no dither).  Visible regression but the
 * program keeps running.
 */
static void shade_to_terminal(Canvas *c)
{
    int    w = c->w, h = c->h;
    int    n = w * h;
    float *buf = malloc((size_t)n * sizeof(float));
    if (!buf) {
        /* fallback: direct linear mapping */
        for (int i = 0; i < n; i++) {
            float v = c->intensity[i];
            if (v < 0.f) { c->pixels[i] = CANVAS_MISS; continue; }
            int idx = (int)(v*(float)(RAMP_N-1)+0.5f);
            c->pixels[i] = idx < RAMP_N ? idx : RAMP_N-1;
        }
        return;
    }

    /* Step 1+2: gamma correction + carry into working buffer. */
    for (int i = 0; i < n; i++) {
        float v = c->intensity[i];
        if (v < 0.f) { buf[i] = INTENSITY_MISS; continue; }
        v = fmaxf(0.f, fminf(1.f, v));
        v = powf(v, 1.f/2.2f);
        buf[i] = v;
    }

    /* Step 3: Floyd-Steinberg error diffusion. */
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int   i = y*w + x;
            float v = buf[i];

            if (v < INTENSITY_MISS + 0.5f) {
                c->pixels[i] = CANVAS_MISS;
                continue;
            }

            int   idx = lut_index(v);
            c->pixels[i] = idx;

            float qv  = lut_value(idx);
            float err = v - qv;

            if (x+1 < w && buf[i+1] > INTENSITY_MISS + 0.5f)
                buf[i+1]   += err * (7.f/16.f);
            if (y+1 < h) {
                if (x-1 >= 0 && buf[i+w-1] > INTENSITY_MISS + 0.5f)
                    buf[i+w-1] += err * (3.f/16.f);
                if (buf[i+w] > INTENSITY_MISS + 0.5f)
                    buf[i+w]   += err * (5.f/16.f);
                if (x+1 < w && buf[i+w+1] > INTENSITY_MISS + 0.5f)
                    buf[i+w+1] += err * (1.f/16.f);
            }
        }
    }

    free(buf);
}

/* ── §15 canvas render + production canvas_draw ──────────────────────── *
 *
 * Render: walk every pixel, call cast_ray, fill hits[] AND
 * intensity[] (production view's input).  Then run shade_to_terminal
 * to populate pixels[] for canvas_draw.
 *
 * Production canvas_draw paints from pixels[] using the Bourke ramp.
 * Debug overlays (§16) read hits[] directly without going through
 * shade_to_terminal.
 */
static void canvas_render(Canvas *c, int prim, float s, float ry, V3 light,
                          float cam_z)
{
    /* Phase 1: cast each ray, store full Hit + intensity. */
    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            int idx = py * c->w + px;
            Hit h = cast_ray(px, py, c->w, c->h, prim, s, ry, light, cam_z);
            c->hits     [idx] = h;
            c->intensity[idx] = h.hit ? h.intensity : -1.f;
        }
    }
    /* Phase 2: gamma + LUT + Floyd-Steinberg → pixels[]. */
    shade_to_terminal(c);
}

/* canvas_draw — production view (Bourke glyph + theme colour). */
static void canvas_draw(const Canvas *c, int tcols, int trows)
{
    int ox=(tcols-c->w)/2, oy=(trows-c->h)/2;
    for (int y=0; y<c->h; y++) {
        for (int x=0; x<c->w; x++) {
            int idx=c->pixels[y*c->w+x];
            if (idx==CANVAS_MISS) continue;
            int tx=ox+x, ty=oy+y;
            if (tx<0||tx>=tcols||ty<0||ty>=trows) continue;
            char   ch   = k_ramp[idx];
            attr_t attr = lumi_attr((idx*LUMI_N)/RAMP_N);
            attron(attr);
            mvaddch(ty,tx,(chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── §16 debug overlays — see the rendering signals raw ──────────────── *
 *
 * Three educational visualisations, each isolating ONE field of the
 * Hit struct.  Debug overlays use a simple 13-char glyph ramp
 * (skipping Floyd-Steinberg) so the visualisation isn't blurred by
 * dithering.
 */

static const char DEBUG_GLYPHS[] = " .,:;+*oxOX#@";
#define DEBUG_GLYPHS_N ((int)(sizeof DEBUG_GLYPHS - 1))

static char debug_glyph(float v)
{
    int idx = (int)(v * (float)(DEBUG_GLYPHS_N - 1) + 0.5f);
    if (idx < 0)              idx = 0;
    if (idx >= DEBUG_GLYPHS_N) idx = DEBUG_GLYPHS_N - 1;
    return DEBUG_GLYPHS[idx];
}

static int debug_slot(float v)
{
    int idx = (int)(v * (float)(DEBUG_GLYPHS_N - 1) + 0.5f);
    int slot = (idx * LUMI_N) / DEBUG_GLYPHS_N;
    return slot;
}

/*
 * canvas_draw_normals — surface-normal direction → colour band.
 *      hue   = atan2(N.x, N.z) / 2π + 0.5   (azimuth around y-axis)
 *      glyph = N.y · 0.5 + 0.5              (up-facing → bright)
 */
static void canvas_draw_normals(const Canvas *c, int tcols, int trows)
{
    int ox=(tcols-c->w)/2, oy=(trows-c->h)/2;
    for (int y=0; y<c->h; y++) {
        for (int x=0; x<c->w; x++) {
            const Hit *h = &c->hits[y*c->w+x];
            if (!h->hit) continue;
            int tx=ox+x, ty=oy+y;
            if (tx<0||tx>=tcols||ty<0||ty>=trows) continue;

            V3    N       = h->normal;
            float azimuth = atan2f(N.x, N.z) / (2.f * (float)M_PI) + 0.5f;
            float y_lit   = N.y * 0.5f + 0.5f;
            if (y_lit < 0.f) y_lit = 0.f;
            if (y_lit > 1.f) y_lit = 1.f;

            attr_t attr = lumi_attr(debug_slot(azimuth));   /* hue from azimuth */
            attron(attr);
            mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(y_lit));
            attroff(attr);
        }
    }
}

/*
 * canvas_draw_depth — hit distance t → glyph + colour.  Closer hits
 * get higher depth_n.  Range bracketed by camera position and the
 * primitive's bounding sphere.
 */
static void canvas_draw_depth(const Canvas *c, int tcols, int trows,
                              float cam_z)
{
    int ox=(tcols-c->w)/2, oy=(trows-c->h)/2;
    float t_min = cam_z - PRIM_SIZE_MAX;
    float t_max = cam_z + PRIM_SIZE_MAX;
    if (t_min < 0.f) t_min = 0.f;

    for (int y=0; y<c->h; y++) {
        for (int x=0; x<c->w; x++) {
            const Hit *h = &c->hits[y*c->w+x];
            if (!h->hit) continue;
            int tx=ox+x, ty=oy+y;
            if (tx<0||tx>=tcols||ty<0||ty>=trows) continue;

            float depth_n = (t_max - h->t) / (t_max - t_min);
            if (depth_n < 0.f) depth_n = 0.f;
            if (depth_n > 1.f) depth_n = 1.f;

            attr_t attr = lumi_attr(debug_slot(depth_n));
            attron(attr);
            mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(depth_n));
            attroff(attr);
        }
    }
}

/*
 * canvas_draw_steps — march iteration count → glyph + colour.
 * Grazing silhouette rays take many steps to converge; this overlay
 * makes the silhouette glow.
 */
static void canvas_draw_steps(const Canvas *c, int tcols, int trows)
{
    int ox=(tcols-c->w)/2, oy=(trows-c->h)/2;
    for (int y=0; y<c->h; y++) {
        for (int x=0; x<c->w; x++) {
            const Hit *h = &c->hits[y*c->w+x];
            if (!h->hit) continue;
            int tx=ox+x, ty=oy+y;
            if (tx<0||tx>=tcols||ty<0||ty>=trows) continue;

            float steps_n = (float)h->steps / (float)RM_MAX_STEPS;
            if (steps_n > 1.f) steps_n = 1.f;

            attr_t attr = lumi_attr(debug_slot(steps_n));
            attron(attr);
            mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(steps_n));
            attroff(attr);
        }
    }
}

/* ── §17 scene + screen + app ────────────────────────────────────────── */

typedef struct {
    Canvas    canvas;
    int       prim;
    float     size;
    float     ry;            /* Y-spin angle (fast axis)               */
    float     rot_spd;       /* Y speed; X = rot_spd · ROT_X_RATIO     */
    float     time;
    float     cam_z;
    int       theme_index;
    DebugMode debug_mode;
    bool      paused;
} Scene;

static V3 scene_light(void)
{
    return v3(LIGHT_X, LIGHT_Y, LIGHT_Z);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    canvas_alloc(&s->canvas, cols, rows);
    s->prim        = 0;
    s->size        = PRIM_SIZE_DEFAULT;
    s->ry          = 0.f;
    s->rot_spd     = ROT_Y_SPD_DEFAULT;
    s->cam_z       = CAM_Z_DEFAULT;
    s->theme_index = 0;
    s->debug_mode  = DEBUG_NORMAL;
}

static void scene_free(Scene *s)
{
    canvas_free(&s->canvas);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    canvas_free(&s->canvas);
    canvas_alloc(&s->canvas, cols, rows);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
    s->ry   += s->rot_spd * dt;
}

static void scene_render(Scene *s)
{
    canvas_render(&s->canvas, s->prim, s->size, s->ry, scene_light(),
                  s->cam_z);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    switch (s->debug_mode) {
    case DEBUG_NORMAL:  canvas_draw         (&s->canvas, cols, rows);            break;
    case DEBUG_NORMALS: canvas_draw_normals (&s->canvas, cols, rows);            break;
    case DEBUG_DEPTH:   canvas_draw_depth   (&s->canvas, cols, rows, s->cam_z);  break;
    case DEBUG_STEPS:   canvas_draw_steps   (&s->canvas, cols, rows);            break;
    default:            canvas_draw         (&s->canvas, cols, rows);            break;
    }
}

/* §17.1 screen — ncurses init + HUD + present. */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free  (Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* HUD layout (CLAUDE.md spec):
 *   row 0       PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1  PAIR_HINT (cyan   + bold) — key hint
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sfps)
{
    erase();
    scene_draw(sc, s->cols, s->rows);

    char status[200];
    snprintf(status, sizeof status,
             " %4.1f fps  [%2d/%2d] %-14s  size:%.2f  zoom:%.2f  "
             "theme:%s  debug:%s  sim:%d  %s ",
             fps, sc->prim+1, N_PRIMS, k_prims[sc->prim].name,
             sc->size, sc->cam_z,
             THEMES[sc->theme_index].display_name,
             DEBUG_MODE_NAMES[sc->debug_mode],
             sfps,
             sc->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > s->cols) slen = s->cols;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, s->cols - slen, "%s", status);
    mvprintw(0, 0, " PRIMITIVES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  Tab/t/T:prim  +/-:size  r/R:spin  "
             "z/Z:zoom  c/C:theme  d/D:debug ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void){ wnoutrefresh(stdscr); doupdate(); }

/* §17.2 app — main loop, signals, key handling. */

typedef struct {
    Scene  scene;
    Screen screen;
    int    sim_fps;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit_signal  (int sig){ (void)sig; g_app.running=0; }
static void on_resize(int sig){ (void)sig; g_app.need_resize=1; }
static void cleanup  (void)  { endwin(); }

static void app_do_resize(App *a)
{
    screen_resize(&a->screen);
    scene_resize(&a->scene, a->screen.cols, a->screen.rows);
    a->need_resize = 0;
}

static bool app_handle_key(App *a, int ch)
{
    Scene *s = &a->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case '\t': case 't':
        s->prim = (s->prim + 1) % N_PRIMS; s->ry = 0.f; break;
    case 'T':
        s->prim = (s->prim + N_PRIMS - 1) % N_PRIMS; s->ry = 0.f; break;

    case ' ': s->paused = !s->paused; break;

    case '=': case '+':
        s->size *= PRIM_SIZE_STEP;
        if (s->size > PRIM_SIZE_MAX) s->size = PRIM_SIZE_MAX;
        break;
    case '-':
        s->size /= PRIM_SIZE_STEP;
        if (s->size < PRIM_SIZE_MIN) s->size = PRIM_SIZE_MIN;
        break;

    case 'r':
        s->rot_spd += 0.15f;
        if (s->rot_spd > ROT_SPD_MAX) s->rot_spd = ROT_SPD_MAX;
        break;
    case 'R':
        s->rot_spd -= 0.15f;
        if (s->rot_spd < ROT_SPD_MIN) s->rot_spd = ROT_SPD_MIN;
        break;

    case 'z':
        s->cam_z -= CAM_ZOOM_STEP;
        if (s->cam_z < CAM_Z_MIN) s->cam_z = CAM_Z_MIN;
        break;
    case 'Z':
        s->cam_z += CAM_ZOOM_STEP;
        if (s->cam_z > CAM_Z_MAX) s->cam_z = CAM_Z_MAX;
        break;

    case 'c':
        s->theme_index = (s->theme_index + 1) % THEME_COUNT;
        theme_apply(s->theme_index);
        break;
    case 'C':
        s->theme_index = (s->theme_index + THEME_COUNT - 1) % THEME_COUNT;
        theme_apply(s->theme_index);
        break;

    case 'd':
        s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
        break;
    case 'D':
        s->debug_mode =
            (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,  on_exit_signal);
    signal(SIGTERM, on_exit_signal);
    signal(SIGWINCH, on_resize);

    App *app = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    lut_init();
    screen_init(&app->screen);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

    int64_t ft = clock_ns(), sa = 0, fa = 0;
    int     fc = 0;
    double  fpsd = 0.;

    while (app->running) {
        if (app->need_resize) { app_do_resize(app); ft = clock_ns(); sa = 0; }

        int64_t now = clock_ns(), dt = now - ft;
        ft = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick = TICK_NS(app->sim_fps);
        float   dts  = (float)tick / (float)NS_PER_SEC;
        sa += dt;
        while (sa >= tick) { scene_tick(&app->scene, dts); sa -= tick; }

        scene_render(&app->scene);

        fc++; fa += dt;
        if (fa >= FPS_UPDATE_MS * NS_PER_MS) {
            fpsd = (double)fc / ((double)fa / (double)NS_PER_SEC);
            fc = 0; fa = 0;
        }

        int64_t el = clock_ns() - ft + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - el);

        screen_draw(&app->screen, &app->scene, fpsd, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    scene_free (&app->scene);
    screen_free(&app->screen);
    return 0;
}
