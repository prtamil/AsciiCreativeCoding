/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * neon_edges.c — Tron-style edge glow via Sobel + bloom
 *
 * DEMO: Three low-poly platonic-style shapes — a cube, a tetrahedron,
 *       and an octahedron — float on a near-black floor in deep
 *       atmosphere. Their surfaces are barely lit (the scene is
 *       intentionally dark), but their SILHOUETTES and CREASE EDGES
 *       glow bright cyan, with a soft halo bleeding outward. Each
 *       shape spins slowly on its own axis as the camera orbits.
 *
 *       The glow comes from a Sobel filter applied to the G-buffer's
 *       depth + normal channels: any pixel whose neighbours have a
 *       big depth jump (silhouette) or a big normal change (crease)
 *       is flagged as an edge. The edge mask drives a HDR neon
 *       colour into a side buffer, that buffer gets added to the lit
 *       output, and the existing bloom pipeline does the rest.
 *
 *       Press 'e' to toggle edge detection — the shapes go dark
 *       outside their dim direct-lit areas, and you see what the
 *       algorithm contributes. Press 'b' to toggle bloom — edges
 *       become hard-pixel lines without the soft glow.
 *
 * Study alongside:
 *   raster/bloom_finale.c                — same lightpass + bloom pipeline
 *   raster/ssao_pipeline.c               — same G-buffer (with z_view)
 *   raster/deferred_rendering_pipeline.c — same MVP setup
 *
 * Section map:
 *   §1  config     — frame, view, scene, edge, bloom, lighting, ramp
 *   §2  clock      — monotonic timer + sleep
 *   §3  math       — V3, V4, Mat4 + perspective / lookat
 *   §4  paint      — 216-pair RGB cube + Bourke ramp + paint_cell
 *   §5  mesh       — Vertex / Triangle types + box / tetra / octa / quad
 *   §6  gbuffer    — geometry pass (pos, normal, albedo, depth, view-z)
 *   §7  edge       — Sobel on z_view + normal → g_edge (HDR neon)
 *   §8  lightpass  — Blinn-Phong + g_edge (HDR output)
 *   §9  bloom      — extract → separable Gaussian (H + V) → composite
 *   §10 scene      — Scene struct, init / view / tick (per-object spin)
 *   §11 screen     — render_scene + HUD (CLAUDE.md spec)
 *   §12 app        — signals, resize, fixed-step main loop
 *
 * Keys:
 *   e / E     toggle edge detection on/off
 *   b / B     toggle bloom on/off
 *   + / =     zoom in
 *   - / _     zoom out
 *   space     pause / resume rotation + camera orbit
 *   r / R     reset
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/neon_edges.c -o neon -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Screen-space edge detection via Sobel filter on the
 *                  G-buffer, fed into the bloom pipeline. Sobel is a
 *                  3×3 image convolution that approximates the spatial
 *                  gradient of a scalar field; large gradient = edge.
 *                  We run it on TWO sources: linear view-space z (to
 *                  catch silhouettes against the void), and each
 *                  component of the world normal (to catch creases
 *                  where adjacent faces meet at an angle). Combine
 *                  the two magnitudes, smooth-threshold, multiply by
 *                  a HDR neon colour. The bloom pipeline then bleeds
 *                  it into a soft glow.
 *
 * Data-structure : Existing G-buffer (g_pos, g_normal, g_albedo,
 *                  g_zbuf, g_z_view, g_valid) + one new HDR buffer
 *                  g_edge holding the neon colour per pixel + the
 *                  bloom buffers (g_bloom, g_bloom_tmp). All static.
 *
 * Rendering      : Per frame:
 *                    1. render_gbuffer  — pos / normal / albedo /
 *                                         NDC-z / view-z / valid
 *                    2. edge_pass       — Sobel → g_edge (HDR)
 *                    3. render_lightpass — dim Blinn-Phong + g_edge
 *                    4. bloom_extract → bloom_blur_h → bloom_blur_v
 *                       → bloom_composite
 *                    5. paint_cell — Reinhard tone-map collapses HDR
 *
 * Performance    : Edge pass is O(pixels × 9 reads × 4 channels) =
 *                  trivial at terminal resolution. Bloom is the same
 *                  cost as bloom_finale.c.
 *
 * References     : Sobel & Feldman, "A 3×3 Isotropic Gradient
 *                    Operator for Image Processing," talk at SAIL
 *                    (1968). The original Sobel filter.
 *                  Mitchell et al., "Real-Time Rendering Tricks for
 *                    Ambient Occlusion and Edge Detection," GDC 2007.
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
 * Don't model edge geometry. Detect edges in the IMAGE after the
 * scene is rendered. Two things define a visual edge: a place where
 * depth jumps suddenly (silhouette) or where the surface normal
 * suddenly flips (crease between two faces). Both show up as a high
 * spatial gradient in the G-buffer. Sobel measures that gradient in
 * one cheap convolution; threshold it; colour the result; let bloom
 * bleed it into a glow.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine running your finger across the depth buffer. On a smooth
 * surface, the depth slides under your finger smoothly — small
 * derivative. At a silhouette, the depth abruptly drops to the void
 * (or the next surface) — large derivative. At a crease, the normal
 * flips abruptly — large derivative on the normal field. Sobel is
 * just "compute that derivative magnitude per pixel". Bright where
 * the derivative is large = exactly the edges.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │  flat surface       silhouette       crease  │
 *      │                                              │
 *      │   ── ── ──         ── ──        ────┐        │
 *      │                          ╲             ╲     │
 *      │                           ╲              ╲   │
 *      │                                                  │
 *      │  ∇z ≈ 0          ∇z = HUGE        ∇z small  │
 *      │  ∇N ≈ 0          ∇N = small       ∇N = HUGE │
 *      └──────────────────────────────────────────────┘
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 *  1. G-BUFFER PASS — pos / normal / albedo / NDC-z / view-z. View-z
 *     is the linear depth Sobel needs (NDC-z is non-linear after
 *     perspective and would skew the gradient).
 *
 *  2. EDGE PASS — for each visible pixel (r, c):
 *       a. Sample 3×3 neighbourhood of view-z. If any neighbour has
 *          g_valid=0, treat it as -CAM_FAR — that creates a HUGE
 *          gradient at silhouettes against the void.
 *       b. depth_grad = magnitude of Sobel on view-z.
 *       c. For each normal component (Nx, Ny, Nz): take the Sobel
 *          magnitude, sum.
 *       d. edge = max(depth_grad · DEPTH_SCALE,
 *                     normal_grad · NORMAL_SCALE)
 *       e. t = smoothstep(THRESHOLD, THRESHOLD + KNEE, edge)
 *       f. g_edge[r][c] = NEON · t · INTENSITY
 *
 *  3. LIGHT PASS — dim Blinn-Phong:
 *       lit = ambient·albedo + albedo·sun·max(0, N·L)
 *           + sun·spec_falloff
 *           + g_edge[r][c]                 ← HDR edge contribution
 *     Sun + ambient are intentionally dim, so the EDGES dominate the
 *     visual energy. lit is HDR — the bloom pipeline expects that.
 *
 *  4. BLOOM — extract pixels above THRESHOLD, separable Gaussian
 *     blur, composite. Same as bloom_finale.c.
 *
 *  5. PAINT — Reinhard tone-map + 6×6×6 cube + Bourke ramp.
 *
 * KEY FORMULAS
 * ────────────
 *   Sobel kernels (3×3):
 *     Sx = | -1  0  1 |     Sy = | -1 -2 -1 |
 *          | -2  0  2 |          |  0  0  0 |
 *          | -1  0  1 |          |  1  2  1 |
 *
 *   gx = Σ Sx_ij · field_neighbour_ij
 *   gy = Σ Sy_ij · field_neighbour_ij
 *   |∇field| = sqrt(gx² + gy²)
 *
 *   Edge magnitude:
 *     depth_e   = |∇ z_view|
 *     normal_e  = |∇ N.x| + |∇ N.y| + |∇ N.z|
 *     edge      = max(depth_e · DEPTH_SCALE, normal_e · NORMAL_SCALE)
 *
 *   Smooth threshold:
 *     t = clamp01((edge − THRESHOLD) / KNEE)
 *     t = t² · (3 − 2t)             (smoothstep)
 *
 *   Edge buffer:
 *     g_edge = NEON_COLOR · t · INTENSITY
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Linear depth, NOT NDC-z — NDC-z is non-linear after perspective
 *     and gives spurious gradients on receding flat surfaces. We use
 *     g_z_view (linear, in view space) for Sobel.
 *
 *   • Invalid neighbours — at the silhouette, a 3×3 neighbourhood
 *     straddles the object boundary; some samples fall on g_valid=0
 *     pixels (the void). Substituting -CAM_FAR for those creates a
 *     large depth jump → silhouette correctly detected.
 *
 *   • Subdivision crease noise — a smooth surface tessellated into
 *     many flat triangles has small normal jumps at every triangle
 *     boundary. NORMAL_SCALE × THRESHOLD must reject those. Tuned
 *     here so only "hard" creases (~30°+ between faces) trigger.
 *
 *   • Bloom threshold — must be below the dim ambient + diffuse
 *     baseline so it ONLY fires on edges, not on faintly-lit faces.
 *     Set just below NEON · INTENSITY so even partial edges glow.
 *
 *   • Ambient must stay LOW or the scene drowns the edges. Tron
 *     atmosphere = near-black canvas, glowing wireframe. We use
 *     AMBIENT ~ 0.03 — almost zero.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Toggle 'e' off: shapes go nearly invisible (only the dim
 *     Blinn-Phong remains). The bright cyan outlines and creases
 *     vanish — that's exactly the edge detection's contribution.
 *
 *   • Toggle 'b' off: edges become hard pixel-precise lines, no
 *     soft halo. Toggle on: a 3-cell-wide cyan glow blooms outward.
 *
 *   • Pause (space): cube creases stay sharp, octahedron silhouette
 *     traces its 8 outward triangle edges, tetrahedron shows its
 *     four faces' creases. Each face of each shape has constant
 *     normal, so the FACE INTERIORS don't trigger — only the
 *     boundaries between faces.
 *
 *   • Watch a face rotate edge-on to the camera: depth gradient
 *     near the silhouette grows large → silhouette glow. Watch a
 *     face rotate to face the camera directly: silhouette is now
 *     at the perimeter of a flat disc, normal gradient near zero
 *     in the interior → only the rim glows.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read raster/bloom_finale.c first if you don't yet
 *      know the bloom (extract → blur → composite) pipeline; this
 *      file ADDS an edge stage in front of the bloom rather than
 *      replacing it.
 *   2. §1 config — every constant has a unit-bearing comment.
 *      Note the SCALE / THRESHOLD / KNEE triple — those are the
 *      knobs that decide WHAT counts as an edge.
 *   3. §7 edge — the Sobel pass. THE HEART of this file. Read
 *      AFTER tutorials T1-T4. The G-buffer it reads is identical
 *      to ssao_pipeline.c's; the buffer it writes (g_edge) is
 *      consumed only by §8 lightpass.
 *   4. §6 gbuffer — same forward rasteriser as everywhere else.
 *      Skim if you've seen cube_raster.c.
 *   5. §8 lightpass + §9 bloom — read AFTER bloom_finale.c.
 *      The only addition here is the line that adds g_edge into
 *      the lit HDR sum.
 *   6. §3 math + §4 paint + §10-§12 — infrastructure; skip on
 *      first read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   g_z_view[r][c]      LINEAR view-space depth (positive = away).
 *                       Sobel must run on this, NOT on g_zbuf
 *                       (which is non-linear NDC-z).
 *   g_valid[r][c]       1 = a triangle wrote here this frame, 0 =
 *                       background (void).
 *   g_edge[r][c]        HDR cyan colour added to the lit output;
 *                       0 = no edge.
 *   gx, gy              Sobel x/y partial sums for one channel.
 *   depth_grad          √(gx² + gy²) on z_view.
 *   normal_grad         sum of three √(gx² + gy²) on Nx, Ny, Nz.
 *   edge                Combined gradient before threshold.
 *   t                   smoothstep((edge − THRESHOLD) / KNEE).
 *
 * Background you need
 * ───────────────────
 *   - The G-buffer pattern (deferred_rendering_pipeline.c).
 *   - Convolution: a kernel (3×3 weights) applied at every pixel
 *     to produce a new pixel.
 *   - Bloom basics (bloom_finale.c) — extract → Gaussian blur →
 *     composite.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Stencil-buffer outline tricks. Those operate during the
 *     forward pass and require multiple draws per object; we do
 *     it once in screen space.
 *   - Geometry shader / inverted-hull outlines. Same — those
 *     require GPU geometry-stage features we don't have.
 *   - The full Canny edge-detection pipeline (Sobel + non-max
 *     suppression + double threshold + hysteresis). We use only
 *     Sobel — for real-time stylised edges that's enough.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Eight tutorials that build screen-space edge detection from
 * first principles.
 *
 *   T1  Two kinds of edges — silhouette and crease
 *   T2  Why screen-space edge detection at all
 *   T3  The Sobel kernel — what those numbers MEAN
 *   T4  Combining depth and normal gradients
 *   T5  Linear depth (z_view) vs. non-linear depth (z_NDC)
 *   T6  The void problem — borrowing -CAM_FAR for invalid samples
 *   T7  Smooth thresholding — why a hard cut-off looks bad
 *   T8  Plugging into bloom — edges as HDR contributors
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  TWO KINDS OF EDGES — SILHOUETTE AND CREASE
 * ──────────────────────────────────────────────
 * A pencil sketch of a cube has two visually distinct line types:
 *
 *   SILHOUETTE   the outline against the empty page — separates
 *                "object" from "no object." Crosses the boundary
 *                between cube depth and infinite background depth.
 *
 *   CREASE       the lines BETWEEN faces. The depth varies
 *                smoothly across a crease (small jump), but the
 *                surface ORIENTATION (normal) flips sharply.
 *
 * A robust outline detector must catch BOTH. One signal can't
 * cover both because:
 *
 *   - Silhouette has huge depth jump, small normal change at the
 *     boundary (you're sliding off into the void).
 *   - Crease has small depth jump, huge normal change.
 *
 * Solution: run two detectors and combine them with `max`. Any
 * pixel that hits either condition gets glow.
 *
 *      ┌─────────────────────────────────────────────────────┐
 *      │                                                     │
 *      │       silhouette                  crease            │
 *      │                                                     │
 *      │       ┌────────                  ┌─────             │
 *      │       │  cube                    │face1│            │
 *      │       │ (foreground)             │     │            │
 *      │       │                          ├─ ─ ─┤            │
 *      │       │                          │face2│            │
 *      │       └────────                  └─────             │
 *      │                                                     │
 *      │  ∇z = HUGE                  ∇z ≈ 0                  │
 *      │  ∇N ≈ 0                     ∇N = HUGE               │
 *      └─────────────────────────────────────────────────────┘
 *
 * T2  WHY SCREEN-SPACE EDGE DETECTION AT ALL
 * ──────────────────────────────────────────
 * Several other ways to draw outlines exist:
 *
 *   GEOMETRIC      For each silhouette edge in 3-D, find adjacent
 *                  triangles, test whether one faces the camera and
 *                  the other away, draw a line in 3-D. Heavy CPU
 *                  topology work; per-frame; fragile under deforming
 *                  meshes.
 *
 *   INVERTED HULL  Draw the model twice: a slightly-fat black copy
 *                  with reversed winding, then the normal model on
 *                  top. The black halo peeks out at silhouettes.
 *                  Cheap but only does silhouettes; no creases.
 *
 *   STENCIL        Mark pixels covered by the model in the stencil
 *                  buffer; expand the mark by one pixel; XOR with
 *                  the original. Same constraint — silhouettes only.
 *
 *   SCREEN-SPACE   Render the scene normally → look at the resulting
 *                  G-buffer → find pixels where depth or normal
 *                  changes fast → tag those pixels.
 *
 * Screen-space wins on three axes for our use case:
 *
 *   1. ONE pass for everything in the scene, regardless of mesh
 *      complexity or topology.
 *   2. Catches BOTH silhouettes and creases for free (just look at
 *      different G-buffer channels).
 *   3. Works downstream of the regular pipeline — every other shader
 *      stays unchanged.
 *
 * Cost: nine G-buffer reads per pixel (the 3×3 neighbourhood).
 * Trivial.
 *
 * T3  THE SOBEL KERNEL — WHAT THOSE NUMBERS MEAN
 * ──────────────────────────────────────────────
 * Sobel is a 3×3 convolution that approximates the spatial gradient.
 * Two kernels — one for x, one for y:
 *
 *     Sx = | -1  0  +1 |       Sy = | -1 -2 -1 |
 *          | -2  0  +2 |            |  0  0  0 |
 *          | -1  0  +1 |            | +1 +2 +1 |
 *
 * To apply Sx at pixel p, you take its 8 neighbours + itself, weight
 * each by the kernel, sum:
 *
 *     gx = -1·NW + 0·N + 1·NE
 *          -2·W  + 0·P + 2·E
 *          -1·SW + 0·S + 1·SE
 *
 * Read in plain English: "how much darker is the LEFT side than the
 * RIGHT side, with the MIDDLE row counting double?" That's the rate
 * of change in x. Sy does the same vertically.
 *
 * Why double the middle? Two reasons:
 *
 *   - The middle row is on the same y-line as P; it gives the most
 *     direct measurement of the x-gradient AT P.
 *   - The corner samples are √2 farther from P than the edge
 *     samples; downweighting them by 2× compensates for the larger
 *     distance, making the response approximately rotationally
 *     invariant.
 *
 * The combined gradient magnitude is √(gx² + gy²) — Pythagorean
 * length of the gradient vector.
 *
 * Sobel is just one specific 3×3 derivative kernel. Variants
 * (Prewitt, Scharr) tweak the weights for slightly different
 * isotropy properties. For our purpose Sobel is plenty.
 *
 * T4  COMBINING DEPTH AND NORMAL GRADIENTS
 * ────────────────────────────────────────
 * We compute four Sobel magnitudes per pixel:
 *
 *     |∇ z_view|     ← scalar, one number
 *     |∇ Nx|         ← Sobel on the X component of the world normal
 *     |∇ Ny|
 *     |∇ Nz|
 *
 * The depth gradient gives silhouettes. The three normal gradients
 * combine into one number:
 *
 *     normal_grad = |∇ Nx| + |∇ Ny| + |∇ Nz|
 *
 * (This is the L¹ norm of the per-component gradient. Could equally
 * use L²; L¹ is cheaper and slightly noisier — fine here.)
 *
 * Final edge magnitude is the bigger of the two:
 *
 *     edge = max(depth_grad  · DEPTH_SCALE,
 *                normal_grad · NORMAL_SCALE)
 *
 * The two SCALE constants exist because the two signals have
 * incomparable units. depth_grad is in view-space units (≈ scene
 * size). normal_grad is dimensionless (normals are unit length).
 * The scales are tuned so a "real" silhouette and a "real" crease
 * produce roughly the same edge magnitude — that way one
 * THRESHOLD value works for both.
 *
 * T5  LINEAR DEPTH (z_view) VS. NON-LINEAR DEPTH (z_NDC)
 * ──────────────────────────────────────────────────────
 * The G-buffer has TWO depth fields:
 *
 *   g_zbuf[r][c]      NDC z, in [-1, +1]. Used by the rasteriser
 *                     for z-test. After perspective divide, NDC z
 *                     is HEAVILY non-linear: 90% of the [-1, +1]
 *                     range is occupied by the near 10% of the
 *                     scene. Tiny depth changes near the camera
 *                     produce huge NDC changes; large depth
 *                     changes far away produce tiny NDC changes.
 *
 *   g_z_view[r][c]    Linear view-space z, in scene units. A
 *                     surface 1 unit farther always reads 1 unit
 *                     bigger.
 *
 * Sobel needs LINEAR. Otherwise:
 *
 *   - A flat floor receding into the distance has zero real depth
 *     gradient (it's flat) but a NON-zero NDC-z gradient (NDC is
 *     non-linear). Sobel on NDC-z would draw bright lines along
 *     receding floor stripes — false silhouettes everywhere.
 *
 *   - A real silhouette at the back of the scene has small NDC-z
 *     change (because the non-linearity has already saturated at
 *     the far plane). Sobel on NDC-z would miss it.
 *
 * Linear z_view solves both. Cost: one extra float per G-buffer
 * pixel.
 *
 * T6  THE VOID PROBLEM — BORROWING -CAM_FAR FOR INVALID SAMPLES
 * ─────────────────────────────────────────────────────────────
 * At a silhouette, the 3×3 neighbourhood STRADDLES the object
 * boundary. Some samples land on the object (g_valid = 1, real
 * z_view), others fall on the empty background (g_valid = 0, no
 * real depth — never written to).
 *
 * If we feed 0.0 in for the void samples, Sobel sees a TINY depth
 * jump and the silhouette doesn't trigger. If we feed +∞, the
 * gradient overflows. The fix is to substitute a SENTINEL value
 * that is both far away and finite:
 *
 *     z_void = -CAM_FAR
 *
 * (Negative because in our convention, view-space looks down -Z,
 * so a far point has z_view < 0.)
 *
 * Now the silhouette pixel sees neighbours at depth roughly equal
 * to its own actual depth on one side, and at -CAM_FAR on the
 * other. The Sobel sum is enormous → silhouette correctly flagged.
 *
 * Same trick for the normal field: void neighbours contribute a
 * sentinel normal (we use (0, 0, 0)), creating a gradient at the
 * boundary. The exact sentinel doesn't matter much for the normal
 * — any value distinct from the surface normal triggers detection.
 *
 * T7  SMOOTH THRESHOLDING — WHY A HARD CUT-OFF LOOKS BAD
 * ──────────────────────────────────────────────────────
 * After computing edge magnitude, we want a per-pixel "edge-ness"
 * in [0, 1]. The naïve approach is a hard cut-off:
 *
 *     t = (edge >= THRESHOLD) ? 1 : 0
 *
 * This produces aliased, jagged edges — every pixel is either
 * fully glowing or fully off. At the boundary you'd see 1-pixel
 * stair-stepping.
 *
 * SMOOTHSTEP softens the transition over a small range:
 *
 *     x = clamp01((edge − THRESHOLD) / KNEE)
 *     t = x² · (3 − 2x)               ← Hermite ease curve
 *
 * Now t ramps from 0 (below threshold) through 0.5 (at midpoint)
 * to 1 (well past threshold). Visually: edges fade in over KNEE
 * units of magnitude, eliminating the binary stair-step.
 *
 * KNEE is the WIDTH of the transition zone. Wide KNEE = soft, ramp-
 * like edges. Narrow KNEE = sharp but anti-aliased. Tuned here so
 * the transition is just wide enough to hide single-pixel aliasing
 * without losing sharpness.
 *
 * T8  PLUGGING INTO BLOOM — EDGES AS HDR CONTRIBUTORS
 * ────────────────────────────────────────────────────
 * The lightpass for this scene is intentionally DIM:
 *
 *     ambient ≈ 0.03 (very low)
 *     diffuse + spec lit by a weak sun
 *     lit_total = ambient·albedo + sun·albedo·max(0, N·L) + ...
 *
 * Add the edge contribution at the END:
 *
 *     lit_total += g_edge[r][c]
 *
 * g_edge is HDR — its components can exceed 1.0. The bloom
 * pipeline (see bloom_finale.c T6) extracts pixels above its own
 * threshold and Gaussian-blurs them. Because the surfaces are
 * dim, only edge pixels exceed bloom threshold, and the bleed
 * traces the outline.
 *
 * Composition: the soft cyan halo around each shape is the bloom
 * stage doing its normal job on a HDR signal that happens to be
 * concentrated along screen-space edges. None of the bloom code
 * is changed from bloom_finale.c — only the source of the bright
 * pixels is.
 *
 * Toggling 'b' OFF leaves the edge contribution but skips the
 * blur stage — you see hard pixel-precise lines. Toggling 'e'
 * OFF skips the edge stage entirely — only the dim Phong lighting
 * remains. The pair gives a clean A/B for what each stage adds.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI */
enum {
    FPS_TARGET    = 60,
    FPS_UPDATE_MS = 500,
    HUD_ROWS      = 5,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define DT_CAP_NS       (100 * NS_PER_MS)

/* §1.2 G-buffer dimensions (static; sized for a large terminal). */
#define GBUF_MAX_W   300
#define GBUF_MAX_H    80

/* §1.3 view geometry — eye orbits the scene. */
#define CAM_FOV       (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR      0.1f
#define CAM_FAR       50.0f

#define CAM_DIST       5.5f
#define CAM_DIST_MIN   3.0f
#define CAM_DIST_MAX  10.0f
#define CAM_ZOOM_STEP  0.4f
#define CAM_EYE_Y      2.4f
#define CAM_LOOK_Y     0.5f
#define CAM_ORBIT_RAD_PER_SEC  0.18f

#define CELL_W        8
#define CELL_H       16

/* §1.4 lighting — intentionally DIM so edges dominate.
 *
 * Tron atmosphere: near-black canvas with glowing wireframe. If
 * AMBIENT or SUN_COL push too high, the dim faces compete with the
 * edges and the demo loses its aesthetic punch. Keep both well
 * under 0.2 per channel. */
static const float SUN_DIR[3]    = { -0.55f, -0.65f, 0.30f };
static const float SUN_COL[3]    = {  0.10f,  0.12f,  0.14f };
static const float AMBIENT_COL[3]= {  0.03f,  0.04f,  0.06f };
#define SHININESS    24.0f
#define SPEC_GAIN     0.20f

/* §1.5 edge detection
 *
 * NEON_COLOR is HDR (channels exceed 1.0). On a fully-lit edge
 * pixel, g_edge = NEON_COLOR · INTENSITY = (0.75, 3.0, 3.75) for
 * the cyan default — well above 1.0 → bloom triggers.
 *
 * THRESHOLD/KNEE shape the smoothstep that turns the raw gradient
 * magnitude into a [0, 1] edge factor:
 *   edge ≤ THRESHOLD              → t = 0   (no glow)
 *   edge ≥ THRESHOLD + KNEE       → t = 1   (full glow)
 *   in between                    → smooth interpolation
 *
 * DEPTH_SCALE/NORMAL_SCALE balance the two signals. Silhouettes
 * naturally produce huge depth gradients (because the Sobel sample
 * outside the silhouette is -CAM_FAR), so DEPTH_SCALE can stay
 * small. Creases produce normal gradients of order 1; NORMAL_SCALE
 * scales those into the threshold range. */
static const float NEON_COLOR[3] = { 0.50f, 2.00f, 2.50f };
#define EDGE_INTENSITY    1.50f
#define EDGE_THRESHOLD    0.75f
#define EDGE_KNEE         0.45f
#define DEPTH_SCALE       0.05f
#define NORMAL_SCALE      0.40f

/* §1.6 bloom parameters
 *
 * Same separable Gaussian as bloom_finale.c. Threshold low enough
 * that any partially-lit edge bleeds; intensity high enough that
 * the halo extends a few cells past the edge. */
#define BLOOM_THRESHOLD   0.90f
#define BLOOM_INTENSITY   1.50f
#define BLOOM_RADIUS         3
#define BLOOM_TAPS    (2 * BLOOM_RADIUS + 1)
static const float BLOOM_KERNEL[BLOOM_TAPS] = {
    0.0702f, 0.1311f, 0.1907f, 0.2161f, 0.1907f, 0.1311f, 0.0702f
};

/* §1.7 scene geometry — three low-poly platonic-style shapes.
 *
 *               (cube)        (tetrahedron)      (octahedron)
 *                ▢                ▲                  ◆
 *           ──────────────────────────────────────────────
 *                              floor (near-black)              */
#define FLOOR_HALF_X     3.0f
#define FLOOR_HALF_Z     3.0f

#define CUBE_HALF        0.55f
#define CUBE_CX         -1.40f
#define CUBE_CY          0.65f
#define CUBE_CZ          0.20f

#define TETRA_R          0.85f
#define TETRA_CX         0.00f
#define TETRA_CY         0.70f
#define TETRA_CZ        -0.20f

#define OCTA_R           0.70f
#define OCTA_CX          1.40f
#define OCTA_CY          0.75f
#define OCTA_CZ          0.30f

enum {
    OBJ_FLOOR = 0,
    OBJ_CUBE,
    OBJ_TETRA,
    OBJ_OCTA,
    N_OBJECTS,
};

/* §1.8 character ramp — Paul Bourke 92-char density ladder. */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.9 Bayer 4×4 dither. */
static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};
#define DITHER_AMP   0.10f

/* §1.10 ncurses pair IDs — 216 cube + yellow HUD + cyan hint. */
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
static inline float v3_luma(Vec3 c)
{
    return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
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

/* m4_lookat — standard glm convention (cross(forward, up)). */
static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up)
{
    Vec3 f = v3_norm(v3_sub(at, eye));
    Vec3 r = v3_norm(v3(f.y*up.z - f.z*up.y,
                        f.z*up.x - f.x*up.z,
                        f.x*up.y - f.y*up.x));
    Vec3 u = v3(r.y*f.z - r.z*f.y,
                r.z*f.x - r.x*f.z,
                r.x*f.y - r.y*f.x);
    Mat4 m = m4_identity();
    m.m[0][0] = r.x; m.m[0][1] = r.y; m.m[0][2] = r.z; m.m[0][3] = -v3_dot(r, eye);
    m.m[1][0] = u.x; m.m[1][1] = u.y; m.m[1][2] = u.z; m.m[1][3] = -v3_dot(u, eye);
    m.m[2][0] = -f.x; m.m[2][1] = -f.y; m.m[2][2] = -f.z; m.m[2][3] = v3_dot(f, eye);
    return m;
}

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

/* Rotations around X / Y axes — used for per-object spin. */
static Mat4 m4_rotate_x(float a)
{
    Mat4 m = m4_identity();
    float c = cosf(a), s = sinf(a);
    m.m[1][1] = c;  m.m[1][2] = -s;
    m.m[2][1] = s;  m.m[2][2] =  c;
    return m;
}
static Mat4 m4_rotate_y(float a)
{
    Mat4 m = m4_identity();
    float c = cosf(a), s = sinf(a);
    m.m[0][0] =  c;  m.m[0][2] = s;
    m.m[2][0] = -s;  m.m[2][2] = c;
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
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_CUBE_BASE, COLOR_WHITE,  -1);
        init_pair(PAIR_HUD,       COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,   -1);
    }
}

static inline float clamp01  (float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float reinhard (float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

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
 * tessellate_polyhedron — given an array of unique vertex positions
 * and an array of triangle face indices (CCW from outside), produce
 * a Mesh with one FLAT face normal per triangle. Each face writes 3
 * fresh vertices so adjacent faces don't share a normal — exactly
 * what we want for crease detection.
 */
static Mesh tessellate_polyhedron(const Vec3 *verts, int n_verts,
                                  const int (*faces)[3], int n_faces)
{
    (void)n_verts;
    Mesh m;
    m.verts = malloc((size_t)n_faces * 3 * sizeof(Vertex));
    m.tris  = malloc((size_t)n_faces *     sizeof(Triangle));
    m.nvert = 0; m.ntri = 0;

    for (int f = 0; f < n_faces; f++) {
        Vec3 a = verts[faces[f][0]];
        Vec3 b = verts[faces[f][1]];
        Vec3 c = verts[faces[f][2]];
        Vec3 e1 = v3_sub(b, a);
        Vec3 e2 = v3_sub(c, a);
        Vec3 nrm = v3_norm(v3(e1.y*e2.z - e1.z*e2.y,
                              e1.z*e2.x - e1.x*e2.z,
                              e1.x*e2.y - e1.y*e2.x));
        int v0 = m.nvert;
        m.verts[m.nvert++] = (Vertex){ a, nrm, 0.f, 0.f };
        m.verts[m.nvert++] = (Vertex){ b, nrm, 1.f, 0.f };
        m.verts[m.nvert++] = (Vertex){ c, nrm, 0.f, 1.f };
        m.tris [m.ntri++ ] = (Triangle){{ v0, v0+1, v0+2 }};
    }
    return m;
}

static Mesh tessellate_box(float hx, float hy, float hz)
{
    Mesh m;
    m.verts = malloc(24 * sizeof(Vertex));
    m.tris  = malloc(12 * sizeof(Triangle));
    m.nvert = 0; m.ntri = 0;

    mesh_add_quad(&m, v3( hx,-hy,-hz), v3(0, 2*hy, 0), v3(0, 0, 2*hz),  v3( 1, 0, 0));
    mesh_add_quad(&m, v3(-hx,-hy, hz), v3(0, 2*hy, 0), v3(0, 0,-2*hz),  v3(-1, 0, 0));
    mesh_add_quad(&m, v3(-hx, hy, hz), v3(2*hx, 0, 0), v3(0, 0,-2*hz),  v3( 0, 1, 0));
    mesh_add_quad(&m, v3(-hx,-hy,-hz), v3(2*hx, 0, 0), v3(0, 0, 2*hz),  v3( 0,-1, 0));
    mesh_add_quad(&m, v3(-hx,-hy, hz), v3(2*hx, 0, 0), v3(0, 2*hy, 0),  v3( 0, 0, 1));
    mesh_add_quad(&m, v3( hx,-hy,-hz), v3(-2*hx, 0, 0), v3(0, 2*hy, 0), v3( 0, 0,-1));
    return m;
}

/*
 * tessellate_tetrahedron — regular tetrahedron centred at origin,
 * each face has its own flat normal. 4 faces, 4 sharp creases.
 *   verts: alternating corners of the (±1, ±1, ±1) cube with even
 *          parity (signs multiply to +1).
 */
static Mesh tessellate_tetrahedron(float r)
{
    float s = r / sqrtf(3.f);
    Vec3 verts[4] = {
        v3( s,  s,  s),
        v3( s, -s, -s),
        v3(-s,  s, -s),
        v3(-s, -s,  s),
    };
    static const int faces[4][3] = {
        {0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2},
    };
    return tessellate_polyhedron(verts, 4, faces, 4);
}

/*
 * tessellate_octahedron — 6 axis-point vertices, 8 triangular faces
 * (one per octant). Each face has its own flat normal pointing
 * outward in (sx, sy, sz) direction.
 */
static Mesh tessellate_octahedron(float r)
{
    Vec3 verts[6] = {
        v3( r, 0, 0), v3(-r, 0, 0),     /* 0=+X, 1=-X */
        v3( 0, r, 0), v3( 0,-r, 0),     /* 2=+Y, 3=-Y */
        v3( 0, 0, r), v3( 0, 0,-r),     /* 4=+Z, 5=-Z */
    };
    static const int faces[8][3] = {
        {0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4},   /* +Z half */
        {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5},   /* -Z half */
    };
    return tessellate_polyhedron(verts, 6, faces, 8);
}

static Mesh tessellate_quad(Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm)
{
    Mesh m;
    m.verts = malloc(4 * sizeof(Vertex));
    m.tris  = malloc(2 * sizeof(Triangle));
    m.nvert = 0; m.ntri = 0;
    mesh_add_quad(&m, origin, e1, e2, nrm);
    return m;
}

/* ── §6 G-buffer — geometry pass ─────────────────────────────────────── */

static Vec3    g_pos    [GBUF_MAX_H][GBUF_MAX_W];
static Vec3    g_normal [GBUF_MAX_H][GBUF_MAX_W];
static Vec3    g_albedo [GBUF_MAX_H][GBUF_MAX_W];
static float   g_zbuf   [GBUF_MAX_H][GBUF_MAX_W];
static float   g_z_view [GBUF_MAX_H][GBUF_MAX_W];   /* linear, for Sobel */
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

static void rasterize_object(const Mesh *mesh, Vec3 albedo,
                             Mat4 mvp, Mat4 model, Mat4 modelview,
                             Mat4 norm_mat, int cols, int rows)
{
    for (int ti = 0; ti < mesh->ntri; ti++) {
        const Triangle *tri = &mesh->tris[ti];

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

        float sx[3], sy[3], sz[3];
        for (int vi = 0; vi < 3; vi++) {
            float w = clip[vi].w; if (fabsf(w) < 1e-6f) w = 1e-6f;
            sx[vi] = ( clip[vi].x / w + 1.f) * 0.5f * (float)cols;
            sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows;
            sz[vi] =   clip[vi].z / w;
        }

        /* Back-face cull. Screen Y-flip turns OpenGL CCW front faces
         * into NEGATIVE signed area; keep negative, reject the rest. */
        float area = (sx[1] - sx[0]) * (sy[2] - sy[0])
                   - (sx[2] - sx[0]) * (sy[1] - sy[0]);
        if (area >= 0.f) continue;

        int x0 = (int)fmaxf(0.f,        floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
        int x1 = (int)fminf(cols - 1.f,  ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
        int y0 = (int)fmaxf(0.f,        floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
        int y1 = (int)fminf(rows - 1.f,  ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

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

/* ── §7 edge — Sobel on z_view + normal ──────────────────────────────── */

static Vec3 g_edge[GBUF_MAX_H][GBUF_MAX_W];   /* HDR neon per pixel */

/* Sobel magnitude on a 3×3 sample window stored row-major in `field[9]`.
 *   field[0..2] = top row, field[3..5] = middle, field[6..8] = bottom.  */
static float sobel_magnitude(const float field[9])
{
    float gx = (field[2] + 2.f*field[5] + field[8])
             - (field[0] + 2.f*field[3] + field[6]);
    float gy = (field[6] + 2.f*field[7] + field[8])
             - (field[0] + 2.f*field[1] + field[2]);
    return sqrtf(gx*gx + gy*gy);
}

static inline int clamp_ix(int v, int max) { return v < 0 ? 0 : (v >= max ? max - 1 : v); }

/* Sample a 3×3 neighbourhood of g_z_view at (r, c). Invalid neighbours
 * (outside the screen or g_valid=0) are recorded as -CAM_FAR — that
 * gap creates a HUGE Sobel gradient at silhouettes, which is exactly
 * how we detect them. */
static void sample_zview_3x3(int r, int c, int cols, int rows, float out[9])
{
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            int rr = clamp_ix(r + dr, rows);
            int cc = clamp_ix(c + dc, cols);
            out[(dr+1)*3 + (dc+1)] =
                g_valid[rr][cc] ? g_z_view[rr][cc] : -CAM_FAR;
        }
    }
}

/* Sample 3×3 of one normal component (axis 0=x, 1=y, 2=z). Invalid
 * neighbours record 0 — the Sobel gradient on creases is normal-flip
 * driven, not silhouette driven (silhouettes are caught by depth). */
static void sample_normal_3x3(int r, int c, int axis,
                              int cols, int rows, float out[9])
{
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            int rr = clamp_ix(r + dr, rows);
            int cc = clamp_ix(c + dc, cols);
            float v = 0.f;
            if (g_valid[rr][cc]) {
                Vec3 n = g_normal[rr][cc];
                v = (axis == 0) ? n.x : (axis == 1) ? n.y : n.z;
            }
            out[(dr+1)*3 + (dc+1)] = v;
        }
    }
}

/* GLSL-style smoothstep on x ∈ [edge0, edge1] → [0, 1] with a smooth
 * cubic falloff at both ends. */
static float smoothstep(float edge0, float edge1, float x)
{
    float t = (x - edge0) / (edge1 - edge0);
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    return t * t * (3.f - 2.f * t);
}

/*
 * edge_pass — the Sobel post-process. For each visible pixel:
 *
 *   1. depth_grad  = Sobel magnitude of g_z_view.
 *   2. normal_grad = Σ Sobel magnitudes of N.x, N.y, N.z.
 *   3. edge        = max(depth_grad · DEPTH_SCALE,
 *                        normal_grad · NORMAL_SCALE).
 *   4. t           = smoothstep(THRESHOLD, THRESHOLD+KNEE, edge).
 *   5. g_edge      = NEON_COLOR · t · INTENSITY.
 *
 * Lightpass adds g_edge straight into the HDR g_light, then bloom
 * extracts pixels above its own threshold and bleeds them outward.
 */
static void edge_pass(int cols, int rows)
{
    Vec3 neon = v3(NEON_COLOR[0], NEON_COLOR[1], NEON_COLOR[2]);

    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) { g_edge[r][c] = v3(0, 0, 0); continue; }

            float field[9];

            sample_zview_3x3(r, c, cols, rows, field);
            float depth_grad = sobel_magnitude(field);

            sample_normal_3x3(r, c, 0, cols, rows, field);
            float nx_grad = sobel_magnitude(field);
            sample_normal_3x3(r, c, 1, cols, rows, field);
            float ny_grad = sobel_magnitude(field);
            sample_normal_3x3(r, c, 2, cols, rows, field);
            float nz_grad = sobel_magnitude(field);

            float normal_grad = nx_grad + ny_grad + nz_grad;
            float edge        = fmaxf(depth_grad  * DEPTH_SCALE,
                                      normal_grad * NORMAL_SCALE);
            float t           = smoothstep(EDGE_THRESHOLD,
                                           EDGE_THRESHOLD + EDGE_KNEE, edge);

            g_edge[r][c] = v3_scale(neon, t * EDGE_INTENSITY);
        }
    }
}

/* ── §8 lightpass — dim Blinn-Phong + edge contribution ──────────────── */

static Vec3 g_light[GBUF_MAX_H][GBUF_MAX_W];

static void render_lightpass(Vec3 cam_pos, bool edges_on, int cols, int rows)
{
    Vec3 sun_dir = v3(SUN_DIR[0],     SUN_DIR[1],     SUN_DIR[2]);
    Vec3 sun_col = v3(SUN_COL[0],     SUN_COL[1],     SUN_COL[2]);
    Vec3 ambient = v3(AMBIENT_COL[0], AMBIENT_COL[1], AMBIENT_COL[2]);
    Vec3 L       = v3_norm(v3_neg(sun_dir));

    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) { g_light[r][c] = v3(0,0,0); continue; }

            Vec3 P      = g_pos   [r][c];
            Vec3 N      = g_normal[r][c];
            Vec3 albedo = g_albedo[r][c];

            Vec3 amb = v3(ambient.x * albedo.x,
                          ambient.y * albedo.y,
                          ambient.z * albedo.z);

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

            Vec3 edge = edges_on ? g_edge[r][c] : v3(0, 0, 0);

            /* HDR — edges can push channels well past 1.0. */
            g_light[r][c] = v3(amb.x + dif.x + sp.x + edge.x,
                               amb.y + dif.y + sp.y + edge.y,
                               amb.z + dif.z + sp.z + edge.z);
        }
    }
}

/* ── §9 bloom — extract → blur → composite (same as bloom_finale.c) ──── */

static Vec3 g_bloom    [GBUF_MAX_H][GBUF_MAX_W];
static Vec3 g_bloom_tmp[GBUF_MAX_H][GBUF_MAX_W];

static void bloom_extract(float threshold, int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            Vec3 lit = g_light[r][c];
            g_bloom[r][c] = (v3_luma(lit) > threshold) ? lit : v3(0, 0, 0);
        }
    }
}

static void bloom_blur_h(int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            Vec3 sum = v3(0, 0, 0);
            for (int k = 0; k < BLOOM_TAPS; k++) {
                int   sc = clamp_ix(c + k - BLOOM_RADIUS, cols);
                float w  = BLOOM_KERNEL[k];
                sum = v3_add(sum, v3_scale(g_bloom[r][sc], w));
            }
            g_bloom_tmp[r][c] = sum;
        }
    }
}

static void bloom_blur_v(int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            Vec3 sum = v3(0, 0, 0);
            for (int k = 0; k < BLOOM_TAPS; k++) {
                int   sr = clamp_ix(r + k - BLOOM_RADIUS, rows);
                float w  = BLOOM_KERNEL[k];
                sum = v3_add(sum, v3_scale(g_bloom_tmp[sr][c], w));
            }
            g_bloom[r][c] = sum;
        }
    }
}

static void bloom_composite(float intensity, int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            g_light[r][c] = v3_add(g_light[r][c],
                                   v3_scale(g_bloom[r][c], intensity));
        }
    }
}

/* ── §10 scene ───────────────────────────────────────────────────────── */

typedef struct {
    Mesh        meshes [N_OBJECTS];
    Vec3        albedos[N_OBJECTS];
    Mat4        models [N_OBJECTS];

    Mat4        view, proj;
    Vec3        cam_pos;
    float       cam_dist;
    float       cam_yaw;

    /* per-object spin (only the three platonic shapes rotate) */
    float       spin_angle[N_OBJECTS];
    float       spin_speed[N_OBJECTS];

    bool        edges_on;
    bool        bloom_on;
    bool        paused;
    int         scene_cols;
    int         scene_rows;
} Scene;

static void scene_rebuild_proj(Scene *s, int cols, int rows)
{
    float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
    s->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void scene_rebuild_view(Scene *s)
{
    float r = s->cam_dist;
    s->cam_pos = v3(sinf(s->cam_yaw) * r,
                    CAM_EYE_Y,
                    cosf(s->cam_yaw) * r);
    s->view = m4_lookat(s->cam_pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
}

/* Rebuild a shape's model matrix as translate · rotate_y · rotate_x.
 * Each shape spins on its own axes — the rotation is what reveals
 * the silhouette + crease edges from changing angles. */
static Mat4 spin_model(float cx, float cy, float cz, float angle)
{
    Mat4 ry = m4_rotate_y(angle);
    Mat4 rx = m4_rotate_x(angle * 0.6f);
    Mat4 t  = m4_translate(cx, cy, cz);
    return m4_mul(t, m4_mul(ry, rx));
}

static void scene_init(Scene *s, int total_cols, int total_rows)
{
    for (int i = 0; i < N_OBJECTS; i++) mesh_free(&s->meshes[i]);

    memset(s, 0, sizeof *s);
    s->scene_cols = total_cols;
    s->scene_rows = total_rows - HUD_ROWS;
    s->edges_on   = true;
    s->bloom_on   = true;
    s->cam_dist   = CAM_DIST;
    s->cam_yaw    = 0.f;

    /* OBJ_FLOOR — near-black slate; doesn't spin. */
    s->meshes [OBJ_FLOOR] = tessellate_quad(
        v3(-FLOOR_HALF_X, 0.f,  FLOOR_HALF_Z),
        v3( 2*FLOOR_HALF_X, 0.f,  0.f),
        v3( 0.f,            0.f, -2*FLOOR_HALF_Z),
        v3( 0.f, 1.f, 0.f));
    s->albedos[OBJ_FLOOR] = v3(0.06f, 0.07f, 0.09f);
    s->models [OBJ_FLOOR] = m4_identity();
    s->spin_speed[OBJ_FLOOR] = 0.f;

    /* OBJ_CUBE — 6 faces, 12 creases. Slow spin. */
    s->meshes [OBJ_CUBE] = tessellate_box(CUBE_HALF, CUBE_HALF, CUBE_HALF);
    s->albedos[OBJ_CUBE] = v3(0.10f, 0.10f, 0.13f);
    s->spin_speed[OBJ_CUBE] = 0.45f;
    s->models [OBJ_CUBE] = spin_model(CUBE_CX, CUBE_CY, CUBE_CZ, 0.f);

    /* OBJ_TETRA — 4 faces, 6 creases. Faster spin (smaller poly count). */
    s->meshes [OBJ_TETRA] = tessellate_tetrahedron(TETRA_R);
    s->albedos[OBJ_TETRA] = v3(0.10f, 0.10f, 0.13f);
    s->spin_speed[OBJ_TETRA] = 0.65f;
    s->models [OBJ_TETRA] = spin_model(TETRA_CX, TETRA_CY, TETRA_CZ, 0.f);

    /* OBJ_OCTA — 8 faces, 12 creases, sharp star silhouette. */
    s->meshes [OBJ_OCTA] = tessellate_octahedron(OCTA_R);
    s->albedos[OBJ_OCTA] = v3(0.10f, 0.10f, 0.13f);
    s->spin_speed[OBJ_OCTA] = -0.55f;          /* opposite direction */
    s->models [OBJ_OCTA] = spin_model(OCTA_CX, OCTA_CY, OCTA_CZ, 0.f);

    scene_rebuild_proj(s, total_cols, s->scene_rows);
    scene_rebuild_view(s);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    s->cam_yaw += CAM_ORBIT_RAD_PER_SEC * dt;
    scene_rebuild_view(s);

    s->spin_angle[OBJ_CUBE]  += s->spin_speed[OBJ_CUBE]  * dt;
    s->spin_angle[OBJ_TETRA] += s->spin_speed[OBJ_TETRA] * dt;
    s->spin_angle[OBJ_OCTA]  += s->spin_speed[OBJ_OCTA]  * dt;

    s->models[OBJ_CUBE]  = spin_model(CUBE_CX,  CUBE_CY,  CUBE_CZ,
                                      s->spin_angle[OBJ_CUBE]);
    s->models[OBJ_TETRA] = spin_model(TETRA_CX, TETRA_CY, TETRA_CZ,
                                      s->spin_angle[OBJ_TETRA]);
    s->models[OBJ_OCTA]  = spin_model(OCTA_CX,  OCTA_CY,  OCTA_CZ,
                                      s->spin_angle[OBJ_OCTA]);
}

/* ── §11 screen — render_scene + HUD ─────────────────────────────────── */

static void render_scene(const Scene *s)
{
    int cols = s->scene_cols;
    int rows = s->scene_rows;

    for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
            if (!g_valid[r][c]) continue;
            paint_cell(c, r, g_light[r][c]);
        }
    }
}

static void hud_draw(const Scene *s, double fps)
{
    int hr   = s->scene_rows;
    int cols = s->scene_cols;

    int total_tris = 0;
    for (int i = 0; i < N_OBJECTS; i++) total_tris += s->meshes[i].ntri;

    char status[160];
    snprintf(status, sizeof status,
             " %5.1f fps  edges:%s  bloom:%s  zoom:%.1f  tris:%d  %s ",
             fps,
             s->edges_on ? "ON " : "OFF",
             s->bloom_on ? "ON " : "OFF",
             (double)s->cam_dist, total_tris,
             s->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > cols) slen = cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - slen, "%s", status);
    mvprintw(0, 0, " NEON EDGES · SOBEL + BLOOM ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(hr + 0, 1,
             "passes: gbuffer -> edge(Sobel on z+normal) -> "
             "lightpass(HDR) -> bloom -> paint");
    mvprintw(hr + 1, 1,
             "edge: threshold=%.2f knee=%.2f  depth_scale=%.2f normal_scale=%.2f",
             (double)EDGE_THRESHOLD, (double)EDGE_KNEE,
             (double)DEPTH_SCALE, (double)NORMAL_SCALE);
    mvprintw(hr + 2, 1,
             "Toggle 'e' off: shapes go nearly invisible.   "
             "'b' off: edges are hard pixel lines, no glow.");
    attroff(COLOR_PAIR(PAIR_HUD));

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(hr + HUD_ROWS - 1, 0,
             " q:quit  spc:pause  e:edges  b:bloom  +/-:zoom  r:reset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §12 app ─────────────────────────────────────────────────────────── */

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
    case ' ':           s->paused   = !s->paused;   break;
    case 'r': case 'R': scene_init(s, app->total_cols, app->total_rows); break;
    case 'e': case 'E': s->edges_on = !s->edges_on; break;
    case 'b': case 'B': s->bloom_on = !s->bloom_on; break;
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
 * One-frame pipeline. Reads as plain pseudocode — each named pass
 * does what it says, conditionally, in dependency order.
 */
static void render_frame(const Scene *s)
{
    render_gbuffer(s->meshes, s->albedos, s->models, N_OBJECTS,
                   s->view, s->proj, s->scene_cols, s->scene_rows);

    if (s->edges_on)
        edge_pass(s->scene_cols, s->scene_rows);

    render_lightpass(s->cam_pos, s->edges_on, s->scene_cols, s->scene_rows);

    if (s->bloom_on) {
        bloom_extract  (BLOOM_THRESHOLD, s->scene_cols, s->scene_rows);
        bloom_blur_h   (s->scene_cols, s->scene_rows);
        bloom_blur_v   (s->scene_cols, s->scene_rows);
        bloom_composite(BLOOM_INTENSITY, s->scene_cols, s->scene_rows);
    }
}

int main(void)
{
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
        render_frame(s);
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
