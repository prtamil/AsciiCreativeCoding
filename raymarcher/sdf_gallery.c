/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sdf_gallery.c — five SDF composition operations + a perf-aware renderer
 *
 * DEMO: One sphere-tracing renderer.  Five scenes that show different
 *       ways of BUILDING geometry by COMBINING simpler distance
 *       functions:
 *
 *         1  blend    — animated smooth-union of two spheres + torus
 *         2  boolean  — union / intersection / subtraction side-by-side
 *         3  twist    — rounded box rotated by p.y · k before SDF
 *         4  repeat   — modular fold of p — one sphere → infinite lattice
 *         5  sculpt   — humanoid figure assembled from seven smin'd parts
 *
 *       Three lighting modes (N·V / Phong / Flat), four debug overlays
 *       (NORMAL / NORMALS / DEPTH / STEPS), five colour themes, gamma-
 *       encoded 8-char ramp, full-frame-per-tick rendering, per-frame
 *       trig hoisted into a tiny SceneCache.
 *
 * Study alongside: raymarcher/raymarcher_primitives.c (gallery of 17
 *       PRIMITIVE shapes, each rendered alone — varies the LEAF of
 *       the SDF tree).  This file varies the BRANCHES — the
 *       composition operators that combine a small set of leaves
 *       into more interesting geometry.
 *
 * Section map:
 *   §1   config         — every tunable named, no magic numbers later
 *   §2   clock          — monotonic timer
 *   §3   color          — themes + 2-pair HUD spec (yellow + cyan)
 *   §4   vec3           — value-type 3-D math
 *   §5   SDF primitives — sphere, capsule, torus, round box
 *   §6   SDF operators  — smin, twist, domain repetition
 *   §7   scene cache    — per-frame trig hoisted out of the SDF hot loop
 *   §8   scene 1: blend
 *   §9   scene 2: boolean
 *   §10  scene 3: twist
 *   §11  scene 4: repeat
 *   §12  scene 5: sculpt
 *   §13  scene_map      — dispatch by preset index
 *   §14  march          — 4-tap normal, shadow, AO, trace + step count
 *   §15  shade          — three lighting modes (N·V / Phong / Flat)
 *   §16  cast_pixel     — full per-pixel pipeline → Pixel
 *   §17  canvas         — full-frame render + production draw + debug
 *   §18  app            — main loop, signals, key handling
 *
 * Keys:
 *   q / ESC      quit
 *   1..5         select scene
 *   space / p    pause / resume
 *   t            cycle theme    (Studio / Ember / Arctic / Toxic / Neon)
 *   l            cycle lighting (N·V → Phong → Flat)
 *   d / D        cycle debug overlay (NORMAL / NORMALS / DEPTH / STEPS)
 *   s            toggle soft shadows  (Phong only)
 *   o            toggle ambient occlusion
 *   z / Z        zoom in / out (camera closer / farther)
 *   + / -        orbit speed up / down
 *   r            reset camera
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/sdf_gallery.c \
 *       -o sdf_gallery -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is its own textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         what the algorithm is + references.
 *   • MENTAL MODEL     intuition + ASCII diagrams + per-frame ALGORITHM
 *                      IN STEPS that names every helper involved.
 *   • GUIDED TUTORIAL  ten short answers — composition tree, the five
 *                      scenes, lighting modes, then three performance
 *                      stories (4-tap normal / per-frame trig cache /
 *                      full-frame rendering) tied back to specific
 *                      visible behaviour.
 *   • §1..§18          the actual code, each section short and focused.
 *
 * Ten-minute version: read the GUIDED TUTORIAL.  By the end the §-
 * sections feel like reviewing notes, not learning new material.
 *
 * Math notation used in the code:
 *      p          — a 3-D point (the SDF input)
 *      d          — signed distance (positive outside, negative inside)
 *      col        — material colour signal in [0, 1] (per-region tint)
 *      k          — smin blend strength (small = sharp join)
 *      ro, rd     — ray origin, ray direction (unit)
 *      t          — distance the ray has marched
 *      N          — surface normal at hit
 *      L, V, R    — light, view, reflection unit vectors
 *
 * Background you need:
 *   • basic vector arithmetic (add, dot, cross, length, normalise)
 *   • read raymarcher.c first if sphere tracing is unfamiliar
 *   • read raymarcher_primitives.c first if "what is an SDF?" is new
 *   • the file is performance-aware: tutorials T8, T9, T10 explain the
 *     three explicit choices that keep render under-budget for a
 *     real-time terminal renderer
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : SPHERE TRACING (Hart 1996) of a SCENE that's built
 *                each frame by COMPOSING simpler SDFs at evaluation
 *                time.  Five scenes; each is one composition tree:
 *
 *                  blend      smin( smin(spheres), torus )
 *                  boolean    {min, max, max(·,−·)} of pairs
 *                  twist      sdRoundBox( twist(p, k) )
 *                  repeat     sdSphere( fold(p, cell) )
 *                  sculpt     reduce( smin, [body, head, …, arms] )
 *
 *                Each scene returns SDF2 = (d, col): d drives the
 *                marcher; col is a per-region material signal in
 *                [0, 1] that picks a colour band at draw time.
 *
 *                Per pixel:
 *                  1. ray from camera through cell
 *                  2. sphere-march scene_map(preset, p, t).d
 *                  3. on hit: 4-tap tetrahedral normal, soft shadow,
 *                             5-step AO, shade by mode
 *                  4. (luma, col) → glyph + theme colour pair
 *
 * Data         : Stateless math (Vec3 + 4 primitives + 3 operators +
 *                5 scene functions + dispatcher) — plus ONE SceneCache
 *                global that holds per-frame cosf/sinf values, refreshed
 *                once per tick before any pixel is rendered.  Per-pixel
 *                result is `Pixel` (luma / col / N / hit_t / steps /
 *                hit), held in TWO framebuffers — g_fbuf for the
 *                in-progress canvas pass, g_stable for the most-recent
 *                completed frame.
 *
 * Rendering    : One ray per terminal cell.  Every TICK renders a FULL
 *                FRAME (no progressive row-streaming) — combined with
 *                the trig cache that's affordable.  Glyph from an
 *                8-char gamma-encoded ramp; colour from the active
 *                theme's 8-band 256-colour palette indexed by col plus
 *                an animated offset.
 *
 * Performance  : Per pixel: ~30-100 SDF calls in the trace + 4 in the
 *                normal + 5 in AO + (Phong) 24 in soft shadow.  The
 *                three perf-aware design choices (read together as
 *                tutorials T6, T8, T10):
 *                  • 4-tap tetrahedral normal vs 6-tap differences
 *                    (saves 2 SDF evals per hit pixel)
 *                  • per-frame trig cache — cosf/sinf of scene_t
 *                    computed once per frame instead of per SDF eval
 *                    (saves ~50 K trig calls per frame)
 *                  • full-frame-per-tick rendering — frame-rate
 *                    matches displayed-image rate (no banding from
 *                    progressive row streaming)
 *
 * References   :
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer* 12(10):527-545.
 *   • Quílez, I. — "Distance Functions" + "Smooth Minimum"
 *     https://iquilezles.org/articles/distfunctions/
 *     https://iquilezles.org/articles/smin/
 *     The four primitives in §5 + the smin formula in §6 are taken
 *     directly from these articles.
 *   • Phong, B. T. (1975) — "Illumination for Computer Generated
 *     Pictures", *CACM* 18(6):311-317.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Geometry is a TREE of operations on simpler geometry.  Leaves are
 * the primitive SDFs (sphere, box, capsule, torus).  Internal nodes
 * are the operators (smin, min, max, twist, domain-fold).  Whatever
 * you build, the result is itself an SDF — sphere tracing doesn't
 * know that the "thing" it's tracing happens to be a humanoid figure
 * assembled from seven smin'd primitives.  It just calls the SDF
 * and walks the safe step.  Change the tree → different geometry.
 * Change the FIVE LEAVES of one tree to be FRACTAL primitives →
 * fractals.  The renderer is invariant.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of the renderer as a PROBE WALKER.  You — the reader — are
 * the GEOMETRY DESIGNER.  You construct a tree of operations that
 * answers, for any 3-D point, "how far am I from the surface?"  The
 * walker doesn't care what you built; it asks, walks, asks, walks
 * until it touches the surface.  Then a SHADER colours the touch.
 * Then a PIXELISER picks a glyph.  Each layer has ONE job.  Adding
 * a new scene is adding a new tree — every other layer keeps
 * working.
 *
 *      one tick of rendering, in cross-section:
 *
 *      ┌──────────────────────────────────────────────────────────┐
 *      │  app_tick → scene_t advances                              │
 *      │  scene_cache_update(scene_t) ─→ refresh cosf/sinf once    │
 *      │                                                           │
 *      │  for each pixel (ROWS_PER_TICK = 55 → full canvas):       │
 *      │    cast_pixel:                                            │
 *      │      build ray  →  sphere-march scene_map(...)            │
 *      │                          │                                │
 *      │      switch (preset):────┘                                │
 *      │        case 0: scene1_blend   (smin tree)                 │
 *      │        case 1: scene2_boolean (min/max/max(·,−·))         │
 *      │        case 2: scene3_twist   (rotate p by p.y·k → SDF)   │
 *      │        case 3: scene4_repeat  (fold p → SDF)              │
 *      │        case 4: scene5_sculpt  (smin chain of 7 primitives)│
 *      │                                                           │
 *      │      on hit: 4-tap tetrahedral normal                     │
 *      │              shade by mode (N·V / Phong / Flat)           │
 *      │              store Pixel{luma, col, N, t, steps}          │
 *      │                                                           │
 *      │  draw frame from g_fbuf (full frame coherent)             │
 *      └──────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per tick:
 *   1. animate           advance scene_t and cam_phi (paused freezes)
 *   2. cache trig        scene_cache_update(scene_t) refreshes cos/sin
 *                        of scene_t for scenes 1, 2, 3, 5 (T8)
 *
 * Per render pass (ROWS_PER_TICK = 55, so once per tick):
 *   3. snapshot camera   freeze cam basis at row 0 (also = pass start)
 *   4. for each row × col in canvas:
 *        4a. ray         primary ray from camera through cell
 *        4b. trace       sphere-march scene_map(preset, p, scene_t).d
 *                        until d < HIT_EPS (hit) or t > MAX_T (miss)
 *        4c. on hit      4-tap tetrahedral normal (T6)
 *        4d. shade       N·V / Phong / Flat (T7)
 *        4e. store       Pixel{luma, col, N, hit_t, steps, hit}
 *   5. on completion     memcpy g_fbuf → g_stable (frame ready)
 *
 * Per draw frame:
 *   6. for each cell:    read Pixel; (luma, col) → (glyph, attr) via
 *                        gamma encoding (T9); paint
 *                        OR if debug mode != NORMAL: read N / hit_t /
 *                        steps directly and paint
 *   7. HUD overlay       title + fps left, settings right (yellow);
 *                        key hint at last row (cyan)
 *
 * KEY FORMULAS
 * ────────────
 * Smooth-union (Quílez polynomial):
 *      h    = max(k − |a − b|, 0) / k
 *      smin = min(a, b) − h² · k / 4
 *
 * CSG booleans (each is one operator):
 *      union          d = min(d_a, d_b)
 *      intersection   d = max(d_a, d_b)
 *      subtraction    d = max(d_a, −d_b)
 *
 * Twist (domain warp before the SDF):
 *      angle = p.y · k
 *      p'    = (cos(angle)·p.x − sin(angle)·p.z, p.y,
 *               sin(angle)·p.x + cos(angle)·p.z)
 *      d     = sdRoundBox(p', halfsize, corner_r)
 *
 * Domain repetition (modular fold of p):
 *      p'.x = p.x − cell · round(p.x / cell)
 *      p'.z = p.z − cell · round(p.z / cell)
 *      d    = sdSphere(p', r)            // one sphere → infinite lattice
 *
 * 4-tap tetrahedral normal (Quílez):
 *      k = {(+,−,−), (−,−,+), (−,+,−), (+,+,+)}   (each scaled by ε)
 *      N = normalise(Σ kᵢ · sdf(p + kᵢ))
 *
 * Sphere SDF        f(p, r)        = |p| − r
 * Round box SDF     f(p, b, r)     = |max(|p|−b, 0)| + min(max(|p|−b), 0) − r
 * Torus SDF         f(p, R, r)     = |(|p.xz|−R, p.y)| − r
 * Capsule SDF       h = clamp((p−a)·(b−a) / |b−a|², 0, 1)
 *                   f = |p − (a + (b−a)·h)| − r
 *
 * Sphere trace with step count:
 *      t = 0,  steps = 0
 *      repeat MARCH_MAX:
 *          d = scene_map(preset, ro + t·rd, scene_t).d
 *          if d < EPS:    HIT  (return t, steps)
 *          if t > FAR:    MISS
 *          t += d · MARCH_STEP   (or MARCH_TW for twist scene)
 *          steps++
 *
 * Shading (three modes):
 *      mode = 0 (N·V)    L = KA + KD · max(0, N·V) · ao
 *      mode = 1 (Phong)  L = KA + (KD·ndl_key + KS·spec) · shadow · ao
 *                              + FILL_STR·KD·ndl_fill · ao
 *                              + RIM_STR·KD·ndl_rim
 *      mode = 2 (Flat)   L = 1.0   (col only — no shape gradient)
 *
 * Gamma encode (before glyph quantisation):
 *      v_perceptual = max(luma, 0) ^ 0.45
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • smin's `k` must be > 0 (divide-by-zero otherwise).  We special-
 *     case k < 1e-6 → fall back to plain min.  At k ≈ 0.05 the bead
 *     is barely visible; at k ≈ 1.0 primitives bulge from far away
 *     even before they meet.
 *
 *   • Twist scene: the warp breaks Lipschitz-1 (a unit step in p
 *     amplifies at large radii).  MARCH_TW = 0.60 (vs MARCH_STEP =
 *     0.85) keeps the marcher safe.  Set step_scale = 1.0 globally
 *     and the twist's surface speckles.
 *
 *   • Domain repeat: SDF is evaluated in WRAPPED space (one cell), but
 *     the col signal uses ORIGINAL p (atan2 on original coords) so the
 *     rainbow tint sweeps across the lattice rather than repeating.
 *
 *   • Camera snapshot at row 0 of each render pass: with full-frame-
 *     per-tick (T10) and snapshot-at-row-0, the camera is frozen for
 *     the tick.  Drop the snapshot and individual rows would see
 *     slightly different cameras → shearing.
 *
 *   • Soft shadow runs ONLY in Phong mode.  N·V mode treats every hit
 *     as directly lit (modulated only by AO + cos angle).  This is
 *     intentional: cast shadows would compete with the per-region col
 *     signal that N·V is meant to highlight.
 *
 *   • Flat mode: every hit pixel reaches the brightest glyph ('@').
 *     Use to inspect the col gradient without lighting interference.
 *
 *   • The 8-char ramp's slot 0 is ' ' (no-hit).  pixel_to_cell clamps
 *     hit pixels to slot ≥ 1, so even the dimmest hit paints '.' not
 *     space.
 *
 *   • CAM_DIST_MIN (1.0) prevents the camera from entering the larger
 *     primitives.  Going closer would put eye inside geometry — first
 *     SDF returns negative, marcher behaves badly.
 *
 *   • The HUD's right-aligned status string is clamped to (cols − llen)
 *     so it can't overlap the left label (title + fps).  Fps stays
 *     visible even when the settings string overflows.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press 1..5 to switch scenes.  Each should look DISTINCTLY
 *     different — not just "different shape" but "different
 *     composition" (smooth blob vs 3-column CSG vs torsion vs lattice
 *     vs humanoid).
 *
 *   • Scene 1 (Blend): the smin's k animates over time.  Watch the
 *     join between the spheres and torus thicken and thin.  Pause to
 *     freeze a particular k value.
 *
 *   • Scene 2 (Boolean): three columns.  LEFT shows union (sphere AND
 *     box visible).  CENTRE shows intersection (only the lens-shaped
 *     overlap).  RIGHT shows subtraction (sphere with a vertical hole
 *     drilled through).
 *
 *   • Scene 3 (Twist): rotated rounded box.  At twist_k = 0 it's just
 *     a box; animation drives twist_k between negative and positive,
 *     so the column visibly torsions like a barber pole.
 *
 *   • Scene 4 (Repeat): an infinite GRID of spheres with a rainbow
 *     tint sweeping across.  Resize the terminal — the lattice
 *     extends further as more cells fit.
 *
 *   • Scene 5 (Sculpt): a four-armed humanoid.  Pause and inspect —
 *     body, head, neck, belt, four arms form one CONTINUOUS surface
 *     with no visible joins.  Switch to Flat mode (l) to see the
 *     height-mapped colour gradient feet → head.
 *
 *   • Press l to cycle lighting modes.  N·V is shape-revealing
 *     (default).  Phong adds 3-point lighting + shadows + AO.  Flat
 *     removes lighting (only colour varies).
 *
 *   • Press d to cycle debug overlays.  NORMALS shows raw geometry
 *     hue.  DEPTH shows hit-distance brightness.  STEPS makes
 *     silhouettes glow (more steps = harder convergence).
 *
 *   • Press z / Z to zoom.  Geometry stays still; camera moves.
 *
 *   • Resize while running — frame redraws; HUD re-aligns; rotation
 *     stays smooth (full-frame-per-tick rendering).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — ten short answers ─────────────────────────────── *
 *
 *
 * T1: One renderer, five scenes — function-pointer dispatch
 * ─────────────────────────────────────────────────────────
 * The renderer (sphere trace + normal estimator + shader) is one
 * fixed pipeline.  What VARIES per scene is the SDF function it
 * calls — and that's a single switch dispatched on the active preset:
 *
 *      static SDF2 scene_map(int preset, Vec3 p, float t) {
 *          switch (preset) {
 *          case 0: return scene1_blend  (p, t);
 *          case 1: return scene2_boolean(p, t);
 *          ...
 *          }
 *      }
 *
 * Every other layer (cast_pixel, sdf_march, sdf_normal, shade_luma,
 * canvas_render, draw_active) is OBLIVIOUS to which scene it's
 * tracing.  Adding a sixth scene means adding ONE function and ONE
 * switch case — nothing else changes.  This is the architectural
 * payoff: COMPOSITION lives in the scenes, not in the renderer.
 *
 *
 * T2: A scene as a TREE of operations
 * ───────────────────────────────────
 * Every scene is a composition tree.  Leaves are PRIMITIVE SDFs
 * (sphere, box, capsule, torus); internal nodes are OPERATORS
 * (smin, min, max, twist, domain-fold).
 *
 *      scene1_blend tree:
 *          smin(k₂)
 *           ├─ smin(k₁)
 *           │   ├─ sdSphere(p − ⟨−sep, 0, 0⟩)
 *           │   └─ sdSphere(p − ⟨ +sep, 0, 0⟩)
 *           └─ sdTorus(p)
 *
 *      scene2_boolean tree:
 *          ARGMIN(d, col)
 *           ├─ min(sdSphere, sdRoundBox)            [tag col=0.15]
 *           ├─ max(sdSphere, sdRoundBox)            [tag col=0.50]
 *           └─ max(sdSphere, −sdCapsule)            [tag col=0.85]
 *
 * The CONCRETE TREE is the geometry.  Two scenes differ only in
 * which tree they build.  Reading scene1_blend's body in §8 you can
 * see the smin tree directly in the code — no tricks.
 *
 *
 * T3: SDF2 — bundling distance with a material signal
 * ───────────────────────────────────────────────────
 * Each scene returns SDF2 = (d, col):
 *
 *      d     signed distance — drives the marcher
 *      col   material signal in [0, 1] — picks a colour band at
 *            draw time
 *
 * Why bundle them?  Because they're chosen TOGETHER per region.  In
 * scene2_boolean, the ARGMIN over three CSG columns picks WHICH
 * column produces the closest distance — and that decision also
 * picks which colour the resulting pixel gets:
 *
 *      if (left  ≤ centre && left ≤ right) { d = left;  col = 0.15; }
 *      else if (centre ≤ right)            { d = centre; col = 0.50; }
 *      else                                { d = right; col = 0.85; }
 *
 * Without SDF2 we'd need a separate "material map" data structure
 * or a second SDF call to recover which region produced d.  Bundle
 * once at the source.
 *
 *
 * T4: Five composition operators
 * ──────────────────────────────
 *      smin           polynomial smooth-union — bulges the surface
 *                     OUTWARD into the gap between two SDFs
 *      min            hard CSG union — sharp circle where surfaces meet
 *      max            CSG intersection — keep only the OVERLAP
 *      max(a, −b)     CSG subtraction — drill b out of a
 *      twist          rotate p around y by angle = p.y · k BEFORE the SDF
 *      domain-fold    p.x −= cell·round(p.x/cell) BEFORE the SDF
 *
 * The first four work on SDF VALUES (combining d's).  The last two
 * work on the QUERY POINT itself (warping p before the SDF is
 * evaluated).  Both kinds compose freely — you can twist the query
 * point THEN evaluate a smin tree, or evaluate the smin tree THEN
 * apply a hard intersection with another SDF.  Composition is
 * generic; the renderer doesn't care which combinator you used.
 *
 *
 * T5: The four primitives + sphere tracing recap
 * ──────────────────────────────────────────────
 * Four primitives shared by all five scenes:
 *
 *      sdSphere(p, r)            f = |p| − r
 *      sdCapsule(p, a, b, r)     f = dist_to_segment(p, a, b) − r
 *      sdTorus(p, R, r)          radial-reduction: 2-D distance to ring
 *      sdRoundBox(p, b, r)       Quílez box, surface offset by r
 *
 * Sphere tracing (Hart 1996):
 *
 *      t = 0,  steps = 0
 *      repeat MARCH_MAX:
 *          d = scene_map(preset, ro + t·rd, scene_t).d
 *          if d < HIT_EPS:    HIT (return t, steps)
 *          if t > MAX_T:      MISS
 *          t += d · MARCH_STEP
 *          steps++
 *
 * `steps` is captured for the AO proxy and the DEBUG_STEPS overlay
 * (silhouette glow because grazing rays take many iterations).
 *
 *
 * T6: Tetrahedral 4-tap normal — sharper edges, fewer SDF calls
 * ─────────────────────────────────────────────────────────────
 * For ANY SDF, the gradient ∇f points OUTWARD from the surface, so:
 *
 *      N = ∇f / |∇f|
 *
 * Two ways to estimate ∇f at a hit point:
 *
 *      6-tap CENTRAL DIFFERENCES (one pair per axis):
 *          Nₓ = sdf(p+εx) − sdf(p−εx)
 *          Nᵧ = sdf(p+εy) − sdf(p−εy)
 *          N_z = sdf(p+εz) − sdf(p−εz)
 *      → 6 SDF evaluations per hit pixel.
 *
 *      4-tap TETRAHEDRAL (Quílez):
 *          k = {(+,−,−), (−,−,+), (−,+,−), (+,+,+)}   (× ε)
 *          N = normalise(Σ kᵢ · sdf(p + kᵢ))
 *      → 4 SDF evaluations per hit pixel.
 *
 * 4-tap saves 2 SDF evals AND produces a CLEANER normal at sharp
 * edges — because no two tetrahedron offsets share an axis, the
 * normal at an edge isn't smeared across both sides.  For our scene
 * 5 (humanoid), the rounded-box body has sharp edges at the corners
 * where smooth-min joins arms; 4-tap keeps those edges crisp.
 *
 *
 * T7: Three lighting modes — N·V, Phong, Flat
 * ───────────────────────────────────────────
 * Three different "ways of seeing" the same geometry, toggled with l:
 *
 *      N·V (mode 0, default)
 *        L = KA + KD · max(0, N·V) · ao
 *        Brightness = how directly the surface faces the CAMERA.
 *        No directional light → no global gradient competing with the
 *        col field.  Shape and depth are the primary visual signals.
 *
 *      Phong (mode 1)
 *        L = KA + (KD·ndl_key + KS·spec) · shadow · ao
 *              + FILL_STR · KD · ndl_fill · ao
 *              + RIM_STR  · KD · ndl_rim
 *        Three-point rig: orbiting key + fixed cool fill + back rim.
 *        Shadows + AO darken occluded regions.  More dramatic;
 *        slower (24-step shadow march per hit pixel).
 *
 *      Flat (mode 2)
 *        L = 1.0
 *        Pure col display.  Use to judge the colour gradient without
 *        any shape interference.
 *
 *
 * T8: Per-frame trig hoisting — the SceneCache
 * ────────────────────────────────────────────
 * Scenes 1, 2, 3, 5 each call cosf/sinf on scene_t inside the SDF.
 * scene_t is CONSTANT during a frame, but the SDF gets called 30-65
 * times per pixel (trace + 4-tap normal + AO + maybe shadow).  At
 * thousands of pixels per frame, that's tens of thousands of
 * IDENTICAL trig computations per frame:
 *
 *      Without cache:  ~50 SDF evals × ~1500 hit pixels × ~6 trig calls
 *                      ≈ 450 K trig calls per frame
 *
 *      With cache:     6 trig calls per frame (computed in
 *                      scene_cache_update, read from globals in
 *                      scenes)
 *
 * Implementation: a SceneCache global, populated ONCE per tick by
 * scene_cache_update(scene_t), holds:
 *
 *      scene1_sep, scene1_k                 (blend)
 *      scene2_rot_c, scene2_rot_s           (boolean rotation)
 *      scene3_twist_k                       (twist strength)
 *      scene5_rot_c, scene5_rot_s           (sculpt rotation)
 *
 * The `t` parameter on scene_map and the scene functions stays
 * (callable by old API) but is ignored — `(void)t;` at the top of
 * each scene that uses cached values.
 *
 *
 * T9: Gamma encoding for perceptual ramp uniformity
 * ─────────────────────────────────────────────────
 * Phong/N·V luma is computed in LINEAR LIGHT space.  The terminal
 * (and the eye) responds NON-LINEARLY: a value of 0.5 looks like
 * roughly 0.22 brightness without correction.  Without correction
 * most luma values cluster at the bright end of the 8-char ramp
 * ('#', '@'); the lower 5 characters (' ', '.', ':', '=', '+') see
 * almost no use.
 *
 * Apply a perceptual encoding before glyph quantisation:
 *
 *      v_perceptual = v_linear ^ 0.45     (≈ 1/2.2, the gamma)
 *
 * Now every step of the ramp gets approximately equal usage.  The
 * encoding lives in pixel_to_cell() in §3.
 *
 *
 * T10: Full-frame-per-tick rendering — coherent motion
 * ────────────────────────────────────────────────────
 * The previous design was PROGRESSIVE: each tick rendered a small
 * number of rows (ROWS_PER_TICK = 4) into g_fbuf; the rest of the
 * displayed image came from g_stable (last completed frame).  Full
 * frame finished after ch / ROWS_PER_TICK ticks.
 *
 * Trade-off the progressive design made:
 *      pro  Tick rate stays high (each tick is cheap).
 *      con  Effective new-frame rate is much lower than tick rate.
 *      con  Camera advances every tick, but only a band of ROWS_PER_
 *           TICK rows shows the new camera each tick → motion appears
 *           in JERKY 4-row STEPS.
 *
 * For continuously rotating scenes the cons win.  Solution: render
 * the full frame each tick.  ROWS_PER_TICK = 55 (≥ CANVAS_MAX_H) →
 * the inner loop processes the entire canvas before returning.
 * Tick rate may drop slightly on huge terminals, but EFFECTIVE
 * frame rate equals tick rate (always coherent).
 *
 * Combined with T8 (trig cache) and T6 (4-tap normal), full-frame
 * rendering stays comfortably above 30 fps on typical terminals.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 canvas + frame layout.
 *
 * ROWS_PER_TICK = CANVAS_MAX_H means every tick renders a full frame
 * (T10).  Setting it lower switches to a progressive renderer at the
 * cost of jerky rotation; the trig cache (§7) makes full-frame
 * affordable.
 */
#define CANVAS_MAX_W 220
#define CANVAS_MAX_H 55
#define ROWS_PER_TICK 55   /* = CANVAS_MAX_H — full frame per tick */
#define CELL_ASPECT 2.1f   /* terminal cell h / w                  */
#define FOV_HALF_TAN 0.57f /* tan(30°): 60° vertical FOV           */

/* §1.2 sphere-trace tunables.  MARCH_TW is a smaller step for the twist
 * scene (its DE is non-Lipschitz; full step over-shoots — see T5/§10). */
#define MARCH_MAX 120
#define MARCH_EPS 0.0015f
#define MARCH_FAR 18.0f
#define MARCH_STEP 0.85f
#define MARCH_TW 0.60f

/* §1.3 normals + shadow + AO. */
#define NORM_H 0.007f /* tetrahedral normal epsilon (T6)       */
#define SH_STEPS 24
#define SH_K 12.0f
#define SH_FLOOR 0.12f /* never fully-black shadow             */
#define AO_STEPS 5
#define AO_STEP 0.12f
#define AO_DECAY 0.72f

/* §1.4 Phong shading coefficients (used in mode 1). */
#define KA 0.10f
#define KD 0.72f
#define KS 0.26f
#define SHIN 24.0f
#define FILL_STR 0.14f /* fill-light strength                  */
#define RIM_STR 0.09f  /* rim-light strength                   */

/* §1.5 camera + zoom limits.  CAM_DIST_MIN keeps the eye outside the
 * larger primitives; closer than ~1.0 the marcher starts inside. */
#define CAM_DIST_DEF 2.8f
#define CAM_DIST_MIN 1.0f
#define CAM_DIST_MAX 10.0f
#define CAM_ZOOM_STEP 0.20f
#define CAM_THETA_DEF 0.38f
#define CAM_PHI_DEF 0.0f
#define CAM_ORBIT_DEF 0.30f

/* §1.6 colour pair indices.
 *      CP_HUD     bright yellow + bold — top status row 0
 *      CP_HINT    bright cyan   + bold — bottom key hint row
 *      CP_BASE..  +0..+(N_THEMES·GRAD_N − 1) — gradient pairs per theme
 */
#define GRAD_N 8
#define N_THEMES 5
#define CP_HUD 1
#define CP_HINT 2
#define CP_BASE 20

/* §1.7 luminance ramp.  Slot 0 = ' ' (no-hit); slots 1..7 are the
 * actual visible glyphs.  Same ramp pattern as mandelbulb. */
#define BOURKE_LEN 8
static const char k_bourke[BOURKE_LEN + 1] = " .:=+*#@";

/* §1.8 debug overlays — d / D cycles between them. */
typedef enum {
  DEBUG_NORMAL = 0,  /* full lit scene (production view)        */
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

/* ── §2 clock — monotonic timer ──────────────────────────────────────── */

static double clock_now(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ── §3 color — themes + 2-pair HUD spec ─────────────────────────────── *
 *
 * Five themes × GRAD_N = 8 foreground colours each.  Design rule:
 * every step is bright enough to read on a dark background — minimum
 * brightness is a saturated mid-tone, maximum near-white.  Per the
 * CLAUDE.md theme-brightness rule, no entry sits below the bright
 * half of the 256-cube.
 *
 * pixel_to_cell applies gamma encoding (T9) before quantising luma
 * to a glyph slot.
 */

static const short k_palette[N_THEMES][GRAD_N] = {
    /* 0 Studio   gold → yellow → warm white (all r=4..5)          */
    {214, 220, 221, 226, 227, 228, 230, 231},
    /* 1 Ember    red → orange → yellow fire (r=5 throughout)      */
    {196, 202, 203, 208, 209, 214, 220, 226},
    /* 2 Arctic   cyan → sky → lavender → white                    */
    {51, 87, 123, 159, 153, 189, 225, 231},
    /* 3 Toxic    bright green → lime → yellow-green               */
    {46, 82, 118, 119, 154, 155, 190, 226},
    /* 4 Neon     magenta → violet → pink → white                  */
    {201, 165, 171, 207, 177, 213, 219, 231},
};

static int g_theme = 0;
static int g_color_offset = 0; /* rotated each tick for palette anim  */

static void colors_init(void) {
  start_color();
  use_default_colors();

  /* HUD pairs — theme-independent yellow + cyan per CLAUDE.md spec. */
  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }

  for (int th = 0; th < N_THEMES; th++)
    for (int i = 0; i < GRAD_N; i++)
      init_pair((short)(CP_BASE + th * GRAD_N + i), k_palette[th][i], -1);
}

/*
 * pixel_to_cell — map (luma, col) → (glyph, attribute).
 *
 *   GLYPH from luma.  Apply v ← v^0.45 (gamma encoding, T9) to
 *   redistribute the linear-light range over the 8-char ramp.  Slot
 *   0 (space) is reserved for no-hit; clamp visible pixels to slot
 *   ≥ 1 so even the dimmest hit paints '.' not ' '.
 *
 *   ATTRIBUTE from col + animated offset.  col is already in [0, 1];
 *   add g_color_offset (rotated each tick) for slow palette
 *   animation.  Pair index = CP_BASE + theme · GRAD_N + (col_idx +
 *   offset) mod GRAD_N.
 */
static void pixel_to_cell(float luma, float col, char *ch_out,
                          attr_t *attr_out) {
  float lg = powf(luma > 0.0f ? luma : 0.0f, 0.45f);
  int ri = (int)(lg * (float)(BOURKE_LEN - 1) + 0.5f);
  if (ri < 1)
    ri = 1;
  if (ri >= BOURKE_LEN)
    ri = BOURKE_LEN - 1;
  *ch_out = k_bourke[ri];

  int gi = (int)(col * (float)(GRAD_N - 1) + 0.5f + g_color_offset) % GRAD_N;
  if (gi < 0)
    gi += GRAD_N;
  *attr_out = COLOR_PAIR(CP_BASE + g_theme * GRAD_N + gi) | A_BOLD;
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
static inline Vec3 v3neg(Vec3 a) { return v3(-a.x, -a.y, -a.z); }
static inline Vec3 v3cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline Vec3 v3norm(Vec3 a) {
  float l = v3len(a);
  return l > 1e-9f ? v3mul(a, 1.0f / l) : v3(0, 1, 0);
}

/* ── §5 SDF primitives ───────────────────────────────────────────────── *
 *
 * Four primitives — the leaves of every scene's composition tree.
 * Each is a Quílez canonical SDF.
 *
 *   SDF2 — distance + material colour signal (T3).  Every scene
 *   returns this; the renderer reads .d to march and .col to colour.
 */
typedef struct {
  float d;
  float col;
} SDF2;

/* Sphere centred at origin, radius r. */
static float sdSphere(Vec3 p, float r) { return v3len(p) - r; }

/* Capsule — line segment from a to b, radius r. */
static float sdCapsule(Vec3 p, Vec3 a, Vec3 b, float r) {
  Vec3 ab = v3sub(b, a);
  Vec3 ap = v3sub(p, a);
  float t = v3dot(ap, ab) / v3dot(ab, ab);
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  return v3len(v3sub(p, v3add(a, v3mul(ab, t)))) - r;
}

/* Torus in the xz plane.  Major radius R (centre circle), tube r. */
static float sdTorus(Vec3 p, float R, float r) {
  float qx = sqrtf(p.x * p.x + p.z * p.z) - R;
  return sqrtf(qx * qx + p.y * p.y) - r;
}

/*
 * Round box: half-extents b, corner radius r.  Quílez exact box SDF
 * minus r to offset the surface outward (rounded corners).
 */
static float sdRoundBox(Vec3 p, Vec3 b, float r) {
  float qx = fabsf(p.x) - b.x;
  float qy = fabsf(p.y) - b.y;
  float qz = fabsf(p.z) - b.z;
  float ex = qx > 0.0f ? qx : 0.0f;
  float ey = qy > 0.0f ? qy : 0.0f;
  float ez = qz > 0.0f ? qz : 0.0f;
  float ins = fminf(fmaxf(qx, fmaxf(qy, qz)), 0.0f);
  return sqrtf(ex * ex + ey * ey + ez * ez) + ins - r;
}

/* ── §6 SDF operators — smin, twist, domain repetition ───────────────── *
 *
 * Three composition operators that produce new geometry from the
 * primitives above.  smin combines two distances; twist and
 * domain_rep_xz pre-warp the QUERY POINT before the SDF is evaluated.
 *
 * Boolean ops (union = min, intersection = max, subtraction =
 * max(a, −b)) are inlined at scene-2 call sites because they're
 * literally one operator each — wrapping them in helpers wouldn't
 * add clarity.
 */

/* smin — Quílez polynomial smooth-min (T4).  k controls the bead
 * width; k = 0 is a hard min (sharp join). */
static float smin(float a, float b, float k) {
  if (k < 1e-6f)
    return a < b ? a : b;
  float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
  return fminf(a, b) - h * h * k * 0.25f;
}

/* twist — rotate p around the Y axis by angle = p.y · k.  Bottom
 * (p.y → 0) is unrotated; top (p.y → 1) is rotated by k radians.
 * Apply BEFORE evaluating the SDF.  T4. */
static Vec3 twist(Vec3 p, float k) {
  float angle = p.y * k;
  float c = cosf(angle), s = sinf(angle);
  return v3(c * p.x - s * p.z, p.y, s * p.x + c * p.z);
}

/* domain_rep_xz — fold p into a single cell on the xz lattice.  T4. */
static Vec3 domain_rep_xz(Vec3 p, float cell) {
  p.x -= cell * roundf(p.x / cell);
  p.z -= cell * roundf(p.z / cell);
  return p;
}

/* ── §7 scene cache — per-frame trig hoisted out of the SDF hot loop ── *
 *
 * Tutorial T8 explained the win.  Each scene function used to call
 * cosf/sinf on scene_t inside its body — and was called many tens of
 * thousands of times per frame.  Hoist those calls into a global
 * struct populated ONCE per tick.
 *
 * scene_cache_update is invoked from main() right before
 * canvas_render_rows; the scenes read from g_scene_cache directly
 * (and treat the `t` parameter as decorative).
 */
typedef struct {
  /* scene 1: blend */
  float scene1_sep; /* 0.55 + 0.38 · cos(t · 0.70)         */
  float scene1_k;   /* 0.14 + 0.11 · sin(t · 0.50)         */
  /* scene 2: boolean — Y-rotation matrix entries */
  float scene2_rot_c, scene2_rot_s; /* cos/sin(t · 0.28)          */
  /* scene 3: twist — current twist strength */
  float scene3_twist_k; /* 1.85 · sin(t · 0.48)                */
  /* scene 5: sculpt — Y-rotation matrix entries */
  float scene5_rot_c, scene5_rot_s; /* cos/sin(t · 0.38)          */
} SceneCache;

static SceneCache g_scene_cache;

static void scene_cache_update(float t) {
  g_scene_cache.scene1_sep = 0.55f + 0.38f * cosf(t * 0.70f);
  g_scene_cache.scene1_k = 0.14f + 0.11f * sinf(t * 0.50f);

  float a2 = t * 0.28f;
  g_scene_cache.scene2_rot_c = cosf(a2);
  g_scene_cache.scene2_rot_s = sinf(a2);

  g_scene_cache.scene3_twist_k = 1.85f * sinf(t * 0.48f);

  float a5 = t * 0.38f;
  g_scene_cache.scene5_rot_c = cosf(a5);
  g_scene_cache.scene5_rot_s = sinf(a5);
}

/* ── §8 scene 1: blend — smooth union of sphere + sphere + torus ─────── *
 *
 * The smin tree (T2):
 *      d = smin( smin(sphere_left, sphere_right, k), torus, k · 0.75 )
 *
 * Both `sep` (sphere separation) and `k` (smin strength) animate
 * over time, so the join visibly thickens and thins.  Both come
 * from g_scene_cache (T8).
 *
 * col = radial distance from origin / 1.3 — shows how the surface
 * gradient varies from centre to edge.
 */
static SDF2 scene1_blend(Vec3 p, float t) {
  (void)t; /* trig hoisted to g_scene_cache */
  float sep = g_scene_cache.scene1_sep;
  float k = g_scene_cache.scene1_k;

  float s1 = sdSphere(v3sub(p, v3(-sep, 0.0f, 0.0f)), 0.48f);
  float s2 = sdSphere(v3sub(p, v3(sep, 0.0f, 0.0f)), 0.48f);
  float to = sdTorus(p, 0.62f, 0.17f);

  float d = smin(smin(s1, s2, k), to, k * 0.75f);

  float r = v3len(p) / 1.3f;
  float col = r > 1.0f ? 1.0f : (r < 0.0f ? 0.0f : r);
  return (SDF2){d, col};
}

/* ── §9 scene 2: boolean — union / intersection / subtraction ────────── *
 *
 * Three columns side-by-side; each applies one CSG operator (T4):
 *
 *      LEFT    d = min(sphere, box)         union
 *      CENTRE  d = max(sphere, box)         intersection
 *      RIGHT   d = max(sphere, −capsule)    subtraction
 *
 * Pick whichever column has the smallest d at this point and tag
 * with its colour band (col = 0.15 / 0.50 / 0.85).  A slow Y rotation
 * is applied to the whole scene so all sides become visible.
 */
static SDF2 scene2_boolean(Vec3 p, float t) {
  (void)t; /* trig hoisted to g_scene_cache */
  /* slow Y rotation around the whole scene */
  float c = g_scene_cache.scene2_rot_c;
  float s = g_scene_cache.scene2_rot_s;
  Vec3 pr = v3(c * p.x - s * p.z, p.y, s * p.x + c * p.z);

  /* LEFT — union of sphere and round box */
  Vec3 pl = v3sub(pr, v3(-2.0f, 0.0f, 0.0f));
  float la = sdSphere(pl, 0.60f);
  float lb = sdRoundBox(v3sub(pl, v3(0.25f, 0.25f, 0.25f)),
                        v3(0.38f, 0.38f, 0.38f), 0.05f);
  float ld = fminf(la, lb); /* min = union */

  /* CENTRE — intersection of sphere and round box */
  float ca = sdSphere(pr, 0.65f);
  float cb = sdRoundBox(pr, v3(0.46f, 0.46f, 0.46f), 0.05f);
  float cd = fmaxf(ca, cb); /* max = intersection */

  /* RIGHT — subtraction: sphere minus capsule */
  Vec3 prr = v3sub(pr, v3(2.0f, 0.0f, 0.0f));
  float ra = sdSphere(prr, 0.62f);
  float rb =
      sdCapsule(prr, v3(0.0f, -0.75f, 0.0f), v3(0.0f, 0.75f, 0.0f), 0.24f);
  float rd = fmaxf(ra, -rb); /* max(a, −b) = subtraction */

  /* Argmin over the three columns; tag with that column's colour. */
  float d, col;
  if (ld <= cd && ld <= rd) {
    d = ld;
    col = 0.15f;
  } else if (cd <= rd) {
    d = cd;
    col = 0.50f;
  } else {
    d = rd;
    col = 0.85f;
  }

  return (SDF2){d, col};
}

/* ── §10 scene 3: twist — rotated query point before SDF ─────────────── *
 *
 * Pre-warp p with twist() before evaluating sdRoundBox.  twist_k
 * animates between negative and positive over time, so the box
 * appears to torsion over its height.
 *
 * The twist breaks the SDF's Lipschitz property (a unit step in p
 * amplifies at large radii), so the marcher uses MARCH_TW (smaller
 * step) for this preset only — see sdf_march in §14.
 *
 * col = vertical position so the twist gradient is visible.
 */
static SDF2 scene3_twist(Vec3 p, float t) {
  (void)t; /* trig hoisted to g_scene_cache */
  float twist_k = g_scene_cache.scene3_twist_k;
  Vec3 tp = twist(p, twist_k);
  float d = sdRoundBox(tp, v3(0.33f, 1.10f, 0.33f), 0.08f);

  float col = (p.y + 1.1f) / 2.2f;
  col = col > 1.0f ? 1.0f : (col < 0.0f ? 0.0f : col);
  return (SDF2){d, col};
}

/* ── §11 scene 4: repeat — domain-repeated sphere lattice ────────────── *
 *
 * Wrap p into a single cell, then evaluate one sphere SDF.  Result:
 * an infinite 2-D lattice of identical spheres for the cost of one
 * sphere evaluation.
 *
 * The col signal uses the ORIGINAL p (atan2(p.z, p.x)) — captured
 * BEFORE the wrap — so the rainbow tint sweeps across the lattice
 * rather than repeating per cell.
 */
static SDF2 scene4_repeat(Vec3 p, float t) {
  (void)t;
  float cell = 2.2f;

  /* azimuth on ORIGINAL p — captured before the wrap */
  float ang = atan2f(p.z, p.x) / (2.0f * (float)M_PI) + 0.5f;

  /* fold p into a single cell */
  Vec3 pr = domain_rep_xz(p, cell);

  float d = sdSphere(pr, 0.72f);
  return (SDF2){d, ang};
}

/* ── §12 scene 5: sculpt — humanoid from seven smin'd primitives ─────── *
 *
 * Compose 8 primitives (round box body, sphere head, capsule neck,
 * torus belt, four capsule arms) with smin into one continuous
 * surface.  No visible seams: head meets body in a smooth neck
 * transition, arms emerge from the body without sharp joins.
 * Demonstrates that smooth-union produces ORGANIC GEOMETRY from
 * composition of rigid primitives.
 *
 * Slow Y rotation (cached in g_scene_cache) so all four arms become
 * visible over time.
 */
static SDF2 scene5_sculpt(Vec3 p, float t) {
  (void)t; /* trig hoisted to g_scene_cache */
  /* slow Y rotation */
  float c = g_scene_cache.scene5_rot_c;
  float s = g_scene_cache.scene5_rot_s;
  Vec3 pr = v3(c * p.x - s * p.z, p.y, s * p.x + c * p.z);

  float k = 0.10f;  /* normal blend strength               */
  float k2 = 0.06f; /* tighter blend for the belt          */

  float body = sdRoundBox(v3sub(pr, v3(0.0f, -0.35f, 0.0f)),
                          v3(0.40f, 0.44f, 0.30f), 0.14f);
  float head = sdSphere(v3sub(pr, v3(0.0f, 0.82f, 0.0f)), 0.27f);
  float neck =
      sdCapsule(pr, v3(0.0f, 0.20f, 0.0f), v3(0.0f, 0.58f, 0.0f), 0.13f);
  float belt = sdTorus(v3sub(pr, v3(0.0f, -0.62f, 0.0f)), 0.43f, 0.075f);
  float armL =
      sdCapsule(pr, v3(-0.42f, -0.10f, 0.0f), v3(-0.86f, -0.52f, 0.0f), 0.10f);
  float armR =
      sdCapsule(pr, v3(0.42f, -0.10f, 0.0f), v3(0.86f, -0.52f, 0.0f), 0.10f);
  float armF =
      sdCapsule(pr, v3(0.0f, -0.10f, 0.40f), v3(0.0f, -0.52f, 0.82f), 0.09f);
  float armB =
      sdCapsule(pr, v3(0.0f, -0.10f, -0.40f), v3(0.0f, -0.52f, -0.82f), 0.09f);

  float d = smin(body, head, k);
  d = smin(d, neck, k);
  d = smin(d, belt, k2);
  d = smin(d, armL, k);
  d = smin(d, armR, k);
  d = smin(d, armF, k);
  d = smin(d, armB, k);

  float col = (pr.y + 1.0f) / 2.2f;
  col = col > 1.0f ? 1.0f : (col < 0.0f ? 0.0f : col);
  return (SDF2){d, col};
}

/* ── §13 scene_map — dispatch by preset index (T1) ───────────────────── */

static SDF2 scene_map(int preset, Vec3 p, float t) {
  switch (preset) {
  case 0:
    return scene1_blend(p, t);
  case 1:
    return scene2_boolean(p, t);
  case 2:
    return scene3_twist(p, t);
  case 3:
    return scene4_repeat(p, t);
  default:
    return scene5_sculpt(p, t);
  }
}

/* ── §14 march — 4-tap normal, shadow, AO, trace + step count ────────── */

/*
 * sdf_normal — 4-tap tetrahedral normal (Quílez form, T6).
 *
 * Sample at four corners of a regular tetrahedron in {±ε}³:
 *      k₀ = ( ε, −ε, −ε)
 *      k₁ = (−ε, −ε,  ε)
 *      k₂ = (−ε,  ε, −ε)
 *      k₃ = ( ε,  ε,  ε)
 * Then N = normalise(Σ kᵢ · scene_map(p + kᵢ).d).
 *
 * 4 SDF evals per hit pixel, vs 6 for central differences.  No two
 * offsets share an axis → cleaner normals at sharp features (the
 * smin'd corner regions in scene 5, the box edges in scene 3).
 */
static Vec3 sdf_normal(int preset, Vec3 p, float t) {
  const float e = NORM_H;
  Vec3 k0 = v3(e, -e, -e);
  Vec3 k1 = v3(-e, -e, e);
  Vec3 k2 = v3(-e, e, -e);
  Vec3 k3 = v3(e, e, e);

  float d0 = scene_map(preset, v3add(p, k0), t).d;
  float d1 = scene_map(preset, v3add(p, k1), t).d;
  float d2 = scene_map(preset, v3add(p, k2), t).d;
  float d3 = scene_map(preset, v3add(p, k3), t).d;

  Vec3 n = v3add(v3add(v3mul(k0, d0), v3mul(k1, d1)),
                 v3add(v3mul(k2, d2), v3mul(k3, d3)));
  return v3norm(n);
}

/*
 * sdf_shadow — soft shadow ray (running-min penumbra).  Returns
 * visibility in [SH_FLOOR, 1].  SH_FLOOR keeps fully-shadowed
 * pixels readable rather than collapsing to ' '.
 */
static float sdf_shadow(int preset, Vec3 ro, Vec3 rd, float t) {
  float res = 1.0f;
  float tm = 0.025f;
  for (int i = 0; i < SH_STEPS && tm < MARCH_FAR; i++) {
    float d = scene_map(preset, v3add(ro, v3mul(rd, tm)), t).d;
    if (d < MARCH_EPS)
      return SH_FLOOR;
    float r = SH_K * d / tm;
    if (r < res)
      res = r;
    tm += d;
  }
  return res < SH_FLOOR ? SH_FLOOR : res;
}

/*
 * sdf_ao — 5-step ambient occlusion march along the surface normal.
 * Accumulates the deficit between marched distance and SDF reading;
 * larger deficit ⇒ more nearby geometry ⇒ more occlusion.
 */
static float sdf_ao(int preset, Vec3 p, Vec3 N, float t) {
  float occ = 0.0f, wt = 1.0f;
  for (int i = 1; i <= AO_STEPS; i++) {
    float dist = (float)i * AO_STEP;
    float d = scene_map(preset, v3add(p, v3mul(N, dist)), t).d;
    occ += wt * (dist - d);
    wt *= AO_DECAY;
  }
  float ao = 1.0f - 2.0f * occ;
  return ao < 0.0f ? 0.0f : (ao > 1.0f ? 1.0f : ao);
}

/*
 * sdf_march — sphere trace with step count.  Returns t at hit (≥0)
 * or -1 on miss; writes the col_value at the hit and the step count
 * via *out_steps.  Twist scene uses MARCH_TW (smaller step) because
 * its DE is non-Lipschitz (T5).
 */
static float sdf_march(int preset, Vec3 ro, Vec3 rd, float t, float *col_out,
                       int *out_steps) {
  float step_scale = (preset == 2) ? MARCH_TW : MARCH_STEP;
  float tm = 0.0f;
  int step;
  for (step = 0; step < MARCH_MAX; step++) {
    Vec3 p = v3add(ro, v3mul(rd, tm));
    SDF2 s = scene_map(preset, p, t);
    if (s.d < MARCH_EPS) {
      *col_out = s.col;
      *out_steps = step + 1;
      return tm;
    }
    if (tm > MARCH_FAR)
      break;
    tm += s.d * step_scale;
  }
  *out_steps = step;
  return -1.0f;
}

/* ── §15 shade — three lighting modes (T7) ───────────────────────────── *
 *
 *      mode 0 (N·V)     KA + KD · max(0, N·V) · ao
 *      mode 1 (Phong)   KA + (KD·ndl_key + KS·spec) · shadow · ao
 *                            + FILL_STR·KD·ndl_fill · ao
 *                            + RIM_STR·KD·ndl_rim
 *      mode 2 (Flat)    1.0  (col only, no shape gradient)
 */
static float shade_luma(int preset, Vec3 hit, Vec3 N, Vec3 V, Vec3 key_L,
                        Vec3 fill_L, Vec3 rim_L, float t, bool do_shadow,
                        bool do_ao, int light_mode) {
  if (light_mode == 2)
    return 1.0f;

  float ao = 1.0f;
  if (do_ao)
    ao = sdf_ao(preset, hit, N, t);

  if (light_mode == 0) {
    /* N·V — brightness from how directly surface faces the camera */
    float ndv = fmaxf(0.0f, v3dot(N, V));
    float luma = KA + KD * ndv * ao;
    return luma > 1.0f ? 1.0f : luma;
  }

  /* Phong (light_mode == 1) */
  float ndl_key = fmaxf(0.0f, v3dot(N, key_L));
  float ndl_fill = fmaxf(0.0f, v3dot(N, fill_L));
  float ndl_rim = fmaxf(0.0f, v3dot(N, rim_L));

  Vec3 R = v3sub(v3mul(N, 2.0f * ndl_key), key_L);
  float spec = powf(fmaxf(0.0f, v3dot(R, V)), SHIN);

  float sh = 1.0f;
  if (do_shadow) {
    Vec3 sro = v3add(hit, v3mul(N, 0.010f)); /* offset off surface */
    sh = sdf_shadow(preset, sro, key_L, t);
  }

  float luma = KA + (KD * ndl_key + KS * spec) * sh * ao +
               FILL_STR * KD * ndl_fill * ao + RIM_STR * KD * ndl_rim;
  return luma > 1.0f ? 1.0f : luma;
}

/* ── §16 cast_pixel — full per-pixel pipeline → Pixel ────────────────── *
 *
 * Per-pixel result.  Production view reads luma+col; debug overlays
 * read N / hit_t / steps directly without going through gamma+pixel
 * encoding (so the visualisation isn't blurred by perceptual
 * remapping).
 */
typedef struct {
  float luma;
  float col;
  Vec3 N;      /* surface normal at hit (for DEBUG_NORMALS)    */
  float hit_t; /* ray distance at hit   (for DEBUG_DEPTH)      */
  int steps;   /* march iterations      (for DEBUG_STEPS)      */
  bool hit;
} Pixel;

static Pixel cast_pixel(int px, int py, int cw, int ch, Vec3 cam_pos, Vec3 fwd,
                        Vec3 right, Vec3 up, int preset, float scene_t,
                        Vec3 key_L, Vec3 fill_L, Vec3 rim_L, bool do_shadow,
                        bool do_ao, int light_mode) {
  Pixel out = {0.0f, 0.0f, {0, 0, 1}, 0.0f, 0, false};

  float u = ((float)px + 0.5f) / (float)cw * 2.0f - 1.0f;
  float v = -(((float)py + 0.5f) / (float)ch * 2.0f - 1.0f);
  float aspect = ((float)ch * CELL_ASPECT) / (float)cw;

  Vec3 rd = v3norm(v3add(v3add(v3mul(right, u * FOV_HALF_TAN),
                               v3mul(up, v * FOV_HALF_TAN * aspect)),
                         fwd));

  float col_val = 0.0f;
  int steps = 0;
  float hit_t = sdf_march(preset, cam_pos, rd, scene_t, &col_val, &steps);
  out.steps = steps;
  if (hit_t < 0.0f)
    return out;

  out.hit = true;
  out.col = col_val;
  out.hit_t = hit_t;

  Vec3 hit = v3add(cam_pos, v3mul(rd, hit_t));
  Vec3 N = sdf_normal(preset, hit, scene_t);
  Vec3 V = v3norm(v3sub(cam_pos, hit));
  out.N = N;

  out.luma = shade_luma(preset, hit, N, V, key_L, fill_L, rim_L, scene_t,
                        do_shadow, do_ao, light_mode);
  return out;
}

/* ── §17 canvas — full-frame render + production draw + debug ────────── *
 *
 * Two framebuffers retained from earlier progressive design (T10):
 * g_fbuf is the in-progress buffer; g_stable is the most-recent
 * completed frame.  With ROWS_PER_TICK = CANVAS_MAX_H, each tick
 * completes one full frame — g_fbuf is fully painted and copied to
 * g_stable in a single tick.  canvas_draw paints fresh rows from
 * g_fbuf and pending rows from g_stable, but at full-frame-per-tick
 * the "pending" set is always empty.
 *
 * Camera snapshot at row 0 of every pass keeps all rows on a single
 * camera, even though cam_phi advances every tick.
 */

static Pixel g_fbuf[CANVAS_MAX_H][CANVAS_MAX_W];
static Pixel g_stable[CANVAS_MAX_H][CANVAS_MAX_W];
static int g_render_row = 0;
static bool g_dirty = true;

static Vec3 g_snap_cam, g_snap_fwd, g_snap_right, g_snap_up;

static void camera_basis(float cam_dist, float cam_theta, float cam_phi,
                         Vec3 *cam_pos, Vec3 *fwd, Vec3 *right, Vec3 *up) {
  float ct = cosf(cam_theta), st = sinf(cam_theta);
  float cp = cosf(cam_phi), sp = sinf(cam_phi);
  *cam_pos = v3mul(v3(ct * cp, st, ct * sp), cam_dist);
  *fwd = v3norm(v3neg(*cam_pos));
  /* gimbal-lock guard: swap world-up when fwd nearly aligns with it */
  Vec3 wup =
      (fabsf(v3dot(*fwd, v3(0, 1, 0))) > 0.99f) ? v3(0, 0, 1) : v3(0, 1, 0);
  *right = v3norm(v3cross(*fwd, wup));
  *up = v3cross(*right, *fwd);
}

/*
 * canvas_render_rows — process up to ROWS_PER_TICK rows into g_fbuf.
 * With ROWS_PER_TICK = 55, the inner check `g_render_row >= ch`
 * always trips after `ch` rows — so each call returns true (frame
 * complete) and copies g_fbuf → g_stable.
 */
static bool canvas_render_rows(int cw, int ch, int preset, float scene_t,
                               float cam_dist, float cam_theta, float cam_phi,
                               Vec3 key_L, Vec3 fill_L, Vec3 rim_L,
                               bool do_shadow, bool do_ao, int light_mode) {
  if (g_render_row == 0)
    camera_basis(cam_dist, cam_theta, cam_phi, &g_snap_cam, &g_snap_fwd,
                 &g_snap_right, &g_snap_up);

  for (int k = 0; k < ROWS_PER_TICK; k++) {
    int py = g_render_row;
    for (int px = 0; px < cw; px++) {
      g_fbuf[py][px] = cast_pixel(
          px, py, cw, ch, g_snap_cam, g_snap_fwd, g_snap_right, g_snap_up,
          preset, scene_t, key_L, fill_L, rim_L, do_shadow, do_ao, light_mode);
    }
    if (++g_render_row >= ch) {
      g_render_row = 0;
      memcpy(g_stable, g_fbuf, sizeof g_stable);
      return true;
    }
  }
  return false;
}

/* §17.1 production draw + three debug overlays.
 *
 * canvas_draw is the production view (gamma-encoded ramp).  Debug
 * overlays bypass the gamma path — they paint raw signals so the
 * visualisation isn't smeared by perceptual remapping.
 */

static const Pixel *canvas_row(int vy, int cw) {
  (void)cw;
  return (vy < g_render_row) ? g_fbuf[vy] : g_stable[vy];
}

/*
 * canvas_offsets — top-left of the canvas on the terminal.  Reserve
 * row 0 for the yellow status HUD and the last row for the cyan key
 * hint (CLAUDE.md spec).  Canvas occupies rows 1..rows-2.
 */
static void canvas_offsets(int cw, int ch, int cols, int rows, int *out_off_x,
                           int *out_off_y) {
  *out_off_x = (cols - cw) / 2;
  *out_off_y = 1 + (rows - 2 - ch) / 2;
  if (*out_off_y < 1)
    *out_off_y = 1;
}

static void canvas_draw(int cw, int ch, int cols, int rows) {
  int off_x, off_y;
  canvas_offsets(cw, ch, cols, rows, &off_x, &off_y);

  for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
    const Pixel *row_src = canvas_row(vy, cw);
    for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
      const Pixel *px = &row_src[vx];
      if (!px->hit)
        continue;

      char ch_c;
      attr_t attr;
      pixel_to_cell(px->luma, px->col, &ch_c, &attr);

      int tx = off_x + vx;
      int ty = off_y + vy;
      if (tx < 0 || tx >= cols || ty < 0 || ty >= rows - 1)
        continue;
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)ch_c);
      attroff(attr);
    }
  }
}

/* Simple glyph + slot for debug overlays — bypasses gamma. */
static const char DEBUG_GLYPHS[] = " .:=+*#@";
#define DEBUG_GLYPHS_N ((int)(sizeof DEBUG_GLYPHS - 1))

static char debug_glyph(float v) {
  int idx = (int)(v * (float)(DEBUG_GLYPHS_N - 1) + 0.5f);
  if (idx < 1)
    idx = 1;
  if (idx >= DEBUG_GLYPHS_N)
    idx = DEBUG_GLYPHS_N - 1;
  return DEBUG_GLYPHS[idx];
}

static attr_t debug_attr(float v) {
  int gi = (int)(v * (float)(GRAD_N - 1) + 0.5f) % GRAD_N;
  if (gi < 0)
    gi += GRAD_N;
  return COLOR_PAIR(CP_BASE + g_theme * GRAD_N + gi) | A_BOLD;
}

/* NORMALS — hue from atan2(N.x, N.z); glyph from N.y · 0.5 + 0.5. */
static void canvas_draw_normals(int cw, int ch, int cols, int rows) {
  int off_x, off_y;
  canvas_offsets(cw, ch, cols, rows, &off_x, &off_y);

  for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
    const Pixel *row_src = canvas_row(vy, cw);
    for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
      const Pixel *px = &row_src[vx];
      if (!px->hit)
        continue;

      Vec3 N = px->N;
      float azimuth = atan2f(N.x, N.z) / (2.0f * (float)M_PI) + 0.5f;
      float y_lit = N.y * 0.5f + 0.5f;
      if (y_lit < 0.0f)
        y_lit = 0.0f;
      if (y_lit > 1.0f)
        y_lit = 1.0f;

      int tx = off_x + vx;
      int ty = off_y + vy;
      if (tx < 0 || tx >= cols || ty < 0 || ty >= rows - 1)
        continue;
      attr_t attr = debug_attr(azimuth);
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(y_lit));
      attroff(attr);
    }
  }
}

/* DEPTH — hit distance t → brightness.  Closer = brighter. */
static void canvas_draw_depth(int cw, int ch, int cols, int rows,
                              float cam_dist) {
  int off_x, off_y;
  canvas_offsets(cw, ch, cols, rows, &off_x, &off_y);

  /* Bracket t by camera distance so closer hits ≈ 1. */
  float t_min = cam_dist - 1.5f;
  float t_max = cam_dist + 1.5f;
  if (t_min < 0.0f)
    t_min = 0.0f;

  for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
    const Pixel *row_src = canvas_row(vy, cw);
    for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
      const Pixel *px = &row_src[vx];
      if (!px->hit)
        continue;

      float depth_n = (t_max - px->hit_t) / (t_max - t_min);
      if (depth_n < 0.0f)
        depth_n = 0.0f;
      if (depth_n > 1.0f)
        depth_n = 1.0f;

      int tx = off_x + vx;
      int ty = off_y + vy;
      if (tx < 0 || tx >= cols || ty < 0 || ty >= rows - 1)
        continue;
      attr_t attr = debug_attr(depth_n);
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(depth_n));
      attroff(attr);
    }
  }
}

/* STEPS — march iteration count → brightness.  Silhouette glow. */
static void canvas_draw_steps(int cw, int ch, int cols, int rows) {
  int off_x, off_y;
  canvas_offsets(cw, ch, cols, rows, &off_x, &off_y);

  for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
    const Pixel *row_src = canvas_row(vy, cw);
    for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
      const Pixel *px = &row_src[vx];
      if (!px->hit)
        continue;

      float steps_n = (float)px->steps / (float)MARCH_MAX;
      if (steps_n > 1.0f)
        steps_n = 1.0f;

      int tx = off_x + vx;
      int ty = off_y + vy;
      if (tx < 0 || tx >= cols || ty < 0 || ty >= rows - 1)
        continue;
      attr_t attr = debug_attr(steps_n);
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(steps_n));
      attroff(attr);
    }
  }
}

/* ── §18 app — main loop, signals, key handling ──────────────────────── */

static const char *k_preset_names[5] = {"1:Blend", "2:Boolean", "3:Twist",
                                        "4:Repeat", "5:Sculpt"};
static const char *k_theme_names[N_THEMES] = {"Studio", "Ember", "Arctic",
                                              "Toxic", "Neon"};

typedef struct {
  int cw, ch; /* canvas dimensions in characters         */
  int preset; /* current scene index 0..4                */

  float cam_dist;
  float cam_theta;
  float cam_phi;
  float orbit_spd;

  bool paused;
  bool do_shadow;
  bool do_ao;
  int light_mode;       /* 0=N·V  1=Phong  2=Flat              */
  DebugMode debug_mode; /* 0=NORMAL 1=NORMALS 2=DEPTH 3=STEPS  */

  float scene_t;     /* time fed to scene functions         */
  float color_phase; /* accumulates for palette animation   */

  bool quit;
} App;

static volatile sig_atomic_t g_resize = 0;
static void on_sigwinch(int sig) {
  (void)sig;
  g_resize = 1;
}

static void app_init(App *a, int cols, int rows) {
  memset(a, 0, sizeof *a);
  a->cw = (cols > CANVAS_MAX_W) ? CANVAS_MAX_W : cols;
  /* Reserve TWO rows for the HUD: row 0 (yellow status) + last row
   * (cyan hint).  Canvas occupies rows 1..rows-2. */
  a->ch = (rows - 2 > CANVAS_MAX_H) ? CANVAS_MAX_H : rows - 2;
  a->preset = 0;
  a->cam_dist = CAM_DIST_DEF;
  a->cam_theta = CAM_THETA_DEF;
  a->cam_phi = CAM_PHI_DEF;
  a->orbit_spd = CAM_ORBIT_DEF;
  a->do_ao = true;
  a->do_shadow = false; /* off by default for speed */
  a->light_mode = 0;    /* N·V default */
  a->debug_mode = DEBUG_NORMAL;

  memset(g_fbuf, 0, sizeof g_fbuf);
  memset(g_stable, 0, sizeof g_stable);
  g_render_row = 0;
  g_dirty = true;

  camera_basis(a->cam_dist, a->cam_theta, a->cam_phi, &g_snap_cam, &g_snap_fwd,
               &g_snap_right, &g_snap_up);
}

static void app_resize(App *a, int cols, int rows) {
  a->cw = (cols > CANVAS_MAX_W) ? CANVAS_MAX_W : cols;
  a->ch = (rows - 2 > CANVAS_MAX_H) ? CANVAS_MAX_H : rows - 2;
  g_render_row = 0;
  g_dirty = true;
}

static void app_handle_key(App *a, int ch) {
  switch (ch) {
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
    if (a->preset != ch - '1') {
      a->preset = ch - '1';
      g_dirty = true;
      a->scene_t = 0.0f;
    }
    break;
  case 't':
    g_theme = (g_theme + 1) % N_THEMES;
    break;
  case 'p':
  case ' ':
    a->paused = !a->paused;
    break;
  case 's':
    a->do_shadow = !a->do_shadow;
    g_dirty = true;
    break;
  case 'o':
    a->do_ao = !a->do_ao;
    g_dirty = true;
    break;
  case 'l':
    a->light_mode = (a->light_mode + 1) % 3;
    g_dirty = true;
    break;
  case 'd':
    a->debug_mode = (DebugMode)((a->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    a->debug_mode =
        (DebugMode)((a->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
    break;
  case '+':
  case '=':
    a->orbit_spd += 0.05f;
    break;
  case '-':
    a->orbit_spd -= 0.05f;
    if (a->orbit_spd < 0.0f)
      a->orbit_spd = 0.0f;
    break;
  case 'z':
    /* zoom IN — closer camera, smaller cam_dist */
    a->cam_dist -= CAM_ZOOM_STEP;
    if (a->cam_dist < CAM_DIST_MIN)
      a->cam_dist = CAM_DIST_MIN;
    g_dirty = true;
    break;
  case 'Z':
    /* zoom OUT — farther camera, larger cam_dist */
    a->cam_dist += CAM_ZOOM_STEP;
    if (a->cam_dist > CAM_DIST_MAX)
      a->cam_dist = CAM_DIST_MAX;
    g_dirty = true;
    break;
  case 'r':
    a->cam_dist = CAM_DIST_DEF;
    a->cam_theta = CAM_THETA_DEF;
    a->cam_phi = CAM_PHI_DEF;
    a->orbit_spd = CAM_ORBIT_DEF;
    g_dirty = true;
    break;
  case 'q':
  case 27: /* ESC */
    a->quit = true;
    break;
  default:
    break;
  }
}

static void app_tick(App *a, float dt) {
  if (a->paused)
    return;

  a->scene_t += dt;
  a->cam_phi += a->orbit_spd * dt;

  /* palette animation: g_color_offset shifts slowly through GRAD_N */
  a->color_phase += 0.4f * dt;
  g_color_offset = (int)(a->color_phase) % GRAD_N;
}

/*
 * draw_active — dispatch to production view or one of three debug
 * overlays.  Production view goes through pixel_to_cell (gamma +
 * theme); overlays use a simpler direct mapping.
 */
static void draw_active(const App *a, int cols, int rows) {
  switch (a->debug_mode) {
  case DEBUG_NORMAL:
    canvas_draw(a->cw, a->ch, cols, rows);
    break;
  case DEBUG_NORMALS:
    canvas_draw_normals(a->cw, a->ch, cols, rows);
    break;
  case DEBUG_DEPTH:
    canvas_draw_depth(a->cw, a->ch, cols, rows, a->cam_dist);
    break;
  case DEBUG_STEPS:
    canvas_draw_steps(a->cw, a->ch, cols, rows);
    break;
  default:
    canvas_draw(a->cw, a->ch, cols, rows);
    break;
  }
}

/*
 * draw_hud — CLAUDE.md HUD spec.
 *   row 0          CP_HUD  (yellow + bold) — title + fps left, status right
 *   row rows-1     CP_HINT (cyan   + bold) — key hint
 *
 * FPS lives in the LEFT label so it stays visible even when the
 * settings status string is wider than the terminal.
 */
static void draw_hud(const App *a, double fps, int cols, int rows) {
  char left[48];
  snprintf(left, sizeof left, " SDF GALLERY  %5.1f fps ", fps);
  int llen = (int)strlen(left);

  char status[220];
  snprintf(status, sizeof status,
           " scene:%s  theme:%s  light:%s  debug:%s  "
           "shadow:%s  ao:%s  zoom:%.2f  orbit:%.2f  %s ",
           k_preset_names[a->preset], k_theme_names[g_theme],
           a->light_mode == 1 ? "Phong" : (a->light_mode == 2 ? "Flat" : "N·V"),
           DEBUG_MODE_NAMES[a->debug_mode], a->do_shadow ? "on" : "off",
           a->do_ao ? "on" : "off", (double)a->cam_dist, (double)a->orbit_spd,
           a->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  /* clamp so status doesn't overlap the left label */
  int max_slen = cols - llen;
  if (max_slen < 0)
    max_slen = 0;
  if (slen > max_slen)
    slen = max_slen;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, 0, "%s", left);
  if (slen > 0)
    mvprintw(0, cols - slen, "%.*s", slen, status);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* Bottom row — cyan key hint. */
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc:pause  1-5:scene  l:light  d/D:debug  "
           "t:theme  s:shadow  o:ao  z/Z:zoom  +/-:orbit  r:reset ");
  clrtoeol();
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

int main(void) {
  signal(SIGWINCH, on_sigwinch);

  initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  colors_init();

  int cols, rows;
  getmaxyx(stdscr, rows, cols);

  App a;
  app_init(&a, cols, rows);

  double t_prev = clock_now();
  double fps_window = 0.0;  /* accumulated dt within window */
  int fps_frames = 0;       /* frames in current window     */
  double fps_display = 0.0; /* most recent measured fps     */

  while (!a.quit) {
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      app_resize(&a, cols, rows);
    }

    int ch;
    while ((ch = getch()) != ERR)
      app_handle_key(&a, ch);

    double t_now = clock_now();
    float dt = (float)(t_now - t_prev);
    if (dt > 0.1f)
      dt = 0.1f;
    t_prev = t_now;

    app_tick(&a, dt);

    /* FPS measurement window — refresh every ~0.5 s. */
    fps_frames++;
    fps_window += dt;
    if (fps_window >= 0.5) {
      fps_display = (double)fps_frames / fps_window;
      fps_frames = 0;
      fps_window = 0.0;
    }

    /* 3-point lighting directions (used in Phong mode). */
    float kang = a.scene_t * 0.40f;
    Vec3 key_L = v3norm(v3(cosf(kang) * 0.70f, 0.65f, sinf(kang) * 0.70f));
    Vec3 fill_L = v3norm(v3(-1.5f, 0.50f, -1.2f));
    Vec3 rim_L = v3norm(v3(0.0f, -0.40f, -1.0f));

    if (g_dirty) {
      g_render_row = 0;
      memset(g_fbuf, 0, sizeof(Pixel) * (size_t)a.ch * CANVAS_MAX_W);
      g_dirty = false;
    }

    /* Hoist per-frame trig out of every SDF call (T8). */
    scene_cache_update(a.scene_t);

    canvas_render_rows(a.cw, a.ch, a.preset, a.scene_t, a.cam_dist, a.cam_theta,
                       a.cam_phi, key_L, fill_L, rim_L, a.do_shadow, a.do_ao,
                       a.light_mode);

    erase();
    draw_active(&a, cols, rows);
    draw_hud(&a, fps_display, cols, rows);
    wnoutrefresh(stdscr);
    doupdate();

    struct timespec sl = {0, 16000000L};
    nanosleep(&sl, NULL);
  }

  endwin();
  return 0;
}
