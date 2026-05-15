/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * raymarcher_cube.c — a 3-D Phong-shaded ASCII cube
 *
 * DEMO: A cube spins continuously while a light orbits around it.
 *       Every terminal cell is one ray traced into a 3-D scene.
 *       Same skeleton as raymarcher.c (sphere) — only the SDF
 *       (Signed Distance Function) changes from sphere to cube.
 *
 * Study alongside: raymarcher/raymarcher.c — read it first if
 *       sphere tracing is unfamiliar.  This file is the natural
 *       "next primitive" once the sphere version makes sense.
 *
 * Section map:
 *   §1  config        — every tunable named, no magic numbers later
 *   §2  clock         — monotonic ns timer + sleep
 *   §3  color         — themes + HUD pairs (CLAUDE.md HUD spec)
 *   §4  vec3          — 3-D math, value types
 *   §5  box SDF       — the cube's distance function
 *   §6  rotate point  — the trick that makes the cube appear to spin
 *   §7  scene SDF     — composition: rotate, then box
 *   §8  normal        — tetrahedral 4-tap finite difference
 *   §9  trace         — sphere-tracing march loop (Hart 1996)
 *   §10 shade         — Phong: ambient + diffuse + specular
 *   §11 cast_ray      — one pixel's full pipeline → Hit
 *   §12 canvas        — the Hit framebuffer (one Hit per pixel)
 *   §13 render        — fill the Hit array
 *   §14 draw          — production overlay (Hit → glyph + colour)
 *   §15 debug         — three educational overlays
 *   §16 scene         — Scene struct + light + tick
 *   §17 screen        — ncurses init + HUD + present
 *   §18 app           — main loop, signals, key handling
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   ]  [      spin faster / slower
 *   =  -      grow / shrink the cube
 *   l / L     light orbit faster / slower
 *   z / Z     zoom in / out (camera closer / farther)
 *   t / T     next / previous colour theme
 *               (CLASSIC / AMBER / MATRIX / NEON / ICE / COPPER)
 *   d / D     cycle debug overlay
 *               (NORMAL / NORMALS / DEPTH / STEPS)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/raymarcher_cube.c \
 *       -o cube -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file teaches its own algorithm — read top to bottom.
 *
 *   • CONCEPTS         names the algorithm and gives references.
 *   • MENTAL MODEL     intuition + an ASCII diagram of the trace loop.
 *   • GUIDED TUTORIAL  eight short answers to the questions a learner
 *                      hits while reading the code (data-flow order:
 *                      pixel → ray → trace → hit → shade → glyph).
 *   • §1..§18          the actual code, each section short and focused.
 *
 * If you have ten minutes: read the GUIDED TUTORIAL.  By the end the
 * §-sections feel like reviewing notes, not learning new material.
 *
 * Math notation used in the code:
 *      p   — a 3-D point (the SDF input)
 *      ro  — ray origin
 *      rd  — ray direction (unit vector)
 *      t   — distance the ray has marched
 *      N   — unit surface normal
 *      L   — unit vector from hit point toward the light
 *      V   — unit vector from hit point toward the camera
 *      R   — light direction reflected about N
 *      h   — cube half-extent (so the cube spans -h..+h on each axis)
 *
 * Long names appear in struct fields and orchestrator functions
 * (`cast_ray`, `phong_shade`, `surface_normal`).  Short math letters
 * appear inside formulas where the equation is one line away.
 *
 * Background you need:
 *   • basic vector arithmetic (add, dot product, length, normalise)
 *   • sphere tracing — read raymarcher.c first; same march loop.
 * No matrix theory required — rotations are written out as cosθ/sinθ.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : SPHERE TRACING (Hart 1996) of a BOX SDF (Quílez).
 *                A Signed Distance Function returns "how far is point
 *                p from the surface" — positive outside, negative
 *                inside, zero on the surface.  For a cube this can be
 *                written in two lines (§5).  Sphere tracing turns the
 *                SDF into a renderer: at each ray step, ask the SDF
 *                for distance, walk exactly that far, repeat.
 *
 * Data         : Stateless math (Vec3 + SDF + trace + shade) on the
 *                hot path.  Each pixel produces one `Hit` struct
 *                (hit / hit_point / normal / intensity / t / steps).
 *                The `Canvas` stores `Hit[w*h]` so render and draw
 *                are cleanly decoupled — four overlays (production +
 *                three debug views) read the SAME Hit array.
 *
 * Rendering    : One ray per terminal cell.  Glyph from the 13-char
 *                ramp " .,:;+=oxOX#@".  Colour from the active
 *                theme's 8-band luma palette.  Aspect correction in
 *                the ray direction so the cube renders square on
 *                terminal cells that are roughly twice as tall as
 *                they are wide.
 *
 * Performance  : ~5 SDF evaluations per hit pixel (1 trace step at
 *                hit + 4 normal taps).  ~80 trace steps cap per ray.
 *                On a 80×24 terminal that's well under a million
 *                SDF evals per frame — trivial at 60 fps.  Cost is
 *                linear in terminal area.
 *
 * References   :
 *   • Hart, J. C. (1996) "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer* 12(10):527-545.  The paper that introduced this
 *     style of marching.
 *   • Phong, B. T. (1975) "Illumination for Computer Generated
 *     Pictures", *CACM* 18(6):311-317.  The lighting model in §10.
 *   • Quílez, I. — "Distance Functions" (article catalogue):
 *     https://iquilezles.org/articles/distfunctions/
 *     The exact box SDF used in §5 is item #2.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The cube doesn't move.  Ever.  It sits at the origin in some
 * private chamber where it always looks "axis-aligned, sides of
 * length 2h".  To make it APPEAR to spin, the renderer rotates the
 * QUESTION instead of the cube: before asking the SDF "how far am
 * I from your surface?", it rotates the query point by the cube's
 * current orientation angles.  The same "rotate the question, not
 * the answer" trick lets one SDF describe an infinite family of
 * orientations / scales / translations of the same shape.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the cube as a sleeping librarian who only answers in their
 * native dialect: "I know boxes that sit at the origin facing forward.
 * Tell me where you are in MY frame, and I'll tell you the distance
 * to my nearest surface."  The renderer's job is to translate every
 * world-space ray sample into the librarian's dialect — a pure
 * coordinate change — before posing the question.  Spinning the
 * cube means changing the translation, not waking the librarian.
 *
 * One frame of trace, viewed in cross section:
 *
 *        camera
 *          ●─────────────────────────────────────►  ray
 *          │\                                       direction
 *          │ \  d₁ = SDF(p₁)  →  big safe step
 *          │  ●·····················●
 *          │                       p₂  d₂ = SDF(p₂)  →  smaller step
 *          │                       ●·······●
 *          │                              p₃  d₃ < ε  →  HIT
 *          │                              ◍─────┐
 *          │                              │ N   │  surface_normal()
 *          │                              │ ↑   │  via 4-tap finite
 *          │                              │     │  difference
 *          │                              └─────┘
 *          │                          ▲    ▲    ▲
 *          │                        cube   cube cube
 *          │                        face   edge corner
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per frame:
 *   1. animate    accumulate rotation and time (paused freezes both)
 *   2. position   light orbits in world space (Lissajous curve)
 *
 * Once per pixel:
 *   3. ray       build a primary ray from camera through the cell
 *   4. trace     sphere-march until d < ε (hit) or t > max (miss)
 *   5. normal    if hit, sample SDF at 4 tetrahedral offsets, normalise
 *   6. shade     Phong: ambient + diffuse + specular → intensity ∈ [0,1]
 *
 * Once per cell:
 *   7. paint     intensity → glyph + theme colour (or one of three
 *                debug overlays if d/D was pressed)
 *
 * KEY FORMULAS
 * ────────────
 * Box SDF (Quílez, exact):
 *      q = |p| − h                   (component-wise abs, then subtract)
 *      d = |max(q, 0)| + min(max(q.x, q.y, q.z), 0)
 *          └ outside ─┘   └ inside (negative inside the box) ──┘
 *
 * Rotation applied to the query point:
 *      R_y(θ):  x' =  x·cosθ + z·sinθ
 *               z' = −x·sinθ + z·cosθ
 *      R_x(φ):  y' =  y·cosφ − z·sinφ
 *               z' =  y·sinφ + z·cosφ
 *
 * Tetrahedral normal (4 SDF samples, no shared-axis pair):
 *      k = {(+,−,−), (−,−,+), (−,+,−), (+,+,+)} · ε
 *      N = normalise( Σ kᵢ · SDF(p + kᵢ) )
 *
 * Phong intensity at hit point H, with surface normal N:
 *      L = normalise(light − H)
 *      V = normalise(camera − H)
 *      R = 2·(N·L)·N − L
 *      I = KA + KD·max(0, N·L) + KS·max(0, R·V)^SHIN     (clamped to [0,1])
 *
 * Pixel → ray (per cell at column col, row row, canvas w×h):
 *      u =  (col + 0.5) / w · 2 − 1
 *      v = −(row + 0.5) / h · 2 + 1
 *      rd = normalise(u·F, v·F·aspect, −1),  F = tan(FOV/2)
 *      aspect = h · CELL_ASPECT / w           (≈ 2 on a square terminal)
 *
 * Light orbit (Lissajous, never enters the cube):
 *      α = time · light_spd
 *      L(α) = (R_x·cos α,  Y₀ + Y_amp·sin(rate_y · α),  R_z·sin α + Z₀)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Camera inside the cube.  If the user grows the cube past the
 *     camera distance, every ray's first SDF returns negative.  The
 *     marcher then either stalls or returns instantly — visible as
 *     a fully-painted screen.  CAM_Z_MIN (2.6) and SIZE_MAXX (2.5)
 *     are chosen so this can't happen unless the user actively zooms
 *     into a maximally-grown cube.
 *
 *   • Tiny cube vs. epsilon.  At SIZE_MIN (0.15) the cube is small
 *     enough that RM_NORM_EPS (0.001) starts to matter — normals
 *     get a touch noisier, edges chamfer slightly.  Visible if you
 *     pause at small size.
 *
 *   • Trace starts at t = RM_T_START = 0.5, not 0.  This skips the
 *     "ray emerges from inside the camera plane" case where the
 *     first SDF could spuriously trigger HIT_EPS at grazing angles.
 *     Setting it to 0.0 produces broken silhouettes near the cube.
 *
 *   • Rotation sign quirk.  rotate_query_point applies +ry / +rx to
 *     the QUERY POINT.  The canonical inverse (for "cube spins by
 *     +ry") would use −ry / −rx.  Net effect: the cube spins in the
 *     opposite direction from the angle's sign.  Since the spin
 *     speeds are positive constants the user just sees "a spinning
 *     cube" — the sign quirk has no UX impact, but a beginner
 *     deriving the math from scratch will get the OTHER direction.
 *
 *   • Pause freezes everything (rotation, light, time accumulator).
 *     Render still recasts every pixel — image is still, FPS is
 *     valid.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Pause and look at one face: it should be a UNIFORM colour
 *     band.  All face normals are exactly ±X, ±Y, or ±Z, so Phong
 *     intensity is constant across a face.  A gradient ACROSS one
 *     face means surface_normal is being smeared across an edge —
 *     check that the tetrahedral sample offsets don't share an axis.
 *
 *   • Press d to switch to NORMALS overlay.  You should see up to
 *     six discrete colour bands (one per face direction) with hard
 *     edges between them.  This visually confirms "the cube is six
 *     flat faces, no curvature".
 *
 *   • Press d again for DEPTH.  Closer hits are brighter — a face
 *     closer to the camera should look brighter than a face farther
 *     away on the same frame.
 *
 *   • Press d again for STEPS.  The silhouette glows because grazing
 *     rays take many march iterations before they decide to miss.
 *
 *   • Press = several times to reach SIZE_MAXX, then z to zoom in.
 *     Watch the face normals stay flat as the cube grows / fills the
 *     screen — confirms the shape is a true cube, not a sphere.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — eight questions a learner asks ────────────────── *
 *
 * Each tutorial is a question + a short answer + a worked example.
 * They follow the data flow through one pixel: pixel → ray → trace →
 * hit → shade → glyph.
 *
 *
 * Q1: How does a flat terminal cell turn into a 3-D ray?
 * ──────────────────────────────────────────────────────
 * Each cell is a question: "if I shoot a ray from the camera through
 * the centre of this cell, what does it hit?"  Building the ray:
 *
 *      u = (col + 0.5) / cw · 2 − 1     in [−1, +1]; +0.5 = cell centre
 *      v = (row + 0.5) / ch · 2 − 1     same, then NEGATED so row 0 is top
 *      ray_direction = normalise(u·F, v·F·aspect, −1)
 *      where F = tan(FOV/2); −1 means "looking down −Z"
 *
 * The `aspect` factor is crucial.  Terminal cells are roughly twice
 * as tall as they are wide; without compensation, vertical ray
 * spacing is "twice as much physical space per cell" — a sphere
 * would render as a tall ellipse, a cube as a tall rectangle.
 * Multiplying v by aspect = (rows · CELL_ASPECT / cols) cancels it.
 *
 *      Without aspect fix:                 With aspect fix:
 *      ┌───┐                               ┌─┐
 *      │   │   ← cube is tall              │ │   ← cube is square
 *      │   │     because cells are tall    └─┘
 *      └───┘
 *
 *
 * Q2: We have a ray.  How do we know what it hits?
 * ────────────────────────────────────────────────
 * Not by checking "does this ray intersect this cube?" — that's the
 * triangle-mesh approach.  Instead the cube is described by an
 * implicit function f(p) — the SDF — that returns the SIGNED
 * distance from p to the cube's surface (negative inside, positive
 * outside).  Given f, one number drives everything: at any point
 * along the ray, f tells us the largest safe step we can take.
 *
 *      ray ●─→
 *      │    p₁     f(p₁) = 1.4   →   step 1.4 forward
 *      │     ●───────────────●
 *      │                     p₂   f(p₂) = 0.6   →   step 0.6
 *      │                     ●─────●
 *      │                          p₃   f(p₃) = 0.001 < ε   →   HIT
 *
 * The step is GUARANTEED safe because f is a true distance — nothing
 * can be closer than f.  After log-many iterations the ray converges
 * to the surface or escapes (t > MAX_DIST).  Algorithm in code:
 *
 *      t = RM_T_START
 *      repeat up to RM_MAX_STEPS times:
 *          d = f(ray_origin + t · ray_direction)
 *          if d < HIT_EPS:    return t      (HIT)
 *          if t > MAX_DIST:   return -1     (MISS)
 *          t += d
 *
 *
 * Q3: How do we describe a cube as a distance function?
 * ─────────────────────────────────────────────────────
 * For a cube of half-extent h centred at the origin, the surface is
 * the set of points where max(|x|, |y|, |z|) = h.  Distance from any
 * point p to that surface depends on which region p is in:
 *
 *      ┌─────────────────────────────────────────────┐
 *      │  · corner ·                  ·  ·         · │
 *      │   ╲                                          │
 *      │    ╲   region OUTSIDE                       │
 *      │     ╲   ┌────────────────┐                  │
 *      │   edge  │  region        │   edge           │
 *      │     ╱   │  INSIDE        │     ╲            │
 *      │    ╱    │   (negative)   │      ╲           │
 *      │   ╱     └────────────────┘       ╲          │
 *      │  · corner                          · corner │
 *      └─────────────────────────────────────────────┘
 *
 *      OUTSIDE near a face   →  perpendicular distance to that face
 *      OUTSIDE near an edge  →  distance to the line segment
 *      OUTSIDE near a corner →  distance to the vertex
 *      INSIDE                →  negative perpendicular distance
 *                               to the nearest face
 *
 * Quílez's trick handles all four cases in two lines.  See Q4.
 *
 *
 * Q4: How does the box SDF formula actually work?
 * ───────────────────────────────────────────────
 *      q = |p| − h
 *      d = |max(q, 0)| + min(max(q.x, q.y, q.z), 0)
 *
 * Step by step.  First, |p| is COMPONENT-WISE absolute value.  This
 * folds all 8 octants of space into the +X, +Y, +Z octant — the
 * cube is symmetric, so we only need to think about one octant.
 *
 * Then q = |p| − h shifts things so the cube's corner sits at the
 * origin.  Now any component of q that's POSITIVE means "p is
 * outside the cube along this axis"; NEGATIVE means "p is inside
 * the cube along this axis (between the two parallel faces)".
 *
 *      max(q, 0)   keeps only the OUTSIDE components.
 *                  Length of this gives correct distance to the
 *                  nearest face/edge/corner OUTSIDE the cube.
 *      → corner if all three q > 0;  edge if two; face if one.
 *
 *      max_component(q)   the LEAST-NEGATIVE q component when p is
 *                  inside, i.e. the closest face from inside.
 *                  min(..., 0) clamps to zero outside (where the
 *                  outside formula already handled it).
 *
 * Worked check at p = (0.4, 0.2, 0), h = 0.5:
 *      q   = (0.4 − 0.5, 0.2 − 0.5, 0 − 0.5) = (−0.1, −0.3, −0.5)
 *      max(q,0) = (0, 0, 0)        →   |…| = 0
 *      max(q.x, q.y, q.z) = −0.1   →   min(−0.1, 0) = −0.1
 *      d = 0 + (−0.1) = −0.1       →   p is 0.1 inside the +X face. ✓
 *
 * Worked check at p = (0.7, 0.6, 0), h = 0.5:
 *      q   = (0.2, 0.1, −0.5)
 *      max(q,0) = (0.2, 0.1, 0)    →   |…| = √(0.04 + 0.01) ≈ 0.224
 *      max(q.x, q.y, q.z) = 0.2    →   min(0.2, 0) = 0
 *      d = 0.224                   →   p is ≈0.22 outside, near the
 *                                       +X+Y edge. ✓
 *
 *
 * Q5: How does the cube appear to spin without ever moving?
 * ─────────────────────────────────────────────────────────
 * The SDF only knows axis-aligned cubes at the origin.  To make
 * one APPEAR to rotate by angle ry around Y, we don't rotate the
 * SDF — we rotate the QUERY POINT before passing it in:
 *
 *      d = sdf_box(rotate_y(p, ry), h)
 *
 * Geometrically: imagine the cube lives in its own private "library
 * frame" where it's always axis-aligned.  To check whether a world-
 * space point p is inside, translate p into the library frame first,
 * then ask the librarian.
 *
 *      world frame:                      library (cube) frame:
 *      ┌──────────────┐                  ┌──────────────┐
 *      │              │                  │              │
 *      │     ◇        │       =          │     □        │
 *      │   (rotated   │   sample at      │  (axis-      │
 *      │    cube)     │   rot⁻¹ · p      │   aligned)   │
 *      └──────────────┘                  └──────────────┘
 *
 * In this file, rotate_query_point applies R_y(+ry) then R_x(+rx).
 * Strictly the canonical "cube spins by (+ry, +rx)" needs the
 * INVERSE: R_x(−rx) · R_y(−ry).  The code's choice has the cube
 * spinning the OPPOSITE direction.  Because spin_speed_x and
 * spin_speed_y are positive constants, the user just sees a
 * spinning cube — the sign quirk is invisible at the UX level.
 *
 *
 * Q6: When the ray hits, which way does the surface face?
 * ───────────────────────────────────────────────────────
 * The Phong shader needs the SURFACE NORMAL — the unit vector
 * pointing OUTWARD from the surface at the hit point.  For the box
 * SDF there's no closed-form normal, so we approximate the gradient
 * ∇f via finite differences.
 *
 *      ∇f points OUTWARD from the surface (this is true for any
 *      SDF — the gradient of "signed distance" naturally points
 *      away from the zero level set).
 *
 *      N = ∇f / |∇f|
 *
 * Two common variants of the gradient estimator:
 *
 *      6-tap CENTRAL DIFFERENCES (one pair per axis):
 *          Nx = (f(p+εx) − f(p−εx)) / 2ε
 *          Ny = (f(p+εy) − f(p−εy)) / 2ε
 *          Nz = (f(p+εz) − f(p−εz)) / 2ε
 *
 *      4-tap TETRAHEDRAL (Quílez):
 *          k₀..k₃ = corners of a regular tetrahedron in {±1}³
 *          N = normalise(Σ kᵢ · f(p + ε · kᵢ))
 *
 * 4-tap is faster (4 SDF evals vs 6) AND cleaner near sharp edges —
 * because no two tetrahedron offsets share an axis, the normal at
 * a cube edge doesn't get smeared across both adjacent faces.  For
 * a SPHERE this wouldn't matter; for the CUBE it's the difference
 * between sharp edges and bevelled edges.
 *
 *
 * Q7: How does one number become a brightness value?
 * ──────────────────────────────────────────────────
 * Phong shading layers three components:
 *
 *      AMBIENT     KA          a constant base — keeps back-faces
 *                              from being totally black.
 *      DIFFUSE     KD · (N·L)  Lambert's law: surfaces facing the
 *                              light receive more energy per area.
 *                              N·L is just cos(angle to light).
 *      SPECULAR    KS · (R·V)^SHIN
 *                              The bright highlight where the
 *                              light's reflection points at the
 *                              camera.  SHIN tightens the spot.
 *
 * Reflection vector geometric derivation:
 *
 *      L decomposes into a part PARALLEL to N (length N·L) and a
 *      part PERPENDICULAR to N.  The reflected vector R keeps the
 *      perpendicular part and FLIPS the parallel part:
 *
 *          L = (N·L)·N + L_⊥
 *          R = (N·L)·N − L_⊥
 *            = (N·L)·N − (L − (N·L)·N)
 *            = 2·(N·L)·N − L
 *
 * Final intensity:
 *      I = KA + KD · max(0, N·L) + KS · max(0, R·V)^SHIN
 *
 * The two max(0, ·) clamps stop back-faces and reflected-away
 * highlights from going negative.  This file uses KA=0.08,
 * KD=0.72, KS=0.65, SHIN=50 — slightly higher specular than a
 * sphere because flat cube faces produce a sharp, narrow
 * highlight that benefits from a tighter exponent.
 *
 *
 * Q8: From "intensity 0.6" to a glyph on the screen.
 * ──────────────────────────────────────────────────
 * The Hit struct carries the result of cast_ray:
 *
 *      Hit { hit, hit_point, normal, intensity, t, steps }
 *
 * Production overlay: index the LUMA_RAMP " .,:;+=oxOX#@" by
 * intensity, and pick a theme colour by quantising intensity to
 * one of LUMI_N (8) bands.  That's all canvas_draw does.
 *
 * Three debug overlays read DIFFERENT fields of the same Hit:
 *
 *      DEBUG_NORMALS    pick a luma slot per face direction
 *                       (cube has 6 faces → up to 6 distinct bands)
 *      DEBUG_DEPTH      normalise t into [0,1] using the camera
 *                       distance range; closer = brighter
 *      DEBUG_STEPS      step count / RM_MAX_STEPS; silhouette
 *                       glows because grazing rays take more steps
 *
 * That's why §11..§15 are organised the way they are: cast_ray
 * produces a rich Hit, and each overlay is a simple, standalone
 * "Hit → glyph" function.  Adding a fifth overlay would be a 30-
 * line function — no rewrite of the renderer needed.
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

/* ── §1 config ───────────────────────────────────────────────────────── *
 *
 * Every tunable lives here.  No magic numbers anywhere else in the
 * file — if a literal carries meaning, it gets a name in §1.
 */

/* §1.1 frame rate. */
enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 24,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,
  FPS_UPDATE_MS = 500,
};

/* §1.2 canvas / cell mapping.
 *
 * The canvas is 1:1 with the terminal — each canvas pixel becomes
 * one terminal cell.  Aspect correction lives inside cast_ray
 * (vertical ray component scaled by phys_aspect) so that one cube
 * face spans the same world distance horizontally and vertically
 * even though terminal cells are about twice as tall as they are
 * wide.
 */
#define CELL_W 1
#define CELL_H 1
#define CELL_ASPECT 2.0f /* physical height ÷ width of one terminal cell */

static inline int canvas_w_from_cols(int cols) { return cols / CELL_W; }
static inline int canvas_h_from_rows(int rows) { return rows / CELL_H; }

/* §1.3 sphere tracing limits. */
#define RM_MAX_STEPS 80   /* hard cap per ray                    */
#define RM_HIT_EPS 0.003f /* "touching the surface" threshold    */
#define RM_MAX_DIST 20.0f /* ray-length budget before declaring miss */
#define RM_T_START 0.5f   /* skip self-hit at the camera plane   */

/* §1.4 normal estimation epsilon. */
#define RM_NORM_EPS 0.001f /* tetrahedron offset for finite difference */

/* §1.5 camera (zoom). */
#define CAM_Z_DEFAULT 4.5f
#define CAM_Z_MIN 2.6f /* keeps camera outside SIZE_MAXX cube  */
#define CAM_Z_MAX 12.0f
#define CAM_ZOOM_STEP 0.30f
#define FOV_HALF_TAN 0.65f /* tan(FOV/2); 0.65 ≈ 66° wide          */

/* §1.6 cube size (half-extent). */
#define CUBE_H_DEFAULT 0.9f
#define SIZE_STEP 1.15f
#define SIZE_MIN 0.15f
#define SIZE_MAXX 2.5f

/* §1.7 rotation speeds (radians / second). */
#define ROT_X_DEFAULT 0.7f
#define ROT_Y_DEFAULT 1.1f
#define ROT_STEP 1.3f
#define ROT_MIN 0.02f
#define ROT_MAX 10.0f

/* §1.8 light orbit speed (radians/sec). */
#define LIGHT_SPD_DEFAULT 0.6f
#define LIGHT_SPD_STEP 1.3f
#define LIGHT_SPD_MIN 0.02f
#define LIGHT_SPD_MAX 8.0f

/* §1.9 light orbit shape (Lissajous parameters). */
#define LIGHT_RADIUS_X 3.5f
#define LIGHT_RADIUS_Z 3.5f
#define LIGHT_BIAS_Y 2.0f
#define LIGHT_AMPLITUDE_Y 1.0f
#define LIGHT_RATE_Y 0.6f
#define LIGHT_BIAS_Z 1.0f

/* §1.10 Phong shading coefficients. */
#define KA 0.08f   /* ambient                                   */
#define KD 0.72f   /* diffuse                                   */
#define KS 0.65f   /* specular  (higher than sphere for sharp highlight) */
#define SHIN 50.0f /* specular sharpness — bigger = tighter spot */

/* §1.11 luma ramp + colour pair indices. */
enum {
  LUMI_N = 8,             /* 8 colour pairs hold the luma ramp   */
  PAIR_HUD = LUMI_N + 1,  /* yellow + bold — top status row      */
  PAIR_HINT = LUMI_N + 2, /* cyan   + bold — bottom hint row     */
};

static const char LUMA_RAMP[] = " .,:;+=oxOX#@";
#define RAMP_LEN ((int)(sizeof LUMA_RAMP - 1)) /* = 13 */

/* §1.12 themes — six 8-band 256-colour ramps, one active at a time.
 *
 * theme_apply (§3) re-points the eight luma pairs to the chosen
 * theme.  Geometry, lighting, and ramp glyphs all stay identical.
 * Per the CLAUDE.md theme-brightness rule, every entry sits in the
 * bright half of the 256-cube so even the dimmest ramp slot remains
 * visible against a black terminal background.
 */
typedef struct {
  const char *display_name;
  short ramp_256[LUMI_N];
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

/* §1.13 debug overlays — d / D cycles between them. */
typedef enum {
  DEBUG_NORMAL = 0,  /* full Phong + theme (production view) */
  DEBUG_NORMALS = 1, /* face direction → luma slot           */
  DEBUG_DEPTH = 2,   /* hit distance t → brightness          */
  DEBUG_STEPS = 3,   /* march iterations → brightness        */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "NORMALS",
    "DEPTH  ",
    "STEPS  ",
};

/* §1.14 time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── *
 *
 * CLOCK_MONOTONIC advances at one second per second with no NTP
 * jumps — exactly what a frame timer wants.
 */

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

/* ── §3 color — themes + HUD/hint pairs ──────────────────────────────── *
 *
 * Eight colour pairs (1..8) hold the active theme's luma ramp.
 * Two more pairs (PAIR_HUD, PAIR_HINT) are reserved for the HUD
 * and key-hint strips per the CLAUDE.md HUD spec — yellow + bold
 * for status, cyan + bold for hints.
 */

/* theme_apply — re-point the 8 luma pairs to the chosen theme. */
static void theme_apply(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const Theme *theme = &THEMES[theme_index];

  if (COLORS >= 256) {
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), theme->ramp_256[i], COLOR_BLACK);
  } else {
    /* 8-colour fallback: themes have no effect; lumi_attr fakes
     * brightness via A_DIM / A_BOLD on COLOR_WHITE. */
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

/* lumi_attr — ncurses attribute for luma slot l ∈ [0, LUMI_N). */
static attr_t lumi_attr(int l) {
  if (l < 0)
    l = 0;
  if (l > LUMI_N - 1)
    l = LUMI_N - 1;
  attr_t a = COLOR_PAIR(l + 1);
  if (COLORS < 256) {
    if (l < 3)
      a |= A_DIM;
    else if (l >= 6)
      a |= A_BOLD;
  }
  return a;
}

/* ── §4 vec3 — value-type 3-D math ───────────────────────────────────── *
 *
 * All operations return Vec3 by value.  At -O2 every call here
 * inlines to register operations.  v3abs and v3max0 are the two
 * component-wise primitives the box SDF (§5) needs.
 */

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

/* component-wise absolute value */
static inline Vec3 v3abs(Vec3 a) {
  return v3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
}

/* component-wise max(a, 0) */
static inline Vec3 v3max0(Vec3 a) {
  return v3(fmaxf(a.x, 0.0f), fmaxf(a.y, 0.0f), fmaxf(a.z, 0.0f));
}

/* ── §5 box SDF — Quílez exact box ───────────────────────────────────── *
 *
 * Tutorial Q3+Q4 derived this in two halves.  Implementation is
 * two lines of math: outside term + inside term.
 *
 *      q = |p| − h
 *      d = |max(q, 0)| + min(max_component(q), 0)
 *
 * The outside term gives true Euclidean distance to the nearest
 * face/edge/corner.  The inside term gives negative distance to
 * the nearest face from inside (clamped to zero outside, where
 * the outside term already handled it).
 *
 * For a cube of half-extent h:
 *      sdf_box returns positive outside, negative inside,
 *      zero on the surface.  Lipschitz-1 (true distance, exact —
 *      not an approximation), so it's safe for sphere tracing.
 */
static float sdf_box(Vec3 p, float cube_half) {
  Vec3 q = v3sub(v3abs(p), v3(cube_half, cube_half, cube_half));
  Vec3 q_pos = v3max0(q);
  float outside = v3len(q_pos);
  float inside = fminf(fmaxf(q.x, fmaxf(q.y, q.z)), 0.0f);
  return outside + inside;
}

/* ── §6 rotate query point — make the cube appear to spin ────────────── *
 *
 * Q5 explained the trick: the cube doesn't move; we rotate the
 * query point before passing it into sdf_box.
 *
 * Y-rotation matrix (positive angle, right-handed coordinates):
 *      [ cos(θ)    0    sin(θ)]
 *      [   0       1      0   ]
 *      [-sin(θ)    0    cos(θ)]
 *
 * X-rotation matrix:
 *      [   1       0      0   ]
 *      [   0    cos(φ)  -sin(φ)]
 *      [   0    sin(φ)   cos(φ)]
 *
 * This function applies R_y(ry) then R_x(rx) — see the EDGE CASES
 * "Rotation sign quirk" note in MENTAL MODEL for why this is the
 * opposite-sign rotation from the canonical inverse.
 */
static Vec3 rotate_query_point(Vec3 p, float rx, float ry) {
  /* R_y(ry): rotate around Y axis */
  float cy = cosf(ry), sy = sinf(ry);
  float px = p.x * cy + p.z * sy;
  float pz = -p.x * sy + p.z * cy;
  p.x = px;
  p.z = pz;

  /* R_x(rx): rotate around X axis */
  float cx = cosf(rx), sx = sinf(rx);
  float py = p.y * cx - p.z * sx;
  pz = p.y * sx + p.z * cx;
  p.y = py;
  p.z = pz;

  return p;
}

/* ── §7 scene SDF — composition: rotate, then box ────────────────────── *
 *
 * The sphere tracer calls sdf_scene at every march step.  The
 * tracer is generic; only this one function knows the scene is
 * a rotated box.  Add a second cube, a torus, a Boolean union,
 * etc. — change happens here, the marcher doesn't care.
 */
static float sdf_scene(Vec3 p, float rx, float ry, float cube_half) {
  Vec3 p_local = rotate_query_point(p, rx, ry);
  return sdf_box(p_local, cube_half);
}

/* ── §8 surface normal — tetrahedral 4-tap finite difference ─────────── *
 *
 * Q6 explained the choice of 4-tap over 6-tap.  The four sample
 * directions are the corners of a regular tetrahedron in {±1}³,
 * scaled by ε:
 *
 *      k₀ = ( ε, −ε, −ε)
 *      k₁ = (−ε, −ε,  ε)
 *      k₂ = (−ε,  ε, −ε)
 *      k₃ = ( ε,  ε,  ε)
 *
 * No two share an axis — that's the property that gives sharp
 * cube edges instead of bevelled ones.
 *
 * The 1/√3 factor that would normalise k_i to unit length cancels
 * in the final normalise(), so we omit it.
 */
static Vec3 surface_normal(Vec3 p, float rx, float ry, float cube_half) {
  const float e = RM_NORM_EPS;

  Vec3 k0 = v3(e, -e, -e);
  Vec3 k1 = v3(-e, -e, e);
  Vec3 k2 = v3(-e, e, -e);
  Vec3 k3 = v3(e, e, e);

  float d0 = sdf_scene(v3add(p, k0), rx, ry, cube_half);
  float d1 = sdf_scene(v3add(p, k1), rx, ry, cube_half);
  float d2 = sdf_scene(v3add(p, k2), rx, ry, cube_half);
  float d3 = sdf_scene(v3add(p, k3), rx, ry, cube_half);

  Vec3 n = v3add(v3add(v3mul(k0, d0), v3mul(k1, d1)),
                 v3add(v3mul(k2, d2), v3mul(k3, d3)));
  return v3norm(n);
}

/* ── §9 sphere trace — Hart 1996 march loop ──────────────────────────── *
 *
 * Q2 explained the algorithm.  Returns the ray-parameter t at the
 * hit, or -1 on miss.  Optionally writes the step count via
 * out_steps (used by the STEPS debug overlay).
 *
 * Starts at t = RM_T_START to skip a "ray begins inside camera
 * plane" pathology — see MENTAL MODEL → EDGE CASES.
 */
static float sphere_trace(Vec3 ro, Vec3 rd, float rx, float ry, float cube_half,
                          int *out_steps) {
  float t = RM_T_START;
  int step;
  for (step = 0; step < RM_MAX_STEPS; step++) {
    Vec3 p = v3add(ro, v3mul(rd, t));
    float d = sdf_scene(p, rx, ry, cube_half);
    if (d < RM_HIT_EPS) {
      if (out_steps)
        *out_steps = step + 1;
      return t;
    }
    if (t > RM_MAX_DIST)
      break;
    t += d;
  }
  if (out_steps)
    *out_steps = step;
  return -1.0f;
}

/* ── §10 Phong shade — ambient + diffuse + specular ──────────────────── *
 *
 * Q7 derived the formula.  Implementation walks each component in
 * order and clamps the final intensity to [0, 1].
 */
static float phong_shade(Vec3 N, Vec3 hit, Vec3 cam, Vec3 light) {
  Vec3 L = v3norm(v3sub(light, hit));
  Vec3 V = v3norm(v3sub(cam, hit));
  float ndl = fmaxf(0.0f, v3dot(N, L));
  Vec3 R = v3sub(v3mul(N, 2.0f * ndl), L);
  float spec = powf(fmaxf(0.0f, v3dot(R, V)), SHIN);

  float I = KA + KD * ndl + KS * spec;
  if (I < 0.0f)
    I = 0.0f;
  if (I > 1.0f)
    I = 1.0f;
  return I;
}

/* ── §11 cast_ray — one pixel's full pipeline → Hit ──────────────────── *
 *
 * Builds a ray, sphere-traces, computes the normal and Phong
 * intensity if the ray hit, and returns a `Hit` carrying every
 * field any overlay might need.
 *
 * The Hit struct is the seam between rendering math and overlay
 * code: production view + three debug overlays all read the SAME
 * Hit array and never re-trace.
 */

typedef struct {
  bool hit;
  Vec3 hit_point;
  Vec3 normal;
  float intensity;      /* Phong result in [0,1]                     */
  float trace_distance; /* ray parameter at hit (DEBUG_DEPTH input)  */
  int step_count;       /* march iterations    (DEBUG_STEPS input)   */
} Hit;

static Hit cast_ray(int col, int row, int cw, int ch, float rx, float ry,
                    float cube_half, Vec3 light, float cam_z) {
  Hit h = {false, {0, 0, 0}, {0, 0, 1}, 0.0f, 0.0f, 0};

  /* NDC for the cell centre (Q1). */
  float u = ((float)col + 0.5f) / (float)cw * 2.0f - 1.0f;
  float v = -((float)row + 0.5f) / (float)ch * 2.0f + 1.0f;

  /* Aspect: terminal cells are ~2× taller than wide.  Scale v
   * so equal physical distance per unit in both axes. */
  float phys_aspect = ((float)ch * CELL_ASPECT) / (float)cw;

  Vec3 ro = v3(0.0f, 0.0f, cam_z);
  Vec3 rd = v3norm(v3(u * FOV_HALF_TAN, v * FOV_HALF_TAN * phys_aspect, -1.0f));

  int steps = 0;
  float t = sphere_trace(ro, rd, rx, ry, cube_half, &steps);
  h.step_count = steps;

  if (t < 0.0f)
    return h;

  h.hit = true;
  h.trace_distance = t;
  h.hit_point = v3add(ro, v3mul(rd, t));
  h.normal = surface_normal(h.hit_point, rx, ry, cube_half);
  h.intensity = phong_shade(h.normal, h.hit_point, ro, light);
  return h;
}

/* ── §12 canvas — the Hit framebuffer ────────────────────────────────── *
 *
 * One Hit per canvas pixel.  Sized at startup and on resize; never
 * reallocated mid-frame.  Render writes; draw / debug-overlay
 * functions read.
 */

typedef struct {
  int w, h;
  Hit *hits;
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows) {
  c->w = canvas_w_from_cols(cols);
  c->h = canvas_h_from_rows(rows);
  c->hits = calloc((size_t)(c->w * c->h), sizeof(Hit));
}

static void canvas_free(Canvas *c) {
  free(c->hits);
  c->hits = NULL;
  c->w = c->h = 0;
}

/* ── §13 render — fill the Hit array for one frame ───────────────────── *
 *
 * Pure math.  Knows nothing about glyphs, colour, or terminals.
 * cam_z is a parameter (not a constant) so the user can zoom.
 */
static void canvas_render(Canvas *c, float rx, float ry, float cube_half,
                          Vec3 light, float cam_z) {
  for (int row = 0; row < c->h; row++) {
    for (int col = 0; col < c->w; col++) {
      c->hits[row * c->w + col] =
          cast_ray(col, row, c->w, c->h, rx, ry, cube_half, light, cam_z);
    }
  }
}

/* ── §14 draw — production overlay (Hit → glyph + theme colour) ──────── *
 *
 * The default view: intensity drives glyph and colour.  Three
 * helpers handle the bookkeeping (ramp index, attribute slot,
 * terminal-centre offset) so canvas_draw itself reads as a tight
 * loop.
 */

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

/* canvas_offsets — top-left screen position so the canvas centres. */
static void canvas_offsets(const Canvas *c, int term_cols, int term_rows,
                           int *out_off_x, int *out_off_y) {
  int total_w = c->w * CELL_W;
  int total_h = c->h * CELL_H;
  *out_off_x = (term_cols - total_w) / 2;
  *out_off_y = (term_rows - total_h) / 2;
}

/* emit_block — paint a CELL_W × CELL_H block at one canvas pixel. */
static void emit_block(int tx0, int ty0, char glyph, attr_t attr, int term_cols,
                       int term_rows) {
  attron(attr);
  for (int by = 0; by < CELL_H; by++) {
    for (int bx = 0; bx < CELL_W; bx++) {
      int tx = tx0 + bx;
      int ty = ty0 + by;
      if (tx < 0 || tx >= term_cols)
        continue;
      if (ty < 0 || ty >= term_rows)
        continue;
      mvaddch(ty, tx, (chtype)(unsigned char)glyph);
    }
  }
  attroff(attr);
}

static void canvas_draw(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      char glyph = intensity_to_glyph(h->intensity);
      attr_t attr = intensity_to_attr(h->intensity);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

/* ── §15 debug overlays — see the rendering signals raw ──────────────── *
 *
 * Three educational visualisations.  Each isolates ONE piece of
 * intermediate state from the Hit struct and paints it directly.
 *
 *   NORMALS  — cube faces become flat colour bands (one per face)
 *              because all face normals are exactly ±X, ±Y, ±Z.
 *              The clearest way to see "yes, this is a cube".
 *
 *   DEPTH    — closer hit points render brighter.  Confirms that
 *              the trace converged at sensible distances.
 *
 *   STEPS    — silhouette glow.  Grazing rays take many march steps
 *              before deciding to miss — those pixels light up.
 */

/* face_slot_for_normal — six faces → six luma slots.
 *
 * The dominant component of N tells us which face was hit:
 *   |N.x| largest  → ±X face → slots 6, 7
 *   |N.y| largest  → ±Y face → slots 4, 5
 *   |N.z| largest  → ±Z face → slots 1, 2
 * Sign of the dominant component picks between the two slots.
 */
static int face_slot_for_normal(Vec3 N) {
  float ax = fabsf(N.x), ay = fabsf(N.y), az = fabsf(N.z);
  if (ax >= ay && ax >= az)
    return (N.x > 0) ? 7 : 6;
  if (ay >= az)
    return (N.y > 0) ? 5 : 4;
  return (N.z > 0) ? 2 : 1;
}

static void canvas_draw_normals(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      int slot = face_slot_for_normal(h->normal);
      char glyph = LUMA_RAMP[(slot * RAMP_LEN) / LUMI_N];
      attr_t attr = lumi_attr(slot);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

static void canvas_draw_depth(const Canvas *c, int term_cols, int term_rows,
                              float cam_z) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  /* The cube spans at most ±SIZE_MAXX·√3 from the origin (corner
   * distance).  Closer hits get higher depth_n. */
  float t_min = cam_z - SIZE_MAXX * 1.732f;
  float t_max = cam_z + SIZE_MAXX * 1.732f;
  if (t_min < 0.0f)
    t_min = 0.0f;

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      float depth_n = (t_max - h->trace_distance) / (t_max - t_min);
      if (depth_n < 0.0f)
        depth_n = 0.0f;
      if (depth_n > 1.0f)
        depth_n = 1.0f;

      char glyph = intensity_to_glyph(depth_n);
      attr_t attr = intensity_to_attr(depth_n);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

static void canvas_draw_steps(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      float steps_n = (float)h->step_count / (float)RM_MAX_STEPS;
      if (steps_n > 1.0f)
        steps_n = 1.0f;

      char glyph = intensity_to_glyph(steps_n);
      attr_t attr = intensity_to_attr(steps_n);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

/* ── §16 scene — Scene struct + light + tick ─────────────────────────── */

typedef struct {
  Canvas canvas;
  float rot_angle_x;  /* current cube rotation around X (rad) */
  float rot_angle_y;  /* current cube rotation around Y (rad) */
  float spin_speed_x; /* X spin (rad/sec)                     */
  float spin_speed_y; /* Y spin (rad/sec)                     */
  float light_spd;    /* light orbit speed (rad/sec)          */
  float cube_half;    /* cube half-extent                     */
  float time;         /* seconds since start                  */
  float cam_z;        /* camera z; smaller = zoomed-in        */
  int theme_index;    /* index into THEMES[]                  */
  DebugMode debug_mode;
  bool paused;
} Scene;

/* Lissajous orbit — never enters the cube. */
static Vec3 scene_light(const Scene *s) {
  float a = s->time * s->light_spd;
  return v3(LIGHT_RADIUS_X * cosf(a),
            LIGHT_BIAS_Y + LIGHT_AMPLITUDE_Y * sinf(LIGHT_RATE_Y * a),
            LIGHT_RADIUS_Z * sinf(a) + LIGHT_BIAS_Z);
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  canvas_alloc(&s->canvas, cols, rows);
  s->rot_angle_x = 0.3f;
  s->rot_angle_y = 0.5f;
  s->spin_speed_x = ROT_X_DEFAULT;
  s->spin_speed_y = ROT_Y_DEFAULT;
  s->light_spd = LIGHT_SPD_DEFAULT;
  s->cube_half = CUBE_H_DEFAULT;
  s->time = 0.0f;
  s->cam_z = CAM_Z_DEFAULT;
  s->theme_index = 0;
  s->debug_mode = DEBUG_NORMAL;
  s->paused = false;
}

static void scene_free(Scene *s) { canvas_free(&s->canvas); }

static void scene_resize(Scene *s, int cols, int rows) {
  canvas_free(&s->canvas);
  canvas_alloc(&s->canvas, cols, rows);
}

static void scene_tick(Scene *s, float dt_sec) {
  if (s->paused)
    return;
  s->rot_angle_x += s->spin_speed_x * dt_sec;
  s->rot_angle_y += s->spin_speed_y * dt_sec;
  s->time += dt_sec;
}

static void scene_render(Scene *s) {
  canvas_render(&s->canvas, s->rot_angle_x, s->rot_angle_y, s->cube_half,
                scene_light(s), s->cam_z);
}

/* dispatch to the active overlay */
static void scene_draw_active(const Scene *s, int cols, int rows) {
  switch (s->debug_mode) {
  case DEBUG_NORMAL:
    canvas_draw(&s->canvas, cols, rows);
    break;
  case DEBUG_NORMALS:
    canvas_draw_normals(&s->canvas, cols, rows);
    break;
  case DEBUG_DEPTH:
    canvas_draw_depth(&s->canvas, cols, rows, s->cam_z);
    break;
  case DEBUG_STEPS:
    canvas_draw_steps(&s->canvas, cols, rows);
    break;
  default:
    canvas_draw(&s->canvas, cols, rows);
    break;
  }
}

/* ── §17 screen — ncurses init / HUD / present ───────────────────────── */

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
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 */
static void screen_draw(Screen *s, const Scene *sc, double fps) {
  erase();
  scene_draw_active(sc, s->cols, s->rows);

  char status[200];
  snprintf(status, sizeof status,
           " %5.1f fps  spd:%.2f  h:%.2f  zoom:%.2f  theme:%s  "
           "debug:%s  [%dx%d]  %s ",
           fps, sc->spin_speed_y, sc->cube_half, sc->cam_z,
           THEMES[sc->theme_index].display_name,
           DEBUG_MODE_NAMES[sc->debug_mode], sc->canvas.w, sc->canvas.h,
           sc->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " CUBE ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  ]/[:spin  +/-:size  l/L:light  "
           "z/Z:zoom  t/T:theme  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §18 app — main loop, signals, key handling ──────────────────────── */

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

  case ']':
    s->spin_speed_x *= ROT_STEP;
    s->spin_speed_y *= ROT_STEP;
    if (s->spin_speed_x > ROT_MAX)
      s->spin_speed_x = ROT_MAX;
    if (s->spin_speed_y > ROT_MAX)
      s->spin_speed_y = ROT_MAX;
    break;
  case '[':
    s->spin_speed_x /= ROT_STEP;
    s->spin_speed_y /= ROT_STEP;
    if (s->spin_speed_x < ROT_MIN)
      s->spin_speed_x = ROT_MIN;
    if (s->spin_speed_y < ROT_MIN)
      s->spin_speed_y = ROT_MIN;
    break;

  case '=':
  case '+':
    s->cube_half *= SIZE_STEP;
    if (s->cube_half > SIZE_MAXX)
      s->cube_half = SIZE_MAXX;
    break;
  case '-':
    s->cube_half /= SIZE_STEP;
    if (s->cube_half < SIZE_MIN)
      s->cube_half = SIZE_MIN;
    break;

  case 'l':
    s->light_spd *= LIGHT_SPD_STEP;
    if (s->light_spd > LIGHT_SPD_MAX)
      s->light_spd = LIGHT_SPD_MAX;
    break;
  case 'L':
    s->light_spd /= LIGHT_SPD_STEP;
    if (s->light_spd < LIGHT_SPD_MIN)
      s->light_spd = LIGHT_SPD_MIN;
    break;

  case 'z':
    s->cam_z -= CAM_ZOOM_STEP;
    if (s->cam_z < CAM_Z_MIN)
      s->cam_z = CAM_Z_MIN;
    break;
  case 'Z':
    s->cam_z += CAM_ZOOM_STEP;
    if (s->cam_z > CAM_Z_MAX)
      s->cam_z = CAM_Z_MAX;
    break;

  case 't':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;
  case 'T':
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
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    scene_render(&app->scene);

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
