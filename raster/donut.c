/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * donut.c — pedagogical edition: Andy Sloane's spinning ASCII torus
 *
 *   A donut tumbling in front of a fixed light bulb.  Built top-to-
 *   bottom as a guided tutorial: the prose teaches the algorithm,
 *   and the working code is the worked exercises.
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume rotation
 *   ] / [        spin faster / slower
 *   = / -        torus larger / smaller
 *   t / T        next / previous colour theme
 *                  (CLASSIC / AMBER / MATRIX / NEON / ICE / COPPER)
 *   d / D        cycle debug overlay (NORMAL / DEPTH / WIRE / NO_ZBUF)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/donut.c -o donut \
 *       -lncurses -lm
 *
 * Section map (each section ≤ ~100 lines, teaches one concept):
 *   §1  config        — every tunable, every magic number, themes table
 *   §2  clock         — monotonic timer + sleep
 *   §3  color         — luminance pairs + theme apply + HUD/hint pairs
 *   §4  math          — V2 / V3 / Rot constructors
 *   §5  framebuffer   — Torus struct + buffers + clear
 *   §6  perspective   — K1 sizing (auto-fit the donut to terminal)
 *   §7  tick          — advance rotation angles
 *   §8  tube point    — sample one point on the tube cross-section
 *   §9  surface point — sample one point on the rotated 3-D surface
 *   §10 project       — perspective divide + bounds check
 *   §11 luminance     — Lambertian dot product (closed form)
 *   §12 emit          — z-buffer test + glyph store
 *   §13 render        — orchestrator: walks (θ, φ), calls §8..§12
 *   §14 draw          — paint the framebuffer to the terminal
 *   §15 debug         — four educational overlays (see §15 preamble)
 *   §16 screen        — ncurses init / HUD draw / present
 *   §17 app           — main loop, signals, key handling
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is ordered to TEACH, not to compile-optimally.  Read top to
 * bottom.  Each section builds on the previous.  The reading order:
 *
 *   1. CONCEPTS         (high-level "what is this program?")
 *   2. MENTAL MODEL     (intuition + ASCII diagram of the algorithm)
 *   3. GUIDED TUTORIAL  (8 mini-tutorials that build the algorithm
 *                        from first principles, no math required —
 *                        plain English + diagrams + pseudocode)
 *   4. §1..§17          (actual code, each section short and focused)
 *
 * If you only have ten minutes: read the GUIDED TUTORIAL.  After it,
 * the code reads like familiar territory rather than a math wall.
 *
 * The code uses LONG DESCRIPTIVE variable names everywhere
 * (`tube_point.x` instead of `cx`, `one_over_z` instead of `ooz`).
 * Long names add visual weight but turn every line into self-
 * documentation.  When you scan a function, the names tell you
 * what's happening before you read the math.
 *
 * Convention for math notation:
 *   – θ (theta)  walks around the BIG circle (the torus axis)
 *   – φ (phi)    walks around the SMALL circle (the tube)
 *   – A          Euler rotation angle around the X axis (tumble)
 *   – B          Euler rotation angle around the Z axis (spin)
 *   – K1         perspective scaling — controls projected size
 *   – K2         camera-to-donut distance — controls foreshortening
 *
 * In code these become `theta`, `phi`, `angle_A`, `angle_B`, `K1`,
 * `K2` — short enough to read fluently, named enough that you don't
 * forget which is which.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : Analytic torus point-sampling.  No raymarching, no
 *                tessellated triangles.  We walk a (θ, φ) parameter
 *                grid where θ goes around the torus axis (the BIG
 *                circle) and φ goes around the tube cross-section
 *                (the SMALL circle); each (θ, φ) gives one 3-D point
 *                on the torus surface in closed form.  Rotate every
 *                point by two Euler angles, project to 2-D with 1/z
 *                perspective division, run a per-cell depth buffer
 *                that keeps only the closest dot.  The donut you see
 *                is thousands of independent points fighting for the
 *                front-most spot per cell.
 *
 * Data         : Three flat buffers sized to TORUS_MAX_CELLS:
 *                  zbuf  [r·cols+c]  → 1/z of the closest dot here
 *                                       so far  (zero = empty cell)
 *                  glyph [r·cols+c]  → ASCII char from LUMI_RAMP, or
 *                                       space if no dot landed here
 *                  luma  [r·cols+c]  → luma slot 0..7 (so the draw
 *                                       step picks the colour pair
 *                                       without re-deriving it)
 *                All three are zeroed at the start of each render.
 *                No heap allocation after init.
 *
 * Rendering    : Z-buffer convention is INVERTED — we store 1/z, not
 *                z, because perspective division produces 1/z
 *                naturally and bigger 1/z means closer.  The depth
 *                test is `if (one_over_z > zbuf[idx])` — accept the
 *                new dot if it's closer.  Empty cells start at zero
 *                ("infinitely far"), and every visible torus point
 *                has positive z thanks to the +K2 shift in
 *                surface_point().
 *
 * Performance  : O(N_θ · N_φ) per frame ≈ 90 × 314 = 28 000 surface
 *                samples regardless of terminal size.  Each sample
 *                is ~25 floating-point ops; ~1 ms / frame on a
 *                modern CPU at the default sample density.  Halving
 *                THETA_STEP doubles the cost.  Rendering is
 *                independent of cell count — increasing the terminal
 *                size makes the donut bigger, not slower.
 *
 * References   :
 *   • Sloane, A. (2011) — "Donut math: how donut.c works"
 *     https://www.a1k0n.net/2011/07/20/donut-math.html
 *     The canonical write-up of the original algorithm.  Every
 *     formula in this file matches one in that post.
 *   • Wikipedia — "Torus § Parametric equation"
 *     https://en.wikipedia.org/wiki/Torus#Geometry
 *     The (R, r, θ, φ) parameterisation we use.
 *   • Newman & Sproull (1979) — *Principles of Interactive Computer
 *     Graphics*, 2nd ed.  Ch. 22 covers the z-buffer visibility
 *     algorithm.
 *   • Foley, van Dam, Feiner & Hughes — *Computer Graphics:
 *     Principles and Practice*.  Ch. 5 has the perspective-divide
 *     derivation.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A torus is two nested circles: walk a SMALL circle (the tube
 * cross-section, parameter φ) while sweeping that whole circle
 * around a BIGGER circle (the torus axis, parameter θ).  Sampling
 * (θ, φ) on a fine grid produces a dense cloud of 3-D surface
 * points.  Rotate every point by two Euler angles, project to 2-D
 * with 1/z perspective, and let a z-buffer decide which sample wins
 * each terminal cell.  The donut you see is just thousands of lit
 * dots fighting for the front-most spot.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a sparkler tracing a small circle while you slowly rotate
 * your wrist.  The trail covers a doughnut in space.  Now photograph
 * that doughnut from a fixed camera while the doughnut itself
 * tumbles — each sparkler dot is one (θ, φ) sample.  The screen is
 * film: each pixel remembers ONLY the closest sparkler dot that hit
 * it, and the brightness comes from how the dot's surface tilts
 * toward a light bulb above and behind the camera.
 *
 *      ┌────────────────────────────────────────────────┐
 *      │     θ  (around torus axis, big circle)         │
 *      │       ↻↻↻↻↻↻↻↻↻↻↻↻↻↻↻↻↻↻                       │
 *      │   ╭───╮  ╭───╮  ╭───╮ ◀── tube cross-section   │
 *      │   │ φ │  │ φ │  │ φ │     (small circle)       │
 *      │   ╰───╯  ╰───╯  ╰───╯                          │
 *      │                                                │
 *      │   point = tube_point(θ) revolved by φ          │
 *      │         → rotate by A around X, B around Z     │
 *      │         → project: (xp, yp, ooz)               │
 *      │         → keep if ooz > zbuf[idx]              │
 *      └────────────────────────────────────────────────┘
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 *  Per frame:
 *    1. Reset zbuf[] to 0 (= infinitely far, since we store 1/z),
 *       glyph[] to space, luma[] to zero.
 *    2. Pre-compute (sin_A, cos_A, sin_B, cos_B) — constant for every
 *       sample within one frame.
 *
 *  Per (θ, φ) sample (≈ 28 000 of them):
 *    3. tube_point(θ)            → 2-D point on tube cross-section
 *    4. surface_point(...)       → 3-D world point (rotated, +K2)
 *    5. project_to_screen(...)   → (col, row, one_over_z) + bounds
 *    6. surface_luminance(...)   → scalar L; skip if L ≤ 0 (back-face)
 *    7. try_emit_pixel(...)      → z-buffer test + glyph + luma store
 *
 *  After the loop:
 *    8. torus_draw walks the buffers, emits every non-space cell
 *       with its colour pair.
 *
 *  Each tick (frame-rate independent):
 *    9. angle_A += rate_A · dt;  angle_B += rate_B · dt.
 *
 * KEY FORMULAS
 * ────────────
 *   Tube point:
 *     tube.x = R2 + R1 · cos θ      (circle in the plane,
 *     tube.y =      R1 · sin θ       before revolving)
 *
 *   X-then-Z combined rotation (after multiplying out the matrices):
 *     x = tube.x · (cos B · cos φ + sin A · sin B · sin φ)
 *       − tube.y ·  cos A · sin B
 *     y = tube.x · (sin B · cos φ − sin A · cos B · sin φ)
 *       + tube.y ·  cos A · cos B
 *     z = K2 + cos A · tube.x · sin φ + tube.y · sin A
 *
 *   1/z perspective projection (Y_ASPECT halves vertical):
 *     one_over_z = 1 / z
 *     screen_col =  cols/2 + K1 · one_over_z · x
 *     screen_row =  rows/2 − K1 · one_over_z · y · Y_ASPECT
 *
 *   Lambertian luminance with hard-coded light dir (0, 1, −1)/√2:
 *     L = cos φ · cos θ · sin B
 *       − cos A · cos θ · sin φ
 *       − sin A · sin θ
 *       + cos B · (cos A · sin θ − cos θ · sin A · sin φ)
 *
 *   K1 sizing (auto-fit to terminal):
 *     K1 = TARGET_FILL · min(cols/2, rows) · (K2 + R2 + R1) / (R2 + R1)
 *
 *   Glyph index (0..LUMI_RAMP_LEN−1):
 *     glyph_slot = clamp( floor(L · LUMI_RAMP_LEN), 0, LUMI_RAMP_LEN − 1 )
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Z-BUFFER CONVENTION IS INVERTED.  zbuf[] holds 1/z, so larger
 *     means closer.  A naïve `if (ooz < zbuf[idx])` draws the BACK
 *     of the donut on top — a classic introductory bug.
 *
 *   • Initial zbuf = 0 only works because every visible surface
 *     point has z > 0 (the +K2 shift puts the donut in front of
 *     the camera).
 *
 *   • Y_ASPECT = 0.5 is NOT physics — it's terminal aspect
 *     correction for cells that are roughly twice as tall as wide.
 *     On a square-cell terminal you'd set Y_ASPECT = 1.0.
 *
 *   • Sample undersampling shows on the TUBE before the AXIS because
 *     the tube is the smaller circle.  PHI_STEP (0.02) is about 3.5×
 *     finer than THETA_STEP (0.07) for that reason.
 *
 *   • Pause freezes A and B but render still runs every frame — the
 *     donut stays visible, just stationary.  Useful for inspecting
 *     individual frames at A=0 or after a specific manual rotation.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • At startup, the donut appears as a recognisable doughnut
 *     silhouette tumbling slowly.  Sun-facing top is bright, far
 *     side is dim.
 *
 *   • Press space.  Rotation freezes.  Press ] five times then space
 *     to resume — donut spins ~3.7× faster (1.3^5 ≈ 3.71).
 *
 *   • Press = until growth stops at TORUS_SIZE_MAX.  Press - back to
 *     original size.  K1 changes; the donut never disappears off-
 *     screen because torus_k1 always fits to the smaller dimension.
 *
 *   • Press d to cycle debug overlays.  DEPTH paints by 1/z so you
 *     literally see what the z-buffer knows.  WIRE paints only every
 *     10th sample so you see the (θ, φ) sampling grid.  NO_ZBUF
 *     skips the depth test so the back of the donut bleeds through.
 *
 *   • Setting THETA_STEP to 0.5 deliberately produces a sparse
 *     "wire cage" donut — proves the (θ, φ) loop is the only source
 *     of sample density.
 *
 *   • Hard test: temporarily flip the depth comparison to `<`
 *     instead of `>`.  The BACK of the donut now draws on top of
 *     the front.  Confirms the inverted-z convention is doing the
 *     work.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — first-principles build-up of the algorithm ────── *
 *
 * Eight short tutorials.  Each builds one piece of the puzzle.  Read
 * top to bottom; later sections refer back to these by number.
 *
 *
 * TUTORIAL 1: What IS a torus, geometrically?
 * ───────────────────────────────────────────
 * A torus is two circles at right angles.  Take a small circle
 * (radius r), and revolve it around a line whose distance from the
 * circle's centre is R.  The surface swept out is a doughnut with
 * an inner hole of radius (R − r) and an outer radius of (R + r).
 *
 *      side view:                         top view (looking down):
 *
 *           tube (small circle)            ⬢⬢⬢⬢⬢⬢⬢
 *           radius r                     ⬢⬢       ⬢⬢
 *      ●╮╭●  ↶                           ⬢   hole   ⬢
 *      ╰─╯  /                           ⬢   R−r    ⬢
 *      ▲   /                            ⬢           ⬢
 *      │  / R                            ⬢⬢       ⬢⬢
 *      │ /                                 ⬢⬢⬢⬢⬢⬢⬢
 *      │/                                outer radius R + r
 *      ●  ← axis of revolution
 *
 * To DESCRIBE every point on the surface mathematically we need TWO
 * angles: one to go around the small circle (φ), one to go around
 * the big revolution (θ).  Pick any pair (θ, φ) and you get exactly
 * one point on the surface.
 *
 *
 * TUTORIAL 2: Parametric form (the tube point)
 * ────────────────────────────────────────────
 * Before we revolve, the tube cross-section sits in a 2-D plane.
 * It's a circle of radius r centred at distance R from the origin:
 *
 *      (cx, cy) on the tube cross-section, parameterised by θ:
 *      cx = R + r · cos θ        (horizontal: starts at R+r at θ=0,
 *                                 ends at R−r at θ=π)
 *      cy =     r · sin θ        (vertical: zero at θ=0 and π,
 *                                 peaks at θ=π/2)
 *
 *      θ = 0:                  cx ▲
 *        ●──── tube point        │
 *      θ = π/2:                  ●cy
 *        ●  (at top)             │
 *        │                       ●──→ cx
 *      θ = π:                    R   R+r
 *        ●──── tube point
 *      ...
 *
 * That's the work `tube_point(cos_theta, sin_theta)` does in §8.
 * It converts a θ value into a 2-D (cx, cy) on the tube cross-
 * section, BEFORE we revolve.
 *
 *
 * TUTORIAL 3: Revolving + 3-D (the surface point)
 * ───────────────────────────────────────────────
 * Now we revolve the tube cross-section around the y-axis by
 * sweeping φ from 0 to 2π.  At each φ value we rotate the (cx, cy)
 * into a 3-D position:
 *
 *      x = cx · cos φ      (sweep horizontally)
 *      y = cy              (vertical position is unchanged by revolution)
 *      z = cx · sin φ      (sweep into the third dimension)
 *
 * That gives an UN-ROTATED torus surface.  But the donut on screen
 * tumbles!  So we apply two more rotations: an Euler angle A around
 * the X axis (tumble forward/back) and B around the Z axis (spin
 * left/right).  The composition of those two is what makes the
 * donut visibly spin.
 *
 * The combined rotation, after multiplying out the 3×3 matrices,
 * is in MENTAL MODEL § KEY FORMULAS.  Don't memorise the formulas;
 * what matters is that you can identify which symbol does what:
 *
 *   – cos B / sin B     spin around Z (the "spin" axis)
 *   – cos A / sin A     tumble around X (the "fall forward" axis)
 *   – cos φ / sin φ     revolve around the y axis (where on tube)
 *
 * Plus we add K2 to z so the donut sits in FRONT of the camera
 * (camera is at z = 0; donut's z values are around K2 ≈ 5).  This
 * shift is what makes z always positive, which is what makes the
 * z-buffer convention (tutorial 5) work.
 *
 * That's the work `surface_point(...)` does in §9.
 *
 *
 * TUTORIAL 4: 1/z perspective projection
 * ──────────────────────────────────────
 * To put a 3-D point on a 2-D screen we need to decide how big it
 * appears.  Things farther from the camera should appear smaller —
 * that's perspective.  The simplest formula:
 *
 *      screen_col = cols/2 + K1 · x / z
 *      screen_row = rows/2 − K1 · y / z · Y_ASPECT
 *
 * The DIVIDE BY Z is the whole point: a point with z = 5 has half
 * the projected size of a point with z = 2.5.  This single divide
 * is "1/z perspective" — the cheapest possible perspective system.
 *
 *      camera                            far things appear smaller:
 *        ●─────────● ─── screen plane            x     screen
 *         ╲       ╱                              ●
 *          ╲     ╱                          (a)  ●  ────────────●  ← x/z big
 *           ╲   ╱                                ●    z = 2
 *            ╲ ╱                          (b)  ●  ────────────────●  ← x/z small
 *      world point ●                            ●         z = 5
 *                                          two points with same
 *                                          x but different z
 *
 * The minus sign on screen_row is the standard screen-Y-flip
 * (terminal rows grow downward, world y grows upward).  Y_ASPECT
 * halves the vertical so the donut renders round on cells that are
 * twice as tall as wide.
 *
 * That's the work `project_to_screen(...)` does in §10.
 *
 *
 * TUTORIAL 5: What is a z-buffer (and why inverted)?
 * ──────────────────────────────────────────────────
 * Many (θ, φ) samples may project to the SAME terminal cell.
 * Without help, the LAST one to write wins — including dots that
 * are behind other dots.  We want the CLOSEST dot to win.
 *
 * Solution: a per-cell "z-buffer" remembers how close the closest
 * dot so far was.  When a new dot arrives, we keep it only if it's
 * closer.
 *
 *      cell (col, row) starts empty:    zbuf[idx] = 0
 *
 *      sample 1 lands here, z = 5:      ooz = 1/5 = 0.2
 *                                       0.2 > 0 → keep, zbuf = 0.2
 *
 *      sample 2 lands here, z = 8:      ooz = 1/8 = 0.125
 *                                       0.125 > 0.2 ? NO → reject
 *
 *      sample 3 lands here, z = 3:      ooz = 1/3 = 0.333
 *                                       0.333 > 0.2 ? YES → keep, zbuf = 0.333
 *
 * Note we store ONE OVER Z, not z.  Why?  Two reasons:
 *
 *   1. We already computed `1/z` for the projection (tutorial 4).
 *      Storing it skips a division per cell.
 *   2. With one-over-z, BIGGER means CLOSER — and zero means
 *      "infinitely far" (the natural empty-cell value).
 *
 * The convention is INVERTED relative to the textbook "smaller z =
 * closer" rule.  Forgetting the flip and writing `if (ooz < zbuf)`
 * is the most common z-buffer bug.
 *
 * That's the work `try_emit_pixel(...)` does in §12.
 *
 *
 * TUTORIAL 6: Lambertian lighting (closed-form on the torus)
 * ──────────────────────────────────────────────────────────
 * A surface looks brighter when it FACES the light.  Lambert's law
 * says brightness ∝ N · L, where N is the surface normal (a unit
 * vector pointing outward from the surface) and L is the light
 * direction (a unit vector toward the light).  The dot product
 * captures "how aligned" they are:
 *
 *      N · L = 1     (face directly into the light, brightest)
 *      N · L = 0     (perpendicular, no direct light)
 *      N · L < 0     (back-facing — we skip those)
 *
 * For a torus, the normal at any (θ, φ) has a clean closed form
 * (it's just the radial direction from the tube's centre).  After
 * applying the same Euler rotations that moved the surface point,
 * we get a surface normal in screen space, and we dot it with the
 * fixed light direction (0, 1, −1)/√2 (above and behind the
 * camera).  The whole calculation collapses to:
 *
 *      L = cos φ · cos θ · sin B
 *        − cos A · cos θ · sin φ
 *        − sin A · sin θ
 *        + cos B · (cos A · sin θ − cos θ · sin A · sin φ)
 *
 * Don't try to memorise this — that's why it has a function name.
 * What matters: the result is a number in [-√2, +√2] that tells us
 * how bright this surface patch is.  We clamp negatives to "skip
 * back-faces", and use the positive value to pick a glyph from the
 * 12-character ramp.
 *
 * That's the work `surface_luminance(...)` does in §11.
 *
 *
 * TUTORIAL 7: Why parametric sampling beats raymarching here
 * ──────────────────────────────────────────────────────────
 * For a torus we have a CLOSED-FORM equation: any (θ, φ) gives a
 * 3-D surface point in 25 floating-point ops.  Two for-loops cover
 * the whole surface in ~28 000 evaluations.
 *
 * Raymarching, by contrast, would do the OPPOSITE: for each pixel,
 * shoot a ray and step along it until you hit the surface.  Per-
 * pixel cost = many (often 30+) distance-estimator evaluations.
 *
 *      this file (parametric):                raymarching:
 *      ───────────────────────                ────────────
 *      for each (θ, φ):                       for each (col, row):
 *          point = formula(θ, φ)                  ray = pixel → world
 *          paint(point)                           march along ray
 *                                                 until hit or miss
 *
 *      cost = N_samples (fixed)               cost = pixels × march_steps
 *      best for KNOWN surfaces                best for IMPLICIT surfaces
 *      (sphere, torus, polygons)              (Mandelbulb, KIFS, SDF
 *                                              fractals, anything where
 *                                              "what is the nearest
 *                                              surface" is hard)
 *
 * Both pipelines exist in this codebase: see donut.c (this file)
 * and mandelbulb.c (raymarcher).  They're complementary tools.
 *
 *
 * TUTORIAL 8: Why is the φ-step finer than θ-step?
 * ────────────────────────────────────────────────
 * The tube has radius R1 (small).  The torus axis traces a big
 * circle of radius R2 (big).  Same NUMBER of samples around each
 * gives DIFFERENT density on the surface:
 *
 *      θ samples (around big circle, circumference 2π·R2):
 *          spacing per sample = 2π · R2 / N_theta
 *
 *      φ samples (around small circle, circumference 2π·R1):
 *          spacing per sample = 2π · R1 / N_phi
 *
 * With R2 = 2 and R1 = 1, equal sample counts mean the tube is
 * sampled at HALF the surface density of the axis.  Visually that
 * shows up as gaps on the tube before gaps appear on the axis.
 *
 * The fix: make φ-step proportionally smaller.  THETA_STEP = 0.07,
 * PHI_STEP = 0.02 — about 3.5× finer.  Both directions hit ~equal
 * surface density.
 *
 * Try setting THETA_STEP = 0.5 in §1 and recompiling.  You'll see a
 * sparse "wire cage" — proves the (θ, φ) loop is the only source
 * of sample density.
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

/* ── §1 config — every tunable, every magic number named ─────────────── *
 *
 * Every literal that has meaning lives here.  The principle: a
 * reader should be able to identify what a constant CONTROLS
 * without leaving §1.
 */

/* §1.1 — frame rate + UI layout. */
enum {
    SIM_FPS_DEFAULT  = 30,
    FPS_UPDATE_MS    = 500,
    HUD_STATUS_COLS  = 90,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(fps)    (NS_PER_SEC / (fps))

/* §1.2 — angles and π helpers. */
#define TWO_PI          (2.0f * (float)M_PI)

/* §1.3 — rotation speeds (radians per second).
 *
 * angle_A is the X-axis tumble (think: nodding the donut forward).
 * angle_B is the Z-axis spin   (think: rolling the donut in place).
 * Both rates multiply by SPEED_SCALE per ] keypress (and divide on
 * [).  This preserves the relative tumble:spin ratio.
 */
#define ROT_RATE_A_DEFAULT  1.2f
#define ROT_RATE_B_DEFAULT  0.6f
#define SPEED_SCALE         1.3f
#define SPEED_MIN           0.05f
#define SPEED_MAX          12.0f

/* §1.4 — torus geometry.
 *
 *   TORUS_R1  tube radius (small circle).
 *   TORUS_R2  axis radius — distance from torus centre to tube centre
 *             (big circle).  Visible as the "doughnut hole" radius:
 *             inner_radius = R2 − R1, outer_radius = R2 + R1.
 *   TORUS_K2  viewer distance — added to z so the donut sits in
 *             front of the camera (z always positive).
 */
#define TORUS_R1        1.0f
#define TORUS_R2        2.0f
#define TORUS_K2        5.0f

/* §1.5 — perspective sizing (auto-fit + user scale).
 *
 * K1 (perspective scaling) is computed per-frame so the projected
 * outer edge of the torus fills TORUS_TARGET_FILL of the smaller
 * terminal half-dimension.  TORUS_SIZE_SCALE is the user-controlled
 * multiplier on K1, adjustable via = / -.
 */
#define TORUS_TARGET_FILL   0.42f
#define TORUS_SIZE_SCALE    1.15f
#define TORUS_SIZE_MIN      0.30f
#define TORUS_SIZE_MAX      5.00f

/* §1.6 — sample density.
 *
 * THETA_STEP is the angular increment around the BIG circle (axis).
 * PHI_STEP   is the angular increment around the SMALL circle (tube).
 * PHI_STEP must be finer than THETA_STEP because the tube is a
 * smaller circle (see TUTORIAL 8).
 *
 *   N_θ samples  ≈  2π / 0.07  ≈  90
 *   N_φ samples  ≈  2π / 0.02  ≈  314
 *   total samples ≈ 28 000 per frame.
 */
#define THETA_STEP      0.07f
#define PHI_STEP        0.02f

/* §1.7 — terminal cell aspect correction.
 *
 *   Y_ASPECT = 0.5  for typical terminals (cells ~2× taller than wide)
 *   Y_ASPECT = 1.0  for square-cell terminals
 *
 * Applied as a multiplier on the projected y so the donut reads as a
 * round shape rather than a vertically stretched ellipse.
 */
#define Y_ASPECT        0.5f

/* §1.8 — luminance ramp.  Twelve glyphs, dim → bright.
 *
 * The render picks a glyph from L · LUMI_RAMP_LEN.  The colour pair
 * is picked from the same index quantised to LUMI_LEVELS slots.
 */
static const char LUMI_RAMP[] = ".,-~:;=!*#$@";
#define LUMI_RAMP_LEN   ((int)(sizeof LUMI_RAMP - 1))   /* = 12 */

/* §1.9 — colour-pair scheme.
 *
 *   PAIR_LUMI_BASE..+7   eight luma-shaded slots, dim → bright.  The
 *                        actual COLOURS at each slot come from the
 *                        active theme (§1.10) and can be re-pointed
 *                        at runtime by `theme_apply` (§3).
 *   PAIR_HUD             bright yellow (status row, CLAUDE.md spec)
 *   PAIR_HINT            bright cyan   (key-hint row, CLAUDE.md spec)
 */
enum {
    LUMI_LEVELS      = 8,
    PAIR_LUMI_BASE   = 1,                     /* +0..+7 */
    PAIR_HUD         = PAIR_LUMI_BASE + LUMI_LEVELS,    /* = 9  */
    PAIR_HINT        = PAIR_HUD + 1,                    /* = 10 */
};

/* §1.10 — colour themes.
 *
 * A theme is just a table of eight 256-colour codes — one per luma
 * slot, ordered dim → bright.  `theme_apply` (§3) re-points the eight
 * PAIR_LUMI_BASE pairs at the chosen theme's colours; nothing else
 * changes.  Geometry, shading, sample density — all unchanged.
 *
 * Per the CLAUDE.md theme-brightness rule, every entry sits in the
 * bright half of the 256-colour cube (≥ 24 in the 6×6×6 region or
 * ≥ 240 in the grayscale region).  Even the "darkest" tier of any
 * theme stays visible against a black terminal background.
 *
 * Press `t` (or `T`) to cycle.  CLASSIC is the default (slot 0).
 */
typedef struct {
    const char *display_name;
    short       ramp_256[LUMI_LEVELS];     /* slot 0 dim → slot 7 bright */
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    /* CLASSIC — eight greys in the bright half (240..255).
     * The original donut.c look — neutral, professional. */
    { "CLASSIC ",
      { 240, 243, 246, 248, 250, 252, 254, 255 } },

    /* AMBER — old phosphor-monitor amber, deep brown → bright gold.
     * Channels the 1980s green-screen-but-orange aesthetic. */
    { "AMBER   ",
      { 130, 136, 166, 172, 178, 208, 214, 220 } },

    /* MATRIX — eight green shades, deep moss → lime.  Cyberpunk. */
    { "MATRIX  ",
      {  28,  34,  40,  46,  82, 118, 154, 190 } },

    /* NEON — magenta → pink → fuchsia → cream-pink.  Synthwave. */
    { "NEON    ",
      {  53,  91, 129, 165, 201, 207, 213, 227 } },

    /* ICE — navy → bright cyan.  Cool / cold-storage look. */
    { "ICE     ",
      {  25,  31,  38,  45,  51,  87, 123, 159 } },

    /* COPPER — bronze → orange → amber.  Warm metallic. */
    { "COPPER  ",
      {  94, 130, 136, 166, 172, 208, 214, 220 } },
};

/* §1.11 — framebuffer sizing.
 *
 * Anything larger than TORUS_MAX_COLS × TORUS_MAX_ROWS is quietly
 * clipped — we'd need to malloc to avoid that, but 512 × 256 =
 * 131 072 cells handles every realistic terminal.
 */
#define TORUS_MAX_COLS    512
#define TORUS_MAX_ROWS    256
#define TORUS_MAX_CELLS   (TORUS_MAX_COLS * TORUS_MAX_ROWS)

/* §1.12 — debug overlays (educational helpers).
 *
 * Press `d` to cycle.  Each overlay swaps out the production
 * draw step for a different visualisation of one ASPECT of the
 * algorithm.  Reading the source for each overlay teaches what
 * the underlying signal LOOKS LIKE.
 *
 *   DEBUG_NORMAL    full lit donut (production view)
 *   DEBUG_DEPTH     colour by 1/z — see what the z-buffer knows
 *   DEBUG_WIRE      paint only every 10th sample — see the (θ,φ) grid
 *   DEBUG_NO_ZBUF   skip z-buffer test — see what z-buffer prevents
 */
typedef enum {
    DEBUG_NORMAL    = 0,
    DEBUG_DEPTH     = 1,
    DEBUG_WIRE      = 2,
    DEBUG_NO_ZBUF   = 3,
    DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ", "DEPTH  ", "WIRE   ", "NO_ZBUF",
};

#define DEBUG_WIRE_STRIDE  10        /* paint 1 in every 10 samples */

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── *
 *
 * CLOCK_MONOTONIC is guaranteed to advance at one second per second,
 * with no jumps from NTP or DST.  Perfect for "how long since the
 * last frame" measurements.
 */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t nanoseconds)
{
    if (nanoseconds <= 0) return;
    struct timespec request = {
        .tv_sec  = (time_t)(nanoseconds / NS_PER_SEC),
        .tv_nsec = (long)  (nanoseconds % NS_PER_SEC),
    };
    nanosleep(&request, NULL);
}

/* ── §3 color — luminance pairs + theme apply + HUD/hint pairs ───────── *
 *
 * Eight luma-shaded pairs (PAIR_LUMI_BASE..+7) hold the active
 * theme's 256-colour codes.  `theme_apply` re-points them at a
 * different theme; `lumi_attr` reads from PAIR_LUMI_BASE+slot to
 * paint a cell.
 *
 * In 8-colour-only terminals (rare nowadays) we fall back to
 * A_DIM/A_NORMAL/A_BOLD on COLOR_WHITE — themes have no effect.
 */

/*
 * theme_apply — re-init the eight luma pairs for the chosen theme.
 *
 *   theme_index  index into THEMES[] (clamps to 0 if out of range).
 *
 * Called once at startup (with index 0) and on every t/T keypress.
 * The eight PAIR_LUMI_BASE pairs simply get re-pointed at new
 * 256-colour codes; nothing else in the program changes.
 */
static void theme_apply(int theme_index)
{
    if (theme_index < 0 || theme_index >= THEME_COUNT) theme_index = 0;
    const Theme *theme = &THEMES[theme_index];

    if (COLORS >= 256) {
        for (int slot = 0; slot < LUMI_LEVELS; slot++)
            init_pair((short)(PAIR_LUMI_BASE + slot),
                      theme->ramp_256[slot], -1);
    } else {
        /* 8-colour fallback — themes have no effect; we layer
         * A_DIM/A_BOLD via lumi_attr to fake brightness levels. */
        for (int slot = 0; slot < LUMI_LEVELS; slot++)
            init_pair((short)(PAIR_LUMI_BASE + slot), COLOR_WHITE, -1);
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
    theme_apply(0);                      /* default to CLASSIC */
}

/*
 * lumi_attr — ncurses attribute for luma slot l ∈ [0, LUMI_LEVELS).
 *
 *   In 256-colour mode each slot has its own grey pair.
 *   In 8-colour mode we layer DIM/NORMAL/BOLD on top of the white
 *     pair so eight slots still produce three visibly distinct
 *     brightnesses.
 */
static attr_t lumi_attr(int slot)
{
    if (slot < 0)              slot = 0;
    if (slot > LUMI_LEVELS-1)  slot = LUMI_LEVELS - 1;

    attr_t attr = COLOR_PAIR(PAIR_LUMI_BASE + slot);
    if (COLORS < 256) {
        if      (slot < 3) attr |= A_DIM;
        else if (slot > 5) attr |= A_BOLD;
    }
    return attr;
}

/* ── §4 math — V2 / V3 / Rot constructors mirror the formulas ────────── *
 *
 * The vector types exist purely so call sites read like the math:
 *
 *     V2 tube  = tube_point(...);
 *     V3 world = surface_point(tube, cos_phi, sin_phi, rot);
 *
 * No operator overloading, no arithmetic helpers — just the
 * constructor and field access.  C does not get in the way of the
 * algebra.
 */

typedef struct { float x, y;       } V2;
typedef struct { float x, y, z;    } V3;

/*
 * Rot — the four trig values we need at every (θ, φ) sample.
 *
 * Computed ONCE per frame in torus_render and passed by value to the
 * helpers.  Avoids 56 000+ redundant sinf/cosf calls per frame
 * (each θ × each φ would re-evaluate sin_A and cos_A otherwise).
 */
typedef struct {
    float sin_A, cos_A;     /* Euler angle around X (tumble) */
    float sin_B, cos_B;     /* Euler angle around Z (spin)   */
} Rot;

static inline V2 v2(float x, float y)
{
    return (V2){ x, y };
}

static inline V3 v3(float x, float y, float z)
{
    return (V3){ x, y, z };
}

static inline Rot rot_make(float angle_A, float angle_B)
{
    return (Rot){
        .sin_A = sinf(angle_A), .cos_A = cosf(angle_A),
        .sin_B = sinf(angle_B), .cos_B = cosf(angle_B),
    };
}

/* ── §5 framebuffer — Torus state + buffers + clear ──────────────────── *
 *
 * The Torus owns every mutable scrap of state: the rotation angles,
 * rotation rates, user-controlled size scale, paused flag, current
 * terminal dimensions, and the three flat framebuffers.
 *
 * Memory layout — three parallel arrays indexed by `row * cols + col`:
 *
 *      cell index    →    row · cols + col
 *
 *      zbuf [idx]    →    1/z of the closest sample seen so far.
 *                         Zero means "infinitely far / empty".
 *      glyph[idx]    →    The character to print; ' ' = empty.
 *      luma [idx]    →    Luma slot 0..LUMI_LEVELS-1, used by the
 *                         draw step to pick a colour pair.  Stored
 *                         alongside the glyph so we don't have to
 *                         back-derive luma via strchr in the painter.
 *
 *      memory:
 *                ┌──────┬──────┬──────┬─── ... ───┬──────┐
 *      zbuf  →   │ 0.00 │ 0.32 │ 0.00 │           │ 0.51 │   floats
 *                ├──────┼──────┼──────┼─── ... ───┼──────┤
 *      glyph →   │  ' ' │ '#'  │  ' ' │           │ '@'  │   chars
 *                ├──────┼──────┼──────┼─── ... ───┼──────┤
 *      luma  →   │  0   │  6   │  0   │           │  7   │   uint8s
 *                └──────┴──────┴──────┴─── ... ───┴──────┘
 *      idx →        0      1      2                  N-1
 */

typedef struct {
    /* Rotation state. */
    float    angle_A;          /* tumble (around X), radians  */
    float    angle_B;          /* spin   (around Z), radians  */
    float    rot_rate_A;       /* tumble rate, rad/sec        */
    float    rot_rate_B;       /* spin   rate, rad/sec        */

    /* Size + control state. */
    float    k1_user_scale;    /* multiplier on K1 (=/- keys) */
    bool     paused;
    int      theme_index;      /* index into THEMES[] (t/T keys) */
    DebugMode debug_mode;

    /* Terminal dimensions. */
    int      cols, rows;

    /* Flat framebuffers. */
    float    zbuf [TORUS_MAX_CELLS];
    char     glyph[TORUS_MAX_CELLS];
    uint8_t  luma [TORUS_MAX_CELLS];
} Torus;

/*
 * torus_init — zero the buffers and seed the angles + rates.  Called
 * once at startup; resize doesn't re-init (the buffers are already
 * sized for the worst case).
 */
static void torus_init(Torus *torus, int cols, int rows)
{
    memset(torus, 0, sizeof *torus);
    torus->rot_rate_A    = ROT_RATE_A_DEFAULT;
    torus->rot_rate_B    = ROT_RATE_B_DEFAULT;
    torus->k1_user_scale = 1.0f;
    torus->cols          = cols;
    torus->rows          = rows;
    torus->theme_index   = 0;             /* CLASSIC */
    torus->debug_mode    = DEBUG_NORMAL;
}

/*
 * torus_resize — terminal grew or shrank; just update cols/rows.
 * The static buffers already hold enough room.
 */
static void torus_resize(Torus *torus, int cols, int rows)
{
    torus->cols = cols;
    torus->rows = rows;
}

/*
 * torus_clear_buffers — zero the three framebuffers.
 *
 *   zbuf  → 0   (= "infinitely far"; any real depth wins)
 *   glyph → ' ' (= empty cell, won't paint anything)
 *   luma  → 0   (irrelevant when glyph is space, but tidy)
 *
 * One memset call each.  Cheap.
 */
static void torus_clear_buffers(Torus *torus)
{
    int total_cells = torus->cols * torus->rows;
    memset(torus->zbuf,  0,   sizeof(float)   * (size_t)total_cells);
    memset(torus->glyph, ' ', sizeof(char)    * (size_t)total_cells);
    memset(torus->luma,  0,   sizeof(uint8_t) * (size_t)total_cells);
}

/* ── §6 perspective — K1 sizing (auto-fit) ───────────────────────────── *
 *
 * K1 (the perspective scale) is chosen so the projected outer edge
 * of the torus fills TORUS_TARGET_FILL of the smaller terminal half-
 * dimension at user scale 1.0.  Recomputed every frame so resize
 * updates the size automatically.
 *
 * Derivation.  The outer edge of the torus is at world-space
 * x = R2 + R1.  After the +K2 shift, that point is at distance
 * z = K2 + R2 + R1 from the camera.  The 1/z projection then
 * gives screen distance:
 *
 *     target_pixels  =  K1 · (R2 + R1) / (K2 + R2 + R1)
 *  ⇒  K1             =  target_pixels · (K2 + R2 + R1) / (R2 + R1)
 *
 * We pick `target_pixels = TARGET_FILL · min(cols/2, rows)` so the
 * donut never gets clipped on the smaller axis.  Multiplied by the
 * user-controlled `k1_user_scale` so = / - keys grow / shrink it.
 */

static float compute_k1(const Torus *torus)
{
    float half_width    = (float)torus->cols * 0.5f;
    float full_height   = (float)torus->rows;
    float min_half_dim  = (half_width < full_height ? half_width : full_height);
    float target_pixels = min_half_dim * TORUS_TARGET_FILL;

    float k1 = target_pixels * (TORUS_K2 + TORUS_R2 + TORUS_R1)
                             / (TORUS_R2 + TORUS_R1);
    return k1 * torus->k1_user_scale;
}

/* ── §7 tick — advance rotation angles ───────────────────────────────── *
 *
 * The whole "physics" of this demo is this:
 *
 *     angle_A += rot_rate_A · dt
 *     angle_B += rot_rate_B · dt
 *
 * rates are in rad/sec, dt is in seconds, so the visual rotation
 * speed is frame-rate independent.  No accumulator needed.
 *
 * Skipped while paused so the frame pipeline still renders the same
 * stationary donut every frame (useful for inspecting a held pose).
 */

static void torus_tick(Torus *torus, float dt_seconds)
{
    if (torus->paused) return;
    torus->angle_A += torus->rot_rate_A * dt_seconds;
    torus->angle_B += torus->rot_rate_B * dt_seconds;
}

/* ── §8 tube point — sample one point on the tube cross-section ──────── *
 *
 * See TUTORIAL 2.
 *
 *     tube.x = R2 + R1 · cos θ      (circle in the plane,
 *     tube.y =      R1 · sin θ       before revolving)
 *
 * The caller supplies cos θ and sin θ pre-computed; the inner φ-
 * loop uses the same θ for many φ values, so this saves trig.
 *
 * Returns: 2-D point on the tube's cross-section.
 *
 *   COORDINATE SYSTEM: pre-revolution.  This (cx, cy) is the point's
 *   position in the (x, y) plane BEFORE we sweep it around the y
 *   axis by φ.  After revolution it'll have all three of (x, y, z).
 */

static V2 tube_point(float cos_theta, float sin_theta)
{
    return v2(TORUS_R2 + TORUS_R1 * cos_theta,
                         TORUS_R1 * sin_theta);
}

/* ── §9 surface point — full 3-D surface position ────────────────────── *
 *
 * See TUTORIAL 3.
 *
 * Conceptually three steps composed:
 *   1. Take the tube point (cx, cy) at angle θ.
 *   2. Revolve around the y axis by angle φ — this places a 3-D
 *      point on the un-rotated torus surface.
 *   3. Rotate that 3-D point by A around X, then by B around Z,
 *      then translate by +K2 along z.
 *
 * The composition of those rotations expanded out gives the
 * explicit formulas below — same as in Sloane's original donut.c.
 *
 *   COORDINATE SYSTEM: world space, with the camera at the origin
 *   looking down +z, and the donut centred at z = K2.
 *
 * `cos_phi` / `sin_phi` are passed in (not computed here) so the
 * caller's inner φ-loop can compute them once and reuse them for
 * both the surface point AND the luminance.
 */

static V3 surface_point(V2 tube, float cos_phi, float sin_phi, Rot rot)
{
    return v3(
        tube.x * (rot.cos_B * cos_phi + rot.sin_A * rot.sin_B * sin_phi)
      - tube.y *  rot.cos_A * rot.sin_B,

        tube.x * (rot.sin_B * cos_phi - rot.sin_A * rot.cos_B * sin_phi)
      + tube.y *  rot.cos_A * rot.cos_B,

        TORUS_K2 + rot.cos_A * tube.x * sin_phi + tube.y * rot.sin_A);
}

/* ── §10 project to screen — perspective divide + bounds ─────────────── *
 *
 * See TUTORIAL 4.
 *
 *     one_over_z  = 1 / z              (one-over-z; bigger = closer)
 *     screen_col  =  cols/2 + K1 · one_over_z · x
 *     screen_row  =  rows/2 − K1 · one_over_z · y · Y_ASPECT
 *
 * The minus sign on `screen_row` is the standard screen-Y-flip
 * (terminal rows grow downward, world y grows upward).  Y_ASPECT
 * corrects for cells being roughly twice as tall as wide.
 *
 *   COORDINATE SYSTEMS at play:
 *     – world space (input)        x, y, z all in float units
 *     – projected pixel space      screen_col, screen_row in ints
 *     – the framebuffer index      computed elsewhere as row · cols + col
 *
 * Returned alongside the integer pixel coords so the caller can:
 *   – skip out-of-bounds samples cheaply (`in_bounds`)
 *   – feed `one_over_z` directly into the z-buffer compare without
 *     recomputing it (which would be 1/z_actual; expensive).
 */

typedef struct {
    int   col;          /* projected pixel column, in [0, cols) when in_bounds */
    int   row;          /* projected pixel row,    in [0, rows) when in_bounds */
    float one_over_z;   /* 1/z — bigger = closer (z-buffer feeds this directly) */
    bool  in_bounds;
} ScreenPos;

static ScreenPos project_to_screen(V3 world, int cols, int rows, float K1)
{
    float one_over_z = 1.0f / world.z;
    int   col = (int)((float)cols * 0.5f + K1 * one_over_z * world.x);
    int   row = (int)((float)rows * 0.5f - K1 * one_over_z * world.y * Y_ASPECT);
    bool  in_bounds = (col >= 0 && col < cols && row >= 0 && row < rows);
    return (ScreenPos){ col, row, one_over_z, in_bounds };
}

/* ── §11 surface luminance — Lambert dot product (closed form) ───────── *
 *
 * See TUTORIAL 6.
 *
 *     L = cos φ · cos θ · sin B
 *       − cos A · cos θ · sin φ
 *       − sin A · sin θ
 *       + cos B · (cos A · sin θ − cos θ · sin A · sin φ)
 *
 * This is N · light_direction where:
 *   – N is the (rotated) surface normal at this (θ, φ) sample,
 *   – light_direction is the fixed vector (0, 1, −1)/√2 (above and
 *     behind the camera) from Sloane's original.
 *
 * The dot product expands out to a sum of products of rotation
 * sines/cosines (the same ones used for surface_point), which is
 * why it has a closed form — no per-pixel matrix math needed.
 *
 * Range: roughly [-√2, +√2].  The caller treats L ≤ 0 as a back-
 * facing surface (skip).  L > 0 maps to a glyph-ramp slot.
 */

static float surface_luminance(float cos_theta, float sin_theta,
                                float cos_phi,   float sin_phi,
                                Rot rot)
{
    return cos_phi  * cos_theta * rot.sin_B
         - rot.cos_A * cos_theta * sin_phi
         - rot.sin_A * sin_theta
         + rot.cos_B * (rot.cos_A * sin_theta - cos_theta * rot.sin_A * sin_phi);
}

/* ── §12 emit pixel — z-buffer test + glyph store ────────────────────── *
 *
 * See TUTORIAL 5.
 *
 * Three short-circuits:
 *
 *   1. L ≤ 0          surface faces away from light, contributes nothing
 *   2. !in_bounds     projected pixel is off-screen
 *   3. one_over_z ≤ zbuf[idx]   a closer sample already won this cell
 *
 * If all three pass, write the glyph + luma slot.
 *
 * The z-buffer convention: we store 1/z, BIGGER means CLOSER.  If
 * you read this code expecting the textbook "smaller z wins"
 * convention, you'll think the comparison is wrong — see TUTORIAL 5.
 *
 * The `force_overwrite` parameter is for the DEBUG_NO_ZBUF overlay
 * (skip the depth test).  In production it's always false.
 */

static void try_emit_pixel(Torus *torus, ScreenPos sp, float L,
                            bool force_overwrite)
{
    if (L <= 0.0f || !sp.in_bounds) return;

    int cell_index = sp.row * torus->cols + sp.col;
    if (!force_overwrite && sp.one_over_z <= torus->zbuf[cell_index]) {
        return;     /* lost the depth test */
    }

    /* Pick a glyph slot from the luminance.  L is clamped to the
     * [0, ramp_len) range; the cast truncates the fractional part. */
    int glyph_slot = (int)(L * (float)LUMI_RAMP_LEN);
    if (glyph_slot < 0)                  glyph_slot = 0;
    if (glyph_slot >= LUMI_RAMP_LEN)     glyph_slot = LUMI_RAMP_LEN - 1;

    torus->zbuf [cell_index] = sp.one_over_z;
    torus->glyph[cell_index] = LUMI_RAMP[glyph_slot];

    /* Map the 0..LUMI_RAMP_LEN-1 glyph slot down to the 0..LUMI_LEVELS-1
     * colour slot — the painter (§14) reads this. */
    torus->luma[cell_index] =
        (uint8_t)((glyph_slot * LUMI_LEVELS) / LUMI_RAMP_LEN);
}

/* ── §13 render — orchestrator that walks (θ, φ) and calls §8..§12 ───── *
 *
 * The pseudocode form is what the §8..§12 helpers were named after:
 *
 *     for theta in [0, 2π) step THETA_STEP:
 *         tube  = tube_point(cos θ, sin θ)
 *         for phi in [0, 2π) step PHI_STEP:
 *             world  = surface_point(tube, cos φ, sin φ, rot)
 *             screen = project_to_screen(world, cols, rows, K1)
 *             L      = surface_luminance(cos θ, sin θ, cos φ, sin φ, rot)
 *             try_emit_pixel(torus, screen, L)
 *
 * The body below mirrors that pseudocode line-for-line.  Five
 * named verbs per inner-loop iteration.  No buried logic.
 *
 * The `wire_stride` argument supports the DEBUG_WIRE overlay: when
 * set to N > 1, only every Nth (θ, φ) sample is emitted.  In
 * production it's 1 (every sample emitted).
 *
 * The `force_overwrite` argument supports DEBUG_NO_ZBUF: when true,
 * the z-buffer test in try_emit_pixel is skipped.
 */

static void torus_render(Torus *torus, int wire_stride, bool force_overwrite)
{
    torus_clear_buffers(torus);

    Rot   rot = rot_make(torus->angle_A, torus->angle_B);
    float K1  = compute_k1(torus);

    int sample_index = 0;
    for (float theta = 0.f; theta < TWO_PI; theta += THETA_STEP) {
        float cos_theta = cosf(theta);
        float sin_theta = sinf(theta);
        V2    tube      = tube_point(cos_theta, sin_theta);

        for (float phi = 0.f; phi < TWO_PI; phi += PHI_STEP) {
            sample_index++;

            /* WIRE overlay: skip most samples to expose the (θ,φ) grid. */
            if (wire_stride > 1 && (sample_index % wire_stride) != 0)
                continue;

            float cos_phi = cosf(phi);
            float sin_phi = sinf(phi);

            V3        world = surface_point   (tube, cos_phi, sin_phi, rot);
            ScreenPos sp    = project_to_screen(world,
                                                torus->cols, torus->rows, K1);
            float     L     = surface_luminance(cos_theta, sin_theta,
                                                cos_phi,   sin_phi,   rot);

            try_emit_pixel(torus, sp, L, force_overwrite);
        }
    }
}

/* ── §14 draw — paint the framebuffer to the terminal ────────────────── *
 *
 * Walk every cell in row order; emit any non-empty glyph with its
 * stored luma slot picking the colour pair.  Empty cells are skipped
 * (they keep whatever was there from `erase()`).
 *
 * The luma slot was stored alongside the glyph by `try_emit_pixel`,
 * so the painter doesn't need to back-derive it (no strchr search).
 */

static void torus_draw(const Torus *torus, WINDOW *window)
{
    int cols = torus->cols;
    int total_cells = cols * torus->rows;

    for (int idx = 0; idx < total_cells; idx++) {
        char glyph = torus->glyph[idx];
        if (glyph == ' ') continue;

        attr_t attr = lumi_attr(torus->luma[idx]);
        wattron(window, attr);
        mvwaddch(window, idx / cols, idx % cols,
                 (chtype)(unsigned char)glyph);
        wattroff(window, attr);
    }
}

/* ── §15 debug overlays — see what the algorithm is doing ────────────── *
 *
 * Press `d` to cycle.  Each overlay teaches a different aspect:
 *
 *   DEBUG_NORMAL    full lit donut.  This is the production view.
 *
 *   DEBUG_DEPTH     paint glyphs by 1/z — closer = brighter.  You
 *                   literally see what the z-buffer knows.  The
 *                   front-most ring of the donut shows up brightest;
 *                   the rear surface (which lost the z-buffer fight)
 *                   isn't drawn at all.
 *
 *   DEBUG_WIRE      paint only every WIRE_STRIDE-th sample.  You see
 *                   a sparse cage of dots — proves the (θ, φ) loop
 *                   is the only source of sample density, and shows
 *                   how the dots cluster on the rear of the tube
 *                   (where samples bunch up due to the perspective
 *                   foreshortening).
 *
 *   DEBUG_NO_ZBUF   skip the z-buffer test.  Both front and back
 *                   surfaces paint, in undefined order — proves
 *                   why we need the z-buffer.  The donut becomes a
 *                   semi-transparent mesh of clashing samples.
 *
 * For DEPTH we re-run the render to fill zbuf[] (so we can read
 * 1/z values), then paint glyphs by reading zbuf[] directly.
 */

static void render_debug_depth(Torus *torus, WINDOW *window)
{
    /* Production render fills zbuf[] for us. */
    torus_render(torus, 1, false);

    int cols = torus->cols;
    int total_cells = cols * torus->rows;

    /* Find the maximum 1/z value (the closest sample) for normalisation. */
    float max_one_over_z = 0.0f;
    for (int idx = 0; idx < total_cells; idx++)
        if (torus->zbuf[idx] > max_one_over_z)
            max_one_over_z = torus->zbuf[idx];
    if (max_one_over_z < 1e-6f) return;

    /* Paint each filled cell with a depth-based glyph. */
    for (int idx = 0; idx < total_cells; idx++) {
        if (torus->zbuf[idx] < 1e-6f) continue;

        float depth_normalised = torus->zbuf[idx] / max_one_over_z;
        int   slot = (int)(depth_normalised * (float)(LUMI_RAMP_LEN - 1) + 0.5f);
        if (slot < 0)                slot = 0;
        if (slot >= LUMI_RAMP_LEN)   slot = LUMI_RAMP_LEN - 1;

        char glyph = LUMI_RAMP[slot];
        attr_t attr = lumi_attr((slot * LUMI_LEVELS) / LUMI_RAMP_LEN);
        wattron(window, attr);
        mvwaddch(window, idx / cols, idx % cols,
                 (chtype)(unsigned char)glyph);
        wattroff(window, attr);
    }
}

static void render_debug_wire(Torus *torus, WINDOW *window)
{
    /* Same render path, but only every Nth sample is emitted. */
    torus_render(torus, DEBUG_WIRE_STRIDE, false);
    torus_draw  (torus, window);
}

static void render_debug_no_zbuf(Torus *torus, WINDOW *window)
{
    /* Same render path, but z-buffer test is skipped — front and
     * back samples both paint, in iteration order.  You can see the
     * back surface bleed through the front. */
    torus_render(torus, 1, true);
    torus_draw  (torus, window);
}

/*
 * draw_active_view — dispatch by debug mode.
 */
static void draw_active_view(Torus *torus, WINDOW *window)
{
    switch (torus->debug_mode) {
    case DEBUG_NORMAL:
        torus_render(torus, 1, false);
        torus_draw  (torus, window);
        break;
    case DEBUG_DEPTH:    render_debug_depth   (torus, window); break;
    case DEBUG_WIRE:     render_debug_wire    (torus, window); break;
    case DEBUG_NO_ZBUF:  render_debug_no_zbuf (torus, window); break;
    default:
        torus_render(torus, 1, false);
        torus_draw  (torus, window);
        break;
    }
}

/* ── §16 screen — ncurses init / resize / HUD draw / present ─────────── */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *screen)
{
    initscr();
    noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_free(Screen *screen) { (void)screen; endwin(); }

static void screen_resize(Screen *screen)
{
    endwin(); refresh();
    getmaxyx(stdscr, screen->rows, screen->cols);
}

/*
 * screen_draw — full frame: torus then HUD overlay.
 *
 * HUD layout (CLAUDE.md spec):
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 */
static void screen_draw(Screen *screen, Torus *torus, double fps)
{
    erase();
    draw_active_view(torus, stdscr);

    /* Top row — yellow status. */
    char status[HUD_STATUS_COLS + 1];
    snprintf(status, sizeof status,
             " %5.1f fps  spd:%4.2f  size:%4.2f  theme:%s  debug:%s%s ",
             fps, (double)torus->rot_rate_A,
             (double)torus->k1_user_scale,
             THEMES[torus->theme_index].display_name,
             DEBUG_MODE_NAMES[torus->debug_mode],
             torus->paused ? "  PAUSED" : "");
    int slen = (int)strlen(status); if (slen > screen->cols) slen = screen->cols;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, screen->cols - slen, "%s", status);
    mvprintw(0, 0, " DONUT ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom row — cyan key hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(screen->rows - 1, 0,
             " q:quit  spc:pause  ]/[:speed  +/-:size  t/T:theme  d/D:debug ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §17 app — main loop, signals, key handling, cleanup ─────────────── */

typedef struct {
    Torus                 torus;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup         (void)    { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    torus_resize (&app->torus, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/*
 * app_handle_key — returns false to quit.  Keys are listed in the
 * file header.
 */
static bool app_handle_key(App *app, int ch)
{
    Torus *torus = &app->torus;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':                              torus->paused = !torus->paused; break;

    case ']':
        torus->rot_rate_A *= SPEED_SCALE;
        torus->rot_rate_B *= SPEED_SCALE;
        if (torus->rot_rate_A > SPEED_MAX) torus->rot_rate_A = SPEED_MAX;
        if (torus->rot_rate_B > SPEED_MAX) torus->rot_rate_B = SPEED_MAX;
        break;

    case '[':
        torus->rot_rate_A /= SPEED_SCALE;
        torus->rot_rate_B /= SPEED_SCALE;
        if (torus->rot_rate_A < SPEED_MIN) torus->rot_rate_A = SPEED_MIN;
        if (torus->rot_rate_B < SPEED_MIN) torus->rot_rate_B = SPEED_MIN;
        break;

    case '=': case '+':
        torus->k1_user_scale *= TORUS_SIZE_SCALE;
        if (torus->k1_user_scale > TORUS_SIZE_MAX) torus->k1_user_scale = TORUS_SIZE_MAX;
        break;

    case '-':
        torus->k1_user_scale /= TORUS_SIZE_SCALE;
        if (torus->k1_user_scale < TORUS_SIZE_MIN) torus->k1_user_scale = TORUS_SIZE_MIN;
        break;

    case 't':
        torus->theme_index = (torus->theme_index + 1) % THEME_COUNT;
        theme_apply(torus->theme_index);
        break;
    case 'T':
        torus->theme_index =
            (torus->theme_index + THEME_COUNT - 1) % THEME_COUNT;
        theme_apply(torus->theme_index);
        break;

    case 'd':
        torus->debug_mode =
            (DebugMode)((torus->debug_mode + 1) % DEBUG_MODE_COUNT);
        break;
    case 'D':
        torus->debug_mode =
            (DebugMode)((torus->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    torus_init (&app->torus, app->screen.cols, app->screen.rows);

    /*
     * dt-loop state — same scaffold as the rest of the project.
     *
     * Pseudo-code:
     *   while running:
     *       handle resize
     *       compute dt
     *       advance simulation by dt (fixed step)
     *       update fps counter
     *       sleep until next frame deadline
     *       render screen
     *       handle key input
     */
    int64_t frame_time_ns = clock_ns();
    int64_t sim_accumulator_ns = 0;
    int64_t fps_accumulator_ns = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time_ns      = clock_ns();
            sim_accumulator_ns = 0;
        }

        /* ── compute real dt since last frame ────────────────────── */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - frame_time_ns;
        frame_time_ns  = now_ns;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;

        /* ── fixed-step sim accumulator ──────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accumulator_ns += dt_ns;
        while (sim_accumulator_ns >= tick_ns) {
            torus_tick(&app->torus, dt_sec);
            sim_accumulator_ns -= tick_ns;
        }

        /* ── fps counter (rolling window) ────────────────────────── */
        frame_count++;
        fps_accumulator_ns += dt_ns;
        if (fps_accumulator_ns >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accumulator_ns / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accumulator_ns = 0;
        }

        /* ── frame cap (sleep BEFORE I/O so writes don't drift) ──── */
        int64_t elapsed_ns = clock_ns() - frame_time_ns + dt_ns;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed_ns);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw   (&app->screen, &app->torus, fps_display);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
