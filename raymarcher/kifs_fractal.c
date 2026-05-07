/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * kifs_fractal.c — Kaleidoscopic IFS fractals (Knighty 2010)
 *
 * DEMO: Three preset fractals built from "fold + contract" iteration.
 *       TETRA = Sierpinski tetrahedron, MENGER = Menger sponge,
 *       KIFS_ROT = an animated rotating crystal.  All three share
 *       the same renderer; only the per-iteration fold differs.
 *       Six colour themes, four debug overlays, manual + auto orbit,
 *       live iteration depth control.
 *
 * Study alongside: raymarcher/mandelbulb.c (also a fractal SDF, but
 *       built from a single non-linear iteration rather than a
 *       composition of linear folds + scales).  Both files use the
 *       same sphere-trace skeleton; the SDF is what differs.
 *
 * Section map:
 *   §1   config       — every tunable named, no magic numbers later
 *   §2   clock        — monotonic timer + sleep
 *   §3   color        — orbit-trap palette + HUD/hint pairs
 *   §4   vec3         — 3-D math, value types
 *   §5   KifsParams   — per-frame view of all DE parameters
 *   §6   fold helpers — three preset-specific reflections
 *   §7   fold dispatch — pick the right helper per preset
 *   §8   contract     — the contractive map after each fold
 *   §9   menger fold-back — MENGER-only post-step
 *   §10  orbit trap   — track running min |p|² during folding
 *   §11  primitive DEs — sphere + box (Quílez form)
 *   §12  DE orchestrator — the kifs_de loop body
 *   §13  normal       — central-difference gradient of DE
 *   §14  sphere trace — Hart 1996 march along a ray
 *   §15  camera       — orthonormal basis + per-pixel ray
 *   §16  lighting     — Lambert + step-count AO
 *   §17  cell decoration + emit
 *   §18  scene        — Scene struct + tick + build_kifs
 *   §19  render       — orchestrator: walk pixels, trace, decorate
 *   §20  debug overlays — see the raw rendering signals
 *   §21  screen       — ncurses init / HUD / present
 *   §22  app          — main loop, signals, key handling
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume orbit + KIFS_ROT animation
 *   r / R      reset camera + iteration depth
 *   n / N      next / previous preset (TETRA / MENGER / KIFS_ROT)
 *   t / T      next / previous theme
 *                (GOLD / ICE / COBALT / COPPER / ALIEN / MONO)
 *   d / D      cycle debug overlay
 *                (NORMAL / TRAP / STEPS / NORMALS)
 *   i / I      iteration depth − / +   (3..18)
 *   z / Z      camera farther / closer
 *   arrows     manual orbit (left/right yaw, up/down pitch)
 *   ] / [      simulation rate up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/kifs_fractal.c \
 *       -o kifs -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is structured as a textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         names the algorithm and lists references.
 *   • MENTAL MODEL     intuition + an ASCII diagram of one iteration.
 *   • GUIDED TUTORIAL  eight short answers walking from "what is an
 *                      IFS?" to "how does this become a renderable SDF?"
 *   • §1..§22          the actual code, each section short and focused.
 *
 * Ten-minute version: read the GUIDED TUTORIAL.  By the end the
 * §-sections feel like reviewing notes.
 *
 * Math notation used in code:
 *      p              — a 3-D point (the iteration / DE input)
 *      offset         — the contraction's fixed point
 *      scale          — the contraction multiplier (> 1 in our presets)
 *      iters          — iteration depth (3..18)
 *      DE(p)          — distance estimate at point p
 *      trap           — orbit trap value (smallest |p| during folding)
 *      N              — surface normal at the hit
 *      L              — light direction (unit vector)
 *
 * Background you need:
 *   • basic vector arithmetic (add, dot, cross, length, normalise)
 *   • read raymarcher.c first for sphere tracing if unfamiliar
 *   • optional but helpful: the Sierpinski triangle "chaos game"
 *     — the canonical 2D IFS that motivates this whole approach
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : KALEIDOSCOPIC ITERATED FUNCTION SYSTEM (KIFS,
 *                Knighty 2010).  An ITERATED FUNCTION SYSTEM is a
 *                set of CONTRACTIVE MAPS whose attractor is the
 *                fractal: every point in space, when iterated under
 *                random selections of the maps, eventually lands on
 *                the fractal (the "chaos game" theorem of Barnsley).
 *
 *                The Kaleidoscopic part adds REFLECTIONS (folds)
 *                before each contraction.  A fold collapses many
 *                regions of space onto one; the contraction then
 *                pulls everything toward a fixed point.  After
 *                ~10 iterations, the point's position is dominated
 *                by which sequence of folds + contractions it went
 *                through — and that sequence is the "address" of
 *                the attractor leaf it converges to.
 *
 *                For RENDERING via sphere tracing we need a
 *                distance estimator, not just "did this converge?".
 *                The trick: run the iteration N times, evaluate
 *                a simple primitive DE (sphere or box) at the final
 *                point, then divide by scale^N to undo the scaling
 *                and get back to world units.
 *
 *                Per pixel:
 *                  1. ray from camera through cell
 *                  2. sphere-trace using kifs_de
 *                  3. on hit:  6-tap central-difference normal
 *                              orbit trap value (smallest |p| during
 *                                                 fold loop)
 *                              Lambert + step-count AO → luminance
 *                  4. (luminance, trap, theme) → glyph + colour pair
 *
 * Data         : Stateless math on the hot path (no globals, no
 *                allocation).  Per-frame `KifsParams` packs all DE
 *                parameters so the inner loop touches one cache
 *                line.  Per-pixel result is `Hit` (hit / p / normal
 *                / trap / steps); rendered to one (glyph, colour
 *                pair, attr) `Cell` per terminal cell.
 *
 * Rendering    : One ray per terminal cell (no virtual canvas).
 *                Glyph from an 8-step luma ramp; colour from the
 *                active theme's 8-tier orbit-trap ramp.
 *                attron/attroff batched on (pair, attr) change so
 *                uniform regions don't thrash ncurses.
 *
 * Performance  : ~iters folds + 1 primitive DE per DE call.  Hit
 *                pixels: 1 trace step at hit + 6-tap normal + 1
 *                trap-extract = ~8 DE calls × iters folds ≈ ~70
 *                fold ops per hit pixel.  At iters=10 and modest
 *                terminals, holds 60+ fps.
 *
 * References   :
 *   • Knighty (2010) — "Kaleidoscopic (escape time) IFS"
 *     Fractal Forums thread originating the technique.
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer* 12(10):527-545.  The march loop.
 *   • Barnsley, M. (1988) — "Fractals Everywhere".  IFS theory and
 *     the chaos game.
 *   • Quílez, I. — "Distance Functions"
 *     https://iquilezles.org/articles/distfunctions/  (sphere/box
 *     primitives we evaluate at the end of the iteration).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take a point in 3D space.  Fold it (reflect across a few planes).
 * Then pull it toward a fixed point by a contraction (multiply by
 * scale, shift by offset).  Repeat 10× or so.  After a few iterations
 * the point's trajectory has been "kneaded" by the fold-and-scale
 * dynamics until it lands somewhere on the fractal's attractor.
 * For SDF rendering we don't run the iteration to convergence — we
 * run it a fixed number of times, evaluate a simple primitive (a
 * sphere or box) at the final point, and divide by scale^iterations
 * to bring the distance back to world units.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine an origami artist who knows three folds.  Hand them a flat
 * sheet of paper labelled "I am here".  They fold it three times,
 * then shrink the result toward a magnet.  Hand the shrunken result
 * back as input and they repeat.  After a dozen rounds the paper
 * has been folded into something neither flat nor random — it's
 * been driven onto an ATTRACTOR by the dynamics of the fold-and-
 * shrink rule.  KIFS does this in 3D space, with planes-of-symmetry
 * for folds and a fixed point + scale for the shrink.
 *
 * One iteration of the TETRA preset, in cross-section:
 *
 *      input p ●                                         FOLD step
 *               \                                        ┌─────────┐
 *                ●  if (p.x + p.y) < 0:                  │ reflect │
 *                 \    swap (p.x, p.y) → (−p.y, −p.x)   │ across  │
 *                  ●                                     │ 3 planes │
 *                                                        └─────────┘
 *               folded p                                       │
 *                       ●                                      ▼
 *                                                       CONTRACT step
 *                                                       p ← p · scale
 *                                                            − offset · (scale−1)
 *               result ●                                           │
 *                                                                  ▼
 *                                                           [next iter]
 *
 * Repeat ~10 times → attractor leaf.  The DE at the leaf is just
 * sphere_de or box_de (depending on preset) divided by scale^iters.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per frame:
 *   1. orbit          advance auto-yaw and KIFS_ROT angle (paused
 *                     freezes both)
 *   2. build_kifs     pack Scene state → KifsParams (cache scale^−iters)
 *
 * Once per pixel:
 *   3. ray            primary ray from camera through the cell
 *   4. trace          sphere-march using kifs_de
 *   5. on hit:        6-tap central-difference normal
 *                     re-evaluate kifs_de_with_trap (extract orbit trap)
 *                     Lambert + step-count AO → luminance
 *
 * Once per cell:
 *   6. quantise       luminance → glyph slot, trap → colour slot
 *   7. emit           with attron/attroff batched on attr change
 *
 * KEY FORMULAS
 * ────────────
 * Fold (TETRA — three plane reflections):
 *      if (p.x + p.y < 0):  (p.x, p.y) ← (−p.y, −p.x)
 *      if (p.x + p.z < 0):  (p.x, p.z) ← (−p.z, −p.x)
 *      if (p.y + p.z < 0):  (p.y, p.z) ← (−p.z, −p.y)
 *
 * Fold (MENGER — abs + descending sort):
 *      p ← (|p.x|, |p.y|, |p.z|)
 *      sort so p.x ≥ p.y ≥ p.z
 *
 * Fold (KIFS_ROT — Y-rotation + abs + 1 swap):
 *      (p.x, p.z) ← rotate by fold_rot around y
 *      p ← (|p.x|, |p.y|, |p.z|)
 *      if p.x < p.y: swap(p.x, p.y)
 *
 * Contraction (toward fixed point `offset`, multiplier `scale`):
 *      p ← p · scale  −  offset · (scale − 1)
 *      verify: offset is the fixed point: offset·scale − offset·(scale−1) = offset
 *
 * KIFS distance estimator (after iters folds + contracts):
 *      DE(p) = primitive_de(p_final) · scale^(−iters)
 *
 * Orbit trap (running min over iterations):
 *      trap = √( min over i of |p_i|² )
 *
 * Sphere trace:
 *      t = 0
 *      repeat MAX_STEPS:
 *          d = kifs_de(ro + t·rd)
 *          if d < HIT_EPS:   hit, return t
 *          if t > MAX_T:     miss
 *          t += d
 *
 * Lambert + AO:
 *      lum   = AMBIENT_LUM + DIFFUSE_LUM · max(0, N·L)
 *      ao    = max(AO_FLOOR, 1 − steps / MAX_STEPS)
 *      final = lum · ao
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • The KIFS DE is a CONSERVATIVE LOWER BOUND on true distance,
 *     not an exact distance like sphere_de or box_de.  Sphere
 *     tracing tolerates this — never overshoots — but march steps
 *     can shrink unhelpfully near the surface.  HIT_EPS = 1.5e-3
 *     is generous; tighter values increase step count without
 *     visible quality.
 *
 *   • TETRA needs HIGH iters (12) to look fractal — at 5 it looks
 *     like a smoothly bevelled tetrahedron.  MENGER converges
 *     faster and uses 7.  KIFS_ROT lives between (10).
 *
 *   • The MENGER preset includes a "z fold-back" heuristic (§9)
 *     that pulls p.z back into the box-DE's expected range.
 *     Without it the central column reads the wrong distance and
 *     the recursion ladder visibly breaks (large gaps on one side).
 *
 *   • Camera distance < CAM_DIST_MIN (1.6) puts the eye inside the
 *     fractal hull — the marcher then starts with negative DE and
 *     does weird things.  The clamp prevents that case.
 *
 *   • At ITERS_MAX (18) floating-point precision starts to limit
 *     detail — beyond that, more iterations don't reveal more
 *     structure, just noise.
 *
 *   • Pause freezes the orbit AND the KIFS_ROT animation, so
 *     KIFS_ROT renders as a static crystal at the current angle.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press n to cycle through TETRA → MENGER → KIFS_ROT.  Each
 *     should look distinctly different even at default iters.
 *
 *   • TETRA at iters=12 should show a clear Sierpinski-tetrahedron
 *     hierarchy: four large lobes, each subdivided into four smaller
 *     ones, recursively.  Pause and orbit (arrows) to inspect.
 *
 *   • MENGER at iters=7 should show the cubic Menger sponge: three
 *     square holes through the centre of each face, with smaller
 *     holes through each remaining cube.
 *
 *   • Press i to drop iters to 3.  All three presets become smooth
 *     blobs — fractal detail emerges only at higher iterations.
 *
 *   • Press d to cycle debug overlays.  TRAP shows where the colour
 *     ramp comes from (ignores lighting).  STEPS shows the AO source
 *     (deeper concavities glow).  NORMALS shows raw geometry hue.
 *
 *   • Press t through all 6 themes.  Geometry stays identical; only
 *     the orbit-trap colour ramp changes.
 *
 *   • For KIFS_ROT, watch the fractal slowly morph as the fold_rot
 *     angle advances — the rotation is what gives this preset its
 *     "growing crystal" appearance.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — eight short answers ───────────────────────────── *
 *
 *
 * T1: What is an Iterated Function System?
 * ────────────────────────────────────────
 * An IFS is a small set of CONTRACTIVE MAPS f₁, f₂, …, fₙ acting on
 * a metric space.  Each map shrinks distances (|fᵢ(a) − fᵢ(b)| <
 * c·|a − b| for some c < 1).  Barnsley's chaos-game theorem: pick
 * any starting point, repeatedly apply a RANDOMLY CHOSEN map, and
 * the trajectory of points eventually fills out the same fractal
 * — the IFS's ATTRACTOR — regardless of starting point.
 *
 * The classic 2D example: Sierpinski triangle.  Three maps, each
 * "halve the distance to one of three vertices":
 *
 *      f₁(p) = (p + v₁) / 2
 *      f₂(p) = (p + v₂) / 2
 *      f₃(p) = (p + v₃) / 2
 *
 * Iterate from any starting point, picking f₁/f₂/f₃ randomly each
 * step, and within ~50 iterations you've drawn the Sierpinski
 * triangle.
 *
 *
 * T2: From IFS to KIFS — adding folds
 * ───────────────────────────────────
 * The IFS approach (random map per iteration) is great for plotting
 * a fractal point-by-point but bad for ray-tracing — we don't want
 * stochasticity in our DE.  KIFS uses a DIFFERENT trick: a single
 * DETERMINISTIC pipeline of FOLD + CONTRACT, applied the same way
 * to every point.
 *
 * The fold step REPLACES the random map selection: it reflects
 * regions of space across symmetry planes so that all the "different
 * orbits" the IFS would explore stochastically are now compressed
 * into one canonical region.  Then contract.  Then repeat.
 *
 * This makes KIFS a deterministic SDF: at any point p in space, the
 * iteration produces a specific number representing distance to the
 * fractal surface.  Sphere tracing can then march to find the
 * intersection with each ray.
 *
 *
 * T3: The contraction step — pulling toward a fixed point
 * ───────────────────────────────────────────────────────
 * After folding, every iteration scales p toward a fixed offset:
 *
 *      p ← p · scale − offset · (scale − 1)
 *
 * Verify the FIXED POINT is `offset`:
 *
 *      offset · scale − offset · (scale − 1)
 *    = offset · scale − offset · scale + offset
 *    = offset                                              ✓
 *
 * For our presets scale > 1, so this is actually an EXPANSION
 * (points are pushed AWAY from offset).  The fold step beforehand
 * REFLECTS far points toward the offset's region, so after fold +
 * "expand" the net effect is convergence onto the attractor.
 *
 * Worked example, TETRA preset (offset = (1,1,1), scale = 2):
 *      p = (0.3, 0.4, 0.5)
 *      fold: no plane crossings (all sums positive)
 *      contract: p ← (0.6, 0.8, 1.0) − (1,1,1) · 1 = (−0.4, −0.2, 0)
 *      Next iteration: now folds DO trigger because of the negative
 *      components, and the dynamics start to work.
 *
 *
 * T4: TETRA — the Sierpinski tetrahedron
 * ──────────────────────────────────────
 * Three plane reflections + contraction toward the four-vertex offset:
 *
 *      if (p.x + p.y < 0):  (p.x, p.y) ← (−p.y, −p.x)        plane #1
 *      if (p.x + p.z < 0):  (p.x, p.z) ← (−p.z, −p.x)        plane #2
 *      if (p.y + p.z < 0):  (p.y, p.z) ← (−p.z, −p.y)        plane #3
 *
 * The three planes (x + y = 0, x + z = 0, y + z = 0) intersect at
 * the origin and have the four vertices of a regular tetrahedron at
 * their fixed points.  Each iteration drives p toward one of those
 * vertices; after many iterations, every starting point has been
 * herded to one of the four corners of the recursive sub-tetrahedron
 * structure.  Hence: Sierpinski tetrahedron.
 *
 * 12 iterations of fold + contract suffice for the visible scale of
 * our terminal canvas; finer detail beyond that is sub-pixel.
 *
 *
 * T5: MENGER — the Menger sponge
 * ──────────────────────────────
 * Two-step fold:
 *      p ← (|p.x|, |p.y|, |p.z|)              octant fold
 *      sort so p.x ≥ p.y ≥ p.z                three swaps if needed
 *
 * The abs-fold collapses the 8 octants of space into one (the +X,
 * +Y, +Z octant).  The sort then orders the components so the
 * largest is in p.x.  Combined with the contraction (offset =
 * (1,1,1), scale = 3), this produces the Menger sponge's cell-
 * removal pattern: 20 of the 27 sub-cubes get pulled outward at
 * each level, leaving the 7 central cubes empty (the central one
 * plus the six face-centred ones).
 *
 * MENGER also requires a Z FOLD-BACK heuristic (§9) before the
 * primitive DE is evaluated, otherwise the central column reads the
 * wrong distance and the recursion ladder visibly breaks.
 *
 *
 * T6: KIFS_ROT — the animated rotating crystal
 * ────────────────────────────────────────────
 * Three-step fold:
 *      (p.x, p.z) ← rotate by fold_rot around y-axis
 *      p ← (|p.x|, |p.y|, |p.z|)              octant fold
 *      if p.x < p.y: swap                     ONE swap (not 3)
 *
 * The rotation angle `fold_rot` advances at FOLD_ROT_RATE radians
 * per second.  Each frame the fractal looks slightly different — at
 * fold_rot = 0 it resembles a crystalline pillar; as the angle
 * sweeps, structures emerge, twist, and dissolve.
 *
 * The single swap (vs MENGER's three) gives a less-symmetric
 * fractal — chunkier crystals instead of cubic sponge.
 *
 *
 * T7: How does an iteration become a distance?
 * ────────────────────────────────────────────
 * After N iterations of fold + contract, p has been driven onto the
 * fractal's attractor.  But what's the DE there?  Trick:
 *
 *      DE(p) = primitive_de(p_final) · scale^(−iters)
 *
 * where `primitive_de` is just sphere_de(p) = |p| − 1 (or box_de
 * for MENGER).  The intuition: each contraction multiplied p by
 * `scale`, so distance estimates at the END of the iteration are
 * scaled by scale^iters too large.  Multiplying by scale^(−iters)
 * undoes this.
 *
 * This DE is a CONSERVATIVE LOWER BOUND, not exact distance.
 * Sphere tracing tolerates lower bounds (never overshoots), but
 * march speed near the surface is reduced compared to a true DE.
 *
 *
 * T8: Orbit trap — colour by iteration trajectory
 * ───────────────────────────────────────────────
 * Naïve colouring: pick a colour by the final value of p.  Result:
 * blocky bands where neighbouring pixels' iterations diverged.
 *
 * Orbit trap colouring: track a feature of the ITERATION TRAJECTORY,
 * not just the final value.  Common choice: the smallest |p|² seen
 * over all iterations:
 *
 *      trap = √( min over i of |p_i|² )
 *
 * This produces SMOOTH gradients across the fractal surface,
 * because nearby pixels have nearby orbit trajectories and hence
 * nearby trap values.
 *
 * In code we track |p|² (squared) to skip the sqrt per iteration,
 * then sqrt once at the end.  ~10× cheaper across the loop.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* End of textbook.  The rest of the file is the worked exercises. */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI layout. */
enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,
    FPS_UPDATE_MS    = 500,
    HUD_ROWS         =   2,         /* row 0 status + last row hint */
    ITERS_MIN        =   3,
    ITERS_MAX        =  18,
};

/* §1.2 colour-pair IDs. */
enum {
    PAIR_HUD         =  1,          /* yellow status row 0          */
    PAIR_HINT        =  2,          /* cyan key-hint last row       */
    PAIR_TRAP_BASE   =  3,          /* +0..+7 — orbit-trap ramp     */
    PAIR_BG          = 11,          /* fractal "miss" background    */
};

/* §1.3 time helpers. */
#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* §1.4 camera. */
#define CAM_DIST_DEFAULT  3.6f
#define CAM_DIST_MIN      1.6f      /* don't enter the fractal hull */
#define CAM_DIST_MAX     12.0f
#define CAM_DIST_STEP     0.20f
#define FOV_DEG          55.0f
#define CELL_ASPECT       2.0f      /* terminal cell h / w           */
#define ORBIT_YAW_RATE    0.35f     /* rad / sec — auto orbit         */
#define FOLD_ROT_RATE     0.18f     /* rad / sec — KIFS_ROT animation */
#define MANUAL_YAW_STEP   0.12f
#define MANUAL_PITCH_STEP 0.08f
#define MANUAL_PITCH_MAX  1.20f     /* clamp short of the poles      */

/* §1.5 sphere-trace tunables. */
#define MAX_STEPS         70
#define HIT_EPS           1.5e-3f
#define MAX_T            14.0f
#define NORMAL_EPS        6.0e-3f   /* ≈ 4·HIT_EPS empirically        */

/* §1.6 lighting + shading.
 *      lum = AMBIENT_LUM + DIFFUSE_LUM · max(0, N·L)
 *      ao  = max(AO_FLOOR, 1 − steps / MAX_STEPS)
 */
#define AMBIENT_LUM       0.18f
#define DIFFUSE_LUM       0.82f
#define AO_FLOOR          0.60f

/* §1.7 quantisation. */
#define LUMA_SLOTS         8
#define LUMA_SLOT_FLT      7.999f       /* (LUMA_SLOTS - 0.001) */
#define TRAP_NORM_RANGE    1.4f         /* empirical max trap value */
#define TRAP_NORM_INV      (1.0f / TRAP_NORM_RANGE)

/* §1.8 fractal preset table.
 *
 * Each row defines one preset's geometry: default fold iterations,
 * contraction scale, contraction fixed point (offset), and which
 * primitive DE to evaluate after the folds.
 */
typedef enum {
    PRESET_TETRA    = 0,
    PRESET_MENGER   = 1,
    PRESET_KIFS_ROT = 2,
    N_PRESETS       = 3,
} Preset;

typedef struct {
    const char *name;
    int   default_iters;
    float scale;
    float offset_x, offset_y, offset_z;
    int   primitive;            /* 0 = sphere, 1 = box */
    float bound_radius;         /* hint for camera distance */
} PresetParams;

static const PresetParams PRESETS[N_PRESETS] = {
    /*   name        iters scale  offx offy offz  prim  bound */
    /* TETRA   */  { "TETRA   ", 12, 2.00f, 1.00f, 1.00f, 1.00f,  0,  1.8f },
    /* MENGER  */  { "MENGER  ",  7, 3.00f, 1.00f, 1.00f, 1.00f,  1,  1.8f },
    /* KIFS_ROT*/  { "KIFS_ROT", 10, 2.05f, 0.85f, 1.10f, 0.85f,  0,  1.8f },
};

/* §1.9 themes — 8-step orbit-trap ramp.  Per CLAUDE.md, every entry
 * sits in the bright half of the 256-colour cube. */
typedef struct {
    const char *name;
    short       trap[LUMA_SLOTS];
} Theme;

#define N_THEMES 6

static const Theme THEMES[N_THEMES] = {
    { "GOLD   ", { 130, 137, 173, 215, 222, 229, 230, 231 } },
    { "ICE    ", {  24,  31,  38,  45,  87, 153, 195, 231 } },
    { "COBALT ", {  25,  26,  27,  33,  39,  45,  51, 159 } },
    { "COPPER ", { 130, 166, 173, 209, 215, 222, 229, 230 } },
    { "ALIEN  ", {  53,  91, 134, 165, 207, 213, 219, 159 } },
    { "MONO   ", { 244, 246, 248, 250, 252, 253, 254, 255 } },
};

/* §1.10 luminance ramp — dim → bright. */
static const char LUMA_GLYPHS[LUMA_SLOTS] = {
    '`', '.', ',', ':', '-', '+', '*', '#'
};

/* §1.11 debug overlays — d / D cycles between them. */
typedef enum {
    DEBUG_NORMAL    = 0,    /* full lit fractal (production view)        */
    DEBUG_TRAP      = 1,    /* orbit trap value as glyph + colour        */
    DEBUG_STEPS     = 2,    /* march step count → AO signal source       */
    DEBUG_NORMALS   = 3,    /* surface normal direction → hue band       */
    DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ", "TRAP   ", "STEPS  ", "NORMALS",
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
    struct timespec req = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                            .tv_nsec = (long)  (ns % NS_PER_SEC) };
    nanosleep(&req, NULL);
}

/* ── §3 color — orbit-trap palette + HUD/hint pairs ──────────────────── *
 *
 * 8 trap pairs (PAIR_TRAP_BASE..+7) hold the active theme's colours.
 * theme_apply re-points them; cell decoration reads from
 * PAIR_TRAP_BASE+slot to paint a cell.
 */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &THEMES[idx];
        for (int i = 0; i < LUMA_SLOTS; i++)
            init_pair((short)(PAIR_TRAP_BASE + i), t->trap[i], -1);
    } else {
        static const short FB[LUMA_SLOTS] = {
            COLOR_BLUE,    COLOR_MAGENTA, COLOR_CYAN,    COLOR_GREEN,
            COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE,   COLOR_WHITE,
        };
        for (int i = 0; i < LUMA_SLOTS; i++)
            init_pair((short)(PAIR_TRAP_BASE + i), FB[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
        init_pair(PAIR_BG,   242, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
        init_pair(PAIR_BG,   COLOR_BLACK,  -1);
    }
    theme_apply(0);
}

/* ── §4 vec3 — value-type 3-D math ───────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3   (float x, float y, float z) { return (V3){x, y, z}; }
static inline V3    v3add(V3 a, V3 b)                { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3sub(V3 a, V3 b)                { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3scale(float s, V3 a)           { return v3(s*a.x, s*a.y, s*a.z); }
static inline float v3dot(V3 a, V3 b)                { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3    v3cross(V3 a, V3 b)
{
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }
static inline V3    v3norm(V3 a)
{
    float L = v3len(a);
    return (L > 1e-12f) ? v3scale(1.0f / L, a) : v3(0, 1, 0);
}

static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ── §5 KifsParams — per-frame view of all DE parameters ─────────────── *
 *
 * The KIFS DE is called many times per frame.  Rather than pass 8
 * arguments to every helper, we pack the per-frame state into one
 * struct.  scene_build_kifs (§18) fills it once per frame from
 * Scene state; every DE-related helper takes a `const KifsParams *`.
 */
typedef struct {
    int   preset;          /* TETRA / MENGER / KIFS_ROT */
    int   iters;           /* fold iterations (3..18)   */
    float scale;           /* contraction multiplier    */
    float sm1;             /* scale − 1 (cached)        */
    float offx, offy, offz;          /* contraction fixed point   */
    float fold_rot_c;      /* cos(fold_rot) — KIFS_ROT only */
    float fold_rot_s;      /* sin(fold_rot) — KIFS_ROT only */
    float inv_scale_pow;   /* scale^(−iters), cached    */
} KifsParams;

/* ── §6 fold helpers — three preset-specific reflections ─────────────── *
 *
 * Tutorials T4, T5, T6 derived the math.  Each helper mutates p in
 * place — one fold step per iteration.
 */

/* TETRA: three plane-fold reflections (see T4). */
static inline void fold_iter_tetra(V3 *p)
{
    if (p->x + p->y < 0) { float t = -p->y; p->y = -p->x; p->x = t; }
    if (p->x + p->z < 0) { float t = -p->z; p->z = -p->x; p->x = t; }
    if (p->y + p->z < 0) { float t = -p->z; p->z = -p->y; p->y = t; }
}

/* MENGER: abs-fold + descending sort (see T5). */
static inline void fold_iter_menger(V3 *p)
{
    p->x = fabsf(p->x); p->y = fabsf(p->y); p->z = fabsf(p->z);
    if (p->x < p->y) { float t = p->x; p->x = p->y; p->y = t; }
    if (p->x < p->z) { float t = p->x; p->x = p->z; p->z = t; }
    if (p->y < p->z) { float t = p->y; p->y = p->z; p->z = t; }
}

/* KIFS_ROT: Y-rotation + abs-fold + ONE swap (see T6). */
static inline void fold_iter_rot(V3 *p, float c, float s)
{
    float xr = p->x * c - p->z * s;
    float zr = p->x * s + p->z * c;
    p->x = xr; p->z = zr;
    p->x = fabsf(p->x); p->y = fabsf(p->y); p->z = fabsf(p->z);
    if (p->x < p->y) { float t = p->x; p->x = p->y; p->y = t; }
}

/* ── §7 fold dispatch — pick the right helper by preset ──────────────── *
 *
 * The compiler inlines this; the switch becomes a single branch on
 * a value that's constant within a frame.  Outside the hot loop
 * the dispatch cost is invisible.
 */
static inline void fold_iter(V3 *p, const KifsParams *kp)
{
    switch (kp->preset) {
    case PRESET_TETRA:    fold_iter_tetra (p);                                 break;
    case PRESET_MENGER:   fold_iter_menger(p);                                 break;
    case PRESET_KIFS_ROT: fold_iter_rot   (p, kp->fold_rot_c, kp->fold_rot_s); break;
    }
}

/* ── §8 contract — the contractive map after each fold ───────────────── *
 *
 * Tutorial T3 derived this.  Pseudocode:
 *      p ← p · scale  −  offset · (scale − 1)
 * Fixed point: offset.  Verify: offset · scale − offset · (scale−1) = offset.
 */
static inline void contract_toward_offset(V3 *p, const KifsParams *kp)
{
    p->x = p->x * kp->scale - kp->offx * kp->sm1;
    p->y = p->y * kp->scale - kp->offy * kp->sm1;
    p->z = p->z * kp->scale - kp->offz * kp->sm1;
}

/* ── §9 menger z-foldback — MENGER-only post-step ────────────────────── *
 *
 * Pulls p.z back into a sensible range for the box DE that follows
 * (§11), so we don't measure distance from the WRONG side of the
 * box.  Pure heuristic — every public KIFS implementation includes
 * this line for the Menger preset; without it the central column
 * reads the wrong distance and the recursion ladder visibly breaks.
 */
static inline void menger_z_foldback(V3 *p, const KifsParams *kp)
{
    if (p->z < -0.5f * kp->offz * kp->sm1)
        p->z += kp->offz * kp->sm1;
}

/* ── §10 orbit trap — track running min |p|² during folding ──────────── *
 *
 * Tutorial T8 explained the idea.  We track squared magnitude (skip
 * sqrt per iteration) and sqrt once at the end.
 */
static inline void track_orbit_trap(V3 p, float *trap_sq)
{
    float r2 = p.x*p.x + p.y*p.y + p.z*p.z;
    if (r2 < *trap_sq) *trap_sq = r2;
}

/* ── §11 primitive DEs — sphere + box (Quílez form) ──────────────────── *
 *
 * After all the folding, we ask "how far is p from a simple unit
 * shape?".  Tutorial T7 explained why.  These two primitives are
 * the only "true" DEs in the file — both are Lipschitz-1 exact
 * Euclidean distance.
 */

/* sphere of radius 1 centred at origin — f(p) = |p| − 1 */
static inline float sphere_de(V3 p) { return v3len(p) - 1.0f; }

/* box of half-side 1 centred at origin — Quílez exact box DE */
static inline float box_de(V3 p)
{
    float qx = fabsf(p.x) - 1.0f;
    float qy = fabsf(p.y) - 1.0f;
    float qz = fabsf(p.z) - 1.0f;
    float dx = fmaxf(qx, 0), dy = fmaxf(qy, 0), dz = fmaxf(qz, 0);
    float outside = sqrtf(dx*dx + dy*dy + dz*dz);
    float inside  = fminf(fmaxf(qx, fmaxf(qy, qz)), 0.0f);
    return outside + inside;
}

/* dispatch: only MENGER uses the box; the others use sphere. */
static inline float primitive_de(int preset, V3 p)
{
    return (preset == PRESET_MENGER) ? box_de(p) : sphere_de(p);
}

/* ── §12 DE orchestrator — the kifs_de loop body ─────────────────────── *
 *
 * Tutorial T7 derived the formula.  Loop body matches the four-
 * step pseudocode from MENTAL MODEL: fold → contract → (menger
 * fold-back if MENGER) → track trap.  Final result is
 * primitive_de(p_final) · scale^(−iters).
 *
 * trap_out is optional — pass NULL when marching (don't need the
 * trap during the trace, only on hit).
 */
static float kifs_de_with_trap(V3 p, const KifsParams *kp, float *trap_out)
{
    float trap_sq = 1e10f;

    for (int i = 0; i < kp->iters; i++) {
        fold_iter              (&p, kp);
        contract_toward_offset (&p, kp);
        if (kp->preset == PRESET_MENGER)
            menger_z_foldback  (&p, kp);
        track_orbit_trap       (p, &trap_sq);
    }

    if (trap_out) *trap_out = sqrtf(trap_sq);
    return primitive_de(kp->preset, p) * kp->inv_scale_pow;
}

/* thin wrapper for the common case (no trap output) */
static inline float kifs_de(V3 p, const KifsParams *kp)
{
    return kifs_de_with_trap(p, kp, NULL);
}

/* ── §13 normal — central-difference gradient of DE ──────────────────── *
 *
 * Surface normal via 6-tap central difference:
 *      Nₓ ≈ DE(p + ε x̂) − DE(p − ε x̂)         and similarly for y, z
 *      N  = normalise(Nₓ, Nᵧ, N_z)
 *
 * Forward differences (3 evals) bias the normal toward one octant
 * — visible as skewed shading on highly-folded surfaces.  6 is
 * worth it.
 */
static V3 kifs_normal(V3 p, const KifsParams *kp)
{
    float e = NORMAL_EPS;
    float dx = kifs_de(v3(p.x + e, p.y, p.z), kp)
             - kifs_de(v3(p.x - e, p.y, p.z), kp);
    float dy = kifs_de(v3(p.x, p.y + e, p.z), kp)
             - kifs_de(v3(p.x, p.y - e, p.z), kp);
    float dz = kifs_de(v3(p.x, p.y, p.z + e), kp)
             - kifs_de(v3(p.x, p.y, p.z - e), kp);
    return v3norm(v3(dx, dy, dz));
}

/* ── §14 sphere trace — Hart 1996 march along a ray ──────────────────── *
 *
 * Same march loop as the sphere/cube files.  On hit we re-evaluate
 * kifs_de_with_trap once more to extract the orbit trap value (we
 * don't track it during the march — only the LAST hit's trap
 * matters for colour).  One extra DE per hit pixel: cheap relative
 * to the 6 in the normal estimator.
 */

typedef struct {
    bool  hit;
    V3    p;
    V3    normal;
    float trap;        /* orbit trap, normalised to [0, 1] */
    int   steps;
} Hit;

static Hit sphere_trace(V3 origin, V3 dir, const KifsParams *kp)
{
    Hit out = { false, {0,0,0}, {0,1,0}, 0.0f, 0 };
    float t = 0.0f;

    for (int i = 0; i < MAX_STEPS; i++) {
        V3    p = v3add(origin, v3scale(t, dir));
        float d = kifs_de(p, kp);

        if (d < HIT_EPS) {
            float trap = 0.0f;
            (void)kifs_de_with_trap(p, kp, &trap);

            out.hit    = true;
            out.p      = p;
            out.steps  = i;
            out.trap   = clampf(trap * TRAP_NORM_INV, 0.0f, 1.0f);
            out.normal = kifs_normal(p, kp);
            return out;
        }

        if (t > MAX_T) break;
        t += d;
    }
    return out;
}

/* Forward declaration — Scene's full definition lives in §18, but
 * camera_basis (§15) needs to take a `const Scene *`. */
typedef struct Scene Scene;

/* ── §15 camera — orthonormal basis + per-pixel ray ──────────────────── *
 *
 * Camera orbits horizontally around origin at distance cam_dist,
 * looking at (0, 0, 0) (the fractal centre).  Three orthonormal
 * vectors define the view: forward (look at origin), right
 * (perpendicular to forward, in the world horizontal plane), up
 * (perpendicular to both).
 *
 * Per pixel ray:
 *      u = (col + 0.5) / cols  · 2 − 1                  ∈ [−1, +1]
 *      v = ((row + 0.5) / rows · 2 − 1) · −1            (Y-flip)
 *      direction = forward
 *                + u · tan(FOV/2)              · right
 *                + v · tan(FOV/2) · phys_aspect · up
 *      direction = normalise(direction)
 *
 * phys_aspect = (rows · CELL_ASPECT) / cols corrects for terminal
 * cells being roughly twice as tall as wide, so circles stay round.
 */

typedef struct {
    V3    origin;
    V3    fwd, right, up;
    float fov_t;
    float phys_aspect;
} Camera;

/* camera_basis is forward-declared because Scene's full definition
 * (and hence camera_basis's ability to read from it) lives in §18.
 * The actual definition is at the bottom of §18. */
static Camera camera_basis(const Scene *s, int rows_eff)
;

static V3 pixel_ray(int col, int row, int cols, int rows_eff, const Camera *c)
{
    float u =  ((float)col + 0.5f) / (float)cols     * 2.0f - 1.0f;
    float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);
    V3 sx = v3scale(u * c->fov_t,                  c->right);
    V3 sy = v3scale(v * c->fov_t * c->phys_aspect, c->up);
    return v3norm(v3add(c->fwd, v3add(sx, sy)));
}

/* ── §16 lighting — Lambert + step-count AO ──────────────────────────── *
 *
 *      diffuse = max(0, N·L)                          Lambert's law
 *      lum     = AMBIENT_LUM + DIFFUSE_LUM · diffuse
 *      ao      = max(AO_FLOOR, 1 − steps / MAX_STEPS) cheap AO proxy
 *      final   = lum · ao
 *
 * Step-count AO: cells inside concavities take more march steps to
 * converge → step count correlates with concavity → naturally
 * darker crevices.  Geometrically nonsense but visually convincing
 * and free.
 */
static float lambert_with_ao(V3 normal, int steps, V3 light)
{
    float ndl = v3dot(normal, light);
    if (ndl < 0.0f) ndl = 0.0f;
    float lum = AMBIENT_LUM + DIFFUSE_LUM * ndl;

    float ao = 1.0f - (float)steps / (float)MAX_STEPS;
    if (ao < AO_FLOOR) ao = AO_FLOOR;
    return lum * ao;
}

/* ── §17 cell decoration + emit ──────────────────────────────────────── */

static int to_slot(float x_01)
{
    int s = (int)(x_01 * LUMA_SLOT_FLT);
    if (s < 0)               s = 0;
    if (s >= LUMA_SLOTS)     s = LUMA_SLOTS - 1;
    return s;
}

/* Cell — (glyph, colour pair, attribute) for one terminal cell. */
typedef struct { char glyph; int pair; attr_t attr; } Cell;

/* Production-view cell: glyph from luma, colour pair from orbit trap. */
static Cell shade_hit(const Hit *h, V3 light)
{
    if (!h->hit) {
        return (Cell){ ' ', PAIR_BG, A_NORMAL };
    }

    float lum    = lambert_with_ao(h->normal, h->steps, light);
    int   s_lum  = to_slot(lum);
    int   s_clr  = to_slot(h->trap);

    return (Cell){
        .glyph = LUMA_GLYPHS[s_lum],
        .pair  = PAIR_TRAP_BASE + s_clr,
        .attr  = (s_lum >= 6) ? A_BOLD
               : (s_lum <= 1) ? A_DIM
               :                A_NORMAL,
    };
}

/* emit_cell — paint one cell with attron/attroff batched on (pair,
 * attr) change.  Halves attribute thrash on uniform regions. */
static void emit_cell(int row, int col, Cell c,
                      int *last_pair, attr_t *last_attr)
{
    if (c.pair != *last_pair || c.attr != *last_attr) {
        if (*last_pair >= 0) attroff(COLOR_PAIR(*last_pair) | *last_attr);
        attron(COLOR_PAIR(c.pair) | c.attr);
        *last_pair = c.pair;
        *last_attr = c.attr;
    }
    mvaddch(row, col, (chtype)(unsigned char)c.glyph);
}

/* ── §18 scene state — Scene struct + tick + build_kifs ──────────────── */

struct Scene {
    bool       paused;
    int        current_preset;
    int        current_theme;
    int        iters_override;       /* 0 = use preset default */
    DebugMode  debug_mode;
    int        cols, rows;

    /* Camera state (yaw + pitch around origin). */
    float  cam_dist;
    float  orbit_yaw;             /* auto-advancing */
    float  orbit_pitch;
    float  user_yaw, user_pitch;  /* manual offsets via arrow keys */

    /* KIFS_ROT animated angle. */
    float  fold_rot;
};

static int scene_iters(const Scene *s)
{
    int it = (s->iters_override > 0)
           ? s->iters_override
           : PRESETS[s->current_preset].default_iters;
    if (it < ITERS_MIN) it = ITERS_MIN;
    if (it > ITERS_MAX) it = ITERS_MAX;
    return it;
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->current_preset  = PRESET_TETRA;
    s->current_theme   = 0;
    s->iters_override  = 0;
    s->debug_mode      = DEBUG_NORMAL;
    s->cols            = cols;
    s->rows            = rows;
    s->cam_dist        = CAM_DIST_DEFAULT;
    s->orbit_yaw       = 0.5f;
    s->orbit_pitch     = 0.25f;
    s->user_yaw        = 0.0f;
    s->user_pitch      = 0.0f;
    s->fold_rot        = 0.4f;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reset_cam(Scene *s)
{
    s->cam_dist    = CAM_DIST_DEFAULT;
    s->orbit_yaw   = 0.5f;
    s->orbit_pitch = 0.25f;
    s->user_yaw    = 0.0f;
    s->user_pitch  = 0.0f;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->orbit_yaw += ORBIT_YAW_RATE * dt;
    if (s->orbit_yaw >  (float)(2.0 * M_PI)) s->orbit_yaw -= (float)(2.0 * M_PI);
    if (s->orbit_yaw < -(float)(2.0 * M_PI)) s->orbit_yaw += (float)(2.0 * M_PI);

    s->fold_rot  += FOLD_ROT_RATE * dt;
    if (s->fold_rot >  (float)(2.0 * M_PI)) s->fold_rot -= (float)(2.0 * M_PI);
}

/* scene_build_kifs — pack per-frame state into a flat KifsParams the
 * DE inner loop can read without chasing pointers.  Caches
 * inv_scale_pow = scale^(−iters) so per-pixel DE doesn't recompute. */
static void scene_build_kifs(const Scene *s, KifsParams *kp)
{
    const PresetParams *pp = &PRESETS[s->current_preset];
    int iters = scene_iters(s);

    kp->preset        = s->current_preset;
    kp->iters         = iters;
    kp->scale         = pp->scale;
    kp->sm1           = pp->scale - 1.0f;
    kp->offx          = pp->offset_x;
    kp->offy          = pp->offset_y;
    kp->offz          = pp->offset_z;
    kp->fold_rot_c    = cosf(s->fold_rot);
    kp->fold_rot_s    = sinf(s->fold_rot);
    kp->inv_scale_pow = expf(-(float)iters * logf(pp->scale));
}

/* camera_basis — definition (forward-declared in §15). */
static Camera camera_basis(const Scene *s, int rows_eff)
{
    float yaw   = s->orbit_yaw   + s->user_yaw;
    float pitch = clampf(s->orbit_pitch + s->user_pitch,
                         -MANUAL_PITCH_MAX, MANUAL_PITCH_MAX);

    Camera c;
    c.origin = v3(s->cam_dist * cosf(pitch) * cosf(yaw),
                  s->cam_dist * sinf(pitch),
                  s->cam_dist * cosf(pitch) * sinf(yaw));
    c.fwd        = v3norm(v3sub(v3(0, 0, 0), c.origin));
    V3 wup       = v3(0, 1, 0);
    c.right      = v3norm(v3cross(c.fwd, wup));
    c.up         = v3cross(c.right, c.fwd);
    c.fov_t      = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
    c.phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;
    return c;
}

/* ── §19 render — orchestrator: walk pixels, trace, decorate ─────────── *
 *
 * Production view.  Reads top-to-bottom as the algorithm pseudocode:
 * each line is one named helper from §6..§17.
 */
static void render_normal(const Scene *s)
{
    int rows_eff = s->rows - HUD_ROWS;
    if (rows_eff < 1) return;

    Camera     cam   = camera_basis(s, rows_eff);
    V3         light = v3norm(v3(0.55f, 0.75f, 0.35f));
    KifsParams kp;
    scene_build_kifs(s, &kp);

    int    last_pair = -1;
    attr_t last_attr = 0;
    int    y0        = 1;       /* shift down 1 for top HUD row */

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            V3   ray = pixel_ray (col, row, s->cols, rows_eff, &cam);
            Hit  h   = sphere_trace(cam.origin, ray, &kp);
            Cell c   = shade_hit (&h, light);
            emit_cell(y0 + row, col, c, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* ── §20 debug overlays — see the raw rendering signals ──────────────── *
 *
 * Three educational visualisations.  Each isolates ONE piece of
 * intermediate state and paints it directly.
 *
 *   TRAP    — orbit-trap value as glyph + colour, no lighting.
 *             Shows where the colour ramp comes from.
 *   STEPS   — march-step count → glyph + colour.  Shows the AO
 *             signal's source — high-step regions glow.
 *   NORMALS — surface-normal direction → colour band.  Hue from
 *             azimuth around y-axis; reveals raw geometry without
 *             lighting interference.
 */

static Cell debug_cell_for_trap(const Hit *h)
{
    if (!h->hit) return (Cell){ ' ', PAIR_BG, A_NORMAL };
    int s_clr = to_slot(h->trap);
    return (Cell){
        .glyph = LUMA_GLYPHS[s_clr],
        .pair  = PAIR_TRAP_BASE + s_clr,
        .attr  = A_NORMAL,
    };
}

static Cell debug_cell_for_steps(const Hit *h)
{
    if (!h->hit) return (Cell){ ' ', PAIR_BG, A_NORMAL };
    float t = (float)h->steps / (float)(MAX_STEPS - 1);
    int   slot = to_slot(t);
    return (Cell){
        .glyph = LUMA_GLYPHS[slot],
        .pair  = PAIR_TRAP_BASE + slot,
        .attr  = (slot >= 6) ? A_BOLD : A_NORMAL,
    };
}

static Cell debug_cell_for_normals(const Hit *h)
{
    if (!h->hit) return (Cell){ ' ', PAIR_BG, A_NORMAL };
    float azimuth = atan2f(h->normal.z, h->normal.x);   /* −π..+π */
    float t = (azimuth + (float)M_PI) / (2.0f * (float)M_PI);
    int   slot = to_slot(t);
    float y_lit = (h->normal.y * 0.5f + 0.5f);          /* 0..1 */
    int   g_slot = to_slot(y_lit);
    return (Cell){
        .glyph = LUMA_GLYPHS[g_slot],
        .pair  = PAIR_TRAP_BASE + slot,
        .attr  = A_NORMAL,
    };
}

/* render_debug — same outer loop as render_normal; only the cell
 * decorator differs. */
static void render_debug(const Scene *s, DebugMode mode)
{
    int rows_eff = s->rows - HUD_ROWS;
    if (rows_eff < 1) return;

    Camera     cam = camera_basis(s, rows_eff);
    KifsParams kp;
    scene_build_kifs(s, &kp);

    int    last_pair = -1;
    attr_t last_attr = 0;
    int    y0        = 1;

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            V3  ray = pixel_ray(col, row, s->cols, rows_eff, &cam);
            Hit h   = sphere_trace(cam.origin, ray, &kp);

            Cell c;
            switch (mode) {
            case DEBUG_TRAP:    c = debug_cell_for_trap   (&h); break;
            case DEBUG_STEPS:   c = debug_cell_for_steps  (&h); break;
            case DEBUG_NORMALS: c = debug_cell_for_normals(&h); break;
            default:            c = (Cell){ ' ', PAIR_BG, A_NORMAL }; break;
            }
            emit_cell(y0 + row, col, c, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

static void render_active_view(const Scene *s)
{
    if (s->debug_mode == DEBUG_NORMAL) render_normal(s);
    else                               render_debug (s, s->debug_mode);
}

/* ── §21 screen — ncurses init / HUD / present ───────────────────────── */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free        (Screen *sc) { (void)sc; endwin(); }

static void screen_resize_curses(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* HUD layout (CLAUDE.md spec):
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint */
static void hud_draw(const Screen *sc, const Scene *s,
                     double fps, int sim_fps)
{
    char status[160];
    snprintf(status, sizeof status,
             " %5.1f fps  %3d Hz  preset:%s  theme:%s  iters:%2d  "
             "debug:%s  dist:%4.2f  %s ",
             fps, sim_fps,
             PRESETS[s->current_preset].name,
             THEMES[s->current_theme].name,
             scene_iters(s),
             DEBUG_MODE_NAMES[s->debug_mode],
             (double)s->cam_dist,
             s->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > sc->cols) slen = sc->cols;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sc->cols - slen, "%s", status);
    mvprintw(0, 0, " KIFS · FRACTAL ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  n/N:preset  t/T:theme  "
             "d/D:debug  i/I:iters  z/Z:zoom  arrows:orbit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    render_active_view(s);
    hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §22 app — main loop, signals, key handling ──────────────────────── */

typedef struct {
    Scene                 scene;
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
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reset_cam(s); s->iters_override = 0;    break;

    case 'n':
        s->current_preset = (s->current_preset + 1) % N_PRESETS;
        s->iters_override = 0;
        break;
    case 'N':
        s->current_preset = (s->current_preset + N_PRESETS - 1) % N_PRESETS;
        s->iters_override = 0;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'd':
        s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
        break;
    case 'D':
        s->debug_mode =
            (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
        break;

    case 'i': {
        int it = scene_iters(s);
        if (it > ITERS_MIN) s->iters_override = it - 1;
        break;
    }
    case 'I': {
        int it = scene_iters(s);
        if (it < ITERS_MAX) s->iters_override = it + 1;
        break;
    }

    case 'z':
        s->cam_dist += CAM_DIST_STEP;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        break;
    case 'Z':
        s->cam_dist -= CAM_DIST_STEP;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        break;

    case KEY_LEFT:  s->user_yaw   -= MANUAL_YAW_STEP;   break;
    case KEY_RIGHT: s->user_yaw   += MANUAL_YAW_STEP;   break;
    case KEY_UP:    s->user_pitch += MANUAL_PITCH_STEP; break;
    case KEY_DOWN:  s->user_pitch -= MANUAL_PITCH_STEP; break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
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
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw   (&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
