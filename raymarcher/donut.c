/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * donut.c — Andy Sloane's spinning ASCII torus, in the framework style
 *
 * DEMO: A donut tumbling in front of a fixed light bulb.  The torus is
 *       sampled on a (θ, φ) grid (~28 000 surface dots), every dot is
 *       rotated by two Euler angles, projected to the terminal with
 *       1/z perspective, and a depth buffer keeps the closest dot per
 *       cell.  Brightness comes from the analytic dot product of the
 *       surface normal with a fixed light direction; brightness picks
 *       the glyph from a 12-character ramp `.,-~:;=!*#$@` and one of
 *       eight grey colour pairs.  No raymarching, no rasteriser — just
 *       point sampling + z-buffer.  A reader who can read a for-loop
 *       can follow the entire algorithm.
 *
 * Study alongside:
 *   raster/torus_raster.c    — same torus geometry rendered through
 *                              the project's full triangle rasteriser
 *                              (vertex/fragment shaders, MVP matrix,
 *                              barycentric raster).  Compare and you
 *                              see why this 700-line file beats the
 *                              1300-line one for pedagogy: the math
 *                              is on display, not buried in pipeline.
 *
 * Section map:
 *   §1 config   — all tunable constants + glyph ramp + colour-pair IDs
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — luminance pairs + HUD/hint pairs (CLAUDE.md spec)
 *   §4 math     — V2 / V3 / Rot constructors (mirror the formulas)
 *   §5 torus    — Torus state + the five sampling helpers + render
 *   §6 screen   — ncurses init / resize / HUD draw / present
 *   §7 app      — main loop, signals, key handling, cleanup
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume rotation
 *   ] / [        spin faster / slower    (multiplies both rot speeds)
 *   = / -        torus larger / smaller  (multiplies K1)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/donut.c -o donut \
 *       -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic torus point-sampling.  We never march a ray
 *                  and we never tessellate triangles.  We walk a (θ, φ)
 *                  parameter grid where θ goes around the torus axis
 *                  (the BIG circle) and φ goes around the tube cross-
 *                  section (the SMALL circle); each (θ, φ) gives one
 *                  3-D point on the torus surface in closed form.
 *                  Rotate every point by two Euler angles, project to
 *                  2-D with 1/z perspective division, run a per-cell
 *                  depth buffer that keeps only the closest dot.  The
 *                  donut you see is thousands of independent points
 *                  fighting for the front-most spot per cell.
 *
 * Data-structure : Two flat buffers sized to TORUS_MAX_CELLS:
 *                    zbuf [r·cols+c]  → 1/z of the closest dot here so
 *                                       far  (zero = empty cell)
 *                    glyph[r·cols+c]  → ASCII char from LUMI_RAMP, or
 *                                       space if no dot landed here
 *                    luma [r·cols+c]  → luma slot 0..7 (so the draw
 *                                       step picks the colour pair
 *                                       without re-deriving it)
 *                  All three are zeroed at the start of each render.
 *                  No heap allocation after init.
 *
 * Rendering      : Z-buffer convention is INVERTED — we store 1/z, not
 *                  z, because perspective division produces 1/z naturally
 *                  and bigger 1/z = closer.  The depth test is
 *                  `if (ooz > zbuf[idx])` — accept the new dot if it's
 *                  closer.  Empty cells start at zbuf=0 (infinitely
 *                  far), and every visible torus point has positive z
 *                  thanks to the +K2 shift in surface_point().
 *
 * Performance    : O(N_θ · N_φ) per frame ≈ 90 × 314 = 28 000 surface
 *                  samples regardless of terminal size.  Each sample is
 *                  ~25 floating-point ops; ~1 ms / frame on a modern CPU
 *                  at the default sample density.  Halving THETA_STEP
 *                  doubles the cost.  Rendering cost is independent of
 *                  cell count — increasing the terminal makes the donut
 *                  bigger but not slower.
 *
 * References     : Sloane, Andy (2011) — "Donut math: how donut.c works"
 *                    https://www.a1k0n.net/2011/07/20/donut-math.html
 *                    The canonical write-up of the original algorithm.
 *                    Every formula in this file matches one in that post.
 *                  Wikipedia — "Torus § Parametric equation"
 *                    https://en.wikipedia.org/wiki/Torus#Geometry
 *                    The (R, r, θ, φ) parameterisation we use.
 *                  Newman & Sproull (1979) — "Principles of Interactive
 *                    Computer Graphics," 2nd ed.  Ch. 22 covers the
 *                    z-buffer visibility algorithm we lift here.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A torus is two nested circles: walk a SMALL circle (the tube cross-
 * section, parameter φ) while sweeping that whole circle around a
 * BIGGER circle (the torus axis, parameter θ).  Sampling (θ, φ) on a
 * fine grid produces a dense cloud of 3-D surface points.  Rotate
 * every point by two Euler angles, project to 2-D with 1/z
 * perspective, and let a z-buffer decide which sample wins each
 * terminal cell.  The donut you see is just thousands of lit dots
 * fighting for the front-most spot.
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
 *    2. Pre-compute (sinA, cosA, sinB, cosB) for the current Euler
 *       angles — they're constant for every sample within one frame.
 *
 *  Per (θ, φ) sample (≈ 28 000 of them):
 *    3. tube_point(θ)            → 2-D point on the tube cross-section
 *    4. surface_point(...)       → 3-D world point (rotated, +K2 shifted)
 *    5. project_to_screen(...)   → (xp, yp, ooz) with bounds flag
 *    6. surface_luminance(...)   → scalar L; skip if L ≤ 0 (back-facing)
 *    7. try_emit_pixel(...)      → z-buffer test + glyph + luma store
 *
 *  After the loop:
 *    8. torus_draw walks the buffers, emits every non-space cell with
 *       its colour pair (luma slot → grey ramp index).
 *
 *  Each tick (frame-rate independent):
 *    9. A += rot_a · dt;  B += rot_b · dt — the only state mutation.
 *
 * KEY FORMULAS
 * ────────────
 *   Tube point        cx = R2 + R1 · cos θ      ┐ circle in the plane
 *                     cy =      R1 · sin θ      ┘ before revolving
 *
 *   X-then-Z combined rotation (after multiplying out the 3×3 matrices):
 *     x  = cx · (cos B · cos φ + sin A · sin B · sin φ) − cy · cos A · sin B
 *     y  = cx · (sin B · cos φ − sin A · cos B · sin φ) + cy · cos A · cos B
 *     z  = K2  +  cos A · cx · sin φ  +  cy · sin A
 *
 *   1/z perspective projection (Y_ASPECT halves vertical to undo the
 *   2:1 terminal cell aspect):
 *     ooz = 1 / z                                        (one-over-z)
 *     xp  =  cols/2 + K1 · ooz · x
 *     yp  =  rows/2 − K1 · ooz · y · Y_ASPECT
 *
 *   Lambertian luminance with hard-coded light dir (0, 1, −1)/√2:
 *     L  =  cos φ · cos θ · sin B
 *         − cos A · cos θ · sin φ
 *         − sin A · sin θ
 *         + cos B · (cos A · sin θ − cos θ · sin A · sin φ)
 *
 *   Sizing — K1 chosen so the projected outer edge of the torus fills
 *   TORUS_TARGET_FILL of the smaller terminal half-dimension:
 *     K1 = TARGET_FILL · min(cols/2, rows) · (K2 + R2 + R1) / (R2 + R1)
 *
 *   Glyph index (0..LUMI_RAMP_LEN−1):
 *     li = clamp( floor(L · LUMI_RAMP_LEN), 0, LUMI_RAMP_LEN − 1 )
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Z-BUFFER CONVENTION IS INVERTED: zbuf[] holds 1/z, so larger =
 *     closer.  A naïve `if (ooz < zbuf[idx])` draws the BACK of the
 *     donut on top — a classic introductory bug.
 *
 *   • Initial zbuf = 0 only works because every visible surface point
 *     has z > 0 (the +K2 shift in surface_point puts the donut in
 *     front of the camera).  If the camera ever crossed K2 the whole
 *     render breaks; we don't have any code path that does, so it's
 *     safe.
 *
 *   • Y_ASPECT = 0.5 is NOT physics — it's terminal aspect correction
 *     for cells that are roughly twice as tall as wide.  On a square-
 *     cell terminal you'd set Y_ASPECT = 1.0 and the donut would be
 *     vertically squashed otherwise.
 *
 *   • Sample undersampling shows on the TUBE before the AXIS because
 *     the tube is the smaller circle.  PHI_STEP (0.02) is about 3.5×
 *     finer than THETA_STEP (0.07) for that reason — same density per
 *     unit length on the surface.
 *
 *   • Resize must update Torus.cols / Torus.rows but NEVER exceed
 *     TORUS_MAX_*; the flat buffers are sized for the worst case so
 *     larger terminals quietly clip.
 *
 *   • Pause freezes A and B but render still runs every frame — the
 *     donut stays visible, just stationary.  Useful for inspecting
 *     individual frames at A=0 or after a specific manual rotation.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • At startup the donut should appear as a recognisable doughnut
 *     silhouette tumbling slowly.  Sun-facing top is bright, far side
 *     is dim.
 *
 *   • Press space.  Rotation freezes.  Press ] five times then space
 *     to resume — donut should now spin ~3.7× faster (1.3^5 ≈ 3.71).
 *
 *   • Press = until growth stops at SIZE_MAX.  Press - back to original
 *     size.  K1 changes; the donut never disappears off-screen because
 *     torus_k1 always fits to the smaller dimension.
 *
 *   • Resize the terminal: the donut re-centres (it's drawn at
 *     cols/2, rows/2 always) and re-fits to TORUS_TARGET_FILL of the
 *     smaller dimension automatically.
 *
 *   • Setting THETA_STEP to 0.5 deliberately should produce a sparse
 *     "wire cage" donut — proves the (θ, φ) loop is the only source
 *     of sample density.
 *
 *   • Hard test: temporarily flip the depth comparison to `<` instead
 *     of `>`.  The BACK of the donut should now draw on top of the
 *     front.  Confirms the inverted-z convention is doing the work.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* §1.1 frame rate + HUD layout */
enum {
    SIM_FPS_DEFAULT  = 30,
    FPS_UPDATE_MS    = 500,
    HUD_STATUS_COLS  = 40,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(fps)    (NS_PER_SEC / (fps))

/* §1.2 angles + π helpers */
#define TWO_PI          (2.0f * (float)M_PI)

/* §1.3 rotation speed (radians per second) — A is the X-axis tumble,
 * B is the Z-axis spin.  ] and [ multiply BOTH by SPEED_SCALE so the
 * relative tumble:spin ratio stays at the original feel. */
#define ROT_A_DEFAULT   1.2f
#define ROT_B_DEFAULT   0.6f
#define SPEED_SCALE     1.3f
#define SPEED_MIN       0.05f
#define SPEED_MAX      12.0f

/* §1.4 torus geometry constants — pure shape, no animation.
 *   R1  tube radius (small circle).
 *   R2  distance from torus centre to tube centre (big circle).
 *   K2  viewer distance — added to z so the donut sits in front of
 *       the camera (z always positive).  Larger = less perspective
 *       distortion.  K1 (perspective scale) is computed per-frame
 *       in torus_k1 so the torus auto-fits the terminal. */
#define TORUS_R1        1.0f
#define TORUS_R2        2.0f
#define TORUS_K2        5.0f

/* §1.5 K1 sizing — the torus should fill TORUS_TARGET_FILL of the
 * smaller terminal dimension at user scale 1.0.  See torus_k1 for
 * the derivation. */
#define TORUS_TARGET_FILL  0.42f
#define TORUS_SIZE_SCALE   1.15f
#define TORUS_SIZE_MIN     0.30f
#define TORUS_SIZE_MAX     5.00f

/* §1.6 sample density — finer = smoother surface, slower frames.
 * PHI_STEP is finer than THETA_STEP because the tube (φ direction)
 * is the smaller circle and shows undersampling first. */
#define THETA_STEP      0.07f
#define PHI_STEP        0.02f

/* §1.7 terminal cell aspect correction.
 *   Y_ASPECT = 0.5  for typical terminals (cells ~2× taller than wide)
 *   Y_ASPECT = 1.0  for square-cell terminals
 * Applied as a multiplier on the projected y so the donut reads as a
 * round shape rather than a vertically stretched ellipse. */
#define Y_ASPECT        0.5f

/* §1.8 luminance ramp — dim → bright.  Twelve glyphs.  The render
 * picks the index from L · LUMI_RAMP_LEN; the colour pair is picked
 * from the same index quantised to LUMI_LEVELS slots. */
static const char LUMI_RAMP[] = ".,-~:;=!*#$@";
#define LUMI_RAMP_LEN   ((int)(sizeof LUMI_RAMP - 1))   /* = 12 */

/* §1.9 colour-pair scheme — eight grey luminance pairs plus the two
 * named CLAUDE.md HUD pairs (yellow status, cyan key hint).
 *
 *   PAIR_LUMI_BASE..PAIR_LUMI_BASE+7   eight grey shades dim → bright
 *   PAIR_HUD                           bright yellow (status row)
 *   PAIR_HINT                          bright cyan   (key-hint row)
 */
enum {
    LUMI_LEVELS      = 8,
    PAIR_LUMI_BASE   = 1,                    /* +0..+7  */
    PAIR_HUD         = PAIR_LUMI_BASE + LUMI_LEVELS,    /* = 9  */
    PAIR_HINT        = PAIR_HUD + 1,                    /* = 10 */
};

/* §1.10 framebuffer sizing.  Anything larger than TORUS_MAX_COLS ×
 * TORUS_MAX_ROWS is quietly clipped — we'd need to malloc to avoid
 * that, but 512×256 = 131 072 cells handles every realistic terminal. */
#define TORUS_MAX_COLS  512
#define TORUS_MAX_ROWS  256
#define TORUS_MAX_CELLS (TORUS_MAX_COLS * TORUS_MAX_ROWS)

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

/* ── §3 color — luminance pairs + HUD/hint pairs ─────────────────────── *
 *
 * Eight grey shades come from the xterm-256 grey ramp (indices 232..255
 * are 24 even greys dark→light).  We pick eight indices in the BRIGHTER
 * half so the donut is vivid against a black background, even on the
 * dimmest luma slot.  In 8-colour mode we fall back to A_DIM/A_NORMAL/
 * A_BOLD on COLOR_WHITE to fake three brightness levels.
 */

static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        /* Eight evenly-spaced greys in the bright half (240..255). */
        static const short LUMI_GREYS[LUMI_LEVELS] =
            { 240, 243, 246, 248, 250, 252, 254, 255 };
        for (int i = 0; i < LUMI_LEVELS; i++)
            init_pair((short)(PAIR_LUMI_BASE + i), LUMI_GREYS[i], -1);

        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        for (int i = 0; i < LUMI_LEVELS; i++)
            init_pair((short)(PAIR_LUMI_BASE + i), COLOR_WHITE, -1);
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/*
 * lumi_attr — ncurses attribute for luma slot l ∈ [0, LUMI_LEVELS).
 *
 * In 256-colour mode each slot has its own grey pair.
 * In 8-colour mode we layer DIM/NORMAL/BOLD on top of the white pair
 * so eight slots still produce three visibly distinct brightnesses.
 */
static attr_t lumi_attr(int l)
{
    if (l < 0)              l = 0;
    if (l > LUMI_LEVELS-1)  l = LUMI_LEVELS - 1;

    attr_t a = COLOR_PAIR(PAIR_LUMI_BASE + l);
    if (COLORS < 256) {
        if      (l < 3) a |= A_DIM;
        else if (l > 5) a |= A_BOLD;
    }
    return a;
}

/* ── §4 math — V2 / V3 / Rot constructors mirror the formulas ────────── *
 *
 * The vector types exist purely so call sites read like the math:
 *     V2 tube  = tube_point(...);
 *     V3 world = surface_point(tube, cosph, sinph, rot);
 *
 * No operator overloading, no arithmetic helpers — just the
 * constructor and the field access.  C does not get in the way of
 * the algebra here.
 */

typedef struct { float x, y;       } V2;
typedef struct { float x, y, z;    } V3;

/*
 * Rot — the four trig values we need at every (θ, φ) sample, computed
 * ONCE per frame in torus_render and passed by value to the helpers.
 * Avoids 56 000+ redundant sinf/cosf calls per frame.
 */
typedef struct { float sinA, cosA, sinB, cosB; } Rot;

static inline V2  v2 (float x, float y)            { return (V2){ x, y };    }
static inline V3  v3 (float x, float y, float z)   { return (V3){ x, y, z }; }

static inline Rot rot_make(float A, float B)
{
    return (Rot){ sinf(A), cosf(A), sinf(B), cosf(B) };
}

/* ── §5 torus — state + sampling helpers + render + draw ─────────────── */

/*
 * Torus — owns every mutable scrap of state for the demo.
 *
 *   A, B          current Euler rotation angles (radians)
 *   rot_a, rot_b  rotation speed (rad/sec) — multiplied by dt each tick
 *   k1_scale      user size multiplier — '=' / '-' adjust this
 *   paused        if true, torus_tick leaves A, B untouched
 *   cols, rows    terminal dimensions, resampled on SIGWINCH
 *   zbuf          1/z of the closest dot in each cell so far
 *   glyph         ASCII char to draw in each cell  (' ' = empty)
 *   luma          luma slot 0..LUMI_LEVELS-1 (lookup for the colour
 *                 pair; saved to avoid re-deriving it from the glyph
 *                 in torus_draw)
 *
 * The three flat buffers are sized for TORUS_MAX_CELLS (the worst
 * case).  No heap allocation after init.
 */
typedef struct {
    float   A, B;
    float   rot_a, rot_b;
    float   k1_scale;
    bool    paused;
    int     cols, rows;
    float   zbuf [TORUS_MAX_CELLS];
    char    glyph[TORUS_MAX_CELLS];
    uint8_t luma [TORUS_MAX_CELLS];
} Torus;

/*
 * torus_init — zero the buffers and seed the angles + speeds.
 */
static void torus_init(Torus *t, int cols, int rows)
{
    memset(t, 0, sizeof *t);
    t->rot_a    = ROT_A_DEFAULT;
    t->rot_b    = ROT_B_DEFAULT;
    t->k1_scale = 1.0f;
    t->cols     = cols;
    t->rows     = rows;
}

/*
 * torus_resize — terminal grew or shrank; just update cols/rows.  The
 * static buffers already hold enough room; render() will use the new
 * dimensions next frame.
 */
static void torus_resize(Torus *t, int cols, int rows)
{
    t->cols = cols;
    t->rows = rows;
}

/*
 * torus_tick — advance the two rotation angles by dt seconds.
 * Skipped when paused so the frame pipeline still renders the same
 * stationary donut every frame — useful for inspecting a held pose.
 */
static void torus_tick(Torus *t, float dt)
{
    if (t->paused) return;
    t->A += t->rot_a * dt;
    t->B += t->rot_b * dt;
}

/*
 * torus_k1 — perspective scaling factor.
 *
 * We want the projected outer edge of the torus (which sits at
 * x = R2 + R1 in object space, distance K2 + R2 + R1 from the camera
 * after the +K2 shift) to land at TARGET_FILL × min(cols/2, rows)
 * pixels from screen centre.  The 1/z projection then gives:
 *
 *   target_pixels = K1 · (R2 + R1) / (K2 + R2 + R1)
 * ⇒ K1            = target_pixels · (K2 + R2 + R1) / (R2 + R1)
 *
 * Multiplied by user-controlled k1_scale.  Recomputed every frame so
 * a resize updates the size automatically.
 */
static float torus_k1(const Torus *t)
{
    float half_w   = (float)t->cols * 0.5f;
    float half_h   = (float)t->rows;
    float min_half = (half_w < half_h ? half_w : half_h);
    float target   = min_half * TORUS_TARGET_FILL;

    float k1 = target * (TORUS_K2 + TORUS_R2 + TORUS_R1)
                      / (TORUS_R2 + TORUS_R1);
    return k1 * t->k1_scale;
}

/* ── §5 sampling helpers — five tiny functions, one math step each ──── */

/*
 * tube_point — point on the torus's TUBE cross-section at angle θ.
 *
 *   cx = R2 + R1 · cos θ      ┐ a circle of radius R1 centred at
 *   cy =      R1 · sin θ      ┘ (R2, 0) before being revolved by φ
 */
static V2 tube_point(float costh, float sinth)
{
    return v2(TORUS_R2 + TORUS_R1 * costh,
                         TORUS_R1 * sinth);
}

/*
 * surface_point — full 3-D world position for a (θ, φ) sample.
 *
 * Conceptually three steps composed:
 *   1. Take the tube point (cx, cy) at angle θ.
 *   2. Revolve it around the y-axis by angle φ — this places a 3-D
 *      point on the un-rotated torus surface.
 *   3. Rotate that 3-D point by A around the X axis, then by B around
 *      the Z axis, then translate by +K2 along z.
 *
 * The composition of those rotations expanded out gives the explicit
 * formulas below — same as in Sloane's original donut.c.  See KEY
 * FORMULAS in the MENTAL MODEL block for the matrix derivation.
 *
 * `cosph` / `sinph` are passed in (not computed here) so the caller's
 * inner φ-loop can compute them once and reuse them for both the
 * surface point AND the luminance.
 */
static V3 surface_point(V2 tube, float cosph, float sinph, Rot r)
{
    return v3(
        tube.x * (r.cosB * cosph + r.sinA * r.sinB * sinph)
      - tube.y *  r.cosA * r.sinB,

        tube.x * (r.sinB * cosph - r.sinA * r.cosB * sinph)
      + tube.y *  r.cosA * r.cosB,

        TORUS_K2 + r.cosA * tube.x * sinph + tube.y * r.sinA);
}

/*
 * project_to_screen — 1/z perspective projection.
 *
 *   ooz = 1 / z              (one-over-z; bigger ooz = closer)
 *   xp  = cols/2 + K1 · ooz · x
 *   yp  = rows/2 − K1 · ooz · y · Y_ASPECT     (Y-flip + aspect fix)
 *
 * The minus sign on yp is the standard screen-Y-flip (terminal rows
 * grow downward, world y grows upward).  Y_ASPECT corrects for cells
 * being roughly twice as tall as wide.
 *
 * Returned alongside the integer pixel coords so the caller can:
 *   - skip out-of-bounds samples cheaply (in_bounds flag)
 *   - feed ooz directly into the z-buffer compare without recomputing
 */
typedef struct { int xp, yp; float ooz; bool in_bounds; } ScreenPos;

static ScreenPos project_to_screen(V3 p, int cols, int rows, float K1)
{
    float ooz = 1.0f / p.z;
    int   xp  = (int)((float)cols * 0.5f + K1 * ooz * p.x);
    int   yp  = (int)((float)rows * 0.5f - K1 * ooz * p.y * Y_ASPECT);
    bool  ok  = (xp >= 0 && xp < cols && yp >= 0 && yp < rows);
    return (ScreenPos){ xp, yp, ooz, ok };
}

/*
 * surface_luminance — Lambertian L = N · light_dir, baked into a
 * closed form by substituting the rotated surface normal and the
 * fixed light direction (0, 1, −1)/√2 (above and behind the camera).
 *
 *   L =  cos φ · cos θ · sin B
 *      − cos A · cos θ · sin φ
 *      − sin A · sin θ
 *      + cos B · (cos A · sin θ − cos θ · sin A · sin φ)
 *
 * Range is roughly [-√2, +√2].  Caller skips samples with L ≤ 0
 * (the surface points away from the light → no contribution).
 */
static float surface_luminance(float costh, float sinth,
                                float cosph, float sinph, Rot r)
{
    return cosph * costh * r.sinB
         - r.cosA * costh * sinph
         - r.sinA * sinth
         + r.cosB * (r.cosA * sinth - costh * r.sinA * sinph);
}

/*
 * try_emit_pixel — z-buffer test + glyph store.
 *
 * Three guards short-circuit:
 *   1. L ≤ 0          surface faces away from light, contributes nothing
 *   2. !in_bounds     projected pixel is off-screen
 *   3. ooz ≤ zbuf     a closer sample already won this cell
 *
 * If all three pass, write the glyph + luma slot for the cell.
 */
static void try_emit_pixel(Torus *t, ScreenPos sp, float L)
{
    if (L <= 0.0f || !sp.in_bounds) return;

    int idx = sp.yp * t->cols + sp.xp;
    if (sp.ooz <= t->zbuf[idx]) return;     /* lost the depth test */

    int li = (int)(L * (float)LUMI_RAMP_LEN);
    if (li < 0)                  li = 0;
    if (li >= LUMI_RAMP_LEN)     li = LUMI_RAMP_LEN - 1;

    t->zbuf [idx] = sp.ooz;
    t->glyph[idx] = LUMI_RAMP[li];
    t->luma [idx] = (uint8_t)((li * LUMI_LEVELS) / LUMI_RAMP_LEN);
}

/*
 * torus_clear_buffers — zero zbuf (= "infinitely far"), space the
 * glyph buffer, zero the luma buffer.  One memset each.
 */
static void torus_clear_buffers(Torus *t)
{
    int n = t->cols * t->rows;
    memset(t->zbuf,  0,   sizeof(float)   * (size_t)n);
    memset(t->glyph, ' ', sizeof(char)    * (size_t)n);
    memset(t->luma,  0,   sizeof(uint8_t) * (size_t)n);
}

/*
 * torus_render — the orchestrator.  Reads top-to-bottom as the
 * algorithm pseudocode itself.  Every line either calls one of the
 * five sample helpers above or advances the (θ, φ) loop variables.
 */
static void torus_render(Torus *t)
{
    torus_clear_buffers(t);

    Rot   r  = rot_make(t->A, t->B);
    float K1 = torus_k1(t);

    for (float theta = 0.f; theta < TWO_PI; theta += THETA_STEP) {
        float costh = cosf(theta), sinth = sinf(theta);
        V2    tube  = tube_point(costh, sinth);

        for (float phi = 0.f; phi < TWO_PI; phi += PHI_STEP) {
            float cosph = cosf(phi), sinph = sinf(phi);

            V3        world = surface_point   (tube, cosph, sinph, r);
            ScreenPos sp    = project_to_screen(world, t->cols, t->rows, K1);
            float     L     = surface_luminance(costh, sinth, cosph, sinph, r);

            try_emit_pixel(t, sp, L);
        }
    }
}

/*
 * torus_draw — walk the rendered buffers and emit each non-empty cell
 * with its colour pair.  The luma slot is stored alongside the glyph
 * by try_emit_pixel, so we don't need to back-derive it here.
 */
static void torus_draw(const Torus *t, WINDOW *w)
{
    int cols = t->cols, n = t->cols * t->rows;

    for (int i = 0; i < n; i++) {
        char c = t->glyph[i];
        if (c == ' ') continue;

        attr_t attr = lumi_attr(t->luma[i]);
        wattron(w, attr);
        mvwaddch(w, i / cols, i % cols, (chtype)(unsigned char)c);
        wattroff(w, attr);
    }
}

/* ── §6 screen — ncurses init / resize / HUD draw / present ──────────── */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho(); cbreak(); curs_set(0);
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

/*
 * screen_draw — full frame: torus then HUD overlay.
 *
 * HUD layout (CLAUDE.md spec):
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 */
static void screen_draw(Screen *s, const Torus *t, double fps)
{
    erase();
    torus_draw(t, stdscr);

    /* Row 0 — yellow status. */
    char status[HUD_STATUS_COLS + 1];
    snprintf(status, sizeof status,
             " %5.1f fps  spd:%4.2f%s ",
             fps, (double)t->rot_a,
             t->paused ? "  PAUSED" : "");
    int slen = (int)strlen(status); if (slen > s->cols) slen = s->cols;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, s->cols - slen, "%s", status);
    mvprintw(0, 0, " DONUT ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom row — cyan key hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  ]/[:speed  +/-:size ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §7 app — main loop, signals, key handling, cleanup ──────────────── */

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
 * file header DEMO + Keys block.
 */
static bool app_handle_key(App *app, int ch)
{
    Torus *t = &app->torus;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':                    t->paused = !t->paused; break;

    case ']':
        t->rot_a *= SPEED_SCALE;
        t->rot_b *= SPEED_SCALE;
        if (t->rot_a > SPEED_MAX) t->rot_a = SPEED_MAX;
        if (t->rot_b > SPEED_MAX) t->rot_b = SPEED_MAX;
        break;

    case '[':
        t->rot_a /= SPEED_SCALE;
        t->rot_b /= SPEED_SCALE;
        if (t->rot_a < SPEED_MIN) t->rot_a = SPEED_MIN;
        if (t->rot_b < SPEED_MIN) t->rot_b = SPEED_MIN;
        break;

    case '=': case '+':
        t->k1_scale *= TORUS_SIZE_SCALE;
        if (t->k1_scale > TORUS_SIZE_MAX) t->k1_scale = TORUS_SIZE_MAX;
        break;

    case '-':
        t->k1_scale /= TORUS_SIZE_SCALE;
        if (t->k1_scale < TORUS_SIZE_MIN) t->k1_scale = TORUS_SIZE_MIN;
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

    /* dt-loop state — same scaffold as the rest of the project. */
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

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── fixed-step sim accumulator ──────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            torus_tick(&app->torus, dt_sec);
            sim_accum -= tick_ns;
        }

        /* ── render into the framebuffer ─────────────────────────── */
        torus_render(&app->torus);

        /* ── fps counter (rolling window) ────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE I/O so writes don't drift) ──── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

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
