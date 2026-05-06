/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cube_raytrace.c — analytic ray-traced cube via the slab method
 *
 * DEMO: A solid cube tumbling in space, lit by three coloured lights.
 *       Cycle phong → normals → wireframe → depth to see how the same
 *       slab-method intersection feeds different visualisations. Watch
 *       the silhouette change from square (face-on) to hexagon (corner-on)
 *       as the cube rotates.
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c   — same skeleton, ONE quadratic
 *   raytracing/capsule_raytrace.c  — same skeleton, decomposed analytic
 *   raytracing/torus_raytrace.c    — same skeleton, QUARTIC (much harder)
 *
 * Section map:
 *   §1 config     — frame rate, FOV, cube geometry, camera, ramp, HUD
 *   §2 clock      — monotonic timer + sleep
 *   §3 math       — V3, Mat3 (with the inverse-rotation explanation)
 *   §4 color      — themes + 256-colour cube + ASCII-ramp painter
 *   §5 cube       — THE CORE: ray vs axis-aligned box (slab method)
 *                   §5.1 single-slab solve (one axis at a time)
 *                   §5.2 ray_aabb dispatcher (combine the 3 slabs)
 *                   §5.3 face_edge_dist (for wireframe mode)
 *   §6 shading    — phong / normals / wireframe / depth
 *   §7 render     — one frame: ray-per-cell, object-space transform
 *   §8 screen     — ncurses init + HUD
 *   §9 app        — signals, input, main loop
 *
 * Keys:
 *   s         cycle shade mode (phong → normals → wireframe → depth)
 *   t         cycle theme (iron → sapphire → copper → jade → obsidian → plasma)
 *   p / SPC   pause / resume rotation
 *   r         reset rotation angles to zero
 *   + / =     zoom in
 *   -         zoom out
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/cube_raytrace.c \
 *       -o cube_rt -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-AABB intersection by the SLAB METHOD.
 *                  An axis-aligned bounding box is the INTERSECTION of three
 *                  pairs of parallel planes — three "slabs," one per axis.
 *                  Per axis i the ray spends t ∈ [tn_i, tf_i] inside the
 *                  slab. The ray is inside the BOX only when it lies inside
 *                  ALL THREE slabs simultaneously, i.e.:
 *                    t_enter = max(tn_x, tn_y, tn_z)
 *                    t_exit  = min(tf_x, tf_y, tf_z)
 *                  If t_enter ≤ t_exit and t_exit > 0, the ray hits the box.
 *                  The face that produced t_enter (the LAST slab to start
 *                  containing the ray) IS the entry face — and its outward
 *                  normal is one of ±X, ±Y, ±Z determined by the sign of
 *                  the ray direction along that axis.
 *
 * Data-structure : AABB ≡ (V3 min, V3 max). Stored implicitly as a single
 *                  half-extent CUBE_S because the box is axis-aligned and
 *                  centred at the origin in OBJECT space; the world-space
 *                  rotation is applied to the RAY (inverse-rotation trick),
 *                  not to the box geometry.
 *
 * Rendering      : One ray per terminal cell. Phong with three coloured
 *                  lights. Normal mode encodes N as RGB (excellent diagnostic
 *                  for verifying face-normal orientation). Wireframe mode
 *                  draws only the cells within WIRE_THRESH of the nearest
 *                  face edge in object-space face UV coordinates. Depth mode
 *                  encodes hit distance as a brightness gradient.
 *
 * Performance    : O(3) per pixel — three single-slab solves, three max/min.
 *                  No square roots, no trig, no iterations. The cube is the
 *                  CHEAPEST primitive in the raytracing folder.
 *
 * References     : Andrew Woo, "Fast Ray-Box Intersection",
 *                    in Andrew S. Glassner (ed.) "Graphics Gems" (1990).
 *                  Inigo Quílez, "Box — intersection",
 *                    https://iquilezles.org/articles/intersectors/
 *                  Kay & Kajiya, "Ray Tracing Complex Scenes",
 *                    SIGGRAPH '86. (Original slab method paper.)
 *                  Real-Time Rendering 4e §22.7 (ray-box variants).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To check if a ray hits a 3D box, ask THREE 1D questions ("when does it
 * cross the X faces?", "the Y faces?", "the Z faces?"), then ask whether
 * those three time intervals share a common moment. They share a moment
 * iff the LATEST entry time is still earlier than the EARLIEST exit
 * time. The slab method reduces "is the ray inside this box?" to "do
 * three 1D intervals overlap?" — and interval overlap is two arithmetic
 * comparisons.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a cube as a sandwich of three pairs of parallel walls:
 *
 *           ┌─────────────────────┐
 *          ╱                     ╱│
 *         ╱       Y-slab        ╱ │
 *        ┌─────────────────────┐  │ ← X-slab (left/right walls)
 *        │                     │  │
 *        │    box interior     │  │
 *        │  =  X ∩ Y ∩ Z slabs │ ╱
 *        │                     │╱  ← Z-slab (front/back walls)
 *        └─────────────────────┘
 *
 * As a ray flies through space, it spends a time interval inside each
 * slab:
 *
 *      t →   ───[==== X-slab ====]──────────────────
 *      t →   ─────────[========= Y-slab ===========]
 *      t →   ───[============ Z-slab ===]──────────
 *                  ↑                ↑
 *                  t_enter        t_exit  ← ray IS inside the box
 *                                            in this overlap region
 *
 * The ray is inside the box ONLY in the overlap of all three intervals.
 * `t_enter` is the most-restrictive lower bound (the latest start);
 * `t_exit` is the most-restrictive upper bound (the earliest end). If
 * the upper bound has not run out yet by the time the latest slab
 * starts, the ray made it inside.
 *
 * The face that produced `t_enter` is exactly the face the ray
 * entered through, because that's the slab that was the "last" to
 * start admitting the ray.
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS  (per pixel)
 * ──────────────────────────────────────────────────
 *  1. Build a primary ray from camera into world space:
 *        rd_ws = normalize( fwd  +  pu·right  +  pv·up )
 *  2. Transform ray to OBJECT space (the box stays axis-aligned there):
 *        ro_os = M^T · cam            (M is the cube's rotation matrix)
 *        rd_os = M^T · rd_ws
 *  3. For each axis i ∈ {0,1,2}:
 *        if rd_os[i] is ~0:
 *           if ro_os[i] is outside [-s, s]   →  ray is parallel to and
 *                                                outside this slab → MISS.
 *           else                              →  this slab imposes no
 *                                                t-bound (ray stays inside).
 *        else:
 *           tn, tf = (-s − ro)/rd, (+s − ro)/rd     (swap if tn > tf)
 *           if tn > tmin:  tmin = tn;  remember (axis, sign(rd))
 *           if tf < tmax:  tmax = tf
 *           if tmin > tmax:  early exit → MISS.
 *  4. After the 3 axes:
 *        if tmax < ε                    → MISS (whole interval is behind us)
 *        if tmin < ε:  t = tmax         → ray ORIGIN is inside box; exit hit
 *        else:         t = tmin         → ordinary entry hit
 *        N = ±axis_i for the remembered (axis, sign). Sign is −sign(rd[i]).
 *  5. Transform N back to world space (M·N) and shade.
 *
 * KEY FORMULAS
 * ────────────
 *   For axis i, plane at position p, with rd_i ≠ 0:
 *     t = (p − ro_i) / rd_i             [solve ro + t·rd = p on this axis]
 *
 *   Per-slab interval for box [-s, s] on axis i:
 *     t0 = (−s − ro_i) / rd_i
 *     t1 = (+s − ro_i) / rd_i
 *     tn = min(t0, t1)                  [enter time]
 *     tf = max(t0, t1)                  [exit  time]
 *
 *   Combine across three axes:
 *     tmin = max(tn_x, tn_y, tn_z)
 *     tmax = min(tf_x, tf_y, tf_z)
 *
 *   Hit test:
 *     hit ⇔ tmin ≤ tmax  AND  tmax > 0
 *
 *   Outward normal at entry:
 *     N = e_i · (−sign(rd_i))     where i is the axis that set tmin.
 *     (At t = tmin, the ray is just stepping ONTO that face, so the
 *      outward normal points OPPOSITE to the ray direction component
 *      on that axis.)
 *
 * WORKED EXAMPLE  (verify by hand)
 * ────────────────────────────────
 *   Cube:    [-0.8, +0.8]^3 (object space)
 *   Camera:  ro = (0, 0, −3.2), rd = (0, 0, +1)   ← head-on, no rotation
 *
 *   X-slab:  rd.x = 0  AND  ro.x = 0 ∈ [−0.8, 0.8]   → no t-bound from X
 *   Y-slab:  rd.y = 0  AND  ro.y = 0 ∈ [−0.8, 0.8]   → no t-bound from Y
 *   Z-slab:  t0 = (−0.8 − (−3.2)) / 1 = +2.4
 *            t1 = (+0.8 − (−3.2)) / 1 = +4.0
 *            tn_z = 2.4, tf_z = 4.0
 *
 *   tmin = 2.4 (from Z-slab),   tmax = 4.0 (from Z-slab)
 *   tmin ≤ tmax ✓ and tmax > 0 ✓ → HIT at t = 2.4
 *   The setting axis is Z; sign(rd.z) = +1 → outward normal sign = −1
 *   ⇒ N_obj = (0, 0, −1)        (front face, points toward camera) ✓
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • rd_i ≈ 0 (ray parallel to slab): use a 1e-9 tolerance. If origin is
 *    inside [-s, s] on that axis the slab imposes no bound; otherwise
 *    the ray never enters that slab → return MISS immediately.
 *  • Inside the box (tmin < 0 < tmax): the "entry" is in the past; we
 *    return tmax (the exit) instead and the normal then points OUTWARD
 *    from the EXIT face. This is correct for a self-intersecting hit.
 *  • Corner / edge hits: two slabs may produce numerically equal tn —
 *    the slab method picks ONE (whichever was checked first whose tn
 *    was strictly greater). The normal becomes that one face's normal.
 *    On a terminal grid, this discontinuity is invisible because the
 *    pixel at the geometric corner is already a single cell.
 *  • t < 1e-4 epsilon prevents self-intersection at t≈0 when the
 *    camera grazes a face.
 *  • The inverse-rotation trick: rotating a cube means rotating 8
 *    vertices and 6 face equations. Keeping the cube fixed at the
 *    origin and rotating the RAY by M^T is equivalent and costs ONE
 *    matrix×ray multiply per pixel — the same trick used in every
 *    file in this folder.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Static cube, no rotation, viewed head-on:  silhouette is a SQUARE
 *    of solid colour (one visible face). MODE_NORMAL paints it a single
 *    flat colour — the (0,0,−1) face is RGB(0.5, 0.5, 0).
 *  • Cube viewed from a corner (rotate 45°/45°): silhouette is a
 *    HEXAGON, and you see THREE faces meeting at the central vertex
 *    forming a tri-radial "Y". MODE_NORMAL paints each face a different
 *    flat colour.
 *  • Worked-example check: head-on ray to a 1.6-cube at distance 3.2
 *    should hit at t = 2.4. Compare to the value the code computes.
 *  • Wireframe mode shows 9 visible edges (3 per visible face minus
 *    none — the inner edges of the "Y" plus the 6-edge silhouette
 *    outline). If you see 12 edges, the back-face cull (driven by the
 *    outward normal pointing toward the camera) has failed.
 *  • Doubling CUBE_S doubles the silhouette in every dimension and
 *    halves the apparent distance — i.e. brings the cube closer.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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
#define FOV_DEG       55.0f                /* full vertical-equivalent FOV  */

/* §1.3 cube (object space, axis-aligned, centred on origin) */
#define CUBE_S        0.80f                /* half-extent: spans [-S, +S]  */

/* §1.4 wireframe mode */
#define WIRE_THRESH   0.055f               /* edge band as fraction of S    */

/* §1.5 rotation rates (rad/sec) */
#define ROT_Y         0.52f                /* primary spin around Y         */
#define ROT_X         0.35f                /* tilt around X                 */

/* §1.6 camera (orbits along −Z; bigger distance = smaller cube) */
#define CAM_DIST_DEF  3.2f
#define CAM_DIST_MIN  1.5f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f

/* §1.7 shading */
#define AMBIENT       0.05f
#define SHININESS     40.0f                /* phong exponent                */

/* §1.8 character ramp — Paul Bourke 92-char density ladder.
 * Index 0 (space) is invisible; index N−1 ('@') is densest. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN  ((int)(sizeof k_ramp - 1))

/* §1.9 ncurses pair IDs (256-colour cube + reserved HUD/HINT) */
#define PAIR_CUBE_BASE   1                 /* + 0..215 = 6×6×6 cube       */
#define PAIR_HUD       217
#define PAIR_HINT      218

/* §1.10 epsilons */
#define T_EPS         1e-4f                /* reject t < this (self-hit)  */
#define PARALLEL_EPS  1e-9f                /* |rd_i| below this = parallel*/

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

/* ── §3 math (V3, Mat3) ──────────────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3add   (V3 a, V3 b)    { return (V3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline V3    v3sub   (V3 a, V3 b)    { return (V3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline V3    v3scale (float s, V3 a) { return (V3){s*a.x,  s*a.y,  s*a.z};   }
static inline V3    v3mul   (V3 a, V3 b)    { return (V3){a.x*b.x, a.y*b.y, a.z*b.z}; }
static inline float v3dot   (V3 a, V3 b)    { return a.x*b.x + a.y*b.y + a.z*b.z;    }
static inline float v3len   (V3 a)          { return sqrtf(v3dot(a, a));             }
static inline V3    v3norm  (V3 a)          { float l=v3len(a); return l>1e-9f ? v3scale(1.f/l,a) : (V3){0,1,0}; }
static inline V3    v3reflect(V3 v, V3 n)   { return v3sub(v, v3scale(2.f*v3dot(v,n), n)); }
static inline V3    v3clamp1(V3 v)
{
    return (V3){ v.x<0?0:v.x>1?1:v.x, v.y<0?0:v.y>1?1:v.y, v.z<0?0:v.z>1?1:v.z };
}

/* 3×3 rotation matrix — rows stored as V3 for `v3dot` ergonomics. */
typedef struct { V3 r[3]; } Mat3;

/*
 * mat3_rot — composed rotation Rx(rx) · Ry(ry).
 *
 * This is the OBJECT→WORLD transform (mat3_mul). For ray tracing we
 * need the OPPOSITE direction (rotate WORLD ray into the cube's object
 * space, where the box stays axis-aligned). For a pure rotation the
 * inverse equals the transpose, so mat3_mulT below is the inverse —
 * cheap, no determinant, no division.
 */
static Mat3 mat3_rot(float rx, float ry)
{
    float cx = cosf(rx), sx = sinf(rx);
    float cy = cosf(ry), sy = sinf(ry);
    Mat3 m;
    m.r[0] = (V3){  cy,      0.f,    sy    };
    m.r[1] = (V3){  sx*sy,   cx,    -sx*cy };
    m.r[2] = (V3){ -cx*sy,   sx,     cx*cy };
    return m;
}

/* M · v   — object → world (forward) */
static V3 mat3_mul(Mat3 m, V3 v)
{
    return (V3){ v3dot(m.r[0], v), v3dot(m.r[1], v), v3dot(m.r[2], v) };
}

/* M^T · v — world → object (inverse, since M is orthogonal) */
static V3 mat3_mulT(Mat3 m, V3 v)
{
    return (V3){
        m.r[0].x*v.x + m.r[1].x*v.y + m.r[2].x*v.z,
        m.r[0].y*v.x + m.r[1].y*v.y + m.r[2].y*v.z,
        m.r[0].z*v.x + m.r[1].z*v.y + m.r[2].z*v.z
    };
}

/* ── §4 color / themes ───────────────────────────────────────────────── */

/*
 * Theme — five colour vectors per palette:
 *   obj         base albedo of the cube
 *   spec        specular highlight tint (key + rim)
 *   key/fill/rim three-light tint (warm·cool·accent)
 */
typedef struct {
    V3 obj, spec, key_col, fill_col, rim_col;
    const char *name;
} Theme;

static const Theme g_themes[] = {
    /* iron — cool grey metallic */
    {{0.65f,0.65f,0.70f},{0.95f,0.95f,1.00f},
     {1.00f,0.95f,0.85f},{0.25f,0.35f,0.80f},{0.80f,0.85f,1.00f},"iron"},
    /* sapphire — deep blue */
    {{0.10f,0.25f,0.90f},{0.60f,0.75f,1.00f},
     {0.80f,0.88f,1.00f},{0.05f,0.10f,0.60f},{0.30f,0.55f,1.00f},"sapphire"},
    /* copper — warm orange-bronze */
    {{0.85f,0.42f,0.12f},{1.00f,0.82f,0.55f},
     {1.00f,0.88f,0.65f},{0.40f,0.15f,0.10f},{1.00f,0.50f,0.15f},"copper"},
    /* jade — rich green */
    {{0.12f,0.62f,0.32f},{0.55f,1.00f,0.65f},
     {0.70f,1.00f,0.55f},{0.05f,0.30f,0.45f},{0.18f,0.85f,0.40f},"jade"},
    /* obsidian — near-black, bright specular */
    {{0.18f,0.16f,0.22f},{0.90f,0.82f,1.00f},
     {1.00f,0.95f,0.90f},{0.15f,0.15f,0.30f},{0.75f,0.50f,1.00f},"obsidian"},
    /* plasma — neon violet */
    {{0.55f,0.05f,0.90f},{0.95f,0.70f,1.00f},
     {0.90f,0.75f,1.00f},{0.08f,0.05f,0.55f},{0.85f,0.15f,1.00f},"plasma"},
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
 *   - colour gives "what it is" (warm copper face vs cool iron edge)
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

/* ── §5 ray-AABB intersection (THE CORE) ─────────────────────────────── */

/*
 * Cube = axis-aligned box [-s, +s]^3 in OBJECT space.
 *
 * The slab method asks: "for each axis, between which two times does
 * the ray sit between the two parallel walls of the slab?" Then it
 * combines those three intervals via max-of-entries / min-of-exits.
 *
 * §5.1 — solve a SINGLE slab (the smallest unit of work).
 * §5.2 — combine the three slabs into a hit decision (the dispatcher).
 * §5.3 — face_edge_dist for wireframe (uses the resolved face).
 */

/* §5.1 ── single-axis slab solve ─────────────────────────────────────── */

/*
 * slab_test — solve the 1D ray-vs-slab problem on one axis.
 *
 * Inputs:  ro_i, rd_i — the i-th components of ray origin/direction
 *          s          — half-extent (slab spans [-s, +s] on this axis)
 *
 * Outputs: on success, *tn = entry time, *tf = exit time, *enter_sign
 *          = sign of the OUTWARD normal on the entry face for this axis
 *          (it's −sign(rd_i): if the ray is moving in +X, it enters
 *          through the −X face).
 *
 * Returns: 1 if the ray ever sits inside this slab (possibly from
 *          t=−∞ to +∞ when parallel and inside);
 *          0 if the ray is parallel to the slab AND outside it (the
 *            ray is forever outside this slab → no possible hit).
 *
 * This function does NO axial bound check — that's the dispatcher's
 * job, which combines three slabs.
 */
static int slab_test(float ro_i, float rd_i, float s,
                     float *tn, float *tf, float *enter_sign)
{
    if (fabsf(rd_i) < PARALLEL_EPS) {
        /* Ray runs parallel to the two walls of this slab. */
        if (ro_i < -s || ro_i > s) return 0;     /* outside → permanent miss */
        *tn = -1e30f;                            /* slab imposes no t-bound  */
        *tf =  1e30f;
        *enter_sign = 0.f;                       /* no entry face contributed*/
        return 1;
    }
    float inv = 1.f / rd_i;
    float t0 = (-s - ro_i) * inv;
    float t1 = ( s - ro_i) * inv;
    if (t0 < t1) { *tn = t0; *tf = t1; }
    else         { *tn = t1; *tf = t0; }
    /* Outward normal sign on the entry face: opposite to ray direction. */
    *enter_sign = (rd_i > 0.f) ? -1.f : 1.f;
    return 1;
}

/* §5.2 ── ray vs box: combine three slabs into one hit decision ──────── */

/*
 * ray_aabb — slab-method ray-AABB intersection.
 *
 * Inputs:  ro, rd — ray in OBJECT space (axis-aligned coordinates).
 *          s      — half-extent of the cube.
 * Outputs: on hit, *t_hit = entry time, *N_os = object-space normal of
 *          the hit face (one of ±X, ±Y, ±Z basis vectors).
 * Returns: 1 on hit, 0 on miss.
 *
 * The dispatcher walks the three axes, intersects their per-slab time
 * intervals, and remembers WHICH axis tightened the entry bound — that
 * axis's face is the one we hit, and we already computed its normal
 * sign in slab_test.
 */
static int ray_aabb(V3 ro, V3 rd, float s, float *t_hit, V3 *N_os)
{
    /* Pack into arrays for a clean axis loop. */
    float ro_a[3] = { ro.x, ro.y, ro.z };
    float rd_a[3] = { rd.x, rd.y, rd.z };

    float tmin = -1e30f, tmax = 1e30f;
    int   near_axis  = -1;                       /* which axis set tmin?   */
    float near_sign  = 0.f;                      /* sign of outward N      */

    for (int i = 0; i < 3; i++) {
        float tn, tf, esign;
        if (!slab_test(ro_a[i], rd_a[i], s, &tn, &tf, &esign))
            return 0;                            /* parallel & outside    */

        /* Tighten entry bound: take the LATEST of the three start times. */
        if (tn > tmin) {
            tmin       = tn;
            near_axis  = i;
            near_sign  = esign;
        }
        /* Tighten exit bound: take the EARLIEST of the three end times. */
        if (tf < tmax) tmax = tf;

        /* If the intervals don't overlap, no t can satisfy all three. */
        if (tmin > tmax) return 0;
    }

    if (tmax < T_EPS) return 0;                  /* whole interval behind */

    /* Two cases:
     *   tmin > T_EPS  → ordinary entry hit, t = tmin.
     *   tmin ≤ T_EPS  → ray origin is INSIDE the box → return the EXIT
     *                    (t = tmax) and the exit-face normal. We rebuild
     *                    that normal from sign(rd) on the axis whose tf
     *                    matched tmax. */
    float t;
    if (tmin > T_EPS) {
        t = tmin;
    } else {
        /* Inside-the-box case — find which axis owns tmax, with the
         * exit-face normal pointing in the SAME direction as rd_i (the
         * ray is leaving through that wall). */
        t = tmax;
        for (int i = 0; i < 3; i++) {
            float tn, tf, esign;
            if (!slab_test(ro_a[i], rd_a[i], s, &tn, &tf, &esign)) return 0;
            if (fabsf(tf - tmax) < 1e-6f) {
                near_axis = i;
                near_sign = (rd_a[i] > 0.f) ? +1.f : -1.f;
                break;
            }
        }
        if (near_axis < 0) return 0;
    }

    /* Face normal: the axis basis vector with the recorded sign. */
    *N_os = (V3){0,0,0};
    if      (near_axis == 0) N_os->x = near_sign;
    else if (near_axis == 1) N_os->y = near_sign;
    else                     N_os->z = near_sign;
    *t_hit = t;
    return 1;
}

/* §5.3 ── face_edge_dist (for wireframe mode) ────────────────────────── */

/*
 * face_edge_dist — fraction-to-edge of a hit point on its face.
 *
 * Returns 0 when P is exactly on an edge of the face, and 1 when P is
 * at the face centre. Used by MODE_WIRE to draw only the cells whose
 * fraction-to-edge is below WIRE_THRESH.
 *
 * We pick the two object-space coordinates that LIE WITHIN the face
 * (the two NOT aligned with the face's normal axis) and measure how
 * close each is to the face's boundary at ±s.
 */
static float face_edge_dist(V3 P, V3 N, float s)
{
    float u, v;
    if      (fabsf(N.x) > 0.5f) { u = P.y; v = P.z; }   /* face on ±X     */
    else if (fabsf(N.y) > 0.5f) { u = P.x; v = P.z; }   /* face on ±Y     */
    else                        { u = P.x; v = P.y; }   /* face on ±Z     */

    float du = s - fabsf(u);                      /* dist to nearer u-edge */
    float dv = s - fabsf(v);                      /* dist to nearer v-edge */
    return fminf(du, dv) / s;                     /* normalise to [0,1]    */
}

/* ── §6 shading ──────────────────────────────────────────────────────── */

typedef enum { MODE_PHONG=0, MODE_NORMAL, MODE_WIRE, MODE_DEPTH, MODE_N } ShadeMode;
static const char *const k_mode_names[] = { "phong","normals","wireframe","depth" };

/* Three fixed world-space lights — positions, not directions. */
static const V3 L_KEY  = { 3.0f, 4.0f, -2.0f };
static const V3 L_FILL = {-4.0f, 1.0f, -1.0f };
static const V3 L_RIM  = { 0.5f,-1.0f,  5.0f };

/*
 * Phong: ambient + Σ_lights (kd·N·L  +  ks·(R·V)^n).
 *
 * Per light:
 *   L = normalize(light_pos − P)         direction TO the light
 *   d = max(0, N·L)                       diffuse  (Lambert)
 *   R = reflect(−L, N) = 2(N·L)N − L      reflection of incoming light
 *   s = max(0, R·V_dir)^shininess         specular (Phong)
 *
 * Three lights: warm KEY, cool FILL, accent RIM. Splitting them by
 * §-blocks below shows clearly which contribution comes from which
 * light without parameter blizzard.
 */
static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th)
{
    V3 col = v3scale(AMBIENT, th->obj);

    /* §6.1 KEY light — primary diffuse + sharp specular. */
    {
        V3    L = v3norm(v3sub(L_KEY, P));
        float d = fmaxf(0.f, v3dot(N, L));
        V3    R = v3reflect(v3scale(-1.f, L), N);
        float s = powf(fmaxf(0.f, v3dot(R, V_dir)), SHININESS);
        col = v3add(col, v3scale(d * 0.65f, v3mul(th->obj, th->key_col)));
        col = v3add(col, v3scale(s * 0.50f, th->spec));
    }
    /* §6.2 FILL light — soft diffuse, no specular. Lifts the shadow side. */
    {
        V3    L = v3norm(v3sub(L_FILL, P));
        float d = fmaxf(0.f, v3dot(N, L));
        col = v3add(col, v3scale(d * 0.22f, v3mul(th->obj, th->fill_col)));
    }
    /* §6.3 RIM light — wide specular kissing the back silhouette. */
    {
        V3    L = v3norm(v3sub(L_RIM, P));
        float d = fmaxf(0.f, v3dot(N, L));
        V3    R = v3reflect(v3scale(-1.f, L), N);
        float s = powf(fmaxf(0.f, v3dot(R, V_dir)), 10.f);
        col = v3add(col, v3scale(d * 0.18f, v3mul(th->obj, th->rim_col)));
        col = v3add(col, v3scale(s * 0.60f, th->rim_col));
    }
    return v3clamp1(col);
}

/* §6.4 Normal mode — RGB-encoded surface normal (diagnostic).
 * For an axis-aligned cube, each visible face is a SINGLE flat colour:
 *   +X → (1.0, 0.5, 0.5)   −X → (0.0, 0.5, 0.5)
 *   +Y → (0.5, 1.0, 0.5)   −Y → (0.5, 0.0, 0.5)
 *   +Z → (0.5, 0.5, 1.0)   −Z → (0.5, 0.5, 0.0)
 */
static V3 shade_normal(V3 N)
{
    return (V3){ N.x*.5f + .5f, N.y*.5f + .5f, N.z*.5f + .5f };
}

/* §6.5 Wire mode — only cells within WIRE_THRESH of a face edge.
 * The face_edge_dist call has already been done in render() to early-
 * out non-edge cells; this helper just maps "close to edge" into a
 * brightness ramp. We also return the surface normal as the colour so
 * each edge inherits the face's normal-encoded hue. */
static V3 shade_wire(V3 N, float ed)
{
    V3 col = shade_normal(N);
    /* Pull saturation up at the edge centre (ed=0) and down further away. */
    float k = 1.f - ed / WIRE_THRESH;
    return v3clamp1(v3scale(0.6f + 0.4f * k, col));
}

/* §6.6 Depth mode — encode hit distance as brightness (closer = bright).
 * d² (rather than d) gives a steeper falloff so the silhouette pops. */
static V3 shade_depth(float t, float t_max, const Theme *th)
{
    float d = 1.f - fminf(t / t_max, 1.f);
    d = d * d;
    return v3clamp1(v3scale(d, th->obj));
}

/* Rec. 601 luminance for ramp-index choice. */
static inline float rec601_luma(V3 c)
{
    return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/* ── §7 render frame ─────────────────────────────────────────────────── */

/*
 * The inverse-rotation trick:
 *
 *   Forward way    :  rotate cube (8 vertices, 6 faces), fire ray in
 *                     world space. Cost: rotate every primitive every
 *                     frame.
 *
 *   Backward way   :  keep cube fixed at object space (axis-aligned),
 *                     fire ray, then transform ray INTO object space
 *                     by multiplying by M^T = M⁻¹ (orthogonal matrix).
 *                     Cost: ONE matrix×ray multiply per pixel.
 *
 * For an axis-aligned box this is essential: a NON-axis-aligned box
 * would force the slab method to use general-plane equations (3
 * dot-products per slab instead of 1 scalar). Keeping the box AABB in
 * object space lets us divide instead of dot.
 */
static void render(int cols, int rows,
                   float angle_x, float angle_y, float cam_dist,
                   int theme_idx, ShadeMode mode)
{
    const Theme *th  = &g_themes[theme_idx % THEME_N];
    float fov_tan    = tanf(FOV_DEG * (float)M_PI / 360.f);

    Mat3 M           = mat3_rot(angle_x, angle_y);

    /* §7.1 — fixed camera on −Z, looking toward origin. */
    V3 cam = { 0.f, 0.f, -cam_dist };
    V3 fwd = { 0.f, 0.f,  1.f };
    V3 rgt = { 1.f, 0.f,  0.f };
    V3 up  = { 0.f, 1.f,  0.f };

    float cx = cols * 0.5f, cy = rows * 0.5f;

    /* §7.2 — primary loop: one ray per cell. Skip the bottom row for HUD. */
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            /* Normalised screen coords with terminal-cell aspect baked in. */
            float pu =  (col - cx) / cx * fov_tan;
            float pv = -(row - cy) / cx * fov_tan / ASPECT;

            V3 rd_ws = v3norm(v3add(fwd, v3add(v3scale(pu, rgt),
                                               v3scale(pv, up))));

            /* Transform ray to object space (cube fixed at origin). */
            V3 ro_os = mat3_mulT(M, cam);
            V3 rd_os = mat3_mulT(M, rd_ws);

            float t_hit;
            V3    N_os;
            if (!ray_aabb(ro_os, rd_os, CUBE_S, &t_hit, &N_os))
                continue;

            /* Hit point in OBJECT and WORLD space; world-space normal. */
            V3 P_os  = v3add(ro_os, v3scale(t_hit, rd_os));
            V3 P_ws  = v3add(cam,   v3scale(t_hit, rd_ws));
            V3 N_ws  = mat3_mul(M, N_os);
            V3 V_dir = v3norm(v3sub(cam, P_ws));

            V3    color;
            float lum;

            switch (mode) {
            default:
            case MODE_PHONG:
                color = shade_phong(P_ws, N_ws, V_dir, th);
                lum   = rec601_luma(color);
                break;
            case MODE_NORMAL:
                color = shade_normal(N_ws);
                lum   = (N_ws.x*.5f+.5f)*.3f
                      + (N_ws.y*.5f+.5f)*.6f
                      + (N_ws.z*.5f+.5f)*.1f;
                break;
            case MODE_WIRE: {
                float ed = face_edge_dist(P_os, N_os, CUBE_S);
                if (ed > WIRE_THRESH) continue;       /* face interior: skip */
                color = shade_wire(N_ws, ed);
                lum   = 0.7f + 0.3f * (1.f - ed / WIRE_THRESH);
                break;
            }
            case MODE_DEPTH:
                color = shade_depth(t_hit, cam_dist * 2.f, th);
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
    snprintf(buf2, sizeof buf2, " mode: %s ", k_mode_names[mode]);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(0, 0, "%s", buf2);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* §8.3 bottom hint strip — cyan, BOLD. ASCII only. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc/p:pause  s:mode  t:theme  r:reset  +/-:zoom ");
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
    float     angle_x   = 0.f;
    float     angle_y   = 0.f;
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

        /* §9.3 advance rotation if not paused. */
        if (!paused) {
            float sec = (float)dt * 1e-9f;
            angle_y += ROT_Y * sec;
            angle_x += ROT_X * sec;
        }

        /* §9.4 fps rolling average over half-second windows. */
        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps     = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        /* §9.5 paint frame. */
        long long t0 = clock_ns();
        erase();
        render(cols, rows, angle_x, angle_y, cam_dist, theme_idx, mode);
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
        case 'r': case 'R':
            angle_x = 0.f; angle_y = 0.f; break;
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
